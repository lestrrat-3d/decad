package dynamics

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Trace keeps the rounded sweep certificates and the event states for replay.
// A world of four or more bodies records its step as event-free slices and
// the events between them (docs/multibody-dynamics-design.md §3.4); the two-
// and three-body steps keep their fixed slots.
type Trace struct {
	start               State
	pre                 State
	post                State
	end                 State
	preSweep            *decad.SweepReport
	postSweep           *decad.SweepReport
	grazingSweep        *decad.SweepReport
	rotationalPrefix    *decad.SweepReport
	rotationalRemainder *decad.SweepReport
	threeSweeps         [3]*decad.SweepReport
	threeSlices         []threeTraceSlice
	threeEvents         []threeTraceEvent
	slices              []traceSlice // a world of four or more bodies, docs/multibody-dynamics-design.md §3.4
	events              []traceEvent // the events between those slices
	scheduled           bool         // slices and events hold the whole record, even when both are empty
	pairCalls           uint64       // SweptBox and SweepPair calls the step made, reused ones excluded
	duration            units.Value
	eventAt             units.Value
	hasEvent            bool
	excluded            bool
}

// traceSlice is one event-free interval [start, end] of a step of a world of
// four or more bodies (docs/multibody-dynamics-design.md §3.4). Every body
// moves on paths[i], which runs over the held span [start, span] to the end
// of the step; every scheduled pair carries the rounded certificate that
// proves those paths over the same span, or the swept-box exclusion that
// made no sweep necessary. A slice cut short by an event replays only its
// prefix up to end.
type traceSlice struct {
	start, end units.Value // held times from the start of the step
	span       units.Value // held end of the paths and certificates: the step's duration
	from, to   State
	paths      []decad.PairPath // world order
	proofs     []pairProof      // scheduled pairs, canonical order
}

// pairProof is the certificate one scheduled pair holds for a slice: the
// pair's sweep, or boxClear when the two swept boxes are strictly disjoint
// and the pair was never swept (§4.3).
type pairProof struct {
	pair     int // index into World.pairs
	sweep    *decad.SweepReport
	boxClear bool
}

// traceEvent is one event time of a step of a world of four or more bodies:
// its held time from the step start, the states on both sides of it, and the
// islands solved there (docs/multibody-dynamics-design.md §3.4).
type traceEvent struct {
	at        *big.Rat
	pre, post State
	islands   []int // indices into StepReport.Islands
}

// Sample evaluates a recorded rounded path and its cached geometry proof.
// It performs no response solve. A world of four or more bodies also reads
// the bodies' bounds through Document.SweptBox to check its swept-box
// exclusions at the sampled poses (docs/multibody-dynamics-design.md §7.1).
// Sample writes nothing, so concurrent calls on one Trace are safe.
func (tr Trace) Sample(t units.Value) (State, error) {
	timeValue, durationValue := exactBase(t), exactBase(tr.duration)
	if t.Kind() != units.Time || !finite(t.Base()) || timeValue == nil || durationValue == nil ||
		timeValue.Sign() < 0 || timeValue.Cmp(durationValue) > 0 {
		return State{}, fmt.Errorf("%w: trace time outside step", ErrInvalidInput)
	}
	if tr.scheduled {
		return tr.sampleSlices(t, timeValue, durationValue)
	}
	if len(tr.threeSlices) != 0 {
		return tr.sampleThreeSlices(t, timeValue)
	}
	var eventValue *big.Rat
	if tr.hasEvent {
		eventValue = exactBase(tr.eventAt)
		if eventValue == nil {
			return State{}, fmt.Errorf("%w: trace event time is invalid", ErrUnsupported)
		}
	}
	if tr.hasEvent && timeValue.Cmp(eventValue) == 0 {
		return tr.post, nil
	}
	if timeValue.Sign() == 0 {
		return tr.start, nil
	}
	if timeValue.Cmp(durationValue) == 0 {
		return tr.end, nil
	}
	if tr.grazingSweep != nil {
		state := tr.post.clone()
		a, b, err := tr.grazingSweep.CertifiedPosesAtInterval(t, units.Seconds(0), tr.duration)
		if err != nil {
			return State{}, fmt.Errorf("%w: %w", ErrUnsupported, err)
		}
		state.entries[0].Pose, state.entries[1].Pose = a, b
		return state, nil
	}
	if len(tr.start.entries) == 3 && tr.threeSweeps != ([3]*decad.SweepReport{}) {
		return tr.sampleThreeSweeps(t)
	}
	state := tr.end
	sweep := tr.preSweep
	if !tr.hasEvent && sweep == nil {
		sweep = tr.rotationalRemainder
	}
	sliceStart, sliceEnd := units.Seconds(0), tr.duration
	if tr.hasEvent {
		if timeValue.Cmp(eventValue) < 0 {
			state = tr.pre
			sliceEnd = tr.eventAt
			if sweep == nil {
				// The impact prefix is the original full-step rotating sweep.
				sweep, sliceEnd = tr.rotationalPrefix, tr.duration
			}
		} else {
			state = tr.post
			sweep = tr.postSweep
			sliceStart = tr.eventAt
			if sweep == nil {
				sweep = tr.rotationalRemainder
			}
		}
	}
	if tr.excluded {
		return tr.sampleExcluded(timeValue, durationValue)
	}
	if sweep == nil {
		return State{}, fmt.Errorf("%w: interior trace sample has no rounded path certificate", ErrUnsupported)
	}
	a, b, err := sweep.CertifiedPosesAtInterval(t, sliceStart, sliceEnd)
	if err != nil {
		return State{}, fmt.Errorf("%w: %w", ErrUnsupported, err)
	}
	state = state.clone()
	state.entries[0].Pose, state.entries[1].Pose = a, b
	return state, nil
}

// sampleThreeSweeps replays a three-body step whose three pair sweeps cover
// the whole step.
func (tr Trace) sampleThreeSweeps(t units.Value) (State, error) {
	if tr.start.world == nil || tr.start.world.three == nil {
		return State{}, fmt.Errorf("%w: three-body trace has no world", ErrUnsupported)
	}
	state := tr.post
	poses := make(map[*decad.Body]r3.Transform, 3)
	for key, sweep := range tr.threeSweeps {
		if sweep == nil {
			continue
		}
		a, b, err := sweep.CertifiedPosesAtInterval(t, units.Seconds(0), tr.duration)
		if err != nil {
			return State{}, fmt.Errorf("%w: %w", ErrUnsupported, err)
		}
		pair := tr.start.world.three.pairs[key]
		for side, pose := range [2]r3.Transform{a, b} {
			body := pair.bodies[side].definition.Body
			if held, seen := poses[body]; seen && held != pose {
				return State{}, fmt.Errorf("%w: three-body sweeps disagree on a shared pose", ErrUnsupported)
			}
			poses[body] = pose
		}
	}
	if len(poses) != 3 {
		return State{}, fmt.Errorf("%w: three-body trace lacks a pair certificate", ErrUnsupported)
	}
	for body, pose := range poses {
		entry, ok := state.Body(body)
		if !ok {
			return State{}, fmt.Errorf("%w: three-body trace names an unknown body", ErrUnsupported)
		}
		entry.Pose = pose
		state = withBodyState(state, entry)
	}
	return state, nil
}

// sampleExcluded replays an excluded two-body pair by interpolating each
// body's translation.
func (tr Trace) sampleExcluded(timeValue, durationValue *big.Rat) (State, error) {
	if tr.hasEvent {
		return State{}, fmt.Errorf("%w: excluded trace has an event", ErrUnsupported)
	}
	from, to := tr.start, tr.end
	fraction, _ := new(big.Rat).Quo(timeValue, durationValue).Float64()
	state := tr.end.clone()
	for i := range state.entries {
		start, end := from.entries[i].Pose.Translation(), to.entries[i].Pose.Translation()
		delta := end.Sub(start).Scale(fraction)
		pose, err := translatePose(from.entries[i].Pose, delta)
		if err != nil {
			return State{}, fmt.Errorf("%w: excluded replay pose is not finite: %v", ErrUnsupported, err)
		}
		state.entries[i].Pose = pose
	}
	return state, nil
}

// sampleSlices is §7.1 for a trace of slices: an event's post state at its
// exact time, the start state at zero, the end state at the duration, and
// inside a slice every body's pose from the pair certificates that cover it,
// or from its own path when only swept-box exclusions cover it. Velocities
// inside a slice are the slice's from velocities.
func (tr Trace) sampleSlices(t units.Value, timeValue, durationValue *big.Rat) (State, error) {
	// The end comes first: an event at the step's end leaves its post state
	// as the end, which carries the step's contact set and cache.
	if timeValue.Cmp(durationValue) == 0 && timeValue.Sign() > 0 {
		return tr.end, nil
	}
	for _, event := range tr.events {
		if timeValue.Cmp(event.at) == 0 {
			return event.post, nil
		}
	}
	if timeValue.Sign() == 0 {
		return tr.start, nil
	}
	if timeValue.Cmp(durationValue) == 0 {
		return tr.end, nil
	}
	for _, slice := range tr.slices {
		start, end := exactBase(slice.start), exactBase(slice.end)
		if timeValue.Cmp(end) == 0 {
			return slice.to, nil
		}
		if timeValue.Cmp(start) <= 0 || timeValue.Cmp(end) > 0 {
			continue
		}
		return slice.sample(t, timeValue, start)
	}
	return State{}, fmt.Errorf("%w: trace time has no certified slice", ErrUnsupported)
}

// sample replays one slice at t. Every path and pair certificate of the slice
// spans [slice.start, slice.span], so the replay fraction is the share of
// that span elapsed at t, whatever earlier time ends the slice.
func (slice traceSlice) sample(t units.Value, timeValue, start *big.Rat) (State, error) {
	world := slice.from.world
	state := slice.from.clone()
	covered := make([]bool, len(state.entries))
	for _, proof := range slice.proofs {
		if proof.sweep == nil {
			continue
		}
		poseA, poseB, err := proof.sweep.CertifiedPosesAtInterval(t, slice.start, slice.span)
		if err != nil {
			return State{}, fmt.Errorf("%w: %w", ErrUnsupported, err)
		}
		pair := world.pairs[proof.pair]
		for _, side := range [2]struct {
			body int
			pose r3.Transform
		}{{pair.a, poseA}, {pair.b, poseB}} {
			if covered[side.body] && state.entries[side.body].Pose != side.pose {
				return State{}, fmt.Errorf("%w: pair certificates disagree on a shared pose", ErrUnsupported)
			}
			state.entries[side.body].Pose, covered[side.body] = side.pose, true
		}
	}
	span := new(big.Rat).Sub(exactBase(slice.span), start)
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(timeValue, start), span)
	for i := range state.entries {
		if covered[i] {
			continue
		}
		pose, err := pathPoseAt(slice.paths[i], fraction)
		if err != nil {
			return State{}, fmt.Errorf("%w: slice path pose is not finite: %v", ErrUnsupported, err)
		}
		state.entries[i].Pose = pose
	}
	// §7.1: a pair the swept boxes exclude was never swept, and the rounded
	// poses may leave the exact path by an ulp. Their bounds must still be
	// strictly apart at the sampled poses.
	key, err := world.boxExclusionsHold(context.Background(), directWork(world), state, slice.proofs)
	if err != nil {
		return State{}, fmt.Errorf("%w: %w", ErrUnsupported, err)
	}
	if key >= 0 {
		return State{}, fmt.Errorf("%w: rounded poses of a box-excluded pair are not strictly apart", ErrUnsupported)
	}
	return state, nil
}

// boxExclusionsHold checks every box-excluded pair of a slice at the rounded
// poses of state: the two bodies' bounds, mapped exactly through their poses
// by Document.SweptBox along a stationary path, must be strictly disjoint.
// The swept boxes that excluded the pair enclose its ideal path; a rounded
// pose can leave that path by an ulp, so the exclusion alone does not cover
// it. It returns the first pair key whose bounds are not strictly apart, or
// -1 when every exclusion holds.
func (w *World) boxExclusionsHold(ctx context.Context, work *stepWork, state State, proofs []pairProof) (int, error) {
	boxes := make(map[int]decad.SweptBox)
	boxOf := func(index int) (decad.SweptBox, error) {
		if box, ok := boxes[index]; ok {
			return box, nil
		}
		pose := state.entries[index].Pose
		box, err := work.sweptBox(ctx, index, decad.PoseSegment{From: pose, To: pose, Duration: units.Seconds(1)})
		if err != nil {
			return decad.SweptBox{}, err
		}
		boxes[index] = box
		return box, nil
	}
	for _, proof := range proofs {
		if !proof.boxClear {
			continue
		}
		pair := w.pairs[proof.pair]
		a, err := boxOf(pair.a)
		if err != nil {
			return -1, err
		}
		b, err := boxOf(pair.b)
		if err != nil {
			return -1, err
		}
		if !a.StrictlyDisjoint(b) {
			return proof.pair, nil
		}
	}
	return -1, nil
}
