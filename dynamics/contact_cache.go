package dynamics

import (
	"context"
	"errors"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
)

// This file is the reuse cache and the work budget of the step
// (docs/multibody-dynamics-design.md §3.2, §3.3, §5.3,
// §6.2 and §12): every SweptBox and SweepPair call the step makes passes
// through one stepWork, which serves a call whose inputs it has seen before
// from the step's own record or from the input state's cache, and charges
// every other call against MaxPairSweeps. A solved island whose whole
// problem matches a cached one restarts the proposal from that island's
// final state. Nothing here changes a published value but a sweep count: a
// reused report is the report the same call returns, and a restarted
// proposal is the one the cold solve published.

// errPairBudget stops a step whose SweptBox and SweepPair calls would exceed
// MaxPairSweeps. The step reports it as StepPairBudget.
var errPairBudget = errors.New("dynamics: pair sweep budget exhausted")

// contactCache is the immutable reuse record one scheduled step attaches to
// the state it publishes: every swept box and pair sweep the step used, keyed
// by their exact inputs, and every island it certified. It
// describes entries, the state it was published with, and is read only by a
// step from that state.
type contactCache struct {
	entries []BodyState
	boxes   map[boxKey]decad.SweptBox
	sweeps  map[sweepKey]*decad.SweepReport
	islands []*warmIsland
}

// boxKey names one SweptBox call. A stationary PoseSegment's box is its
// body's bounds at the pose, whatever the duration, so it is keyed by the
// pose alone: a Fixed body's box is computed once.
type boxKey struct {
	body int
	path decad.PairPath
}

// sweepKey names one SweepPair call by its exact inputs. SweepPair is a pure
// function of its bodies, paths and request, so equal keys return equal
// reports.
type sweepKey struct {
	pair    int
	a, b    decad.PairPath
	request decad.SweepRequest
}

// warmIsland is one certified island: the exact problem it solved (§6.2's
// inputs), the proposal's final state, whether that state is a fixed point
// of the sweeps, and the sweeps the solve ran. Its
// impulses are held per pair and manifold point, and each point's
// ContactFeature pair is part of the problem a restart must match, so a
// restart never keys by a topology slice index
// (docs/rigid-dynamics-design.md "Response").
type warmIsland struct {
	bodies  []int
	pre     []BodyState     // island body order
	drive   []*driverMotion // island body order; nil for a body without a driver velocity
	pairs   []warmPair      // canonical order
	lambda  []float64       // island point order
	tangent [][2]float64    // island point order, in each point's tangent basis
	current []nominalBody   // the proposal's final velocities, island body order
	fixed   bool            // the last sweep changed no impulse
	sweeps  int             // sweeps the cold solve ran
}

// warmPair is one island pair's manifold as the proposal read it.
type warmPair struct {
	key    int
	points []decad.ContactPoint
}

// stepWork is one step's SweptBox and SweepPair calls and island restarts.
// It lives for one Step call; its out record becomes the published state's
// cache. A stepWork without an out record (directWork) calls decad directly
// and charges nothing, as Trace.Sample does.
type stepWork struct {
	w     *World
	input *contactCache
	out   *contactCache
	calls uint64
}

// directWork calls decad for every query, with no reuse and no budget.
func directWork(w *World) *stepWork {
	return &stepWork{w: w}
}

// newStepWork reads the cache of from when it describes from's entries.
func newStepWork(w *World, from State) *stepWork {
	work := &stepWork{w: w, out: &contactCache{boxes: map[boxKey]decad.SweptBox{},
		sweeps: map[sweepKey]*decad.SweepReport{}}}
	if from.cache != nil && slices.Equal(from.cache.entries, from.entries) {
		work.input = from.cache
	}
	return work
}

// charge counts one decad call against MaxPairSweeps.
func (s *stepWork) charge() error {
	if s.calls >= s.w.step.MaxPairSweeps {
		return errPairBudget
	}
	s.calls++
	return nil
}

// sweptBox is Document.SweptBox for one world body, reused when the same
// box was computed before.
func (s *stepWork) sweptBox(ctx context.Context, index int, path decad.PairPath) (decad.SweptBox, error) {
	body := s.w.bodies[index].definition.Body
	if s.out == nil {
		return s.w.doc.SweptBox(ctx, body, path)
	}
	key := boxKey{body: index, path: path}
	if segment, ok := path.(decad.PoseSegment); ok && segment.From == segment.To {
		key.path = decad.PoseSegment{From: segment.From, To: segment.To}
	}
	if box, ok := s.out.boxes[key]; ok {
		return box, nil
	}
	if s.input != nil {
		if box, ok := s.input.boxes[key]; ok {
			s.out.boxes[key] = box
			return box, nil
		}
	}
	if err := s.charge(); err != nil {
		return decad.SweptBox{}, err
	}
	box, err := s.w.doc.SweptBox(ctx, body, path)
	if err != nil {
		return decad.SweptBox{}, err
	}
	s.out.boxes[key] = box
	return box, nil
}

// sweepPair is Document.SweepPair for one scheduled pair, reused when the
// same call was made before (§5.3).
func (s *stepWork) sweepPair(ctx context.Context, key int, a, b decad.PairPath,
	request decad.SweepRequest) (*decad.SweepReport, error) {
	pair := s.w.pairs[key]
	bodyA, bodyB := s.w.bodies[pair.a].definition.Body, s.w.bodies[pair.b].definition.Body
	if s.out == nil {
		return s.w.doc.SweepPair(ctx, bodyA, bodyB, a, b, request)
	}
	held := sweepKey{pair: key, a: a, b: b, request: request}
	if sweep, ok := s.out.sweeps[held]; ok {
		return sweep, nil
	}
	if s.input != nil {
		if sweep, ok := s.input.sweeps[held]; ok {
			s.out.sweeps[held] = sweep
			return sweep, nil
		}
	}
	if err := s.charge(); err != nil {
		return nil, err
	}
	sweep, err := s.w.doc.SweepPair(ctx, bodyA, bodyB, a, b, request)
	if err != nil {
		return nil, err
	}
	s.out.sweeps[held] = sweep
	return sweep, nil
}

// warmStart returns the cached island whose problem equals isl at pre with
// drive exactly, or nil.
func (s *stepWork) warmStart(isl island, pre State, drive map[int]driverMotion) *warmIsland {
	if s.input == nil {
		return nil
	}
	problem := newWarmIsland(isl, pre, drive)
	for _, held := range s.input.islands {
		if held.sameProblem(problem) {
			return held
		}
	}
	return nil
}

// keep records a certified island for the next step.
func (s *stepWork) keep(held *warmIsland) {
	if s.out == nil {
		return
	}
	s.out.islands = append(s.out.islands, held)
}

// cache returns the record of this step as the cache of the state it
// publishes with entries.
func (s *stepWork) cache(entries []BodyState) *contactCache {
	if s.out == nil {
		return nil
	}
	out := *s.out
	out.entries = entries
	return &out
}

// newWarmIsland reads the problem of isl at pre with drive: its bodies,
// their pre-solve entries and driver velocities, and every pair's manifold.
func newWarmIsland(isl island, pre State, drive map[int]driverMotion) *warmIsland {
	out := &warmIsland{bodies: slices.Clone(isl.bodies)}
	for _, index := range isl.bodies {
		out.pre = append(out.pre, pre.entries[index])
		v, ok := drive[index]
		if !ok {
			out.drive = append(out.drive, nil)
			continue
		}
		var held driverMotion
		for axis := range 3 {
			held.linear[axis] = new(big.Rat).Set(v.linear[axis])
			held.angular[axis] = new(big.Rat).Set(v.angular[axis])
		}
		out.drive = append(out.drive, &held)
	}
	for _, pair := range isl.pairs {
		out.pairs = append(out.pairs, warmPair{key: pair.key, points: slices.Clone(pair.manifold.Points)})
	}
	return out
}

// sameProblem compares two island problems exactly: the same bodies with
// the same entries and driver velocities, and the same pairs with the same
// manifold points, features included.
func (h *warmIsland) sameProblem(o *warmIsland) bool {
	if !slices.Equal(h.bodies, o.bodies) || !slices.Equal(h.pre, o.pre) || len(h.pairs) != len(o.pairs) {
		return false
	}
	for i, pair := range h.pairs {
		if pair.key != o.pairs[i].key || !slices.Equal(pair.points, o.pairs[i].points) {
			return false
		}
	}
	for i, v := range h.drive {
		u := o.drive[i]
		if (v == nil) != (u == nil) {
			return false
		}
		if v == nil {
			continue
		}
		for axis := range 3 {
			if v.linear[axis].Cmp(u.linear[axis]) != 0 || v.angular[axis].Cmp(u.angular[axis]) != 0 {
				return false
			}
		}
	}
	return true
}
