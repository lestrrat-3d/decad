package dynamics

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A threeTraceSlice is a drift between two consecutive event boundaries.
// Rounded pair reports must agree on the pose of each shared body.
type threeTraceSlice struct {
	start, end units.Value
	from, to   State
	proofs     [3]*decad.SweepReport
}

type threeTraceEvent struct {
	at        units.Value
	pre, post State
}

func (tr Trace) sampleThreeSlices(t units.Value, timeValue *big.Rat) (State, error) {
	for _, event := range tr.threeEvents {
		if timeValue.Cmp(exactBase(event.at)) == 0 {
			return event.post, nil
		}
	}
	if timeValue.Sign() == 0 {
		return tr.start, nil
	}
	if timeValue.Cmp(exactBase(tr.duration)) == 0 {
		return tr.end, nil
	}
	for _, slice := range tr.threeSlices {
		if timeValue.Cmp(exactBase(slice.start)) <= 0 ||
			timeValue.Cmp(exactBase(slice.end)) >= 0 {
			continue
		}
		state := slice.from
		poses := make(map[*decad.Body]r3.Transform, 3)
		for key, proof := range slice.proofs {
			if proof == nil {
				continue
			}
			a, b, err := proof.CertifiedPosesAtInterval(t, slice.start, slice.end)
			if err != nil {
				return State{}, fmt.Errorf("%w: %w", ErrUnsupported, err)
			}
			pair := tr.start.world.three.pairs[key]
			for side, pose := range [2]r3.Transform{a, b} {
				body := pair.parts[side].definition.Body
				if held, seen := poses[body]; seen && held != pose {
					return State{}, fmt.Errorf("%w: pair replay poses disagree", ErrUnsupported)
				}
				poses[body] = pose
			}
		}
		for _, entry := range slice.from.Entries() {
			if pose, seen := poses[entry.Body]; seen {
				entry.Pose = pose
				state = withBodyState(state, entry)
				continue
			}
			if tr.start.world.three.dynamicCount == 1 &&
				entry.Body == tr.start.world.three.parts[tr.start.world.three.dynamic].Body {
				return State{}, fmt.Errorf("%w: moving body has no replay proof", ErrUnsupported)
			}
			if tr.start.world.three.dynamicCount == 2 {
				for key, indices := range threePairs {
					inPair := tr.start.world.three.parts[indices[0]].Body == entry.Body ||
						tr.start.world.three.parts[indices[1]].Body == entry.Body
					if inPair && slice.proofs[key] == nil &&
						!threePairExcluded(tr.start.world.three.excluded,
							tr.start.world.three.parts, indices) {
						return State{}, fmt.Errorf("%w: moving pair has no replay proof", ErrUnsupported)
					}
				}
				advanced, err := tr.start.world.threeDriftState(slice.from, t.Base()-slice.start.Base())
				if err != nil {
					return State{}, fmt.Errorf("%w: excluded replay pose is not finite: %v", ErrUnsupported, err)
				}
				replayed, _ := advanced.Body(entry.Body)
				state = withBodyState(state, replayed)
			}
		}
		return state, nil
	}
	return State{}, fmt.Errorf("%w: trace time has no certified slice", ErrUnsupported)
}

func zeroStepInput() StepInput {
	z := units.MillimetersPerSecondSquared(0)
	return StepInput{Gravity: QuantityVec{X: z, Y: z, Z: z}}
}

func (w *World) sequentialBoxCandidate(sweep *decad.SweepReport) bool {
	if sweep == nil || sweep.Outcome != decad.SweepImpactBracket || sweep.Bracket == nil ||
		sweep.Event == nil || sweep.Event.Manifold == nil || len(sweep.Event.Manifold.Points) != 4 {
		return false
	}
	for _, point := range sweep.Event.Manifold.Points {
		if point.FaceA == nil || point.FaceB == nil {
			return false
		}
	}
	return true
}

func (w *World) sequentialCandidate(sweep *decad.SweepReport) bool {
	if w.three.dynamicCount == 1 {
		return w.sequentialBoxCandidate(sweep)
	}
	return sweep != nil && sweep.Outcome == decad.SweepImpactBracket &&
		sweep.Bracket != nil && sweep.Event != nil &&
		sweep.Event.Manifold != nil && len(sweep.Event.Manifold.Points) != 0
}

// stepThreeSequential uses the existing pair response for each event but
// commits only its certified prefix. Every new event restarts all active pairs.
// The second result says whether this path owned the input.
func (w *World) stepThreeSequential(ctx context.Context, from, kicked State,
	input StepInput, loads [2]*BodyLoad, dt units.Value, reference *World) (*StepReport, bool, error) {
	keys := make([]int, 0, 3)
	for key, pair := range w.three.pairs {
		if pair == nil {
			continue
		}
		if threePairExcluded(w.three.excluded, w.three.parts, threePairs[key]) {
			if w.three.dynamicCount == 1 {
				return nil, false, nil
			}
			continue
		}
		if w.three.dynamicCount == 1 &&
			(pair.friction.lower.Sign() != 0 || pair.friction.upper.Sign() != 0) {
			return nil, false, nil
		}
		if w.three.dynamicCount == 1 &&
			(pair.restitution.Base() <= 0 || w.step.ImpactSpeed.Base() != 0) {
			return nil, false, nil
		}
		keys = append(keys, key)
	}
	if w.three.dynamicCount == 1 && len(keys) != 2 {
		return nil, false, nil
	}
	for _, entry := range kicked.Entries() {
		if !zeroAngularVelocity(entry.AngularVelocity) {
			return nil, false, nil
		}
	}
	var initial [3]*decad.SweepReport
	anyImpact := false
	for _, key := range keys {
		pair := w.three.pairs[key]
		sweep, err := pair.sweep(ctx, pairState(kicked, pair), dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, true, err
		}
		initial[key] = sweep
		switch sweep.Outcome {
		case decad.SweepClear:
		case decad.SweepUndecided:
		case decad.SweepImpactBracket:
			if !w.sequentialCandidate(sweep) {
				return nil, false, nil
			}
			anyImpact = true
		default:
			return nil, false, nil
		}
	}
	if !anyImpact && w.three.dynamicCount == 1 {
		return nil, false, nil
	}
	for key, indices := range threePairs {
		if w.three.pairs[key] != nil ||
			threePairExcluded(w.three.excluded, w.three.parts, indices) {
			continue
		}
		a, _ := kicked.Body(w.three.parts[indices[0]].Body)
		b, _ := kicked.Body(w.three.parts[indices[1]].Body)
		contact, err := w.doc.ContactPair(ctx, a.Body, b.Body, a.Pose, b.Pose, w.step.Contact)
		if err != nil {
			return nil, true, err
		}
		if contact.Relation != decad.ContactSeparated {
			return w.threeUndecided(key, "fixed third-body pair is not separated"), true, nil
		}
	}
	current := kicked
	at := units.Seconds(0)
	var events []ContactEvent
	var slices []threeTraceSlice
	var boundaries []threeTraceEvent
	var policy [3]decad.SweepStartPolicy
	for exactBase(at).Cmp(exactBase(dt)) < 0 {
		// MaxEvents is the full-step proof budget. The design refuses any
		// remaining time once the recorded event count reaches that limit.
		if len(events) >= w.step.MaxEvents {
			return w.threeUndecided(-1, "maximum contact events reached before step end"), true, nil
		}
		remaining := units.Seconds(dt.Base() - at.Base())
		if !validQuantity(remaining, units.Time, true) {
			return w.threeUndecided(-1, "remaining step time is not representable"), true, nil
		}
		var reports [3]*decad.SweepReport
		for _, key := range keys {
			pair := w.three.pairs[key]
			var err error
			if len(events) == 0 {
				reports[key] = initial[key]
			} else {
				reports[key], err = pair.sweep(ctx, pairState(current, pair), remaining, policy[key])
			}
			if err != nil {
				return nil, true, err
			}
		}
		selected := -1
		var selectedRight *big.Rat
		for _, key := range keys {
			sweep := reports[key]
			if sweep.Outcome == decad.SweepClear || sweep.Outcome == decad.SweepDepartedClear {
				continue
			}
			if sweep.Outcome == decad.SweepUndecided {
				continue
			}
			if !w.sequentialCandidate(sweep) ||
				w.three.dynamicCount == 2 && w.three.pairs[key].restitution.Base() <= 0 {
				return w.threeUndecided(key, "sequential pair has no supported first impact"), true, nil
			}
			right := new(big.Rat).Mul(exactBase(sweep.Bracket.To.Fraction), exactBase(remaining))
			if selected < 0 || right.Cmp(selectedRight) < 0 {
				selected, selectedRight = key, right
			}
		}
		if selected < 0 {
			for _, key := range keys {
				if reports[key].Outcome == decad.SweepUndecided {
					return w.threeUndecided(key, "three-body pair sweep is undecided"), true, nil
				}
			}
			endPair, err := driftState(pairState(current, reference), remaining.Base())
			if err != nil {
				report, stepErr := undecidedArithmetic(reference, "final drift pose is not finite", err)
				report.Excluded = w.Excluded()
				return report, true, stepErr
			}
			end := withPairState(current, endPair)
			if w.three.dynamicCount == 2 {
				end, err = w.threeDriftState(current, remaining.Base())
				if err != nil {
					// The report carries the unsupported arithmetic outcome.
					//nolint:nilerr
					return w.threeUndecided(-1, "final three-body drift pose is not finite"), true, nil
				}
			}
			proofs, ok, err := w.threeSequentialProofs(ctx, keys, current, end, remaining, policy, -1, nil)
			if err != nil {
				return nil, true, err
			}
			if !ok {
				return w.threeUndecided(-1, "final pair paths lack rounded certificates"), true, nil
			}
			slices = append(slices, threeTraceSlice{start: at, end: dt, from: current, to: end, proofs: proofs})
			trace := Trace{start: from, end: end, duration: dt,
				threeSlices: slices, threeEvents: boundaries}
			var conservation StepConservation
			var valid bool
			if w.three.dynamicCount == 2 {
				conservation, valid = w.threeTwoDynamicConservation(from, kicked, end, trace,
					events, input, dt)
			} else {
				conservation, valid = w.threeSequentialConservation(reference, from, kicked, end,
					trace, events, input.Gravity, loads, dt)
			}
			if !valid {
				return w.threeUndecided(-1, "sequential conservation cannot be bounded"), true, nil
			}
			return &StepReport{Status: Advanced, Next: &end, Events: events,
				Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, true, nil
		}
		for _, key := range keys {
			if key == selected {
				continue
			}
			var left *big.Rat
			switch reports[key].Outcome {
			case decad.SweepImpactBracket:
				left = exactBase(reports[key].Bracket.From.Fraction)
			case decad.SweepUndecided:
				if reports[key].Unresolved == nil {
					return w.threeUndecided(key, "undecided pair has no time interval"), true, nil
				}
				left = exactBase(reports[key].Unresolved.From.Fraction)
			default:
				continue
			}
			if left == nil {
				return w.threeUndecided(key, "pair event time is not representable"), true, nil
			}
			left.Mul(left, exactBase(remaining))
			if left.Cmp(selectedRight) <= 0 {
				return w.threeUndecided(key, "another pair may contact before the selected event"), true, nil
			}
		}
		pair := w.three.pairs[selected]
		child, err := pair.Step(ctx, pairState(current, pair), zeroStepInput(), remaining)
		if err != nil {
			return nil, true, err
		}
		if child == nil || child.Status != Advanced || len(child.Events) != 1 ||
			!child.Trace.hasEvent || child.Trace.preSweep == nil {
			return w.threeUndecided(selected, "pair response lacks a certified event prefix"), true, nil
		}
		if child.Events[0].Kind != ContactImpact || child.Events[0].Pair != (BodyPair{
			A: pair.parts[0].definition.Body, B: pair.parts[1].definition.Body,
		}) || child.Events[0].Bracket != *reports[selected].Bracket {
			return w.threeUndecided(selected, "pair response changed the selected impact"), true, nil
		}
		localAt := child.Events[0].Time
		globalAt := units.Seconds(at.Base() + localAt.Base())
		if !validQuantity(globalAt, units.Time, true) || exactBase(globalAt).Cmp(exactBase(at)) <= 0 ||
			exactBase(globalAt).Cmp(exactBase(dt)) > 0 {
			return w.threeUndecided(selected, "global event time is not representable"), true, nil
		}
		bracketLo := new(big.Rat).Mul(exactBase(reports[selected].Bracket.From.Fraction),
			exactBase(remaining))
		bracketHi := new(big.Rat).Mul(exactBase(reports[selected].Bracket.To.Fraction),
			exactBase(remaining))
		globalLocal := new(big.Rat).Sub(exactBase(globalAt), exactBase(at))
		for adjustment := 0; adjustment < 4 && globalLocal.Cmp(bracketHi) > 0; adjustment++ {
			globalAt = units.Seconds(math.Nextafter(globalAt.Base(), math.Inf(-1)))
			globalLocal.Sub(exactBase(globalAt), exactBase(at))
		}
		for adjustment := 0; adjustment < 4 && globalLocal.Cmp(bracketLo) < 0; adjustment++ {
			globalAt = units.Seconds(math.Nextafter(globalAt.Base(), math.Inf(1)))
			globalLocal.Sub(exactBase(globalAt), exactBase(at))
		}
		if globalLocal.Cmp(bracketLo) < 0 || globalLocal.Cmp(bracketHi) > 0 {
			return w.threeUndecided(selected, "global event time falls outside its bracket"), true, nil
		}
		if exactBase(globalAt).Cmp(exactBase(at)) <= 0 ||
			exactBase(globalAt).Cmp(exactBase(dt)) > 0 {
			return w.threeUndecided(selected, "adjusted event time is outside the step"), true, nil
		}
		pre := withPairState(current, child.Trace.pre)
		if w.three.dynamicCount == 2 {
			pre, err = w.threeDriftState(current, localAt.Base())
			if err != nil {
				// The report carries the unsupported arithmetic outcome.
				//nolint:nilerr
				return w.threeUndecided(selected, "event three-body drift pose is not finite"), true, nil
			}
			pre = withPairState(pre, child.Trace.pre)
		}
		post := withPairState(pre, child.Trace.post)
		proofs, ok, err := w.threeSequentialProofs(ctx, keys, current, pre,
			localAt, policy, selected, child.Trace.preSweep)
		if err != nil {
			return nil, true, err
		}
		if !ok {
			return w.threeUndecided(selected, "event prefix lacks both rounded pair certificates"), true, nil
		}
		for _, key := range keys {
			if key == selected {
				continue
			}
			other := w.three.pairs[key]
			before, after := pairState(pre, other), pairState(post, other)
			if before.entries[0].Pose != after.entries[0].Pose ||
				before.entries[1].Pose != after.entries[1].Pose {
				separated, err := threeClearCorrection(ctx, other, before, after)
				if err != nil {
					return nil, true, err
				}
				if !separated {
					return w.threeUndecided(key, "event correction reaches another body"), true, nil
				}
			}
		}
		event := child.Events[0]
		event.SliceStart, event.SliceDuration, event.Time = at, remaining, globalAt
		if reason := pair.eventConservationFailure(event); reason != "" {
			return w.threeUndecided(selected, reason), true, nil
		}
		events = append(events, event)
		slices = append(slices, threeTraceSlice{start: at, end: globalAt,
			from: current, to: pre, proofs: proofs})
		boundaries = append(boundaries, threeTraceEvent{at: globalAt, pre: pre, post: post})
		current, at = post, globalAt
		policy[selected] = decad.ContinueSeparatingTouch
		for _, key := range keys {
			if key != selected {
				policy[key] = decad.StopAtInitialContact
			}
		}
	}
	if exactBase(at).Cmp(exactBase(dt)) != 0 {
		return w.threeUndecided(-1, "sequential step lacks a final slice"), true, nil
	}
	trace := Trace{start: from, end: current, duration: dt,
		threeSlices: slices, threeEvents: boundaries}
	var conservation StepConservation
	var ok bool
	if w.three.dynamicCount == 2 {
		conservation, ok = w.threeTwoDynamicConservation(from, kicked, current, trace,
			events, input, dt)
	} else {
		conservation, ok = w.threeSequentialConservation(reference, from, kicked, current,
			trace, events, input.Gravity, loads, dt)
	}
	if !ok {
		return w.threeUndecided(-1, "sequential conservation cannot be bounded"), true, nil
	}
	return &StepReport{Status: Advanced, Next: &current, Events: events,
		Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, true, nil
}

func (w *World) threeSequentialProofs(ctx context.Context, keys []int,
	start, end State, duration units.Value, policy [3]decad.SweepStartPolicy,
	impact int, impactPrefix *decad.SweepReport) ([3]*decad.SweepReport, bool, error) {
	var proofs [3]*decad.SweepReport
	for _, key := range keys {
		if key == impact {
			proofs[key] = impactPrefix
			if !impactPrefix.HasAffineReplayProof() {
				return proofs, false, nil
			}
			continue
		}
		pair := w.three.pairs[key]
		ideal, err := pair.sweep(ctx, pairState(start, pair), duration, policy[key])
		if err != nil {
			return proofs, false, err
		}
		if ideal.Outcome != decad.SweepClear && ideal.Outcome != decad.SweepDepartedClear {
			return proofs, false, nil
		}
		rounded, err := pair.sweepPoses(ctx, pairState(start, pair), pairState(end, pair),
			duration, policy[key])
		if err != nil {
			return proofs, false, err
		}
		if rounded.Outcome != ideal.Outcome || !rounded.HasAffineReplayProof() {
			return proofs, false, nil
		}
		proofs[key] = rounded
	}
	return proofs, true, nil
}

func (w *World) threeSequentialConservation(reference *World,
	from, kicked, end State, trace Trace, events []ContactEvent,
	gravity QuantityVec, loads [2]*BodyLoad, dt units.Value) (StepConservation, bool) {
	input, ok := reference.conservationState(pairState(from, reference))
	if !ok {
		return StepConservation{}, false
	}
	afterKick, ok := reference.conservationState(pairState(kicked, reference))
	if !ok {
		return StepConservation{}, false
	}
	completion, ok := reference.conservationState(pairState(end, reference))
	if !ok {
		return StepConservation{}, false
	}
	gravityImpulse, loadImpulse, ok := reference.forceImpulses(gravity, loads, dt)
	if !ok {
		return StepConservation{}, false
	}
	torqueImpulse, ok := reference.torqueImpulse(loads, dt)
	if !ok {
		return StepConservation{}, false
	}
	var driftSlices [][2]State
	for _, slice := range trace.threeSlices {
		driftSlices = append(driftSlices, [2]State{
			pairState(slice.from, reference), pairState(slice.to, reference),
		})
	}
	driftChange, ok := reference.driftConservationSlices(driftSlices)
	if !ok {
		return StepConservation{}, false
	}
	contactImpulse, ok := w.threeSequentialContactImpulse(events)
	if !ok {
		return StepConservation{}, false
	}
	zero, ok := boundedReading(new(big.Rat), new(big.Rat), new(big.Rat),
		units.KilogramSquareMillimeterPerSecondSquared)
	if !ok {
		return StepConservation{}, false
	}
	return StepConservation{Input: input, AfterKick: afterKick, Completion: completion,
		GravityImpulse: gravityImpulse, LoadImpulse: loadImpulse, TorqueImpulse: torqueImpulse,
		ContactImpulse: contactImpulse, DriftChange: driftChange, KinematicWork: zero}, true
}

func (w *World) threeSequentialContactImpulse(events []ContactEvent) (MomentumReading, bool) {
	var value, low, high [3]*big.Rat
	for axis := range value {
		value[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for _, event := range events {
		var pair *World
		for key, indices := range threePairs {
			if event.Pair == (BodyPair{A: w.three.parts[indices[0]].Body,
				B: w.three.parts[indices[1]].Body}) {
				pair = w.three.pairs[key]
				break
			}
		}
		if pair == nil {
			return MomentumReading{}, false
		}
		reading, ok := pair.externalContactImpulse([]ContactEvent{event})
		if !ok {
			return MomentumReading{}, false
		}
		for axis := range value {
			component, bound := exactBase(velocityComponent(reading.Value, axis)),
				exactBase(velocityComponent(reading.Bound, axis))
			if component == nil || bound == nil || bound.Sign() < 0 {
				return MomentumReading{}, false
			}
			value[axis].Add(value[axis], component)
			low[axis].Add(low[axis], new(big.Rat).Sub(component, bound))
			high[axis].Add(high[axis], new(big.Rat).Add(component, bound))
		}
	}
	return boundedMomentum(value, low, high)
}
