package dynamics

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

var threePairs = [3][2]int{{0, 1}, {0, 2}, {1, 2}}

// threeBodyWorld keeps one two-body response world per pair-table entry with a
// dynamic body, keyed by canonical pair index. The current three-body step
// admits one, two, or three dynamic bodies; every other active pair must have
// a certified clear path before a result is published.
type threeBodyWorld struct {
	pairs        [3]*World
	dynamic      int
	dynamicCount int
}

// newThreeBodyWorld reads the admitted bodies and the pair table of a
// three-body world. A Fixed/Fixed pair gets no response world; the step still
// queries it with ContactPair.
func newThreeBodyWorld(w *World) *threeBodyWorld {
	three := &threeBodyWorld{dynamic: -1}
	for i, body := range w.bodies {
		if body.definition.Role != Dynamic {
			continue
		}
		if three.dynamic < 0 {
			three.dynamic = i
		}
		three.dynamicCount++
	}
	for key, pair := range w.pairs {
		if w.bodies[pair.a].definition.Role != Dynamic && w.bodies[pair.b].definition.Role != Dynamic {
			continue
		}
		three.pairs[key] = w.pairWorld(key)
	}
	return three
}

func pairState(state State, pair *World) State {
	first, _ := state.Body(pair.bodies[0].definition.Body)
	second, _ := state.Body(pair.bodies[1].definition.Body)
	return State{world: pair, entries: []BodyState{first, second}}
}

func withPairState(original State, pairState State) State {
	out := original.clone()
	for _, entry := range pairState.entries {
		for i := range out.entries {
			if out.entries[i].Body == entry.Body {
				out.entries[i] = entry
			}
		}
	}
	return out
}

func (w *World) stepThreeBodies(ctx context.Context, from State, input StepInput,
	dt units.Value) (*StepReport, error) {
	if err := validateQuantityVec(input.Gravity, units.Acceleration); err != nil {
		return nil, err
	}
	if len(input.Drivers) != 0 {
		return nil, fmt.Errorf("%w: this world has no kinematic body", ErrInvalidInput)
	}
	live := w.doc.Bodies()
	for _, part := range w.bodies {
		if !containsBody(live, part.definition.Body) {
			return nil, fmt.Errorf("%w: world body was retired", ErrInvalidInput)
		}
	}
	var reference *World
	referenceKey := -1
	for key, pair := range w.three.pairs {
		if pair != nil {
			reference = pair
			referenceKey = key
			break
		}
	}
	if reference == nil {
		return nil, fmt.Errorf("%w: three-body world has no response pair", ErrUnsupported)
	}
	if w.three.dynamicCount == 2 {
		return w.stepThreeTwoDynamic(ctx, from, input, dt, reference)
	}
	if w.three.dynamicCount == 3 {
		return w.stepThreeAllDynamic(ctx, from, input, dt, reference)
	}
	loads, err := reference.validateLoads(input.Loads)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	kickPair, ok := reference.kickByLoads(pairState(from, reference), input.Gravity, loads, dt)
	if !ok {
		return w.threeUndecided(referenceKey, "force kick exceeds the velocity residual"), nil
	}
	kicked := withPairState(from, kickPair)
	if w.three.dynamicCount == 1 {
		if report, handled, err := w.stepThreeSequential(ctx, from, kicked, input, loads, dt, reference); handled || err != nil {
			return report, err
		}
	}
	active := -1
	var simultaneous [3]*decad.SweepReport
	activeCount := 0
	for key, indices := range threePairs {
		if w.pairs[key].excluded {
			continue
		}
		pair := w.three.pairs[key]
		if pair == nil {
			a, _ := kicked.Body(w.bodies[indices[0]].definition.Body)
			b, _ := kicked.Body(w.bodies[indices[1]].definition.Body)
			contact, err := w.doc.ContactPair(ctx, a.Body, b.Body, a.Pose, b.Pose, w.step.Contact)
			if err != nil {
				return nil, err
			}
			if contact.Relation != decad.ContactSeparated {
				return w.threeUndecided(key, "fixed third-body pair is not separated"), nil
			}
			continue
		}
		first, err := pair.sweep(ctx, pairState(kicked, pair), dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if first.Outcome == decad.SweepClear {
			continue
		}
		if first.Outcome == decad.SweepUndecided {
			return w.threeUndecided(key, "a three-body pair sweep is undecided"), nil
		}
		if active < 0 {
			active = key
		}
		activeCount++
		simultaneous[key] = first
	}
	if activeCount > 1 {
		allInitial := true
		for _, sweep := range simultaneous {
			if sweep != nil && sweep.Outcome != decad.SweepInitiallyTouching {
				allInitial = false
			}
		}
		if !allInitial {
			for key, sweep := range simultaneous {
				if sweep != nil && key != active {
					return w.threeUndecided(key, "more than one three-body pair may contact during the step"), nil
				}
			}
		}
		return w.stepThreeSimultaneousInitial(ctx, from, kicked, input.Gravity, loads,
			dt, simultaneous, reference)
	}
	if active < 0 {
		active = referenceKey
	}
	// NewWorld constructs two dynamic/fixed child worlds, so the selected
	// response pair always has a child solver.
	chosen := w.three.pairs[active]
	result, err := chosen.Step(ctx, pairState(from, chosen), input, dt)
	if err != nil || result == nil || result.Status != Advanced || result.Next == nil {
		if result != nil {
			result.Excluded = w.Excluded()
		}
		return result, err
	}
	for key, pair := range w.three.pairs {
		if pair == nil || key == active || w.pairs[key].excluded {
			continue
		}
		if ok, err := w.threeOtherPairClear(ctx, pair, kicked, result, dt); err != nil {
			return nil, err
		} else if !ok {
			return w.threeUndecided(key, "third-body pair lacks a clear response path"), nil
		}
	}
	result.Excluded = w.Excluded()
	next := withPairState(from, *result.Next)
	result.Next = &next
	trace := result.Trace
	trace.start = from
	trace.end = *result.Next
	if trace.hasEvent {
		trace.pre = withPairState(from, trace.pre)
		trace.post = withPairState(from, trace.post)
	}
	result.Trace = trace
	return result, nil
}

func (w *World) threeUndecided(key int, reason string) *StepReport {
	report := undecided(w, reason)
	if key >= 0 && key < len(threePairs) {
		report.Diagnostics[0].Pair = w.bodyPair(w.pairs[key])
	}
	return report
}

func (w *World) threeOtherPairClear(ctx context.Context, pair *World, kicked State,
	active *StepReport, dt units.Value) (bool, error) {
	start := pairState(kicked, pair)
	end := pairState(withPairState(kicked, *active.Next), pair)
	if !active.Trace.hasEvent {
		return threeClearSegment(ctx, pair, start, end, dt)
	}
	pre := pairState(withPairState(kicked, active.Trace.pre), pair)
	post := pairState(withPairState(kicked, active.Trace.post), pair)
	elapsed := active.Trace.eventAt.Base()
	if elapsed > 0 {
		separated, err := threeClearSegment(ctx, pair, start, pre, units.Seconds(elapsed))
		if err != nil || !separated {
			return separated, err
		}
	}
	// Position correction is a separate path, even though it consumes no step time.
	if pre.entries[0].Pose != post.entries[0].Pose || pre.entries[1].Pose != post.entries[1].Pose {
		separated, err := threeClearCorrection(ctx, pair, pre, post)
		if err != nil || !separated {
			return separated, err
		}
	}
	remaining := dt.Base() - elapsed
	if remaining > 0 {
		return threeClearSegment(ctx, pair, post, end, units.Seconds(remaining))
	}
	return threeClearEndpoint(ctx, pair, end)
}

func threeClearCorrection(ctx context.Context, pair *World, pre, post State) (bool, error) {
	// The correction is the actual pose-to-pose path. Incoming velocity does
	// not continue during its arbitrary sweep parameter interval.
	sweep, err := pair.sweepPoses(ctx, pre, post, units.Seconds(1), decad.StopAtInitialContact)
	if err != nil {
		return false, err
	}
	if sweep.Outcome != decad.SweepClear {
		return false, nil
	}
	return threeClearEndpoint(ctx, pair, post)
}

func threeClearSegment(ctx context.Context, pair *World, start, end State,
	duration units.Value) (bool, error) {
	ideal, err := pair.sweep(ctx, start, duration, decad.StopAtInitialContact)
	if err != nil {
		return false, err
	}
	if ideal.Outcome != decad.SweepClear {
		return false, nil
	}
	rounded, err := pair.sweepPoses(ctx, start, end, duration, decad.StopAtInitialContact)
	if err != nil {
		return false, err
	}
	if rounded.Outcome != decad.SweepClear {
		return false, nil
	}
	return threeClearEndpoint(ctx, pair, end)
}

func threeClearEndpoint(ctx context.Context, pair *World, state State) (bool, error) {
	contact, err := pair.doc.ContactPair(ctx, pair.bodies[0].definition.Body,
		pair.bodies[1].definition.Body, state.entries[0].Pose, state.entries[1].Pose,
		pair.step.Contact)
	if err != nil {
		return false, err
	}
	return contact.Relation == decad.ContactSeparated, nil
}
