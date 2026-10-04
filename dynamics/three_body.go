package dynamics

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

var threePairs = [3][2]int{{0, 1}, {0, 2}, {1, 2}}

// threeBodyWorld keeps each possible response pair in world order. The current
// three-body step admits one dynamic body and two fixed bodies; every other
// pair must have a certified clear path before a result is published.
type threeBodyWorld struct {
	parts    [3]RigidBody
	pairs    [3]*World
	excluded []BodyPair
	dynamic  int
}

func newThreeBodyWorld(ctx context.Context, doc *decad.Document, cfg WorldConfig) (*World, error) {
	three := &threeBodyWorld{dynamic: -1}
	copy(three.parts[:], cfg.Bodies)
	live := doc.Bodies()
	indices := make(map[*decad.Body]int, 3)
	for i, part := range three.parts {
		if part.Body == nil {
			return nil, fmt.Errorf("%w: nil body", ErrInvalidInput)
		}
		if part.Body.Document() != doc || !containsBody(live, part.Body) {
			return nil, fmt.Errorf("%w: body is foreign or retired", ErrInvalidInput)
		}
		if _, duplicate := indices[part.Body]; duplicate {
			return nil, fmt.Errorf("%w: duplicate body", ErrInvalidInput)
		}
		indices[part.Body] = i
		if part.Body.Kind() != decad.BodySolid || !part.Body.IsSolid() {
			return nil, fmt.Errorf("%w: body is not a sound solid", ErrInvalidInput)
		}
		if err := validateMaterial(part.Material); err != nil {
			return nil, err
		}
		if part.Role == Dynamic {
			if three.dynamic >= 0 {
				return nil, fmt.Errorf("%w: three-body response with two dynamic bodies", ErrUnsupported)
			}
			three.dynamic = i
		} else if part.Role != Fixed {
			return nil, fmt.Errorf("%w: three-body kinematic response", ErrUnsupported)
		}
	}
	if three.dynamic < 0 {
		return nil, fmt.Errorf("%w: three-body world needs a dynamic body", ErrUnsupported)
	}
	excluded := make(map[int]struct{}, len(cfg.Excluded))
	for _, pair := range cfg.Excluded {
		key, ok := threePairIndex(indices, pair)
		if !ok {
			return nil, fmt.Errorf("%w: exclusion names an unknown pair", ErrInvalidInput)
		}
		if _, duplicate := excluded[key]; duplicate {
			return nil, fmt.Errorf("%w: duplicate pair exclusion", ErrInvalidInput)
		}
		excluded[key] = struct{}{}
	}
	overrides := make(map[int]PairMaterial, len(cfg.Overrides))
	for _, override := range cfg.Overrides {
		key, ok := threePairIndex(indices, override.Pair)
		if !ok {
			return nil, fmt.Errorf("%w: override names an unknown pair", ErrInvalidInput)
		}
		if _, duplicate := overrides[key]; duplicate {
			return nil, fmt.Errorf("%w: duplicate pair override", ErrInvalidInput)
		}
		if _, hidden := excluded[key]; hidden {
			return nil, fmt.Errorf("%w: excluded pair has a material override", ErrInvalidInput)
		}
		if err := validateMaterial(Material{Restitution: override.Restitution, Friction: override.Friction}); err != nil {
			return nil, err
		}
		overrides[key] = override
	}
	for key, pair := range threePairs {
		a, b := three.parts[pair[0]], three.parts[pair[1]]
		canonical := BodyPair{A: a.Body, B: b.Body}
		if _, skip := excluded[key]; skip {
			three.excluded = append(three.excluded, canonical)
		}
		if pair[0] != three.dynamic && pair[1] != three.dynamic {
			continue
		}
		pairCfg := WorldConfig{Bodies: []RigidBody{a, b}, Step: cfg.Step}
		if _, skip := excluded[key]; skip {
			pairCfg.Excluded = []BodyPair{canonical}
		}
		if override, present := overrides[key]; present {
			pairCfg.Overrides = []PairMaterial{override}
		}
		child, err := NewWorld(ctx, doc, pairCfg)
		if err != nil {
			return nil, err
		}
		three.pairs[key] = child
	}
	// Every body belongs to a validated dynamic/fixed child world. The fixed
	// pair still receives a real ContactPair query during each step.
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return &World{doc: doc, step: cfg.Step, three: three}, nil
}

func threePairIndex(indices map[*decad.Body]int, pair BodyPair) (int, bool) {
	a, okA := indices[pair.A]
	b, okB := indices[pair.B]
	if !okA || !okB || a == b {
		return 0, false
	}
	for key, ordered := range threePairs {
		if ordered == [2]int{a, b} || ordered == [2]int{b, a} {
			return key, true
		}
	}
	return 0, false
}

func (w *World) newThreeBodyState(entries []BodyState) (State, error) {
	if len(entries) != 3 {
		return State{}, fmt.Errorf("%w: state requires exactly three bodies", ErrInvalidInput)
	}
	byBody := make(map[*decad.Body]BodyState, 3)
	for _, entry := range entries {
		if entry.Body == nil {
			return State{}, fmt.Errorf("%w: nil state body", ErrInvalidInput)
		}
		if _, duplicate := byBody[entry.Body]; duplicate {
			return State{}, fmt.Errorf("%w: duplicate state body", ErrInvalidInput)
		}
		byBody[entry.Body] = entry
	}
	if len(byBody) != 3 {
		return State{}, fmt.Errorf("%w: state body count", ErrInvalidInput)
	}
	ordered := State{world: w, hasThird: true}
	for i, part := range w.three.parts {
		entry, present := byBody[part.Body]
		if !present {
			return State{}, fmt.Errorf("%w: state contains an unknown body", ErrInvalidInput)
		}
		if i < 2 {
			ordered.entries[i] = entry
		} else {
			ordered.third = entry
		}
	}
	for _, pair := range w.three.pairs {
		if pair == nil {
			continue
		}
		_, err := pair.NewState([]BodyState{
			byBody[pair.parts[0].definition.Body], byBody[pair.parts[1].definition.Body],
		})
		if err != nil {
			return State{}, err
		}
	}
	return ordered, nil
}

func pairState(state State, pair *World) State {
	first, _ := state.Body(pair.parts[0].definition.Body)
	second, _ := state.Body(pair.parts[1].definition.Body)
	return State{world: pair, entries: [2]BodyState{first, second}}
}

func withPairState(original State, pairState State) State {
	out := original
	for _, entry := range pairState.entries {
		for i := range out.entries {
			if out.entries[i].Body == entry.Body {
				out.entries[i] = entry
			}
		}
		if out.third.Body == entry.Body {
			out.third = entry
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
	for _, part := range w.three.parts {
		if !containsBody(live, part.Body) {
			return nil, fmt.Errorf("%w: world body was retired", ErrInvalidInput)
		}
	}
	var reference *World
	for _, pair := range w.three.pairs {
		if pair != nil {
			reference = pair
			break
		}
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
		return undecided(w, "force kick exceeds the velocity residual"), nil
	}
	kicked := withPairState(from, kickPair)
	active := -1
	for key, indices := range threePairs {
		if threePairExcluded(w.three.excluded, w.three.parts, indices) {
			continue
		}
		pair := w.three.pairs[key]
		if pair == nil {
			a, _ := kicked.Body(w.three.parts[indices[0]].Body)
			b, _ := kicked.Body(w.three.parts[indices[1]].Body)
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
		if active >= 0 {
			return w.threeUndecided(key, "more than one three-body pair may contact during the step"), nil
		}
		if first.Outcome == decad.SweepUndecided {
			return w.threeUndecided(key, "a three-body pair sweep is undecided"), nil
		}
		active = key
	}
	if active < 0 {
		for key, pair := range w.three.pairs {
			if pair != nil {
				active = key
				break
			}
		}
	}
	chosen := w.three.pairs[active]
	if chosen == nil {
		return undecided(w, "no three-body response pair is available"), nil
	}
	result, err := chosen.Step(ctx, pairState(from, chosen), input, dt)
	if err != nil || result == nil || result.Status != Advanced || result.Next == nil {
		if result != nil {
			result.Excluded = w.Excluded()
		}
		return result, err
	}
	for key, pair := range w.three.pairs {
		if pair == nil || key == active ||
			threePairExcluded(w.three.excluded, w.three.parts, threePairs[key]) {
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

func threePairExcluded(excluded []BodyPair, parts [3]RigidBody, pair [2]int) bool {
	a, b := parts[pair[0]].Body, parts[pair[1]].Body
	for _, item := range excluded {
		if item.A == a && item.B == b {
			return true
		}
	}
	return false
}

func (w *World) threeUndecided(key int, reason string) *StepReport {
	report := undecided(w, reason)
	if key >= 0 && key < len(threePairs) {
		indices := threePairs[key]
		report.Diagnostics[0].Pair = BodyPair{
			A: w.three.parts[indices[0]].Body, B: w.three.parts[indices[1]].Body,
		}
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
	contact, err := pair.doc.ContactPair(ctx, pair.parts[0].definition.Body,
		pair.parts[1].definition.Body, state.entries[0].Pose, state.entries[1].Pose,
		pair.step.Contact)
	if err != nil {
		return false, err
	}
	return contact.Relation == decad.ContactSeparated, nil
}
