package dynamics

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the event schedule of a world of four or more bodies
// (docs/multibody-dynamics-design.md §5): one full-step kick, then slices of
// drift from event to event. Each slice sweeps the broad phase's candidate
// pairs from its start to the end of the step; the earliest event cuts it,
// every body advances to that event on its certified path, the event's
// islands are solved, and the next slice starts from the post-event state.

// sliceSweeps is one slice's broad phase and pair sweeps: every body's path,
// the candidate pairs in canonical order, and each candidate's sweep.
type sliceSweeps struct {
	paths      []decad.PairPath
	candidates []int
	swept      map[int]*decad.SweepReport
}

// scheduleRun is one scheduled step in progress: its fixed inputs and the
// certified prefix built so far. It lives for one Step call only, so the
// World stays immutable and Step stays safe for concurrent use.
type scheduleRun struct {
	w         *World
	from      State
	kicked    State
	dt        units.Value
	drivers   []decad.PoseSegment // full-step kinematic drivers, world order
	scheduled map[int]struct{}
	gravity   QuantityVec
	loads     []*BodyLoad

	at       units.Value // held start of the current slice, from the step start
	state    State       // the certified state at the slice start
	policies map[int]decad.SweepStartPolicy
	stalled  int       // consecutive event times that did not advance the clock
	work     *stepWork // the step's SweptBox and SweepPair calls, reuse and budget

	// The certified prefix: slices and event times, the published events and
	// islands, the drift slices for the conservation readings, and the last
	// certified time and state.
	slices    []traceSlice
	events    []traceEvent
	published []ContactEvent
	islands   []IslandReport
	drift     [][2]State
	prefixAt  units.Value
	prefixEnd State
}

// stepScheduled is the step of a world of four or more bodies
// (docs/multibody-dynamics-design.md §5).
func (w *World) stepScheduled(ctx context.Context, from State, input StepInput,
	dt units.Value) (*StepReport, error) {
	if err := validateQuantityVec(input.Gravity, units.Acceleration); err != nil {
		return nil, err
	}
	loads, err := w.validateWorldLoads(input.Loads)
	if err != nil {
		return nil, err
	}
	drivers, err := w.validateWorldDrivers(from, input.Drivers, dt)
	if err != nil {
		return nil, err
	}
	live := w.doc.Bodies()
	for _, body := range w.bodies {
		if !containsBody(live, body.definition.Body) {
			return nil, fmt.Errorf("%w: world body was retired", ErrInvalidInput)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	run := &scheduleRun{w: w, from: from, dt: dt, drivers: drivers, scheduled: w.pairSchedule(),
		gravity: input.Gravity, loads: loads, at: units.Seconds(0), policies: map[int]decad.SweepStartPolicy{},
		prefixAt: units.Seconds(0), prefixEnd: from, work: newStepWork(w, from)}
	kicked, ok := w.kickByLoads(from, input.Gravity, loads, dt)
	if !ok {
		return run.undecided(run.diagnostic(StepKickUnbounded, BodyPair{},
			"force kick or torque kick exceeds its velocity residual")), nil
	}
	run.kicked, run.state = kicked, kicked
	run.policies = w.carriedContacts(from, kicked)
	diagnostics, err := w.fixedPairRelations(ctx, kicked)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) != 0 {
		for i := range diagnostics {
			diagnostics[i].From, diagnostics[i].To = units.Seconds(0), dt
		}
		return run.undecided(diagnostics...), nil
	}
	report, err := run.execute(ctx)
	if errors.Is(err, errPairBudget) {
		d := scheduleDiagnostic(StepPairBudget, BodyPair{},
			fmt.Sprintf("SweptBox and SweepPair calls exceed MaxPairSweeps %d", w.step.MaxPairSweeps))
		d.From, d.To, d.Limit = run.prefixAt, dt, units.Scalar(float64(w.step.MaxPairSweeps))
		return run.undecided(d), nil
	}
	return report, err
}

// carriedContacts is §5 step 2: the contact set of the input state, less
// every pair the kick changed. A pair stays when neither body is Kinematic
// and the kick left both bodies' entries as the previous step published
// them, so the persistent track that ended that step continues under the
// same velocities; a kicked pair starts afresh under StopAtInitialContact
// and reaches its island as an initial contact.
func (w *World) carriedContacts(from, kicked State) map[int]decad.SweepStartPolicy {
	policies := make(map[int]decad.SweepStartPolicy, len(from.contacts))
	for _, key := range from.contacts {
		pair := w.pairs[key]
		if w.bodies[pair.a].definition.Role == Kinematic || w.bodies[pair.b].definition.Role == Kinematic ||
			kicked.entries[pair.a] != from.entries[pair.a] || kicked.entries[pair.b] != from.entries[pair.b] {
			continue
		}
		policies[key] = decad.ContinueCertifiedTouch
	}
	return policies
}

// execute runs slices until the step completes or stops.
func (r *scheduleRun) execute(ctx context.Context) (*StepReport, error) {
	for {
		if exactBase(r.at).Cmp(exactBase(r.dt)) >= 0 {
			// An event at the step end leaves no time to drift.
			return r.complete(ctx, r.state, nil)
		}
		remaining := r.remaining()
		paths := r.w.slicePaths(r.state, r.sliceDrivers(remaining), remaining)
		sweeps, diagnostics, err := r.w.sweepSlice(ctx, r.work, paths, remaining, r.scheduled, r.policies)
		if err != nil {
			return nil, err
		}
		if len(diagnostics) != 0 {
			return r.undecided(r.timed(diagnostics)...), nil
		}
		plan, diagnostics := r.classify(sweeps)
		if len(diagnostics) != 0 {
			return r.undecided(diagnostics...), nil
		}
		if plan.cut == nil {
			return r.finish(ctx, sweeps, plan)
		}
		diagnostics, err = r.advance(ctx, sweeps, plan)
		if err != nil {
			return nil, err
		}
		if len(diagnostics) != 0 {
			return r.undecided(diagnostics...), nil
		}
	}
}

// remaining is the held span of the next slice, from its start to the end of
// the step. It is the paths' duration; when the exact difference is not a
// float the nearest one labels it, and the held clock still maps the slice
// onto [at, dt] exactly.
func (r *scheduleRun) remaining() units.Value {
	span := new(big.Rat).Sub(exactBase(r.dt), exactBase(r.at))
	seconds, _ := span.Float64()
	return units.Seconds(seconds)
}

// sliceDrivers slices each kinematic driver to the rest of the step: from the
// body's current pose to the driver's end pose (§5 step 3).
func (r *scheduleRun) sliceDrivers(remaining units.Value) []decad.PoseSegment {
	if r.at.Base() == 0 {
		return r.drivers
	}
	out := make([]decad.PoseSegment, len(r.drivers))
	for i, driver := range r.drivers {
		if r.w.bodies[i].definition.Role != Kinematic {
			continue
		}
		out[i] = decad.PoseSegment{From: r.state.entries[i].Pose, To: driver.To, Duration: remaining}
	}
	return out
}

// slicePlan is one slice's classified events. A cutting event (an impact or
// transition bracket at its exact right fraction, or an initial contact at
// fraction zero) ends the slice at the earliest such fraction, cut; a graze
// before cut is published without ending the slice, since its pair's sweep
// replays the whole slice.
type slicePlan struct {
	cut       *big.Rat
	instant   decad.SweepInstant // the slice instant at cut
	at        []int              // candidate keys whose cutting event lies at cut, canonical order
	grazes    []int              // candidate keys whose graze lies before cut, canonical order
	fractions map[int]*big.Rat
	bands     map[int]struct{} // candidate keys whose cutting event is a band track's end (§10.3)
}

// classify reads every candidate sweep of a slice (§5 step 4).
func (r *scheduleRun) classify(sweeps sliceSweeps) (slicePlan, []StepDiagnostic) {
	plan := slicePlan{fractions: map[int]*big.Rat{}, bands: map[int]struct{}{}}
	var diagnostics []StepDiagnostic
	var cutting, grazing []int
	one := big.NewRat(1, 1)
	for _, key := range sweeps.candidates {
		sweep, pair := sweeps.swept[key], r.w.bodyPair(r.w.pairs[key])
		policy := r.policyOf(key)
		switch sweep.Outcome {
		case decad.SweepClear, decad.SweepDepartedClear:
		case decad.SweepPersistentTouch:
			if policy != decad.ContinueCertifiedTouch || !r.w.fullTrackWithin(sweep) {
				d := r.diagnostic(StepTrackUnproved, pair,
					"persistent track does not span the slice within the penetration residual")
				d.Limit = r.w.step.PenetrationResidual
				diagnostics = append(diagnostics, d)
			}
		case decad.SweepPersistentBand:
			// §10.3: a band track continues the pair while its depth stays
			// within PenetrationResidual; the slice ends where it no longer
			// does, or at the track's end, and the pair enters an island there.
			cut, full, ok := r.w.bandEnd(sweep)
			if policy != decad.ContinueCertifiedTouch || !ok {
				d := r.diagnostic(StepTrackUnproved, pair,
					"band track does not stay within the penetration residual for any positive time")
				d.Limit = r.w.step.PenetrationResidual
				diagnostics = append(diagnostics, d)
				continue
			}
			if full {
				continue
			}
			plan.fractions[key] = cut
			plan.bands[key] = struct{}{}
			cutting = append(cutting, key)
		case decad.SweepInitiallyTouching, decad.SweepInitiallyOverlapping:
			if policy != decad.StopAtInitialContact {
				diagnostics = append(diagnostics, r.diagnostic(StepTrackUnproved, pair,
					fmt.Sprintf("a pair continued in contact starts %v", sweep.Outcome)))
				continue
			}
			plan.fractions[key] = new(big.Rat)
			cutting = append(cutting, key)
		case decad.SweepImpactBracket, decad.SweepContactTransitionBracket:
			f := bracketRight(sweep)
			if f == nil || f.Sign() <= 0 || f.Cmp(one) > 0 || sweep.Event == nil {
				diagnostics = append(diagnostics, r.diagnostic(StepPairUndecided, pair,
					fmt.Sprintf("%v has no bracket inside the slice", sweep.Outcome)))
				continue
			}
			plan.fractions[key] = f
			cutting = append(cutting, key)
		case decad.SweepGrazingTouch:
			var f *big.Rat
			if sweep.Event != nil {
				f = exactBase(sweep.Event.At.Fraction)
			}
			if f == nil || f.Sign() <= 0 || f.Cmp(one) >= 0 {
				diagnostics = append(diagnostics, r.diagnostic(StepPairUndecided, pair,
					"graze has no instant inside the slice"))
				continue
			}
			plan.fractions[key] = f
			grazing = append(grazing, key)
		case decad.SweepUndecided:
			d := r.diagnostic(StepPairUndecided, pair, fmt.Sprintf("candidate pair sweep is undecided (%v)", sweep.Cause))
			if sweep.Unresolved != nil {
				d.From, d.To = r.sliceTime(sweep.Unresolved.From.Fraction), r.sliceTime(sweep.Unresolved.To.Fraction)
			}
			diagnostics = append(diagnostics, d)
		default:
			diagnostics = append(diagnostics, r.diagnostic(StepUnsupported, pair,
				fmt.Sprintf("candidate pair sweep returned %v", sweep.Outcome)))
		}
	}
	for _, key := range cutting {
		if f := plan.fractions[key]; plan.cut == nil || f.Cmp(plan.cut) < 0 {
			plan.cut = f
		}
	}
	for _, key := range cutting {
		if plan.fractions[key].Cmp(plan.cut) != 0 {
			continue
		}
		if len(plan.at) == 0 {
			sweep := sweeps.swept[key]
			switch _, band := plan.bands[key]; {
			case band:
				plan.instant = fractionInstant(plan.cut, pathDuration(sweep.PathA))
			case sweep.Bracket != nil:
				plan.instant = sweep.Bracket.To
			default:
				plan.instant = sweep.Event.At
			}
		}
		plan.at = append(plan.at, key)
	}
	for _, key := range grazing {
		switch f := plan.fractions[key]; {
		case plan.cut == nil || f.Cmp(plan.cut) < 0:
			plan.grazes = append(plan.grazes, key)
		case f.Cmp(plan.cut) == 0:
			diagnostics = append(diagnostics, r.diagnostic(StepUnsupported, r.w.bodyPair(r.w.pairs[key]),
				"a graze coincides with another event"))
		}
	}
	return plan, diagnostics
}

// bracketRight is the exact right fraction of a sweep's bracket, as the
// public Fraction states it. A Fraction that rounds above the proved right
// endpoint fails the replay that every advance runs at it.
func bracketRight(sweep *decad.SweepReport) *big.Rat {
	if sweep.Bracket == nil {
		return nil
	}
	return exactBase(sweep.Bracket.To.Fraction)
}

func (r *scheduleRun) policyOf(key int) decad.SweepStartPolicy {
	if policy, ok := r.policies[key]; ok {
		return policy
	}
	return decad.StopAtInitialContact
}

// sliceTime maps a slice fraction onto the step clock, as a label.
func (r *scheduleRun) sliceTime(fraction units.Value) units.Value {
	f := exactBase(fraction)
	if f == nil {
		return r.at
	}
	span := new(big.Rat).Sub(exactBase(r.dt), exactBase(r.at))
	at := new(big.Rat).Add(exactBase(r.at), span.Mul(span, f))
	seconds, _ := at.Float64()
	return units.Seconds(seconds)
}

// eventLabel is the held time of the event at slice fraction f: the exact
// time when it is a float, else the float just below it, so every time the
// prefix replays maps to a fraction at or below f. A label that does not
// pass the slice start cannot order the event and is refused.
func (r *scheduleRun) eventLabel(f *big.Rat) (units.Value, bool) {
	switch {
	case f.Sign() == 0:
		return r.at, true
	case f.Cmp(big.NewRat(1, 1)) == 0:
		return r.dt, true
	}
	start := exactBase(r.at)
	span := new(big.Rat).Sub(exactBase(r.dt), start)
	exact := new(big.Rat).Add(start, span.Mul(span, f))
	seconds, _ := exact.Float64()
	if ratFloat(seconds).Cmp(exact) > 0 {
		seconds = math.Nextafter(seconds, math.Inf(-1))
	}
	label := units.Seconds(seconds)
	return label, exactBase(label).Cmp(start) > 0
}

// finish completes a slice that holds no cutting event: every body reaches
// the end of the step on its certified path (§5 step 5).
func (r *scheduleRun) finish(ctx context.Context, sweeps sliceSweeps, plan slicePlan) (*StepReport, error) {
	end, proofs, diagnostics, err := r.w.slicePoses(ctx, r.work, r.state, sweeps, big.NewRat(1, 1), r.scheduled)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) != 0 {
		return r.undecided(r.timed(diagnostics)...), nil
	}
	if diagnostics := r.publishGrazes(sweeps, plan); len(diagnostics) != 0 {
		return r.undecided(diagnostics...), nil
	}
	r.recordSlice(sweeps, proofs, r.dt, end)
	return r.complete(ctx, end, &sweeps)
}

// complete checks the contact set at the completed poses and publishes. The
// published state carries the contact set that rests at the end of the step
// and the step's cache (§3.2): last holds the final slice's sweeps, or is
// nil when an event at the step's end completed it.
func (r *scheduleRun) complete(ctx context.Context, end State, last *sliceSweeps) (*StepReport, error) {
	diagnostics, err := r.w.completedContacts(ctx, end, r.policies)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) != 0 {
		return r.undecided(r.timed(diagnostics)...), nil
	}
	end.contacts = r.restingContacts(last)
	end.cache = r.work.cache(end.entries)
	trace := Trace{start: r.from, end: end, duration: r.dt, slices: r.slices, events: r.events, scheduled: true,
		pairCalls: r.work.calls}
	return r.publish(end, trace), nil
}

// restingContacts is the contact set the next step reads (§5 step 2): every
// pair continued under ContinueCertifiedTouch whose final sweep is a
// persistent touch track through the end of the step, or a band track that
// spans it within PenetrationResidual (§10.3), or, when an event at the
// end completed the step, every pair that event left under
// ContinueCertifiedTouch. Keys are in canonical order.
func (r *scheduleRun) restingContacts(last *sliceSweeps) []int {
	var out []int
	for key, policy := range r.policies {
		if policy != decad.ContinueCertifiedTouch {
			continue
		}
		if last != nil {
			sweep, ok := last.swept[key]
			if !ok || !r.w.continuesTrack(sweep) {
				continue
			}
		}
		out = append(out, key)
	}
	slices.Sort(out)
	return out
}

// recordSlice appends the slice from the current state to end, at held time
// to, and moves the certified prefix there.
func (r *scheduleRun) recordSlice(sweeps sliceSweeps, proofs []pairProof, to units.Value, end State) {
	r.slices = append(r.slices, traceSlice{start: r.at, end: to, span: r.dt, from: r.state, to: end,
		paths: sweeps.paths, proofs: proofs})
	r.drift = append(r.drift, [2]State{r.state, end})
	r.prefixAt, r.prefixEnd = to, end
}

// advance moves every body to the slice's earliest event, solves the event's
// islands and starts the next slice from the post-event state (§5 steps 6–8).
// It returns the diagnostics that stop the step, if any.
func (r *scheduleRun) advance(ctx context.Context, sweeps sliceSweeps, plan slicePlan) ([]StepDiagnostic, error) {
	label, ok := r.eventLabel(plan.cut)
	if !ok {
		return []StepDiagnostic{r.diagnostic(StepUnsupported, r.w.bodyPair(r.w.pairs[plan.at[0]]),
			"event time does not advance the step clock")}, nil
	}
	pre := r.state
	if plan.cut.Sign() > 0 {
		r.stalled = 0
		var proofs []pairProof
		var diagnostics []StepDiagnostic
		var err error
		pre, proofs, diagnostics, err = r.w.slicePoses(ctx, r.work, r.state, sweeps, plan.cut, r.scheduled)
		if err != nil {
			return nil, err
		}
		if len(diagnostics) != 0 {
			return r.timed(diagnostics), nil
		}
		if diagnostics := r.publishGrazes(sweeps, plan); len(diagnostics) != 0 {
			return diagnostics, nil
		}
		r.recordSlice(sweeps, proofs, label, pre)
	} else {
		// §5.1: a zero-time repeat is legal only as a resting solve, which
		// moves the pair into a continuation policy; a third event time at
		// one clock reading has made no progress.
		r.stalled++
		if r.stalled > 2 {
			return []StepDiagnostic{r.diagnostic(StepUnsupported, r.w.bodyPair(r.w.pairs[plan.at[0]]),
				"events repeat at one time without progress")}, nil
		}
	}
	event, diagnostics, err := r.solveEvent(ctx, sweeps, plan, pre, label)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) != 0 {
		for i := range diagnostics {
			if diagnostics[i].From.Kind() == units.Time {
				continue
			}
			// A refusal at the event names its time; the budget names the
			// time it leaves undone.
			diagnostics[i].From, diagnostics[i].To = label, label
			if diagnostics[i].Code == StepEventBudget {
				diagnostics[i].To = r.dt
			}
		}
		return diagnostics, nil
	}
	if event != nil {
		r.events = append(r.events, *event)
		r.prefixAt, r.prefixEnd = label, event.post
		r.state = event.post
	} else {
		r.state = pre
	}
	r.at = label
	// §12: reaching MaxEvents with time remaining stops the step after the
	// event that reached it, which stays in the certified prefix.
	if len(r.published) >= r.w.step.MaxEvents && exactBase(label).Cmp(exactBase(r.dt)) < 0 {
		return []StepDiagnostic{r.eventBudget(label)}, nil
	}
	return nil, nil
}

// diagnostic names a code over the current slice: from its start to the end
// of the step.
func (r *scheduleRun) diagnostic(code StepReason, pair BodyPair, reason string) StepDiagnostic {
	return StepDiagnostic{Code: code, Pair: pair, From: r.at, To: r.dt, Reason: reason}
}

// timed fills the time interval of diagnostics raised without one.
func (r *scheduleRun) timed(diagnostics []StepDiagnostic) []StepDiagnostic {
	for i := range diagnostics {
		if diagnostics[i].From.Kind() != units.Time {
			diagnostics[i].From, diagnostics[i].To = r.at, r.dt
		}
	}
	return diagnostics
}

// undecided stops the step. The report carries every diagnostic and, in its
// Trace, the certified prefix up to the last certified time (§12).
func (r *scheduleRun) undecided(diagnostics ...StepDiagnostic) *StepReport {
	return &StepReport{Status: Undecided, Excluded: r.w.Excluded(), Diagnostics: diagnostics,
		Events: r.published, Islands: r.islands,
		Trace: Trace{start: r.from, end: r.prefixEnd, duration: r.prefixAt, slices: r.slices, events: r.events,
			scheduled: true, pairCalls: r.work.calls}}
}

// publish attaches the conservation readings of an advanced step.
func (r *scheduleRun) publish(end State, trace Trace) *StepReport {
	w := r.w
	input, okInput := w.conservationState(r.from)
	afterKick, okKick := w.conservationState(r.kicked)
	completion, okEnd := w.conservationState(end)
	gravityImpulse, loadImpulse, okForce := w.forceImpulses(r.gravity, r.loads, r.dt)
	torqueImpulse, okTorque := w.torqueImpulse(r.loads, r.dt)
	contactImpulse, okContact := w.islandContactImpulse(r.published)
	driftChange, okDrift := w.driftConservationSlices(r.drift)
	kinematicWork, okWork := w.islandKinematicWork(r.published)
	if !okInput || !okKick || !okEnd || !okForce || !okTorque || !okContact || !okDrift || !okWork {
		return r.undecided(r.diagnostic(StepConservationFailed, BodyPair{},
			"conservation readings cannot be represented with finite bounds"))
	}
	return &StepReport{Status: Advanced, Next: &end, Events: r.published, Islands: r.islands,
		Excluded: w.Excluded(), Trace: trace,
		Conservation: &StepConservation{Input: input, AfterKick: afterKick, Completion: completion,
			GravityImpulse: gravityImpulse, LoadImpulse: loadImpulse, ContactImpulse: contactImpulse,
			TorqueImpulse: torqueImpulse, KinematicWork: kinematicWork, DriftChange: driftChange}}
}

// sweepSlice runs §4.3's broad phase over every body's slice path and sweeps
// each candidate pair under its start policy: the contact-set policy in
// policies, or StopAtInitialContact. Every swept box and sweep goes through
// work, which reuses a call it has seen and charges every other one.
func (w *World) sweepSlice(ctx context.Context, work *stepWork, paths []decad.PairPath, duration units.Value,
	scheduled map[int]struct{}, policies map[int]decad.SweepStartPolicy) (sliceSweeps, []StepDiagnostic, error) {
	boxes := make([]decad.SweptBox, len(w.bodies))
	for i := range w.bodies {
		if err := ctx.Err(); err != nil {
			return sliceSweeps{}, nil, err
		}
		box, err := work.sweptBox(ctx, i, paths[i])
		if errors.Is(err, decad.ErrUnsupported) {
			return sliceSweeps{}, []StepDiagnostic{scheduleDiagnostic(StepTravelUnbounded, BodyPair{},
				fmt.Sprintf("swept box of body %d is unbounded: %v", i, err))}, nil
		}
		if err != nil {
			return sliceSweeps{}, nil, err
		}
		boxes[i] = box
	}
	out := sliceSweeps{paths: paths, candidates: broadPhaseCandidates(boxes, scheduled),
		swept: make(map[int]*decad.SweepReport)}
	for _, key := range out.candidates {
		if err := ctx.Err(); err != nil {
			return sliceSweeps{}, nil, err
		}
		policy, ok := policies[key]
		if !ok {
			policy = decad.StopAtInitialContact
		}
		pair := w.pairs[key]
		sweep, err := work.sweepPair(ctx, key, paths[pair.a], paths[pair.b], w.sweepRequest(duration, policy))
		if err != nil {
			return sliceSweeps{}, nil, err
		}
		out.swept[key] = sweep
	}
	return out, nil, nil
}

// slicePoses publishes every body's pose at the exact slice fraction f
// (§4.3, §5 step 6): a swept body takes the pose its pair certificates
// replay there, and every certificate covering it must replay the same one;
// a body no sweep covers drifts on its own slice path (§5.4), as Trace.Sample
// replays it. Every pair the swept boxes excluded must keep its bounds
// strictly apart at the rounded poses (§7.1). It returns the state at f and
// every scheduled pair's proof.
func (w *World) slicePoses(ctx context.Context, work *stepWork, state State, sweeps sliceSweeps, f *big.Rat,
	scheduled map[int]struct{}) (State, []pairProof, []StepDiagnostic, error) {
	fraction, _ := f.Float64()
	if ratFloat(fraction).Cmp(f) != 0 {
		return State{}, nil, []StepDiagnostic{scheduleDiagnostic(StepUnsupported, BodyPair{},
			"event fraction is not a float")}, nil
	}
	out := state.clone()
	covered := make([]bool, len(out.entries))
	proofs := make([]pairProof, 0, len(scheduled))
	for key := range w.pairs {
		if _, ok := scheduled[key]; !ok {
			continue
		}
		sweep, ok := sweeps.swept[key]
		if !ok {
			proofs = append(proofs, pairProof{pair: key, boxClear: true})
			continue
		}
		// The published pose of every swept body is the one its pair
		// certificates replay; a refused or disagreeing pose stops the step.
		pair := w.pairs[key]
		poseA, poseB, err := sweep.CertifiedPosesAtInterval(units.Seconds(fraction), units.Seconds(0),
			units.Seconds(1))
		if err != nil {
			return State{}, nil, []StepDiagnostic{scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
				fmt.Sprintf("rounded slice poses lack the pair certificate: %v", err))}, nil
		}
		for _, side := range [2]struct {
			body int
			pose r3.Transform
		}{{pair.a, poseA}, {pair.b, poseB}} {
			if covered[side.body] && out.entries[side.body].Pose != side.pose {
				return State{}, nil, []StepDiagnostic{scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
					"pair certificates replay different slice poses for one body")}, nil
			}
			out.entries[side.body].Pose, covered[side.body] = side.pose, true
		}
		proofs = append(proofs, pairProof{pair: key, sweep: sweep})
	}
	for i := range out.entries {
		if covered[i] {
			continue
		}
		pose, err := pathPoseAt(sweeps.paths[i], f)
		if err != nil {
			return State{}, nil, []StepDiagnostic{scheduleDiagnostic(StepUnsupported, BodyPair{},
				fmt.Sprintf("body %d has a non-finite slice pose: %v", i, err))}, nil
		}
		out.entries[i].Pose = pose
	}
	key, err := w.boxExclusionsHold(ctx, work, out, proofs)
	if err != nil {
		return State{}, nil, nil, err
	}
	if key >= 0 {
		return State{}, nil, []StepDiagnostic{scheduleDiagnostic(StepPairUndecided, w.bodyPair(w.pairs[key]),
			"rounded poses of a box-excluded pair are not strictly apart")}, nil
	}
	return out, proofs, nil, nil
}

// fullTrackWithin requires a persistent track over the whole slice whose
// manifold, read at both ends and the middle, stays within the contact
// request and PenetrationResidual.
func (w *World) fullTrackWithin(sweep *decad.SweepReport) bool {
	track := sweep.ContactTrack
	if track == nil || track.Start().Fraction.Base() != 0 || track.End().Fraction.Base() != 1 {
		return false
	}
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(0.5), units.Scalar(1)} {
		manifold, err := track.ManifoldAt(fraction)
		if err != nil || !w.manifoldWithin(manifold) || !w.penetrationWithin(manifold) {
			return false
		}
	}
	return true
}

// penetrationWithin bounds every point's separation, value and bound, by
// PenetrationResidual.
func (w *World) penetrationWithin(manifold *decad.ContactManifold) bool {
	for _, p := range manifold.Points {
		if outwardSum(math.Abs(p.Separation.Value.Base()), p.Separation.Bound.Base()) >
			w.step.PenetrationResidual.Base() {
			return false
		}
	}
	return true
}

// completedContacts is §5 step 5 for the pairs that continued in touch: at
// the completed poses each must be separated, or touching or shallowly
// overlapping with a bounded manifold whose penetration lies within
// PenetrationResidual. Two co-moving bodies drift on separately rounded
// translations, so their completed poses may overlap by an ulp.
func (w *World) completedContacts(ctx context.Context, end State,
	policies map[int]decad.SweepStartPolicy) ([]StepDiagnostic, error) {
	keys := make([]int, 0, len(policies))
	for key := range policies {
		keys = append(keys, key)
	}
	slices.Sort(keys)
	for _, key := range keys {
		if policies[key] != decad.ContinueCertifiedTouch {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		pair := w.pairs[key]
		contact, err := w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body, w.bodies[pair.b].definition.Body,
			end.entries[pair.a].Pose, end.entries[pair.b].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
		switch {
		case contact.Relation == decad.ContactSeparated:
		case (contact.Relation == decad.ContactTouching || contact.Relation == decad.ContactOverlapping ||
			contact.Relation == decad.ContactBand && w.contactBandWithin(contact.Gap)) &&
			w.manifoldWithin(contact.Manifold) && w.penetrationWithin(contact.Manifold):
		default:
			d := scheduleDiagnostic(StepTrackUnproved, w.bodyPair(pair),
				fmt.Sprintf("completed contact relation is %v beyond the penetration residual", contact.Relation))
			d.Limit = w.step.PenetrationResidual
			return []StepDiagnostic{d}, nil
		}
	}
	return nil, nil
}

func scheduleDiagnostic(code StepReason, pair BodyPair, reason string) StepDiagnostic {
	return StepDiagnostic{Code: code, Pair: pair, Reason: reason}
}

// pairSchedule is §4.1's schedule: every pair that is not excluded and has a
// moving body, keyed by canonical pair index.
func (w *World) pairSchedule() map[int]struct{} {
	scheduled := make(map[int]struct{}, len(w.pairs))
	for key, pair := range w.pairs {
		if !pair.excluded && pair.moving {
			scheduled[key] = struct{}{}
		}
	}
	return scheduled
}

// fixedPairRelations queries every non-excluded Fixed/Fixed pair once at its
// constant poses (§3.1). An Overlapping or Undecided relation is reported.
func (w *World) fixedPairRelations(ctx context.Context, state State) ([]StepDiagnostic, error) {
	var diagnostics []StepDiagnostic
	for _, pair := range w.pairs {
		if pair.excluded || pair.moving {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		contact, err := w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body,
			w.bodies[pair.b].definition.Body, state.entries[pair.a].Pose, state.entries[pair.b].Pose,
			w.step.Contact)
		if err != nil {
			return nil, err
		}
		// §10.4: a fixed pair in a ContactBand may overlap by its gap band,
		// so the band must lie within PenetrationResidual.
		if contact.Relation == decad.ContactOverlapping || contact.Relation == decad.ContactUndecided ||
			contact.Relation == decad.ContactBand && !w.contactBandWithin(contact.Gap) {
			d := scheduleDiagnostic(StepFixedPairRelation, w.bodyPair(pair),
				fmt.Sprintf("fixed pair relation is %v", contact.Relation))
			if contact.Relation == decad.ContactBand {
				d.Limit = w.step.PenetrationResidual
			}
			diagnostics = append(diagnostics, d)
		}
	}
	return diagnostics, nil
}

// slicePaths builds each body's path over one slice (§5 step 3): a Fixed
// body a constant PoseSegment, a Kinematic body its sliced driver, a Dynamic
// body a RigidDriftSegment from its current pose, world mass center and
// velocities.
func (w *World) slicePaths(state State, drivers []decad.PoseSegment, duration units.Value) []decad.PairPath {
	paths := make([]decad.PairPath, len(w.bodies))
	for i, body := range w.bodies {
		entry := state.entries[i]
		switch body.definition.Role {
		case Fixed:
			paths[i] = decad.PoseSegment{From: entry.Pose, To: entry.Pose, Duration: duration}
		case Kinematic:
			paths[i] = drivers[i]
		default:
			paths[i] = decad.RigidDriftSegment{From: entry.Pose,
				Center:         entry.Pose.Apply(body.mass.Center.Value),
				LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
				Duration: duration}
		}
	}
	return paths
}

// validateWorldLoads returns one load slot per body in world order. Only a
// dynamic member takes a load, and at most one.
func (w *World) validateWorldLoads(entries []BodyLoad) ([]*BodyLoad, error) {
	loads := make([]*BodyLoad, len(w.bodies))
	for i := range entries {
		load := &entries[i]
		index, ok := w.index[load.Body]
		if !ok || w.bodies[index].definition.Role != Dynamic {
			return nil, fmt.Errorf("%w: load body is not a dynamic member of this world", ErrInvalidInput)
		}
		if loads[index] != nil {
			return nil, fmt.Errorf("%w: duplicate load body", ErrInvalidInput)
		}
		if err := validateQuantityVec(load.Force, units.Force); err != nil {
			return nil, err
		}
		if err := validateQuantityVec(load.Torque, units.Torque); err != nil {
			return nil, err
		}
		loads[index] = load
	}
	return loads, nil
}

// validateWorldDrivers returns each kinematic body's driver in world order.
// Every kinematic body needs exactly one PoseSegment driver whose duration is
// exactly dt and whose start is the state pose.
func (w *World) validateWorldDrivers(from State, drivers []KinematicDriver,
	dt units.Value) ([]decad.PoseSegment, error) {
	out := make([]decad.PoseSegment, len(w.bodies))
	driven := make([]bool, len(w.bodies))
	for _, driver := range drivers {
		index, ok := w.index[driver.Body]
		if !ok || w.bodies[index].definition.Role != Kinematic {
			return nil, fmt.Errorf("%w: driver body is not kinematic", ErrInvalidInput)
		}
		if driven[index] {
			return nil, fmt.Errorf("%w: duplicate driver body", ErrInvalidInput)
		}
		var path decad.PoseSegment
		switch supplied := driver.Path.(type) {
		case decad.PoseSegment:
			path = supplied
		case *decad.PoseSegment:
			if supplied == nil {
				return nil, fmt.Errorf("%w: nil kinematic path", ErrInvalidInput)
			}
			path = *supplied
		case nil:
			return nil, fmt.Errorf("%w: nil kinematic path", ErrInvalidInput)
		default:
			return nil, fmt.Errorf("%w: only PoseSegment kinematic drivers are implemented", ErrUnsupported)
		}
		statePose := from.entries[index].Pose
		if !validQuantity(path.Duration, units.Time, true) ||
			exactBase(path.Duration).Cmp(exactBase(dt)) != 0 ||
			!path.From.IsValid() || !path.To.IsValid() ||
			path.From.IsReflection() || path.To.IsReflection() ||
			path.From.Translation() != statePose.Translation() || !sameOrientation(path.From, statePose) {
			return nil, fmt.Errorf("%w: driver duration or start differs from the step", ErrInvalidInput)
		}
		out[index], driven[index] = path, true
	}
	for i, body := range w.bodies {
		if body.definition.Role == Kinematic && !driven[i] {
			return nil, fmt.Errorf("%w: exactly one driver is required for each kinematic body", ErrInvalidInput)
		}
	}
	return out, nil
}

// pathPoseAt evaluates a slice path at the exact fraction f of its duration
// with the float operations SweepPair's replay uses for the same path, so a
// body no sweep covers lands on the pose a sweep would replay. A rotating
// RigidDriftSegment turns about its center by |ω|·t and then translates by
// v·t, with t the duration fraction rounded once; a translating one moves by
// the exact displacement times f, rounded once. A PoseSegment returns its
// endpoints at f = 0 and 1, moves by its exact displacement times f when it
// only translates, and otherwise follows its screw at the rounded fraction.
func pathPoseAt(path decad.PairPath, f *big.Rat) (r3.Transform, error) {
	switch p := path.(type) {
	case decad.RigidDriftSegment:
		if f.Sign() == 0 {
			return p.From, nil
		}
		elapsed := new(big.Rat).Mul(exactBase(p.Duration), f)
		axis := r3.Vec{X: p.AngularVelocity.X.Base(), Y: p.AngularVelocity.Y.Base(),
			Z: p.AngularVelocity.Z.Base()}
		if axis == (r3.Vec{}) {
			return translateByRat(p.From, [3]*big.Rat{
				new(big.Rat).Mul(exactBase(p.LinearVelocity.X), elapsed),
				new(big.Rat).Mul(exactBase(p.LinearVelocity.Y), elapsed),
				new(big.Rat).Mul(exactBase(p.LinearVelocity.Z), elapsed)})
		}
		seconds, _ := elapsed.Float64()
		norm := math.Hypot(axis.X, math.Hypot(axis.Y, axis.Z))
		turn, err := r3.RotationAround(p.Center, axis, units.Radians(norm*seconds))
		if err != nil {
			return r3.Transform{}, err
		}
		pose, err := p.From.Then(turn)
		if err != nil {
			return r3.Transform{}, err
		}
		return translatePose(pose, r3.Vec{X: p.LinearVelocity.X.Base() * seconds,
			Y: p.LinearVelocity.Y.Base() * seconds, Z: p.LinearVelocity.Z.Base() * seconds})
	case decad.PoseSegment:
		if f.Sign() == 0 {
			return p.From, nil
		}
		if f.Cmp(big.NewRat(1, 1)) == 0 {
			return p.To, nil
		}
		if p.From.Basis() == p.To.Basis() {
			start, end := p.From.Translation(), p.To.Translation()
			var delta [3]*big.Rat
			for axis, pair := range [3][2]float64{{start.X, end.X}, {start.Y, end.Y}, {start.Z, end.Z}} {
				delta[axis] = new(big.Rat).Sub(ratFloat(pair[1]), ratFloat(pair[0]))
				delta[axis].Mul(delta[axis], f)
			}
			return translateByRat(p.From, delta)
		}
		inverse, err := p.From.Inverse()
		if err != nil {
			return r3.Transform{}, err
		}
		relative, err := inverse.Then(p.To)
		if err != nil {
			return r3.Transform{}, err
		}
		screw, err := relative.Screw()
		if err != nil {
			return r3.Transform{}, err
		}
		fraction, _ := f.Float64()
		step, err := screw.At(fraction)
		if err != nil {
			return r3.Transform{}, err
		}
		return p.From.Then(step)
	default:
		return r3.Transform{}, fmt.Errorf("%w: unknown slice path", ErrUnsupported)
	}
}

func translateByRat(pose r3.Transform, delta [3]*big.Rat) (r3.Transform, error) {
	x, _ := delta[0].Float64()
	y, _ := delta[1].Float64()
	z, _ := delta[2].Float64()
	return translatePose(pose, r3.Vec{X: x, Y: y, Z: z})
}
