package dynamics

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// traceSlice is one event-free interval of a step of a world of four or more
// bodies (docs/multibody-dynamics-design.md §3.4). Every body moves on
// paths[i]; every scheduled pair carries the rounded certificate that proves
// those paths, or the swept-box exclusion that made no sweep necessary.
type traceSlice struct {
	start, end units.Value // from the start of the step
	from, to   State
	paths      []decad.PairPath // world order, each over [start, end]
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

// traceEvent is one event of a step of a world of four or more bodies: its
// exact time from the step start, the states on both sides of it, and the
// islands that published it (docs/multibody-dynamics-design.md §3.4).
type traceEvent struct {
	at        *big.Rat
	pre, post State
	islands   []int // indices into StepReport.Islands
}

// sliceSweeps is one slice's broad phase and pair sweeps: every body's path,
// the candidate pairs in canonical order, and each candidate's sweep.
type sliceSweeps struct {
	paths      []decad.PairPath
	candidates []int
	swept      map[int]*decad.SweepReport
}

// stepScheduled is the step of a world of four or more bodies
// (docs/multibody-dynamics-design.md §5): one full-step kick, then a drift
// slice over [0, dt]. Pairs that touch at the step start form islands
// (§6.1) that are solved, certified (§6.2, §6.3) and corrected (§6.6) as one
// event at time zero; the slice is then swept again from the post-event
// state, each solved pair under its §5.2 continuation policy. Every other
// candidate must be proved clear or departed over the slice. An event inside
// the slice (an impact or transition bracket, a graze, a track that ends)
// stops the step as Undecided with StepUnsupported until the multi-event
// trace of §13 PR 6.
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
	kicked, ok := w.kickByLoads(from, input.Gravity, loads, dt)
	if !ok {
		return w.scheduleUndecided(scheduleDiagnostic(StepKickUnbounded, BodyPair{},
			"force kick or torque kick exceeds its velocity residual")), nil
	}
	if diagnostics, err := w.fixedPairRelations(ctx, kicked); err != nil || len(diagnostics) != 0 {
		if err != nil {
			return nil, err
		}
		return w.scheduleUndecided(diagnostics...), nil
	}
	scheduled := w.pairSchedule()
	sweeps, report, err := w.sweepSlice(ctx, kicked, drivers, dt, scheduled, nil)
	if report != nil || err != nil {
		return report, err
	}
	gathered, diagnostics := w.gatherInitialContacts(sweeps)
	if len(diagnostics) != 0 {
		return w.scheduleUndecided(diagnostics...), nil
	}
	if len(gathered) == 0 {
		end, proofs, report := w.finishSlice(kicked, sweeps, dt, scheduled)
		if report != nil {
			return report, nil
		}
		trace := Trace{start: from, end: end, duration: dt, slices: []traceSlice{{
			start: units.Seconds(0), end: dt, from: kicked, to: end, paths: sweeps.paths, proofs: proofs,
		}}}
		return w.publishScheduled(from, kicked, end, trace, nil, nil, [][2]State{{kicked, end}},
			input.Gravity, loads, dt), nil
	}
	event, report, err := w.initialIslands(ctx, kicked, dt, gathered, scheduled)
	if report != nil || err != nil {
		return report, err
	}
	sweeps, report, err = w.sweepSlice(ctx, event.post, drivers, dt, scheduled, event.policies)
	if report != nil || err != nil {
		return report, err
	}
	for _, key := range sweeps.candidates {
		sweep, pair := sweeps.swept[key], w.pairs[key]
		switch sweep.Outcome {
		case decad.SweepClear, decad.SweepDepartedClear:
		case decad.SweepPersistentTouch:
			if event.policies[key] != decad.ContinueCertifiedTouch || !w.fullTrackWithin(sweep) {
				diagnostics = append(diagnostics, scheduleDiagnostic(StepTrackUnproved, w.bodyPair(pair),
					"persistent track does not span the slice within the penetration residual"))
			}
		case decad.SweepUndecided:
			diagnostics = append(diagnostics, scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
				fmt.Sprintf("continuation sweep is undecided (%v)", sweep.Cause)))
		default:
			diagnostics = append(diagnostics, scheduleDiagnostic(StepUnsupported, w.bodyPair(pair),
				fmt.Sprintf("continuation sweep returned %v; an event inside a slice needs the multi-event trace",
					sweep.Outcome)))
		}
	}
	if len(diagnostics) != 0 {
		return w.scheduleUndecided(diagnostics...), nil
	}
	end, proofs, report := w.finishSlice(event.post, sweeps, dt, scheduled)
	if report != nil {
		return report, nil
	}
	diagnostics, err = w.completedContacts(ctx, end, event.policies)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) != 0 {
		return w.scheduleUndecided(diagnostics...), nil
	}
	islands := make([]int, len(event.islands))
	for i := range islands {
		islands[i] = i
	}
	trace := Trace{start: from, end: end, duration: dt,
		slices: []traceSlice{{start: units.Seconds(0), end: dt, from: event.post, to: end, paths: sweeps.paths,
			proofs: proofs}},
		events: []traceEvent{{at: new(big.Rat), pre: kicked, post: event.post, islands: islands}}}
	return w.publishScheduled(from, kicked, end, trace, event.events, event.islands,
		[][2]State{{event.post, end}}, input.Gravity, loads, dt), nil
}

// gatherInitialContacts classifies a slice's candidate sweeps at the step
// start: clear and departed pairs need nothing, initially touching or
// overlapping pairs are gathered with their manifolds (§6.1), and every
// other outcome is a diagnostic.
func (w *World) gatherInitialContacts(sweeps sliceSweeps) ([]islandPair, []StepDiagnostic) {
	var gathered []islandPair
	var diagnostics []StepDiagnostic
	for _, key := range sweeps.candidates {
		sweep, pair := sweeps.swept[key], w.pairs[key]
		switch sweep.Outcome {
		case decad.SweepClear, decad.SweepDepartedClear:
		case decad.SweepInitiallyTouching, decad.SweepInitiallyOverlapping:
			event := sweep.InitialEvent
			if event == nil {
				event = sweep.Event
			}
			if event == nil || !w.manifoldWithin(event.Manifold) {
				diagnostics = append(diagnostics, scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
					"initial contact has no manifold within the contact request"))
				continue
			}
			gathered = append(gathered, islandPair{key: key, a: pair.a, b: pair.b,
				manifold: cloneManifold(*event.Manifold), at: event.At})
		case decad.SweepUndecided:
			diagnostics = append(diagnostics, scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
				fmt.Sprintf("candidate pair sweep is undecided (%v)", sweep.Cause)))
		default:
			diagnostics = append(diagnostics, scheduleDiagnostic(StepUnsupported, w.bodyPair(pair),
				fmt.Sprintf("candidate pair sweep returned %v; an event inside a slice needs the multi-event trace",
					sweep.Outcome)))
		}
	}
	return gathered, diagnostics
}

// sweepSlice runs §4.3's broad phase over every body's slice path from state
// and sweeps each candidate pair under its start policy: the contact-set
// policy in policies, or StopAtInitialContact.
func (w *World) sweepSlice(ctx context.Context, state State, drivers []decad.PoseSegment, dt units.Value,
	scheduled map[int]struct{}, policies map[int]decad.SweepStartPolicy) (sliceSweeps, *StepReport, error) {
	paths := w.slicePaths(state, drivers, dt)
	boxes := make([]decad.SweptBox, len(w.bodies))
	for i, body := range w.bodies {
		if err := ctx.Err(); err != nil {
			return sliceSweeps{}, nil, err
		}
		box, err := w.doc.SweptBox(ctx, body.definition.Body, paths[i])
		if errors.Is(err, decad.ErrUnsupported) {
			return sliceSweeps{}, w.scheduleUndecided(scheduleDiagnostic(StepTravelUnbounded, BodyPair{},
				fmt.Sprintf("swept box of body %d is unbounded: %v", i, err))), nil
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
		sweep, err := w.doc.SweepPair(ctx, w.bodies[pair.a].definition.Body, w.bodies[pair.b].definition.Body,
			paths[pair.a], paths[pair.b], w.sweepRequest(dt, policy))
		if err != nil {
			return sliceSweeps{}, nil, err
		}
		out.swept[key] = sweep
	}
	return out, nil, nil
}

// finishSlice publishes every body's end pose on its slice path and requires
// each swept pair's certificate to replay exactly those poses at the slice
// end (§4.3). It returns the end state and every scheduled pair's proof.
func (w *World) finishSlice(state State, sweeps sliceSweeps, dt units.Value,
	scheduled map[int]struct{}) (State, []pairProof, *StepReport) {
	end := state.clone()
	for i := range end.entries {
		pose, err := pathPoseAt(sweeps.paths[i], big.NewRat(1, 1))
		if err != nil {
			return State{}, nil, w.scheduleUndecided(scheduleDiagnostic(StepUnsupported, BodyPair{},
				fmt.Sprintf("body %d has a non-finite end pose: %v", i, err)))
		}
		end.entries[i].Pose = pose
	}
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
		// The published end pose of every swept body is the one its pair
		// certificate replays; a refused or different pose stops the step.
		pair := w.pairs[key]
		poseA, poseB, err := sweep.CertifiedPosesAtInterval(dt, units.Seconds(0), dt)
		if err != nil || poseA != end.entries[pair.a].Pose || poseB != end.entries[pair.b].Pose {
			reason := "rounded end poses differ from the pair certificate"
			if err != nil {
				reason = fmt.Sprintf("rounded end poses lack the pair certificate: %v", err)
			}
			return State{}, nil, w.scheduleUndecided(scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair), reason))
		}
		proofs = append(proofs, pairProof{pair: key, sweep: sweep})
	}
	return end, proofs, nil
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
// the completed poses each must be separated, or touching with its
// penetration within PenetrationResidual.
func (w *World) completedContacts(ctx context.Context, end State,
	policies map[int]decad.SweepStartPolicy) ([]StepDiagnostic, error) {
	for key := range w.pairs {
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
		case contact.Relation == decad.ContactTouching && contact.Manifold != nil && w.penetrationWithin(contact.Manifold):
		default:
			return []StepDiagnostic{scheduleDiagnostic(StepTrackUnproved, w.bodyPair(pair),
				fmt.Sprintf("completed contact relation is %v beyond the penetration residual", contact.Relation))}, nil
		}
	}
	return nil, nil
}

// publishScheduled attaches the conservation readings of an advanced step.
// drift lists the recorded slices' end states; events are the island events.
func (w *World) publishScheduled(from, kicked, end State, trace Trace, events []ContactEvent,
	islands []IslandReport, drift [][2]State, gravity QuantityVec, loads []*BodyLoad, dt units.Value) *StepReport {
	input, okInput := w.conservationState(from)
	afterKick, okKick := w.conservationState(kicked)
	completion, okEnd := w.conservationState(end)
	gravityImpulse, loadImpulse, okForce := w.forceImpulses(gravity, loads, dt)
	torqueImpulse, okTorque := w.torqueImpulse(loads, dt)
	contactImpulse, okContact := w.islandContactImpulse(events)
	driftChange, okDrift := w.driftConservationSlices(drift)
	kinematicWork, okWork := boundedReading(new(big.Rat), new(big.Rat), new(big.Rat),
		units.KilogramSquareMillimeterPerSecondSquared)
	if !okInput || !okKick || !okEnd || !okForce || !okTorque || !okContact || !okDrift || !okWork {
		return w.scheduleUndecided(scheduleDiagnostic(StepConservationFailed, BodyPair{},
			"conservation readings cannot be represented with finite bounds"))
	}
	return &StepReport{Status: Advanced, Next: &end, Events: events, Islands: islands, Excluded: w.Excluded(),
		Trace: trace, Conservation: &StepConservation{Input: input, AfterKick: afterKick, Completion: completion,
			GravityImpulse: gravityImpulse, LoadImpulse: loadImpulse, ContactImpulse: contactImpulse,
			TorqueImpulse: torqueImpulse, KinematicWork: kinematicWork, DriftChange: driftChange}}
}

func scheduleDiagnostic(code StepReason, pair BodyPair, reason string) StepDiagnostic {
	return StepDiagnostic{Code: code, Pair: pair, Reason: reason}
}

func (w *World) scheduleUndecided(diagnostics ...StepDiagnostic) *StepReport {
	return &StepReport{Status: Undecided, Excluded: w.Excluded(), Diagnostics: diagnostics}
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
		if contact.Relation == decad.ContactOverlapping || contact.Relation == decad.ContactUndecided {
			diagnostics = append(diagnostics, scheduleDiagnostic(StepFixedPairRelation, w.bodyPair(pair),
				fmt.Sprintf("fixed pair relation is %v", contact.Relation)))
		}
	}
	return diagnostics, nil
}

// slicePaths builds each body's path over [0, dt] (§5 step 3): a Fixed body
// a constant PoseSegment, a Kinematic body its driver, a Dynamic body a
// RigidDriftSegment from its kicked pose, world mass center and velocities.
func (w *World) slicePaths(kicked State, drivers []decad.PoseSegment, dt units.Value) []decad.PairPath {
	paths := make([]decad.PairPath, len(w.bodies))
	for i, body := range w.bodies {
		entry := kicked.entries[i]
		switch body.definition.Role {
		case Fixed:
			paths[i] = decad.PoseSegment{From: entry.Pose, To: entry.Pose, Duration: dt}
		case Kinematic:
			paths[i] = drivers[i]
		default:
			paths[i] = decad.RigidDriftSegment{From: entry.Pose,
				Center:         entry.Pose.Apply(body.mass.Center.Value),
				LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
				Duration: dt}
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

// sampleSlices is §7.1 for a trace of slices: the start state at zero, the
// end state at the duration, and inside a slice every body's pose from the
// pair certificates that cover it, or from its own path when only swept-box
// exclusions cover it. Velocities inside a slice are the slice's from
// velocities.
func (tr Trace) sampleSlices(t units.Value, timeValue, durationValue *big.Rat) (State, error) {
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
		return slice.sample(t, timeValue, start, end)
	}
	return State{}, fmt.Errorf("%w: trace time has no certified slice", ErrUnsupported)
}

func (slice traceSlice) sample(t units.Value, timeValue, start, end *big.Rat) (State, error) {
	world := slice.from.world
	state := slice.from.clone()
	covered := make([]bool, len(state.entries))
	for _, proof := range slice.proofs {
		if proof.sweep == nil {
			continue
		}
		poseA, poseB, err := proof.sweep.CertifiedPosesAtInterval(t, slice.start, slice.end)
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
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(timeValue, start), new(big.Rat).Sub(end, start))
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
	return state, nil
}
