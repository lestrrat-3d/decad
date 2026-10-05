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

type BodyLoad struct {
	Body   *decad.Body
	Force  QuantityVec
	Torque QuantityVec
}

type KinematicDriver struct {
	Body *decad.Body
	Path decad.PairPath
}

type StepInput struct {
	Gravity QuantityVec
	Loads   []BodyLoad
	Drivers []KinematicDriver
}

type StepStatus int

const (
	Advanced StepStatus = iota + 1
	Undecided
)

// StepReason classifies why a step stopped (docs/multibody-dynamics-design.md
// §12). The step of a world of four or more bodies sets it; the two- and
// three-body steps leave it StepNoReason.
type StepReason int

const (
	StepNoReason           StepReason = iota
	StepPairUndecided                 // a candidate sweep is Undecided before the next event
	StepManifoldMissing               // an event pair has no manifold within StepConfig.Contact
	StepIslandDegenerate              // a closing constraint with no dynamic body, or K <= 0, or a non-finite proposal
	StepIslandResidual                // a solver gate exceeds its limit at MaxIterations
	StepCorrectionFailed              // a position correction exceeds its allowance or loses a relation
	StepTrackUnproved                 // a contact-set pair has neither a persistent nor a band track
	StepKickUnbounded                 // the force kick cannot be bounded within the velocity residuals
	StepConservationFailed            // an island or step conservation gate fails
	StepEventBudget                   // MaxEvents reached with time remaining
	StepPairBudget                    // MaxPairSweeps reached
	StepTravelUnbounded               // SweptBox returned ErrUnsupported for a body
	StepFixedPairRelation             // a non-excluded Fixed/Fixed pair is Overlapping or Undecided
	StepUnsupported                   // the current phase has no solver for this event family
)

// StepDiagnostic says why a step is Undecided. The step of a world of four
// or more bodies fills the typed fields (docs/multibody-dynamics-design.md
// §12); the two- and three-body steps set Pair and Reason only.
type StepDiagnostic struct {
	Code StepReason
	// Pair is the responsible pair, when one is.
	Pair BodyPair
	// Bodies lists the responsible island's bodies in world order, when an
	// island is responsible.
	Bodies []*decad.Body
	// From and To bound the time interval the diagnostic applies to, from the
	// start of the step.
	From, To units.Value
	// Limit is the configured limit a residual, allowance or budget exceeded,
	// when one did. A count limit such as MaxEvents is a dimensionless scalar.
	Limit units.Value
	// Reason is a human-readable message; callers branch on Code.
	Reason string
}

// ContactEventKind separates an impulse-bearing contact from a geometry-only transition.
type ContactEventKind int

const (
	ContactImpact ContactEventKind = iota + 1
	ContactTransition
	ContactGraze
)

type ContactEvent struct {
	Kind ContactEventKind
	Pair BodyPair
	// Bracket is local to the sweep starting at SliceStart for SliceDuration.
	Bracket                   decad.SweepInterval
	SliceStart, SliceDuration units.Value
	// Time is measured from the start of the complete step.
	Time                                       units.Value
	Manifold                                   decad.ContactManifold
	NormalImpulse                              units.Value
	TangentImpulse                             QuantityVec
	PointImpulses                              []ContactPointImpulse
	Solver                                     *ContactSolverReport
	PreVelocity                                QuantityVec
	PostVelocity                               QuantityVec
	PositionChange                             r3.Vec
	PreVelocityA, PreVelocityB                 QuantityVec
	PostVelocityA, PostVelocityB               QuantityVec
	PreAngularVelocityA, PreAngularVelocityB   QuantityVec
	PostAngularVelocityA, PostAngularVelocityB QuantityVec
	PoseA, PoseB                               r3.Transform
	PositionChangeA, PositionChangeB           r3.Vec
	// Island is the index into StepReport.Islands of the island that
	// published this event, in a world of four or more bodies; a graze or a
	// transition enters no solve there and carries -1.
	Island int
}

// ContactPointImpulse follows the matching point in ContactEvent.Manifold.
type ContactPointImpulse struct {
	Normal  units.Value
	Tangent QuantityVec
}

// ContactSolverReport records bounded residuals for a joint contact solve.
type ContactSolverReport struct {
	NormalResidual      units.Value
	TangentResidual     units.Value
	ConeResidual        units.Value
	PenetrationResidual units.Value
	AngularUpper        units.Value
	Iterations          int
	// The island solver of a world of four or more bodies also publishes
	// the largest attained value of each certificate gate
	// (docs/multibody-dynamics-design.md §6.3): the linear and angular law
	// residuals, the kinetic-energy change's upper end, and the island's
	// linear and angular momentum residuals about the world origin. Its
	// AngularUpper bounds the largest published post-solve angular speed.
	LinearResidual          units.Value
	AngularResidual         units.Value
	EnergyResidual          units.Value
	MomentumResidual        units.Value
	AngularMomentumResidual units.Value
}

func zeroImpulseVec() QuantityVec {
	zero := units.KilogramMillimetersPerSecond(0)
	return QuantityVec{X: zero, Y: zero, Z: zero}
}

type StepReport struct {
	Status       StepStatus
	Next         *State
	Events       []ContactEvent
	Excluded     []BodyPair
	Trace        Trace
	Diagnostics  []StepDiagnostic
	Conservation *StepConservation
	// Islands lists the simultaneous solves of a world of four or more
	// bodies, in solve order.
	Islands []IslandReport
}

func driftState(start State, seconds float64) (State, error) {
	out := start.clone()
	for i := range out.entries {
		entry := out.entries[i]
		pose := entry.Pose
		omega := r3.Vec{X: entry.AngularVelocity.X.Base(),
			Y: entry.AngularVelocity.Y.Base(), Z: entry.AngularVelocity.Z.Base()}
		if omega != (r3.Vec{}) {
			center := pose.Apply(start.world.bodies[i].mass.Center.Value)
			rate := math.Hypot(omega.X, math.Hypot(omega.Y, omega.Z))
			turn, err := r3.RotationAround(center, omega, units.Radians(rate*seconds))
			if err != nil {
				return State{}, err
			}
			pose, err = pose.Then(turn)
			if err != nil {
				return State{}, err
			}
		}
		v := out.entries[i].LinearVelocity
		delta := r3.Vec{X: v.X.Base() * seconds, Y: v.Y.Base() * seconds, Z: v.Z.Base() * seconds}
		pose, err := translatePose(pose, delta)
		if err != nil {
			return State{}, err
		}
		out.entries[i].Pose = pose
	}
	return out, nil
}

func translatePose(pose r3.Transform, delta r3.Vec) (r3.Transform, error) {
	translation, err := r3.Translation(delta)
	if err != nil {
		return r3.Transform{}, err
	}
	return pose.Then(translation)
}

func velocityComponent(v QuantityVec, axis int) units.Value {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

func setVelocityComponent(v *QuantityVec, axis int, value units.Value) {
	switch axis {
	case 0:
		v.X = value
	case 1:
		v.Y = value
	default:
		v.Z = value
	}
}

func axisNormal(normal r3.Vec) (int, float64, bool) {
	components := [3]float64{normal.X, normal.Y, normal.Z}
	for axis, component := range components {
		if math.Abs(component) != 1 {
			continue
		}
		othersZero := true
		for other, value := range components {
			othersZero = othersZero && (other == axis || value == 0)
		}
		return axis, component, othersZero
	}
	return 0, 0, false
}

func correctPair(start State, normal r3.Vec, depth float64, inverseMass [2]float64) (State, error) {
	out := start.clone()
	total := inverseMass[0] + inverseMass[1]
	if !finite(depth, total) || total <= 0 {
		return State{}, fmt.Errorf("invalid pair correction")
	}
	for i, inverse := range inverseMass {
		if inverse == 0 {
			continue
		}
		signed := depth * inverse / total
		if i == 0 {
			signed = -signed
		}
		pose, err := translatePose(out.entries[i].Pose, normal.Scale(signed))
		if err != nil {
			return State{}, err
		}
		out.entries[i].Pose = pose
	}
	return out, nil
}

func pairCorrectionWithin(before, after State, axis int, allowance float64) bool {
	if !finite(allowance) || allowance < 0 {
		return false
	}
	actual := 0.0
	for i := range before.entries {
		delta := after.entries[i].Pose.Translation().Sub(before.entries[i].Pose.Translation())
		components := [3]float64{delta.X, delta.Y, delta.Z}
		for other, value := range components {
			if other != axis && value != 0 {
				return false
			}
		}
		if components[axis] != 0 {
			actual = outwardSum(actual, math.Nextafter(math.Abs(components[axis]), math.Inf(1)))
		}
	}
	return finite(actual) && actual <= allowance
}

// Step advances the admitted pair through certified clear, contact, or edge-transition paths.
// A world of four or more bodies drifts every body from event to event on
// paths whose candidate pairs the broad phase selects and SweepPair certifies;
// at each event time it solves the touching and impacting pairs as certified
// Coulomb islands and continues (docs/multibody-dynamics-design.md §5,
// §6). Its Undecided report carries typed diagnostics and the certified prefix
// in its Trace.
func (w *World) Step(ctx context.Context, from State, input StepInput, dt units.Value) (*StepReport, error) {
	if w == nil || ctx == nil || from.world != w || !validQuantity(dt, units.Time, true) {
		return nil, fmt.Errorf("%w: invalid context, world, state, or duration", ErrInvalidInput)
	}
	if w.three != nil {
		report, err := w.stepThreeBodies(ctx, from, input, dt)
		setDefaultEventSlices(report, dt)
		return report, err
	}
	if len(w.bodies) != 2 {
		return w.stepScheduled(ctx, from, input, dt)
	}
	if err := validateQuantityVec(input.Gravity, units.Acceleration); err != nil {
		return nil, err
	}
	loads, err := w.validateLoads(input.Loads)
	if err != nil {
		return nil, err
	}
	driver, err := w.validateDriver(from, input.Drivers, dt)
	if err != nil {
		return nil, err
	}
	live := w.doc.Bodies()
	for _, part := range w.bodies {
		if !containsBody(live, part.definition.Body) {
			return nil, fmt.Errorf("%w: world body was retired", ErrInvalidInput)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	kicked, ok := w.kickByLoads(from, input.Gravity, loads[:], dt)
	if !ok {
		return undecided(w, "force kick or torque kick exceeds its velocity residual"), nil
	}
	report, err := w.stepKicked(ctx, from, kicked, dt, driver)
	if report != nil {
		report.Excluded = w.Excluded()
		setDefaultEventSlices(report, dt)
	}
	if err != nil || report == nil || report.Status != Advanced {
		return report, err
	}
	if report.Next == nil {
		return undecided(w, "advanced step has no next state"), nil
	}
	for _, event := range report.Events {
		if reason := w.eventConservationFailure(event); reason != "" {
			return undecided(w, reason), nil
		}
	}
	conservation, ok := w.conservationReadings(from, kicked, *report.Next, report.Trace, report.Events,
		input.Gravity, loads[:], dt)
	if !ok {
		return undecided(w, "conservation readings cannot be represented with finite bounds"), nil
	}
	report.Conservation = &conservation
	return report, nil
}

func setDefaultEventSlices(report *StepReport, dt units.Value) {
	if report == nil {
		return
	}
	for i := range report.Events {
		if report.Events[i].SliceDuration.Kind() != units.Time {
			report.Events[i].SliceStart = units.Seconds(0)
			report.Events[i].SliceDuration = dt
		}
	}
}

func (w *World) stepKicked(ctx context.Context, from, kicked State, dt units.Value,
	driver kinematicMotion) (*StepReport, error) {
	if w.pairs[0].excluded {
		return w.stepExcluded(ctx, from, kicked, dt, driver)
	}
	rotating := false
	for _, entry := range kicked.entries {
		rotating = rotating || !zeroAngularVelocity(entry.AngularVelocity)
	}
	if rotating && driver.index >= 0 {
		return undecided(w, "rotating contact with a kinematic driver is not certified"), nil
	}
	if driver.index >= 0 {
		return w.stepKinematicPush(ctx, from, kicked, dt, driver)
	}
	moving := false
	for _, entry := range kicked.entries {
		components := [3]units.Value{entry.LinearVelocity.X, entry.LinearVelocity.Y, entry.LinearVelocity.Z}
		for _, component := range components {
			if component.Mag() != 0 {
				moving = true
			}
		}
	}
	if !moving && !rotating {
		return w.stepStill(ctx, from, kicked, dt)
	}
	oblique := false
	if !rotating {
		for _, entry := range kicked.entries {
			if entry.Pose.ApplyDir(r3.Vec{X: 1}) != (r3.Vec{X: 1}) ||
				entry.Pose.ApplyDir(r3.Vec{Y: 1}) != (r3.Vec{Y: 1}) ||
				entry.Pose.ApplyDir(r3.Vec{Z: 1}) != (r3.Vec{Z: 1}) {
				oblique = true
				break
			}
		}
	}

	first, err := w.sweep(ctx, kicked, dt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if oblique && first.Outcome != decad.SweepGrazingTouch {
		return w.stepObliqueSupport(ctx, from, kicked, dt)
	}
	report := &StepReport{Trace: Trace{start: from, duration: dt}}
	switch first.Outcome {
	case decad.SweepClear:
		if rotating {
			return w.stepRotatingClear(from, kicked, dt, first)
		}
		end, err := driftState(kicked, dt.Base())
		if err != nil {
			return undecidedArithmetic(w, "non-finite clear-path pose", err)
		}
		actual, err := w.sweepPoses(ctx, kicked, end, dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if actual.Outcome != decad.SweepClear {
			return undecided(w, fmt.Sprintf("numerical clear path returned %v", actual.Outcome)), nil
		}
		report.Status, report.Next, report.Trace.end = Advanced, &end, end
		report.Trace.preSweep = actual
		return report, nil
	case decad.SweepGrazingTouch:
		if rotating {
			return undecided(w, "rotating grazing contact is not certified"), nil
		}
		return w.stepGrazingTouch(ctx, from, kicked, dt, first)
	case decad.SweepInitiallyTouching:
		if w.pairs[0].friction.lower.Sign() != 0 && w.sphereFloorFrictionCandidate(first.Event) {
			return w.stepInitialSphereFloorFriction(ctx, from, kicked, dt, first)
		}
		if first.Event != nil && w.pairs[0].friction.lower.Sign() > 0 &&
			isObliqueSpherePairEvent(first.Event.Manifold) &&
			spherePairVelocity(kicked.entries[0].LinearVelocity) !=
				spherePairVelocity(kicked.entries[1].LinearVelocity) {
			return w.stepInitialOffAxisSpherePairFriction(ctx, from, kicked, dt, first)
		}
		if first.Event != nil && w.pairs[0].restitution.Base() > 0 &&
			(w.pairs[0].friction.lower.Sign() > 0 && isSourceSpherePairEvent(first.Event.Manifold) &&
				math.Abs(first.Event.Manifold.Points[0].Normal.Value.X) == 1 ||
				w.pairs[0].friction.upper.Sign() == 0 &&
					w.exactSpherePairFrictionCandidate(kicked, first.Event.Manifold)) {
			return w.stepInitialSpherePairFriction(ctx, from, kicked, dt, first)
		}
		if rotating {
			return undecided(w, "rotating initial contact needs a certified response track"), nil
		}
		if first.Event != nil && (isObliqueSpherePairEvent(first.Event.Manifold) ||
			(w.pairs[0].restitution.Base() > 0 && w.bodies[0].definition.Role == Dynamic &&
				w.bodies[1].definition.Role == Dynamic && w.pairs[0].friction.upper.Sign() == 0 &&
				isSourceSpherePairEvent(first.Event.Manifold))) {
			if spherePairVelocity(kicked.entries[0].LinearVelocity) ==
				spherePairVelocity(kicked.entries[1].LinearVelocity) {
				continuation, continuationErr := w.sweep(ctx, kicked, dt, decad.ContinueCertifiedTouch)
				if continuationErr != nil {
					return nil, continuationErr
				}
				return w.stepNoImpulse(ctx, from, kicked, dt, continuation)
			}
			return w.stepObliqueSpherePair(ctx, from, kicked, kicked, dt,
				units.Seconds(0), 0, first, nil)
		}
		if w.fixedOffcenterPatch(first.Event) {
			return w.stepFixedOffcenter(ctx, from, kicked, dt, first)
		}
		if len(w.bodies) == 2 && w.bodies[0].definition.Role == Dynamic &&
			w.bodies[1].definition.Role == Dynamic && w.pairs[0].friction.upper.Sign() == 0 &&
			offcenterPairPatch(first.Event, w.bodies[0].mass, w.bodies[1].mass) {
			return w.stepInitialTwoDynamicFriction(ctx, from, kicked, dt, first)
		}
		if w.pairs[0].friction.lower.Sign() != 0 {
			return w.stepInitialFriction(ctx, from, kicked, dt, first)
		}
		return w.stepInitialTouch(ctx, from, kicked, dt, first)
	case decad.SweepImpactBracket:
		if rotating {
			for _, entry := range kicked.entries {
				if entry.AngularVelocity.X.Base() != 0 || entry.AngularVelocity.Y.Base() != 0 {
					return undecided(w, "rotating impact requires spin about the face normal"), nil
				}
			}
		}
		// Continue below, consuming the geometry producer's event and manifold.
	default:
		return undecided(w, fmt.Sprintf("first sweep returned %v", first.Outcome)), nil
	}
	if first.Bracket == nil || first.Event == nil || first.Event.Manifold == nil ||
		len(first.Event.Manifold.Points) == 0 {
		return undecided(w, "impact has no certified bracket and manifold"), nil
	}
	impactTime := dt.Base() * first.Bracket.To.Fraction.Base()
	if !finite(impactTime) || impactTime < 0 || impactTime > dt.Base() {
		return undecided(w, "impact time is outside the step"), nil
	}
	eventAt := units.Seconds(impactTime)
	fraction, duration := exactBase(first.Bracket.To.Fraction), exactBase(dt)
	if fraction == nil || duration == nil || fraction.Sign() <= 0 ||
		fraction.Cmp(big.NewRat(1, 1)) > 0 {
		return undecided(w, "impact fraction is outside the step"), nil
	}
	if fraction.Cmp(big.NewRat(1, 1)) == 0 {
		// The public fraction can round to one before the proved right endpoint.
		// Confirm that the original sweep covers the exact step end first.
		if !first.BracketEndsAtDuration() {
			return undecided(w, "impact bracket does not reach the exact step end"), nil
		}
		eventAt = dt
	} else if exactBase(eventAt).Cmp(duration) >= 0 {
		return undecided(w, "impact time exceeds the exact step duration"), nil
	}
	pre, err := driftState(kicked, impactTime)
	if err != nil {
		return undecidedArithmetic(w, "non-finite impact pose", err)
	}
	var roundedPrefix *decad.SweepReport
	if rotating && !rotatingImpactPoseMatches(first, pre) {
		return undecided(w, "rotating impact pose differs from certified bracket sample"), nil
	}
	if !rotating {
		roundedPrefix, err = w.sweepPoses(ctx, kicked, pre, units.Seconds(impactTime),
			decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if first.HasAffineReplayProof() &&
			!roundedImpactPrefixAtEnd(roundedPrefix, first, w.step.PenetrationResidual) {
			return undecided(w, "published impact prefix lacks a matching rounded endpoint bracket"), nil
		}
	}
	if w.pairs[0].friction.lower.Sign() != 0 {
		if isSourceSpherePairEvent(first.Event.Manifold) &&
			w.bodies[0].definition.Role == Dynamic && w.bodies[1].definition.Role == Dynamic {
			return w.stepInteriorSpherePairFriction(ctx, from, kicked, dt, first)
		}
		if w.sphereFloorFrictionCandidate(first.Event) {
			return w.stepInteriorSphereFloorFriction(ctx, from, kicked, pre, dt,
				eventAt, impactTime, first, roundedPrefix)
		}
		return w.stepInteriorFriction(ctx, from, kicked, pre, dt, eventAt, impactTime,
			first, roundedPrefix)
	}
	if isObliqueSpherePairEvent(first.Event.Manifold) {
		return w.stepObliqueSpherePair(ctx, from, kicked, pre, dt, eventAt,
			impactTime, first, roundedPrefix)
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	if !ok {
		return undecided(w, "contact normal or point is outside the admitted resolution"), nil
	}
	axis, normalSign, valid := axisNormal(normal)
	if !valid {
		return undecided(w, "impact normal is not a supported axis"), nil
	}
	if rotating && axis != 2 {
		return undecided(w, "rotating impact requires a horizontal face"), nil
	}
	preSpeed := [2]units.Value{
		velocityComponent(kicked.entries[0].LinearVelocity, axis),
		velocityComponent(kicked.entries[1].LinearVelocity, axis),
	}
	relativeSpeed := (preSpeed[1].Base() - preSpeed[0].Base()) * normalSign
	if relativeSpeed >= -w.step.VelocityResidual.Base() {
		return undecided(w, "impact is not closing"), nil
	}
	var inverseMass [2]float64
	for i := range w.bodies {
		if w.bodies[i].definition.Role == Dynamic {
			inverseMass[i] = 1 / w.bodies[i].mass.Mass.Value.Base()
		}
	}
	denominator := inverseMass[0] + inverseMass[1]
	if !finite(denominator) || denominator <= 0 {
		return undecided(w, "effective mass is not finite and positive"), nil
	}
	coefficient := w.pairs[0].restitution
	idealRelative := new(big.Rat).Sub(exactBase(preSpeed[1]), exactBase(preSpeed[0]))
	idealThreshold := exactBase(w.step.ImpactSpeed)
	if idealRelative == nil || idealThreshold == nil {
		return undecided(w, "impact speed is not representable"), nil
	}
	idealRelative.Mul(idealRelative, new(big.Rat).SetInt64(int64(normalSign)))
	target := 0.0
	effectiveCoefficient := units.Scalar(0)
	if new(big.Rat).Neg(idealRelative).Cmp(idealThreshold) > 0 {
		effectiveCoefficient = coefficient
		target = -coefficient.Base() * relativeSpeed
	}
	impulse := (target - relativeSpeed) / denominator
	if !finite(impulse) || impulse <= 0 {
		return undecided(w, "impulse is not finite and positive"), nil
	}
	postSpeed := [2]float64{
		preSpeed[0].Base() - impulse*normalSign*inverseMass[0],
		preSpeed[1].Base() + impulse*normalSign*inverseMass[1],
	}
	// At zero restitution against a fixed body, the dynamic body's exact
	// normal velocity equals the fixed body's velocity for every admitted
	// mass. Publish that value directly so float cancellation cannot turn a
	// persistent face contact into an artificial separating sweep.
	if effectiveCoefficient.Base() == 0 {
		switch {
		case w.bodies[0].definition.Role == Fixed && w.bodies[1].definition.Role == Dynamic:
			postSpeed[1] = preSpeed[0].Base()
		case w.bodies[1].definition.Role == Fixed && w.bodies[0].definition.Role == Dynamic:
			postSpeed[0] = preSpeed[1].Base()
		}
	}
	if !responsePairResidualsWithin(preSpeed, normalSign, effectiveCoefficient,
		w.bodies, target, impulse, postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) {
		return undecided(w, "exact response bounds exceed velocity or impulse residual"), nil
	}
	for i, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		if !omittedSpinWithin(first.Event.Manifold, pre.entries[i].Pose, part.mass,
			i, axis, impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
			return undecided(w, "off-center impulse exceeds angular velocity residual"), nil
		}
		if !w.omittedCylinderMotionWithin(first.Event.Manifold, pre.entries[i].Pose,
			i, axis, impulse, dt) {
			return undecided(w, "omitted cylinder point motion exceeds residual"), nil
		}
	}
	bracketTravel, valid := boundBracketTravel(*first.Bracket, preSpeed[1].Base()-preSpeed[0].Base())
	if !valid || !finite(separation, bound) || separation > bound {
		return undecided(w, "impact penetration or bracket travel is not certified"), nil
	}
	correctionAllowance := outwardSum(bracketTravel, w.step.ContactSlop.Base(), bound)
	if !finite(correctionAllowance) || -separation > correctionAllowance {
		return undecided(w, "impact penetration exceeds its certified bracket"), nil
	}
	post, err := correctPair(pre, normal, -separation, inverseMass)
	if err != nil {
		return undecidedArithmetic(w, "position correction is not finite", err)
	}
	if !pairCorrectionWithin(pre, post, axis, correctionAllowance) {
		return undecided(w, "actual position correction exceeds its certified allowance"), nil
	}
	for i := range post.entries {
		setVelocityComponent(&post.entries[i].LinearVelocity, axis, units.MillimetersPerSecond(postSpeed[i]))
	}
	contact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	for iteration := 1; iteration < w.step.MaxIterations && contact.Relation == decad.ContactOverlapping; iteration++ {
		if contact.Manifold == nil {
			break
		}
		nextNormal, nextSeparation, nextBound, valid := reducedContact(contact.Manifold, w.step.Contact)
		if !valid || nextNormal != normal || nextSeparation >= 0 ||
			!finite(nextSeparation, nextBound) {
			break
		}
		increment := -nextSeparation
		if !finite(increment) {
			break
		}
		candidate, err := correctPair(post, normal, increment, inverseMass)
		if err != nil {
			return undecidedArithmetic(w, "position correction is not finite", err)
		}
		correctionAllowance = outwardSum(correctionAllowance, nextBound)
		if !finite(correctionAllowance) || !pairCorrectionWithin(pre, candidate, axis, correctionAllowance) {
			break
		}
		post = candidate
		contact, err = w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
			post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
	}
	remaining := dt.Base() - impactTime
	if rotating && remaining > 0 && contact.Relation == decad.ContactSeparated &&
		contact.Gap != nil && contact.Gap.Bound.Base() == 0 &&
		contact.Gap.Value.Base() > 0 &&
		contact.Gap.Value.Base() <= correctionAllowance {
		candidate, err := correctPair(post, normal, -contact.Gap.Value.Base(), inverseMass)
		if err != nil {
			return undecidedArithmetic(w, "rotating contact correction is not finite", err)
		}
		if pairCorrectionWithin(pre, candidate, axis, correctionAllowance) {
			corrected, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
				candidate.entries[0].Pose, candidate.entries[1].Pose, w.step.Contact)
			if err != nil {
				return nil, err
			}
			if corrected.Relation == decad.ContactTouching {
				post, contact = candidate, corrected
			}
		}
	}
	postRelative := (postSpeed[1] - postSpeed[0]) * normalSign
	if !finite(postRelative) || postRelative < -w.step.VelocityResidual.Base() {
		return undecided(w, "impact response remains closing"), nil
	}
	policy := decad.ContinueSeparatingTouch
	persistent := postRelative <= w.step.VelocityResidual.Base()
	if contact.Relation != decad.ContactTouching &&
		!(remaining == 0 && !persistent && correctedEndpointGapWithin(contact, correctionAllowance)) {
		return undecided(w, fmt.Sprintf("corrected impact pose has relation %v (separation %.17g)",
			contact.Relation, separation)), nil
	}
	if persistent {
		policy = decad.ContinueCertifiedTouch
	}
	var continuation *decad.SweepReport
	if remaining > 0 {
		continuation, err = w.sweep(ctx, post, units.Seconds(remaining), policy)
		if err != nil {
			return nil, err
		}
		if (persistent && !w.persistentTrackWithin(continuation, normal)) ||
			(!persistent && continuation.Outcome != decad.SweepDepartedClear) {
			return undecided(w, fmt.Sprintf("rebound departure returned %v", continuation.Outcome)), nil
		}
	}
	end, err := driftState(post, remaining)
	if err != nil {
		return undecidedArithmetic(w, "non-finite final pose", err)
	}
	var roundedContinuation *decad.SweepReport
	if remaining > 0 && !rotating {
		actual, err := w.sweepPoses(ctx, post, end, units.Seconds(remaining), policy)
		if err != nil {
			return nil, err
		}
		roundedContinuation = actual
		if (persistent && !w.persistentTrackWithin(actual, normal)) ||
			(!persistent && actual.Outcome != decad.SweepDepartedClear) {
			return undecided(w, fmt.Sprintf("numerical rebound path returned %v", actual.Outcome)), nil
		}
	}
	if rotating && remaining > 0 && !rotatingEndpointMatches(continuation, end) {
		return undecided(w, "rotating rebound endpoint differs from certified sweep"), nil
	}
	finalContact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if finalContact.Relation != decad.ContactSeparated && finalContact.Relation != decad.ContactTouching {
		return undecided(w, "final pair relation is not proved clear"), nil
	}
	if persistent {
		if finalContact.Relation != decad.ContactTouching || finalContact.Manifold == nil {
			return undecided(w, "persistent contact lacks endpoint manifold"), nil
		}
		finalNormal, finalSeparation, finalBound, valid := reducedContact(finalContact.Manifold, w.step.Contact)
		if !valid || finalNormal != normal ||
			math.Abs(finalSeparation)+finalBound > w.step.PenetrationResidual.Base() {
			return undecided(w, "persistent endpoint exceeds penetration residual"), nil
		}
	}
	impulseValue := units.KilogramMillimetersPerSecond(impulse)
	changeA := post.entries[0].Pose.Translation().Sub(pre.entries[0].Pose.Translation())
	changeB := post.entries[1].Pose.Translation().Sub(pre.entries[1].Pose.Translation())
	reportBody := 0
	if w.bodies[0].definition.Role == Fixed {
		reportBody = 1
	}
	report.Status = Advanced
	report.Next = &end
	report.Events = []ContactEvent{{
		Kind:            ContactImpact,
		Pair:            BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
		Bracket:         *first.Bracket,
		Time:            eventAt,
		Manifold:        cloneManifold(*first.Event.Manifold),
		NormalImpulse:   impulseValue,
		TangentImpulse:  zeroImpulseVec(),
		PreVelocity:     kicked.entries[reportBody].LinearVelocity,
		PostVelocity:    post.entries[reportBody].LinearVelocity,
		PositionChange:  []r3.Vec{changeA, changeB}[reportBody],
		PreVelocityA:    kicked.entries[0].LinearVelocity,
		PreVelocityB:    kicked.entries[1].LinearVelocity,
		PostVelocityA:   post.entries[0].LinearVelocity,
		PostVelocityB:   post.entries[1].LinearVelocity,
		PositionChangeA: changeA,
		PositionChangeB: changeB,
	}}
	if rotating {
		event := &report.Events[0]
		event.PreAngularVelocityA, event.PreAngularVelocityB =
			pre.entries[0].AngularVelocity, pre.entries[1].AngularVelocity
		event.PostAngularVelocityA, event.PostAngularVelocityB =
			post.entries[0].AngularVelocity, post.entries[1].AngularVelocity
		event.PoseA, event.PoseB = pre.entries[0].Pose, pre.entries[1].Pose
		event.PointImpulses = make([]ContactPointImpulse, len(event.Manifold.Points))
		for i := range event.PointImpulses {
			event.PointImpulses[i] = ContactPointImpulse{
				Normal:  units.KilogramMillimetersPerSecond(impulse / float64(len(event.PointImpulses))),
				Tangent: zeroImpulseVec(),
			}
		}
	}
	report.Trace = Trace{start: from, pre: pre, post: post, end: end, duration: dt,
		eventAt: eventAt, hasEvent: true,
		preSweep: roundedPrefix, postSweep: roundedContinuation}
	if rotating {
		report.Trace.rotationalPrefix = first
		report.Trace.rotationalRemainder = continuation
	}
	return report, nil
}

// At the final instant, a rounded correction can put the pair just clear of
// the ideal impact. The contact kernel must prove that the complete pair gap
// fits the same bound that admitted the correction.
func correctedEndpointGapWithin(contact *decad.ContactReport, allowance float64) bool {
	if contact.Relation != decad.ContactSeparated || contact.Gap == nil ||
		!finite(allowance) || allowance < 0 {
		return false
	}
	value, bound := exactBase(contact.Gap.Value), exactBase(contact.Gap.Bound)
	if value == nil || bound == nil || bound.Sign() < 0 ||
		new(big.Rat).Sub(value, bound).Sign() <= 0 {
		return false
	}
	return new(big.Rat).Add(value, bound).Cmp(new(big.Rat).SetFloat64(allowance)) <= 0
}

// stepRotatingClear publishes a rotating endpoint only when SweepPair proves
// the full ideal drift and its rounded samples separated.
func (w *World) stepRotatingClear(from, kicked State, dt units.Value,
	sweep *decad.SweepReport) (*StepReport, error) {
	if sweep.Outcome != decad.SweepClear {
		return undecided(w, fmt.Sprintf("rotating sweep returned %v", sweep.Outcome)), nil
	}
	end, err := driftState(kicked, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite rotating drift", err)
	}
	found := false
	for _, sample := range sweep.Samples {
		if sample.At.Fraction.Base() != 1 {
			continue
		}
		if found || sample.Ideal.Relation != decad.ContactSeparated ||
			sample.FloatContact == nil || sample.FloatContact.Relation != decad.ContactSeparated ||
			sample.PoseA != end.entries[0].Pose || sample.PoseB != end.entries[1].Pose {
			return undecided(w, "rotating endpoint differs from certified sweep sample"), nil
		}
		found = true
	}
	if !found {
		return undecided(w, "rotating sweep lacks a certified endpoint"), nil
	}
	return &StepReport{Status: Advanced, Next: &end,
		Trace: Trace{start: from, end: end, duration: dt, rotationalRemainder: sweep}}, nil
}

// The original ideal right sample and the published rounded pose may straddle
// exact touch. Admit that relation change only for the same source features
// when their complete separation intervals fit the configured residual.
func roundedImpactPrefixAtEnd(sweep, original *decad.SweepReport, residual units.Value) bool {
	if sweep == nil || original == nil || sweep.Outcome != decad.SweepImpactBracket ||
		sweep.Bracket == nil || sweep.Event == nil || original.Event == nil ||
		sweep.Event.Manifold == nil || original.Event.Manifold == nil ||
		len(sweep.Event.Manifold.Points) != len(original.Event.Manifold.Points) ||
		exactBase(sweep.Bracket.To.Fraction).Cmp(exactBase(units.Scalar(1))) != 0 {
		return false
	}
	for _, relation := range []decad.ContactRelation{sweep.Event.Relation, original.Event.Relation} {
		if relation != decad.ContactTouching && relation != decad.ContactOverlapping {
			return false
		}
	}
	limit := exactBase(residual)
	for i, point := range sweep.Event.Manifold.Points {
		originalPoint := original.Event.Manifold.Points[i]
		if point.FaceA != originalPoint.FaceA || point.FaceB != originalPoint.FaceB ||
			point.FeatureA != originalPoint.FeatureA || point.FeatureB != originalPoint.FeatureB ||
			!roundedImpactNormalsMatch(point, originalPoint, original.Request.ContactRequest) {
			return false
		}
		difference := absRat(new(big.Rat).Sub(exactBase(point.Separation.Value),
			exactBase(originalPoint.Separation.Value)))
		difference.Add(difference, exactBase(point.Separation.Bound))
		difference.Add(difference, exactBase(originalPoint.Separation.Bound))
		if difference.Cmp(limit) > 0 {
			return false
		}
	}
	return true
}

func roundedImpactNormalsMatch(a, b decad.ContactPoint, request decad.ContactRequest) bool {
	if a.Normal.Value == b.Normal.Value {
		return true
	}
	if a.Normal.Bound.Base() == 0 && b.Normal.Bound.Base() == 0 {
		return false
	}
	limit := exactBase(request.NormalResolution)
	boundA, boundB := exactBase(a.Normal.Bound), exactBase(b.Normal.Bound)
	angleA, angleB := exactBase(a.NormalAngle), exactBase(b.NormalAngle)
	if limit == nil || boundA == nil || boundB == nil || angleA == nil || angleB == nil ||
		boundA.Sign() < 0 || boundB.Sign() < 0 || angleA.Cmp(limit) > 0 || angleB.Cmp(limit) > 0 {
		return false
	}
	budget := new(big.Rat).Sub(limit, boundA)
	budget.Sub(budget, boundB)
	if budget.Sign() < 0 {
		return false
	}
	squared := new(big.Rat)
	for _, component := range [][2]float64{{a.Normal.Value.X, b.Normal.Value.X},
		{a.Normal.Value.Y, b.Normal.Value.Y}, {a.Normal.Value.Z, b.Normal.Value.Z}} {
		if !finite(component[0], component[1]) {
			return false
		}
		difference := new(big.Rat).Sub(ratFloat(component[0]), ratFloat(component[1]))
		squared.Add(squared, new(big.Rat).Mul(difference, difference))
	}
	return squared.Cmp(new(big.Rat).Mul(budget, budget)) <= 0
}

func rotatingEndpointMatches(sweep *decad.SweepReport, end State) bool {
	if sweep == nil || sweep.Outcome != decad.SweepDepartedClear {
		return false
	}
	found := false
	for _, sample := range sweep.Samples {
		if sample.At.Fraction.Base() != 1 {
			continue
		}
		if found || sample.Ideal.Relation != decad.ContactSeparated ||
			sample.FloatContact == nil || sample.FloatContact.Relation != decad.ContactSeparated ||
			sample.PoseA != end.entries[0].Pose || sample.PoseB != end.entries[1].Pose {
			return false
		}
		found = true
	}
	return found
}

func rotatingImpactPoseMatches(sweep *decad.SweepReport, pre State) bool {
	if sweep == nil || sweep.Bracket == nil {
		return false
	}
	found := false
	for _, sample := range sweep.Samples {
		if sample.At.Fraction.Base() != sweep.Bracket.To.Fraction.Base() {
			continue
		}
		if found || sample.PoseA != pre.entries[0].Pose || sample.PoseB != pre.entries[1].Pose {
			return false
		}
		found = true
	}
	return found
}

func (w *World) stepExcluded(ctx context.Context, from, kicked State, dt units.Value,
	driver kinematicMotion) (*StepReport, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	end, err := driftState(kicked, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite excluded-pair drift", err)
	}
	if driver.index >= 0 {
		end.entries[driver.index].Pose = driver.path.To
	}
	return &StepReport{Status: Advanced, Next: &end,
		Trace: Trace{start: from, end: end, duration: dt, excluded: true}}, nil
}

// Compare the published float response against the exact law applied to the
// held input values. Mass.Bound is the certified uncertainty of that law.
func responseResidualsWithin(velocity units.Value, relativeSign float64, restitution units.Value,
	mass decad.Measurement, target, impulse float64, velocityLimit, impulseLimit units.Value) bool {
	if !finite(relativeSign, target, impulse) || math.Abs(relativeSign) != 1 {
		return false
	}
	v, e, m, massBound := exactBase(velocity), exactBase(restitution),
		exactBase(mass.Value), exactBase(mass.Bound)
	vLimit, jLimit := exactBase(velocityLimit), exactBase(impulseLimit)
	if v == nil || e == nil || m == nil || massBound == nil || vLimit == nil || jLimit == nil ||
		massBound.Sign() < 0 || vLimit.Sign() < 0 || jLimit.Sign() < 0 {
		return false
	}
	u := new(big.Rat).Mul(v, new(big.Rat).SetInt64(int64(relativeSign)))
	idealTarget := new(big.Rat).Neg(new(big.Rat).Mul(e, u))
	floatTarget := new(big.Rat).SetFloat64(target)
	floatImpulse := new(big.Rat).SetFloat64(impulse)
	if floatTarget == nil || floatImpulse == nil ||
		absRat(new(big.Rat).Sub(idealTarget, floatTarget)).Cmp(vLimit) > 0 {
		return false
	}
	factor := new(big.Rat).Sub(idealTarget, u)
	idealImpulse := new(big.Rat).Mul(factor, m)
	uncertainty := new(big.Rat).Mul(absRat(new(big.Rat).Set(factor)), massBound)
	uncertainty.Add(uncertainty, absRat(new(big.Rat).Sub(idealImpulse, floatImpulse)))
	return uncertainty.Cmp(jLimit) <= 0
}

func responsePairResidualsWithin(speed [2]units.Value, normalSign float64, restitution units.Value,
	parts []worldBody, target, impulse float64, post [2]float64,
	velocityLimit, impulseLimit units.Value) bool {
	if !finite(normalSign, target, impulse, post[0], post[1]) || math.Abs(normalSign) != 1 {
		return false
	}
	vA, vB := exactBase(speed[0]), exactBase(speed[1])
	e, vLimit, jLimit := exactBase(restitution), exactBase(velocityLimit), exactBase(impulseLimit)
	if vA == nil || vB == nil || e == nil || vLimit == nil || jLimit == nil {
		return false
	}
	u := new(big.Rat).Sub(vB, vA)
	u.Mul(u, new(big.Rat).SetInt64(int64(normalSign)))
	idealTarget := new(big.Rat).Neg(new(big.Rat).Mul(e, u))
	floatTarget := new(big.Rat).SetFloat64(target)
	floatImpulse := new(big.Rat).SetFloat64(impulse)
	if floatTarget == nil || floatImpulse == nil ||
		absRat(new(big.Rat).Sub(floatTarget, idealTarget)).Cmp(vLimit) > 0 {
		return false
	}
	factor := new(big.Rat).Sub(idealTarget, u)
	if factor.Sign() <= 0 {
		return false
	}
	var low, high [2]*big.Rat
	for i, part := range parts {
		if part.definition.Role != Dynamic {
			continue
		}
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil || bound.Sign() < 0 {
			return false
		}
		low[i] = new(big.Rat).Sub(mass, bound)
		high[i] = new(big.Rat).Add(mass, bound)
		if low[i].Sign() <= 0 {
			return false
		}
	}
	var jLow, jHigh, aShareLow, aShareHigh, bShareLow, bShareHigh *big.Rat
	switch {
	case low[0] == nil:
		jLow, jHigh = new(big.Rat).Mul(factor, low[1]), new(big.Rat).Mul(factor, high[1])
		aShareLow, aShareHigh = new(big.Rat), new(big.Rat)
		bShareLow, bShareHigh = big.NewRat(1, 1), big.NewRat(1, 1)
	case low[1] == nil:
		jLow, jHigh = new(big.Rat).Mul(factor, low[0]), new(big.Rat).Mul(factor, high[0])
		aShareLow, aShareHigh = big.NewRat(1, 1), big.NewRat(1, 1)
		bShareLow, bShareHigh = new(big.Rat), new(big.Rat)
	default:
		jLow = new(big.Rat).Mul(factor, reducedMass(low[0], low[1]))
		jHigh = new(big.Rat).Mul(factor, reducedMass(high[0], high[1]))
		aShareLow = massShare(high[0], low[1])
		aShareHigh = massShare(low[0], high[1])
		bShareLow = massShare(high[1], low[0])
		bShareHigh = massShare(low[1], high[0])
	}
	if intervalDeviation(floatImpulse, jLow, jHigh).Cmp(jLimit) > 0 {
		return false
	}
	sign := big.NewRat(int64(normalSign), 1)
	aLow := new(big.Rat).Sub(vA, new(big.Rat).Mul(sign, new(big.Rat).Mul(factor, aShareHigh)))
	aHigh := new(big.Rat).Sub(vA, new(big.Rat).Mul(sign, new(big.Rat).Mul(factor, aShareLow)))
	bLow := new(big.Rat).Add(vB, new(big.Rat).Mul(sign, new(big.Rat).Mul(factor, bShareLow)))
	bHigh := new(big.Rat).Add(vB, new(big.Rat).Mul(sign, new(big.Rat).Mul(factor, bShareHigh)))
	if normalSign < 0 {
		aLow, aHigh = aHigh, aLow
		bLow, bHigh = bHigh, bLow
	}
	for i, pair := range [2][2]*big.Rat{{aLow, aHigh}, {bLow, bHigh}} {
		published := new(big.Rat).SetFloat64(post[i])
		if published == nil || intervalDeviation(published, pair[0], pair[1]).Cmp(vLimit) > 0 {
			return false
		}
	}
	return true
}

func reducedMass(a, b *big.Rat) *big.Rat {
	return new(big.Rat).Quo(new(big.Rat).Mul(a, b), new(big.Rat).Add(a, b))
}

func massShare(own, other *big.Rat) *big.Rat {
	return new(big.Rat).Quo(other, new(big.Rat).Add(own, other))
}

func intervalDeviation(value, low, high *big.Rat) *big.Rat {
	a := absRat(new(big.Rat).Sub(value, low))
	b := absRat(new(big.Rat).Sub(value, high))
	if a.Cmp(b) < 0 {
		return b
	}
	return a
}

func exactBase(value units.Value) *big.Rat {
	mag := new(big.Rat).SetFloat64(value.Mag())
	factor := new(big.Rat).SetFloat64(value.Unit().Factor())
	if mag == nil || factor == nil {
		return nil
	}
	return new(big.Rat).Mul(mag, factor)
}

func absRat(value *big.Rat) *big.Rat {
	if value.Sign() < 0 {
		value.Neg(value)
	}
	return value
}

// Row dominance proves a positive lower eigenvalue for every tensor inside
// the six published inertia intervals. It may reject a valid wider tensor.
func certifiedInertiaLower(m decad.MassProperties) *big.Rat {
	diagonal := []decad.Measurement{m.Inertia.XX, m.Inertia.YY, m.Inertia.ZZ}
	off := []decad.Measurement{m.Inertia.XY, m.Inertia.XZ, m.Inertia.YZ}
	var offUpper [3]*big.Rat
	for i, component := range off {
		value, bound := exactBase(component.Value), exactBase(component.Bound)
		if value == nil || bound == nil || bound.Sign() < 0 {
			return nil
		}
		offUpper[i] = new(big.Rat).Add(absRat(value), bound)
	}
	var lower *big.Rat
	for i, component := range diagonal {
		value, bound := exactBase(component.Value), exactBase(component.Bound)
		if value == nil || bound == nil || bound.Sign() < 0 {
			return nil
		}
		row := new(big.Rat).Sub(value, bound)
		switch i {
		case 0:
			row.Sub(row, offUpper[0]).Sub(row, offUpper[1])
		case 1:
			row.Sub(row, offUpper[0]).Sub(row, offUpper[2])
		case 2:
			row.Sub(row, offUpper[1]).Sub(row, offUpper[2])
		}
		if lower == nil || row.Cmp(lower) < 0 {
			lower = row
		}
	}
	return lower
}

// This slice publishes zero spin. Bound the spin omitted by that choice using
// the full contact-point ball, center ball, response residual, and inertia floor.
func omittedSpinWithin(manifold *decad.ContactManifold, pose r3.Transform, mass decad.MassProperties,
	side, axis int, impulse float64, impulseLimit, angularLimit units.Value) bool {
	if manifold == nil || len(manifold.Points) == 0 || !finite(impulse) {
		return false
	}
	if side < 0 || side > 1 || axis < 0 || axis > 2 {
		return false
	}
	lower := certifiedInertiaLower(mass)
	jLimit, wLimit, centerBound := exactBase(impulseLimit), exactBase(angularLimit), exactBase(mass.Center.Bound)
	if lower == nil || lower.Sign() <= 0 || jLimit == nil || wLimit == nil || centerBound == nil {
		return false
	}
	tangents := [2]int{(axis + 1) % 3, (axis + 2) % 3}
	coordinates := [2]*big.Rat{new(big.Rat), new(big.Rat)}
	pointBound := new(big.Rat)
	for _, point := range manifold.Points {
		witness := point.OnA
		if side == 1 {
			witness = point.OnB
		}
		components := [3]float64{witness.Value.X, witness.Value.Y, witness.Value.Z}
		bound := exactBase(witness.Bound)
		if bound == nil || bound.Sign() < 0 {
			return false
		}
		for i, tangent := range tangents {
			coordinate := new(big.Rat).SetFloat64(components[tangent])
			if coordinate == nil {
				return false
			}
			coordinates[i].Add(coordinates[i], coordinate)
		}
		pointBound.Add(pointBound, bound)
	}
	count := new(big.Rat).SetInt64(int64(len(manifold.Points)))
	for _, coordinate := range coordinates {
		coordinate.Quo(coordinate, count)
	}
	pointBound.Quo(pointBound, count)
	// Each witness bound is a spatial ball; twice its radius safely bounds
	// the sum of the two tangential coordinate errors.
	pointBound.Mul(pointBound, big.NewRat(2, 1))
	center := pose.Apply(mass.Center.Value)
	centerValues := [3]float64{center.X, center.Y, center.Z}
	committedCenter := [3]float64{mass.Center.Value.X, mass.Center.Value.Y, mass.Center.Value.Z}
	translation := [3]float64{pose.Translation().X, pose.Translation().Y, pose.Translation().Z}
	j := new(big.Rat).SetFloat64(impulse)
	if j == nil {
		return false
	}
	lever := new(big.Rat).Set(pointBound)
	for i, tangent := range tangents {
		c := new(big.Rat).SetFloat64(centerValues[tangent])
		committed := new(big.Rat).SetFloat64(committedCenter[tangent])
		move := new(big.Rat).SetFloat64(translation[tangent])
		if c == nil || committed == nil || move == nil {
			return false
		}
		exactCenter := new(big.Rat).Add(committed, move)
		lever.Add(lever, absRat(new(big.Rat).Sub(coordinates[i], c)))
		lever.Add(lever, absRat(new(big.Rat).Sub(c, exactCenter)))
	}
	lever.Add(lever, new(big.Rat).Mul(centerBound, big.NewRat(2, 1)))
	upperImpulse := absRat(j)
	upperImpulse.Add(upperImpulse, jLimit)
	spin := new(big.Rat).Mul(upperImpulse, lever)
	spin.Quo(spin, lower) // radians are a scalar at this response boundary.
	return spin.Cmp(wLimit) <= 0
}

// The reported elapsed readings enclose the exact dyadic sweep instants.
// Outward arithmetic makes their interval a conservative travel allowance.
func boundBracketTravel(bracket decad.SweepInterval, velocity float64) (float64, bool) {
	from, to := bracket.From.Elapsed, bracket.To.Elapsed
	if from.Value.Kind() != units.Time || from.Bound.Kind() != units.Time ||
		to.Value.Kind() != units.Time || to.Bound.Kind() != units.Time ||
		!finite(velocity, from.Value.Base(), from.Bound.Base(), to.Value.Base(), to.Bound.Base()) ||
		from.Bound.Base() < 0 || to.Bound.Base() < 0 {
		return 0, false
	}
	upper := math.Nextafter(to.Value.Base()+to.Bound.Base(), math.Inf(1))
	lower := math.Nextafter(from.Value.Base()-from.Bound.Base(), math.Inf(-1))
	width := math.Nextafter(upper-lower, math.Inf(1))
	travel := math.Nextafter(math.Abs(velocity)*width, math.Inf(1))
	return travel, finite(upper, lower, width, travel) && width >= 0
}

func outwardSum(values ...float64) float64 {
	sum := 0.0
	for _, value := range values {
		sum += value
		if sum != 0 {
			sum = math.Nextafter(sum, math.Inf(1))
		}
	}
	return sum
}

func correctionWithin(before, after r3.Transform, allowance float64) bool {
	z0, z1 := before.Translation().Z, after.Translation().Z
	if !finite(z0, z1, allowance) || allowance < 0 {
		return false
	}
	if z0 == z1 {
		return true
	}
	actualUpper := math.Nextafter(math.Abs(z1-z0), math.Inf(1))
	return finite(actualUpper) && actualUpper <= allowance
}

func (w *World) sweep(ctx context.Context, state State, duration units.Value,
	policy decad.SweepStartPolicy) (*decad.SweepReport, error) {
	var paths [2]decad.PairPath
	for i, entry := range state.entries {
		if w.bodies[i].definition.Role == Fixed {
			paths[i] = decad.PoseSegment{From: entry.Pose, To: entry.Pose, Duration: duration}
			continue
		}
		paths[i] = decad.RigidDriftSegment{
			From: entry.Pose, Center: entry.Pose.Apply(w.bodies[i].mass.Center.Value),
			LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
			Duration: duration,
		}
	}
	return w.doc.SweepPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		paths[0], paths[1], w.sweepRequest(duration, policy))
}

func (w *World) sweepPoses(ctx context.Context, from, to State, duration units.Value,
	policy decad.SweepStartPolicy) (*decad.SweepReport, error) {
	return w.doc.SweepPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		decad.PoseSegment{From: from.entries[0].Pose, To: to.entries[0].Pose, Duration: duration},
		decad.PoseSegment{From: from.entries[1].Pose, To: to.entries[1].Pose, Duration: duration},
		w.sweepRequest(duration, policy))
}

// sweepRequest is the request of every SweepPair the step runs. RestSpeed is
// VelocityResidual (docs/multibody-dynamics-design.md §10.8): §6.3 leaves a
// resting point's normal speed within that residual of zero, so a lifted
// vertex the solve just rested is held on both sides of the support plane on
// the next slice, while one that arrives faster still ends its band track a
// grid step before the plane.
func (w *World) sweepRequest(duration units.Value, policy decad.SweepStartPolicy) decad.SweepRequest {
	resolution := w.step.TimeResolution
	if duration.Base() < resolution.Base() {
		resolution = duration
	}
	return decad.SweepRequest{ContactRequest: w.step.Contact, TimeResolution: resolution,
		MaxPoseEvaluations: w.step.MaxPoseEvaluations, StartPolicy: policy, RestSpeed: w.step.VelocityResidual}
}

func reducedContact(manifold *decad.ContactManifold, req decad.ContactRequest) (r3.Vec, float64, float64, bool) {
	var normal r3.Vec
	var separation, bound float64
	for _, cp := range manifold.Points {
		if !finite(cp.OnA.Value.X, cp.OnA.Value.Y, cp.OnA.Value.Z,
			cp.OnB.Value.X, cp.OnB.Value.Y, cp.OnB.Value.Z,
			cp.OnA.Bound.Base(), cp.OnB.Bound.Base(), cp.Normal.Bound.Base(),
			cp.NormalAngle.Base(), cp.Separation.Value.Base(), cp.Separation.Bound.Base()) ||
			cp.OnA.Bound.Base() > req.PointResolution.Base() ||
			cp.OnB.Bound.Base() > req.PointResolution.Base() ||
			cp.NormalAngle.Base() > req.NormalResolution.Base() ||
			cp.Normal.Bound.Base() != 0 || cp.NormalAngle.Base() != 0 {
			return r3.Vec{}, 0, 0, false
		}
		if len(manifold.Points) > 1 && cp.Normal.Value != manifold.Points[0].Normal.Value {
			return r3.Vec{}, 0, 0, false
		}
		normal = normal.Add(cp.Normal.Value)
		separation += cp.Separation.Value.Base()
		pointBound := outwardSum(cp.Separation.Bound.Base(), cp.OnA.Bound.Base(), cp.OnB.Bound.Base())
		if !finite(pointBound) {
			return r3.Vec{}, 0, 0, false
		}
		bound = math.Max(bound, pointBound)
	}
	count := float64(len(manifold.Points))
	normal, separation = normal.Scale(1/count), separation/count
	return normal, separation, bound, finite(normal.X, normal.Y, normal.Z, separation)
}

func cloneManifold(m decad.ContactManifold) decad.ContactManifold {
	m.Points = append([]decad.ContactPoint(nil), m.Points...)
	return m
}

// undecided names the world's only pair in a two-body world; a larger world
// leaves the pair to the caller that knows which pair stopped the step.
func undecided(w *World, reason string) *StepReport {
	var pair BodyPair
	if len(w.bodies) == 2 {
		pair = w.bodyPair(w.pairs[0])
	}
	return &StepReport{Status: Undecided, Excluded: w.Excluded(), Diagnostics: []StepDiagnostic{{
		Pair: pair, Reason: reason,
	}}}
}

func undecidedArithmetic(w *World, reason string, err error) (*StepReport, error) {
	return undecided(w, reason+": "+err.Error()), nil
}
