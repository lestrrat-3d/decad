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

type StepDiagnostic struct {
	Pair   BodyPair
	Reason string
}

// ContactEventKind separates an impulse-bearing contact from a geometry-only transition.
type ContactEventKind int

const (
	ContactImpact ContactEventKind = iota + 1
	ContactTransition
)

type ContactEvent struct {
	Kind                             ContactEventKind
	Pair                             BodyPair
	Bracket                          decad.SweepInterval
	Time                             units.Value
	Manifold                         decad.ContactManifold
	NormalImpulse                    units.Value
	TangentImpulse                   QuantityVec
	PointImpulses                    []ContactPointImpulse
	Solver                           *ContactSolverReport
	PreVelocity                      QuantityVec
	PostVelocity                     QuantityVec
	PositionChange                   r3.Vec
	PreVelocityA, PreVelocityB       QuantityVec
	PostVelocityA, PostVelocityB     QuantityVec
	PositionChangeA, PositionChangeB r3.Vec
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
}

// Trace keeps the certified drift and the event state for replay in this first slice.
type Trace struct {
	start    State
	pre      State
	post     State
	end      State
	duration units.Value
	eventAt  units.Value
	hasEvent bool
}

// Sample returns recorded checkpoint states. Interior poses need a separate
// float-pose certificate and are refused by this first slice.
func (tr Trace) Sample(t units.Value) (State, error) {
	if t.Kind() != units.Time || !finite(t.Base()) || t.Base() < 0 || t.Base() > tr.duration.Base() {
		return State{}, fmt.Errorf("%w: trace time outside step", ErrInvalidInput)
	}
	if tr.hasEvent && t.Base() == tr.eventAt.Base() {
		return tr.post, nil
	}
	if t.Base() == 0 {
		return tr.start, nil
	}
	if t.Base() == tr.duration.Base() {
		return tr.end, nil
	}
	return State{}, fmt.Errorf("%w: interior trace sample has no float-pose contact certificate", ErrUnsupported)
}

func driftState(start State, seconds float64) (State, error) {
	out := start
	for i := range out.entries {
		v := out.entries[i].LinearVelocity
		delta := r3.Vec{X: v.X.Base() * seconds, Y: v.Y.Base() * seconds, Z: v.Z.Base() * seconds}
		pose, err := translatePose(out.entries[i].Pose, delta)
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
	out := start
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
func (w *World) Step(ctx context.Context, from State, input StepInput, dt units.Value) (*StepReport, error) {
	if w == nil || ctx == nil || from.world != w || !validQuantity(dt, units.Time, true) {
		return nil, fmt.Errorf("%w: invalid context, world, state, or duration", ErrInvalidInput)
	}
	if w.three != nil {
		return w.stepThreeBodies(ctx, from, input, dt)
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
	for _, part := range w.parts {
		if !containsBody(live, part.definition.Body) {
			return nil, fmt.Errorf("%w: world body was retired", ErrInvalidInput)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	kicked, ok := w.kickByLoads(from, input.Gravity, loads, dt)
	if !ok {
		return undecided(w, "force kick exceeds the velocity residual"), nil
	}
	report, err := w.stepKicked(ctx, from, kicked, dt, driver)
	if report != nil {
		report.Excluded = w.Excluded()
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
		input.Gravity, loads, dt)
	if !ok {
		return undecided(w, "conservation readings cannot be represented with finite bounds"), nil
	}
	report.Conservation = &conservation
	return report, nil
}

func (w *World) stepKicked(ctx context.Context, from, kicked State, dt units.Value,
	driver kinematicMotion) (*StepReport, error) {
	if len(w.excluded) != 0 {
		return w.stepExcluded(ctx, from, kicked, dt, driver)
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
	if !moving {
		return w.stepStill(ctx, from, kicked, dt)
	}
	for _, entry := range kicked.entries {
		if entry.Pose.ApplyDir(r3.Vec{X: 1}) != (r3.Vec{X: 1}) ||
			entry.Pose.ApplyDir(r3.Vec{Y: 1}) != (r3.Vec{Y: 1}) ||
			entry.Pose.ApplyDir(r3.Vec{Z: 1}) != (r3.Vec{Z: 1}) {
			return nil, fmt.Errorf("%w: rotated poses are not implemented", ErrUnsupported)
		}
	}

	first, err := w.sweep(ctx, kicked, dt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	report := &StepReport{Trace: Trace{start: from, duration: dt}}
	switch first.Outcome {
	case decad.SweepClear:
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
		return report, nil
	case decad.SweepInitiallyTouching:
		if w.friction.lower.Sign() != 0 {
			return w.stepInitialFriction(ctx, from, kicked, dt, first)
		}
		return w.stepInitialTouch(ctx, from, kicked, dt, first)
	case decad.SweepImpactBracket:
		if w.friction.lower.Sign() != 0 {
			return undecided(w, "frictional interior impact is not certified"), nil
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
	pre, err := driftState(kicked, impactTime)
	if err != nil {
		return undecidedArithmetic(w, "non-finite impact pose", err)
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	if !ok {
		return undecided(w, "contact normal or point is outside the admitted resolution"), nil
	}
	axis, normalSign, valid := axisNormal(normal)
	if !valid {
		return undecided(w, "impact normal is not a supported axis"), nil
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
	for i := range w.parts {
		if w.parts[i].definition.Role == Dynamic {
			inverseMass[i] = 1 / w.parts[i].mass.Mass.Value.Base()
		}
	}
	denominator := inverseMass[0] + inverseMass[1]
	if !finite(denominator) || denominator <= 0 {
		return undecided(w, "effective mass is not finite and positive"), nil
	}
	coefficient := w.restitution
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
	if !responsePairResidualsWithin(preSpeed, normalSign, effectiveCoefficient,
		w.parts, target, impulse, postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) {
		return undecided(w, "exact response bounds exceed velocity or impulse residual"), nil
	}
	for i, part := range w.parts {
		if part.definition.Role != Dynamic {
			continue
		}
		if !omittedSpinWithin(first.Event.Manifold, pre.entries[i].Pose, part.mass,
			i, axis, impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
			return undecided(w, "off-center impulse exceeds angular velocity residual"), nil
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
	contact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
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
		contact, err = w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
			post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
	}
	if contact.Relation != decad.ContactTouching {
		return undecided(w, fmt.Sprintf("corrected impact pose has relation %v (separation %.17g)",
			contact.Relation, separation)), nil
	}
	remaining := dt.Base() - impactTime
	postRelative := (postSpeed[1] - postSpeed[0]) * normalSign
	if !finite(postRelative) || postRelative < -w.step.VelocityResidual.Base() {
		return undecided(w, "impact response remains closing"), nil
	}
	policy := decad.ContinueSeparatingTouch
	persistent := postRelative <= w.step.VelocityResidual.Base()
	if persistent {
		policy = decad.ContinueCertifiedTouch
	}
	if remaining > 0 {
		continuation, err := w.sweep(ctx, post, units.Seconds(remaining), policy)
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
	if remaining > 0 {
		actual, err := w.sweepPoses(ctx, post, end, units.Seconds(remaining), policy)
		if err != nil {
			return nil, err
		}
		if (persistent && !w.persistentTrackWithin(actual, normal)) ||
			(!persistent && actual.Outcome != decad.SweepDepartedClear) {
			return undecided(w, fmt.Sprintf("numerical rebound path returned %v", actual.Outcome)), nil
		}
	}
	finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
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
	if w.parts[0].definition.Role == Fixed {
		reportBody = 1
	}
	report.Status = Advanced
	report.Next = &end
	report.Events = []ContactEvent{{
		Kind:            ContactImpact,
		Pair:            BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:         *first.Bracket,
		Time:            units.Seconds(impactTime),
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
	report.Trace = Trace{start: from, pre: pre, post: post, end: end, duration: dt,
		eventAt: units.Seconds(impactTime), hasEvent: true}
	return report, nil
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
		Trace: Trace{start: from, end: end, duration: dt}}, nil
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
	parts [2]worldBody, target, impulse float64, post [2]float64,
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
		if w.parts[i].definition.Role == Fixed {
			paths[i] = decad.PoseSegment{From: entry.Pose, To: entry.Pose, Duration: duration}
			continue
		}
		paths[i] = decad.RigidDriftSegment{
			From: entry.Pose, Center: entry.Pose.Apply(w.parts[i].mass.Center.Value),
			LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
			Duration: duration,
		}
	}
	return w.doc.SweepPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		paths[0], paths[1], w.sweepRequest(duration, policy))
}

func (w *World) sweepPoses(ctx context.Context, from, to State, duration units.Value,
	policy decad.SweepStartPolicy) (*decad.SweepReport, error) {
	return w.doc.SweepPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		decad.PoseSegment{From: from.entries[0].Pose, To: to.entries[0].Pose, Duration: duration},
		decad.PoseSegment{From: from.entries[1].Pose, To: to.entries[1].Pose, Duration: duration},
		w.sweepRequest(duration, policy))
}

func (w *World) sweepRequest(duration units.Value, policy decad.SweepStartPolicy) decad.SweepRequest {
	resolution := w.step.TimeResolution
	if duration.Base() < resolution.Base() {
		resolution = duration
	}
	return decad.SweepRequest{ContactRequest: w.step.Contact, TimeResolution: resolution,
		MaxPoseEvaluations: w.step.MaxPoseEvaluations, StartPolicy: policy}
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

func undecided(w *World, reason string) *StepReport {
	return &StepReport{Status: Undecided, Excluded: w.Excluded(), Diagnostics: []StepDiagnostic{{
		Pair: BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body}, Reason: reason,
	}}}
}

func undecidedArithmetic(w *World, reason string, err error) (*StepReport, error) {
	return undecided(w, reason+": "+err.Error()), nil
}
