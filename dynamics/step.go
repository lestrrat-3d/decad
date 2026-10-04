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

type ContactEvent struct {
	Pair           BodyPair
	Bracket        decad.SweepInterval
	Time           units.Value
	Manifold       decad.ContactManifold
	NormalImpulse  units.Value
	PreVelocity    QuantityVec
	PostVelocity   QuantityVec
	PositionChange r3.Vec
}

type StepReport struct {
	Status      StepStatus
	Next        *State
	Events      []ContactEvent
	Trace       Trace
	Diagnostics []StepDiagnostic
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
	if t.Base() == 0 {
		return tr.start, nil
	}
	if t.Base() == tr.duration.Base() {
		return tr.end, nil
	}
	if tr.hasEvent && t.Base() == tr.eventAt.Base() {
		return tr.post, nil
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

// Step advances the admitted pair with a certified first-impact bracket and frictionless impulse.
func (w *World) Step(ctx context.Context, from State, input StepInput, dt units.Value) (*StepReport, error) {
	if w == nil || ctx == nil || from.world != w || !validQuantity(dt, units.Time, true) {
		return nil, fmt.Errorf("%w: invalid context, world, state, or duration", ErrInvalidInput)
	}
	if err := validateQuantityVec(input.Gravity, units.Acceleration); err != nil {
		return nil, err
	}
	if len(input.Loads) != 0 || len(input.Drivers) != 0 ||
		input.Gravity.X.Base() != 0 || input.Gravity.Y.Base() != 0 || input.Gravity.Z.Base() != 0 {
		return nil, fmt.Errorf("%w: gravity, loads, and drivers are not implemented", ErrUnsupported)
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
	dyn := 0
	if w.parts[0].definition.Role != Dynamic {
		dyn = 1
	}
	velocity := from.entries[dyn].LinearVelocity
	if velocity.X.Base() != 0 || velocity.Y.Base() != 0 || velocity.Z.Base() == 0 {
		return nil, fmt.Errorf("%w: this stage requires nonzero vertical translation", ErrUnsupported)
	}
	for _, entry := range from.entries {
		if entry.Pose.ApplyDir(r3.Vec{X: 1}) != (r3.Vec{X: 1}) ||
			entry.Pose.ApplyDir(r3.Vec{Y: 1}) != (r3.Vec{Y: 1}) ||
			entry.Pose.ApplyDir(r3.Vec{Z: 1}) != (r3.Vec{Z: 1}) {
			return nil, fmt.Errorf("%w: rotated poses are not implemented", ErrUnsupported)
		}
	}

	first, err := w.sweep(ctx, from, dt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	report := &StepReport{Trace: Trace{start: from, duration: dt}}
	switch first.Outcome {
	case decad.SweepClear:
		end, err := driftState(from, dt.Base())
		if err != nil {
			return undecidedArithmetic(w, "non-finite clear-path pose", err)
		}
		actual, err := w.sweepPoses(ctx, from, end, dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if actual.Outcome != decad.SweepClear {
			return undecided(w, fmt.Sprintf("numerical clear path returned %v", actual.Outcome)), nil
		}
		report.Status, report.Next, report.Trace.end = Advanced, &end, end
		return report, nil
	case decad.SweepImpactBracket:
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
	pre, err := driftState(from, impactTime)
	if err != nil {
		return undecidedArithmetic(w, "non-finite impact pose", err)
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	if !ok {
		return undecided(w, "contact normal or point is outside the admitted resolution"), nil
	}
	if normal.X != 0 || normal.Y != 0 || math.Abs(normal.Z) != 1 {
		return undecided(w, "nonvertical impact requires angular response"), nil
	}
	relativeSpeed := velocity.Z.Base() * normal.Z
	if dyn == 0 {
		relativeSpeed = -relativeSpeed
	}
	if relativeSpeed >= -w.step.VelocityResidual.Base() {
		return undecided(w, "impact is not closing"), nil
	}
	mass := w.parts[dyn].mass.Mass.Value.Base()
	coefficient := w.parts[0].definition.Material.Restitution
	if exactBase(w.parts[1].definition.Material.Restitution).Cmp(exactBase(coefficient)) < 0 {
		coefficient = w.parts[1].definition.Material.Restitution
	}
	relativeSign := normal.Z
	if dyn == 0 {
		relativeSign = -normal.Z
	}
	idealRelative := exactBase(velocity.Z)
	idealThreshold := exactBase(w.step.ImpactSpeed)
	if idealRelative == nil || idealThreshold == nil {
		return undecided(w, "impact speed is not representable"), nil
	}
	idealRelative.Mul(idealRelative, new(big.Rat).SetInt64(int64(relativeSign)))
	target := 0.0
	effectiveCoefficient := units.Scalar(0)
	if new(big.Rat).Neg(idealRelative).Cmp(idealThreshold) > 0 {
		effectiveCoefficient = coefficient
		target = -coefficient.Base() * relativeSpeed
	}
	impulse := (target - relativeSpeed) * mass
	if !finite(impulse) || impulse <= 0 {
		return undecided(w, "impulse is not finite and positive"), nil
	}
	if !responseResidualsWithin(velocity.Z, relativeSign, effectiveCoefficient,
		w.parts[dyn].mass.Mass, target, impulse,
		w.step.VelocityResidual, w.step.ImpulseResidual) {
		return undecided(w, "exact response bounds exceed velocity or impulse residual"), nil
	}
	if !omittedSpinWithin(first.Event.Manifold, pre.entries[dyn].Pose, w.parts[dyn].mass,
		impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
		return undecided(w, "off-center impulse exceeds angular velocity residual"), nil
	}
	postSpeed := -target * normal.Z
	if dyn == 1 {
		postSpeed = target * normal.Z
	}
	if math.Abs(postSpeed-velocity.Z.Base()) < w.step.VelocityResidual.Base() {
		return undecided(w, "impact did not change velocity"), nil
	}
	bracketTravel, valid := boundBracketTravel(*first.Bracket, velocity.Z.Base())
	if !valid || !finite(separation, bound) || separation > bound {
		return undecided(w, "impact penetration or bracket travel is not certified"), nil
	}
	correctionAllowance := outwardSum(bracketTravel, w.step.ContactSlop.Base(), bound)
	if !finite(correctionAllowance) || -separation > correctionAllowance {
		return undecided(w, "impact penetration exceeds its certified bracket"), nil
	}
	correction := -separation * normal.Z
	if dyn == 0 {
		correction = separation * normal.Z
	}
	post := pre
	post.entries[dyn].Pose, err = translatePose(pre.entries[dyn].Pose, r3.Vec{Z: correction})
	if err != nil {
		return undecidedArithmetic(w, "position correction is not finite", err)
	}
	if !correctionWithin(pre.entries[dyn].Pose, post.entries[dyn].Pose, correctionAllowance) {
		return undecided(w, "actual position correction exceeds its certified allowance"), nil
	}
	post.entries[dyn].LinearVelocity.Z = units.MillimetersPerSecond(postSpeed)
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
		increment := -nextSeparation * normal.Z
		if dyn == 0 {
			increment = nextSeparation * normal.Z
		}
		if !finite(increment) {
			break
		}
		candidate, err := translatePose(post.entries[dyn].Pose, r3.Vec{Z: increment})
		if err != nil {
			return undecidedArithmetic(w, "position correction is not finite", err)
		}
		correctionAllowance = outwardSum(correctionAllowance, nextBound)
		if !finite(correctionAllowance) || !correctionWithin(pre.entries[dyn].Pose, candidate, correctionAllowance) {
			break
		}
		post.entries[dyn].Pose = candidate
		correction += increment
		contact, err = w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
			post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
	}
	if contact.Relation != decad.ContactTouching {
		return undecided(w, fmt.Sprintf("corrected impact pose has relation %v (separation %.17g, correction %.17g, pose z %.17g)",
			contact.Relation, separation, correction, post.entries[dyn].Pose.Translation().Z)), nil
	}
	correction = post.entries[dyn].Pose.Translation().Z - pre.entries[dyn].Pose.Translation().Z
	remaining := dt.Base() - impactTime
	if remaining > 0 {
		continuation, err := w.sweep(ctx, post, units.Seconds(remaining), decad.ContinueSeparatingTouch)
		if err != nil {
			return nil, err
		}
		if continuation.Outcome != decad.SweepDepartedClear {
			return undecided(w, fmt.Sprintf("rebound departure returned %v", continuation.Outcome)), nil
		}
	}
	end, err := driftState(post, remaining)
	if err != nil {
		return undecidedArithmetic(w, "non-finite final pose", err)
	}
	if remaining > 0 {
		actual, err := w.sweepPoses(ctx, post, end, units.Seconds(remaining), decad.ContinueSeparatingTouch)
		if err != nil {
			return nil, err
		}
		if actual.Outcome != decad.SweepDepartedClear {
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
	impulseValue := units.KilogramMillimetersPerSecond(impulse)
	report.Status = Advanced
	report.Next = &end
	report.Events = []ContactEvent{{
		Pair:           BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:        *first.Bracket,
		Time:           units.Seconds(impactTime),
		Manifold:       cloneManifold(*first.Event.Manifold),
		NormalImpulse:  impulseValue,
		PreVelocity:    velocity,
		PostVelocity:   post.entries[dyn].LinearVelocity,
		PositionChange: r3.Vec{Z: correction},
	}}
	report.Trace = Trace{start: from, pre: pre, post: post, end: end, duration: dt,
		eventAt: units.Seconds(impactTime), hasEvent: true}
	return report, nil
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
	impulse float64, impulseLimit, angularLimit units.Value) bool {
	if manifold == nil || len(manifold.Points) == 0 || !finite(impulse) {
		return false
	}
	lower := certifiedInertiaLower(mass)
	jLimit, wLimit, centerBound := exactBase(impulseLimit), exactBase(angularLimit), exactBase(mass.Center.Bound)
	if lower == nil || lower.Sign() <= 0 || jLimit == nil || wLimit == nil || centerBound == nil {
		return false
	}
	x, y, pointBound := new(big.Rat), new(big.Rat), new(big.Rat)
	for _, point := range manifold.Points {
		ax, ay := new(big.Rat).SetFloat64(point.OnA.Value.X),
			new(big.Rat).SetFloat64(point.OnA.Value.Y)
		bx, by := new(big.Rat).SetFloat64(point.OnB.Value.X),
			new(big.Rat).SetFloat64(point.OnB.Value.Y)
		ab, bb := exactBase(point.OnA.Bound), exactBase(point.OnB.Bound)
		if ax == nil || ay == nil || bx == nil || by == nil || ab == nil || bb == nil ||
			ab.Sign() < 0 || bb.Sign() < 0 {
			return false
		}
		x.Add(x, ax).Add(x, bx)
		y.Add(y, ay).Add(y, by)
		pointBound.Add(pointBound, ab).Add(pointBound, bb)
	}
	count := new(big.Rat).SetInt64(int64(2 * len(manifold.Points)))
	x.Quo(x, count)
	y.Quo(y, count)
	pointBound.Quo(pointBound, count)
	cx, cy := new(big.Rat).SetFloat64(mass.Center.Value.X),
		new(big.Rat).SetFloat64(mass.Center.Value.Y)
	px, py := new(big.Rat).SetFloat64(pose.Translation().X),
		new(big.Rat).SetFloat64(pose.Translation().Y)
	j := new(big.Rat).SetFloat64(impulse)
	if cx == nil || cy == nil || px == nil || py == nil || j == nil {
		return false
	}
	cx.Add(cx, px)
	cy.Add(cy, py)
	lever := absRat(x.Sub(x, cx))
	lever.Add(lever, absRat(y.Sub(y, cy))).Add(lever, pointBound).Add(lever, centerBound)
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
	return &StepReport{Status: Undecided, Diagnostics: []StepDiagnostic{{
		Pair: BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body}, Reason: reason,
	}}}
}

func undecidedArithmetic(w *World, reason string, err error) (*StepReport, error) {
	return undecided(w, reason+": "+err.Error()), nil
}
