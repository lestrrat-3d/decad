package dynamics

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file certifies an island proposal in exact rational interval
// arithmetic (docs/multibody-dynamics-design.md §6.3).
// Every comparison runs over the whole interval: mass, inertia, mass-center,
// contact-point and normal uncertainty all enter as intervals, and the
// published velocities and impulses enter as the exact rationals their
// float64 values denote. The certificate never inverts an interval tensor.

type ivec = [3]proof.RatInterval

// certBody is one island participant read as exact intervals. A Fixed
// participant has zero velocity and no mass reading; a kinematic one moves
// with its driver's exact velocity field (driverMotion) before and after the
// event.
type certBody struct {
	index     int
	dynamic   bool
	kinematic bool
	mass      proof.RatInterval
	inertia   [3][3]proof.RatInterval
	rotation  [3][3]*big.Rat // pose basis, rotation[row][column]
	// defect widens every world inertia component by 3·d·(2+d)·m, with d the
	// entrywise absolute sum of RᵀR − I and m the local tensor's largest
	// magnitude (§8.1's orthonormality-defect bound).
	defect       *big.Rat
	inertiaLower *big.Rat // certified lower eigenvalue, rotation invariant
	rowCeiling   *big.Rat
	center       ivec
	centerL1     *big.Rat // upper bound on |center|_1
	pose         r3.Transform
	v, w         [3]*big.Rat // pre-solve
	vPost, wPost [3]*big.Rat // published
}

// certPoint is one manifold point of an island pair. Its lever intervals run
// from each dynamic body's world mass center to that body's witness.
type certPoint struct {
	pair        int // index into island.pairs
	a, b        int // island body slots
	normal      ivec
	onA, onB    ivec
	rA, rB      ivec
	ballA       *big.Rat // the witness ball radius on A (OnA.Bound)
	ballB       *big.Rat // the witness ball radius on B (OnB.Bound)
	lambda      *big.Rat
	tangent     [3]*big.Rat // published world tangent impulse on B; zero without friction
	mu          *big.Rat    // the pair's lower friction coefficient μ_lo
	restitution *big.Rat
}

// islandGate names one row of §6.3's certificate table.
type islandGate int

const (
	gateLinearLaw islandGate = iota + 1
	gateAngularLaw
	gateNormalSign
	gateRestitutionTarget
	gateNonPenetration
	gateComplementarity
	gateCone
	gateStick
	gateSlip
	gateEnergy
	gateLinearMomentum
	gateAngularMomentum
)

// limitValue publishes a gate's exact limit as a quantity of the gate's kind,
// rounded up.
func (g islandGate) limitValue(limit *big.Rat) units.Value {
	if limit == nil {
		return units.Value{}
	}
	f, _ := limit.Float64()
	if ratFloat(f) != nil && ratFloat(f).Cmp(limit) < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	switch g {
	case gateLinearLaw, gateLinearMomentum:
		return units.KilogramMillimetersPerSecond(f)
	case gateAngularLaw, gateAngularMomentum:
		return units.New(f, units.KilogramSquareMillimeterPerSecond)
	case gateEnergy, gateSlip:
		return units.New(f, units.KilogramSquareMillimeterPerSecondSquared)
	case gateNormalSign, gateCone:
		return units.KilogramMillimetersPerSecond(f)
	default:
		return units.MillimetersPerSecond(f)
	}
}

func (g islandGate) String() string {
	switch g {
	case gateLinearLaw:
		return "linear law"
	case gateAngularLaw:
		return "angular law"
	case gateNormalSign:
		return "normal sign"
	case gateRestitutionTarget:
		return "restitution target"
	case gateNonPenetration:
		return "non-penetration"
	case gateComplementarity:
		return "complementarity"
	case gateCone:
		return "cone"
	case gateStick:
		return "stick"
	case gateSlip:
		return "slip"
	case gateEnergy:
		return "energy"
	case gateLinearMomentum:
		return "linear momentum"
	case gateAngularMomentum:
		return "angular momentum"
	default:
		return "unknown gate"
	}
}

// islandCertificate holds the largest attained value of each gate (the
// signed upper end of the kinetic-energy change; the cone's excess over
// μ_lo·λn; a sticking point's tangent speed), the largest published
// post-solve angular speed, the largest witness torque T_β and spin
// T_β / λ_lo(I_β) over the dynamic bodies, every refused gate, and the first
// refused gate in evaluation order.
type islandCertificate struct {
	linear, angular, normal, energy, momentum, angularMomentum *big.Rat
	cone, tangent, spin                                        *big.Rat
	witnessTorque, witnessSpin                                 *big.Rat
	failed                                                     islandGate
	value, limit                                               *big.Rat
	refused                                                    map[islandGate]struct{}
}

// fail records a refused gate; the first one, in table order of evaluation,
// names the refusal.
func (c *islandCertificate) fail(gate islandGate, value, limit *big.Rat) {
	if c.refused == nil {
		c.refused = map[islandGate]struct{}{}
	}
	c.refused[gate] = struct{}{}
	if c.failed != 0 {
		return
	}
	c.failed, c.value, c.limit = gate, value, limit
}

func ratVec(v r3.Vec) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	for axis, x := range [3]float64{v.X, v.Y, v.Z} {
		out[axis] = ratFloat(x)
		if out[axis] == nil {
			return out, false
		}
	}
	return out, true
}

func quantityRats(q QuantityVec) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	for axis := range out {
		out[axis] = exactBase(velocityComponent(q, axis))
		if out[axis] == nil {
			return out, false
		}
	}
	return out, true
}

func pointIVec(x [3]*big.Rat) ivec {
	var out ivec
	for axis := range out {
		out[axis] = proof.PointInterval(x[axis])
	}
	return out
}

// ballIVec encloses every point within radius of v in each coordinate.
func ballIVec(v r3.Vec, radius *big.Rat) (ivec, bool) {
	center, ok := ratVec(v)
	if !ok || radius == nil || radius.Sign() < 0 {
		return ivec{}, false
	}
	var out ivec
	for axis := range out {
		out[axis] = proof.OwnedInterval(proof.SubRat(new(big.Rat), center[axis], radius),
			proof.AddRat(new(big.Rat), center[axis], radius))
	}
	return out, true
}

func addIVec(a, b ivec) ivec {
	var out ivec
	for axis := range out {
		out[axis] = proof.AddInterval(a[axis], b[axis])
	}
	return out
}

func subIVec(a, b ivec) ivec {
	var out ivec
	for axis := range out {
		out[axis] = proof.SubInterval(a[axis], b[axis])
	}
	return out
}

func zeroIVec() ivec {
	return pointIVec([3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)})
}

// magnitude is the largest absolute value an interval admits.
func magnitude(iv proof.RatInterval) *big.Rat {
	lo, hi := absRat(new(big.Rat).Set(iv.Lo)), absRat(new(big.Rat).Set(iv.Hi))
	if lo.Cmp(hi) > 0 {
		return lo
	}
	return hi
}

// newCertBody reads one island participant at its event pose. A kinematic
// participant reads its driver's exact velocity field from drive: v about
// the world origin, which is its center, and the angular part w, both
// unchanged by the event.
func (w *World) newCertBody(index int, entry BodyState, post BodyState,
	drive map[int]driverMotion) (certBody, bool) {
	body, ok := w.certMotion(index, entry, drive)
	if !ok || !body.dynamic {
		return body, ok
	}
	exact := w.bodies[index].exact
	body.mass = exact.interval
	body.inertia = exact.tensor
	basis := body.pose.Basis()
	for column, axis := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		values, _ := ratVec(axis) // certMotion read the basis as finite
		for row := range 3 {
			body.rotation[row][column] = values[row]
		}
	}
	d := new(big.Rat)
	for i := range 3 {
		for j := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				proof.AddRat(entry, entry, proof.MulRat(new(big.Rat), body.rotation[k][i], body.rotation[k][j]))
			}
			if i == j {
				proof.SubRat(entry, entry, big.NewRat(1, 1))
			}
			proof.AddRat(d, d, absRat(entry))
		}
	}
	body.defect = proof.MulRat(new(big.Rat), big.NewRat(3, 1), d)
	proof.MulRat(body.defect, body.defect, proof.AddRat(new(big.Rat), big.NewRat(2, 1), d))
	proof.MulRat(body.defect, body.defect, exact.largest)
	body.inertiaLower = exact.floor
	body.rowCeiling = exact.rowCeiling
	body.centerL1 = new(big.Rat)
	for axis := range 3 {
		proof.AddRat(body.centerL1, body.centerL1, magnitude(body.center[axis]))
	}
	body.vPost, ok = quantityRats(post.LinearVelocity)
	if !ok {
		return certBody{}, false
	}
	body.wPost, ok = quantityRats(post.AngularVelocity)
	return body, ok
}

// certMotion reads the part of an island participant that its pre-solve
// relative velocity needs: its role, its pre-solve velocities (the post
// ones set equal to them), and the center its levers run from. It fails
// exactly when newCertBody fails on the same entry and an unchanged post
// velocity: a dynamic body needs an exact mass reading with a positive mass
// interval, inertia intervals with a positive certified floor and a row
// ceiling, a finite pose basis, a mass center reading and exact velocities.
// Everything newCertBody adds to it derives from those.
func (w *World) certMotion(index int, entry BodyState, drive map[int]driverMotion) (certBody, bool) {
	body := certBody{index: index, dynamic: w.bodies[index].definition.Role == Dynamic, pose: entry.Pose}
	zero := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	body.v, body.w, body.vPost, body.wPost = zero, zero, zero, zero
	if motion, ok := drive[index]; ok && w.bodies[index].definition.Role == Kinematic {
		body.kinematic = true
		body.v, body.vPost, body.w, body.wPost = motion.linear, motion.linear, motion.angular, motion.angular
		body.center = zeroIVec()
	}
	if !body.dynamic {
		return body, true
	}
	exact := w.bodies[index].exact
	if exact.mass == nil || exact.bound == nil || exact.bound.Sign() < 0 || exact.interval.Lo.Sign() <= 0 ||
		!exact.tensorOK || exact.floor == nil || exact.floor.Sign() <= 0 || exact.rowCeiling == nil {
		return certBody{}, false
	}
	basis := entry.Pose.Basis()
	if !finite(basis.EX.X, basis.EX.Y, basis.EX.Z, basis.EY.X, basis.EY.Y, basis.EY.Z,
		basis.EZ.X, basis.EZ.Y, basis.EZ.Z) {
		return certBody{}, false
	}
	center, centerError, ok := worldCenterReading(entry.Pose, exact)
	if !ok {
		return certBody{}, false
	}
	for axis := range 3 {
		body.center[axis] = proof.OwnedInterval(proof.SubRat(new(big.Rat), center[axis], centerError[axis]),
			proof.AddRat(new(big.Rat), center[axis], centerError[axis]))
	}
	var valid [2]bool
	body.v, valid[0] = quantityRats(entry.LinearVelocity)
	body.w, valid[1] = quantityRats(entry.AngularVelocity)
	if !valid[0] || !valid[1] {
		return certBody{}, false
	}
	body.vPost, body.wPost = body.v, body.w
	return body, true
}

// pointVelocity encloses v + ω×r for a dynamic body slot (r from its mass
// center) and for a kinematic one (r from the world origin, its driver's
// field); a Fixed body, whose v is zero, stands still. With ω exactly zero
// the cross product is exactly zero and the enclosure is v itself.
func pointVelocity(body certBody, v, omega [3]*big.Rat, lever ivec) ivec {
	if (!body.dynamic && !body.kinematic) || zeroRats(omega) {
		return pointIVec(v)
	}
	return addIVec(pointIVec(v), proof.CrossInterval3(pointIVec(omega), lever))
}

// newCertPoint reads one manifold point. The normal ball is the published
// normal bound plus its angle; each witness ball is its own bound.
func newCertPoint(pair, a, b int, point decad.ContactPoint, bodies []certBody,
	restitution *big.Rat) (certPoint, bool) {
	normalBound, angle := exactBase(point.Normal.Bound), exactBase(point.NormalAngle)
	boundA, boundB := exactBase(point.OnA.Bound), exactBase(point.OnB.Bound)
	if normalBound == nil || angle == nil || boundA == nil || boundB == nil {
		return certPoint{}, false
	}
	normal, okN := ballIVec(point.Normal.Value, proof.AddRat(new(big.Rat), normalBound, angle))
	onA, okA := ballIVec(point.OnA.Value, boundA)
	onB, okB := ballIVec(point.OnB.Value, boundB)
	if !okN || !okA || !okB {
		return certPoint{}, false
	}
	p := certPoint{pair: pair, a: a, b: b, normal: normal, onA: onA, onB: onB, restitution: restitution,
		ballA: boundA, ballB: boundB}
	p.rA, p.rB = zeroIVec(), zeroIVec()
	if bodies[a].dynamic || bodies[a].kinematic {
		p.rA = subIVec(onA, bodies[a].center)
	}
	if bodies[b].dynamic || bodies[b].kinematic {
		p.rB = subIVec(onB, bodies[b].center)
	}
	return p, true
}

// preNormalSpeed encloses the pre-solve relative normal speed at a point.
func preNormalSpeed(p certPoint, bodies []certBody) proof.RatInterval {
	relative := subIVec(pointVelocity(bodies[p.b], bodies[p.b].v, bodies[p.b].w, p.rB),
		pointVelocity(bodies[p.a], bodies[p.a].v, bodies[p.a].w, p.rA))
	return proof.DotInterval3(relative, p.normal)
}

// sharedBody is a certBody written over the certificate's SharedDenom
// (internal/proof/shared_interval.go). Only a dynamic body carries its mass,
// inertia, pose and energy readings, and only a dynamic or kinematic one its
// center.
type sharedBody struct {
	index                                      int
	dynamic, kinematic                         bool
	mass                                       proof.SInterval
	inertia                                    [3][3]proof.SInterval
	rotation                                   [3][3]proof.SRat
	defect, inertiaLower, rowCeiling, centerL1 proof.SRat
	center                                     proof.SIVec3
	v, w, vPost, wPost                         [3]proof.SRat
	// spinUpper is spinEnergyChange's reading, valid when spinOK.
	spinUpper proof.SRat
	spinOK    bool
}

// sharedPoint is a certPoint written over the certificate's SharedDenom.
type sharedPoint struct {
	a, b                                  int
	normal, onA, onB, rA, rB              proof.SIVec3
	ballA, ballB, lambda, mu, restitution proof.SRat
	tangent                               [3]proof.SRat
}

// islandRun is one certificate's evaluation over a SharedDenom: the island's
// bodies and points and the gate limits read over it, the largest value each
// gate attained so far, and the certificate that records refusals. A held
// value becomes a big.Rat only once, when the run publishes it (publish), and
// a refusal's value and limit only for the first refused gate, the one the
// certificate names.
type islandRun struct {
	s                                                          *proof.SharedDenom
	bodies                                                     []sharedBody
	points                                                     []sharedPoint
	cert                                                       islandCertificate
	impulseLimit, velocityLimit, angularLimit, impactSpeed     proof.SRat
	linear, angular, normal, momentum, angularMomentum, energy proof.SRat
	cone, tangent, spin, witnessTorque                         proof.SRat
	// witnessSpin is held as the quotient spinTorque / spinFloor, compared by
	// cross products, so that only the published one is divided.
	spinTorque, spinFloor proof.SRat
}

// certifyIsland runs §6.3's rows over one island.
func (w *World) certifyIsland(bodies []certBody, points []certPoint) islandCertificate {
	return w.readIsland(bodies, points).certify()
}

// readIsland reads every input of an island's certificate over one
// SharedDenom before any row runs: a denominator the first reading cannot
// write over D widens D, and the inputs are read again.
func (w *World) readIsland(bodies []certBody, points []certPoint) *islandRun {
	s := proof.NewSharedDenom(nil)
	for {
		run := &islandRun{s: s}
		w.readShared(run, bodies, points)
		next, widen := s.Widen()
		if !widen {
			return run
		}
		s = next
	}
}

// readShared writes the island's bodies, points and gate limits over the
// run's SharedDenom.
func (w *World) readShared(run *islandRun, bodies []certBody, points []certPoint) {
	s := run.s
	run.impulseLimit, run.velocityLimit = s.Lift(exactBase(w.step.ImpulseResidual)), s.Lift(exactBase(w.step.VelocityResidual))
	run.angularLimit, run.impactSpeed = s.Lift(exactBase(w.step.AngularVelocityResidual)), s.Lift(exactBase(w.step.ImpactSpeed))
	out := make([]sharedBody, len(bodies))
	for slot := range bodies {
		body, read := &bodies[slot], &out[slot]
		read.index, read.dynamic, read.kinematic = body.index, body.dynamic, body.kinematic
		read.v, read.w = liftRats(s, body.v), liftRats(s, body.w)
		read.vPost, read.wPost = liftRats(s, body.vPost), liftRats(s, body.wPost)
		if body.dynamic || body.kinematic {
			read.center = s.LiftIVec3(body.center)
		}
		if !body.dynamic {
			continue
		}
		read.mass = s.LiftInterval(body.mass)
		for i := range 3 {
			read.inertia[i] = s.LiftIVec3(body.inertia[i])
			read.rotation[i] = liftRats(s, body.rotation[i])
		}
		read.defect, read.inertiaLower = s.Lift(body.defect), s.Lift(body.inertiaLower)
		read.rowCeiling, read.centerL1 = s.Lift(body.rowCeiling), s.Lift(body.centerL1)
		spinUpper, ok := spinEnergyChange(&w.bodies[body.index].exact.components, body.rotation, body.w, body.wPost)
		if ok {
			read.spinUpper, read.spinOK = s.Lift(spinUpper), true
		}
	}
	readPoints := make([]sharedPoint, len(points))
	for k := range points {
		p, read := &points[k], &readPoints[k]
		read.a, read.b = p.a, p.b
		read.normal, read.onA, read.onB = s.LiftIVec3(p.normal), s.LiftIVec3(p.onA), s.LiftIVec3(p.onB)
		read.rA, read.rB = s.LiftIVec3(p.rA), s.LiftIVec3(p.rB)
		read.ballA, read.ballB, read.lambda = s.Lift(p.ballA), s.Lift(p.ballB), s.Lift(p.lambda)
		read.mu, read.restitution = s.Lift(p.mu), s.Lift(p.restitution)
		read.tangent = liftRats(s, p.tangent)
	}
	run.bodies, run.points = out, readPoints
}

func liftRats(s *proof.SharedDenom, x [3]*big.Rat) [3]proof.SRat {
	return [3]proof.SRat{s.Lift(x[0]), s.Lift(x[1]), s.Lift(x[2])}
}

// raise keeps the larger of a held maximum and a new attained value.
func (r *islandRun) raise(held *proof.SRat, value proof.SRat) {
	if r.s.Cmp(value, *held) > 0 {
		*held = value
	}
}

// fail records a refused gate, converting its value and limit only when it
// is the first.
func (r *islandRun) fail(gate islandGate, value, limit proof.SRat) {
	if r.cert.failed != 0 {
		r.cert.fail(gate, nil, nil)
		return
	}
	r.cert.fail(gate, r.s.Rat(value), r.s.Rat(limit))
}

// check raises held to a residual and refuses gate when it exceeds limit.
func (r *islandRun) check(gate islandGate, held *proof.SRat, residual, limit proof.SRat) {
	r.raise(held, residual)
	if r.s.Cmp(residual, limit) > 0 {
		r.fail(gate, residual, limit)
	}
}

// checkLaw runs check over each component's |change − applied|, in
// component order.
func (r *islandRun) checkLaw(gate islandGate, held *proof.SRat, change, applied proof.SIVec3, limit proof.SRat) {
	s := r.s
	c0, c1, c2 := change[0], change[1], change[2]
	a0, a1, a2 := applied[0], applied[1], applied[2]
	r.check(gate, held, s.Magnitude(s.SubI(c0, a0)), limit)
	r.check(gate, held, s.Magnitude(s.SubI(c1, a1)), limit)
	r.check(gate, held, s.Magnitude(s.SubI(c2, a2)), limit)
}

// certify runs §6.3's rows over the island readIsland read.
func (r *islandRun) certify() islandCertificate {
	s, bodies, points := r.s, r.bodies, r.points
	r.spinFloor = proof.SInt(1)
	// The island's largest lever bound ρ.
	var rho proof.SRat
	for k := range points {
		p := &points[k]
		if bodies[p.a].dynamic {
			r.raise(&rho, r.euclideanUpper(p.rA))
		}
		if bodies[p.b].dynamic {
			r.raise(&rho, r.euclideanUpper(p.rB))
		}
	}
	// Each point's impulse on B, λ·n + λt; A receives its negation.
	impulses := make([]proof.SIVec3, len(points))
	for k := range points {
		p := &points[k]
		impulses[k] = s.AddI3(s.ScaleI3(p.normal, p.lambda), proof.SPoint3(p.tangent))
	}
	var linearMomentum, angularMomentum proof.SIVec3
	var linearMomentumLimit, angularMomentumLimit, energyUpper, energyAllowance, kinematicWork proof.SRat
	for slot := range bodies {
		body := &bodies[slot]
		if !body.dynamic {
			continue
		}
		r.raise(&r.spin, r.euclideanUpper(proof.SPoint3(body.wPost)))
		dv, dw := r.sub3(body.vPost, body.v), r.sub3(body.wPost, body.w)
		var force, torque proof.SIVec3
		for k := range points {
			p := &points[k]
			switch slot {
			case p.b:
				force = s.AddI3(force, impulses[k])
				torque = s.AddI3(torque, s.CrossI3(p.rB, impulses[k]))
			case p.a:
				force = s.SubI3(force, impulses[k])
				torque = s.SubI3(torque, s.CrossI3(p.rA, impulses[k]))
			}
		}
		// Linear law: m·(v' − v) − ΣJ over the mass interval.
		linearLimit := s.Add(r.impulseLimit, s.Mul(body.mass.Hi, r.velocityLimit))
		dv0, dv1, dv2 := dv[0], dv[1], dv[2]
		momentumChange := proof.SIVec3{s.ScaleI(body.mass, dv0), s.ScaleI(body.mass, dv1), s.ScaleI(body.mass, dv2)}
		r.checkLaw(gateLinearLaw, &r.linear, momentumChange, force, linearLimit)
		// Angular law: I_world·(ω' − ω) − Σ r×J over the inertia and lever
		// intervals. The limit carries the body's witness torque T_β; the
		// mass-center ball, the normal ball and the inertia intervals stay in
		// the residual alone.
		spinChange := r.inertiaApply(body, dw)
		witnessTorque := r.bodyWitnessTorque(slot, points, impulses)
		r.raise(&r.witnessTorque, witnessTorque)
		// T_β / λ_lo above the held quotient, both floors positive.
		if s.Cmp(s.Mul(witnessTorque, r.spinFloor), s.Mul(r.spinTorque, body.inertiaLower)) > 0 {
			r.spinTorque, r.spinFloor = witnessTorque, body.inertiaLower
		}
		angularBodyLimit := s.Add(s.Mul(r.impulseLimit, rho), s.Mul(body.inertiaLower, r.angularLimit))
		angularBodyLimit = s.Add(angularBodyLimit, witnessTorque)
		r.checkLaw(gateAngularLaw, &r.angular, spinChange, torque, angularBodyLimit)
		// Island momentum about the world origin: Σ m·Δv and
		// Σ (I·Δω + c×m·Δv), against the impulses delivered by Fixed bodies.
		// The angular limit sums each body's angular-law limit, T_β included.
		linearMomentum = s.AddI3(linearMomentum, momentumChange)
		angularMomentum = s.AddI3(angularMomentum, s.AddI3(spinChange, s.CrossI3(body.center, momentumChange)))
		linearMomentumLimit = s.Add(linearMomentumLimit, linearLimit)
		angularMomentumLimit = s.Add(angularMomentumLimit, angularBodyLimit)
		angularMomentumLimit = s.Add(angularMomentumLimit, s.Mul(body.centerL1, linearLimit))
		// Energy: the squared-speed difference with the mass endpoint that
		// maximises it, the rotational change over the inertia intervals, and
		// the defect widening of both rotational readings.
		var squaredChange proof.SRat
		spinWeight := s.Mul(proof.SInt(3), s.Mul(body.rowCeiling, r.angularLimit))
		for axis := range 3 {
			v, vPost := body.v[axis], body.vPost[axis]
			squaredChange = s.Add(squaredChange, s.Sub(s.Mul(vPost, vPost), s.Mul(v, v)))
			speeds := s.Add(s.Add(proof.SAbs(v), proof.SAbs(vPost)), r.velocityLimit)
			energyAllowance = s.Add(energyAllowance, s.Mul(linearLimit, speeds))
			spins := s.Add(s.Add(proof.SAbs(body.w[axis]), proof.SAbs(body.wPost[axis])), r.angularLimit)
			energyAllowance = s.Add(energyAllowance, s.Mul(spinWeight, spins))
		}
		if squaredChange.Sign() > 0 {
			energyUpper = s.Add(energyUpper, s.Mul(body.mass.Hi, squaredChange))
		} else {
			energyUpper = s.Add(energyUpper, s.Mul(body.mass.Lo, squaredChange))
		}
		if !body.spinOK {
			r.fail(gateEnergy, proof.SRat{}, proof.SRat{})
			continue
		}
		energyUpper = s.Add(energyUpper, body.spinUpper)
		for _, omega := range [2]*[3]proof.SRat{&body.w, &body.wPost} {
			l1 := r.l1(omega)
			energyUpper = s.Add(energyUpper, s.Mul(body.defect, s.Mul(l1, l1)))
		}
	}
	for k := range points {
		p := &points[k]
		// Normal sign: λn ≥ 0 exactly.
		if p.lambda.Sign() < 0 {
			r.fail(gateNormalSign, proof.SNeg(p.lambda), proof.SRat{})
		}
		// Restitution target from the enclosed pre-solve normal speed.
		c := r.preNormalSpeed(p, bodies)
		e, ok := r.restitutionTarget(c, p.restitution)
		if !ok {
			r.fail(gateRestitutionTarget, s.Magnitude(c), r.impactSpeed)
			continue
		}
		// w'·n − target = ((v' + e·v) + (ω' + e·ω)×r)_B−A · n.
		q := s.DotI3(s.SubI3(r.combined(&bodies[p.b], e, p.rB), r.combined(&bodies[p.a], e, p.rA)), p.normal)
		if below := proof.SNeg(q.Lo); s.Cmp(below, r.velocityLimit) > 0 {
			r.fail(gateNonPenetration, below, r.velocityLimit)
		}
		if p.lambda.Sign() > 0 {
			r.check(gateComplementarity, &r.normal, s.Magnitude(q), r.velocityLimit)
		} else if q.Lo.Sign() < 0 {
			r.raise(&r.normal, proof.SNeg(q.Lo))
		}
		r.certifyFriction(p, bodies)
		// Kinematic work: a driver on side A delivers J·V to the island, one
		// on side B −J·V, with J = λ·n + λt the impulse on B and V the
		// driver's field at the contact point; the energy gate admits its
		// upper end.
		if body := &bodies[p.a]; body.kinematic {
			work := s.DotI3(impulses[k], r.pointVelocity(body, &body.v, &body.w, p.rA))
			kinematicWork = s.Add(kinematicWork, work.Hi)
		}
		if body := &bodies[p.b]; body.kinematic {
			work := proof.SNegI(s.DotI3(impulses[k], r.pointVelocity(body, &body.v, &body.w, p.rB)))
			kinematicWork = s.Add(kinematicWork, work.Hi)
		}
		// External impulses: those a Fixed or Kinematic body delivers. A
		// dynamic pair's two sides read the same λ and normal interval and
		// cancel exactly in the linear sum; their torques about the origin use
		// each side's own witness and so stay in the angular sum. Side A
		// receives −J, side B J, and each is subtracted: subtracting the
		// interval −X adds X endpoint for endpoint.
		if bodies[p.a].dynamic {
			angularMomentum = s.AddI3(angularMomentum, s.CrossI3(p.onA, impulses[k]))
			if !bodies[p.b].dynamic {
				linearMomentum = s.AddI3(linearMomentum, impulses[k])
			}
		}
		if bodies[p.b].dynamic {
			angularMomentum = s.SubI3(angularMomentum, s.CrossI3(p.onB, impulses[k]))
			if !bodies[p.a].dynamic {
				linearMomentum = s.SubI3(linearMomentum, impulses[k])
			}
		}
	}
	r.energy = proof.SShift(energyUpper, -1)
	// The island's kinetic energy may grow by the work its drivers deliver.
	energyAllowance = s.Add(energyAllowance, kinematicWork)
	if s.Cmp(r.energy, energyAllowance) > 0 {
		r.fail(gateEnergy, r.energy, energyAllowance)
	}
	r.checkMomentum(linearMomentum, angularMomentum, linearMomentumLimit, angularMomentumLimit)
	return r.publish()
}

// checkMomentum runs the island's momentum rows, per axis the linear row
// before the angular one.
func (r *islandRun) checkMomentum(linear, angular proof.SIVec3, linearLimit, angularLimit proof.SRat) {
	s := r.s
	l0, l1, l2 := linear[0], linear[1], linear[2]
	a0, a1, a2 := angular[0], angular[1], angular[2]
	r.check(gateLinearMomentum, &r.momentum, s.Magnitude(l0), linearLimit)
	r.check(gateAngularMomentum, &r.angularMomentum, s.Magnitude(a0), angularLimit)
	r.check(gateLinearMomentum, &r.momentum, s.Magnitude(l1), linearLimit)
	r.check(gateAngularMomentum, &r.angularMomentum, s.Magnitude(a1), angularLimit)
	r.check(gateLinearMomentum, &r.momentum, s.Magnitude(l2), linearLimit)
	r.check(gateAngularMomentum, &r.angularMomentum, s.Magnitude(a2), angularLimit)
}

// publish converts the held maxima into the certificate's rationals.
func (r *islandRun) publish() islandCertificate {
	s, cert := r.s, r.cert
	cert.linear, cert.angular, cert.normal = s.Rat(r.linear), s.Rat(r.angular), s.Rat(r.normal)
	cert.energy, cert.momentum, cert.angularMomentum = s.Rat(r.energy), s.Rat(r.momentum), s.Rat(r.angularMomentum)
	cert.cone, cert.tangent, cert.spin = s.Rat(r.cone), s.Rat(r.tangent), s.Rat(r.spin)
	cert.witnessTorque = s.Rat(r.witnessTorque)
	cert.witnessSpin = s.Rat(r.spinTorque)
	if cert.witnessSpin.Sign() != 0 {
		cert.witnessSpin.Quo(cert.witnessSpin, s.Rat(r.spinFloor))
	}
	return cert
}

// sub3 is the exact componentwise a − b.
func (r *islandRun) sub3(a, b [3]proof.SRat) [3]proof.SRat {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return [3]proof.SRat{r.s.Sub(a0, b0), r.s.Sub(a1, b1), r.s.Sub(a2, b2)}
}

// l1 is the exact |x|_1.
func (r *islandRun) l1(x *[3]proof.SRat) proof.SRat {
	s := r.s
	return s.Add(s.Add(proof.SAbs(x[0]), proof.SAbs(x[1])), proof.SAbs(x[2]))
}

// inertiaApply encloses I_world·x for an exact vector x: x is taken into
// the local frame with the exact pose basis, multiplied by the local tensor
// intervals, returned to world axes, and widened by the defect bound times
// |x|_1.
func (r *islandRun) inertiaApply(b *sharedBody, x [3]proof.SRat) proof.SIVec3 {
	s := r.s
	x0, x1, x2 := x[0], x[1], x[2]
	column := func(j int) proof.SRat {
		sum := s.Mul(b.rotation[0][j], x0)
		sum = s.Add(sum, s.Mul(b.rotation[1][j], x1))
		return s.Add(sum, s.Mul(b.rotation[2][j], x2))
	}
	l0, l1, l2 := column(0), column(1), column(2)
	tensorRow := func(row *[3]proof.SInterval) proof.SInterval {
		y := s.ScaleI(row[0], l0)
		y = s.AddI(y, s.ScaleI(row[1], l1))
		return s.AddI(y, s.ScaleI(row[2], l2))
	}
	y0, y1, y2 := tensorRow(&b.inertia[0]), tensorRow(&b.inertia[1]), tensorRow(&b.inertia[2])
	widen := s.Mul(r.l1(&x), b.defect)
	worldRow := func(rotation *[3]proof.SRat) proof.SInterval {
		out := s.ScaleI(y0, rotation[0])
		out = s.AddI(out, s.ScaleI(y1, rotation[1]))
		out = s.AddI(out, s.ScaleI(y2, rotation[2]))
		return proof.SInterval{Lo: s.Sub(out.Lo, widen), Hi: s.Add(out.Hi, widen)}
	}
	return proof.SIVec3{worldRow(&b.rotation[0]), worldRow(&b.rotation[1]), worldRow(&b.rotation[2])}
}

// pointVelocity encloses v + ω×r for a dynamic body slot (r from its mass
// center) and for a kinematic one (r from the world origin, its driver's
// field); a Fixed body, whose v is zero, stands still. With ω exactly zero
// the cross product is exactly zero and the enclosure is v itself.
func (r *islandRun) pointVelocity(body *sharedBody, v, omega *[3]proof.SRat, lever proof.SIVec3) proof.SIVec3 {
	if (!body.dynamic && !body.kinematic) || omega[0].Sign() == 0 && omega[1].Sign() == 0 && omega[2].Sign() == 0 {
		return proof.SPoint3(*v)
	}
	return r.s.AddI3(proof.SPoint3(*v), r.s.PointCrossI3(*omega, lever))
}

// preNormalSpeed encloses the pre-solve relative normal speed at a point.
func (r *islandRun) preNormalSpeed(p *sharedPoint, bodies []sharedBody) proof.SInterval {
	a, b := &bodies[p.a], &bodies[p.b]
	relative := r.s.SubI3(r.pointVelocity(b, &b.v, &b.w, p.rB), r.pointVelocity(a, &a.v, &a.w, p.rA))
	return r.s.DotI3(relative, p.normal)
}

// combined encloses (v' + e·v) + (ω' + e·ω)×r for one side of a point.
func (r *islandRun) combined(body *sharedBody, e proof.SRat, lever proof.SIVec3) proof.SIVec3 {
	u, omega := r.affine(&body.vPost, e, &body.v), r.affine(&body.wPost, e, &body.w)
	return r.pointVelocity(body, &u, &omega, lever)
}

// affine is the exact componentwise a + e·b.
func (r *islandRun) affine(a *[3]proof.SRat, e proof.SRat, b *[3]proof.SRat) [3]proof.SRat {
	s := r.s
	return [3]proof.SRat{s.Add(a[0], s.Mul(e, b[0])), s.Add(a[1], s.Mul(e, b[1])), s.Add(a[2], s.Mul(e, b[2]))}
}

// restitutionTarget selects the point's effective coefficient from its
// enclosed pre-solve normal speed c: e when c lies wholly below
// −ImpactSpeed, zero when it lies wholly at or above it. A straddling
// interval with positive e cannot select a target.
func (r *islandRun) restitutionTarget(c proof.SInterval, e proof.SRat) (proof.SRat, bool) {
	threshold := proof.SNeg(r.impactSpeed)
	switch {
	case e.Sign() == 0 || r.s.Cmp(c.Lo, threshold) >= 0:
		return proof.SRat{}, true
	case r.s.Cmp(c.Hi, threshold) < 0:
		return e, true
	default:
		return proof.SRat{}, false
	}
}

// bodyWitnessTorque is §6.3's witness torque T_β = Σ_k b_k·|J_k|_1 of the
// body in slot: over the body's points, b_k is the body's own witness ball
// (OnA.Bound on side A, OnB.Bound on side B) and |J_k|_1 the L1 norm of the
// point's impulse interval at its upper end. A contact point is known only to
// the ball the caller's PointResolution admitted, so the published impulses,
// applied anywhere within those balls, can differ from their published torque
// by up to T_β per component.
func (r *islandRun) bodyWitnessTorque(slot int, points []sharedPoint, impulses []proof.SIVec3) proof.SRat {
	s := r.s
	var torque proof.SRat
	for k := range points {
		p := &points[k]
		var ball proof.SRat
		switch slot {
		case p.b:
			ball = p.ballB
		case p.a:
			ball = p.ballA
		default:
			continue
		}
		impulse := &impulses[k]
		l1 := s.Add(s.Add(s.Magnitude(impulse[0]), s.Magnitude(impulse[1])), s.Magnitude(impulse[2]))
		torque = s.Add(torque, s.Mul(ball, l1))
	}
	return torque
}

// certifyFriction runs the cone, stick and slip rows at one point. With
// s = ‖λt‖² exact: the cone requires ‖λt‖ ≤ μ_lo·λn + ImpulseResidual,
// compared squared; a point strictly inside the cone by more than
// ImpulseResidual sticks, and its post-solve tangent speed ‖w'_t‖ may not
// exceed VelocityResidual; every other point slips, and its impulse must
// oppose its tangent velocity: λt·w'_t + ‖λt‖·‖w'_t‖ at its upper end is at
// most ImpulseResidual·‖w'_t‖ + VelocityResidual·‖λt‖ at their lower ends.
// w'_t = w' − (w'·n)·n is enclosed over the normal ball and the levers.
func (r *islandRun) certifyFriction(p *sharedPoint, bodies []sharedBody) {
	s := r.s
	t0, t1, t2 := p.tangent[0], p.tangent[1], p.tangent[2]
	square := s.Add(s.Add(s.Mul(t0, t0), s.Mul(t1, t1)), s.Mul(t2, t2))
	coneLower := s.Mul(p.mu, p.lambda)
	allowed := s.Add(coneLower, r.impulseLimit)
	norm := r.sqrtUpper(square)
	if excess := s.Sub(norm, coneLower); excess.Sign() > 0 {
		r.raise(&r.cone, excess)
	}
	if allowed.Sign() < 0 || s.Cmp(square, s.Mul(allowed, allowed)) > 0 {
		r.fail(gateCone, s.Sub(norm, coneLower), r.impulseLimit)
	}
	a, b := &bodies[p.a], &bodies[p.b]
	relative := s.SubI3(r.pointVelocity(b, &b.vPost, &b.wPost, p.rB), r.pointVelocity(a, &a.vPost, &a.wPost, p.rA))
	normalSpeed := s.DotI3(relative, p.normal)
	r0, r1, r2 := relative[0], relative[1], relative[2]
	n0, n1, n2 := p.normal[0], p.normal[1], p.normal[2]
	slide := proof.SIVec3{
		s.SubI(r0, s.MulI(normalSpeed, n0)),
		s.SubI(r1, s.MulI(normalSpeed, n1)),
		s.SubI(r2, s.MulI(normalSpeed, n2)),
	}
	speedUpper := r.euclideanUpper(slide)
	threshold := s.Sub(coneLower, r.impulseLimit)
	if threshold.Sign() > 0 && s.Cmp(square, s.Mul(threshold, threshold)) < 0 {
		r.check(gateStick, &r.tangent, speedUpper, r.velocityLimit)
		return
	}
	opposed := s.Add(s.DotI3(proof.SPoint3(p.tangent), slide).Hi, s.Mul(norm, speedUpper))
	limit := s.Add(s.Mul(r.impulseLimit, r.euclideanLower(slide)), s.Mul(r.velocityLimit, r.sqrtLower(square)))
	if s.Cmp(opposed, limit) > 0 {
		r.fail(gateSlip, opposed, limit)
	}
}

// euclideanUpper bounds the Euclidean length of every vector in an interval
// box from above: the square root of the summed squared magnitudes, taken
// in float64 and raised until its exact square covers the sum.
func (r *islandRun) euclideanUpper(v proof.SIVec3) proof.SRat {
	s := r.s
	v0, v1, v2 := v[0], v[1], v[2]
	m0, m1, m2 := s.Magnitude(v0), s.Magnitude(v1), s.Magnitude(v2)
	sum := s.Add(s.Add(s.Mul(m0, m0), s.Mul(m1, m1)), s.Mul(m2, m2))
	approx, _ := s.Float64(sum)
	root := math.Sqrt(approx)
	for {
		if x, ok := proof.SFloat(root); ok && s.Cmp(s.Mul(x, x), sum) >= 0 {
			return x
		}
		root = math.Nextafter(root, math.Inf(1))
	}
}

// euclideanLower bounds the Euclidean length of every vector in an interval
// box from below: each component contributes its smallest magnitude, zero
// when its interval straddles zero.
func (r *islandRun) euclideanLower(v proof.SIVec3) proof.SRat {
	s := r.s
	v0, v1, v2 := v[0], v[1], v[2]
	sum := s.Add(s.Add(leastSquare(s, v0), leastSquare(s, v1)), leastSquare(s, v2))
	return r.sqrtLower(sum)
}

// leastSquare is the square of the smallest magnitude an interval admits.
func leastSquare(s *proof.SharedDenom, iv proof.SInterval) proof.SRat {
	switch {
	case iv.Lo.Sign() > 0:
		return s.Mul(iv.Lo, iv.Lo)
	case iv.Hi.Sign() < 0:
		return s.Mul(iv.Hi, iv.Hi)
	default:
		return proof.SRat{}
	}
}

// sqrtUpper and sqrtLower bound the square root of a nonnegative rational
// from above and below by floats whose exact squares bracket it.
func (r *islandRun) sqrtUpper(square proof.SRat) proof.SRat {
	s := r.s
	if square.Sign() <= 0 {
		return proof.SRat{}
	}
	approx, _ := s.Float64(square)
	root := math.Sqrt(approx)
	switch {
	case !finite(root):
		// sqrt(s) ≤ s + 1 for every s ≥ 0.
		return s.Add(square, proof.SInt(1))
	case approx == 0:
		// s rounds to zero only below 2^-1074, whose root lies below 2^-537.
		x, _ := proof.SFloat(0x1p-537)
		return x
	}
	for {
		x, _ := proof.SFloat(root)
		if s.Cmp(s.Mul(x, x), square) >= 0 {
			return x
		}
		root = math.Nextafter(root, math.Inf(1))
	}
}

func (r *islandRun) sqrtLower(square proof.SRat) proof.SRat {
	s := r.s
	if square.Sign() <= 0 {
		return proof.SRat{}
	}
	approx, _ := s.Float64(square)
	root := math.Sqrt(approx)
	for root > 0 {
		x, _ := proof.SFloat(root)
		if s.Cmp(s.Mul(x, x), square) <= 0 {
			return x
		}
		root = math.Nextafter(root, 0)
	}
	x, _ := proof.SFloat(root)
	return x
}

// solverReport converts a passing certificate into the published residuals.
func solverReport(cert islandCertificate, penetration *big.Rat, iterations int) (ContactSolverReport, error) {
	up := func(x *big.Rat) (float64, error) {
		f, _ := x.Float64()
		if !finite(f) {
			return 0, fmt.Errorf("%w: island residual is not finite", ErrUnsupported)
		}
		if ratFloat(f).Cmp(x) < 0 {
			f = math.Nextafter(f, math.Inf(1))
		}
		return f, nil
	}
	var out ContactSolverReport
	var err error
	readings := []struct {
		value *big.Rat
		set   func(float64)
	}{
		{cert.normal, func(f float64) { out.NormalResidual = units.MillimetersPerSecond(f) }},
		{penetration, func(f float64) { out.PenetrationResidual = units.Millimeters(f) }},
		{cert.linear, func(f float64) { out.LinearResidual = units.KilogramMillimetersPerSecond(f) }},
		{cert.angular, func(f float64) {
			out.AngularResidual = units.New(f, units.KilogramSquareMillimeterPerSecond)
		}},
		{cert.energy, func(f float64) {
			out.EnergyResidual = units.New(f, units.KilogramSquareMillimeterPerSecondSquared)
		}},
		{cert.momentum, func(f float64) { out.MomentumResidual = units.KilogramMillimetersPerSecond(f) }},
		{cert.angularMomentum, func(f float64) {
			out.AngularMomentumResidual = units.New(f, units.KilogramSquareMillimeterPerSecond)
		}},
	}
	for _, reading := range readings {
		f, convErr := up(reading.value)
		if convErr != nil {
			err = convErr
		}
		reading.set(f)
	}
	tangent, tangentErr := up(cert.tangent)
	cone, coneErr := up(cert.cone)
	spin, spinErr := up(cert.spin)
	witnessTorque, torqueErr := up(cert.witnessTorque)
	witnessSpin, witnessErr := up(cert.witnessSpin)
	for _, readErr := range []error{tangentErr, coneErr, spinErr, torqueErr, witnessErr} {
		if readErr != nil {
			err = readErr
		}
	}
	out.TangentResidual = units.MillimetersPerSecond(tangent)
	out.ConeResidual = units.KilogramMillimetersPerSecond(cone)
	out.AngularUpper = units.RadiansPerSecond(spin)
	out.WitnessTorque = units.New(witnessTorque, units.KilogramSquareMillimeterPerSecond)
	out.WitnessSpin = units.RadiansPerSecond(witnessSpin)
	out.Iterations = iterations
	return out, err
}
