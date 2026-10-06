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

// raise keeps the larger of a held maximum and a new attained value.
func raise(held **big.Rat, value *big.Rat) {
	if *held == nil || value.Cmp(*held) > 0 {
		*held = new(big.Rat).Set(value)
	}
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

func scaleIVec(a ivec, scale *big.Rat) ivec {
	var out ivec
	for axis := range out {
		out[axis] = proof.ScaleInterval(a[axis], scale)
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

// euclideanUpper bounds the Euclidean length of every vector in an interval
// box from above: the square root of the summed squared magnitudes, taken
// in float64 and raised until its exact square covers the sum.
func euclideanUpper(v ivec) *big.Rat {
	sum := new(big.Rat)
	for axis := range v {
		m := magnitude(v[axis])
		proof.AddRat(sum, sum, proof.MulRat(new(big.Rat), m, m))
	}
	approx, _ := sum.Float64()
	root := math.Sqrt(approx)
	for {
		r := ratFloat(root)
		if r != nil && proof.MulRat(new(big.Rat), r, r).Cmp(sum) >= 0 {
			return r
		}
		root = math.Nextafter(root, math.Inf(1))
	}
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

// inertiaApply encloses I_world·x for an exact vector x: x is taken into
// the local frame with the exact pose basis, multiplied by the local tensor
// intervals, returned to world axes, and widened by the defect bound times
// |x|_1.
func (b certBody) inertiaApply(x [3]*big.Rat) ivec {
	var local [3]*big.Rat
	for j := range local {
		local[j] = new(big.Rat)
		for i := range 3 {
			proof.AddRat(local[j], local[j], proof.MulRat(new(big.Rat), b.rotation[i][j], x[i]))
		}
	}
	var y ivec
	for i := range y {
		y[i] = proof.ScaleInterval(b.inertia[i][0], local[0])
		for j := 1; j < 3; j++ {
			y[i] = proof.AddInterval(y[i], proof.ScaleInterval(b.inertia[i][j], local[j]))
		}
	}
	widen := new(big.Rat)
	for _, value := range x {
		proof.AddRat(widen, widen, absRat(new(big.Rat).Set(value)))
	}
	proof.MulRat(widen, widen, b.defect)
	var out ivec
	for row := range out {
		out[row] = proof.ScaleInterval(y[0], b.rotation[row][0])
		for i := 1; i < 3; i++ {
			out[row] = proof.AddInterval(out[row], proof.ScaleInterval(y[i], b.rotation[row][i]))
		}
		out[row] = proof.OwnedInterval(proof.SubRat(new(big.Rat), out[row].Lo, widen), proof.AddRat(new(big.Rat), out[row].Hi, widen))
	}
	return out
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

// restitutionTarget selects the point's effective coefficient from its
// enclosed pre-solve normal speed c: e when c lies wholly below
// −ImpactSpeed, zero when it lies wholly at or above it. A straddling
// interval with positive e cannot select a target.
func restitutionTarget(c proof.RatInterval, e, impactSpeed *big.Rat) (*big.Rat, bool) {
	threshold := new(big.Rat).Neg(impactSpeed)
	switch {
	case e.Sign() == 0 || c.Lo.Cmp(threshold) >= 0:
		return new(big.Rat), true
	case c.Hi.Cmp(threshold) < 0:
		return e, true
	default:
		return nil, false
	}
}

// certifyIsland runs §6.3's rows over one island.
func (w *World) certifyIsland(bodies []certBody, points []certPoint) islandCertificate {
	cert := islandCertificate{linear: new(big.Rat), angular: new(big.Rat), normal: new(big.Rat),
		energy: new(big.Rat), momentum: new(big.Rat), angularMomentum: new(big.Rat),
		cone: new(big.Rat), tangent: new(big.Rat), spin: new(big.Rat),
		witnessTorque: new(big.Rat), witnessSpin: new(big.Rat)}
	impulseLimit, velocityLimit := exactBase(w.step.ImpulseResidual), exactBase(w.step.VelocityResidual)
	angularLimit, impactSpeed := exactBase(w.step.AngularVelocityResidual), exactBase(w.step.ImpactSpeed)
	// The island's largest lever bound ρ.
	rho := new(big.Rat)
	for _, p := range points {
		if bodies[p.a].dynamic {
			raise(&rho, euclideanUpper(p.rA))
		}
		if bodies[p.b].dynamic {
			raise(&rho, euclideanUpper(p.rB))
		}
	}
	// Each point's impulse on B, λ·n + λt; A receives its negation.
	impulses := make([]ivec, len(points))
	for k, p := range points {
		impulses[k] = addIVec(scaleIVec(p.normal, p.lambda), pointIVec(p.tangent))
	}
	linearMomentum, angularMomentum := zeroIVec(), zeroIVec()
	linearMomentumLimit, angularMomentumLimit := new(big.Rat), new(big.Rat)
	energyUpper, energyAllowance, kinematicWork := new(big.Rat), new(big.Rat), new(big.Rat)
	for slot, body := range bodies {
		if !body.dynamic {
			continue
		}
		raise(&cert.spin, euclideanUpper(pointIVec(body.wPost)))
		var dv, dw [3]*big.Rat
		for axis := range 3 {
			dv[axis] = proof.SubRat(new(big.Rat), body.vPost[axis], body.v[axis])
			dw[axis] = proof.SubRat(new(big.Rat), body.wPost[axis], body.w[axis])
		}
		force, torque := zeroIVec(), zeroIVec()
		for k, p := range points {
			switch slot {
			case p.b:
				force = addIVec(force, impulses[k])
				torque = addIVec(torque, proof.CrossInterval3(p.rB, impulses[k]))
			case p.a:
				force = subIVec(force, impulses[k])
				torque = subIVec(torque, proof.CrossInterval3(p.rA, impulses[k]))
			}
		}
		// Linear law: m·(v' − v) − ΣJ over the mass interval.
		linearLimit := proof.AddRat(new(big.Rat), impulseLimit, proof.MulRat(new(big.Rat), body.mass.Hi, velocityLimit))
		momentumChange := ivec{}
		for axis := range 3 {
			momentumChange[axis] = proof.MulInterval(body.mass, proof.PointInterval(dv[axis]))
			residual := magnitude(proof.SubInterval(momentumChange[axis], force[axis]))
			raise(&cert.linear, residual)
			if residual.Cmp(linearLimit) > 0 {
				cert.fail(gateLinearLaw, residual, linearLimit)
			}
		}
		// Angular law: I_world·(ω' − ω) − Σ r×J over the inertia and lever
		// intervals. The limit carries the body's witness torque T_β; the
		// mass-center ball, the normal ball and the inertia intervals stay in
		// the residual alone.
		spinChange := body.inertiaApply(dw)
		witnessTorque := bodyWitnessTorque(slot, points, impulses)
		raise(&cert.witnessTorque, witnessTorque)
		raise(&cert.witnessSpin, new(big.Rat).Quo(witnessTorque, body.inertiaLower))
		angularBodyLimit := proof.AddRat(new(big.Rat), proof.MulRat(new(big.Rat), impulseLimit, rho),
			proof.MulRat(new(big.Rat), body.inertiaLower, angularLimit))
		proof.AddRat(angularBodyLimit, angularBodyLimit, witnessTorque)
		for axis := range 3 {
			residual := magnitude(proof.SubInterval(spinChange[axis], torque[axis]))
			raise(&cert.angular, residual)
			if residual.Cmp(angularBodyLimit) > 0 {
				cert.fail(gateAngularLaw, residual, angularBodyLimit)
			}
		}
		// Island momentum about the world origin: Σ m·Δv and
		// Σ (I·Δω + c×m·Δv), against the impulses delivered by Fixed bodies.
		// The angular limit sums each body's angular-law limit, T_β included.
		linearMomentum = addIVec(linearMomentum, momentumChange)
		angularMomentum = addIVec(angularMomentum,
			addIVec(spinChange, proof.CrossInterval3(body.center, momentumChange)))
		proof.AddRat(linearMomentumLimit, linearMomentumLimit, linearLimit)
		proof.AddRat(angularMomentumLimit, angularMomentumLimit, angularBodyLimit)
		proof.AddRat(angularMomentumLimit, angularMomentumLimit, proof.MulRat(new(big.Rat), body.centerL1, linearLimit))
		// Energy: the squared-speed difference with the mass endpoint that
		// maximises it, the rotational change over the inertia intervals, and
		// the defect widening of both rotational readings.
		squaredChange := new(big.Rat)
		for axis := range 3 {
			proof.AddRat(squaredChange, squaredChange, proof.SubRat(new(big.Rat), proof.MulRat(new(big.Rat), body.vPost[axis], body.vPost[axis]),
				proof.MulRat(new(big.Rat), body.v[axis], body.v[axis])))
			speeds := proof.AddRat(new(big.Rat), absRat(new(big.Rat).Set(body.v[axis])),
				absRat(new(big.Rat).Set(body.vPost[axis])))
			proof.AddRat(speeds, speeds, velocityLimit)
			proof.AddRat(energyAllowance, energyAllowance, proof.MulRat(new(big.Rat), linearLimit, speeds))
			spins := proof.AddRat(new(big.Rat), absRat(new(big.Rat).Set(body.w[axis])),
				absRat(new(big.Rat).Set(body.wPost[axis])))
			proof.AddRat(spins, spins, angularLimit)
			proof.AddRat(energyAllowance, energyAllowance, proof.MulRat(new(big.Rat),
				proof.MulRat(new(big.Rat), big.NewRat(3, 1), proof.MulRat(new(big.Rat), body.rowCeiling, angularLimit)), spins))
		}
		if squaredChange.Sign() > 0 {
			proof.AddRat(energyUpper, energyUpper, proof.MulRat(new(big.Rat), body.mass.Hi, squaredChange))
		} else {
			proof.AddRat(energyUpper, energyUpper, proof.MulRat(new(big.Rat), body.mass.Lo, squaredChange))
		}
		spinUpper, ok := spinEnergyChange(&w.bodies[body.index].exact.components, body.rotation, body.w, body.wPost)
		if !ok {
			cert.fail(gateEnergy, new(big.Rat), new(big.Rat))
			continue
		}
		proof.AddRat(energyUpper, energyUpper, spinUpper)
		for _, omega := range [2][3]*big.Rat{body.w, body.wPost} {
			l1 := new(big.Rat)
			for _, value := range omega {
				proof.AddRat(l1, l1, absRat(new(big.Rat).Set(value)))
			}
			proof.AddRat(energyUpper, energyUpper, proof.MulRat(new(big.Rat), body.defect, proof.MulRat(new(big.Rat), l1, l1)))
		}
	}
	for k, p := range points {
		// Normal sign: λn ≥ 0 exactly.
		if p.lambda.Sign() < 0 {
			cert.fail(gateNormalSign, new(big.Rat).Neg(p.lambda), new(big.Rat))
		}
		// Restitution target from the enclosed pre-solve normal speed.
		c := preNormalSpeed(p, bodies)
		e, ok := restitutionTarget(c, p.restitution, impactSpeed)
		if !ok {
			cert.fail(gateRestitutionTarget, magnitude(c), impactSpeed)
			continue
		}
		// w'·n − target = ((v' + e·v) + (ω' + e·ω)×r)_B−A · n.
		combined := func(body certBody, r ivec) ivec {
			var u, omega [3]*big.Rat
			for axis := range 3 {
				u[axis] = proof.AddRat(new(big.Rat), body.vPost[axis], proof.MulRat(new(big.Rat), e, body.v[axis]))
				omega[axis] = proof.AddRat(new(big.Rat), body.wPost[axis], proof.MulRat(new(big.Rat), e, body.w[axis]))
			}
			return pointVelocity(body, u, omega, r)
		}
		q := proof.DotInterval3(subIVec(combined(bodies[p.b], p.rB), combined(bodies[p.a], p.rA)), p.normal)
		if below := new(big.Rat).Neg(q.Lo); below.Cmp(velocityLimit) > 0 {
			cert.fail(gateNonPenetration, below, velocityLimit)
		}
		if p.lambda.Sign() > 0 {
			residual := magnitude(q)
			raise(&cert.normal, residual)
			if residual.Cmp(velocityLimit) > 0 {
				cert.fail(gateComplementarity, residual, velocityLimit)
			}
		} else if q.Lo.Sign() < 0 {
			raise(&cert.normal, new(big.Rat).Neg(q.Lo))
		}
		w.certifyFriction(&cert, p, bodies, impulseLimit, velocityLimit)
		// Kinematic work: a driver on side A delivers J·V to the island, one
		// on side B −J·V, with J = λ·n + λt the impulse on B and V the
		// driver's field at the contact point; the energy gate admits its
		// upper end.
		for side, slot := range [2]int{p.a, p.b} {
			if !bodies[slot].kinematic {
				continue
			}
			lever := p.rA
			if side == 1 {
				lever = p.rB
			}
			field := pointVelocity(bodies[slot], bodies[slot].v, bodies[slot].w, lever)
			work := proof.DotInterval3(impulses[k], field)
			if side == 1 {
				work = proof.NegInterval(work)
			}
			proof.AddRat(kinematicWork, kinematicWork, work.Hi)
		}
		// External impulses: those a Fixed or Kinematic body delivers. A dynamic pair's
		// two sides read the same λ and normal interval and cancel exactly in
		// the linear sum; their torques about the origin use each side's own
		// witness and so stay in the angular sum.
		for side, slot := range [2]int{p.a, p.b} {
			if !bodies[slot].dynamic {
				continue
			}
			witness, sign := p.onA, big.NewRat(-1, 1)
			if side == 1 {
				witness, sign = p.onB, big.NewRat(1, 1)
			}
			angularMomentum = subIVec(angularMomentum, scaleIVec(proof.CrossInterval3(witness, impulses[k]), sign))
			other := p.b
			if side == 1 {
				other = p.a
			}
			if !bodies[other].dynamic {
				linearMomentum = subIVec(linearMomentum, scaleIVec(impulses[k], sign))
			}
		}
	}
	energyUpper.Quo(energyUpper, big.NewRat(2, 1))
	cert.energy = energyUpper
	// The island's kinetic energy may grow by the work its drivers deliver.
	proof.AddRat(energyAllowance, energyAllowance, kinematicWork)
	if energyUpper.Cmp(energyAllowance) > 0 {
		cert.fail(gateEnergy, energyUpper, energyAllowance)
	}
	for axis := range 3 {
		residual := magnitude(linearMomentum[axis])
		raise(&cert.momentum, residual)
		if residual.Cmp(linearMomentumLimit) > 0 {
			cert.fail(gateLinearMomentum, residual, linearMomentumLimit)
		}
		residual = magnitude(angularMomentum[axis])
		raise(&cert.angularMomentum, residual)
		if residual.Cmp(angularMomentumLimit) > 0 {
			cert.fail(gateAngularMomentum, residual, angularMomentumLimit)
		}
	}
	return cert
}

// bodyWitnessTorque is §6.3's witness torque T_β = Σ_k b_k·|J_k|_1 of the
// body in slot: over the body's points, b_k is the body's own witness ball
// (OnA.Bound on side A, OnB.Bound on side B) and |J_k|_1 the L1 norm of the
// point's impulse interval at its upper end. A contact point is known only to
// the ball the caller's PointResolution admitted, so the published impulses,
// applied anywhere within those balls, can differ from their published torque
// by up to T_β per component.
func bodyWitnessTorque(slot int, points []certPoint, impulses []ivec) *big.Rat {
	torque := new(big.Rat)
	for k, p := range points {
		var ball *big.Rat
		switch slot {
		case p.b:
			ball = p.ballB
		case p.a:
			ball = p.ballA
		default:
			continue
		}
		l1 := new(big.Rat)
		for axis := range 3 {
			proof.AddRat(l1, l1, magnitude(impulses[k][axis]))
		}
		proof.AddRat(torque, torque, proof.MulRat(new(big.Rat), ball, l1))
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
func (w *World) certifyFriction(cert *islandCertificate, p certPoint, bodies []certBody,
	impulseLimit, velocityLimit *big.Rat) {
	square := new(big.Rat)
	for _, component := range p.tangent {
		proof.AddRat(square, square, proof.MulRat(new(big.Rat), component, component))
	}
	coneLower := proof.MulRat(new(big.Rat), p.mu, p.lambda)
	allowed := proof.AddRat(new(big.Rat), coneLower, impulseLimit)
	norm := ratSqrtUpper(square)
	if excess := proof.SubRat(new(big.Rat), norm, coneLower); excess.Sign() > 0 {
		raise(&cert.cone, excess)
	}
	if allowed.Sign() < 0 || square.Cmp(proof.MulRat(new(big.Rat), allowed, allowed)) > 0 {
		cert.fail(gateCone, proof.SubRat(new(big.Rat), norm, coneLower), impulseLimit)
	}
	relative := subIVec(pointVelocity(bodies[p.b], bodies[p.b].vPost, bodies[p.b].wPost, p.rB),
		pointVelocity(bodies[p.a], bodies[p.a].vPost, bodies[p.a].wPost, p.rA))
	normalSpeed := proof.DotInterval3(relative, p.normal)
	var slide ivec
	for axis := range slide {
		slide[axis] = proof.SubInterval(relative[axis], proof.MulInterval(normalSpeed, p.normal[axis]))
	}
	speedUpper := euclideanUpper(slide)
	threshold := proof.SubRat(new(big.Rat), coneLower, impulseLimit)
	if threshold.Sign() > 0 && square.Cmp(proof.MulRat(new(big.Rat), threshold, threshold)) < 0 {
		raise(&cert.tangent, speedUpper)
		if speedUpper.Cmp(velocityLimit) > 0 {
			cert.fail(gateStick, speedUpper, velocityLimit)
		}
		return
	}
	opposed := proof.DotInterval3(pointIVec(p.tangent), slide).Hi
	opposed = proof.AddRat(new(big.Rat), opposed, proof.MulRat(new(big.Rat), norm, speedUpper))
	limit := proof.MulRat(new(big.Rat), impulseLimit, euclideanLower(slide))
	proof.AddRat(limit, limit, proof.MulRat(new(big.Rat), velocityLimit, ratSqrtLower(square)))
	if opposed.Cmp(limit) > 0 {
		cert.fail(gateSlip, opposed, limit)
	}
}

// ratSqrtUpper and ratSqrtLower bound the square root of a nonnegative
// rational from above and below by floats whose exact squares bracket it.
func ratSqrtUpper(square *big.Rat) *big.Rat {
	if square.Sign() <= 0 {
		return new(big.Rat)
	}
	approx, _ := square.Float64()
	root := math.Sqrt(approx)
	switch {
	case !finite(root):
		// sqrt(s) ≤ s + 1 for every s ≥ 0.
		return new(big.Rat).Add(square, big.NewRat(1, 1))
	case approx == 0:
		// s rounds to zero only below 2^-1074, whose root lies below 2^-537.
		return ratFloat(0x1p-537)
	}
	for new(big.Rat).Mul(ratFloat(root), ratFloat(root)).Cmp(square) < 0 {
		root = math.Nextafter(root, math.Inf(1))
	}
	return ratFloat(root)
}

func ratSqrtLower(square *big.Rat) *big.Rat {
	if square.Sign() <= 0 {
		return new(big.Rat)
	}
	approx, _ := square.Float64()
	root := math.Sqrt(approx)
	for root > 0 && new(big.Rat).Mul(ratFloat(root), ratFloat(root)).Cmp(square) > 0 {
		root = math.Nextafter(root, 0)
	}
	return ratFloat(root)
}

// euclideanLower bounds the Euclidean length of every vector in an interval
// box from below: each component contributes its smallest magnitude, zero
// when its interval straddles zero.
func euclideanLower(v ivec) *big.Rat {
	sum := new(big.Rat)
	for axis := range v {
		var least *big.Rat
		switch {
		case v[axis].Lo.Sign() > 0:
			least = v[axis].Lo
		case v[axis].Hi.Sign() < 0:
			least = new(big.Rat).Neg(v[axis].Hi)
		default:
			continue
		}
		sum.Add(sum, new(big.Rat).Mul(least, least))
	}
	return ratSqrtLower(sum)
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
