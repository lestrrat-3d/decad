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
// arithmetic (docs/multibody-dynamics-design.md §6.3, frictionless rows).
// Every comparison runs over the whole interval: mass, inertia, mass-center,
// contact-point and normal uncertainty all enter as intervals, and the
// published velocities and impulses enter as the exact rationals their
// float64 values denote. The certificate never inverts an interval tensor.

type ivec = [3]proof.RatInterval

// certBody is one island participant read as exact intervals. A Fixed
// participant has zero velocity and no mass reading; a kinematic one moves at
// its exact driver translation velocity before and after the event.
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
	reading      decad.InertiaReading
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
	lambda      *big.Rat
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
	case gateEnergy:
		return units.New(f, units.KilogramSquareMillimeterPerSecondSquared)
	case gateNormalSign:
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
// signed upper end of the kinetic-energy change), every refused gate, and
// the first refused gate in evaluation order.
type islandCertificate struct {
	linear, angular, normal, energy, momentum, angularMomentum *big.Rat
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
		out[axis] = proof.OwnedInterval(new(big.Rat).Sub(center[axis], radius),
			new(big.Rat).Add(center[axis], radius))
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
		sum.Add(sum, new(big.Rat).Mul(m, m))
	}
	approx, _ := sum.Float64()
	root := math.Sqrt(approx)
	for {
		r := ratFloat(root)
		if r != nil && new(big.Rat).Mul(r, r).Cmp(sum) >= 0 {
			return r
		}
		root = math.Nextafter(root, math.Inf(1))
	}
}

// newCertBody reads one island participant at its event pose. A kinematic
// participant reads its exact driver velocity from drive.
func (w *World) newCertBody(index int, entry BodyState, post BodyState,
	drive map[int][3]*big.Rat) (certBody, bool) {
	body := certBody{index: index, dynamic: w.bodies[index].definition.Role == Dynamic, pose: entry.Pose}
	zero := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	body.v, body.w, body.vPost, body.wPost = zero, zero, zero, zero
	if v, ok := drive[index]; ok && w.bodies[index].definition.Role == Kinematic {
		body.kinematic, body.v, body.vPost = true, v, v
	}
	if !body.dynamic {
		return body, true
	}
	mass := w.bodies[index].mass
	m, bound := exactBase(mass.Mass.Value), exactBase(mass.Mass.Bound)
	if m == nil || bound == nil || bound.Sign() < 0 {
		return certBody{}, false
	}
	body.mass = proof.OwnedInterval(new(big.Rat).Sub(m, bound), new(big.Rat).Add(m, bound))
	if body.mass.Lo.Sign() <= 0 {
		return certBody{}, false
	}
	body.reading = mass.Inertia
	largest := new(big.Rat)
	components := [3][3]decad.Measurement{
		{mass.Inertia.XX, mass.Inertia.XY, mass.Inertia.XZ},
		{mass.Inertia.XY, mass.Inertia.YY, mass.Inertia.YZ},
		{mass.Inertia.XZ, mass.Inertia.YZ, mass.Inertia.ZZ}}
	for i, row := range components {
		for j, component := range row {
			value, componentBound := exactBase(component.Value), exactBase(component.Bound)
			if value == nil || componentBound == nil || componentBound.Sign() < 0 {
				return certBody{}, false
			}
			body.inertia[i][j] = proof.OwnedInterval(new(big.Rat).Sub(value, componentBound),
				new(big.Rat).Add(value, componentBound))
			if m := magnitude(body.inertia[i][j]); m.Cmp(largest) > 0 {
				largest = m
			}
		}
	}
	basis := entry.Pose.Basis()
	for column, axis := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		values, ok := ratVec(axis)
		if !ok {
			return certBody{}, false
		}
		for row := range 3 {
			body.rotation[row][column] = values[row]
		}
	}
	d := new(big.Rat)
	for i := range 3 {
		for j := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				entry.Add(entry, new(big.Rat).Mul(body.rotation[k][i], body.rotation[k][j]))
			}
			if i == j {
				entry.Sub(entry, big.NewRat(1, 1))
			}
			d.Add(d, absRat(entry))
		}
	}
	body.defect = new(big.Rat).Mul(big.NewRat(3, 1), d)
	body.defect.Mul(body.defect, new(big.Rat).Add(big.NewRat(2, 1), d))
	body.defect.Mul(body.defect, largest)
	body.inertiaLower = certifiedInertiaFloor(mass)
	body.rowCeiling = inertiaRowCeiling(mass.Inertia)
	if body.inertiaLower == nil || body.inertiaLower.Sign() <= 0 || body.rowCeiling == nil {
		return certBody{}, false
	}
	center, centerError, ok := worldCenterReading(entry.Pose, mass.Center)
	if !ok {
		return certBody{}, false
	}
	body.centerL1 = new(big.Rat)
	for axis := range 3 {
		body.center[axis] = proof.OwnedInterval(new(big.Rat).Sub(center[axis], centerError[axis]),
			new(big.Rat).Add(center[axis], centerError[axis]))
		body.centerL1.Add(body.centerL1, magnitude(body.center[axis]))
	}
	var valid [4]bool
	body.v, valid[0] = quantityRats(entry.LinearVelocity)
	body.w, valid[1] = quantityRats(entry.AngularVelocity)
	body.vPost, valid[2] = quantityRats(post.LinearVelocity)
	body.wPost, valid[3] = quantityRats(post.AngularVelocity)
	return body, valid[0] && valid[1] && valid[2] && valid[3]
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
			local[j].Add(local[j], new(big.Rat).Mul(b.rotation[i][j], x[i]))
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
		widen.Add(widen, absRat(new(big.Rat).Set(value)))
	}
	widen.Mul(widen, b.defect)
	var out ivec
	for row := range out {
		out[row] = proof.ScaleInterval(y[0], b.rotation[row][0])
		for i := 1; i < 3; i++ {
			out[row] = proof.AddInterval(out[row], proof.ScaleInterval(y[i], b.rotation[row][i]))
		}
		out[row] = proof.OwnedInterval(new(big.Rat).Sub(out[row].Lo, widen), new(big.Rat).Add(out[row].Hi, widen))
	}
	return out
}

// pointVelocity encloses v + ω×r for a dynamic body slot; a kinematic body
// translates at v and a Fixed body, whose v is zero, stands still.
func pointVelocity(body certBody, v, omega [3]*big.Rat, lever ivec) ivec {
	if !body.dynamic {
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
	normal, okN := ballIVec(point.Normal.Value, new(big.Rat).Add(normalBound, angle))
	onA, okA := ballIVec(point.OnA.Value, boundA)
	onB, okB := ballIVec(point.OnB.Value, boundB)
	if !okN || !okA || !okB {
		return certPoint{}, false
	}
	p := certPoint{pair: pair, a: a, b: b, normal: normal, onA: onA, onB: onB, restitution: restitution}
	p.rA, p.rB = zeroIVec(), zeroIVec()
	if bodies[a].dynamic {
		p.rA = subIVec(onA, bodies[a].center)
	}
	if bodies[b].dynamic {
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

// certifyIsland runs §6.3's frictionless rows over one island.
func (w *World) certifyIsland(bodies []certBody, points []certPoint) islandCertificate {
	cert := islandCertificate{linear: new(big.Rat), angular: new(big.Rat), normal: new(big.Rat),
		energy: new(big.Rat), momentum: new(big.Rat), angularMomentum: new(big.Rat)}
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
	// Each point's impulse on B, λ·n; A receives its negation.
	impulses := make([]ivec, len(points))
	for k, p := range points {
		impulses[k] = scaleIVec(p.normal, p.lambda)
	}
	linearMomentum, angularMomentum := zeroIVec(), zeroIVec()
	linearMomentumLimit, angularMomentumLimit := new(big.Rat), new(big.Rat)
	energyUpper, energyAllowance, kinematicWork := new(big.Rat), new(big.Rat), new(big.Rat)
	for slot, body := range bodies {
		if !body.dynamic {
			continue
		}
		var dv, dw [3]*big.Rat
		for axis := range 3 {
			dv[axis] = new(big.Rat).Sub(body.vPost[axis], body.v[axis])
			dw[axis] = new(big.Rat).Sub(body.wPost[axis], body.w[axis])
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
		linearLimit := new(big.Rat).Add(impulseLimit, new(big.Rat).Mul(body.mass.Hi, velocityLimit))
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
		// intervals.
		spinChange := body.inertiaApply(dw)
		angularBodyLimit := new(big.Rat).Add(new(big.Rat).Mul(impulseLimit, rho),
			new(big.Rat).Mul(body.inertiaLower, angularLimit))
		for axis := range 3 {
			residual := magnitude(proof.SubInterval(spinChange[axis], torque[axis]))
			raise(&cert.angular, residual)
			if residual.Cmp(angularBodyLimit) > 0 {
				cert.fail(gateAngularLaw, residual, angularBodyLimit)
			}
		}
		// Island momentum about the world origin: Σ m·Δv and
		// Σ (I·Δω + c×m·Δv), against the impulses delivered by Fixed bodies.
		linearMomentum = addIVec(linearMomentum, momentumChange)
		angularMomentum = addIVec(angularMomentum,
			addIVec(spinChange, proof.CrossInterval3(body.center, momentumChange)))
		linearMomentumLimit.Add(linearMomentumLimit, linearLimit)
		angularMomentumLimit.Add(angularMomentumLimit, angularBodyLimit)
		angularMomentumLimit.Add(angularMomentumLimit, new(big.Rat).Mul(body.centerL1, linearLimit))
		// Energy: the squared-speed difference with the mass endpoint that
		// maximises it, the rotational change over the inertia intervals, and
		// the defect widening of both rotational readings.
		squaredChange := new(big.Rat)
		for axis := range 3 {
			squaredChange.Add(squaredChange, new(big.Rat).Sub(new(big.Rat).Mul(body.vPost[axis], body.vPost[axis]),
				new(big.Rat).Mul(body.v[axis], body.v[axis])))
			speeds := new(big.Rat).Add(absRat(new(big.Rat).Set(body.v[axis])),
				absRat(new(big.Rat).Set(body.vPost[axis])))
			speeds.Add(speeds, velocityLimit)
			energyAllowance.Add(energyAllowance, new(big.Rat).Mul(linearLimit, speeds))
			spins := new(big.Rat).Add(absRat(new(big.Rat).Set(body.w[axis])),
				absRat(new(big.Rat).Set(body.wPost[axis])))
			spins.Add(spins, angularLimit)
			energyAllowance.Add(energyAllowance, new(big.Rat).Mul(
				new(big.Rat).Mul(big.NewRat(3, 1), new(big.Rat).Mul(body.rowCeiling, angularLimit)), spins))
		}
		if squaredChange.Sign() > 0 {
			energyUpper.Add(energyUpper, new(big.Rat).Mul(body.mass.Hi, squaredChange))
		} else {
			energyUpper.Add(energyUpper, new(big.Rat).Mul(body.mass.Lo, squaredChange))
		}
		spinUpper, ok := spinEnergyChange(body.reading, body.pose, ratQuantity(body.w), ratQuantity(body.wPost))
		if !ok {
			cert.fail(gateEnergy, new(big.Rat), new(big.Rat))
			continue
		}
		energyUpper.Add(energyUpper, spinUpper)
		for _, omega := range [2][3]*big.Rat{body.w, body.wPost} {
			l1 := new(big.Rat)
			for _, value := range omega {
				l1.Add(l1, absRat(new(big.Rat).Set(value)))
			}
			energyUpper.Add(energyUpper, new(big.Rat).Mul(body.defect, new(big.Rat).Mul(l1, l1)))
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
				u[axis] = new(big.Rat).Add(body.vPost[axis], new(big.Rat).Mul(e, body.v[axis]))
				omega[axis] = new(big.Rat).Add(body.wPost[axis], new(big.Rat).Mul(e, body.w[axis]))
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
		// Kinematic work: a driver on side A delivers λ·(n·V) to the island,
		// one on side B −λ·(n·V); the energy gate admits its upper end.
		for side, slot := range [2]int{p.a, p.b} {
			if !bodies[slot].kinematic {
				continue
			}
			work := proof.ScaleInterval(proof.DotInterval3(p.normal, pointIVec(bodies[slot].v)), p.lambda)
			if side == 1 {
				work = proof.NegInterval(work)
			}
			kinematicWork.Add(kinematicWork, work.Hi)
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
	energyAllowance.Add(energyAllowance, kinematicWork)
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

// ratQuantity wraps exact angular velocity components for spinEnergyChange,
// which reads them back through exactBase; the values are float64 values
// already, so the round trip is exact.
func ratQuantity(x [3]*big.Rat) QuantityVec {
	var out QuantityVec
	for axis, value := range x {
		f, _ := value.Float64()
		setVelocityComponent(&out, axis, units.RadiansPerSecond(f))
	}
	return out
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
	out.TangentResidual = units.MillimetersPerSecond(0)
	out.ConeResidual = units.KilogramMillimetersPerSecond(0)
	out.AngularUpper = units.RadiansPerSecond(0)
	out.Iterations = iterations
	return out, err
}
