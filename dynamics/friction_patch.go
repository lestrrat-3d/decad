package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// frictionPatchResponse is the bounded four-corner response for one fixed
// floor and one translating box. Step consumes it only for the admitted patch.
type frictionPatchResponse struct {
	Points          []frictionPointImpulse
	Post            QuantityVec
	AngularUpper    units.Value
	NormalResidual  units.Value
	TangentResidual units.Value
	ConeResidual    units.Value
	Iterations      int
}

type frictionPointImpulse struct {
	Normal, TangentX, TangentY units.Value
}

type patchPoint struct {
	lever      r3.Vec
	leverExact [3]*big.Rat
	bound      *big.Rat
	jn         float64
	jx         float64
	jy         float64
}

// solveCenteredInteriorFrictionPatch divides one centered face impulse among
// the four real corners. The normal center of pressure balances the torque
// from tangential friction. The rational certificate audits every held impulse
// and the omitted angular motion before the step uses this response.
func solveCenteredInteriorFrictionPatch(manifold *decad.ContactManifold, mass decad.MassProperties,
	pose r3.Transform, pre QuantityVec, mu frictionCoefficient, cfg StepConfig) (frictionPatchResponse, bool) {
	if manifold == nil || len(manifold.Points) != 4 || validateMass(mass) != nil ||
		pose.Basis() != r3.Identity().Basis() ||
		validateQuantityVec(pre, units.Velocity) != nil ||
		pre.X.Base() <= 0 || pre.Y.Base() != 0 || pre.Z.Base() >= 0 ||
		mu.lower == nil || mu.upper == nil || mu.lower.Sign() <= 0 ||
		mass.Center.Bound.Base() != 0 || !finite(mu.nominal.Base()) {
		return frictionPatchResponse{}, false
	}
	center := pose.Apply(mass.Center.Value)
	first := manifold.Points[0].OnB.Value.Sub(center)
	halfX, halfY, height := math.Abs(first.X), math.Abs(first.Y), first.Z
	if !finite(halfX, halfY, height) || halfX <= 0 || halfY <= 0 || height >= 0 {
		return frictionPatchResponse{}, false
	}
	var points [4]patchPoint
	var corners [2][2]bool
	for i, witness := range manifold.Points {
		if witness.OnA.Value != witness.OnB.Value || witness.OnA.Bound.Base() != 0 ||
			witness.OnB.Bound.Base() != 0 || witness.Normal.Value != (r3.Vec{Z: 1}) ||
			witness.Normal.Bound.Base() != 0 || witness.NormalAngle.Base() != 0 {
			return frictionPatchResponse{}, false
		}
		lever := witness.OnB.Value.Sub(center)
		if math.Abs(lever.X) != halfX || math.Abs(lever.Y) != halfY || lever.Z != height {
			return frictionPatchResponse{}, false
		}
		x, y := 0, 0
		if lever.X > 0 {
			x = 1
		}
		if lever.Y > 0 {
			y = 1
		}
		if corners[x][y] {
			return frictionPatchResponse{}, false
		}
		corners[x][y] = true
		points[i] = patchPoint{lever: lever, leverExact: exactTranslatedPatchLever(
			witness.OnB.Value, mass.Center.Value, pose.Translation()),
			bound: exactBase(witness.OnB.Bound)}
	}
	jn := -pre.Z.Base() * mass.Mass.Value.Base()
	jt := -math.Min(pre.X.Base()*mass.Mass.Value.Base(), mu.nominal.Base()*jn)
	pressure := height * jt / jn
	if !finite(jn, jt, pressure) || jn <= 0 || jt >= 0 || math.Abs(pressure) >= halfX {
		return frictionPatchResponse{}, false
	}
	for i := range points {
		points[i].jn = jn * (1 + pressure*points[i].lever.X/(halfX*halfX)) / 4
		points[i].jx = points[i].jn * jt / jn
	}
	postX := pre.X.Base() + jt/mass.Mass.Value.Base()
	return certifyFrictionPatch(manifold, mass, pre, mu, cfg, points, r3.Vec{X: postX}, 1)
}

// solveFixedFloorFrictionPatch uses the real four-corner manifold in its
// published order. A fixed-order projected solve proposes impulses; exact
// rational residuals over the mass, point, center, and inertia bounds admit
// only a zero-spin, positive-X-slip response. Initial spin is exactly zero.
// The dynamic pose may translate without rotating. Its held translation is
// added exactly to the committed mass center for the response certificate.
func solveFixedFloorFrictionPatch(manifold *decad.ContactManifold, mass decad.MassProperties,
	pose r3.Transform, pre QuantityVec, mu frictionCoefficient, cfg StepConfig) (frictionPatchResponse, bool) {
	if manifold == nil || len(manifold.Points) != 4 || validateMass(mass) != nil ||
		pose.Basis() != r3.Identity().Basis() ||
		validateQuantityVec(pre, units.Velocity) != nil ||
		!validQuantity(mu.nominal, units.Dimensionless, true) || mu.lower == nil || mu.upper == nil ||
		mu.lower.Sign() <= 0 || mu.upper.Cmp(mu.lower) < 0 || cfg.MaxIterations <= 0 ||
		!validQuantity(cfg.VelocityResidual, units.Velocity, true) ||
		!validQuantity(cfg.AngularVelocityResidual, units.AngularVelocity, true) ||
		!validQuantity(cfg.ImpulseResidual, units.Impulse, true) ||
		!validQuantity(cfg.PenetrationResidual, units.Length, true) ||
		pre.X.Base() <= 0 || pre.Y.Mag() != 0 || pre.Z.Base() >= 0 {
		return frictionPatchResponse{}, false
	}
	massValue := mass.Mass.Value.Base()
	inverseMass := 1 / massValue
	inertia := mass.Inertia
	tensor, err := r3.NewSymmetricTensor(inertia.XX.Value.Base(), inertia.YY.Value.Base(),
		inertia.ZZ.Value.Base(), inertia.XY.Value.Base(), inertia.XZ.Value.Base(), inertia.YZ.Value.Base())
	if err != nil {
		return frictionPatchResponse{}, false
	}
	inverseInertia, err := tensor.Inverse()
	if err != nil {
		return frictionPatchResponse{}, false
	}
	var points [4]patchPoint
	worldCenter := pose.Apply(mass.Center.Value)
	for i, witness := range manifold.Points {
		if witness.Normal.Value != (r3.Vec{Z: 1}) || witness.Normal.Bound.Mag() != 0 ||
			witness.NormalAngle.Mag() != 0 || witness.OnA.Value != witness.OnB.Value ||
			witness.OnA.Bound.Base() > cfg.Contact.PointResolution.Base() ||
			witness.OnB.Bound.Base() > cfg.Contact.PointResolution.Base() ||
			math.Abs(witness.Separation.Value.Base())+witness.Separation.Bound.Base() >
				cfg.PenetrationResidual.Base() {
			return frictionPatchResponse{}, false
		}
		point := patchPoint{lever: witness.OnB.Value.Sub(worldCenter),
			bound: exactBase(witness.OnB.Bound)}
		point.leverExact = exactTranslatedPatchLever(witness.OnB.Value,
			mass.Center.Value, pose.Translation())
		points[i] = point
	}
	v := r3.Vec{X: pre.X.Base(), Y: pre.Y.Base(), Z: pre.Z.Base()}
	spin := r3.Vec{}
	normal, tangentX, tangentY := r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1}
	for iteration := 1; iteration <= cfg.MaxIterations; iteration++ {
		for i := range points {
			point := &points[i]
			r := point.lever
			normalArm := r.Cross(normal)
			kn := inverseMass + normalArm.Dot(inverseInertia.Apply(normalArm))
			if !finite(kn) || kn <= 0 {
				return frictionPatchResponse{}, false
			}
			at := v.Add(spin.Cross(r))
			nextNormal := math.Max(0, point.jn-at.Z/kn)
			change := normal.Scale(nextNormal - point.jn)
			point.jn = nextNormal
			v = v.Add(change.Scale(inverseMass))
			spin = spin.Add(inverseInertia.Apply(r.Cross(change)))
			xArm, yArm := r.Cross(tangentX), r.Cross(tangentY)
			kxx := inverseMass + xArm.Dot(inverseInertia.Apply(xArm))
			kyy := inverseMass + yArm.Dot(inverseInertia.Apply(yArm))
			kxy := xArm.Dot(inverseInertia.Apply(yArm))
			lip := math.Max(kxx, kyy) + math.Abs(kxy)
			if !finite(lip) || lip <= 0 {
				return frictionPatchResponse{}, false
			}
			at = v.Add(spin.Cross(r))
			qx, qy := point.jx-at.X/lip, point.jy-at.Y/lip
			coneLimit := mu.nominal.Base() * point.jn
			length := math.Hypot(qx, qy)
			if !finite(qx, qy, coneLimit, length) || coneLimit < 0 {
				return frictionPatchResponse{}, false
			}
			if length > coneLimit {
				qx, qy = qx*coneLimit/length, qy*coneLimit/length
			}
			change = tangentX.Scale(qx - point.jx).Add(tangentY.Scale(qy - point.jy))
			point.jx, point.jy = qx, qy
			v = v.Add(change.Scale(inverseMass))
			spin = spin.Add(inverseInertia.Apply(r.Cross(change)))
		}
		if !finite(v.X, v.Y, v.Z, spin.X, spin.Y, spin.Z) {
			return frictionPatchResponse{}, false
		}
		if response, ok := certifyFrictionPatch(manifold, mass, pre, mu, cfg, points, v, iteration); ok {
			return response, true
		}
	}
	return frictionPatchResponse{}, false
}

// certifyFrictionPatch uses nominal inverse inertia only through the proposed
// impulses. It bounds their aggregate torque before publishing zero spin.
func certifyFrictionPatch(manifold *decad.ContactManifold, mass decad.MassProperties,
	pre QuantityVec, mu frictionCoefficient, cfg StepConfig, points [4]patchPoint,
	v r3.Vec, iteration int) (frictionPatchResponse, bool) {
	massLow := new(big.Rat).Sub(exactBase(mass.Mass.Value), exactBase(mass.Mass.Bound))
	massHigh := new(big.Rat).Add(exactBase(mass.Mass.Value), exactBase(mass.Mass.Bound))
	lambda := certifiedInertiaLower(mass)
	if massLow.Sign() <= 0 || lambda == nil || lambda.Sign() <= 0 {
		return frictionPatchResponse{}, false
	}
	preHeld := [3]units.Value{pre.X, pre.Y, pre.Z}
	// Publish the constrained directions exactly. The interval check below
	// charges the change from the nominal iterate to the held impulse law.
	post := [3]float64{v.X, 0, 0}
	if math.Abs(post[0]) <= cfg.VelocityResidual.Base() {
		post[0] = 0
	}
	var total, torque, torqueError [3]*big.Rat
	for i := range 3 {
		total[i], torque[i], torqueError[i] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	centerBound := exactBase(mass.Center.Bound)
	for i, point := range points {
		j := [3]*big.Rat{ratFloat(point.jx), ratFloat(point.jy), ratFloat(point.jn)}
		r := point.leverExact
		if j[0] == nil || j[1] == nil || j[2] == nil || r[0] == nil || r[1] == nil || r[2] == nil ||
			point.bound == nil || point.bound.Sign() < 0 {
			return frictionPatchResponse{}, false
		}
		for axis := range 3 {
			total[axis].Add(total[axis], j[axis])
		}
		uncertainty := new(big.Rat).Add(point.bound, centerBound)
		for axis := range 3 {
			a, b := (axis+1)%3, (axis+2)%3
			term := new(big.Rat).Sub(new(big.Rat).Mul(r[a], j[b]), new(big.Rat).Mul(r[b], j[a]))
			torque[axis].Add(torque[axis], term)
			errorTerm := new(big.Rat).Add(absRat(new(big.Rat).Set(j[a])), absRat(new(big.Rat).Set(j[b])))
			torqueError[axis].Add(torqueError[axis], errorTerm.Mul(errorTerm, uncertainty))
		}
		if !finite(manifold.Points[i].OnB.Bound.Base()) {
			return frictionPatchResponse{}, false
		}
	}
	angularNumerator := new(big.Rat)
	for i := range 3 {
		angularNumerator.Add(angularNumerator, absRat(torque[i]))
		angularNumerator.Add(angularNumerator, torqueError[i])
	}
	angularUpper := new(big.Rat).Quo(angularNumerator, lambda)
	if angularUpper.Cmp(exactBase(cfg.AngularVelocityResidual)) > 0 {
		return frictionPatchResponse{}, false
	}
	var linearError [3]*big.Rat
	for axis := range 3 {
		lo := new(big.Rat).Add(exactBase(preHeld[axis]), new(big.Rat).Quo(total[axis], massLow))
		hi := new(big.Rat).Add(exactBase(preHeld[axis]), new(big.Rat).Quo(total[axis], massHigh))
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		linearError[axis] = intervalDeviation(ratFloat(post[axis]), lo, hi)
		if linearError[axis].Cmp(exactBase(cfg.VelocityResidual)) > 0 {
			return frictionPatchResponse{}, false
		}
	}
	var maxNormal, maxTangent, maxCone *big.Rat
	maxNormal, maxTangent, maxCone = new(big.Rat), new(big.Rat), new(big.Rat)
	impulseLimit := exactBase(cfg.ImpulseResidual)
	velocityLimit := exactBase(cfg.VelocityResidual)
	for _, point := range points {
		jn, jx, jy := ratFloat(point.jn), ratFloat(point.jx), ratFloat(point.jy)
		if jn == nil || jx == nil || jy == nil || jn.Sign() <= 0 || jx.Sign() > 0 {
			return frictionPatchResponse{}, false
		}
		coneLower := new(big.Rat).Mul(mu.lower, jn)
		coneUpper := new(big.Rat).Mul(mu.upper, jn)
		coneSquare := new(big.Rat).Add(new(big.Rat).Mul(jx, jx), new(big.Rat).Mul(jy, jy))
		allowed := new(big.Rat).Add(coneLower, impulseLimit)
		if coneSquare.Cmp(new(big.Rat).Mul(allowed, allowed)) > 0 {
			return frictionPatchResponse{}, false
		}
		_, coneNormUpper, ok := positiveSqrtBracket(coneSquare)
		if !ok {
			return frictionPatchResponse{}, false
		}
		coneResidual := new(big.Rat).Sub(coneNormUpper, coneLower)
		if coneResidual.Sign() > 0 {
			if coneResidual.Cmp(impulseLimit) > 0 {
				return frictionPatchResponse{}, false
			}
			if coneResidual.Cmp(maxCone) > 0 {
				maxCone = coneResidual
			}
		}
		lever := absRat(new(big.Rat).Set(point.leverExact[0]))
		lever.Add(lever, absRat(new(big.Rat).Set(point.leverExact[1])))
		lever.Add(lever, absRat(new(big.Rat).Set(point.leverExact[2])))
		lever.Add(lever, new(big.Rat).Mul(new(big.Rat).Add(point.bound, centerBound), big.NewRat(3, 1)))
		pointError := new(big.Rat).Mul(angularUpper, lever)
		normal := new(big.Rat).Add(absRat(ratFloat(post[2])), linearError[2])
		normal.Add(normal, pointError)
		tangent := new(big.Rat).Add(absRat(ratFloat(post[1])), linearError[1])
		tangent.Add(tangent, pointError)
		if normal.Cmp(velocityLimit) > 0 || tangent.Cmp(velocityLimit) > 0 {
			return frictionPatchResponse{}, false
		}
		if normal.Cmp(maxNormal) > 0 {
			maxNormal = normal
		}
		if tangent.Cmp(maxTangent) > 0 {
			maxTangent = tangent
		}
		slipLower := new(big.Rat).Sub(ratFloat(post[0]), linearError[0])
		slipLower.Sub(slipLower, pointError)
		if slipLower.Sign() <= 0 {
			sticking := new(big.Rat).Add(absRat(ratFloat(post[0])), linearError[0])
			sticking.Add(sticking, pointError)
			if sticking.Cmp(velocityLimit) > 0 {
				return frictionPatchResponse{}, false
			}
			if sticking.Cmp(maxTangent) > 0 {
				maxTangent = sticking
			}
			continue
		}
		direction := absRat(new(big.Rat).Add(jx, coneLower))
		upperDirection := absRat(new(big.Rat).Add(jx, coneUpper))
		if upperDirection.Cmp(direction) > 0 {
			direction = upperDirection
		}
		direction.Add(direction, absRat(new(big.Rat).Set(jy)))
		if direction.Cmp(impulseLimit) > 0 {
			return frictionPatchResponse{}, false
		}
		if direction.Cmp(maxCone) > 0 {
			maxCone = direction
		}
	}
	response := frictionPatchResponse{Post: QuantityVec{X: units.MillimetersPerSecond(post[0]),
		Y: units.MillimetersPerSecond(post[1]), Z: units.MillimetersPerSecond(post[2])},
		AngularUpper:    units.RadiansPerSecond(outwardRatFloat(angularUpper)),
		NormalResidual:  units.MillimetersPerSecond(outwardRatFloat(maxNormal)),
		TangentResidual: units.MillimetersPerSecond(outwardRatFloat(maxTangent)),
		ConeResidual:    units.KilogramMillimetersPerSecond(outwardRatFloat(maxCone)),
		Iterations:      iteration}
	for _, point := range points {
		response.Points = append(response.Points, frictionPointImpulse{
			Normal:   units.KilogramMillimetersPerSecond(point.jn),
			TangentX: units.KilogramMillimetersPerSecond(point.jx),
			TangentY: units.KilogramMillimetersPerSecond(point.jy)})
	}
	return response, true
}

func ratFloat(value float64) *big.Rat { return new(big.Rat).SetFloat64(value) }

func exactPatchLever(witness, center r3.Vec) [3]*big.Rat {
	w := [3]float64{witness.X, witness.Y, witness.Z}
	c := [3]float64{center.X, center.Y, center.Z}
	var lever [3]*big.Rat
	for axis := range 3 {
		lever[axis] = new(big.Rat).Sub(ratFloat(w[axis]), ratFloat(c[axis]))
	}
	return lever
}

func exactTranslatedPatchLever(witness, center, translation r3.Vec) [3]*big.Rat {
	w := [3]float64{witness.X, witness.Y, witness.Z}
	c := [3]float64{center.X, center.Y, center.Z}
	t := [3]float64{translation.X, translation.Y, translation.Z}
	var lever [3]*big.Rat
	for axis := range 3 {
		worldCenter := new(big.Rat).Add(ratFloat(c[axis]), ratFloat(t[axis]))
		lever[axis] = new(big.Rat).Sub(ratFloat(w[axis]), worldCenter)
	}
	return lever
}

func outwardRatFloat(value *big.Rat) float64 {
	float, _ := value.Float64()
	if !finite(float) {
		return math.Inf(1)
	}
	if ratFloat(float).Cmp(value) < 0 {
		return math.Nextafter(float, math.Inf(1))
	}
	return float
}
