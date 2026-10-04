package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// certifyDynamicPairResponse evaluates the held point impulses as exact
// rationals. It widens both bodies' linear and angular laws by their mass,
// inertia, center, and contact-point bounds before checking contact residuals.
func certifyDynamicPairResponse(bodies [2]pairPatchBody, points [4]pairPatchPoint,
	mu frictionCoefficient, restitution units.Value, cfg StepConfig,
	iteration int) (pairPatchResponse, bool) {
	var total [3]*big.Rat
	for axis := range total {
		total[axis] = new(big.Rat)
	}
	for _, point := range points {
		for axis, value := range [3]float64{point.jx, point.jy, point.jn} {
			if !finite(value) {
				return pairPatchResponse{}, false
			}
			total[axis].Add(total[axis], ratFloat(value))
		}
	}
	response := pairPatchResponse{Iterations: iteration}
	var linearError [2][3]*big.Rat
	var angularError [2]*big.Rat
	for i, body := range bodies {
		massLow := new(big.Rat).Sub(exactBase(body.mass.Mass.Value), exactBase(body.mass.Mass.Bound))
		massHigh := new(big.Rat).Add(exactBase(body.mass.Mass.Value), exactBase(body.mass.Mass.Bound))
		if massLow.Sign() <= 0 {
			return pairPatchResponse{}, false
		}
		sign := int64(1)
		if i == 0 {
			sign = -1
		}
		pre := [3]units.Value{body.pre.X, body.pre.Y, body.pre.Z}
		post := [3]float64{body.v.X, body.v.Y, body.v.Z}
		spin := [3]float64{body.spin.X, body.spin.Y, body.spin.Z}
		if !finite(post[0], post[1], post[2], spin[0], spin[1], spin[2]) {
			return pairPatchResponse{}, false
		}
		for axis := range linearError[i] {
			impulse := new(big.Rat).Mul(total[axis], big.NewRat(sign, 1))
			low := new(big.Rat).Add(exactBase(pre[axis]), new(big.Rat).Quo(impulse, massLow))
			high := new(big.Rat).Add(exactBase(pre[axis]), new(big.Rat).Quo(impulse, massHigh))
			if low.Cmp(high) > 0 {
				low, high = high, low
			}
			linearError[i][axis] = intervalDeviation(ratFloat(post[axis]), low, high)
			if linearError[i][axis].Cmp(exactBase(cfg.VelocityResidual)) > 0 {
				return pairPatchResponse{}, false
			}
		}
		angularError[i] = pairAngularLawError(body.mass, points, i, spin, sign)
		if angularError[i] == nil ||
			angularError[i].Cmp(exactBase(cfg.AngularVelocityResidual)) > 0 {
			return pairPatchResponse{}, false
		}
		spinUpper := new(big.Rat).Set(angularError[i])
		for _, component := range spin {
			spinUpper.Add(spinUpper, absRat(ratFloat(component)))
		}
		response.Post[i] = QuantityVec{X: units.MillimetersPerSecond(post[0]),
			Y: units.MillimetersPerSecond(post[1]), Z: units.MillimetersPerSecond(post[2])}
		response.PostAngular[i] = QuantityVec{X: units.RadiansPerSecond(spin[0]),
			Y: units.RadiansPerSecond(spin[1]), Z: units.RadiansPerSecond(spin[2])}
		response.AngularError[i] = units.RadiansPerSecond(outwardRatFloat(angularError[i]))
		response.AngularUpper[i] = units.RadiansPerSecond(outwardRatFloat(spinUpper))
	}
	response.NormalResidual, response.TangentResidual, response.ConeResidual =
		units.MillimetersPerSecond(0), units.MillimetersPerSecond(0),
		units.KilogramMillimetersPerSecond(0)
	var maxNormal, maxTangent, maxCone big.Rat
	velocityLimit, impulseLimit := exactBase(cfg.VelocityResidual), exactBase(cfg.ImpulseResidual)
	targetExact := new(big.Rat)
	preRelative := new(big.Rat).Sub(exactBase(bodies[1].pre.Z), exactBase(bodies[0].pre.Z))
	closing := new(big.Rat).Neg(preRelative)
	if closing.Cmp(exactBase(cfg.ImpactSpeed)) > 0 {
		targetExact.Mul(preRelative, exactBase(restitution))
		targetExact.Neg(targetExact)
	}
	for _, point := range points {
		jn, jx, jy := ratFloat(point.jn), ratFloat(point.jx), ratFloat(point.jy)
		if jn.Sign() <= 0 {
			return pairPatchResponse{}, false
		}
		velocity, errorBound := pairPointVelocityInterval(bodies, point, linearError, angularError)
		if velocity[0] == nil {
			return pairPatchResponse{}, false
		}
		normal := absRat(new(big.Rat).Sub(velocity[2], targetExact))
		normal.Add(normal, errorBound[2])
		if normal.Cmp(velocityLimit) > 0 {
			return pairPatchResponse{}, false
		}
		if normal.Cmp(&maxNormal) > 0 {
			maxNormal.Set(normal)
		}
		if mu.upper.Sign() == 0 {
			if jx.Sign() != 0 || jy.Sign() != 0 {
				return pairPatchResponse{}, false
			}
			response.Points = append(response.Points, frictionPointImpulse{
				Normal:   units.KilogramMillimetersPerSecond(point.jn),
				TangentX: units.KilogramMillimetersPerSecond(0),
				TangentY: units.KilogramMillimetersPerSecond(0)})
			continue
		}
		tangentError := new(big.Rat).Add(errorBound[0], errorBound[1])
		stickResidual := new(big.Rat).Add(absRat(new(big.Rat).Set(velocity[0])),
			absRat(new(big.Rat).Set(velocity[1])))
		stickResidual.Add(stickResidual, tangentError)
		slipSquared := new(big.Rat).Add(new(big.Rat).Mul(velocity[0], velocity[0]),
			new(big.Rat).Mul(velocity[1], velocity[1]))
		coneLower := new(big.Rat).Mul(mu.lower, jn)
		coneUpper := new(big.Rat).Mul(mu.upper, jn)
		allowed := new(big.Rat).Add(coneLower, impulseLimit)
		lengthSquared := new(big.Rat).Add(new(big.Rat).Mul(jx, jx), new(big.Rat).Mul(jy, jy))
		if lengthSquared.Cmp(new(big.Rat).Mul(allowed, allowed)) > 0 {
			return pairPatchResponse{}, false
		}
		if stickResidual.Cmp(velocityLimit) <= 0 {
			if stickResidual.Cmp(&maxTangent) > 0 {
				maxTangent.Set(stickResidual)
			}
			response.Points = append(response.Points, frictionPointImpulse{
				Normal:   units.KilogramMillimetersPerSecond(point.jn),
				TangentX: units.KilogramMillimetersPerSecond(point.jx),
				TangentY: units.KilogramMillimetersPerSecond(point.jy)})
			continue
		}
		slipLow, slipHigh, ok := positiveSqrtBracket(slipSquared)
		if !ok {
			return pairPatchResponse{}, false
		}
		minimumSlip := new(big.Rat).Sub(slipLow, tangentError)
		if minimumSlip.Sign() <= 0 {
			return pairPatchResponse{}, false
		}
		factorLow := new(big.Rat).Quo(coneLower, slipHigh)
		factorHigh := new(big.Rat).Quo(coneUpper, slipLow)
		impulse := [2]*big.Rat{jx, jy}
		direction := new(big.Rat)
		for axis := range impulse {
			low := new(big.Rat).Mul(factorLow, velocity[axis])
			high := new(big.Rat).Mul(factorHigh, velocity[axis])
			if low.Cmp(high) > 0 {
				low, high = high, low
			}
			low.Add(low, impulse[axis])
			high.Add(high, impulse[axis])
			component := absRat(low)
			if upper := absRat(high); upper.Cmp(component) > 0 {
				component = upper
			}
			direction.Add(direction, component)
		}
		// Unit-vector normalization is at most 2E/(|u|-E) away
		// when the tangent velocity's L1 error is E.
		direction.Add(direction, new(big.Rat).Quo(
			new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), coneUpper), tangentError),
			minimumSlip))
		if direction.Cmp(impulseLimit) > 0 {
			return pairPatchResponse{}, false
		}
		if direction.Cmp(&maxCone) > 0 {
			maxCone.Set(direction)
		}
		if tangentError.Cmp(&maxTangent) > 0 {
			maxTangent.Set(tangentError)
		}
		response.Points = append(response.Points, frictionPointImpulse{
			Normal:   units.KilogramMillimetersPerSecond(point.jn),
			TangentX: units.KilogramMillimetersPerSecond(point.jx),
			TangentY: units.KilogramMillimetersPerSecond(point.jy)})
	}
	response.NormalResidual = units.MillimetersPerSecond(outwardRatFloat(&maxNormal))
	response.TangentResidual = units.MillimetersPerSecond(outwardRatFloat(&maxTangent))
	response.ConeResidual = units.KilogramMillimetersPerSecond(outwardRatFloat(&maxCone))
	return response, true
}

func pairAngularLawError(mass decad.MassProperties, points [4]pairPatchPoint,
	body int, spin [3]float64, sign int64) *big.Rat {
	lambda := certifiedInertiaLower(mass)
	if lambda == nil || lambda.Sign() <= 0 {
		return nil
	}
	components := [3][3]decad.Measurement{
		{mass.Inertia.XX, mass.Inertia.XY, mass.Inertia.XZ},
		{mass.Inertia.XY, mass.Inertia.YY, mass.Inertia.YZ},
		{mass.Inertia.XZ, mass.Inertia.YZ, mass.Inertia.ZZ},
	}
	var torque, torqueError [3]*big.Rat
	for axis := range torque {
		torque[axis], torqueError[axis] = new(big.Rat), new(big.Rat)
	}
	centerBound := exactBase(mass.Center.Bound)
	for _, point := range points {
		j := [3]*big.Rat{ratFloat(point.jx), ratFloat(point.jy), ratFloat(point.jn)}
		uncertainty := new(big.Rat).Add(point.bound[body], centerBound)
		for axis := range torque {
			a, b := (axis+1)%3, (axis+2)%3
			cross := new(big.Rat).Sub(new(big.Rat).Mul(point.exactArm[body][a], j[b]),
				new(big.Rat).Mul(point.exactArm[body][b], j[a]))
			torque[axis].Add(torque[axis], new(big.Rat).Mul(cross, big.NewRat(sign, 1)))
			component := new(big.Rat).Add(absRat(new(big.Rat).Set(j[a])),
				absRat(new(big.Rat).Set(j[b])))
			torqueError[axis].Add(torqueError[axis], new(big.Rat).Mul(component, uncertainty))
		}
	}
	residual := new(big.Rat)
	for axis := range torque {
		action := new(big.Rat)
		for column := range components[axis] {
			action.Add(action, new(big.Rat).Mul(exactBase(components[axis][column].Value),
				ratFloat(spin[column])))
			torqueError[axis].Add(torqueError[axis],
				new(big.Rat).Mul(exactBase(components[axis][column].Bound),
					absRat(ratFloat(spin[column]))))
		}
		residual.Add(residual, absRat(new(big.Rat).Sub(action, torque[axis])))
		residual.Add(residual, torqueError[axis])
	}
	return residual.Quo(residual, lambda)
}

func pairPointVelocityInterval(bodies [2]pairPatchBody, point pairPatchPoint,
	linear [2][3]*big.Rat, angular [2]*big.Rat) ([3]*big.Rat, [3]*big.Rat) {
	var relative, errorBound [3]*big.Rat
	for axis := range relative {
		relative[axis], errorBound[axis] = new(big.Rat), new(big.Rat)
	}
	for i, body := range bodies {
		sign := int64(1)
		if i == 0 {
			sign = -1
		}
		v := [3]float64{body.v.X, body.v.Y, body.v.Z}
		spin := [3]float64{body.spin.X, body.spin.Y, body.spin.Z}
		lever := new(big.Rat)
		spinMagnitude := new(big.Rat)
		for axis := range point.exactArm[i] {
			lever.Add(lever, absRat(new(big.Rat).Set(point.exactArm[i][axis])))
			spinMagnitude.Add(spinMagnitude, absRat(ratFloat(spin[axis])))
		}
		positionError := new(big.Rat).Add(point.bound[i], exactBase(body.mass.Center.Bound))
		lever.Add(lever, positionError)
		uncertainty := new(big.Rat).Add(new(big.Rat).Mul(angular[i], lever),
			new(big.Rat).Mul(spinMagnitude, positionError))
		for axis := range relative {
			a, b := (axis+1)%3, (axis+2)%3
			cross := new(big.Rat).Sub(new(big.Rat).Mul(ratFloat(spin[a]), point.exactArm[i][b]),
				new(big.Rat).Mul(ratFloat(spin[b]), point.exactArm[i][a]))
			cross.Add(cross, ratFloat(v[axis]))
			relative[axis].Add(relative[axis], new(big.Rat).Mul(cross, big.NewRat(sign, 1)))
			errorBound[axis].Add(errorBound[axis], linear[i][axis])
			errorBound[axis].Add(errorBound[axis], uncertainty)
		}
	}
	return relative, errorBound
}

func positiveSqrtBracket(square *big.Rat) (*big.Rat, *big.Rat, bool) {
	if square == nil || square.Sign() <= 0 {
		return nil, nil, false
	}
	approximate, _ := new(big.Float).SetPrec(256).SetRat(square).Float64()
	root := math.Sqrt(approximate)
	if !finite(root) || root <= 0 {
		return nil, nil, false
	}
	lower, upper := root, root
	for new(big.Rat).Mul(ratFloat(lower), ratFloat(lower)).Cmp(square) > 0 {
		lower = math.Nextafter(lower, 0)
	}
	for new(big.Rat).Mul(ratFloat(upper), ratFloat(upper)).Cmp(square) < 0 {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	return ratFloat(lower), ratFloat(upper), true
}
