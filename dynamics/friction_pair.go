package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// pairPatchResponse keeps the numerical impulses and the independently
// bounded response of both dynamic bodies. The translating step publishes
// zero spin only when both omitted-spin bounds meet its angular residual.
type pairPatchResponse struct {
	Points          []frictionPointImpulse
	Post            [2]QuantityVec
	PostAngular     [2]QuantityVec
	AngularUpper    [2]units.Value
	AngularError    [2]units.Value
	NormalResidual  units.Value
	TangentResidual units.Value
	ConeResidual    units.Value
	Iterations      int
}

type pairPatchBody struct {
	mass    decad.MassProperties
	pre     QuantityVec
	center  r3.Vec
	inverse float64
	inertia r3.SymmetricTensor
	v, spin r3.Vec
}

type pairPatchPoint struct {
	arm      [2]r3.Vec
	exactArm [2][3]*big.Rat
	bound    [2]*big.Rat
	jn, jx   float64
	jy       float64
}

// solveTwoDynamicFrictionPatch solves the four real contact points together.
// Both bodies contribute inverse mass and inertia to each projected impulse.
// The final rational audit bounds both linear laws, both torque sums, and
// every point's normal, slip, and friction-cone residual.
func solveTwoDynamicFrictionPatch(manifold *decad.ContactManifold, masses [2]decad.MassProperties,
	poses [2]r3.Transform, pre [2]QuantityVec, mu frictionCoefficient,
	restitution units.Value, cfg StepConfig) (pairPatchResponse, bool) {
	if manifold == nil || len(manifold.Points) != 4 || cfg.MaxIterations <= 0 ||
		!validQuantity(mu.nominal, units.Dimensionless, true) || mu.lower == nil || mu.upper == nil ||
		mu.lower.Sign() <= 0 || mu.upper.Cmp(mu.lower) < 0 ||
		!validQuantity(restitution, units.Dimensionless, false) || restitution.Base() > 1 {
		return pairPatchResponse{}, false
	}
	var bodies [2]pairPatchBody
	for i := range bodies {
		if validateMass(masses[i]) != nil || poses[i] != r3.Identity() ||
			validateQuantityVec(pre[i], units.Velocity) != nil {
			return pairPatchResponse{}, false
		}
		inertia := masses[i].Inertia
		tensor, err := r3.NewSymmetricTensor(inertia.XX.Value.Base(), inertia.YY.Value.Base(),
			inertia.ZZ.Value.Base(), inertia.XY.Value.Base(), inertia.XZ.Value.Base(), inertia.YZ.Value.Base())
		if err != nil {
			return pairPatchResponse{}, false
		}
		inverse, err := tensor.Inverse()
		if err != nil {
			return pairPatchResponse{}, false
		}
		bodies[i] = pairPatchBody{mass: masses[i], pre: pre[i], center: masses[i].Center.Value,
			inverse: 1 / masses[i].Mass.Value.Base(), inertia: inverse,
			v: r3.Vec{X: pre[i].X.Base(), Y: pre[i].Y.Base(), Z: pre[i].Z.Base()}}
	}
	if !finite(bodies[0].inverse, bodies[1].inverse) ||
		bodies[0].inverse <= 0 || bodies[1].inverse <= 0 ||
		pre[1].X.Base()-pre[0].X.Base() <= 0 ||
		pre[1].Y.Base() != pre[0].Y.Base() ||
		pre[1].Z.Base()-pre[0].Z.Base() >= 0 {
		return pairPatchResponse{}, false
	}
	target := -restitution.Base() * (pre[1].Z.Base() - pre[0].Z.Base())
	if -pre[1].Z.Base()+pre[0].Z.Base() <= cfg.ImpactSpeed.Base() {
		target = 0
	}
	if !finite(target) || target < 0 {
		return pairPatchResponse{}, false
	}
	var points [4]pairPatchPoint
	for i, witness := range manifold.Points {
		if witness.Normal.Value != (r3.Vec{Z: 1}) || witness.Normal.Bound.Mag() != 0 ||
			witness.NormalAngle.Mag() != 0 || witness.OnA.Value != witness.OnB.Value ||
			math.Abs(witness.Separation.Value.Base())+witness.Separation.Bound.Base() >
				cfg.PenetrationResidual.Base() {
			return pairPatchResponse{}, false
		}
		positions := [2]decad.VecMeasurement{witness.OnA, witness.OnB}
		for j := range bodies {
			if positions[j].Bound.Base() > cfg.Contact.PointResolution.Base() {
				return pairPatchResponse{}, false
			}
			points[i].arm[j] = positions[j].Value.Sub(bodies[j].center)
			points[i].exactArm[j] = exactPatchLever(positions[j].Value, bodies[j].center)
			points[i].bound[j] = exactBase(positions[j].Bound)
		}
	}
	normal, tangentX, tangentY := r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1}
	for iteration := 1; iteration <= cfg.MaxIterations; iteration++ {
		for i := range points {
			point := &points[i]
			kn := pairEffectiveMass(bodies, *point, normal)
			if !finite(kn) || kn <= 0 {
				return pairPatchResponse{}, false
			}
			relative := pairPointVelocity(bodies, *point)
			next := math.Max(0, point.jn+(target-relative.Z)/kn)
			pairApplyImpulse(&bodies, *point, normal.Scale(next-point.jn))
			point.jn = next
			kxx := pairEffectiveMass(bodies, *point, tangentX)
			kyy := pairEffectiveMass(bodies, *point, tangentY)
			kxy := pairCrossMass(bodies, *point, tangentX, tangentY)
			lip := math.Max(kxx, kyy) + math.Abs(kxy)
			if !finite(lip) || lip <= 0 {
				return pairPatchResponse{}, false
			}
			relative = pairPointVelocity(bodies, *point)
			jx, jy := point.jx-relative.X/lip, point.jy-relative.Y/lip
			limit := mu.nominal.Base() * point.jn
			length := math.Hypot(jx, jy)
			if !finite(jx, jy, limit, length) || limit < 0 {
				return pairPatchResponse{}, false
			}
			if length > limit {
				jx, jy = jx*limit/length, jy*limit/length
			}
			pairApplyImpulse(&bodies, *point, r3.Vec{X: jx - point.jx, Y: jy - point.jy})
			point.jx, point.jy = jx, jy
		}
		// A common spin is a useful certified representative when the
		// numerical responses differ only within the angular residual.
		// The rational audit below checks both torque laws after rounding.
		if candidate, ok := commonPairSpinCandidate(bodies, cfg.AngularVelocityResidual.Base()); ok {
			if response, valid := certifyDynamicPairResponse(candidate, points, mu, restitution,
				cfg, iteration); valid {
				return response, true
			}
		}
		if response, ok := certifyDynamicPairResponse(bodies, points, mu, restitution,
			cfg, iteration); ok {
			return response, true
		}
	}
	return pairPatchResponse{}, false
}

func commonPairSpinCandidate(bodies [2]pairPatchBody, tolerance float64) ([2]pairPatchBody, bool) {
	components := [2][3]float64{{bodies[0].spin.X, bodies[0].spin.Y, bodies[0].spin.Z},
		{bodies[1].spin.X, bodies[1].spin.Y, bodies[1].spin.Z}}
	if !finite(tolerance) || tolerance <= 0 {
		return bodies, false
	}
	var common [3]float64
	for axis := range common {
		if !finite(components[0][axis], components[1][axis]) ||
			math.Abs(components[0][axis]-components[1][axis]) > tolerance/4 {
			return bodies, false
		}
		common[axis] = components[0][axis]/2 + components[1][axis]/2
		if !finite(common[axis]) {
			return bodies, false
		}
	}
	shared := r3.Vec{X: common[0], Y: common[1], Z: common[2]}
	bodies[0].spin, bodies[1].spin = shared, shared
	return bodies, true
}

func pairPointVelocity(bodies [2]pairPatchBody, point pairPatchPoint) r3.Vec {
	a := bodies[0].v.Add(bodies[0].spin.Cross(point.arm[0]))
	b := bodies[1].v.Add(bodies[1].spin.Cross(point.arm[1]))
	return b.Sub(a)
}

func pairEffectiveMass(bodies [2]pairPatchBody, point pairPatchPoint, direction r3.Vec) float64 {
	k := bodies[0].inverse + bodies[1].inverse
	for i := range bodies {
		arm := point.arm[i].Cross(direction)
		k += arm.Dot(bodies[i].inertia.Apply(arm))
	}
	return k
}

func pairCrossMass(bodies [2]pairPatchBody, point pairPatchPoint, a, b r3.Vec) float64 {
	k := 0.0
	for i := range bodies {
		k += point.arm[i].Cross(a).Dot(bodies[i].inertia.Apply(point.arm[i].Cross(b)))
	}
	return k
}

func pairApplyImpulse(bodies *[2]pairPatchBody, point pairPatchPoint, impulse r3.Vec) {
	for i := range bodies {
		sign := float64(1)
		if i == 0 {
			sign = -1
		}
		bodies[i].v = bodies[i].v.Add(impulse.Scale(sign * bodies[i].inverse))
		bodies[i].spin = bodies[i].spin.Add(
			bodies[i].inertia.Apply(point.arm[i].Cross(impulse)).Scale(sign))
	}
}
