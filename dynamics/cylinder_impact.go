package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Source-cylinder face manifolds have one representative witness. Before a zero-spin
// response, bound the point speed, energy, and full-step travel that an omitted
// off-axis torque could cause over the entire source body.
func (w *World) omittedCylinderMotionWithin(manifold *decad.ContactManifold, pose r3.Transform,
	dynamic, axis int, impulse float64, duration units.Value) bool {
	body := w.parts[dynamic].definition.Body
	if manifold == nil || len(manifold.Points) == 0 || !finite(impulse) {
		return false
	}
	point := manifold.Points[0]
	face, witness := point.FaceA, point.OnA
	if dynamic == 1 {
		face, witness = point.FaceB, point.OnB
	}
	if !cylinderSourceFaceOnBody(body, face) {
		return true
	}
	if len(manifold.Points) != 1 {
		return false
	}
	mass := w.parts[dynamic].mass
	center, centerError, ok := worldCenterReading(pose, mass.Center)
	pointBound, inertia := exactBase(witness.Bound), certifiedInertiaLower(mass)
	impulseLimit, angularLimit := exactBase(w.step.ImpulseResidual),
		exactBase(w.step.AngularVelocityResidual)
	velocityLimit, pointLimit := exactBase(w.step.VelocityResidual),
		exactBase(w.step.Contact.PointResolution)
	contactSlop, penetrationLimit := exactBase(w.step.ContactSlop),
		exactBase(w.step.PenetrationResidual)
	span, massValue, massBound := exactBase(duration), exactBase(mass.Mass.Value),
		exactBase(mass.Mass.Bound)
	if !ok || pointBound == nil || inertia == nil || inertia.Sign() <= 0 ||
		impulseLimit == nil || angularLimit == nil || velocityLimit == nil ||
		pointLimit == nil || contactSlop == nil || penetrationLimit == nil ||
		span == nil || massValue == nil || massBound == nil || massBound.Sign() < 0 {
		return false
	}
	coords := [3]float64{witness.Value.X, witness.Value.Y, witness.Value.Z}
	lever := new(big.Rat)
	for _, tangent := range [2]int{(axis + 1) % 3, (axis + 2) % 3} {
		coordinate := ratFloat(coords[tangent])
		if coordinate == nil || center[tangent] == nil || centerError[tangent] == nil {
			return false
		}
		arm := absRat(new(big.Rat).Sub(coordinate, center[tangent]))
		arm.Add(arm, centerError[tangent]).Add(arm, pointBound)
		lever.Add(lever, arm)
	}
	upperImpulse := new(big.Rat).Add(ratFloat(math.Abs(impulse)), impulseLimit)
	angularMomentum := new(big.Rat).Mul(upperImpulse, lever)
	spin := new(big.Rat).Quo(angularMomentum, inertia)
	if spin.Cmp(angularLimit) > 0 {
		return false
	}
	reach, ok := cylinderBodyReach(body, mass.Center)
	if !ok {
		return false
	}
	pointSpeed := new(big.Rat).Mul(spin, reach)
	if pointSpeed.Cmp(velocityLimit) > 0 {
		return false
	}
	energy := new(big.Rat).Mul(angularMomentum, angularMomentum)
	energy.Quo(energy, new(big.Rat).Mul(big.NewRat(2, 1), inertia))
	massUpper := new(big.Rat).Add(massValue, massBound)
	energyAllowance := new(big.Rat).Mul(massUpper,
		new(big.Rat).Mul(velocityLimit, velocityLimit))
	energyAllowance.Quo(energyAllowance, big.NewRat(2, 1))
	if energy.Cmp(energyAllowance) > 0 {
		return false
	}
	travel := new(big.Rat).Mul(pointSpeed, span)
	return travel.Cmp(pointLimit) <= 0 && travel.Cmp(contactSlop) <= 0 &&
		travel.Cmp(penetrationLimit) <= 0
}

func cylinderSourceFaceOnBody(body *decad.Body, face *decad.Face) bool {
	if face == nil {
		return false
	}
	switch face.Surface().(type) {
	case decad.Plane, decad.Cylinder:
	default:
		return false
	}
	faces := body.Faces()
	if len(faces) != 3 {
		return false
	}
	found, walls := false, 0
	for _, candidate := range faces {
		found = found || candidate == face
		if _, ok := candidate.Surface().(decad.Cylinder); ok {
			walls++
		}
	}
	return found && walls == 1
}

// An expanded source bounding box contains every body point. Its L1 radius
// about the supplied mass center bounds distance after any rigid rotation.
func cylinderBodyReach(body *decad.Body, center decad.VecMeasurement) (*big.Rat, bool) {
	box, err := body.Bounds()
	boxError, centerError := exactBase(box.Bound), exactBase(center.Bound)
	if err != nil || boxError == nil || centerError == nil ||
		boxError.Sign() < 0 || centerError.Sign() < 0 {
		return nil, false
	}
	lo := [3]float64{box.Min.X, box.Min.Y, box.Min.Z}
	hi := [3]float64{box.Max.X, box.Max.Y, box.Max.Z}
	middle := [3]float64{center.Value.X, center.Value.Y, center.Value.Z}
	reach := new(big.Rat)
	for axis := range 3 {
		if !finite(lo[axis], hi[axis], middle[axis]) {
			return nil, false
		}
		low := absRat(new(big.Rat).Sub(ratFloat(lo[axis]), ratFloat(middle[axis])))
		high := absRat(new(big.Rat).Sub(ratFloat(hi[axis]), ratFloat(middle[axis])))
		if high.Cmp(low) > 0 {
			low = high
		}
		reach.Add(reach, low)
		reach.Add(reach, boxError)
		reach.Add(reach, centerError)
	}
	return reach, true
}
