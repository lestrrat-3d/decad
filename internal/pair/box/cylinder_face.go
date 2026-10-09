package box

import (
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// CylinderFaceCorridor selects one complete box-face corridor. A circular
// sidewall supports a transverse axis at the same exact extremum as its outer
// box; the other transverse coordinate and axial interval remain inside the
// box face. The caller has already admitted the cylinder and its topology.
func CylinderFaceCorridor(cylinder, box AxisBox, cylinderAxis int, hasWall bool) (int, int, proof.Dyadic, bool) {
	selected := -1
	var selectedSide int
	var selectedGap proof.Dyadic
	for axis := range 3 {
		if axis != cylinderAxis && (!hasWall || cylinderAxis != 2) {
			continue
		}
		if !CylinderInsideFace(cylinder, box, axis) {
			continue
		}
		var side int
		var gap proof.Dyadic
		switch {
		case proof.DyCmp(cylinder.Lo[axis], box.Hi[axis]) >= 0:
			side, gap = 1, proof.DySubScalar(cylinder.Lo[axis], box.Hi[axis])
		case proof.DyCmp(cylinder.Hi[axis], box.Lo[axis]) <= 0:
			side, gap = 0, proof.DySubScalar(box.Lo[axis], cylinder.Hi[axis])
		case proof.DyCmp(cylinder.Lo[axis], box.Lo[axis]) > 0 &&
			proof.DyCmp(cylinder.Hi[axis], box.Hi[axis]) > 0:
			side, gap = 1, proof.DySubScalar(cylinder.Lo[axis], box.Hi[axis])
		case proof.DyCmp(cylinder.Hi[axis], box.Hi[axis]) < 0 &&
			proof.DyCmp(cylinder.Lo[axis], box.Lo[axis]) < 0:
			side, gap = 0, proof.DySubScalar(box.Lo[axis], cylinder.Hi[axis])
		default:
			continue
		}
		if selected >= 0 {
			return 0, 0, proof.Dyadic{}, false
		}
		selected, selectedSide, selectedGap = axis, side, gap
	}
	return selected, selectedSide, selectedGap, selected >= 0
}

// CylinderInsideFace proves strict containment in both directions across the
// selected box face. The strict inequalities exclude edge contact.
func CylinderInsideFace(cylinder, box AxisBox, normalAxis int) bool {
	for axis := range 3 {
		if axis == normalAxis {
			continue
		}
		if proof.DyCmp(cylinder.Lo[axis], box.Lo[axis]) <= 0 ||
			proof.DyCmp(cylinder.Hi[axis], box.Hi[axis]) >= 0 {
			return false
		}
	}
	return true
}

// CylinderFacePoint reads one point on each support surface at the center of
// the cylinder's transverse outer box. The caller binds these points to the
// original faces after checking that those face identities are present.
func CylinderFacePoint(cylinder, box AxisBox, axis, side int, signedGap proof.Dyadic,
	pointResolutionMM float64) (PointReading, PointReading, pair.ScalarReading, bool) {
	var boxPoint, cylinderPoint proof.DyV3
	for i := range 3 {
		if i == axis {
			if side == 1 {
				boxPoint[i], cylinderPoint[i] = box.Hi[i], cylinder.Lo[i]
			} else {
				boxPoint[i], cylinderPoint[i] = box.Lo[i], cylinder.Hi[i]
			}
			continue
		}
		center := proof.DyMul(proof.DyAdd(cylinder.Lo[i], cylinder.Hi[i]), proof.MustDyOf(.5))
		boxPoint[i], cylinderPoint[i] = center, center
	}
	boxWitness, okBox := ReadPointAt(&boxPoint)
	cylinderWitness, okCylinder := ReadPointAt(&cylinderPoint)
	if !okBox || !okCylinder ||
		boxWitness.BoundMM > pointResolutionMM || cylinderWitness.BoundMM > pointResolutionMM {
		return PointReading{}, PointReading{}, pair.ScalarReading{}, false
	}
	separation, ok := SignedReading(signedGap)
	return boxWitness, cylinderWitness, separation, ok
}
