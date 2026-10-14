package spiralsector

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/r3"
)

// AxisCopyMap returns the exact linear map of a copy whose true solid is the
// image under a rotation about the X axis through the origin, and false for a
// nil record or any record that is not such a rotation. It is a structural
// check on an exact record, never a residual, so it can only refuse.
func AxisCopyMap(rec *exactrotation.Rotation) ([3][3]*big.Rat, bool) {
	if rec == nil || !rec.FixesXAxisAtOrigin() {
		return [3][3]*big.Rat{}, false
	}
	return rec.Linear(), true
}

// NearAxisRotation reports whether t is a pure rotation about the X axis whose
// X basis vector is within one float step of unit length.
// r3.Rotation's Rodrigues evaluation can round its X basis length one float
// step below or above one. Keep that narrow case separate from the exact-axis
// class, because the sector certificate must charge its axial displacement.
func NearAxisRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t == r3.Identity() ||
		t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EX.X >= math.Nextafter(1, 0) &&
		b.EX.X <= math.Nextafter(1, math.Inf(1)) &&
		b.EX.Y == 0 && b.EX.Z == 0 && b.EY.X == 0 && b.EZ.X == 0 &&
		b.EY.Y == b.EZ.Z && b.EY.Z == -b.EZ.Y
}
