package spiralsector

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

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
