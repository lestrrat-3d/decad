package proof

import "github.com/lestrrat-3d/r3"

// TriangleOpenFloat reports whether the float triangle a, b, c is proven
// not collinear: one component of the exact normal (b−a)×(c−a) is proven
// non-zero by a float evaluation whose error bound it clears.
func TriangleOpenFloat(a, b, c r3.Vec) bool {
	for _, x := range [...]float64{a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z} {
		m := Abs(x)
		if !(m == 0 || m >= 0x1p-300 && m <= 0x1p300) {
			return false
		}
	}
	ux, uy, uz := b.X-a.X, b.Y-a.Y, b.Z-a.Z
	vx, vy, vz := c.X-a.X, c.Y-a.Y, c.Z-a.Z
	return orientFloatProven(uy, uz, vy, vz) || orientFloatProven(uz, ux, vz, vx) ||
		orientFloatProven(ux, uy, vx, vy)
}

func orientFloatProven(p, q, r, s float64) bool {
	left, right := float64(p*s), float64(q*r)
	det := left - right
	bound := (Abs(left) + Abs(right)) * 0x1p-50
	return Abs(det) > bound
}
