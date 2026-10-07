package proofbound

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// IvVec3 encloses a 3D vector coordinate-wise. A vector built from held
// float64s alone encloses it EXACTLY — every interval is a point — and only a
// square root or a sine widens one.
type IvVec3 [3]RatInterval

// IvVec3Of encloses a held vector exactly, one point interval per coordinate.
func IvVec3Of(v r3.Vec) (IvVec3, bool) {
	x, y, z := proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)
	if x == nil || y == nil || z == nil {
		return IvVec3{}, false
	}
	return IvVec3{PointInterval(x), PointInterval(y), PointInterval(z)}, true
}

func IvVec3Add(a, b IvVec3) IvVec3 {
	return IvVec3{IntervalAdd(a[0], b[0]), IntervalAdd(a[1], b[1]), IntervalAdd(a[2], b[2])}
}

func IvVec3Sub(a, b IvVec3) IvVec3 {
	return IvVec3{IntervalSub(a[0], b[0]), IntervalSub(a[1], b[1]), IntervalSub(a[2], b[2])}
}

func IvVec3Cross(a, b IvVec3) IvVec3 {
	return IvVec3{
		IntervalSub(IntervalMul(a[1], b[2]), IntervalMul(a[2], b[1])),
		IntervalSub(IntervalMul(a[2], b[0]), IntervalMul(a[0], b[2])),
		IntervalSub(IntervalMul(a[0], b[1]), IntervalMul(a[1], b[0])),
	}
}

func IvVec3Mul(a IvVec3, s RatInterval) IvVec3 {
	return IvVec3{IntervalMul(a[0], s), IntervalMul(a[1], s), IntervalMul(a[2], s)}
}

func IvVec3Dot(a, b IvVec3) RatInterval {
	return IntervalAdd(IntervalAdd(IntervalMul(a[0], b[0]), IntervalMul(a[1], b[1])), IntervalMul(a[2], b[2]))
}

// IvVec3NormSq is |v|² — IntervalSquare per coordinate, never IntervalMul with
// itself, so a coordinate straddling zero cannot contribute a negative low
// end.
func IvVec3NormSq(a IvVec3) RatInterval {
	return IntervalAdd(IntervalAdd(IntervalSquare(a[0]), IntervalSquare(a[1])), IntervalSquare(a[2]))
}
