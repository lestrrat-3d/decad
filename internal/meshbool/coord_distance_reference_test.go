package meshbool_test

import (
	"math"
	"math/big"
	"math/rand/v2"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// referenceCoordDistance is CoordDistance as it stood before the exact-vertex
// shortcut. It is the differential tests' and benchmark's reference.
func referenceCoordDistance(p proof.Xpt, v r3.Vec) *big.Rat {
	d := proof.Xsub(p, proof.XptOf(v))
	d.X.Abs(d.X)
	d.Y.Abs(d.Y)
	d.Z.Abs(d.Z)
	d.W.Abs(d.W)
	n := d.X
	if d.Y.Cmp(n) > 0 {
		n = d.Y
	}
	if d.Z.Cmp(n) > 0 {
		n = d.Z
	}
	return proof.RatFrac(new(big.Rat), n, d.W)
}

// coordDistanceFloat returns a finite float64 spread over many magnitudes,
// with the exact zeros, signed zeros, extremes and subnormals mixed in.
func coordDistanceFloat(r *rand.Rand) float64 {
	switch r.IntN(12) {
	case 0:
		return 0
	case 1:
		return math.Copysign(0, -1)
	case 2:
		return math.MaxFloat64 * (r.Float64()*2 - 1)
	case 3:
		return math.SmallestNonzeroFloat64 * float64(r.IntN(9)-4)
	case 4:
		return math.Ldexp(r.Float64()*2-1, r.IntN(2000)-1000)
	case 5:
		return float64(r.IntN(2001) - 1000)
	case 6:
		return math.Ldexp(float64(r.Int64N(1<<53)), r.IntN(200)-100)
	default:
		return (r.Float64()*2 - 1) * 100
	}
}

func coordDistanceVec(r *rand.Rand) r3.Vec {
	return r3.Vec{X: coordDistanceFloat(r), Y: coordDistanceFloat(r), Z: coordDistanceFloat(r)}
}

// coordDistancePoint builds an exact point near v: v itself, one coordinate
// off by an ulp, or an unrelated point; spelled reduced, scaled by a common
// factor, or over an odd weight.
func coordDistancePoint(r *rand.Rand, v r3.Vec) proof.Xpt {
	switch r.IntN(6) {
	case 0:
		return proof.XptOf(coordDistanceVec(r))
	case 1:
		w := v
		switch r.IntN(3) {
		case 0:
			w.X = math.Nextafter(w.X, math.Inf(1))
		case 1:
			w.Y = math.Nextafter(w.Y, math.Inf(-1))
		default:
			w.Z = math.Nextafter(w.Z, 0)
		}
		return proof.XptOf(w)
	case 2:
		// A non-dyadic point: a third of v plus a third of another vector.
		o := coordDistanceVec(r)
		return scaledPoint(proof.XptOf(v), proof.XptOf(o), 3)
	}
	p := proof.XptOf(v)
	if r.IntN(2) == 0 {
		return p
	}
	// The same point under a common factor on all four fields.
	k := big.NewInt(int64(1 + r.IntN(1000)))
	if r.IntN(2) == 0 {
		k.Lsh(k, uint(r.IntN(300)))
	}
	return proof.Xpt{
		X: new(big.Int).Mul(p.X, k), Y: new(big.Int).Mul(p.Y, k),
		Z: new(big.Int).Mul(p.Z, k), W: new(big.Int).Mul(p.W, k),
	}
}

// scaledPoint is (a + b) / n over an odd weight, so it is not a float.
func scaledPoint(a, b proof.Xpt, n int64) proof.Xpt {
	s := proof.Xlerp(a, b, big.NewInt(1), big.NewInt(n))
	return s
}
