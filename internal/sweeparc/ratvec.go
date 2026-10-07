package sweeparc

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/r3"
)

// RatVec holds one spatial vector in exact rational coordinates.
type RatVec [3]*big.Rat

func VecOf(v r3.Vec) RatVec {
	return RatVec{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func FromDyadic(v proofarith.DyV3) RatVec {
	return RatVec{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func Add(vectors ...RatVec) RatVec {
	out := RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, vector := range vectors {
		for i := range out {
			out[i].Add(out[i], vector[i])
		}
	}
	return out
}

func Sub(a, b RatVec) RatVec {
	return RatVec{
		new(big.Rat).Sub(a[0], b[0]),
		new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func Scale(v RatVec, scale *big.Rat) RatVec {
	return RatVec{
		new(big.Rat).Mul(v[0], scale),
		new(big.Rat).Mul(v[1], scale),
		new(big.Rat).Mul(v[2], scale),
	}
}

func Dot(a, b RatVec) *big.Rat {
	return proofbound.RatAdd(
		new(big.Rat).Mul(a[0], b[0]),
		new(big.Rat).Mul(a[1], b[1]),
		new(big.Rat).Mul(a[2], b[2]),
	)
}

func Cross(a, b RatVec) RatVec {
	component := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(
			new(big.Rat).Mul(a[i], b[j]),
			new(big.Rat).Mul(a[j], b[i]),
		)
	}
	return RatVec{component(1, 2), component(2, 0), component(0, 1)}
}

func IsZero(v RatVec) bool {
	return v[0].Sign() == 0 && v[1].Sign() == 0 && v[2].Sign() == 0
}

// Held rounds one exact value and returns its outward error bound.
func Held(value *big.Rat) (float64, float64, bool) {
	held, _ := value.Float64()
	if math.IsNaN(held) || math.IsInf(held, 0) {
		return 0, 0, false
	}
	bound := proofarith.RationalFloatError(value, held)
	return held, bound, !math.IsNaN(bound) && !math.IsInf(bound, 0)
}

// Normalized2 reads two rational direction components as binary64 values.
func Normalized2(u, v, lengthSquared *big.Rat) (float64, float64, bool) {
	const precision = 256
	length := new(big.Float).SetPrec(precision).SetRat(lengthSquared)
	length.Sqrt(length)
	if length.Sign() == 0 {
		return 0, 0, false
	}
	component := func(value *big.Rat) float64 {
		f := new(big.Float).SetPrec(precision).SetRat(value)
		f.Quo(f, length)
		held, _ := f.Float64()
		return held
	}
	heldU, heldV := component(u), component(v)
	if !revolveaxis.FiniteAxisValues(heldU, heldV) {
		return 0, 0, false
	}
	return heldU, heldV, true
}
