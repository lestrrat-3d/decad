package massmoment_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// adversarialRat draws zero, an integer, a float lift (power-of-two
// denominator), a non-reduced-looking fraction over a smooth denominator, or a
// huge numerator over a huge denominator, each of either sign.
func adversarialRat(rng *rand.Rand) *big.Rat {
	sign := int64(1 - 2*rng.IntN(2))
	switch rng.IntN(7) {
	case 0:
		return new(big.Rat)
	case 1:
		return new(big.Rat).SetInt64(sign * rng.Int64N(50))
	case 2:
		return proofarith.FloatRat(rng.NormFloat64())
	case 3:
		return proofarith.FloatRat(math.Ldexp(rng.NormFloat64(), rng.IntN(300)-150))
	case 4:
		den := big.NewInt(int64(2+rng.IntN(60)) * 3 * 5 * 7)
		return new(big.Rat).SetFrac(big.NewInt(sign*int64(rng.IntN(1000))*3*5), den)
	case 5:
		num := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<40)+1), uint(rng.IntN(300)))
		den := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<40)*2+1), uint(rng.IntN(300)))
		return new(big.Rat).SetFrac(num.Mul(num, big.NewInt(sign)), den)
	}
	return big.NewRat(sign, 1)
}

func adversarialMatrix(rng *rand.Rand) [3][3]*big.Rat {
	var m [3][3]*big.Rat
	for i := range m {
		for j := range m[i] {
			m[i][j] = adversarialRat(rng)
		}
	}
	if rng.IntN(4) == 0 {
		// Entries sharing one *big.Rat reach MulRat with a == b.
		m[1][1], m[2][2], m[0][1] = m[0][0], m[0][0], m[0][0]
	}
	return m
}

func sameRat(x, y *big.Rat) bool {
	return x.Num().Cmp(y.Num()) == 0 && x.Denom().Cmp(y.Denom()) == 0
}

func orthonormalityDefectRaw(m [3][3]*big.Rat) *big.Rat {
	defect := new(big.Rat)
	for i := range m {
		for j := range m {
			dot := new(big.Rat)
			for k := range m {
				dot.Add(dot, new(big.Rat).Mul(m[k][i], m[k][j]))
			}
			if i == j {
				dot.Sub(dot, big.NewRat(1, 1))
			}
			defect.Add(defect, dot.Abs(dot))
		}
	}
	return defect
}

func rotateTensorRaw(m [3][3]*big.Rat, local [3][3]proofbound.RatInterval) [3][3]proofbound.RatInterval {
	var out [3][3]proofbound.RatInterval
	for i := range out {
		for j := range out[i] {
			sum := proofbound.PointInterval(new(big.Rat))
			for k := range local {
				for l := range local[k] {
					coefficient := new(big.Rat).Mul(m[i][k], m[j][l])
					sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(local[k][l], coefficient))
				}
			}
			out[i][j] = sum
		}
	}
	return out
}

func gershgorinLowerRaw(t [3][3]proofbound.RatInterval) *big.Rat {
	var lower *big.Rat
	for i := range t {
		row := new(big.Rat).Set(t[i][i].Lo)
		for j := range t[i] {
			if i != j {
				row.Sub(row, proofbound.IntervalAbsUpper(t[i][j]))
			}
		}
		if lower == nil || row.Cmp(lower) < 0 {
			lower = row
		}
	}
	return lower
}

func prismRotationRaw(frame r3.Frame, xform r3.Transform) [3][3]*big.Rat {
	component := func(v r3.Vec, i int) float64 { return [3]float64{v.X, v.Y, v.Z}[i] }
	basis := xform.Basis()
	placement := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	frameAxes := [3]r3.Vec{frame.U(), frame.V(), frame.N()}
	var out [3][3]*big.Rat
	for i := range out {
		for k := range out[i] {
			sum := new(big.Rat)
			for l := range placement {
				entry := proofarith.FloatRat(component(placement[l], i))
				axis := proofarith.FloatRat(component(frameAxes[k], l))
				sum.Add(sum, entry.Mul(entry, axis))
			}
			out[i][k] = sum
		}
	}
	return out
}

func TestOrthonormalityDefectMatchesRawRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 5))
	for range 4000 {
		m := adversarialMatrix(rng)
		want := orthonormalityDefectRaw(m)
		got := massmoment.OrthonormalityDefect(m)
		require.True(t, sameRat(got, want), "defect %v, want %v", got, want)
	}
	// An exactly orthonormal map has defect zero.
	identity := [3][3]*big.Rat{}
	for i := range identity {
		for j := range identity[i] {
			identity[i][j] = big.NewRat(int64(1-min(1, (i-j)*(i-j))), 1)
		}
	}
	require.Zero(t, massmoment.OrthonormalityDefect(identity).Sign())
}

func TestRotateTensorAndGershgorinMatchRawRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 9))
	interval := func() proofbound.RatInterval {
		a, b := adversarialRat(rng), adversarialRat(rng)
		if a.Cmp(b) > 0 {
			a, b = b, a
		}
		return proofbound.RatInterval{Lo: a, Hi: b}
	}
	for range 1500 {
		m := adversarialMatrix(rng)
		var local [3][3]proofbound.RatInterval
		for i := range local {
			for j := range local[i] {
				local[i][j] = interval()
			}
		}
		want := rotateTensorRaw(m, local)
		got := massmoment.RotateTensor(m, local)
		for i := range got {
			for j := range got[i] {
				require.True(t, sameRat(got[i][j].Lo, want[i][j].Lo) && sameRat(got[i][j].Hi, want[i][j].Hi),
					"entry (%d, %d): got [%v, %v], want [%v, %v]", i, j,
					got[i][j].Lo, got[i][j].Hi, want[i][j].Lo, want[i][j].Hi)
			}
		}
		require.True(t, sameRat(massmoment.GershgorinLower(local), gershgorinLowerRaw(local)),
			"GershgorinLower %v, want %v", massmoment.GershgorinLower(local), gershgorinLowerRaw(local))
	}
}

func TestPrismRotationAndMidLevelMatchRawRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 13))
	axis := func() r3.Vec { return r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5} }
	for range 1000 {
		frame, err := r3.NewFrame(r3.Vec{X: rng.Float64()}, axis(), axis())
		require.NoError(t, err)
		xform, err := r3.Rotation(axis(), units.Degrees(rng.Float64()*360))
		require.NoError(t, err)
		got, err := massmoment.PrismRotation(frame, xform)
		require.NoError(t, err)
		want := prismRotationRaw(frame, xform)
		for i := range got {
			for k := range got[i] {
				require.True(t, sameRat(got[i][k], want[i][k]), "entry (%d, %d): got %v, want %v", i, k, got[i][k], want[i][k])
			}
		}

		z0, z1 := rng.NormFloat64()*100, math.Ldexp(rng.NormFloat64(), rng.IntN(60)-30)
		mid, err := massmoment.PrismMidLevel(z0, z1)
		require.NoError(t, err)
		wantMid := new(big.Rat).Quo(new(big.Rat).Add(proofarith.FloatRat(z0), proofarith.FloatRat(z1)), big.NewRat(2, 1))
		require.True(t, sameRat(mid, wantMid), "mid %v, want %v", mid, wantMid)
	}
}
