package occupancy

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"

	"github.com/stretchr/testify/require"
)

func applyAffineFixture(angle, tx, ty, tz float64) Affine {
	c, s := math.Cos(angle), math.Sin(angle)
	return Affine{
		M: [3][3]float64{{c, -s, 0}, {s, c * 0.8, -0.3}, {0.1, 0.2, 0.97}},
		T: [3]float64{tx, ty, tz},
	}
}

// applyTermwise is exactAffine.apply's term-by-term form, reducing the sum
// after every term: the reference the single-fraction form must equal.
func applyTermwise(e exactAffine, p xvec) xvec {
	var out xvec
	var prod big.Rat
	for i := range 3 {
		s := new(big.Rat).Set(e.t[i])
		for j := range 3 {
			if e.m[i][j].Sign() == 0 || p[j].Sign() == 0 {
				continue
			}
			s.Add(s, prod.Mul(e.m[i][j], p[j]))
		}
		out[i] = s
	}
	return out
}

func TestExactAffineApplyMatchesTermwise(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	pick := func() float64 {
		switch rng.IntN(5) {
		case 0:
			return 0
		case 1:
			return float64(rng.IntN(9) - 4)
		default:
			return (rng.Float64() - 0.5) * 40
		}
	}
	for range 400 {
		a := applyAffineFixture(rng.Float64()*6, pick(), pick(), pick())
		for i := range 3 {
			for j := range 3 {
				if rng.IntN(4) == 0 {
					a.M[i][j] = 0
				}
			}
		}
		e, err := invert(a)
		if err != nil {
			continue
		}
		p := xvec{ratOf(pick()), ratOf(pick()), ratOf(pick())}
		want := applyTermwise(e, p)
		got := e.apply(p)
		again := e.apply(got)
		wantAgain := applyTermwise(e, want)
		for k := range 3 {
			require.Zero(t, want[k].Cmp(got[k]), "component %d", k)
			require.Zero(t, wantAgain[k].Cmp(again[k]), "chained component %d", k)
			require.Zero(t, want[k].Num().Cmp(got[k].Num()), "numerator %d", k)
			require.Zero(t, want[k].Denom().Cmp(got[k].Denom()), "denominator %d", k)
		}
	}
}

func BenchmarkExactApplyChain(b *testing.B) {
	var inv []exactAffine
	for _, a := range []Affine{applyAffineFixture(0.3, 1.5, -2.25, 7.125), applyAffineFixture(1.1, -4.5, 3.75, 0.5)} {
		e, err := invert(a)
		require.NoError(b, err)
		inv = append(inv, e)
	}
	p := xvec{ratOf(3.14159), ratOf(-2.71828), ratOf(1.41421)}
	var out xvec
	b.ResetTimer()
	for range b.N {
		x := p
		for _, e := range inv {
			x = e.apply(x)
		}
		out = x
	}
	_ = out
}

func TestDotExactMatchesTermwise(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 5))
	pick := func() float64 {
		if rng.IntN(5) == 0 {
			return 0
		}
		return (rng.Float64() - 0.5) * 100
	}
	for range 400 {
		a := r3Vec(pick(), pick(), pick())
		e, err := invert(applyAffineFixture(rng.Float64()*6, pick(), pick(), pick()))
		require.NoError(t, err)
		p := e.apply(xvec{ratOf(pick()), ratOf(pick()), ratOf(pick())})
		want := new(big.Rat).Mul(ratOf(a.X), p[0])
		want.Add(want, new(big.Rat).Mul(ratOf(a.Y), p[1]))
		want.Add(want, new(big.Rat).Mul(ratOf(a.Z), p[2]))
		got := dotExact(ratVecOf(a), p)
		require.Zero(t, want.Num().Cmp(got.Num()))
		require.Zero(t, want.Denom().Cmp(got.Denom()))
	}
}

func BenchmarkSeparatedProjection(b *testing.B) {
	e, err := invert(applyAffineFixture(0.3, 1.5, -2.25, 7.125))
	require.NoError(b, err)
	var verts []xvec
	for i := range 12 {
		verts = append(verts, e.apply(xvec{ratOf(float64(i) * 1.37), ratOf(float64(i%5) - 2.1), ratOf(0.7 * float64(i))}))
	}
	var tri [3]xvec
	for i := range tri {
		tri[i] = e.apply(xvec{ratOf(float64(i) + 0.3), ratOf(2.2 - float64(i)), ratOf(5.5)})
	}
	axis := r3Vec(0.3, 0.5, 0.81)
	b.ResetTimer()
	for range b.N {
		separated(axis, tri, verts)
	}
}

func r3Vec(x, y, z float64) r3.Vec { return r3.Vec{X: x, Y: y, Z: z} }
