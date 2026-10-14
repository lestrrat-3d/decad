package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestOrientSignFloatAxisZero checks the shortcut for four points sharing one
// coordinate: it must be certain, and must equal the exact sign of zero.
func TestOrientSignFloatAxisZero(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x3f9a1c05, 0x77e2b4d9))
	rand3 := func() r3.Vec {
		return r3.Vec{X: 1e3 * (rng.Float64() - 0.5), Y: 1e3 * (rng.Float64() - 0.5), Z: 1e3 * (rng.Float64() - 0.5)}
	}
	for axis := range 3 {
		for range 200 {
			pts := [4]r3.Vec{rand3(), rand3(), rand3(), rand3()}
			shared := 1e3*rng.Float64() - 500
			for i := range pts {
				switch axis {
				case 0:
					pts[i].X = shared
				case 1:
					pts[i].Y = shared
				default:
					pts[i].Z = shared
				}
			}
			sign, certain := proof.OrientSignFloat(pts[0], pts[1], pts[2], pts[3])
			require.True(t, certain)
			require.Zero(t, sign)
			require.Zero(t, proof.OrientSignExact(proof.XptOf(pts[0]), proof.XptOf(pts[1]),
				proof.XptOf(pts[2]), proof.XptOf(pts[3])))
		}
	}
}

// TestOrientSignFloatAgreesWithExact checks that a certain float answer is
// the exact sign, including near-degenerate and subnormal inputs where the
// differences are tiny but not zero.
func TestOrientSignFloatAgreesWithExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x9d2e6b13, 0x41c8f07a))
	tiny := math.Ldexp(1, -1070)
	cases := [][4]r3.Vec{
		// Nearly coplanar: z differs in the last place only.
		{{X: 0, Y: 0, Z: 1}, {X: 1, Y: 0, Z: 1}, {X: 0, Y: 1, Z: 1}, {X: 0.3, Y: 0.3, Z: math.Nextafter(1, 2)}},
		// Subnormal offsets on one axis: not all zero, so no shortcut.
		{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0}, {X: 0.5, Y: 0.5, Z: tiny}},
		{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: tiny}, {X: 0, Y: 1, Z: 0}, {X: 0.5, Y: 0.5, Z: 0}},
		// Two of three differences zero on an axis.
		{{X: 5, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}, {X: 5, Y: 0, Z: 1}, {X: math.Nextafter(5, 6), Y: 1, Z: 1}},
		// Repeated point.
		{{X: 1, Y: 2, Z: 3}, {X: 1, Y: 2, Z: 3}, {X: 4, Y: 5, Z: 6}, {X: 7, Y: 8, Z: 9}},
	}
	for range 400 {
		var c [4]r3.Vec
		for i := range c {
			c[i] = r3.Vec{X: float64(rng.IntN(3)), Y: float64(rng.IntN(3)), Z: float64(rng.IntN(3))}
			if rng.IntN(4) == 0 {
				c[i].Z += tiny * float64(rng.IntN(3))
			}
		}
		cases = append(cases, c)
	}
	for n, c := range cases {
		want := proof.OrientSignExact(proof.XptOf(c[0]), proof.XptOf(c[1]), proof.XptOf(c[2]), proof.XptOf(c[3]))
		if sign, certain := proof.OrientSignFloat(c[0], c[1], c[2], c[3]); certain {
			require.Equal(t, want, sign, "case %d", n)
		}
		require.Equal(t, want, proof.OrientSign(c[0], c[1], c[2], c[3]), "case %d", n)
	}
}

func TestOrientSignFloatUnderflowWrongSign(t *testing.T) {
	m := math.SmallestNonzeroFloat64
	a := r3.Vec{X: 0.25, Y: 0.25, Z: m}
	b := r3.Vec{X: 1, Y: math.Copysign(0, -1)}
	c := r3.Vec{X: 0.98046875, Y: -0.1953125}
	d := r3.Vec{X: 0.923828125, Y: -0.3828125}

	sign, certain := proof.OrientSignFloat(a, b, c, d)
	require.False(t, certain)
	require.Zero(t, sign)
	want := proof.OrientSignExact(proof.XptOf(a), proof.XptOf(b), proof.XptOf(c), proof.XptOf(d))
	require.Equal(t, 1, want)
	require.Equal(t, want, proof.OrientSign(a, b, c, d))
	xa, xb, xc, xd := proof.XptOf(a), proof.XptOf(b), proof.XptOf(c), proof.XptOf(d)
	n := proof.Xcross(proof.Xsub(xb, xa), proof.Xsub(xc, xa))
	require.Equal(t, want, proof.OrientSignPrepared(a, b, c, d, xa, xd, n))
}

func TestOrientSignFloatUnderflowFalseNonzero(t *testing.T) {
	m := math.SmallestNonzeroFloat64
	a := r3.Vec{X: 0.25, Y: 0.25, Z: m}
	b := r3.Vec{X: 1, Y: math.Copysign(0, -1)}
	c := r3.Vec{X: 0.75, Y: -0.25}
	d := r3.Vec{X: 0.5, Y: -0.5}

	sign, certain := proof.OrientSignFloat(a, b, c, d)
	require.False(t, certain)
	require.Zero(t, sign)
	require.Zero(t, proof.OrientSignExact(proof.XptOf(a), proof.XptOf(b), proof.XptOf(c), proof.XptOf(d)))
	require.Zero(t, proof.OrientSign(a, b, c, d))
}

func TestOrientSignFloatGuardEdges(t *testing.T) {
	a := r3.Vec{}
	c := r3.Vec{Y: 1}
	d := r3.Vec{Z: 1}
	low := math.Ldexp(1, -300)
	high := math.Ldexp(1, 250)
	for _, tc := range []struct {
		name    string
		value   float64
		certain bool
	}{
		{name: "lower endpoint", value: low, certain: true},
		{name: "below lower endpoint", value: math.Nextafter(low, 0)},
		{name: "upper endpoint", value: high, certain: true},
		{name: "above upper endpoint", value: math.Nextafter(high, math.Inf(1))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b := r3.Vec{X: tc.value}
			sign, certain := proof.OrientSignFloat(a, b, c, d)
			require.Equal(t, tc.certain, certain)
			if certain {
				want := proof.OrientSignExact(proof.XptOf(a), proof.XptOf(b), proof.XptOf(c), proof.XptOf(d))
				require.Equal(t, want, sign)
			} else {
				require.Zero(t, sign)
			}
		})
	}
}

func TestOrientSignFloatMixedScaleGuardedSample(t *testing.T) {
	rng := rand.New(rand.NewPCG(0x643ce171, 0xa7289b04))
	exponents := [...]int{-300, -250, -150, -20, 0, 20, 150, 249}
	coord := func() float64 {
		if rng.IntN(5) == 0 {
			return 0
		}
		v := math.Ldexp(1+float64(rng.IntN(4))/4, exponents[rng.IntN(len(exponents))])
		if rng.IntN(2) == 0 {
			return -v
		}
		return v
	}
	var certainCount int
	for i := range 512 {
		pts := [4]r3.Vec{{}}
		for j := 1; j < len(pts); j++ {
			pts[j] = r3.Vec{X: coord(), Y: coord(), Z: coord()}
		}
		sign, certain := proof.OrientSignFloat(pts[0], pts[1], pts[2], pts[3])
		if !certain {
			continue
		}
		certainCount++
		want := proof.OrientSignExact(proof.XptOf(pts[0]), proof.XptOf(pts[1]),
			proof.XptOf(pts[2]), proof.XptOf(pts[3]))
		require.Equal(t, want, sign, "case %d", i)
	}
	require.Positive(t, certainCount)
}
