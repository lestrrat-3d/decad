package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFloatOrientationPlaneExactSigns(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x315a982c, 0x732c864b))
	exponents := [...]int{-1074, -301, -300, -250, -20, 0, 20, 150, 249, 250, 251, 1000}
	coord := func() float64 {
		return math.Ldexp(float64(rng.IntN(9)-4)/4, exponents[rng.IntN(len(exponents))])
	}
	certainCount := 0
	nonzeroCount := 0
	for i := range 1000 {
		var pts [4]r3.Vec
		for k := range pts {
			pts[k] = r3.NewVec(coord(), coord(), coord())
		}
		plane := proof.NewFloatOrientationPlane(pts[0], pts[1], pts[2])
		// Reusing the plane must neither retain nor change the probe.
		for k := range pts {
			got, certain := plane.Sign(pts[k])
			if !certain {
				continue
			}
			certainCount++
			if got != 0 {
				nonzeroCount++
			}
			want := proof.OrientSignExact(proof.XptOf(pts[0]), proof.XptOf(pts[1]),
				proof.XptOf(pts[2]), proof.XptOf(pts[k]))
			require.Equal(t, want, got, "case %d probe %d", i, k)
		}
	}
	require.Positive(t, certainCount)
	require.Positive(t, nonzeroCount)
}

func TestFloatOrientationPlaneGuards(t *testing.T) {
	t.Parallel()
	for _, value := range []float64{math.SmallestNonzeroFloat64, math.Nextafter(0x1p-300, 0),
		0x1p-300, 1, 0x1p250, math.Nextafter(0x1p250, math.Inf(1))} {
		for axis := range 3 {
			pts := [4]r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
			switch axis {
			case 0:
				pts[1].X = value
			case 1:
				pts[2].Y = value
			default:
				pts[3].Z = value
			}
			want, wantCertain := proof.OrientSignFloat(pts[0], pts[1], pts[2], pts[3])
			plane := proof.NewFloatOrientationPlane(pts[0], pts[1], pts[2])
			got, certain := plane.Sign(pts[3])
			require.Equal(t, wantCertain, certain, "axis %d value %g", axis, value)
			require.Equal(t, want, got, "axis %d value %g", axis, value)
		}
	}
	// Coordinate-plane zero remains exact even when other differences fail
	// the scale guard. Tilted near-coplanar points still require a fallback.
	plane := proof.NewFloatOrientationPlane(r3.Vec{}, r3.Vec{X: 0x1p900}, r3.Vec{Y: 1})
	sign, certain := plane.Sign(r3.Vec{X: 1, Y: 1})
	require.True(t, certain)
	require.Zero(t, sign)
	plane = proof.NewFloatOrientationPlane(r3.Vec{}, r3.Vec{X: 1, Z: 1}, r3.Vec{Y: 1, Z: 1})
	sign, certain = plane.Sign(r3.Vec{X: 0.25, Y: 0.25, Z: math.Nextafter(0.5, 1)})
	require.False(t, certain)
	require.Zero(t, sign)
}
