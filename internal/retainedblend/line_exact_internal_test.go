package retainedblend

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// squareInput is the chamfer of the edge along +Y at x = −5, z = 0 between the faces z = 0 (A, toward −X) and
// x = −5 (B, toward +Z), over the end planes y = 0 and y = 16.
func squareInput() chamferFeetInput {
	return chamferFeetInput{
		v:     [2]r3.Vec{{X: -5, Y: 0, Z: 0}, {X: -5, Y: 16, Z: 0}},
		onA:   [2]r3.Vec{{X: -10, Y: 0, Z: 0}, {X: -10, Y: 16, Z: 0}},
		onB:   [2]r3.Vec{{X: -5, Y: 0, Z: 6}, {X: -5, Y: 16, Z: 6}},
		intoA: r3.Vec{X: -1},
		intoB: r3.Vec{Z: 1},
		tA:    0.5,
		tB:    0.5,
	}
}

func requireFoot(t *testing.T, f lineFoot, want r3.Vec) {
	t.Helper()
	require.Equal(t, want, f.p)
	require.Equal(t, 0.0, f.bound)
}

// ratDist2 is the squared distance of the held point p from the exact point want.
func ratDist2(p r3.Vec, want [3]*big.Rat) *big.Rat {
	got := ratVecOf(p)
	sum := new(big.Rat)
	for i := range got {
		d := new(big.Rat).Sub(got[i], want[i])
		sum.Add(sum, d.Mul(d, d))
	}
	return sum
}

func TestExactChamferFeet(t *testing.T) {
	t.Run("square", func(t *testing.T) {
		feet, ok := exactChamferFeet(squareInput())
		require.True(t, ok)
		requireFoot(t, feet[0][0], r3.Vec{X: -5.5, Y: 0, Z: 0})
		requireFoot(t, feet[1][0], r3.Vec{X: -5.5, Y: 16, Z: 0})
		requireFoot(t, feet[0][1], r3.Vec{X: -5, Y: 0, Z: 0.5})
		requireFoot(t, feet[1][1], r3.Vec{X: -5, Y: 16, Z: 0.5})
	})
	t.Run("pythagorean", func(t *testing.T) {
		in := chamferFeetInput{
			v:     [2]r3.Vec{{}, {Z: 16}},
			onA:   [2]r3.Vec{{X: 6, Y: 8}, {X: 6, Y: 8, Z: 16}},
			onB:   [2]r3.Vec{{X: -8, Y: 6}, {X: -8, Y: 6, Z: 16}},
			intoA: r3.Vec{X: 0.6, Y: 0.8},
			intoB: r3.Vec{X: -0.8, Y: 0.6},
			tA:    1.25,
			tB:    1.25,
		}
		feet, ok := exactChamferFeet(in)
		require.True(t, ok)
		requireFoot(t, feet[0][0], r3.Vec{X: 0.75, Y: 1, Z: 0})
		requireFoot(t, feet[1][0], r3.Vec{X: 0.75, Y: 1, Z: 16})
		requireFoot(t, feet[0][1], r3.Vec{X: -1, Y: 0.75, Z: 0})
		requireFoot(t, feet[1][1], r3.Vec{X: -1, Y: 0.75, Z: 16})
	})
	t.Run("tilted", func(t *testing.T) {
		in := chamferFeetInput{
			v:     [2]r3.Vec{{X: 5}, {X: 5, Y: 16}},
			onA:   [2]r3.Vec{{X: 10}, {X: 10, Y: 16}},
			onB:   [2]r3.Vec{{X: 6, Z: 6}, {X: 6, Y: 16, Z: 6}},
			intoA: r3.Vec{X: 1},
			intoB: r3.Vec{X: 1, Z: 6},
			tA:    0.5,
			tB:    0.5,
		}
		feet, ok := exactChamferFeet(in)
		require.True(t, ok)
		requireFoot(t, feet[0][0], r3.Vec{X: 5.5})
		requireFoot(t, feet[1][0], r3.Vec{X: 5.5, Y: 16})
		// B's direction is (1, 0, 6)/√37, so its foot is (5 + 0.5/√37, y, 3/√37).
		root := new(big.Float).SetPrec(512).Sqrt(new(big.Float).SetPrec(512).SetInt64(37))
		half := new(big.Float).SetPrec(512).SetFloat64(0.5)
		dx := new(big.Float).SetPrec(512).Quo(half, root)
		dz := new(big.Float).SetPrec(512).Quo(new(big.Float).SetPrec(512).SetInt64(3), root)
		for i, y := range []float64{0, 16} {
			f := feet[i][1]
			require.Equal(t, y, f.p.Y)
			require.Greater(t, f.bound, 0.0)
			require.LessOrEqual(t, f.bound, 1e-15)
			ex := new(big.Float).SetPrec(512).Add(new(big.Float).SetPrec(512).SetInt64(5), dx)
			gx := new(big.Float).SetPrec(512).Sub(new(big.Float).SetPrec(512).SetFloat64(f.p.X), ex)
			gz := new(big.Float).SetPrec(512).Sub(new(big.Float).SetPrec(512).SetFloat64(f.p.Z), dz)
			d2 := new(big.Float).SetPrec(512).Add(new(big.Float).SetPrec(512).Mul(gx, gx),
				new(big.Float).SetPrec(512).Mul(gz, gz))
			dist, _ := new(big.Float).SetPrec(512).Sqrt(d2).Float64()
			require.LessOrEqual(t, dist, f.bound)
		}
	})
	t.Run("oblique end plane", func(t *testing.T) {
		in := squareInput()
		in.onA[0] = r3.Vec{X: -8, Y: 1, Z: 0}
		in.onB[0] = r3.Vec{X: -5, Y: -2, Z: 6}
		feet, ok := exactChamferFeet(in)
		require.True(t, ok)
		third := new(big.Rat).SetFrac64(1, 6)
		wantA := [3]*big.Rat{big.NewRat(-11, 2), third, new(big.Rat)}
		wantB := [3]*big.Rat{big.NewRat(-5, 1), new(big.Rat).Neg(third), big.NewRat(1, 2)}
		for i, c := range []struct {
			f    lineFoot
			want [3]*big.Rat
		}{{feet[0][0], wantA}, {feet[0][1], wantB}} {
			require.Greater(t, c.f.bound, 0.0, "foot %d", i)
			d2 := ratDist2(c.f.p, c.want)
			b2 := new(big.Rat).SetFloat64(c.f.bound)
			require.LessOrEqual(t, d2.Cmp(b2.Mul(b2, b2)), 0, "foot %d", i)
		}
		requireFoot(t, feet[1][0], r3.Vec{X: -5.5, Y: 16, Z: 0})
		requireFoot(t, feet[1][1], r3.Vec{X: -5, Y: 16, Z: 0.5})
	})
	t.Run("degenerate", func(t *testing.T) {
		on := squareInput()
		on.onA[0] = r3.Vec{X: -5, Y: 8, Z: 0}
		_, ok := exactChamferFeet(on)
		require.False(t, ok, "carrier on the edge line")

		square := squareInput()
		square.intoA = r3.Vec{Z: 1}
		_, ok = exactChamferFeet(square)
		require.False(t, ok, "way into A square to the carrier direction")

		wide := squareInput()
		wide.intoA = r3.Vec{X: -1, Z: 1}
		_, ok = exactChamferFeet(wide)
		require.False(t, ok, "way into A 45 degrees off")

		flat := squareInput()
		flat.onB[0] = r3.Vec{X: -5, Y: 0, Z: 0}
		_, ok = exactChamferFeet(flat)
		require.False(t, ok, "carrier at the edge end")

		nan := squareInput()
		nan.tA = math.NaN()
		_, ok = exactChamferFeet(nan)
		require.False(t, ok, "non-finite setback")
	})
	t.Run("sense picks the direction", func(t *testing.T) {
		in := squareInput()
		in.intoA = r3.Vec{X: 2}
		feet, ok := exactChamferFeet(in)
		require.True(t, ok)
		requireFoot(t, feet[0][0], r3.Vec{X: -4.5})
	})
}
