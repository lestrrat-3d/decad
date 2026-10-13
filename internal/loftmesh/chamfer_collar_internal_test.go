package loftmesh

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferMiddleHeldRootInsideClipsToSlab(t *testing.T) {
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0},
		{X: 0.5, Y: 0, Z: 0},
		{X: 0, Y: 0.5, Z: 1},
		{X: 3, Y: 0, Z: -1},
	}
	tris := [][3]int{{0, 1, 2}, {0, 1, 3}}
	zero, one, minusOne := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(-1, 1)

	// The far vertex lies below the slab, so the clip removes it.
	require.True(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, zero, one, 1))
	// A slab reaching down to the far vertex keeps it, outside the disk.
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, minusOne, one, 1))

	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, -1, zero, one, 1))
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 3, 0, zero, one, 1))
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, one, one, 1))
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, zero, one, 0))
}

// legacyHeldRootInside is the big.Rat form of ChamferMiddleHeldRootInside.
// It is the reference the integer form must match on every input.
func legacyHeldRootInside(verts []r3.Vec, tris [][3]int, walls, root int,
	zLo, zHi *big.Rat, radius float64) bool {
	if root < 0 || walls > len(tris) || zLo == nil || zHi == nil ||
		zLo.Cmp(zHi) >= 0 || radius <= 0 || proofbound.IsNonFinite(radius) {
		return false
	}
	r := proofarith.FloatRat(radius)
	r2 := new(big.Rat).Mul(r, r)
	for j := 2 * root; j < walls; j++ {
		tri := tris[j]
		poly := []proofarith.Xpt{
			proofarith.XptOf(verts[tri[0]]),
			proofarith.XptOf(verts[tri[1]]),
			proofarith.XptOf(verts[tri[2]]),
		}
		poly = legacyClipZ(poly, zLo, true)
		poly = legacyClipZ(poly, zHi, false)
		for _, p := range poly {
			x, y := meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1)
			radius2 := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
			if radius2.Cmp(r2) >= 0 {
				return false
			}
		}
	}
	return true
}

func legacyClipZ(poly []proofarith.Xpt, level *big.Rat, keepAbove bool) []proofarith.Xpt {
	if len(poly) == 0 {
		return nil
	}
	inside := func(p proofarith.Xpt) bool {
		cmp := meshbool.RatCoordOf(p, 2).Cmp(level)
		return keepAbove && cmp >= 0 || !keepAbove && cmp <= 0
	}
	var out []proofarith.Xpt
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		inA, inB := inside(a), inside(b)
		if inA != inB {
			za, zb := meshbool.RatCoordOf(a, 2), meshbool.RatCoordOf(b, 2)
			t := new(big.Rat).Quo(new(big.Rat).Sub(level, za), new(big.Rat).Sub(zb, za))
			out = append(out, proofarith.Xlerp(a, b, t.Num(), t.Denom()))
		}
		if inB {
			out = append(out, b)
		}
	}
	return out
}

func TestChamferMiddleHeldRootInsideMatchesRational(t *testing.T) {
	rng := rand.New(rand.NewPCG(13, 17))
	pow3 := new(big.Int).Exp(big.NewInt(3), big.NewInt(20), nil)
	pow3b := new(big.Int).Exp(big.NewInt(3), big.NewInt(21), nil)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 60))
	half := proofarith.FloatRat(0.5)
	levels := []*big.Rat{
		big.NewRat(1, 3), big.NewRat(2, 5), big.NewRat(1, 4), half,
		new(big.Rat).Add(half, tiny), new(big.Rat).Sub(half, tiny),
		new(big.Rat).SetFrac(new(big.Int).Add(pow3, big.NewInt(1)), pow3b),
		new(big.Rat).SetFrac(pow3, new(big.Int).Add(pow3b, big.NewInt(1))),
		big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(-1, 3), big.NewRat(7, 5),
	}
	zs := []float64{0, 1, 0.5, 1.0 / 3.0, 0.4, 0.25, -0.5}
	radii := []float64{4, math.Nextafter(4, math.Inf(1)), math.Nextafter(4, math.Inf(-1)), 3.9, 4.1}
	scales := []float64{1, 1, 1, 0x1p40, 0x1p-40}
	trueN, falseN := 0, 0
	const cases = 250
	for range cases {
		scale := scales[rng.IntN(len(scales))]
		n := 40 + rng.IntN(81)
		verts := make([]r3.Vec, n)
		for i := range verts {
			rad := 4.0
			switch rng.IntN(4) {
			case 0:
				rad = 3.9 + 0.2*rng.Float64()
			case 1:
				rad = 3 * rng.Float64()
			}
			ang := 2 * math.Pi * rng.Float64()
			z := zs[rng.IntN(len(zs))]
			if rng.IntN(4) == 0 {
				z = rng.Float64()
			}
			verts[i] = r3.Vec{X: rad * math.Cos(ang) * scale, Y: rad * math.Sin(ang) * scale, Z: z * scale}
		}
		tris := make([][3]int, n)
		for i := range tris {
			a, b, c := rng.IntN(n), rng.IntN(n), rng.IntN(n)
			switch rng.IntN(5) {
			case 0: // degenerate: repeated vertex
				c = a
			case 1: // an edge in a level plane
				verts[b].Z = verts[a].Z
			}
			tris[i] = [3]int{a, b, c}
		}
		i, j := rng.IntN(len(levels)), rng.IntN(len(levels))
		lo, hi := levels[min(i, j)], levels[max(i, j)]
		sc := proofarith.FloatRat(scale)
		loS, hiS := new(big.Rat).Mul(lo, sc), new(big.Rat).Mul(hi, sc)
		radius := radii[rng.IntN(len(radii))] * scale
		root := []int{0, 1, 3}[rng.IntN(3)]
		walls := len(tris)
		if rng.IntN(4) == 0 {
			walls = len(tris) - rng.IntN(10)
		}
		want := legacyHeldRootInside(verts, tris, walls, root, loS, hiS, radius)
		got := ChamferMiddleHeldRootInside(verts, tris, walls, root, loS, hiS, radius)
		require.Equal(t, want, got)
		if want {
			trueN++
		} else {
			falseN++
		}
	}
	// Confirm both answers occur in the sample.
	require.Positive(t, falseN)
	t.Logf("true=%d false=%d", trueN, falseN)

	// Slab-contained sets exercise the true path: every vertex strictly inside.
	for range 40 {
		n := 6 + rng.IntN(20)
		verts := make([]r3.Vec, n)
		for i := range verts {
			ang := 2 * math.Pi * rng.Float64()
			verts[i] = r3.Vec{X: 3.99 * math.Cos(ang), Y: 3.99 * math.Sin(ang), Z: rng.Float64()}
		}
		tris := make([][3]int, n)
		for i := range tris {
			tris[i] = [3]int{rng.IntN(n), rng.IntN(n), rng.IntN(n)}
		}
		lo, hi := big.NewRat(1, 7), big.NewRat(5, 6)
		for _, radius := range []float64{4, 3.99} {
			require.Equal(t,
				legacyHeldRootInside(verts, tris, n, 0, lo, hi, radius),
				ChamferMiddleHeldRootInside(verts, tris, n, 0, lo, hi, radius))
		}
	}

	// Vertices exactly on the circle: the strict comparison must refuse, and
	// a triangle just inside it must pass, in both forms.
	onCircle := []r3.Vec{{X: 4, Z: 0.25}, {Y: 4, Z: 0.75}, {X: -4, Z: 0.5}, {X: 3, Y: 4, Z: 0.5},
		{X: 3, Y: 3.9, Z: 0.5}, {X: 1, Y: 1, Z: 0.5}, {X: 0, Y: 0, Z: 2}, {X: 4, Z: 2}}
	lo, hi := big.NewRat(1, 4), big.NewRat(3, 4)
	for _, radius := range []float64{4, 5, math.Nextafter(5, 6), math.Nextafter(5, 4)} {
		for a := range onCircle {
			for b := range onCircle {
				for c := range onCircle {
					tr := [][3]int{{a, b, c}}
					want := legacyHeldRootInside(onCircle, tr, 1, 0, lo, hi, radius)
					require.Equal(t, want, ChamferMiddleHeldRootInside(onCircle, tr, 1, 0, lo, hi, radius))
					if want {
						trueN++
					}
				}
			}
		}
	}
	require.Positive(t, trueN)

	// Guards.
	verts := []r3.Vec{{X: 1}, {X: 1, Y: 1}, {Y: 1, Z: 1}}
	tris := [][3]int{{0, 1, 2}}
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	for _, c := range []struct {
		walls, root int
		lo, hi      *big.Rat
		radius      float64
	}{
		{1, -1, zero, one, 4}, {2, 0, zero, one, 4}, {1, 0, one, one, 4}, {1, 0, one, zero, 4},
		{1, 0, nil, one, 4}, {1, 0, zero, nil, 4}, {1, 0, zero, one, 0}, {1, 0, zero, one, -1},
		{1, 0, zero, one, math.NaN()}, {1, 0, zero, one, math.Inf(1)}, {1, 1, zero, one, 4},
		{1, 0, zero, one, 4},
	} {
		require.Equal(t,
			legacyHeldRootInside(verts, tris, c.walls, c.root, c.lo, c.hi, c.radius),
			ChamferMiddleHeldRootInside(verts, tris, c.walls, c.root, c.lo, c.hi, c.radius))
	}
}
