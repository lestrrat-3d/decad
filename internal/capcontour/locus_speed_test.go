package capcontour

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// TestLineWallFrameEnclosesDenotedWall pins that the frame
// LineCircleLocusSpeedUpper reads holds the wall the record denotes: e and n
// enclose the unit direction and normal of the exact endpoint difference, and
// the anchor encloses the exact lerp of a trimmed start.
//
// Shown to fail: with lineWallFrameOf taking the exact unit vector of the held
// tangent (TanInU, TanInV) as e and the held start as an exact point, e misses
// the denoted direction, and the trimmed wall's anchor misses its denoted
// start.
func TestLineWallFrameEnclosesDenotedWall(t *testing.T) {
	t.Parallel()
	eu, ev := exactWallUnit()
	frame, ok := lineWallFrameOf(roundedTangentWall(t, 0))
	require.True(t, ok)
	requireEncloses(t, frame.e.U, eu, `e.u`)
	requireEncloses(t, frame.e.V, ev, `e.v`)
	requireEncloses(t, frame.n.U, new(big.Float).Neg(ev), `n.u`)
	requireEncloses(t, frame.n.V, eu, `n.v`)

	// The start at t = 1/3 is 24 + (−24 − 1e−15)/3, 32 − 32/3 as reals.
	const tStart = 1.0 / 3
	trimmed := roundedTangentWall(t, tStart)
	frame, ok = lineWallFrameOf(trimmed)
	require.True(t, ok)
	rt := new(big.Rat).SetFloat64(tStart)
	lerp := func(a, b float64) *big.Float {
		ra := new(big.Rat).SetFloat64(a)
		x := new(big.Rat).Mul(new(big.Rat).Sub(new(big.Rat).SetFloat64(b), ra), rt)
		return new(big.Float).SetPrec(400).SetRat(x.Add(x, ra))
	}
	requireEncloses(t, frame.anchor.U, lerp(24, -1e-15), `anchor u`)
	requireEncloses(t, frame.anchor.V, lerp(32, 0), `anchor v`)
}

// TestLineCircleCornerEnclosesDenotedRadius pins that the line-circle locus
// discriminant reads every radius the circle's record denotes. The ArcSeg runs
// about the origin from (1, 1) to (−1, −1), so its record denotes the radius √2
// while the walk holds √2 rounded to float64. The line y = −2 runs along +u, so
// its normal is (0, 1) and α = −2 exactly. The denoted Δ0 = R² − α² is then
// the rational 2 − 4 = −2, and Δ1 = −2(α + R) is −2(√2 − 2), solved at 400
// bits.
//
// Shown to fail: with lineCircleCornerOf reading the held radius as exact
// again, Δ0 and Δ1 are single points at the held radius, which miss both.
func TestLineCircleCornerEnclosesDenotedRadius(t *testing.T) {
	t.Parallel()
	line, err := boundarywalk.WalkOf(sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 0, V: -2},
		End:   sectionrecord.Point2{U: 1, V: -2},
		TEnd:  1,
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	circle, err := boundarywalk.WalkOf(sectionrecord.ArcSeg{
		Start: sectionrecord.Point2{U: 1, V: 1},
		End:   sectionrecord.Point2{U: -1, V: -1},
		TEnd:  1,
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.NotEqual(t, 0, big.NewFloat(circle.Radius).Cmp(new(big.Float).SetPrec(400).Sqrt(big.NewFloat(2))),
		`the fixture needs a held radius off the denoted one`)

	k, ok := lineCircleCornerOf(survey2d.SideWalk{SegmentWalk: line}, survey2d.SideWalk{SegmentWalk: circle})
	require.True(t, ok)
	requireEnclosesRat(t, k.alpha, big.NewRat(-2, 1), `alpha`)
	requireEnclosesRat(t, k.delta0, big.NewRat(-2, 1), `delta0`)
	root2 := new(big.Float).SetPrec(400).Sqrt(big.NewFloat(2))
	want := new(big.Float).SetPrec(400).Sub(root2, big.NewFloat(2))
	want.Mul(want, big.NewFloat(-2))
	requireEncloses(t, k.delta1, want, `delta1`)
}
