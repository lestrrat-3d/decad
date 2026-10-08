package capcontour

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// roundedTangentWall walks (24, 32) → (−1e−15, 0) over [tStart, 1]. Over the
// whole range the endpoint difference −24 − 1e−15 rounds to −24, so the walk
// holds the tangent (−24, −32), whose unit vector is an exact point enclosure
// that misses the direction the recorded endpoints denote.
func roundedTangentWall(t *testing.T, tStart float64) survey2d.SideWalk {
	t.Helper()
	w, err := boundarywalk.WalkOf(sectionrecord.LineSeg{
		Start:  sectionrecord.Point2{U: 24, V: 32},
		End:    sectionrecord.Point2{U: -1e-15, V: 0},
		TStart: tStart, TEnd: 1,
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	if tStart == 0 {
		require.Equal(t, [2]float64{-24, -32}, [2]float64{w.TanInU, w.TanInV}, `the fixture needs the rounded tangent`)
	}
	return survey2d.SideWalk{SegmentWalk: w}
}

// exactWallUnit is the wall's denoted unit direction at 400 bits.
func exactWallUnit() (*big.Float, *big.Float) {
	du := new(big.Float).SetPrec(400).Sub(big.NewFloat(-1e-15), big.NewFloat(24))
	dv := new(big.Float).SetPrec(400).SetFloat64(-32)
	l := new(big.Float).SetPrec(400).Add(new(big.Float).Mul(du, du), new(big.Float).Mul(dv, dv))
	l.Sqrt(l)
	return du.Quo(du, l), dv.Quo(dv, l)
}

func requireEncloses(t *testing.T, iv proofbound.RatInterval, x *big.Float, what string) {
	t.Helper()
	lo := new(big.Float).SetPrec(400).SetRat(iv.Lo)
	hi := new(big.Float).SetPrec(400).SetRat(iv.Hi)
	require.True(t, lo.Cmp(x) <= 0 && x.Cmp(hi) <= 0, `%s: [%s, %s] must hold %s`,
		what, lo.Text('g', 20), hi.Text('g', 20), x.Text('g', 20))
}

// TestJoinFootEnclosesDenotedLineNormal pins Displacement's arc and G1 foot
// for a line wall: the foot steps along the normal the recorded endpoints
// denote, at the corner the walk's end bound encloses.
//
// Shown to fail: with joinFoot reading OffsetFootOver(j.VU, j.VV, held
// tangent) for line walls, the enclosure is a single point that misses the
// denoted foot at both ends.
func TestJoinFootEnclosesDenotedLineNormal(t *testing.T) {
	t.Parallel()
	const d = 0.25
	w := roundedTangentWall(t, 0)
	eu, ev := exactWallUnit()
	span := proofbound.PointInterval(new(big.Rat).SetFloat64(d))
	for _, atEnd := range []bool{false, true} {
		cu, cv := w.StartU, w.StartV
		if atEnd {
			cu, cv = w.EndU, w.EndV
		}
		foot, ok := joinFoot(Join{VU: cu, VV: cv}, w, atEnd, span)
		require.True(t, ok)
		// corner + d·(−e.V, e.U)
		wantU := new(big.Float).SetPrec(400).Mul(big.NewFloat(d), ev)
		wantU.Sub(big.NewFloat(cu), wantU)
		wantV := new(big.Float).SetPrec(400).Mul(big.NewFloat(d), eu)
		wantV.Add(wantV, big.NewFloat(cv))
		requireEncloses(t, foot.U, wantU, `foot u`)
		requireEncloses(t, foot.V, wantV, `foot v`)
	}
}
