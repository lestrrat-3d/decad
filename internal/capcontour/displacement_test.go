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
// Shown to fail: with joinFoot stepping from the join's held corner along the
// exact unit normal of the held tangent for line walls, the enclosure is a
// single point that misses the denoted foot at both ends.
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
		foot, ok := joinFoot(w, atEnd, span)
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

// TestJoinFootEnclosesDenotedArcNormal pins Displacement's arc and G1 foot
// for a circular wall: the foot steps from the corner the walk's end bound
// encloses along the radius from the recorded centre to that corner. The arc
// runs about the origin between (3, 4) and (−4, 3), so every denoted foot is
// the rational corner ∓ d·corner/5. The walk holds its tangents as math.Sincos
// at math.Atan2 angles, which no float pair can make parallel to (−4, 3) or
// (3, 4) with an exact unit, so a foot read from them misses. Both senses run:
// counterclockwise steps toward the centre and clockwise away from it.
//
// Shown to fail: with joinFoot stepping from the join's held corner along the
// exact unit normal of the held tangent for circular walls again, the
// enclosure misses the denoted foot, first at the counterclockwise wall's
// start.
func TestJoinFootEnclosesDenotedArcNormal(t *testing.T) {
	t.Parallel()
	const d = 0.25
	p := sectionrecord.Point2{U: 3, V: 4}
	q := sectionrecord.Point2{U: -4, V: 3}
	span := proofbound.PointInterval(new(big.Rat).SetFloat64(d))
	for _, ccw := range []bool{true, false} {
		seg := sectionrecord.ArcSeg{Center: sectionrecord.Point2{}, Start: p, End: q, TStart: 0, TEnd: 1}
		if !ccw {
			seg.TStart, seg.TEnd = 1, 0
		}
		w, err := boundarywalk.WalkOf(seg, freeform.NewFreeformWork())
		require.NoError(t, err)
		require.True(t, w.IsCircular())
		for _, atEnd := range []bool{false, true} {
			corner := p
			if atEnd == ccw {
				corner = q
			}
			tu, tv := w.TanInU, w.TanInV
			if atEnd {
				tu, tv = w.TanOutU, w.TanOutV
			}
			cu, cv := new(big.Rat).SetFloat64(corner.U), new(big.Rat).SetFloat64(corner.V)
			// The held tangent is not perpendicular to the radius to its corner.
			radial := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).SetFloat64(tu), cu), new(big.Rat).Mul(new(big.Rat).SetFloat64(tv), cv))
			require.NotZero(t, radial.Sign(), `the fixture needs a held tangent off the denoted one`)

			foot, ok := joinFoot(survey2d.SideWalk{SegmentWalk: w}, atEnd, span)
			require.True(t, ok)
			// corner − s·d·corner/5, s = +1 counterclockwise and −1 clockwise.
			k := big.NewRat(1, 20) // d/5
			if !ccw {
				k.Neg(k)
			}
			wantU := new(big.Rat).Sub(cu, new(big.Rat).Mul(k, cu))
			wantV := new(big.Rat).Sub(cv, new(big.Rat).Mul(k, cv))
			requireEnclosesRat(t, foot.U, wantU, `foot u`)
			requireEnclosesRat(t, foot.V, wantV, `foot v`)
		}
	}
}

func requireEnclosesRat(t *testing.T, iv proofbound.RatInterval, x *big.Rat, what string) {
	t.Helper()
	require.True(t, iv.Lo.Cmp(x) <= 0 && x.Cmp(iv.Hi) <= 0, `%s: [%s, %s] must hold %s`,
		what, iv.Lo.FloatString(20), iv.Hi.FloatString(20), x.FloatString(20))
}
