package coil_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// roundWire is §13's round-wire section: a whole circle of radius 0.5 about
// (3, 0), walked counter-clockwise.
func roundWire() sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 3}, Radius: units.Millimeters(0.5), CCW: true, TStart: 0, TEnd: 1},
	}}
}

func pt2(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }

// slot is a stadium about the line v = 0.5 from u = 2.5 to u = 3.5, radius
// 0.4: two lines and two semicircular arcs, each walked counter-clockwise.
func slot() sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: pt2(2.5, 0.1), End: pt2(3.5, 0.1), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: pt2(3.5, 0.5), Start: pt2(3.5, 0.1), End: pt2(3.5, 0.9), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: pt2(3.5, 0.9), End: pt2(2.5, 0.9), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: pt2(2.5, 0.5), Start: pt2(2.5, 0.9), End: pt2(2.5, 0.1), TStart: 0, TEnd: 1},
	}}
}

// bite is the square ρ ∈ [2, 3.2], ζ ∈ [0, 1] whose right side is a concave
// arc about (3.6, 0.5) through (2.96, 0.5): the arc runs counter-clockwise
// from (3.2, 1) to (3.2, 0), and the loop walks it in reverse.
func bite() sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: pt2(2, 0), End: pt2(3.2, 0), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: pt2(3.6, 0.5), Start: pt2(3.2, 1), End: pt2(3.2, 0), TStart: 1, TEnd: 0},
		sectionrecord.LineSeg{Start: pt2(3.2, 1), End: pt2(2, 1), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: pt2(2, 1), End: pt2(2, 0), TStart: 0, TEnd: 1},
	}}
}

func TestLoopsChordsArcs(t *testing.T) {
	t.Run("round wire", func(t *testing.T) {
		prof, err := coil.Loops(roundWire(), nil, 16)
		require.NoError(t, err)
		require.Len(t, prof.Pts, 16)
		require.Len(t, prof.Segments, 1)
		seg := prof.Segments[0]
		require.True(t, seg.Closed)
		require.Equal(t, 16, seg.Chords)
		require.Equal(t, sectionrecord.Point2{U: 3.5}, prof.Pts[0])
		for v, p := range prof.Pts {
			th := 2 * math.Pi * float64(v) / 16
			require.LessOrEqual(t, math.Hypot(p.U-(3+0.5*math.Cos(th)), p.V-0.5*math.Sin(th)), prof.Round[v]+1e-15, "station %d", v)
			require.Less(t, prof.Round[v], 1e-15)
			// r·Δφ²/8 with Δφ = 2π/16.
			sag, _ := prof.Chord[v].Sag.Float64()
			require.InEpsilon(t, 0.5*(2*math.Pi/16)*(2*math.Pi/16)/8, sag, 1e-12)
		}
		require.Equal(t, 0, prof.Next(0))
		require.Equal(t, 0, prof.Prev(0))
	})
	t.Run("slot", func(t *testing.T) {
		prof, err := coil.Loops(slot(), nil, 16)
		require.NoError(t, err)
		// Each semicircle takes 8 chords: 1 + 8 + 1 + 8 stations.
		require.Len(t, prof.Pts, 18)
		require.Len(t, prof.Segments, 4)
		require.Equal(t, []int{0, 1, 9, 10}, []int{prof.Segments[0].First, prof.Segments[1].First, prof.Segments[2].First, prof.Segments[3].First})
		require.False(t, prof.Segments[1].Reversed)
		require.Equal(t, sectionrecord.Point2{U: 2.5, V: 0.9}, prof.Pts[10])
		for v, p := range prof.Pts[2:9] {
			th := -math.Pi/2 + math.Pi*float64(v+1)/8
			require.LessOrEqual(t, math.Hypot(p.U-(3.5+0.4*math.Cos(th)), p.V-(0.5+0.4*math.Sin(th))), prof.Round[v+2]+1e-15)
		}
		// Every recorded end lies on its circle to the float's own rounding,
		// so the junctions carry at most the radial residual.
		for _, s := range prof.Segments {
			require.Less(t, prof.Round[s.First], 1e-15)
		}
		require.Equal(t, 1, prof.Chord[1].Segment)
		require.Equal(t, 3, prof.Chord[17].Segment)
		require.Nil(t, prof.Chord[0].Sag)
		require.Equal(t, 1, prof.Next(0))
		rev, err := coil.Loops(bite(), nil, 16)
		require.NoError(t, err)
		require.True(t, rev.Segments[1].Reversed)
		// The reversed arc walks from its End and takes ⌈16·0.285⌉ = 5 chords.
		require.Equal(t, sectionrecord.Point2{U: 3.2}, rev.Pts[1])
		require.Equal(t, 5, rev.Segments[1].Chords)
		require.Equal(t, sectionrecord.Point2{U: 3.2, V: 1}, rev.Pts[rev.Segments[2].First])
		require.Equal(t, 0, prof.Next(3))
		require.Equal(t, 3, prof.Prev(0))
		require.True(t, prof.HasArcs())
	})
	t.Run("refusals", func(t *testing.T) {
		trimmed := slot()
		arc := trimmed.Segments[1].(sectionrecord.ArcSeg)
		arc.TEnd = 0.5
		trimmed.Segments[1] = arc
		_, err := coil.Loops(trimmed, nil, 16)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)

		half := roundWire()
		c := half.Segments[0].(sectionrecord.CircleSeg)
		c.TEnd = 0.5
		half.Segments[0] = c
		_, err = coil.Loops(half, nil, 16)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)

		ellipse := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.EllipseSeg{Center: sectionrecord.Point2{U: 3}, Rx: units.Millimeters(0.5), Ry: units.Millimeters(0.3),
				Rotation: units.Degrees(0), CCW: true, TStart: 0, TEnd: 1},
		}}
		_, err = coil.Loops(ellipse, nil, 16)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}

// wallIntegrand is √h at angle α of a circle of radius r about (cu, 0) beside
// the V axis, where ρ = cu + r·cos α and the axial part of the slope is
// k·r·sin α.
func wallIntegrand(cu, r, k, alpha float64) float64 {
	rho := cu + r*math.Cos(alpha)
	s := math.Sin(alpha)
	return math.Sqrt(r*r*rho*rho + k*k*r*r*s*s)
}

// TestArcAreaEnclosesTheQuadrature checks docs/helix-design.md §11.1's
// bracket against independent quadratures of Θ·∫√h dα. The whole circle's
// integrand is periodic and analytic, so the trapezoid rule over 8192 points
// converges far below float64's own rounding; the semicircle reads composite
// Simpson over 20000 intervals, whose error, (π/20000)⁴·max|f⁗|·π/180, is
// below 1e-15 here. Each sample is a float64 evaluation, charged 1e-14
// relative. The quadrature is a falsifier, never the proof.
//
// Leg shown to fail by deleting it and watching this test go red, then
// restoring it: the Taylor remainder |h − g²|³/(16·h_lo^{5/2}). At 1024
// pieces the odd remainder cancels to below the quadrature's own rounding,
// so the four-piece bracket of the whole circle is what went red.
func TestArcAreaEnclosesTheQuadrature(t *testing.T) {
	pitch, turns := big.NewRat(3, 2), big.NewRat(5, 1)
	k := 1.5 / (2 * math.Pi)
	theta := 2 * math.Pi * 5

	requireBracket := func(t *testing.T, got coil.Iv, want float64) {
		t.Helper()
		lo, _ := got.Lo.Float64()
		hi, _ := got.Hi.Float64()
		slack := 1e-14 * want
		require.LessOrEqual(t, lo, want+slack)
		require.GreaterOrEqual(t, hi, want-slack)
		require.Less(t, (hi-lo)/want, 1e-9, "the bracket is narrower than 1e-9 relative")
	}

	t.Run("whole circle", func(t *testing.T) {
		prof, err := coil.Loops(roundWire(), nil, 16)
		require.NoError(t, err)
		got, ok, err := coil.ArcArea(t.Context(), prof.Segments[0], vAxis(), pitch, turns)
		require.NoError(t, err)
		require.True(t, ok)
		const n = 8192
		sum := new(big.Float).SetPrec(256)
		for i := range n {
			sum.Add(sum, big.NewFloat(wallIntegrand(3, 0.5, k, 2*math.Pi*float64(i)/n)))
		}
		f, _ := sum.Float64()
		want := theta * f * 2 * math.Pi / n
		requireBracket(t, got, want)

		// Over four pieces the expansion is far from the integral, and only
		// the remainder keeps the bracket around it.
		coarse, ok, err := coil.ArcAreaWith(t.Context(), prof.Segments[0], vAxis(), pitch, turns, 4)
		require.NoError(t, err)
		require.True(t, ok)
		lo, _ := coarse.Lo.Float64()
		hi, _ := coarse.Hi.Float64()
		require.LessOrEqual(t, lo, want)
		require.GreaterOrEqual(t, hi, want)
		require.Greater(t, (hi-lo)/want, 1e-9)
	})
	t.Run("semicircle", func(t *testing.T) {
		prof, err := coil.Loops(slot(), nil, 16)
		require.NoError(t, err)
		got, ok, err := coil.ArcArea(t.Context(), prof.Segments[1], vAxis(), pitch, turns)
		require.NoError(t, err)
		require.True(t, ok)
		const n = 20000
		step := math.Pi / n
		sum := new(big.Float).SetPrec(256)
		for i := 0; i <= n; i++ {
			w := 2.0
			switch {
			case i == 0 || i == n:
				w = 1
			case i%2 == 1:
				w = 4
			}
			sum.Add(sum, big.NewFloat(w*wallIntegrand(3.5, 0.4, k, -math.Pi/2+step*float64(i))))
		}
		f, _ := sum.Float64()
		requireBracket(t, got, theta*f*step/3)

		// The bite's arc about (3.6, 0.5), radius √0.41, walked in reverse,
		// covers the same angles as its forward sweep.
		rev, err := coil.Loops(bite(), nil, 16)
		require.NoError(t, err)
		back, ok, err := coil.ArcArea(t.Context(), rev.Segments[1], vAxis(), pitch, turns)
		require.NoError(t, err)
		require.True(t, ok)
		a0, a1 := math.Atan2(0.5, -0.4), math.Atan2(-0.5, -0.4)+2*math.Pi
		r := math.Sqrt(0.41)
		step = (a1 - a0) / n
		sum = new(big.Float).SetPrec(256)
		for i := 0; i <= n; i++ {
			w := 2.0
			switch {
			case i == 0 || i == n:
				w = 1
			case i%2 == 1:
				w = 4
			}
			sum.Add(sum, big.NewFloat(w*wallIntegrand(3.6, r, k, a0+step*float64(i))))
		}
		f, _ = sum.Float64()
		requireBracket(t, back, theta*f*step/3)
	})
}

// TestCellProofArcLegs pins the arc chord's charges in CellProofUpper: each
// term grows with the chord's Sag and Arc, and a zero chord reads the line
// cell unchanged.
func TestCellProofArcLegs(t *testing.T) {
	pt := func(x float64) coil.Iv { return coil.Point(new(big.Rat).SetFloat64(x)) }
	pitch, dt := big.NewRat(3, 2), big.NewRat(1, 256)
	line, ok := coil.CellProofUpper(pt(3), pt(0), pt(3.2), pt(0.1), coil.Chord{}, pitch, dt)
	require.True(t, ok)
	sag, arc := big.NewRat(1, 1000), big.NewRat(3, 10)
	chord, ok := coil.CellProofUpper(pt(3), pt(0), pt(3.2), pt(0.1), coil.Chord{Sag: sag, Arc: arc}, pitch, dt)
	require.True(t, ok)
	require.Greater(t, chord.Swept, line.Swept)
	require.Greater(t, chord.Density, line.Density)
	require.Greater(t, chord.Helix, line.Helix)
	require.InDelta(t, 0.3, chord.Ruling, 1e-15)
	require.GreaterOrEqual(t, chord.Ruling, 0.3)

	dep, ok := coil.CellDepartureUpper(pt(3), pt(3.2), pitch, dt)
	require.True(t, ok)
	require.Positive(t, dep.Sign())
}
