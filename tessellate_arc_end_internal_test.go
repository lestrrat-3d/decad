package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvesampling"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds a mesh vertex at an arc's natural t = 1 end to the point the
// arc denotes there: Start's radius at End's angle (docs/evaluator-design.md
// §4, docs/tessellation-design.md §5's deltaStore). Each case draws a sketch
// arc about the origin whose End sits 500 ulps past Start's radius, inside the
// arc preflight's 1024-ulp allowance, and never solves the sketch, so the
// record keeps the drawn End. The chord ending there holds End verbatim, about
// 1.8e-12 mm from the denoted point, so the sample's own bound must reach it.
//
// Shown-to-fail, each leg alone: with tessellation.SampleLoop charging each
// sample its own walk's StartBound (no JunctionStartBound), the prism case's
// sample at End reads a zero bound; with SampleCapBlend's side ring doing the
// same, the cap-blend case's does; with brepChordWall charging its end sample
// the walk's own EndBound (no DenotedEndBound), the brep case's vertex at End
// does; and with revolvesampling.MeridianJunctions enclosing the walk's own StartBound, the
// revolve junction at End encloses only End's own z.

// arcEndOffRadius is v moved 500 ulps away from zero.
func arcEndOffRadius(v float64) float64 {
	for range 500 {
		v = math.Nextafter(v, 2*v)
	}
	return v
}

// arcEndPie draws the quarter disk of radius 20 about the origin, its arc from
// (20, 0) to (0, 20) with End's v off the radius, and extrudes it 10 mm.
func arcEndPie(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	start := s.CreatePoint(20, 0)
	end := s.CreatePoint(0, arcEndOffRadius(20))
	s.CreateLine(o, start)
	s.CreateArc(o, start, end)
	s.CreateLine(end, o)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	return body
}

// arcDenotedEnd is the point arc denotes at t = 1, at 256 bits: Center plus
// Start's radius along End's direction from Center.
func arcDenotedEnd(arc arcSeg) (*big.Float, *big.Float) {
	const prec = 256
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	hypot := func(p Point2) *big.Float {
		du := new(big.Float).SetPrec(prec).Sub(f(p.U), f(arc.Center.U))
		dv := new(big.Float).SetPrec(prec).Sub(f(p.V), f(arc.Center.V))
		du.Mul(du, du)
		dv.Mul(dv, dv)
		return du.Add(du, dv).Sqrt(du)
	}
	scale := new(big.Float).SetPrec(prec).Quo(hypot(arc.Start), hypot(arc.End))
	du := new(big.Float).SetPrec(prec).Sub(f(arc.End.U), f(arc.Center.U))
	dv := new(big.Float).SetPrec(prec).Sub(f(arc.End.V), f(arc.Center.V))
	du.Mul(du, scale).Add(du, f(arc.Center.U))
	dv.Mul(dv, scale).Add(dv, f(arc.Center.V))
	return du, dv
}

// requireReachesArcEnd asserts that bound reaches arc's denoted t = 1 end from
// the held point (u, v) in each component, and returns the larger gap.
func requireReachesArcEnd(t *testing.T, arc arcSeg, u, v float64, bound proofbound.WalkEndBound) float64 {
	t.Helper()
	du, dv := arcDenotedEnd(arc)
	gapU, _ := du.Sub(du, big.NewFloat(u)).Abs(du).Float64()
	gapV, _ := dv.Sub(dv, big.NewFloat(v)).Abs(dv).Float64()
	require.LessOrEqualf(t, gapU, bound.U, `held (%v, %v) is %.3e off the denoted end in u, bound %.3e`, u, v, gapU, bound.U)
	require.LessOrEqualf(t, gapV, bound.V, `held (%v, %v) is %.3e off the denoted end in v, bound %.3e`, u, v, gapV, bound.V)
	return math.Max(gapU, gapV)
}

// onlyArc returns the one arc of segs.
func onlyArc(t *testing.T, segs []curveSegment) arcSeg {
	t.Helper()
	var arcs []arcSeg
	for _, seg := range segs {
		if arc, ok := seg.(arcSeg); ok {
			arcs = append(arcs, arc)
		}
	}
	require.Len(t, arcs, 1)
	require.Equal(t, [2]float64{0, 1}, [2]float64{arcs[0].TStart, arcs[0].TEnd}, `the arc is whole and counter-clockwise`)
	require.NotEqual(t, math.Hypot(arcs[0].Start.U, arcs[0].Start.V), math.Hypot(arcs[0].End.U, arcs[0].End.V),
		`End is off Start's radius`)
	return arcs[0]
}

// TestChordLoopSampleReachesArcNaturalEnd chords the pie prism's loop, the
// chording a prism, a cup and a stacked prism mesh their rings from: the
// sample at End, where the line to the origin starts, must reach the point
// the arc denotes there, and the samples at the two recorded corners both
// neighbours state keep their zero bound.
func TestChordLoopSampleReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	pie := arcEndPie(t, New())
	pp, ok := pie.payload.(prismPayload)
	require.True(t, ok, `got %T`, pie.payload)
	arc := onlyArc(t, pp.profile.Outer.Segments)
	face := &Face{}
	cl, err := tessellation.ChordLoop(t.Context(), pp.profile.Outer, 0.2, 10, freeform.NewFreeformWork(), nil, 0,
		func(survey2d.SideWalk) (*Face, error) { return face, nil }, stationbound.ChordStationBound)
	require.NoError(t, err)
	found := false
	for j, p := range cl.Samples {
		switch p {
		case arc.End:
			found = true
			gap := requireReachesArcEnd(t, arc, p.U, p.V, cl.BoundOf[j])
			require.Greater(t, gap, 1e-12, `End sits well off the denoted point`)
		case arc.Start, Point2{}:
			require.Equalf(t, proofbound.WalkEndBound{}, cl.BoundOf[j], `the recorded corner %v stays exact`, p)
		}
	}
	require.True(t, found, `one sample sits at End`)
}

// arcEndTombstone runs from (20, −30) up to (20, 0), along the quarter arc
// about the origin to E = (0, 20) with E's v off the radius, across to
// (−40, E.v) and around (−40, −30), extruded 10 mm, and cuts it with a
// radius-2 drill along y through (−30, ·, 5), clear of the arc's whole-circle
// box, so the pair builds class B's through reach: a brep body that keeps the
// arc.
func arcEndTombstone(t *testing.T) *Body {
	t.Helper()
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	top := arcEndOffRadius(20)
	o := s.CreatePoint(0, 0)
	a := s.CreatePoint(20, -30)
	b := s.CreatePoint(20, 0)
	e := s.CreatePoint(0, top)
	f := s.CreatePoint(-40, top)
	g := s.CreatePoint(-40, -30)
	s.CreateLine(a, b)
	s.CreateArc(o, b, e)
	s.CreateLine(e, f)
	s.CreateLine(f, g)
	s.CreateLine(g, a)
	tomb, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	dw := sketch.NewWorld()
	ds, err := dw.CreateSketch(dw.XZ())
	require.NoError(t, err)
	c := ds.CreatePoint(-30, 5)
	ds.Fix(c)
	ds.CreateCircle(c, 2)
	drill, err := doc.Extrude(ds, ds.Profiles()[0], Symmetric{D: units.Millimeters(40)})
	require.NoError(t, err)
	cut, err := Cut(t.Context(), tomb, drill)
	require.NoError(t, err)
	return cut
}

// TestBrepChordWallReachesArcNaturalEnd chords the drilled tombstone's arc
// face as the brep mesh does. The vertices at End, at both levels, must reach
// the point the arc denotes there.
func TestBrepChordWallReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	got := arcEndTombstone(t)
	bp, ok := got.payload.(brepPayload)
	require.True(t, ok, `got %T`, got.payload)
	topo, err := brepTopologyContext(t.Context(), bp)
	require.NoError(t, err)
	checked := 0
	for fi, f := range bp.faces {
		arc, ok := f.wall.(arcSeg)
		if f.planar() || !ok {
			continue
		}
		require.Equal(t, 1.0, arc.TEnd, `the wall keeps the arc's natural end`)
		w, e := topo.walls[fi], topo.embeds[fi]
		bounds := map[[3]float64]proofbound.WalkEndBound{}
		var index int
		capture := func(c [3]float64, b proofbound.WalkEndBound) int {
			prev := bounds[c]
			bounds[c] = proofbound.WalkEndBound{U: math.Max(prev.U, b.U), V: math.Max(prev.V, b.V)}
			index++
			return index - 1
		}
		_, err := brepChordWall(t.Context(), f, w, e, nil, 0.2, freeform.NewFreeformWork(), nil, capture)
		require.NoError(t, err)
		for _, z := range []float64{f.z0, f.z1} {
			bound, ok := bounds[e.Canon(w.EndU, w.EndV, z)]
			require.True(t, ok, `the wall places a vertex at End at level %v`, z)
			gap := requireReachesArcEnd(t, arc, w.EndU, w.EndV, bound)
			require.Greater(t, gap, 1e-12, `End sits well off the denoted point`)
			checked++
		}
	}
	require.Equal(t, 2, checked, `one arc wall, checked at both levels`)
}

// TestCapBlendSideRingReachesArcNaturalEnd chamfers the pie prism's end cap
// loop and chords the result's loop as the cap-blend mesh does: the side-ring
// sample at End must reach the point the arc denotes there.
func TestCapBlendSideRingReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	pie := arcEndPie(t, New())
	chamfered, err := pie.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(pie))), units.Millimeters(1))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok, `got %T`, chamfered.payload)
	loops := cbp.loops()
	require.Len(t, loops, 1)
	arc := onlyArc(t, loops[0].Segments)
	lm, err := chordCapBlendLoop(t.Context(), proofbound.NewWorkBudget(t.Context()), cbp, 0, loops[0], 0.2, freeform.NewFreeformWork())
	require.NoError(t, err)
	found := false
	for j, p := range lm.sidePts {
		if p != arc.End {
			continue
		}
		found = true
		gap := requireReachesArcEnd(t, arc, p.U, p.V, lm.sideBound[j])
		require.Greater(t, gap, 1e-12, `End sits well off the denoted point`)
	}
	require.True(t, found, `one side sample sits at End`)
}

// TestRevolveJunctionReachesArcNaturalEnd revolves the pie about the v axis,
// along its line from End to the origin, and reads the meridian junctions the
// revolve mesh rings from: the junction at End must enclose the axial
// coordinate of the point the arc denotes there, |z| = 20 on that axis.
func TestRevolveJunctionReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	start := s.CreatePoint(20, 0)
	end := s.CreatePoint(0, arcEndOffRadius(20))
	s.CreateLine(o, start)
	s.CreateArc(o, start, end)
	s.CreateLine(end, o)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 0, V: 1}}
	body, err := New().Revolve(s, s.Profiles()[0], axis, FullRevolution{})
	require.NoError(t, err)
	rp, ok := body.payload.(revolvePayload)
	require.True(t, ok, `got %T`, body.payload)
	arc := onlyArc(t, rp.profile.Outer.Segments)
	r, err := revolveaxis.ResolveLoop(t.Context(), rp.profile.Outer, freeform.NewFreeformWork(), "test",
		rp.chargedWalk, rp.ax.snapTol)
	require.NoError(t, err)
	js, _, err := revolvesampling.MeridianJunctions(rp.lift(), r)
	require.NoError(t, err)
	du, dv := arcDenotedEnd(arc)
	require.Zero(t, du.Sign(), `the denoted end lies on the axis`)
	z, _ := dv.Rat(nil)
	found := false
	for _, j := range js {
		// The revolve may run its axis either way along v, so z reads ±v.
		if math.Abs(j.Z) != arc.End.V {
			continue
		}
		found = true
		want := z
		if j.Z < 0 {
			want = new(big.Rat).Neg(z)
		}
		require.Truef(t, j.ZIv.Lo.Cmp(want) <= 0 && want.Cmp(j.ZIv.Hi) <= 0,
			`the junction at End encloses z in [%s, %s], not the denoted %s`,
			j.ZIv.Lo.FloatString(20), j.ZIv.Hi.FloatString(20), want.FloatString(20))
	}
	require.True(t, found, `one junction sits at End`)
}

// TestCapBlendCornerChordReachesArcNaturalEnd chamfers the pie prism's end
// cap loop and reads the miter corner at End, where the arc meets the line to
// the origin. CapBlendCornerLocusGap's ellipse needs a lower bound on c*, the
// distance from the denoted corner to the denoted foot. The denoted corner is
// the arc's denoted end D, and the denoted foot lies within footDelta and
// dsDelta of the held foot m at the held ds, so c* is at least |m − D| less
// those two. The published lower bound on c*² must not exceed that.
//
// Shown-to-fail: with CapBlendCornerChordSqLower charging the corner only its
// line walk's own StartBound (zero at the recorded End), the bound exceeds
// the worst-case c*² because D sits about 1.8e-12 mm nearer the foot than
// End does.
func TestCapBlendCornerChordReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	pie := arcEndPie(t, New())
	chamfered, err := pie.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(pie))), units.Millimeters(1))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok, `got %T`, chamfered.payload)
	segs := cbp.loops()[0].Segments
	arc := onlyArc(t, segs)
	walks, joins := capBlendCornerSetup(t, cbp)
	setback, footDelta := cbp.loopSetback(0), cbp.loopBandDelta(0)

	const prec = 256
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	dist := func(u, v *big.Float, j cornerJoin) *big.Float {
		du := new(big.Float).SetPrec(prec).Sub(f(j.m.U), u)
		dv := new(big.Float).SetPrec(prec).Sub(f(j.m.V), v)
		dz := f(setback.ds)
		du.Mul(du, du)
		dv.Mul(dv, dv)
		dz.Mul(dz, dz)
		return du.Add(du, dv).Add(du, dz).Sqrt(du)
	}
	checked := 0
	for i, j := range joins {
		if j.vU != arc.End.U || j.vV != arc.End.V {
			continue
		}
		require.False(t, j.g1, `the corner at End is a miter`)
		lower, err := tessellation.CapBlendCornerChordSqLower(capBlendLocusInput(cbp, walks, i, j))
		require.NoError(t, err)
		du, dv := arcDenotedEnd(arc)
		toDenoted := dist(du, dv, j)
		held := dist(f(j.vU), f(j.vV), j)
		require.Positive(t, new(big.Float).Sub(held, toDenoted).Sign(), `the denoted corner sits nearer the foot than End`)
		worst := new(big.Float).SetPrec(prec).Sub(toDenoted, f(footDelta))
		worst.Sub(worst, f(setback.dsDelta))
		if worst.Sign() < 0 {
			worst.SetFloat64(0)
		}
		worst.Mul(worst, worst)
		require.LessOrEqualf(t, f(lower).Cmp(worst), 0, `the c*² lower bound %.17g exceeds the worst case %s`, lower, worst.Text('g', 20))
		checked++
	}
	require.Equal(t, 1, checked, `one miter corner sits at End`)
}
