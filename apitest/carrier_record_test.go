package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
)

// This file is agent-docs/prism-boolean-design.md Part K §10's public-API tests: a
// disc united with a tooth and its rotated pattern copies, the concave
// roots filleted and a D bore cut, all with analytic edges. The input class
// is a disc (a CircleSeg with line and arc windows) united on its own plane
// with all-line teeth; every volume is compared with the same outline drawn
// in one sketch.

const (
	wheelR      = 15.0 // disc radius
	wheelBase   = 14.0 // radius of a tooth's inner corners
	wheelTip    = 18.0 // radius of a tooth's tip
	wheelInner  = 4.0
	wheelOuter  = 11.0
	wheelSpoke  = 1.5
	wheelRound  = 1.0 // window corner rounds
	wheelHeight = 3.0
	wheelRoot   = 0.8 // root fillet radius
	wheelBoreR  = 2.0
	wheelFlat   = 1.2
)

// wheelTooth is tooth k of n: inside corner, outside corner, outside corner,
// inside corner, in walk order, turned by k/n of a turn.
func wheelTooth(k, n int) [4][2]float64 {
	a := 0.3 + 2*math.Pi*float64(k)/float64(n) // off the circle's seam
	p := func(r, th float64) [2]float64 { return [2]float64{r * math.Cos(a+th), r * math.Sin(a+th)} }
	return [4][2]float64{p(wheelBase, 0), p(wheelTip, 0.05), p(wheelTip, 0.15), p(wheelBase, 0.2)}
}

func wheelSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	return s
}

func wheelFillet(t *testing.T, s *sketch.Sketch, first, second sketch.Entity, at *sketch.Point, r float64) {
	t.Helper()
	var err error
	switch a := first.(type) {
	case *sketch.Line:
		switch c := second.(type) {
		case *sketch.Line:
			_, err = sketch.CreateFillet(s, a, c, at, r)
		case *sketch.Arc:
			_, err = sketch.CreateFillet(s, a, c, at, r)
		}
	case *sketch.Arc:
		switch c := second.(type) {
		case *sketch.Line:
			_, err = sketch.CreateFillet(s, a, c, at, r)
		case *sketch.Arc:
			_, err = sketch.CreateFillet(s, a, c, at, r)
		}
	}
	require.NoError(t, err)
}

// wheelWindows draws windows 0..count-1 of a three-sector disc: an inner and
// an outer arc, two lines and a sketch fillet at each of the four corners.
func wheelWindows(t *testing.T, s *sketch.Sketch, count int) {
	t.Helper()
	o := s.Origin()
	x1 := s.CreatePoint(10, 0)
	ref := s.CreateLine(o, x1)
	ref.SetConstruction(true)
	s.AddConstraint(sketch.NewHorizontal(ref), sketch.NewDistance(o, x1, 10))
	const sectors = 3
	step := 2 * math.Pi / sectors
	for k := range count {
		s1 := math.Pi/2 + step*float64(k)
		s2 := s1 + step
		di, do := math.Asin(wheelSpoke/wheelInner), math.Asin(wheelSpoke/wheelOuter)
		a := s.CreatePoint(wheelInner*math.Cos(s1+di), wheelInner*math.Sin(s1+di))
		b := s.CreatePoint(wheelInner*math.Cos(s2-di), wheelInner*math.Sin(s2-di))
		c := s.CreatePoint(wheelOuter*math.Cos(s2-do), wheelOuter*math.Sin(s2-do))
		d := s.CreatePoint(wheelOuter*math.Cos(s1+do), wheelOuter*math.Sin(s1+do))
		inner, outer := s.CreateArc(o, a, b), s.CreateArc(o, d, c)
		side1, side2 := s.CreateLine(a, d), s.CreateLine(b, c)
		deg := math.Pi / 180
		s.AddConstraint(
			sketch.NewRadius(inner, wheelInner), sketch.NewRadius(outer, wheelOuter),
			sketch.NewAngle(ref, side1, s1/deg), sketch.NewDistancePointLine(o, side1, wheelSpoke),
			sketch.NewAngle(ref, side2, s2/deg), sketch.NewDistancePointLine(o, side2, wheelSpoke),
		)
		wheelFillet(t, s, inner, side1, a, wheelRound)
		wheelFillet(t, s, side2, inner, b, wheelRound)
		wheelFillet(t, s, outer, side2, c, wheelRound)
		wheelFillet(t, s, side1, outer, d, wheelRound)
	}
}

// wheelD draws the D bore outline about the origin: a long arc and a chord at x = wheelFlat.
func wheelD(s *sketch.Sketch) {
	h := math.Sqrt(wheelBoreR*wheelBoreR - wheelFlat*wheelFlat)
	up, lo := s.CreatePoint(wheelFlat, h), s.CreatePoint(wheelFlat, -h)
	arc := s.CreateArc(s.Origin(), up, lo)
	chord := s.CreateLine(lo, up)
	s.AddConstraint(sketch.NewRadius(arc, wheelBoreR), sketch.NewVertical(chord),
		sketch.NewDistancePointLine(s.Origin(), chord, wheelFlat))
}

// wheelProfile is the one profile of s with the given number of holes.
func wheelProfile(t *testing.T, s *sketch.Sketch, holes int) *sketch.Profile {
	t.Helper()
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	var found *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == holes {
			require.Nil(t, found, "the sketch holds exactly one profile with %d holes", holes)
			found = p
		}
	}
	require.NotNil(t, found, "no profile with %d holes", holes)
	return found
}

func wheelExtrude(t *testing.T, doc *decad.Document, s *sketch.Sketch, holes int, height float64) *decad.Body {
	t.Helper()
	body, err := decad.Extrude(t.Context(), doc, s, wheelProfile(t, s, holes),
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func wheelDisc(t *testing.T, doc *decad.Document, windows int) *decad.Body {
	t.Helper()
	s := wheelSketch(t)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), wheelR), wheelR))
	wheelWindows(t, s, windows)
	return wheelExtrude(t, doc, s, windows, wheelHeight)
}

func wheelToothBody(t *testing.T, doc *decad.Document, teeth int) *decad.Body {
	t.Helper()
	s := wheelSketch(t)
	pts := wheelTooth(0, teeth)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	return wheelExtrude(t, doc, s, 0, wheelHeight)
}

// wheelUnion unites the disc with the tooth and its n-1 rotated copies.
func wheelUnion(t *testing.T, windows, teeth int) *decad.Body {
	t.Helper()
	doc := decad.New()
	disc := wheelDisc(t, doc, windows)
	tooth := wheelToothBody(t, doc, teeth)
	var copies []*decad.Body
	if teeth > 1 {
		var err error
		copies, err = decad.PatternCopies(t.Context(), tooth,
			decad.CircularPattern{Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Count: teeth})
		require.NoError(t, err)
	}
	wheel, err := decad.Union(t.Context(), disc, append([]*decad.Body{tooth}, copies...)...)
	require.NoError(t, err)
	return wheel
}

func requireAnalyticEdges(t *testing.T, body *decad.Body) {
	t.Helper()
	require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(body))
	for _, e := range body.Edges() {
		switch e.Curve().(type) {
		case decad.Line3, decad.Arc3, decad.Circle3:
		default:
			t.Fatalf("edge curve %T is not an analytic line, arc or circle", e.Curve())
		}
	}
}

// wheelInPlace draws the united outline in one sketch: the circle between the
// teeth, each tooth's flanks from where they cross the circle, optionally the
// roots rounded and the D bore as a hole.
func wheelInPlace(t *testing.T, windows, teeth int, rounded, bore bool) *decad.Body {
	t.Helper()
	doc := decad.New()
	s := wheelSketch(t)
	cross := func(a, b [2]float64) [2]float64 {
		dx, dy := b[0]-a[0], b[1]-a[1]
		qa, qb, qc := dx*dx+dy*dy, 2*(a[0]*dx+a[1]*dy), a[0]*a[0]+a[1]*a[1]-wheelR*wheelR
		u := (-qb + math.Sqrt(qb*qb-4*qa*qc)) / (2 * qa)
		return [2]float64{a[0] + u*dx, a[1] + u*dy}
	}
	type toothPts struct{ x1, b, c, x2 *sketch.Point }
	pt := func(p [2]float64) *sketch.Point {
		q := s.CreatePoint(p[0], p[1])
		s.Fix(q)
		return q
	}
	tp := make([]toothPts, teeth)
	for k := range teeth {
		p := wheelTooth(k, teeth)
		tp[k] = toothPts{x1: pt(cross(p[0], p[1])), b: pt(p[1]), c: pt(p[2]), x2: pt(cross(p[3], p[2]))}
	}
	side1s := make([]*sketch.Line, teeth)
	side2s := make([]*sketch.Line, teeth)
	arcs := make([]*sketch.Arc, teeth)
	for k := range teeth {
		side1s[k] = s.CreateLine(tp[k].x1, tp[k].b)
		s.CreateLine(tp[k].b, tp[k].c)
		side2s[k] = s.CreateLine(tp[k].c, tp[k].x2)
	}
	for k := range teeth {
		arcs[k] = s.CreateArc(s.Origin(), tp[k].x2, tp[(k+1)%teeth].x1)
	}
	if rounded {
		for k := range teeth {
			next := (k + 1) % teeth
			wheelFillet(t, s, side2s[k], arcs[k], tp[k].x2, wheelRoot)
			wheelFillet(t, s, arcs[k], side1s[next], tp[next].x1, wheelRoot)
		}
	}
	wheelWindows(t, s, windows)
	holes := windows
	if bore {
		wheelD(s)
		holes++
	}
	return wheelExtrude(t, doc, s, holes, wheelHeight)
}

func rootEdges(teeth int) *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Reflex()).Exactly(2 * teeth)
}

// TestCarrierRecordWheelUnionFilletAndBore is §10's wheel-like part: a disc
// with two windows, a tooth and two rotated copies, the six roots rounded and
// a D bore cut, every edge analytic and every volume equal to the same outline
// drawn in one sketch within both bounds.
func TestCarrierRecordWheelUnionFilletAndBore(t *testing.T) {
	t.Parallel()
	const windows, teeth = 2, 3
	wheel := wheelUnion(t, windows, teeth)
	requireAnalyticEdges(t, wheel)
	require.Len(t, wheel.Lumps(), 1)

	drawn := wheelInPlace(t, windows, teeth, false, false)
	got, err := wheel.Volume()
	require.NoError(t, err)
	want, err := drawn.Volume()
	require.NoError(t, err)
	require.InDelta(t, want.Value.Base(), got.Value.Base(), got.Bound.Base()+want.Bound.Base())

	roots, err := rootEdges(teeth).SelectEdges(wheel)
	require.NoError(t, err)
	require.Len(t, roots, 2*teeth)
	filleted, err := decad.Fillet(t.Context(), wheel, rootEdges(teeth), units.Millimeters(wheelRoot))
	require.NoError(t, err)
	requireAnalyticEdges(t, filleted)
	// Each fillet adds the corner area r² less the arc's quarter disc where the
	// flank meets the circle at a right angle, and a little more where they meet at a slant.
	fv, err := filleted.Volume()
	require.NoError(t, err)
	require.Greater(t, fv.Value.Base(), got.Value.Base(), "rounding a concave root adds material")
	drawnRounded := wheelInPlace(t, windows, teeth, true, false)
	wantRounded, err := drawnRounded.Volume()
	require.NoError(t, err)
	require.InDelta(t, wantRounded.Value.Base(), fv.Value.Base(), fv.Bound.Base()+wantRounded.Bound.Base())

	doc := filleted.Document()
	tool := func() *decad.Body {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), -0.75)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		wheelD(s)
		return wheelExtrude(t, doc, s, 0, wheelHeight+1.5)
	}()
	bored, err := decad.Cut(t.Context(), filleted, tool)
	require.NoError(t, err)
	requireAnalyticEdges(t, bored)
	bv, err := bored.Volume()
	require.NoError(t, err)
	drawnBored := wheelInPlace(t, windows, teeth, true, true)
	wantBored, err := drawnBored.Volume()
	require.NoError(t, err)
	require.InDelta(t, wantBored.Value.Base(), bv.Value.Base(), bv.Bound.Base()+wantBored.Bound.Base())

	mesh, err := bored.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// parallelogram draws a block of width w and height h whose top and bottom slant, so
// the translated copies' edges never share a line.
func parallelogram(s *sketch.Sketch, x, y, w, h float64) {
	pts := [4]*sketch.Point{s.CreatePoint(x, y), s.CreatePoint(x+w, y+0.5), s.CreatePoint(x+w, y+h), s.CreatePoint(x, y+h-0.5)}
	for _, p := range pts {
		s.Fix(p)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%4])
	}
}

// fixedRect draws the axis-aligned rectangle with fixed corners.
func fixedRect(s *sketch.Sketch, x0, y0, x1, y1 float64) {
	pts := [4]*sketch.Point{s.CreatePoint(x0, y0), s.CreatePoint(x1, y0), s.CreatePoint(x1, y1), s.CreatePoint(x0, y1)}
	for _, p := range pts {
		s.Fix(p)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%4])
	}
}

// TestCarrierRecordHoledPlateWithTranslatedBlocks unites a holed rectangular
// plate (not disc-shaped) with a block and its translation-pattern copies and
// rounds every root: the extended one-scene Union takes the holed operand and
// the displaced copies, and the part keeps analytic faces.
func TestCarrierRecordHoledPlateWithTranslatedBlocks(t *testing.T) {
	t.Parallel()
	const height, blocks, r = 2.0, 4, 0.5
	doc := decad.New()
	ps := wheelSketch(t)
	fixedRect(ps, 0, 0, 40, 20)
	centre := ps.CreatePoint(20, 10)
	ps.Fix(centre)
	ps.CreateCircle(centre, 3)
	plate := wheelExtrude(t, doc, ps, 1, height)
	bs := wheelSketch(t)
	parallelogram(bs, 3.3, 18, 2, 6)
	block, err := decad.Extrude(t.Context(), doc, bs, bs.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	copies, err := decad.PatternCopies(t.Context(), block,
		decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(6.1), Count: blocks})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, append([]*decad.Body{block}, copies...)...)
	require.NoError(t, err)
	requireAnalyticEdges(t, joined)
	jv, err := joined.Volume()
	require.NoError(t, err)
	wantArea := 40*20 - math.Pi*9 + blocks*2*3.75
	require.InDelta(t, wantArea*height, jv.Value.Base(), jv.Bound.Base())
	require.Greater(t, jv.Bound.Base(), 0.0, "the copies' rounding is charged")

	roots := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Reflex()).Exactly(2 * blocks)
	filleted, err := decad.Fillet(t.Context(), joined, roots, units.Millimeters(r))
	require.NoError(t, err)
	requireAnalyticEdges(t, filleted)
	fv, err := filleted.Volume()
	require.NoError(t, err)
	wantArea += 2 * blocks * r * r * (1 - math.Pi/4)
	require.InDelta(t, wantArea*height, fv.Value.Base(), fv.Bound.Base())
	mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// TestCarrierRecordUnionKeepsTheCopiedPrismJoinRoute pins that the extended
// class takes no set a later route owns: a lobe's rotated copies on a disc
// still take the axial-pattern route, whose result is a loft.
func TestCarrierRecordUnionKeepsTheCopiedPrismJoinRoute(t *testing.T) {
	t.Parallel()
	joined, err := joinedByPattern(t, 3.0, 2.0, 8, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	require.Equal(t, "decad.loftPayload", decadtest.PayloadClass(joined))
	drawn := drawnAtAngles(t, 8)
	got, err := joined.Volume()
	require.NoError(t, err)
	want, err := drawn.Volume()
	require.NoError(t, err)
	require.InDelta(t, want.Value.Base(), got.Value.Base(), want.Bound.Base()+got.Bound.Base())
}

// TestCarrierRecordArcHeavyPairReachesItsRoute unites a disc drawn as eighteen
// arcs with a plate. Counted at 256 pieces per arc the pair exceeded the
// arrangement cap; counted by sweep it fits and reaches the pair route.
func TestCarrierRecordArcHeavyPairReachesItsRoute(t *testing.T) {
	t.Parallel()
	const arcs, r, height = 18, 10.0, 2.0
	doc := decad.New()
	ds := wheelSketch(t)
	pts := make([]*sketch.Point, arcs)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / arcs
		pts[i] = ds.CreatePoint(r*math.Cos(a), r*math.Sin(a))
		ds.Fix(pts[i])
	}
	ds.Fix(ds.Origin())
	for i := range pts {
		ds.CreateArc(ds.Origin(), pts[i], pts[(i+1)%arcs])
	}
	disc, err := decad.Extrude(t.Context(), doc, ds, ds.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	ps := wheelSketch(t)
	fixedRect(ps, 5, -3, 15, 3)
	plate, err := decad.Extrude(t.Context(), doc, ps, ps.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), disc, plate)
	require.NoError(t, err)
	require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(joined), "the pair route builds it analytically")
	// The plate's part outside the circle: the integral of 15 - sqrt(r² - y²) over [-3, 3].
	prim := func(y float64) float64 { return (y*math.Sqrt(r*r-y*y) + r*r*math.Asin(y/r)) / 2 }
	outside := 15*6 - (prim(3) - prim(-3))
	v, err := joined.Volume()
	require.NoError(t, err)
	require.InDelta(t, (math.Pi*r*r+outside)*height, v.Value.Base(), v.Bound.Base())
}

// TestCarrierRecordDiscWithTeeth builds §1's two cases: a disc united with
// one tooth, and with a tooth and its fourteen rotated copies, every root
// selected and rounded, every edge analytic and the volumes equal to the same
// outline drawn in one sketch.
func TestCarrierRecordDiscWithTeeth(t *testing.T) {
	t.Parallel()
	for _, teeth := range []int{1, 15} {
		t.Run(fmt.Sprintf("%d teeth", teeth), func(t *testing.T) {
			t.Parallel()
			wheel := wheelUnion(t, 0, teeth)
			requireAnalyticEdges(t, wheel)
			got, err := wheel.Volume()
			require.NoError(t, err)
			drawn, err := wheelInPlace(t, 0, teeth, false, false).Volume()
			require.NoError(t, err)
			require.InDelta(t, drawn.Value.Base(), got.Value.Base(), got.Bound.Base()+drawn.Bound.Base())

			filleted, err := decad.Fillet(t.Context(), wheel, rootEdges(teeth), units.Millimeters(wheelRoot))
			require.NoError(t, err)
			requireAnalyticEdges(t, filleted)
			fv, err := filleted.Volume()
			require.NoError(t, err)
			wantRounded, err := wheelInPlace(t, 0, teeth, true, false).Volume()
			require.NoError(t, err)
			require.InDelta(t, wantRounded.Value.Base(), fv.Value.Base(), fv.Bound.Base()+wantRounded.Bound.Base())
		})
	}
}
