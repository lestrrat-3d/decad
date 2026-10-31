package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// wedgePlateRectBoss is an 80 x 60 x 10 plate with a 20 x 20 boss on it,
// x 30..50, y 20..40, z 10..25, joined by an exact Union.
func wedgePlateRectBoss(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, 0, 0, 80, 60, 0, 10)
	boss := boxBodyAtZ(t, doc, 30, 20, 50, 40, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return u
}

// wedgeEnds returns e's end positions.
func wedgeEnds(e *decad.Edge) (r3.Vec, r3.Vec) {
	return e.Start().Position().Value, e.End().Position().Value
}

// wedgeCount resolves preds on b and returns the matched edges.
func wedgeSelect(t *testing.T, b *decad.Body, preds ...decad.EdgePredicate) []*decad.Edge {
	t.Helper()
	got, err := decad.Edges(preds...).SelectEdges(b)
	require.NoError(t, err)
	return got
}

// Reflex picks exactly the four boss-root lines on the plate top, which
// Concave cannot: every edge of the exact Union reads IsConvex.
func TestEdgeWedgeRectBossRoot(t *testing.T) {
	t.Parallel()
	u := wedgePlateRectBoss(t)
	edges := u.Edges()
	require.Len(t, edges, 24)

	roots := wedgeSelect(t, u, decad.Reflex())
	require.Len(t, roots, 4)
	for _, e := range roots {
		s, f := wedgeEnds(e)
		require.Equal(t, 10.0, s.Z)
		require.Equal(t, 10.0, f.Z)
		for _, p := range []r3.Vec{s, f} {
			require.True(t, (p.X == 30 || p.X == 50) && (p.Y == 20 || p.Y == 40), "root end %v is a boss footprint corner", p)
		}
		require.Equal(t, 20.0, s.Sub(f).Len())
		require.True(t, e.IsConvex(), "the walked-boundary reading calls the root convex")
	}
	require.Len(t, wedgeSelect(t, u, decad.Salient()), 20)
	_, err := decad.Edges(decad.Concave()).SelectEdges(u)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	for _, e := range edges {
		require.NotEqual(t, decad.WedgeUndecided, e.Wedge())
	}
}

// A reflection reverses every loop's walk against its face's outward normal,
// so the reading must still find the roots of the mirrored part.
func TestEdgeWedgeMirroredBossRoot(t *testing.T) {
	t.Parallel()
	u := wedgePlateRectBoss(t)
	frame, err := r3.NewFrame(r3.Vec{X: 100}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	m, err := decad.Mirrored(t.Context(), u, decad.MirrorFrame{Frame: frame})
	require.NoError(t, err)
	roots := wedgeSelect(t, m, decad.Reflex())
	require.Len(t, roots, 4)
	for _, e := range roots {
		s, f := wedgeEnds(e)
		require.Equal(t, 10.0, s.Z)
		require.Equal(t, 10.0, f.Z)
		require.True(t, s.X >= 150 && s.X <= 170 && f.X >= 150 && f.X <= 170, "root %v-%v lies under the mirrored boss", s, f)
	}
	require.Len(t, wedgeSelect(t, m, decad.Salient()), 20)
}

// The L bracket's one inside corner is the only reflex edge.
func TestEdgeWedgeLBracketInsideCorner(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := boxBodyAtZ(t, doc, 0, 0, 60, 40, 0, 8)
	up := boxBodyAtZ(t, doc, 0, 0, 8, 40, 0, 50)
	u, err := decad.Union(t.Context(), base, up)
	require.NoError(t, err)

	corner := wedgeSelect(t, u, decad.Reflex())
	require.Len(t, corner, 1)
	s, f := wedgeEnds(corner[0])
	require.ElementsMatch(t, []r3.Vec{{X: 8, Y: 0, Z: 8}, {X: 8, Y: 40, Z: 8}}, []r3.Vec{s, f})
	require.Equal(t, decad.WedgeReflex, corner[0].Wedge())
	require.Len(t, wedgeSelect(t, u, decad.Salient()), len(u.Edges())-1)
}

// A through hole's rims are salient by the material and concave by the
// walked boundary: the two readings differ there on purpose.
func TestEdgeWedgeHoleRimSalientAndConcave(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, 0, 0, 80, 60, 0, 10)
	tool := circlePrismAtZ(t, doc, 20, 30, 5, -1, 12)
	drilled, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)

	rims := wedgeSelect(t, drilled, decad.Circular())
	require.Len(t, rims, 2)
	for _, e := range rims {
		c, ok := e.Curve().(decad.Circle3)
		require.True(t, ok)
		require.Equal(t, r3.Vec{X: 20, Y: 30}, r3.Vec{X: c.Center.X, Y: c.Center.Y})
		require.Equal(t, decad.WedgeSalient, e.Wedge())
		require.False(t, e.IsConvex())
	}
	both := wedgeSelect(t, drilled, decad.Salient(), decad.Concave())
	require.Len(t, both, 2)
	_, err = decad.Edges(decad.Reflex()).SelectEdges(drilled)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// A round boss's root circle is reflex and its top rim salient.
func TestEdgeWedgeRoundBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, 0, 0, 80, 60, 0, 10)
	boss := circlePrismAtZ(t, doc, 40, 30, 10, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)

	root := wedgeSelect(t, u, decad.Circular(), decad.Reflex())
	require.Len(t, root, 1)
	s, _ := wedgeEnds(root[0])
	require.Equal(t, 10.0, s.Z)
	top := wedgeSelect(t, u, decad.Circular(), decad.Salient())
	require.Len(t, top, 1)
	s, _ = wedgeEnds(top[0])
	require.Equal(t, 25.0, s.Z)
	for _, e := range []*decad.Edge{root[0], top[0]} {
		c, ok := e.Curve().(decad.Circle3)
		require.True(t, ok)
		require.Equal(t, 10.0, c.Radius.Base())
	}
}

// A rounded-rectangle boss's four vertical lines where a flat wall turns
// into a corner round are tangent junctions: the two outward normals agree
// there, so neither predicate takes them. Every other edge is decided.
func TestEdgeWedgeTangentJunctionUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, 0, 0, 80, 60, 0, 10)
	boss := wedgeRoundedRectAtZ(t, doc, 25, 15, 55, 45, 5, 10, 10)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)

	var undecided []*decad.Edge
	for _, e := range u.Edges() {
		if e.Wedge() == decad.WedgeUndecided {
			undecided = append(undecided, e)
		}
	}
	require.Len(t, undecided, 8)
	for _, e := range undecided {
		s, f := wedgeEnds(e)
		require.Equal(t, s.X, f.X)
		require.Equal(t, s.Y, f.Y)
		require.InDeltaSlice(t, []float64{10, 20}, []float64{math.Min(s.Z, f.Z), math.Max(s.Z, f.Z)}, 0)
		_, ok := e.Curve().(decad.Line3)
		require.True(t, ok)
	}
	// The root is four lines and four quarter arcs, all on the plate top.
	roots := wedgeSelect(t, u, decad.Reflex())
	require.Len(t, roots, 8)
	for _, e := range roots {
		s, f := wedgeEnds(e)
		require.Equal(t, 10.0, s.Z)
		require.Equal(t, 10.0, f.Z)
	}
	require.Len(t, wedgeSelect(t, u, decad.Salient()), len(u.Edges())-16)
}

// wedgeRoundedRectAtZ extrudes the rectangle x0..x1, y0..y1 with corner
// radius r from z0 by h, its straight sides meeting the corner arcs
// tangentially.
func wedgeRoundedRectAtZ(t *testing.T, doc *decad.Document, x0, y0, x1, y1, r, z0, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	type seg struct {
		to     [2]float64
		center *[2]float64
	}
	at := func(x, y float64) *[2]float64 { return &[2]float64{x, y} }
	segs := []seg{
		{to: [2]float64{x1 - r, y0}},
		{to: [2]float64{x1, y0 + r}, center: at(x1-r, y0+r)},
		{to: [2]float64{x1, y1 - r}},
		{to: [2]float64{x1 - r, y1}, center: at(x1-r, y1-r)},
		{to: [2]float64{x0 + r, y1}},
		{to: [2]float64{x0, y1 - r}, center: at(x0+r, y1-r)},
		{to: [2]float64{x0, y0 + r}},
		{to: [2]float64{x0 + r, y0}, center: at(x0+r, y0+r)},
	}
	first := s.CreatePoint(x0+r, y0)
	s.Fix(first)
	cur := first
	for i, sg := range segs {
		next := first
		if i < len(segs)-1 {
			next = s.CreatePoint(sg.to[0], sg.to[1])
			s.Fix(next)
		}
		if sg.center == nil {
			s.CreateLine(cur, next)
		} else {
			c := s.CreatePoint(sg.center[0], sg.center[1])
			s.Fix(c)
			s.CreateArc(c, cur, next)
		}
		cur = next
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// A tilted post sends the Union down the mesh path. The box keeps its exact
// planes and reads salient all round, but the post's root is an ellipse and
// its rim a circle whose displacement from the true curve is unproven, so
// neither matches Reflex or Salient. internal/edgewedge's own tests cover a
// Faceted face, which no public construction here reaches.
func TestEdgeWedgeMeshUnionUnprovenEdges(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	post := circlePrismAtZ(t, doc, 0, 0, 4, 5, 10)
	tilt, err := r3.RotationAround(r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), units.Radians(0.2))
	require.NoError(t, err)
	post, err = decad.Transform(t.Context(), post, tilt)
	require.NoError(t, err)
	u, err := decad.Union(t.Context(), box, post)
	require.NoError(t, err)
	require.True(t, tookMeshPath(u))

	curved := 0
	for _, e := range u.Edges() {
		switch e.Curve().(type) {
		case decad.Line3:
			require.Equal(t, decad.WedgeSalient, e.Wedge())
		default:
			curved++
			require.Equal(t, decad.WedgeUndecided, e.Wedge())
		}
	}
	require.Equal(t, 2, curved)
	require.Len(t, wedgeSelect(t, u, decad.Salient()), len(u.Edges())-2)
	_, err = decad.Edges(decad.Reflex()).SelectEdges(u)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// A revolve's circles between a plane and a coaxial cone, cylinder or sphere
// read their wedge; the circle between the cone and the cylinder has no
// planar face to read the material side from and stays undecided.
func TestEdgeWedgeRevolveCircles(t *testing.T) {
	t.Parallel()
	// The profile in (u, v), v the radius about the u axis: a 2 mm chamfer at
	// u = 0, a radius-10 drum to u = 20, a step down to a radius-6 spigot to
	// u = 30, and a hemispherical radius-6 end beyond it.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0, 0}, {0, 8}, {2, 10}, {20, 10}, {20, 6}, {30, 6}}
	handles := make([]*sketch.Point, 0, len(pts)+1)
	for _, p := range pts {
		h := s.CreatePoint(p[0], p[1])
		s.Fix(h)
		handles = append(handles, h)
	}
	for i := range len(handles) - 1 {
		s.CreateLine(handles[i], handles[i+1])
	}
	tip := s.CreatePoint(36, 0)
	s.Fix(tip)
	center := s.CreatePoint(30, 0)
	s.Fix(center)
	s.CreateArc(center, tip, handles[len(handles)-1])
	s.CreateLine(tip, handles[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	type want struct {
		x, r  float64
		wedge decad.EdgeWedge
	}
	wants := []want{
		{0, 8, decad.WedgeSalient},    // end disk and chamfer cone
		{2, 10, decad.WedgeUndecided}, // chamfer cone and drum: no plane
		{20, 10, decad.WedgeSalient},  // drum and step
		{20, 6, decad.WedgeReflex},    // step and spigot
		{30, 6, decad.WedgeUndecided}, // spigot and sphere meet tangentially
	}
	got := map[[2]float64]decad.EdgeWedge{}
	for _, e := range body.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok {
			continue
		}
		got[[2]float64{c.Center.X, c.Radius.Base()}] = e.Wedge()
	}
	require.Len(t, got, len(wants))
	for _, w := range wants {
		require.Equal(t, w.wedge, got[[2]float64{w.x, w.r}], "circle at x=%v r=%v", w.x, w.r)
	}
}

// Filleting Edges(Reflex()) on the plate and square boss rounds exactly the
// four roots. Each root adds the region between a right-angle corner and a
// radius-r arc, (1 − π/4)r² per unit length, along the 20 mm side; at each of
// the boss's four sharp vertical corners the rolling-ball root turns a
// quarter turn about the corner line, adding that section swept through π/2
// at its centroid's distance d = r(10 − 3π)/(3(4 − π)) from the wall
// (Pappus). So the volume is 54000 + (1 − π/4)r²(4·20 + 2πd).
func TestEdgeWedgeFilletReflexRoots(t *testing.T) {
	t.Parallel()
	u := wedgePlateRectBoss(t)
	const r = 2.0
	got, err := decad.Fillet(t.Context(), u, decad.Edges(decad.Reflex()).Exactly(4), units.Millimeters(r))
	require.NoError(t, err)
	area := (1 - math.Pi/4) * r * r
	d := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	want := 54000 + area*(4*20+2*math.Pi*d)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, vol.Value.Base(), vol.Bound.Base()+1e-9)
	require.Greater(t, vol.Value.Base(), 54000.0)
}

// Reflex and Salient render as their codec tokens.
func TestEdgeWedgePredicateRendering(t *testing.T) {
	t.Parallel()
	q := decad.Edges(decad.Reflex()).Or(decad.Salient(), decad.Circular()).Exactly(3)
	require.Equal(t, "edges(reflex).or(salient, circular).exactly(3)", q.String())
	require.Equal(t, "reflex", decad.WedgeReflex.String())
	require.Equal(t, "salient", decad.WedgeSalient.String())
	require.Equal(t, "undecided", decad.WedgeUndecided.String())
}
