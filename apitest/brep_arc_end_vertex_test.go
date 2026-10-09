package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds every brep body's vertex at an arc's natural t = 1 end to
// the point the arc denotes there: Start's radius at End's angle
// (docs/evaluator-design.md §4). Each section below draws a sketch arc about
// the origin whose End sits 500 ulps past Start's radius, inside the arc
// preflight's 1024-ulp allowance, and never solves the sketch, so the record
// keeps the drawn End. The vertex there holds End verbatim, about 1.8e-12 mm
// (4.4e-13 mm on the radius-5 section) from the denoted point.
//
// Shown-to-fail: with brepgeom.Use's StartBound and EndBound replaced by the
// walk's own bounds (Walk.StartBound and Walk.EndBound) in evalBrepContext,
// the stacked union, the class-B cut and the route-E fillet publish both
// vertices at End Exact, 1.8e-12 mm from the denoted point. The shell side
// opening is coverage only: its canonical-vertex allowance already reached
// the denoted point.

// offRadiusEnd is v moved 500 ulps away from zero.
func offRadiusEnd(v float64) float64 {
	for range 500 {
		v = math.Nextafter(v, 2*v)
	}
	return v
}

// offRadiusSection draws a section through draw, extrudes it over [0, h] and
// returns the body with the one arc its record holds.
func offRadiusSection(t *testing.T, doc *decad.Document, h float64, draw func(*sketch.Sketch)) (*decad.Body, sectionrecord.ArcSeg) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	draw(s)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	record := recordOf(t, s, profiles[0])
	var arcs []sectionrecord.ArcSeg
	for _, seg := range record.Outer.Segments {
		if arc, ok := seg.(sectionrecord.ArcSeg); ok {
			arcs = append(arcs, arc)
		}
	}
	require.Len(t, arcs, 1)
	arc := arcs[0]
	require.Equal(t, [2]float64{0, 1}, [2]float64{arc.TStart, arc.TEnd}, `the arc is whole and counter-clockwise`)
	require.NotEqual(t, math.Hypot(arc.Start.U, arc.Start.V), math.Hypot(arc.End.U, arc.End.V), `End is off Start's radius`)
	body, err := doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body, arc
}

// pieSection is the quarter disk of radius 20 about the origin, its arc from
// (20, 0) to (0, 20) with End's v off the radius.
func pieSection(s *sketch.Sketch) {
	o := s.CreatePoint(0, 0)
	start := s.CreatePoint(20, 0)
	end := s.CreatePoint(0, offRadiusEnd(20))
	s.CreateLine(o, start)
	s.CreateArc(o, start, end)
	s.CreateLine(end, o)
}

// tombstoneSection runs from (20, −30) up to (20, 0), along the quarter arc
// about the origin to E = (0, 20) with E's v off the radius, across to
// (−40, E.v) and around (−40, −30). Every line runs along an axis.
func tombstoneSection(s *sketch.Sketch) {
	top := offRadiusEnd(20)
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
}

// drilledTombstone cuts the tombstone with a radius-2 drill along y through
// (−30, ·, 5), clear of the arc's whole-circle box, so the pair builds class
// B's through reach.
func drilledTombstone(t *testing.T, doc *decad.Document) (*decad.Body, sectionrecord.ArcSeg) {
	t.Helper()
	tomb, arc := offRadiusSection(t, doc, 10, tombstoneSection)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(-30, 5)
	s.Fix(c)
	s.CreateCircle(c, 2)
	drill, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(40)})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), tomb, drill)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(cut), `the cut builds a brep body`)
	return cut, arc
}

// requireArcEndsReached checks every vertex of body against the arc's two
// denoted ends at the vertex's own level. Each end must meet one vertex at
// each of the body's two levels within 1e-6 mm, and that vertex's bound must
// reach it. The vertex at
// Start, a recorded coordinate both neighbours state, stays Exact when
// startExact is set. It returns the largest distance from a vertex at End to
// End's denoted point.
func requireArcEndsReached(t *testing.T, body *decad.Body, arc sectionrecord.ArcSeg, startExact bool) float64 {
	t.Helper()
	ends := denotedSegmentEnds(t, []sectionrecord.CurveSegment{arc})
	require.Len(t, ends, 2)
	worst := 0.0
	for k, p := range ends {
		met := 0
		for _, vtx := range body.Vertices() {
			pos := vtx.Position()
			at := [3]float64{pos.Value.X, pos.Value.Y, pos.Value.Z}
			reach := vertexReach(at, inPlane(p, at[2])[0])
			if reach > 1e-6 {
				continue
			}
			met++
			bound, err := pos.Bound.In(units.Millimeter)
			require.NoError(t, err)
			require.LessOrEqualf(t, reach, bound,
				`vertex %v publishes bound %.3e but sits %.3e mm from the arc's denoted end`, at, bound, reach)
			if k == 0 && startExact {
				require.Equalf(t, decad.Exact, pos.Exactness, `vertex %v at the arc's recorded Start`, at)
			}
			if k == 1 {
				worst = math.Max(worst, reach)
			}
		}
		require.Equal(t, 2, met, `one vertex per level at each arc end`)
	}
	return worst
}

// TestBrepVertexReachesArcNaturalEnd builds four brep bodies whose rim keeps
// an arc with an off-radius End: a stacked union of the quarter disk with a
// boss flush in its corner, the drilled tombstone (class B), that body with
// its (20, −30) corner filleted by route E, and a D section shelled through
// its chord (a side opening). Each vertex at End must reach the arc's denoted
// point, and the vertex at Start stays Exact where both neighbours state it.
func TestBrepVertexReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	t.Run(`stacked union`, func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		pie, arc := offRadiusSection(t, doc, 10, pieSection)
		boss := boxBodyAtZ(t, doc, 0, 0, 5, 5, 10, 5)
		got, err := decad.Union(t.Context(), pie, boss)
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(got), `the union builds a brep body`)
		gap := requireArcEndsReached(t, got, arc, true)
		require.Greater(t, gap, 1e-12, `End sits well off the denoted point`)
	})
	t.Run(`class-B cut`, func(t *testing.T) {
		t.Parallel()
		cut, arc := drilledTombstone(t, decad.New())
		gap := requireArcEndsReached(t, cut, arc, true)
		require.Greater(t, gap, 1e-12)
	})
	t.Run(`route-E fillet`, func(t *testing.T) {
		t.Parallel()
		cut, arc := drilledTombstone(t, decad.New())
		corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(20, -30, 0)))
		filleted, err := cut.Fillet(t.Context(), corner, units.Millimeters(2))
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(filleted))
		require.Len(t, filleted.Faces(), len(cut.Faces())+1, `the corner gains one fillet face`)
		gap := requireArcEndsReached(t, filleted, arc, true)
		require.Greater(t, gap, 1e-12)
	})
	t.Run(`shell side opening`, func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		d, arc := offRadiusSection(t, doc, 10, func(s *sketch.Sketch) {
			o := s.CreatePoint(0, 0)
			a := s.CreatePoint(0, -5)
			b := s.CreatePoint(0, offRadiusEnd(5))
			s.CreateArc(o, a, b)
			s.CreateLine(b, a)
		})
		shelled, err := d.Shell(t.Context(), sideFaceAt(t, d, r3.NewVec(-1, 0, 0), 0), units.Millimeters(1))
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(shelled))
		gap := requireArcEndsReached(t, shelled, arc, false)
		require.Greater(t, gap, 1e-13)
	})
}
