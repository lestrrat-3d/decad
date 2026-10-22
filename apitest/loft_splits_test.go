package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The segment kinds the alignment searches below compare.
const (
	splitKindArc    = "an arc"
	splitKindLine   = "a line"
	splitKindSpline = "a spline"
)

// splitPolygon draws a closed polygon with the given vertices on plane and
// returns its sketch and profile.
func splitPolygon(t *testing.T, w *sketch.World, plane *sketch.Plane, pts [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func splitTopPlane(t *testing.T, w *sketch.World, z float64) *sketch.Plane {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	return plane
}

func regularPolygon(n int, radius float64, phase float64) [][2]float64 {
	pts := make([][2]float64, n)
	for i := range pts {
		a := phase + 2*math.Pi*float64(i)/float64(n)
		pts[i] = [2]float64{radius * math.Cos(a), radius * math.Sin(a)}
	}
	return pts
}

// splitPolygonArea is the shoelace area of a polygon.
func splitPolygonArea(pts [][2]float64) float64 {
	a := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		a += pts[i][0]*pts[j][1] - pts[j][0]*pts[i][1]
	}
	return math.Abs(a) / 2
}

// TestLoftSplitsSquareToOctagon lofts a square to an octagon by cutting each
// square side once, so each side pairs with two octagon sides. The solid is a
// prismatoid, whose volume is h/6·(A0 + 4·Amid + A1), exact here because the
// mid-section of two polygons ruled between matched vertices is the polygon of
// the averaged vertices.
func TestLoftSplitsSquareToOctagon(t *testing.T) {
	t.Parallel()
	// The octagon's corners are the square's corners and its side midpoints,
	// pushed out so every octagon side pairs with half a square side.
	square := [][2]float64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
	octagon := [][2]float64{{-2, -2}, {0, -3}, {2, -2}, {3, 0}, {2, 2}, {0, 3}, {-2, 2}, {-3, 0}}
	const height = 5.0
	w := sketch.NewWorld()
	s0, p0 := splitPolygon(t, w, w.XY(), square)
	s1, p1 := splitPolygon(t, w, splitTopPlane(t, w, height), octagon)
	var cuts []decad.LoftSplit
	for j := range 4 {
		cuts = append(cuts, decad.LoftSplit{Loop: 0, Segment: j, At: []float64{0.5}})
	}
	doc := decad.New()
	body, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1, decad.WithLoftSplits(cuts, nil))
	require.NoError(t, err)
	require.True(t, body.IsSolid())

	mid := make([][2]float64, 8)
	sq8 := [][2]float64{{-2, -2}, {0, -2}, {2, -2}, {2, 0}, {2, 2}, {0, 2}, {-2, 2}, {-2, 0}}
	for i := range mid {
		mid[i] = [2]float64{(sq8[i][0] + octagon[i][0]) / 2, (sq8[i][1] + octagon[i][1]) / 2}
	}
	want := height / 6 * (splitPolygonArea(square) + 4*splitPolygonArea(mid) + splitPolygonArea(octagon))
	v, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-9)
	require.Less(t, v.Bound.Base(), 1e-6, "an exact ruled loft carries a tiny volume bound")
}

// TestLoftSplitsRefusals pins each gate.
func TestLoftSplitsRefusals(t *testing.T) {
	t.Parallel()
	square := [][2]float64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
	hex := regularPolygon(6, 3, 0)
	build := func(opts ...decad.LoftOption) error {
		w := sketch.NewWorld()
		s0, p0 := splitPolygon(t, w, w.XY(), square)
		s1, p1 := splitPolygon(t, w, splitTopPlane(t, w, 4), hex)
		_, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1, opts...)
		return err
	}
	two := []decad.LoftSplit{{Loop: 0, Segment: 0, At: []float64{0.5}}, {Loop: 0, Segment: 1, At: []float64{0.5}}}
	t.Run("counts still unequal", func(t *testing.T) {
		t.Parallel()
		one := two[:1]
		require.ErrorIs(t, build(decad.WithLoftSplits(one, nil)), decad.ErrUnsupported)
		require.ErrorIs(t, build(), decad.ErrUnsupported)
	})
	t.Run("equal counts with a twisted correspondence are refused by the audit", func(t *testing.T) {
		t.Parallel()
		// Four sides cut into six pair with a hexagon's six sides, but the two
		// cut sides sit next to each other, so the walls cross.
		require.ErrorIs(t, build(decad.WithLoftSplits(two, nil)), decad.ErrDegenerate)
	})
	t.Run("a fraction outside (0, 1)", func(t *testing.T) {
		t.Parallel()
		for _, f := range []float64{0, 1, -0.5, 1.5, math.NaN()} {
			bad := []decad.LoftSplit{{Loop: 0, Segment: 0, At: []float64{f}}}
			require.ErrorIs(t, build(decad.WithLoftSplits(bad, nil)), decad.ErrDegenerate, "fraction %v", f)
		}
	})
	t.Run("fractions that do not increase", func(t *testing.T) {
		t.Parallel()
		bad := []decad.LoftSplit{{Loop: 0, Segment: 0, At: []float64{0.6, 0.4}}}
		require.ErrorIs(t, build(decad.WithLoftSplits(bad, nil)), decad.ErrDegenerate)
	})
	t.Run("an index outside its loop", func(t *testing.T) {
		t.Parallel()
		for _, bad := range [][]decad.LoftSplit{
			{{Loop: 0, Segment: 9, At: []float64{0.5}}},
			{{Loop: 3, Segment: 0, At: []float64{0.5}}},
			{{Loop: 0, Segment: -1, At: []float64{0.5}}},
		} {
			require.ErrorIs(t, build(decad.WithLoftSplits(bad, nil)), decad.ErrDegenerate)
		}
	})
	t.Run("a segment split twice and no fractions", func(t *testing.T) {
		t.Parallel()
		dup := []decad.LoftSplit{{Loop: 0, Segment: 0, At: []float64{0.5}}, {Loop: 0, Segment: 0, At: []float64{0.7}}}
		require.ErrorIs(t, build(decad.WithLoftSplits(dup, nil)), decad.ErrDegenerate)
		none := []decad.LoftSplit{{Loop: 0, Segment: 0}}
		require.ErrorIs(t, build(decad.WithLoftSplits(none, nil)), decad.ErrDegenerate)
	})
	t.Run("the option twice", func(t *testing.T) {
		t.Parallel()
		require.ErrorIs(t, build(decad.WithLoftSplits(two, nil), decad.WithLoftSplits(nil, two)), decad.ErrDegenerate)
	})
}

// TestLoftSplitsLeaveAnUnsplitLoftAlone pins that an empty split list changes nothing.
func TestLoftSplitsLeaveAnUnsplitLoftAlone(t *testing.T) {
	t.Parallel()
	square := [][2]float64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
	vol := func(opts ...decad.LoftOption) float64 {
		w := sketch.NewWorld()
		s0, p0 := splitPolygon(t, w, w.XY(), square)
		s1, p1 := splitPolygon(t, w, splitTopPlane(t, w, 3), [][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
		b, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1, opts...)
		require.NoError(t, err)
		v, err := b.Volume()
		require.NoError(t, err)
		return v.Value.Base()
	}
	require.Equal(t, vol(), vol(decad.WithLoftSplits(nil, nil)))
}

// prismatoidVolume is h/6·(A0 + 4·Amid + A1) for a ruled loft between two polygons
// whose vertices correspond one to one and whose side walls are planar.
func prismatoidVolume(h float64, a, b [][2]float64) float64 {
	mid := make([][2]float64, len(a))
	for i := range a {
		mid[i] = [2]float64{(a[i][0] + b[i][0]) / 2, (a[i][1] + b[i][1]) / 2}
	}
	return h / 6 * (splitPolygonArea(a) + 4*splitPolygonArea(mid) + splitPolygonArea(b))
}

// TestLoftSplitsPolygonPairs lofts polygons of different vertex counts. In each
// case the cut sections' vertex lists are what the loft pairs, so the expected
// volume is the prismatoid volume of those two lists.
func TestLoftSplitsPolygonPairs(t *testing.T) {
	t.Parallel()
	midpoint := func(a, b [2]float64) [2]float64 { return [2]float64{(a[0] + b[0]) / 2, (a[1] + b[1]) / 2} }
	tri := regularPolygon(3, 3, math.Pi/2)
	hex := regularPolygon(6, 3, math.Pi/2)
	rect := [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}
	ell := [][2]float64{{0, 0}, {4, 0}, {4, 1}, {2, 1}, {2, 2}, {0, 2}}
	cases := []struct {
		name         string
		a, b         [][2]float64
		cuts0, cuts1 []decad.LoftSplit
		// matchedA is a with the cut points inserted, in walk order.
		matchedA, matchedB [][2]float64
	}{
		{
			name: "triangle to hexagon",
			a:    tri, b: hex,
			cuts0: []decad.LoftSplit{
				{Loop: 0, Segment: 0, At: []float64{0.5}},
				{Loop: 0, Segment: 1, At: []float64{0.5}},
				{Loop: 0, Segment: 2, At: []float64{0.5}},
			},
			matchedA: [][2]float64{tri[0], midpoint(tri[0], tri[1]), tri[1], midpoint(tri[1], tri[2]), tri[2], midpoint(tri[2], tri[0])},
			matchedB: hex,
		},
		{
			name: "rectangle to an L-shaped section",
			a:    rect, b: ell,
			cuts0: []decad.LoftSplit{
				{Loop: 0, Segment: 1, At: []float64{0.5}},
				{Loop: 0, Segment: 2, At: []float64{0.5}},
			},
			matchedA: [][2]float64{{0, 0}, {4, 0}, {4, 1}, {4, 2}, {2, 2}, {0, 2}},
			matchedB: ell,
		},
		{
			name: "cuts on the second section",
			a:    hex, b: tri,
			cuts1: []decad.LoftSplit{
				{Loop: 0, Segment: 0, At: []float64{0.5}},
				{Loop: 0, Segment: 1, At: []float64{0.5}},
				{Loop: 0, Segment: 2, At: []float64{0.5}},
			},
			matchedA: hex,
			matchedB: [][2]float64{tri[0], midpoint(tri[0], tri[1]), tri[1], midpoint(tri[1], tri[2]), tri[2], midpoint(tri[2], tri[0])},
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			const height = 4.0
			w := sketch.NewWorld()
			s0, p0 := splitPolygon(t, w, w.XY(), c.a)
			s1, p1 := splitPolygon(t, w, splitTopPlane(t, w, height), c.b)
			body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1, decad.WithLoftSplits(c.cuts0, c.cuts1))
			require.NoError(t, err)
			require.True(t, body.IsSolid())
			v, err := body.Volume()
			require.NoError(t, err)
			require.InDelta(t, prismatoidVolume(height, c.matchedA, c.matchedB), v.Value.Base(), v.Bound.Base()+1e-9)
			require.Less(t, v.Bound.Base(), 1e-6)
		})
	}
}

// halfDiscProfile draws a half disc of the given radius whose arc is drawn as
// `arcs` equal arcs about the origin, and returns the sketch, the profile and the
// recorded index of each arc in walk order.
func halfDiscProfile(t *testing.T, w *sketch.World, plane *sketch.Plane, radius float64, arcs int) (*sketch.Sketch, *sketch.Profile, []int) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.Origin()
	pt := func(a float64) *sketch.Point {
		p := s.CreatePoint(radius*math.Cos(a), radius*math.Sin(a))
		s.Fix(p)
		return p
	}
	pts := make([]*sketch.Point, arcs+1)
	for i := range pts {
		pts[i] = pt(math.Pi * float64(i) / float64(arcs))
	}
	for i := range arcs {
		s.CreateArc(center, pts[i], pts[i+1])
	}
	s.CreateLine(pts[arcs], pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	p := s.Profiles()[0]
	var arcIdx []int
	for i, e := range p.Outer {
		if _, ok := e.Entity.(*sketch.Arc); ok {
			arcIdx = append(arcIdx, i)
		}
	}
	require.Len(t, arcIdx, arcs)
	return s, p, arcIdx
}

// TestLoftSplitsArcs lofts a half disc whose arc is one arc to a smaller half
// disc whose arc is drawn as two arcs, cutting the first arc in half. The
// solid is half a frustum, whose volume is (π·h/3)(R² + Rr + r²)/2.
func TestLoftSplitsArcs(t *testing.T) {
	t.Parallel()
	const (
		R, r, height = 3.0, 2.0, 4.0
	)
	w := sketch.NewWorld()
	s0, p0, arc0 := halfDiscProfile(t, w, w.XY(), R, 1)
	s1, p1, arc1 := halfDiscProfile(t, w, splitTopPlane(t, w, height), r, 2)
	// The first section has two segments; cutting its arc gives three. The line
	// pairs with the second section's line, wherever each profile starts.
	cut := []decad.LoftSplit{{Loop: 0, Segment: arc0[0], At: []float64{0.5}}}
	lineIdx := func(arcs ...int) int {
		for i := range 3 {
			if !slices.Contains(arcs, i) {
				return i
			}
		}
		return -1
	}
	// After the cut the first section's segments are, in walk order, either
	// [piece, piece, line] or [line, piece, piece] or [piece, line, piece]; the
	// pieces replace index arc0[0] and the line is the other original index.
	order0 := make([]string, 0, 3)
	for i := range 2 {
		if i == arc0[0] {
			order0 = append(order0, splitKindArc, splitKindArc)
		} else {
			order0 = append(order0, splitKindLine)
		}
	}
	order1 := make([]string, 3)
	for i := range order1 {
		order1[i] = splitKindArc
	}
	order1[lineIdx(arc1...)] = splitKindLine
	offset := -1
	for off := range 3 {
		ok := true
		for k := range 3 {
			if order0[k] != order1[(k+off)%3] {
				ok = false
			}
		}
		if ok {
			offset = off
			break
		}
	}
	require.GreaterOrEqual(t, offset, 0)
	body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1,
		decad.WithLoftSplits(cut, nil), decad.WithLoftAlignment(offset))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	want := math.Pi * height / 3 * (R*R + R*r + r*r) / 2
	v, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-9)
	require.Less(t, v.Bound.Base(), 0.05*want, "the chorded arcs' volume bound stays small")
}

// arcLensProfile draws a closed section bounded below by a line from (-a, 0) to
// (a, 0) and above by fit splines through the given points, which run from (a, 0)
// back to (-a, 0). More than one spline is drawn when `breaks` names indices of
// pts at which the upper boundary is divided. It returns the recorded index of each
// spline in walk order.
func fitLensProfile(t *testing.T, w *sketch.World, plane *sketch.Plane, pts [][2]float64, breaks []int) (*sketch.Sketch, *sketch.Profile, []int) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	cuts := append([]int{0}, breaks...)
	cuts = append(cuts, len(pts)-1)
	for i := 0; i+1 < len(cuts); i++ {
		_, err := s.CreateFitSpline(sp[cuts[i] : cuts[i+1]+1]...)
		require.NoError(t, err)
	}
	s.CreateLine(sp[len(pts)-1], sp[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	p := s.Profiles()[0]
	var idx []int
	for i, e := range p.Outer {
		if _, ok := e.Entity.(*sketch.FitSpline); ok {
			idx = append(idx, i)
		}
	}
	require.Len(t, idx, len(cuts)-1)
	return s, p, idx
}

// TestLoftSplitsFitSplines lofts a section whose upper boundary is one fit spline
// to a section whose upper boundary is two, cutting the first spline in half.
// The loft is a ruled solid between two sections, so its volume lies between the
// section areas times the height; the held mesh must also verify.
func TestLoftSplitsFitSplines(t *testing.T) {
	t.Parallel()
	const height = 3.0
	arch := func(scale float64) [][2]float64 {
		pts := make([][2]float64, 5)
		for i := range pts {
			a := math.Pi * float64(i) / 4
			pts[i] = [2]float64{scale * 2 * math.Cos(a), scale * (math.Sin(a) + 0.2*math.Sin(2*a))}
		}
		return pts
	}
	w := sketch.NewWorld()
	s0, p0, spl0 := fitLensProfile(t, w, w.XY(), arch(1), nil)
	s1, p1, spl1 := fitLensProfile(t, w, splitTopPlane(t, w, height), arch(0.5), []int{2})
	cut := []decad.LoftSplit{{Loop: 0, Segment: spl0[0], At: []float64{0.5}}}
	// Each profile has the line and its splines; find the alignment offset that
	// pairs the line with the line and the splines in order.
	kinds := func(p *sketch.Profile, extra int) []string {
		var k []string
		for _, e := range p.Outer {
			if _, ok := e.Entity.(*sketch.FitSpline); ok {
				k = append(k, splitKindSpline)
				if extra > 0 {
					k = append(k, splitKindSpline)
				}
			} else {
				k = append(k, splitKindLine)
			}
		}
		return k
	}
	k0, k1 := kinds(p0, 1), kinds(p1, 0)
	require.Len(t, k0, len(k1))
	offset := -1
	for off := range k1 {
		ok := true
		for i := range k0 {
			if k0[i] != k1[(i+off)%len(k1)] {
				ok = false
			}
		}
		if ok {
			offset = off
			break
		}
	}
	require.GreaterOrEqual(t, offset, 0)
	require.Len(t, spl1, 2)
	body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1,
		decad.WithLoftSplits(cut, nil), decad.WithLoftAlignment(offset))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	v, err := body.Volume()
	require.NoError(t, err)
	lo, hi := height*p1.Area, height*p0.Area
	require.GreaterOrEqual(t, v.Value.Base()+v.Bound.Base(), lo)
	require.LessOrEqual(t, v.Value.Base()-v.Bound.Base(), hi)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// archApex is the depth of the apex archSection closes its sections at.
const archApex = 12

// archSection draws a section whose top edge is a fit spline through the given
// points, running from the right end to the left, closed below by two lines
// through the apex (0, -archApex). With a straight top edge, pass two points.
func archSection(t *testing.T, w *sketch.World, plane *sketch.Plane, top [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(top))
	for i, p := range top {
		pts[i] = s.CreatePoint(p[0], p[1])
		s.Fix(pts[i])
	}
	bottom := s.CreatePoint(0, -archApex)
	s.Fix(bottom)
	if len(top) == 2 {
		s.CreateLine(pts[0], pts[1])
	} else {
		_, err = s.CreateFitSpline(pts...)
		require.NoError(t, err)
	}
	s.CreateLine(pts[len(pts)-1], bottom)
	s.CreateLine(bottom, pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// TestLoftLineAgainstFitSpline lofts a section whose top edge is a fit spline arch
// to a triangle whose top edge is the straight chord, so one pair is a fit spline
// against a line and the other two pairs are identical lines. The sections are
// A(t) regions between a moving top curve and fixed sides, so the volume lies
// between the triangle's area and the arch section's area times the height, and
// the held mesh verifies.
func TestLoftLineAgainstFitSpline(t *testing.T) {
	t.Parallel()
	const height = 30.0
	arch := make([][2]float64, 5)
	for i := range arch {
		a := math.Pi * float64(i) / 4
		arch[i] = [2]float64{20 * math.Cos(a), 1.6 * (math.Sin(a) + 0.2*math.Sin(2*a))}
	}
	w := sketch.NewWorld()
	s0, p0 := archSection(t, w, w.XY(), arch)
	s1, p1 := archSection(t, w, splitTopPlane(t, w, height), [][2]float64{{20, 0}, {-20, 0}})
	require.Less(t, p1.Area, p0.Area)
	body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1)
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	v, err := body.Volume()
	require.NoError(t, err)
	require.GreaterOrEqual(t, v.Value.Base()+v.Bound.Base(), height*p1.Area)
	require.LessOrEqual(t, v.Value.Base()-v.Bound.Base(), height*p0.Area)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	// The pair is symmetric: the line on the first section and the arch on the second.
	w2 := sketch.NewWorld()
	t0, q0 := archSection(t, w2, w2.XY(), [][2]float64{{20, 0}, {-20, 0}})
	t1, q1 := archSection(t, w2, splitTopPlane(t, w2, height), arch)
	swapped, err := decad.Loft(t.Context(), decad.New(), t0, q0, t1, q1)
	require.NoError(t, err)
	sv, err := swapped.Volume()
	require.NoError(t, err)
	require.InDelta(t, v.Value.Base(), sv.Value.Base(), v.Bound.Base()+sv.Bound.Base())
}

// TestLoftLineAgainstArcStaysRefused pins what the line lift does not reach: an arc
// has no exact common form with a line, so the pair stays ErrUnsupported.
func TestLoftLineAgainstArcStaysRefused(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	round, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	right, left, bottom := round.CreatePoint(20, 0), round.CreatePoint(-20, 0), round.CreatePoint(0, -12)
	for _, p := range []*sketch.Point{right, left, bottom} {
		round.Fix(p)
	}
	round.CreateArc(round.Origin(), right, left)
	round.CreateLine(left, bottom)
	round.CreateLine(bottom, right)
	_, err = round.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, round.Profiles(), 1)
	s1, p1 := archSection(t, w, splitTopPlane(t, w, 30), [][2]float64{{20, 0}, {-20, 0}})
	_, err = decad.Loft(t.Context(), decad.New(), round, round.Profiles()[0], s1, p1)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
