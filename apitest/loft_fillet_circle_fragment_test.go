package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// bladeLoft lofts a rectangle-like tab with a fit-spline top between two identical
// planes: two vertical sides at x = ±halfWidth from y = base to y = top, closed
// below by a line and above by a spline arch. The sides cross a circle of radius
// discRadius about the origin when base is inside it and top outside.
func bladeLoft(t *testing.T, doc *decad.Document, halfWidth, base, top, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		pt := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		bl, br, tr, tl := pt(-halfWidth, base), pt(halfWidth, base), pt(halfWidth, top), pt(-halfWidth, top)
		s.CreateLine(bl, br)
		s.CreateLine(br, tr)
		_, err = s.CreateFitSpline(tr, pt(halfWidth/2, top+0.5), pt(0, top+0.7), pt(-halfWidth/2, top+0.5), tl)
		require.NoError(t, err)
		s.CreateLine(tl, bl)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(w.XY())
	s1, p1 := section(splitTopPlane(t, w, height))
	b, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return b
}

// TestLoftFilletAtALineMeetingACircleFragment rounds the two concave corners where a
// blade's straight sides meet a disc. The union leaves each corner as a line meeting
// a trimmed circle fragment. A fillet of radius r in a free wedge of angle a adds the
// area r^2 (cot(a/2) - (pi - a)/2).
func TestLoftFilletAtALineMeetingACircleFragment(t *testing.T) {
	t.Parallel()
	const (
		discR, halfWidth, base, top, height = 6.0, 1.0, 4.0, 8.0, 3.0
		radius                              = 0.3
	)
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, discR, 0, height)
	blade := bladeLoft(t, doc, halfWidth, base, top, height)
	joined, err := decad.Union(t.Context(), disc, blade)
	require.NoError(t, err)
	v0, err := joined.Volume()
	require.NoError(t, err)

	cornerY := math.Sqrt(discR*discR - halfWidth*halfWidth)
	all, err := decad.Edges(decad.ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var sel *decad.EdgeQuery
	found := 0
	for _, sx := range []float64{-halfWidth, halfWidth} {
		for _, e := range all {
			p := e.Start().Position().Value
			if p.Z != 0 || math.Hypot(p.X-sx, p.Y-cornerY) > 1e-6 {
				continue
			}
			q := []decad.EdgePredicate{decad.ParallelTo(r3.Vec{Z: 1}), decad.EndpointAt(p)}
			if sel == nil {
				sel = decad.Edges(q...)
			} else {
				sel = sel.Or(q...)
			}
			found++
			break
		}
	}
	require.Equal(t, 2, found, "both corner edges are on the joined body")

	rounded, err := decad.Fillet(t.Context(), joined, sel, units.Millimeters(radius))
	require.NoError(t, err)
	v1, err := rounded.Volume()
	require.NoError(t, err)
	// The free wedge at (halfWidth, cornerY) is between the side going up and the
	// circle's tangent going right and down.
	theta := math.Asin(halfWidth / discR)
	alpha := math.Pi/2 + theta
	perCorner := radius * radius * (1/math.Tan(alpha/2) - (math.Pi-alpha)/2)
	want := 2 * perCorner * height
	gain := v1.Value.Base() - v0.Value.Base()
	require.InDelta(t, want, gain, v0.Bound.Base()+v1.Bound.Base()+1e-9, "within the stated volume bounds")
	// The held connector is chorded, so its area sits a little under the ideal
	// arc's; the bounds above are loose for a disc this size, so also pin the
	// gain to the analytic corner area within the chord's sagitta.
	require.InEpsilon(t, want, gain, 0.1)
	require.Greater(t, v1.Value.Base(), v0.Value.Base())
	require.True(t, rounded.IsSolid())
}

// cornerEdges selects the axial edges at z = 0 whose start lies within 1e-6 of any of
// the given plane points, each by its own stored end point.
func cornerEdges(t *testing.T, body *decad.Body, corners [][2]float64) *decad.EdgeQuery {
	t.Helper()
	all, err := decad.Edges(decad.ParallelTo(r3.Vec{Z: 1})).SelectEdges(body)
	require.NoError(t, err)
	var sel *decad.EdgeQuery
	for _, c := range corners {
		found := false
		for _, e := range all {
			p := e.Start().Position().Value
			if p.Z != 0 || math.Hypot(p.X-c[0], p.Y-c[1]) > 1e-6 {
				continue
			}
			q := []decad.EdgePredicate{decad.ParallelTo(r3.Vec{Z: 1}), decad.EndpointAt(p)}
			if sel == nil {
				sel = decad.Edges(q...)
			} else {
				sel = sel.Or(q...)
			}
			found = true
			break
		}
		require.True(t, found, "no axial edge at %v", c)
	}
	return sel
}

func TestLoftFilletCircleFragmentRefusals(t *testing.T) {
	t.Parallel()
	const discR, halfWidth, height = 6.0, 1.0, 3.0
	cy := math.Sqrt(discR*discR - halfWidth*halfWidth)
	build := func() (*decad.Body, *decad.EdgeQuery) {
		doc := decad.New()
		disc := circleBodyAtZ(t, doc, 0, discR, 0, height)
		blade := bladeLoft(t, doc, halfWidth, 4, 8, height)
		joined, err := decad.Union(t.Context(), disc, blade)
		require.NoError(t, err)
		return joined, cornerEdges(t, joined, [][2]float64{{halfWidth, cy}})
	}
	t.Run("a radius that consumes the blade side", func(t *testing.T) {
		t.Parallel()
		joined, sel := build()
		_, err := decad.Fillet(t.Context(), joined, sel, units.Millimeters(40))
		require.Error(t, err)
		require.True(t, errors.Is(err, decad.ErrUnsupported) || errors.Is(err, decad.ErrDegenerate), "got %v", err)
	})
	t.Run("an edge in the middle of the circle fragment", func(t *testing.T) {
		t.Parallel()
		joined, _ := build()
		all, err := decad.Edges(decad.ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
		require.NoError(t, err)
		var sel *decad.EdgeQuery
		for _, e := range all {
			p := e.Start().Position().Value
			if p.Z == 0 && p.Y < 0 && math.Abs(math.Hypot(p.X, p.Y)-discR) < 1e-9 {
				sel = decad.Edges(decad.ParallelTo(r3.Vec{Z: 1}), decad.EndpointAt(p))
				break
			}
		}
		require.NotNil(t, sel, "an axial edge on the circle's far side")
		_, err = decad.Fillet(t.Context(), joined, sel, units.Millimeters(0.3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// TestLoftFilletEveryCornerOfLobesOnADisc rounds all 2N corners where N lobes meet a
// disc, which are N circle fragments each meeting a line at both ends, then bores
// and chamfers the result.
func TestLoftFilletEveryCornerOfLobesOnADisc(t *testing.T) {
	t.Parallel()
	const r, height, n, radius = 3.0, 2.0, 8, 0.05
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobes := make([]*decad.Body, n)
	var corners [][2]float64
	for k := range lobes {
		rot, half := 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n)
		lobes[k] = lobeLoft(t, doc, r, rot, half, height)
		for _, a := range []float64{-half, half} {
			corners = append(corners, [2]float64{r * math.Cos(a+rot), r * math.Sin(a+rot)})
		}
	}
	joined, err := decad.Union(t.Context(), disc, lobes...)
	require.NoError(t, err)
	v0, err := joined.Volume()
	require.NoError(t, err)
	rounded, err := decad.Fillet(t.Context(), joined, cornerEdges(t, joined, corners), units.Millimeters(radius))
	require.NoError(t, err)
	v1, err := rounded.Volume()
	require.NoError(t, err)
	require.Greater(t, v1.Value.Base(), v0.Value.Base(), "a concave fillet adds material at every corner")
	require.True(t, rounded.IsSolid())

	// The finishing order that follows a root fillet: a centered bore, then both outer caps.
	bore := circleBodyAtZ(t, doc, 0, 0.8, -1, height+2)
	bored, err := decad.Cut(t.Context(), rounded, bore)
	require.NoError(t, err)
	bv, err := bored.Volume()
	require.NoError(t, err)
	require.InDelta(t, v1.Value.Base()-math.Pi*0.8*0.8*height, bv.Value.Base(), v1.Bound.Base()+bv.Bound.Base())
	// A bore that rebuilds the loft names its caps by the bored body, and one that keeps
	// the filleted loft's roles names them by that loft; take the one that resolves.
	caps := decad.Edges(decad.OuterLoopOf(decad.CapStart(bored))).Or(decad.OuterLoopOf(decad.CapEnd(bored)))
	if _, err := caps.SelectEdges(bored); err != nil {
		caps = decad.Edges(decad.OuterLoopOf(decad.CapStart(rounded))).Or(decad.OuterLoopOf(decad.CapEnd(rounded)))
	}
	chamfered, err := decad.Chamfer(t.Context(), bored, caps, units.Millimeters(0.02))
	require.NoError(t, err)
	cv, err := chamfered.Volume()
	require.NoError(t, err)
	require.Less(t, cv.Value.Base(), bv.Value.Base(), "a chamfer removes material")
}
