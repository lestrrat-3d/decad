package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
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

// TestLoftFilletLargeRadiusRestsOnTheBladeTop rounds the corner where a blade's
// side meets a disc with a radius far longer than the side. The fillet circle rolls
// along the disc until it rests on the side's top end, where the fit-spline arch
// turns away from it, so the side leaves the section and the arch stays whole.
func TestLoftFilletLargeRadiusRestsOnTheBladeTop(t *testing.T) {
	t.Parallel()
	const (
		discR, halfWidth, base, top, height = 6.0, 1.0, 4.0, 8.0, 3.0
		radius                              = 40.0
	)
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, discR, 0, height)
	blade := bladeLoft(t, doc, halfWidth, base, top, height)
	joined, err := decad.Union(t.Context(), disc, blade)
	require.NoError(t, err)
	v0, err := joined.Volume()
	require.NoError(t, err)
	corner := [2]float64{halfWidth, math.Sqrt(discR*discR - halfWidth*halfWidth)}
	rounded, err := decad.Fillet(t.Context(), joined, cornerEdges(t, joined, [][2]float64{corner}),
		units.Millimeters(radius))
	require.NoError(t, err)
	require.True(t, rounded.IsSolid())
	v1, err := rounded.Volume()
	require.NoError(t, err)
	want := vertexGap(t, discR, radius, corner, [2]float64{halfWidth, top}) * height
	gain := v1.Value.Base() - v0.Value.Base()
	require.InDelta(t, want, gain, v0.Bound.Base()+v1.Bound.Base()+1e-9, "within the stated volume bounds")
	require.InEpsilon(t, want, gain, 1e-2, "the gain matches the integrated gap")
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified(), "the filleted solid is watertight")
	require.True(t, mesh.VolumeVerified())
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
	t.Run("fillets resting on line ends whose feet cross on the root fragment", func(t *testing.T) {
		// The root fragment between two wide lobes spans 12 degrees. Each radius-0.45
		// fillet consumes its 0.35 line and rests on the vertex where the line meets
		// a flank leaning 0.2 off it; its circle foot lies about 7.5 degrees along
		// the fragment, so the second foot lies before the first.
		t.Parallel()
		const r, height, n, line = 3.0, 2.0, 6, 0.35
		half := math.Pi/n - 6*math.Pi/180
		doc := decad.New()
		disc := circleBodyAtZ(t, doc, 0, r, 0, height)
		lobes := make([]*decad.Body, n)
		for k := range lobes {
			lobes[k] = flankLobeLoft(t, doc, r, 2*math.Pi*float64(k)/n, half, height, straightFlank(r, line, half, 0.2))
		}
		joined, err := decad.Union(t.Context(), disc, lobes...)
		require.NoError(t, err)
		next := 2*math.Pi/n - half
		sel := cornerEdges(t, joined, [][2]float64{{r * math.Cos(half), r * math.Sin(half)},
			{r * math.Cos(next), r * math.Sin(next)}})
		_, err = decad.Fillet(t.Context(), joined, sel, units.Millimeters(0.45))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "outside its circle fragment")
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

// rolledGap is the area a radius-rho fillet adds at the concave corner where a
// radial line leaves the circle of radius r about the origin at the point corner and
// continues, past its end at fit(0), as the curve fit(s), s ∈ [0, 1], with the gap
// on the clockwise side of its outward travel. The fillet circle touches the root
// circle from outside and the curve from the gap side, so the line lies wholly
// inside the region it fills. The region is integrated by the shoelace formula
// over a fine polygon of its four boundary pieces.
func rolledGap(t *testing.T, r, rho float64, corner [2]float64, fit func(s float64) ([2]float64, [2]float64)) float64 {
	t.Helper()
	center := func(s float64) [2]float64 {
		p, d := fit(s)
		l := math.Hypot(d[0], d[1])
		return [2]float64{p[0] + rho*d[1]/l, p[1] - rho*d[0]/l}
	}
	residual := func(s float64) float64 {
		c := center(s)
		return math.Hypot(c[0], c[1]) - (r + rho)
	}
	lo, hi := 0.0, 1.0
	require.Negative(t, residual(lo), "the fillet circle reaches past the line's end")
	require.Positive(t, residual(hi))
	for range 200 {
		mid := (lo + hi) / 2
		if residual(mid) < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	foot := (lo + hi) / 2
	o := center(foot)
	circleFoot := [2]float64{o[0] * r / (r + rho), o[1] * r / (r + rho)}
	const samples = 4096
	poly := [][2]float64{corner}
	for i := range samples + 1 {
		p, _ := fit(foot * float64(i) / samples)
		poly = append(poly, p)
	}
	arc := func(c [2]float64, radius float64, from, to [2]float64) {
		a0 := math.Atan2(from[1]-c[1], from[0]-c[0])
		d := math.Atan2(to[1]-c[1], to[0]-c[0]) - a0
		d = math.Remainder(d, 2*math.Pi)
		for i := 1; i <= samples; i++ {
			a := a0 + d*float64(i)/samples
			poly = append(poly, [2]float64{c[0] + radius*math.Cos(a), c[1] + radius*math.Sin(a)})
		}
	}
	end, _ := fit(foot)
	arc(o, rho, end, circleFoot)
	arc([2]float64{}, r, circleFoot, corner)
	area := 0.0
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		area += a[0]*b[1] - a[1]*b[0]
	}
	return math.Abs(area) / 2
}

// TestLoftFilletRollsPastAShortLine rounds every root corner of lobes on a disc with
// a radius whose tangent foot on the radial line would lie past the line's end. The
// fillet then consumes the line and touches the fit-spline flank beyond it. Each
// case checks the volume gain against the gap integrated independently from the
// flank's own fit interpolant, and the solid's verified tessellation.
func TestLoftFilletRollsPastAShortLine(t *testing.T) {
	t.Parallel()
	const r, height, n, line = 3.0, 2.0, 6, 0.35
	half := 0.4 * math.Pi / n
	// straight draws a straight flank from the line's end, turned toward the lobe's
	// axis by lean from the radial direction.
	straight := func(lean float64) func(f float64) (float64, float64) {
		return func(f float64) (float64, float64) {
			x := (r+line)*math.Cos(half) + 0.65*f*math.Cos(half-lean)
			y := (r+line)*math.Sin(half) + 0.65*f*math.Sin(half-lean)
			return math.Hypot(x, y), math.Atan2(y, x)
		}
	}
	cases := []struct {
		name   string
		radius float64
		flank  func(f float64) (float64, float64)
	}{
		{"a straight flank leaning off the line", 0.45, straight(0.1)},
		{"a curved flank bending off the line", 0.45, func(f float64) (float64, float64) {
			return r + line + f*0.65, half * (1 - 0.4*f*f)
		}},
		{"a curved flank with a larger radius", 0.6, func(f float64) (float64, float64) {
			return r + line + f*0.65, half * (1 - 0.4*f*f)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			disc := circleBodyAtZ(t, doc, 0, r, 0, height)
			lobes := make([]*decad.Body, n)
			var corners [][2]float64
			for k := range lobes {
				rot := 2 * math.Pi * float64(k) / n
				lobes[k] = flankLobeLoft(t, doc, r, rot, half, height, tc.flank)
				for _, a := range []float64{-half, half} {
					corners = append(corners, [2]float64{r * math.Cos(a+rot), r * math.Sin(a+rot)})
				}
			}
			joined, err := decad.Union(t.Context(), disc, lobes...)
			require.NoError(t, err)
			v0, err := joined.Volume()
			require.NoError(t, err)
			rounded, err := decad.Fillet(t.Context(), joined, cornerEdges(t, joined, corners), units.Millimeters(tc.radius))
			require.NoError(t, err)
			require.True(t, rounded.IsSolid())
			v1, err := rounded.Volume()
			require.NoError(t, err)

			// The right flank of the unturned lobe, as the section records it: a
			// natural cubic through its fixed fit points, from the line's end outward.
			const fitPoints = 5
			coords := make([][2]float64, fitPoints)
			for i := range coords {
				rad, a := tc.flank(float64(i) / (fitPoints - 1))
				coords[i] = [2]float64{rad * math.Cos(-a), rad * math.Sin(-a)}
			}
			interp, err := geom.NewFitInterpolant(coords)
			require.NoError(t, err)
			fit := func(s float64) ([2]float64, [2]float64) {
				x, y := interp.Eval(s)
				dx, dy := interp.EvalDeriv(s)
				return [2]float64{x, y}, [2]float64{dx, dy}
			}
			gap := rolledGap(t, r, tc.radius, [2]float64{r * math.Cos(-half), r * math.Sin(-half)}, fit)
			want := 2 * n * gap * height
			gain := v1.Value.Base() - v0.Value.Base()
			require.InDelta(t, want, gain, v0.Bound.Base()+v1.Bound.Base()+1e-9, "within the stated volume bounds")
			// The loft's volume reading is chorded: the concave connector's chords lie
			// on the gap side and add a little material, so the gain sits about half
			// a percent over the ideal gap. The bounds above are loose, so also pin it.
			require.InEpsilon(t, want, gain, 1e-2, "the gain matches the integrated gap")

			mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified(), "the filleted solid is watertight")
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// straightFlank draws a straight flank for flankLobeLoft: a radial line of the
// given length from the circle of radius r at angle half, then 0.65 along a
// direction turned toward the lobe's axis by lean from the radial direction.
func straightFlank(r, line, half, lean float64) func(f float64) (float64, float64) {
	return func(f float64) (float64, float64) {
		x := (r+line)*math.Cos(half) + 0.65*f*math.Cos(half-lean)
		y := (r+line)*math.Sin(half) + 0.65*f*math.Sin(half-lean)
		return math.Hypot(x, y), math.Atan2(y, x)
	}
}

// vertexGap is the area a radius-rho fillet adds at the concave corner where a line
// leaves the circle of radius r about the origin at the point corner and ends at
// vertex, when the fillet circle touches the root circle from outside and passes
// through vertex. The fillet circle's centre is found by bisection along the root
// circle's offset, from the vertex's angle toward the gap on the clockwise side.
// The region is integrated by the shoelace formula over a fine polygon of its
// three boundary pieces.
func vertexGap(t *testing.T, r, rho float64, corner, vertex [2]float64) float64 {
	t.Helper()
	a := math.Atan2(vertex[1], vertex[0])
	center := func(s float64) [2]float64 {
		return [2]float64{(r + rho) * math.Cos(a-s), (r + rho) * math.Sin(a-s)}
	}
	reach := func(s float64) float64 {
		c := center(s)
		return math.Hypot(c[0]-vertex[0], c[1]-vertex[1]) - rho
	}
	lo, hi := 0.0, math.Pi/2
	require.Negative(t, reach(lo), "the fillet circle reaches past the vertex")
	require.Positive(t, reach(hi))
	for range 200 {
		mid := (lo + hi) / 2
		if reach(mid) < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	o := center(lo)
	circleFoot := [2]float64{o[0] * r / (r + rho), o[1] * r / (r + rho)}
	const samples = 4096
	poly := [][2]float64{corner, vertex}
	arc := func(c [2]float64, radius float64, from, to [2]float64) {
		a0 := math.Atan2(from[1]-c[1], from[0]-c[0])
		d := math.Atan2(to[1]-c[1], to[0]-c[0]) - a0
		d = math.Remainder(d, 2*math.Pi)
		for i := 1; i <= samples; i++ {
			a := a0 + d*float64(i)/samples
			poly = append(poly, [2]float64{c[0] + radius*math.Cos(a), c[1] + radius*math.Sin(a)})
		}
	}
	arc(o, rho, vertex, circleFoot)
	arc([2]float64{}, r, circleFoot, corner)
	area := 0.0
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		area += a[0]*b[1] - a[1]*b[0]
	}
	return math.Abs(area) / 2
}

// TestLoftFilletRestsOnTheLineFlankVertex rounds every root corner of lobes on a
// disc with a radius that consumes the radial line, where the flank leaves the
// line turned toward the lobe's axis by more than the fillet circle through the
// line's far end turns there. The fillet circle then rests on that vertex: it
// touches the root circle and passes through the vertex, and the flank stays
// whole. Each case checks the volume gain against the gap integrated from an
// independently found fillet circle, and the solid's verified tessellation.
func TestLoftFilletRestsOnTheLineFlankVertex(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 6
	half := 0.4 * math.Pi / n
	cases := []struct {
		name         string
		line, radius float64
		flank        func(f float64) (float64, float64)
	}{
		{"a radius just past a short line", 0.35, 0.38, straightFlank(r, 0.35, half, 0.1)},
		{"a shorter line and a larger radius", 0.2, 0.3, straightFlank(r, 0.2, half, 0.4)},
		{"a flank turned far off the line", 0.35, 0.45, func(f float64) (float64, float64) {
			return r + 0.35 + f*0.65, half * (1 - 0.4*f)
		}},
		{"a curved flank", 0.35, 0.4, func(f float64) (float64, float64) {
			return r + 0.35 + f*0.65, half * (1 - 0.25*f - 0.15*f*f)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			disc := circleBodyAtZ(t, doc, 0, r, 0, height)
			lobes := make([]*decad.Body, n)
			var corners [][2]float64
			for k := range lobes {
				rot := 2 * math.Pi * float64(k) / n
				lobes[k] = flankLobeLoft(t, doc, r, rot, half, height, tc.flank)
				for _, a := range []float64{-half, half} {
					corners = append(corners, [2]float64{r * math.Cos(a+rot), r * math.Sin(a+rot)})
				}
			}
			joined, err := decad.Union(t.Context(), disc, lobes...)
			require.NoError(t, err)
			v0, err := joined.Volume()
			require.NoError(t, err)
			rounded, err := decad.Fillet(t.Context(), joined, cornerEdges(t, joined, corners), units.Millimeters(tc.radius))
			require.NoError(t, err)
			require.True(t, rounded.IsSolid())
			v1, err := rounded.Volume()
			require.NoError(t, err)

			// The right side of the unturned lobe: its line runs from the root
			// circle at angle -half to the flank's first fit point.
			rad, a := tc.flank(0)
			vertex := [2]float64{rad * math.Cos(-a), rad * math.Sin(-a)}
			gap := vertexGap(t, r, tc.radius, [2]float64{r * math.Cos(-half), r * math.Sin(-half)}, vertex)
			want := 2 * n * gap * height
			gain := v1.Value.Base() - v0.Value.Base()
			require.InDelta(t, want, gain, v0.Bound.Base()+v1.Bound.Base()+1e-9, "within the stated volume bounds")
			// The loft's volume reading is chorded: the concave connector's chords lie
			// on the gap side and add a little material. The bounds above are loose,
			// so also pin the gain to the integrated gap.
			require.InEpsilon(t, want, gain, 1e-2, "the gain matches the integrated gap")

			mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified(), "the filleted solid is watertight")
			require.True(t, mesh.VolumeVerified())
		})
	}
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
