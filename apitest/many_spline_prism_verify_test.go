package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// wavyOutlineBody extrudes a closed outline r(t) = 20 + 2 cos(lobes t) drawn as
// two fit splines per lobe, each through pointsPerSpline fixed points: one over
// the convex arc about the peak and one over the concave arc about the valley,
// so no spline crosses an inflection.
func wavyOutlineBody(t *testing.T, lobes, pointsPerSpline int) *decad.Body {
	t.Helper()
	const r0, amp = 20.0, 2.0
	n := float64(lobes)
	radius := func(a float64) float64 { return r0 + amp*math.Cos(n*a) }
	// Curvature numerator r^2 + 2 r'^2 - r r'' changes sign once on (0, pi/n).
	kappa := func(a float64) float64 {
		r, d1, d2 := radius(a), -amp*n*math.Sin(n*a), -amp*n*n*math.Cos(n*a)
		return r*r + 2*d1*d1 - r*d2
	}
	lo, hi := 0.0, math.Pi/n
	for range 80 {
		mid := (lo + hi) / 2
		if kappa(mid) > 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	inflect := (lo + hi) / 2
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	point := func(a float64) *sketch.Point {
		p := s.CreatePoint(radius(a)*math.Cos(a), radius(a)*math.Sin(a))
		s.Fix(p)
		return p
	}
	// Arc boundaries, in angle order: -inflect, +inflect, 2pi/n-inflect, ...
	var bounds []float64
	for k := range lobes {
		base := 2 * math.Pi * float64(k) / n
		bounds = append(bounds, base+inflect, base+2*math.Pi/n-inflect)
	}
	first := point(bounds[len(bounds)-1] - 2*math.Pi)
	start := first
	for i, end := range bounds {
		from := bounds[(i+len(bounds)-1)%len(bounds)]
		if i == 0 {
			from = bounds[len(bounds)-1] - 2*math.Pi
		}
		pts := []*sketch.Point{start}
		for j := 1; j < pointsPerSpline-1; j++ {
			pts = append(pts, point(from+(end-from)*float64(j)/float64(pointsPerSpline-1)))
		}
		if i == len(bounds)-1 {
			pts = append(pts, first)
		} else {
			pts = append(pts, point(end))
		}
		_, err := s.CreateFitSpline(pts...)
		require.NoError(t, err)
		start = pts[len(pts)-1]
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(8)})
	require.NoError(t, err)
	return body
}

// TestManySplineOutlineTessellatesUnderVerifyAll extrudes an outline of 30 fit
// splines and tessellates it with full verification. The build raises its work
// ceiling with the record's cubic piece count, and replaying that build's charge
// at tessellation must meet the same ceiling.
func TestManySplineOutlineTessellatesUnderVerifyAll(t *testing.T) {
	t.Parallel()
	body := wavyOutlineBody(t, 15, 14)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := body.Volume()
	require.NoError(t, err)
	// Area of r = 20 + 2 cos(15 t) is pi (400 + 2) (the cosine term integrates to its mean square).
	want := math.Pi * (400 + 2) * 8
	require.InDelta(t, want, volume.Value.Base(), 0.01*want)
}
