package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// windowedHolePlate is a 40x40x5 plate with four windows at N, E, S, W and a
// round hole of radius 2 at (-14, -14). Each window's inner arc lies on the
// Ø14 boss circle (about the origin, radius just above 7) and its sides are
// axis-aligned, so the boss union is a holed A1 brep whose merged carrier
// states a CarrierCharge, plus a round hole whose full wall carries the
// record's displacement.
func windowedHolePlate(t *testing.T) *decad.Body {
	t.Helper()
	hub := math.Nextafter(7, 8)
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	o := s.Origin()
	c := [4]*sketch.Point{s.CreatePoint(-20, -20), s.CreatePoint(20, -20), s.CreatePoint(20, 20), s.CreatePoint(-20, 20)}
	for i := range 4 {
		s.CreateLine(c[i], c[(i+1)%4])
	}
	y := math.Sqrt(hub*hub - 4)
	rot := func(k int, x, y float64) (float64, float64) {
		switch k {
		case 0:
			return x, y
		case 1:
			return -y, x
		case 2:
			return -x, -y
		}
		return y, -x
	}
	for k := range 4 {
		pt := func(x, y float64) *sketch.Point {
			px, py := rot(k, x, y)
			return s.CreatePoint(px, py)
		}
		a, b := pt(2, y), pt(-2, y)
		d, e := pt(2, 14), pt(-2, 14)
		s.CreateArc(o, a, b)
		s.CreateLine(a, d)
		s.CreateLine(d, e)
		s.CreateLine(e, b)
	}
	s.CreateCircle(s.CreatePoint(-14, -14), 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 5 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(t.Context(), decad.New(), s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestStackedBrepViewFilletChargedWall fillets the top rim of a round hole in
// the holed A1 union of windowedHolePlate and its Ø14 boss. The hole's wall is
// a swept circular wall of the record, which lies within the record's
// displacement of the cylinder it denotes. The view keeps its Cylinder tag and
// states that displacement on it, so the retained edge blend builds the rim
// round and keeps the blend face faceted. The removed volume is that of a
// fillet of radius 0.5 on a rim of radius 2, and the charged wall is not
// published as an analytic face. The charge's growth of the bounds against an
// uncharged twin is shown in internal/retainedblend (TestFilletChargesCylinderWall).
//
// Shown red: without the view's wall charge the hole wall is untagged in the
// view and the rim keeps the receivers' section-displacement refusal
// (brep-modify SB1).
func TestStackedBrepViewFilletChargedWall(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	plate := windowedHolePlate(t)
	out, err := bossOnCap(t, plate, 14)
	require.NoError(t, err)
	require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(out))
	var rim r3.Vec
	var found bool
	for _, e := range out.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok || e.Start().Position().Value.Z != 5 {
			continue
		}
		r, err := c.Radius.In(units.Millimeter)
		require.NoError(t, err)
		if math.Abs(r-2) < 1e-9 && math.Abs(c.Center.X+14) < 1e-9 && math.Abs(c.Center.Y+14) < 1e-9 {
			rim, found = e.Start().Position().Value, true
		}
	}
	require.True(t, found, "the hole's top rim")

	before, err := out.Volume()
	require.NoError(t, err)
	edge := decad.Edges(decad.EndpointAt(rim), decad.Circular()).Exactly(1)
	filleted, err := decad.Fillet(t.Context(), out, edge, mm(0.5))
	require.NoError(t, err)
	require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(filleted))
	after, err := filleted.Volume()
	require.NoError(t, err)
	const r, rho = 2.0, 0.5
	k := (10 - 3*math.Pi) / (12 - 3*math.Pi)
	removed := volumeMM(t, before) - volumeMM(t, after)
	require.InDelta(t, 2*math.Pi*(1-math.Pi/4)*rho*rho*(r+k*rho), removed, boundMM3(t, before)+boundMM3(t, after))

	for _, f := range filleted.Faces() {
		_, torus := f.Surface().(decad.Torus)
		require.False(t, torus, "the charged blend face stays faceted")
		if c, ok := f.Surface().(decad.Cylinder); ok {
			radius, err := c.Radius.In(units.Millimeter)
			require.NoError(t, err)
			require.NotEqual(t, r, radius, "the charged wall is not published")
		}
	}
}
