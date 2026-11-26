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
	body, err := decad.Extrude(decad.New(), s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// TestStackedBrepViewFilletChargedWall fillets the top rim of a round hole in
// the holed A1 union of windowedHolePlate and its Ø14 boss. The hole's wall is
// a swept circular wall of the record, which lies within the record's
// displacement of the cylinder it denotes. The view keeps its Cylinder tag and
// states that displacement on it, so the retained edge blend builds the rim
// round and keeps the blend face faceted. The removed volume is that of a
// fillet of radius 0.5 on a rim of radius 2. The blend publishes the charged
// wall: the result's hole wall is still a Cylinder of radius 2, stating the
// displacement. Later operations read it again: the hole's bottom rim chamfers
// with the exact ring volume removed, a mesh Cut away from the hole keeps the
// wall and the rim still chamfers, and an open arc on a charged wall (a
// window's bottom inner arc) is refused. The charge's growth of the bounds
// against an uncharged twin is shown in internal/retainedblend
// (TestFilletChargesCylinderWall, TestFilletPublishesChargedCylinderWall).
//
// Shown red: without the view's wall charge the hole wall is untagged in the
// view and the rim keeps the receivers' section-displacement refusal
// (brep-modify SB1). Without the blend's publication the wall reads Faceted
// after the fillet and the subtests find no Cylinder of radius 2.
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
	filleted, err := decad.Fillet(out, edge, mm(0.5)).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(filleted))
	after, err := filleted.Volume()
	require.NoError(t, err)
	const r, rho = 2.0, 0.5
	k := (10 - 3*math.Pi) / (12 - 3*math.Pi)
	removed := volumeMM(t, before) - volumeMM(t, after)
	require.InDelta(t, 2*math.Pi*(1-math.Pi/4)*rho*rho*(r+k*rho), removed, boundMM3(t, before)+boundMM3(t, after))

	var wall *decad.Cylinder
	for _, f := range filleted.Faces() {
		_, torus := f.Surface().(decad.Torus)
		require.False(t, torus, "the charged blend face stays faceted")
		c, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		radius, err := c.Radius.In(units.Millimeter)
		require.NoError(t, err)
		if math.Abs(radius-r) < 1e-9 {
			wall = &c
		}
	}
	require.NotNil(t, wall, "the hole wall is published as a Cylinder of radius 2")

	// bottomRim is the hole's rim at z = 0, a circle of radius 2 about (-14, -14).
	bottomRim := func(t *testing.T, b *decad.Body) r3.Vec {
		t.Helper()
		var at r3.Vec
		n := 0
		for _, e := range b.Edges() {
			c, ok := e.Curve().(decad.Circle3)
			if !ok || e.Start().Position().Value.Z != 0 {
				continue
			}
			radius, err := c.Radius.In(units.Millimeter)
			require.NoError(t, err)
			if math.Abs(radius-r) < 1e-9 && math.Abs(c.Center.X+14) < 1e-9 && math.Abs(c.Center.Y+14) < 1e-9 {
				at = e.Start().Position().Value
				n++
			}
		}
		require.Equal(t, 1, n, "the hole's bottom rim is one Circle3 of radius 2 about (-14, -14)")
		return at
	}
	// chamfered is the body with the hole's bottom rim chamfered by 0.5. The removed ring has a right triangle
	// of legs d and area d^2/2 as its section, with its centroid at radius r + d/3.
	chamfered := func(t *testing.T, b *decad.Body) {
		t.Helper()
		const d = 0.5
		beforeVol, err := b.Volume()
		require.NoError(t, err)
		edge := decad.Edges(decad.EndpointAt(bottomRim(t, b)), decad.Circular()).Exactly(1)
		out, err := decad.Chamfer(b, edge, mm(d)).Do(t.Context())
		require.NoError(t, err)
		afterVol, err := out.Volume()
		require.NoError(t, err)
		removedVol := volumeMM(t, beforeVol) - volumeMM(t, afterVol)
		require.InDelta(t, 2*math.Pi*(r+d/3)*d*d/2, removedVol, boundMM3(t, beforeVol)+boundMM3(t, afterVol))
	}

	// Each subtest takes its own copy: a Chamfer or Cut retires the body it reads. The copies share one Document,
	// which is not safe for concurrent use, so the subtests run in turn.
	copies := make([]*decad.Body, 3)
	for i := range copies {
		copies[i], err = decad.Copy(filleted).Do(t.Context())
		require.NoError(t, err)
	}

	t.Run("the published wall's bottom rim chamfers", func(t *testing.T) {
		chamfered(t, copies[0])
	})

	t.Run("a mesh Cut away from the hole keeps the wall and the rim chamfers", func(t *testing.T) {
		doc := copies[1].Document()
		tool := translated(t, boxBody(t, doc, 0, 0, 4, 4, 7), 10, 10, -1)
		cut, err := decad.Cut(copies[1], tool).Do(t.Context())
		require.NoError(t, err)
		require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(cut))
		var kept bool
		for _, f := range cut.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok {
				radius, err := c.Radius.In(units.Millimeter)
				require.NoError(t, err)
				kept = kept || math.Abs(radius-r) < 1e-9
			}
		}
		require.True(t, kept, "the Cut keeps the charged hole wall as a Cylinder of radius 2")
		chamfered(t, cut)
	})

	t.Run("a window's bottom inner arc on the charged wall is refused", func(t *testing.T) {
		var at r3.Vec
		n := 0
		for _, e := range copies[2].Edges() {
			c, ok := e.Curve().(decad.Arc3)
			if !ok || e.Start().Position().Value.Z != 0 {
				continue
			}
			radius, err := c.Radius.In(units.Millimeter)
			require.NoError(t, err)
			if math.Abs(radius-7) < 0.01 {
				at = e.Start().Position().Value
				n++
			}
		}
		require.Positive(t, n, "the windows' bottom inner arcs are Arc3 edges")
		edge := decad.Edges(decad.EndpointAt(at), decad.Circular()).Exactly(1)
		_, err := decad.Chamfer(copies[2], edge, mm(0.5)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "an arc on a wall whose tag states a displacement is not built")
	})
}
