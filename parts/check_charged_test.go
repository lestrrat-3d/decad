package parts_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// chargedHolePlate is a holed stacked brep plate (a 40 x 40 x 5 plate, four windows, a round hole of radius 2 at
// (-14, -14), and a boss) whose hole top rim is filleted. The fillet leaves the hole wall a Cylinder tag that
// states a wall charge (retained-edge-blend §6.1, "Publishing a charged wall").
func chargedHolePlate(t *testing.T, doc *decad.Document) *decad.Body {
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
	body, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	face := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).Exactly(1)
	out, err := decad.Boss(t.Context(), body, decad.BossSpec{Face: face, At: r3.Vec{Z: 5},
		Diameter: units.Millimeters(14), Height: units.Millimeters(1)})
	require.NoError(t, err)
	edge := decad.Edges(decad.EndpointAt(r3.Vec{X: -12, Y: -14, Z: 5}), decad.Circular()).Exactly(1)
	filleted, err := decad.Fillet(t.Context(), out, edge, units.Millimeters(0.5))
	require.NoError(t, err)
	return filleted
}

// A pin in the bore of a charged wall reads Undecided with CauseUnreadFace: the wall's true radius and axis may
// differ from its tag's, so the tag's radius is no measurement. The same joint on an uncharged hole decides.
func TestCheckChargedWallUnread(t *testing.T) {
	t.Parallel()
	joint := parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate,
		Axis: &parts.Axis{Point: r3.Vec{X: -14, Y: -14}, Dir: r3.Vec{Z: 1}}}
	check := func(t *testing.T, plate *decad.Body, doc *decad.Document) parts.JointResult {
		t.Helper()
		s := parts.NewSet(doc)
		mustAdd(t, s,
			parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: plate},
			parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinderAt(t, doc, -14, -14, 1.8, -2, 9)})
		mustJoin(t, s, joint)
		return checkOne(t, s)
	}

	doc := decad.New()
	plain, err := decad.Cut(t.Context(), mustBlock(t, doc), cylinderAt(t, doc, -14, -14, 2, -1, 7))
	require.NoError(t, err)
	twin := check(t, plain, doc)
	require.Equal(t, parts.Confirmed, twin.Status, twin.Reason)
	require.InDelta(t, 0.2, twin.Measured.Value.Base(), 1e-9)

	charged := chargedHolePlate(t, doc)
	var wall bool
	for _, f := range charged.Faces() {
		if c, ok := f.Surface().(decad.Cylinder); ok {
			radius, err := c.Radius.In(units.Millimeter)
			require.NoError(t, err)
			wall = wall || radius == 2
		}
	}
	require.True(t, wall, "the filleted plate keeps the hole wall as a Cylinder tag")
	got := check(t, charged, doc)
	require.Equal(t, parts.Undecided, got.Status, got.Reason)
	require.Equal(t, parts.CauseUnreadFace, got.Cause)
	require.Nil(t, got.Measured)
}

func mustBlock(t *testing.T, doc *decad.Document) *decad.Body {
	return block(t, doc, -20, -20, 20, 20, 0, 5)
}
