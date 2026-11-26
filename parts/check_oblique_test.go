package parts_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
)

// obliqueHubDisc extrudes a disc of radius 21, 5 thick from z = 0, with four windows between arms 4 wide whose
// sides run at 45 degrees to the axes. Each window's inner wall is an arc of radius 7 about the axis, and every
// window corner is rounded with radius 2.
func obliqueHubDisc(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	const (
		hub      = 7.0
		rimInner = 18.0
		armHalf  = 2.0
		round    = 2.0
	)
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	o := s.Origin()
	x1 := s.CreatePoint(10, 0)
	ref := s.CreateLine(o, x1)
	ref.SetConstruction(true)
	s.AddConstraint(sketch.NewHorizontal(ref), sketch.NewDistance(o, x1, 10))
	rim := s.CreateCircle(o, 21)
	s.AddConstraint(sketch.NewRadius(rim, 21))
	for k := range 4 {
		s1 := math.Pi/4 + math.Pi/2*float64(k)
		s2 := s1 + math.Pi/2
		di, do := math.Asin(armHalf/hub), math.Asin(armHalf/rimInner)
		a := s.CreatePoint(hub*math.Cos(s1+di), hub*math.Sin(s1+di))
		bb := s.CreatePoint(hub*math.Cos(s2-di), hub*math.Sin(s2-di))
		c := s.CreatePoint(rimInner*math.Cos(s2-do), rimInner*math.Sin(s2-do))
		d := s.CreatePoint(rimInner*math.Cos(s1+do), rimInner*math.Sin(s1+do))
		inner := s.CreateArc(o, a, bb)
		outer := s.CreateArc(o, d, c)
		side1 := s.CreateLine(a, d)
		side2 := s.CreateLine(bb, c)
		s.AddConstraint(
			sketch.NewRadius(inner, hub), sketch.NewRadius(outer, rimInner),
			sketch.NewAngle(ref, side1, s1*180/math.Pi), sketch.NewDistancePointLine(o, side1, armHalf),
			sketch.NewAngle(ref, side2, s2*180/math.Pi), sketch.NewDistancePointLine(o, side2, armHalf),
		)
		_, err = sketch.CreateFillet(s, inner, side1, a, round)
		require.NoError(t, err)
		_, err = sketch.CreateFillet(s, side2, inner, bb, round)
		require.NoError(t, err)
		_, err = sketch.CreateFillet(s, outer, side2, c, round)
		require.NoError(t, err)
		_, err = sketch.CreateFillet(s, side1, outer, d, round)
		require.NoError(t, err)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 4 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	body, err := decad.Extrude(doc, s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return doc, body
}

// obliqueSideBlock is a box 5 tall whose one face lies on the plane of the hub's window side n . x = 2, with
// n = (1, -1) / sqrt2, and stands in the window beside it, across the side's straight stretch.
func obliqueSideBlock(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	n, d := r3.Vec{X: math.Sqrt2 / 2, Y: -math.Sqrt2 / 2}, r3.Vec{X: math.Sqrt2 / 2, Y: math.Sqrt2 / 2}
	at := func(t, depth float64) *sketch.Point {
		p := n.Scale(2 + depth).Add(d.Scale(t))
		return s.CreatePoint(p.X, p.Y)
	}
	p0, p1, p2, p3 := at(10, 0), at(15, 0), at(15, 1), at(10, 1)
	s.CreateLine(p0, p1)
	s.CreateLine(p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p0)
	s.Fix(p0)
	s.Fix(p1)
	s.Fix(p2)
	s.Fix(p3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return b
}

// A block pressed on a window side of the spoked hub reads Undecided with CauseUnreadFace once the hub's root-
// filleted boss states the side's tag as a plane that may lie off the true wall (ExtentDelta,
// charged-window-chamfer-design §2): the tag's plane is no measurement. The same joint on the hub disc alone, whose
// sides are exact planes, decides.
func TestCheckObliqueWallUnread(t *testing.T) {
	t.Parallel()
	check := func(t *testing.T, hub *decad.Body, doc *decad.Document) parts.JointResult {
		t.Helper()
		s := parts.NewSet(doc)
		mustAdd(t, s,
			parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: hub},
			parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: obliqueSideBlock(t, doc)})
		mustJoin(t, s, parts.Joint{Kind: parts.FaceContact, A: nameBlock, B: namePlate})
		return checkOne(t, s)
	}

	doc, disc := obliqueHubDisc(t)
	twin := check(t, disc, doc)
	require.Equal(t, parts.Confirmed, twin.Status, twin.Reason)

	boss, err := decad.Boss(t.Context(), disc, decad.BossSpec{
		Face: decad.Faces(decad.FaceCreatedBy(decad.CapEnd(disc))).Exactly(1), At: r3.Vec{Z: 5},
		Diameter: units.Millimeters(14), Height: units.Millimeters(1), RootFillet: units.Millimeters(0.5)})
	require.NoError(t, err)
	got := check(t, boss, doc)
	require.Equal(t, parts.Undecided, got.Status, got.Reason)
	require.Equal(t, parts.CauseUnreadFace, got.Cause)
}
