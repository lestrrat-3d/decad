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

// This file is agent-docs/emboss-design.md's public-surface tests for Emboss:
// a 40 x 30 x 10 plate (z from 0 to 10) takes raised and engraved reliefs on
// its top and bottom faces.

const embossPlateVolume = 40 * 30 * 10

func embossPlate(tb testing.TB) (*decad.Document, *decad.Body) {
	tb.Helper()
	doc := decad.New()
	return doc, decadtest.NewBlock(tb, doc, 0, 0, 40, 30, units.Millimeters(10))
}

// embossSketch returns a sketch on the plane z = z with one rectangle per
// entry of rects, and the profile of each.
func embossSketch(tb testing.TB, z float64, rects ...[4]float64) (*sketch.Sketch, []*sketch.Profile) {
	tb.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(tb, err)
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	for _, r := range rects {
		rect := s.CreateRectangle(r[0], r[1], r[2], r[3])
		s.Fix(rect.A)
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	require.Len(tb, s.Profiles(), len(rects))
	return s, s.Profiles()
}

func embossFace(top bool, b *decad.Body) decad.FaceSelector {
	if top {
		return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b)))
	}
	return decad.Faces(decad.FaceCreatedBy(decad.CapStart(b)))
}

func TestEmbossRaisedOnTop(t *testing.T) {
	t.Parallel()
	doc, plate := embossPlate(t)
	s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	operations := doc.OperationHistory().Operations()
	require.Len(t, operations, 2)
	require.Equal(t, decad.OperationEmboss, operations[1].Kind())
	require.Equal(t, []decad.OperationInput{{Body: plate, Role: "target", Use: decad.InputConsumed}}, operations[1].Inputs())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume+10*6*2))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 40, Y: 30, Z: 12})
}

func TestEmbossEngravedOnTop(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossEngraved, units.Millimeters(2))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume-10*6*2))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 40, Y: 30, Z: 10})
}

// The bottom cap faces -z while the sketch normal faces +z, so the tool sides
// swap: a raised relief stands below the plate.
func TestEmbossRaisedOnBottom(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossSketch(t, 0, [4]float64{10, 10, 20, 16}, [4]float64{25, 10, 35, 16})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(false, plate), decad.EmbossRaised, units.Millimeters(3))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume+2*10*6*3))
	decadtest.MeasuresBounds(t, out, r3.Vec{Z: -3}, r3.Vec{X: 40, Y: 30, Z: 10})
}

func TestEmbossEngravedOnBottom(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossSketch(t, 0, [4]float64{10, 10, 20, 16})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(false, plate), decad.EmbossEngraved, units.Millimeters(3))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume-10*6*3))
}

// A profile with a hole keeps the hole: a ring relief encloses an untouched
// island of the face.
func TestEmbossRingKeepsItsHole(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	outer := s.CreateRectangle(10, 10, 20, 20)
	s.Fix(outer.A)
	inner := s.CreateRectangle(13, 13, 17, 17)
	s.Fix(inner.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var ring *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			ring = p
		}
	}
	require.NotNil(t, ring)
	out, err := decad.Emboss(t.Context(), plate, s, []*sketch.Profile{ring}, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume+(100-16)*2))
}

func TestEmbossCircleRelief(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(20, 15)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	out, err := decad.Emboss(t.Context(), plate, s, s.Profiles(), embossFace(true, plate), decad.EmbossEngraved, units.Millimeters(4))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume-math.Pi*25*4))
}

func TestEmbossRefusals(t *testing.T) {
	t.Parallel()
	rect := [4]float64{10, 10, 20, 16}
	cases := []struct {
		name   string
		z      float64
		rect   [4]float64
		mode   decad.EmbossMode
		height units.Value
		want   error
		text   string
	}{
		{"zero height", 10, rect, decad.EmbossRaised, units.Millimeters(0), decad.ErrDegenerate, ""},
		{"wrong height kind", 10, rect, decad.EmbossRaised, units.Degrees(2), decad.ErrUnitKind, ""},
		{"unknown mode", 10, rect, decad.EmbossMode(9), units.Millimeters(2), decad.ErrDegenerate, "unknown emboss mode"},
		{"sketch off the face plane", 9, rect, decad.EmbossRaised, units.Millimeters(2), decad.ErrDegenerate, "off the face plane"},
		{"relief beside the body", 10, [4]float64{100, 10, 110, 16}, decad.EmbossRaised, units.Millimeters(2),
			decad.ErrDegenerate, "would not reach"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := embossPlate(t)
			s, profiles := embossSketch(t, tc.z, tc.rect)
			_, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), tc.mode, tc.height)
			require.ErrorIs(t, err, tc.want)
			require.ErrorContains(t, err, tc.text)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused emboss leaves the plate live and alone")
		})
	}
}

func TestEmbossRefusesTiltedSketchAndUntangentCurvedFace(t *testing.T) {
	t.Parallel()
	doc, plate := embossPlate(t)
	w := sketch.NewWorld()
	// A plane through the top face's plane origin, standing on it.
	upright, err := r3.NewFrame(r3.Vec{Z: 10}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(upright)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(10, 0, 20, 5)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = decad.Emboss(t.Context(), plate, s, s.Profiles(), embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "not parallel")

	flat, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
	_, err = decad.Emboss(t.Context(), plate, flat, nil, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrDegenerate, "an empty profile list")
	_, err = decad.Emboss(t.Context(), plate, flat, []*sketch.Profile{nil}, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil profile")
	_, err = decad.Emboss(t.Context(), plate, flat, profiles, nil, decad.EmbossRaised, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil selector")
	require.Equal(t, []*decad.Body{plate}, doc.Bodies())

	drilled, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face: embossFace(true, plate), At: r3.Vec{X: 20, Y: 15, Z: 10}, Diameter: units.Millimeters(6), Depth: decad.ThroughHole{},
	})
	require.NoError(t, err)
	// A sketch on the top face's plane is not tangent to the hole's wall, so it names no relief on it.
	_, err = decad.Emboss(t.Context(), drilled, flat, profiles, decad.Faces(decad.Cylindrical()), decad.EmbossRaised, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "not parallel")
}
