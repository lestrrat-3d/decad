package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md's public-surface tests for Hole: the
// plain, blind, counterbored and countersunk holes of a 20 x 20 x 10 plate,
// and the refusals that leave the plate live.

const drillPlateVolume = 4000.0

// holePlate returns a 20 x 20 x 10 plate on the XY plane, z from 0 to 10.
func drillPlate(tb testing.TB) (*decad.Document, *decad.Body) {
	tb.Helper()
	doc := decad.New()
	return doc, decadtest.NewBlock(tb, doc, 0, 0, 20, 20, units.Millimeters(10))
}

// capFace selects the plate's top (z = 10) or bottom (z = 0) cap by the
// feature that created it.
func capFace(top bool, b *decad.Body) decad.FaceSelector {
	if top {
		return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b)))
	}
	return decad.Faces(decad.FaceCreatedBy(decad.CapStart(b)))
}

func cylinderFrustum(r0, r1, h float64) float64 {
	return math.Pi * h / 3 * (r0*r0 + r0*r1 + r1*r1)
}

func TestHoleBlind(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    decad.BlindHole{Depth: units.Millimeters(6)},
	})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the plate and the tools leave the document")
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-math.Pi*9*6))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 7, decad.KindCylinder: 1})
	_, err = decad.Hole(t.Context(), plate, decad.HoleSpec{})
	require.ErrorIs(t, err, decad.ErrRetiredBody, "the plate is retired")
}

func TestHoleThroughFromBottom(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(false, plate),
		At:       r3.Vec{X: 5, Y: 15, Z: 0},
		Diameter: units.Millimeters(4),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-math.Pi*4*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
}

func TestHoleCounterbore(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(4),
		Depth:    decad.ThroughHole{},
		Head:     decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(3)},
	})
	require.NoError(t, err)
	want := drillPlateVolume - math.Pi*4*10 - math.Pi*(16-4)*3
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want))
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 7, decad.KindCylinder: 2})
}

func TestHoleCountersink(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(4),
		Depth:    decad.ThroughHole{},
		Head:     decad.Countersink{Diameter: units.Millimeters(8), Angle: units.Degrees(90)},
	})
	require.NoError(t, err)
	// A 90 degree sink 8 wide over a bore 4 wide is 2 deep: the cone frustum
	// from radius 2 to 4 replaces the bore's top 2 mm.
	want := drillPlateVolume - math.Pi*4*10 - (cylinderFrustum(2, 4, 2) - math.Pi*4*2)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want))
	kinds := map[decad.SurfaceKind]int{}
	for _, f := range out.Faces() {
		kinds[f.Surface().Kind()]++
	}
	require.Equal(t, 1, kinds[decad.KindCylinder], "the bore wall")
	require.Equal(t, 1, kinds[decad.KindCone], "the sink wall")
}

func TestHoleRefusals(t *testing.T) {
	t.Parallel()
	good := func(plate *decad.Body) decad.HoleSpec {
		return decad.HoleSpec{
			Face:     capFace(true, plate),
			At:       r3.Vec{X: 10, Y: 10, Z: 10},
			Diameter: units.Millimeters(4),
			Depth:    decad.BlindHole{Depth: units.Millimeters(5)},
		}
	}
	cases := []struct {
		name string
		edit func(*decad.HoleSpec)
		want error
	}{
		{"no selector", func(s *decad.HoleSpec) { s.Face = nil }, decad.ErrDegenerate},
		{"no depth", func(s *decad.HoleSpec) { s.Depth = nil }, decad.ErrDegenerate},
		{"zero diameter", func(s *decad.HoleSpec) { s.Diameter = units.Millimeters(0) }, decad.ErrDegenerate},
		{"wrong diameter kind", func(s *decad.HoleSpec) { s.Diameter = units.Degrees(4) }, decad.ErrUnitKind},
		{"zero depth", func(s *decad.HoleSpec) { s.Depth = decad.BlindHole{Depth: units.Millimeters(0)} }, decad.ErrDegenerate},
		{"centre off the plane", func(s *decad.HoleSpec) { s.At = r3.Vec{X: 10, Y: 10, Z: 9} }, decad.ErrDegenerate},
		{"centre outside the body", func(s *decad.HoleSpec) { s.At = r3.Vec{X: 40, Y: 10, Z: 10} }, decad.ErrDegenerate},
		{"counterbore as wide as the hole", func(s *decad.HoleSpec) {
			s.Head = decad.Counterbore{Diameter: units.Millimeters(4), Depth: units.Millimeters(1)}
		}, decad.ErrDegenerate},
		{"counterbore as deep as the hole", func(s *decad.HoleSpec) {
			s.Head = decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(5)}
		}, decad.ErrDegenerate},
		{"countersink deeper than the hole", func(s *decad.HoleSpec) {
			s.Head = decad.Countersink{Diameter: units.Millimeters(40), Angle: units.Degrees(90)}
		}, decad.ErrDegenerate},
		{"countersink angle of a straight line", func(s *decad.HoleSpec) {
			s.Head = decad.Countersink{Diameter: units.Millimeters(8), Angle: units.Degrees(180)}
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := drillPlate(t)
			spec := good(plate)
			tc.edit(&spec)
			_, err := decad.Hole(t.Context(), plate, spec)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused hole leaves the plate live and alone")
		})
	}
}

func TestHoleRefusesNonPlanarFace(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	withBore, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	_, err = decad.Hole(t.Context(), withBore, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.Vec{X: 13, Y: 10, Z: 5},
		Diameter: units.Millimeters(1),
		Depth:    decad.ThroughHole{},
	})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{withBore}, doc.Bodies())
}

// A plate turned onto its side takes the hole in its rotated top cap: the hole
// follows the face's own frame, not the world axes.
func TestHoleOnRotatedPlate(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	turn, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 1, 0), units.Degrees(90))
	require.NoError(t, err)
	turned, err := decad.Transform(t.Context(), plate, turn)
	require.NoError(t, err)
	// The top cap (z = 10) turns to x = 10; its centre (10, 10, 10) maps to
	// (10, 10, -10).
	out, err := decad.Hole(t.Context(), turned, decad.HoleSpec{
		Face:     capFace(true, turned),
		At:       turn.Apply(r3.Vec{X: 10, Y: 10, Z: 10}),
		Diameter: units.Millimeters(6),
		Depth:    decad.BlindHole{Depth: units.Millimeters(6)},
	})
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-math.Pi*9*6))
}
