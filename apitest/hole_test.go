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
	out, err := decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10},
		units.Millimeters(6), decad.BlindHole{Depth: units.Millimeters(6)}).
		Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the plate and the tools leave the document")
	operations := doc.OperationHistory().Operations()
	require.Len(t, operations, 2)
	require.Equal(t, decad.OperationHole, operations[1].Kind())
	require.Equal(t, []decad.OperationInput{{Body: plate, Role: "target", Use: decad.InputConsumed}}, operations[1].Inputs())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-math.Pi*9*6))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 7, decad.KindCylinder: 1})
	_, err = decad.Hole(plate, nil, r3.Vec{}, units.Value{}, nil).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody, "the plate is retired")
}

func TestHoleThroughFromBottom(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(plate, capFace(false, plate), r3.Vec{X: 5, Y: 15, Z: 0},
		units.Millimeters(4), decad.ThroughHole{}).
		Do(t.Context())
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-math.Pi*4*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
}

func TestHoleCounterbore(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10},
		units.Millimeters(4), decad.ThroughHole{}).
		Head(decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(3)}).
		Do(t.Context())
	require.NoError(t, err)
	want := drillPlateVolume - math.Pi*4*10 - math.Pi*(16-4)*3
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want))
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 7, decad.KindCylinder: 2})
}

func TestHoleCountersink(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10},
		units.Millimeters(4), decad.ThroughHole{}).
		Head(decad.Countersink{Diameter: units.Millimeters(8), Angle: units.Degrees(90)}).
		Do(t.Context())
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
	// refused is the 4 mm hole at (10, 10, 10) of the plate's top face, 5 mm
	// deep, with the given parts replaced.
	refused := func(plate *decad.Body, face decad.FaceSelector, at r3.Vec, diameter units.Value,
		depth decad.HoleDepth) *decad.HoleCommand {
		return decad.Hole(plate, face, at, diameter, depth)
	}
	top := r3.Vec{X: 10, Y: 10, Z: 10}
	blind := decad.BlindHole{Depth: units.Millimeters(5)}
	cases := []struct {
		name string
		cmd  func(plate *decad.Body) *decad.HoleCommand
		want error
	}{
		{"no selector", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, nil, top, units.Millimeters(4), blind)
		}, decad.ErrDegenerate},
		{"no depth", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(4), nil)
		}, decad.ErrDegenerate},
		{"zero diameter", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(0), blind)
		}, decad.ErrDegenerate},
		{"wrong diameter kind", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Degrees(4), blind)
		}, decad.ErrUnitKind},
		{"zero depth", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(4), decad.BlindHole{Depth: units.Millimeters(0)})
		}, decad.ErrDegenerate},
		{"centre off the plane", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), r3.Vec{X: 10, Y: 10, Z: 9}, units.Millimeters(4), blind)
		}, decad.ErrDegenerate},
		{"centre outside the body", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), r3.Vec{X: 40, Y: 10, Z: 10}, units.Millimeters(4), blind)
		}, decad.ErrDegenerate},
		{"counterbore as wide as the hole", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(4), blind).
				Head(decad.Counterbore{Diameter: units.Millimeters(4), Depth: units.Millimeters(1)})
		}, decad.ErrDegenerate},
		{"counterbore as deep as the hole", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(4), blind).
				Head(decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(5)})
		}, decad.ErrDegenerate},
		{"countersink deeper than the hole", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(4), blind).
				Head(decad.Countersink{Diameter: units.Millimeters(40), Angle: units.Degrees(90)})
		}, decad.ErrDegenerate},
		{"countersink angle of a straight line", func(p *decad.Body) *decad.HoleCommand {
			return refused(p, capFace(true, p), top, units.Millimeters(4), blind).
				Head(decad.Countersink{Diameter: units.Millimeters(8), Angle: units.Degrees(180)})
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := drillPlate(t)
			_, err := tc.cmd(plate).Do(t.Context())
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused hole leaves the plate live and alone")
		})
	}
}

// A face that is neither a plane nor a cylinder, here a sphere, is refused.
func TestHoleRefusesAFaceThatIsNeitherPlaneNorCylinder(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ball := sphereBody(t, doc)
	_, err := decad.Hole(ball, decad.Faces(), r3.Vec{X: 10, Y: 0, Z: 0}, units.Millimeters(1), decad.ThroughHole{}).
		Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{ball}, doc.Bodies())
}

// A plate turned onto its side takes the hole in its rotated top cap: the hole
// follows the face's own frame, not the world axes.
func TestHoleOnRotatedPlate(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	turn, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 1, 0), units.Degrees(90))
	require.NoError(t, err)
	turned, err := decad.Transform(plate, turn).Do(t.Context())
	require.NoError(t, err)
	// The top cap (z = 10) turns to x = 10; its centre (10, 10, 10) maps to
	// (10, 10, -10).
	out, err := decad.Hole(turned, capFace(true, turned), turn.Apply(r3.Vec{X: 10, Y: 10, Z: 10}),
		units.Millimeters(6), decad.BlindHole{Depth: units.Millimeters(6)}).
		Do(t.Context())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-math.Pi*9*6))
}
