package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/section-outline-design.md's public-surface tests for
// Body.SectionOutline.

func outlinePlane(t *testing.T, origin, normal r3.Vec) r3.Frame {
	t.Helper()
	// Any frame with the wanted origin and normal: its U axis is a vector
	// perpendicular to the normal.
	n, ok := normal.Normalize()
	require.True(t, ok)
	seed := r3.NewVec(1, 0, 0)
	if math.Abs(n.X) > 0.9 {
		seed = r3.NewVec(0, 1, 0)
	}
	u, ok := n.Cross(seed).Normalize()
	require.True(t, ok)
	// NewFrame takes the U and V axes, and the normal is U × V: V = n × U.
	frame, err := r3.NewFrame(origin, u, n.Cross(u))
	require.NoError(t, err)
	require.True(t, frame.N().Equal(n, 1e-12), "the frame's normal is %v, want %v", frame.N(), n)
	return frame
}

func TestSectionOutlineOfABlockIsItsSquare(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := boxBody(t, doc, 0, 0, 20, 20, 10)
	plane := outlinePlane(t, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 1))
	out, err := block.SectionOutline(t.Context(), plane, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, out.Loops, 1)
	require.InDelta(t, 400, out.PolygonArea().Base(), 1e-9)
	require.Less(t, out.OffSurface.Base(), 1e-9, "a block's mesh departs from it by rounding alone")
	for _, p := range out.Loops[0] {
		require.InDelta(t, 5, p.Z, 1e-12)
		onSide := math.Abs(p.X) < 1e-12 || math.Abs(p.X-20) < 1e-12 || math.Abs(p.Y) < 1e-12 || math.Abs(p.Y-20) < 1e-12
		require.True(t, onSide, "a point off the square's boundary: %v", p)
	}
}

func TestSectionOutlineFlippedNormalReversesTheLoop(t *testing.T) {
	t.Parallel()
	block := boxBody(t, decad.New(), 0, 0, 20, 20, 10)
	out, err := block.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, -1)), units.Millimeters(0.1))
	require.NoError(t, err)
	require.InDelta(t, 400, out.PolygonArea().Base(), 1e-9, "counter-clockwise seen from the flipped normal")
}

func TestSectionOutlineOfADrilledPlateHasAHole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	drilled, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.NewVec(0, 0, 10),
		Diameter: units.Millimeters(8),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	out, err := drilled.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 1)), units.Millimeters(0.05))
	require.NoError(t, err)
	require.Len(t, out.Loops, 2, "the plate's outline and the hole's")
	// The hole's polygon has its vertices on the radius-4 circle and its chords
	// inside it, so its area lies just under the disc's and the plate's polygon
	// area, 1600 less the hole's, lies just over 1600 less the disc's.
	area := out.PolygonArea().Base()
	require.Greater(t, area, 1600-math.Pi*16-1e-6)
	require.Less(t, area, 1600-math.Pi*16+1)
	for _, loop := range out.Loops {
		for _, p := range loop {
			if math.Abs(p.X) < 5 && math.Abs(p.Y) < 5 { // a hole point
				require.LessOrEqual(t, math.Abs(math.Hypot(p.X, p.Y)-4), out.OffSurface.Base(),
					"a hole point lies within OffSurface of the true cylinder")
			}
		}
	}
	// Section refuses this body; the outline reads its mesh.
	_, _, err = drilled.Section(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 1)))
	require.Error(t, err)
}

func TestSectionOutlineObliquePlaneCutsAHexagon(t *testing.T) {
	t.Parallel()
	cube := boxBody(t, decad.New(), 0, 0, 10, 10, 10)
	out, err := cube.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(5, 5, 5), r3.NewVec(1, 1, 1)), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, out.Loops, 1)
	// A plane through a cube's centre, square to its diagonal, cuts a regular
	// hexagon of side a/√2: area 3√3/4 · a^2.
	require.InDelta(t, 3*math.Sqrt(3)/4*100, out.PolygonArea().Base(), 1e-6)
}

func TestSectionOutlineThroughAFaceReadsJustBehindIt(t *testing.T) {
	t.Parallel()
	block := boxBody(t, decad.New(), 0, 0, 20, 20, 10)
	out, err := block.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 10), r3.NewVec(0, 0, 1)), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, out.Loops, 1)
	require.InDelta(t, 400, out.PolygonArea().Base(), 1e-9, "a plane in the top face reads the cut just under it")
	above, err := block.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 10), r3.NewVec(0, 0, -1)), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Empty(t, above.Loops, "seen from above the top face, nothing lies on the normal's side to cut")
}

func TestSectionOutlineMissesAndRefusals(t *testing.T) {
	t.Parallel()
	block := boxBody(t, decad.New(), 0, 0, 20, 20, 10)
	missed, err := block.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 50), r3.NewVec(0, 0, 1)), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Empty(t, missed.Loops)
	require.Zero(t, missed.PolygonArea().Base())

	good := outlinePlane(t, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 1))
	_, err = block.SectionOutline(t.Context(), r3.Frame{}, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate, "an invalid plane")
	_, err = block.SectionOutline(t.Context(), good, units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	var none *decad.Body
	_, err = none.SectionOutline(t.Context(), good, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
