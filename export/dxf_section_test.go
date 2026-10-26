package export_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/dxf-export-design.md §10's tests for DXFSection.

// midPlane is the plane z = 5 with the drawing's X and Y along the world's, from the origin.
func midPlane(t *testing.T, origin r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

func sectionFile(t *testing.T, body *decad.Body, plane r3.Frame, hatch export.DXFHatch) ([]dxfLine, string) {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.DXFSection(t.Context(), &out, body, plane, units.Millimeters(0.1), hatch))
	lines, _ := readDXFLines(t, out.String())
	return lines, out.String()
}

func byLayer(lines []dxfLine, layer string) []dxfLine {
	var out []dxfLine
	for _, l := range lines {
		if l.layer == layer {
			out = append(out, l)
		}
	}
	return out
}

func length(l dxfLine) float64 { return math.Hypot(l.b[0]-l.a[0], l.b[1]-l.a[1]) }

func TestDXFSectionHatchesABlock(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	lines, _ := sectionFile(t, block, midPlane(t, r3.NewVec(0, 0, 5)), export.DXFHatch{
		Spacing: units.Millimeters(2), Angle: units.Degrees(0)})
	outline := byLayer(lines, export.DXFSectionLayer)
	perimeter := 0.0
	for _, l := range outline {
		perimeter += length(l)
	}
	require.InDelta(t, 100, perimeter, 1e-9, "the cut's outline is the 20 x 30 rectangle's perimeter, however the mesh splits its sides")
	hatch := byLayer(lines, export.DXFHatchLayer)
	// Horizontal lines at y = 0, 2, ... 28, each the block's 20 mm wide. The one at y = 0 lies along
	// the cut's lower edge, which the half-open rule of the hatch counts as inside; y = 30 is not.
	require.Len(t, hatch, 15)
	for i, l := range hatch {
		require.InDelta(t, 2*float64(i), l.a[1], 1e-9)
		require.InDelta(t, l.a[1], l.b[1], 1e-9)
		require.InDelta(t, 20, length(l), 1e-9)
		require.InDelta(t, 0, math.Min(l.a[0], l.b[0]), 1e-9)
	}
	// The drawing's axes are the plane's: a plane origin moves the drawing, and the hatch keeps
	// to whole multiples of the spacing from it.
	moved, _ := sectionFile(t, block, midPlane(t, r3.NewVec(3, 1, 5)), export.DXFHatch{
		Spacing: units.Millimeters(2), Angle: units.Degrees(0)})
	for _, l := range byLayer(moved, export.DXFHatchLayer) {
		require.InDelta(t, 0, math.Mod(l.a[1]+1e-9, 2), 1e-6, "y = %v is a multiple of the spacing", l.a[1])
		require.InDelta(t, -3, math.Min(l.a[0], l.b[0]), 1e-9, "the block's left edge is 3 left of the plane's origin")
	}
}

// platedHole is a 40 x 40 plate 10 thick with a 12 mm hole through its middle.
func platedHole(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(20, 20), 6)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(t.Context(), decad.New(), s, profile, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestDXFSectionLeavesAHoleUnhatched(t *testing.T) {
	t.Parallel()
	plate := platedHole(t)
	const spacing = 0.5
	lines, _ := sectionFile(t, plate, midPlane(t, r3.NewVec(0, 0, 5)), export.DXFHatch{Spacing: units.Millimeters(spacing)})
	hatch := byLayer(lines, export.DXFHatchLayer)
	total := 0.0
	for _, l := range hatch {
		total += length(l)
		mid := [2]float64{(l.a[0] + l.b[0]) / 2, (l.a[1] + l.b[1]) / 2}
		require.Greater(t, math.Hypot(mid[0]-20, mid[1]-20), 6-0.01, "a hatch line crosses the hole at %v", mid)
		// 45 degrees by default.
		require.InDelta(t, 1, math.Abs((l.b[0]-l.a[0])/(l.b[1]-l.a[1])), 1e-6)
	}
	// Hatch lines are a spacing apart, so the length times the spacing is the area: the plate's less the
	// hole's, to the chord polygon's own deficit and the lines' edge effects.
	want := 40*40 - math.Pi*36
	require.InDelta(t, want, total*spacing, 0.01*want)
	// The hole's outline is drawn too: the plate's 160 mm and the hole's chords, a little under its circle.
	outline := 0.0
	for _, l := range byLayer(lines, export.DXFSectionLayer) {
		outline += length(l)
	}
	require.InDelta(t, 160+2*math.Pi*6, outline, 0.02*outline)
}

func TestDXFSectionHatchAngleAndMissingPlane(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	plane := midPlane(t, r3.NewVec(0, 0, 5))
	lines, _ := sectionFile(t, block, plane, export.DXFHatch{Spacing: units.Millimeters(3), Angle: units.Degrees(-30)})
	for _, l := range byLayer(lines, export.DXFHatchLayer) {
		dx, dy := l.b[0]-l.a[0], l.b[1]-l.a[1]
		require.InDelta(t, 0, dx*math.Sin(-math.Pi/6)-dy*math.Cos(-math.Pi/6), 1e-9, "each line runs at -30 degrees")
	}
	// A plane above the block cuts nothing: the file is complete and holds no entities.
	missed, file := sectionFile(t, block, midPlane(t, r3.NewVec(0, 0, 50)), export.DXFHatch{Spacing: units.Millimeters(3)})
	require.Empty(t, missed)
	require.Contains(t, file, "ENTITIES")
	require.Contains(t, file, "EOF")
}

func TestDXFSectionIsDeterministic(t *testing.T) {
	t.Parallel()
	plate := platedHole(t)
	_, a := sectionFile(t, plate, midPlane(t, r3.NewVec(0, 0, 5)), export.DXFHatch{Spacing: units.Millimeters(1)})
	_, b := sectionFile(t, plate, midPlane(t, r3.NewVec(0, 0, 5)), export.DXFHatch{Spacing: units.Millimeters(1)})
	require.Equal(t, a, b)
}

func TestDXFSectionRefusals(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	plane := midPlane(t, r3.NewVec(0, 0, 5))
	tol := units.Millimeters(0.1)
	good := export.DXFHatch{Spacing: units.Millimeters(2)}
	for name, tc := range map[string]struct {
		hatch export.DXFHatch
		want  error
	}{
		"no spacing":       {export.DXFHatch{}, decad.ErrUnitKind},
		"zero spacing":     {export.DXFHatch{Spacing: units.Millimeters(0)}, decad.ErrDegenerate},
		"negative spacing": {export.DXFHatch{Spacing: units.Millimeters(-1)}, decad.ErrNegativeMagnitude},
		"an angle":         {export.DXFHatch{Spacing: units.Degrees(1)}, decad.ErrUnitKind},
		"a length angle":   {export.DXFHatch{Spacing: units.Millimeters(1), Angle: units.Millimeters(1)}, decad.ErrUnitKind},
		"too fine":         {export.DXFHatch{Spacing: units.Millimeters(1e-4)}, decad.ErrDegenerate},
	} {
		var out bytes.Buffer
		require.ErrorIs(t, export.DXFSection(t.Context(), &out, block, plane, tol, tc.hatch), tc.want, name)
		require.Zero(t, out.Len(), "%s writes nothing", name)
	}
	var out bytes.Buffer
	require.ErrorIs(t, export.DXFSection(nil, &out, block, plane, tol, good), decad.ErrDegenerate) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, export.DXFSection(t.Context(), nil, block, plane, tol, good), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFSection(t.Context(), &out, nil, plane, tol, good), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFSection(t.Context(), &out, block, r3.Frame{}, tol, good), decad.ErrDegenerate)
	require.Zero(t, out.Len())
}
