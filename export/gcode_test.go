package export_test

import (
	"bytes"
	"math"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/gcode-design.md's tests for GCodePerimeters.

func gcodeSpec() export.GCodeSpec {
	return export.GCodeSpec{
		LayerHeight:      units.Millimeters(0.5),
		LineWidth:        units.Millimeters(0.4),
		FilamentDiameter: units.Millimeters(1.75),
		FeedRate:         units.MillimetersPerSecond(30),
	}
}

func TestGCodePerimetersTraceABlockLayerByLayer(t *testing.T) {
	t.Parallel()
	// A 20 x 10 x 6 mm block is twelve 0.5 mm layers of a 60 mm perimeter.
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	var out bytes.Buffer
	require.NoError(t, export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), gcodeSpec()))
	lines := strings.Split(strings.TrimSpace(out.String()), "\n")
	require.Equal(t, []string{"G21", "G90", "M82", "G92 E0"}, lines[1:5], "millimetres, absolute positioning and extrusion")
	layers, extrusions := 0, 0
	z := 0.0
	lastE := 0.0
	field := func(line, prefix string) float64 {
		for f := range strings.FieldsSeq(line) {
			if num, ok := strings.CutPrefix(f, prefix); ok {
				v, err := strconv.ParseFloat(num, 64)
				require.NoError(t, err)
				return v
			}
		}
		t.Fatalf("no %s in %q", prefix, line)
		return 0
	}
	for _, line := range lines {
		switch {
		case strings.HasPrefix(line, "; layer"):
			layers++
			z = field(line, "z=")
			require.InDelta(t, 0.5*float64(layers), z, 1e-9, "layer tops are whole layers up from the lowest point")
		case strings.HasPrefix(line, "G0 "):
			require.InDelta(t, z, field(line, "Z"), 1e-9)
		case strings.HasPrefix(line, "G1 "):
			extrusions++
			x, y := field(line, "X"), field(line, "Y")
			require.GreaterOrEqual(t, x, -1e-9)
			require.LessOrEqual(t, x, 20+1e-9)
			require.GreaterOrEqual(t, y, -1e-9)
			require.LessOrEqual(t, y, 10+1e-9)
			e := field(line, "E")
			require.GreaterOrEqual(t, e, lastE, "absolute extrusion never backs up")
			lastE = e
			require.InDelta(t, 1800, field(line, "F"), 1e-6, "30 mm/s is 1800 mm/min")
		}
	}
	require.Equal(t, 12, layers)
	require.GreaterOrEqual(t, extrusions, 12*4, "at least four sides to a layer")
	require.Zero(t, extrusions%12, "every layer traces the same outline")
	// 720 mm of line, each millimetre 0.5 x 0.4 mm² of plastic from 1.75 mm filament's π·0.875² mm².
	require.InDelta(t, 720*0.5*0.4/(math.Pi*0.875*0.875), lastE, 1e-2)
}

func TestGCodePerimetersRefusals(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	write := func(spec export.GCodeSpec) (string, error) {
		var out bytes.Buffer
		err := export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), spec)
		return out.String(), err
	}
	with := func(edit func(*export.GCodeSpec)) export.GCodeSpec {
		s := gcodeSpec()
		edit(&s)
		return s
	}
	for name, tc := range map[string]struct {
		spec export.GCodeSpec
		want error
	}{
		"zero layer":     {with(func(s *export.GCodeSpec) { s.LayerHeight = units.Millimeters(0) }), decad.ErrDegenerate},
		"negative width": {with(func(s *export.GCodeSpec) { s.LineWidth = units.Millimeters(-1) }), decad.ErrNegativeMagnitude},
		"filament kind":  {with(func(s *export.GCodeSpec) { s.FilamentDiameter = units.Degrees(2) }), decad.ErrUnitKind},
		"feed kind":      {with(func(s *export.GCodeSpec) { s.FeedRate = units.Millimeters(2) }), decad.ErrUnitKind},
		"zero feed":      {with(func(s *export.GCodeSpec) { s.FeedRate = units.MillimetersPerSecond(0) }), decad.ErrDegenerate},
		"taller layer":   {with(func(s *export.GCodeSpec) { s.LayerHeight = units.Millimeters(7) }), decad.ErrDegenerate},
		"too many":       {with(func(s *export.GCodeSpec) { s.LayerHeight = units.Millimeters(0.00001) }), decad.ErrDegenerate},
	} {
		got, err := write(tc.spec)
		require.ErrorIs(t, err, tc.want, name)
		require.Empty(t, got, "%s writes nothing", name)
	}
	require.ErrorIs(t, export.GCodePerimeters(t.Context(), nil, block, units.Millimeters(0.1), gcodeSpec()), decad.ErrDegenerate)
	require.ErrorIs(t, export.GCodePerimeters(t.Context(), &bytes.Buffer{}, nil, units.Millimeters(0.1), gcodeSpec()), decad.ErrDegenerate)
	require.ErrorIs(t, export.GCodePerimeters(nil, &bytes.Buffer{}, block, units.Millimeters(0.1), gcodeSpec()), decad.ErrDegenerate) //nolint:staticcheck // a nil context is the case under test
}

// gcodeExtrusion is the plastic length of one millimetre of line for gcodeSpec: a 0.5 x 0.4 mm² line from 1.75 mm
// filament's π·0.875² mm² cross-section.
const gcodeExtrusion = 0.5 * 0.4 / (math.Pi * 0.875 * 0.875)

// gcodeLayers splits a file into the layers and, in each, the groups of extruding moves that follow one travel move
// (a loop of the perimeter, or one line of infill), each move as the X and Y it ends on.
func gcodeLayers(t *testing.T, file string) [][][][2]float64 {
	t.Helper()
	var layers [][][][2]float64
	for line := range strings.SplitSeq(file, "\n") {
		switch {
		case strings.HasPrefix(line, "; layer"):
			layers = append(layers, nil)
		case strings.HasPrefix(line, "G0 "):
			layers[len(layers)-1] = append(layers[len(layers)-1], nil)
		case strings.HasPrefix(line, "G1 "):
			var xy [2]float64
			for f := range strings.FieldsSeq(line) {
				if num, ok := strings.CutPrefix(f, "X"); ok {
					v, err := strconv.ParseFloat(num, 64)
					require.NoError(t, err)
					xy[0] = v
				}
				if num, ok := strings.CutPrefix(f, "Y"); ok {
					v, err := strconv.ParseFloat(num, 64)
					require.NoError(t, err)
					xy[1] = v
				}
			}
			groups := layers[len(layers)-1]
			groups[len(groups)-1] = append(groups[len(groups)-1], xy)
		}
	}
	return layers
}

func lastExtrusion(t *testing.T, file string) float64 {
	t.Helper()
	last := 0.0
	for line := range strings.SplitSeq(file, "\n") {
		if !strings.HasPrefix(line, "G1 ") {
			continue
		}
		for f := range strings.FieldsSeq(line) {
			if num, ok := strings.CutPrefix(f, "E"); ok {
				v, err := strconv.ParseFloat(num, 64)
				require.NoError(t, err)
				last = v
			}
		}
	}
	return last
}

func TestGCodeInfillFillsABlockWithAlternatingLines(t *testing.T) {
	t.Parallel()
	// A 20 x 10 x 6 mm block with a line every 5 mm: an even layer holds two lines along X at y = 2.5 and 7.5, each
	// 19.2 mm between the 0.4 mm insets, and an odd layer four along Y at x = 2.5, 7.5, 12.5 and 17.5, each 9.2 mm.
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	spec := gcodeSpec()
	spec.InfillSpacing = units.Millimeters(5)
	var out bytes.Buffer
	require.NoError(t, export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), spec))
	layers := gcodeLayers(t, out.String())
	require.Len(t, layers, 12)
	for i, groups := range layers {
		infill := groups[1:] // the one loop of the perimeter comes first
		if i%2 == 0 {
			require.Len(t, infill, 2, "layer %d", i)
			for _, g := range infill {
				require.Len(t, g, 1)
				require.InDelta(t, 0, math.Mod(g[0][1]-2.5, 5), 1e-9, "a line along X at y = 2.5 or 7.5")
			}
			continue
		}
		require.Len(t, infill, 4, "layer %d", i)
		for _, g := range infill {
			require.Len(t, g, 1)
			require.InDelta(t, 0, math.Mod(g[0][0]-2.5, 5), 1e-9, "a line along Y at x = 2.5, 7.5, ...")
		}
	}
	// 720 mm of perimeter and 6 x (2 x 19.2 + 4 x 9.2) = 451.2 mm of infill.
	require.InDelta(t, (720+451.2)*gcodeExtrusion, lastExtrusion(t, out.String()), 1e-2)
}

func TestGCodeInfillSkipsTheHoleOfAPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := decadtest.NewBlock(t, doc, 0, 0, 20, 20, units.Millimeters(1))
	hole := decadtest.NewBlock(t, doc, 7, 7, 13, 13, units.Millimeters(3))
	holed, err := decad.Cut(t.Context(), plate, hole)
	require.NoError(t, err)
	spec := gcodeSpec()
	spec.InfillSpacing = units.Millimeters(5)
	var out bytes.Buffer
	require.NoError(t, export.GCodePerimeters(t.Context(), &out, holed, units.Millimeters(0.1), spec))
	require.Len(t, gcodeLayers(t, out.String()), 2)
	// Two layers of 80 + 24 = 104 mm of perimeter. A layer's four lines at 2.5, 7.5, 12.5 and 17.5 take two full
	// lengths of 19.2 mm, and the two that cross the 6 mm hole each break into two pieces of 6.2 mm, 63.2 mm in all.
	require.InDelta(t, (2*104+2*63.2)*gcodeExtrusion, lastExtrusion(t, out.String()), 1e-2)
}

func TestGCodeInfillSpacingRefusals(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	for name, tc := range map[string]struct {
		spacing units.Value
		want    error
	}{
		"negative": {units.Millimeters(-1), decad.ErrNegativeMagnitude},
		"kind":     {units.Degrees(5), decad.ErrUnitKind},
	} {
		spec := gcodeSpec()
		spec.InfillSpacing = tc.spacing
		var out bytes.Buffer
		require.ErrorIs(t, export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), spec), tc.want, name)
		require.Empty(t, out.String(), "%s writes nothing", name)
	}
}

func TestGCodeSolidLayersFillTheBottomAndTheTop(t *testing.T) {
	t.Parallel()
	// A 20 x 10 x 6 mm block in twelve layers with two solid layers at each end and a sparse line every 5 mm
	// between. A solid even layer has 25 lines along X at y = 0.2, 0.6, ... of 19.2 mm, and a solid odd one 50 along
	// Y of 9.2 mm; the eight sparse layers are 4 x 38.4 + 4 x 36.8 = 300.8 mm.
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	spec := gcodeSpec()
	spec.InfillSpacing = units.Millimeters(5)
	spec.SolidLayers = 2
	var out bytes.Buffer
	require.NoError(t, export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), spec))
	layers := gcodeLayers(t, out.String())
	require.Len(t, layers, 12)
	wantLines := []int{25, 50, 2, 4, 2, 4, 2, 4, 2, 4, 25, 50}
	for i, groups := range layers {
		require.Len(t, groups[1:], wantLines[i], "layer %d", i)
	}
	solid := 2*25*19.2 + 2*50*9.2
	require.InDelta(t, (720+solid+300.8)*gcodeExtrusion, lastExtrusion(t, out.String()), 1e-2)

	// Solid layers alone, with no sparse infill between.
	spec.InfillSpacing = units.Value{}
	out.Reset()
	require.NoError(t, export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), spec))
	layers = gcodeLayers(t, out.String())
	for i, groups := range layers {
		want := 0
		if i < 2 || i >= 10 {
			want = wantLines[i]
		}
		require.Len(t, groups[1:], want, "layer %d", i)
	}

	spec.SolidLayers = -1
	out.Reset()
	require.ErrorIs(t, export.GCodePerimeters(t.Context(), &out, block, units.Millimeters(0.1), spec), decad.ErrDegenerate)
	require.Empty(t, out.String())
}
