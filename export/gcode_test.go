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
