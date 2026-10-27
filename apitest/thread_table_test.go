package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/thread-design.md §9's public-surface tests for
// LookupThread.

func requireThreadSize(t *testing.T, designation string, major, pitch float64) {
	t.Helper()
	got, err := decad.LookupThread(designation)
	require.NoError(t, err, designation)
	m, err := got.Major.In(units.Millimeter)
	require.NoError(t, err)
	p, err := got.Pitch.In(units.Millimeter)
	require.NoError(t, err)
	require.InDelta(t, major, m, 1e-9, "%s major", designation)
	require.InDelta(t, pitch, p, 1e-9, "%s pitch", designation)
}

func TestLookupThreadReadsMetricCoarseSizes(t *testing.T) {
	t.Parallel()
	for designation, size := range map[string][2]float64{
		"M1": {1, 0.25}, "M3": {3, 0.5}, "M4": {4, 0.7}, "M5": {5, 0.8}, "M6": {6, 1}, "M8": {8, 1.25},
		"M10": {10, 1.5}, "m12": {12, 1.75}, "M16": {16, 2}, "M20": {20, 2.5}, "M24": {24, 3}, "M36": {36, 4}, "M64": {64, 6},
	} {
		requireThreadSize(t, designation, size[0], size[1])
	}
	requireThreadSize(t, "M1.6", 1.6, 0.35)
	requireThreadSize(t, "M2.5", 2.5, 0.45)
}

func TestLookupThreadReadsAMetricPitchStated(t *testing.T) {
	t.Parallel()
	requireThreadSize(t, "M10x1.25", 10, 1.25)
	requireThreadSize(t, "m8X1", 8, 1)
	requireThreadSize(t, "M7x0.75", 7, 0.75)
}

func TestLookupThreadReadsUnifiedSizes(t *testing.T) {
	t.Parallel()
	// Nominal diameters in inches, threads per inch: pitch is 25.4/tpi mm.
	for designation, size := range map[string][2]float64{
		"1/4-20": {0.25, 20}, "1/4-28": {0.25, 28}, "5/16-18": {5.0 / 16, 18}, "3/8-24": {0.375, 24},
		"7/16-14": {7.0 / 16, 14}, "1/2-13": {0.5, 13}, "9/16-18": {9.0 / 16, 18}, "5/8-11": {0.625, 11},
		"3/4-16": {0.75, 16}, "7/8-9": {0.875, 9}, "1-8": {1, 8}, "1-12": {1, 12},
		"5/16-24": {5.0 / 16, 24}, "3/8-16": {0.375, 16}, "7/16-20": {7.0 / 16, 20}, "1/2-20": {0.5, 20},
		"9/16-12": {9.0 / 16, 12}, "5/8-18": {0.625, 18}, "3/4-10": {0.75, 10}, "7/8-14": {0.875, 14},
	} {
		requireThreadSize(t, designation, size[0]*25.4, 25.4/size[1])
	}
}

func TestLookupThreadReadsUnifiedNumberedSizes(t *testing.T) {
	t.Parallel()
	// Size #n is 0.060 + 0.013 n inch across: #4 is 0.112, #6 0.138, #8 0.164, #10 0.190 and #12 0.216 inch.
	for designation, size := range map[string][2]float64{
		"#0-80": {0.060, 80}, "#1-64": {0.073, 64}, "#1-72": {0.073, 72}, "#2-56": {0.086, 56}, "#2-64": {0.086, 64},
		"#3-48": {0.099, 48}, "#3-56": {0.099, 56}, "#4-40": {0.112, 40}, "#4-48": {0.112, 48}, "#5-40": {0.125, 40},
		"#5-44": {0.125, 44}, "#6-32": {0.138, 32}, "#6-40": {0.138, 40}, "#8-32": {0.164, 32}, "#8-36": {0.164, 36},
		"#10-24": {0.190, 24}, "#10-32": {0.190, 32}, "#12-24": {0.216, 24}, "#12-28": {0.216, 28},
	} {
		requireThreadSize(t, designation, size[0]*25.4, 25.4/size[1])
	}
	for _, bad := range []string{"#", "#-24", "#10", "#10-", "#10-20", "#7-32", "#13-20", "#0-64", "#x-32", "#10-24x", "# 10-24", "#-1-24"} {
		_, err := decad.LookupThread(bad)
		require.ErrorIs(t, err, decad.ErrDegenerate, "%q", bad)
	}
}

func TestLookupThreadRefusals(t *testing.T) {
	t.Parallel()
	for _, bad := range []string{
		"", "   ", "M", "Mx1", "M10x", "M10xabc", "M10x0", "M10x-1", "M10x12", "M7", "M65", "M-3",
		"1/4-21", "1/4", "1/4-", "-20", "3/32-48", "1/3-20", "2-4", "0/4-20", "10-24", "M10 x1.5", "thread",
	} {
		_, err := decad.LookupThread(bad)
		require.ErrorIs(t, err, decad.ErrDegenerate, "%q", bad)
	}
}

func TestLookupThreadSizeCutsTheThreadItNames(t *testing.T) {
	t.Parallel()
	size, err := decad.LookupThread("M10")
	require.NoError(t, err)
	major, err := size.Major.In(units.Millimeter)
	require.NoError(t, err)
	rod := circleBodyAtZ(t, decad.New(), 0, major/2, 0, 20)
	out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{Face: decad.Faces(decad.Cylindrical()), Pitch: size.Pitch})
	require.NoError(t, err)
	v, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, v, math.Pi*25*20-removedExternal(5, 20, 1.5))
}
