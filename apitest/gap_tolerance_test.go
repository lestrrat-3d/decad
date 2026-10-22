package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// gapPlate extrudes the 100×60 plate in a document made with opts.
func gapPlate(t *testing.T, opts ...decad.DocumentOption) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	doc := decad.New(opts...)
	body, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestDocumentGapToleranceOptions(t *testing.T) {
	t.Parallel()
	slide, err := r3.Translation(r3.Vec{X: 1, Y: 2, Z: 3})
	require.NoError(t, err)

	for name, opts := range map[string][]decad.DocumentOption{
		"no option":              nil,
		"exact only":             {decad.ExactOnly()},
		"a length":               {decad.WithGapTolerance(units.Millimeters(0.05))},
		"zero":                   {decad.WithGapTolerance(units.Millimeters(0))},
		"in another length unit": {decad.WithGapTolerance(units.Meters(1e-5))},
		"the last option wins":   {decad.WithGapTolerance(units.Millimeters(1)), decad.ExactOnly()},
	} {
		t.Run("accepts "+name, func(t *testing.T) {
			t.Parallel()
			body := gapPlate(t, opts...)
			moved, err := decad.Transform(t.Context(), body, slide)
			require.NoError(t, err)
			centroid, err := moved.Centroid()
			require.NoError(t, err)
			require.InDelta(t, 3, centroid.Value.Z-shellBoxHeight/2, 1e-9)
		})
	}

	for name, tc := range map[string]struct {
		opts []decad.DocumentOption
		want error
	}{
		"a negative length":  {[]decad.DocumentOption{decad.WithGapTolerance(units.Millimeters(-0.01))}, decad.ErrNegativeMagnitude},
		"an angle":           {[]decad.DocumentOption{decad.WithGapTolerance(units.Degrees(1))}, decad.ErrUnitKind},
		"a dimensionless":    {[]decad.DocumentOption{decad.WithGapTolerance(units.Scalar(1))}, decad.ErrUnitKind},
		"not a number":       {[]decad.DocumentOption{decad.WithGapTolerance(units.Millimeters(math.NaN()))}, decad.ErrNotFinite},
		"an infinite length": {[]decad.DocumentOption{decad.WithGapTolerance(units.Millimeters(math.Inf(1)))}, decad.ErrNotFinite},
		"an invalid option before a valid one": {
			[]decad.DocumentOption{decad.WithGapTolerance(units.Millimeters(-1)), decad.ExactOnly()}, decad.ErrNegativeMagnitude,
		},
		"a nil option": {[]decad.DocumentOption{nil}, decad.ErrDegenerate},
	} {
		t.Run("refuses "+name+" when an operation reads it", func(t *testing.T) {
			t.Parallel()
			body := gapPlate(t, tc.opts...)
			_, err := decad.Transform(t.Context(), body, slide)
			require.ErrorIs(t, err, tc.want)
			require.Len(t, body.Document().Bodies(), 1, "a refused operation leaves the body live")
		})
	}
}
