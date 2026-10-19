package prismextent_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestProfileBoundaryExtremesEncloseSplineInteriorMaximum(t *testing.T) {
	t.Parallel()
	segment := sectionrecord.FitSplineSeg{
		Fit:  []sectionrecord.Point2{{}, {U: 1, V: 3}, {U: 3, V: 1}, {U: 4}},
		TEnd: 1,
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{segment},
	}}
	lo, hi, bound, err := prismextent.ProfileBoundaryExtremes(
		t.Context(), profile, 0, 1, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.Less(t, lo, hi)
	require.Greater(t, hi+bound, 1.0)
}
