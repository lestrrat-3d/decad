package prismcells_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExtendSetBoundChangesOnlyTheSelectedParameter(t *testing.T) {
	t.Parallel()
	for _, row := range []struct {
		name    string
		seg     sectionrecord.CurveSegment
		atStart bool
		bound   float64
	}{
		{name: "line at end", seg: sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 100}, TEnd: 0.4}, bound: 0.7},
		{name: "circle at end", seg: sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 3, V: -7}, Radius: units.Millimeters(50), CCW: true, TStart: 0.25, TEnd: 0.75}, bound: 0.875},
		{name: "arc at start", seg: sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: 100}, End: sectionrecord.Point2{U: -100}, TStart: 0.5}, atStart: true, bound: 0.75},
	} {
		t.Run(row.name, func(t *testing.T) {
			start, end, err := prismcells.SegmentParamRange(row.seg)
			require.NoError(t, err)
			widened := prismcells.ExtendSetBound(row.seg, row.atStart, row.bound)
			gotStart, gotEnd, err := prismcells.SegmentParamRange(widened)
			require.NoError(t, err)
			if row.atStart {
				require.Equal(t, row.bound, gotStart)
				require.Equal(t, end, gotEnd)
				return
			}
			require.Equal(t, start, gotStart)
			require.Equal(t, row.bound, gotEnd)
		})
	}
}
