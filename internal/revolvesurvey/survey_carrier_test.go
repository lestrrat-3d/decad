package revolvesurvey

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestNextCarrierID(t *testing.T) {
	last := 2
	for _, tc := range []struct{ shared, want int }{{1, 1}, {0, 3}, {2, 2}, {0, 4}} {
		require.Equal(t, tc.want, nextCarrierID(tc.shared, &last))
	}
	require.Equal(t, 4, last)
}

func TestWallReadingCarrierTable(t *testing.T) {
	axis := revolveaxis.Frame{DU: 1}
	arc, err := boundarywalk.WalkOf(sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: 10}, End: sectionrecord.Point2{U: -10}, TEnd: 1}, freeform.NewFreeformWork())
	require.NoError(t, err)
	line, err := boundarywalk.WalkOf(sectionrecord.LineSeg{Start: sectionrecord.Point2{U: -10}, End: sectionrecord.Point2{U: 10}, TEnd: 1}, freeform.NewFreeformWork())
	require.NoError(t, err)
	loops := [][]survey2d.SideWalk{{{SegmentWalk: axis.Walk(arc)}, {SegmentWalk: axis.Walk(line)}}}
	nilResult, err := WallReading(nil, loops, nil, axis, true, 0, 0, 0, 0)
	require.NoError(t, err)
	zeroResult, err := WallReading(nil, loops, [][]int{{0, 0}}, axis, true, 0, 0, 0, 0)
	require.NoError(t, err)
	require.Equal(t, nilResult.Ok, zeroResult.Ok)
	require.Equal(t, nilResult.Bound, zeroResult.Bound)
	require.Equal(t, nilResult.Reading == nil, zeroResult.Reading == nil)
	require.NotNil(t, nilResult.Reading)
	require.NotNil(t, zeroResult.Reading)
	require.Equal(t, *nilResult.Reading, *zeroResult.Reading)
	_, err = WallReading(nil, loops, [][]int{{2, 0}}, axis, true, 0, 0, 0, 0)
	require.NoError(t, err)
	for _, table := range [][][]int{{}, {{}}, {{-1, 0}}} {
		_, err := WallReading(nil, loops, table, axis, true, 0, 0, 0, 0)
		require.ErrorContains(t, err, "revolve wall carrier table:")
	}
}
