package momentinput_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func walkMatchLine(end float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: end}, TEnd: 1},
	}}}
}

func TestResolvedWalksRefuseChangedRecord(t *testing.T) {
	t.Parallel()
	base := walkMatchLine(1)
	walks, err := momentinput.ResolveProfileWalks(base, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.True(t, walks.Reusable(base))

	oneULP := walkMatchLine(math.Nextafter(1, 2))
	moreSegments := walkMatchLine(1)
	moreSegments.Outer.Segments = append(moreSegments.Outer.Segments,
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{U: 2}, TEnd: 1})
	withHole := walkMatchLine(1)
	withHole.Holes = []sectionrecord.LoopRecord{walkMatchLine(2).Outer}
	for _, tc := range []struct {
		name    string
		profile momentinput.Profile
	}{{"one ulp", oneULP}, {"segment count", moreSegments}, {"hole", withHole}} {
		t.Run(tc.name, func(t *testing.T) {
			require.False(t, walks.Matches(tc.profile))
			_, err := momentinput.CoordinateEnvelope(tc.profile, freeform.NewFreeformWork(), walks)
			require.Error(t, err)
		})
	}
}
