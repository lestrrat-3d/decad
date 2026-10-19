package capband

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestLoopCoordinateUpperReadsAWholeCircle pins loopLocalCoordinateUpper's
// fallback on a whole circle, which SegmentCoordinateUpper does not read: a
// circle of radius 10 about (3, −4) reads the walk's L1 bound 7 + 10√2
// (momentinput.WalkCoordinateUpper), which covers the circle's true
// max(|u|, |v|) of 14.
//
// Shown to fail first: read through the walk's own CoordUpper,
// |cu| + |cv| + 2R, the loop read 27.
func TestLoopCoordinateUpperReadsAWholeCircle(t *testing.T) {
	t.Parallel()
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 3, V: -4}, Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1},
	}}
	got, err := loopLocalCoordinateUpper(loop, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.LessOrEqual(t, 14.0, got)
	require.InDelta(t, 7+10*math.Sqrt2, got, 1e-12)
}
