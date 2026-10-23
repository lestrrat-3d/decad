package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// TestUnionManyJoinsTwistedLoftTeethToADisc joins a circular pattern of
// twisted lofted teeth, one per pitch, to a disc in a single Union. The
// teeth touch the disc along their root arcs, so the joined volume is the disc
// plus every tooth.
func TestUnionManyJoinsTwistedLoftTeethToADisc(t *testing.T) {
	const root, height, twist = 3.0, 2.0, 14.5 * math.Pi / 180
	for _, tc := range []struct {
		teeth, fit int
		half       float64
	}{{8, 5, 0.2}, {17, 5, 0.1}} {
		doc := decad.New()
		w := sketch.NewWorld()
		s0, p0 := twistedFitToothSection(t, w, w.XY(), root, tc.fit, tc.half, 0)
		s1, p1 := twistedFitToothSection(t, w, splitTopPlane(t, w, height), root, tc.fit, tc.half, twist)
		tooth, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
		require.NoError(t, err, "teeth %d", tc.teeth)
		toothVol, err := tooth.Volume()
		require.NoError(t, err)
		disc := circleBodyAtZ(t, doc, 0, root, 0, height)
		discVol, err := disc.Volume()
		require.NoError(t, err)

		copies, err := decad.PatternCopies(t.Context(), tooth, decad.CircularPattern{Axis: r3.Vec{Z: 1}, Count: tc.teeth})
		require.NoError(t, err)
		joined, err := decad.Union(t.Context(), disc, append([]*decad.Body{tooth}, copies...)...)
		require.NoError(t, err, "teeth %d", tc.teeth)
		require.True(t, joined.IsSolid())
		got, err := joined.Volume()
		require.NoError(t, err)

		want := discVol.Value.Base() + float64(tc.teeth)*toothVol.Value.Base()
		slack := got.Bound.Base() + discVol.Bound.Base() + float64(tc.teeth)*toothVol.Bound.Base()
		require.LessOrEqual(t, math.Abs(got.Value.Base()-want), slack,
			"teeth %d: joined %v want %v", tc.teeth, got.Value, want)
	}
}
