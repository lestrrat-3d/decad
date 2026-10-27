package stackedunion

// An internal test: sameRegion is unexported, and no public operand pair
// reaches it with two holed records of one operand today (A1's Union admits
// hole-free regions, and A7's two prism operands each hold one slab), so only
// this reading shows the hole comparison deciding.

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSameRegionComparesHoles(t *testing.T) {
	t.Parallel()
	square := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: -10, V: -10}, End: sectionrecord.Point2{U: 10, V: -10}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 10, V: -10}, End: sectionrecord.Point2{U: 10, V: 10}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 10, V: 10}, End: sectionrecord.Point2{U: -10, V: 10}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: -10, V: 10}, End: sectionrecord.Point2{U: -10, V: -10}, TEnd: 1},
	}}
	bore := func(r float64) sectionrecord.LoopRecord {
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
			Radius: units.Millimeters(r), TStart: 1, TEnd: 0,
		}}}
	}
	// A counterbore's two slabs: one outer, a bore of radius 2 below and a
	// head of radius 4 above.
	shoulder := profileRecord{Outer: square, Holes: []loopRecord{bore(2)}}
	head := profileRecord{Outer: square, Holes: []loopRecord{bore(4)}}
	solid := profileRecord{Outer: square}
	for _, tc := range []struct {
		name string
		a, b profileRecord
		want bool
	}{
		{"same-holes", shoulder, profileRecord{Outer: square, Holes: []loopRecord{bore(2)}}, true},
		{"hole-free", solid, solid, true},
		{"different-hole", shoulder, head, false},
		{"hole-against-none", shoulder, solid, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := sameRegion(proofbound.NewWorkBudget(t.Context()), tc.a, tc.b)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
