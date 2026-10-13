package loftfillet_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// squareLoop builds the counter-clockwise LineSeg square [-half, half]².
func squareLoop(half float64) sectionrecord.LoopRecord {
	corners := []sectionrecord.Point2{
		{U: -half, V: -half}, {U: half, V: -half}, {U: half, V: half}, {U: -half, V: half},
	}
	segs := make([]sectionrecord.CurveSegment, 4)
	for i := range corners {
		segs[i] = sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestCircleHoleInsideRequiresStrictContainment(t *testing.T) {
	t.Parallel()
	outer := squareLoop(2)
	for _, tc := range []struct {
		name   string
		center sectionrecord.Point2
		want   bool
	}{
		{name: "inside", center: sectionrecord.Point2{U: 0, V: 0}, want: true},
		{name: "outside", center: sectionrecord.Point2{U: 4, V: 0}},
		{name: "crosses boundary", center: sectionrecord.Point2{U: 1.75, V: 0}},
		{name: "touches boundary", center: sectionrecord.Point2{U: 1.5, V: 0}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			hole := sectionrecord.CircleSeg{Center: tc.center, Radius: units.Millimeters(0.5), TStart: 1, TEnd: 0}
			got, err := loftfillet.CircleHoleInside(proofbound.NewWorkBudget(t.Context()), outer, hole, 0,
				freeform.NewFreeformWork())
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
