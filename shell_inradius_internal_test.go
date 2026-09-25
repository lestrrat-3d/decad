package decad

import (
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestShellRectCircleWitnessKeepsToleranceAndFallsBack(t *testing.T) {
	t.Parallel()
	outer := LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{0, 0}, End: Point2{100, 0}, TEnd: 1},
		LineSeg{Start: Point2{100, 0}, End: Point2{100, 60}, TEnd: 1},
		LineSeg{Start: Point2{100, 60}, End: Point2{0, 60}, TEnd: 1},
		LineSeg{Start: Point2{0, 60}, End: Point2{0, 0}, TEnd: 1},
	}}
	hole := LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: Point2{50, 30}, Radius: units.Millimeters(8), TStart: 1},
	}}
	for _, tc := range []struct {
		name      string
		profile   ProfileRecord
		thickness float64
		want      bool
	}{
		{"empty rectangle", ProfileRecord{Outer: outer}, 5, true},
		{"circle post", ProfileRecord{Outer: outer, Holes: []LoopRecord{hole}}, 5, true},
		{"post blocks large disk", ProfileRecord{Outer: outer, Holes: []LoopRecord{hole}}, 25, false},
		{"margin passes", ProfileRecord{Outer: outer}, 30 - 4e-8, true},
		{"margin refuses", ProfileRecord{Outer: outer}, 30 - 2e-8, false},
		{"nonrectangle falls back", ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{0, 0}, End: Point2{100, 0}, TEnd: 1},
			LineSeg{Start: Point2{100, 0}, End: Point2{50, 60}, TEnd: 1},
			LineSeg{Start: Point2{50, 60}, End: Point2{0, 0}, TEnd: 1},
		}}}, 5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget := newWorkBudget(t.Context())
			loops, err := recordLoopsBudget(budget, tc.profile)
			require.NoError(t, err)
			got, err := shellRectCircleWitness(budget, tc.profile, loops, tc.thickness, 0)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
