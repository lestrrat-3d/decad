package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestShellRectCircleWitnessKeepsToleranceAndFallsBack(t *testing.T) {
	t.Parallel()
	outer := loopRecord{Segments: []curveSegment{
		lineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 100, V: 0}, TEnd: 1},
		lineSeg{Start: Point2{U: 100, V: 0}, End: Point2{U: 100, V: 60}, TEnd: 1},
		lineSeg{Start: Point2{U: 100, V: 60}, End: Point2{U: 0, V: 60}, TEnd: 1},
		lineSeg{Start: Point2{U: 0, V: 60}, End: Point2{U: 0, V: 0}, TEnd: 1},
	}}
	hole := loopRecord{Segments: []curveSegment{
		circleSeg{Center: Point2{U: 50, V: 30}, Radius: units.Millimeters(8), TStart: 1},
	}}
	for _, tc := range []struct {
		name      string
		profile   profileRecord
		thickness float64
		want      bool
	}{
		{"empty rectangle", profileRecord{Outer: outer}, 5, true},
		{"circle post", profileRecord{Outer: outer, Holes: []loopRecord{hole}}, 5, true},
		{"post blocks large disk", profileRecord{Outer: outer, Holes: []loopRecord{hole}}, 25, false},
		{"margin passes", profileRecord{Outer: outer}, 30 - 4e-8, true},
		{"margin refuses", profileRecord{Outer: outer}, 30 - 2e-8, false},
		{"nonrectangle falls back", profileRecord{Outer: loopRecord{Segments: []curveSegment{
			lineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 100, V: 0}, TEnd: 1},
			lineSeg{Start: Point2{U: 100, V: 0}, End: Point2{U: 50, V: 60}, TEnd: 1},
			lineSeg{Start: Point2{U: 50, V: 60}, End: Point2{U: 0, V: 0}, TEnd: 1},
		}}}, 5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget := proofbound.NewWorkBudget(t.Context())
			loops, err := boundarywalk.SurveyLoopsBudget(budget, boundarywalk.Profile(tc.profile))
			require.NoError(t, err)
			got, err := shellsurvey.RectangleCircleWitness(budget, boundarywalk.Profile(tc.profile), loops, tc.thickness, 0, shellTol)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
