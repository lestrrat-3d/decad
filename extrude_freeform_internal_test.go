package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file covers docs/spline-design.md §10 P4b's build obligations that
// need a record built directly rather than through sketch's own arrangement
// — the same pattern extrude_work_test.go and prism_boolean_internal_test.go
// already use for a synthetic prismPayload.

// TestEvalPrismCollapsedSpanRunStillBuilds asserts observable test 11: a
// walk carrying a RUN of collapsed spans inside a longer clamped net still
// builds and reports its Volume — §6.5 SKIPS a collapsed span rather than
// refusing the body (Table K). This reuses
// TestConsecutiveCollapsedSpansPairAcrossTheWholeRun's own fixture
// (spline_convexity_internal_test.go), which already pins the certificate's
// own verdict on it (freeformConvexityPositive); this test pins the BUILD.
// Tessellating it is P5, out of this increment's scope.
func TestEvalPrismCollapsedSpanRunStillBuilds(t *testing.T) {
	seg := NURBSSeg{
		Degree: 1,
		Control: []Point2{
			{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 0}, {U: 1, V: 0}, {U: 1, V: 1},
		},
		Knots:   []float64{0, 0, 1, 2, 3, 4, 4},
		Weights: []float64{1, 1, 1, 1, 1},
		TStart:  0,
		TEnd:    1,
	}
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		seg,
		LineSeg{Start: Point2{U: 1, V: 1}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)

	body, err := evalPrism(New(), 0, prismPayload{
		profile: profile,
		frame:   frame,
		z1:      3,
		xform:   r3.Identity(),
	}, newFreeformWork())
	require.NoError(t, err)
	require.NotNil(t, body)
	require.Greater(t, body.volume.Value.Mag(), 0.0, "the walk's own length stays positive despite the collapsed run")
}
