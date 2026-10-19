package boundarywalk_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestSurveyLoopsCancellationIsBounded(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	segments := make([]sectionrecord.CurveSegment, proofbound.WorkPollInterval+64)
	for i := range segments {
		segments[i] = sectionrecord.LineSeg{Start: sectionrecord.Point2{U: float64(i)}, End: sectionrecord.Point2{U: float64(i + 1)}, TEnd: 1}
	}
	_, err := boundarywalk.SurveyLoops(proofbound.NewWorkBudget(ctx), boundarywalk.Profile{
		Outer: sectionrecord.LoopRecord{Segments: segments},
	})
	require.ErrorIs(t, err, context.Canceled)
}
