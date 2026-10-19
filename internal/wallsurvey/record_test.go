package wallsurvey_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoopsCancellationIsBounded(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	segments := make([]sectionrecord.CurveSegment, proofbound.WorkPollInterval+64)
	for i := range segments {
		segments[i] = sectionrecord.LineSeg{Start: sectionrecord.Point2{U: float64(i)}, End: sectionrecord.Point2{U: float64(i + 1)}, TEnd: 1}
	}
	_, err := wallsurvey.RevolveLoops(proofbound.NewWorkBudget(ctx), momentinput.Profile{
		Outer: sectionrecord.LoopRecord{Segments: segments},
	}, revolveaxis.Frame{DU: 1})
	require.ErrorIs(t, err, context.Canceled)
}
