package prismrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func freeformWalkFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return frame
}

func freeformWalkPayload(t *testing.T) prismrecord.Payload {
	t.Helper()
	start := sectionrecord.Point2{}
	end := sectionrecord.Point2{U: 8}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: start, End: end, TEnd: 1},
		sectionrecord.FitSplineSeg{
			Fit: []sectionrecord.Point2{start, {U: 4, V: 3}, end}, TStart: 1,
		},
	}}}
	return prismrecord.Payload{Profile: profile, Frame: freeformWalkFrame(t), Z1: 5, Xform: r3.Identity()}
}

func TestBuildCollapsedSpanRunReportsVolume(t *testing.T) {
	t.Parallel()
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.NURBSSeg{
			Degree: 1,
			Control: []sectionrecord.Point2{
				{}, {U: 1}, {U: 1}, {U: 1}, {U: 1, V: 1},
			},
			Knots: []float64{0, 0, 1, 2, 3, 4, 4}, Weights: []float64{1, 1, 1, 1, 1}, TEnd: 1,
		},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1, V: 1}, End: sectionrecord.Point2{}, TEnd: 1},
	}}}
	payload := prismrecord.Payload{Profile: profile, Frame: freeformWalkFrame(t), Z1: 3, Xform: r3.Identity()}
	body, err := prismrecord.Build(t.Context(), captureBuildEnv(), payload, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.InDelta(t, 1.5, body.Volume.Value.Mag(), 1e-12)
}

func TestBuildFreeformWalkReuseMatchesFreshBuildAndBudget(t *testing.T) {
	t.Parallel()
	payload := freeformWalkPayload(t)
	initial, err := prismrecord.Build(t.Context(), captureBuildEnv(), payload, freeform.NewFreeformWork())
	require.NoError(t, err)
	cached := initial.Payload.(prismrecord.Payload)
	require.NotNil(t, cached.Walks)
	require.True(t, cached.Walks.Reusable(cached.Profile))

	motion, err := r3.Translation(r3.NewVec(3, -4, 7))
	require.NoError(t, err)
	cached.Xform = motion
	warmWork := freeform.NewFreeformWork()
	warm, err := prismrecord.Build(t.Context(), captureBuildEnv(), cached, warmWork)
	require.NoError(t, err)
	require.Same(t, cached.Walks, warm.Payload.(prismrecord.Payload).Walks)

	fresh := cached
	fresh.Walks = nil
	coldWork := freeform.NewFreeformWork()
	cold, err := prismrecord.Build(t.Context(), captureBuildEnv(), fresh, coldWork)
	require.NoError(t, err)
	requireCaptureBody(t, cold, warm)
	require.Equal(t, coldWork.Spent, warmWork.Spent)
	require.Equal(t, coldWork.ReconstructionSpent, warmWork.ReconstructionSpent)
	require.Positive(t, coldWork.Spent)

	starved := func() *freeform.FreeformWork {
		work := freeform.NewFreeformWork()
		require.NoError(t, work.Step(freeform.FreeformWorkLimit-coldWork.Spent+1))
		return work
	}
	_, warmErr := prismrecord.Build(t.Context(), captureBuildEnv(), cached, starved())
	_, coldErr := prismrecord.Build(t.Context(), captureBuildEnv(), fresh, starved())
	require.ErrorIs(t, warmErr, momentinput.ErrUnsupported)
	require.ErrorIs(t, coldErr, momentinput.ErrUnsupported)
}

func TestBuildFreeformWalkReuseIsReadOnlyAcrossGoroutines(t *testing.T) {
	t.Parallel()
	initial, err := prismrecord.Build(t.Context(), captureBuildEnv(), freeformWalkPayload(t), freeform.NewFreeformWork())
	require.NoError(t, err)
	payload := initial.Payload.(prismrecord.Payload)
	first, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	second, err := r3.Translation(r3.NewVec(-2, 3, 4))
	require.NoError(t, err)

	type result struct {
		body *topo.Body
		err  error
	}
	results := make(chan result, 2)
	for _, motion := range []r3.Transform{first, second} {
		go func() {
			placed := payload
			placed.Xform = motion
			body, buildErr := prismrecord.Build(t.Context(), captureBuildEnv(), placed, freeform.NewFreeformWork())
			results <- result{body: body, err: buildErr}
		}()
	}
	for range 2 {
		got := <-results
		require.NoError(t, got.err)
		require.Same(t, payload.Walks, got.body.Payload.(prismrecord.Payload).Walks)
		require.InDelta(t, initial.Volume.Value.Mag(), got.body.Volume.Value.Mag(), 1e-9)
	}
}
