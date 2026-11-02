package prismrecord_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCentroidEnvelopeSignedReplayMatchesRecordedEnvelope(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 1e12}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 3}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 3}, End: sectionrecord.Point2{U: 3, V: 2}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 3, V: 2}, End: sectionrecord.Point2{V: 2}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{V: 2}, End: sectionrecord.Point2{}, TEnd: 1},
	}}}
	coord, err := momentinput.CoordinateEnvelope(profile, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 10, Z1: 20, Profile: profile}
	held := pp.Point(1.5, 1, 15)
	want, err := pp.CentroidGeometryBound(profile, held, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	meter := proofbound.NewReplayTrialBudget()
	got, err := pp.CentroidEnvelopeSignedReplay(t.Context(), coord, held, meter)
	require.NoError(t, err)
	require.Equal(t, want, got)
	work, records := meter.Usage()
	require.Equal(t, uint64(160), work)
	require.Zero(t, records)
	require.Zero(t, meter.Bytes())
	exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
	require.NoError(t, err)
	_, err = pp.CentroidEnvelopeSignedReplay(t.Context(), coord, held, exact)
	require.NoError(t, err)
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	require.NoError(t, err)
	_, err = pp.CentroidEnvelopeSignedReplay(t.Context(), coord, held, short)
	require.Error(t, err)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = pp.CentroidEnvelopeSignedReplay(ctx, coord, held, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
}
