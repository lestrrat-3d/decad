package prismrecord_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCircleCurveBoundSignedReplayMatchesOrdinary(t *testing.T) {
	for _, tc := range []struct {
		name        string
		origin      r3.Vec
		radiusBound float64
		reverse     bool
	}{
		{"plain", r3.Vec{}, 0, false},
		{"reversed", r3.Vec{X: 4, Y: -3}, 0, true},
		{"far placed", r3.Vec{X: 1e12, Y: -1e12}, 1e-12, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, r3.Vec{X: 1}, r3.Vec{Y: 1})
			require.NoError(t, err)
			pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), SectionDelta: 1e-13}
			center := pp.Point(1, 2, 3)
			axis := pp.Dir(0, 0, 1)
			if tc.reverse {
				axis = axis.Scale(-1)
			}
			want, wantOK := pp.CircleCurveBound(1, 2, 3, 0, 0, 5, tc.radiusBound, center, axis)
			meter := proofbound.NewReplayTrialBudget()
			got, gotOK, err := pp.CircleCurveBoundSignedReplay(t.Context(), 1, 2, 3,
				0, 0, 5, tc.radiusBound, center, axis, meter)
			require.NoError(t, err)
			require.Equal(t, wantOK, gotOK)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
			w, r := meter.Usage()
			bytes := meter.Bytes()
			for _, limits := range [...][3]uint64{{w - 1, r, bytes}, {w, r - 1, bytes}, {w, r, bytes - 1}} {
				short, e := proofbound.NewReplayBudgetLimits(limits[0], limits[1], limits[2])
				require.NoError(t, e)
				_, _, e = pp.CircleCurveBoundSignedReplay(t.Context(), 1, 2, 3,
					0, 0, 5, tc.radiusBound, center, axis, short)
				require.Error(t, e)
			}
		})
	}
}

func TestCircleCurveBoundSignedReplayRefusesTiltedFrame(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	require.NoError(t, err)
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity()}
	center := pp.Point(1, 2, 3)
	_, _, err = pp.CircleCurveBoundSignedReplay(t.Context(), 1, 2, 3, 0, 0, 5, 0,
		center, pp.Dir(0, 0, 1), proofbound.NewReplayTrialBudget())
	require.Error(t, err)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter := proofbound.NewReplayTrialBudget()
	_, _, err = pp.CircleCurveBoundSignedReplay(ctx, 1, 2, 3, 0, 0, 5, 0,
		center, pp.Dir(0, 0, 1), meter)
	require.ErrorIs(t, err, context.Canceled)
	w, r := meter.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, meter.Bytes())
}
