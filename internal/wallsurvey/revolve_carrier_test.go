package wallsurvey_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRevolveCarrierInput(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1},
			TStart: 0, TEnd: 1,
		}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(0, 1, 2, 1), line(2, 1, 2, 3), line(2, 3, 0, 3), line(0, 3, 0, 1),
	}}}
	ax := revolveaxis.Frame{DU: 1, SnapTol: 1e-9}
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	lift := revolvemesh.RevolveLift{Frame: frame, DU: 1}
	axis := revolvemesh.AxisBound{}
	xform := r3.Identity()

	in, ok, err := wallsurvey.RevolveCarrierInput(proofbound.NewWorkBudget(t.Context()), profile, ax,
		lift, axis, xform, false, 0.25, 1.5)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, in.Walls, 4)
	for _, wall := range in.Walls {
		require.NotEqual(t, revolveaxis.WallAxis, wall.Kind)
	}
	require.Equal(t, 0.25, in.Phi0)
	require.Equal(t, 1.5, in.Phi1)
	require.Equal(t, xform, in.Transform)
	require.Equal(t, lift, in.Lift)
	require.False(t, in.Full)

	// A small profile never reaches the budget's polling interval, so the
	// canceled budget reports the cancellation at its first step instead.
	canceled := &proofbound.WorkBudget{
		StepFn: func() error { return context.Canceled },
		ErrFn:  func() error { return context.Canceled },
	}
	_, ok, err = wallsurvey.RevolveCarrierInput(canceled, profile, ax,
		lift, axis, xform, false, 0.25, 1.5)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
