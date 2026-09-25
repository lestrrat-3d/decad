package decad

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveBoundsSharedProfileMatchesIndependentExtents(t *testing.T) {
	t.Parallel()
	profiles := []struct {
		name    string
		profile ProfileRecord
		full    bool
		phi1    float64
		den     sweepDenotation
	}{
		{
			name:    "straight full turn",
			profile: dipShaftBandProfile(10, 0),
			full:    true,
			phi1:    2 * math.Pi,
			den:     fullTurnDenotation(),
		},
		{
			name: "circular quarter turn",
			profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
				CircleSeg{Center: Point2{U: 2, V: 5}, Radius: units.Millimeters(2), CCW: true, TStart: 0, TEnd: 1},
			}}},
			phi1: math.Pi / 2,
			den:  quarterTurnDenotation(),
		},
	}
	for _, test := range profiles {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			rp := revolvePayload{
				profile: test.profile,
				frame:   axisAlignedFrame(t),
				ax:      axisFrame{dU: 1},
				phi1:    test.phi1,
				full:    test.full,
				den:     test.den,
				xform:   r3.Identity(),
			}
			got, err := revolveBoundsContext(t.Context(), rp, newFreeformWork())
			require.NoError(t, err)
			cached, err := resolveAnalyticRevolveExtentProfile(t.Context(), test.profile, newFreeformWork())
			require.NoError(t, err)
			envelope, err := profileCoordinateEnvelope(test.profile, newFreeformWork(), nil)
			require.NoError(t, err)
			require.Equal(t, envelope, cached.coordUpper)

			axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
			var low, high [3]float64
			bound := 0.0
			for i, axis := range axes {
				var axisBound float64
				low[i], high[i], axisBound, err = rp.extentBoundedAlong(t.Context(), axis, newFreeformWork())
				require.NoError(t, err)
				bound = math.Max(bound, axisBound)
			}
			want := Box{
				Min:       r3.NewVec(low[0], low[1], low[2]),
				Max:       r3.NewVec(high[0], high[1], high[2]),
				Exactness: exactnessOf(bound),
				Bound:     units.Millimeters(bound),
			}
			require.Equal(t, want, got)
		})
	}
}

type cancelAfterExtentChecks struct {
	remaining int
}

func (*cancelAfterExtentChecks) Deadline() (time.Time, bool) { return time.Time{}, false }
func (*cancelAfterExtentChecks) Done() <-chan struct{}       { return nil }
func (*cancelAfterExtentChecks) Value(any) any               { return nil }

func (ctx *cancelAfterExtentChecks) Err() error {
	ctx.remaining--
	if ctx.remaining <= 0 {
		return context.Canceled
	}
	return nil
}

func TestRevolveBoundsSharedProfilePollsCancellation(t *testing.T) {
	t.Parallel()
	ctx := &cancelAfterExtentChecks{remaining: 3}
	_, err := resolveAnalyticRevolveExtentProfile(ctx, dipShaftBandProfile(10, 0), newFreeformWork())
	require.ErrorIs(t, err, context.Canceled)
}
