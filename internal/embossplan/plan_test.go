package embossplan_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/embossplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func frame(tb testing.TB, z float64, flip bool) r3.Frame {
	tb.Helper()
	u, v := r3.Vec{X: 1}, r3.Vec{Y: 1}
	if flip {
		u, v = v, u
	}
	f, err := r3.NewFrame(r3.Vec{Z: z}, u, v)
	require.NoError(tb, err)
	return f
}

func TestPlanSides(t *testing.T) {
	t.Parallel()
	up, down := r3.Vec{Z: 1}, r3.Vec{Z: -1}
	cases := []struct {
		name   string
		sketch r3.Frame
		face   r3.Frame
		out    r3.Vec
		raised bool
		want   embossplan.Sweep
	}{
		{"raised, sketch normal outward", frame(t, 10, false), frame(t, 10, false), up, true, embossplan.Sweep{Along: 2, Against: embossplan.Margin}},
		{"engraved, sketch normal outward", frame(t, 10, false), frame(t, 10, false), up, false, embossplan.Sweep{Along: embossplan.Margin, Against: 2}},
		{"raised, sketch normal inward", frame(t, 0, false), frame(t, 0, true), down, true, embossplan.Sweep{Along: embossplan.Margin, Against: 2}},
		{"engraved, sketch normal inward", frame(t, 0, false), frame(t, 0, true), down, false, embossplan.Sweep{Along: 2, Against: embossplan.Margin}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := embossplan.Plan(tc.sketch, tc.face, tc.out, 2, tc.raised)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

func TestPlanRefusals(t *testing.T) {
	t.Parallel()
	top := frame(t, 10, false)
	_, err := embossplan.Plan(frame(t, 9, false), top, r3.Vec{Z: 1}, 2, true)
	require.ErrorContains(t, err, "off the face plane")
	tilted, err := r3.NewFrame(r3.Vec{Z: 10}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	_, err = embossplan.Plan(tilted, top, r3.Vec{Z: 1}, 2, true)
	require.ErrorContains(t, err, "not parallel")
	_, err = embossplan.Plan(top, top, r3.Vec{Z: 1}, 0, true)
	require.ErrorContains(t, err, "height")
}

func TestRemovesNothing(t *testing.T) {
	t.Parallel()
	require.True(t, embossplan.RemovesNothing(100, 0, 100, 0))
	require.False(t, embossplan.RemovesNothing(100, 0, 99, 0))
	require.False(t, embossplan.RemovesNothing(100, 1, 100, 0), "a bound on the reading before keeps the answer open")
}
