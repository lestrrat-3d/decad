package holeplan_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/holeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func topFrame(tb testing.TB) r3.Frame {
	tb.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(tb, err)
	return f
}

func TestComputeCountersinkDepth(t *testing.T) {
	t.Parallel()
	plan, err := holeplan.Compute(holeplan.Input{
		Frame:    topFrame(t),
		Outward:  r3.Vec{Z: 1},
		At:       r3.Vec{X: 3, Y: 4, Z: 10},
		Diameter: 4,
		Through:  true,
		Reach:    11,
		Head:     holeplan.Head{Kind: holeplan.HeadCountersink, Diameter: 8, Angle: math.Pi / 2},
	})
	require.NoError(t, err)
	require.True(t, plan.Against, "the face normal points outward, so the hole runs against it")
	require.Equal(t, 3.0, plan.U)
	require.Equal(t, 4.0, plan.V)
	require.NotNil(t, plan.Sink)
	// A 90 degree sink 8 wide over a bore 4 wide is 2 deep.
	require.InDelta(t, -2.0, plan.Sink.Profile[0][1], 1e-12)
	require.InDelta(t, 2.0, plan.Sink.Profile[1][0], 1e-12)
	require.Equal(t, holeplan.MouthMargin+11.0, plan.Bore.Tool())
}

func TestComputeRefusals(t *testing.T) {
	t.Parallel()
	base := holeplan.Input{
		Frame: topFrame(t), Outward: r3.Vec{Z: 1}, At: r3.Vec{Z: 10}, Diameter: 4, Depth: 5, Reach: 11,
	}
	cases := map[string]func(*holeplan.Input){
		"centre off the plane": func(in *holeplan.Input) { in.At.Z = 9 },
		"no reach":             func(in *holeplan.Input) { in.Reach = 0 },
		"nan diameter":         func(in *holeplan.Input) { in.Diameter = math.NaN() },
		"counterbore too deep": func(in *holeplan.Input) {
			in.Head = holeplan.Head{Kind: holeplan.HeadCounterbore, Diameter: 8, Depth: 5}
		},
	}
	for name, edit := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			in := base
			edit(&in)
			_, err := holeplan.Compute(in)
			require.Error(t, err)
		})
	}
}

func TestReach(t *testing.T) {
	t.Parallel()
	lo, hi := r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10}
	require.Equal(t, 10+0.5+holeplan.ThroughMargin, holeplan.Reach(lo, hi, r3.Vec{Z: 10}, r3.Vec{Z: 1}, 0.5))
	require.Equal(t, 10+0.5+holeplan.ThroughMargin, holeplan.Reach(lo, hi, r3.Vec{}, r3.Vec{Z: -1}, 0.5))
	require.Zero(t, holeplan.Reach(lo, hi, r3.Vec{Z: 0}, r3.Vec{Z: 1}, 0.5), "no material behind a face looking at the body")
}

func TestComputeRectCorners(t *testing.T) {
	t.Parallel()
	in := holeplan.Input{
		Frame: topFrame(t), Outward: r3.Vec{Z: 1}, At: r3.Vec{X: 3, Y: 4, Z: 10}, Through: true, Reach: 11,
		Rect: &holeplan.Rect{Width: 6, Height: 4},
	}
	plan, err := holeplan.Compute(in)
	require.NoError(t, err)
	require.NotNil(t, plan.Corners)
	require.Equal(t, [4][2]float64{{0, 2}, {6, 2}, {6, 6}, {0, 6}}, *plan.Corners, "an unturned rectangle is exact")

	in.Rect = &holeplan.Rect{Width: 6, Height: 4, Angle: math.Pi / 2}
	plan, err = holeplan.Compute(in)
	require.NoError(t, err)
	// A quarter turn about (3, 4) takes the corner (-3, -2) offset to (2, -3).
	require.InDelta(t, 5.0, plan.Corners[0][0], 1e-12)
	require.InDelta(t, 1.0, plan.Corners[0][1], 1e-12)
}

func TestComputeRectRefusals(t *testing.T) {
	t.Parallel()
	base := holeplan.Input{
		Frame: topFrame(t), Outward: r3.Vec{Z: 1}, At: r3.Vec{Z: 10}, Through: true, Reach: 11,
		Rect: &holeplan.Rect{Width: 6, Height: 4},
	}
	cases := map[string]func(*holeplan.Input){
		"zero width": func(in *holeplan.Input) { in.Rect = &holeplan.Rect{Width: 0, Height: 4} },
		"nan angle":  func(in *holeplan.Input) { in.Rect = &holeplan.Rect{Width: 6, Height: 4, Angle: math.NaN()} },
		"with a head": func(in *holeplan.Input) {
			in.Head = holeplan.Head{Kind: holeplan.HeadCountersink, Diameter: 8, Angle: 1}
		},
	}
	for name, edit := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			in := base
			edit(&in)
			_, err := holeplan.Compute(in)
			require.Error(t, err)
		})
	}
}

func TestComputeSlotTool(t *testing.T) {
	t.Parallel()
	in := holeplan.Input{
		Frame: topFrame(t), Outward: r3.Vec{Z: 1}, At: r3.Vec{X: 3, Y: 4, Z: 10}, Through: true, Reach: 11,
		Slot: &holeplan.Slot{Length: 10, Width: 4},
	}
	plan, err := holeplan.Compute(in)
	require.NoError(t, err)
	require.NotNil(t, plan.SlotTool)
	require.Equal(t, holeplan.SlotTool{
		Start: [2]float64{0, 4}, End: [2]float64{6, 4},
		P0: [2]float64{0, 2}, P1: [2]float64{6, 2}, P2: [2]float64{6, 6}, P3: [2]float64{0, 6},
	}, *plan.SlotTool, "an unturned slot is exact")

	in.Slot = &holeplan.Slot{Length: 10, Width: 4, Angle: math.Pi / 2}
	plan, err = holeplan.Compute(in)
	require.NoError(t, err)
	// A quarter turn about (3, 4) lays the end centres along y.
	require.InDelta(t, 3.0, plan.SlotTool.Start[0], 1e-12)
	require.InDelta(t, 1.0, plan.SlotTool.Start[1], 1e-12)
	require.InDelta(t, 7.0, plan.SlotTool.End[1], 1e-12)
}

func TestComputeSlotRefusals(t *testing.T) {
	t.Parallel()
	base := holeplan.Input{
		Frame: topFrame(t), Outward: r3.Vec{Z: 1}, At: r3.Vec{Z: 10}, Through: true, Reach: 11,
		Slot: &holeplan.Slot{Length: 10, Width: 4},
	}
	cases := map[string]func(*holeplan.Input){
		"length equals width": func(in *holeplan.Input) { in.Slot = &holeplan.Slot{Length: 4, Width: 4} },
		"nan angle":           func(in *holeplan.Input) { in.Slot = &holeplan.Slot{Length: 10, Width: 4, Angle: math.NaN()} },
		"both shapes":         func(in *holeplan.Input) { in.Rect = &holeplan.Rect{Width: 1, Height: 1} },
	}
	for name, edit := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			in := base
			edit(&in)
			_, err := holeplan.Compute(in)
			require.Error(t, err)
		})
	}
}
