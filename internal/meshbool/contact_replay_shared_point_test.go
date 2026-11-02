package meshbool

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// These tests need package access to the shared-point certificate and fallback.
func TestReplaySharedPointControls(t *testing.T) {
	a := [3]r3.Vec{{}, {X: 2}, {Y: 2}}
	cases := []struct {
		name    string
		b       [3]r3.Vec
		certain bool
	}{
		{"point", [3]r3.Vec{{}, {X: -1, Y: -1}, {X: -2, Y: -1}}, true},
		{"segment", [3]r3.Vec{{}, {X: 1}, {Y: -1}}, false},
		{"region", [3]r3.Vec{{}, {X: 1, Y: 0.25}, {X: 0.25, Y: 1}}, false},
		{"collinear boundary", [3]r3.Vec{{}, {X: -1}, {Y: -1}}, false},
		{"two shared vertices", [3]r3.Vec{{}, {X: 2}, {Y: -1}}, false},
		{"no shared vertex", [3]r3.Vec{{X: 3, Y: 3}, {X: 4, Y: 3}, {X: 3, Y: 4}}, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			for shiftA := range 3 {
				for shiftB := range 3 {
					for reverse := range 2 {
						x, y := a, tc.b
						for i := range 3 {
							x[i], y[i] = a[(i+shiftA)%3], tc.b[(i+shiftB)%3]
						}
						if reverse != 0 {
							y[1], y[2] = y[2], y[1]
						}
						shared, certain := replaySharedPointSeparated(x, y, 0, 1)
						require.Equal(t, tc.certain, certain)
						if certain {
							require.Equal(t, r3.Vec{}, x[shared])
						}
						requireReplaySharedPointMatchesExact(t, x, y)
					}
				}
			}
		})
	}
}

func requireReplaySharedPointMatchesExact(t *testing.T, a, b [3]r3.Vec) {
	t.Helper()
	var xa, xb [3]proof.Xpt
	for i := range 3 {
		xa[i], xb[i] = proof.XptOf(a[i]), proof.XptOf(b[i])
	}
	na, nb := proof.XtriangleNormal(xa[0], xa[1], xa[2]), proof.XtriangleNormal(xb[0], xb[1], xb[2])
	want, err := replayCoplanarContactHeld(t.Context(), a, b, xa, xb, na, nb, proofbound.NewBrakeReplayBudget(), false)
	require.NoError(t, err)
	got, err := replayCoplanarContactHeld(t.Context(), a, b, xa, xb, na, nb, proofbound.NewBrakeReplayBudget(), true)
	require.NoError(t, err)
	require.Equal(t, want.Kind, got.Kind)
	require.Equal(t, want.EdgeA, got.EdgeA)
	require.Equal(t, want.EdgeB, got.EdgeB)
	if got.Kind == ContactPoint {
		require.True(t, want.P0.Key() == got.P0.Key(), "first contact point changed")
		require.True(t, want.P1.Key() == got.P1.Key(), "last contact point changed")
	}
}

func TestReplaySharedPointUncertainBoundaries(t *testing.T) {
	a := [3]r3.Vec{{}, {X: 1, Y: 1}, {X: 1, Y: math.Nextafter(1, math.Inf(1))}}
	b := [3]r3.Vec{{}, {X: -1, Y: -1}, {X: -1, Y: math.Nextafter(-1, math.Inf(-1))}}
	_, certain := replaySharedPointSeparated(a, b, 0, 1)
	require.False(t, certain)
	requireReplaySharedPointMatchesExact(t, a, b)
	for _, scale := range []float64{1e-310, 1e100} {
		a = [3]r3.Vec{{}, {X: scale}, {Y: scale}}
		b = [3]r3.Vec{{}, {X: -scale, Y: -scale}, {X: -2 * scale, Y: -scale}}
		_, certain = replaySharedPointSeparated(a, b, 0, 1)
		require.False(t, certain)
		requireReplaySharedPointMatchesExact(t, a, b)
	}
	for _, value := range []float64{math.Inf(1), math.Inf(-1), math.NaN()} {
		a = [3]r3.Vec{{}, {X: value}, {Y: 1}}
		b = [3]r3.Vec{{}, {X: -1, Y: -1}, {X: -2, Y: -1}}
		_, certain = replaySharedPointSeparated(a, b, 0, 1)
		require.False(t, certain)
	}
}

func TestReplaySharedPointRandomExactComparison(t *testing.T) {
	rng := rand.New(rand.NewPCG(491, 881))
	for range 400 {
		var a, b [3]r3.Vec
		for i := 1; i < 3; i++ {
			a[i] = r3.NewVec(float64(rng.IntN(129)-64)/16, float64(rng.IntN(129)-64)/16, 0)
			b[i] = r3.NewVec(float64(rng.IntN(129)-64)/16, float64(rng.IntN(129)-64)/16, 0)
		}
		requireReplaySharedPointMatchesExact(t, a, b)
	}
}

func TestReplaySharedPointBudgetAndOwnership(t *testing.T) {
	a := [3]r3.Vec{{}, {X: 2}, {Y: 2}}
	b := [3]r3.Vec{{}, {X: -1, Y: -1}, {X: -2, Y: -1}}
	var xa, xb [3]proof.Xpt
	for i := range 3 {
		xa[i], xb[i] = proof.XptOf(a[i]), proof.XptOf(b[i])
	}
	na, nb := proof.XtriangleNormal(xa[0], xa[1], xa[2]), proof.XtriangleNormal(xb[0], xb[1], xb[2])
	run := func(ctx context.Context, meter *proofbound.ReplayBudget) (TriContact, error) {
		return replayCoplanarContactHeld(ctx, a, b, xa, xb, na, nb, meter, true)
	}
	full := proofbound.NewBrakeReplayBudget()
	got, err := run(t.Context(), full)
	require.NoError(t, err)
	require.Equal(t, ContactPoint, got.Kind)
	require.True(t, got.P0 == xa[0] && got.P1 == xa[0], "the original shared vertex must be retained")
	work, records := full.Usage()
	require.Equal(t, uint64(0), records)
	require.Zero(t, full.Bytes())
	for _, short := range []uint64{0, 1} {
		meter := proofbound.NewBrakeReplayBudget()
		require.NoError(t, meter.SpendWork(t.Context(), proofbound.BrakeReplayWork-work+short))
		require.NoError(t, meter.ReserveRecords(t.Context(), proofbound.BrakeReplayRecords))
		require.NoError(t, meter.ReserveBytes(t.Context(), proofbound.BrakeReplayBytes))
		_, err := run(t.Context(), meter)
		if short == 0 {
			require.NoError(t, err)
		} else {
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter := proofbound.NewBrakeReplayBudget()
	_, err = run(ctx, meter)
	require.ErrorIs(t, err, context.Canceled)
	work, records = meter.Usage()
	require.Zero(t, work)
	require.Zero(t, records)
	require.Zero(t, meter.Bytes())
	meter = proofbound.NewBrakeReplayBudget()
	require.Zero(t, testing.AllocsPerRun(10, func() {
		result, e := run(t.Context(), meter)
		if e != nil || result.P0 != xa[0] {
			panic("shared point changed")
		}
	}))
}

func TestReplaySharedPointBothTrianglesAndPlanes(t *testing.T) {
	for _, axes := range [][2]int{{0, 1}, {2, 0}, {1, 2}} {
		point := func(u, v float64) r3.Vec {
			var coordinates [3]float64
			coordinates[axes[0]], coordinates[axes[1]] = u, v
			return r3.NewVec(coordinates[0], coordinates[1], coordinates[2])
		}
		// Neither edge of a separates b. The certificate needs an edge of b.
		a := [3]r3.Vec{point(0, 0), point(2, 0), point(2, 1)}
		b := [3]r3.Vec{point(0, 0), point(-1, 2), point(-1, -2)}
		shared, certain := replaySharedPointSeparated(a, b, axes[0], axes[1])
		require.True(t, certain)
		require.Zero(t, shared)
		requireReplaySharedPointMatchesExact(t, a, b)
		requireReplaySharedPointMatchesExact(t, b, a)
	}
	// Degenerate normals retain the existing exact classifier's behavior.
	a := [3]r3.Vec{{}, {X: 2}, {Y: 2}}
	b := [3]r3.Vec{{X: 2}, {X: 3, Y: -1}, {X: 4, Y: -2}}
	requireReplaySharedPointMatchesExact(t, a, b)
	requireReplaySharedPointMatchesExact(t, b, a)
}

func TestReplaySharedPointSpatialExactComparison(t *testing.T) {
	rng := rand.New(rand.NewPCG(777, 991))
	certified, uncertain := 0, 0
	for range 500 {
		var a, b [3]r3.Vec
		var xa, xb [3]proof.Xpt
		for i := 1; i < 3; i++ {
			a[i] = r3.NewVec(float64(rng.IntN(65)-32)/8, float64(rng.IntN(65)-32)/8, float64(rng.IntN(65)-32)/8)
			b[i] = r3.NewVec(float64(rng.IntN(65)-32)/8, float64(rng.IntN(65)-32)/8, float64(rng.IntN(65)-32)/8)
		}
		for i := range 3 {
			xa[i], xb[i] = proof.XptOf(a[i]), proof.XptOf(b[i])
		}
		na, nb := proof.XtriangleNormal(xa[0], xa[1], xa[2]), proof.XtriangleNormal(xb[0], xb[1], xb[2])
		certain, err := replaySharedPointSeparatedAxes(t.Context(), a, b, na, nb, proofbound.NewBrakeReplayBudget())
		require.NoError(t, err)
		if !certain {
			uncertain++
			continue
		}
		certified++
		meter := proofbound.NewBrakeReplayBudget()
		sa, sb, separated, err := replayContactSigns(t.Context(), a, b, xa, xb, na, nb, meter)
		require.NoError(t, err)
		require.False(t, separated)
		var exact TriContact
		if CountZero(sa) == 3 || CountZero(sb) == 3 {
			exact, err = replayCoplanarContactHeld(t.Context(), a, b, xa, xb, na, nb, meter, false)
		} else {
			exact, err = replayCrossingContactHeld(t.Context(), a, b, xa, xb, na, nb, sa, sb, r3.Vec{}, r3.Vec{}, meter, true)
		}
		require.NoError(t, err)
		require.Equal(t, ContactPoint, exact.Kind)
		require.True(t, exact.P0.Key() == xa[0].Key(), "first exact endpoint differs from the shared vertex")
		require.True(t, exact.P1.Key() == xa[0].Key(), "last exact endpoint differs from the shared vertex")
	}
	require.Positive(t, certified)
	require.Positive(t, uncertain)
}

func TestReplaySharedPointProjectionBudget(t *testing.T) {
	a := [3]r3.Vec{{}, {X: 2}, {Y: 2}}
	cases := []struct {
		b       [3]r3.Vec
		certain bool
	}{
		{[3]r3.Vec{{}, {X: -1, Y: -1, Z: 1}, {X: -2, Y: -1, Z: 3}}, true},
		{[3]r3.Vec{{}, {X: 1}, {Y: -1, Z: 1}}, false},
		{[3]r3.Vec{{}, {X: 1, Y: 1, Z: 1}, {X: 1, Y: 1, Z: -1}}, false},
		{[3]r3.Vec{{}, {Y: -1}, {Z: 1}}, false},
	}
	for _, tc := range cases {
		var xa, xb [3]proof.Xpt
		for i := range 3 {
			xa[i], xb[i] = proof.XptOf(a[i]), proof.XptOf(tc.b[i])
		}
		na, nb := proof.XtriangleNormal(xa[0], xa[1], xa[2]), proof.XtriangleNormal(xb[0], xb[1], xb[2])
		run := func(ctx context.Context, meter *proofbound.ReplayBudget) (bool, error) {
			return replaySharedPointSeparatedAxes(ctx, a, tc.b, na, nb, meter)
		}
		full := proofbound.NewBrakeReplayBudget()
		certain, err := run(t.Context(), full)
		require.NoError(t, err)
		require.Equal(t, tc.certain, certain)
		work, records := full.Usage()
		require.Zero(t, records)
		require.Zero(t, full.Bytes())
		for _, short := range []uint64{0, 1} {
			meter := proofbound.NewBrakeReplayBudget()
			require.NoError(t, meter.SpendWork(t.Context(), proofbound.BrakeReplayWork-work+short))
			_, err = run(t.Context(), meter)
			if short == 0 {
				require.NoError(t, err)
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
			}
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		meter := proofbound.NewBrakeReplayBudget()
		_, err = run(ctx, meter)
		require.ErrorIs(t, err, context.Canceled)
		work, records = meter.Usage()
		require.Zero(t, work)
		require.Zero(t, records)
		require.Zero(t, meter.Bytes())
		meter = proofbound.NewBrakeReplayBudget()
		require.Zero(t, testing.AllocsPerRun(10, func() {
			_, e := run(t.Context(), meter)
			if e != nil {
				panic(e)
			}
		}))
	}
}
