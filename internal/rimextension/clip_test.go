package rimextension

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestClipRefusesMaterialOutsideCertifiedWindow(t *testing.T) {
	sec := Section{Stop: r3.Vec{}, End: r3.Vec{X: 1}, Toe: r3.Vec{Z: -1}, Heel: r3.Vec{Y: 1}}
	h, ok := newChart(sec)
	require.True(t, ok)
	// An unrelated cap triangle can lie above the infinite extension of a
	// bevel plane. The edit may remove only the bounded source region.
	vertices := []r3.Vec{{X: .1, Z: 2}, {X: .2, Z: 2}, {X: .1, Z: 3}}
	m := rewrite{ctx: t.Context(), budget: proofbound.NewStrictReplayBudget(), chart: h,
		support: SourceWindow{XMin: 0, XMax: 1, YMin: 0, YMax: 1, OffsetMin: -1, OffsetMax: 0}, out: Mesh{Vertices: vertices},
		exactPoints: map[int]point{}, distances: map[int][3]*big.Rat{}}
	_, changed := m.clipTriangle([3]int{0, 1, 2})
	require.False(t, changed)
	require.ErrorIs(t, m.err, errChart)
	// A degenerate triangle cannot overwrite a prior cancellation or work error.
	m.err = context.Canceled
	m.emit(0, 0, 0, 0)
	require.ErrorIs(t, m.err, context.Canceled)
}

func TestConvexFanPreservesSplitOnNarrowTriangle(t *testing.T) {
	v := []r3.Vec{{X: math.Nextafter(1, 0)}, {X: 1}, {X: 1, Y: 1}, {X: 1, Y: 2}}
	m := rewrite{ctx: t.Context(), budget: proofbound.NewStrictReplayBudget(), out: Mesh{Vertices: v}, exactPoints: map[int]point{}}
	m.convex([]int{0, 1, 2, 3}, 1)
	require.NoError(t, m.err)
	require.Equal(t, [][3]int{{0, 1, 2}, {0, 2, 3}}, m.out.Triangles)
}
