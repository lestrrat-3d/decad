package prismshell

import (
	"context"
	"errors"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func bossMeshTestInput(t *testing.T) BossMeshInput {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	return BossMeshInput{
		Verts: []r3.Vec{
			{X: -4, Y: -3, Z: 5}, {X: 4, Y: -3, Z: 5},
			{X: 4, Y: 3, Z: 5}, {X: -4, Y: 3, Z: 5},
		},
		VertexBound: []float64{0, 0, 0, 0},
		Groups:      []facetedrecord.Group{{Planar: true}, {Planar: true}},
		Tris:        [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 1, 3}},
		Src:         []int{1, 1, 0},
		Boss:        prismrecord.Payload{Frame: frame, Xform: r3.Identity()},
		PlateRect:   BossRect{U0: -10, V0: -10, U1: 10, V1: 10},
		BossRect:    BossRect{U0: -4, V0: -3, U1: 4, V1: 3},
		PlateBottom: 0, InterfaceZ: 4, BossBottom: 3, BossTop: 5,
		UnitLo: 1, UnitHi: 1, Outer: [4]int{0, 1, 2, 3},
		TopGroup: 1, Stations: 16,
	}
}

func TestBossMeshKeepsSourceOrderAndBoundsNewRoot(t *testing.T) {
	input := bossMeshTestInput(t)
	const producer provenance.Producer = 17
	mesh, err := PrismUnionBossShellMeshOf(t.Context(), producer, input, 1)
	if err != nil {
		t.Fatal(err)
	}
	if len(mesh.Verts) != 84 || len(mesh.VertexBound) != len(mesh.Verts) ||
		mesh.RootStart != 4 || mesh.BossPointEnd != 76 {
		t.Fatalf("vertices=%d bounds=%d root=[%d,%d)",
			len(mesh.Verts), len(mesh.VertexBound), mesh.RootStart, mesh.BossPointEnd)
	}
	if len(mesh.Tris) != len(mesh.Src) || mesh.Tris[0] != ([3]int{0, 1, 3}) ||
		mesh.Src[0] != 0 {
		t.Fatalf("retained triangle=%v group=%d, triangle count=%d, source count=%d",
			mesh.Tris[0], mesh.Src[0], len(mesh.Tris), len(mesh.Src))
	}
	if mesh.Verts[mesh.BossPointEnd-4].Z != input.BossBottom ||
		math.IsInf(mesh.MaxVertexError, 0) || math.IsNaN(mesh.MaxVertexError) {
		t.Fatalf("root end z=%g bound=%g", mesh.Verts[mesh.BossPointEnd-4].Z, mesh.MaxVertexError)
	}
	if len(mesh.Groups) != 20 ||
		mesh.Groups[2].Origins[0] != (provenance.Ref{Producer: producer, Role: "shellRim(0)"}) ||
		mesh.Groups[10].Origins[0] != (provenance.Ref{Producer: producer, Role: "shellRootBand(0)"}) ||
		mesh.Groups[10].Planar {
		t.Fatalf("groups=%d rim=%v root=%v",
			len(mesh.Groups), mesh.Groups[2].Origins, mesh.Groups[10].Origins)
	}
	for i, group := range mesh.Src {
		if group < 0 || group >= len(mesh.Groups) {
			t.Fatalf("triangle %d group=%d", i, group)
		}
	}
	input.Verts[0].X = 99
	input.Groups[0].Planar = false
	if mesh.Verts[0].X != -4 || !mesh.Groups[0].Planar {
		t.Fatal("source vertex or group storage was aliased")
	}
}

// Every root-band station lies at distance t = 1 from the root edge, the boss
// outline at InterfaceZ = 4: its inset q from the outline and its depth below
// InterfaceZ lie on the quarter circle centred on that edge. The band runs
// from the outline itself at the ledge to the inset t at InterfaceZ.
func TestBossMeshRootStationsKeepThicknessFromRootEdge(t *testing.T) {
	t.Parallel()
	input := bossMeshTestInput(t)
	mesh, err := PrismUnionBossShellMeshOf(t.Context(), 17, input, 1)
	require.NoError(t, err)
	// The upper walls' eight vertices come first, then the rings from the
	// station next to InterfaceZ down to the ledge.
	rings := mesh.Verts[mesh.RootStart+8 : mesh.BossPointEnd]
	require.Len(t, rings, 4*input.Stations)
	for i, v := range rings {
		q := min(v.X+4, 4-v.X, v.Y+3, 3-v.Y)
		depth := input.InterfaceZ - v.Z
		require.True(t, q >= 0 && depth > 0 && depth <= 1, "station %d at %v", i, v)
		require.InDelta(t, 1, math.Hypot(q, depth), mesh.VertexBound[mesh.RootStart+8+i]+1e-12,
			"station %d at %v", i, v)
	}
	ledge := rings[len(rings)-4:]
	for i, uv := range PrismUnionBossShellCorners(input.BossRect) {
		require.Equal(t, r3.Vec{X: uv[0], Y: uv[1], Z: input.BossBottom}, ledge[i], "ledge ring corner %d", i)
	}
}

func TestBossMeshKeepsUnsupportedAndTriangulationCancellation(t *testing.T) {
	input := bossMeshTestInput(t)
	input.UnitLo = 0
	_, err := PrismUnionBossShellMeshOf(t.Context(), 17, input, 1)
	if !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("unbounded lift error=%v", err)
	}
	input = bossMeshTestInput(t)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = PrismUnionBossShellMeshOf(ctx, 17, input, 1)
	if !errors.Is(err, decaderr.ErrUnsupported) ||
		!strings.Contains(err.Error(), context.Canceled.Error()) {
		t.Fatalf("triangulation cancellation error=%v", err)
	}
}
