package tessellation_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/stretchr/testify/require"
)

func TestReplayClosedMeshMatchesAudit(t *testing.T) {
	for _, tris := range [][][3]int{tetrahedron(), {{0, 1, 2}, {0, 2, 3}}, append(tetrahedron(), [3]int{0, 1, 2})} {
		ordinary := tessellation.RequireClosedMesh(tris)
		meter := proofbound.NewReplayTrialBudget()
		got := tessellation.RequireClosedMeshReplay(t.Context(), tris, meter)
		require.Equal(t, ordinary == nil, got == nil)
		if ordinary != nil {
			requireAudit(t, got, tessellation.Degenerate)
		}
		require.Positive(t, meter.Bytes())
	}
}

func TestReplayVertexLinksMatchAudit(t *testing.T) {
	separate := append(tetrahedron(), [][3]int{{4, 5, 6}, {4, 6, 7}, {4, 7, 5}, {5, 7, 6}}...)
	pinched := append(tetrahedron(), [][3]int{{0, 4, 5}, {0, 5, 6}, {0, 6, 4}, {4, 6, 5}}...)
	for _, tc := range []struct {
		verts int
		tris  [][3]int
	}{{4, tetrahedron()}, {8, separate}, {7, pinched}, {5, [][3]int{{0, 1, 2}, {0, 2, 3}}},
		{606, vertexLinkBipyramid(600)}} {
		ordinary := tessellation.RequireVertexLinks(t.Context(), tc.verts, tc.tris)
		meter := proofbound.NewReplayTrialBudget()
		got := tessellation.RequireVertexLinksReplay(t.Context(), tc.verts, tc.tris, meter)
		require.Equal(t, ordinary == nil, got == nil)
		if ordinary != nil {
			require.ErrorIs(t, got, decaderr.ErrUnsupported)
		}
		require.Positive(t, meter.Bytes())
	}
}

func TestReplayMeshAuditsChargeBeforeAllocation(t *testing.T) {
	for _, audit := range []func(*proofbound.ReplayBudget) error{
		func(b *proofbound.ReplayBudget) error {
			return tessellation.RequireClosedMeshReplay(t.Context(), tetrahedron(), b)
		},
		func(b *proofbound.ReplayBudget) error {
			return tessellation.RequireVertexLinksReplay(t.Context(), 4, tetrahedron(), b)
		},
	} {
		measured := proofbound.NewReplayTrialBudget()
		require.NoError(t, audit(measured))
		work, _ := measured.Usage()
		bytes := measured.Bytes()
		for _, short := range []bool{false, true} {
			b := proofbound.NewReplayTrialBudget()
			used := proofbound.ReplayTrialBytes - bytes
			if short {
				used++
			}
			require.NoError(t, b.ReserveBytes(t.Context(), used))
			err := audit(b)
			if short {
				require.True(t, errors.Is(err, decaderr.ErrUnsupported))
			} else {
				require.NoError(t, err)
			}
			b = proofbound.NewReplayTrialBudget()
			used = proofbound.ReplayTrialWork - work
			if short {
				used++
			}
			require.NoError(t, b.SpendWork(t.Context(), used))
			err = audit(b)
			if short {
				require.True(t, errors.Is(err, decaderr.ErrUnsupported))
			} else {
				require.NoError(t, err)
			}
		}
	}
}

func TestReplayMeshAuditsRecordLimitsAndCancellation(t *testing.T) {
	audits := []func(context.Context, *proofbound.ReplayBudget) error{
		func(ctx context.Context, b *proofbound.ReplayBudget) error {
			return tessellation.RequireClosedMeshReplay(ctx, tetrahedron(), b)
		},
		func(ctx context.Context, b *proofbound.ReplayBudget) error {
			return tessellation.RequireVertexLinksReplay(ctx, 4, tetrahedron(), b)
		},
	}
	for i, audit := range audits {
		b := proofbound.NewReplayTrialBudget()
		require.NoError(t, audit(t.Context(), b))
		_, records := b.Usage()
		for _, short := range []bool{false, true} {
			m := proofbound.NewReplayTrialBudget()
			used := proofbound.ReplayTrialRecords - records
			if short {
				used++
			}
			require.NoError(t, m.ReserveRecords(t.Context(), used))
			err := audit(t.Context(), m)
			if short && !errors.Is(err, decaderr.ErrUnsupported) || !short && err != nil {
				t.Fatalf("audit=%d short=%t error=%v", i, short, err)
			}
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		if err := audit(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
			t.Fatalf("audit=%d cancel=%v", i, err)
		}
	}
}
