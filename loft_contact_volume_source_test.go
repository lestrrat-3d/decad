package decad

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestLoftVolumeSourceEqualityRealAndRefusals(t *testing.T) {
	for _, tc := range []struct {
		name         string
		lower, upper [4]float64
	}{
		{"right_x", [4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}},
		{"upper_y", [4]float64{4, 8, 6, 12}, [4]float64{4, 8, 6, 13}},
		{"left_x", [4]float64{-2, 4, 2, 6}, [4]float64{-3, 4, 2, 6}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			prism, loft := loftPartitionRealBodies(t, tc.lower, tc.upper, 0, 4)
			pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
			for order, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
				name := "prism_first"
				if order == 1 {
					name = "loft_first"
				}
				t.Run(name, func(t *testing.T) {
					operation := proofbound.NewWorkBudget(t.Context())
					source, admitted, err := certifyLoftSourcePartition(t.Context(), pair[0], pair[1], operation)
					require.NoError(t, err)
					require.True(t, admitted)
					proof, admitted, err := certifyLoftOutputBoundary(t.Context(), pair[0], pair[1], operation)
					require.NoError(t, err)
					require.True(t, admitted)
					const outputWorkLimit uint64 = 65_536
					require.Less(t, proof.workVisits, outputWorkLimit)
					steps := 0
					counted := &proofbound.WorkBudget{
						StepFn: func() error { steps++; return operation.Step() },
						ErrFn:  operation.Err,
					}
					valid, err := loftVolumeSourceEquality(t.Context(),
						proofbound.NewWallWorkBudgetWithOperation(outputWorkLimit-proof.workVisits, counted),
						pp, lp, source, proof.shell)
					require.NoError(t, err)
					require.True(t, valid)
					require.LessOrEqual(t, proof.workVisits+uint64(steps), outputWorkLimit)
					t.Logf("output work visits=%d source-equality steps=%d", proof.workVisits, steps)
					if tc.name == "right_x" && order == 0 {
						loftVolumeSourceAdversarialCases(t, pp, lp, source, proof.shell)
					}
				})
			}
		})
	}
}

func loftVolumeSourceAdversarialCases(t *testing.T, pp prismPayload, lp loftPayload,
	source loftSourcePartition, original loftOutputShell) {
	t.Helper()
	clone := func() loftOutputShell {
		s := original
		s.vertices = append([]loftOutputVertex(nil), original.vertices...)
		s.triangles = append([][3]int(nil), original.triangles...)
		s.faces = append([]loftOutputFace(nil), original.faces...)
		s.triangleBounds = append([]float64(nil), original.triangleBounds...)
		return s
	}
	remove := func(s *loftOutputShell, kind int) {
		for i, face := range s.faces {
			if face.kind != kind {
				continue
			}
			s.triangles = append(s.triangles[:i:i], s.triangles[i+1:]...)
			s.faces = append(s.faces[:i:i], s.faces[i+1:]...)
			s.triangleBounds = append(s.triangleBounds[:i:i], s.triangleBounds[i+1:]...)
			return
		}
	}
	for _, tc := range []struct {
		name   string
		change func(*loftOutputShell)
	}{
		{"missing_cap", func(s *loftOutputShell) { remove(s, 0) }},
		{"missing_side", func(s *loftOutputShell) { remove(s, 1) }},
		{"doubled_patch", func(s *loftOutputShell) {
			s.triangles = append(s.triangles, s.triangles[0])
			s.faces = append(s.faces, s.faces[0])
			s.triangleBounds = append(s.triangleBounds, s.triangleBounds[0])
		}},
		{"internal_owner", func(s *loftOutputShell) { s.faces[0].owner = 2 }},
		{"reversed_orientation", func(s *loftOutputShell) {
			s.triangles[0][1], s.triangles[0][2] = s.triangles[0][2], s.triangles[0][1]
		}},
		{"changed_exact_vertex", func(s *loftOutputShell) {
			s.vertices[0].point.u = new(big.Rat).Add(s.vertices[0].point.u, big.NewRat(1, 1))
		}},
		{"changed_world_vertex", func(s *loftOutputShell) {
			s.vertices[0].world[0] = new(big.Rat).Add(s.vertices[0].world[0], big.NewRat(1, 1))
		}},
		{"oversized_rational", func(s *loftOutputShell) {
			s.vertices[0].world[0] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8193))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			shell := clone()
			tc.change(&shell)
			valid, err := loftVolumeSourceEquality(t.Context(),
				proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
				pp, lp, source, shell)
			require.NoError(t, err)
			require.False(t, valid)
		})
	}
	t.Run("wrong_partition_wall", func(t *testing.T) {
		bad := source
		bad.walls = append([]int8(nil), source.walls...)
		bad.walls[0] = 2
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("wrong_cap_level", func(t *testing.T) {
		bad := source
		bad.capLevels[1] = big.NewRat(3, 1)
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("oversized_cap_level", func(t *testing.T) {
		bad := source
		bad.capLevels[1] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8193))
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("cap_comparison_product_limit", func(t *testing.T) {
		bad := source
		bad.capLevels[1] = new(big.Rat).SetFrac(
			big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8190))
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("pairwise_comparison_bounds", func(t *testing.T) {
		comparison, bounded := loftVolumeSourceCmp(big.NewRat(3, 2), big.NewRat(6, 4))
		require.True(t, bounded)
		require.Zero(t, comparison)

		large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8190))
		tiny := new(big.Rat).SetFrac(
			big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8190))
		require.True(t, loftVolumeSourceRatOK(large))
		require.True(t, loftVolumeSourceRatOK(tiny))
		_, bounded = loftVolumeSourceCmp(large, big.NewRat(1, 4))
		require.False(t, bounded)
		_, bounded = loftVolumeSourceCmp(big.NewRat(4, 1), tiny)
		require.False(t, bounded)
		_, bounded = loftVolumeSourceCmp(tiny, big.NewRat(4, 1))
		require.False(t, bounded)

		atLimit := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8188))
		comparison, bounded = loftVolumeSourceCmp(atLimit, big.NewRat(1, 4))
		require.True(t, bounded)
		require.Positive(t, comparison)
		comparison, bounded = loftVolumeSourceCmp(big.NewRat(1, 4), atLimit)
		require.True(t, bounded)
		require.Negative(t, comparison)
		atDenominatorLimit := new(big.Rat).SetFrac(
			big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8188))
		comparison, bounded = loftVolumeSourceCmp(big.NewRat(4, 1), atDenominatorLimit)
		require.True(t, bounded)
		require.Positive(t, comparison)
		_, bounded = loftVolumeSourceCmp(nil, big.NewRat(1, 1))
		require.False(t, bounded)
	})
	t.Run("wrong_cap_owner", func(t *testing.T) {
		bad := source
		bad.capOwners[0][0]++
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("lost_seam", func(t *testing.T) {
		bad := source
		bad.seams[0] = loftcontact.LineSeam{}
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("shared_work_limit", func(t *testing.T) {
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context())),
			pp, lp, source, original)
		require.False(t, valid)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	})
	t.Run("canceled_during_source_check", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		defer cancel()
		steps := 0
		operation := &proofbound.WorkBudget{
			StepFn: func() error {
				steps++
				if steps == 50 {
					cancel()
				}
				return ctx.Err()
			},
			ErrFn: ctx.Err,
		}
		valid, err := loftVolumeSourceEquality(ctx,
			proofbound.NewWallWorkBudgetWithOperation(65_536, operation),
			pp, lp, source, original)
		require.False(t, valid)
		require.True(t, errors.Is(err, context.Canceled))
		require.Equal(t, 50, steps)
	})
}
