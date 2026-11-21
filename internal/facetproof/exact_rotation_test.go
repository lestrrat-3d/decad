package facetproof_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExactRotationFoldRetry(t *testing.T) {
	rotation, d, err := exactrotation.New(r3.Vec{X: 1, Y: 2}, r3.Vec{Z: 1}, units.Radians(0.7))
	require.NoError(t, err)
	linear, translation, ok := rotation.Gap(d)
	require.True(t, ok)
	// A triangular prism has three nearly coincident corners at each end. Its
	// position relative to the output float lattice decides whether the three
	// denoted positions can be rounded to distinct, embedded corners.
	const width = 0x1p-56
	tris := [][3]int{{0, 2, 1}, {3, 4, 5}, {0, 1, 4}, {0, 4, 3}, {1, 2, 5}, {1, 5, 4}, {2, 0, 3}, {2, 3, 5}}
	for _, tc := range []struct {
		name     string
		y        float64
		embedded bool
	}{
		{"exact corners clear", -3 * 0x1p-55, true},
		{"exact image inverts", -7 * 0x1p-55, false},
		{"both images fold", -8 * 0x1p-55, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			y := tc.y
			verts := []r3.Vec{
				{Y: y}, {X: width, Y: y}, {Y: y + width},
				{Y: y, Z: 1}, {X: width, Y: y, Z: 1}, {Y: y + width, Z: 1},
			}
			beta := make([]float64, len(verts))
			ctx := t.Context()
			_, err := facetproof.PlaceMeshDenoting(ctx, proofbound.NewWorkBudget(ctx), verts, tris, beta, 0, 0, d, linear, translation)
			var fold *meshbool.HeldFoldError
			require.ErrorAs(t, err, &fold)
			placed, err := facetproof.PlaceRotatedMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, beta, 0, 0, d, rotation)
			if !tc.embedded {
				require.ErrorAs(t, err, &fold)
				return
			}
			require.NoError(t, err)
			require.Nil(t, placed.EmbeddingReach)
			_, err = facetproof.AuditFacetedMesh(ctx, placed.Verts, placed.Tris)
			require.NoError(t, err)
			moved := make([]bool, len(verts))
			exact := make([]proof.Xpt, len(verts))
			for i, v := range verts {
				moved[i] = true
				p := rotation.Image(v)
				exact[i] = p
				x, y, z := proof.XhpRat(proof.Xhp(p))
				var sq big.Rat
				held := placed.Verts[i]
				components := [3]float64{held.X, held.Y, held.Z}
				for j, q := range []*big.Rat{x, y, z} {
					component := components[j]
					r := new(big.Rat).Sub(q, proof.FloatRat(component))
					sq.Add(&sq, r.Mul(r, r))
				}
				b := proof.FloatRat(placed.VertexBound[i])
				require.GreaterOrEqual(t, b.Mul(b, b).Cmp(&sq), 0)
			}
			folds, err := meshbool.HeldFolds(ctx, meshbool.HeldRounding{
				Verts: placed.Verts, Tris: placed.Tris, Moved: moved, Movable: make([]bool, len(verts)), Exact: exact,
			})
			require.NoError(t, err)
			require.Empty(t, folds)
			canceled, cancel := context.WithCancel(ctx)
			cancel()
			_, err = facetproof.PlaceRotatedMesh(canceled, proofbound.NewWorkBudget(canceled), verts, tris, beta, 0, 0, d, rotation)
			require.ErrorIs(t, err, context.Canceled)
		})
	}
}
