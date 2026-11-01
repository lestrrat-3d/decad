package tessellation

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ReplayOrientationSign computes the exact signed tetrahedron sum at the
// origin. It does not prove that the input facets form an embedded shell.
func ReplayOrientationSign(ctx context.Context, vertices []r3.Vec, triangles [][3]int, budget *proofbound.ReplayBudget) (int, error) {
	if budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	total, err := proof.ReplayFloatRat(ctx, 0, budget)
	if err != nil {
		return 0, err
	}
	op := func(a, b *big.Rat, kind byte) (*big.Rat, error) {
		return proof.ReplayRatArithmetic(ctx, a, b, kind, budget)
	}
	for i := range triangles {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return 0, err
		}
		tri := triangles[i]
		var v [3][3]*big.Rat
		for j, index := range tri {
			if index < 0 || index >= len(vertices) || !proofbound.FiniteVec(vertices[index]) {
				return 0, decaderr.ErrUnsupported
			}
			p := vertices[index]
			for k, x := range [3]float64{p.X, p.Y, p.Z} {
				v[j][k], err = proof.ReplayFloatRat(ctx, x, budget)
				if err != nil {
					return 0, err
				}
			}
		}
		var det *big.Rat
		for j := range 3 {
			a, err := op(v[1][(j+1)%3], v[2][(j+2)%3], '*')
			if err != nil {
				return 0, err
			}
			b, err := op(v[1][(j+2)%3], v[2][(j+1)%3], '*')
			if err != nil {
				return 0, err
			}
			cross, err := op(a, b, '-')
			if err != nil {
				return 0, err
			}
			term, err := op(v[0][j], cross, '*')
			if err != nil {
				return 0, err
			}
			if det == nil {
				det = term
			} else {
				det, err = op(det, term, '+')
				if err != nil {
					return 0, err
				}
			}
		}
		total, err = op(total, det, '+')
		if err != nil {
			return 0, err
		}
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return total.Sign(), nil
}
