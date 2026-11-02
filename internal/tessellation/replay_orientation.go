package tessellation

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ClosedOrientationSignReplay reads the exact signed volume of a mesh whose
// closure was already proved. Closure makes this sum independent of anchor.
func ClosedOrientationSignReplay(ctx context.Context, vertices []r3.Vec, triangles [][3]int, b *proofbound.ReplayBudget) (int, error) {
	if b == nil {
		return 0, fmt.Errorf("%w: bounded orientation needs a budget", decaderr.ErrUnsupported)
	}
	var total proof.Dyadic
	for _, tri := range triangles {
		if err := b.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		var p [3]proof.DyV3
		for k, i := range tri {
			if i < 0 || i >= len(vertices) || !proofbound.FiniteVec(vertices[i]) {
				return 0, fmt.Errorf("%w: invalid orientation vertex", decaderr.ErrUnsupported)
			}
			if err := b.SpendWork(ctx, 3); err != nil {
				return 0, err
			}
			p[k] = proof.DyVec(vertices[i])
		}
		var term proof.Dyadic
		for axis := range 3 {
			j, k := (axis+1)%3, (axis+2)%3
			x, err := proof.ReplayDyadicArithmetic(ctx, p[1][j], p[2][k], '*', b)
			if err != nil {
				return 0, err
			}
			y, err := proof.ReplayDyadicArithmetic(ctx, p[1][k], p[2][j], '*', b)
			if err != nil {
				return 0, err
			}
			cross, err := proof.ReplayDyadicArithmetic(ctx, x, y, '-', b)
			if err != nil {
				return 0, err
			}
			product, err := proof.ReplayDyadicArithmetic(ctx, p[0][axis], cross, '*', b)
			if err != nil {
				return 0, err
			}
			term, err = proof.ReplayDyadicArithmetic(ctx, term, product, '+', b)
			if err != nil {
				return 0, err
			}
		}
		var err error
		total, err = proof.ReplayDyadicArithmetic(ctx, total, term, '+', b)
		if err != nil {
			return 0, err
		}
	}
	return total.Sign(), nil
}
