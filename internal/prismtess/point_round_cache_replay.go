package prismtess

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// pointRoundReplayCache stores exact composed frame coefficients. The cache is
// private to a single source and replay meter.
type pointRoundReplayCache struct {
	meter       *proofbound.ReplayBudget
	frame       r3.Frame
	xform       r3.Transform
	coefficient [4][3]proof.Dyadic
	expected    [4][3]proof.Dyadic
}

func newPointRoundReplayCache(ctx context.Context, p prismrecord.Payload, b *proofbound.ReplayBudget) (*pointRoundReplayCache, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 256); err != nil {
		return nil, err
	}
	basis := p.Xform.Basis()
	vectors := [...]r3.Vec{p.Frame.Origin(), p.Frame.U(), p.Frame.V(), p.Frame.N(), basis.EX, basis.EY, basis.EZ, p.Xform.Translation()}
	for _, v := range vectors {
		if !proofbound.FiniteVec(v) {
			return nil, decaderr.ErrUnsupported
		}
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(pointRoundReplayCache{}))); err != nil {
		return nil, err
	}
	c := &pointRoundReplayCache{meter: b, frame: p.Frame, xform: p.Xform}
	var d [8][3]proof.Dyadic
	for i, v := range vectors {
		d[i] = [3]proof.Dyadic{proof.DyOfFinite(v.X), proof.DyOfFinite(v.Y), proof.DyOfFinite(v.Z)}
	}
	for j := range 4 {
		for i := range 3 {
			a, err := proof.ReplayDyadicArithmetic(ctx, d[4][i], d[j][0], '*', b)
			if err != nil {
				return nil, err
			}
			x, err := proof.ReplayDyadicArithmetic(ctx, d[5][i], d[j][1], '*', b)
			if err != nil {
				return nil, err
			}
			y, err := proof.ReplayDyadicArithmetic(ctx, d[6][i], d[j][2], '*', b)
			if err != nil {
				return nil, err
			}
			a, err = proof.ReplayDyadicArithmetic(ctx, a, x, '+', b)
			if err != nil {
				return nil, err
			}
			a, err = proof.ReplayDyadicArithmetic(ctx, a, y, '+', b)
			if err != nil {
				return nil, err
			}
			if j == 0 {
				a, err = proof.ReplayDyadicArithmetic(ctx, a, d[7][i], '+', b)
				if err != nil {
					return nil, err
				}
			}
			c.coefficient[j][i] = a
		}
	}
	c.expected = c.coefficient
	return c, nil
}

func (c *pointRoundReplayCache) bound(ctx context.Context, p prismrecord.Payload, u, v, z float64, held r3.Vec, b *proofbound.ReplayBudget) (float64, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if c == nil || b == nil || c.meter != b || c.frame != p.Frame || c.xform != p.Xform {
		return 0, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 128); err != nil {
		return 0, err
	}
	// Dyadic values are immutable outside package proof. Comparing their
	// private representations detects replacement of a cached coefficient.
	if c.coefficient != c.expected {
		return 0, decaderr.ErrUnsupported
	}
	if !proofbound.FiniteVec(held) || proofbound.IsNonFinite(u) || proofbound.IsNonFinite(v) || proofbound.IsNonFinite(z) {
		return 0, decaderr.ErrUnsupported
	}
	scalar := [3]proof.Dyadic{proof.DyOfFinite(u), proof.DyOfFinite(v), proof.DyOfFinite(z)}
	heldValues := [3]float64{held.X, held.Y, held.Z}
	gap := 0.0
	for i := range 3 {
		value := c.coefficient[0][i]
		for j := range 3 {
			term, err := proof.ReplayDyadicArithmetic(ctx, c.coefficient[j+1][i], scalar[j], '*', b)
			if err != nil {
				return 0, err
			}
			value, err = proof.ReplayDyadicArithmetic(ctx, value, term, '+', b)
			if err != nil {
				return 0, err
			}
		}
		delta, err := proof.ReplayDyadicFloatError(ctx, value, heldValues[i], b)
		if err != nil {
			return 0, err
		}
		gap = max(gap, delta)
	}
	return proofbound.Radius3D(gap), nil
}
