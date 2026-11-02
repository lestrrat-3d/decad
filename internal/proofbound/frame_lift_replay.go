package proofbound

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// ExactFrameLiftRoundReplay charges the finite binary64 expression in
// ExactFrameLiftRound before the held point or any exact dyadic is calculated.
// It then uses the ordinary expression so both published values have the same
// bits as an unbounded prism lift.
func ExactFrameLiftRoundReplay(ctx context.Context, frame r3.Frame, xform r3.Transform,
	u, v, z float64, budget *ReplayBudget) (r3.Vec, float64, error) {
	if budget == nil {
		return r3.Vec{}, 0, fmt.Errorf("%w: nil frame-lift budget", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return r3.Vec{}, 0, err
	}
	basis := xform.Basis()
	leaves := [...]r3.Vec{frame.Origin(), frame.U(), frame.V(), frame.N(), basis.EX, basis.EY, basis.EZ, xform.Translation(), {X: u, Y: v, Z: z}}
	minExp, maxTop := int(^uint(0)>>1), -int(^uint(0)>>1)-1
	for _, leaf := range leaves {
		for _, value := range [...]float64{leaf.X, leaf.Y, leaf.Z} {
			if IsNonFinite(value) {
				return r3.Vec{}, 0, fmt.Errorf("%w: nonfinite frame lift", decaderr.ErrUnsupported)
			}
			if value == 0 {
				continue
			}
			bits := math.Float64bits(math.Abs(value))
			field := int((bits >> 52) & 0x7ff)
			exp := -1074
			if field != 0 {
				exp = field - 1023 - 52
			}
			minExp = min(minExp, exp)
			maxTop = max(maxTop, exp+53)
		}
	}
	if minExp > maxTop {
		minExp, maxTop = 0, 1
	}
	// The expression has two affine stages. Every source is a binary64 with
	// at most 53 mantissa bits. Three products and four sums per coordinate
	// cannot span more than three source exponent ranges plus 16 guard bits.
	// This also covers the final subtraction from the held binary64 point.
	width := 3*(maxTop-minExp) + 16
	if width <= 0 || width > 65536 {
		return r3.Vec{}, 0, fmt.Errorf("%w: frame-lift width", decaderr.ErrUnsupported)
	}
	words := uint64((width + 63) / 64)
	if words > 1024 {
		return r3.Vec{}, 0, fmt.Errorf("%w: frame-lift words", decaderr.ErrUnsupported)
	}
	// ExactFrameLiftRound performs 18 products, 21 additions/subtractions
	// and three binary64 conversions. 128 word-pair passes per maximum word
	// pair covers those operations and their normalization/conversion scans.
	// The byte envelope allows 8192 bytes per word across all temporary
	// big.Int and big.Float values in the fixed expression.
	work := 4096 + 128*words*words
	bytes := 4096 + 8192*words
	if err := budget.ReserveBundle(ctx, work, 256, bytes); err != nil {
		return r3.Vec{}, 0, err
	}
	local := frame.ToWorldUV(u, v)
	held := xform.Apply(local.Add(frame.N().Scale(z)))
	if !FiniteVec(held) {
		return r3.Vec{}, 0, fmt.Errorf("%w: nonfinite held frame lift", decaderr.ErrUnsupported)
	}
	bound := ExactFrameLiftRound(frame, xform, u, v, z, held)
	if IsNonFinite(bound) {
		return r3.Vec{}, 0, fmt.Errorf("%w: unbounded frame lift", decaderr.ErrUnsupported)
	}
	return held, bound, nil
}
