package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func quarterRootComparisonFits(a, b *big.Rat) bool {
	return a != nil && b != nil &&
		a.Num().BitLen() <= 8192 && a.Denom().BitLen() <= 8192 &&
		b.Num().BitLen() <= 8192 && b.Denom().BitLen() <= 8192 &&
		a.Num().BitLen()+b.Denom().BitLen() <= 8192 &&
		b.Num().BitLen()+a.Denom().BitLen() <= 8192
}

func quarterRootBracketFits(xlo, xhi, ylo, yhi, square *big.Rat) bool {
	if !quarterRootComparisonFits(xlo, xhi) ||
		!quarterRootComparisonFits(xlo, big.NewRat(2, 1)) ||
		!quarterRootComparisonFits(xhi, big.NewRat(6, 1)) ||
		!quarterRootComparisonFits(ylo, big.NewRat(1, 1)) ||
		!quarterRootComparisonFits(yhi, big.NewRat(1, 1)) || square == nil {
		return false
	}
	for _, x := range []*big.Rat{xlo, xhi} {
		if 2*x.Num().BitLen() > 8192 || 2*x.Denom().BitLen() > 8192 ||
			2*x.Num().BitLen()+square.Denom().BitLen() > 8192 ||
			square.Num().BitLen()+2*x.Denom().BitLen() > 8192 {
			return false
		}
	}
	return true
}

// loftQuarterSourceHeldY1 is an inactive private proof of one selected seam.
// It does not certify the y=3 seam, complete shell, volume, or public Union.
func loftQuarterSourceHeldY1(ctx context.Context, first, second *Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY1, bool, error) {
	var no loftcontact.QuarterSourceHeldY1
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if sourceOperation == nil || outputOperation == nil {
		return no, false, nil
	}
	withContext := func(operation *proofbound.WorkBudget) *proofbound.WorkBudget {
		return &proofbound.WorkBudget{
			StepFn: func() error {
				if err := ctx.Err(); err != nil {
					return err
				}
				return operation.Step()
			},
			ErrFn: func() error {
				if err := ctx.Err(); err != nil {
					return err
				}
				return operation.Err()
			},
		}
	}
	sourceBudget := proofbound.NewWallWorkBudgetWithOperation(4096, withContext(sourceOperation))
	outputBudget := proofbound.NewWallWorkBudgetWithOperation(65_536, withContext(outputOperation))
	envelope, accepted, err := certifyLoftQuarterSourceEnvelope(
		ctx, first, second, sourceBudget)
	if err != nil || !accepted {
		return no, false, err
	}
	wall := envelope.wall
	if wall.prism == nil || wall.loft == nil || wall.prism == wall.loft ||
		wall.lower < 0 || wall.arcSegment < 0 ||
		len(envelope.order.quarter[0]) != 2 ||
		len(envelope.order.quarter[1]) != 2 ||
		envelope.order.source.levels[0] == nil ||
		envelope.order.source.levels[1] == nil ||
		envelope.order.source.levels[2] == nil {
		return no, false, nil
	}
	for i, want := range [3]*big.Rat{big.NewRat(0, 1), big.NewRat(10, 1), big.NewRat(12, 1)} {
		if err := sourceBudget.Step(); err != nil {
			return no, false, err
		}
		if envelope.order.source.levels[i].Cmp(want) != 0 {
			return no, false, nil
		}
	}
	for capIndex, wantQ := range [2]*big.Rat{big.NewRat(24, 1), big.NewRat(117, 4)} {
		found := false
		for _, event := range envelope.order.quarter[capIndex] {
			if err := sourceBudget.Step(); err != nil {
				return no, false, err
			}
			if event.line.segment != wall.lower {
				continue
			}
			if found || event.line.body != wall.prism ||
				event.arc.body != wall.loft || event.arc.segment != wall.arcSegment ||
				event.appearances == 0 {
				return no, false, nil
			}
			found = true
			line, isLine := event.line.curve.(sectionrecord.LineSeg)
			if !isLine || line.Start.V != 1 || line.End.V != 1 ||
				!((line.Start.U == 2 && line.End.U == 6) ||
					(line.Start.U == 6 && line.End.U == 2)) {
				return no, false, nil
			}
			x, y := event.witness.Point()
			xlo, xhi := x.Bounds()
			ylo, yhi := y.Bounds()
			if !quarterRootBracketFits(xlo, xhi, ylo, yhi, wantQ) {
				return no, false, nil
			}
			if err := sourceBudget.Step(); err != nil {
				return no, false, err
			}
			if xlo.Sign() <= 0 || xlo.Cmp(xhi) > 0 {
				return no, false, nil
			}
			if _, err := sourceBudget.StepN(6); err != nil {
				return no, false, err
			}
			one := big.NewRat(1, 1)
			if xlo.Cmp(big.NewRat(2, 1)) <= 0 || xhi.Cmp(big.NewRat(6, 1)) >= 0 ||
				ylo.Cmp(one) > 0 || yhi.Cmp(one) < 0 ||
				new(big.Rat).Mul(xlo, xlo).Cmp(wantQ) > 0 ||
				new(big.Rat).Mul(xhi, xhi).Cmp(wantQ) < 0 {
				return no, false, nil
			}
		}
		if !found {
			return no, false, nil
		}
	}
	if err := sourceBudget.Err(); err != nil {
		return no, false, err
	}
	result, accepted, err := loftcontact.CertifyQuarterSourceHeldY1(ctx,
		outputBudget, &wall.prism.n, &wall.loft.n, wall.arcSegment, wall.radius)
	if err != nil || !accepted {
		return no, false, err
	}
	if err := sourceBudget.Err(); err != nil {
		return no, false, err
	}
	if err := outputBudget.Err(); err != nil {
		return no, false, err
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	return result, true, nil
}
