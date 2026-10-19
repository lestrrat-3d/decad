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

func quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, carrier, square *big.Rat) bool {
	if !quarterRootComparisonFits(xlo, xhi) ||
		!quarterRootComparisonFits(xlo, big.NewRat(2, 1)) ||
		!quarterRootComparisonFits(xhi, big.NewRat(6, 1)) ||
		!quarterRootComparisonFits(ylo, carrier) ||
		!quarterRootComparisonFits(yhi, carrier) || square == nil {
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

func quarterRootBracketFits(xlo, xhi, ylo, yhi, square *big.Rat) bool {
	return quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, big.NewRat(1, 1), square)
}

// loftQuarterSourceHeld is an inactive private proof of one selected seam.
// It does not certify the paired seam, complete shell, volume, or public Union.
func loftQuarterSourceHeld(ctx context.Context, first, second *Body, carrierY int,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldSeam, bool, error) {
	var no loftcontact.QuarterSourceHeldSeam
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if (carrierY != 1 && carrierY != 3) || sourceOperation == nil || outputOperation == nil {
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
		wall.lower < 0 || wall.upper < 0 || wall.arcSegment < 0 ||
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
	selectedLine := wall.lower
	wantSquares := [2]*big.Rat{big.NewRat(24, 1), big.NewRat(117, 4)}
	if carrierY == 3 {
		selectedLine = wall.upper
		wantSquares = [2]*big.Rat{big.NewRat(16, 1), big.NewRat(85, 4)}
	}
	for capIndex, wantQ := range wantSquares {
		found := false
		for _, event := range envelope.order.quarter[capIndex] {
			if err := sourceBudget.Step(); err != nil {
				return no, false, err
			}
			if event.line.segment != selectedLine {
				continue
			}
			if found || event.line.body != wall.prism ||
				event.arc.body != wall.loft || event.arc.segment != wall.arcSegment ||
				event.appearances == 0 {
				return no, false, nil
			}
			found = true
			line, isLine := event.line.curve.(sectionrecord.LineSeg)
			if !isLine || line.Start.V != float64(carrierY) || line.End.V != float64(carrierY) ||
				!((line.Start.U == 2 && line.End.U == 6) ||
					(line.Start.U == 6 && line.End.U == 2)) {
				return no, false, nil
			}
			x, y := event.witness.Point()
			xlo, xhi := x.Bounds()
			ylo, yhi := y.Bounds()
			carrier := big.NewRat(int64(carrierY), 1)
			if !quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, carrier, wantQ) {
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
			if xlo.Cmp(big.NewRat(2, 1)) <= 0 || xhi.Cmp(big.NewRat(6, 1)) >= 0 ||
				ylo.Cmp(carrier) > 0 || yhi.Cmp(carrier) < 0 ||
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
	var result loftcontact.QuarterSourceHeldSeam
	if carrierY == 1 {
		result, accepted, err = loftcontact.CertifyQuarterSourceHeldY1(ctx,
			outputBudget, &wall.prism.n, &wall.loft.n, wall.arcSegment, wall.radius)
	} else {
		result, accepted, err = loftcontact.CertifyQuarterSourceHeldY3(ctx,
			outputBudget, &wall.prism.n, &wall.loft.n, wall.arcSegment, wall.radius)
	}
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

func loftQuarterSourceHeldY1(ctx context.Context, first, second *Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY1, bool, error) {
	return loftQuarterSourceHeld(ctx, first, second, 1, sourceOperation, outputOperation)
}

func loftQuarterSourceHeldY3(ctx context.Context, first, second *Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY3, bool, error) {
	return loftQuarterSourceHeld(ctx, first, second, 3, sourceOperation, outputOperation)
}
