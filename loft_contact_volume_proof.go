package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const loftVolumeComposerRatBits = 8192
const loftVolumeOutputSteps = 65_536

// loftVolumeProof is transient. Its bound is meaningful only for the source
// partition and boundary proof certified from the same live operands.
type loftVolumeProof struct {
	boundary          loftOutputBoundaryProof
	sourceVolume      *big.Rat
	symDiffUpper      float64
	totalOutputVisits uint64
}

// loftVolumeCompose uses the source partition carried by its boundary proof.
// The payloads must come from the same live records passed to that proof.
func loftVolumeCompose(ctx context.Context, operation *proofbound.WorkBudget,
	pp prismPayload, lp loftPayload, boundary loftOutputBoundaryProof,
) (loftVolumeProof, bool, error) {
	var no loftVolumeProof
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if operation == nil || boundary.workVisits == 0 ||
		boundary.workVisits >= loftVolumeOutputSteps ||
		len(boundary.shell.triangles) == 0 || len(boundary.shell.triangles) > 512 ||
		len(boundary.shell.vertices) == 0 || len(boundary.shell.vertices) > 512 {
		return no, false, nil
	}
	limit := proofbound.NewWallWorkBudgetWithOperation(
		loftVolumeOutputSteps-boundary.workVisits, operation)
	var volumeVisits uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			if err := limit.Step(); err != nil {
				return err
			}
			volumeVisits++
			return nil
		},
		ErrFn: limit.Err,
	}
	shell := boundary.shell
	if admitted, err := loftVolumeSourceEquality(ctx, budget, pp, lp, boundary.source, shell); err != nil || !admitted {
		return no, false, err
	}
	sourceVolume, admitted, err := loftVolumeExactShell(budget, shell)
	if err != nil || !admitted {
		return no, false, err
	}
	if !loftVolumeRatFits(sourceVolume) || sourceVolume.Sign() <= 0 {
		return no, false, nil
	}
	bound := new(big.Rat)
	for _, tri := range shell.triangles {
		if err := ctx.Err(); err != nil {
			return no, false, err
		}
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		var points [6][3]*big.Rat
		for corner, index := range tri {
			if index < 0 || index >= len(shell.vertices) {
				return no, false, nil
			}
			vertex := shell.vertices[index]
			for axis, coordinate := range vertex.world {
				if !loftVolumeRatFits(coordinate) {
					return no, false, nil
				}
				points[corner][axis] = coordinate
			}
			for axis, coordinate := range [3]float64{vertex.held.X, vertex.held.Y, vertex.held.Z} {
				if proofbound.IsNonFinite(coordinate) {
					return no, false, nil
				}
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				points[corner+3][axis] = proofarith.FloatRat(coordinate)
				if !loftVolumeRatFits(points[corner+3][axis]) {
					return no, false, nil
				}
			}
		}
		term, admitted, err := loftcontact.VolumeHullBound(budget, points)
		if err != nil || !admitted {
			return no, false, err
		}
		if !loftVolumeRatFits(term) || term.Sign() < 0 || !loftVolumeSumFits(bound, term) {
			return no, false, nil
		}
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		bound.Add(bound, term)
		if !loftVolumeRatFits(bound) {
			return no, false, nil
		}
	}
	if err := budget.Step(); err != nil {
		return no, false, err
	}
	upper, rounded := loftVolumeRoundUp(bound)
	if !rounded {
		return no, false, nil
	}
	if proofbound.IsNonFinite(upper) || upper < 0 {
		return no, false, nil
	}
	upperRat := proofarith.FloatRat(upper)
	if !loftVolumeCompareFits(sourceVolume, upperRat) || sourceVolume.Cmp(upperRat) <= 0 {
		return no, false, nil
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	return loftVolumeProof{
		boundary: boundary, sourceVolume: new(big.Rat).Set(sourceVolume),
		symDiffUpper: upper, totalOutputVisits: boundary.workVisits + volumeVisits,
	}, true, nil
}

func loftVolumeRatFits(value *big.Rat) bool {
	return value != nil && value.Denom().Sign() > 0 &&
		value.Num().BitLen() <= loftVolumeComposerRatBits &&
		value.Denom().BitLen() <= loftVolumeComposerRatBits
}

// loftVolumeRoundUp preflights the integer alignment used by RatFloat64 and
// the cross products used by the exact rounding-direction comparison.
func loftVolumeRoundUp(value *big.Rat) (float64, bool) {
	if !loftVolumeRatFits(value) || value.Sign() < 0 {
		return 0, false
	}
	if value.Sign() == 0 {
		return 0, true
	}
	numeratorBits, denominatorBits := value.Num().BitLen(), value.Denom().BitLen()
	if value.Denom().TrailingZeroBits() != uint(denominatorBits-1) {
		shift := 54 - (numeratorBits - denominatorBits)
		if shift > 0 && numeratorBits+shift > loftVolumeComposerRatBits {
			return 0, false
		}
	}
	nearest, _ := proofarith.RatFloat64(value)
	if proofbound.IsNonFinite(nearest) {
		return 0, false
	}
	nearestRat := proofarith.FloatRat(nearest)
	if !loftVolumeCompareFits(nearestRat, value) {
		return 0, false
	}
	if nearestRat.Cmp(value) < 0 {
		nearest = proofarith.NextUp(nearest)
	}
	return nearest, !proofbound.IsNonFinite(nearest)
}

// Adding a/b and c/d can need ad+bc over bd before reduction. Refuse before
// that allocation if either unreduced intermediate could pass the ceiling.
func loftVolumeSumFits(a, b *big.Rat) bool {
	if !loftVolumeRatFits(a) || !loftVolumeRatFits(b) {
		return false
	}
	if a.Sign() == 0 || b.Sign() == 0 {
		return true
	}
	an, ad := a.Num().BitLen(), a.Denom().BitLen()
	bn, bd := b.Num().BitLen(), b.Denom().BitLen()
	return max(an+bd, bn+ad)+1 <= loftVolumeComposerRatBits &&
		ad+bd <= loftVolumeComposerRatBits
}

func loftVolumeCompareFits(a, b *big.Rat) bool {
	return loftVolumeRatFits(a) && loftVolumeRatFits(b) &&
		a.Num().BitLen()+b.Denom().BitLen() <= loftVolumeComposerRatBits &&
		b.Num().BitLen()+a.Denom().BitLen() <= loftVolumeComposerRatBits
}
