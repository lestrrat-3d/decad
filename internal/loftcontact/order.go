package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type cubic [4]*big.Rat

// A constructor-produced seam has dyadic coefficients. Limiting each part
// to 512 bits bounds every raw integer product and sum in the endpoint
// comparison and cubic Bernstein test below 6000 bits. Check before those
// operations: a finite float64 alone does not bound their cubic products
// below the source proof's 8192-bit ceiling.
const orderedSeamMaxPartBits = 512

func lineSeamOrderingBound(budget *proofbound.WorkBudget, seam LineSeam) (bool, error) {
	coefficients := [9]*big.Rat{
		seam.lineNum[0], seam.lineNum[1], seam.lineDen[0], seam.lineDen[1],
		seam.carrierNum[0], seam.carrierNum[1], seam.carrierNum[2],
		seam.carrierDen[0], seam.carrierDen[1],
	}
	for _, value := range coefficients {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if value == nil {
			return false, nil
		}
		denBits := value.Denom().BitLen()
		if value.Num().BitLen() > orderedSeamMaxPartBits || denBits > orderedSeamMaxPartBits ||
			denBits == 0 || value.Denom().TrailingZeroBits() != uint(denBits-1) {
			return false, nil
		}
	}
	return true, nil
}

// CertifyOrderedLineSeams proves that two transverse wall contacts stay
// strictly ordered on the same straight carrier at every axial height.
// The caller must authenticate both paired Loft lines and their common frame.
// Refusal leaves all source geometry untouched.
func CertifyOrderedLineSeams(ctx context.Context, budget *proofbound.WorkBudget,
	bottomA, topA, bottomB, topB [2]sectionrecord.Point2,
	carrier [2]sectionrecord.Point2) ([2]LineSeam, bool, error) {
	var seams [2]LineSeam
	var ok bool
	var err error
	seams[0], ok, err = CertifyLineSeam(ctx, budget, bottomA, topA, carrier)
	if err != nil || !ok {
		return [2]LineSeam{}, false, err
	}
	if ok, err = lineSeamOrderingBound(budget, seams[0]); err != nil || !ok {
		return [2]LineSeam{}, false, err
	}
	seams[1], ok, err = CertifyLineSeam(ctx, budget, bottomB, topB, carrier)
	if err != nil || !ok {
		return [2]LineSeam{}, false, err
	}
	if ok, err = lineSeamOrderingBound(budget, seams[1]); err != nil || !ok {
		return [2]LineSeam{}, false, err
	}
	if err := budget.Step(); err != nil {
		return [2]LineSeam{}, false, err
	}
	// vB-vA = (nB*dA-nA*dB)/(dA*dB). Each denominator keeps its
	// endpoint sign because the individual seam certificates passed.
	product := subCubic(
		mulQuadraticLinear(seams[1].carrierNum, seams[0].carrierDen),
		mulQuadraticLinear(seams[0].carrierNum, seams[1].carrierDen),
	)
	sign := seams[0].carrierDen[0].Sign() * seams[1].carrierDen[0].Sign()
	if !sameSignCubicBernstein(product, sign) {
		return [2]LineSeam{}, false, nil
	}
	if err := ctx.Err(); err != nil {
		return [2]LineSeam{}, false, err
	}
	return seams, true, nil
}

func mulQuadraticLinear(a quadratic, b linear) cubic {
	return cubic{
		new(big.Rat).Mul(a[0], b[0]),
		add(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
		add(new(big.Rat).Mul(a[1], b[1]), new(big.Rat).Mul(a[2], b[0])),
		new(big.Rat).Mul(a[2], b[1]),
	}
}

func subCubic(a, b cubic) cubic {
	var result cubic
	for i := range result {
		result[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return result
}

// Positive Bernstein coefficients are sufficient for one cubic to keep
// a strict sign over the whole interval, including both end sections.
func sameSignCubicBernstein(a cubic, sign int) bool {
	oneThird := big.NewRat(1, 3)
	twoThirds := big.NewRat(2, 3)
	b1 := add(a[0], new(big.Rat).Mul(a[1], oneThird))
	b2 := add(add(a[0], new(big.Rat).Mul(a[1], twoThirds)),
		new(big.Rat).Mul(a[2], oneThird))
	b3 := add(add(a[0], a[1]), add(a[2], a[3]))
	return a[0].Sign() == sign && b1.Sign() == sign &&
		b2.Sign() == sign && b3.Sign() == sign
}
