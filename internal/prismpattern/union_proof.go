package prismpattern

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// UnionProofBounds charges the circular root seam and its occupied-volume
// annulus after the source walls have passed their clearance checks.
func UnionProofBounds(source *facetedrecord.PrismPatternSource, radius, toothBound, toothVolume,
	connectorReach, rootHeldExcess, maxStep, ringRound float64) (float64, float64, error) {
	ringSag := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(maxStep, maxStep)/8)
	seamBand := proofbound.AbsSumUpper(source.RootGap, connectorReach,
		rootHeldExcess, ringSag, ringRound)
	bound := proofbound.AbsSumUpper(toothBound, seamBand)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return 0, 0, fmt.Errorf("%w: copied-prism join exceeds the 0.1 mm boundary proof: bound %g tooth %g root %g reach %g ring %g",
			decaderr.ErrUnsupported, bound, toothBound, source.RootGap, connectorReach, ringSag)
	}
	if seamBand >= radius {
		return 0, 0, fmt.Errorf("%w: copied-prism join band reaches the axis", decaderr.ErrUnsupported)
	}
	// The symmetric annulus encloses every face changed by removal of the
	// root wall and replacement of the cylinder wall. Its volume is exactly
	// 4*pi*radius*band*height. Each original tooth mesh contributes its own
	// independently verified occupied-volume error once.
	annulusArea := proofbound.ProductUpper(
		proofbound.RatFloatUp(proofbound.PiUpper),
		proofbound.ProductUpper(proofbound.ProductUpper(4, radius), seamBand))
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(source.Blank.Z1), proofarith.FloatRat(source.Blank.Z0)))
	volumeGap := proofbound.AbsSumUpper(toothVolume, proofbound.ProductUpper(annulusArea, height))
	return bound, volumeGap, nil
}
