package circularbounds

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// BoreRing samples an ordered circular bore boundary in its
// local plane. Its radial bound covers station rounding and every chord's
// departure from the exact circle. Callers prove their own cap and side
// clearance, then lift these points into their respective section frames.
func BoreRing(ctx context.Context, radius float64, angles []float64) ([]sectionrecord.Point2, float64, error) {
	return BoreRingAtStep(ctx, radius, angles, 0.025)
}

// BoreRingAtStep permits a caller with its own certified
// angular plan to supply that plan's largest step. The returned sag bound
// still uses the actual upper bound of every station interval.
func BoreRingAtStep(ctx context.Context, radius float64,
	angles []float64, maxAllowedStep float64) ([]sectionrecord.Point2, float64, error) {
	if len(angles) < 3 || radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, 0, fmt.Errorf("%w: circular bore ring has no positive radius or complete stations", decaderr.ErrUnsupported)
	}
	if maxAllowedStep <= 0 || maxAllowedStep > math.Pi/2 || proofbound.IsNonFinite(maxAllowedStep) {
		return nil, 0, fmt.Errorf("%w: circular bore ring has no bounded station step", decaderr.ErrUnsupported)
	}
	points := make([]sectionrecord.Point2, len(angles))
	ratRadius := proofarith.FloatRat(radius)
	maxRound, maxStep := 0.0, 0.0
	for i, angle := range angles {
		if err := ctx.Err(); err != nil {
			return nil, 0, err
		}
		if proofbound.IsNonFinite(angle) {
			return nil, 0, fmt.Errorf("%w: circular bore ring angle is not finite", decaderr.ErrUnsupported)
		}
		ratAngle := proofarith.FloatRat(angle)
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(ratAngle)
		if !ok {
			return nil, 0, fmt.Errorf("%w: circular bore ring angle has no trigonometric enclosure", decaderr.ErrUnsupported)
		}
		points[i] = sectionrecord.Point2{U: radius * math.Cos(angle), V: radius * math.Sin(angle)}
		uErr := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv, ratRadius), points[i].U)
		vErr := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv, ratRadius), points[i].V)
		maxRound = max(maxRound, proofbound.Radius3D(max(uErr, vErr)))

		var step *big.Rat
		if i+1 == len(angles) {
			step = new(big.Rat).Add(proofbound.TwoPiInterval().Hi,
				new(big.Rat).Sub(proofarith.FloatRat(angles[0]), ratAngle))
		} else {
			if proofbound.IsNonFinite(angles[i+1]) {
				return nil, 0, fmt.Errorf("%w: circular bore ring angle is not finite", decaderr.ErrUnsupported)
			}
			step = new(big.Rat).Sub(proofarith.FloatRat(angles[i+1]), ratAngle)
		}
		stepUp := proofbound.RatFloatUp(step)
		if step.Sign() <= 0 || proofbound.IsNonFinite(stepUp) || stepUp > maxAllowedStep {
			return nil, 0, fmt.Errorf("%w: circular bore ring stations are unordered", decaderr.ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
	}
	sag := proofbound.ProductUpper(radius, proofbound.ProductUpper(maxStep, maxStep)/8)
	return points, proofbound.AbsSumUpper(sag, maxRound), nil
}
