package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// certifiedCircularBoreRing samples an ordered circular bore boundary in its
// local plane. Its radial bound covers station rounding and every chord's
// departure from the exact circle. Callers prove their own cap and side
// clearance, then lift these points into their respective section frames.
func certifiedCircularBoreRing(ctx context.Context, radius float64, angles []float64) ([]Point2, float64, error) {
	if len(angles) < 3 || radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, 0, fmt.Errorf("%w: circular bore ring has no positive radius or complete stations", ErrUnsupported)
	}
	points := make([]Point2, len(angles))
	ratRadius := proofarith.FloatRat(radius)
	maxRound, maxStep := 0.0, 0.0
	for i, angle := range angles {
		if err := ctx.Err(); err != nil {
			return nil, 0, err
		}
		if proofbound.IsNonFinite(angle) {
			return nil, 0, fmt.Errorf("%w: circular bore ring angle is not finite", ErrUnsupported)
		}
		ratAngle := proofarith.FloatRat(angle)
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(ratAngle)
		if !ok {
			return nil, 0, fmt.Errorf("%w: circular bore ring angle has no trigonometric enclosure", ErrUnsupported)
		}
		points[i] = Point2{U: radius * math.Cos(angle), V: radius * math.Sin(angle)}
		uErr := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv, ratRadius), points[i].U)
		vErr := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv, ratRadius), points[i].V)
		maxRound = max(maxRound, proofbound.Radius3D(max(uErr, vErr)))

		var step *big.Rat
		if i+1 == len(angles) {
			step = new(big.Rat).Add(proofbound.TwoPiInterval().Hi,
				new(big.Rat).Sub(proofarith.FloatRat(angles[0]), ratAngle))
		} else {
			if proofbound.IsNonFinite(angles[i+1]) {
				return nil, 0, fmt.Errorf("%w: circular bore ring angle is not finite", ErrUnsupported)
			}
			step = new(big.Rat).Sub(proofarith.FloatRat(angles[i+1]), ratAngle)
		}
		stepUp := proofbound.RatFloatUp(step)
		if step.Sign() <= 0 || proofbound.IsNonFinite(stepUp) || stepUp > 0.025 {
			return nil, 0, fmt.Errorf("%w: circular bore ring stations are unordered", ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
	}
	sag := proofbound.ProductUpper(radius, proofbound.ProductUpper(maxStep, maxStep)/8)
	return points, proofbound.AbsSumUpper(sag, maxRound), nil
}
