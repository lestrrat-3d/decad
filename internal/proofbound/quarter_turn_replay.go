package proofbound

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

var replayTurnFour = big.NewRat(4, 1)
var replayQuarterFour = big.NewInt(4)
var replayTurnMinusOne = big.NewRat(-1, 1)

// ReplayQuarterTurnSinCos returns exact point intervals at quarter turns and
// the charged turn-series enclosure at other exact rational turns.
func ReplayQuarterTurnSinCos(ctx context.Context, t *big.Rat,
	budget *ReplayBudget) (RatInterval, RatInterval, error) {
	quadrants, err := proofarith.ReplayRatArithmetic(ctx, t, replayTurnFour, '*', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if !quadrants.IsInt() {
		return ReplayTurnSinCosInterval(ctx, t, budget)
	}
	index, err := replayFixedIntOp(ctx, quadrants.Num(), replayQuarterFour, '%', 0, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if index.Sign() < 0 {
		index, err = replayFixedIntOp(ctx, index, replayQuarterFour, '+', 0, budget)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
	}
	var sinValue, cosValue *big.Rat
	switch index.Int64() {
	case 0:
		sinValue, cosValue = replayTurnZero, replayAtanOne
	case 1:
		sinValue, cosValue = replayAtanOne, replayTurnZero
	case 2:
		sinValue, cosValue = replayTurnZero, replayTurnMinusOne
	default:
		sinValue, cosValue = replayTurnMinusOne, replayTurnZero
	}
	sin, err := replayIntervalCopy(ctx, RatInterval{Lo: sinValue, Hi: sinValue}, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cos, err := replayIntervalCopy(ctx, RatInterval{Lo: cosValue, Hi: cosValue}, budget)
	return sin, cos, err
}
