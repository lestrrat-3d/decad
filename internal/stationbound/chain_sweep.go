package stationbound

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChainSweepArcStation encloses its angular station using exact rational
// turns. The held midpoint is merely an output coordinate; interval error
// bounds its distance to the true circular station.
func ChainSweepArcStation(height, radius, radial float64, j, n int) (r3.Vec, float64, error) {
	sin, cos := proofbound.TurnSinCosInterval(big.NewRat(int64(j), int64(2*n)))
	r := proofarith.FloatRat(radial)
	x := proofbound.IntervalSub(proofbound.PointInterval(proofarith.FloatRat(radius)),
		proofbound.IntervalScale(cos, r))
	z := proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(height)),
		proofbound.IntervalScale(sin, r))
	mid := func(iv proofbound.RatInterval) float64 {
		value, _ := new(big.Rat).Quo(new(big.Rat).Add(iv.Lo, iv.Hi), big.NewRat(2, 1)).Float64()
		return value
	}
	p := r3.Vec{X: mid(x), Z: mid(z)}
	if !proofbound.FiniteVec(p) {
		return r3.Vec{}, 0, fmt.Errorf(`%w: the chain sweep arc station is not finite`, decaderr.ErrUnsupported)
	}
	gap := proofbound.AbsSumUpper(proofbound.IntervalFloatError(x, p.X),
		proofbound.IntervalFloatError(z, p.Z))
	if proofbound.IsNonFinite(gap) {
		return r3.Vec{}, 0, fmt.Errorf(`%w: the chain sweep arc station has no finite coordinate bound`, decaderr.ErrUnsupported)
	}
	return p, gap, nil
}
