package diameter

import (
	"context"
	"iter"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/r3"
)

// HeldPoint is a boundary point a builder holds beside the proven distance
// between it and the point the body denotes.
type HeldPoint struct {
	Value r3.Vec
	Bound float64
}

// HeldPoints reads a lower diameter bound from boundary points a builder
// holds. extraAllow covers section displacement and any endpoint rounding
// the points' own bounds do not charge. It reports ok false when a point is
// not finite or carries no usable bound. Cancellation of ctx is returned as
// an error.
func HeldPoints(ctx context.Context, held iter.Seq[HeldPoint], extraAllow float64) (float64, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	var points []r3.Vec
	maxBound := 0.0
	for p := range held {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		if !proofbound.FiniteVec(p.Value) || !tolerance.UsableMagnitude(p.Bound) {
			return 0, false, nil
		}
		points = append(points, p.Value)
		maxBound = math.Max(maxBound, p.Bound)
	}
	d, ok, err := PointsContext(ctx, points)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = LowerForDisplacement(d, proofbound.AbsSumUpper(maxBound, extraAllow))
	return d, ok && d > 0, nil
}

// ArcLength is the proven length of one circle or circular arc edge. Unbounded
// is true when no bound on the length can be published.
type ArcLength struct {
	Value     float64
	Bound     float64
	Unbounded bool
}

// CircularArcs reads a lower diameter bound from circle and arc edge lengths.
// Every swept circle or connected circular arc of at most one turn has
// diameter at least its arc length divided by pi, and each length's Bound
// already includes its radius and angular uncertainty. The first usable edge
// decides; sectionDelta is the displacement subtracted from its reference.
// An Unbounded length withholds the reference. lengths is read lazily and
// cancellation of ctx is returned as an error.
func CircularArcs(ctx context.Context, lengths iter.Seq[ArcLength], sectionDelta float64) (float64, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	for length := range lengths {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		if length.Unbounded {
			return 0, false, nil
		}
		value, bound := length.Value, length.Bound
		if !tolerance.UsableMagnitude(value) || !tolerance.UsableMagnitude(bound) || value <= bound {
			continue
		}
		lengthLow := new(big.Rat).Sub(new(big.Rat).SetFloat64(value), new(big.Rat).SetFloat64(bound))
		denominator := new(big.Rat).SetFloat64(proofbound.TwoPiUpper())
		diameterLow := new(big.Rat).Quo(lengthLow.Mul(lengthLow, big.NewRat(2, 1)), denominator)
		d, ok := LowerForDisplacement(proofbound.RatFloatDown(diameterLow), sectionDelta)
		if ok && d > 0 {
			return d, true, nil
		}
	}
	return 0, false, nil
}
