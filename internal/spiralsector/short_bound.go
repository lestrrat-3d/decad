package spiralsector

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type shortFloatBox [3][2]float64

// ShortFloatCover bounds both directions of a short-cap box match with
// outward floats and reports whether the result lies below minBound.
func ShortFloatCover(ctx context.Context, source, mesh [][3]loftface.Interval,
	minBound float64) (float64, bool) {
	convert := func(boxes [][3]loftface.Interval) ([]shortFloatBox, bool) {
		out := make([]shortFloatBox, len(boxes))
		for i, box := range boxes {
			if i%256 == 0 && ctx.Err() != nil {
				return nil, false
			}
			for axis, interval := range box {
				if interval.Lo == nil || interval.Hi == nil || interval.Lo.Cmp(interval.Hi) > 0 {
					return nil, false
				}
				out[i][axis] = [2]float64{
					proofbound.RatFloatDown(interval.Lo), proofbound.RatFloatUp(interval.Hi),
				}
				if proofbound.IsNonFinite(out[i][axis][0]) || proofbound.IsNonFinite(out[i][axis][1]) {
					return nil, false
				}
			}
		}
		return out, true
	}
	a, okA := convert(source)
	b, okB := convert(mesh)
	if !okA || !okB || len(a) == 0 || len(b) == 0 {
		return 0, false
	}
	// One step down encloses the exact square of a finite positive float.
	limit := math.Nextafter(minBound*minBound, math.Inf(-1))
	if limit <= 0 || proofbound.IsNonFinite(limit) {
		return 0, false
	}
	check := func(from, to []shortFloatBox) (float64, bool) {
		largest, previous := 0.0, 0
		for i, box := range from {
			if i%16 == 0 && ctx.Err() != nil {
				return 0, false
			}
			found := false
			for offset := range to {
				candidate := (previous + offset) % len(to)
				upper := shortFloatDistanceSquared(box, to[candidate])
				if upper < limit {
					largest = max(largest, upper)
					previous, found = candidate, true
					break
				}
			}
			if !found {
				return 0, false
			}
		}
		return largest, true
	}
	first, okFirst := check(a, b)
	if !okFirst {
		return 0, false
	}
	second, okSecond := check(b, a)
	if !okSecond || ctx.Err() != nil {
		return 0, false
	}
	return proofbound.RatSqrtUp(ratF(max(first, second))), true
}

// The outward endpoints enclose each exact coordinate interval. Either
// opposite-endpoint difference bounds the maximum separation on that axis.
func shortFloatDistanceSquared(a, b shortFloatBox) float64 {
	var squared float64
	for axis := range a {
		far := max(0,
			math.Nextafter(a[axis][1]-b[axis][0], math.Inf(1)),
			math.Nextafter(b[axis][1]-a[axis][0], math.Inf(1)))
		if proofbound.IsNonFinite(far) {
			return math.Inf(1)
		}
		squared = proofbound.AbsSumUpper(squared, proofbound.ProductUpper(far, far))
	}
	return squared
}

// ShortBoxesDistanceSquared returns the exact largest squared distance
// between two three-axis interval boxes.
func ShortBoxesDistanceSquared(a, b [3]loftface.Interval) (*big.Rat, bool) {
	distance := new(big.Rat)
	for axis := range a {
		if a[axis].Lo == nil || a[axis].Hi == nil || b[axis].Lo == nil || b[axis].Hi == nil ||
			a[axis].Lo.Cmp(a[axis].Hi) > 0 || b[axis].Lo.Cmp(b[axis].Hi) > 0 {
			return nil, false
		}
		lo := new(big.Rat).Abs(proof.SubRat(new(big.Rat), a[axis].Lo, b[axis].Hi))
		hi := new(big.Rat).Abs(proof.SubRat(new(big.Rat), a[axis].Hi, b[axis].Lo))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		distance = proof.AddRat(new(big.Rat), distance, proof.MulRat(new(big.Rat), lo, lo))
	}
	return distance, true
}
