// Package box contains contact proofs for admitted axis and oriented boxes.
package box

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// AxisBox is the complete occupied interval of an admitted signed-axis box.
type AxisBox struct {
	Lo, Hi [3]proof.Dyadic
}

// AxisBoxRequest bounds the error permitted for each published witness.
type AxisBoxRequest struct {
	PointResolutionMM float64
}

// PointReading encloses a world-space point in a millimetre-radius ball.
type PointReading struct {
	Value   r3.Vec
	BoundMM float64
}

// FaceSlot selects one of an admitted box's original support faces.
// Side 0 is the minimum support face; side 1 is the maximum.
type FaceSlot struct {
	Axis, Side int
}

// AxisBoxPoint contains the two boundary witnesses at one patch corner.
type AxisBoxPoint struct {
	OnA, OnB PointReading
}

// AxisBoxPatch describes a face patch without importing public topology.
type AxisBoxPatch struct {
	FaceA, FaceB           FaceSlot
	NormalAxis, NormalSign int
	Separation             pair.ScalarReading
	Points                 []AxisBoxPoint
}

// AxisBoxResult is the complete relation and optional bounded face patch.
type AxisBoxResult struct {
	Relation pair.Relation
	Reason   pair.Reason
	Gap      *pair.ScalarReading
	Patch    *AxisBoxPatch
}

// ClassifyAxisBoxes compares complete occupied intervals and computes a
// bounded face patch when one support direction and two face interiors prove
// it. Its inputs have already passed source-box admission in the caller.
func ClassifyAxisBoxes(a, b AxisBox, req AxisBoxRequest) AxisBoxResult {
	var gaps [3]proof.Dyadic
	touchAxes := 0
	overlaps := true
	for axis := range 3 {
		switch {
		case proof.DyCmp(a.Hi[axis], b.Lo[axis]) < 0:
			gaps[axis] = proof.DySubScalar(b.Lo[axis], a.Hi[axis])
			overlaps = false
		case proof.DyCmp(b.Hi[axis], a.Lo[axis]) < 0:
			gaps[axis] = proof.DySubScalar(a.Lo[axis], b.Hi[axis])
			overlaps = false
		case proof.DyCmp(a.Hi[axis], b.Lo[axis]) == 0 || proof.DyCmp(b.Hi[axis], a.Lo[axis]) == 0:
			touchAxes++
			overlaps = false
		}
	}
	for _, gap := range gaps {
		if gap.Sign() <= 0 {
			continue
		}
		reading, ok := AxisGap(gaps)
		if !ok {
			return AxisBoxResult{Reason: pair.NoGapProof}
		}
		return AxisBoxResult{Relation: pair.Separated, Gap: &reading}
	}
	if touchAxes > 0 {
		zero := pair.ScalarReading{}
		result := AxisBoxResult{Relation: pair.Touching, Gap: &zero}
		if touchAxes != 1 {
			result.Reason = pair.AmbiguousFeature
			return result
		}
		for axis := range 3 {
			if proof.DyCmp(a.Hi[axis], b.Lo[axis]) == 0 {
				result.Patch, result.Reason = FacePatch(a, b, axis, 1, proof.DyZero(), req)
				return result
			}
			if proof.DyCmp(b.Hi[axis], a.Lo[axis]) == 0 {
				result.Patch, result.Reason = FacePatch(a, b, axis, -1, proof.DyZero(), req)
				return result
			}
		}
	}
	if !overlaps {
		return AxisBoxResult{Reason: pair.PayloadUnsupported}
	}
	result := AxisBoxResult{Relation: pair.Overlapping}
	axis, sign, depth, unique := axisBoxTranslation(a, b)
	if !unique {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	if sign > 0 {
		if proof.DyCmp(b.Lo[axis], a.Lo[axis]) <= 0 || proof.DyCmp(b.Hi[axis], a.Hi[axis]) <= 0 {
			result.Reason = pair.AmbiguousFeature
			return result
		}
	} else if proof.DyCmp(b.Lo[axis], a.Lo[axis]) >= 0 || proof.DyCmp(b.Hi[axis], a.Hi[axis]) >= 0 {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	result.Patch, result.Reason = FacePatch(a, b, axis, sign, proof.DyNeg(depth), req)
	return result
}

func axisBoxTranslation(a, b AxisBox) (int, int, proof.Dyadic, bool) {
	var best proof.Dyadic
	axis, sign, ties := 0, 0, false
	for i := range 3 {
		for _, candidate := range []struct {
			value proof.Dyadic
			sign  int
		}{
			{proof.DySubScalar(a.Hi[i], b.Lo[i]), 1},
			{proof.DySubScalar(b.Hi[i], a.Lo[i]), -1},
		} {
			if candidate.value.Sign() <= 0 {
				return 0, 0, proof.Dyadic{}, false
			}
			cmp := proof.DyCmp(candidate.value, best)
			if sign == 0 || cmp < 0 {
				axis, sign, best, ties = i, candidate.sign, candidate.value, false
			} else if cmp == 0 {
				ties = true
			}
		}
	}
	return axis, sign, best, !ties
}

// FacePatch computes the bounded rectangle shared by two certified support
// faces. Other admitted pair paths may use it after proving their face slots.
func FacePatch(a, b AxisBox, axis, sign int, separation proof.Dyadic,
	req AxisBoxRequest) (*AxisBoxPatch, pair.Reason) {
	var sideA, sideB int
	if sign > 0 {
		sideA, sideB = 1, 0
	} else {
		sideA, sideB = 0, 1
	}
	var projected [2]int
	n := 0
	for i := range 3 {
		if i != axis {
			projected[n] = i
			n++
		}
	}
	var low, high [2]proof.Dyadic
	for i, ax := range projected {
		low[i], high[i] = dyMax(a.Lo[ax], b.Lo[ax]), dyMin(a.Hi[ax], b.Hi[ax])
		if proof.DyCmp(low[i], high[i]) >= 0 {
			return nil, pair.AmbiguousFeature
		}
	}
	sep, ok := SignedReading(separation)
	if !ok {
		return nil, pair.PointTooCoarse
	}
	patch := &AxisBoxPatch{
		FaceA:      FaceSlot{Axis: axis, Side: sideA},
		FaceB:      FaceSlot{Axis: axis, Side: sideB},
		NormalAxis: axis, NormalSign: sign, Separation: sep,
		Points: make([]AxisBoxPoint, 0, 4),
	}
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		var pA, pB proof.DyV3
		for i, ax := range projected {
			coord := low[i]
			if corner[i] == 1 {
				coord = high[i]
			}
			pA[ax], pB[ax] = coord, coord
		}
		if sign > 0 {
			pA[axis], pB[axis] = a.Hi[axis], b.Lo[axis]
		} else {
			pA[axis], pB[axis] = a.Lo[axis], b.Hi[axis]
		}
		onA, okA := ReadPointAt(&pA)
		onB, okB := ReadPointAt(&pB)
		if !okA || !okB || onA.BoundMM > req.PointResolutionMM || onB.BoundMM > req.PointResolutionMM {
			return nil, pair.PointTooCoarse
		}
		patch.Points = append(patch.Points, AxisBoxPoint{OnA: onA, OnB: onB})
	}
	return patch, pair.NoReason
}

// AxisGap reads the length of a vector of nonnegative exact axis gaps.
func AxisGap(gaps [3]proof.Dyadic) (pair.ScalarReading, bool) {
	positive := 0
	var only proof.Dyadic
	var squared proof.Dyadic
	for _, gap := range gaps {
		if gap.Sign() > 0 {
			positive++
			only = gap
			squared = proof.DyAdd(squared, proof.DyMul(gap, gap))
		}
	}
	if positive == 1 {
		value, exact := only.Float64()
		if !finite(value) {
			return pair.ScalarReading{}, false
		}
		bound := proof.DyadicFloatError(only, value)
		return pair.ScalarReading{ValueMM: value, BoundMM: bound}, exact || bound < value
	}
	lo, hi := proof.DySqrtDown(squared), proof.DySqrtUp(squared)
	if !finite(lo) || !finite(hi) || lo <= 0 {
		return pair.ScalarReading{}, false
	}
	value := lo + (hi-lo)/2
	bound := proof.ProvenUpRound(math.Max(value-lo, hi-value))
	return pair.ScalarReading{ValueMM: value, BoundMM: bound}, finite(value) && finite(bound)
}

// ReadPointAt converts an exact point into a float point and proven radius.
// The pointer avoids copying its integer-backed dyadic components.
func ReadPointAt(point *proof.DyV3) (PointReading, bool) {
	var coords [3]float64
	bound := 0.0
	for i := range 3 {
		coords[i], _ = point[i].Float64()
		if !finite(coords[i]) {
			return PointReading{}, false
		}
		bound = math.Max(bound, proof.DyadicFloatError(point[i], coords[i]))
	}
	bound = proof.Radius3D(bound)
	if !finite(bound) {
		return PointReading{}, false
	}
	return PointReading{Value: r3.Vec{X: coords[0], Y: coords[1], Z: coords[2]}, BoundMM: bound}, true
}

// SignedReading converts an exact signed length into a bounded float length.
func SignedReading(value proof.Dyadic) (pair.ScalarReading, bool) {
	held, _ := value.Float64()
	if !finite(held) {
		return pair.ScalarReading{}, false
	}
	bound := proof.DyadicFloatError(value, held)
	return pair.ScalarReading{ValueMM: held, BoundMM: bound}, finite(bound)
}

func dyMax(a, b proof.Dyadic) proof.Dyadic {
	if proof.DyCmp(a, b) >= 0 {
		return a
	}
	return b
}

func dyMin(a, b proof.Dyadic) proof.Dyadic {
	if proof.DyCmp(a, b) <= 0 {
		return a
	}
	return b
}

func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
