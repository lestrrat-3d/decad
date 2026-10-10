// Package diameter computes lower bounds on distances between held witnesses
// for Verify's geometry-relative tolerance reference.
package diameter

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Points reads a witness set without a work budget.
func Points(points []r3.Vec) (float64, bool) {
	d, ok, _ := PointsWithBudget(nil, points)
	return d, ok
}

// PointsContext reads a witness set with cancellation through ctx.
func PointsContext(ctx context.Context, points []r3.Vec) (float64, bool, error) {
	return PointsWithBudget(proofbound.NewWorkBudget(ctx), points)
}

// PointsWithBudget selects a witness pair in float arithmetic, then publishes
// that pair's exact-rational distance rounded down. A float scan can choose
// any pair without invalidating the lower bound; it only affects tightness.
// The returned value never exceeds the maximum exact pair distance.
//
// The float scan cannot publish its own result: Sub and Len can round ABOVE
// the exact distance. A 6×6×7 box's diagonal, exactly 11, can read as
// 11.000000000000002 in float arithmetic. Every Verify reference needs a
// lower bound (docs/verification-design.md §3); an overstated one could
// loosen the tolerance gate. A near-tie misordered by the float scan only
// changes which real pair the exact step proves. An empty set, an unusable
// float distance, or a winning pair without rational coordinates returns no
// answer. A zero maximum needs no rounding step.
func PointsWithBudget(budget *proofbound.WorkBudget, points []r3.Vec) (float64, bool, error) {
	if len(points) == 0 {
		return 0, false, nil
	}
	best := 0.0
	bestI, bestJ := 0, 0
	skipSq := 0.0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			if budget != nil {
				if err := budget.Step(); err != nil {
					return 0, false, err
				}
			}
			delta := points[i].Sub(points[j])
			// This wide gap keeps the rounded squared length strictly below
			// any pair Len could select. Out-of-range and non-finite inputs
			// still reach Len and its original refusal path.
			if skipSq > 0 && delta.X*delta.X+delta.Y*delta.Y+delta.Z*delta.Z < skipSq {
				continue
			}
			distance := delta.Len()
			if !usableMagnitude(distance) {
				return 0, false, nil
			}
			if distance > best {
				best, bestI, bestJ = distance, i, j
				if best > 1e-100 && best < 1e100 {
					limit := best * 0.9
					skipSq = limit * limit
				} else {
					skipSq = 0
				}
			}
		}
	}
	if budget != nil {
		if err := budget.Err(); err != nil {
			return 0, false, err
		}
	}
	if best == 0 {
		return 0, true, nil
	}
	d, ok := pairDistanceDown(points[bestI], points[bestJ])
	if !ok {
		return 0, false, nil
	}
	return d, true, nil
}

// pairDistanceDown returns the largest float64 at or below the exact
// distance between two finite held points. Every float coordinate has an
// exact rational form; RatSqrtDown checks the candidate's exact square
// against the exact sum. Non-finite coordinates have no rational form and
// return no answer.
func pairDistanceDown(a, b r3.Vec) (float64, bool) {
	total := new(big.Rat)
	for _, axis := range [3][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		ra, rb := proofarith.FloatRat(axis[0]), proofarith.FloatRat(axis[1])
		if ra == nil || rb == nil {
			return 0, false
		}
		d := ra.Sub(ra, rb)
		total.Add(total, d.Mul(d, d))
	}
	return proofbound.RatSqrtDown(total), true
}

// LowerForDisplacement shrinks a held witness diameter by twice the
// displacement each witness can carry. The subtraction rounds down so the
// result remains a lower bound on the denoted body's diameter. Zero
// displacement leaves the original bit pattern untouched.
func LowerForDisplacement(d, displacement float64) (float64, bool) {
	if displacement == 0 {
		return d, true
	}
	if !usableMagnitude(d) || !usableMagnitude(displacement) {
		return 0, false
	}
	d = freeform.DownRound(d - 2*displacement)
	return d, d > 0 && usableMagnitude(d)
}

func usableMagnitude(v float64) bool {
	return v >= 0 && v <= math.MaxFloat64
}
