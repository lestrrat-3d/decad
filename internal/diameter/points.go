// Package diameter computes lower bounds on distances between held witnesses
// for Verify's geometry-relative tolerance reference.
package diameter

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"runtime"
	"sync"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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
	if len(points) >= 512 && runtime.GOMAXPROCS(0) > 1 && parallelScanSafe(points) {
		return pointsContextParallel(ctx, points)
	}
	// The poll budget returns what NewWorkBudget's would at every step; its
	// StepN charges the scan's skipped blocks in one move per poll.
	return PointsWithBudget(proofbound.NewPollWorkBudget(ctx), points)
}

type pairMaximum struct {
	distance float64
	i, j     int
}

const pointBlockSize = 64

// skipMargin scales the best float distance found so far into the threshold
// below which the scans skip a block pair or a single pair. The scan returns
// the pair with the largest float Len under the (i, j) tie-break and then reads
// that one pair exactly, so the answer is unchanged by any margin that skips
// only pairs whose float Len is strictly below the best, because such a pair
// can never win betterPair.
//
// The margin is sound for a margin m = 1 - 2^-16 by this rounding argument.
// For a pair (p, q) in a skipped block pair, let D be its exact distance and U
// the exact squared extent of the two boxes, so D^2 <= U. The float
// blockPairDistanceUpperSq forms three differences (one rounding each; abs and
// max are exact), three squares and two sums of non-negative terms, so at most
// eight roundings of relative size 2^-53 each, and the computed value is at
// least U(1 - 2^-49). The skip fires when that value is below skipSq, which is
// (best*m)^2 rounded twice and at most best^2 m^2 (1 + 2^-52). Hence
// D^2 < best^2 m^2 (1 + 2^-52)/(1 - 2^-49). The float Len of the pair's Sub
// is D times at most (1 + 2^-49) by the same count of roundings (three
// differences, three squares, two sums, one square root; outside the ordinary
// range r3 uses math.Hypot, within an ulp, which the slack also covers). So Len < best*m*(1 +
// 2^-47) < best, since m*(1 + 2^-47) < 1. The per-pair test
// dx*dx+dy*dy+dz*dz < skipSq has the same shape. The 2^-16 gap is nine
// orders of magnitude wider than the rounding, so no float Len at or above
// best is ever skipped. The thresholds are armed only for best in (1e-100,
// 1e100), where no square or sum above underflows or overflows a normal
// float64 for a pair that is not already far below best.
const skipMargin = 1 - 1.0/65536

type pointBlock struct {
	min, max r3.Vec
}

func pointBlocks(points []r3.Vec) []pointBlock {
	blocks := make([]pointBlock, (len(points)+pointBlockSize-1)/pointBlockSize)
	for block := range blocks {
		start := block * pointBlockSize
		end := min(start+pointBlockSize, len(points))
		lo, hi := points[start], points[start]
		for _, point := range points[start+1 : end] {
			lo.X = math.Min(lo.X, point.X)
			lo.Y = math.Min(lo.Y, point.Y)
			lo.Z = math.Min(lo.Z, point.Z)
			hi.X = math.Max(hi.X, point.X)
			hi.Y = math.Max(hi.Y, point.Y)
			hi.Z = math.Max(hi.Z, point.Z)
		}
		blocks[block] = pointBlock{min: lo, max: hi}
	}
	return blocks
}

func pointBlockDistanceUpperSq(point r3.Vec, block pointBlock) float64 {
	dx := math.Max(math.Abs(point.X-block.min.X), math.Abs(point.X-block.max.X))
	dy := math.Max(math.Abs(point.Y-block.min.Y), math.Abs(point.Y-block.max.Y))
	dz := math.Max(math.Abs(point.Z-block.min.Z), math.Abs(point.Z-block.max.Z))
	return dx*dx + dy*dy + dz*dz
}

// parallelScanSafe keeps the sequential refusal order for non-finite or very
// large coordinates. Bounded coordinates also keep every pair norm finite.
func parallelScanSafe(points []r3.Vec) bool {
	for _, point := range points {
		if !(math.Abs(point.X) <= 1e100 && math.Abs(point.Y) <= 1e100 && math.Abs(point.Z) <= 1e100) {
			return false
		}
	}
	return true
}

func pointsContextParallel(ctx context.Context, points []r3.Vec) (float64, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	workers := min(runtime.GOMAXPROCS(0), 4)
	ordered, blocks := spatialPointBlocks(points)
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	seed := extremePair(points)
	maxima := make([]pairMaximum, workers)
	var group sync.WaitGroup
	group.Add(workers)
	for worker := range workers {
		go func() {
			defer group.Done()
			best := seed
			skipSq := 0.0
			if best.distance > 1e-100 && best.distance < 1e100 {
				limit := best.distance * skipMargin
				skipSq = limit * limit
			}
			work := 0
			for ai := worker; ai < len(blocks); ai += workers {
				a := blocks[ai]
				for bi := ai; bi < len(blocks); bi++ {
					b := blocks[bi]
					work++
					if work%proofbound.WorkPollInterval == 0 && ctx.Err() != nil {
						return
					}
					// Each box encloses every point in its leaf. skipMargin
					// keeps the floating upper bound above skipped pairs.
					if skipSq > 0 && blockPairDistanceUpperSq(a.box, b.box) < skipSq {
						count := (a.end - a.begin) * (b.end - b.begin)
						if ai == bi {
							count = (a.end - a.begin) * (a.end - a.begin - 1) / 2
						}
						work += count
						if ctx.Err() != nil {
							return
						}
						continue
					}
					for first := a.begin; first < a.end; first++ {
						secondBegin := b.begin
						if ai == bi {
							secondBegin = first + 1
						}
						for second := secondBegin; second < b.end; second++ {
							work++
							if work%proofbound.WorkPollInterval == 0 && ctx.Err() != nil {
								return
							}
							i, j := ordered[first].index, ordered[second].index
							if i > j {
								i, j = j, i
							}
							delta := points[i].Sub(points[j])
							if skipSq > 0 && delta.X*delta.X+delta.Y*delta.Y+delta.Z*delta.Z < skipSq {
								continue
							}
							distance := delta.Len()
							candidate := pairMaximum{distance: distance, i: i, j: j}
							if betterPair(candidate, best) {
								best = candidate
								if distance > 1e-100 && distance < 1e100 {
									limit := distance * skipMargin
									skipSq = limit * limit
								} else {
									skipSq = 0
								}
							}
						}
					}
				}
			}
			maxima[worker] = best
		}()
	}
	group.Wait()
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	best := pairMaximum{}
	for _, candidate := range maxima {
		if betterPair(candidate, best) {
			best = candidate
		}
	}
	if best.distance == 0 {
		return 0, true, nil
	}
	d, ok := pairDistanceDown(points[best.i], points[best.j])
	return d, ok, nil
}

// PointsWithBudget selects a witness pair in float arithmetic, then publishes
// that pair's exact-rational distance rounded down. A float scan can choose
// any pair without invalidating the lower bound; it only affects tightness.
// The returned value never exceeds the maximum exact pair distance.
//
// The float scan cannot publish its own result: Sub and Len can round ABOVE
// the exact distance. A 6×6×7 box's diagonal, exactly 11, can read as
// 11.000000000000002 in float arithmetic. Every Verify reference needs a
// lower bound (agent-docs/verification-design.md §3); an overstated one could
// loosen the tolerance gate. A near-tie misordered by the float scan only
// changes which real pair the exact step proves. An empty set, an unusable
// float distance, or a winning pair without rational coordinates returns no
// answer. A zero maximum needs no rounding step.
func PointsWithBudget(budget *proofbound.WorkBudget, points []r3.Vec) (float64, bool, error) {
	if budget != nil && budget.Replay {
		return 0, false, fmt.Errorf("%w: replay diameter requires an explicit context", decaderr.ErrUnsupported)
	}
	best, i, j, ok, err := pointsScan(budget, points)
	if err != nil || !ok {
		return 0, false, err
	}
	if best == 0 {
		return 0, true, nil
	}
	d, ok := pairDistanceDown(points[i], points[j])
	if !ok {
		return 0, false, nil
	}
	return d, true, nil
}

// PointsReplay measures the same witnesses using the caller's replay context.
func PointsReplay(ctx context.Context, budget *proofbound.WorkBudget, points []r3.Vec) (float64, bool, error) {
	if budget == nil || !budget.Replay {
		return 0, false, fmt.Errorf("%w: missing replay diameter budget", decaderr.ErrUnsupported)
	}
	best, i, j, ok, err := pointsScan(budget, points)
	if err != nil || !ok {
		return 0, false, err
	}
	if best == 0 {
		return 0, true, nil
	}
	d, err := replayPairDistanceDown(ctx, budget, points[i], points[j])
	if err != nil {
		return 0, false, err
	}
	return d, true, nil
}

func pointsScan(budget *proofbound.WorkBudget, points []r3.Vec) (float64, int, int, bool, error) {
	if len(points) == 0 {
		return 0, 0, 0, false, nil
	}
	best := 0.0
	bestI, bestJ := 0, 0
	skipSq := 0.0
	var blocks []pointBlock
	if len(points) >= 512 && parallelScanSafe(points) {
		if budget != nil && budget.Replay {
			n := (len(points) + pointBlockSize - 1) / pointBlockSize
			if err := budget.ReserveBytesMul(uint64(n), uint64(unsafe.Sizeof(pointBlock{}))); err != nil {
				return 0, 0, 0, false, err
			}
			if _, err := budget.StepN(uint64(len(points))); err != nil {
				return 0, 0, 0, false, err
			}
		}
		blocks = pointBlocks(points)
	}
	for i := range points {
		for j := i + 1; j < len(points); {
			end := len(points)
			if blocks != nil {
				block := j / pointBlockSize
				end = min((block+1)*pointBlockSize, end)
				if skipSq > 0 && pointBlockDistanceUpperSq(points[i], blocks[block]) < skipSq {
					// A budgeted scan still charges each skipped pair in order.
					// StepN(n) is n Step calls stopping at the first error, and
					// nothing reads a skipped pair, so one StepN call per block
					// returns the same error after the same steps and context
					// polls as one Step per pair. A poll budget replays it in
					// one move per poll.
					if budget != nil {
						if _, err := budget.StepN(uint64(end - j)); err != nil {
							return 0, 0, 0, false, err
						}
					}
					j = end
					continue
				}
			}
			for ; j < end; j++ {
				if budget != nil {
					if err := budget.Step(); err != nil {
						return 0, 0, 0, false, err
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
					return 0, 0, 0, false, nil
				}
				if distance > best {
					best, bestI, bestJ = distance, i, j
					if best > 1e-100 && best < 1e100 {
						limit := best * skipMargin
						skipSq = limit * limit
					} else {
						skipSq = 0
					}
				}
			}
		}
	}
	if budget != nil {
		if err := budget.Err(); err != nil {
			return 0, 0, 0, false, err
		}
	}
	return best, bestI, bestJ, true, nil
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

// AxisExtremesContext selects six actual held vertices as a lower
// diameter witness for the transient tool operand. A subset's exact pair
// distance cannot exceed the full mesh diameter.
func AxisExtremesContext(ctx context.Context, verts []r3.Vec) (float64, bool, error) {
	if len(verts) == 0 {
		return 0, false, nil
	}
	witness := [6]r3.Vec{verts[0], verts[0], verts[0], verts[0], verts[0], verts[0]}
	for i, v := range verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		if !proofbound.FiniteVec(v) {
			return 0, false, nil
		}
		if v.X < witness[0].X {
			witness[0] = v
		}
		if v.X > witness[1].X {
			witness[1] = v
		}
		if v.Y < witness[2].Y {
			witness[2] = v
		}
		if v.Y > witness[3].Y {
			witness[3] = v
		}
		if v.Z < witness[4].Z {
			witness[4] = v
		}
		if v.Z > witness[5].Z {
			witness[5] = v
		}
	}
	return PointsContext(ctx, witness[:])
}
