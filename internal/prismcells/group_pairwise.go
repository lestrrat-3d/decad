package prismcells

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// MaxPairwiseTotalSegments bounds the arranger segments charged over every
// scene of one pairwise proof. Each scene stays under MaxArrangementSegments
// on its own; this is the sum.
const MaxPairwiseTotalSegments = 16 * MaxArrangementSegments

// pairwiseScenesPerRegion bounds the scene count of one pairwise proof at
// this many scenes for every region.
const pairwiseScenesPerRegion = 4

// pairwiseBoxSlack is the relative margin added to every region box, on top of
// the caller's own displacement. It exceeds the few ulps the box arithmetic
// can round by and the relative band of sketch's own contact readings, so a
// pair of boxes that stay apart leaves sketch nothing to read between them.
const pairwiseBoxSlack = 1e-9

// ProveGroupApartPairwise is ProveGroupApart for a group whose one
// arrangement scene would exceed MaxArrangementSegments. It answers the same
// question, the outers pairwise disjoint and touching nowhere, by cases.
//
// Every outer gets a conservative box: its lines' endpoints and the full
// circle box of each circle or arc, inflated by inflate (the caller's charged
// displacement of the regions) and a relative slack. Two regions whose boxes
// have a positive gap cannot meet, and need no sketch scene. Each pair of
// regions with overlapping boxes is put to ProveGroupApart as a scene of its
// own, and a region in no such pair to a scene of its own for its validity.
// So every scene keeps today's per-scene cap and is as cheap as today's
// admitted scenes, and the total grows with the number of overlapping pairs.
//
// The box test is a decad-side 2D proof, a stated exception to the "ask
// sketch" default (agent-docs/mirror-pattern-design.md §6.3): it wins on
// cost, and separation of two closed point sets by an axis-aligned gap is
// exact. It can only skip a scene for a pair no curve of which comes near the
// other; a pair with any overlap, or a box it cannot read, goes to sketch.
// The proof refuses with ErrUnsupported when there are more than
// pairwiseScenesPerRegion scenes per region, or MaxPairwiseTotalSegments
// arranger segments over all scenes.
//
// The returned walk charge is the largest over all scenes.
func ProveGroupApartPairwise(ctx context.Context, budget *proofbound.WorkBudget,
	regions []momentinput.Profile, inflate float64) (bool, float64, error) {
	boxes, segments, ok, err := pairwiseBoxes(budget, regions, inflate)
	if err != nil || !ok {
		return false, 0, err
	}
	pairs, err := overlappingPairs(budget, boxes)
	if err != nil {
		return false, 0, err
	}
	inPair := make([]bool, len(regions))
	scenes := len(pairs)
	total := 0
	for _, p := range pairs {
		inPair[p[0]], inPair[p[1]] = true, true
		total += segments[p[0]] + segments[p[1]]
	}
	for i, in := range inPair {
		if !in {
			scenes++
			total += segments[i]
		}
	}
	if scenes > pairwiseScenesPerRegion*len(regions) {
		return false, 0, WithWorkLimit(fmt.Errorf(
			`%w: the pairwise disjointness proof needs %d scenes for %d regions, more than %d per region`,
			decaderr.ErrUnsupported, scenes, len(regions), pairwiseScenesPerRegion), scenes)
	}
	if total > MaxPairwiseTotalSegments {
		return false, 0, WithWorkLimit(fmt.Errorf(
			`%w: the pairwise disjointness scenes charge %d arranger segments in total against a limit of %d`,
			decaderr.ErrUnsupported, total, MaxPairwiseTotalSegments), total)
	}
	walk := 0.0
	for i, in := range inPair {
		if in {
			continue
		}
		apart, charge, err := ProveGroupApart(ctx, budget, regions[i:i+1])
		if err != nil || !apart {
			return false, 0, err
		}
		walk = math.Max(walk, charge)
	}
	for _, p := range pairs {
		apart, charge, err := ProveGroupApart(ctx, budget, []momentinput.Profile{regions[p[0]], regions[p[1]]})
		if err != nil || !apart {
			return false, 0, err
		}
		walk = math.Max(walk, charge)
	}
	return true, walk, nil
}

// regionBox is a closed axis-aligned box in section coordinates.
type regionBox struct{ minU, minV, maxU, maxV float64 }

// pairwiseBoxes returns every region's inflated box and its charged segment
// count. ok is false when a coordinate is not finite, which leaves the group
// unproven.
func pairwiseBoxes(budget *proofbound.WorkBudget, regions []momentinput.Profile,
	inflate float64) ([]regionBox, []int, bool, error) {
	if err := budget.Reserve(uint64(len(regions))); err != nil {
		return nil, nil, false, err
	}
	boxes := make([]regionBox, len(regions))
	segments := make([]int, len(regions))
	scale := 1.0
	for i, region := range regions {
		box := regionBox{minU: math.Inf(1), minV: math.Inf(1), maxU: math.Inf(-1), maxV: math.Inf(-1)}
		grow := func(u0, v0, u1, v1 float64) {
			box.minU, box.minV = math.Min(box.minU, u0), math.Min(box.minV, v0)
			box.maxU, box.maxV = math.Max(box.maxU, u1), math.Max(box.maxV, v1)
		}
		for _, seg := range region.Outer.Segments {
			if err := budget.Step(); err != nil {
				return nil, nil, false, err
			}
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return nil, nil, false, err
			}
			switch {
			case w.IsLine():
				segments[i]++
				grow(math.Min(w.StartU, w.EndU), math.Min(w.StartV, w.EndV),
					math.Max(w.StartU, w.EndU), math.Max(w.StartV, w.EndV))
			case w.IsCircular():
				segments[i] += 256
				grow(w.CU-w.Radius, w.CV-w.Radius, w.CU+w.Radius, w.CV+w.Radius)
			default:
				return nil, nil, false, nil
			}
		}
		for _, c := range []float64{box.minU, box.minV, box.maxU, box.maxV} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return nil, nil, false, nil
			}
			scale = math.Max(scale, math.Abs(c))
		}
		boxes[i] = box
	}
	if math.IsNaN(inflate) || math.IsInf(inflate, 0) || inflate < 0 {
		return nil, nil, false, nil
	}
	slack := inflate + pairwiseBoxSlack*scale
	for i := range boxes {
		boxes[i].minU -= slack
		boxes[i].minV -= slack
		boxes[i].maxU += slack
		boxes[i].maxV += slack
	}
	return boxes, segments, true, nil
}

// overlappingPairs sorts the boxes by their low U edge and sweeps them,
// returning every index pair (i < j) whose boxes overlap or touch. It stops
// refusing as soon as the pairs alone exceed the scene limit.
func overlappingPairs(budget *proofbound.WorkBudget, boxes []regionBox) ([][2]int, error) {
	order := make([]int, len(boxes))
	for i := range order {
		order[i] = i
	}
	if _, err := budget.StepN(uint64(len(boxes))); err != nil {
		return nil, err
	}
	slices.SortFunc(order, func(a, b int) int { return cmp.Compare(boxes[a].minU, boxes[b].minU) })
	limit := pairwiseScenesPerRegion * len(boxes)
	var pairs [][2]int
	for x, i := range order {
		for _, j := range order[x+1:] {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if boxes[j].minU > boxes[i].maxU {
				break
			}
			if boxes[j].minV > boxes[i].maxV || boxes[i].minV > boxes[j].maxV {
				continue
			}
			pairs = append(pairs, [2]int{min(i, j), max(i, j)})
			if len(pairs) > limit {
				return nil, WithWorkLimit(fmt.Errorf(
					`%w: the pairwise disjointness proof finds more than %d overlapping region boxes for %d regions`,
					decaderr.ErrUnsupported, limit, len(boxes)), len(pairs))
			}
		}
	}
	slices.SortFunc(pairs, func(a, b [2]int) int {
		return cmp.Or(cmp.Compare(a[0], b[0]), cmp.Compare(a[1], b[1]))
	})
	return pairs, nil
}
