package prismcells

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// ExactRegions is MergeRegionsExact's answer: one record per connected
// selected region, the largest cut charge any recorded loop still owes, and
// whether a surviving edge is a cut arc or circle.
type ExactRegions struct {
	Regions   []momentinput.Profile
	CutDelta  float64
	CurvedCut bool
}

// MergeRegionsExact is reserved for Body.Section of a mesh Boolean result read
// from its history (internal/sectionhistory, agent-docs/section-design.md "The
// 2D operation"). It merges the cells selected marks into regions that may
// carry holes and may be several. Interior walls drop as Merge drops them and
// the survivors chain into simple loops by exact walked ends; a vertex two
// survivors leave, such as two regions touching at a point, is unresolved.
//
// Selected cells sharing a wall are one region, which is Sketch's own cell
// adjacency. Which loop of a region is its outer loop is Sketch's containment
// verdict: every merged loop is declared as a loop of spans of the scene's
// entities, the scene is published again, and a cell of the region must lie
// inside exactly one of its loops, the outer one, and outside the rest, its
// holes. A missing or unproven verdict is unresolved. No coordinate is
// compared to place a loop.
//
// A loop is restated exactly when every cut end is a certified line-line
// crossing and every arc and circle is whole. Otherwise it keeps its
// fragments and their cut charge, which the caller must refuse.
// resolved=false (err nil) means a shape this merge does not cover.
func MergeRegionsExact(ctx context.Context, budget *proofbound.WorkBudget, scene *sketch.Sketch,
	profiles []*sketch.Profile, selected []bool) (ExactRegions, bool, error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	keyOf := func(e sketch.BoundaryEdge) edgeKey { return edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd} }
	n := len(profiles)
	// sides lists every selected cell walking each arrangement edge.
	sides := map[edgeKey][]int{}
	var picked []*sketch.Profile
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return ExactRegions{}, false, err
		}
		if !selected[i] {
			continue
		}
		picked = append(picked, p)
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return ExactRegions{}, false, err
				}
				sides[keyOf(e)] = append(sides[keyOf(e)], i)
			}
		}
	}

	// Selected cells sharing a wall are one region.
	region := make([]int, n)
	for i := range region {
		region[i] = -1
	}
	var first []int // a representative cell of each region
	for i := range n {
		if !selected[i] || region[i] >= 0 {
			continue
		}
		r := len(first)
		first = append(first, i)
		region[i] = r
		queue := []int{i}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return ExactRegions{}, false, err
			}
			cur := queue[0]
			queue = queue[1:]
			for _, loop := range append([][]sketch.BoundaryEdge{profiles[cur].Outer}, profiles[cur].Holes...) {
				for _, e := range loop {
					for _, nb := range sides[keyOf(e)] {
						if region[nb] < 0 {
							region[nb] = r
							queue = append(queue, nb)
						}
					}
				}
			}
		}
	}

	survivors, resolved, err := mergeSurvivors(budget, picked, "section")
	if err != nil || !resolved {
		return ExactRegions{}, false, err
	}
	chains, resolved, err := ChainClosedLoops(budget, survivors)
	if err != nil || !resolved {
		return ExactRegions{}, false, err
	}

	// A survivor is walked by one selected cell, so every edge of a loop names
	// its region. A loop bordering two regions falsifies the chaining.
	out := ExactRegions{Regions: make([]momentinput.Profile, len(first))}
	owners := make([]int, len(chains))
	records := make([]sectionrecord.LoopRecord, len(chains))
	for c, chain := range chains {
		owner := -1
		for _, e := range chain {
			if err := budget.Step(); err != nil {
				return ExactRegions{}, false, err
			}
			cells := sides[keyOf(e)]
			if len(cells) != 1 || (owner >= 0 && owner != region[cells[0]]) {
				return ExactRegions{}, false, nil
			}
			owner = region[cells[0]]
			if e.Partial {
				if _, isLine := e.Entity.(*sketch.Line); !isLine {
					out.CurvedCut = true
				}
			}
		}
		loop, delta, err := recordChain(budget, chain, nil)
		if err != nil {
			return ExactRegions{}, false, err
		}
		if delta != 0 {
			if exact, ok := reparameterizeExactCurveLoop(scene, chain, loop.Segments); ok {
				loop, delta = sectionrecord.LoopRecord{Segments: exact}, 0
			}
		}
		out.CutDelta = math.Max(out.CutDelta, delta)
		owners[c], records[c] = owner, loop
	}

	outer, resolved, err := outerLoops(ctx, budget, scene, profiles, chains, owners, first)
	if err != nil || !resolved {
		return ExactRegions{}, false, err
	}
	hasOuter := make([]bool, len(first))
	for c, record := range records {
		r := owners[c]
		if !outer[c] {
			out.Regions[r].Holes = append(out.Regions[r].Holes, record)
			continue
		}
		if hasOuter[r] {
			return ExactRegions{}, false, nil
		}
		hasOuter[r] = true
		out.Regions[r].Outer = record
	}
	for _, ok := range hasOuter {
		if !ok {
			return ExactRegions{}, false, nil
		}
	}
	return out, true, nil
}

// outerLoops declares every merged loop on the scene, publishes the scene
// again, and reads whether each loop holds its region's representative cell:
// inside is the region's outer loop and outside is a hole. The republished
// cells must walk exactly the edges the first publication's cells walked, so
// the verdicts describe the same cells.
func outerLoops(ctx context.Context, budget *proofbound.WorkBudget, scene *sketch.Sketch,
	profiles []*sketch.Profile, chains [][]sketch.BoundaryEdge, owners, first []int) ([]bool, bool, error) {
	if len(first) == 0 {
		return nil, false, nil
	}
	before := len(profiles[first[0]].Containments())
	for _, chain := range chains {
		spans := make([]sketch.LoopSpan, len(chain))
		for i, e := range chain {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			spans[i] = sketch.LoopSpan{Entity: e.Entity, TStart: e.TStart, TEnd: e.TEnd}
		}
		if _, err := scene.DeclareLoopSpans(spans...); err != nil {
			return nil, false, nil //nolint:nilerr // a loop Sketch will not declare is a miss
		}
	}
	fresh, err := CellProfiles(ctx, budget, scene)
	if err != nil || len(fresh) != len(profiles) {
		return nil, false, err
	}
	for _, i := range first {
		if ok, err := sameCell(budget, profiles[i], fresh[i]); err != nil || !ok {
			return nil, false, err
		}
	}
	outer := make([]bool, len(chains))
	for c := range chains {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		switch loopVerdict(fresh[first[owners[c]]], before+c) {
		case sketch.ContainmentInside:
			outer[c] = true
		case sketch.ContainmentOutside:
		default:
			return nil, false, nil
		}
	}
	return outer, true, nil
}

// sameCell reports whether two publications of one cell walk the same edges,
// entity, range and direction alike, in the same order.
func sameCell(budget *proofbound.WorkBudget, a, b *sketch.Profile) (bool, error) {
	loopsA := append([][]sketch.BoundaryEdge{a.Outer}, a.Holes...)
	loopsB := append([][]sketch.BoundaryEdge{b.Outer}, b.Holes...)
	if len(loopsA) != len(loopsB) {
		return false, nil
	}
	for l := range loopsA {
		if len(loopsA[l]) != len(loopsB[l]) {
			return false, nil
		}
		for i, e := range loopsA[l] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			f := loopsB[l][i]
			if e.Entity != f.Entity || e.TStart != f.TStart || e.TEnd != f.TEnd || e.Reversed != f.Reversed {
				return false, nil
			}
		}
	}
	return true, nil
}

// reparameterizeExactCurveLoop is reparameterizeExactLineLoop for a loop that
// may also hold whole arcs. Every Partial edge must be a line, and every cut
// end must be a line-line crossing Sketch certifies and exact rational
// membership confirms on both source lines. A whole arc keeps its own record,
// and its walked ends must meet its neighbours' ends exactly. Sketch has no
// certificate that names a line-arc or arc-arc crossing as a binary64 point,
// so a loop with a cut arc or circle keeps its fragments and its cut charge.
func reparameterizeExactCurveLoop(scene *sketch.Sketch, chain []sketch.BoundaryEdge,
	segments []sectionrecord.CurveSegment) ([]sectionrecord.CurveSegment, bool) {
	n := len(chain)
	if scene == nil || n < 2 || len(segments) != n {
		return nil, false
	}
	type end struct {
		line   sectionrecord.LineSeg
		isLine bool
	}
	ends := make([]end, n)
	for i, segment := range segments {
		switch segment := segment.(type) {
		case sectionrecord.LineSeg:
			ends[i] = end{line: segment, isLine: true}
		case sectionrecord.ArcSeg:
			if chain[i].Partial || !WholeSegmentRange(segment.TStart, segment.TEnd) {
				return nil, false
			}
		default:
			return nil, false
		}
	}
	nodes := make([]sectionrecord.Point2, n)
	certified := make([]bool, n)
	cutCount := 0
	for i := range n {
		j := (i + 1) % n
		left, right := ends[i], ends[j]
		leftCut := left.isLine && chain[i].Partial && left.line.TEnd != 0 && left.line.TEnd != 1
		rightCut := right.isLine && chain[j].Partial && right.line.TStart != 0 && right.line.TStart != 1
		if !leftCut && !rightCut {
			continue
		}
		cutCount++
		if !leftCut || !rightCut || !chain[i].TExact || !chain[j].TExact {
			return nil, false
		}
		cross, ok := scene.CertifyLineLineJunction(chain[i], chain[j])
		if !ok {
			return nil, false
		}
		p := sectionrecord.Point2{U: cross[0], V: cross[1]}
		if _, ok := sourceLineParameter(left.line, p); !ok {
			return nil, false
		}
		if _, ok := sourceLineParameter(right.line, p); !ok || !sourceLinesCross(left.line, right.line) {
			return nil, false
		}
		nodes[i], certified[i] = p, true
	}
	if cutCount == 0 {
		return nil, false
	}
	exact := make([]sectionrecord.CurveSegment, n)
	var firstStart, previousEnd sectionrecord.Point2
	for i, segment := range segments {
		var start, stop sectionrecord.Point2
		if arc, isArc := segment.(sectionrecord.ArcSeg); isArc {
			start, stop = arc.Start, arc.End
			if arc.TStart == 1 {
				start, stop = stop, start
			}
			exact[i] = arc
		} else {
			line := ends[i].line
			var ok bool
			start, ok = exactLineEnd(line, line.TStart, nodes[(i+n-1)%n], certified[(i+n-1)%n])
			if !ok {
				return nil, false
			}
			stop, ok = exactLineEnd(line, line.TEnd, nodes[i], certified[i])
			if !ok {
				return nil, false
			}
			startT, ok := sourceLineParameter(line, start)
			if !ok {
				return nil, false
			}
			endT, ok := sourceLineParameter(line, stop)
			if !ok || (line.TStart < line.TEnd && startT.Cmp(endT) >= 0) ||
				(line.TStart > line.TEnd && startT.Cmp(endT) <= 0) {
				return nil, false
			}
			exact[i] = sectionrecord.LineSeg{Start: start, End: stop, TStart: 0, TEnd: 1}
		}
		if i == 0 {
			firstStart = start
		} else if previousEnd != start {
			return nil, false
		}
		previousEnd = stop
	}
	if previousEnd != firstStart {
		return nil, false
	}
	return exact, true
}
