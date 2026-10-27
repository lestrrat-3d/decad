package prismcells

import (
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
// Every answer about regions is read from Sketch's own arrangement: selected
// cells sharing a wall form one region, and of that region's loops the outer
// one is the loop whose far side reaches the unbounded face without entering
// the region. Each region must have exactly one such loop; its other loops
// are its holes. No coordinate is compared to place a loop.
//
// A loop is restated exactly when every cut end is a certified line-line
// crossing and every arc and circle is whole. Otherwise it keeps its
// fragments and their cut charge, which the caller must refuse.
// resolved=false (err nil) means a shape this merge does not cover.
func MergeRegionsExact(budget *proofbound.WorkBudget, scene *sketch.Sketch, profiles []*sketch.Profile,
	selected []bool) (ExactRegions, bool, error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	keyOf := func(e sketch.BoundaryEdge) edgeKey { return edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd} }
	n := len(profiles)
	// sides lists every cell walking each arrangement edge. A key walked by
	// one cell has the unbounded face, node n, on its far side.
	sides := map[edgeKey][]int{}
	var picked []*sketch.Profile
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return ExactRegions{}, false, err
		}
		if selected[i] {
			picked = append(picked, p)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return ExactRegions{}, false, err
				}
				sides[keyOf(e)] = append(sides[keyOf(e)], i)
			}
		}
	}
	links := make([][]int, n+1)
	for k, cells := range sides {
		if err := budget.Step(); err != nil {
			return ExactRegions{}, false, err
		}
		switch len(cells) {
		case 1:
			sides[k] = append(cells, n)
		case 2:
		default:
			return ExactRegions{}, false, nil
		}
		a, b := sides[k][0], sides[k][1]
		links[a] = append(links[a], b)
		links[b] = append(links[b], a)
	}

	// Selected cells sharing a wall are one region.
	region := make([]int, n)
	for i := range region {
		region[i] = -1
	}
	regions := 0
	for i := range n {
		if !selected[i] || region[i] >= 0 {
			continue
		}
		region[i] = regions
		queue := []int{i}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return ExactRegions{}, false, err
			}
			cur := queue[0]
			queue = queue[1:]
			for _, nb := range links[cur] {
				if nb < n && selected[nb] && region[nb] < 0 {
					region[nb] = regions
					queue = append(queue, nb)
				}
			}
		}
		regions++
	}

	survivors, resolved, err := mergeSurvivors(budget, picked, "section")
	if err != nil || !resolved {
		return ExactRegions{}, false, err
	}
	chains, resolved, err := ChainClosedLoops(budget, survivors)
	if err != nil || !resolved {
		return ExactRegions{}, false, err
	}

	// outside[r][c] marks the cells reached from the unbounded face without
	// entering region r.
	outside := make([][]bool, regions)
	reach := func(r int) ([]bool, error) {
		seen := make([]bool, n+1)
		seen[n] = true
		queue := []int{n}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			cur := queue[0]
			queue = queue[1:]
			for _, nb := range links[cur] {
				if seen[nb] || (nb < n && region[nb] == r) {
					continue
				}
				seen[nb] = true
				queue = append(queue, nb)
			}
		}
		return seen, nil
	}

	// The refusals in this loop falsify Sketch's topology rather than test the
	// geometry: a survivor between two selected cells, a loop bordering two
	// regions or two complement components, and a region with no outer loop
	// or two. No consistent arrangement reaches them, so no fixture does.
	out := ExactRegions{Regions: make([]momentinput.Profile, regions)}
	hasOuter := make([]bool, regions)
	for _, chain := range chains {
		owner, far := -1, -1
		for _, e := range chain {
			if err := budget.Step(); err != nil {
				return ExactRegions{}, false, err
			}
			cells := sides[keyOf(e)]
			inner, other := cells[0], cells[1]
			if inner == n || !selected[inner] {
				inner, other = other, inner
			}
			if inner == n || !selected[inner] || (other < n && selected[other]) {
				return ExactRegions{}, false, nil
			}
			if owner >= 0 && owner != region[inner] {
				return ExactRegions{}, false, nil
			}
			owner = region[inner]
			if outside[owner] == nil {
				seen, err := reach(owner)
				if err != nil {
					return ExactRegions{}, false, err
				}
				outside[owner] = seen
			}
			// Every edge of one boundary loop faces one complement component.
			reached := 0
			if outside[owner][other] {
				reached = 1
			}
			if far >= 0 && far != reached {
				return ExactRegions{}, false, nil
			}
			far = reached
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
		if far == 1 {
			if hasOuter[owner] {
				return ExactRegions{}, false, nil
			}
			hasOuter[owner] = true
			out.Regions[owner].Outer = loop
			continue
		}
		out.Regions[owner].Holes = append(out.Regions[owner].Holes, loop)
	}
	for _, ok := range hasOuter {
		if !ok {
			return ExactRegions{}, false, nil
		}
	}
	return out, true, nil
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
