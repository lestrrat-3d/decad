package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// loftQuarterSourceEnvelope relates certified cap crossings to the original
// Loft wall. It does not select output patches or prove occupied volume.
type loftQuarterSourceEnvelope struct {
	order  loftCapSourceOrder
	wall   loftQuarterWallOrderProof
	visits uint64
}

// certifyLoftQuarterSourceEnvelope reads live same-document producers in
// either operand order. All source stages share the unchanged 4096-step wall.
func certifyLoftQuarterSourceEnvelope(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget) (loftQuarterSourceEnvelope, bool, error) {
	var no loftQuarterSourceEnvelope
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil || first == second ||
		first.n.Doc == nil || first.n.Doc != second.n.Doc {
		return no, false, nil
	}
	prism, loft := first, second
	if _, ok := prism.n.Payload.(prismPayload); !ok {
		prism, loft = second, first
	}
	if _, ok := prism.n.Payload.(prismPayload); !ok {
		return no, false, nil
	}
	if _, ok := loft.n.Payload.(loftPayload); !ok {
		return no, false, nil
	}
	wall := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
	var visits uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			if err := wall.Step(); err != nil {
				return err
			}
			visits++
			return nil
		},
		ErrFn: wall.Err,
	}
	source, accepted, err := certifyLoftSourceSpanCaps(ctx, budget, prism, loft)
	if err != nil || !accepted {
		return no, false, err
	}
	order, accepted, err := certifyLoftCapSourceOrder(ctx, budget, source,
		&loftQuarterWork{budget: budget})
	if err != nil || !accepted {
		return no, false, err
	}
	wallOrder, accepted, err := certifyLoftQuarterWallOrder(ctx, budget, source)
	if err != nil || !accepted {
		return no, false, err
	}
	for cap := range order.quarter {
		if len(order.quarter[cap]) != 2 {
			return no, false, nil
		}
		seen := [2]bool{}
		for _, event := range order.quarter[cap] {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			if event.arc.segment != wallOrder.arcSegment {
				return no, false, nil
			}
			edge := -1
			switch event.line.segment {
			case wallOrder.lower:
				edge = 0
			case wallOrder.upper:
				edge = 1
			}
			if edge < 0 || seen[edge] {
				return no, false, nil
			}
			seen[edge] = true
		}
		if !seen[0] || !seen[1] {
			return no, false, nil
		}
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	return loftQuarterSourceEnvelope{order: order, wall: wallOrder, visits: visits}, true, nil
}

// loftQuarterWork debits both Sketch categories against the original source wall.
type loftQuarterWork struct {
	budget                  *proofbound.WorkBudget
	raw, proof, pairs       uint64
	cap                     int
	tags                    map[sketch.Entity]prismcells.Origin
	decline                 sketch.JunctionDecline
	declined                bool
	leftTag, rightTag       prismcells.Origin
	leftTagged, rightTagged bool
}

func (w *loftQuarterWork) JunctionDeclined(d sketch.JunctionDecline) {
	if w.declined {
		return
	}
	w.declined = true
	w.decline = d
	w.leftTag, w.leftTagged = w.tags[d.LeftEntity]
	w.rightTag, w.rightTagged = w.tags[d.RightEntity]
}

func (w *loftQuarterWork) Step(kind sketch.JunctionCharge) error {
	if w == nil || w.budget == nil {
		return sketch.ErrJunctionWorkLimit
	}
	switch kind {
	case sketch.JunctionRawVisit:
		if w.raw >= 896 {
			return sketch.ErrJunctionWorkLimit
		}
	case sketch.JunctionRootProof:
		if w.proof >= 2048 {
			return sketch.ErrJunctionWorkLimit
		}
	default:
		return sketch.ErrJunctionWorkLimit
	}
	if err := w.budget.Step(); err != nil {
		return err
	}
	if kind == sketch.JunctionRawVisit {
		w.raw++
	} else {
		w.proof++
	}
	return nil
}

func (w *loftQuarterWork) Err() error {
	if w == nil || w.budget == nil {
		return sketch.ErrJunctionWorkLimit
	}
	return w.budget.Err()
}

type loftQuarterSourceEvent struct {
	witness     sketch.QuarterArcJunction
	line, arc   loftSourceCapSpan
	appearances int
}

func loftQuarterRawLoop(p *sketch.Profile, loop int) []sketch.BoundaryEdge {
	if p == nil {
		return nil
	}
	if loop == -1 {
		return p.Outer
	}
	if loop < 0 || loop >= len(p.Holes) {
		return nil
	}
	return p.Holes[loop]
}

func loftQuarterPieceRange(l prismcells.ProfileLineage, loop, edge int) (
	prismcells.RawEdgePieces, bool) {
	entries := l.Outer
	if loop >= 0 {
		if loop >= len(l.Holes) {
			return prismcells.RawEdgePieces{}, false
		}
		entries = l.Holes[loop]
	}
	if edge < 0 || edge >= len(entries) {
		return prismcells.RawEdgePieces{}, false
	}
	return entries[edge], true
}

func mapLoftQuarterWitnesses(ctx context.Context, budget *proofbound.WorkBudget,
	raw, split []*sketch.Profile, lineage []prismcells.ProfileLineage,
	tags map[sketch.Entity]prismcells.Origin, source loftSourceSpanCaps, cap int,
	witnesses []sketch.QuarterArcJunction) ([]loftQuarterSourceEvent, bool, error) {
	if len(raw) != len(split) || len(raw) != len(lineage) {
		return nil, false, nil
	}
	out := make([]loftQuarterSourceEvent, 0, len(witnesses))
	for _, witness := range witnesses {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		occurrences := witness.Occurrences()
		if len(occurrences) == 0 {
			return nil, false, nil
		}
		event := loftQuarterSourceEvent{witness: witness}
		sources := witness.Sources()
		if sources[0].Kind == sources[1].Kind {
			return nil, false, nil
		}
		for _, at := range occurrences {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			if at.Profile < 0 || at.Profile >= len(raw) ||
				at.LeftEdge < 0 || at.RightEdge < 0 {
				return nil, false, nil
			}
			rawLoop := loftQuarterRawLoop(raw[at.Profile], at.Loop)
			splitLoop := loftQuarterRawLoop(split[at.Profile], at.Loop)
			if len(rawLoop) == 0 || at.LeftEdge >= len(rawLoop) ||
				at.RightEdge >= len(rawLoop) ||
				(at.LeftEdge+1)%len(rawLoop) != at.RightEdge {
				return nil, false, nil
			}
			left, right := rawLoop[at.LeftEdge], rawLoop[at.RightEdge]
			leftPieces, ok := loftQuarterPieceRange(lineage[at.Profile],
				at.Loop, at.LeftEdge)
			if !ok {
				return nil, false, nil
			}
			rightPieces, ok := loftQuarterPieceRange(lineage[at.Profile],
				at.Loop, at.RightEdge)
			if !ok || leftPieces.Last < 0 || leftPieces.Last >= len(splitLoop) ||
				rightPieces.First < 0 || rightPieces.First >= len(splitLoop) ||
				len(left.Polyline) < 2 || len(right.Polyline) < 2 ||
				len(splitLoop[leftPieces.Last].Polyline) < 2 ||
				len(splitLoop[rightPieces.First].Polyline) < 2 ||
				splitLoop[leftPieces.Last].Polyline[len(splitLoop[leftPieces.Last].Polyline)-1] !=
					left.Polyline[len(left.Polyline)-1] ||
				splitLoop[rightPieces.First].Polyline[0] != right.Polyline[0] {
				return nil, false, nil
			}
			lo, lok := tags[left.Entity]
			ro, rok := tags[right.Entity]
			if !lok || !rok || lo.IsB == ro.IsB ||
				lo.Region != 0 || ro.Region != 0 || lo.Hole != -1 || ro.Hole != -1 {
				return nil, false, nil
			}
			pair := [2]prismcells.Origin{lo, ro}
			edges := [2]sketch.BoundaryEdge{left, right}
			for i, origin := range pair {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				if !edges[i].Partial || !edges[i].TExact {
					return nil, false, nil
				}
				// Sources describes the representative walk. Other cells may
				// reach the same token with the opposite walked order or with
				// different fragment ranges.
				if event.appearances == 0 &&
					(edges[i].TStart != sources[i].TStart ||
						edges[i].TEnd != sources[i].TEnd ||
						edges[i].Reversed != sources[i].Reversed ||
						sources[i].AtWalkEnd != (i == 0)) {
					return nil, false, nil
				}
				if origin.IsB {
					_, isArc := edges[i].Entity.(*sketch.Arc)
					if !isArc || event.appearances == 0 &&
						sources[i].Kind != sketch.JunctionArc ||
						origin.Segment < 0 ||
						origin.Segment >= len(source.loftCaps[cap]) {
						return nil, false, nil
					}
					span := source.loftCaps[cap][origin.Segment]
					if _, ok := span.curve.(sectionrecord.ArcSeg); !ok {
						return nil, false, nil
					}
					if event.appearances != 0 && event.arc.segment != span.segment {
						return nil, false, nil
					}
					event.arc = span
				} else {
					_, isLine := edges[i].Entity.(*sketch.Line)
					if !isLine || event.appearances == 0 &&
						sources[i].Kind != sketch.JunctionLine ||
						origin.Segment < 0 ||
						origin.Segment >= len(source.prismCaps[cap]) {
						return nil, false, nil
					}
					span := source.prismCaps[cap][origin.Segment]
					if _, ok := span.curve.(sectionrecord.LineSeg); !ok {
						return nil, false, nil
					}
					if event.appearances != 0 && event.line.segment != span.segment {
						return nil, false, nil
					}
					event.line = span
				}
			}
			event.appearances++
		}
		out = append(out, event)
	}
	return out, true, nil
}

// quarterSourcePairs independently classifies each original cross-operand pair.
// The first class admits disjoint line pairs and a strictly convex quarter arc
// crossed by a line with one endpoint inside and the other outside its disk.
func quarterSourcePairs(budget *proofbound.WorkBudget, work *loftQuarterWork,
	source loftSourceSpanCaps, cap int, events []loftQuarterSourceEvent) (bool, error) {
	expected := make(map[[2]int]bool)
	for _, p := range source.prismCaps[cap] {
		for _, l := range source.loftCaps[cap] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			work.pairs++
			pLine, pOK := p.curve.(sectionrecord.LineSeg)
			if !pOK {
				return false, nil
			}
			switch lc := l.curve.(type) {
			case sectionrecord.LineSeg:
				if quarterLineBoxesOverlap(pLine, lc) {
					return false, nil
				}
			case sectionrecord.ArcSeg:
				if _, err := budget.StepN(16); err != nil {
					return false, err
				}
				root, ok := quarterLineArcPair(pLine, lc)
				if !ok {
					return false, nil
				}
				if root {
					expected[[2]int{p.segment, l.segment}] = true
				}
			default:
				return false, nil
			}
		}
	}
	if len(expected) != len(events) {
		return false, nil
	}
	for _, e := range events {
		key := [2]int{e.line.segment, e.arc.segment}
		if !expected[key] {
			return false, nil
		}
		delete(expected, key)
	}
	return len(expected) == 0, nil
}

func quarterLineBox(line sectionrecord.LineSeg) [4]float64 {
	minU, maxU := line.Start.U, line.End.U
	if minU > maxU {
		minU, maxU = maxU, minU
	}
	minV, maxV := line.Start.V, line.End.V
	if minV > maxV {
		minV, maxV = maxV, minV
	}
	return [4]float64{minU, minV, maxU, maxV}
}

func quarterBoxesOverlap(a, b [4]float64) bool {
	return !(a[2] < b[0] || b[2] < a[0] || a[3] < b[1] || b[3] < a[1])
}

func quarterLineBoxesOverlap(a, b sectionrecord.LineSeg) bool {
	return quarterBoxesOverlap(quarterLineBox(a), quarterLineBox(b))
}

func quarterLineArcPair(line sectionrecord.LineSeg,
	arc sectionrecord.ArcSeg) (bool, bool) {
	coords := [...]float64{line.Start.U, line.Start.V, line.End.U, line.End.V,
		arc.Center.U, arc.Center.V, arc.Start.U, arc.Start.V,
		arc.End.U, arc.End.V}
	var r [10]*big.Rat
	for i, v := range coords {
		r[i] = new(big.Rat).SetFloat64(v)
		if r[i] == nil || r[i].Num().BitLen() > 16 ||
			r[i].Denom().BitLen() > 16 {
			return false, false
		}
	}
	radius := new(big.Rat).Sub(r[6], r[4])
	if radius.Sign() <= 0 || r[7].Cmp(r[5]) != 0 ||
		r[8].Cmp(r[4]) != 0 ||
		radius.Cmp(new(big.Rat).Sub(r[9], r[5])) != 0 {
		return false, false
	}
	if !quarterBoxesOverlap(quarterLineBox(line),
		[4]float64{arc.Center.U, arc.Center.V, arc.Start.U, arc.End.V}) {
		return false, true
	}
	r2 := new(big.Rat).Mul(radius, radius)
	sign := func(x, y *big.Rat) int {
		dx := new(big.Rat).Sub(x, r[4])
		dy := new(big.Rat).Sub(y, r[5])
		d2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx),
			new(big.Rat).Mul(dy, dy))
		return d2.Cmp(r2)
	}
	a, b := sign(r[0], r[1]), sign(r[2], r[3])
	if a == 0 || b == 0 {
		return false, false
	}
	if a < 0 && b < 0 {
		return false, true
	}
	if a*b < 0 {
		return true, true
	}
	return false, false
}

// quarterSourceIntervals checks that every original cap segment has continuous
// arranged coverage over its complete natural [0,1] domain.
func quarterSourceIntervals(budget *proofbound.WorkBudget,
	source loftSourceSpanCaps, cap int, cells []loftCapSourceCell) (bool, error) {
	type key struct {
		loft    bool
		segment int
	}
	type span struct{ lo, hi float64 }
	all := make(map[key][]span)
	for _, cell := range cells {
		for _, edge := range cell.edges {
			if err := budget.Step(); err != nil {
				return false, err
			}
			k := key{loft: edge.span.body == source.loft, segment: edge.span.segment}
			if edge.span.body != source.loft && edge.span.body != source.prism {
				return false, nil
			}
			if len(all[k]) >= 128 {
				return false, nil
			}
			all[k] = append(all[k], span{edge.tStart, edge.tEnd})
		}
	}
	check := func(k key) (bool, error) {
		intervals := all[k]
		if len(intervals) == 0 {
			return false, nil
		}
		for i := 1; i < len(intervals); i++ {
			j := i
			for j > 0 {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if intervals[j-1].lo <= intervals[j].lo {
					break
				}
				intervals[j-1], intervals[j] = intervals[j], intervals[j-1]
				j--
			}
		}
		reach := float64(0)
		for _, v := range intervals {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if v.lo > reach || v.hi <= v.lo || v.hi > 1 {
				return false, nil
			}
			if v.hi > reach {
				reach = v.hi
			}
		}
		return reach == 1, nil
	}
	if len(all) != len(source.prismCaps[cap])+len(source.loftCaps[cap]) {
		return false, nil
	}
	for i := range source.prismCaps[cap] {
		ok, err := check(key{segment: i})
		if err != nil || !ok {
			return ok, err
		}
	}
	for i := range source.loftCaps[cap] {
		ok, err := check(key{loft: true, segment: i})
		if err != nil || !ok {
			return ok, err
		}
	}
	return true, nil
}
