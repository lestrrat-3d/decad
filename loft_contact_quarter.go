package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

func quarterRootComparisonFits(a, b *big.Rat) bool {
	return a != nil && b != nil &&
		a.Num().BitLen() <= 8192 && a.Denom().BitLen() <= 8192 &&
		b.Num().BitLen() <= 8192 && b.Denom().BitLen() <= 8192 &&
		a.Num().BitLen()+b.Denom().BitLen() <= 8192 &&
		b.Num().BitLen()+a.Denom().BitLen() <= 8192
}

func quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, carrier, square *big.Rat) bool {
	if !quarterRootComparisonFits(xlo, xhi) ||
		!quarterRootComparisonFits(xlo, big.NewRat(2, 1)) ||
		!quarterRootComparisonFits(xhi, big.NewRat(6, 1)) ||
		!quarterRootComparisonFits(ylo, carrier) ||
		!quarterRootComparisonFits(yhi, carrier) || square == nil {
		return false
	}
	for _, x := range []*big.Rat{xlo, xhi} {
		if 2*x.Num().BitLen() > 8192 || 2*x.Denom().BitLen() > 8192 ||
			2*x.Num().BitLen()+square.Denom().BitLen() > 8192 ||
			square.Num().BitLen()+2*x.Denom().BitLen() > 8192 {
			return false
		}
	}
	return true
}

func quarterRootBracketFits(xlo, xhi, ylo, yhi, square *big.Rat) bool {
	return quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, big.NewRat(1, 1), square)
}

// loftQuarterSourceHeldSelected is an inactive proof of selected arc seams.
// It does not certify a complete shell, volume, or public Union.
func loftQuarterSourceHeldSelected(ctx context.Context, first, second *Body, selectedMask uint8,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldPair, bool, error) {
	var no loftcontact.QuarterSourceHeldPair
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if (selectedMask == 0 || selectedMask > 3) || sourceOperation == nil || outputOperation == nil {
		return no, false, nil
	}
	withContext := func(operation *proofbound.WorkBudget) *proofbound.WorkBudget {
		return &proofbound.WorkBudget{
			StepFn: func() error {
				if err := ctx.Err(); err != nil {
					return err
				}
				return operation.Step()
			},
			ErrFn: func() error {
				if err := ctx.Err(); err != nil {
					return err
				}
				return operation.Err()
			},
		}
	}
	sourceBudget := proofbound.NewWallWorkBudgetWithOperation(4096, withContext(sourceOperation))
	outputBudget := proofbound.NewWallWorkBudgetWithOperation(65_536, withContext(outputOperation))
	envelope, accepted, err := certifyLoftQuarterSourceEnvelope(
		ctx, first, second, sourceBudget)
	if err != nil || !accepted {
		return no, false, err
	}
	wall := envelope.wall
	if wall.prism == nil || wall.loft == nil || wall.prism == wall.loft ||
		wall.lower < 0 || wall.upper < 0 || wall.arcSegment < 0 ||
		len(envelope.order.quarter[0]) != 2 ||
		len(envelope.order.quarter[1]) != 2 ||
		envelope.order.source.levels[0] == nil ||
		envelope.order.source.levels[1] == nil ||
		envelope.order.source.levels[2] == nil {
		return no, false, nil
	}
	for i, want := range [3]*big.Rat{big.NewRat(0, 1), big.NewRat(10, 1), big.NewRat(12, 1)} {
		if err := sourceBudget.Step(); err != nil {
			return no, false, err
		}
		if envelope.order.source.levels[i].Cmp(want) != 0 {
			return no, false, nil
		}
	}
	if wall.lower == wall.upper {
		return no, false, nil
	}
	wantSquares := [2][2]*big.Rat{
		{big.NewRat(24, 1), big.NewRat(16, 1)},
		{big.NewRat(117, 4), big.NewRat(85, 4)},
	}
	for capIndex, events := range envelope.order.quarter {
		var found [2]bool
		for _, event := range events {
			if err := sourceBudget.Step(); err != nil {
				return no, false, err
			}
			index := -1
			switch event.line.segment {
			case wall.lower:
				index = 0
			case wall.upper:
				index = 1
			}
			if index < 0 || selectedMask&uint8(1<<index) == 0 {
				continue
			}
			if found[index] || event.line.body != wall.prism ||
				event.arc.body != wall.loft || event.arc.segment != wall.arcSegment ||
				event.appearances == 0 {
				return no, false, nil
			}
			found[index] = true
			carrierY := 1 + 2*index
			wantQ := wantSquares[capIndex][index]
			line, isLine := event.line.curve.(sectionrecord.LineSeg)
			if !isLine || line.Start.V != float64(carrierY) || line.End.V != float64(carrierY) ||
				!((line.Start.U == 2 && line.End.U == 6) ||
					(line.Start.U == 6 && line.End.U == 2)) {
				return no, false, nil
			}
			x, y := event.witness.Point()
			xlo, xhi := x.Bounds()
			ylo, yhi := y.Bounds()
			carrier := big.NewRat(int64(carrierY), 1)
			if !quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, carrier, wantQ) {
				return no, false, nil
			}
			if err := sourceBudget.Step(); err != nil {
				return no, false, err
			}
			if xlo.Sign() <= 0 || xlo.Cmp(xhi) > 0 {
				return no, false, nil
			}
			if _, err := sourceBudget.StepN(6); err != nil {
				return no, false, err
			}
			if xlo.Cmp(big.NewRat(2, 1)) <= 0 || xhi.Cmp(big.NewRat(6, 1)) >= 0 ||
				ylo.Cmp(carrier) > 0 || yhi.Cmp(carrier) < 0 ||
				new(big.Rat).Mul(xlo, xlo).Cmp(wantQ) > 0 ||
				new(big.Rat).Mul(xhi, xhi).Cmp(wantQ) < 0 {
				return no, false, nil
			}
		}
		for index := range found {
			if selectedMask&uint8(1<<index) != 0 && !found[index] {
				return no, false, nil
			}
		}
	}
	if err := sourceBudget.Err(); err != nil {
		return no, false, err
	}
	result, accepted, err := loftcontact.CertifyQuarterSourceHeldSelected(ctx,
		outputBudget, &wall.prism.n, &wall.loft.n, wall.arcSegment, wall.radius, selectedMask)
	if err != nil || !accepted {
		return no, false, err
	}
	if err := sourceBudget.Err(); err != nil {
		return no, false, err
	}
	if err := outputBudget.Err(); err != nil {
		return no, false, err
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	return result, true, nil
}

func loftQuarterSourceHeldY1(ctx context.Context, first, second *Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY1, bool, error) {
	pair, accepted, err := loftQuarterSourceHeldSelected(ctx, first, second, 1,
		sourceOperation, outputOperation)
	return pair.Y1, accepted, err
}

func loftQuarterSourceHeldY3(ctx context.Context, first, second *Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY3, bool, error) {
	pair, accepted, err := loftQuarterSourceHeldSelected(ctx, first, second, 2,
		sourceOperation, outputOperation)
	return pair.Y3, accepted, err
}

// loftQuarterSourceHeldPair certifies only the two selected arc-wall seams.
func loftQuarterSourceHeldPair(ctx context.Context, first, second *Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldPair, bool, error) {
	return loftQuarterSourceHeldSelected(ctx, first, second, 3,
		sourceOperation, outputOperation)
}

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
	if !first.n.Doc.IsLive(&first.n) || !first.n.Doc.IsLive(&second.n) {
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
	tags map[sketch.Entity]prismcells.Origin, source loftSourceSpanCaps, capIndex int,
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
						origin.Segment >= len(source.loftCaps[capIndex]) {
						return nil, false, nil
					}
					span := source.loftCaps[capIndex][origin.Segment]
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
						origin.Segment >= len(source.prismCaps[capIndex]) {
						return nil, false, nil
					}
					span := source.prismCaps[capIndex][origin.Segment]
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

// quarterSourcePairs adapts authenticated body spans to the plain-data pair
// classifier. Every original segment retains its source index.
func quarterSourcePairs(budget *proofbound.WorkBudget, work *loftQuarterWork,
	source loftSourceSpanCaps, capIndex int, events []loftQuarterSourceEvent) (bool, error) {
	prism := make([]sectionrecord.CurveSegment, len(source.prismCaps[capIndex]))
	loft := make([]sectionrecord.CurveSegment, len(source.loftCaps[capIndex]))
	for i, span := range source.prismCaps[capIndex] {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if span.body != source.prism || span.segment != i {
			return false, nil
		}
		prism[i] = span.curve
	}
	for i, span := range source.loftCaps[capIndex] {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if span.body != source.loft || span.segment != i {
			return false, nil
		}
		loft[i] = span.curve
	}
	pairs := make([][2]int, len(events))
	for i, event := range events {
		if err := budget.Step(); err != nil {
			return false, err
		}
		pairs[i] = [2]int{event.line.segment, event.arc.segment}
	}
	visited, accepted, err := loftcontact.QuarterSourcePairs(
		budget, prism, loft, pairs)
	work.pairs += visited
	return accepted, err
}

// quarterSourceIntervals checks that every original cap segment has continuous
// arranged coverage over its complete natural [0,1] domain.
func quarterSourceIntervals(budget *proofbound.WorkBudget,
	source loftSourceSpanCaps, capIndex int, cells []loftCapSourceCell) (bool, error) {
	var fragments []loftcontact.QuarterSourceInterval
	for _, cell := range cells {
		for _, edge := range cell.edges {
			if err := budget.Step(); err != nil {
				return false, err
			}
			isLoft := edge.span.body == source.loft
			if !isLoft && edge.span.body != source.prism {
				return false, nil
			}
			fragments = append(fragments, loftcontact.QuarterSourceInterval{
				Loft: isLoft, Segment: edge.span.segment,
				Start: edge.tStart, End: edge.tEnd,
			})
		}
	}
	return loftcontact.QuarterSourceIntervals(budget,
		len(source.prismCaps[capIndex]), len(source.loftCaps[capIndex]), fragments)
}

// loftQuarterWallOrderProof is a private source-only reading. It identifies
// the two rectangle edges crossing one paired quarter-arc wall at every
// height. It does not select output patches or certify held-mesh coverage.
type loftQuarterWallOrderProof struct {
	prism, loft  *Body
	arcSegment   int
	lower, upper int // source indices of the rectangle's lower and upper edges
	radius       [2]*big.Rat
	wallFaces    [2][]provenance.Ref
}

// certifyLoftQuarterWallOrder accepts only a concentric, first-quadrant
// quarter-arc pair and one axis-aligned rectangle. The caller passes the
// authenticated source spans and their original 4096-step source wall.
func certifyLoftQuarterWallOrder(ctx context.Context, budget *proofbound.WorkBudget,
	source loftSourceSpanCaps) (loftQuarterWallOrderProof, bool, error) {
	var no loftQuarterWallOrderProof
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if budget == nil || source.prism == nil || source.loft == nil ||
		source.prism == source.loft {
		return no, false, nil
	}
	pp, prismOK := source.prism.n.Payload.(prismPayload)
	lp, loftOK := source.loft.n.Payload.(loftPayload)
	if !prismOK || !loftOK || !lp.AuthenticatedReconstruction ||
		lp.SourceIdentity == nil || lp.SourceIdentity.Origin != source.loft.n.Origin.Producer ||
		len(lp.Alignment) > 1 || len(lp.Alignment) == 1 && lp.Alignment[0] != 0 ||
		len(pp.Profile.Outer.Segments) != 4 ||
		len(lp.Profile0.Outer.Segments) != 3 || len(lp.Profile1.Outer.Segments) != 3 ||
		len(lp.Verts) > 512 || len(lp.Tris) > 512 ||
		len(source.prismCaps[0]) != 4 || len(source.prismCaps[1]) != 4 ||
		len(source.loftCaps[0]) != 3 || len(source.loftCaps[1]) != 3 ||
		source.levels[0] == nil || source.levels[1] == nil || source.levels[2] == nil {
		return no, false, nil
	}
	checked := &loftVolumeArithmetic{ok: true}
	if checked.cmp(source.levels[0], big.NewRat(0, 1)) != 0 ||
		checked.cmp(source.levels[0], source.levels[1]) >= 0 ||
		checked.cmp(source.levels[1], source.levels[2]) >= 0 || !checked.ok {
		return no, false, nil
	}
	rect, edges, ok, err := loftQuarterWallRectangle(ctx, budget, source, pp)
	if err != nil || !ok {
		return no, false, err
	}
	proof := loftQuarterWallOrderProof{prism: source.prism, loft: source.loft,
		lower: edges[0], upper: edges[1], arcSegment: -1}
	for cap, loop := range [2]sectionrecord.LoopRecord{lp.Profile0.Outer, lp.Profile1.Outer} {
		arcIndex := -1
		axisLines := [2]bool{}
		for i, raw := range loop.Segments {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			span := source.loftCaps[cap][i]
			if span.body != source.loft || span.segment != i {
				return no, false, nil
			}
			switch segment := raw.(type) {
			case sectionrecord.ArcSeg:
				borrowed, same := span.curve.(sectionrecord.ArcSeg)
				if !same || borrowed != segment || arcIndex >= 0 ||
					segment.TStart != 0 || segment.TEnd != 1 ||
					segment.Center.U != 0 || segment.Center.V != 0 ||
					proofbound.IsNonFinite(segment.Start.U) ||
					segment.Start.U <= 0 || segment.Start.V != 0 ||
					segment.End.U != 0 || segment.End.V != segment.Start.U ||
					len(span.wallFaces) == 0 {
					return no, false, nil
				}
				arcIndex = i
				proof.radius[cap] = proofarith.FloatRat(segment.Start.U)
				proof.wallFaces[cap] = span.wallFaces
			case sectionrecord.LineSeg:
				borrowed, same := span.curve.(sectionrecord.LineSeg)
				if !same || borrowed != segment ||
					!loftcontact.QuarterWallAxisLine(segment, proofarith.FloatRat(loopArcRadius(loop))) {
					return no, false, nil
				}
				if segment.Start.V == 0 && segment.End.V == 0 {
					if axisLines[0] {
						return no, false, nil
					}
					axisLines[0] = true
				} else {
					if axisLines[1] {
						return no, false, nil
					}
					axisLines[1] = true
				}
			default:
				return no, false, nil
			}
		}
		if arcIndex < 0 || !axisLines[0] || !axisLines[1] ||
			cap == 1 && arcIndex != proof.arcSegment {
			return no, false, nil
		}
		if cap == 1 {
			if len(lp.Segment) != 1 || len(lp.Segment[0]) > 512 ||
				len(proof.wallFaces[0]) != len(proof.wallFaces[1]) {
				return no, false, nil
			}
			matched := 0
			for cell, segment := range lp.Segment[0] {
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				if segment != arcIndex {
					continue
				}
				if matched >= len(proof.wallFaces[0]) {
					return no, false, nil
				}
				for half := range 2 {
					if err := budget.Step(); err != nil {
						return no, false, err
					}
					expected := provenance.Ref{
						Producer: lp.SourceIdentity.Origin,
						Role:     fmt.Sprintf("side(0,%d,%d)", cell, half),
					}
					if proof.wallFaces[half][matched] != expected {
						return no, false, nil
					}
				}
				matched++
			}
			if matched == 0 || matched != len(proof.wallFaces[0]) {
				return no, false, nil
			}
		}
		proof.arcSegment = arcIndex
	}
	// The authenticated Loft pairs corresponding source parameters by a
	// straight rule. Matching quarter arcs therefore have radius bounded by
	// the two endpoint radii at every intervening height.
	if _, err := budget.StepN(12); err != nil {
		return no, false, err
	}
	rMin, rMax := proof.radius[0], proof.radius[1]
	if checked.cmp(rMin, rMax) > 0 {
		rMin, rMax = rMax, rMin
	}
	minSquare := checked.mul(rMin, rMin)
	maxSquare := checked.mul(rMax, rMax)
	xMinSquare := checked.mul(rect[0], rect[0])
	xMaxSquare := checked.mul(rect[1], rect[1])
	yMinSquare := checked.mul(rect[2], rect[2])
	yMaxSquare := checked.mul(rect[3], rect[3])
	leftUpper := checked.add(xMinSquare, yMaxSquare)
	rightLower := checked.add(xMaxSquare, yMinSquare)
	if !checked.ok || checked.cmp(minSquare, leftUpper) <= 0 ||
		checked.cmp(maxSquare, rightLower) >= 0 || !checked.ok {
		return no, false, nil
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	return proof, true, nil
}

func loftQuarterWallRectangle(ctx context.Context, budget *proofbound.WorkBudget,
	source loftSourceSpanCaps, pp prismPayload) ([4]*big.Rat, [2]int, bool, error) {
	var bounds [4]*big.Rat
	var horizontal [2]int
	if len(pp.Profile.Outer.Segments) != 4 {
		return bounds, horizontal, false, nil
	}
	var lines [4]sectionrecord.LineSeg
	for i, raw := range pp.Profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return bounds, horizontal, false, err
		}
		if err := budget.Step(); err != nil {
			return bounds, horizontal, false, err
		}
		line, ok := raw.(sectionrecord.LineSeg)
		if !ok {
			return bounds, horizontal, false, nil
		}
		for cap := range 2 {
			span := source.prismCaps[cap][i]
			borrowed, same := span.curve.(sectionrecord.LineSeg)
			if !same || borrowed != line || span.body != source.prism || span.segment != i {
				return bounds, horizontal, false, nil
			}
		}
		lines[i] = line
	}
	bounds, horizontal, ok, err := loftcontact.QuarterWallRectangle(budget, lines[:])
	if err != nil || !ok {
		return bounds, horizontal, ok, err
	}
	checked := &loftVolumeArithmetic{ok: true}
	for _, value := range bounds {
		checked.valid(value)
	}
	return bounds, horizontal, checked.ok, nil
}

func loopArcRadius(loop sectionrecord.LoopRecord) float64 {
	for _, raw := range loop.Segments {
		if arc, ok := raw.(sectionrecord.ArcSeg); ok {
			return arc.Start.U
		}
	}
	return 0
}
