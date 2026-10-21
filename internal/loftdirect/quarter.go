package loftdirect

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"

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

func QuarterRootBracketFits(xlo, xhi, ylo, yhi, square *big.Rat) bool {
	return quarterRootBracketFitsAtY(xlo, xhi, ylo, yhi, big.NewRat(1, 1), square)
}

// loftQuarterSourceHeldSelected is an inactive proof of selected arc seams.
// It does not certify a complete shell, volume, or public Union.
func loftQuarterSourceHeldSelected(ctx context.Context, first, second *topo.Body, selectedMask uint8,
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
	envelope, accepted, err := CertifyLoftQuarterSourceEnvelope(
		ctx, first, second, sourceBudget)
	if err != nil || !accepted {
		return no, false, err
	}
	wall := envelope.Wall
	if wall.Prism == nil || wall.Loft == nil || wall.Prism == wall.Loft ||
		wall.Lower < 0 || wall.Upper < 0 || wall.ArcSegment < 0 ||
		len(envelope.Order.Quarter[0]) != 2 ||
		len(envelope.Order.Quarter[1]) != 2 ||
		envelope.Order.source.Levels[0] == nil ||
		envelope.Order.source.Levels[1] == nil ||
		envelope.Order.source.Levels[2] == nil {
		return no, false, nil
	}
	for i, want := range [3]*big.Rat{big.NewRat(0, 1), big.NewRat(10, 1), big.NewRat(12, 1)} {
		if err := sourceBudget.Step(); err != nil {
			return no, false, err
		}
		if envelope.Order.source.Levels[i].Cmp(want) != 0 {
			return no, false, nil
		}
	}
	if wall.Lower == wall.Upper {
		return no, false, nil
	}
	wantSquares := [2][2]*big.Rat{
		{big.NewRat(24, 1), big.NewRat(16, 1)},
		{big.NewRat(117, 4), big.NewRat(85, 4)},
	}
	for capIndex, events := range envelope.Order.Quarter {
		var found [2]bool
		for _, event := range events {
			if err := sourceBudget.Step(); err != nil {
				return no, false, err
			}
			index := -1
			switch event.Line.Segment {
			case wall.Lower:
				index = 0
			case wall.Upper:
				index = 1
			}
			if index < 0 || selectedMask&uint8(1<<index) == 0 {
				continue
			}
			if found[index] || event.Line.Body != wall.Prism ||
				event.arc.Body != wall.Loft || event.arc.Segment != wall.ArcSegment ||
				event.Appearances == 0 {
				return no, false, nil
			}
			found[index] = true
			carrierY := 1 + 2*index
			wantQ := wantSquares[capIndex][index]
			line, isLine := event.Line.Curve.(sectionrecord.LineSeg)
			if !isLine || line.Start.V != float64(carrierY) || line.End.V != float64(carrierY) ||
				!((line.Start.U == 2 && line.End.U == 6) ||
					(line.Start.U == 6 && line.End.U == 2)) {
				return no, false, nil
			}
			x, y := event.Witness.Point()
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
		outputBudget, wall.Prism, wall.Loft, wall.ArcSegment, wall.Radius, selectedMask)
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

func LoftQuarterSourceHeldY1(ctx context.Context, first, second *topo.Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY1, bool, error) {
	pair, accepted, err := loftQuarterSourceHeldSelected(ctx, first, second, 1,
		sourceOperation, outputOperation)
	return pair.Y1, accepted, err
}

func LoftQuarterSourceHeldY3(ctx context.Context, first, second *topo.Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldY3, bool, error) {
	pair, accepted, err := loftQuarterSourceHeldSelected(ctx, first, second, 2,
		sourceOperation, outputOperation)
	return pair.Y3, accepted, err
}

// LoftQuarterSourceHeldPair certifies only the two selected arc-wall seams.
func LoftQuarterSourceHeldPair(ctx context.Context, first, second *topo.Body,
	sourceOperation, outputOperation *proofbound.WorkBudget) (
	loftcontact.QuarterSourceHeldPair, bool, error) {
	return loftQuarterSourceHeldSelected(ctx, first, second, 3,
		sourceOperation, outputOperation)
}

// loftQuarterSourceEnvelope relates certified cap crossings to the original
// Loft wall. It does not select output patches or prove occupied volume.
type loftQuarterSourceEnvelope struct {
	Order  loftCapSourceOrder
	Wall   loftQuarterWallOrderProof
	Visits uint64
}

// CertifyLoftQuarterSourceEnvelope reads live same-document producers in
// either operand order. All source stages share the unchanged 4096-step wall.
func CertifyLoftQuarterSourceEnvelope(ctx context.Context, first, second *topo.Body,
	operation *proofbound.WorkBudget) (loftQuarterSourceEnvelope, bool, error) {
	var no loftQuarterSourceEnvelope
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil || first == second ||
		first.Doc == nil || first.Doc != second.Doc {
		return no, false, nil
	}
	if !first.Doc.IsLive(first) || !first.Doc.IsLive(second) {
		return no, false, nil
	}
	prism, loft := first, second
	if _, ok := prism.Payload.(prismrecord.Payload); !ok {
		prism, loft = second, first
	}
	if _, ok := prism.Payload.(prismrecord.Payload); !ok {
		return no, false, nil
	}
	if _, ok := loft.Payload.(loftrecord.Payload); !ok {
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
	source, accepted, err := CertifyLoftSourceSpanCaps(ctx, budget, prism, loft)
	if err != nil || !accepted {
		return no, false, err
	}
	order, accepted, err := CertifyLoftCapSourceOrder(ctx, budget, source,
		&LoftQuarterWork{Budget: budget})
	if err != nil || !accepted {
		return no, false, err
	}
	wallOrder, accepted, err := CertifyLoftQuarterWallOrder(ctx, budget, source)
	if err != nil || !accepted {
		return no, false, err
	}
	for cap := range order.Quarter {
		if len(order.Quarter[cap]) != 2 {
			return no, false, nil
		}
		seen := [2]bool{}
		for _, event := range order.Quarter[cap] {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			if event.arc.Segment != wallOrder.ArcSegment {
				return no, false, nil
			}
			edge := -1
			switch event.Line.Segment {
			case wallOrder.Lower:
				edge = 0
			case wallOrder.Upper:
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
	return loftQuarterSourceEnvelope{Order: order, Wall: wallOrder, Visits: visits}, true, nil
}

// LoftQuarterWork debits both Sketch categories against the original source wall.
type LoftQuarterWork struct {
	Budget                  *proofbound.WorkBudget
	Raw, Proof, Pairs       uint64
	Cap                     int
	tags                    map[sketch.Entity]prismcells.Origin
	Decline                 sketch.JunctionDecline
	Declined                bool
	LeftTag, RightTag       prismcells.Origin
	LeftTagged, RightTagged bool
}

func (w *LoftQuarterWork) JunctionDeclined(d sketch.JunctionDecline) {
	if w.Declined {
		return
	}
	w.Declined = true
	w.Decline = d
	w.LeftTag, w.LeftTagged = w.tags[d.LeftEntity]
	w.RightTag, w.RightTagged = w.tags[d.RightEntity]
}

func (w *LoftQuarterWork) Step(kind sketch.JunctionCharge) error {
	if w == nil || w.Budget == nil {
		return sketch.ErrJunctionWorkLimit
	}
	switch kind {
	case sketch.JunctionRawVisit:
		if w.Raw >= 896 {
			return sketch.ErrJunctionWorkLimit
		}
	case sketch.JunctionRootProof:
		if w.Proof >= 2048 {
			return sketch.ErrJunctionWorkLimit
		}
	default:
		return sketch.ErrJunctionWorkLimit
	}
	if err := w.Budget.Step(); err != nil {
		return err
	}
	if kind == sketch.JunctionRawVisit {
		w.Raw++
	} else {
		w.Proof++
	}
	return nil
}

func (w *LoftQuarterWork) Err() error {
	if w == nil || w.Budget == nil {
		return sketch.ErrJunctionWorkLimit
	}
	return w.Budget.Err()
}

type loftQuarterSourceEvent struct {
	Witness     sketch.QuarterArcJunction
	Line, arc   loftSourceCapSpan
	Appearances int
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
	tags map[sketch.Entity]prismcells.Origin, source LoftSourceSpanCaps, capIndex int,
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
		event := loftQuarterSourceEvent{Witness: witness}
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
				if event.Appearances == 0 &&
					(edges[i].TStart != sources[i].TStart ||
						edges[i].TEnd != sources[i].TEnd ||
						edges[i].Reversed != sources[i].Reversed ||
						sources[i].AtWalkEnd != (i == 0)) {
					return nil, false, nil
				}
				if origin.IsB {
					_, isArc := edges[i].Entity.(*sketch.Arc)
					if !isArc || event.Appearances == 0 &&
						sources[i].Kind != sketch.JunctionArc ||
						origin.Segment < 0 ||
						origin.Segment >= len(source.LoftCaps[capIndex]) {
						return nil, false, nil
					}
					span := source.LoftCaps[capIndex][origin.Segment]
					if _, ok := span.Curve.(sectionrecord.ArcSeg); !ok {
						return nil, false, nil
					}
					if event.Appearances != 0 && event.arc.Segment != span.Segment {
						return nil, false, nil
					}
					event.arc = span
				} else {
					_, isLine := edges[i].Entity.(*sketch.Line)
					if !isLine || event.Appearances == 0 &&
						sources[i].Kind != sketch.JunctionLine ||
						origin.Segment < 0 ||
						origin.Segment >= len(source.PrismCaps[capIndex]) {
						return nil, false, nil
					}
					span := source.PrismCaps[capIndex][origin.Segment]
					if _, ok := span.Curve.(sectionrecord.LineSeg); !ok {
						return nil, false, nil
					}
					if event.Appearances != 0 && event.Line.Segment != span.Segment {
						return nil, false, nil
					}
					event.Line = span
				}
			}
			event.Appearances++
		}
		out = append(out, event)
	}
	return out, true, nil
}

// quarterSourcePairs adapts authenticated body spans to the plain-data pair
// classifier. Every original segment retains its source index.
func quarterSourcePairs(budget *proofbound.WorkBudget, work *LoftQuarterWork,
	source LoftSourceSpanCaps, capIndex int, events []loftQuarterSourceEvent) (bool, error) {
	prism := make([]sectionrecord.CurveSegment, len(source.PrismCaps[capIndex]))
	loft := make([]sectionrecord.CurveSegment, len(source.LoftCaps[capIndex]))
	for i, span := range source.PrismCaps[capIndex] {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if span.Body != source.Prism || span.Segment != i {
			return false, nil
		}
		prism[i] = span.Curve
	}
	for i, span := range source.LoftCaps[capIndex] {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if span.Body != source.Loft || span.Segment != i {
			return false, nil
		}
		loft[i] = span.Curve
	}
	pairs := make([][2]int, len(events))
	for i, event := range events {
		if err := budget.Step(); err != nil {
			return false, err
		}
		pairs[i] = [2]int{event.Line.Segment, event.arc.Segment}
	}
	visited, accepted, err := loftcontact.QuarterSourcePairs(
		budget, prism, loft, pairs)
	work.Pairs += visited
	return accepted, err
}

// quarterSourceIntervals checks that every original cap segment has continuous
// arranged coverage over its complete natural [0,1] domain.
func quarterSourceIntervals(budget *proofbound.WorkBudget,
	source LoftSourceSpanCaps, capIndex int, cells []loftCapSourceCell) (bool, error) {
	var fragments []loftcontact.QuarterSourceInterval
	for _, cell := range cells {
		for _, edge := range cell.Edges {
			if err := budget.Step(); err != nil {
				return false, err
			}
			isLoft := edge.Span.Body == source.Loft
			if !isLoft && edge.Span.Body != source.Prism {
				return false, nil
			}
			fragments = append(fragments, loftcontact.QuarterSourceInterval{
				Loft: isLoft, Segment: edge.Span.Segment,
				Start: edge.TStart, End: edge.TEnd,
			})
		}
	}
	return loftcontact.QuarterSourceIntervals(budget,
		len(source.PrismCaps[capIndex]), len(source.LoftCaps[capIndex]), fragments)
}

// loftQuarterWallOrderProof is a private source-only reading. It identifies
// the two rectangle edges crossing one paired quarter-arc wall at every
// height. It does not select output patches or certify held-mesh coverage.
type loftQuarterWallOrderProof struct {
	Prism, Loft  *topo.Body
	ArcSegment   int
	Lower, Upper int // source indices of the rectangle's lower and upper edges
	Radius       [2]*big.Rat
	WallFaces    [2][]provenance.Ref
}

// CertifyLoftQuarterWallOrder accepts only a concentric, first-quadrant
// quarter-arc pair and one axis-aligned rectangle. The caller passes the
// authenticated source spans and their original 4096-step source wall.
func CertifyLoftQuarterWallOrder(ctx context.Context, budget *proofbound.WorkBudget,
	source LoftSourceSpanCaps) (loftQuarterWallOrderProof, bool, error) {
	var no loftQuarterWallOrderProof
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if budget == nil || source.Prism == nil || source.Loft == nil ||
		source.Prism == source.Loft {
		return no, false, nil
	}
	pp, prismOK := source.Prism.Payload.(prismrecord.Payload)
	lp, loftOK := source.Loft.Payload.(loftrecord.Payload)
	if !prismOK || !loftOK || !lp.AuthenticatedReconstruction ||
		lp.SourceIdentity == nil || lp.SourceIdentity.Origin != source.Loft.Origin.Producer ||
		len(lp.Alignment) > 1 || len(lp.Alignment) == 1 && lp.Alignment[0] != 0 ||
		len(pp.Profile.Outer.Segments) != 4 ||
		len(lp.Profile0.Outer.Segments) != 3 || len(lp.Profile1.Outer.Segments) != 3 ||
		len(lp.Verts) > 512 || len(lp.Tris) > 512 ||
		len(source.PrismCaps[0]) != 4 || len(source.PrismCaps[1]) != 4 ||
		len(source.LoftCaps[0]) != 3 || len(source.LoftCaps[1]) != 3 ||
		source.Levels[0] == nil || source.Levels[1] == nil || source.Levels[2] == nil {
		return no, false, nil
	}
	checked := &loftcontact.Arithmetic{OK: true}
	if checked.Cmp(source.Levels[0], big.NewRat(0, 1)) != 0 ||
		checked.Cmp(source.Levels[0], source.Levels[1]) >= 0 ||
		checked.Cmp(source.Levels[1], source.Levels[2]) >= 0 || !checked.OK {
		return no, false, nil
	}
	rect, edges, ok, err := loftQuarterWallRectangle(ctx, budget, source, pp)
	if err != nil || !ok {
		return no, false, err
	}
	proof := loftQuarterWallOrderProof{Prism: source.Prism, Loft: source.Loft,
		Lower: edges[0], Upper: edges[1], ArcSegment: -1}
	for cap, loop := range [2]sectionrecord.LoopRecord{lp.Profile0.Outer, lp.Profile1.Outer} {
		arcIndex := -1
		axisLines := [2]bool{}
		for i, raw := range loop.Segments {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			span := source.LoftCaps[cap][i]
			if span.Body != source.Loft || span.Segment != i {
				return no, false, nil
			}
			switch segment := raw.(type) {
			case sectionrecord.ArcSeg:
				borrowed, same := span.Curve.(sectionrecord.ArcSeg)
				if !same || borrowed != segment || arcIndex >= 0 ||
					segment.TStart != 0 || segment.TEnd != 1 ||
					segment.Center.U != 0 || segment.Center.V != 0 ||
					proofbound.IsNonFinite(segment.Start.U) ||
					segment.Start.U <= 0 || segment.Start.V != 0 ||
					segment.End.U != 0 || segment.End.V != segment.Start.U ||
					len(span.WallFaces) == 0 {
					return no, false, nil
				}
				arcIndex = i
				proof.Radius[cap] = proofarith.FloatRat(segment.Start.U)
				proof.WallFaces[cap] = span.WallFaces
			case sectionrecord.LineSeg:
				borrowed, same := span.Curve.(sectionrecord.LineSeg)
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
			cap == 1 && arcIndex != proof.ArcSegment {
			return no, false, nil
		}
		if cap == 1 {
			if len(lp.Segment) != 1 || len(lp.Segment[0]) > 512 ||
				len(proof.WallFaces[0]) != len(proof.WallFaces[1]) {
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
				if matched >= len(proof.WallFaces[0]) {
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
					if proof.WallFaces[half][matched] != expected {
						return no, false, nil
					}
				}
				matched++
			}
			if matched == 0 || matched != len(proof.WallFaces[0]) {
				return no, false, nil
			}
		}
		proof.ArcSegment = arcIndex
	}
	// The authenticated Loft pairs corresponding source parameters by a
	// straight rule. Matching quarter arcs therefore have radius bounded by
	// the two endpoint radii at every intervening height.
	if _, err := budget.StepN(12); err != nil {
		return no, false, err
	}
	rMin, rMax := proof.Radius[0], proof.Radius[1]
	if checked.Cmp(rMin, rMax) > 0 {
		rMin, rMax = rMax, rMin
	}
	minSquare := checked.Mul(rMin, rMin)
	maxSquare := checked.Mul(rMax, rMax)
	xMinSquare := checked.Mul(rect[0], rect[0])
	xMaxSquare := checked.Mul(rect[1], rect[1])
	yMinSquare := checked.Mul(rect[2], rect[2])
	yMaxSquare := checked.Mul(rect[3], rect[3])
	leftUpper := checked.Add(xMinSquare, yMaxSquare)
	rightLower := checked.Add(xMaxSquare, yMinSquare)
	if !checked.OK || checked.Cmp(minSquare, leftUpper) <= 0 ||
		checked.Cmp(maxSquare, rightLower) >= 0 || !checked.OK {
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
	source LoftSourceSpanCaps, pp prismrecord.Payload) ([4]*big.Rat, [2]int, bool, error) {
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
			span := source.PrismCaps[cap][i]
			borrowed, same := span.Curve.(sectionrecord.LineSeg)
			if !same || borrowed != line || span.Body != source.Prism || span.Segment != i {
				return bounds, horizontal, false, nil
			}
		}
		lines[i] = line
	}
	bounds, horizontal, ok, err := loftcontact.QuarterWallRectangle(budget, lines[:])
	if err != nil || !ok {
		return bounds, horizontal, ok, err
	}
	checked := &loftcontact.Arithmetic{OK: true}
	for _, value := range bounds {
		checked.Valid(value)
	}
	return bounds, horizontal, checked.OK, nil
}

func loopArcRadius(loop sectionrecord.LoopRecord) float64 {
	for _, raw := range loop.Segments {
		if arc, ok := raw.(sectionrecord.ArcSeg); ok {
			return arc.Start.U
		}
	}
	return 0
}
