package prismboolean

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
)

// This file is the extended class of the one-scene prism Union
// (agent-docs/prism-boolean-design.md Part K §5.3): whole line, circle and arc
// sections, holed operands and displaced operands that carry a valid carrier
// record, all on one frame and placement over one sweep interval. Every
// failure after admission is a miss, so the set takes the next Union route;
// only cancellation returns an error.

// lineClass reports whether the set belongs to the original one-scene class
// (prism-boolean §4.6), which keeps its own behaviour and refusals.
func lineClass(operands []*topo.Body) bool {
	for _, operand := range operands {
		p, ok := operand.Payload.(prismrecord.Payload)
		if !ok || p.SectionDelta != 0 || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) == 0 {
			return false
		}
		for _, segment := range p.Profile.Outer.Segments {
			line, ok := segment.(sectionrecord.LineSeg)
			if !ok || !prismcells.WholeSegmentRange(line.TStart, line.TEnd) {
				return false
			}
		}
	}
	return true
}

// unionRegions is the extended class's evaluation. matched=false (err nil)
// is a miss.
func unionRegions(ctx context.Context, env topo.Env, operands []*topo.Body) (*topo.Body, bool, error) {
	body, ok, err := unionRegionsTry(ctx, env, operands)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil, true, err
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, true, ctxErr
		}
		// Every other failure, a *prismcells.WorkLimitError (the recount over
		// the cap) included, is a silent miss to the next Union route.
		return nil, false, nil
	}
	return body, ok, nil
}

func unionRegionsTry(ctx context.Context, env topo.Env, operands []*topo.Body) (*topo.Body, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	payloads := make([]prismrecord.Payload, len(operands))
	records := make([]prismrecord.CarrierRecord, len(operands))
	profiles := make([]momentinput.Profile, len(operands))
	z0Delta, z1Delta, incoming := 0.0, 0.0, 0.0
	displaced := false
	for i, operand := range operands {
		p, ok := operand.Payload.(prismrecord.Payload)
		if !ok || p.SurfaceResult || len(p.Profile.Outer.Segments) == 0 {
			return nil, false, nil
		}
		// U1: one frame, one placement, equal sweep intervals.
		if i > 0 && (p.Frame != payloads[0].Frame || p.Xform != payloads[0].Xform ||
			!prismplacement.UnionZIntervalMatches(payloads[0].PlacementOperand(), p.PlacementOperand())) {
			return nil, false, nil
		}
		// U2: every segment whole.
		for _, loop := range append([]sectionrecord.LoopRecord{p.Profile.Outer}, p.Profile.Holes...) {
			for _, segment := range loop.Segments {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				if !wholeAnalytic(segment) {
					return nil, false, nil
				}
			}
		}
		// U3: no displacement, or a valid record.
		cr, valid, err := prismrecord.CarrierRecordOf(budget, operand)
		if err != nil || !valid {
			return nil, false, err
		}
		payloads[i], records[i], profiles[i] = p, cr, p.Profile
		displaced = displaced || cr.Max() > 0
		incoming = math.Max(incoming, p.SectionDelta)
		z0Delta = max(z0Delta, p.Z0Delta)
		z1Delta = max(z1Delta, p.Z1Delta)
	}
	first := payloads[0]
	// U4 (screen), U5a.
	if _, withinCap, err := prismcells.RegionsWithinWorkCap(budget, profiles...); err != nil || !withinCap {
		return nil, false, err
	}
	if err := distinctLineCarriers(budget, profiles); err != nil {
		return nil, false, err
	}
	s, tags, charge, err := prismcells.BuildSceneRegions(budget, prismcells.SceneProfiles(profiles), nil,
		&prismcells.Reexpression{Identity: true})
	if err != nil {
		return nil, false, err // a *WorkLimitError (U4's recount) is a miss in unionRegions
	}
	if charge.A != 0 || charge.B != 0 {
		return nil, false, nil
	}
	cells, err := prismcells.ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, false, err
	}
	cells, resolved, err := prismcells.SplitRuns(budget, cells)
	if err != nil || !resolved || len(cells) == 0 {
		return nil, false, err
	}
	// U5b: a withdrawn span is two near-identical carriers sketch merged.
	shared, ok, err := prismcells.CoincidentEdgesRegions(budget, tags, cells)
	if err != nil || !ok || len(shared.Spans) != 0 {
		return nil, false, err
	}
	member, resolved, err := prismcells.ClassifyRegions(budget, tags, cells)
	if err != nil || !resolved {
		return nil, false, err
	}
	var selected []*sketch.Profile
	for i, cell := range cells {
		for _, m := range member {
			if m[i] {
				selected = append(selected, cell)
				break
			}
		}
	}
	if len(selected) == 0 {
		return nil, false, nil
	}
	// V1 and the vertex charge.
	vr, ok, err := prismcells.ReadVertices(budget, tags, cells, records)
	if err != nil || !ok {
		return nil, false, err
	}
	if displaced {
		arr, ok, err := prismcells.NewArrangement(budget, tags, cells, records, vr)
		if err != nil || !ok {
			return nil, false, err
		}
		floor, err := sceneFloor(budget, profiles)
		if err != nil {
			return nil, false, err
		}
		// V2, V3.
		if stable, err := arr.Stable(budget, floor); err != nil || !stable {
			return nil, false, err
		}
		if ordered, err := arr.Ordered(budget); err != nil || !ordered {
			return nil, false, err
		}
	}
	loops, chains, cutDelta, resolved, err := prismcells.MergeLoopsWithChains(budget, selected, "union")
	if err != nil || !resolved || len(loops) == 0 {
		return nil, false, err
	}
	crossing := 0.0
	for _, v := range vr.Vertices {
		crossing = math.Max(crossing, v.Delta)
	}
	entries := make([]loopEntries, len(loops))
	for i := range loops {
		var entriesOK bool
		entries[i], entriesOK = chainEntries(tags, records, vr, chains[i], loops[i])
		if !entriesOK {
			return nil, false, nil
		}
	}
	order, ok, err := chooseOuter(budget, loops, entries)
	if err != nil || !ok {
		return nil, false, err
	}
	profile := momentinput.Profile{Outer: loops[order[0]]}
	for _, i := range order[1:] {
		profile.Holes = append(profile.Holes, loops[i])
	}
	sectionDelta := proofbound.AbsSumUpper(math.Max(incoming, crossing), cutDelta)
	record := prismrecord.ZeroCarrierRecord(profile)
	for l, i := range order {
		record.Segment[l], record.Junction[l] = entries[i].segment, entries[i].junction
	}
	// CR4: the result's displacement is at least every loop's h_loop.
	hMax := record.HMax()
	if proofbound.IsNonFinite(hMax) {
		return nil, false, nil
	}
	// S9's candidate-point verdict (§6.5) rests on a certified gap: with any
	// operand displaced, the merged profile passes the certified contact test
	// before the audit's S9 is trusted.
	if displaced {
		merged := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
		if apart, _, err := prismcells.CertifiedContact(budget, merged, hMax); err != nil || !apart {
			return nil, false, err
		}
	}
	if err := prismcells.AuditMergeSection(budget, first, profile); err != nil {
		return nil, false, err
	}
	sectionDelta = math.Max(sectionDelta, hMax)
	record.SectionDelta = sectionDelta
	if record.Max() > sectionDelta {
		return nil, false, nil
	}
	z0Denoted := prismplacement.DenoteLevel(first.DenotedZ0(), first.Z0, z0Delta)
	z1Denoted := prismplacement.DenoteLevel(first.DenotedZ1(), first.Z1, z1Delta)
	result := prismrecord.Payload{Profile: profile, Frame: first.Frame, Xform: first.Xform,
		Z0: first.Z0, Z1: first.Z1, Z0Delta: z0Delta, Z1Delta: z1Delta,
		Z0Denoted: z0Denoted, Z1Denoted: z1Denoted, SectionDelta: sectionDelta}
	if sectionDelta != 0 {
		result.Carriers = &record
	}
	body, err := AnalyticBody(ctx, env, result)
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}

// wholeAnalytic reports a line, circle or arc over its whole range (U2).
func wholeAnalytic(segment sectionrecord.CurveSegment) bool {
	switch s := segment.(type) {
	case sectionrecord.LineSeg:
		return prismcells.WholeSegmentRange(s.TStart, s.TEnd)
	case sectionrecord.CircleSeg:
		return prismcells.WholeSegmentRange(s.TStart, s.TEnd)
	case sectionrecord.ArcSeg:
		return prismcells.WholeSegmentRange(s.TStart, s.TEnd)
	default:
		return false
	}
}

// errSharedLine marks two operands' lines on one carrier (U5): a miss.
var errSharedLine = errors.New("prismboolean: two operands share a line carrier")

// distinctLineCarriers is U5's line half: no two operands' lines lie on one
// infinite line.
func distinctLineCarriers(budget *proofbound.WorkBudget, profiles []momentinput.Profile) error {
	type sourceLine struct {
		owner int
		line  sectionrecord.LineSeg
	}
	var lines []sourceLine
	for i, p := range profiles {
		for _, loop := range append([]sectionrecord.LoopRecord{p.Outer}, p.Holes...) {
			for _, segment := range loop.Segments {
				if line, ok := segment.(sectionrecord.LineSeg); ok {
					lines = append(lines, sourceLine{owner: i, line: line})
				}
			}
		}
	}
	for i, line := range lines {
		for _, prior := range lines[:i] {
			if prior.owner == line.owner {
				continue
			}
			if err := budget.Step(); err != nil {
				return err
			}
			if prismcells.SameLineCarrier(prior.line, line.line) {
				return errSharedLine
			}
		}
	}
	return nil
}

// sceneFloor is §7's contact floor: ContactEps times the diagonal of the
// true bounding box of every operand segment.
func sceneFloor(budget *proofbound.WorkBudget, profiles []momentinput.Profile) (float64, error) {
	var loops []sectionrecord.LoopRecord
	for _, p := range profiles {
		loops = append(loops, p.Outer)
		loops = append(loops, p.Holes...)
	}
	entries, err := sectionaudit.EntriesOf(budget, loops)
	if err != nil {
		return 0, err
	}
	return sectionaudit.ContactFloor(budget, entries)
}

// loopEntries are one merged loop's carrier-record entries (§5.3, "Result
// record").
type loopEntries struct {
	segment, junction []float64
}

// chainEntries states each result segment's ε (its source segment's) and
// each junction's δ: Δ_v at a cut vertex, the operand's own junction entry
// where both result segments end at their natural bounds.
func chainEntries(tags map[sketch.Entity]prismcells.Origin, records []prismrecord.CarrierRecord,
	vr prismcells.VertexReading, chain []sketch.BoundaryEdge, loop sectionrecord.LoopRecord) (loopEntries, bool) {
	n := len(chain)
	out := loopEntries{segment: make([]float64, n), junction: make([]float64, n)}
	if n != len(loop.Segments) || n == 0 {
		return out, false
	}
	for j := range n {
		o, ok := tags[chain[j].Entity]
		if !ok {
			return out, false
		}
		src, ok := sourceEntry(records, o)
		if !ok {
			return out, false
		}
		out.segment[j] = src
		k := (j + 1) % n
		endCut, _, err := prismcells.CutEnd(chain[j], loop.Segments[j], false)
		if err != nil {
			return out, false
		}
		startCut, _, err := prismcells.CutEnd(chain[k], loop.Segments[k], true)
		if err != nil {
			return out, false
		}
		if endCut || startCut {
			at, ok := prismcells.ChainEnd(chain[j])
			if !ok {
				return out, false
			}
			if v, has := vr.Vertices[at]; has {
				out.junction[j] = v.Delta
				continue
			}
			// two pieces of one circle meet at its seam
			seam, ok := prismcells.SeamJunction(chain[j], chain[k], tags, records)
			if !ok {
				return out, false
			}
			out.junction[j] = seam
			continue
		}
		if n == 1 {
			continue // a whole circle states no junction
		}
		next, ok := tags[chain[k].Entity]
		if !ok || next.Region != o.Region || next.Hole != o.Hole {
			return out, false
		}
		jd, ok := ownJunction(records[o.Region], o, next)
		if !ok {
			return out, false
		}
		out.junction[j] = jd
	}
	return out, true
}

// chooseOuter reads each merged loop's signed area as an enclosure and
// returns the loop order, outer first. Exactly one loop's enclosure must be
// positive; every other loop's must be negative, the hole orientation.
func chooseOuter(budget *proofbound.WorkBudget, loops []sectionrecord.LoopRecord,
	entries []loopEntries) ([]int, bool, error) {
	outer := -1
	for i, loop := range loops {
		rec := prismrecord.ZeroCarrierRecord(momentinput.Profile{Outer: loop})
		rec.Segment[0], rec.Junction[0] = entries[i].segment, entries[i].junction
		hLoop := rec.LoopH(0)
		area, ok, err := prismcells.LoopAreaEnclosure(budget, loop, hLoop)
		if err != nil || !ok {
			return nil, false, err
		}
		switch {
		case area.Lo.Sign() > 0:
			if outer >= 0 {
				return nil, false, nil // several lumps: not this route
			}
			outer = i
		case area.Hi.Sign() < 0:
		default:
			return nil, false, nil // an enclosure holding zero
		}
	}
	if outer < 0 {
		return nil, false, nil
	}
	order := []int{outer}
	for i := range loops {
		if i != outer {
			order = append(order, i)
		}
	}
	return order, true, nil
}

// sourceEntry is the ε of the source segment an origin names.
func sourceEntry(records []prismrecord.CarrierRecord, o prismcells.Origin) (float64, bool) {
	if o.Region < 0 || o.Region >= len(records) {
		return 0, false
	}
	l := o.Hole + 1
	r := records[o.Region]
	if l < 0 || l >= len(r.Segment) || o.Segment < 0 || o.Segment >= len(r.Segment[l]) {
		return 0, false
	}
	return r.Segment[l][o.Segment], true
}

// ownJunction is the junction δ between two consecutive segments of one
// operand loop, in either walk direction.
func ownJunction(r prismrecord.CarrierRecord, a, b prismcells.Origin) (float64, bool) {
	l := a.Hole + 1
	if l < 0 || l >= len(r.Junction) {
		return 0, false
	}
	n := len(r.Junction[l])
	switch {
	case n == 0:
		return 0, false
	case b.Segment == (a.Segment+1)%n:
		return r.Junction[l][a.Segment], true
	case a.Segment == (b.Segment+1)%n:
		return r.Junction[l][b.Segment], true
	default:
		return 0, false
	}
}
