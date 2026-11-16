package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the lateral carrier route of agent-docs/carrier-record-design.md
// §7: a Fillet or Chamfer of a prism whose section is displaced from the one it
// denotes but carries a valid carrier record. The construction is the
// ordinary corner rewrite on the recorded carriers (offset2d's carrier
// override). Table CB holds a margin on every decision the displacement could
// flip, §7.5 charges what a displaced carrier or junction moves, and the
// result carries a carrier record of its own (§7.7).

// CarrierRefusal is a Table CB row's refusal (agent-docs/carrier-record-design.md
// §7.6). It is an ErrUnsupported: the body exists and this evaluator cannot
// prove it. Chamfer falls through on one; Fillet returns it.
type CarrierRefusal struct { //nolint:errname // a Table CB row's refusal, named for the doc's term
	Row    string // CB0 … CB7
	Corner string // the corner's section point, when one is at fault
	Reason string
}

func (e *CarrierRefusal) Error() string {
	where := ""
	if e.Corner != "" {
		where = ", " + e.Corner
	}
	return fmt.Sprintf(`%v: the displaced receiver's carrier record cannot prove this blend (agent-docs/carrier-record-design.md Table %s%s): %s`,
		decaderr.ErrUnsupported, e.Row, where, e.Reason)
}

func (e *CarrierRefusal) Unwrap() error { return decaderr.ErrUnsupported }

func cbRefusal(row, corner, format string, args ...any) error {
	return &CarrierRefusal{Row: row, Corner: corner, Reason: fmt.Sprintf(format, args...)}
}

// carrierBlendKind names the payload's blend kind, as every prism blend
// records it.
func carrierBlendKind(fillet bool) string {
	if fillet {
		return "fillet"
	}
	return "chamfer"
}

// disabledRows lets a test switch a Table CB row off to show it is the guard
// that refuses (agent-docs/carrier-record-design.md §10). Production code
// never sets it.
var disabledRows atomic.Pointer[[]string]

func rowOn(row string) bool {
	off := disabledRows.Load()
	if off == nil {
		return true
	}
	return !slices.Contains(*off, row)
}

// cbCorner is one selected corner's reading.
type cbCorner struct {
	matched       modifyselect.MatchedCorner
	arrive, leave survey2d.SideWalk
	a, b          cbCarrier
	deltaJ, aJ    float64
	convention    bool // an exact-carrier fillet corner or all-zero chamfer corner (§7.3)
	dfA, dfB      float64
	connEps       float64
}

func (c *cbCorner) where() string { return c.matched.String() }

// walkEntries reads a walk's record entries (§7.2): its carrier entry, the
// junction entry before its first segment and the one after its last. ok=false
// is CB0: a merged walk carries a nonzero entry.
func walkEntries(cr prismrecord.CarrierRecord, li int, w survey2d.SideWalk, nseg int) (eps, startJ, endJ float64, ok bool) {
	if len(w.Segs) == 0 {
		return 0, 0, 0, false
	}
	first, last := w.Segs[0], w.Segs[len(w.Segs)-1]
	if len(w.Segs) > 1 {
		for k, s := range w.Segs {
			if cr.Segment[li][s] != 0 || (k < len(w.Segs)-1 && cr.Junction[li][s] != 0) {
				return 0, 0, 0, false
			}
		}
	} else {
		eps = cr.Segment[li][first]
	}
	return eps, cr.Junction[li][(first+nseg-1)%nseg], cr.Junction[li][last], true
}

// CarrierLateralBlend is §7's carrier route: the fillet (fillet=true, radius
// size) or equal chamfer (distance size) of every selected lateral corner of
// a displaced prism with a valid carrier record. matched=false (err nil)
// means a selected edge is not a lateral corner, and the caller keeps its
// existing route. A Table CB refusal is a *CarrierRefusal.
func CarrierLateralBlend(ctx context.Context, env topo.Env, pp prismrecord.Payload, cr prismrecord.CarrierRecord,
	sel string, edges []*topo.Edge, size float64, fillet bool) (Result, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return Result{}, true, err
	}
	sections := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	if len(loops) != len(sections) || len(cr.Segment) != len(sections) {
		return Result{}, false, nil
	}
	blendAt := make([]map[int]*offset2d.Blend, len(loops))
	corners := make([]map[int]*cbCorner, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*offset2d.Blend{}
		corners[i] = map[int]*cbCorner{}
	}
	var matched []modifyselect.MatchedCorner
	for ei, e := range edges {
		li, ci, found, err := modifyselect.MatchCorner(budget, pp, loops, e)
		if err != nil {
			return Result{}, true, err
		}
		if !found {
			return Result{}, false, nil
		}
		mc := modifyselect.NewMatchedCorner(ei, e, li, ci, loops[li])
		matched = append(matched, mc)
		blendAt[li][ci] = nil
		corners[li][ci] = &cbCorner{matched: mc}
	}
	fail := func(err error) (Result, bool, error) {
		if _, ok := errors.AsType[*CarrierRefusal](err); ok {
			return Result{}, true, fmt.Errorf(`%w; selector %s`, err, sel)
		}
		return Result{}, true, err
	}

	// CB0: a merged walk of a loop with a selected corner carries no entry.
	for li := range loops {
		if len(corners[li]) == 0 {
			continue
		}
		for wi, w := range loops[li].Walks {
			if _, _, _, ok := walkEntries(cr, li, w, len(sections[li].Segments)); !ok && rowOn("CB0") {
				return fail(cbRefusal("CB0", "", `walk %d of loop %d covers several recorded segments and one of their entries is nonzero, so the denoted section may turn where the record does not`, wi, li))
			}
		}
	}

	tol := sectionaudit.Tolerance
	for li := range loops {
		walks := loops[li].Walks
		n := len(walks)
		nseg := len(sections[li].Segments)
		for ci, c := range corners[li] {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return Result{}, true, err
			}
			c.arrive, c.leave = walks[(ci+n-1)%n], walks[ci]
			epsA, _, endJ, _ := walkEntries(cr, li, c.arrive, nseg)
			epsB, _, _, _ := walkEntries(cr, li, c.leave, nseg)
			c.deltaJ = endJ
			c.aJ = math.Max(proofbound.WalkEndBoundAllow(c.arrive.EndBound), proofbound.WalkEndBoundAllow(c.leave.StartBound))
			var okA, okB bool
			c.a, okA = newCBCarrier(sections[li].Segments[c.arrive.Segs[len(c.arrive.Segs)-1]], c.arrive, epsA)
			c.b, okB = newCBCarrier(sections[li].Segments[c.leave.Segs[0]], c.leave, epsB)
			if !okA || !okB {
				return fail(cbRefusal("CB0", c.where(), `a carrier at the corner is not a finite line or circle, or a displaced circle has a radius below twice its displacement`))
			}
			blend, err := cornerBlend(c, fillet, size, tol)
			if err != nil {
				return fail(err)
			}
			blendAt[li][ci] = blend
		}
	}
	widened, err := carrierWiden(cr, sections, loops, blendAt, corners)
	if err != nil {
		return fail(err)
	}

	profile, blendSegs, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blendAt)
	if err != nil {
		return Result{}, true, err
	}
	// S6 runs on the cutbacks as constructed first: an overrun there is the
	// ordinary audit's own verdict. Only an overrun the CB5 widening adds is
	// a Table CB refusal, which a chamfer falls through on.
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blendAt); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return Result{}, true, err
		}
		return Result{}, true, modifyselect.WrapAuditError(sel, matched, err)
	}
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, widened); err != nil && rowOn("CB5") {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return Result{}, true, err
		}
		return fail(cbRefusal("CB5", "", `the displacement widens a cutback past what S6 admits: %v`, err))
	}
	rewritten := make([]bool, len(blendAt))
	for li := range blendAt {
		rewritten[li] = len(blendAt[li]) > 0
	}
	profile, seams := closeSeams(profile, rewritten)
	record, err := carrierRewriteRecord(cr, sections, loops, blendAt, corners, profile, seams)
	if err != nil {
		return fail(err)
	}
	if err := carrierAuditTransfer(budget, record); err != nil {
		return fail(err)
	}
	kind := carrierBlendKind(fillet)
	out := prismrecord.Payload{
		Profile:      profile,
		Frame:        pp.Frame,
		Z0:           pp.Z0,
		Z1:           pp.Z1,
		Z0Delta:      pp.Z0Delta,
		Z1Delta:      pp.Z1Delta,
		Z0Denoted:    pp.Z0Denoted,
		Z1Denoted:    pp.Z1Denoted,
		Xform:        pp.Xform,
		BlendSegs:    blendSegs,
		BlendKind:    kind,
		SectionDelta: record.SectionDelta,
	}
	if record.SectionDelta != 0 {
		out.Carriers = &record
	}
	var body *topo.Body
	if out.Carriers != nil {
		body, err = prismrecord.BuildWithRecord(ctx, env, out, *out.Carriers)
	} else {
		body, err = prismrecord.Build(ctx, env, out, freeform.NewFreeformWork())
	}
	if err != nil {
		return Result{}, true, err
	}
	return Result{Body: body}, true, nil
}

// cornerBlend builds one selected corner's blend from the recorded carriers
// and holds Table CB's margins on its decisions, filling in the corner's feet
// charges and connector entry.
func cornerBlend(c *cbCorner, fillet bool, size, tol float64) (*offset2d.Blend, error) {
	ovA, ovB := &c.a.ov, &c.b.ov
	walks := []survey2d.SideWalk{c.arrive, c.leave}
	thetaA, thetaB := c.a.theta1(c.deltaJ, c.aJ), c.b.theta1(c.deltaJ, c.aJ)
	if !fillet {
		blend, err := offset2d.ChamferWithCarriers(walks, 1, size, size, tol, ovA, ovB)
		if err != nil {
			return nil, fmt.Errorf(`%w; %s`, err, c.where())
		}
		cross, _ := offset2d.CornerCrossWithCarriers(walks, 1, ovA, ovB)
		if err := carrierTurnMargin(c, cross, thetaA, thetaB); err != nil {
			return nil, err
		}
		return blend, chamferCharge(c, blend, size)
	}
	blend, dec, err := offset2d.FilletWithCarriers(walks, 1, size, tol, ovA, ovB)
	if err != nil {
		return nil, fmt.Errorf(`%w; %s`, err, c.where())
	}
	if err := carrierTurnMargin(c, dec.Cross, thetaA, thetaB); err != nil {
		return nil, err
	}
	return blend, filletCharge(c, blend, dec, size, thetaA)
}

// carrierTurnMargin is CB1 (brep-modify PD5a with these θ): the S4 turn
// exceeds tol + θ_A + θ_B + 2⁻⁴⁸.
func carrierTurnMargin(c *cbCorner, cross, thetaA, thetaB float64) error {
	margin := sumUp(sectionaudit.Tolerance, thetaA, thetaB, senseRounding)
	if math.Abs(cross) > margin || !rowOn("CB1") {
		return nil
	}
	return cbRefusal("CB1", c.where(), `the corner's turn %g is within %g of a smooth corner, so the displacement may decide which way it turns`, cross, margin)
}

// filletCharge is §7.4 and §7.5 for one fillet corner: Claim D′ where both
// carriers are exact, else Lemma T's centre displacement D; then the feet,
// the arc, and CB2 to CB4 and CB6.
func filletCharge(c *cbCorner, blend *offset2d.Blend, dec offset2d.FilletDecisions, r, thetaA float64) error {
	sign := 1.0
	if dec.Cross < 0 {
		sign = -1
	}
	c.convention = c.a.eps == 0 && c.b.eps == 0
	pu, pv, ok := ratOf2(dec.Center[0], dec.Center[1])
	if !ok {
		return cbRefusal("CB2", c.where(), `the fillet centre is not finite`)
	}
	centreRat := [2]*big.Rat{pu, pv}
	var ctr centre
	if !c.convention {
		ctr, ok = centreDisplacement(c.a, c.b, sign, dec.Center, r)
		if !ok && rowOn("CB2") {
			return cbRefusal("CB2", c.where(), `Lemma T finds no single crossing of the denoted offsets about the centre (%g, %g)`, dec.Center[0], dec.Center[1])
		}
	}
	// CB3: the root nearest the corner is nearer by more than the corner's and
	// both centres' displacements.
	if other, has := otherRoot(dec); has {
		var ctr1 centre
		if !c.convention {
			if ctr1, ok = centreDisplacement(c.a, c.b, sign, other, r); !ok && rowOn("CB2") {
				return cbRefusal("CB2", c.where(), `Lemma T finds no single crossing of the denoted offsets about the second centre (%g, %g)`, other[0], other[1])
			}
		}
		cu, cv := c.leave.StartU, c.leave.StartV
		d0 := math.Hypot(dec.Center[0]-cu, dec.Center[1]-cv)
		d1 := math.Hypot(other[0]-cu, other[1]-cv)
		margin := sumUp(proofbound.ProductUpper(2, sumUp(c.deltaJ, c.aJ)), ctr.d, ctr1.d, math.Ldexp(d0+d1, -48))
		if !(d1-d0 > margin) && rowOn("CB3") {
			return cbRefusal("CB3", c.where(), `the two fillet centres lie %g and %g from the corner, within %g of each other`, d0, d1, margin)
		}
	}
	// Feet (§7.5): Δf from the exact foot of ĉ on each recorded carrier.
	feet := [2][2]float64{{blend.FA.U, blend.FA.V}, {blend.FB.U, blend.FB.V}}
	carriers := [2]cbCarrier{c.a, c.b}
	e := [2]float64{ctr.eA, ctr.eB}
	var df [2]float64
	for i := range carriers {
		rhoF, ok := carriers[i].footResidual(feet[i], centreRat)
		if !ok {
			return cbRefusal("CB2", c.where(), `the fillet foot's residual from its recorded carrier cannot be stated`)
		}
		rc := 0.0
		if !carriers[i].line {
			dist, okd := carriers[i].centreDist(centreRat)
			if !okd {
				return cbRefusal("CB2", c.where(), `the fillet centre lies on a circular carrier's centre`)
			}
			rc = dist.loDown()
		}
		df[i] = carriers[i].filletFootDelta(rhoF, ctr.d, e[i], r, rc)
	}
	c.dfA, c.dfB = df[0], df[1]
	// CB4: the connector arc's sense reading clears the feet's and centre's displacement.
	margin := sumUp(c.dfA, ctr.d, proofbound.ProductUpper(r, sumUp(thetaA, senseRounding)))
	if !(math.Abs(dec.Sense) > margin) && rowOn("CB4") {
		return cbRefusal("CB4", c.where(), `the fillet arc's sense reading %g is within %g of zero`, dec.Sense, margin)
	}
	// CB6: an inward circular carrier keeps its offset radius above tol + ε + D.
	// A corner on the convention (§7.3) has no displaced carrier and no D.
	if !c.convention {
		for _, cc := range carriers {
			if cc.line {
				continue
			}
			off, need, ok := cc.cb6Margin(sign, r, ctr.d)
			if !ok {
				return cbRefusal("CB6", c.where(), `the fillet radius %g leaves a circular carrier no positive offset radius`, r)
			}
			if cc.inside*sign > 0 && !(off > need) && rowOn("CB6") {
				return cbRefusal("CB6", c.where(), `the fillet radius %g leaves an offset radius %g, not above the displacement %g`,
					r, off, need)
			}
		}
	}
	if c.convention {
		return nil
	}
	// The connector arc: ε_arc = D + | |Start − ĉ| − r |, with Start the emitted
	// ArcSeg's own Start: offset2d.ArcSegment stores a clockwise arc reversed,
	// so its Start is f̂B and not f̂A.
	arc, isArc := blend.Connector.(sectionrecord.ArcSeg)
	if !isArc {
		return cbRefusal("CB2", c.where(), `the fillet connector is not an arc`)
	}
	fu, fv, ok := ratOf2(arc.Start.U, arc.Start.V)
	if !ok {
		return cbRefusal("CB2", c.where(), `the fillet foot is not finite`)
	}
	rad, ok := norm(ivPoint(new(big.Rat).Sub(fu, pu)), ivPoint(new(big.Rat).Sub(fv, pv)))
	rr, okr := ivFloat(r)
	if !ok || !okr {
		return cbRefusal("CB2", c.where(), `the fillet arc's radius cannot be stated`)
	}
	c.connEps = proofbound.AbsSumUpper(ctr.d, rad.sub(rr).maxAbs())
	return nil
}

// chamferCharge is §7.5's chamfer feet and the connector entry. A corner whose
// carriers and junction are all exact takes today's float chord with a zero
// entry and charges each foot only its residual ρ_f (§7.3).
func chamferCharge(c *cbCorner, blend *offset2d.Blend, d float64) error {
	c.convention = c.a.eps == 0 && c.b.eps == 0 && c.deltaJ == 0
	corner := [2]float64{c.leave.StartU, c.leave.StartV}
	aA := proofbound.WalkEndBoundAllow(c.arrive.EndBound)
	aB := proofbound.WalkEndBoundAllow(c.leave.StartBound)
	rhoA, okA := c.a.chamferFootResidual([2]float64{blend.FA.U, blend.FA.V}, corner, aA, d, true)
	rhoB, okB := c.b.chamferFootResidual([2]float64{blend.FB.U, blend.FB.V}, corner, aB, d, false)
	if !okA || !okB {
		return cbRefusal("CB0", c.where(), `the chamfer foot's residual from its recorded carrier cannot be stated`)
	}
	c.dfA = c.a.chamferFootDelta(rhoA, c.deltaJ, d)
	c.dfB = c.b.chamferFootDelta(rhoB, c.deltaJ, d)
	if !c.convention {
		c.connEps = math.Max(c.dfA, c.dfB)
	}
	return nil
}

// otherRoot is the second distinct offset root, if any.
func otherRoot(dec offset2d.FilletDecisions) ([2]float64, bool) {
	for _, r := range dec.Roots {
		if r != dec.Center {
			return r, true
		}
	}
	return [2]float64{}, false
}

// seamMove is what closeSeams charges to one segment it moved: read is how
// far its walk-start read point moved plus the allowance of the new walk start
// itself, carrier how far the recorded field it rewrote moved (a line's carrier end, or an arc's radius).
type seamMove struct{ Read, Carrier float64 }

// seamEnds reads a line or arc's walk ends in walk order, as the record reads
// them.
func seamEnds(seg sectionrecord.CurveSegment) (sU, sV, eU, eV float64, ok bool) {
	switch seg.(type) {
	case sectionrecord.LineSeg, sectionrecord.ArcSeg:
	default:
		return 0, 0, 0, 0, false
	}
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return 0, 0, 0, 0, false
	}
	return w.StartU, w.StartV, w.EndU, w.EndV, true
}

// seamStartAt rewrites the recorded field a segment's walk starts at to p. A
// line or arc whose walk starts at its own Start (parameter 0) or End
// (parameter 1) is rewritable; a narrowed segment is not, since its read
// point is no recorded field. ok=false leaves it alone.
func seamStartAt(seg sectionrecord.CurveSegment, p sectionrecord.Point2) (sectionrecord.CurveSegment, sectionrecord.Point2, bool) {
	switch b := seg.(type) {
	case sectionrecord.LineSeg:
		switch b.TStart {
		case 0:
			old := b.Start
			b.Start = p
			return b, old, true
		case 1:
			old := b.End
			b.End = p
			return b, old, true
		}
	case sectionrecord.ArcSeg:
		switch b.TStart {
		case 0:
			old := b.Start
			b.Start = p
			return b, old, true
		case 1:
			old := b.End
			b.End = p
			return b, old, true
		}
	}
	return seg, sectionrecord.Point2{}, false
}

func seamDistUp(au, av, bu, bv float64) (float64, bool) {
	a1, a2, ok1 := ratOf2(au, av)
	b1, b2, ok2 := ratOf2(bu, bv)
	if !ok1 || !ok2 {
		return 0, false
	}
	return pointDistUp(a1, a2, b1, b2), true
}

// closeSeams snaps a rewritten loop's segment start onto the previous
// segment's end where the two read ends differ only by a walk end's rounding
// (a circle's seam at parameter 1 reads sin(2π) ≈ −2.4e-16·R and parameter 0
// reads zero), so a section whose segments do not close can be recorded
// again. Ends are compared in walk order. A kept loop is left alone. Every
// move is charged: the returned moves, indexed like each loop's segments, say
// how far the read start and the rewritten field moved, and the record adds
// them to that end's displacement.
func closeSeams(p momentinput.Profile, rewritten []bool) (momentinput.Profile, [][]seamMove) {
	var moves [][]seamMove
	snap := func(li int, loop sectionrecord.LoopRecord) sectionrecord.LoopRecord {
		mv := make([]seamMove, len(loop.Segments))
		moves = append(moves, mv)
		if !rewritten[li] || len(loop.Segments) < 2 {
			return loop
		}
		segs := append([]sectionrecord.CurveSegment(nil), loop.Segments...)
		for j := range segs {
			k := (j + 1) % len(segs)
			_, _, eu, ev, okJ := seamEnds(segs[j])
			su, sv, _, _, okK := seamEnds(segs[k])
			if !okJ || !okK {
				continue
			}
			gap := math.Hypot(su-eu, sv-ev)
			if gap == 0 || gap > 1e-12*math.Max(1, math.Hypot(eu, ev)) {
				continue
			}
			moved, old, ok := seamStartAt(segs[k], sectionrecord.Point2{U: eu, V: ev})
			if !ok {
				continue
			}
			mw, err := boundarywalk.WalkOf(moved, nil)
			if err != nil {
				continue
			}
			read, ok1 := seamDistUp(mw.StartU, mw.StartV, su, sv)
			field, ok2 := seamDistUp(eu, ev, old.U, old.V)
			if !ok1 || !ok2 {
				continue
			}
			segs[k] = moved
			// The moved segment's new walk start is itself a float read of its
			// recorded field (agent-docs/carrier-record-design.md §7.7, "Seam
			// snap"), so its allowance joins the distance the read start moved.
			read = proofbound.AbsSumUpper(read, proofbound.WalkEndBoundAllow(mw.StartBound))
			mv[k] = seamMove{Read: read, Carrier: field}
		}
		return sectionrecord.LoopRecord{Segments: segs}
	}
	out := momentinput.Profile{Outer: snap(0, p.Outer)}
	if p.Holes != nil {
		out.Holes = make([]sectionrecord.LoopRecord, 0, len(p.Holes))
	}
	for i, h := range p.Holes {
		out.Holes = append(out.Holes, snap(i+1, h))
	}
	return out, moves
}
