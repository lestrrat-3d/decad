package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
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
	"github.com/lestrrat-3d/units"
)

// This file is route P's displaced reading of agent-docs/brep-modify-design.md
// §4.4 ("brep-modify §4.4"): a brep receiver whose record carries a section
// displacement (a class-B crossing result) that reads as a prism is filleted
// or chamfered at its lateral edges from the EXACT carriers its record holds
// — each line's plane level and each circular wall's recorded operand circle
// (breprecord.Face.WallCarrier) — never from the displaced vertices. Every
// displaced vertex is a corner the selection blends (PD4), so none survives
// in the rewritten section. Table PD's gates are reject-only: each refuses
// more, never less.

// pdMargin is the 2⁻⁵⁰ PD5a adds for the float evaluation of the cross
// product it compares, a number of order one.
var pdMargin = math.Ldexp(1, -50)

// senseRounding is PD5c's relative rounding allowance, 2⁻⁴⁸ of the fillet
// radius: it covers the float evaluation of rot90(fA − O)·tA, whose two
// products and sum each round by an ulp of a term of size r, the
// normalisation of the unit tangent, and a circular walk's tangent read
// through sincos at a computed angle, each a few ulps of r.
var senseRounding = math.Ldexp(1, -48)

// SectionCarriers is a displaced route-P reading's exact carriers
// (brep-modify §4.4), one entry per segment of the section, loop by loop:
// Delta is the receiver's section displacement δ, and Radius and Center are a
// circular segment's claiming wall's recorded operand circle, in the section
// frame (Radius zero for a line). sb1 is the receiver's SB1 refusal, which
// every PD refusal extends.
type SectionCarriers struct {
	Delta  float64
	Radius [][]float64
	Center [][]sectionrecord.Point2
	sb1    error
}

// refuse is a PD row's refusal: the receiver's SB1 text, then the row and its
// reason, so a refusal still names SB1.
func (c SectionCarriers) refuse(row, reason string) error {
	return pdRefusal(c.sb1, row, reason)
}

func pdRefusal(sb1 error, row, reason string) error {
	return fmt.Errorf(`%w; its displaced reading (brep-modify §4.4) refuses: %s (brep-modify %s)`, sb1, reason, row)
}

// displacedReceiver is the brep route for a receiver with a section
// displacement (brep-modify §4.4, gate stages 2a and 2b): a Shell, an
// asymmetric chamfer, an op with no corner construction, or a record carrying
// route L bands is SB1's text alone, with no PD row; otherwise the displaced
// reading, which is SB1 with PD1's or PD2's reason where no axis admits.
func displacedReceiver(ctx context.Context, b *topo.Body, bp breprecord.Payload, req Request) (Route, error) {
	sb1 := breprecord.RequireExactSection(bp, req.Op)
	if req.Shell || req.Asym != nil || req.Blend == nil || len(bp.LoopBands) > 0 {
		return Route{}, sb1
	}
	route, reason, err := displacedPrismRoute(ctx, b, bp, req, sb1)
	switch {
	case err != nil:
		return Route{}, err
	case route.Prism != nil:
		return route, nil
	case reason != nil:
		return Route{}, reason
	default:
		return Route{}, sb1
	}
}

// displacedPrismRoute is route P over axes 0, 1, 2 for a displaced record
// (§4.4): the first prism reading (§4.1, P1–P5) whose claim map passes PD2 and
// whose classification the op admits is taken, carrying the record's section
// displacement and its carriers. reason is the first PD2 or PD1 refusal, and
// nil when no axis reads as a prism; err is a context error.
func displacedPrismRoute(ctx context.Context, b *topo.Body, bp breprecord.Payload, req Request, sb1 error) (Route, error, error) {
	embeds, err := breprecord.Embeds(bp.Faces)
	if err != nil {
		return Route{}, nil, nil //nolint:nilerr // no embeds is no prism reading
	}
	delta := bp.SectionDelta()
	var reason error
	for k := range 3 {
		read, ok, err := recognisePrism(ctx, bp, embeds, k)
		if err != nil {
			return Route{}, nil, err
		}
		if !ok {
			continue
		}
		claims, ok := read.walls.Claims(read.pp.Profile, freeform.NewFreeformWork())
		if !ok {
			continue
		}
		carriers, err := sectionCarriers(read.pp.Profile, claims, delta, sb1)
		if err != nil {
			if reason == nil {
				reason = err
			}
			continue
		}
		pp := read.pp
		pp.SectionDelta = delta
		caps := read.caps(b, bp)
		admitErr := req.Admits(pp, caps)
		if admitErr == nil {
			return Route{Prism: &pp, Caps: caps, Carriers: &carriers}, nil, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return Route{}, nil, ctxErr
		}
		if errors.Is(admitErr, context.Canceled) || errors.Is(admitErr, context.DeadlineExceeded) {
			return Route{}, nil, admitErr
		}
		if reason == nil {
			reason = pdRefusal(sb1, "PD1", fmt.Sprintf(`the selection is no lateral blend of the prism along reference axis %d: %v`, k, admitErr))
		}
	}
	return Route{}, reason, nil
}

// sectionCarriers is PD2 over one reading's claim map: every circular segment
// of the section is claimed by a P2(a) wall that records its operand circle,
// and every line by a P2(b) planar rectangle, never a restated straight wall.
func sectionCarriers(section momentinput.Profile, claims []brepgeom.SectionClaim, delta float64, sb1 error) (SectionCarriers, error) {
	out := SectionCarriers{Delta: delta, sb1: sb1}
	i := 0
	for li, loop := range sectionLoops(section) {
		radius := make([]float64, len(loop.Segments))
		center := make([]sectionrecord.Point2, len(loop.Segments))
		for si, seg := range loop.Segments {
			c := claims[i]
			i++
			switch seg.(type) {
			case sectionrecord.LineSeg:
				if c.Wall || !c.Planar {
					return SectionCarriers{}, pdRefusal(sb1, "PD2", fmt.Sprintf(`section loop %d segment %d is a line no planar face states`, li, si))
				}
			case sectionrecord.ArcSeg, sectionrecord.CircleSeg:
				if !c.Wall || !(c.Carrier.Radius > 0) {
					return SectionCarriers{}, pdRefusal(sb1, "PD2", fmt.Sprintf(`section loop %d segment %d lies on a wall that records no operand circle`, li, si))
				}
				radius[si], center[si] = c.Carrier.Radius, c.Carrier.Center
			default:
				return SectionCarriers{}, pdRefusal(sb1, "PD2", fmt.Sprintf(`section loop %d segment %d is neither a line nor a circular arc`, li, si))
			}
		}
		out.Radius = append(out.Radius, radius)
		out.Center = append(out.Center, center)
	}
	return out, nil
}

// sectionLoops is a section's loops in record order: the outer, then each
// hole.
func sectionLoops(p momentinput.Profile) []sectionrecord.LoopRecord {
	return append([]sectionrecord.LoopRecord{p.Outer}, p.Holes...)
}

// DisplacedLateralFillet is the fillet arm of brep-modify §4.4 (Claim D):
// the fillet of radius rmm at every selected lateral corner of pp, built from
// the exact carriers. It refuses with Table PD's rows; S4, S5 and the audit
// keep their own sentinels. The result is the prism over the filleted
// section with no section displacement.
func DisplacedLateralFillet(ctx context.Context, env topo.Env, pp prismrecord.Payload, carriers SectionCarriers,
	sel string, edges []*topo.Edge, rmm float64) (Result, error) {
	return displacedLateralBlend(ctx, env, pp, carriers, sel, edges, rmm, true)
}

// DisplacedLateralChamfer is the chamfer arm of brep-modify §4.4 (Claim C):
// the equal chamfer of distance dmm at every selected lateral corner of pp,
// its feet set back along the exact carriers. Its result carries δ_out
// (PD7), the displacement the corners' proven displacement moves its feet by.
func DisplacedLateralChamfer(ctx context.Context, env topo.Env, pp prismrecord.Payload, carriers SectionCarriers,
	sel string, edges []*topo.Edge, dmm float64) (Result, error) {
	return displacedLateralBlend(ctx, env, pp, carriers, sel, edges, dmm, false)
}

func displacedLateralBlend(ctx context.Context, env topo.Env, pp prismrecord.Payload, carriers SectionCarriers,
	sel string, edges []*topo.Edge, size float64, fillet bool) (Result, error) {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return Result{}, err
	}
	blendAt := make([]map[int]*offset2d.Blend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*offset2d.Blend{}
	}
	matched := make([]modifyselect.MatchedCorner, 0, len(edges))
	for ei, e := range edges {
		li, ci, found, err := modifyselect.MatchCorner(budget, pp, loops, e)
		if err != nil {
			return Result{}, err
		}
		if !found {
			return Result{}, carriers.refuse("PD1", fmt.Sprintf(`%s is not a lateral edge of the prism`, modifyselect.EdgeContext(ei, e)))
		}
		matched = append(matched, modifyselect.NewMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil
	}
	if err := carriers.requireCircles(pp.Profile); err != nil {
		return Result{}, err
	}
	if err := carriers.requireConsumed(loops, blendAt); err != nil {
		return Result{}, err
	}
	exact, err := carriers.carrierLoops(loops)
	if err != nil {
		return Result{}, err
	}

	kind := "chamfer"
	if fillet {
		kind = "fillet"
	}
	for _, corner := range matched {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return Result{}, err
		}
		walks := exact[corner.Loop].Walks
		cb, err := carriers.blendCorner(walks, corner.Corner, size, fillet)
		if err != nil {
			if errors.Is(err, errPD) {
				return Result{}, fmt.Errorf(`%w; selector %s, %s`, carriers.refuse(pdRow(err), err.Error()), sel, corner)
			}
			return Result{}, fmt.Errorf(`%w; selector %s, %s`, err, sel, corner)
		}
		blendAt[corner.Loop][corner.Corner] = cb
	}

	profile, blendSegs, err := offset2d.RewriteProfileBudget(budget, pp.Profile, exact, blendAt)
	if err != nil {
		return Result{}, err
	}
	widened := carriers.widenDisplaced(exact, blendAt)
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, exact, widened); err != nil {
		if errors.Is(err, context.Canceled) {
			return Result{}, context.Canceled
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return Result{}, context.DeadlineExceeded
		}
		return Result{}, modifyselect.WrapAuditError(sel, matched, err)
	}

	sectionDelta := 0.0
	if !fillet {
		sectionDelta = carriers.chamferCharge(exact, blendAt)
	}
	body, err := prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile:      profile,
		Frame:        pp.Frame,
		Z0:           pp.Z0,
		Z1:           pp.Z1,
		Z0Delta:      pp.Z0Delta,
		Z1Delta:      pp.Z1Delta,
		Xform:        pp.Xform,
		BlendSegs:    blendSegs,
		BlendKind:    kind,
		SectionDelta: sectionDelta,
	}, freeform.NewFreeformWork())
	if err != nil {
		return Result{}, err
	}
	return Result{Body: body}, nil
}

// requireCircles is PD3: each circular segment's recorded operand circle has
// the segment's own centre bit for bit, the displacement is below half its
// radius, and every recorded endpoint P of the segment lies in the annulus
// (R − δ)² ≤ |P − C|² ≤ (R + δ)², compared exactly. A whole circle states its
// radius, which must be the operand's bit for bit. It refuses; it admits
// nothing (CLAUDE.md: a check may only falsify).
func (c SectionCarriers) requireCircles(section momentinput.Profile) error {
	for li, loop := range sectionLoops(section) {
		for si, seg := range loop.Segments {
			r := c.Radius[li][si]
			if r == 0 {
				continue
			}
			centre := c.Center[li][si]
			if err := requireCarrierCircle(seg, centre, r, c.Delta); err != nil {
				return c.refuse("PD3", fmt.Sprintf(`section loop %d segment %d: %v`, li, si, err))
			}
		}
	}
	return nil
}

// requireCarrierCircle is PD3 for one circular segment and its carrier.
func requireCarrierCircle(seg sectionrecord.CurveSegment, centre sectionrecord.Point2, r, delta float64) error {
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		mm, err := s.Radius.In(units.Millimeter)
		if err != nil || s.Center != centre || mm != r {
			return errors.New(`the circle is not the operand circle its wall records`)
		}
		return nil
	case sectionrecord.ArcSeg:
		if s.Center != centre {
			return errors.New(`the arc's centre is not the centre of the operand circle its wall records`)
		}
		rr, ok1 := proofbound.RatOf(r)
		dd, ok2 := proofbound.RatOf(delta)
		if !ok1 || !ok2 {
			return errors.New(`the operand circle's radius or the displacement is not finite`)
		}
		if new(big.Rat).Mul(dd, big.NewRat(2, 1)).Cmp(rr) >= 0 {
			return errors.New(`the displacement is not below half the operand circle's radius`)
		}
		lo := new(big.Rat).Sub(rr, dd)
		lo.Mul(lo, lo)
		hi := new(big.Rat).Add(rr, dd)
		hi.Mul(hi, hi)
		for _, p := range []sectionrecord.Point2{s.Start, s.End} {
			d2, ok := distanceSquared(p, centre)
			if !ok || d2.Cmp(lo) < 0 || d2.Cmp(hi) > 0 {
				return fmt.Errorf(`the recorded endpoint (%g, %g) lies farther than the displacement from the operand circle`, p.U, p.V)
			}
		}
		return nil
	default:
		return errors.New(`the segment is not circular`)
	}
}

// distanceSquared is |p − c|², exactly.
func distanceSquared(p, c sectionrecord.Point2) (*big.Rat, bool) {
	pu, ok1 := proofbound.RatOf(p.U)
	pv, ok2 := proofbound.RatOf(p.V)
	cu, ok3 := proofbound.RatOf(c.U)
	cv, ok4 := proofbound.RatOf(c.V)
	if !ok1 || !ok2 || !ok3 || !ok4 {
		return nil, false
	}
	du := new(big.Rat).Sub(pu, cu)
	dv := new(big.Rat).Sub(pv, cv)
	du.Mul(du, du)
	dv.Mul(dv, dv)
	return du.Add(du, dv), true
}

// displacedCorner reports whether corner ci of a corner loop is a junction a
// circular walk takes part in: the only junctions of a class-B section whose
// point is displaced (§4.4, Claim X).
func displacedCorner(walks []survey2d.SideWalk, ci int) bool {
	n := len(walks)
	if n < 2 {
		return false
	}
	return walks[(ci+n-1)%n].IsCircular() || walks[ci].IsCircular()
}

// requireConsumed is PD4: every corner at which a circular walk meets another
// walk is selected, so the rewrite re-emits every displaced point as a foot
// on an exact carrier.
func (c SectionCarriers) requireConsumed(loops []survey2d.CornerLoop, blendAt []map[int]*offset2d.Blend) error {
	for li, loop := range loops {
		for ci, w := range loop.Walks {
			if !displacedCorner(loop.Walks, ci) {
				continue
			}
			if _, selected := blendAt[li][ci]; selected {
				continue
			}
			return c.refuse("PD4", fmt.Sprintf(`the corner at section point (u, v) = (%g, %g), where the cut meets a cylinder, is not selected; select every edge where the cut meets the cylinder`,
				w.StartU, w.StartV))
		}
	}
	return nil
}

// carrierLoops copies the corner loops with every circular walk on its
// recorded operand circle: the exact radius, and the length that radius gives
// the walk's sweep. A walk's centre is already exact (PD3), and a line walk's
// direction and fixed coordinate are (Claim X), so neither changes.
func (c SectionCarriers) carrierLoops(loops []survey2d.CornerLoop) ([]survey2d.CornerLoop, error) {
	out := make([]survey2d.CornerLoop, len(loops))
	for li, loop := range loops {
		walks := append([]survey2d.SideWalk(nil), loop.Walks...)
		for wi := range walks {
			w := &walks[wi]
			if !w.IsCircular() {
				continue
			}
			if len(w.Segs) != 1 {
				return nil, c.refuse("PD2", fmt.Sprintf(`section loop %d walk %d covers %d circular segments`, li, wi, len(w.Segs)))
			}
			r := c.Radius[li][w.Segs[0]]
			w.Radius = r
			w.Length = r * math.Abs(w.Th1-w.Th0)
		}
		out[li] = survey2d.CornerLoop{Walks: walks}
	}
	return out, nil
}

// errPD marks a per-corner PD5 refusal; pdRowError carries its row.
var errPD = errors.New(`displaced corner margin`)

type pdRowError struct {
	row, reason string
}

func (e pdRowError) Error() string { return e.reason }
func (e pdRowError) Is(target error) bool {
	return target == errPD
}

func pdRow(err error) string {
	if pe, ok := errors.AsType[pdRowError](err); ok {
		return pe.row
	}
	return "PD5"
}

// turnAllowance is θ = δ/(R − δ) rounded up, the most a circular walk's
// tangent turns between a recorded corner within δ of the true one and the
// true corner (§4.4, Claim D step 2); 0 for a line.
func (c SectionCarriers) turnAllowance(w survey2d.SideWalk) float64 {
	if !w.IsCircular() {
		return 0
	}
	return ratQuotientUp(c.Delta, w.Radius, false)
}

// transport is δ' (§4.4, PD6): how far a recorded corner within δ of the
// true one moves a measurement along a walk — δ along a line, and
// R·δ/(R − δ) rounded up along a circular walk.
func (c SectionCarriers) transport(w survey2d.SideWalk) float64 {
	if !w.IsCircular() {
		return c.Delta
	}
	return ratQuotientUp(c.Delta, w.Radius, true)
}

// ratQuotientUp is δ/(R − δ), or R·δ/(R − δ) when scaled, computed exactly
// and rounded up once; +Inf when R ≤ δ or a value is not finite.
func ratQuotientUp(delta, r float64, scaled bool) float64 {
	dd, ok1 := proofbound.RatOf(delta)
	rr, ok2 := proofbound.RatOf(r)
	if !ok1 || !ok2 {
		return math.Inf(1)
	}
	den := new(big.Rat).Sub(rr, dd)
	if den.Sign() <= 0 {
		return math.Inf(1)
	}
	q := new(big.Rat).Quo(dd, den)
	if scaled {
		q.Mul(q, rr)
	}
	return proofbound.RatFloatUp(q)
}

// sumUp is the exact sum of finite floats rounded up once.
func sumUp(values ...float64) float64 {
	sum := new(big.Rat)
	for _, v := range values {
		r, ok := proofbound.RatOf(v)
		if !ok {
			return math.Inf(1)
		}
		sum.Add(sum, r)
	}
	return proofbound.RatFloatUp(sum)
}

// requireTurnMargin is PD5a: |cross| > tol + θ_A + θ_B + 2⁻⁵⁰, with tol S4's
// sectionaudit.Tolerance, so the true corner turns the same way the recorded
// one does.
func requireTurnMargin(cross, thetaA, thetaB float64) error {
	margin := sumUp(sectionaudit.Tolerance, thetaA, thetaB, pdMargin)
	if math.Abs(cross) > margin {
		return nil
	}
	return pdRowError{row: "PD5a", reason: fmt.Sprintf(`the corner's turn %g is within %g of a smooth corner, so the displacement may decide which way it turns`, cross, margin)}
}

// requireRootMargin is PD5b: of two distinct offset roots, the one nearest
// the recorded corner k is nearer by more than 2δ (plus the distances' own
// rounding), so it is also the root nearest the true corner.
func requireRootMargin(roots [][2]float64, ku, kv, delta float64) error {
	if len(roots) < 2 || roots[0] == roots[1] {
		return nil
	}
	d0 := math.Hypot(roots[0][0]-ku, roots[0][1]-kv)
	d1 := math.Hypot(roots[1][0]-ku, roots[1][1]-kv)
	margin := sumUp(delta, delta, math.Ldexp(d0+d1, -48))
	if math.Abs(d0-d1) > margin {
		return nil
	}
	return pdRowError{row: "PD5b", reason: fmt.Sprintf(`the two fillet centres lie %g and %g from the corner, within %g of each other, so the displacement may decide which one is nearest`, d0, d1, margin)}
}

// requireSenseMargin is PD5c: |rot90(fA − O)·tA| > r·(θ_A + 2⁻⁴⁸), so the
// connector arc's sense depends neither on the displaced tangent nor on the
// dot product's rounding, which scales with r (senseRounding).
func requireSenseMargin(sense, r, thetaA float64) error {
	margin := proofbound.ProductUpper(r, sumUp(thetaA, senseRounding))
	if math.Abs(sense) > margin {
		return nil
	}
	return pdRowError{row: "PD5c", reason: fmt.Sprintf(`the fillet arc's sense reading %g is within %g of zero`, sense, margin)}
}

// blendCorner builds one selected corner's blend on the carrier walks and
// holds Table PD's per-corner margins (gate stage 4): the construction's own
// S4 and, for a fillet, S5 on the exact radius come first, then PD5a, and for
// a fillet PD5b and PD5c. A line–line corner is exact and holds no margin.
func (c SectionCarriers) blendCorner(walks []survey2d.SideWalk, ci int, size float64, fillet bool) (*offset2d.Blend, error) {
	n := len(walks)
	arrive, leave := walks[(ci+n-1)%n], walks[ci]
	thetaA, thetaB := c.turnAllowance(arrive), c.turnAllowance(leave)
	tol := sectionaudit.Tolerance
	if !fillet {
		cb, err := offset2d.Chamfer(walks, ci, size, size, tol)
		if err != nil || !displacedCorner(walks, ci) {
			return cb, err
		}
		cross, _ := offset2d.CornerCross(walks, ci)
		return cb, requireTurnMargin(cross, thetaA, thetaB)
	}
	cb, dec, err := offset2d.FilletWithRoots(walks, ci, size, tol)
	if err != nil || !displacedCorner(walks, ci) {
		return cb, err
	}
	if err := requireTurnMargin(dec.Cross, thetaA, thetaB); err != nil {
		return nil, err
	}
	if err := requireRootMargin(dec.Roots, leave.StartU, leave.StartV, c.Delta); err != nil {
		return nil, err
	}
	if err := requireSenseMargin(dec.Sense, size, thetaA); err != nil {
		return nil, err
	}
	return cb, nil
}

// widenDisplaced is PD6: a copy of blendAt in which each displaced corner's
// cutback on each side grows by 2δ' of that side's walk, rounded up, so S6
// passing on the recorded section implies it passes on the true one.
func (c SectionCarriers) widenDisplaced(loops []survey2d.CornerLoop, blendAt []map[int]*offset2d.Blend) []map[int]*offset2d.Blend {
	out := make([]map[int]*offset2d.Blend, len(blendAt))
	for li, at := range blendAt {
		out[li] = make(map[int]*offset2d.Blend, len(at))
		walks := loops[li].Walks
		n := len(walks)
		for ci, cb := range at {
			if cb == nil || !displacedCorner(walks, ci) {
				out[li][ci] = cb
				continue
			}
			wide := *cb
			da, db := c.transport(walks[(ci+n-1)%n]), c.transport(walks[ci])
			wide.CutbackA = sumUp(cb.CutbackA, da, da)
			wide.CutbackB = sumUp(cb.CutbackB, db, db)
			out[li][ci] = &wide
		}
	}
	return out
}

// chamferCharge is PD7, Claim C's δ_out: the larger of δ and, over every
// circular walk at a chamfered displaced corner, R·δ/(R − δ) rounded up —
// how far the corner's proven displacement moves a foot set back along that
// walk. A foot along a line moves by at most δ.
func (c SectionCarriers) chamferCharge(loops []survey2d.CornerLoop, blendAt []map[int]*offset2d.Blend) float64 {
	out := c.Delta
	for li, at := range blendAt {
		walks := loops[li].Walks
		n := len(walks)
		for ci, cb := range at {
			if cb == nil || !displacedCorner(walks, ci) {
				continue
			}
			out = max(out, c.transport(walks[(ci+n-1)%n]), c.transport(walks[ci]))
		}
	}
	return out
}
