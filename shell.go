package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the shell of docs/modify-design.md §8: Body.Shell removes the
// selected cap faces of a straight prism and lines what remains with a wall of
// the given thickness. On a prism the whole construction is the recorded
// section's own exact offset (§7): P ⊖ t inward, P ⊕ t outward, per-feature and
// topology-preserving. A both-caps shell of a hole-free section is a TUBE — a
// plain prismPayload over the annular section {Outer, Hole} (Table B, B2/B3),
// so nothing downstream needs a new case for it (§12). A one-cap shell is a CUP
// — the new cupPayload, two co-directional prisms over the same plane (B5/B6).
//
// The gate order is §4's, and the sentinel of each refusal follows the §1
// existence test. Staged, never a wrong body: side-wall removal is S2, a
// topology-changing offset is S11, a both-caps shell of a HOLED section is S12
// (a prismPayload holds one region), each ErrUnsupported.

// ShellOption configures Shell, including its wall sense.
type ShellOption interface {
	option.Interface
	shellOption()
}

type shellOption struct{ option.Interface }

func (shellOption) shellOption() {}

// ShellSense is the wall sense of a shell (docs/modify-design.md §8): the
// thickness is a magnitude and carries no sign (core §8.1), so which way the
// wall grows is enumerated, not signed.
type ShellSense int

const (
	// Inward grows the wall into the original solid; the outer skin does not
	// move. It is the default — what "shell this box" means everywhere.
	Inward ShellSense = iota
	// Outward grows the wall off the original solid; the original solid becomes
	// the cavity.
	Outward
)

// String renders the sense for diagnostics.
func (s ShellSense) String() string {
	switch s {
	case Inward:
		return "Inward"
	case Outward:
		return "Outward"
	default:
		return fmt.Sprintf("ShellSense(%d)", int(s))
	}
}

type identShellSense struct{}

// WithShellSense sets the wall sense (Inward or Outward). Without it the sense
// is Inward (docs/modify-design.md §8).
func WithShellSense(s ShellSense) ShellOption {
	return shellOption{option.New(identShellSense{}, s)}
}

// Shell removes the selected cap faces of a straight prism and lines the rest
// with a wall of thickness t, returning the new body and retiring the receiver
// (docs/modify-design.md §8, core §8). sel names the faces to REMOVE — the
// openings — resolved against the live receiver; a query matching nothing is
// loud (ErrNoMatch / ErrCardinality, S16). t is a length magnitude, gated like
// every other (S15); a zero t is S14 (the wall is the empty region). A removed
// SIDE wall is S2, a receiver whose payload is not a prism is S3, both
// ErrUnsupported. The offset section faces the §5 audit before anything is
// built, so no unproven body is ever made.
func (b *Body) Shell(ctx context.Context, sel FaceSelector, t units.Value, opts ...ShellOption) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc

	// Stage 1 pre-gates (§4): a live receiver (S17), the options, a valid
	// magnitude (S15), a non-zero one (S14), then a selector that matches (S16).
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	// Table X (docs/surface-design.md §11): refused ahead of the selector gate
	// so the answer does not depend on what the selector matched.
	if err := refuseSheetOperand(b, "Shell"); err != nil {
		return nil, err
	}
	sense := Inward
	for _, raw := range opts {
		if raw == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		// decad owns the option vocabulary. Embedding ShellOption can promote
		// its sealed marker onto a foreign type, so admit the owned concrete
		// implementation before invoking any option callback.
		o, ok := raw.(shellOption)
		if !ok {
			return nil, fmt.Errorf(`%w: the shell option is not a decad shell option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identShellSense:
			v, ok := option.Get[ShellSense](o)
			if !ok {
				return nil, fmt.Errorf(`%w: WithShellSense carries no sense`, ErrDegenerate)
			}
			if v != Inward && v != Outward {
				return nil, fmt.Errorf(`%w: unknown shell sense %d`, ErrDegenerate, int(v))
			}
			sense = v
		default:
			return nil, fmt.Errorf(`%w: unknown shell option identifier %T`, ErrDegenerate, ident)
		}
	}
	tmm, tDelta, err := magnitudeInBounded(t, units.Length, units.Millimeter, "the shell thickness")
	if err != nil {
		return nil, err
	}
	if tmm == 0 {
		// S14: a face is removed and the wall is P \ P — the empty region, no
		// solid at all.
		return nil, fmt.Errorf(`%w: a zero-thickness shell leaves no wall`, ErrDegenerate)
	}
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. Reject foreign implementations before invoking
	// their callback, and treat a typed nil query like an untyped nil.
	q, ok := sel.(*FaceQuery)
	switch {
	case sel == nil:
		return nil, errNilSelector
	case !ok:
		return nil, fmt.Errorf(`%w: the shell's face selector is not a decad face query (%T)`, ErrDegenerate, sel)
	case q == nil:
		return nil, errNilSelector
	}
	removed, err := q.SelectFaces(b)
	if err != nil {
		return nil, err
	}

	// SX10: a capBlendPayload receiver is staged before the generic "not a
	// prism" refusal, so the more specific reason leads.
	if err := requireNotCapBlendReceiver(b.payload, "shells"); err != nil {
		return nil, err
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every removed face
	// is a cap (S2).
	pp, ok := b.payload.(prismPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator shells a straight prism only`, ErrUnsupported)
	}
	if err := requireExactSection(pp, "shells"); err != nil {
		return nil, err
	}
	removedStart, removedEnd, err := classifyRemovedCaps(b, removed)
	if err != nil {
		return nil, err
	}
	bothCaps := removedStart && removedEnd

	s := 1.0 // inward: erode
	if sense == Outward {
		s = -1.0 // outward: dilate
	}
	holed := len(pp.profile.Holes) > 0
	h := pp.z1 - pp.z0
	offsetBudget := newWorkBudget(ctx)
	if err := offsetBudget.err(); err != nil {
		return nil, err
	}

	// Stage 3 (§4): the construction's own gates — S18 (the inward section
	// survey stays within its fixed work budget), S10 (the cavity is non-empty,
	// inward only), then S11a (no feature the offset drops as it is built).
	if s > 0 {
		// The section limit: P ⊖ t is non-empty exactly when t is strictly less
		// than the section's inradius. A contained disk can certify success;
		// otherwise survey2d.go computes the same reading Wall.Minimum answers.
		inradius, enough, err := sectionInradius(offsetBudget, pp.profile, tmm, tDelta)
		if err != nil {
			return nil, err
		}
		// The accept boundary sits shellTol*max(1, inradius) below the limit —
		// a SCALE-RELATIVE rounding tolerance that grows with the part's scale,
		// not a fixed sub-nanometre margin — so the refusal reports the computed
		// accepted maximum thickness rather than the bare inradius (at a large
		// scale the two differ by far more than a noise floor).
		if maxT := inradius - shellTol*math.Max(1, inradius); !enough && tmm >= maxT {
			if maxT <= 0 {
				// The inradius itself is at or below the rounding tolerance, so
				// the accepted maximum is non-positive — no positive thickness
				// leaves a cavity. A smaller thickness cannot help; the section
				// must be enlarged.
				return nil, fmt.Errorf(`%w: the section's inradius %s is at or below the evaluator's rounding tolerance, so this evaluator accepts no positive shell thickness here (the tolerance-adjusted maximum is non-positive); enlarge the section`, ErrDegenerate, units.Millimeters(inradius))
			}
			return nil, fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the section's inradius %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, ErrDegenerate, t, units.Millimeters(maxT), units.Millimeters(inradius))
		}
		// The height limit: where a cap is kept (a cup), the wall behind it is a
		// floor t thick, so the cavity is swept over an interval of length h − t,
		// non-empty exactly when t < h. A both-caps shell keeps no cap and has no
		// floor, so only the section limit reaches it (§8).
		// The same scale-relative rounding tolerance as the section limit
		// (shellTol*max(1, h), growing with the part's scale, not a fixed
		// sub-nanometre margin): the refusal reports the computed accepted
		// maximum thickness rather than the bare height.
		if maxT := h - shellTol*math.Max(1, h); !bothCaps && tmm >= maxT {
			if maxT <= 0 {
				// The sweep height itself is at or below the rounding tolerance,
				// so the accepted maximum is non-positive — no positive thickness
				// leaves a floor cavity. A smaller thickness cannot help; the
				// sweep height must be increased.
				return nil, fmt.Errorf(`%w: the sweep height %s is at or below the evaluator's rounding tolerance, so this evaluator accepts no positive shell thickness here (the tolerance-adjusted maximum is non-positive); increase the sweep height`, ErrDegenerate, units.Millimeters(h))
			}
			return nil, fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the sweep height %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, ErrDegenerate, t, units.Millimeters(maxT), units.Millimeters(h))
		}
	}

	// The exact per-feature offset (§7). A dropped feature or a miter that does
	// not close is caught here — antecedent to the audit (S11a / S11) — before
	// there is any constructed section to audit.
	offset, err := offsetProfile(offsetBudget, pp.profile, s, tmm)
	if err != nil {
		return nil, err
	}

	// Stage 4 (§4/§5): the audit of the OFFSET section — S8 (orientation), then
	// the crossing test (a crossing or boundary contact of offset loops is S11b,
	// §8), then S9 (nesting). The shared §5 audit, run on decad's own synthesized
	// geometry. A shell mints no cutback, so S6 cannot fire.
	if err := auditOffsetSectionBudget(offsetBudget, pp.profile, offset); err != nil {
		if errors.Is(err, context.Canceled) {
			return nil, context.Canceled
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return nil, context.DeadlineExceeded
		}
		return nil, err
	}

	ref := d.nextProducerID()

	var body *Body
	switch {
	case bothCaps:
		if holed {
			// B4: the wall is one band around the outer loop plus one band lining
			// each hole — 1 + k disjoint lumps, which no prismPayload holds (S12).
			return nil, fmt.Errorf(`%w: a both-caps shell of a holed section is %d disjoint lumps; this evaluator has no multi-lump payload`, ErrUnsupported, 1+len(pp.profile.Holes))
		}
		body, err = evalTubeContext(ctx, d, ref, pp, offset, s)
	default:
		// A one-cap shell is a cup (B5/B6), for any k ≥ 0: the offset section's
		// loops are proven simple and correctly nested by the §5 audit above, and
		// evalCup wraps a wall around each post, all hanging off the one floor slab
		// (one lump). The holed BOTH-caps case keeps no floor and is 1 + k lumps
		// (B4, S12), refused above.
		body, err = evalCupContext(ctx, d, ref, cupPayloadFor(pp, offset, s, tmm, tDelta, removedEnd))
	}
	if err != nil {
		return nil, err
	}
	// Keep the consumed input aligned with document liveness at the commit edge.
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}

// shellTol is the closed-form degeneracy tolerance for the shell's own gates:
// a limit reached within it is treated as reached. The section is decad's own
// exact geometry, so this only absorbs float noise, never an admission.
const shellTol = 1e-9

// shellInradiusWorkLimit is S18's hard ceiling over one inward shell's
// candidate generation and whole-boundary validation. Candidate-family visits
// are checked against it before the wall kernel starts; every generated and
// validation visit then charges the same counter.
const shellInradiusWorkLimit uint64 = 1 << 20

// classifyRemovedCaps decides which caps a removed-face set names: every face
// must be a cap of the receiver (else S2, a side wall — ErrUnsupported), and it
// reports whether the start cap, the end cap, or both were removed.
func classifyRemovedCaps(b *Body, removed []*Face) (start, end bool, err error) {
	caps := map[*Face]string{}
	for _, f := range b.Faces() {
		for _, o := range f.origins {
			if o.producer == b.origin.producer && (o.Role == roleCapStart || o.Role == roleCapEnd) {
				caps[f] = o.Role
			}
		}
	}
	for _, f := range removed {
		role, ok := caps[f]
		if !ok {
			// S2: the cavity of a side-wall removal is the offset of an open
			// chain closed against the removed wall's own surface — a different
			// 2D machine.
			return false, false, fmt.Errorf(`%w: a shell that removes a side wall is not supported by this evaluator`, ErrUnsupported)
		}
		if role == roleCapStart {
			start = true
		} else {
			end = true
		}
	}
	return start, end, nil
}

// sectionInradius proves the requested thickness fits, or returns the largest
// inscribed disk of a recorded section from survey2d.go
// (docs/modify-design.md §8, the reading that answers Wall.Minimum). S18
// checks the candidate-family count before entering the kernel and shares one
// fixed work budget across its streamed generation and validation. An
// undecided or over-budget build-time gate is ErrUnsupported: it has no
// Suspect result to fall back on.
//
// The kernel publishes that inradius as an interval (wallSurveyOut), and this
// gate reads its midpoint: the caller's own accept boundary already sits a
// scale-relative shellTol below the limit — 1e-9 of the section's own size,
// decades above the aggregate's own half-width — so the interval cannot reach
// across a decision the margin has not already made.
func sectionInradius(budget *workBudget, profile ProfileRecord, thickness, thicknessDelta float64) (float64, bool, error) {
	if err := wallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	loops, err := recordLoopsBudget(budget, profile)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return 0, false, err
		}
		return 0, false, fmt.Errorf(`%w: this evaluator cannot read the shell section: %v`, ErrUnsupported, err)
	}
	var elems []surveyElem
	var verts [][2]float64
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].closed
		for _, w := range loop {
			if err := wallBudgetStep(budget); err != nil {
				return 0, false, err
			}
			el, ok := walkElem(w.segmentWalk)
			if !ok {
				return 0, false, fmt.Errorf(`%w: this evaluator cannot survey the shell section's curve type`, ErrUnsupported)
			}
			elems = append(elems, el)
			if single {
				continue
			}
			verts = append(verts, [2]float64{w.startU, w.startV})
		}
	}
	if err := wallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	candidateWork, ok := wallCandidateWork(len(elems), len(verts), false)
	if err := wallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	if !ok {
		return 0, false, fmt.Errorf(`%w: inward shell section survey candidate count overflows the checked work counter (fixed work budget %d)`, ErrUnsupported, shellInradiusWorkLimit)
	}
	if candidateWork > shellInradiusWorkLimit {
		return 0, false, fmt.Errorf(`%w: inward shell section survey needs %d candidate-family visits, above the fixed work budget of %d`, ErrUnsupported, candidateWork, shellInradiusWorkLimit)
	}
	// A contained disk can prove only the success side of S10. Failure and all
	// diagnostics still use the full inradius survey. The S18 count above runs
	// first even when this shorter proof succeeds.
	enough, err := shellRectCircleWitness(budget, profile, loops, thickness, thicknessDelta)
	if err != nil {
		return 0, false, err
	}
	if enough {
		return 0, true, nil
	}
	// fitMax is +Inf: the inradius is a property of the section alone, with no
	// height constraint (that constraint only bears on spanning, not the
	// largest inscribed disk).
	k, err := newWallKernelBudget(budget, elems, nil, verts, 0, exactScalar(0), false, math.Inf(1))
	if err != nil {
		return 0, false, err
	}
	out, err := k.runBudget(newWallWorkBudgetWithOperation(shellInradiusWorkLimit, budget))
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return 0, false, err
	}
	if errors.Is(err, errWallWorkBudget) {
		return 0, false, fmt.Errorf(`%w: inward shell section survey exceeded the fixed work budget of %d during candidate generation or validation`, ErrUnsupported, shellInradiusWorkLimit)
	}
	if err != nil {
		return 0, false, fmt.Errorf(`%w: inward shell section survey failed: %v`, ErrUnsupported, err)
	}
	if !out.ok {
		return 0, false, fmt.Errorf(`%w: this evaluator cannot prove the eroded section non-empty`, ErrUnsupported)
	}
	return out.inradius, false, nil
}

// shellRectCircleWitness proves a disk larger than the requested wall fits a
// rectangular section with circular holes. It tries nine exact-rational grid
// centers. A failed search says nothing: sectionInradius then runs its usual
// complete survey. Only recorded line endpoints with zero displacement and
// whole circles with bounded converted radii enter this proof.
func shellRectCircleWitness(budget *workBudget, profile ProfileRecord, loops [][]sideWalk, thickness, thicknessDelta float64) (bool, error) {
	if len(loops) == 0 || len(loops[0]) != 4 {
		return false, nil
	}
	outer := loops[0]
	minX, maxX := outer[0].startU, outer[0].startU
	minY, maxY := outer[0].startV, outer[0].startV
	for _, side := range outer {
		w := side.segmentWalk
		if w.kind != walkLine || w.startBound != (walkEndBound{}) || w.endBound != (walkEndBound{}) {
			return false, nil
		}
		minX = math.Min(minX, w.startU)
		maxX = math.Max(maxX, w.startU)
		minY = math.Min(minY, w.startV)
		maxY = math.Max(maxY, w.startV)
	}
	if !(minX < maxX && minY < maxY) ||
		isNonFinite(minX) || isNonFinite(maxX) || isNonFinite(minY) || isNonFinite(maxY) {
		return false, nil
	}
	var sides uint8
	for _, side := range outer {
		w := side.segmentWalk
		var bit uint8
		switch {
		case w.startU == minX && w.endU == minX &&
			((w.startV == minY && w.endV == maxY) || (w.startV == maxY && w.endV == minY)):
			bit = 1
		case w.startU == maxX && w.endU == maxX &&
			((w.startV == minY && w.endV == maxY) || (w.startV == maxY && w.endV == minY)):
			bit = 2
		case w.startV == minY && w.endV == minY &&
			((w.startU == minX && w.endU == maxX) || (w.startU == maxX && w.endU == minX)):
			bit = 4
		case w.startV == maxY && w.endV == maxY &&
			((w.startU == minX && w.endU == maxX) || (w.startU == maxX && w.endU == minX)):
			bit = 8
		default:
			return false, nil
		}
		if sides&bit != 0 {
			return false, nil
		}
		sides |= bit
	}
	if sides != 15 {
		return false, nil
	}
	type circle struct{ x, y, radius *big.Rat }
	holes := make([]circle, 0, len(loops)-1)
	for i, loop := range loops[1:] {
		if len(loop) != 1 || i >= len(profile.Holes) || len(profile.Holes[i].Segments) != 1 {
			return false, nil
		}
		segment, ok := profile.Holes[i].Segments[0].(CircleSeg)
		if !ok || segment.CCW {
			return false, nil
		}
		w := loop[0].segmentWalk
		if w.kind != walkCircular || !w.closed || w.radiusBound != 0 ||
			isNonFinite(w.cU) || isNonFinite(w.cV) || isNonFinite(w.radius) || w.radius <= 0 {
			return false, nil
		}
		radius, radiusDelta, err := magnitudeInBounded(segment.Radius, units.Length, units.Millimeter, "the hole radius")
		if err != nil {
			return false, err
		}
		if radius != w.radius || isNonFinite(radiusDelta) {
			return false, nil
		}
		radiusUpper := new(big.Rat).Add(floatRat(radius), floatRat(radiusDelta))
		holes = append(holes, circle{floatRat(w.cU), floatRat(w.cV), radiusUpper})
	}
	xlo, xhi, ylo, yhi := floatRat(minX), floatRat(maxX), floatRat(minY), floatRat(maxY)
	width := new(big.Rat).Sub(xhi, xlo)
	height := new(big.Rat).Sub(yhi, ylo)
	upper := new(big.Rat).Set(width)
	if height.Cmp(upper) < 0 {
		upper.Set(height)
	}
	upper.Quo(upper, big.NewRat(2, 1))
	if upper.Cmp(big.NewRat(1, 1)) < 0 {
		upper.SetInt64(1)
	}
	// Inradius is at most half the rectangle's narrower side. This threshold
	// therefore includes the full shellTol margin even though the true
	// inradius has not been computed.
	need := new(big.Rat).Add(floatRat(thickness), floatRat(thicknessDelta))
	need.Add(need, new(big.Rat).Mul(floatRat(shellTol), upper))
	quarters := [...]*big.Rat{big.NewRat(1, 4), big.NewRat(1, 2), big.NewRat(3, 4)}
	for _, u := range quarters {
		x := new(big.Rat).Add(xlo, new(big.Rat).Mul(width, u))
		for _, v := range quarters {
			if err := wallBudgetStep(budget); err != nil {
				return false, err
			}
			y := new(big.Rat).Add(ylo, new(big.Rat).Mul(height, v))
			fits := true
			for _, edge := range []*big.Rat{
				new(big.Rat).Sub(x, xlo), new(big.Rat).Sub(xhi, x),
				new(big.Rat).Sub(y, ylo), new(big.Rat).Sub(yhi, y),
			} {
				if edge.Cmp(need) <= 0 {
					fits = false
					break
				}
			}
			if !fits {
				continue
			}
			for _, hole := range holes {
				if err := wallBudgetStep(budget); err != nil {
					return false, err
				}
				dx, dy := new(big.Rat).Sub(x, hole.x), new(big.Rat).Sub(y, hole.y)
				distance2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
				separation := new(big.Rat).Add(hole.radius, need)
				if distance2.Cmp(new(big.Rat).Mul(separation, separation)) <= 0 {
					fits = false
					break
				}
			}
			if fits {
				return true, wallBudgetErr(budget)
			}
		}
	}
	return false, wallBudgetErr(budget)
}

// evalTube builds the both-caps hole-free shell (Table B, B2/B3): a plain prism
// over the annular section — {Outer: P, Hole: reverse(Q)} inward, {Outer: Q,
// Hole: reverse(P)} outward — so it IS a prismPayload, admitted as a receiver
// (R1) and first-class downstream (§12). The inner loop is walked as a hole
// (reversed sense), which is what makes its wall's material lie outside it.
func evalTubeContext(ctx context.Context, d *Document, ref producerID, pp prismPayload, offset ProfileRecord, s float64) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var outer, inner LoopRecord
	if s > 0 { // inward: P is the outside, Q the cavity
		outer = pp.profile.Outer
		inner = offset.Outer
	} else { // outward: Q is the outside, P the cavity
		outer = offset.Outer
		inner = pp.profile.Outer
	}
	holeLoop, err := reverseLoopRecordContext(ctx, inner)
	if err != nil {
		return nil, err
	}
	section := ProfileRecord{Outer: outer, Holes: []LoopRecord{holeLoop}}
	// The annular section is a NEW record no preflight has seen, so the build
	// opens its one counter here (docs/spline-design.md §5.2).
	// A tube keeps the receiver's sweep, so each end keeps its own axial
	// displacement too.
	return evalPrismContext(ctx, d, ref, prismPayload{
		profile: section,
		frame:   pp.frame,
		z0:      pp.z0,
		z1:      pp.z1,
		z0Delta: pp.z0Delta,
		z1Delta: pp.z1Delta,
		xform:   pp.xform,
	}, newFreeformWork())
}
