package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the shell of docs/modify-design.md §8: Body.Shell removes the
// selected cap faces of a straight prism and lines what remains with a wall of
// the given thickness. On a prism the whole construction is the recorded
// section's own exact offset (§7): P ⊖ t inward, P ⊕ t outward, per-feature and
// topology-preserving. A both-caps shell of a hole-free section is a TUBE — a
// plain prismPayload over the annular section {Outer, Hole} (Table B, B2/B3),
// so nothing downstream needs a new case for it (§12). A both-caps shell of a
// section with k holes is 1 + k disjoint bands, one stacked slab with one
// region per band (docs/modify-reach-design.md BX8). A one-cap shell is a CUP
// — cupPayload, a floor slab under a slab of wall bands (B5/B6).
//
// The gate order is §4's, and the sentinel of each refusal follows the §1
// existence test. Staged, never a wrong body: side-wall removal is S2 and a
// topology-changing offset is S11, each ErrUnsupported.

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
// built, so no unproven body is ever made. Removing both caps of a section
// with k holes returns 1 + k lumps: the band inside the outer loop, then one
// band lining each hole (docs/modify-reach-design.md BX8).
//
// A partial revolve is shelled when sel removes both of its angular caps: the
// wall is its meridian's offset swept over the same angle, and a meridian walk
// on the axis grows no wall (docs/modify-reach-design.md §9.3). A revolve
// also takes a side opening — one connected run of its generated side faces,
// on a full turn or beside both removed angular caps — where each opening end
// meets a straight removed walk at a right angle and the kept chain is one
// piece; the rim there is the removed face's own cut through the wall. Any
// other side selection, a kept angular cap, a holed meridian, a meridian
// meeting the axis along more than one walk and an offset reaching the axis
// are ErrUnsupported.
//
// WithNoOpenings asks for a closed hollow body that keeps every face; it is
// the one call form that takes a nil sel, and a non-nil sel beside it is
// ErrDegenerate (docs/modify-reach-design.md §2, SX1). A full-turn revolve
// builds it: the meridian's wall region swept a whole turn, an outer shell
// around one void shell (Shell.IsVoid) that is the cavity. So does a
// hole-free straight prism: both caps keep a floor t thick, so an inward wall
// needs t below half the sweep (SX11, ErrDegenerate). Every other receiver
// returns ErrUnsupported after the stage-1 gates pass — a holed prism section,
// and a partial revolve, since it keeps both angular caps. Two WithShellSense
// options naming different senses are ErrDegenerate (SX1).
//
// An analytic boolean result (a brep or stacked body) that reads as a prism
// along a reference axis is shelled as that prism
// (docs/brep-modify-design.md route P) when the removed faces are its caps;
// otherwise it is ErrUnsupported: SB3, the prism's own S2, where it reads as
// a prism, and SB10 where it reads as none.
func (b *Body) Shell(ctx context.Context, sel FaceSelector, t units.Value, opts ...ShellOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a shell`, ErrDegenerate)
	}
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
	o, err := decodeShellOptions(opts)
	if err != nil {
		return nil, err
	}
	sense := o.Sense
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. A typed nil query reads as an untyped nil.
	q, isQuery := sel.(*FaceQuery)
	nilSel := sel == nil || (isQuery && q == nil)
	// SX1 (docs/modify-reach-design.md §2): WithNoOpenings keeps every face,
	// so a selector naming faces to remove beside it names no single intent.
	if o.NoOpenings && !nilSel {
		return nil, errOptionConflict(`WithNoOpenings keeps every face, and a non-nil selector names faces to remove`)
	}
	tmm, tDelta, err := extent.MagnitudeInBounded(t, units.Length, units.Millimeter, "the shell thickness")
	if err != nil {
		return nil, err
	}
	if tmm == 0 {
		// S14: a face is removed and the wall is P \ P — the empty region, no
		// solid at all.
		return nil, fmt.Errorf(`%w: a zero-thickness shell leaves no wall`, ErrDegenerate)
	}
	s := 1.0 // inward: erode
	if sense == Outward {
		s = -1.0 // outward: dilate
	}
	if o.NoOpenings {
		// The one nil-selector shell: no face is removed, so there is no
		// selection to resolve. SX10 still leads for a cap-blend receiver.
		if err := requireNotCapBlendReceiver(b.payload, "shells"); err != nil {
			return nil, err
		}
		// Reach §9.3: a full turn over a hole-free meridian keeps every face
		// and sweeps its closed wall region over the whole turn.
		if rp, ok := b.payload.(revolvePayload); ok && rp.full && len(rp.profile.Holes) == 0 {
			return b.shellRevolve(ctx, rp, nil, s, t, tmm, tDelta)
		}
		// Reach BX5: a hole-free straight prism keeps both caps and lines
		// them with a wall, closing a cavity.
		if pp, ok := b.payload.(prismPayload); ok && len(pp.profile.Holes) == 0 {
			return b.shellClosedPrism(ctx, pp, s, t, tmm, tDelta)
		}
		return nil, refuseClosedShell(b.payload)
	}
	// Reject foreign implementations before invoking their callback.
	switch {
	case nilSel:
		return nil, errNilSelector
	case !isQuery:
		return nil, fmt.Errorf(`%w: the shell's face selector is not a decad face query (%T)`, ErrDegenerate, sel)
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
	// Reach RX2 (docs/modify-reach-design.md §9.3): a revolve receiver shells
	// its meridian and sweeps the wall over its own angular interval.
	if rp, ok := b.payload.(revolvePayload); ok {
		return b.shellRevolve(ctx, rp, removed, s, t, tmm, tDelta)
	}
	// A brep or stacked receiver takes the brep route
	// (docs/brep-modify-design.md §2), ahead of the generic refusal.
	route, err := modifyBrepReceiver(ctx, b, brepModifyRequest{op: "shells", shell: true,
		admits: func(_ prismPayload, caps prismCaps) error {
			_, _, err := classifyRemovedCaps(caps, removed)
			return err
		}})
	if err != nil {
		return nil, err
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every removed face
	// is a cap (S2).
	pp, ok := b.payload.(prismPayload)
	caps := prismCapsOf(b)
	if route.prism != nil {
		pp, ok, caps = *route.prism, true, route.caps
	}
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator shells a straight prism only`, ErrUnsupported)
	}
	if err := requireExactSection(pp, "shells"); err != nil {
		return nil, err
	}
	removedStart, removedEnd, err := classifyRemovedCaps(caps, removed)
	if err != nil {
		return nil, err
	}
	bothCaps := removedStart && removedEnd

	holed := len(pp.profile.Holes) > 0
	h := pp.z1 - pp.z0
	offsetBudget := proofbound.NewWorkBudget(ctx)
	if err := offsetBudget.Err(); err != nil {
		return nil, err
	}

	// Stage 3 (§4): the construction's own gates — S18 (the inward section
	// survey stays within its fixed work budget), S10 (the cavity is non-empty,
	// inward only), then S11a (no feature the offset drops as it is built).
	if s > 0 {
		// The section limit: P ⊖ t is non-empty exactly when t is strictly less
		// than the section's inradius. A contained disk can certify success;
		// otherwise internal/survey2d/wall_kernel.go computes the same reading Wall.Minimum answers.
		inradius, enough, err := sectionInradius(offsetBudget, pp.profile, tmm, tDelta)
		if err != nil {
			return nil, err
		}
		if err := requireSectionCavity(t, tmm, inradius, enough); err != nil {
			return nil, err
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
	case bothCaps && !holed:
		body, err = evalTubeContext(ctx, d, ref, pp, offset, s)
	case bothCaps:
		// B4 / reach BX8: the wall is one band around the outer loop plus one
		// band lining each hole — 1 + k disjoint lumps, one stacked slab with
		// one region per band.
		offsetDelta, derr := offsetSectionDelta(offsetBudget, pp.profile, s, tmm, tDelta)
		if derr != nil {
			return nil, shellCancelCause(derr)
		}
		body, err = evalShellBandsContext(ctx, d, ref, pp, offset, s, offsetDelta)
	default:
		// A one-cap shell is a cup (B5/B6), for any k ≥ 0: the offset section's
		// loops are proven simple and correctly nested by the §5 audit above, and
		// the cup wraps a wall band around each post, all hanging off the one
		// floor slab (one lump).
		// The offset section is a float evaluation of P ⊖ t*; the cup records
		// how far it may sit from that denoted offset beside it (§9).
		offsetDelta, derr := offsetSectionDelta(offsetBudget, pp.profile, s, tmm, tDelta)
		if derr != nil {
			return nil, shellCancelCause(derr)
		}
		cup, perr := cupPayloadFor(pp, offset, s, tmm, tDelta, offsetDelta, removedEnd).payload(ctx)
		if perr != nil {
			return nil, perr
		}
		body, err = evalCupContext(ctx, d, ref, cup)
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

// shellCancelCause maps an error wrapping a context cancellation to the bare
// context error, as every shell build stage reports it.
func shellCancelCause(err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return err
}

// shellWallBands lists the bands a shell's wall section holds between the
// outer region O and the cavity region C: the band between O's outer loop and
// C's (C's outer reversed as its one hole), then, for each hole i of O, the
// band between C's hole i (reversed, an outer) and O's hole i. Inward O is the
// receiver's section P and C its erosion; outward O is the dilation and C is
// P. The §5 audit has proven C's loops simple and nested in O's, so the bands
// are regular and pairwise disjoint (docs/modify-reach-design.md §9.2).
func shellWallBands(ctx context.Context, outer, cavity ProfileRecord) ([]ProfileRecord, error) {
	if len(outer.Holes) != len(cavity.Holes) {
		return nil, fmt.Errorf(`%w: the shell's outer and cavity regions have different loop counts`, ErrDegenerate)
	}
	cavityOuter, err := reverseLoopRecordContext(ctx, cavity.Outer)
	if err != nil {
		return nil, err
	}
	bands := []ProfileRecord{{Outer: outer.Outer, Holes: []LoopRecord{cavityOuter}}}
	for i, hole := range cavity.Holes {
		post, err := reverseLoopRecordContext(ctx, hole)
		if err != nil {
			return nil, err
		}
		bands = append(bands, ProfileRecord{Outer: post, Holes: []LoopRecord{outer.Holes[i]}})
	}
	return bands, nil
}

// evalShellBandsContext builds reach BX8: the both-caps shell of a section
// with k >= 1 holes is 1 + k disjoint bands over the receiver's sweep, one
// stacked slab holding one region per band (a prism group,
// docs/stacked-prism-design.md §2.2). Each band is its own lump with its own
// rim caps, capStart and capEnd, and its walls carry slab(0).region(m).side(i,j)
// with the outer band first and the hole linings in ProfileRecord order. The
// offset loops sit within offsetDelta of the offset the thickness denotes, so
// the payload carries it as its section displacement.
func evalShellBandsContext(ctx context.Context, d *Document, ref producerID, pp prismPayload, offset ProfileRecord, s, offsetDelta float64) (*Body, error) {
	outer, cavity := pp.profile, offset
	if s < 0 {
		outer, cavity = offset, pp.profile
	}
	bands, err := shellWallBands(ctx, outer, cavity)
	if err != nil {
		return nil, err
	}
	return evalStackedContext(ctx, d, ref, stackedPrismPayload{
		slabs: []prismSlab{{regions: bands, z0: pp.z0, z1: pp.z1, z0Delta: pp.z0Delta, z1Delta: pp.z1Delta}},
		frame: pp.frame, xform: pp.xform, sectionDelta: offsetDelta,
	})
}

// refuseClosedShell is Shell's answer to WithNoOpenings on every receiver but
// a full-turn revolve over a hole-free meridian, which shellRevolve builds,
// and a hole-free straight prism, which shellClosedPrism builds
// (docs/modify-reach-design.md Tables RX/SX, §14). Each receiver gets the row
// that stages it: a faceted boolean result SX9, a brep or stacked receiver
// SX16, a holed prism section or revolve meridian SX8, a partial revolve SX8
// (it keeps both angular caps), and any other receiver base S3. Every one is
// ErrUnsupported: the closed body exists, and this evaluator does not build it.
func refuseClosedShell(payload featurePayload) error {
	switch p := payload.(type) {
	case facetedPayload:
		return fmt.Errorf(`%w: this evaluator modifies no faceted boolean result (modify-reach SX9)`, ErrUnsupported)
	case brepPayload, stackedPrismPayload:
		return fmt.Errorf(`%w: this evaluator does not build a closed shell of a brep or stacked receiver (modify-reach SX16)`, ErrUnsupported)
	case prismPayload:
		// Shell routes a hole-free section to shellClosedPrism, so the prism
		// that arrives here is holed.
		return fmt.Errorf(`%w: a closed shell of a holed prism section is outside the shell extension (modify-reach SX8)`, ErrUnsupported)
	case revolvePayload:
		if len(p.profile.Holes) > 0 {
			return fmt.Errorf(`%w: a closed shell of a revolve whose meridian holds a hole is outside the shell extension (modify-reach SX8)`, ErrUnsupported)
		}
		// Shell routes a full turn over a hole-free meridian to shellRevolve,
		// so the revolve that arrives here is a partial turn.
		return fmt.Errorf(`%w: a closed shell of a partial revolve keeps both angular caps, whose walls are planes at constant distance that no revolve holds (modify-reach SX8)`, ErrUnsupported)
	default:
		return fmt.Errorf(`%w: this evaluator shells a straight prism only`, ErrUnsupported)
	}
}

// shellClosedPrism is reach BX5 (docs/modify-reach-design.md §9.2): WithNoOpenings
// on a hole-free straight prism keeps every face and lines it with a wall of
// thickness t, closing a cavity. The record is three stacked slabs: inward,
// the section P on [z0, z0 + t], the band between P and its erosion Q on
// [z0 + t, z1 − t], and P on [z1 − t, z1]; outward, the dilation Q on
// [z0 − t, z0], the band between Q and P on [z0, z1], and Q on [z1, z1 + t]. The
// two interfaces expose the cavity region (Q inward, P outward) as the
// cavity's floor and ceiling, and the cavity walls with them form one void
// shell inside the outer one (Shell.IsVoid).
//
// The gates are the cup's with both caps kept: S18 and S10's section limit
// (inward), then SX11, the axial cavity h − 2t, which must stay positive
// (inward; ErrDegenerate), then S11a as the offset is built and the §5 audit.
// The offset loops carry their proven displacement as the stack's section
// displacement.
func (b *Body) shellClosedPrism(ctx context.Context, pp prismPayload, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := b.doc
	if err := requireExactSection(pp, "shells"); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if s > 0 {
		inradius, enough, err := sectionInradius(budget, pp.profile, tmm, tDelta)
		if err != nil {
			return nil, err
		}
		if err := requireSectionCavity(t, tmm, inradius, enough); err != nil {
			return nil, err
		}
		// SX11: the two kept caps each eat t of the sweep, so the cavity's
		// axial height h − 2t is positive exactly when t < h/2, read with the
		// section limit's scale-relative rounding tolerance.
		h := pp.z1 - pp.z0
		if maxT := h/2 - shellTol*math.Max(1, h); tmm >= maxT {
			return nil, fmt.Errorf(`%w: the shell thickness %s leaves no cavity between the two kept caps of a %s sweep (the accepted maximum is %s, half the sweep less the evaluator's rounding tolerance; modify-reach SX11)`,
				ErrDegenerate, t, units.Millimeters(h), units.Millimeters(math.Max(maxT, 0)))
		}
	}
	offset, err := offsetProfile(budget, pp.profile, s, tmm)
	if err != nil {
		return nil, err
	}
	if err := auditOffsetSectionBudget(budget, pp.profile, offset); err != nil {
		return nil, shellCancelCause(err)
	}
	offsetDelta, err := offsetSectionDelta(budget, pp.profile, s, tmm, tDelta)
	if err != nil {
		return nil, shellCancelCause(err)
	}
	outer, cavity := pp.profile, offset
	if s < 0 {
		outer, cavity = offset, pp.profile
	}
	bands, err := shellWallBands(ctx, outer, cavity)
	if err != nil {
		return nil, err
	}
	// Each derived level carries its source end's displacement, the thickness
	// conversion and this float sum's rounding, as a cup's floor level does.
	step := func(from, delta, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))
	}
	lo, loDelta := step(pp.z0, pp.z0Delta, s*tmm)
	hi, hiDelta := step(pp.z1, pp.z1Delta, -s*tmm)
	caps := prismSlab{regions: []ProfileRecord{outer}}
	first, middle, last := caps, prismSlab{regions: bands}, caps
	if s > 0 {
		first.z0, first.z0Delta, first.z1, first.z1Delta = pp.z0, pp.z0Delta, lo, loDelta
		middle.z0, middle.z0Delta, middle.z1, middle.z1Delta = lo, loDelta, hi, hiDelta
		last.z0, last.z0Delta, last.z1, last.z1Delta = hi, hiDelta, pp.z1, pp.z1Delta
	} else {
		first.z0, first.z0Delta, first.z1, first.z1Delta = lo, loDelta, pp.z0, pp.z0Delta
		middle.z0, middle.z0Delta, middle.z1, middle.z1Delta = pp.z0, pp.z0Delta, pp.z1, pp.z1Delta
		last.z0, last.z0Delta, last.z1, last.z1Delta = pp.z1, pp.z1Delta, hi, hiDelta
	}
	exposed := []ProfileRecord{cavity}
	ref := d.nextProducerID()
	body, err := evalStackedContext(ctx, d, ref, stackedPrismPayload{
		slabs:        []prismSlab{first, middle, last},
		interfaces:   []prismSlabInterface{{lowerExposed: exposed}, {upperExposed: exposed}},
		frame:        pp.frame,
		xform:        pp.xform,
		sectionDelta: offsetDelta,
	})
	if err != nil {
		return nil, err
	}
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

// requireSectionCavity is S10's section limit (docs/modify-design.md §8): P ⊖ t
// is non-empty exactly when t is strictly less than the section's inradius.
// enough reports that a contained disk already proved the thickness fits.
// The accept boundary sits shellTol*max(1, inradius) below the limit — a
// SCALE-RELATIVE rounding tolerance that grows with the part's scale, not a
// fixed sub-nanometre margin — so the refusal reports the computed accepted
// maximum thickness rather than the bare inradius (at a large scale the two
// differ by far more than a noise floor).
func requireSectionCavity(t units.Value, tmm, inradius float64, enough bool) error {
	maxT := inradius - shellTol*math.Max(1, inradius)
	if enough || tmm < maxT {
		return nil
	}
	if maxT <= 0 {
		// The inradius itself is at or below the rounding tolerance, so the
		// accepted maximum is non-positive — no positive thickness leaves a
		// cavity. A smaller thickness cannot help; the section must be enlarged.
		return fmt.Errorf(`%w: the section's inradius %s is at or below the evaluator's rounding tolerance, so this evaluator accepts no positive shell thickness here (the tolerance-adjusted maximum is non-positive); enlarge the section`, ErrDegenerate, units.Millimeters(inradius))
	}
	return fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the section's inradius %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, ErrDegenerate, t, units.Millimeters(maxT), units.Millimeters(inradius))
}

// classifyRemovedCaps decides which caps a removed-face set names: every face
// must be one of the prism's two cap faces (else S2, a side wall —
// ErrUnsupported), and it reports whether the start cap, the end cap, or both
// were removed. caps names the receiver's own capStart/capEnd faces, or a
// brep receiver's route P caps (docs/brep-modify-design.md §4.2).
func classifyRemovedCaps(caps prismCaps, removed []*Face) (bool, bool, error) {
	var start, end bool
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
		case caps.end != nil && f == caps.end:
			end = true
		default:
			// S2: the cavity of a side-wall removal is the offset of an open
			// chain closed against the removed wall's own surface — a different
			// 2D machine.
			return false, false, fmt.Errorf(`%w: a shell that removes a side wall is not supported by this evaluator`, ErrUnsupported)
		}
	}
	return start, end, nil
}

// sectionInradius proves the requested thickness fits, or returns the largest
// inscribed disk of a recorded section from internal/survey2d/wall_kernel.go
// (docs/modify-design.md §8, the reading that answers Wall.Minimum). S18
// checks the candidate-family count before entering the kernel and shares one
// fixed work budget across its streamed generation and validation. An
// undecided or over-budget build-time gate is ErrUnsupported: it has no
// Suspect result to fall back on.
//
// The kernel publishes that inradius as an interval (survey2d.WallSurveyOut), and this
// gate reads its midpoint: the caller's own accept boundary already sits a
// scale-relative shellTol below the limit — 1e-9 of the section's own size,
// decades above the aggregate's own half-width — so the interval cannot reach
// across a decision the margin has not already made.
func sectionInradius(budget *proofbound.WorkBudget, profile ProfileRecord, thickness, thicknessDelta float64) (float64, bool, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	loops, err := recordLoopsBudget(budget, profile)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return 0, false, err
		}
		return 0, false, fmt.Errorf(`%w: this evaluator cannot read the shell section: %v`, ErrUnsupported, err)
	}
	var elems []survey2d.SurveyElem
	var verts [][2]float64
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].Closed
		for _, w := range loop {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, false, err
			}
			el, ok := walkElem(w.SegmentWalk)
			if !ok {
				return 0, false, fmt.Errorf(`%w: this evaluator cannot survey the shell section's curve type`, ErrUnsupported)
			}
			elems = append(elems, el)
			if single {
				continue
			}
			verts = append(verts, [2]float64{w.StartU, w.StartV})
		}
	}
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	if err := requireWallSurveyWork(budget, len(elems), len(verts)); err != nil {
		return 0, false, err
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
	inradius, err := wallSurveyInradius(budget, elems, verts)
	return inradius, false, err
}

// requireWallSurveyWork is S18's preflight: the inward section survey's
// candidate-family visits, counted under checked arithmetic before the wall
// kernel starts, must stay within shellInradiusWorkLimit.
func requireWallSurveyWork(budget *proofbound.WorkBudget, elems, verts int) error {
	candidateWork, ok := proofbound.WallCandidateWork(elems, verts, false)
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return err
	}
	if !ok {
		return fmt.Errorf(`%w: inward shell section survey candidate count overflows the checked work counter (fixed work budget %d)`, ErrUnsupported, shellInradiusWorkLimit)
	}
	if candidateWork > shellInradiusWorkLimit {
		return fmt.Errorf(`%w: inward shell section survey needs %d candidate-family visits, above the fixed work budget of %d`, ErrUnsupported, candidateWork, shellInradiusWorkLimit)
	}
	return nil
}

// wallSurveyInradius runs internal/survey2d/wall_kernel.go over a section's
// survey elements and returns its inradius, charging generation and validation
// to S18's one shared work budget. An over-budget or undecided survey is
// ErrUnsupported: a build-time gate has no Suspect result to fall back on.
func wallSurveyInradius(budget *proofbound.WorkBudget, elems []survey2d.SurveyElem, verts [][2]float64) (float64, error) {
	// fitMax is +Inf: the inradius is a property of the section alone, with no
	// height constraint (that constraint only bears on spanning, not the
	// largest inscribed disk).
	k, err := survey2d.NewWallKernelBudget(budget, elems, nil, verts, 0, proofbound.ExactScalar(0), false, math.Inf(1))
	if err != nil {
		return 0, err
	}
	out, err := k.RunBudget(proofbound.NewWallWorkBudgetWithOperation(shellInradiusWorkLimit, budget))
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return 0, err
	}
	if errors.Is(err, proofbound.ErrWallWorkBudget) {
		return 0, fmt.Errorf(`%w: inward shell section survey exceeded the fixed work budget of %d during candidate generation or validation`, ErrUnsupported, shellInradiusWorkLimit)
	}
	if err != nil {
		return 0, fmt.Errorf(`%w: inward shell section survey failed: %v`, ErrUnsupported, err)
	}
	if !out.Ok {
		return 0, fmt.Errorf(`%w: this evaluator cannot prove the eroded section non-empty`, ErrUnsupported)
	}
	return out.Inradius, nil
}

// shellRectCircleWitness passes the shell's recorded holes and unit conversion
// to the exact rectangular-section witness.
func shellRectCircleWitness(budget *proofbound.WorkBudget, profile ProfileRecord, loops [][]survey2d.SideWalk, thickness, thicknessDelta float64) (bool, error) {
	return survey2d.RectangleCircleWitness(budget, profile.Holes, loops, thickness, thicknessDelta, shellTol,
		func(radius units.Value) (float64, float64, error) {
			return extent.MagnitudeInBounded(radius, units.Length, units.Millimeter, "the hole radius")
		})
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
	}, freeform.NewFreeformWork())
}
