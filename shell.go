package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/freeform"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
)

// This file is the shell of docs/modify-design.md §8: Shell removes the
// selected cap faces of a straight prism or a certified one-line Sweep prism
// reduction and lines what remains with a wall of the given thickness.
// On a prism the whole construction is the recorded
// section's own exact offset (§7): P ⊖ t inward, P ⊕ t outward, per-feature and
// topology-preserving. A both-caps shell of a hole-free section is a TUBE — a
// plain prismPayload over the annular section {Outer, Hole} (Table B, B2/B3),
// so nothing downstream needs a new case for it (§12). A both-caps shell of a
// section with k holes is 1 + k disjoint bands, one stacked slab with one
// region per band (docs/modify-reach-design.md BX8). A one-cap shell is a CUP
// — cupPayload, a floor slab under a slab of wall bands (B5/B6).
//
// The gate order is §4's, and the sentinel of each refusal follows the §1
// existence test. Staged, never a wrong body: a topology-changing offset is
// S11, ErrUnsupported. Removing side faces of a prism is a side opening,
// docs/shell-opening-design.md's (shell_opening.go).

// ShellOption configures Shell, including its wall sense.
type ShellOption interface{ shellOption() }

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

// WithShellSense sets the wall sense (Inward or Outward). Without it the sense
// is Inward (docs/modify-design.md §8).
func WithShellSense(s ShellSense) ShellOption {
	return shellOptionValue{encoded: modifyoption.WithShellSense(int(s))}
}

// Shell removes the faces of b that sel selects and lines the rest
// with a wall of thickness t, returning the new body and retiring b
// (docs/modify-design.md §8, core §8). A nil b is ErrDegenerate. sel names the
// faces to REMOVE — the openings — resolved against the live b; a query matching nothing is
// loud (ErrNoMatch / ErrCardinality, S16). t is a length magnitude, gated like
// every other (S15); a zero t is S14 (the wall is the empty region). A
// receiver outside the admitted prism, straight Sweep, revolve, brep and draft routes is S3,
// ErrUnsupported. A straight prism's offset
// section faces the §5 audit before anything is built, so no unproven body is
// ever made. Removing both caps of a section with k holes returns 1 + k
// lumps: the band inside the outer loop, then one band lining each hole
// (docs/modify-reach-design.md BX8).
//
// A prism or one-line solid Sweep also takes a side opening
// (docs/shell-opening-design.md): one
// connected run of its outer side faces removed, with or without its caps, on
// a hole-free section whose every walk is a line, along a section axis or
// oblique, or a circular arc. Each end of the kept walks closes on the removed
// face's own plane or cylinder, cut by the kept wall's offset. With both caps
// removed the result is a prism over the wall section; otherwise it is an
// analytic face record with face(k)/wall(k) roles, an axis-aligned removed
// face's plane holding the opening as its hole and an oblique one stated as
// coplanar pieces. A removed hole wall, two runs, every side face, or a holed
// section is ErrUnsupported (SO6); so are a kept arc meeting a removed arc at
// an end corner, an oblique removed end face recorded as several segments,
// and, under a kept cap, a rim cut on a removed arc at a point that is not
// exactly on the arc's circle (SO5). Inward, the kept caps must leave a
// cavity height (SO3, ErrDegenerate).
//
// A partial revolve is shelled when sel removes both of its angular caps: the
// wall is its meridian's offset swept over the same angle, and a meridian walk
// on the axis grows no wall (docs/modify-reach-design.md §9.3). A revolve
// also takes a side opening — one connected run of its generated side faces,
// on a full turn or beside both removed angular caps — where the kept chain is
// one piece; the rim at each opening end is the removed face's own cut
// through the wall (docs/shell-opening-design.md §8). Any
// other side selection, a kept angular cap, a holed meridian, a meridian
// meeting the axis along more than one walk and an offset reaching the axis
// are ErrUnsupported.
//
// A positive-taper one-sided full-circle draft with its far cap removed
// inward builds a conical cup through draft §10.6's bounded meridian.
//
// WithNoOpenings asks for a closed hollow body that keeps every face; it is
// the one call form that takes a nil sel, and a non-nil sel beside it is
// ErrDegenerate (docs/modify-reach-design.md §2, SX1). A full-turn revolve
// builds it: the meridian's wall region swept a whole turn, an outer shell
// around one void shell (FaceShell.IsVoid) that is the cavity. So does a
// hole-free straight prism: both caps keep a floor t thick, so an inward wall
// needs t below half the sweep (SX11, ErrDegenerate). A two-slab rectangular
// boss Union builds the bounded inward cavity of modify-general §3.1a. Other
// receivers return ErrUnsupported after the stage-1 gates pass — a holed
// prism section, a partial revolve, and brep or stacked results outside that
// boss class (SG2). Two WithShellSense
// options naming different senses are ErrDegenerate (SX1).
//
// An analytic boolean result (a brep or stacked body) that reads as a prism
// along a reference axis is shelled as that prism
// (docs/brep-modify-design.md route P) when the removed faces are its caps.
// A brep that reads as a prism along one reference axis cut by through tools
// along others is shelled inward with one or both of that prism's caps, one
// connected run of its straight side walls along section axes, or both
// removed (docs/modify-general-design.md route S): the cavity is the eroded
// prism — with a wall run, the side opening's cavity section
// (docs/shell-opening-design.md) — cut by each tool dilated by t, and the
// result is a brep holding the receiver's kept faces, the cavity's faces and
// the rim at each removed face. One whole-circle tool wall through a rectangular
// box builds a three-section shell (modify-general §3.1d). On other records,
// an outward shell (SG1), a removed tool or hole wall (SG4), a removed curved
// outer wall or a wall run ending at a reflex corner (SG5), and dilated tools
// that meet, reach a cap of the cavity or the material past the walls they pierce (SG6) are
// ErrUnsupported, and a wall run refuses as a prism side opening does
// (SO1–SO6, with their sentinels). A brep or
// stacked result that reads as neither is ErrUnsupported: SB3, the prism's own
// S2, where it reads as a prism, and SB10 where it does not, each naming the
// first face route S does not take (SG3). One exact exception is a pair of
// nested rectangular slabs: removing the boss top inward builds the plate
// and boss cavity with four cylinder root patches (modify-general §3.1a).
// A coaxial two-level counterbore has a separate inward route (§3.1e).
func Shell(ctx context.Context, b *Body, sel FaceSelector, t units.Value, opts ...ShellOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a shell`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)

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
	sense := ShellSense(o.Sense)
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. A typed nil query reads as an untyped nil.
	q, isQuery := sel.(*FaceQuery)
	nilSel := sel == nil || (isQuery && q == nil)
	// SX1 (docs/modify-reach-design.md §2): WithNoOpenings keeps every face,
	// so a selector naming faces to remove beside it names no single intent.
	if o.NoOpenings && !nilSel {
		return nil, modifyoption.ErrOptionConflict(`WithNoOpenings keeps every face, and a non-nil selector names faces to remove`)
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
		if err := requireNotCapBlendReceiver(b.n.Payload, "shells"); err != nil {
			return nil, err
		}
		// Reach §9.3: a full turn over a hole-free meridian keeps every face
		// and sweeps its closed wall region over the whole turn.
		if rp, ok := b.n.Payload.(revolvePayload); ok && rp.full && len(rp.profile.Holes) == 0 {
			return b.shellRevolve(ctx, rp, nil, s, t, tmm, tDelta)
		}
		// Reach BX5: a hole-free straight prism keeps both caps and lines
		// them with a wall, closing a cavity.
		if pp, ok := b.n.Payload.(prismPayload); ok && len(pp.Profile.Holes) == 0 {
			return b.shellClosedPrism(ctx, pp, s, t, tmm, tDelta)
		}
		if pp, ok := straightSweepPrismReduction(b.n.Payload); ok && len(pp.Profile.Holes) == 0 {
			return b.shellClosedPrism(ctx, pp, s, t, tmm, tDelta)
		}
		if sp, ok := b.n.Payload.(stackedPrismPayload); ok && sense == Inward {
			view, _, err := brepOfStackedWithRoles(ctx, sp)
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, ctxErr
			}
			if err == nil {
				body, matched, err := shellStackedBoss(ctx, b, sp, view,
					brepShellCall{sense: sense, t: t, tmm: tmm, tDelta: tDelta})
				if !matched && err == nil {
					reversed, eligible, reverseErr := reverseClosedBossStack(ctx, sp)
					if reverseErr != nil {
						return nil, reverseErr
					}
					if eligible {
						reversedView, _, viewErr := brepOfStackedWithRoles(ctx, reversed)
						if viewErr != nil {
							return nil, viewErr
						}
						body, matched, err = shellStackedBoss(ctx, b, reversed, reversedView,
							brepShellCall{sense: sense, t: t, tmm: tmm, tDelta: tDelta})
					}
				}
				if matched || err != nil {
					if err != nil {
						return nil, err
					}
					return commitModifyResult(ctx, b, body)
				}
			}
		}
		return nil, refuseClosedShell(b.n.Payload)
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
	if err := requireNotCapBlendReceiver(b.n.Payload, "shells"); err != nil {
		return nil, err
	}
	if out, taken, err := tryDraftCircleShell(ctx, b, removed, sense, tmm, tDelta); taken {
		return out, err
	}
	if err := requireNotDraftReceiver(b.n.Payload, "shells"); err != nil {
		return nil, err
	}
	// Reach RX2 (docs/modify-reach-design.md §9.3): a revolve receiver shells
	// its meridian and sweeps the wall over its own angular interval.
	if rp, ok := b.n.Payload.(revolvePayload); ok {
		return b.shellRevolve(ctx, rp, removed, s, t, tmm, tDelta)
	}
	if tp, ok := b.n.Payload.(twistedSweepPayload); ok {
		return b.shellTwistedSweep(ctx, tp, removed, sense, tmm, tDelta)
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok && fp.prismUnionFillet != nil {
		if out, matched, err := b.shellPrismUnionBoss(ctx, fp, removed, sense, tmm, tDelta); matched || err != nil {
			return out, err
		}
	}
	// A brep or stacked receiver takes the brep route
	// (docs/brep-modify-design.md §2), ahead of the generic refusal.
	route, err := modifyBrepReceiver(ctx, b, brepModifyRequest{op: "shells", shell: true,
		admits: func(_ prismPayload, caps prismCaps) error {
			_, _, err := classifyRemovedCaps(caps, removed)
			return err
		},
		shellCall: brepShellCall{removed: removed, sense: sense, t: t, tmm: tmm, tDelta: tDelta}})
	if err != nil {
		return nil, err
	}
	if route.body != nil {
		return commitModifyResult(ctx, b, route.body)
	}

	// Stage 2 (§4): the receiver's payload class (S3), then the removed faces:
	// caps by role, and side faces by the receiver's exact prism segment,
	// which routes the call to the side opening (shell-opening §5). A brep
	// receiver's route P takes caps only (SB3).
	pp, ok := b.n.Payload.(prismPayload)
	caps := prismCapsOf(b)
	var sweepStraight bool
	if route.prism != nil {
		pp, ok, caps = *route.prism, true, route.caps
	} else if sweepPrism, admitted := straightSweepPrismReduction(b.n.Payload); admitted {
		pp, ok, sweepStraight = sweepPrism, true, true
	}
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator shells a straight prism or its one-line Sweep reduction only`, ErrUnsupported)
	}
	if err := pp.RequireExactSection("shells"); err != nil {
		return nil, err
	}
	var removedStart, removedEnd bool
	var sides map[int]struct{}
	switch {
	case route.prism != nil:
		removedStart, removedEnd, err = classifyRemovedCaps(caps, removed)
	case sweepStraight:
		removedStart, removedEnd, sides, err = classifyRemovedSweepFaces(b, caps, pp, removed)
	default:
		removedStart, removedEnd, sides, err = classifyRemovedFaces(b, caps, removed)
	}
	if err == nil && len(sides) > 0 {
		return b.shellSideOpening(ctx, pp, removedStart, removedEnd, sides, s, t, tmm, tDelta)
	}
	if err != nil {
		return nil, err
	}
	bothCaps := removedStart && removedEnd

	holed := len(pp.Profile.Holes) > 0
	h := pp.Z1 - pp.Z0
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
		inradius, enough, err := shellsurvey.SectionInradius(offsetBudget, boundarywalk.Profile(pp.Profile), tmm, tDelta, shellTol)
		if err != nil {
			return nil, err
		}
		if err := shellsurvey.RequireSectionCavity(t, tmm, inradius, enough, shellTol); err != nil {
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
	offset, err := offsetProfile(offsetBudget, pp.Profile, s, tmm)
	if err != nil {
		return nil, err
	}

	// Stage 4 (§4/§5): the audit of the OFFSET section — S8 (orientation), then
	// the crossing test (a crossing or boundary contact of offset loops is S11b,
	// §8), then S9 (nesting). The shared §5 audit, run on decad's own synthesized
	// geometry. A shell mints no cutback, so S6 cannot fire.
	if err := offset2d.AuditOffsetSectionBudget(offsetBudget, pp.Profile, offset); err != nil {
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
		offsetDelta, derr := offsetSectionDelta(offsetBudget, pp.Profile, s, tmm, tDelta)
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
		offsetDelta, derr := offsetSectionDelta(offsetBudget, pp.Profile, s, tmm, tDelta)
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

// evalShellBandsContext builds reach BX8: the both-caps shell of a section
// with k >= 1 holes is 1 + k disjoint bands over the receiver's sweep, one
// stacked slab holding one region per band (a prism group,
// docs/stacked-prism-design.md §2.2). Each band is its own lump with its own
// rim caps, capStart and capEnd, and its walls carry slab(0).region(m).side(i,j)
// with the outer band first and the hole linings in ProfileRecord order. The
// offset loops sit within offsetDelta of the offset the thickness denotes, so
// the payload carries it as its section displacement.
func evalShellBandsContext(ctx context.Context, d *Document, ref producerID, pp prismPayload, offset profileRecord, s, offsetDelta float64) (*Body, error) {
	outer, cavity := pp.Profile, offset
	if s < 0 {
		outer, cavity = offset, pp.Profile
	}
	bands, err := prismshell.ShellWallBands(ctx, outer, cavity)
	if err != nil {
		return nil, err
	}
	return evalStackedContext(ctx, d, ref, stackedPrismPayload{
		slabs: []stackedrecord.Slab{{Regions: bands, Z0: pp.Z0, Z1: pp.Z1, Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta}},
		frame: pp.Frame, xform: pp.Xform, sectionDelta: offsetDelta,
	})
}

// refuseClosedShell is Shell's answer to WithNoOpenings on every receiver but
// a full-turn revolve over a hole-free meridian, a hole-free straight prism,
// and an admitted two-slab rectangular boss. Their own builders run first
// (docs/modify-reach-design.md Tables RX/SX, §14). Each receiver gets the row
// that stages it: a faceted boolean result SX9, a brep or stacked receiver
// docs/modify-general-design.md's SG2, a holed prism section or revolve
// meridian SX8, a partial revolve SX8 (it keeps both angular caps), and any
// other receiver base S3. Every one is
// ErrUnsupported: the closed body exists, and this evaluator does not build it.
func refuseClosedShell(payload topo.Payload) error {
	switch p := payload.(type) {
	case facetedPayload:
		return fmt.Errorf(`%w: this evaluator modifies no faceted boolean result (modify-reach SX9)`, ErrUnsupported)
	case brepPayload, stackedPrismPayload:
		return fmt.Errorf(`%w: this evaluator does not build a closed shell of this brep or stacked receiver; it has no certified void-shell construction (modify-general SG2)`, ErrUnsupported)
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
// shell inside the outer one (FaceShell.IsVoid).
//
// The gates are the cup's with both caps kept: S18 and S10's section limit
// (inward), then SX11, the axial cavity h − 2t, which must stay positive
// (inward; ErrDegenerate), then S11a as the offset is built and the §5 audit.
// The offset loops carry their proven displacement as the stack's section
// displacement.
func (b *Body) shellClosedPrism(ctx context.Context, pp prismPayload, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := documentHandle(b.n.Doc)
	if err := pp.RequireExactSection("shells"); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if s > 0 {
		inradius, enough, err := shellsurvey.SectionInradius(budget, boundarywalk.Profile(pp.Profile), tmm, tDelta, shellTol)
		if err != nil {
			return nil, err
		}
		if err := shellsurvey.RequireSectionCavity(t, tmm, inradius, enough, shellTol); err != nil {
			return nil, err
		}
		// SX11: the two kept caps each eat t of the sweep, so the cavity's
		// axial height h − 2t is positive exactly when t < h/2, read with the
		// section limit's scale-relative rounding tolerance.
		h := pp.Z1 - pp.Z0
		if maxT := h/2 - shellTol*math.Max(1, h); tmm >= maxT {
			return nil, fmt.Errorf(`%w: the shell thickness %s leaves no cavity between the two kept caps of a %s sweep (the accepted maximum is %s, half the sweep less the evaluator's rounding tolerance; modify-reach SX11)`,
				ErrDegenerate, t, units.Millimeters(h), units.Millimeters(math.Max(maxT, 0)))
		}
	}
	offset, err := offsetProfile(budget, pp.Profile, s, tmm)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, pp.Profile, offset); err != nil {
		return nil, shellCancelCause(err)
	}
	offsetDelta, err := offsetSectionDelta(budget, pp.Profile, s, tmm, tDelta)
	if err != nil {
		return nil, shellCancelCause(err)
	}
	outer, cavity := pp.Profile, offset
	if s < 0 {
		outer, cavity = offset, pp.Profile
	}
	bands, err := prismshell.ShellWallBands(ctx, outer, cavity)
	if err != nil {
		return nil, err
	}
	// Each derived level carries its source end's displacement, the thickness
	// conversion and this float sum's rounding, as a cup's floor level does.
	step := func(from, delta, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))
	}
	lo, loDelta := step(pp.Z0, pp.Z0Delta, s*tmm)
	hi, hiDelta := step(pp.Z1, pp.Z1Delta, -s*tmm)
	caps := stackedrecord.Slab{Regions: []profileRecord{outer}}
	first, middle, last := caps, stackedrecord.Slab{Regions: bands}, caps
	if s > 0 {
		first.Z0, first.Z0Delta, first.Z1, first.Z1Delta = pp.Z0, pp.Z0Delta, lo, loDelta
		middle.Z0, middle.Z0Delta, middle.Z1, middle.Z1Delta = lo, loDelta, hi, hiDelta
		last.Z0, last.Z0Delta, last.Z1, last.Z1Delta = hi, hiDelta, pp.Z1, pp.Z1Delta
	} else {
		first.Z0, first.Z0Delta, first.Z1, first.Z1Delta = lo, loDelta, pp.Z0, pp.Z0Delta
		middle.Z0, middle.Z0Delta, middle.Z1, middle.Z1Delta = pp.Z0, pp.Z0Delta, pp.Z1, pp.Z1Delta
		last.Z0, last.Z0Delta, last.Z1, last.Z1Delta = pp.Z1, pp.Z1Delta, hi, hiDelta
	}
	exposed := []profileRecord{cavity}
	ref := d.nextProducerID()
	body, err := evalStackedContext(ctx, d, ref, stackedPrismPayload{
		slabs:        []stackedrecord.Slab{first, middle, last},
		interfaces:   []stackedrecord.Interface{{LowerExposed: exposed}, {UpperExposed: exposed}},
		frame:        pp.Frame,
		xform:        pp.Xform,
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

// classifyRemovedCaps decides which caps a removed-face set names on a brep
// receiver's route P (docs/brep-modify-design.md §4.2): every face must be one
// of the recognised prism's two cap faces, else ErrUnsupported, which the
// route reports as SB3. It reports whether the start cap, the end cap, or
// both were removed. A prism receiver classifies its removed faces with
// classifyRemovedFaces instead, which routes side faces to the side opening.
func classifyRemovedCaps(caps prismCaps, removed []*Face) (bool, bool, error) {
	var start, end bool
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
		case caps.end != nil && f == caps.end:
			end = true
		default:
			// Route P maps removed faces to the recognised prism's caps only
			// (shell-opening SO7).
			return false, false, fmt.Errorf(`%w: a shell that removes a side wall is not supported by this evaluator`, ErrUnsupported)
		}
	}
	return start, end, nil
}

// evalTube builds the both-caps hole-free shell (Table B, B2/B3): a plain prism
// over the annular section — {Outer: P, Hole: reverse(Q)} inward, {Outer: Q,
// Hole: reverse(P)} outward — so it IS a prismPayload, admitted as a receiver
// (R1) and first-class downstream (§12). The inner loop is walked as a hole
// (reversed sense), which is what makes its wall's material lie outside it.
func evalTubeContext(ctx context.Context, d *Document, ref producerID, pp prismPayload, offset profileRecord, s float64) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var outer, inner loopRecord
	if s > 0 { // inward: P is the outside, Q the cavity
		outer = pp.Profile.Outer
		inner = offset.Outer
	} else { // outward: Q is the outside, P the cavity
		outer = offset.Outer
		inner = pp.Profile.Outer
	}
	holeLoop, err := offset2d.ReverseLoopRecordContext(ctx, inner)
	if err != nil {
		return nil, err
	}
	section := profileRecord{Outer: outer, Holes: []loopRecord{holeLoop}}
	// The annular section is a NEW record no preflight has seen, so the build
	// opens its one counter here (docs/spline-design.md §5.2).
	// A tube keeps the receiver's sweep, so each end keeps its own axial
	// displacement too.
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile: section,
		Frame:   pp.Frame,
		Z0:      pp.Z0,
		Z1:      pp.Z1,
		Z0Delta: pp.Z0Delta,
		Z1Delta: pp.Z1Delta,
		Xform:   pp.Xform,
	}, freeform.NewFreeformWork()))
}
