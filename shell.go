package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/units"
)

// This file is the shell of agent-docs/modify-design.md §8: Shell removes the
// selected cap faces of a straight prism or a certified one-line Sweep prism
// reduction and lines what remains with a wall of the given thickness.
// On a prism the whole construction is the recorded
// section's own exact offset (§7): P ⊖ t inward, P ⊕ t outward, per-feature and
// topology-preserving. A both-caps shell of a hole-free section is a TUBE — a
// plain prismPayload over the annular section {Outer, Hole} (Table B, B2/B3),
// so nothing downstream needs a new case for it (§12). A both-caps shell of a
// section with k holes is 1 + k disjoint bands, one stacked slab with one
// region per band (agent-docs/modify-reach-design.md BX8). A one-cap shell is a CUP
// — cupPayload, a floor slab under a slab of wall bands (B5/B6).
//
// The gate order is §4's, and the sentinel of each refusal follows the §1
// existence test. Staged, never a wrong body: a topology-changing offset is
// S11, ErrUnsupported. Removing side faces of a prism is a side opening,
// agent-docs/shell-opening-design.md's (shell_opening.go).

// ShellOption configures Shell, including its wall sense.
type ShellOption interface{ shellOption() }

// ShellSense is the wall sense of a shell (agent-docs/modify-design.md §8): the
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
// is Inward (agent-docs/modify-design.md §8).
func WithShellSense(s ShellSense) ShellOption {
	return shellOptionValue{encoded: modifyoption.WithShellSense(int(s))}
}

// Shell removes the faces of b that sel selects and lines the rest
// with a wall of thickness t, returning the new body and retiring b
// (agent-docs/modify-design.md §8, core §8). A nil b is ErrDegenerate. sel names the
// faces to REMOVE — the openings — resolved against the live b; a query matching nothing is
// loud (ErrNoMatch / ErrCardinality, S16). t is a length magnitude, gated like
// every other (S15); a zero t is S14 (the wall is the empty region). A
// receiver outside the admitted prism, straight Sweep, revolve, brep and draft routes is S3,
// ErrUnsupported. A straight prism's offset
// section faces the §5 audit before anything is built, so no unproven body is
// ever made. Removing both caps of a section with k holes returns 1 + k
// lumps: the band inside the outer loop, then one band lining each hole
// (agent-docs/modify-reach-design.md BX8).
//
// A prism or one-line solid Sweep also takes a side opening
// (agent-docs/shell-opening-design.md): one
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
// on the axis grows no wall (agent-docs/modify-reach-design.md §9.3). A revolve
// also takes a side opening — one connected run of its generated side faces,
// on a full turn or beside both removed angular caps — where the kept chain is
// one piece; the rim at each opening end is the removed face's own cut
// through the wall (agent-docs/shell-opening-design.md §8). Any
// other side selection, a kept angular cap, a holed meridian, a meridian
// meeting the axis along more than one walk and an offset reaching the axis
// are ErrUnsupported.
//
// A faceted solid whose every face squares to a world axis and whose held vertices carry no bound,
// such as a part read from STL and made a solid by SolidFromMesh, shells inward on a grid:
// the cavity is the solid eroded by a cube of side twice t, so the walls are mitered at
// every edge, the removed faces open it, and the result is a mesh solid whose axis faces keep
// their planes (agent-docs/orthogonal-shell-design.md). A wall is t thick to the rounding
// of x ± t; walls that meet are ErrDegenerate. An Outward shell of such a solid wraps it
// instead, the solid dilated by a cube less itself, ending flush with a removed face's plane.
//
// A positive-taper one-sided full-circle draft with its far cap removed
// inward builds a conical cup through draft §10.6's bounded meridian.
//
// WithNoOpenings asks for a closed hollow body that keeps every face; it is
// the one call form that takes a nil sel, and a non-nil sel beside it is
// ErrDegenerate (agent-docs/modify-reach-design.md §2, SX1). A full-turn revolve
// builds it: the meridian's wall region swept a whole turn, an outer shell
// around one void shell (FaceShell.IsVoid) that is the cavity. So does a
// straight prism: both caps keep a floor t thick, so an inward wall needs t
// below half the sweep (SX11, ErrDegenerate). A holed section builds inward
// only, its cavity a ring round each hole's lining, and so does a brep result
// that reads as a prism along some reference axis, such as a drilled bar
// (modify-general §3.2). A two-slab rectangular boss Union builds the bounded
// inward cavity of modify-general §3.1a. Other receivers return
// ErrUnsupported after the stage-1 gates pass — an outward shell of a holed
// prism section (SX8), a partial revolve, and brep or stacked results outside
// those classes (SG2). Two WithShellSense
// options naming different senses are ErrDegenerate (SX1).
//
// An analytic boolean result (a brep or stacked body) that reads as a prism
// along a reference axis is shelled as that prism
// (agent-docs/brep-modify-design.md route P) when the removed faces are its caps.
// A brep that reads as a prism along one reference axis cut by through tools
// along others is shelled inward with one or both of that prism's caps, one
// connected run of its straight side walls along section axes, or both
// removed (agent-docs/modify-general-design.md route S): the cavity is the eroded
// prism — with a wall run, the side opening's cavity section
// (agent-docs/shell-opening-design.md) — cut by each tool dilated by t, and the
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
func Shell(ctx context.Context, b *Body, sel FaceSelector, t units.Value,
	opts ...ShellOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationShell, retErr) }()
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
	// Table X (agent-docs/surface-design.md §11): refused ahead of the selector gate
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
	// SX1 (agent-docs/modify-reach-design.md §2): WithNoOpenings keeps every face,
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
	call := brepmodify.ShellCall{NoOpenings: o.NoOpenings, Inward: sense == Inward, T: t, Tmm: tmm, TDelta: tDelta,
		CurveName: publicCurveName}
	if !o.NoOpenings {
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
		call.Removed = faceNodes(removed)
	}
	if out, ok, err := shellOrthogonal(ctx, d, b, call); ok || err != nil {
		return out, err
	}
	node, err := brepmodify.Shell(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), &b.n, call)
	if err != nil {
		return nil, err
	}
	return commitModifyResult(ctx, OperationShell, b, bodyHandle(node))
}

// shellTol is the closed-form degeneracy tolerance for the shell's own gates
// and for the exact section offset of agent-docs/modify-design.md §7
// (offset2d.OffsetProfile and offset2d.OffsetSectionDelta, which hold the
// construction, its S11a/S11 refusals and the §5 audit): offset2d.ShellTol,
// the value the cap-band builder reads too.
const shellTol = offset2d.ShellTol
