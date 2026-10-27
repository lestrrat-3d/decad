package prismshell

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// CapShellInput is a straight prism shell that removes caps only or keeps
// every face (agent-docs/modify-design.md §8, agent-docs/modify-reach-design.md BX5/BX8):
// the receiver's exact record and the held thickness.
type CapShellInput struct {
	Prism prismrecord.Payload
	// KeptCaps is 0 when both caps are removed, 1 for a cup, and 2 for
	// WithNoOpenings, which keeps both.
	KeptCaps int
	// Sense is +1 inward (erode) and -1 outward (dilate).
	Sense          float64
	Thickness      units.Value
	HeldThickness  float64
	ThicknessDelta float64
}

// CancelCause returns context.Canceled or context.DeadlineExceeded unwrapped
// when err wraps one, and err unchanged otherwise. Shell callers apply it to
// some stages only: the offset section audit, the offset's displacement
// bound, the side-opening regions and build, and route S's rim regions and
// rim audit. Other stages, such as the section inradius survey and the offset
// construction, return their errors unmapped.
func CancelCause(err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return err
}

// OffsetSection runs §4's construction gates and returns the offset section
// Q: S18 and S10's section limit, then the floor limit each kept cap adds
// (both inward only), then the exact per-feature offset of §7 (S11a), then
// §5's audit of Q (S8, S11b, S9).
func (in CapShellInput) OffsetSection(budget *proofbound.WorkBudget) (momentinput.Profile, error) {
	pp := in.Prism
	if in.Sense > 0 {
		// The section limit: P ⊖ t is non-empty exactly when t is strictly less
		// than the section's inradius. A contained disk can certify success;
		// otherwise internal/survey2d/wall_kernel.go computes the same reading Wall.Minimum answers.
		inradius, enough, err := shellsurvey.SectionInradius(budget, boundarywalk.Profile(pp.Profile), in.HeldThickness, in.ThicknessDelta, offset2d.ShellTol)
		if err != nil {
			return momentinput.Profile{}, err
		}
		if err := shellsurvey.RequireSectionCavity(in.Thickness, in.HeldThickness, inradius, enough, offset2d.ShellTol); err != nil {
			return momentinput.Profile{}, err
		}
		if err := in.requireFloorCavity(); err != nil {
			return momentinput.Profile{}, err
		}
	}
	// A dropped feature or a miter that does not close is caught here —
	// antecedent to the audit (S11a / S11) — before there is any constructed
	// section to audit.
	offset, err := offset2d.OffsetProfile(budget, pp.Profile, in.Sense, in.HeldThickness, offset2d.ShellTol)
	if err != nil {
		return momentinput.Profile{}, err
	}
	// The shared §5 audit, run on decad's own synthesized geometry. A shell
	// mints no cutback, so S6 cannot fire.
	if err := offset2d.AuditOffsetSectionBudget(budget, pp.Profile, offset); err != nil {
		return momentinput.Profile{}, CancelCause(err)
	}
	return offset, nil
}

// requireFloorCavity is the height limit of an inward shell that keeps caps,
// read with the section limit's scale-relative rounding tolerance
// (offset2d.ShellTol*max(1, h), growing with the part's scale, not a fixed
// sub-nanometre margin). A cup's floor is t thick, so its cavity is swept
// over h − t (§8); WithNoOpenings keeps two floors, so its cavity's axial
// height is h − 2t (SX11). With both caps removed there is no floor.
func (in CapShellInput) requireFloorCavity() error {
	h := in.Prism.Z1 - in.Prism.Z0
	tmm := in.HeldThickness
	switch in.KeptCaps {
	case 1:
		maxT := h - offset2d.ShellTol*math.Max(1, h)
		if tmm < maxT {
			return nil
		}
		if maxT <= 0 {
			// The sweep height itself is at or below the rounding tolerance,
			// so no positive thickness leaves a floor cavity. A smaller
			// thickness cannot help; the sweep height must be increased.
			return fmt.Errorf(`%w: the sweep height %s is at or below the evaluator's rounding tolerance, so this evaluator accepts no positive shell thickness here (the tolerance-adjusted maximum is non-positive); increase the sweep height`, decaderr.ErrDegenerate, units.Millimeters(h))
		}
		return fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the sweep height %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, decaderr.ErrDegenerate, in.Thickness, units.Millimeters(maxT), units.Millimeters(h))
	case 2:
		if maxT := h/2 - offset2d.ShellTol*math.Max(1, h); tmm >= maxT {
			return fmt.Errorf(`%w: the shell thickness %s leaves no cavity between the two kept caps of a %s sweep (the accepted maximum is %s, half the sweep less the evaluator's rounding tolerance; modify-reach SX11)`,
				decaderr.ErrDegenerate, in.Thickness, units.Millimeters(h), units.Millimeters(math.Max(maxT, 0)))
		}
	}
	return nil
}

// SectionDelta is how far the offset section may sit from the offset the
// thickness denotes (§9): the offset is a float evaluation of P ⊖ t*.
func (in CapShellInput) SectionDelta(budget *proofbound.WorkBudget) (float64, error) {
	delta, err := offset2d.OffsetSectionDelta(budget, in.Prism.Profile, in.Sense, in.HeldThickness, in.ThicknessDelta, offset2d.ShellTol)
	if err != nil {
		return 0, CancelCause(err)
	}
	return delta, nil
}

// wallRegions returns the outer region O and the cavity C of a shell over its
// offset section: P and Q inward, Q and P outward.
func (in CapShellInput) wallRegions(offset momentinput.Profile) (momentinput.Profile, momentinput.Profile) {
	if in.Sense < 0 {
		return offset, in.Prism.Profile
	}
	return in.Prism.Profile, offset
}

// BuildTube builds the both-caps hole-free shell (Table B, B2/B3): a plain
// prism over the annular section — {Outer: P, Hole: reverse(Q)} inward,
// {Outer: Q, Hole: reverse(P)} outward — so it IS a prism payload, admitted as
// a receiver (R1) and first-class downstream (§12). The inner loop is walked
// as a hole (reversed sense), which is what makes its wall's material lie
// outside it.
func BuildTube(ctx context.Context, env topo.Env, in CapShellInput, offset momentinput.Profile) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	outer, cavity := in.wallRegions(offset)
	holeLoop, err := offset2d.ReverseLoopRecordContext(ctx, cavity.Outer)
	if err != nil {
		return nil, err
	}
	pp := in.Prism
	// The annular section is a NEW record no preflight has seen, so the build
	// opens its one counter here (agent-docs/spline-design.md §5.2). A tube keeps
	// the receiver's sweep, so each end keeps its own axial displacement too.
	return prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile: momentinput.Profile{Outer: outer.Outer, Holes: []sectionrecord.LoopRecord{holeLoop}},
		Frame:   pp.Frame,
		Z0:      pp.Z0,
		Z1:      pp.Z1,
		Z0Delta: pp.Z0Delta,
		Z1Delta: pp.Z1Delta,
		Xform:   pp.Xform,
	}, freeform.NewFreeformWork())
}

// BuildBands builds reach BX8: the both-caps shell of a section with k >= 1
// holes is 1 + k disjoint bands over the receiver's sweep, one stacked slab
// holding one region per band (a prism group, agent-docs/stacked-prism-design.md
// §2.2). Each band is its own lump with its own rim caps, capStart and
// capEnd, and its walls carry slab(0).region(m).side(i,j) with the outer band
// first and the hole linings in ProfileRecord order. The offset loops sit
// within offsetDelta of the offset the thickness denotes, so the payload
// carries it as its section displacement.
func BuildBands(ctx context.Context, env topo.Env, in CapShellInput, offset momentinput.Profile, offsetDelta float64) (*topo.Body, error) {
	outer, cavity := in.wallRegions(offset)
	bands, err := ShellWallBands(ctx, outer, cavity)
	if err != nil {
		return nil, err
	}
	pp := in.Prism
	return stackedrecord.Build(ctx, env, stackedrecord.Payload{
		Slabs: []stackedrecord.Slab{{Regions: bands, Z0: pp.Z0, Z1: pp.Z1, Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta}},
		Frame: pp.Frame, Xform: pp.Xform, SectionDelta: offsetDelta,
	}, stackedrecord.OwnPlan{})
}

// BuildClosed is reach BX5 (agent-docs/modify-reach-design.md §9.2): WithNoOpenings
// on a straight prism keeps every face and lines it with a wall of thickness
// t, closing a cavity. The record is three stacked slabs: inward, the section
// P on [z0, z0 + t], the wall bands between P and its erosion Q on
// [z0 + t, z1 − t], and P on [z1 − t, z1]; outward, the dilation Q on
// [z0 − t, z0], the band between Q and P on [z0, z1], and Q on [z1, z1 + t].
// The two interfaces expose the cavity region (Q inward, P outward) as the
// cavity's floor and ceiling, and the cavity walls with them form one void
// shell inside the outer one. A section with k holes builds inward only: its
// wall slab holds 1 + k bands (ShellWallBands), each hole lined by a band
// whose outer loop is Q's hole, and the cavity is a ring around every lining
// (agent-docs/stacked-prism-design.md §2.4). The record carries
// stackedrecord.Payload.ClosedShell, and the built body must read as one lump
// of one outer shell around one void shell.
//
// The gates are the cup's with both caps kept: S18 and S10's section limit
// (inward), then SX11, the axial cavity h − 2t, which must stay positive
// (inward; ErrDegenerate), then S11a as the offset is built and the §5 audit.
// An outward shell of a holed section is SX8. The offset loops carry their
// proven displacement as the stack's section displacement. in.KeptCaps must
// be 2.
func BuildClosed(ctx context.Context, env topo.Env, in CapShellInput) (*topo.Body, error) {
	pp := in.Prism
	if len(pp.Profile.Holes) > 0 && in.Sense < 0 {
		return nil, fmt.Errorf(`%w: an outward closed shell of a holed prism section is outside the shell extension; only an inward one builds (modify-reach SX8)`, decaderr.ErrUnsupported)
	}
	if err := pp.RequireExactSection("shells"); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	offset, err := in.OffsetSection(budget)
	if err != nil {
		return nil, err
	}
	offsetDelta, err := in.SectionDelta(budget)
	if err != nil {
		return nil, err
	}
	outer, cavity := in.wallRegions(offset)
	bands, err := ShellWallBands(ctx, outer, cavity)
	if err != nil {
		return nil, err
	}
	// Each derived level carries its source end's displacement, the thickness
	// conversion and this float sum's rounding, as a cup's floor level does.
	step := func(from, delta, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(delta, in.ThicknessDelta, proofarith.AddRoundError(from, by, to))
	}
	s, tmm := in.Sense, in.HeldThickness
	lo, loDelta := step(pp.Z0, pp.Z0Delta, s*tmm)
	hi, hiDelta := step(pp.Z1, pp.Z1Delta, -s*tmm)
	caps := stackedrecord.Slab{Regions: []momentinput.Profile{outer}}
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
	exposed := []momentinput.Profile{cavity}
	body, err := stackedrecord.Build(ctx, env, stackedrecord.Payload{
		Slabs:        []stackedrecord.Slab{first, middle, last},
		Interfaces:   []stackedrecord.Interface{{LowerExposed: exposed}, {UpperExposed: exposed}},
		Frame:        pp.Frame,
		Xform:        pp.Xform,
		SectionDelta: offsetDelta,
		ClosedShell:  true,
	}, stackedrecord.OwnPlan{})
	if err != nil {
		return nil, err
	}
	if err := requireOneCavity(body.Lumps); err != nil {
		return nil, err
	}
	return body, nil
}

// requireOneCavity is the closed shell's lump reading
// (agent-docs/stacked-prism-design.md §3): one lump of one closed outer shell
// around exactly one closed void shell. Any other reading disproves the
// record's ClosedShell mark, and the shell refuses it.
func requireOneCavity(lumps []*topo.Lump) error {
	if len(lumps) != 1 || len(lumps[0].Shells) != 2 {
		shells := 0
		for _, l := range lumps {
			shells += len(l.Shells)
		}
		return fmt.Errorf(`%w: the closed shell's faces read as %d lump(s) of %d shell(s), not one outer shell around one cavity (modify-reach BX5)`,
			decaderr.ErrUnsupported, len(lumps), shells)
	}
	outer, cavity := lumps[0].Shells[0], lumps[0].Shells[1]
	if outer.Void || outer.Open || !cavity.Void || cavity.Open {
		return fmt.Errorf(`%w: the closed shell's faces do not close into one outer shell around one cavity (modify-reach BX5)`, decaderr.ErrUnsupported)
	}
	return nil
}
