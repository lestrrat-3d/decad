package decad

import (
	"fmt"
	"math"

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

// ShellOption configures Shell. ShellOpts is the StepOpts variant a shell
// fills — its Sense (§8) — so the option group is not empty.
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

// MarshalText encodes the sense by name, so a recorded step stays readable and
// a renumbered constant could never silently reinterpret an old recipe. An
// unknown value refuses to encode.
func (s ShellSense) MarshalText() ([]byte, error) {
	switch s {
	case Inward:
		return []byte("inward"), nil
	case Outward:
		return []byte("outward"), nil
	default:
		return nil, fmt.Errorf(`decad: unknown shell sense %d`, int(s))
	}
}

// UnmarshalText decodes the sense by name; an unknown name is an error, never a
// default.
func (s *ShellSense) UnmarshalText(text []byte) error {
	switch string(text) {
	case "inward":
		*s = Inward
	case "outward":
		*s = Outward
	default:
		return fmt.Errorf(`decad: unknown shell sense %q`, string(text))
	}
	return nil
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
func (b *Body) Shell(sel FaceSelector, t units.Value, opts ...ShellOption) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc

	// Stage 1 pre-gates (§4): a live receiver (S17), the options, a valid
	// magnitude (S15), a non-zero one (S14), then a selector that matches (S16).
	if err := d.requireLive(b); err != nil {
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
	tmm, err := magnitudeIn(t, units.Length, units.Millimeter, "the shell thickness")
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

	// Stage 2 (§4): the receiver's payload class (S3), then every removed face
	// is a cap (S2).
	pp, ok := b.payload.(prismPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator shells a straight prism only`, ErrUnsupported)
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

	// Stage 3 (§4): the construction's own gates — S10 (the cavity is non-empty,
	// inward only), then S11a (no feature the offset drops as it is built).
	if s > 0 {
		// The section limit: P ⊖ t is non-empty exactly when t is strictly less
		// than the section's inradius, which survey2d.go computes exactly
		// (docs/modify-design.md §8, the same reading MinWallThickness answers).
		inradius, ok := sectionInradius(pp.profile)
		if !ok {
			return nil, fmt.Errorf(`%w: this evaluator cannot prove the eroded section non-empty`, ErrUnsupported)
		}
		// The accept boundary sits shellTol*max(1, inradius) below the limit —
		// a SCALE-RELATIVE rounding tolerance that grows with the part's scale,
		// not a fixed sub-nanometre margin — so the refusal reports the computed
		// accepted maximum thickness rather than the bare inradius (at a large
		// scale the two differ by far more than a noise floor).
		if maxT := inradius - shellTol*math.Max(1, inradius); tmm >= maxT {
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
	offset, err := offsetProfile(pp.profile, s, tmm)
	if err != nil {
		return nil, err
	}

	// Stage 4 (§4/§5): the audit of the OFFSET section — S8 (orientation), then
	// the crossing test (a crossing or boundary contact of offset loops is S11b,
	// §8), then S9 (nesting). The shared §5 audit, run on decad's own synthesized
	// geometry. A shell mints no cutback, so S6 cannot fire.
	if err := auditOffsetSection(pp.profile, offset); err != nil {
		return nil, err
	}

	// Build. The recipe records the shell intent — OpShell, the deep-copied face
	// query, the thickness and the sense — never the offset section (§11): the
	// reduction is the evaluator's, so a second evaluator replays the same step
	// against the true faces (§2).
	step := Step{
		Op:        OpShell,
		Inputs:    []StepRef{b.originStep()},
		Selectors: cloneSelectors([]Selector{q}),
		Values:    []units.Value{t},
		Opts:      ShellOpts{Sense: sense},
	}
	ref := d.nextStepRef()

	var body *Body
	switch {
	case bothCaps:
		if holed {
			// B4: the wall is one band around the outer loop plus one band lining
			// each hole — 1 + k disjoint lumps, which no prismPayload holds (S12).
			return nil, fmt.Errorf(`%w: a both-caps shell of a holed section is %d disjoint lumps; this evaluator has no multi-lump payload`, ErrUnsupported, 1+len(pp.profile.Holes))
		}
		body, err = evalTube(d, ref, pp, offset, s)
	default:
		// A one-cap shell is a cup (B5/B6), for any k ≥ 0: the offset section's
		// loops are proven simple and correctly nested by the §5 audit above, and
		// evalCup wraps a wall around each post, all hanging off the one floor slab
		// (one lump). The holed BOTH-caps case keeps no floor and is 1 + k lumps
		// (B4, S12), refused above.
		body, err = evalCup(d, ref, cupPayloadFor(pp, offset, s, tmm, removedEnd))
	}
	if err != nil {
		return nil, err
	}
	d.commit(step, body, b)
	return body, nil
}

// shellTol is the closed-form degeneracy tolerance for the shell's own gates:
// a limit reached within it is treated as reached. The section is decad's own
// exact geometry, so this only absorbs float noise, never an admission.
const shellTol = 1e-9

// classifyRemovedCaps decides which caps a removed-face set names: every face
// must be a cap of the receiver (else S2, a side wall — ErrUnsupported), and it
// reports whether the start cap, the end cap, or both were removed.
func classifyRemovedCaps(b *Body, removed []*Face) (start, end bool, err error) {
	caps := map[*Face]string{}
	for _, f := range b.Faces() {
		for _, o := range f.origins {
			if o.Step == b.origin.Step && (o.Role == roleCapStart || o.Role == roleCapEnd) {
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

// sectionInradius is the largest inscribed disk of a recorded section — the
// exact 2D inradius survey2d.go computes as part of the wall survey
// (docs/modify-design.md §8, the reading that answers MinWallThickness). ok is
// false when the kernel cannot decide, which a build-time gate treats as
// ErrUnsupported (it has no Suspect to fall back on).
func sectionInradius(profile ProfileRecord) (float64, bool) {
	loops, err := recordLoops(profile)
	if err != nil {
		return 0, false
	}
	var elems []surveyElem
	var verts [][2]float64
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].closed
		for _, w := range loop {
			el, ok := walkElem(w.segmentWalk)
			if !ok {
				return 0, false
			}
			elems = append(elems, el)
			if single {
				continue
			}
			verts = append(verts, [2]float64{w.startU, w.startV})
		}
	}
	// fitMax is +Inf: the inradius is a property of the section alone, with no
	// height constraint (that constraint only bears on spanning, not the
	// largest inscribed disk).
	k := newWallKernel(elems, nil, verts, 0, 0, false, math.Inf(1))
	out := k.run()
	if !out.ok {
		return 0, false
	}
	return out.inradius, true
}

// evalTube builds the both-caps hole-free shell (Table B, B2/B3): a plain prism
// over the annular section — {Outer: P, Hole: reverse(Q)} inward, {Outer: Q,
// Hole: reverse(P)} outward — so it IS a prismPayload, admitted as a receiver
// (R1) and first-class downstream (§12). The inner loop is walked as a hole
// (reversed sense), which is what makes its wall's material lie outside it.
func evalTube(d *Document, ref StepRef, pp prismPayload, offset ProfileRecord, s float64) (*Body, error) {
	var outer, inner LoopRecord
	if s > 0 { // inward: P is the outside, Q the cavity
		outer = pp.profile.Outer
		inner = offset.Outer
	} else { // outward: Q is the outside, P the cavity
		outer = offset.Outer
		inner = pp.profile.Outer
	}
	holeLoop, err := reverseLoopRecord(inner)
	if err != nil {
		return nil, err
	}
	section := ProfileRecord{Outer: outer, Holes: []LoopRecord{holeLoop}}
	return evalPrism(d, ref, prismPayload{
		profile: section,
		frame:   pp.frame,
		z0:      pp.z0,
		z1:      pp.z1,
		xform:   pp.xform,
	})
}
