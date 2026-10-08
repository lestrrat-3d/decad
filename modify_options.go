package decad

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the option vocabulary of docs/modify-reach-design.md §2: the
// three reach options, the private records each modify call decodes its
// options into, the SX1 gate (Table SX) that rejects an option list naming
// no single intent, and the asymmetric chamfer's reference resolution (§6,
// SX3). The decoders run in stage 1 of the reach gate order (§4), and the
// reference resolves in stage 3, after tangent expansion.

// FilletChamferOption is accepted by both Fillet and Chamfer.
type FilletChamferOption interface {
	FilletOption
	ChamferOption
}

type filletChamferOption struct{ option.Interface }

func (filletChamferOption) filletOption()  {}
func (filletChamferOption) chamferOption() {}

type chamferOption struct{ option.Interface }

func (chamferOption) chamferOption() {}

type identTangentChain struct{}
type identAsymmetricChamfer struct{}
type identNoOpenings struct{}

// WithTangentChain expands each selected seed edge across the edges that
// continue it with proven G1 continuity (docs/modify-reach-design.md §5): an
// edge leaving the seed's endpoint along the exactly opposite tangent ray,
// whose two adjacent faces have the same outward normals there as the
// seed's. The expansion repeats from every edge it adds. The seed query's
// cardinality assertion applies to the seeds, before expansion. A
// continuation with more than one candidate, or one this evaluator cannot
// decide, is ErrUnsupported; the call never picks a branch or stops early.
// Repeating the option is the same as passing it once.
func WithTangentChain() FilletChamferOption {
	return filletChamferOption{option.New(identTangentChain{}, struct{}{})}
}

// WithAsymmetricChamfer sets back the chamfer's positional distance across
// reference and otherDistance across the other face adjacent to each selected
// edge (docs/modify-reach-design.md §6). The option copies reference when it
// is called, so a later change to the caller's query does not reach the
// chamfer. reference resolves against the receiver after tangent expansion,
// and every chamfered edge must have exactly one adjacent face in it, and
// every face it resolves to must touch a chamfered edge; anything else is
// ErrCardinality. otherDistance is a length magnitude gated like the
// positional distance. Passing the option twice is ErrDegenerate.
func WithAsymmetricChamfer(reference FaceSelector, otherDistance units.Value) ChamferOption {
	a := asymmetricChamferOpts{Other: otherDistance}
	switch q := reference.(type) {
	case nil:
	case *FaceQuery:
		if q != nil {
			// Copy every union branch and its predicates: a later Or on the
			// caller's query appends to its branch list, and that append
			// must not reach a shared backing array.
			branches := make([][]FacePredicate, len(q.branches))
			for i, b := range q.branches {
				branches[i] = slices.Clone(b)
			}
			a.Reference = &FaceQuery{branches: branches, card: q.card}
		}
	default:
		a.foreign = fmt.Sprintf(`%T`, reference)
	}
	return chamferOption{option.New(identAsymmetricChamfer{}, a)}
}

// WithNoOpenings asks Shell to keep every face and build a closed hollow body
// (docs/modify-reach-design.md §9.2). It is the only call form in which Shell
// accepts a nil face selector, and a non-nil selector beside it is
// ErrDegenerate. No receiver builds a closed shell yet: Shell returns
// ErrUnsupported for every receiver it accepts the option on.
func WithNoOpenings() ShellOption {
	return shellOption{option.New(identNoOpenings{}, struct{}{})}
}

// filletOpts is the record a Fillet call decodes its options into.
type filletOpts struct {
	TangentChain bool
}

// chamferOpts is the record a Chamfer call decodes its options into.
type chamferOpts struct {
	TangentChain bool
	Asymmetric   *asymmetricChamferOpts
}

// asymmetricChamferOpts is WithAsymmetricChamfer's payload: the copied
// reference query, or the type name of a selector decad does not own, and
// the other distance as the caller stated it. otherMM is that distance in
// millimetres and otherDelta the rounding its unit conversion committed,
// both filled by the decoder once the magnitude gates pass.
type asymmetricChamferOpts struct {
	Reference  *FaceQuery
	foreign    string
	Other      units.Value
	otherMM    float64
	otherDelta float64
}

// shellOpts is the record a Shell call decodes its options into
// (docs/modify-reach-design.md §2's ShellOpts): the wall sense, and whether
// the shell keeps every face.
type shellOpts struct {
	Sense      ShellSense
	NoOpenings bool
}

// errOptionConflict is SX1's sentinel wording for an option list naming no
// single intent.
func errOptionConflict(format string, args ...any) error {
	return fmt.Errorf(`%w: `+format+` (modify-reach SX1)`, append([]any{ErrDegenerate}, args...)...)
}

// decodeFilletOptions folds a Fillet call's options into filletOpts. A nil
// option, an implementation decad does not own, or a payload the option
// cannot carry is ErrDegenerate.
func decodeFilletOptions(opts []FilletOption) (filletOpts, error) {
	var out filletOpts
	for _, raw := range opts {
		if raw == nil {
			return filletOpts{}, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		// Embedding FilletOption can promote its sealed marker onto a foreign
		// type, so admit the owned concrete implementation before invoking
		// any option callback.
		o, ok := raw.(filletChamferOption)
		if !ok {
			return filletOpts{}, fmt.Errorf(`%w: the fillet option is not a decad fillet option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identTangentChain:
			out.TangentChain = true
		default:
			return filletOpts{}, errOptionConflict(`unknown fillet option identifier %T`, ident)
		}
	}
	return out, nil
}

// decodeChamferOptions folds a Chamfer call's options into chamferOpts. A
// second WithAsymmetricChamfer names a second reference and distance pair,
// which is SX1. The other distance passes the magnitude gates here: a wrong
// Kind, a non-finite or a negative value is base S15, and zero is
// ErrDegenerate, since §6 requires both distances strictly positive.
func decodeChamferOptions(opts []ChamferOption) (chamferOpts, error) {
	var out chamferOpts
	for _, raw := range opts {
		if raw == nil {
			return chamferOpts{}, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		var o option.Interface
		switch v := raw.(type) {
		case filletChamferOption:
			o = v
		case chamferOption:
			o = v
		default:
			return chamferOpts{}, fmt.Errorf(`%w: the chamfer option is not a decad chamfer option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identTangentChain:
			out.TangentChain = true
		case identAsymmetricChamfer:
			if out.Asymmetric != nil {
				return chamferOpts{}, errOptionConflict(`WithAsymmetricChamfer is given twice; one chamfer takes one reference and one other distance`)
			}
			a, ok := option.Get[asymmetricChamferOpts](o)
			if !ok {
				return chamferOpts{}, errOptionConflict(`WithAsymmetricChamfer carries no reference and distance`)
			}
			mm, mmDelta, err := extent.MagnitudeInBounded(a.Other, units.Length, units.Millimeter, "the asymmetric chamfer's other distance")
			if err != nil {
				return chamferOpts{}, err
			}
			if mm == 0 {
				return chamferOpts{}, fmt.Errorf(`%w: an asymmetric chamfer's other distance must be positive; a zero setback leaves that face where it is`, ErrDegenerate)
			}
			a.otherMM, a.otherDelta = mm, mmDelta
			out.Asymmetric = &a
		default:
			return chamferOpts{}, errOptionConflict(`unknown chamfer option identifier %T`, ident)
		}
	}
	return out, nil
}

// decodeShellOptions folds a Shell call's options into shellOpts, the sense
// defaulting to Inward. Two WithShellSense options naming different senses
// are SX1; repeating one sense, or WithNoOpenings, is the same as passing it
// once.
func decodeShellOptions(opts []ShellOption) (shellOpts, error) {
	out := shellOpts{Sense: Inward}
	sensed := false
	for _, raw := range opts {
		if raw == nil {
			return shellOpts{}, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		// decad owns the option vocabulary. Embedding ShellOption can promote
		// its sealed marker onto a foreign type, so admit the owned concrete
		// implementation before invoking any option callback.
		o, ok := raw.(shellOption)
		if !ok {
			return shellOpts{}, fmt.Errorf(`%w: the shell option is not a decad shell option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identShellSense:
			v, ok := option.Get[ShellSense](o)
			if !ok {
				return shellOpts{}, fmt.Errorf(`%w: WithShellSense carries no sense`, ErrDegenerate)
			}
			if v != Inward && v != Outward {
				return shellOpts{}, fmt.Errorf(`%w: unknown shell sense %d`, ErrDegenerate, int(v))
			}
			if sensed && v != out.Sense {
				return shellOpts{}, errOptionConflict(`WithShellSense names both %s and %s`, out.Sense, v)
			}
			out.Sense, sensed = v, true
		case identNoOpenings:
			out.NoOpenings = true
		default:
			return shellOpts{}, fmt.Errorf(`%w: unknown shell option identifier %T`, ErrDegenerate, ident)
		}
	}
	return out, nil
}

// resolveAsymmetricReference is stage 3 of the reach gate order: it resolves
// the asymmetric chamfer's reference against the receiver and pairs every
// expanded edge with its one reference face (docs/modify-reach-design.md §6,
// SX3). A nil reference is errNilSelector and a foreign one ErrDegenerate,
// the selector errors the seed query takes. An edge with no adjacent face in
// the resolved set, or with both, and a resolved face adjacent to no
// expanded edge, are ErrCardinality.
func resolveAsymmetricReference(b *Body, a *asymmetricChamferOpts, edges []*Edge) (map[*Edge]*Face, error) {
	if a.foreign != "" {
		return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference is not a decad face query (%s)`, ErrDegenerate, a.foreign)
	}
	if a.Reference == nil {
		return nil, fmt.Errorf(`%w (the asymmetric chamfer's reference)`, errNilSelector)
	}
	faces, err := a.Reference.SelectFaces(b)
	if err != nil {
		return nil, err
	}
	resolved := make(map[*Face]struct{}, len(faces))
	for _, f := range faces {
		resolved[f] = struct{}{}
	}
	touched := make(map[*Face]struct{}, len(faces))
	refs := make(map[*Edge]*Face, len(edges))
	for ei, e := range edges {
		var ref *Face
		count := 0
		for _, f := range e.faces {
			if _, ok := resolved[f]; ok {
				ref = f
				count++
			}
		}
		if count != 1 {
			return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference %s names %d of the faces adjacent to %s; it must name exactly one (modify-reach SX3)`,
				ErrCardinality, a.Reference, count, selectedEdgeContext(ei, e))
		}
		refs[e] = ref
		touched[ref] = struct{}{}
	}
	if len(touched) != len(resolved) {
		return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference %s matched %d faces and only %d of them is adjacent to a chamfered edge (modify-reach SX3)`,
			ErrCardinality, a.Reference, len(resolved), len(touched))
	}
	return refs, nil
}

// asymmetricChamfer is a resolved WithAsymmetricChamfer: the receiver whose
// face roles map a reference face to a walk of the recorded section, each
// chamfered edge's reference face, and the two distances in millimetres —
// d, the positional distance, across the reference face, and other across
// the face beside it — each beside the rounding its own unit conversion
// committed.
type asymmetricChamfer struct {
	body               *Body
	refs               map[*Edge]*Face
	d, other           float64
	dDelta, otherDelta float64
}

// setbacks returns the arc lengths a chamfer of corner ci of loop li sets
// back along the arriving and the leaving walk (docs/modify-reach-design.md
// §6): the walk whose wall is e's reference face takes d and the other walk
// takes other. A side wall's side(i,j) roles name the recorded segments it
// is built from, and the reference face's roles decide which walk it is; a
// face whose roles name segments of both walks or of neither is
// ErrUnsupported, since this evaluator cannot tell which setback it takes.
func (a *asymmetricChamfer) setbacks(loop cornerLoop, li, ci int, e *Edge) (float64, float64, error) {
	ref := a.refs[e]
	if ref == nil {
		return 0, 0, fmt.Errorf(`%w: the asymmetric chamfer has no reference face for this edge`, ErrUnsupported)
	}
	segs := faceSideSegments(a.body, ref, li)
	n := len(loop.walks)
	arriving := walkHoldsAny(loop.walks[(ci+n-1)%n].Segs, segs)
	leaving := walkHoldsAny(loop.walks[ci].Segs, segs)
	switch {
	case arriving && !leaving:
		return a.d, a.other, nil
	case leaving && !arriving:
		return a.other, a.d, nil
	default:
		return 0, 0, fmt.Errorf(`%w: the asymmetric chamfer's reference face is not the wall of exactly one of the two walks meeting at loop %d corner %d`, ErrUnsupported, li, ci)
	}
}

// capSetbacks returns the start and the end cap's own two setbacks for a
// two-distance chamfer of complete cap loops (docs/modify-reach-design.md
// §8.3.1). Every edge of a cap loop borders its cap face and one side wall, and
// its reference face picks between them: the cap face gives the cap dc = d and
// ds = other, a side wall gives dc = other and ds = d. The pick is made per
// cap. A cap whose edges pick differently, and a loop chamfered on both caps
// whose caps pick differently, give one loop's patches two in-plane offsets,
// and both are SX4 (ErrUnsupported). SX3 has already refused both shapes — a
// cap face borders every edge of its loops, and a side wall borders that
// loop's edges on both caps, so a mixed pick names some edge's two faces or
// neither — and these arms are a second check over the resolved pairs. An
// edge on neither cap face is ErrUnsupported; classifyChamferSelection has
// already put every selected edge on one. A cap with no selected edge reads
// the side-wall pick, which no band reads.
func (a *asymmetricChamfer) capSetbacks(caps prismCaps, edges []*Edge, startLoops, endLoops map[int]bool) (capSetback, capSetback, error) {
	var seen, capRef [2]bool
	for ei, e := range edges {
		c, capFace := -1, (*Face)(nil)
		for _, f := range e.faces {
			switch {
			case f == caps.start && f != nil:
				c, capFace = 0, f
			case f == caps.end && f != nil:
				c, capFace = 1, f
			}
		}
		if c < 0 {
			return capSetback{}, capSetback{}, fmt.Errorf(`%w: the asymmetric chamfer's %s borders neither cap face`, ErrUnsupported, selectedEdgeContext(ei, e))
		}
		ref := a.refs[e] == capFace
		if seen[c] && capRef[c] != ref {
			return capSetback{}, capSetback{}, fmt.Errorf(`%w: the asymmetric chamfer's reference names the cap face for some edges of one cap and a side wall for others; one cap takes one assignment (modify-reach SX4)`, ErrUnsupported)
		}
		seen[c], capRef[c] = true, ref
	}
	if seen[0] && seen[1] && capRef[0] != capRef[1] {
		for li, on := range startLoops {
			if on && endLoops[li] {
				return capSetback{}, capSetback{}, fmt.Errorf(`%w: the asymmetric chamfer's reference names the cap face on one cap of loop %d and its side walls on the other; one loop takes one assignment (modify-reach SX4)`, ErrUnsupported, li)
			}
		}
	}
	pick := func(capReferenced bool) capSetback {
		if capReferenced {
			return capSetback{dc: a.d, dcDelta: a.dDelta, ds: a.other, dsDelta: a.otherDelta}
		}
		return capSetback{dc: a.other, dcDelta: a.otherDelta, ds: a.d, dsDelta: a.dDelta}
	}
	return pick(capRef[0]), pick(capRef[1]), nil
}

// faceSideSegments returns the recorded segments of loop li that face f is
// built from, read from its side(i,j) roles under b's own producer.
func faceSideSegments(b *Body, f *Face, li int) map[int]struct{} {
	out := map[int]struct{}{}
	for _, o := range f.origins {
		if o.producer != b.origin.producer {
			continue
		}
		var i, j int
		if n, err := fmt.Sscanf(o.Role, "side(%d,%d)", &i, &j); err != nil || n != 2 || i != li {
			continue
		}
		if o.Role != fmt.Sprintf("side(%d,%d)", i, j) {
			continue
		}
		out[j] = struct{}{}
	}
	return out
}

// walkHoldsAny reports whether any recorded segment of a coalesced walk is
// in segs.
func walkHoldsAny(walkSegs []int, segs map[int]struct{}) bool {
	for _, s := range walkSegs {
		if _, ok := segs[s]; ok {
			return true
		}
	}
	return false
}
