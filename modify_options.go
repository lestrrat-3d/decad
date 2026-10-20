package decad

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// Reach option constructors copy query input, then the internal option codec
// validates and folds the options. The reference resolver uses root topology.

// FilletChamferOption is accepted by both Fillet and Chamfer.
type FilletChamferOption interface {
	FilletOption
	ChamferOption
}

type filletChamferOptionValue struct {
	encoded modifyoption.FilletChamferOption
}
type chamferOptionValue struct{ encoded modifyoption.ChamferOption }
type shellOptionValue struct{ encoded modifyoption.ShellOption }

func (filletChamferOptionValue) filletOption()  {}
func (filletChamferOptionValue) chamferOption() {}
func (chamferOptionValue) chamferOption()       {}
func (shellOptionValue) shellOption()           {}

func decodeFilletOptions(opts []FilletOption) (modifyoption.FilletConfig, error) {
	encoded := make([]modifyoption.FilletOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		value, ok := opt.(filletChamferOptionValue)
		if !ok {
			return modifyoption.FilletConfig{}, fmt.Errorf(`%w: the fillet option is not a decad fillet option (%T)`, ErrDegenerate, opt)
		}
		encoded[i] = value.encoded
	}
	return modifyoption.DecodeFillet(encoded)
}

func decodeChamferOptions(opts []ChamferOption) (modifyoption.ChamferConfig[*FaceQuery], error) {
	encoded := make([]modifyoption.ChamferOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		switch value := opt.(type) {
		case filletChamferOptionValue:
			encoded[i] = value.encoded
		case chamferOptionValue:
			encoded[i] = value.encoded
		default:
			return modifyoption.ChamferConfig[*FaceQuery]{}, fmt.Errorf(`%w: the chamfer option is not a decad chamfer option (%T)`, ErrDegenerate, opt)
		}
	}
	return modifyoption.DecodeChamfer[*FaceQuery](encoded)
}

func decodeShellOptions(opts []ShellOption) (modifyoption.ShellConfig, error) {
	encoded := make([]modifyoption.ShellOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		value, ok := opt.(shellOptionValue)
		if !ok {
			return modifyoption.ShellConfig{}, fmt.Errorf(`%w: the shell option is not a decad shell option (%T)`, ErrDegenerate, opt)
		}
		encoded[i] = value.encoded
	}
	return modifyoption.DecodeShell(encoded)
}

type asymmetricChamferOpts = modifyoption.Asymmetric[*FaceQuery]

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
	return filletChamferOptionValue{encoded: modifyoption.WithTangentChain()}
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
		a.Foreign = fmt.Sprintf(`%T`, reference)
	}
	return chamferOptionValue{encoded: modifyoption.WithAsymmetricChamfer(a)}
}

// WithNoOpenings asks Shell to keep every face and build a closed hollow body
// (docs/modify-reach-design.md §9.2). It is the only call form in which Shell
// accepts a nil face selector, and a non-nil selector beside it is
// ErrDegenerate. No receiver builds a closed shell yet: Shell returns
// ErrUnsupported for every receiver it accepts the option on.
func WithNoOpenings() ShellOption {
	return shellOptionValue{encoded: modifyoption.WithNoOpenings()}
}

// resolveAsymmetricReference is stage 3 of the reach gate order: it resolves
// the asymmetric chamfer's reference against the receiver and pairs every
// expanded edge with its one reference face (docs/modify-reach-design.md §6,
// SX3). A nil reference is errNilSelector and a foreign one ErrDegenerate,
// the selector errors the seed query takes. An edge with no adjacent face in
// the resolved set, or with both, and a resolved face adjacent to no
// expanded edge, are ErrCardinality.
func resolveAsymmetricReference(b *Body, a *asymmetricChamferOpts, edges []*Edge) (map[*topo.Edge]*topo.Face, error) {
	if a.Foreign != "" {
		return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference is not a decad face query (%s)`, ErrDegenerate, a.Foreign)
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
	refs := make(map[*topo.Edge]*topo.Face, len(edges))
	for ei, e := range edges {
		var ref *Face
		count := 0
		for _, f := range e.n.Faces {
			if _, ok := resolved[faceHandle(f)]; ok {
				ref = faceHandle(f)
				count++
			}
		}
		if count != 1 {
			return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference %s names %d of the faces adjacent to %s; it must name exactly one (modify-reach SX3)`,
				ErrCardinality, a.Reference, count, modifyselect.EdgeContext(ei, edgeNode(e)))
		}
		refs[edgeNode(e)] = faceNode(ref)
		touched[ref] = struct{}{}
	}
	if len(touched) != len(resolved) {
		return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference %s matched %d faces and only %d of them is adjacent to a chamfered edge (modify-reach SX3)`,
			ErrCardinality, a.Reference, len(resolved), len(touched))
	}
	return refs, nil
}
