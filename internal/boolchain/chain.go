// Package boolchain flattens a refused two-body Boolean over mesh Boolean results into
// a normal form of source-body literals, and evaluates a regrouped plan over it
// (agent-docs/boolean-chain-regroup-design.md). It reads no document: a Source supplies
// the recorded producer of a body, and Run takes the Boolean steps as callbacks.
package boolchain

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MaxLiterals bounds the literals of a chain; a longer chain is not flattened.
const MaxLiterals = 6

// Literal is one source body of a chain, complemented when Negated.
type Literal struct {
	Body    *topo.Body
	Negated bool
}

// Chain is a conjunction of literals, or a disjunction of positive literals.
// For a conjunction Literals[0] is positive.
type Chain struct {
	Disjunction bool
	Literals    []Literal
}

// Source reports the public Boolean producer of a body.
type Source interface {
	// BooleanProducer returns the kind of the Cut, Union or Intersect that built n and the
	// consumed inputs in order (a Cut's target then its tool). ok is false when n has no
	// such producer.
	BooleanProducer(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool)
}

// Flatten returns the normal form of op over a and b, expanding each operand that
// is an expandable mesh Boolean result. ok is false when nothing expands, when op is
// not Cut, Union or Intersect, or when the chain has more than MaxLiterals literals.
// a, b and src must not be nil.
func Flatten(op meshbool.OperationKind, a, b *topo.Body, src Source) (Chain, bool) {
	w := walker{src: src}
	var start []Literal
	switch op {
	case meshbool.OpCut:
		start = []Literal{{Body: a}, {Body: b, Negated: true}}
	case meshbool.OpIntersect:
		start = []Literal{{Body: a}, {Body: b}}
	case meshbool.OpUnion:
		w.disjunction = true
		start = []Literal{{Body: a}, {Body: b}}
	default:
		return Chain{}, false
	}
	for _, l := range start {
		if !w.walk(l) {
			return Chain{}, false
		}
	}
	if !w.expanded {
		return Chain{}, false
	}
	return Chain{Disjunction: w.disjunction, Literals: w.out}, true
}

type walker struct {
	src         Source
	disjunction bool
	expanded    bool
	out         []Literal
}

// walk appends l's expansion to the output, depth first. It returns false once the
// literal limit is passed.
func (w *walker) walk(l Literal) bool {
	children := w.replacement(l)
	if children == nil {
		w.out = append(w.out, l)
		return len(w.out) <= MaxLiterals
	}
	w.expanded = true
	for _, child := range children {
		if !w.walk(child) {
			return false
		}
	}
	return true
}

// replacement returns the expansion of l, or nil when l stays a leaf.
func (w *walker) replacement(l Literal) []Literal {
	kind, in, ok := Expandable(w.src, l.Body)
	if !ok {
		return nil
	}
	switch {
	case w.disjunction:
		if kind != meshbool.OpUnion {
			return nil
		}
		return literals(in, false)
	case !l.Negated && kind == meshbool.OpCut:
		if len(in) != 2 {
			return nil
		}
		return []Literal{{Body: in[0]}, {Body: in[1], Negated: true}}
	case !l.Negated && kind == meshbool.OpIntersect:
		return literals(in, false)
	case l.Negated && kind == meshbool.OpUnion:
		return literals(in, true)
	}
	return nil
}

func literals(bodies []*topo.Body, negated bool) []Literal {
	if len(bodies) < 2 {
		return nil
	}
	out := make([]Literal, len(bodies))
	for i, b := range bodies {
		out[i] = Literal{Body: b, Negated: negated}
	}
	return out
}

// Expandable returns n's recorded Boolean producer and consumed inputs when n
// denotes that operation over those inputs exactly (Eligibility of an
// expandable body): n is a mesh Boolean result with no point-cone record whose
// every operation warning is classified as moving no geometry. ok is false
// otherwise. Point location reads the same eligibility
// (agent-docs/point-containment-design.md §8), and so does Body.Section of a
// mesh Boolean result (agent-docs/section-design.md).
func Expandable(src Source, n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	p, ok := n.Payload.(facetedrecord.Payload)
	if !ok || p.PointCone != nil {
		return 0, nil, false
	}
	for _, r := range n.OperationWarnings {
		if !movesNoGeometry(r) {
			return 0, nil, false
		}
	}
	return src.BooleanProducer(n)
}

// movesNoGeometry reports whether r is a warning classified as leaving the
// accepted geometry unchanged. It fails closed: only a chain allowance with no
// change mechanism qualifies, so a record with any other code, including one
// added later, makes the body a leaf.
func movesNoGeometry(r opdiagnostic.Record) bool {
	return r.Code == opdiagnostic.CodeChainAllowanceApplied && r.Change == opdiagnostic.ChangeUnknown
}
