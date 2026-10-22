// Package parambind records the parameter reads a document's callers make, so
// a translator can check its reading of the model source against a real run
// (agent-docs/param-binding-design.md). It evaluates nothing itself.
package parambind

import (
	"slices"

	"github.com/lestrrat-3d/units"
)

// Binding is one read of the parameter table.
type Binding struct {
	// Name is the parameter read by name, empty for an ad-hoc expression.
	Name string
	// Expr is the parameter's source text, or the ad-hoc expression read.
	Expr string
	// Value is what the read evaluated to.
	Value units.Value
	// Dependencies are the parameters Name's expression references directly,
	// sorted. It is nil for an ad-hoc expression.
	Dependencies []string
}

// Ledger is the ordered list of reads. The zero value is empty and ready.
// It is not safe for concurrent use.
type Ledger struct {
	bindings []Binding
}

// Record appends b.
func (l *Ledger) Record(b Binding) {
	b.Dependencies = slices.Clone(b.Dependencies)
	l.bindings = append(l.bindings, b)
}

// Bindings returns a copy of the reads in call order.
func (l *Ledger) Bindings() []Binding {
	out := make([]Binding, len(l.bindings))
	for i, b := range l.bindings {
		b.Dependencies = slices.Clone(b.Dependencies)
		out[i] = b
	}
	return out
}
