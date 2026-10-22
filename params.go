package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/parambind"
	"github.com/lestrrat-3d/sketch/param"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// Binding is one read of a document's parameter table: the parameter's name
// (empty for an ad-hoc expression), its source text, the value it evaluated
// to and the parameters its expression references directly. Bindings lists
// them in call order (agent-docs/param-binding-design.md).
type Binding = parambind.Binding

// paramsOption is the DocumentOption WithParams returns.
type paramsOption struct{ option.Interface }

func (paramsOption) documentOption() {}

type identParams struct{}

// WithParams makes the document read its parameters from t, such as the table
// a sketch World returns from Params, so a sketch dimension and a feature
// value derive from the same definitions. The document does not copy t: a
// later Set on it changes what the next read returns. A nil t is ignored. A
// document with no option owns a fresh table.
func WithParams(t *param.Table) DocumentOption {
	return paramsOption{option.New(identParams{}, t)}
}

// paramsOf resolves a document's options to the table they name, or nil.
// The last WithParams wins.
func paramsOf(opts []DocumentOption) *param.Table {
	var t *param.Table
	for _, opt := range opts {
		p, ok := opt.(paramsOption)
		if !ok || p.Interface == nil {
			continue
		}
		if got, ok := option.Get[*param.Table](p); ok && got != nil {
			t = got
		}
	}
	return t
}

// Params returns the document's parameter table. Define parameters on it with
// Set, SetExpr or SetValue, then read them with Param and Expr. Changing the
// table and running the builder again is how a model is edited.
func (d *Document) Params() *param.Table {
	if d.params == nil {
		d.params = param.New()
	}
	return d.params
}

// Param evaluates the named parameter and returns it in its declared unit, so
// the result passes anywhere a feature takes a [units.Value], such as an
// extent distance, a fillet radius or a chamfer distance. An undefined name or
// a cycle is the table's own error (param.ErrUndefined, param.ErrCycle). The
// read is recorded in Bindings.
func (d *Document) Param(name string) (units.Value, error) {
	t := d.Params()
	v, err := t.GetValue(name)
	if err != nil {
		return units.Value{}, err
	}
	src, _ := t.Source(name)
	deps, err := t.Dependencies(name)
	if err != nil {
		return units.Value{}, err
	}
	d.ledger.Record(parambind.Binding{Name: name, Expr: src, Value: v, Dependencies: deps})
	return v, nil
}

// Expr evaluates an ad-hoc expression against the parameter table and returns
// it in the base unit of the kind it computes to (millimetre for a length,
// radian for an angle). An expression mixing kinds incompatibly is
// param.ErrIncompatibleKind. The read is recorded in Bindings.
func (d *Document) Expr(expr string) (units.Value, error) {
	v, err := d.Params().EvalValue(expr)
	if err != nil {
		return units.Value{}, err
	}
	d.ledger.Record(parambind.Binding{Expr: expr, Value: v})
	return v, nil
}

// ParamInt evaluates the named dimensionless parameter as a whole number, for
// an argument such as a pattern Count. A value that is not an integer, or a
// parameter that carries a unit, is ErrDegenerate.
func (d *Document) ParamInt(name string) (int, error) {
	v, err := d.Param(name)
	if err != nil {
		return 0, err
	}
	if v.Kind() != units.Dimensionless {
		return 0, fmt.Errorf("%w: parameter %q carries a unit and cannot be a count", ErrDegenerate, name)
	}
	x := v.Base()
	if x != math.Trunc(x) || math.Abs(x) > math.MaxInt32 {
		return 0, fmt.Errorf("%w: parameter %q is %v, not a whole number", ErrDegenerate, name, x)
	}
	return int(x), nil
}

// Bindings returns every Param and Expr read the document has made, in call
// order. The slice is a copy.
func (d *Document) Bindings() []Binding {
	return d.ledger.Bindings()
}
