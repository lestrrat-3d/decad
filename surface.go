package decad

import (
	"fmt"

	"github.com/lestrrat-go/option/v3"
)

// This file is the shared surface-result vocabulary of docs/surface-design.md
// §3-§4: the one option every wall-building feature accepts,
// WithSurfaceResult, and the two refusal helpers that keep a sheet body out
// of an operation Table X or Table R does not admit it to. Each feature's own
// file wires the option into its build (extrude.go) or refuses it outright
// (revolve.go, sweep.go, loft.go); boolean.go, fillet.go, chamfer.go,
// shell.go and stops.go consume refuseSheetOperand at their own gates.

// SurfaceResultOption configures every feature WithSurfaceResult reaches
// (docs/surface-design.md §3). A feature this evaluator cannot yet build as a
// surface refuses it with [ErrUnsupported] (Table R row R1).
type SurfaceResultOption interface {
	ExtrudeOption
	RevolveOption
	SweepOption
	LoftOption
}

type surfaceResultOption struct{ option.Interface }

func (surfaceResultOption) extrudeOption() {}
func (surfaceResultOption) revolveOption() {}
func (surfaceResultOption) sweepOption()   {}
func (surfaceResultOption) loftOption()    {}

type identSurfaceResult struct{}

// WithSurfaceResult builds the feature's wall set and omits every face that
// exists only to close the solid, publishing a sheet body — Kind() ==
// BodySheet — instead (docs/surface-design.md §4.1). The option carries no
// payload of its own; its identity is the whole of the signal, and a repeated
// WithSurfaceResult() is idempotent, never an error.
func WithSurfaceResult() SurfaceResultOption {
	return surfaceResultOption{option.New(identSurfaceResult{}, struct{}{})}
}

// refuseSurfaceResult reports [ErrUnsupported] when o is a
// WithSurfaceResult() option, for a feature Table R row R1 stages as a
// permanent or not-yet-built refusal. It asserts on the concrete type alone
// and never calls Ident(): the payload is never read, so the type itself is
// the whole check.
func refuseSurfaceResult(o any, feature string) error {
	if _, ok := o.(surfaceResultOption); ok {
		return fmt.Errorf(`%w: %s does not build a surface result (docs/surface-design.md Table R row R1)`, ErrUnsupported, feature)
	}
	return nil
}

// refuseSheetOperand reports [ErrUnsupported] when b is live and a sheet
// (Kind() == BodySheet), for an operation docs/surface-design.md Table X does
// not admit a sheet to. A nil b names nothing to refuse.
func refuseSheetOperand(b *Body, operation string) error {
	if b != nil && b.Kind() == BodySheet {
		return fmt.Errorf(`%w: %s does not accept a sheet body (docs/surface-design.md Table X)`, ErrUnsupported, operation)
	}
	return nil
}
