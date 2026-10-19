package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
)

// This file is the shared surface-result vocabulary of docs/surface-design.md
// §3-§4: the one option every wall-building feature accepts,
// WithSurfaceResult, and the refusal helper that keeps a sheet body out of an
// operation Table X does not admit it to. Extrude, Revolve, Sweep and Loft
// each wire the option into their own build (extrude.go, internal/revolveplan/build.go,
// sweep.go/internal/sweeprecord/arc.go/internal/compositesweep/assemble.go, loft.go) — Sweep is the
// last of the four to take it, so Table R row R1 now names no feature this
// evaluator still refuses outright; the row stays in the design as the stated
// rule for one added later. boolean.go, fillet.go, chamfer.go, shell.go and
// stops.go consume refuseSheetOperand at their own gates.

// SurfaceResultOption configures every feature WithSurfaceResult reaches
// (docs/surface-design.md §3). A feature this evaluator cannot yet build as a
// surface refuses it with [ErrUnsupported] (Table R row R1).
type SurfaceResultOption interface {
	ExtrudeOption
	RevolveOption
	SweepOption
	LoftOption
}

// WithSurfaceResult builds the feature's wall set and omits every face that
// exists only to close the solid, publishing a sheet body — Kind() ==
// BodySheet — instead (docs/surface-design.md §4.1). The option carries no
// payload of its own; its identity is the whole of the signal, and a repeated
// WithSurfaceResult() is idempotent, never an error.
func WithSurfaceResult() SurfaceResultOption {
	return surfaceResultOptionValue{featureoption.WithSurfaceResult()}
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
