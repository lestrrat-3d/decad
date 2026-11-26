package decad

import "fmt"

// This file is the shared surface-result vocabulary of agent-docs/surface-design.md
// §3-§4: the SurfaceResult method every wall-building feature command
// declares (feature_options.go), and the refusal helper that keeps a sheet body out of an
// operation Table X does not admit it to. Extrude, Revolve, Sweep and Loft
// each wire the option into their own build (extrude.go, internal/revolveplan/build.go,
// sweep.go/internal/sweeprecord/arc.go/internal/compositesweep/assemble.go, loft.go) — Sweep is the
// last of the four to take it, so Table R row R1 now names no feature this
// evaluator still refuses outright; the row stays in the design as the stated
// rule for one added later. boolean.go, fillet.go, chamfer.go, shell.go and
// stops.go consume refuseSheetOperand at their own gates.

// refuseSheetOperand reports [ErrUnsupported] when b is live and a sheet
// (Kind() == BodySheet), for an operation agent-docs/surface-design.md Table X does
// not admit a sheet to. A nil b names nothing to refuse.
func refuseSheetOperand(b *Body, operation string) error {
	if b != nil && b.Kind() == BodySheet {
		return fmt.Errorf(`%w: %s does not accept a sheet body (agent-docs/surface-design.md Table X)`, ErrUnsupported, operation)
	}
	return nil
}
