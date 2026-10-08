package decad

import (
	"context"
	"fmt"
)

// This file is the receiver dispatch of docs/brep-modify-design.md
// ("brep-modify §N" below): which record Fillet, Chamfer and Shell hand to the
// brep route (§2, Table RB), and the gates every brep or stacked receiver
// passes before any route runs (Table SB's SB1 and SB2).

// modifyBrepReceiver is the brep route for one modify op (brep-modify §2).
// op is the verb the refusals name: "fillets", "chamfers" or "shells".
//
// It returns nil when the receiver is neither a brepPayload nor a
// stackedPrismPayload, and the caller continues on its own path. Otherwise
// it runs gate stage 2a (brep-modify §6) in order: the stacked receiver's
// face view (SB2), then the whole-record displacement rule (SB1). A receiver
// that passes both refuses with modify-reach SX16's ErrUnsupported, because
// neither route P (§4) nor route E (§5) is built.
func modifyBrepReceiver(ctx context.Context, payload featurePayload, op string) error {
	bp, ok, err := brepModifyRecord(ctx, payload, op)
	if err != nil || !ok {
		return err
	}
	if err := requireExactBrepSection(bp, op); err != nil {
		return err
	}
	return fmt.Errorf(`%w: this evaluator does not yet rewrite an analytically trimmed (brep) body's faces, so it %s no brep or stacked receiver (modify-reach SX16)`, ErrUnsupported, op)
}

// brepModifyRecord is brep-modify §2's dispatch table: a brepPayload is
// handed over as itself, and a stackedPrismPayload as its face view
// (brepOfStacked). The second result is false for every other payload, which
// the brep route does not take. A stacked receiver with no face view (a
// prism group, or a record its own audit refuses) is SB2: brepOfStacked's
// error, naming the op.
func brepModifyRecord(ctx context.Context, payload featurePayload, op string) (brepPayload, bool, error) {
	switch p := payload.(type) {
	case brepPayload:
		return p, true, nil
	case stackedPrismPayload:
		bp, err := brepOfStacked(ctx, p)
		if err == nil {
			return bp, true, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return brepPayload{}, true, ctxErr
		}
		return brepPayload{}, true, fmt.Errorf(`%w; this evaluator %s a stacked receiver through its face view only, and this one has none (brep-modify SB2)`, err, op)
	default:
		return brepPayload{}, false, nil
	}
}

// requireExactBrepSection is brep-modify SB1 (Table RB's RB3): a brep or
// stacked receiver whose record carries a section displacement on any face is
// ErrUnsupported, naming the displacement. It is requireExactSection's rule
// over the whole record: a rewrite of a record that denotes its body only to
// within a displacement has no proven displacement of its own. Level
// displacements (z0Delta/z1Delta) are not read; no brep-modify construction
// moves a level.
func requireExactBrepSection(bp brepPayload, op string) error {
	delta := bp.sectionDelta()
	if delta == 0 {
		return nil
	}
	return fmt.Errorf(
		`%w: this evaluator %s a brep or stacked receiver whose recorded faces are the faces it denotes only; this body's faces carry a proven section displacement of %g mm from the ones its construction denotes (brep-modify SB1)`,
		ErrUnsupported, op, delta,
	)
}
