package decad

import (
	"context"
	"fmt"
)

// This file is the receiver dispatch of docs/brep-modify-design.md
// ("brep-modify §N" below): which record Fillet, Chamfer and Shell hand to the
// brep route (§2, Table RB), the gates every brep or stacked receiver passes
// before any route runs (Table SB's SB1 and SB2), and the order of the two
// routes: route P (brep_modify_prism.go), then route E for a Fillet or
// Chamfer (brep_modify_edge.go).

// brepModifyRequest is what one modify op hands the brep route
// (brep-modify §2): the verb its refusals name ("fillets", "chamfers" or
// "shells"), whether the op is Shell, and admits, the op's own prism
// classification of its selection (§4.2). admits returns nil when the
// selection classifies against the prism pp with caps as its two cap faces,
// and the op's own refusal otherwise. sel, edges and blend are a Fillet's or
// Chamfer's selector, its resolved edges and its per-corner construction
// (computeFillet or computeChamfer, bound to the op's magnitude), which route
// E (§5) reads; a Shell leaves them nil.
type brepModifyRequest struct {
	op     string
	shell  bool
	admits func(pp prismPayload, caps prismCaps) error
	sel    EdgeSelector
	edges  []*Edge
	blend  *revolveBlendOp
}

// brepRoute is what the brep route hands back to the op. It is empty for a
// receiver the brep route does not take, and the op continues on its own
// path. Route P sets prism, the recognised prism (never stored; the op's
// receiver for the rest of the call), and caps, its two cap faces on the
// receiver body: the op continues on its prism path with them. Route E sets
// body, the result it built, which the op commits (commitModifyResult).
type brepRoute struct {
	prism *prismPayload
	caps  prismCaps
	body  *Body
}

// modifyBrepReceiver is the brep route for one modify op (brep-modify §2).
// It returns an empty route when the receiver is neither a brepPayload nor a
// stackedPrismPayload. Otherwise it runs gate stage 2a (§6) in order — the
// stacked receiver's face view (SB2), then the whole-record displacement
// rule (SB1) — and then route P (§4): the first reference axis along which
// the record reads as a prism the op's classification admits. When none
// admits, a Shell refuses with SB3 (some axis reads as a prism; the prism
// path's own S2) or SB10 (none does), and a Fillet or Chamfer takes route E
// (§5), which builds the result or refuses with a Table SB row.
func modifyBrepReceiver(ctx context.Context, b *Body, req brepModifyRequest) (brepRoute, error) {
	bp, ok, err := brepModifyRecord(ctx, b.payload, req.op)
	if err != nil || !ok {
		return brepRoute{}, err
	}
	if err := requireExactBrepSection(bp, req.op); err != nil {
		return brepRoute{}, err
	}
	route, refusal, err := brepPrismRoute(ctx, b, bp, req)
	switch {
	case err != nil:
		return brepRoute{}, err
	case route.prism != nil:
		return route, nil
	case req.shell && refusal != nil:
		return brepRoute{}, fmt.Errorf(`%w; this body reads as a prism along a reference axis, and no such prism takes the removed faces as its caps (brep-modify SB3)`, refusal)
	case req.shell:
		return brepRoute{}, fmt.Errorf(`%w: this evaluator shells a brep or stacked receiver only where it reads as a prism along a reference axis, and this one reads as none; the three-dimensional offset it needs puts a cylinder along every reflex straight edge, which this record does not hold (brep-modify SB10)`, ErrUnsupported)
	case req.blend == nil:
		return brepRoute{}, fmt.Errorf(`%w: this evaluator %s a brep or stacked receiver through route P or route E only (modify-reach SX16)`, ErrUnsupported, req.op)
	default:
		body, err := brepBlendEdges(ctx, b.doc, bp, req)
		if err != nil {
			return brepRoute{}, err
		}
		return brepRoute{body: body}, nil
	}
}

// commitModifyResult commits a modify op's result in place of its receiver,
// after the receiver's liveness and the context are read once more at the
// commit edge (modify §13's atomic commit).
func commitModifyResult(ctx context.Context, b, body *Body) (*Body, error) {
	if err := b.doc.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	b.doc.commit(body, b)
	return body, nil
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
