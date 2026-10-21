package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/r3"
)

// This file is PatchEdges of agent-docs/surface-design.md §5.2: a planar fill of
// one or more closed chains of a body's own free edges, amended by this PR
// from a one-chain rule to "one or more chains, each proven independently"
// (§5.2's own amendment note). It reuses internal/stitchrecord/unstitch.go's held-B-rep-under-a-
// rigid-motion mechanism — record the receiver's own faces and a rigid
// placement, never a built topology, and replay under a composed motion —
// narrowed here to a whole face SET rather than one face, and extended with
// the new faces a chain fill mints. It also reuses internal/stitchrecord/build.go's per-old-edge
// sharing cache (rebuildTopology's edgeFor) for copying a face SET
// rather than welding one, internal/stitchrecord/plan.go's NewVertexTable is NOT
// needed here — a patch never merges two DIFFERENT edges into one, it only
// gives an already-free edge a SECOND face, so the same-pointer cache alone
// (patchchain.copyFacesUnder's edgeFor) is what stitchrecord's weld groups exist to
// generalize beyond.
//
// Four gates decide the selection, in order, each reject-only
// (agent-docs/surface-design.md §5.2, Table R):
//
//  1. every selected edge is free (buildPatchChains);
//  2. the selection partitions into one or more closed chains, decided by
//     vertex degree over every selected use (partitionPatchChains) — a
//     single CLOSED edge (Circle3, start == end) is a complete chain on its
//     own, and a two-edge chain over one vertex pair gives each vertex one
//     OTHER edge, which the degree-over-all-uses rule reads correctly where
//     §5.2's original "shared with exactly two other edges" wording did not;
//  3. each chain is planar, proven exactly over dyadic.go's rational lift
//     (provePatchChainPlane) — never a residual against a fitted plane;
//  4. each chain is simple in its plane, via fillet_audit.go's existing
//     crossing audit (patchchain.buildFace) — that audit's own ErrUnsupported is
//     remapped to ErrDegenerate at this boundary (patchchain.remapCrossingError),
//     because a self-crossing chain is bad input rather than evaluator
//     reach, on agent-docs/surface-design.md §5.2's own instruction.
//
// Orientation — which sense each new face takes on its chain's edges — is a
// fifth, purely combinatorial decision with no geometry in it (patchchain.orientChain):
// each edge is traversed by the one face already adjacent to it in some
// sense, and the patch takes the opposite sense. Where a chain's own
// adjacent faces do not agree among themselves — unreachable through this
// evaluator's own builders, which each leave one shell consistently
// oriented (agent-docs/surface-design.md §2.3), but not excluded by the public
// seam — that disagreement is ErrDegenerate, Table R row R18 (new: §5.2
// named the case without giving it a row).
//
// patchchain.BuildBody is the whole evaluator, shared by the initial build
// and every later Transform/Copy re-evaluation: gate 3's exact
// plane proof runs ONCE, in buildPatchChains, and is never re-proven (a
// rigid motion preserves planarity exactly, so re-proving would only make
// Transform refuse most rotations for no soundness gain). Its own result is
// discarded once the proof passes — the payload carries only the chain's own
// edges, never a plane value — because the new face's actual plane is
// derived fresh, every evaluation, from that chain's own (placed) oriented
// geometry (patchchain.OrientedNormal). Gate 4's crossing audit and the
// orientation derivation are NOT replayed from a stored answer either: they
// run again on every placement, exactly as Stitch re-runs its own crossing
// audit after a placement, because rounding can bring two placed points into
// contact — or apart — that the unplaced ones were not.

// PatchEdges's own Table R row R19, beside the existing R4 (gate 1), R5 (gate 2,
// and gate 4 remapped), R6 (gate 3), R16 (selector) and R18 (patchchain's
// RowFacesDisagree) rows §5.2 already names.
const (
	// patchRowNoPayload is Table R row R19: the receiver carries no
	// evaluator payload for this evaluator to rebuild from.
	patchRowNoPayload = `agent-docs/surface-design.md Table R row R19`
)

// PatchEdges resolves sel against b's own live topology, fills every closed
// chain of free edges the selection proves, and returns a new body carrying
// b's own faces plus one new planar face per chain, retiring b
// (agent-docs/surface-design.md §5.2). b's own topology is never mutated: the
// result is built by REBUILDING b from its own faces, so a retired b stays
// readable and unmodified for any caller still holding it.
//
// A nil context or a nil b is [ErrDegenerate]. A b this evaluator did not
// build (b.n.Payload == nil) is [ErrUnsupported] (Table R row R19): a body reads
// its readable topology regardless of payload, but PatchEdges's own
// re-evaluation contract — a placed copy replays from the recorded plane
// rather than re-proving it — needs the same payload-carrying contract every
// other rebuilding operation
// (Transform, Copy) already requires. A body owned by a
// different document is [ErrForeignBody], and a retired b is
// [ErrRetiredBody]. sel's own resolution failure ([SelectionError] wrapping
// [ErrNoMatch] or [ErrCardinality], Table R row R16) and every one of gates
// 1-4's refusals (R4, R5, R6, R18) surface unchanged. A failed call leaves
// the document and b unchanged.
func PatchEdges(ctx context.Context, b *Body, sel EdgeSelector) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a patch`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: a nil body names nothing to patch`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.n.Payload == nil {
		return nil, fmt.Errorf(`%w: PatchEdges requires a body this evaluator built (%s)`, ErrUnsupported, patchRowNoPayload)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if sel == nil {
		return nil, errNilSelector
	}
	edges, err := sel.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	chains, err := patchchain.ChainsFromEdges(edgeNodes(edges))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	body, err := bodyResult(patchchain.BuildBody(ctx, envOf(&d.n, ref), faceNodes(b.Faces()), b.n.Bounds, chains, r3.Identity()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}
