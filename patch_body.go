package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// This file is PatchEdges of docs/surface-design.md §5.2: a planar fill of
// one or more closed chains of a body's own free edges, amended by this PR
// from a one-chain rule to "one or more chains, each proven independently"
// (§5.2's own amendment note). It reuses unstitch.go's held-B-rep-under-a-
// rigid-motion mechanism — record the receiver's own faces and a rigid
// placement, never a built topology, and replay under a composed motion —
// narrowed here to a whole face SET rather than one face, and extended with
// the new faces a chain fill mints. It also reuses stitch.go's per-old-edge
// sharing cache (rebuildStitchTopology's edgeFor) for copying a face SET
// rather than welding one, stitch_weld.go's newStitchVertexTable is NOT
// needed here — a patch never merges two DIFFERENT edges into one, it only
// gives an already-free edge a SECOND face, so the same-pointer cache alone
// (patchchain.copyFacesUnder's edgeFor) is what stitch.go's weld groups exist to
// generalize beyond.
//
// Four gates decide the selection, in order, each reject-only
// (docs/surface-design.md §5.2, Table R):
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
//     reach, on docs/surface-design.md §5.2's own instruction.
//
// Orientation — which sense each new face takes on its chain's edges — is a
// fifth, purely combinatorial decision with no geometry in it (patchchain.orientChain):
// each edge is traversed by the one face already adjacent to it in some
// sense, and the patch takes the opposite sense. Where a chain's own
// adjacent faces do not agree among themselves — unreachable through this
// evaluator's own builders, which each leave one shell consistently
// oriented (docs/surface-design.md §2.3), but not excluded by the public
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
	patchRowNoPayload = `docs/surface-design.md Table R row R19`
)

// PatchEdges resolves sel against b's own live topology, fills every closed
// chain of free edges the selection proves, and returns a new body carrying
// b's own faces plus one new planar face per chain, retiring b
// (docs/surface-design.md §5.2). b's own topology is never mutated: the
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
	chains, err := buildPatchChains(edges)
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

// buildPatchChains runs gates 1-3 against the selector's resolved edges: gate
// 1 (every edge free), gate 2 (partition into closed chains), and gate 3
// (each chain's exact plane proof). Gate 4 (simplicity) and the orientation
// derivation are NOT run here — they run inside patchchain.BuildBody, every
// time, including the very first build, since building the actual face
// needs the SAME per-placement machinery a later Transform call replays
// (docs/surface-design.md's "re-run the combinatorial and in-plane gates on
// re-evaluation, but not the plane proof" decision).
func buildPatchChains(edges []*Edge) ([]bodyPatchChain, error) {
	for _, e := range edges {
		if !e.IsFree() {
			return nil, fmt.Errorf(`%w: PatchEdges selection holds an edge that is not free (docs/surface-design.md Table R row R4)`, ErrDegenerate)
		}
	}
	groups, err := partitionPatchChains(edges)
	if err != nil {
		return nil, err
	}
	chains := make([]bodyPatchChain, len(groups))
	for i, g := range groups {
		lvl, axialBound, hasAxialBound, err := provePatchChainPlane(g)
		if err != nil {
			return nil, err
		}
		chains[i] = bodyPatchChain{Edges: edgeNodes(g), AxialBound: axialBound, HasAxialBound: hasAxialBound, Level: lvl}
	}
	return chains, nil
}

// partitionPatchChains is gate 2: the selection partitions into one or more
// closed chains when, and only when, every vertex reached by the selection
// has degree exactly 2 counted over EVERY selected use — a full-circle edge
// (Circle3, start == end) contributes 2 uses to its own single vertex and is
// a complete chain on its own; a two-edge chain of two arcs over one vertex
// pair gives each vertex one OTHER edge, which is degree 2 read this way,
// not the "two other edges" §5.2's original wording asked for. A 2-regular
// (multi)graph is a disjoint union of cycles, so the degree check alone is
// what proves the partition exists; the walk below only recovers WHICH edges
// share a chain, never re-decides that they do.
func partitionPatchChains(edges []*Edge) ([][]*Edge, error) {
	refs := make([]patchchain.EdgeRef[*Edge, *Vertex], len(edges))
	for i, e := range edges {
		refs[i] = patchchain.EdgeRef[*Edge, *Vertex]{Edge: e, Start: vertexHandle(e.n.Start), End: vertexHandle(e.n.End)}
	}
	return patchchain.Partition(refs, func(v *Vertex) string { return renderCoord3(v.n.Position) })
}

// renderCoord3 formats a world position for a Table R diagnostic, the 3D
// analog of renderCoord's plane-local rendering.
func renderCoord3(p r3.Vec) string {
	return fmt.Sprintf(`(%s, %s, %s)`, renderCoord(p.X), renderCoord(p.Y), renderCoord(p.Z))
}

// provePatchChainPlane is gate 3: proves one chain's edges lie in a single
// plane. The first, exact arm (provePatchChainPlaneExact) is tried first and
// is the whole of what earlier increments proved; a second, LEVEL arm
// (provePatchChainPlaneLevel) is tried only when the first refuses on a
// nonzero bound, and never reads a coordinate, a residual or a bound
// magnitude (docs/surface-design.md §5.2's own two-arm amendment). Its
// return states what the level arm admitted: the levelToken and axialBound
// are what patchchain.buildFace reads for the new face's own plane and AxialDelta,
// mirroring what a prism cap already carries; a chain the exact arm admits
// instead publishes the zero token and a zero axialBound, unchanged.
func provePatchChainPlane(edges []*Edge) (levelToken, float64, bool, error) {
	exactErr := provePatchChainPlaneExact(edges)
	if exactErr == nil {
		return levelToken{}, 0, false, nil
	}
	if !errors.Is(exactErr, ErrUnsupported) {
		return levelToken{}, 0, false, exactErr
	}
	if lvl, bound, ok := provePatchChainPlaneLevel(edges); ok {
		return lvl, bound, true, nil
	}
	return levelToken{}, 0, false, exactErr
}

// provePatchChainPlaneLevel is gate 3's second arm: it requires every chain
// vertex, and every chain edge, to carry a levelToken with the SAME
// non-zero id (denotation.go) — an identity comparison alone, never a
// coordinate, a residual or a bound magnitude. A shared level proves the
// chain's true vertices lie on one plane by shared construction
// (docs/surface-design.md §5.2), so nothing here reads a position or a
// bound to decide admission; the returned token and bound are read back off
// the chain's own vertices only to state what the new face's own plane and
// AxialDelta must publish, never to decide whether this arm admits. ok is
// false whenever the chain holds no vertex, any vertex or edge carries no
// level (the zero value, "no certificate"), or not every one of them shares
// the same id.
func provePatchChainPlaneLevel(edges []*Edge) (levelToken, float64, bool) {
	verts := patchChainVertices(edges)
	if len(verts) == 0 {
		return levelToken{}, 0, false
	}
	lvl := verts[0].n.Level
	if lvl.ID == 0 {
		return levelToken{}, 0, false
	}
	bound := 0.0
	for _, v := range verts {
		if !sameLevel(v.n.Level, lvl) {
			return levelToken{}, 0, false
		}
		bound = math.Max(bound, v.n.Bound.Base())
	}
	for _, e := range edges {
		if !sameLevel(e.n.Level, lvl) {
			return levelToken{}, 0, false
		}
	}
	return lvl, bound, true
}

// provePatchChainPlaneExact is gate 3's first, exact arm: proves one chain's
// edges lie in a single plane, decided over dyadic.go's exact rational lift
// as a zero determinant, never a residual against a fitted plane
// (docs/surface-design.md §5.2).
//
// Every chain vertex must carry a zero bound, and every chain edge's own
// geometry must be exact (patchEdgeGeometryExact) — a bounded vertex or edge
// is only known to lie within its own bound, and fitting a plane to it would
// be exactly the fitted geometry §1.3 refuses. Both are [ErrUnsupported]
// (Table R row R6), which provePatchChainPlane's own second arm then gets a
// chance to admit instead.
//
// Three or more vertices that are not all collinear give the plane a
// determinant to take: patchchain.planeFromVertices searches for two vertex
// differences from a common point whose exact cross product is nonzero, and
// that cross product IS the plane's (unoriented) normal. Under three
// independent vertices — a lone closed circular edge has exactly one, a
// two-edge chain of two arcs has exactly two — there is no such determinant,
// and §5.2 is silent on what decides the plane there; this evaluator decides
// it from a curved edge's own carrier plane instead (patchchain.planeFromCarrier):
// a Circle3 or Arc3's Center and Axis already state a plane, and that is the
// only plane its whole curve — not just its two endpoints — can lie in.
//
// Once a candidate plane is in hand, from either source, every chain vertex
// must lie on it (an exact dot product against the plane normal), and every
// curved edge's OWN carrier plane must equal it (its Axis parallel to the
// proven normal, its Center on the proven plane) — a curved edge is planar
// only when its own carrier plane is the chain's, never merely its two
// endpoints. Any failure is a chain proven NON-planar, also
// [ErrUnsupported] (R6): decad's reject-only rule treats "not proven
// planar" and "proven non-planar" alike, since neither admits the chain.
func provePatchChainPlaneExact(edges []*Edge) error {
	verts := patchChainVertices(edges)
	points := make([]patchchain.Vertex, len(verts))
	for i, v := range verts {
		points[i] = patchchain.Vertex{Position: v.n.Position, Bound: v.n.Bound.Base()}
	}
	curves := make([]patchchain.Edge, len(edges))
	for i, e := range edges {
		center, axis, curved := patchCurveCarrier(e)
		curves[i] = patchchain.Edge{Exact: patchEdgeGeometryExact(e), Center: center, Axis: axis, Curved: curved}
	}
	return patchchain.ProvePlane(points, curves)
}

// patchChainVertices returns the chain's distinct vertices, deduplicated by
// pointer identity, in first-seen order.
func patchChainVertices(edges []*Edge) []*Vertex {
	seen := map[*Vertex]struct{}{}
	var out []*Vertex
	for _, e := range edges {
		for _, v := range [2]*Vertex{vertexHandle(e.n.Start), vertexHandle(e.n.End)} {
			if _, ok := seen[v]; ok {
				continue
			}
			seen[v] = struct{}{}
			out = append(out, v)
		}
	}
	return out
}

// patchEdgeGeometryExact reports whether e's own curve is proven exact,
// filling what §5.2 leaves silent about what "a curve carrying a nonzero
// bound" means at this gate. A Line3's whole geometry lives in its two
// vertices, already checked separately, so its own proven length bound —
// the same one Length()'s Exactness answers from — is a sound reading of it
// and is zero whenever they are exact.
//
// Circle3 and Arc3 carry Center, Axis and Radius with no bound field of
// their own (docs/surface-design.md §6.2's Table J amendment) — those ARE
// the held values, with nothing further to widen — so lengthBound is NOT
// this reading for them: a circular arc's length is never an exact
// rational (moments.go), so lengthBound is nonzero for essentially every
// circular edge regardless of how exactly its Center, Axis and Radius are
// known, and reading it here would refuse the commonest thing this gate
// exists to admit (a lone closed circular rim). lengthUnbounded is the one
// genuine signal available instead: this evaluator's own admission that
// even a bound on the edge's length could not be proven, which only a
// construction this gate has no other reason to trust would raise.
func patchEdgeGeometryExact(e *Edge) bool {
	switch e.n.Curve.(type) {
	case surfacegeom.Circle3, surfacegeom.Arc3:
		return !e.n.LengthUnbounded
	default:
		return e.n.LengthBound == 0 && !e.n.LengthUnbounded
	}
}

// patchCurveCarrier returns a curved edge's own carrier plane — Center and
// Axis — and reports true for Circle3 and Arc3. Every other Curve variant
// (Line3, and the free-form kinds no free rim edge of a body carries as a
// whole edge, docs/surface-design.md §6.2's Table J amendment) reports false:
// it has no carrier plane narrower than the chain's own.
func patchCurveCarrier(e *Edge) (center, axis r3.Vec, curved bool) {
	switch c := e.n.Curve.(type) {
	case surfacegeom.Circle3:
		return c.Center, c.Axis, true
	case surfacegeom.Arc3:
		return c.Center, c.Axis, true
	default:
		return r3.Vec{}, r3.Vec{}, false
	}
}

// Rule P (docs/surface-design.md §6.4) is the construction-proof gate a
// bodyPatchPayload earns from its own receiver, letting Rule S
// (stitch_flux.go) admit the "walls, then cap, then stitch" flow §6.1
// motivates. A bodyPatchPayload proves its own boundary does not
// self-intersect — payloadProvesSimple's own bodyPatchPayload arm
// (verify.go) — when all three hold, decided by
// bodyPatchPayloadProvesSimple below:
//
//  1. its receiver — the body PatchEdges was called on — itself admits
//     under Rule S, decided by the identical payloadProvesSimple predicate,
//     never a reimplementation of it;
//  2. every new face's chain is a COMPLETE free-edge chain of that
//     receiver's own end, not a proper subset of one;
//  3. every new face's plane is exactly one of the receiver feature's own
//     end planes.
//
// Under those three the patched assembly IS the receiver feature's own
// solid boundary — every chain vertex and edge PatchEdges selected is the
// SAME object the receiver's own build stamped, never a copy or a fit — so
// the receiver's own construction proof carries onto the patched body
// unchanged. Any chain this cannot decide keeps the payload undecided,
// reject-only exactly as every other Rule S arm.
//
// Conditions 2 and 3 are both decided through the LEVEL half of the
// shared-denotation certificate (denotation.go), never by a coordinate or a
// residual: internal/prismrecord/build.go's prismrecord.Build stamps every rim vertex and
// edge at one end with the SAME level token, minted once per build per end
// whenever the build's own section is drawn straight from its record
// (sectionDelta == 0) — regardless of whether that end's own coordinate
// ends up zero-bound or not. patchChainSharedLevel reads that identity off
// the chain's own edges AND vertices directly: a shared non-zero id proves
// the chain's plane IS that recorded level's plane, on the same terms
// §5.2's own gate 3 level arm already reads the identical token for,
// regardless of which of gate 3's two arms actually admitted the chain's
// planarity. A chain with no shared id — the exact-arm case, a chain from
// any receiver whose build mints no level token at all, or a chain a
// SECOND PatchEdges call selects from an already-patched body
// (patchchain.copyFacesUnder does not propagate a level token onto the copies it
// mints) — is undecided here, never guessed: Rule P names no distance
// threshold that could stand in for the identity check CLAUDE.md's
// reject-only rule requires.
//
// Completeness (condition 2) then asks whether that SAME id's full set of
// the receiver's own free edges is exactly the chain's own edge set: a
// receiver whose end holds more than one disjoint free-edge loop — an
// annular profile's inner and outer rims at one level — proves nothing
// about the WHOLE end's non-self-intersection from patching only one of
// them, so admitting on a proper subset would be unsound.
//
// Only prismPayload mints level tokens today, so Rule P admits a
// PatchEdges-capped surface-extruded tube's rims and nothing wider yet.

// bodyPatchPayloadProvesSimple decides payloadProvesSimple's bodyPatchPayload
// arm — Rule P, this file's own doc comment above. bodies is
// stitchOperandBodies reused verbatim: pp.faces is exactly the shape that
// function already reads an operand's source body from.
func bodyPatchPayloadProvesSimple(ctx context.Context, pp bodyPatchPayload) bool {
	bodies := stitchOperandBodies(faceHandles(pp.Faces))
	if len(bodies) != 1 || bodies[0] == nil || bodies[0].n.Payload == nil {
		return false
	}
	receiver := bodies[0]
	if !payloadProvesSimple(ctx, receiver.n.Payload) {
		return false
	}

	// Every receiver free edge, grouped by its own level id — the candidate
	// set condition 2 checks each chain against. An edge with no
	// certificate (id == 0) joins no group: it cannot be one of the
	// receiver's own end planes (denotation.go), and grouping by "no
	// certificate" would prove nothing about which plane it belongs to.
	receiverFreeByLevel := map[levelID][]*Edge{}
	for _, e := range receiver.Edges() {
		if !e.IsFree() || e.n.Level.ID == 0 {
			continue
		}
		receiverFreeByLevel[e.n.Level.ID] = append(receiverFreeByLevel[e.n.Level.ID], e)
	}

	for _, chain := range pp.Chains {
		chainEdges := edgeHandles(chain.Edges)
		id, ok := patchChainSharedLevel(chainEdges)
		if !ok {
			return false
		}
		if !edgeSetsEqual(receiverFreeByLevel[id], chainEdges) {
			return false
		}
	}
	return true
}

// patchChainSharedLevel is Rule P condition 3's own decision: the single
// non-zero level id every edge AND vertex of edges carries, or false when
// any of them carries a different id, the zero id ("no certificate",
// denotation.go), or edges is empty. Reading id alone, never origin or
// normal, is what keeps this an identity comparison rather than a fitted or
// residual one — sameLevel's own contract. Checking every VERTEX as well as
// every edge is load-bearing: an edge's own id says its CURVE was stamped at
// one level, never that both its endpoints were too.
func patchChainSharedLevel(edges []*Edge) (levelID, bool) {
	if len(edges) == 0 {
		return 0, false
	}
	id := edges[0].n.Level.ID
	if id == 0 {
		return 0, false
	}
	for _, e := range edges {
		if e.n.Level.ID != id || e.n.Start.Level.ID != id || e.n.End.Level.ID != id {
			return 0, false
		}
	}
	return id, true
}

// edgeSetsEqual reports whether a and b hold the same *Edge pointers, order
// and duplicates aside — Rule P condition 2's own set-equality decision,
// never a count alone or a residual.
func edgeSetsEqual(a, b []*Edge) bool {
	if len(a) != len(b) {
		return false
	}
	set := make(map[*Edge]struct{}, len(a))
	for _, e := range a {
		set[e] = struct{}{}
	}
	for _, e := range b {
		if _, ok := set[e]; !ok {
			return false
		}
	}
	return true
}
