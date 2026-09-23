package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Body.Patch of docs/surface-design.md §5.2: a planar fill of
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
// (copyPatchFacesUnder's edgeFor) is what stitch.go's weld groups exist to
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
//     crossing audit (buildPatchFace) — that audit's own ErrUnsupported is
//     remapped to ErrDegenerate at this boundary (patchRemapCrossingError),
//     because a self-crossing chain is bad input rather than evaluator
//     reach, on docs/surface-design.md §5.2's own instruction.
//
// Orientation — which sense each new face takes on its chain's edges — is a
// fifth, purely combinatorial decision with no geometry in it (orientPatchChain):
// each edge is traversed by the one face already adjacent to it in some
// sense, and the patch takes the opposite sense. Where a chain's own
// adjacent faces do not agree among themselves — unreachable through this
// evaluator's own builders, which each leave one shell consistently
// oriented (docs/surface-design.md §2.3), but not excluded by the public
// seam — that disagreement is ErrDegenerate, Table R row R18 (new: §5.2
// named the case without giving it a row).
//
// evalBodyPatchContext is the whole evaluator, shared by the initial build
// and every later Placed/Duplicate/PlacedCopy re-evaluation: gate 3's exact
// plane proof runs ONCE, in buildPatchChains, and is never re-proven (a
// rigid motion preserves planarity exactly, so re-proving would only make
// Placed refuse most rotations for no soundness gain). Its own result is
// discarded once the proof passes — the payload carries only the chain's own
// edges, never a plane value — because the new face's actual plane is
// derived fresh, every evaluation, from that chain's own (placed) oriented
// geometry (patchChainOrientedNormal). Gate 4's crossing audit and the
// orientation derivation are NOT replayed from a stored answer either: they
// run again on every placement, exactly as Stitch re-runs its own crossing
// audit after a placement, because rounding can bring two placed points into
// contact — or apart — that the unplaced ones were not.

// Body.Patch's own two Table R rows this PR adds, beside the existing R4
// (gate 1), R5 (gate 2, and gate 4 remapped), R6 (gate 3) and R16 (selector)
// rows §5.2 already names.
const (
	// patchRowFacesDisagree is Table R row R18: a chain's adjacent faces
	// cannot be brought into agreement on its patch's orientation.
	patchRowFacesDisagree = `docs/surface-design.md Table R row R18`
	// patchRowNoPayload is Table R row R19: the receiver carries no
	// evaluator payload for this evaluator to rebuild from.
	patchRowNoPayload = `docs/surface-design.md Table R row R19`
)

// Patch resolves sel against the receiver's own live topology, fills
// every closed chain of free edges the selection proves, and returns a new
// body carrying the receiver's own faces plus one new planar face per chain,
// retiring the receiver (docs/surface-design.md §5.2). The receiver's own
// topology is never mutated: the result is built by REBUILDING the receiver
// from its own faces, so a retired receiver stays readable and unmodified
// for any caller still holding it.
//
// A nil receiver, or one this evaluator did not build (b.payload == nil), is
// [ErrUnsupported] (Table R row R19): a body reads its readable topology
// regardless of payload, but Patch's own re-evaluation contract — a placed
// copy replays from the recorded plane rather than re-proving it — needs the
// same payload-carrying contract every other rebuilding operation
// (Placed, PlacedCopy, Duplicate) already requires. A body owned by a
// different document is [ErrForeignBody], and a retired receiver is
// [ErrRetiredBody]. sel's own resolution failure ([SelectionError] wrapping
// [ErrNoMatch] or [ErrCardinality], Table R row R16) and every one of gates
// 1-4's refusals (R4, R5, R6, R18) surface unchanged. A failed call leaves
// the document and the receiver unchanged.
func (b *Body) Patch(ctx context.Context, sel EdgeSelector) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a patch`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: a nil body names nothing to patch`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: Body.Patch requires a receiver this evaluator built (%s)`, ErrUnsupported, patchRowNoPayload)
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
	body, err := evalBodyPatchContext(ctx, d, ref, b.Faces(), b.bounds, chains, r3.Identity())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}

// bodyPatchChain is one proven chain, carried in the payload from the
// initial build to every later placement: the chain's own (receiver-owned,
// read-only) edges. Gate 3's proof (provePatchChainPlane) is a pure yes/no
// decision over these same edges' own held data, never re-run once passed —
// a rigid motion preserves planarity exactly, so nothing about the proof
// changes across a placement. A chain the EXACT arm admitted publishes no
// value gate 3 computed: the new face's plane is derived fresh, every
// evaluation, from the chain's own (placed) oriented geometry
// (patchChainOrientedNormal). A chain the LEVEL arm admitted instead
// publishes ITS OWN token, below, which buildPatchFace reads for the
// plane's origin and normal directly — never a value fitted to held vertex
// coordinates, which are only approximately coplanar for a chain admitted
// this way (denotation.go's own doc comment states why).
type bodyPatchChain struct {
	edges []*Edge
	// axialBound and hasAxialBound are the proven axial displacement gate 3's
	// LEVEL arm (provePatchChainPlaneLevel) admitted this chain under —
	// stamped onto the new face's own axialDelta exactly as prism_build.go
	// stamps a prism cap's (buildPatchFace, docs/surface-design.md §5.2).
	// hasAxialBound stays false, and axialBound zero, whenever gate 3 instead
	// admitted the chain through its existing exact (zero-bound) arm.
	axialBound    float64
	hasAxialBound bool
	// level is the token the LEVEL arm admitted this chain under — the zero
	// value whenever hasAxialBound is false. buildPatchFace reads its own
	// origin/normal, transformed by this evaluation's own placement, as the
	// published plane: never patchChainOrientedNormal's fitted vector, which
	// this token's own doc comment (denotation.go) states is unsound for a
	// chain admitted by identity rather than by proven-exact coplanarity.
	level levelToken
}

// bodyPatchPayload is Body.Patch's own record: the retiring receiver's own
// faces, its already-proven Bounds, and each chain's proven plane — never a
// built topology (unstitch.go's "held-B-rep-under-a-rigid-motion" idea,
// widened here from one face to a whole face set plus the faces the patch
// mints on top of it). placed() replays evalBodyPatchContext from these same
// values under the composed transform, exactly as unstitchPayload.placed
// replays evalUnstitchFaceContext.
type bodyPatchPayload struct {
	xform r3.Transform
	// delta is the proven displacement rigidRoundAllow charges against this
	// body's placed geometry — zero exactly when xform is the identity
	// transform, an exact struct comparison (unstitchPayload's own delta).
	delta float64
	// faces is the ORIGINAL (now retired) receiver's own face set: read-only,
	// and never mutated or aliased into a live body's topology
	// (copyPatchFacesUnder always mints a fresh copy from it).
	faces []*Face
	// bounds is the retiring receiver's own proven, UNPLACED Bounds — reused
	// rather than re-derived, on the same reasoning unstitchBounds already
	// states: every new face's boundary is already receiver geometry, and a
	// bounded planar region lies inside its own boundary's box, so the
	// receiver's own box already bounds the patched result too.
	bounds Box
	chains []bodyPatchChain
}

func (pp bodyPatchPayload) transform() r3.Transform { return pp.xform }

func (pp bodyPatchPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	return evalBodyPatchContext(ctx, d, ref, pp.faces, pp.bounds, pp.chains, composed)
}

// buildPatchChains runs gates 1-3 against the selector's resolved edges: gate
// 1 (every edge free), gate 2 (partition into closed chains), and gate 3
// (each chain's exact plane proof). Gate 4 (simplicity) and the orientation
// derivation are NOT run here — they run inside evalBodyPatchContext, every
// time, including the very first build, since building the actual face
// needs the SAME per-placement machinery a later Placed call replays
// (docs/surface-design.md's "re-run the combinatorial and in-plane gates on
// re-evaluation, but not the plane proof" decision).
func buildPatchChains(edges []*Edge) ([]bodyPatchChain, error) {
	for _, e := range edges {
		if !e.IsFree() {
			return nil, fmt.Errorf(`%w: Body.Patch selection holds an edge that is not free (docs/surface-design.md Table R row R4)`, ErrDegenerate)
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
		chains[i] = bodyPatchChain{edges: g, axialBound: axialBound, hasAxialBound: hasAxialBound, level: lvl}
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
	degree := map[*Vertex]int{}
	for _, e := range edges {
		degree[e.start]++
		degree[e.end]++
	}
	for v, n := range degree {
		if n != 2 {
			return nil, fmt.Errorf(`%w: Body.Patch selection is not exactly closed chains — a vertex at %s has degree %d over the selection, not 2 (docs/surface-design.md Table R row R5)`,
				ErrDegenerate, renderCoord3(v.position), n)
		}
	}

	// adj pairs every non-closed edge with the two OTHER edges — never
	// itself — touching each of its own vertices; a closed edge is handled
	// separately below, since both its "slots" are itself.
	adj := map[*Vertex][]*Edge{}
	for _, e := range edges {
		if e.start == e.end {
			continue
		}
		adj[e.start] = append(adj[e.start], e)
		adj[e.end] = append(adj[e.end], e)
	}

	visited := map[*Edge]bool{}
	var chains [][]*Edge
	for _, e0 := range edges {
		if visited[e0] {
			continue
		}
		if e0.start == e0.end {
			visited[e0] = true
			chains = append(chains, []*Edge{e0})
			continue
		}
		visited[e0] = true
		chain := []*Edge{e0}
		start, current := e0.start, e0.end
		for current != start {
			var next *Edge
			for _, cand := range adj[current] {
				if !visited[cand] {
					next = cand
					break
				}
			}
			if next == nil {
				// Unreachable given the degree check above (a 2-regular
				// graph's connected components are exactly cycles), kept as
				// a defensive refusal rather than a silent short chain.
				return nil, fmt.Errorf(`%w: Body.Patch selection does not partition into closed chains (docs/surface-design.md Table R row R5)`, ErrDegenerate)
			}
			visited[next] = true
			chain = append(chain, next)
			if next.start == current {
				current = next.end
			} else {
				current = next.start
			}
		}
		chains = append(chains, chain)
	}
	return chains, nil
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
// are what buildPatchFace reads for the new face's own plane and axialDelta,
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
// axialDelta must publish, never to decide whether this arm admits. ok is
// false whenever the chain holds no vertex, any vertex or edge carries no
// level (the zero value, "no certificate"), or not every one of them shares
// the same id.
func provePatchChainPlaneLevel(edges []*Edge) (levelToken, float64, bool) {
	verts := patchChainVertices(edges)
	if len(verts) == 0 {
		return levelToken{}, 0, false
	}
	lvl := verts[0].level
	if lvl.id == 0 {
		return levelToken{}, 0, false
	}
	bound := 0.0
	for _, v := range verts {
		if !sameLevel(v.level, lvl) {
			return levelToken{}, 0, false
		}
		bound = math.Max(bound, v.bound.Base())
	}
	for _, e := range edges {
		if !sameLevel(e.level, lvl) {
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
// determinant to take: patchPlaneFromVertices searches for two vertex
// differences from a common point whose exact cross product is nonzero, and
// that cross product IS the plane's (unoriented) normal. Under three
// independent vertices — a lone closed circular edge has exactly one, a
// two-edge chain of two arcs has exactly two — there is no such determinant,
// and §5.2 is silent on what decides the plane there; this evaluator decides
// it from a curved edge's own carrier plane instead (patchPlaneFromCarrier):
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
	for _, v := range verts {
		if !finiteVec(v.position) || v.bound.Base() != 0 {
			return fmt.Errorf(`%w: a Body.Patch chain vertex carries a nonzero bound, so its position is not proven exactly (docs/surface-design.md Table R row R6)`, ErrUnsupported)
		}
	}
	for _, e := range edges {
		if !patchEdgeGeometryExact(e) {
			return fmt.Errorf(`%w: a Body.Patch chain edge's own curve carries a nonzero bound, so its geometry is not proven exactly (docs/surface-design.md Table R row R6)`, ErrUnsupported)
		}
		if center, axis, curved := patchCurveCarrier(e); curved && (!finiteVec(center) || !finiteVec(axis)) {
			return fmt.Errorf(`%w: a Body.Patch chain edge's carrier plane is not representable (docs/surface-design.md Table R row R6)`, ErrUnsupported)
		}
	}

	normal, origin, ok := patchPlaneFromVertices(verts)
	if !ok {
		normal, origin, ok = patchPlaneFromCarrier(edges)
	}
	if !ok {
		return fmt.Errorf(`%w: a Body.Patch chain has too few independent points to determine a plane (docs/surface-design.md Table R row R6)`, ErrUnsupported)
	}

	normalDy := dyVec(normal)
	originDy := dyVec(origin)
	for _, v := range verts {
		rel := dvSub(dyVec(v.position), originDy)
		if !dvDot(normalDy, rel).isZero() {
			return fmt.Errorf(`%w: a Body.Patch chain is not planar (docs/surface-design.md Table R row R6)`, ErrUnsupported)
		}
	}
	for _, e := range edges {
		center, axis, curved := patchCurveCarrier(e)
		if !curved {
			continue
		}
		if !dvIsZero(dvCross(dyVec(axis), normalDy)) {
			return fmt.Errorf(`%w: a Body.Patch chain edge's carrier plane does not match the chain's plane (docs/surface-design.md Table R row R6)`, ErrUnsupported)
		}
		rel := dvSub(dyVec(center), originDy)
		if !dvDot(normalDy, rel).isZero() {
			return fmt.Errorf(`%w: a Body.Patch chain edge's carrier plane does not match the chain's plane (docs/surface-design.md Table R row R6)`, ErrUnsupported)
		}
	}
	return nil
}

// patchChainVertices returns the chain's distinct vertices, deduplicated by
// pointer identity, in first-seen order.
func patchChainVertices(edges []*Edge) []*Vertex {
	seen := map[*Vertex]struct{}{}
	var out []*Vertex
	for _, e := range edges {
		for _, v := range [2]*Vertex{e.start, e.end} {
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
	switch e.curve.(type) {
	case Circle3, Arc3:
		return !e.lengthUnbounded
	default:
		return e.lengthBound == 0 && !e.lengthUnbounded
	}
}

// patchCurveCarrier returns a curved edge's own carrier plane — Center and
// Axis — and reports true for Circle3 and Arc3. Every other Curve variant
// (Line3, and the free-form kinds no free rim edge of a body carries as a
// whole edge, docs/surface-design.md §6.2's Table J amendment) reports false:
// it has no carrier plane narrower than the chain's own.
func patchCurveCarrier(e *Edge) (center, axis r3.Vec, curved bool) {
	switch c := e.curve.(type) {
	case Circle3:
		return c.Center, c.Axis, true
	case Arc3:
		return c.Center, c.Axis, true
	default:
		return r3.Vec{}, r3.Vec{}, false
	}
}

// patchPlaneFromVertices searches for three chain vertices that are not all
// collinear — a common point v0 and two others whose position differences
// from it have a nonzero exact cross product — and returns that cross
// product as the plane's (unoriented) normal, with v0 as a point on the
// plane. ok is false when fewer than three vertices exist or every vertex is
// collinear with the first two found, the "under three independent
// vertices" case provePatchChainPlane's own doc comment states.
func patchPlaneFromVertices(verts []*Vertex) (normal, origin r3.Vec, ok bool) {
	if len(verts) < 3 {
		return r3.Vec{}, r3.Vec{}, false
	}
	v0 := verts[0].position
	d0 := dyVec(v0)
	for i := 1; i < len(verts); i++ {
		vi := verts[i].position
		di := dvSub(dyVec(vi), d0)
		if dvIsZero(di) {
			continue
		}
		for j := i + 1; j < len(verts); j++ {
			vj := verts[j].position
			dj := dvSub(dyVec(vj), d0)
			cross := dvCross(di, dj)
			if !dvIsZero(cross) {
				return vi.Sub(v0).Cross(vj.Sub(v0)), v0, true
			}
		}
	}
	return r3.Vec{}, r3.Vec{}, false
}

// patchPlaneFromCarrier returns the first curved edge's own carrier plane —
// the fallback provePatchChainPlane's own doc comment states for a chain
// with fewer than three independent vertices.
func patchPlaneFromCarrier(edges []*Edge) (normal, origin r3.Vec, ok bool) {
	for _, e := range edges {
		if center, axis, curved := patchCurveCarrier(e); curved {
			return axis, center, true
		}
	}
	return r3.Vec{}, r3.Vec{}, false
}

// patchOrientedEdge is one chain edge under the orientation decision: the
// receiver's own (read-only) edge, and the sense — forward from Edge.Start
// to Edge.End, or backward — the new patch face traverses it in.
type patchOrientedEdge struct {
	old     *Edge
	forward bool
}

// orientPatchChain derives the one combinatorial answer §5.2's ORIENTATION
// section states: each chain edge is traversed by its one adjacent face
// (the edge is free, so there is exactly one) in some sense, and the patch
// traverses it in the opposite sense. That per-edge choice is then walked
// into a single connected cyclic order — every edge's chosen End must equal
// exactly one other edge's chosen Start, and following that chain from any
// edge must visit every edge in the chain and return to the start — which is
// what "the patch's own boundary is a consistent walk" actually requires
// beyond the per-edge sense alone. Where it is not (unreachable through this
// evaluator's own builders, each of which leaves one shell consistently
// oriented, docs/surface-design.md §2.3, but not excluded by the public
// seam), that is patchRowFacesDisagree, [ErrDegenerate].
func orientPatchChain(chain bodyPatchChain) ([]patchOrientedEdge, error) {
	oriented := make([]patchOrientedEdge, len(chain.edges))
	for i, e := range chain.edges {
		if len(e.faces) != 1 {
			return nil, fmt.Errorf(`%w: a Body.Patch chain edge is not free (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		dir, ok := coedgeDirectionFor(e.faces[0], e)
		if !ok {
			return nil, fmt.Errorf(`%w: a Body.Patch chain edge's adjacent face does not use it (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		oriented[i] = patchOrientedEdge{old: e, forward: !dir}
	}

	nextFrom := map[*Vertex]*patchOrientedEdge{}
	for i := range oriented {
		oe := &oriented[i]
		start := oe.old.end
		if oe.forward {
			start = oe.old.start
		}
		if _, dup := nextFrom[start]; dup {
			return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		nextFrom[start] = oe
	}

	order := make([]patchOrientedEdge, 0, len(oriented))
	seen := map[*Edge]bool{}
	cur, curForward := oriented[0].old, oriented[0].forward
	for range oriented {
		if seen[cur] {
			return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		seen[cur] = true
		order = append(order, patchOrientedEdge{old: cur, forward: curForward})
		end := cur.start
		if curForward {
			end = cur.end
		}
		nxt, ok := nextFrom[end]
		if !ok {
			return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		cur, curForward = nxt.old, nxt.forward
	}
	if cur != oriented[0].old || len(seen) != len(oriented) {
		return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, ErrDegenerate, patchRowFacesDisagree)
	}
	return order, nil
}

// evalBodyPatchContext is Body.Patch's whole evaluator, shared by the
// initial build and every later placement: it rebuilds the receiver's own
// face set under xform (copyPatchFacesUnder), builds one new face per chain
// on top of that same rebuild (buildPatchFace, which re-derives orientation
// and re-runs gate 4 fresh every time), assembles the result, and publishes
// its measurements.
func evalBodyPatchContext(ctx context.Context, d *Document, ref producerID, srcFaces []*Face, srcBounds Box, chains []bodyPatchChain, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	maxInputAbs := 0.0
	for _, f := range srcFaces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				maxInputAbs = max(maxInputAbs, vecMaxAbs(ce.edge.start.position), vecMaxAbs(ce.edge.end.position))
			}
		}
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}

	newFaces, edgeCopy, err := copyPatchFacesUnder(ctx, srcFaces, xform, delta)
	if err != nil {
		return nil, err
	}

	patchFaces := make([]*Face, len(chains))
	for i, chain := range chains {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		pf, err := buildPatchFace(ctx, ref, chain, edgeCopy, xform, delta)
		if err != nil {
			return nil, err
		}
		patchFaces[i] = pf
	}

	allFaces := make([]*Face, 0, len(newFaces)+len(patchFaces))
	allFaces = append(allFaces, newFaces...)
	allFaces = append(allFaces, patchFaces...)
	if err := attachFaceLoopsContext(ctx, allFaces); err != nil {
		return nil, err
	}

	// Kind and lumps/shells come from the existing sheetLumps, run only AFTER
	// every face is attached (its own contract). Body.Patch never publishes a
	// solid: closing a sheet's last free edge leaves a CLOSED sheet reporting
	// BodySheet with IsSolid() false, since Stitch alone makes the material
	// claim (docs/surface-design.md §2.1).
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	for _, f := range allFaces {
		f.body = body
	}
	body.lumps = sheetLumps(allFaces)

	// Area is the receiver's own faces' area plus each new face's, composed
	// through boundedAdd — never a re-derived reading — exactly the sum
	// evalStitchContext already runs over its own constituent faces
	// (docs/surface-design.md §6.4/§8), reused here for one operand's own
	// face set plus the faces this call adds to it.
	areaAcc := boundedScalar{}
	for _, f := range allFaces {
		areaAcc = boundedAdd(areaAcc, measuredScalar(f.area, f.areaBound))
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(areaAcc.value),
		Exactness: exactnessOf(areaAcc.bound),
		Bound:     units.SquareMillimeters(areaAcc.bound),
	}

	// Bounds is the receiver's own already-proven box: every new face's
	// boundary is already receiver geometry, and a bounded planar region
	// lies inside its own boundary's box, so unstitchBounds's own reasoning
	// — reuse the operand's own extent rather than re-deriving a tighter one
	// — carries unchanged from one face to a whole rebuilt face set.
	bounds, err := unstitchBounds(srcBounds, xform, delta)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds

	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}

	body.payload = bodyPatchPayload{xform: xform, delta: delta, faces: srcFaces, bounds: srcBounds, chains: chains}
	return body, nil
}

// copyPatchFacesUnder deep-copies every one of srcFaces under xform, minting
// fresh Vertex, Edge and Face values that share no pointer with the retiring
// receiver (docs/surface-design.md's "rebuild the receiver from its
// payload" decision): a retired body stays readable, and a caller may still
// hold it, so mutating its topology through a shared pointer would corrupt
// what they read.
//
// It is unstitch.go's copyFaceUnderContext widened from one face to a whole
// face SET: an edge or vertex used by two of the receiver's own faces — an
// ordinary interior edge, exactly as common on a sheet as on a solid — still
// resolves to the SAME new object via the returned edgeCopy cache, the same
// per-old-edge sharing stitch.go's rebuildStitchTopology relies on for its
// own operand faces. Unlike that function, no weld plan is consulted: a
// patch never merges two DIFFERENT edges into one, it only gives an
// already-free edge a second face once buildPatchFace's own coedge is
// attached, so the plain per-pointer cache is the whole of what sharing
// needs here.
func copyPatchFacesUnder(ctx context.Context, srcFaces []*Face, xform r3.Transform, delta float64) ([]*Face, map[*Edge]*Edge, error) {
	newVertByOld := map[*Vertex]*Vertex{}
	vertexFor := func(old *Vertex) (*Vertex, error) {
		if nv, ok := newVertByOld[old]; ok {
			return nv, nil
		}
		p := xform.Apply(old.position)
		if !finiteVec(p) {
			return nil, fmt.Errorf(`%w: a placed patch vertex is not representable`, ErrUnsupported)
		}
		bound := old.bound.Base()
		if delta > 0 {
			bound = absSumUpper(bound, delta)
		}
		// The CURVE half of the shared-denotation certificate (denotation.go)
		// restates under xform, composing rather than overwriting, exactly
		// as unstitch.go's copyFaceUnderContext does.
		nv := &Vertex{position: p, bound: units.Millimeters(bound), denot: old.denot.compose(xform)}
		newVertByOld[old] = nv
		return nv, nil
	}

	newEdgeByOld := map[*Edge]*Edge{}
	edgeFor := func(old *Edge) (*Edge, error) {
		if ne, ok := newEdgeByOld[old]; ok {
			return ne, nil
		}
		curve, err := transformCurve(old.curve, xform)
		if err != nil {
			return nil, err
		}
		start, err := vertexFor(old.start)
		if err != nil {
			return nil, err
		}
		end, err := vertexFor(old.end)
		if err != nil {
			return nil, err
		}
		lengthBound := old.lengthBound
		if delta > 0 {
			lengthBound = absSumUpper(lengthBound, delta)
		}
		ne := &Edge{
			curve:           curve,
			start:           start,
			end:             end,
			convex:          old.convex,
			length:          old.length,
			lengthBound:     lengthBound,
			lengthUnbounded: old.lengthUnbounded,
			denot:           old.denot.compose(xform),
		}
		newEdgeByOld[old] = ne
		return ne, nil
	}

	newFaces := make([]*Face, len(srcFaces))
	for i, f := range srcFaces {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		surface, err := transformSurface(f.surface, xform)
		if err != nil {
			return nil, nil, err
		}
		nf := &Face{
			surface:    surface,
			origins:    append([]FeatureRef(nil), f.origins...),
			area:       f.area,
			areaBound:  f.areaBound,
			reversed:   f.reversed,
			heldPlanar: f.heldPlanar,
		}
		for _, l := range f.loops {
			coedges := make([]coedge, len(l.coedges))
			for j, ce := range l.coedges {
				ne, err := edgeFor(ce.edge)
				if err != nil {
					return nil, nil, err
				}
				coedges[j] = coedge{edge: ne, forward: ce.forward}
			}
			nf.loops = append(nf.loops, &Loop{coedges: coedges, outer: l.outer})
		}
		newFaces[i] = nf
	}
	return newFaces, newEdgeByOld, nil
}

// buildPatchFace builds one chain's new face: it derives the chain's
// orientation (orientPatchChain) and its plane.
//
// A chain the EXACT arm admitted derives the plane frame's normal and origin
// DETERMINISTICALLY from that same oriented, placed geometry
// (patchChainOrientedNormal, patchChainWalkOrigin) — never by trying a
// candidate and correcting its sign from a computed area, which would
// silently absorb a wrong orientation instead of surfacing it. This is
// sound because gate 3's exact arm already proved these SAME held vertex
// coordinates bit-for-bit coplanar (dyadic.go's exact rational lift), so
// fitting a normal to them fits one to data that genuinely, exactly, is
// coplanar.
//
// A chain the LEVEL arm admitted instead reads the plane's normal and
// origin from the chain's own levelToken, transformed by this evaluation's
// own placement (xform) — never fitted to held vertex coordinates, which
// gate 3's level arm never proved exactly coplanar (denotation.go's own doc
// comment states why a fit would be unsound here). patchChainOrientedNormal
// still runs for such a chain, but only to settle which of the token's two
// normal directions (±) matches the chain's own walk sense
// (patchChainLevelNormal) — its MAGNITUDE, and any tilt a fit to
// approximately-coplanar data would carry, are discarded.
//
// Either way this re-runs gate 4's crossing audit, and reads the face's area
// from the same moments engine Document.Patch's own single planar face uses.
func buildPatchFace(ctx context.Context, ref producerID, chain bodyPatchChain, edgeCopy map[*Edge]*Edge, xform r3.Transform, delta float64) (*Face, error) {
	ordered, err := orientPatchChain(chain)
	if err != nil {
		return nil, err
	}

	fitted, err := patchChainOrientedNormal(ordered, edgeCopy)
	if err != nil {
		return nil, err
	}

	normal := fitted
	origin := patchChainWalkOrigin(ordered, edgeCopy)
	axialDelta := chain.axialBound
	if chain.hasAxialBound {
		normal, err = patchChainLevelNormal(fitted, xform.ApplyDir(chain.level.normal))
		if err != nil {
			return nil, err
		}
		origin = xform.Apply(chain.level.origin)
		// A placement's own rounding (rigidRoundAllow) displaces the token's
		// origin exactly as it displaces every other placed coordinate this
		// evaluator publishes (copyPatchFacesUnder's own vertex/edge
		// widening), so it folds into the SAME axialDelta the token's own
		// bound already states.
		if delta > 0 {
			axialDelta = absSumUpper(axialDelta, delta)
		}
	}
	frame, segs, err := patchChainFrameAndSegments(ordered, edgeCopy, origin, normal)
	if err != nil {
		return nil, err
	}

	ig, err := patchChainIntegrals(ctx, segs)
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: a Body.Patch chain encloses no area`, ErrDegenerate)
	}

	budget := newWorkBudget(ctx)
	segEntries, err := buildSegEntriesBudget(budget, []LoopRecord{{Segments: segs}})
	if err != nil {
		return nil, patchRemapCrossingError(err)
	}
	if err := crossingAuditBudget(budget, segEntries); err != nil {
		return nil, patchRemapCrossingError(err)
	}

	coedges := make([]coedge, len(ordered))
	for i, oe := range ordered {
		coedges[i] = coedge{edge: edgeCopy[oe.old], forward: oe.forward}
	}

	return &Face{
		surface:   Plane{Frame: frame},
		origins:   []FeatureRef{{producer: ref, Role: rolePatch}},
		loops:     []*Loop{{coedges: coedges, outer: true}},
		area:      ig.area,
		areaBound: ig.areaBound,
		// A chain gate 3's level arm admitted has a proven-bounded plane
		// origin, never a zero one: the same fields prism_build.go already
		// sets on a prism's own caps (docs/surface-design.md §5.2). A chain
		// the exact arm admitted instead carries a zero, unset bound here.
		axialDelta:    axialDelta,
		hasAxialDelta: chain.hasAxialBound,
	}, nil
}

// patchChainIntegrals runs the same closed-form region integral
// Document.Patch's own single planar face uses (moments.go), over a
// single-loop, no-hole record — reused rather than a bespoke shoelace sum,
// so a patch face's Area answers on the identical Exactness/Bound terms
// docs/surface-design.md §5.1 already states for a sketch-recorded patch.
func patchChainIntegrals(ctx context.Context, segs []CurveSegment) (regionIntegrals, error) {
	work := newFreeformWork()
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}.evaluatorIntegralsContext(ctx, momentAreaOrder, work)
}

// patchRemapCrossingError maps fillet_audit.go's own [ErrUnsupported] boundary
// -- crossing audit refusal to [ErrDegenerate] at gate 4's own boundary
// (docs/surface-design.md §5.2, Table R row R5): a self-crossing chain is
// bad input this evaluator will never admit under a finer tolerance, unlike
// the evaluator-reach refusals ErrUnsupported names elsewhere, so it takes
// the sentinel a caller should branch on as unfixable rather than staged.
func patchRemapCrossingError(err error) error {
	if err == nil {
		return nil
	}
	if errors.Is(err, ErrUnsupported) {
		return fmt.Errorf(`%w: a Body.Patch chain's plane-local walk crosses or touches itself (docs/surface-design.md Table R row R5)`, ErrDegenerate)
	}
	return err
}

// patchChainOrientedNormal derives a normal DETERMINISTICALLY from the
// chain's already-oriented, already-placed geometry — never by trying one of
// the two candidate signs and correcting it from a computed area, which
// would silently accept either sign and so could never surface a wrong
// orientPatchChain answer (a dropped sense reversal there would still read a
// valid, merely mirror-image, loop). For a chain the EXACT arm admitted,
// buildPatchFace publishes this vector as the face's own normal outright.
// For a chain the LEVEL arm admitted, buildPatchFace instead uses it only to
// pick which of the level token's own two normal directions matches this
// walk's sense (patchChainLevelNormal): the fit below is over held vertex
// coordinates gate 3's level arm never proved exactly coplanar, so its
// MAGNITUDE and any tilt away from the true plane are never published.
//
// A curved (Circle3 or Arc3) edge's own effective axis — its Axis when
// walked forward, negated when walked backward, since walking a curve
// backward is CCW about the opposite axis — already states an outward
// normal, so the first one found decides it outright: gate 3 already proved
// every curved edge's carrier plane is the chain's own, so any one of them
// names the same plane, and orientPatchChain already fixed which sense this
// edge is walked in.
//
// A chain of straight edges alone carries no such axis, so this falls back
// to Newell's method: for a closed polygon, Σ Pi × Pi+1 over its vertices IN
// WALK ORDER is twice the signed area vector, and that sum is provably
// independent of where the origin is taken (shifting every point by a
// constant c adds Σ Pi×c + Σ c×Pi+1 + Σ c×c, and the first two terms cancel
// over a closed walk since ΣPi and ΣPi+1 are the same set) — so it needs no
// anchor point, and reversing the walk direction negates it outright, with
// no candidate-and-correct step to mask that reversal.
func patchChainOrientedNormal(ordered []patchOrientedEdge, edgeCopy map[*Edge]*Edge) (r3.Vec, error) {
	for _, oe := range ordered {
		ne := edgeCopy[oe.old]
		var axis r3.Vec
		switch c := ne.curve.(type) {
		case Circle3:
			axis = c.Axis
		case Arc3:
			axis = c.Axis
		default:
			continue
		}
		if !oe.forward {
			axis = axis.Scale(-1)
		}
		return axis, nil
	}

	var sum r3.Vec
	for _, oe := range ordered {
		ne := edgeCopy[oe.old]
		a, b := ne.start.position, ne.end.position
		if !oe.forward {
			a, b = b, a
		}
		sum = sum.Add(a.Cross(b))
	}
	if _, ok := sum.Normalize(); !ok {
		return r3.Vec{}, fmt.Errorf(`%w: a Body.Patch chain's orientation is degenerate`, ErrDegenerate)
	}
	return sum, nil
}

// patchChainWalkOrigin is the exact arm's own plane origin: the chain's
// first (already placed) vertex in walk order. A chain the exact arm
// admitted has this vertex bit-for-bit coplanar with every other chain
// vertex (gate 3's own dyadic proof), so using it as the plane's origin
// introduces no coordinate this evaluator has not already verified.
func patchChainWalkOrigin(ordered []patchOrientedEdge, edgeCopy map[*Edge]*Edge) r3.Vec {
	first := edgeCopy[ordered[0].old]
	if !ordered[0].forward {
		return first.end.position
	}
	return first.start.position
}

// patchChainLevelNormal resolves the sign ambiguity between fitted —
// patchChainOrientedNormal's own vector, fit to held vertex coordinates
// gate 3's level arm never proved exactly coplanar — and tokenNormal, the
// level token's own frame-derived normal (denotation.go), already
// transformed by this evaluation's own placement. fitted decides ONLY which
// of ±tokenNormal matches the chain's own walk sense: its dot product with
// tokenNormal is, up to ordinary floating rounding, the polygon's own
// signed area times |tokenNormal|² — a magnitude many orders above the
// fitting error a rotated frame's rounding could ever introduce
// (docs/surface-design.md §5.2), so that rounding can never flip its sign.
// The PUBLISHED direction is tokenNormal itself, sign-corrected — never
// fitted — so the returned vector is exactly the recorded frame's own
// normal under this evaluation's placement, with no fitting argument
// needed for it at all.
func patchChainLevelNormal(fitted, tokenNormal r3.Vec) (r3.Vec, error) {
	sign := fitted.Dot(tokenNormal)
	if sign == 0 {
		return r3.Vec{}, fmt.Errorf(`%w: a Body.Patch chain's orientation is degenerate against its own level`, ErrDegenerate)
	}
	if sign < 0 {
		return tokenNormal.Scale(-1), nil
	}
	return tokenNormal, nil
}

// patchChainFrameAndSegments builds the new face's plane frame — at origin,
// with normal as given — and the chain's plane-local segment record: one
// CurveSegment per oriented edge, in walk order, so the record is a valid
// connected LoopRecord for fillet_audit.go's crossing audit and moments.go's
// region integral alike. origin and normal come from patchChainWalkOrigin
// and patchChainOrientedNormal for a chain the exact arm admitted, or from
// the chain's own levelToken for one the level arm admitted
// (buildPatchFace's own dispatch).
//
// A curved edge's own effective sweep sense — CCW about +Axis when walked
// forward, CCW about -Axis (so CW about +Axis) when walked backward — may or
// may not match "CCW as plotted in this frame's own (u, v)", since the
// frame's normal sign is a free choice this function's caller is still
// deciding between. Where it matches, the segment's Start/End are the
// coedge's own Start/End, walked forward (TStart 0, TEnd 1): a CircleSeg
// records CCW true, and an ArcSeg's own "swept counter-clockwise from Start
// to End" convention already describes the real arc. Where it does not, the
// SAME two points are recorded with Start and End swapped and the range
// reversed (TStart 1, TEnd 0): LoopRecord's own contract is that the walk
// runs from the point AT TStart to the point AT TEnd, so this keeps the
// walked start/end — and so the chain's own connectivity — unchanged while
// describing the OTHER (complementary) arc between the same two points,
// which is the one the real 3D curve actually is under this frame's chosen
// sign. Swapping the Start/End FIELDS themselves would instead describe the
// same arc read backwards, not the complementary one, which is why the
// fields swap together with the range rather than the range alone.
func patchChainFrameAndSegments(ordered []patchOrientedEdge, edgeCopy map[*Edge]*Edge, origin, normal r3.Vec) (r3.Frame, []CurveSegment, error) {
	frame, err := planeFrameFromNormal(origin, normal)
	if err != nil {
		return r3.Frame{}, nil, err
	}

	segs := make([]CurveSegment, len(ordered))
	for i, oe := range ordered {
		ne := edgeCopy[oe.old]
		start, end := ne.start, ne.end
		if !oe.forward {
			start, end = end, start
		}
		p2Start := patchPoint2(frame, start.position)
		p2End := patchPoint2(frame, end.position)
		switch c := ne.curve.(type) {
		case Line3:
			segs[i] = LineSeg{Start: p2Start, End: p2End, TStart: 0, TEnd: 1}
		case Circle3:
			axis := c.Axis
			if !oe.forward {
				axis = axis.Scale(-1)
			}
			center := patchPoint2(frame, c.Center)
			if axis.Dot(frame.N()) > 0 {
				segs[i] = CircleSeg{Center: center, Radius: c.Radius, CCW: true, TStart: 0, TEnd: 1}
			} else {
				segs[i] = CircleSeg{Center: center, Radius: c.Radius, CCW: false, TStart: 1, TEnd: 0}
			}
		case Arc3:
			axis := c.Axis
			if !oe.forward {
				axis = axis.Scale(-1)
			}
			center := patchPoint2(frame, c.Center)
			if axis.Dot(frame.N()) > 0 {
				segs[i] = ArcSeg{Center: center, Start: p2Start, End: p2End, TStart: 0, TEnd: 1}
			} else {
				segs[i] = ArcSeg{Center: center, Start: p2End, End: p2Start, TStart: 1, TEnd: 0}
			}
		default:
			return r3.Frame{}, nil, fmt.Errorf(`%w: Body.Patch cannot represent curve kind %T`, ErrUnsupported, c)
		}
	}
	return frame, segs, nil
}

// patchPoint2 projects a world position into frame's local (u, v), the same
// reading stitch.go's triangulateStitchFaces takes of a placed vertex.
func patchPoint2(frame r3.Frame, p r3.Vec) Point2 {
	local := frame.ToLocal(p)
	return Point2{U: local.X, V: local.Y}
}

// planeFrameFromNormal builds an orthonormal frame at origin whose normal is
// normal: any vector not parallel to normal is projected into the plane to
// seed the first in-plane axis, and the second is normal's own cross with
// it, which r3.NewFrame's own Gram-Schmidt then only has to normalize
// (CLAUDE.md's rule against hand-rolled coordinate math — every step here is
// an r3.Vec operation, never a raw float computation).
func planeFrameFromNormal(origin, normal r3.Vec) (r3.Frame, error) {
	n, ok := normal.Normalize()
	if !ok {
		return r3.Frame{}, fmt.Errorf(`%w: a Body.Patch chain's plane normal is degenerate`, ErrDegenerate)
	}
	ref := r3.NewVec(1, 0, 0)
	if math.Abs(n.Dot(ref)) > 0.9 {
		ref = r3.NewVec(0, 1, 0)
	}
	u, ok := ref.Sub(n.Scale(ref.Dot(n))).Normalize()
	if !ok {
		return r3.Frame{}, fmt.Errorf(`%w: a Body.Patch chain's plane normal is degenerate`, ErrDegenerate)
	}
	v := n.Cross(u)
	return r3.NewFrame(origin, u, v)
}

// Rule P (docs/surface-design.md §6.4) is the construction-proof gate a
// bodyPatchPayload earns from its own receiver, letting Rule S
// (stitch_flux.go) admit the "walls, then cap, then stitch" flow §6.1
// motivates. A bodyPatchPayload proves its own boundary does not
// self-intersect — payloadProvesSimple's own bodyPatchPayload arm
// (verify.go) — when all three hold, decided by
// bodyPatchPayloadProvesSimple below:
//
//  1. its receiver — the body Body.Patch was called on — itself admits
//     under Rule S, decided by the identical payloadProvesSimple predicate,
//     never a reimplementation of it;
//  2. every new face's chain is a COMPLETE free-edge chain of that
//     receiver's own end, not a proper subset of one;
//  3. every new face's plane is exactly one of the receiver feature's own
//     end planes.
//
// Under those three the patched assembly IS the receiver feature's own
// solid boundary — every chain vertex and edge Body.Patch selected is the
// SAME object the receiver's own build stamped, never a copy or a fit — so
// the receiver's own construction proof carries onto the patched body
// unchanged. Any chain this cannot decide keeps the payload undecided,
// reject-only exactly as every other Rule S arm.
//
// Conditions 2 and 3 are both decided through the LEVEL half of the
// shared-denotation certificate (denotation.go), never by a coordinate or a
// residual: prism_build.go's evalPrismContext stamps every rim vertex and
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
// SECOND Body.Patch call selects from an already-patched body
// (copyPatchFacesUnder does not propagate a level token onto the copies it
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
// Body.Patch-capped surface-extruded tube's rims and nothing wider yet.

// bodyPatchPayloadProvesSimple decides payloadProvesSimple's bodyPatchPayload
// arm — Rule P, this file's own doc comment above. bodies is
// stitchOperandBodies reused verbatim: pp.faces is exactly the shape that
// function already reads an operand's source body from.
func bodyPatchPayloadProvesSimple(ctx context.Context, pp bodyPatchPayload) bool {
	bodies := stitchOperandBodies(pp.faces)
	if len(bodies) != 1 || bodies[0] == nil || bodies[0].payload == nil {
		return false
	}
	receiver := bodies[0]
	if !payloadProvesSimple(ctx, receiver.payload) {
		return false
	}

	// Every receiver free edge, grouped by its own level id — the candidate
	// set condition 2 checks each chain against. An edge with no
	// certificate (id == 0) joins no group: it cannot be one of the
	// receiver's own end planes (denotation.go), and grouping by "no
	// certificate" would prove nothing about which plane it belongs to.
	receiverFreeByLevel := map[levelID][]*Edge{}
	for _, e := range receiver.Edges() {
		if !e.IsFree() || e.level.id == 0 {
			continue
		}
		receiverFreeByLevel[e.level.id] = append(receiverFreeByLevel[e.level.id], e)
	}

	for _, chain := range pp.chains {
		id, ok := patchChainSharedLevel(chain.edges)
		if !ok {
			return false
		}
		if !edgeSetsEqual(receiverFreeByLevel[id], chain.edges) {
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
	id := edges[0].level.id
	if id == 0 {
		return 0, false
	}
	for _, e := range edges {
		if e.level.id != id || e.start.level.id != id || e.end.level.id != id {
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
