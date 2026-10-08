package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/patchchain"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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
	// delta is the proven displacement proofbound.RigidRoundAllow charges against this
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
	refs := make([]patchchain.EdgeRef[*Edge, *Vertex], len(edges))
	for i, e := range edges {
		refs[i] = patchchain.EdgeRef[*Edge, *Vertex]{Edge: e, Start: e.start, End: e.end}
	}
	return patchchain.Partition(refs, func(v *Vertex) string { return renderCoord3(v.position) })
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
	if lvl.ID == 0 {
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
	points := make([]patchchain.Vertex, len(verts))
	for i, v := range verts {
		points[i] = patchchain.Vertex{Position: v.position, Bound: v.bound.Base()}
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

// patchOrientedEdge is one chain edge under the orientation decision: the
// receiver's own (read-only) edge, and the sense — forward from Edge.Start
// to Edge.End, or backward — the new patch face traverses it in.
type patchOrientedEdge struct {
	old     *Edge
	forward bool
}

// orientPatchChain reads each edge's adjacent face and delegates the cycle
// walk to internal/patchchain. See docs/surface-design.md §5.2.
func orientPatchChain(chain bodyPatchChain) ([]patchOrientedEdge, error) {
	oriented := make([]patchchain.OrientedEdge[*Edge, *Vertex], len(chain.edges))
	for i, e := range chain.edges {
		if len(e.faces) != 1 {
			return nil, fmt.Errorf(`%w: a Body.Patch chain edge is not free (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		dir, ok := coedgeDirectionFor(e.faces[0], e)
		if !ok {
			return nil, fmt.Errorf(`%w: a Body.Patch chain edge's adjacent face does not use it (%s)`, ErrDegenerate, patchRowFacesDisagree)
		}
		oriented[i] = patchchain.OrientedEdge[*Edge, *Vertex]{Edge: e, Start: e.start, End: e.end, Forward: !dir}
	}
	walk, err := patchchain.Orient(oriented, patchRowFacesDisagree)
	if err != nil {
		return nil, err
	}
	order := make([]patchOrientedEdge, len(walk))
	for i, oe := range walk {
		order[i] = patchOrientedEdge{old: oe.Edge, forward: oe.Forward}
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
				maxInputAbs = max(maxInputAbs, proofbound.VecMaxAbs(ce.edge.start.position), proofbound.VecMaxAbs(ce.edge.end.position))
			}
		}
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(xform.Translation()))
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
	// through proofbound.BoundedAdd — never a re-derived reading — exactly the sum
	// evalStitchContext already runs over its own constituent faces
	// (docs/surface-design.md §6.4/§8), reused here for one operand's own
	// face set plus the faces this call adds to it.
	areaAcc := proofbound.BoundedScalar{}
	for _, f := range allFaces {
		areaAcc = proofbound.BoundedAdd(areaAcc, proofbound.MeasuredScalar(f.area, f.areaBound))
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(areaAcc.Value),
		Exactness: exactnessOf(areaAcc.Bound),
		Bound:     units.SquareMillimeters(areaAcc.Bound),
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
//
// Each copy carries its source face's own axialDelta, hasAxialDelta and
// normalBound, on unstitch.go's copyFaceUnderContext's terms: the copy holds
// the identical surface and tag, so both figures are as true of it as of
// the source. axialDelta widens by delta under a non-identity placement, and
// a placed copy of a face whose normalBound is nonzero refuses with
// [ErrUnsupported], since no dimensionless term bounds the placement's own
// rotation of the tag frame. A revolve face's denoted surface (Face.denoted)
// carries over as its exact image under xform.
func copyPatchFacesUnder(ctx context.Context, srcFaces []*Face, xform r3.Transform, delta float64) ([]*Face, map[*Edge]*Edge, error) {
	if xform != r3.Identity() {
		for _, f := range srcFaces {
			if f.normalBound != 0 {
				return nil, nil, fmt.Errorf(`%w: a placed copy of a face whose normalBound is nonzero has no dimensionless term to bound the placement's own rotation of the tag frame off the true rotation, so this evaluator refuses rather than guess one`, ErrUnsupported)
			}
		}
	}
	newVertByOld := map[*Vertex]*Vertex{}
	vertexFor := func(old *Vertex) (*Vertex, error) {
		if nv, ok := newVertByOld[old]; ok {
			return nv, nil
		}
		p := xform.Apply(old.position)
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf(`%w: a placed patch vertex is not representable`, ErrUnsupported)
		}
		bound := old.bound.Base()
		if delta > 0 {
			bound = proofbound.AbsSumUpper(bound, delta)
		}
		// The CURVE half of the shared-denotation certificate (denotation.go)
		// restates under xform, composing rather than overwriting, exactly
		// as unstitch.go's copyFaceUnderContext does.
		nv := &Vertex{position: p, bound: units.Millimeters(bound), denot: old.denot.Compose(xform)}
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
			lengthBound = proofbound.AbsSumUpper(lengthBound, delta)
		}
		ne := &Edge{
			curve:           curve,
			start:           start,
			end:             end,
			convex:          old.convex,
			length:          old.length,
			lengthBound:     lengthBound,
			lengthUnbounded: old.lengthUnbounded,
			denot:           old.denot.Compose(xform),
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
		axialDelta := f.axialDelta
		if delta > 0 {
			axialDelta = proofbound.AbsSumUpper(axialDelta, delta)
		}
		nf := &Face{
			surface:       surface,
			origins:       append([]FeatureRef(nil), f.origins...),
			area:          f.area,
			areaBound:     f.areaBound,
			reversed:      f.reversed,
			heldPlanar:    f.heldPlanar,
			axialDelta:    axialDelta,
			hasAxialDelta: f.hasAxialDelta,
			normalBound:   f.normalBound,
			denoted:       f.denotedUnder(xform),
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
		normal, err = patchChainLevelNormal(fitted, xform.ApplyDir(chain.level.Normal))
		if err != nil {
			return nil, err
		}
		origin = xform.Apply(chain.level.Origin)
		// A placement's own rounding (proofbound.RigidRoundAllow) displaces the token's
		// origin exactly as it displaces every other placed coordinate this
		// evaluator publishes (copyPatchFacesUnder's own vertex/edge
		// widening), so it folds into the SAME axialDelta the token's own
		// bound already states.
		if delta > 0 {
			axialDelta = proofbound.AbsSumUpper(axialDelta, delta)
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
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: a Body.Patch chain encloses no area`, ErrDegenerate)
	}

	budget := proofbound.NewWorkBudget(ctx)
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
		area:      ig.Area,
		areaBound: ig.AreaBound,
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
	work := freeform.NewFreeformWork()
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work)
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

// patchChainOrientedNormal adapts placed geometry for internal/patchchain's
// normal derivation. See docs/surface-design.md §5.2.
func patchChainOrientedNormal(ordered []patchOrientedEdge, edgeCopy map[*Edge]*Edge) (r3.Vec, error) {
	return patchchain.OrientedNormal(patchGeometryEdges(ordered, edgeCopy))
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

// patchChainLevelNormal adapts the recorded level normal's sign decision.
func patchChainLevelNormal(fitted, tokenNormal r3.Vec) (r3.Vec, error) {
	return patchchain.LevelNormal(fitted, tokenNormal)
}

// patchChainFrameAndSegments adapts placed edges to plane-local records.
// internal/patchchain owns the curved edge sense rule.
func patchChainFrameAndSegments(ordered []patchOrientedEdge, edgeCopy map[*Edge]*Edge, origin, normal r3.Vec) (r3.Frame, []CurveSegment, error) {
	frame, err := planeFrameFromNormal(origin, normal)
	if err != nil {
		return r3.Frame{}, nil, err
	}
	segs, err := patchchain.SegmentsInFrame(frame, patchGeometryEdges(ordered, edgeCopy))
	if err != nil {
		return r3.Frame{}, nil, err
	}
	return frame, segs, nil
}

// patchGeometryEdges adapts the rebuilt topology to the ordered geometry read.
func patchGeometryEdges(ordered []patchOrientedEdge, edgeCopy map[*Edge]*Edge) []patchchain.GeometryEdge {
	out := make([]patchchain.GeometryEdge, len(ordered))
	for i, oe := range ordered {
		ne := edgeCopy[oe.old]
		g := patchchain.GeometryEdge{Start: ne.start.position, End: ne.end.position, Forward: oe.forward}
		switch c := ne.curve.(type) {
		case Line3:
			g.Kind = patchchain.Line
		case Circle3:
			g.Kind, g.Center, g.Axis, g.Radius = patchchain.Circle, c.Center, c.Axis, c.Radius
		case Arc3:
			g.Kind, g.Center, g.Axis = patchchain.Arc, c.Center, c.Axis
		default:
			g.Kind, g.CurveName = patchchain.Unsupported, fmt.Sprintf("%T", c)
		}
		out[i] = g
	}
	return out
}

// planeFrameFromNormal adapts the patch chain's plane-frame construction.
func planeFrameFromNormal(origin, normal r3.Vec) (r3.Frame, error) {
	return patchchain.PlaneFrameFromNormal(origin, normal)
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
		if !e.IsFree() || e.level.ID == 0 {
			continue
		}
		receiverFreeByLevel[e.level.ID] = append(receiverFreeByLevel[e.level.ID], e)
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
	id := edges[0].level.ID
	if id == 0 {
		return 0, false
	}
	for _, e := range edges {
		if e.level.ID != id || e.start.level.ID != id || e.end.level.ID != id {
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
