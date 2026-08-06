package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/loft-design.md PR 1a: the evaluator half of Loft — the
// loftPayload, its Table P pairing, Table S gates S1-S5 (S9-S11 are the public
// entry point's job, docs/loft-design.md §2/§4), the flat-triangle wall
// construction (§5), the wiring of the already-landed §6 audit
// (loft_audit.go) and §8 mass kernel (loft_moments.go), and the four
// measurements. Document.Loft/LoftContext are PR 1b; nothing here is called
// from outside this file's own tests, the same shape #114 (loft_audit.go/
// loft_moments.go) already shipped.
//
// loftPayload.placed and its delta field are §12 PR 2a (Table D, D7): a
// placement re-lifts both records under the composed motion and re-runs
// §5-§8 from scratch — every record-only Table S gate (S1-S8) and S13's
// coordinate-range gate, plus the placement-only S12, while S9-S11 and S4's
// arity half judge the original
// call's own arguments and never re-run (§4) — so delta is the ONE new term
// this PR adds, composed into every vertex, edge length, face area and body
// measurement §8 already derives.

// loftPayload is the evaluator's own record of a lofted body: the two
// authenticated sections, their planes and frames, the per-loop alignment,
// the accumulated rigid placement, and the assembled triangle set the
// construction produced (docs/loft-design.md §5/§7). verts/tris/walls are the
// same globally oriented triangle set §6's audit classified and §8's
// accumulator integrated — tris[:walls] the wall triangles (side(i,j,k)), the
// rest the two caps' own triangulations — kept on the payload so a later
// Tessellate (PR 2) restates it rather than rebuilding it.
type loftPayload struct {
	profile0, profile1 ProfileRecord
	plane0, plane1     PlaneRecord
	frame0, frame1     r3.Frame
	alignment          []int
	xform              r3.Transform

	// delta is the proven displacement of every held vertex from the exact
	// placed image of the recorded sections (docs/loft-design.md §5, §12 PR
	// 2a): zero for an unplaced body — pl.xform == r3.Identity(), an exact
	// struct comparison, never a tolerance — and otherwise
	// bounds.go's rigidRoundAllow, read at the pre-transform lifted point's
	// own magnitude and the composed translation's magnitude. Every
	// measurement this payload publishes composes it.
	delta float64

	verts []r3.Vec
	tris  [][3]int
	walls int
}

// transform is the accumulated rigid placement.
func (pl loftPayload) transform() r3.Transform { return pl.xform }

// placed re-evaluates the same two records under the composed motion
// (docs/loft-design.md §7, §12 PR 2a): it re-lifts every vertex from the
// record under the FULL composed transform rather than moving the held mesh
// incrementally, so delta does not accumulate across repeated placements —
// one rounding, not one per placement, an advantage over
// facetedPayload.placed's move-the-held-mesh path (boolean_body.go). It is a
// re-evaluation path: no moments preflight has run on either record within
// this call, so the build opens two fresh counters, one per record, exactly
// as prismPayload.placed opens its own (extrude.go). §5's whole-shell
// orientation step re-decides the sign from the placed triangle set on its
// own, so a mirror flips `reversed` with no separate winding-flip case
// needed here.
func (pl loftPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	next := pl
	next.xform = composed
	next.verts, next.tris, next.walls = nil, nil, 0
	return evalLoft(ctx, d, ref, next, newWorkBudget(ctx), newFreeformWork(), newFreeformWork())
}

// validateLoftRecords applies docs/loft-design.md Table S rows S1, S2, S4, S3
// and S5, in §4's stated gate order, from the two authenticated records
// alone — no triangle is built. It returns the normalized per-loop alignment
// offsets: a nil alignment becomes every offset 0 (§2).
func validateLoftRecords(p0, p1 ProfileRecord, pl0, pl1 PlaneRecord, alignment []int, work0, work1 *freeformWork) ([]int, error) {
	if len(p0.Holes) != len(p1.Holes) {
		return nil, fmt.Errorf(`%w: the two profiles have %d and %d holes; a loft has no positional pairing for a hole-count mismatch`,
			ErrUnsupported, len(p0.Holes), len(p1.Holes))
	}
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)
	loopCount := len(loops0)

	for i := range loops0 {
		if len(loops0[i].Segments) != len(loops1[i].Segments) {
			return nil, fmt.Errorf(`%w: loop %d has %d segments on the first profile and %d on the second; a loft has no one-to-one pairing for a segment-count mismatch`,
				ErrUnsupported, i, len(loops0[i].Segments), len(loops1[i].Segments))
		}
	}

	offsets := make([]int, loopCount)
	if alignment != nil {
		if len(alignment) != loopCount {
			return nil, fmt.Errorf(`%w: WithLoftAlignment carries %d offsets for %d loops`,
				ErrDegenerate, len(alignment), loopCount)
		}
		for i, off := range alignment {
			n := len(loops0[i].Segments)
			if off < 0 || off >= n {
				return nil, fmt.Errorf(`%w: loop %d's alignment offset %d is outside [0, %d)`,
					ErrDegenerate, i, off, n)
			}
			offsets[i] = off
		}
	}

	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		for j := range n {
			w0, err := walkOf(loops0[i].Segments[j], work0)
			if err != nil {
				return nil, err
			}
			if !w0.isLine() {
				return nil, fmt.Errorf(`%w: loop %d segment %d of the first profile is not a LineSeg; this evaluator rules straight lines only`,
					ErrUnsupported, i, j)
			}
			k := (j + off) % n
			w1, err := walkOf(loops1[i].Segments[k], work1)
			if err != nil {
				return nil, err
			}
			if !w1.isLine() {
				return nil, fmt.Errorf(`%w: loop %d segment %d of the second profile is not a LineSeg; this evaluator rules straight lines only`,
					ErrUnsupported, i, k)
			}
		}
	}

	if loftPlanesCoincide(pl0, pl1) {
		return nil, fmt.Errorf(`%w: the two profiles lie in the same geometric plane; the loft has zero volume by construction`, ErrDegenerate)
	}

	return offsets, nil
}

// loftPlanesCoincide decides S5 over exact rationals on the recorded U/V/
// Origin floats (boolean_exact.go's xptOf/xcross/xdot, the take-the-floats-
// exactly discipline): the two planes coincide when their normals (U×V) are
// exactly parallel and the displacement between their origins lies in that
// plane. A tolerance here would refuse a legitimately thin loft, and the
// existence claim S5 makes is a structural zero volume, not a small one.
func loftPlanesCoincide(a, b PlaneRecord) bool {
	na := xcross(xptOf(a.U), xptOf(a.V))
	nb := xcross(xptOf(b.U), xptOf(b.V))
	cr := xcross(na, nb)
	if cr.x.Sign() != 0 || cr.y.Sign() != 0 || cr.z.Sign() != 0 {
		return false
	}
	d := xsub(xptOf(b.Origin), xptOf(a.Origin))
	return xdot(na, d).Sign() == 0
}

// loftLoopPair is Table P's correspondence for one loop: the two walk-ordered
// endpoint lists, v from loop0's own segment order and w from loop1's,
// already rotated by that loop's own alignment offset (P4).
type loftLoopPair struct {
	v, w []Point2
}

// loftPairings resolves Table P into one flat correspondence per loop. P1
// pairs by position in Holes, never by area or proximity; P6 is satisfied by
// construction because each list is read in its own loop's own recorded walk
// order and nothing reinterprets it.
func loftPairings(p0, p1 ProfileRecord, offsets []int, work0, work1 *freeformWork) ([]loftLoopPair, error) {
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)
	pairs := make([]loftLoopPair, len(loops0))
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		v := make([]Point2, n)
		w := make([]Point2, n)
		for j := range n {
			w0, err := walkOf(loops0[i].Segments[j], work0)
			if err != nil {
				return nil, err
			}
			v[j] = Point2{U: w0.startU, V: w0.startV}
			k := (j + off) % n
			w1, err := walkOf(loops1[i].Segments[k], work1)
			if err != nil {
				return nil, err
			}
			w[j] = Point2{U: w1.startU, V: w1.startV}
		}
		pairs[i] = loftLoopPair{v: v, w: w}
	}
	return pairs, nil
}

// loftAssembly is the built triangle set plus the index bookkeeping the
// topology needs.
type loftAssembly struct {
	// verts is the shared vertex table; tris is the complete, globally
	// oriented triangle set. tris[:walls] are the wall triangles (Table B's
	// side(i,j,k)); tris[walls:walls+capStartCount] are capStart's own
	// triangles; the rest are capEnd's.
	verts         []r3.Vec
	tris          [][3]int
	walls         int
	capStartCount int
	// reversed records whether §5's whole-shell orientation step flipped
	// every triangle's winding. buildLoftTopology fixes each face's directed
	// boundary from the LOCAL (pre-flip) index convention, so it must reverse
	// every walk it emits by exactly this flag or publish loops that run the
	// material on the wrong side of their own face normal.
	reversed bool
	// cell/side parallel tris[:walls]: cell[k] is {loop index i, cell index
	// j}, side[k] is 0 for lower_j and 1 for upper_j.
	cell [][2]int
	side []uint8
	// vIdx/wIdx are, per loop, the vertex-table index of V[i][j] and W[i][j].
	vIdx, wIdx [][]int
	// delta is the proven displacement every held vertex carries from the
	// exact placed image of the recorded sections (docs/loft-design.md §5,
	// §12 PR 2a) — zero exactly when xform is r3.Identity().
	delta float64
}

// assembleLoft lifts every recorded point once, emits the 2*sum(n_i) wall
// triangles in Table B's order and winding, triangulates both caps through
// triangulate.go's existing polygon-with-holes triangulator with capStart's
// triples reversed and capEnd's retained (§5's cap seeding), and orients the
// complete shell once from the signed tetrahedron sum anchored at the placed
// p0 origin (§5's whole-shell rule). It also owns Table S row S13: every
// placed coordinate it emits, the anchor among them, is proven finite before
// any of them is lifted into an exact rational.
func assembleLoft(ctx context.Context, pairs []loftLoopPair, f0, f1 r3.Frame, plane0 PlaneRecord, xform r3.Transform) (loftAssembly, error) {
	// S13, decided before the first coordinate is lifted into an exact
	// rational: the orientation anchor is the first point loftOrientationSign
	// hands to xptOf, so its own finiteness is the gate's first question.
	anchor := xform.Apply(plane0.Origin)
	if !finiteVec(anchor) {
		return loftAssembly{}, errLoftPointUnrepresentable("placed plane origin")
	}

	vIdx := make([][]int, len(pairs))
	wIdx := make([][]int, len(pairs))
	var verts []r3.Vec
	// maxInputAbs tracks the largest |coordinate| over the frame-lifted,
	// PRE-transform points — the magnitude bounds.go's rigidRoundAllow reads
	// the rounding at, never the placed result's (docs/loft-design.md §5,
	// §12 PR 2a).
	maxInputAbs := 0.0
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return loftAssembly{}, err
		}
		vIdx[i] = make([]int, len(p.v))
		for j, pt := range p.v {
			vIdx[i][j] = len(verts)
			lifted := f0.ToWorldUV(pt.U, pt.V)
			maxInputAbs = max(maxInputAbs, vecMaxAbs(lifted))
			placed := xform.Apply(lifted)
			if !finiteVec(placed) {
				return loftAssembly{}, errLoftPointUnrepresentable(fmt.Sprintf("placed vertex %d of loop %d on the first profile", j, i))
			}
			verts = append(verts, placed)
		}
		wIdx[i] = make([]int, len(p.w))
		for j, pt := range p.w {
			wIdx[i][j] = len(verts)
			lifted := f1.ToWorldUV(pt.U, pt.V)
			maxInputAbs = max(maxInputAbs, vecMaxAbs(lifted))
			placed := xform.Apply(lifted)
			if !finiteVec(placed) {
				return loftAssembly{}, errLoftPointUnrepresentable(fmt.Sprintf("placed vertex %d of loop %d on the second profile", j, i))
			}
			verts = append(verts, placed)
		}
	}

	var tris [][3]int
	var cell [][2]int
	var side []uint8
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return loftAssembly{}, err
		}
		n := len(p.v)
		for j := range n {
			jn := (j + 1) % n
			vj, vjn := vIdx[i][j], vIdx[i][jn]
			wj, wjn := wIdx[i][j], wIdx[i][jn]
			tris = append(tris, [3]int{vj, vjn, wjn})
			cell = append(cell, [2]int{i, j})
			side = append(side, 0)
			tris = append(tris, [3]int{vj, wjn, wj})
			cell = append(cell, [2]int{i, j})
			side = append(side, 1)
		}
	}
	walls := len(tris)

	// Both caps' own triangulation, over each profile's own (u, v) points
	// and loop index arrays — a fresh index space, mapped back to the shared
	// vertex table as each triangle comes back.
	var pts0, pts1 []Point2
	var loopIdx0, loopIdx1 [][]int
	var pts0ToV, pts1ToV []int
	for i, p := range pairs {
		idx0 := make([]int, len(p.v))
		for j, pt := range p.v {
			idx0[j] = len(pts0)
			pts0 = append(pts0, pt)
			pts0ToV = append(pts0ToV, vIdx[i][j])
		}
		loopIdx0 = append(loopIdx0, idx0)

		idx1 := make([]int, len(p.w))
		for j, pt := range p.w {
			idx1[j] = len(pts1)
			pts1 = append(pts1, pt)
			pts1ToV = append(pts1ToV, wIdx[i][j])
		}
		loopIdx1 = append(loopIdx1, idx1)
	}

	tris0, err := triangulate2DContext(ctx, pts0, loopIdx0)
	if err != nil {
		return loftAssembly{}, wrapLoftTriangulationError(err)
	}
	tris1, err := triangulate2DContext(ctx, pts1, loopIdx1)
	if err != nil {
		return loftAssembly{}, wrapLoftTriangulationError(err)
	}

	// capStart reverses each p0 triple (swap 2nd and 3rd); capEnd retains
	// p1's own triples (§5's cap seeding).
	for _, t := range tris0 {
		tris = append(tris, [3]int{pts0ToV[t[0]], pts0ToV[t[2]], pts0ToV[t[1]]})
	}
	capStartCount := len(tris0)
	for _, t := range tris1 {
		tris = append(tris, [3]int{pts1ToV[t[0]], pts1ToV[t[1]], pts1ToV[t[2]]})
	}

	reversed := loftOrientationSign(verts, tris, anchor) < 0
	if reversed {
		for i, t := range tris {
			tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}

	// delta is zero exactly when xform is the identity transform — an exact
	// struct comparison, never a tolerance. This fast path is REQUIRED:
	// without it, every directly-built (unplaced) loft would lose the Exact
	// readings §8/§12 PR 1 publishes (docs/loft-design.md §5, §12 PR 2a).
	delta := 0.0
	if xform != r3.Identity() {
		delta = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}

	return loftAssembly{
		verts: verts, tris: tris, walls: walls, capStartCount: capStartCount,
		reversed: reversed, cell: cell, side: side, vIdx: vIdx, wIdx: wIdx,
		delta: delta,
	}, nil
}

// errLoftPointUnrepresentable is docs/loft-design.md Table S row S13: a
// coordinate this build emits — a recorded section point lifted through its
// own frame and carried by the composed placement, or the orientation anchor
// — runs past the representable float64 range.
//
// The sentinel is ErrUnsupported and never ErrNotFinite. Every INPUT is
// finite: both records' coordinates cleared the seam gates, the plane origins
// are recorded floats, and r3 validates a Transform's own composed
// translation before it ever reaches this evaluator. What runs off float64 is
// decad's OWN evaluation of the lift, and the body EXISTS — it is the rigid
// image of a body this evaluator already built — so modify §1's existence
// test reads "a body this evaluator cannot build". spline_length.go's R15 and
// spline_fit.go's R16 draw the identical line for a finite input whose
// derived magnitude runs off float64; errors.go scopes ErrNotFinite to a
// non-finite PARAMETER or a derived non-finite MEASUREMENT, and
// validateLoftBodyMeasurements already owns that second case.
//
// The gate runs BEFORE the first exact-rational lift, never after it:
// loftOrientationSign lifts the anchor and every vertex through xptOf, whose
// mustRatOf PANICS on a non-finite float, so a check placed any later is a
// panic out of a public method rather than a returned error.
func errLoftPointUnrepresentable(what string) error {
	return fmt.Errorf(`%w: the loft's %s runs past the representable float64 range`, ErrUnsupported, what)
}

// wrapLoftTriangulationError re-sentinels triangulate.go's cap refusal as
// ErrUnsupported (design O8): the caller's two profiles are each individually
// valid per sketch (S9 authenticated them at the original Document.Loft
// call, before any record reached evalLoft; a placement rebuilds from those
// same authenticated records and re-runs no seam gate, §4),
// so a triangulation refusal here is this evaluator's own triangulator
// failing to state the body, never a claim that no such body exists — modify
// §1's existence test applied verbatim. Cancellation is never relabeled.
func wrapLoftTriangulationError(err error) error {
	if err == nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	return fmt.Errorf(`%w: the loft cap triangulator could not state this profile: %s`, ErrUnsupported, err)
}

// loftOrientationSign is the sign of the signed tetrahedron sum §8 defines,
// over the complete triangle set anchored at anchor — the same identity §5's
// whole-shell orientation rule reads, computed once directly over exact
// rationals rather than through the full loftMassAccumulator (which also
// folds in the area/bounds bookkeeping this sign check does not need).
func loftOrientationSign(verts []r3.Vec, tris [][3]int, anchor r3.Vec) int {
	xa := xptOf(anchor)
	sum := new(big.Rat)
	for _, t := range tris {
		a := xsub(xptOf(verts[t[0]]), xa)
		b := xsub(xptOf(verts[t[1]]), xa)
		c := xsub(xptOf(verts[t[2]]), xa)
		sum.Add(sum, xdot(a, xcross(b, c)))
	}
	return sum.Sign()
}

// loftVertex builds a vertex at a recorded (or lifted-from-recorded)
// coordinate: every loft vertex position comes from Plane.Origin + p.U*Plane.U
// + p.V*Plane.V, the identical single float64 evaluation Extrude already
// performs for a cap vertex (§5), so an unplaced vertex (delta 0) carries the
// same zero-bound standing; a placed vertex carries the payload's own
// delta (§12 PR 2a).
func loftVertex(p r3.Vec, delta float64) *Vertex {
	return &Vertex{position: p, bound: units.Millimeters(delta)}
}

// loftEdgeLength is the proven bound on a straight loft edge's held length:
// the square root's own committed error against the exact rational squared
// length (capblend_contour.go's straightEdgeBound/ratSquaredDistance3), no
// new mechanism for an unplaced edge. A placed edge (delta > 0, §12 PR 2a)
// composes that with bounds.go's chainLengthBound(1, delta, held) — both
// endpoints displaced by delta is exactly that helper's own one-chord case —
// through absSumUpper.
func loftEdgeLength(a, b r3.Vec, delta float64) (float64, float64) {
	held := a.Sub(b).Len()
	sq := ratSquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
	bound := straightEdgeBound(held, sq)
	if delta > 0 {
		bound = absSumUpper(bound, chainLengthBound(1, delta, held))
	}
	return held, bound
}

// loftEdge builds one straight loft edge between two vertex-table indices,
// with the given walked-boundary convexity.
func loftEdge(vertexObjs []*Vertex, positions []r3.Vec, a, b int, convex bool, delta float64) *Edge {
	held, bound := loftEdgeLength(positions[a], positions[b], delta)
	return &Edge{curve: Line3{}, start: vertexObjs[a], end: vertexObjs[b], convex: convex, length: held, lengthBound: bound}
}

// junctionApex returns tri's one vertex index that is not in the shared pair
// (a, b) — the OTHER incident triangle's own apex, §5's D.
func junctionApex(tri [3]int, a, b int) int {
	for _, v := range tri {
		if v != a && v != b {
			return v
		}
	}
	return tri[0]
}

// junctionConvex decides a rung or diagonal edge's convexity: orientSign(A,
// B, C, D) < 0, where (A, B, C) is primary's own outward-wound vertex order
// and D is other's apex — design O3, pinned against the box fixture: a
// standard box's vertical edge (a rung) is a genuine convex corner, and this
// is the sign that reads it as one. A zero result is a decided non-convex
// (flat) edge: docs/loft-design.md §5's rule for a flat rung or diagonal.
func junctionConvex(verts []r3.Vec, primary, other [3]int, a, b int) bool {
	apex := junctionApex(other, a, b)
	return orientSign(verts[primary[0]], verts[primary[1]], verts[primary[2]], verts[apex]) < 0
}

// planeFromTriangle builds a face's Plane surface directly from one of its
// own (already outward-oriented) triangles: origin at its first vertex, U and
// V its two edge vectors. r3.NewFrame orthonormalizes them (Gram-Schmidt in
// effect), and the resulting normal U×V is (B-A)x(C-A) up to positive
// scaling — the outward normal of an outward-wound triangle, so the face's
// `reversed` flag stays false (§5's "Exact by construction ... reversed stays
// false").
func planeFromTriangle(verts []r3.Vec, tri [3]int) (Plane, error) {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	f, err := r3.NewFrame(a, b.Sub(a), c.Sub(a))
	if err != nil {
		return Plane{}, fmt.Errorf(`%w: a loft triangle has no plane: %s`, ErrDegenerate, err)
	}
	return Plane{Frame: f}, nil
}

// buildLoftWallFace builds one wall triangle's Face (§7's lower/upper wall
// triangle row): its own Plane, its own proven area bracket
// (loft_moments.go's wallTriangleArea, the identical bracket the mass
// accumulator sums), and its side(i,j,k) role. A placed triangle (delta > 0,
// §12 PR 2a) widens that bracket by bounds.go's perturbedTriangleAreaAllow,
// the same per-triangle correction the mass accumulator sums into Area's own
// bound.
func buildLoftWallFace(body *Body, ref StepRef, verts []r3.Vec, tri [3]int, i, j, side int, delta float64) (*Face, error) {
	surf, err := planeFromTriangle(verts, tri)
	if err != nil {
		return nil, err
	}
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	u := xsub(xptOf(b), xptOf(a))
	v := xsub(xptOf(c), xptOf(a))
	lo, hi := wallTriangleArea(u, v)
	areaBound := upRound(hi - lo)
	if delta > 0 {
		areaBound = absSumUpper(areaBound, perturbedTriangleAreaAllow(a, b, c, delta))
	}
	return &Face{
		surface:   surf,
		origins:   []FeatureRef{{Step: ref, Role: fmt.Sprintf("side(%d,%d,%d)", i, j, side)}},
		body:      body,
		area:      lo,
		areaBound: areaBound,
	}, nil
}

// loftLoopCoedges carries §5's whole-shell reversal into one face's directed
// boundary. Every walk buildLoftTopology emits is written from the LOCAL
// vertex order of §5's construction table, which is the order the triangles
// had BEFORE the whole-shell step; a face's Plane, by contrast, is rebuilt
// from its own
// already-flipped triple (planeFromTriangle), so on a reversed shell the two
// disagree and the published boundary — Loop.CoEdges, CoEdge.Start/End/
// IsForward — walks the material on the RIGHT of the face's own outward
// normal, the opposite of decad's material-on-the-left convention.
//
// Reversing a walk is reversing its coedge order and negating each use's
// sense; nothing but the direction changes. The edge identities and their
// count are untouched, so every edge still bounds exactly the same two faces
// and Loop.Edges' undirected view is merely re-ordered.
func loftLoopCoedges(co []coedge, reversed bool) []coedge {
	if !reversed {
		return co
	}
	out := make([]coedge, len(co))
	for i, ce := range co {
		out[len(co)-1-i] = coedge{edge: ce.edge, forward: !ce.forward}
	}
	return out
}

// buildLoftTopology builds the B-rep topology from the assembled, globally
// oriented triangle set (docs/loft-design.md §5/§7): real Vertex/Edge/Loop/
// Face objects sharing indices with the assembly's own vertex table. Every
// edge bounds exactly two faces by construction (§5's four edge families:
// bottom rim, top rim, diagonal, rung), and every cap-boundary edge opposes
// its incident wall edge, the standard two-manifold convention.
//
// Every loop this builds is stated in §5's LOCAL vertex order and then passed
// through loftLoopCoedges, which is what carries the assembly's own
// whole-shell reversal into the directed boundary each face publishes. A walk
// emitted without it agrees with its face's Plane on one axial spelling of a
// section pair and opposes it on the mirror.
func buildLoftTopology(ctx context.Context, body *Body, ref StepRef, a loftAssembly, cap0Rat, cap1Rat *big.Rat) (*Face, *Face, []*Face, error) {
	vertexObjs := make([]*Vertex, len(a.verts))
	for i, p := range a.verts {
		vertexObjs[i] = loftVertex(p, a.delta)
	}

	loopCount := len(a.vIdx)
	lowerTri := make([][][3]int, loopCount)
	upperTri := make([][][3]int, loopCount)
	for i := range a.vIdx {
		lowerTri[i] = make([][3]int, len(a.vIdx[i]))
		upperTri[i] = make([][3]int, len(a.vIdx[i]))
	}
	for k := range a.walls {
		i, j := a.cell[k][0], a.cell[k][1]
		if a.side[k] == 0 {
			lowerTri[i][j] = a.tris[k]
		} else {
			upperTri[i][j] = a.tris[k]
		}
	}

	var walls []*Face
	var capStartLoops, capEndLoops []*Loop
	for i := range loopCount {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		n := len(a.vIdx[i])
		isOuter := i == 0
		vIdx, wIdx := a.vIdx[i], a.wIdx[i]

		rimBottom := make([]*Edge, n)
		rimTop := make([]*Edge, n)
		diagE := make([]*Edge, n)
		rungE := make([]*Edge, n)
		for j := range n {
			jn := (j + 1) % n
			rimBottom[j] = loftEdge(vertexObjs, a.verts, vIdx[j], vIdx[jn], isOuter, a.delta)
			rimTop[j] = loftEdge(vertexObjs, a.verts, wIdx[j], wIdx[jn], isOuter, a.delta)
		}
		for j := range n {
			jn := (j + 1) % n
			jp := (j - 1 + n) % n
			rungConvex := junctionConvex(a.verts, lowerTri[i][jp], upperTri[i][j], vIdx[j], wIdx[j])
			rungE[j] = loftEdge(vertexObjs, a.verts, vIdx[j], wIdx[j], rungConvex, a.delta)
			diagConvex := junctionConvex(a.verts, lowerTri[i][j], upperTri[i][j], vIdx[j], wIdx[jn])
			diagE[j] = loftEdge(vertexObjs, a.verts, vIdx[j], wIdx[jn], diagConvex, a.delta)
		}

		capStartCo := make([]coedge, n)
		capEndCo := make([]coedge, n)
		for j := range n {
			jn := (j + 1) % n

			lowerFace, err := buildLoftWallFace(body, ref, a.verts, lowerTri[i][j], i, j, 0, a.delta)
			if err != nil {
				return nil, nil, nil, err
			}
			lowerFace.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: rimBottom[j], forward: true},
				{edge: rungE[jn], forward: true},
				{edge: diagE[j], forward: false},
			}, a.reversed)}}
			walls = append(walls, lowerFace)

			upperFace, err := buildLoftWallFace(body, ref, a.verts, upperTri[i][j], i, j, 1, a.delta)
			if err != nil {
				return nil, nil, nil, err
			}
			upperFace.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: diagE[j], forward: true},
				{edge: rimTop[j], forward: false},
				{edge: rungE[j], forward: false},
			}, a.reversed)}}
			walls = append(walls, upperFace)

			capStartCo[n-1-j] = coedge{edge: rimBottom[j], forward: false}
			capEndCo[j] = coedge{edge: rimTop[j], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: isOuter, coedges: loftLoopCoedges(capStartCo, a.reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: isOuter, coedges: loftLoopCoedges(capEndCo, a.reversed)})
	}

	capStartSurf, err := planeFromTriangle(a.verts, a.tris[a.walls])
	if err != nil {
		return nil, nil, nil, err
	}
	capEndSurf, err := planeFromTriangle(a.verts, a.tris[a.walls+a.capStartCount])
	if err != nil {
		return nil, nil, nil, err
	}
	cap0Val, _ := cap0Rat.Float64()
	cap1Val, _ := cap1Rat.Float64()
	capStartBound := rationalFloatError(cap0Rat, cap0Val)
	capEndBound := rationalFloatError(cap1Rat, cap1Val)
	if a.delta > 0 {
		capStartTris := a.tris[a.walls : a.walls+a.capStartCount]
		capEndTris := a.tris[a.walls+a.capStartCount:]
		capStartBound = absSumUpper(capStartBound, capTriangleAreaAllow(a.verts, capStartTris, a.delta))
		capEndBound = absSumUpper(capEndBound, capTriangleAreaAllow(a.verts, capEndTris, a.delta))
	}
	capStart := &Face{
		surface:   capStartSurf,
		loops:     capStartLoops,
		origins:   []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:      body,
		area:      cap0Val,
		areaBound: capStartBound,
	}
	capEnd := &Face{
		surface:   capEndSurf,
		loops:     capEndLoops,
		origins:   []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:      body,
		area:      cap1Val,
		areaBound: capEndBound,
	}

	return capStart, capEnd, walls, nil
}

// capTriangleAreaAllow sums bounds.go's perturbedTriangleAreaAllow over one
// cap's own triangulation triangles (docs/loft-design.md §12 PR 2a) — the
// extra area a placement's delta can add to a cap's exact rational area
// (moments.go), summed the same way loft_moments.go's accumulator sums it
// for the wall triangles.
func capTriangleAreaAllow(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	total := 0.0
	for _, t := range tris {
		total = upRound(total + perturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], delta))
	}
	return total
}

// exactRegionArea returns a LineSeg-only region's exact rational area — the
// same rational moments.go's own region-level accumulator holds, never a
// float re-derivation (docs/loft-design.md §8: "Each cap's own contribution
// reuses moments.go unchanged"). S3 admits only LineSeg pairs, so the
// accumulator is always complete here; an incomplete one is an evaluator
// invariant break rather than a caller-reachable refusal.
func exactRegionArea(p ProfileRecord, work *freeformWork) (*big.Rat, error) {
	ig, err := p.evaluatorIntegrals(momentAreaOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.exactDead || !ig.exact.complete() {
		return nil, fmt.Errorf(`%w: the loft cap's area has no exact rational`, ErrUnsupported)
	}
	return ig.exact.area, nil
}

// validateLoftBodyMeasurements is evalLoft's own finiteness gate (design O2).
// Volume, Centroid and Bounds must be fully finite, exactly as every other
// analytic payload's validateAnalyticBodyMeasurements requires — but Area's
// Bound is deliberately NOT checked: §8 requires a saturated Area bound to
// publish +Inf as a proof term (a wall set whose areas approach float64's own
// ceiling), and checking it here would refuse the very body that fixture
// constructs. Only Area's Value is checked for finiteness.
func validateLoftBodyMeasurements(body *Body) error {
	if !finiteMeasurementValues(body.volume.Value.Base(), body.volume.Bound.Base()) {
		return fmt.Errorf(`%w: the loft's volume measurement is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(body.area.Value.Base()) {
		return fmt.Errorf(`%w: the loft's area value is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(body.centroid.Value.X, body.centroid.Value.Y, body.centroid.Value.Z, body.centroid.Bound.Base()) {
		return fmt.Errorf(`%w: the loft's centroid measurement is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(
		body.bounds.Min.X, body.bounds.Min.Y, body.bounds.Min.Z,
		body.bounds.Max.X, body.bounds.Max.Y, body.bounds.Max.Z,
		body.bounds.Bound.Base(),
	) {
		return fmt.Errorf(`%w: the loft's bounds measurement is not finite`, ErrNotFinite)
	}
	return nil
}

// evalLoft builds the lofted body from the payload's own records
// (docs/loft-design.md §5-§8): pairing, assembly, the §6 audit, topology, and
// the four measurements — all four published at build, never staged (§12).
// budget is shared with the rest of the pre-commit cancellation path exactly
// as modify §5's audits already share one; the caller (LoftContext, PR 1b)
// mints it once for the whole build.
//
// work0/work1 are the per-profile free-form work counters (spline design
// §5.2): the R7 ceiling is one record's across a whole OPERATION, and
// LoftContext also runs falsifyRecordedArea on both records before evalLoft
// is called, so those counters — not two fresh ones minted here — must be
// the ones every walkOf call site in this build spends against. Increment 1
// admits only LineSeg pairs, so nothing here ever actually charges, but the
// counters are still threaded through so PR 3's curved correspondence does
// not silently open a second ceiling per record.
func evalLoft(ctx context.Context, d *Document, ref StepRef, pl loftPayload, budget *workBudget, work0, work1 *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	offsets, err := validateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, work0, work1)
	if err != nil {
		return nil, err
	}

	pairs, err := loftPairings(pl.profile0, pl.profile1, offsets, work0, work1)
	if err != nil {
		return nil, err
	}

	a, err := assembleLoft(ctx, pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform)
	if err != nil {
		return nil, err
	}

	if err := loftCrossingAudit(budget, a.verts, a.tris); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cap0Rat, err := exactRegionArea(pl.profile0, work0)
	if err != nil {
		return nil, err
	}
	cap1Rat, err := exactRegionArea(pl.profile1, work1)
	if err != nil {
		return nil, err
	}

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

	capStart, capEnd, walls, err := buildLoftTopology(ctx, body, ref, a, cap0Rat, cap1Rat)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	faces := append([]*Face{capStart, capEnd}, walls...)
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}

	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	anchor := pl.xform.Apply(pl.plane0.Origin)
	mass := newLoftMassAccumulator(anchor, a.delta)
	for k, t := range a.tris {
		mass.add(a.verts[t[0]], a.verts[t[1]], a.verts[t[2]], k < a.walls)
	}
	body.volume = mass.volume(a.verts, a.tris)
	centroid, err := mass.centroid(a.verts, a.tris)
	if err != nil {
		return nil, err
	}
	body.centroid = centroid
	bounds, ok := mass.bounds()
	if !ok {
		return nil, fmt.Errorf(`%w: the loft has no vertices to bound`, ErrDegenerate)
	}
	body.bounds = bounds
	body.area = mass.area(cap0Rat, cap1Rat)

	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}

	pl.verts, pl.tris, pl.walls = a.verts, a.tris, a.walls
	pl.delta = a.delta
	body.payload = pl
	return body, nil
}
