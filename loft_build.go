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

	verts []r3.Vec
	tris  [][3]int
	walls int
}

// transform is the accumulated rigid placement.
func (pl loftPayload) transform() r3.Transform { return pl.xform }

// placed is staged until PR 2 (docs/loft-design.md Table D, D7): every loft
// surface is a Plane, and evaluator §8 already states that every v1 surface
// variant maps to itself under an isometry, but re-evaluating the pairing and
// re-running §6's audit under a new placement is PR 2's own work.
func (pl loftPayload) placed(_ context.Context, _ *Document, _ StepRef, _ r3.Transform) (*Body, error) {
	return nil, fmt.Errorf(`%w: this evaluator cannot place a lofted body yet`, ErrUnsupported)
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
	// cell/side parallel tris[:walls]: cell[k] is {loop index i, cell index
	// j}, side[k] is 0 for lower_j and 1 for upper_j.
	cell [][2]int
	side []uint8
	// vIdx/wIdx are, per loop, the vertex-table index of V[i][j] and W[i][j].
	vIdx, wIdx [][]int
}

// assembleLoft lifts every recorded point once, emits the 2*sum(n_i) wall
// triangles in Table B's order and winding, triangulates both caps through
// triangulate.go's existing polygon-with-holes triangulator with capStart's
// triples reversed and capEnd's retained (§5's cap seeding), and orients the
// complete shell once from the signed tetrahedron sum anchored at the placed
// p0 origin (§5's whole-shell rule).
func assembleLoft(ctx context.Context, pairs []loftLoopPair, f0, f1 r3.Frame, plane0 PlaneRecord, xform r3.Transform) (loftAssembly, error) {
	vIdx := make([][]int, len(pairs))
	wIdx := make([][]int, len(pairs))
	var verts []r3.Vec
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return loftAssembly{}, err
		}
		vIdx[i] = make([]int, len(p.v))
		for j, pt := range p.v {
			vIdx[i][j] = len(verts)
			verts = append(verts, xform.Apply(f0.ToWorldUV(pt.U, pt.V)))
		}
		wIdx[i] = make([]int, len(p.w))
		for j, pt := range p.w {
			wIdx[i][j] = len(verts)
			verts = append(verts, xform.Apply(f1.ToWorldUV(pt.U, pt.V)))
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

	anchor := xform.Apply(plane0.Origin)
	if loftOrientationSign(verts, tris, anchor) < 0 {
		for i, t := range tris {
			tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}

	return loftAssembly{
		verts: verts, tris: tris, walls: walls, capStartCount: capStartCount,
		cell: cell, side: side, vIdx: vIdx, wIdx: wIdx,
	}, nil
}

// wrapLoftTriangulationError re-sentinels triangulate.go's cap refusal as
// ErrUnsupported (design O8): the caller's two profiles are each individually
// valid per sketch (S9 authenticates them before evalLoft is ever reached),
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

// loftVertex builds an Exact vertex at a recorded (or lifted-from-recorded)
// coordinate: every loft vertex position comes from Plane.Origin + p.U*Plane.U
// + p.V*Plane.V, the identical single float64 evaluation Extrude already
// performs for a cap vertex (§5), so it carries the same zero-bound standing.
func loftVertex(p r3.Vec) *Vertex {
	return &Vertex{position: p, bound: units.Millimeters(0)}
}

// loftEdgeLength is the proven bound on a straight loft edge's held length:
// the square root's own committed error against the exact rational squared
// length (capblend_contour.go's straightEdgeBound/ratSquaredDistance3), no
// new mechanism.
func loftEdgeLength(a, b r3.Vec) (float64, float64) {
	held := a.Sub(b).Len()
	sq := ratSquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
	return held, straightEdgeBound(held, sq)
}

// loftEdge builds one straight loft edge between two vertex-table indices,
// with the given walked-boundary convexity.
func loftEdge(vertexObjs []*Vertex, positions []r3.Vec, a, b int, convex bool) *Edge {
	held, bound := loftEdgeLength(positions[a], positions[b])
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
// accumulator sums), and its side(i,j,k) role.
func buildLoftWallFace(body *Body, ref StepRef, verts []r3.Vec, tri [3]int, i, j, side int) (*Face, error) {
	surf, err := planeFromTriangle(verts, tri)
	if err != nil {
		return nil, err
	}
	u := xsub(xptOf(verts[tri[1]]), xptOf(verts[tri[0]]))
	v := xsub(xptOf(verts[tri[2]]), xptOf(verts[tri[0]]))
	lo, hi := wallTriangleArea(u, v)
	return &Face{
		surface:   surf,
		origins:   []FeatureRef{{Step: ref, Role: fmt.Sprintf("side(%d,%d,%d)", i, j, side)}},
		body:      body,
		area:      lo,
		areaBound: upRound(hi - lo),
	}, nil
}

// buildLoftTopology builds the B-rep topology from the assembled, globally
// oriented triangle set (docs/loft-design.md §5/§7): real Vertex/Edge/Loop/
// Face objects sharing indices with the assembly's own vertex table. Every
// edge bounds exactly two faces by construction (§5's four edge families:
// bottom rim, top rim, diagonal, rung), and every cap-boundary edge opposes
// its incident wall edge, the standard two-manifold convention.
func buildLoftTopology(ctx context.Context, body *Body, ref StepRef, a loftAssembly, cap0Rat, cap1Rat *big.Rat) (*Face, *Face, []*Face, error) {
	vertexObjs := make([]*Vertex, len(a.verts))
	for i, p := range a.verts {
		vertexObjs[i] = loftVertex(p)
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
			rimBottom[j] = loftEdge(vertexObjs, a.verts, vIdx[j], vIdx[jn], isOuter)
			rimTop[j] = loftEdge(vertexObjs, a.verts, wIdx[j], wIdx[jn], isOuter)
		}
		for j := range n {
			jn := (j + 1) % n
			jp := (j - 1 + n) % n
			rungConvex := junctionConvex(a.verts, lowerTri[i][jp], upperTri[i][j], vIdx[j], wIdx[j])
			rungE[j] = loftEdge(vertexObjs, a.verts, vIdx[j], wIdx[j], rungConvex)
			diagConvex := junctionConvex(a.verts, lowerTri[i][j], upperTri[i][j], vIdx[j], wIdx[jn])
			diagE[j] = loftEdge(vertexObjs, a.verts, vIdx[j], wIdx[jn], diagConvex)
		}

		capStartCo := make([]coedge, n)
		capEndCo := make([]coedge, n)
		for j := range n {
			jn := (j + 1) % n

			lowerFace, err := buildLoftWallFace(body, ref, a.verts, lowerTri[i][j], i, j, 0)
			if err != nil {
				return nil, nil, nil, err
			}
			lowerFace.loops = []*Loop{{outer: true, coedges: []coedge{
				{edge: rimBottom[j], forward: true},
				{edge: rungE[jn], forward: true},
				{edge: diagE[j], forward: false},
			}}}
			walls = append(walls, lowerFace)

			upperFace, err := buildLoftWallFace(body, ref, a.verts, upperTri[i][j], i, j, 1)
			if err != nil {
				return nil, nil, nil, err
			}
			upperFace.loops = []*Loop{{outer: true, coedges: []coedge{
				{edge: diagE[j], forward: true},
				{edge: rimTop[j], forward: false},
				{edge: rungE[j], forward: false},
			}}}
			walls = append(walls, upperFace)

			capStartCo[n-1-j] = coedge{edge: rimBottom[j], forward: false}
			capEndCo[j] = coedge{edge: rimTop[j], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: isOuter, coedges: capStartCo})
		capEndLoops = append(capEndLoops, &Loop{outer: isOuter, coedges: capEndCo})
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
	capStart := &Face{
		surface:   capStartSurf,
		loops:     capStartLoops,
		origins:   []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:      body,
		area:      cap0Val,
		areaBound: rationalFloatError(cap0Rat, cap0Val),
	}
	capEnd := &Face{
		surface:   capEndSurf,
		loops:     capEndLoops,
		origins:   []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:      body,
		area:      cap1Val,
		areaBound: rationalFloatError(cap1Rat, cap1Val),
	}

	return capStart, capEnd, walls, nil
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
func evalLoft(ctx context.Context, d *Document, ref StepRef, pl loftPayload, budget *workBudget) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// One free-form work counter per profile for this whole build (spline
	// design §5.2): increment 1 admits only LineSeg pairs, so nothing here
	// ever actually charges against it, but every walkOf call site still
	// needs one to resolve a segment's kind.
	work0 := newFreeformWork()
	work1 := newFreeformWork()

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
	mass := newLoftMassAccumulator(anchor)
	for k, t := range a.tris {
		mass.add(a.verts[t[0]], a.verts[t[1]], a.verts[t[2]], k < a.walls)
	}
	body.volume = mass.volume()
	centroid, err := mass.centroid()
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
	body.payload = pl
	return body, nil
}
