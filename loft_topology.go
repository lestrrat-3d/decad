package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file assembles a loft's paired stations into the flat-triangle solid
// the payload holds, and builds the Body topology over it.
//
// assembleLoft produces the vertex and triangle sets — two triangles per wall
// cell, plus a triangulated cap at each end — and buildLoftTopology turns
// them into faces, loops, coedges and edges. Every face here is planar
// BECAUSE the triangles are the thing actually built: the curved solid the
// paired sections denote is reached through the payload's proven
// displacement, never by publishing a ruled patch nothing constructed. See
// docs/loft-design.md §5.1 and §7.

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
	// pts0/pts1 and loopIdx0/loopIdx1 are the plane-local (U, V) points and
	// per-loop index arrays this construction ACTUALLY triangulated each cap
	// from (§5's cap seeding) — the same arrays capPolygonAreaRat sums, so
	// the published cap area can never disagree with the built cap
	// triangles (docs/loft-design.md §8).
	pts0, pts1         []Point2
	loopIdx0, loopIdx1 [][]int
	// delta is the proven displacement every held vertex carries from the
	// exact placed image of the recorded sections (docs/loft-design.md §5,
	// §12 PR 2a) — absSumUpper(stationRound, placeAllow): zero exactly when
	// xform is r3.Identity() AND every station publishes a zero stationRound
	// (a10-plan.md Part 3 PR 6), never zero merely because the body is
	// unplaced, since a curved pair with interior COMPUTED stations commits
	// its own rounding whether or not the body is later placed. Being a
	// recorded endpoint is not that condition: an untrimmed ArcSeg's t == 1
	// end is recorded verbatim and still carries the arc-end radial residual
	// (arcNaturalEndRadialUpper).
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
//
// stationRound is loftPairings' own accumulated Table S row S14 term
// (a10-plan.md Part 3 PR 6): the proven rounding every COMPUTED circular
// station commits, composed into delta beside the placement's own
// rigidRoundAllow term.
func assembleLoft(ctx context.Context, pairs []loftLoopPair, f0, f1 r3.Frame, plane0 PlaneRecord, xform r3.Transform, stationRound float64) (loftAssembly, error) {
	// S13, decided before the first coordinate is lifted into an exact
	// rational: the orientation anchor is the first point meshOrientationSign
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

	reversed := meshOrientationSign(verts, tris, anchor) < 0
	if reversed {
		for i, t := range tris {
			tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}

	// placeAllow is zero exactly when xform is the identity transform — an
	// exact struct comparison, never a tolerance. This fast path is
	// REQUIRED: without it, every directly-built (unplaced) LineSeg-only loft
	// whose every station is PINNED would lose the Exact readings §8/§12 PR 1
	// publishes (docs/loft-design.md §5, §12 PR 2a). delta =
	// absSumUpper(stationRound, placeAllow) (a10-plan.md Part 3 PR 6) is NO
	// LONGER zero exactly when xform is the identity: a curved pair with
	// interior computed stations carries a positive stationRound whether or
	// not the body is placed, so does a LineSeg pair holding a station at
	// a TRIMMED parameter (loftLineCellStations), and so does an untrimmed
	// ArcSeg pair whose recorded End sits off its own Start's radius
	// (arcNaturalEndRadialUpper). So the fast path this
	// comment used to state is now placeAllow's own, while stationRound is
	// absSumUpper's other, independent leg — absSumUpper(0, 0) is exactly 0.0
	// (upRound never nudges a non-positive value), which is what keeps the
	// delta of an unplaced LineSeg-only loft whose every station is PINNED
	// bit-identical to before.
	placeAllow := 0.0
	if xform != r3.Identity() {
		placeAllow = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}
	delta := absSumUpper(stationRound, placeAllow)

	return loftAssembly{
		verts: verts, tris: tris, walls: walls, capStartCount: capStartCount,
		reversed: reversed, cell: cell, side: side, vIdx: vIdx, wIdx: wIdx,
		pts0: pts0, pts1: pts1, loopIdx0: loopIdx0, loopIdx1: loopIdx1,
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
// meshOrientationSign lifts the anchor and every vertex through xptOf, whose
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

// meshOrientationSign is the sign of the signed tetrahedron sum
// docs/loft-design.md §8 defines, over the complete triangle set anchored at
// anchor — the same identity §5's whole-shell orientation rule reads, computed
// once directly over exact rationals rather than through the full
// loftMassAccumulator (which also folds in the area/bounds bookkeeping this
// sign check does not need). It reads nothing loft-specific, so
// docs/tessellation-design.md §4's signed-volume audit runs on it too, for
// every payload class that assembles its own triangle set.
func meshOrientationSign(verts []r3.Vec, tris [][3]int, anchor r3.Vec) int {
	xa := xptOf(anchor)
	sum := new(big.Rat)
	for _, t := range tris {
		a := xsub(xptOf(verts[t[0]]), xa)
		b := xsub(xptOf(verts[t[1]]), xa)
		c := xsub(xptOf(verts[t[2]]), xa)
		sum.Add(sum, xdotRat(a, xcross(b, c)))
	}
	return sum.Sign()
}

// loftVertex builds a vertex at a recorded (or lifted-from-recorded)
// coordinate: every loft vertex position comes from Plane.Origin + p.U*Plane.U
// + p.V*Plane.V, the identical single float64 evaluation Extrude already
// performs for a cap vertex (§5), so a vertex of a build whose delta is zero
// carries the same zero-bound standing; any other vertex carries the payload's
// own delta (§12 PR 2a). Zero delta is NOT the same claim as unplaced: an
// unplaced build still carries a positive delta wherever a station was
// COMPUTED rather than pinned (loftPayload's own delta doc comment).
func loftVertex(p r3.Vec, delta float64) *Vertex {
	return &Vertex{position: p, bound: units.Millimeters(delta)}
}

// loftEdgeLength is the proven bound on a straight loft edge's held length:
// the square root's own committed error against the exact rational squared
// length (capblend_contour.go's straightEdgeBound/ratSquaredDistance3), no
// new mechanism for an edge whose build carries a zero delta. An edge at a
// positive delta (§12 PR 2a — a placed build, or a COMPUTED station)
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
// `reversed` flag stays false (§5's wall-face row: every wall face is a Plane
// wound outward). The Frame is the exact answer for the three vertices handed
// to it whatever their own standing — §5's surface-parameter carve-out — while
// the face's own area and its vertices' positions carry the payload's delta.
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
		surface:       capStartSurf,
		loops:         capStartLoops,
		origins:       []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:          body,
		area:          cap0Val,
		areaBound:     capStartBound,
		axialDelta:    a.delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       capEndSurf,
		loops:         capEndLoops,
		origins:       []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:          body,
		area:          cap1Val,
		areaBound:     capEndBound,
		axialDelta:    a.delta,
		hasAxialDelta: true,
	}

	return capStart, capEnd, walls, nil
}

// capTriangleAreaAllow sums bounds.go's perturbedTriangleAreaAllow over one
// cap's own triangulation triangles (docs/loft-design.md §12 PR 2a) — the
// extra area a placement's delta can add to a cap's own exact rational area
// (capPolygonAreaRat), summed the same way loft_moments.go's accumulator
// sums it for the wall triangles.
func capTriangleAreaAllow(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	total := 0.0
	for _, t := range tris {
		total = upRound(total + perturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], delta))
	}
	return total
}

// capPolygonAreaRat returns the exact rational shoelace area of the cap
// polygon this construction ACTUALLY assembled: pts in that plane's own
// local (U, V) coordinates, walked per loop in loopIdx's own recorded walk
// order — assembleLoft's own pts0/loopIdx0 or pts1/loopIdx1, the identical
// arrays triangulate2DContext consumed to build that cap's own triangles.
// Reading the SAME points the triangles came from, rather than
// re-deriving the region's area from the record (moments.go), is what
// keeps the published cap Area and the built cap triangles in lockstep by
// construction: whatever assembleLoft walked into a triangle is exactly
// what this sums. On an untrimmed LineSeg profile that walked point IS the
// record's own endpoint, so this sum and moments.go's region-level integral
// are the same rational. On a TRIMMED LineSeg profile they are not: the walk
// lands on walkOf's float lerp2 endpoint while moments.go integrates the
// exact rational ratLerp (moments.go's ratLerp/lerp2 doc comments), and the
// cap reading follows the walked point, because that is the point the cap's
// own triangles have.
//
// The outer loop walks CCW and each hole walks CW
// (docs/sketch-seam-design.md), and a per-loop shoelace sum already nets a
// hole's area out with no special-casing — the identical convention
// moments.go's own Green's-theorem accumulator relies on (ProfileRecord.
// Area's own doc comment: "a hole's clockwise walk subtracts without a
// special case").
//
// Every coordinate is taken exactly as a math/big.Rat off its own float64
// (clearance_poly.go's mustRatOf, the package's take-the-floats-exactly
// discipline) — no float arithmetic anywhere in this sum. mustRatOf's
// finiteness precondition is already proven here: every pts entry is one
// of the SAME (U, V) pairs assembleLoft already lifted through its plane
// frame and checked with finiteVec before this function is ever reached
// (errLoftPointUnrepresentable, S13), so a non-finite U or V would have
// refused the build already.
//
// pts/loopIdx carry no assumption about segment kind, so admitting a
// curved same-kind pairing later needs no rework here: whatever stations
// assembleLoft chords a curve into become more pts entries this same
// shoelace sums unchanged.
func capPolygonAreaRat(pts []Point2, loopIdx [][]int) *big.Rat {
	sum := new(big.Rat)
	for _, idx := range loopIdx {
		n := len(idx)
		for j := range n {
			p, q := pts[idx[j]], pts[idx[(j+1)%n]]
			term := new(big.Rat).Mul(mustRatOf(p.U), mustRatOf(q.V))
			term.Sub(term, new(big.Rat).Mul(mustRatOf(q.U), mustRatOf(p.V)))
			sum.Add(sum, term)
		}
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}
