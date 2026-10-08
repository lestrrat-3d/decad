package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file adapts the flat-triangle assembly in internal/loftmesh and builds
// the Body topology over it.
//
// loftmesh.Assemble produces the vertex and triangle sets — two triangles per
// wall cell, plus a triangulated cap at each end — and buildLoftTopology turns
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
	// §12 PR 2a) — proofbound.AbsSumUpper(stationRound, placeAllow): zero exactly when
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
// any of them is lifted into an exact dyadic.
//
// stationRound is loftPairings' own accumulated Table S row S14 term
// (a10-plan.md Part 3 PR 6): the proven rounding every COMPUTED circular
// station commits, composed into delta beside the placement's own
// proofbound.RigidRoundAllow term.
func assembleLoft(ctx context.Context, pairs []loftLoopPair, f0, f1 r3.Frame, plane0 PlaneRecord, xform r3.Transform, stationRound float64) (loftAssembly, error) {
	records := make([]loftmesh.LoopPair, len(pairs))
	for i, pair := range pairs {
		records[i] = loftmesh.LoopPair{V: pair.v, W: pair.w}
	}
	triangulate := func(ctx context.Context, pts []Point2, loops [][]int) ([][3]int, error) {
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		return tris, wrapLoftTriangulationError(err)
	}
	a, err := loftmesh.Assemble(ctx, records, f0, f1, plane0, xform, stationRound,
		triangulate, errLoftPointUnrepresentable)
	if err != nil {
		return loftAssembly{}, err
	}
	return loftAssembly{
		verts: a.Verts, tris: a.Tris, walls: a.Walls, capStartCount: a.CapStartCount,
		reversed: a.Reversed, cell: a.Cell, side: a.Side, vIdx: a.VIdx, wIdx: a.WIdx,
		pts0: a.Pts0, pts1: a.Pts1, loopIdx0: a.LoopIdx0, loopIdx1: a.LoopIdx1,
		delta: a.Delta,
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
// test reads "a body this evaluator cannot build". internal/freeform/spline_length.go's R15 and
// spline_fit.go's R16 draw the identical line for a finite input whose
// derived magnitude runs off float64; errors.go scopes ErrNotFinite to a
// non-finite PARAMETER or a derived non-finite MEASUREMENT, and
// validateLoftBodyMeasurements already owns that second case.
//
// The gate runs BEFORE the first exact-dyadic lift, never after it:
// tessellation.OrientationSign lifts the anchor and every vertex through dyVec, whose
// mustDyOf PANICS on a non-finite float, so a check placed any later is a
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
// the square root's own committed error against the exact squared length
// (capblend_contour.go's straightEdgeBound/dySquaredDistance3), no
// new mechanism for an edge whose build carries a zero delta. An edge at a
// positive delta (§12 PR 2a — a placed build, or a COMPUTED station)
// composes that with internal/proofbound/bounds.go's proofbound.ChainLengthBound(1, delta, held) — both
// endpoints displaced by delta is exactly that helper's own one-chord case —
// through proofbound.AbsSumUpper.
func loftEdgeLength(a, b r3.Vec, delta float64) (float64, float64) {
	held := a.Sub(b).Len()
	sq, sqOK := dySquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
	bound := straightEdgeBound(held, sq, sqOK)
	if delta > 0 {
		bound = proofbound.AbsSumUpper(bound, proofbound.ChainLengthBound(1, delta, held))
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

// junctionConvex decides a rung or diagonal edge's convexity: proofarith.OrientSign(A,
// B, C, D) < 0, where (A, B, C) is primary's own outward-wound vertex order
// and D is other's apex — design O3, pinned against the box fixture: a
// standard box's vertical edge (a rung) is a genuine convex corner, and this
// is the sign that reads it as one. A zero result is a decided non-convex
// (flat) edge: docs/loft-design.md §5's rule for a flat rung or diagonal.
func junctionConvex(verts []r3.Vec, primary, other [3]int, a, b int) bool {
	apex := junctionApex(other, a, b)
	return proofarith.OrientSign(verts[primary[0]], verts[primary[1]], verts[primary[2]], verts[apex]) < 0
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
// (loft_moments.go's loftmesh.WallTriangleArea, the identical bracket the mass
// accumulator sums), and its side(i,j,k) role. A placed triangle (delta > 0,
// §12 PR 2a) widens that bracket by internal/proofbound/bounds.go's proofbound.PerturbedTriangleAreaAllow,
// the same per-triangle correction the mass accumulator sums into Area's own
// bound.
func buildLoftWallFace(body *Body, ref producerID, verts []r3.Vec, tri [3]int, i, j, side int, delta float64) (*Face, error) {
	surf, err := planeFromTriangle(verts, tri)
	if err != nil {
		return nil, err
	}
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	u := proofarith.Xsub(proofarith.XptOf(b), proofarith.XptOf(a))
	v := proofarith.Xsub(proofarith.XptOf(c), proofarith.XptOf(a))
	lo, hi := loftmesh.WallTriangleArea(u, v)
	areaBound := proofbound.UpRound(hi - lo)
	if delta > 0 {
		areaBound = proofbound.AbsSumUpper(areaBound, proofbound.PerturbedTriangleAreaAllow(a, b, c, delta))
	}
	return &Face{
		surface:   surf,
		origins:   []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(%d,%d,%d)", i, j, side)}},
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
func buildLoftTopology(ctx context.Context, body *Body, ref producerID, a loftAssembly, cap0Rat, cap1Rat *big.Rat) (*Face, *Face, []*Face, error) {
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
	capStartBound := proofarith.RationalFloatError(cap0Rat, cap0Val)
	capEndBound := proofarith.RationalFloatError(cap1Rat, cap1Val)
	if a.delta > 0 {
		capStartTris := a.tris[a.walls : a.walls+a.capStartCount]
		capEndTris := a.tris[a.walls+a.capStartCount:]
		capStartBound = proofbound.AbsSumUpper(capStartBound, capTriangleAreaAllow(a.verts, capStartTris, a.delta))
		capEndBound = proofbound.AbsSumUpper(capEndBound, capTriangleAreaAllow(a.verts, capEndTris, a.delta))
	}
	capStart := &Face{
		surface:       capStartSurf,
		loops:         capStartLoops,
		origins:       []FeatureRef{{producer: ref, Role: roleCapStart}},
		body:          body,
		area:          cap0Val,
		areaBound:     capStartBound,
		axialDelta:    a.delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       capEndSurf,
		loops:         capEndLoops,
		origins:       []FeatureRef{{producer: ref, Role: roleCapEnd}},
		body:          body,
		area:          cap1Val,
		areaBound:     capEndBound,
		axialDelta:    a.delta,
		hasAxialDelta: true,
	}

	return capStart, capEnd, walls, nil
}

// capTriangleAreaAllow sums internal/proofbound/bounds.go's proofbound.PerturbedTriangleAreaAllow over one
// cap's own triangulation triangles (docs/loft-design.md §12 PR 2a) — the
// extra area a placement's delta can add to a cap's own exact rational area
// (capPolygonAreaRat), summed the same way loft_moments.go's accumulator
// sums it for the wall triangles.
func capTriangleAreaAllow(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	total := 0.0
	for _, t := range tris {
		total = proofbound.UpRound(total + proofbound.PerturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], delta))
	}
	return total
}

// capPolygonAreaRat returns the exact rational shoelace area of the cap
// polygon this construction ACTUALLY assembled: pts in that plane's own
// local (U, V) coordinates, walked per loop in loopIdx's own recorded walk
// order — assembleLoft's own pts0/loopIdx0 or pts1/loopIdx1, the identical
// arrays triangulation.Triangulate consumed to build that cap's own triangles.
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
// (polynomial.MustRatOf from internal/polynomial, with its take-the-floats-exactly
// discipline) — no float arithmetic anywhere in this sum. polynomial.MustRatOf's
// finiteness precondition is already proven here: every pts entry is one
// of the SAME (U, V) pairs assembleLoft already lifted through its plane
// frame and checked with proofbound.FiniteVec before this function is ever reached
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
			term := new(big.Rat).Mul(polynomial.MustRatOf(p.U), polynomial.MustRatOf(q.V))
			term.Sub(term, new(big.Rat).Mul(polynomial.MustRatOf(q.U), polynomial.MustRatOf(p.V)))
			sum.Add(sum, term)
		}
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}
