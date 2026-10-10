package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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

// assembleLoft lifts every recorded point once, emits the 2*sum(n_i) wall
// triangles in Table B's order and winding, triangulates both caps through
// internal/triangulation's polygon-with-holes triangulator with capStart's
// triples reversed and capEnd's retained (§5's cap seeding), and orients the
// complete shell once from the signed tetrahedron sum anchored at the placed
// p0 origin (§5's whole-shell rule). It also owns Table S row S13: every
// placed coordinate it emits, the anchor among them, is proven finite before
// any of them is lifted into an exact dyadic.
//
// stationRound is loftmesh.PairRecords' own accumulated Table S row S14 term
// (a10-plan.md Part 3 PR 6): the proven rounding every COMPUTED circular
// station commits, composed into delta beside the placement's own
// proofbound.RigidRoundAllow term.
func assembleLoft(ctx context.Context, pairs []loftmesh.LoopPair, f0, f1 r3.Frame, plane0 planeRecord, xform r3.Transform, stationRound float64) (loftmesh.Assembly, error) {
	triangulate := func(ctx context.Context, pts []sectionrecord.Point2, loops [][]int) ([][3]int, error) {
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		return tris, triangulation.WrapLoftError(err)
	}
	return loftmesh.Assemble(ctx, pairs, f0, f1, plane0, xform, stationRound,
		triangulate, errLoftPointUnrepresentable)
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

// loftEdge builds one straight loft edge between two vertex-table indices,
// with the given walked-boundary convexity.
func loftEdge(vertexObjs []*Vertex, positions []r3.Vec, a, b int, convex bool, delta float64) *Edge {
	held, bound := loftmesh.EdgeLength(positions[a], positions[b], delta)
	return &Edge{curve: Line3{}, start: vertexObjs[a], end: vertexObjs[b], convex: convex, length: held, lengthBound: bound}
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
func buildLoftTopology(ctx context.Context, body *Body, ref producerID, a loftmesh.Assembly, cap0Rat, cap1Rat *big.Rat) (*Face, *Face, []*Face, error) {
	vertexObjs := make([]*Vertex, len(a.Verts))
	for i, p := range a.Verts {
		vertexObjs[i] = loftVertex(p, a.Delta)
	}

	loopCount := len(a.VIdx)
	lowerTri := make([][][3]int, loopCount)
	upperTri := make([][][3]int, loopCount)
	for i := range a.VIdx {
		lowerTri[i] = make([][3]int, len(a.VIdx[i]))
		upperTri[i] = make([][3]int, len(a.VIdx[i]))
	}
	for k := range a.Walls {
		i, j := a.Cell[k][0], a.Cell[k][1]
		if a.Side[k] == 0 {
			lowerTri[i][j] = a.Tris[k]
		} else {
			upperTri[i][j] = a.Tris[k]
		}
	}

	var walls []*Face
	var capStartLoops, capEndLoops []*Loop
	for i := range loopCount {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		n := len(a.VIdx[i])
		isOuter := i == 0
		vIdx, wIdx := a.VIdx[i], a.WIdx[i]

		rimBottom := make([]*Edge, n)
		rimTop := make([]*Edge, n)
		diagE := make([]*Edge, n)
		rungE := make([]*Edge, n)
		for j := range n {
			jn := (j + 1) % n
			rimBottom[j] = loftEdge(vertexObjs, a.Verts, vIdx[j], vIdx[jn], isOuter, a.Delta)
			rimTop[j] = loftEdge(vertexObjs, a.Verts, wIdx[j], wIdx[jn], isOuter, a.Delta)
		}
		for j := range n {
			jn := (j + 1) % n
			jp := (j - 1 + n) % n
			rungConvex := loftmesh.JunctionConvex(a.Verts, lowerTri[i][jp], upperTri[i][j], vIdx[j], wIdx[j])
			rungE[j] = loftEdge(vertexObjs, a.Verts, vIdx[j], wIdx[j], rungConvex, a.Delta)
			diagConvex := loftmesh.JunctionConvex(a.Verts, lowerTri[i][j], upperTri[i][j], vIdx[j], wIdx[jn])
			diagE[j] = loftEdge(vertexObjs, a.Verts, vIdx[j], wIdx[jn], diagConvex, a.Delta)
		}

		capStartCo := make([]coedge, n)
		capEndCo := make([]coedge, n)
		for j := range n {
			jn := (j + 1) % n

			lowerFace, err := buildLoftWallFace(body, ref, a.Verts, lowerTri[i][j], i, j, 0, a.Delta)
			if err != nil {
				return nil, nil, nil, err
			}
			lowerFace.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: rimBottom[j], forward: true},
				{edge: rungE[jn], forward: true},
				{edge: diagE[j], forward: false},
			}, a.Reversed)}}
			walls = append(walls, lowerFace)

			upperFace, err := buildLoftWallFace(body, ref, a.Verts, upperTri[i][j], i, j, 1, a.Delta)
			if err != nil {
				return nil, nil, nil, err
			}
			upperFace.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: diagE[j], forward: true},
				{edge: rimTop[j], forward: false},
				{edge: rungE[j], forward: false},
			}, a.Reversed)}}
			walls = append(walls, upperFace)

			capStartCo[n-1-j] = coedge{edge: rimBottom[j], forward: false}
			capEndCo[j] = coedge{edge: rimTop[j], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: isOuter, coedges: loftLoopCoedges(capStartCo, a.Reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: isOuter, coedges: loftLoopCoedges(capEndCo, a.Reversed)})
	}

	capStartSurf, err := planeFromTriangle(a.Verts, a.Tris[a.Walls])
	if err != nil {
		return nil, nil, nil, err
	}
	capEndSurf, err := planeFromTriangle(a.Verts, a.Tris[a.Walls+a.CapStartCount])
	if err != nil {
		return nil, nil, nil, err
	}
	cap0Val, _ := cap0Rat.Float64()
	cap1Val, _ := cap1Rat.Float64()
	capStartBound := proofarith.RationalFloatError(cap0Rat, cap0Val)
	capEndBound := proofarith.RationalFloatError(cap1Rat, cap1Val)
	if a.Delta > 0 {
		capStartTris := a.Tris[a.Walls : a.Walls+a.CapStartCount]
		capEndTris := a.Tris[a.Walls+a.CapStartCount:]
		capStartBound = proofbound.AbsSumUpper(capStartBound, loftmesh.CapTriangleAreaAllow(a.Verts, capStartTris, a.Delta))
		capEndBound = proofbound.AbsSumUpper(capEndBound, loftmesh.CapTriangleAreaAllow(a.Verts, capEndTris, a.Delta))
	}
	capStart := &Face{
		surface:       capStartSurf,
		loops:         capStartLoops,
		origins:       []FeatureRef{{producer: ref, Role: roleCapStart}},
		body:          body,
		area:          cap0Val,
		areaBound:     capStartBound,
		axialDelta:    a.Delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       capEndSurf,
		loops:         capEndLoops,
		origins:       []FeatureRef{{producer: ref, Role: roleCapEnd}},
		body:          body,
		area:          cap1Val,
		areaBound:     capEndBound,
		axialDelta:    a.Delta,
		hasAxialDelta: true,
	}

	return capStart, capEnd, walls, nil
}
