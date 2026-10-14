package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"

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

// loftVertex builds a vertex at a recorded (or lifted-from-recorded)
// coordinate: every loft vertex position comes from Plane.Origin + p.U*Plane.U
// + p.V*Plane.V, the identical single float64 evaluation Extrude already
// performs for a cap vertex (§5), so a vertex of a build whose delta is zero
// carries the same zero-bound standing; any other vertex carries the payload's
// own delta (§12 PR 2a). Zero delta is NOT the same claim as unplaced: an
// unplaced build still carries a positive delta wherever a station was
// COMPUTED rather than pinned (loftPayload's own delta doc comment).
func loftVertex(p r3.Vec, delta float64) *Vertex {
	return bornVertex(&Vertex{n: topo.Vertex{Position: p, Bound: units.Millimeters(delta)}})
}

// loftEdge builds one straight loft edge between two vertex-table indices,
// with the given walked-boundary convexity.
func loftEdge(vertexObjs []*Vertex, positions []r3.Vec, a, b int, convex bool, delta float64) *Edge {
	held, bound := loftmesh.EdgeLength(positions[a], positions[b], delta)
	return &Edge{n: topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.Line3{}, Start: vertexNode(vertexObjs[a]), End: vertexNode(vertexObjs[b])}}
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
func planeFromTriangle(verts []r3.Vec, tri [3]int) (surfacegeom.Plane, error) {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	f, err := r3.NewFrame(a, b.Sub(a), c.Sub(a))
	if err != nil {
		return surfacegeom.Plane{}, fmt.Errorf(`%w: a loft triangle has no plane: %s`, ErrDegenerate, err)
	}
	return surfacegeom.Plane{Frame: f}, nil
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
	return bornFace(&Face{
		n: topo.Face{
			Area:      lo,
			AreaBound: areaBound,
			Surface:   surf,
			Origins:   []provenance.Ref{{Producer: ref, Role: fmt.Sprintf("side(%d,%d,%d)", i, j, side)}},
			Body:      bodyNode(body),
		},
	}), nil
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
			lowerFace.loops = []*Loop{{n: topo.Loop{Outer: true}, coedges: loftLoopCoedges([]coedge{
				{edge: rimBottom[j], forward: true},
				{edge: rungE[jn], forward: true},
				{edge: diagE[j], forward: false},
			}, a.Reversed)}}
			walls = append(walls, lowerFace)

			upperFace, err := buildLoftWallFace(body, ref, a.Verts, upperTri[i][j], i, j, 1, a.Delta)
			if err != nil {
				return nil, nil, nil, err
			}
			upperFace.loops = []*Loop{{n: topo.Loop{Outer: true}, coedges: loftLoopCoedges([]coedge{
				{edge: diagE[j], forward: true},
				{edge: rimTop[j], forward: false},
				{edge: rungE[j], forward: false},
			}, a.Reversed)}}
			walls = append(walls, upperFace)

			capStartCo[n-1-j] = coedge{edge: rimBottom[j], forward: false}
			capEndCo[j] = coedge{edge: rimTop[j], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{n: topo.Loop{Outer: isOuter}, coedges: loftLoopCoedges(capStartCo, a.Reversed)})
		capEndLoops = append(capEndLoops, &Loop{n: topo.Loop{Outer: isOuter}, coedges: loftLoopCoedges(capEndCo, a.Reversed)})
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
	capStart := bornFace(&Face{
		loops: capStartLoops,
		n: topo.Face{
			Area:          cap0Val,
			AreaBound:     capStartBound,
			AxialDelta:    a.Delta,
			HasAxialDelta: true,
			Surface:       capStartSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:          bodyNode(body),
		},
	})
	capEnd := bornFace(&Face{
		loops: capEndLoops,
		n: topo.Face{
			Area:          cap1Val,
			AreaBound:     capEndBound,
			AxialDelta:    a.Delta,
			HasAxialDelta: true,
			Surface:       capEndSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:          bodyNode(body),
		},
	})

	return capStart, capEnd, walls, nil
}
