package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
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

// buildLoftWallFace builds one wall triangle's Face (§7's lower/upper wall
// triangle row): its own Plane, its own proven area bracket
// (loft_moments.go's loftmesh.WallTriangleArea, the identical bracket the mass
// accumulator sums), and its side(i,j,k) role. A placed triangle (delta > 0,
// §12 PR 2a) widens that bracket by internal/proofbound/bounds.go's proofbound.PerturbedTriangleAreaAllow,
// the same per-triangle correction the mass accumulator sums into Area's own
// bound.
func buildLoftWallFace(body *Body, ref producerID, verts []r3.Vec, tri [3]int, i, j, side int, delta float64, area *loftmesh.WallArea) (*Face, error) {
	surf, err := topobuild.PlaneFromTriangle(verts, tri)
	if err != nil {
		return nil, err
	}
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	u := proofarith.Xsub(proofarith.XptOf(b), proofarith.XptOf(a))
	v := proofarith.Xsub(proofarith.XptOf(c), proofarith.XptOf(a))
	lo, hi := loftmesh.WallTriangleArea(u, v)
	if area != nil {
		*area = loftmesh.WallArea{Lo: lo, Hi: hi}
	}
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

// buildLoftTopology builds the B-rep topology from the assembled, globally
// oriented triangle set (docs/loft-design.md §5/§7): real Vertex/Edge/Loop/
// Face objects sharing indices with the assembly's own vertex table. Every
// edge bounds exactly two faces by construction (§5's four edge families:
// bottom rim, top rim, diagonal, rung), and every cap-boundary edge opposes
// its incident wall edge, the standard two-manifold convention.
//
// Every loop this builds is stated in §5's LOCAL vertex order and then passed
// through topobuild.LoftLoopCoedges, which is what carries the assembly's own
// whole-shell reversal into the directed boundary each face publishes. A walk
// emitted without it agrees with its face's Plane on one axial spelling of a
// section pair and opposes it on the mirror.
func buildLoftTopology(ctx context.Context, body *Body, ref producerID, a loftmesh.Assembly, cap0Rat, cap1Rat *big.Rat) (*Face, *Face, []*Face, []loftmesh.WallArea, error) {
	vertexObjs := make([]*topo.Vertex, len(a.Verts))
	for i, p := range a.Verts {
		vertexObjs[i] = topobuild.LoftVertex(handleAlloc{}, p, a.Delta)
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
	var wallAreas []loftmesh.WallArea
	var capStartLoops, capEndLoops []*Loop
	for i := range loopCount {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, nil, err
		}
		if wallAreas == nil {
			wallAreas = make([]loftmesh.WallArea, 0, a.Walls)
		}
		n := len(a.VIdx[i])
		isOuter := i == 0
		vIdx, wIdx := a.VIdx[i], a.WIdx[i]

		rimBottom := make([]*topo.Edge, n)
		rimTop := make([]*topo.Edge, n)
		diagE := make([]*topo.Edge, n)
		rungE := make([]*topo.Edge, n)
		for j := range n {
			jn := (j + 1) % n
			rimBottom[j] = topobuild.LoftEdge(handleAlloc{}, vertexObjs, a.Verts, vIdx[j], vIdx[jn], isOuter, a.Delta)
			rimTop[j] = topobuild.LoftEdge(handleAlloc{}, vertexObjs, a.Verts, wIdx[j], wIdx[jn], isOuter, a.Delta)
		}
		for j := range n {
			jn := (j + 1) % n
			jp := (j - 1 + n) % n
			rungConvex := loftmesh.JunctionConvex(a.Verts, lowerTri[i][jp], upperTri[i][j], vIdx[j], wIdx[j])
			rungE[j] = topobuild.LoftEdge(handleAlloc{}, vertexObjs, a.Verts, vIdx[j], wIdx[j], rungConvex, a.Delta)
			diagConvex := loftmesh.JunctionConvex(a.Verts, lowerTri[i][j], upperTri[i][j], vIdx[j], wIdx[jn])
			diagE[j] = topobuild.LoftEdge(handleAlloc{}, vertexObjs, a.Verts, vIdx[j], wIdx[jn], diagConvex, a.Delta)
		}

		capStartCo := make([]topo.CoEdge, n)
		capEndCo := make([]topo.CoEdge, n)
		for j := range n {
			jn := (j + 1) % n

			var lowerArea loftmesh.WallArea
			lowerFace, err := buildLoftWallFace(body, ref, a.Verts, lowerTri[i][j], i, j, 0, a.Delta, &lowerArea)
			if err != nil {
				return nil, nil, nil, nil, err
			}
			lowerFace.n.Loops = []*topo.Loop{loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: topobuild.LoftLoopCoedges([]topo.CoEdge{
				{Edge: rimBottom[j], Forward: true},
				{Edge: rungE[jn], Forward: true},
				{Edge: diagE[j], Forward: false},
			}, a.Reversed),
			}}))}
			walls = append(walls, lowerFace)
			wallAreas = append(wallAreas, lowerArea)

			var upperArea loftmesh.WallArea
			upperFace, err := buildLoftWallFace(body, ref, a.Verts, upperTri[i][j], i, j, 1, a.Delta, &upperArea)
			if err != nil {
				return nil, nil, nil, nil, err
			}
			upperFace.n.Loops = []*topo.Loop{loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: topobuild.LoftLoopCoedges([]topo.CoEdge{
				{Edge: diagE[j], Forward: true},
				{Edge: rimTop[j], Forward: false},
				{Edge: rungE[j], Forward: false},
			}, a.Reversed),
			}}))}
			walls = append(walls, upperFace)
			wallAreas = append(wallAreas, upperArea)

			capStartCo[n-1-j] = topo.CoEdge{Edge: rimBottom[j], Forward: false}
			capEndCo[j] = topo.CoEdge{Edge: rimTop[j], Forward: true}
		}
		capStartLoops = append(capStartLoops, bornLoop(&Loop{n: topo.Loop{Outer: isOuter, CoEdges: topobuild.LoftLoopCoedges(capStartCo, a.Reversed)}}))
		capEndLoops = append(capEndLoops, bornLoop(&Loop{n: topo.Loop{Outer: isOuter, CoEdges: topobuild.LoftLoopCoedges(capEndCo, a.Reversed)}}))
	}

	capStartSurf, err := topobuild.PlaneFromTriangle(a.Verts, a.Tris[a.Walls])
	if err != nil {
		return nil, nil, nil, nil, err
	}
	capEndSurf, err := topobuild.PlaneFromTriangle(a.Verts, a.Tris[a.Walls+a.CapStartCount])
	if err != nil {
		return nil, nil, nil, nil, err
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
		n: topo.Face{
			Area:          cap0Val,
			AreaBound:     capStartBound,
			AxialDelta:    a.Delta,
			HasAxialDelta: true,
			Surface:       capStartSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:          bodyNode(body),
			Loops:         loopNodes(capStartLoops),
		},
	})
	capEnd := bornFace(&Face{
		n: topo.Face{
			Area:          cap1Val,
			AreaBound:     capEndBound,
			AxialDelta:    a.Delta,
			HasAxialDelta: true,
			Surface:       capEndSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:          bodyNode(body),
			Loops:         loopNodes(capEndLoops),
		},
	})

	return capStart, capEnd, walls, wallAreas, nil
}
