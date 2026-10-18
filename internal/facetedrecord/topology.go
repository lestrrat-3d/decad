package facetedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// buildTopology chains the face-boundary mesh edges into topological
// Edges (split where the adjacent face pair or the exact hinge convexity
// changes) and builds each face's loops by walking the facet fans. Each
// Vertex reports its own held bound VertexBound[v], and each Edge the largest
// of its chain's vertices' bounds (docs/faceted-vertex-bounds-design.md
// §4.2). Every vertex, edge and loop node comes from alloc.
func buildTopology(
	ctx context.Context,
	alloc topo.Alloc,
	verts []r3.Vec,
	tris [][3]int,
	facetFace []*topo.Face,
	facePlanar map[*topo.Face]bool,
	vertexBound []float64,
	axialSectionJoin bool,
) error {
	return buildTopologyWithBudget(ctx, nil, alloc, verts, tris, facetFace,
		facePlanar, vertexBound, axialSectionJoin)
}

func buildTopologyWithBudget(
	ctx context.Context,
	budget *proofbound.WorkBudget,
	alloc topo.Alloc,
	verts []r3.Vec,
	tris [][3]int,
	facetFace []*topo.Face,
	facePlanar map[*topo.Face]bool,
	vertexBound []float64,
	axialSectionJoin bool,
) error {
	faceID := map[*topo.Face]int{}
	faces := []*topo.Face{}
	facetID := make([]int, len(facetFace))
	planarID := map[int]bool{}
	for i, f := range facetFace {
		id, ok := faceID[f]
		if !ok {
			id = len(faces)
			faceID[f] = id
			faces = append(faces, f)
			planarID[id] = facePlanar[f]
		}
		facetID[i] = id
	}
	var plan facetedtopology.Result
	var err error
	if budget == nil {
		plan, err = facetedtopology.Build(ctx, verts, tris, facetID, planarID, vertexBound)
	} else {
		plan, err = facetedtopology.BuildWithBudget(ctx, budget, verts, tris, facetID, planarID, vertexBound)
	}
	if err != nil {
		return err
	}
	vertexObj := map[int]*topo.Vertex{}
	vertexOf := func(vi int) *topo.Vertex {
		if v, ok := vertexObj[vi]; ok {
			return v
		}
		v := topobuild.NewVertex(alloc, topo.Vertex{Position: verts[vi], Bound: units.Millimeters(vertexBound[vi])})
		vertexObj[vi] = v
		return v
	}
	edgeObj := map[*facetedtopology.Edge]*topo.Edge{}
	for _, chain := range plan.Edges {
		e := topobuild.NewEdge(alloc, topo.Edge{
			Convex:          chain.Convex,
			Length:          chain.Length,
			LengthBound:     chain.LengthBound,
			LengthUnbounded: chain.LengthUnbounded,
			Curve:           surfacegeom.FacetedCurve{Bound: units.Millimeters(chain.Bound)},
			Start:           vertexOf(chain.Vertices[0]),
			End:             vertexOf(chain.Vertices[len(chain.Vertices)-1]),
			Faces:           []*topo.Face{faces[chain.Faces[0]], faces[chain.Faces[1]]},
		})
		if axialSectionJoin {
			e.SelectorDirection, e.SelectorDirectionProven =
				facetedtopology.AxialChainDirection(verts, chain.Vertices)
		}
		edgeObj[chain] = e
	}
	for fi, loops := range plan.FaceLoops {
		f := faces[fi]
		for _, planned := range loops {
			loop := topobuild.NewLoop(alloc, topo.Loop{Outer: planned.Outer})
			for _, ce := range planned.Coedges {
				loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: edgeObj[ce.Edge], Forward: ce.Forward})
			}
			f.Loops = append(f.Loops, loop)
		}
	}
	return nil
}
