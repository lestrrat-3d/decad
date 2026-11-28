package loftrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// TessellateChain restates the triangular faces already audited by BuildChain.
func TessellateChain(ctx context.Context, body *topo.Body) (*topo.Mesh, error) {
	mesh := &topo.Mesh{}
	vertices := make(map[*topo.Vertex]int)
	for _, face := range topobuild.Faces(body) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if len(face.Loops) != 1 || len(face.Loops[0].CoEdges) != 3 {
			return nil, fmt.Errorf("%w: a chain loft face is not a triangle", decaderr.ErrUnsupported)
		}
		var tri [3]int
		bound := 0.0
		for j, edge := range face.Loops[0].CoEdges {
			v := topobuild.CoEdgeStart(edge)
			index, ok := vertices[v]
			if !ok {
				index = len(mesh.Vertices)
				vertices[v] = index
				mesh.Vertices = append(mesh.Vertices, v.Position)
			}
			tri[j] = index
			bound = math.Max(bound, v.Bound.Mag())
		}
		topobuild.AddTriangle(mesh, tri, face)
		topobuild.SetFaceBound(mesh, face, bound)
		mesh.CoordBound = math.Max(mesh.CoordBound, bound)
		lo, hi, ok := tessellation.ChainSweepTriangleArea(mesh.Vertices, tri)
		if !ok {
			return nil, fmt.Errorf("%w: a chain loft face has no positive facet area", decaderr.ErrUnsupported)
		}
		area := proofbound.RatInterval{Lo: new(big.Rat).Set(proofarith.FloatRat(lo)), Hi: new(big.Rat).Set(proofarith.FloatRat(hi))}
		slack, err := tessellation.ChainSweepFaceAreaSlack(face.Area, face.AreaBound, area)
		if err != nil {
			return nil, err
		}
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, slack)
	}
	if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
		return nil, err
	}
	return mesh, nil
}
