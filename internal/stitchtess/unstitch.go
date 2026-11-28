package stitchtess

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// UnstitchPlanar triangulates one straight-edged planar unstitched face.
func UnstitchPlanar(ctx context.Context, body *topo.Body, verify bool, areaProof bool) (*topo.Mesh, error) {
	faces := topobuild.Faces(body)
	if body.Kind != topo.BodySheet || len(faces) != 1 || !stitchrecord.FaceIsTetrahedronEligible(faces[0]) || faces[0].HeldNormalBound != 0 {
		return nil, fmt.Errorf("%w: an unstitched face needs a planar straight-edged mesh", decaderr.ErrUnsupported)
	}
	vertices := topobuild.Vertices(body)
	mesh := &topo.Mesh{Vertices: make([]r3.Vec, len(vertices))}
	indices := make(map[*topo.Vertex]int, len(vertices))
	store := make([]float64, len(vertices))
	budget := proofbound.NewWorkBudget(ctx)
	for i, v := range vertices {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		mesh.Vertices[i], indices[v], store[i] = v.Position, i, v.Bound.Mag()
	}
	coord, err := tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	mesh.CoordBound = coord
	mesh.Triangles, mesh.Source, err = stitchrecord.TriangulateFaces(ctx, faces, indices)
	if err != nil {
		return nil, err
	}
	if err := tessellation.RequireCapBlendFacetAreas(mesh.Vertices, mesh.Triangles, "unstitch sheet"); err != nil {
		return nil, err
	}
	if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
		return nil, err
	}
	if verify {
		if err := loftmesh.LoftCrossingAudit(budget, mesh.Vertices, mesh.Triangles); err != nil {
			return nil, topobuild.LiftAuditError(err)
		}
	}
	axial := map[*topo.Face]float64{faces[0]: faces[0].AxialDelta}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, nil, axial, store, 0)
	if err != nil {
		return nil, err
	}
	for f, bound := range bounds {
		topobuild.SetFaceBound(mesh, f, bound)
	}
	if areaProof {
		for _, tri := range mesh.Triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			delta := proofbound.AbsSumUpper(math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]])), faces[0].AxialDelta)
			if delta > 0 {
				mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
					proofbound.PerturbedTriangleAreaAllow(mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]], delta))
			}
		}
		if proofbound.IsNonFinite(mesh.AreaSlack) {
			return nil, fmt.Errorf("%w: an unstitched face has no finite area bound", decaderr.ErrUnsupported)
		}
	}
	return mesh, nil
}
