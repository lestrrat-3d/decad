package patchmesh

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Level mirrors the public Verification values passed by the root adapter.
type Level int

const (
	VerifyNone Level = iota
	VerifyBoundary
	VerifyAll
)

// Planar triangulates an all-planar PatchEdges result over shared vertices.
// publicSurface and publicCurve run only while formatting a rejected source.
func Planar(ctx context.Context, b *topo.Body, pp patchchain.BodyPayload, verify Level,
	publicSurface func(surfacegeom.Surface) any, publicCurve func(surfacegeom.Curve) any,
) (*topo.Mesh, error) {
	faces := topobuild.Faces(b)
	// curveBound applies only to Circle3 and Arc3. A Line3 follows its end
	// vertices; the cap-band miter whose Line3 tag holds a curved locus has
	// a ruled face with nonzero normalBound, so this gate rejects that case.
	allEligible := true
	for _, f := range faces {
		if !stitchrecord.FaceIsTetrahedronEligible(f) {
			allEligible = false
			break
		}
	}
	if !allEligible {
		for _, f := range faces {
			if _, ok := f.Surface.(surfacegeom.Plane); !ok {
				return nil, fmt.Errorf(`%w: this evaluator has no shared chording for a PatchEdges result's %T face`,
					decaderr.ErrUnsupported, publicSurface(f.Surface))
			}
			if f.NormalBound != 0 {
				return nil, fmt.Errorf(`%w: a PatchEdges result's planar face has no zero-bound normal`, decaderr.ErrUnsupported)
			}
			for _, l := range f.Loops {
				for _, ce := range l.CoEdges {
					if _, ok := ce.Edge.Curve.(surfacegeom.Line3); !ok {
						return nil, fmt.Errorf(`%w: this evaluator has no shared chording for a PatchEdges result's %T edge`,
							decaderr.ErrUnsupported, publicCurve(ce.Edge.Curve))
					}
				}
			}
		}
		return nil, fmt.Errorf(`%w: this evaluator cannot triangulate the PatchEdges result`, decaderr.ErrUnsupported)
	}
	vertices := topobuild.Vertices(b)
	mesh := &topo.Mesh{Vertices: make([]r3.Vec, len(vertices))}
	classOf := make(map[*topo.Vertex]int, len(vertices))
	store := make([]float64, len(vertices))
	budget := proofbound.NewWorkBudget(ctx)
	for i, v := range vertices {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		mesh.Vertices[i] = v.Position
		classOf[v] = i
		store[i] = v.Bound.Base()
	}
	if _, err := tessellation.StoreMax(store); err != nil {
		return nil, err
	}
	var err error
	mesh.Triangles, mesh.Source, err = stitchrecord.TriangulateFaces(ctx, faces, classOf)
	if err != nil {
		return nil, err
	}
	if err := patchchain.OrientBodyPatchFilledFaces(mesh, b, classOf, budget); err != nil {
		return nil, err
	}
	if pp.Xform.IsReflection() {
		for i := range mesh.Triangles {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}
	if err := tessellation.RequireCapBlendFacetAreas(mesh.Vertices, mesh.Triangles, "PatchEdges sheet"); err != nil {
		return nil, err
	}
	if err := topobuild.RequireSheetMesh(ctx, b, mesh); err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := loftmesh.LoftCrossingAudit(budget, mesh.Vertices, mesh.Triangles); err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, ctxErr
			}
			return nil, fmt.Errorf(`%w: a PatchEdges sheet's facets are not proven embedded: %s`, decaderr.ErrUnsupported, err)
		}
	}
	axial := make(map[*topo.Face]float64, len(faces))
	for _, f := range faces {
		axial[f] = f.AxialDelta
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, nil, axial, store, 0)
	if err != nil {
		return nil, err
	}
	for f, bound := range bounds {
		topobuild.SetFaceBound(mesh, f, bound)
	}
	if verify < VerifyAll {
		return mesh, nil
	}
	for i, tri := range mesh.Triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		face := mesh.Source[i]
		delta := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		delta = proofbound.AbsSumUpper(delta, face.AxialDelta)
		if delta == 0 {
			continue
		}
		a, bb, c := mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]]
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
			proofbound.PerturbedTriangleAreaAllow(a, bb, c, delta))
	}
	if proofbound.IsNonFinite(mesh.AreaSlack) {
		return nil, fmt.Errorf(`%w: a PatchEdges result states no finite area bound`, decaderr.ErrUnsupported)
	}
	return mesh, nil
}
