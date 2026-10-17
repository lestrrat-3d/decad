package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// tessellateBodyPatch reuses a circular source prism's station grid or
// triangulates an all-planar PatchEdges result over its shared vertices.
// Other curved sources need their own shared chording and are refused here.
func tessellateBodyPatch(ctx context.Context, b *Body, pp bodyPatchPayload, chord float64, verify Verification) (*Mesh, error) {
	if prism, ok := patchchain.CircularPrismSource(pp); ok {
		return tessellateBodyPatchCircularPrism(ctx, b, pp, prism, chord, verify)
	}
	faces := b.Faces()
	// curveBound applies only to Circle3 and Arc3. A Line3 follows its end
	// vertices; the cap-band miter whose Line3 tag holds a curved locus has
	// a ruled face with nonzero normalBound, so this gate rejects that case.
	allEligible := true
	for _, f := range faces {
		if !stitchrecord.FaceIsTetrahedronEligible(faceNode(f)) {
			allEligible = false
			break
		}
	}
	if !allEligible {
		for _, f := range faces {
			if _, ok := f.n.Surface.(surfacegeom.Plane); !ok {
				return nil, fmt.Errorf(`%w: this evaluator has no shared chording for a PatchEdges result's %T face`,
					ErrUnsupported, f.Surface())
			}
			if f.n.NormalBound != 0 {
				return nil, fmt.Errorf(`%w: a PatchEdges result's planar face has no zero-bound normal`, ErrUnsupported)
			}
			for _, l := range f.n.Loops {
				for _, ce := range l.CoEdges {
					if _, ok := ce.Edge.Curve.(surfacegeom.Line3); !ok {
						return nil, fmt.Errorf(`%w: this evaluator has no shared chording for a PatchEdges result's %T edge`,
							ErrUnsupported, publicCurve(ce.Edge.Curve))
					}
				}
			}
		}
		return nil, fmt.Errorf(`%w: this evaluator cannot triangulate the PatchEdges result`, ErrUnsupported)
	}
	vertices := b.Vertices()
	mesh := bornMesh(&Mesh{n: topo.Mesh{Vertices: make([]r3.Vec, len(vertices))}})
	classOf := make(map[*topo.Vertex]int, len(vertices))
	store := make([]float64, len(vertices))
	budget := proofbound.NewWorkBudget(ctx)
	for i, v := range vertices {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		mesh.n.Vertices[i] = v.n.Position
		classOf[vertexNode(v)] = i
		store[i] = v.n.Bound.Base()
	}
	if _, err := tessellation.StoreMax(store); err != nil {
		return nil, err
	}
	var err error
	mesh.n.Triangles, mesh.n.Source, err = stitchrecord.TriangulateFaces(ctx, faceNodes(faces), classOf)
	if err != nil {
		return nil, err
	}
	if err := orientBodyPatchFilledFaces(mesh, b, classOf, budget); err != nil {
		return nil, err
	}
	if pp.Xform.IsReflection() {
		for i := range mesh.n.Triangles {
			mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
		}
	}
	if err := tessellation.RequireCapBlendFacetAreas(mesh.n.Vertices, mesh.n.Triangles, "PatchEdges sheet"); err != nil {
		return nil, err
	}
	if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := loftmesh.LoftCrossingAudit(budget, mesh.n.Vertices, mesh.n.Triangles); err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, ctxErr
			}
			return nil, fmt.Errorf(`%w: a PatchEdges sheet's facets are not proven embedded: %s`, ErrUnsupported, err)
		}
	}
	axial := make(map[*topo.Face]float64, len(faces))
	for _, f := range faces {
		axial[faceNode(f)] = f.n.AxialDelta
	}
	if err := composeFaceBounds(mesh, nil, axial, store, 0); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return mesh, nil
	}
	for i, tri := range mesh.n.Triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		face := mesh.n.Source[i]
		delta := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		delta = proofbound.AbsSumUpper(delta, face.AxialDelta)
		if delta == 0 {
			continue
		}
		a, bb, c := mesh.n.Vertices[tri[0]], mesh.n.Vertices[tri[1]], mesh.n.Vertices[tri[2]]
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
			proofbound.PerturbedTriangleAreaAllow(a, bb, c, delta))
	}
	if proofbound.IsNonFinite(mesh.n.AreaSlack) {
		return nil, fmt.Errorf(`%w: a PatchEdges result states no finite area bound`, ErrUnsupported)
	}
	return mesh, nil
}

// orientBodyPatchFilledFaces makes each new fill's mesh boundary run opposite
// its source face's mesh boundary. The source face already carries its
// feature's outward sense, including any reflection before PatchEdges. The
// patch's new plane frame can have the opposite local handedness, so its
// triangle winding is settled by the shared mesh edge, not that frame.
func orientBodyPatchFilledFaces(m *Mesh, b *Body, classOf map[*topo.Vertex]int, budget *proofbound.WorkBudget) error {
	byFace := make(map[*topo.Face][]int, len(b.Faces()))
	for i, f := range m.n.Source {
		byFace[f] = append(byFace[f], i)
	}
	for _, f := range b.Faces() {
		if err := budget.Step(); err != nil {
			return err
		}
		if len(f.n.Origins) != 1 || f.n.Origins[0].Producer != b.n.Origin.Producer || f.n.Origins[0].Role != rolePatch {
			continue
		}
		if len(f.n.Loops) == 0 || len(f.n.Loops[0].CoEdges) == 0 {
			return fmt.Errorf(`%w: a PatchEdges fill has no boundary walk to orient`, ErrDegenerate)
		}
		ce := f.n.Loops[0].CoEdges[0]
		start, end := classOf[coedgeStart(ce)], classOf[coedgeEnd(ce)]
		var source *topo.Face
		for _, adjacent := range ce.Edge.Faces {
			if adjacent != &f.n {
				source = adjacent
				break
			}
		}
		if source == nil {
			return fmt.Errorf(`%w: a PatchEdges fill has no source face on its rim`, ErrDegenerate)
		}
		patchSense := tessellation.PatchEdgeSense(m.n.Triangles, byFace[&f.n], start, end)
		sourceSense := tessellation.PatchEdgeSense(m.n.Triangles, byFace[source], start, end)
		if patchSense == 0 || sourceSense == 0 {
			return fmt.Errorf(`%w: a PatchEdges rim has no matching source and fill mesh edges`, ErrUnsupported)
		}
		if patchSense != sourceSense {
			continue
		}
		for _, ti := range byFace[&f.n] {
			m.n.Triangles[ti][1], m.n.Triangles[ti][2] = m.n.Triangles[ti][2], m.n.Triangles[ti][1]
		}
	}
	return nil
}
