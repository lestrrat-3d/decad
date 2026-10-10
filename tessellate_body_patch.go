package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// tessellateBodyPatch reuses a circular source prism's station grid or
// triangulates an all-planar Body.Patch result over its shared vertices.
// Other curved sources need their own shared chording and are refused here.
func tessellateBodyPatch(ctx context.Context, b *Body, pp bodyPatchPayload, chord float64, verify Verification) (*Mesh, error) {
	if prism, ok := bodyPatchCircularPrismSource(pp); ok {
		return tessellateBodyPatchCircularPrism(ctx, b, pp, prism, chord, verify)
	}
	faces := b.Faces()
	// curveBound applies only to Circle3 and Arc3. A Line3 follows its end
	// vertices; the cap-band miter whose Line3 tag holds a curved locus has
	// a ruled face with nonzero normalBound, so this gate rejects that case.
	if !stitchAllTetrahedronEligible(faces) {
		for _, f := range faces {
			if _, ok := f.surface.(Plane); !ok {
				return nil, fmt.Errorf(`%w: this evaluator has no shared chording for a Body.Patch result's %T face`,
					ErrUnsupported, f.Surface())
			}
			if f.normalBound != 0 {
				return nil, fmt.Errorf(`%w: a Body.Patch result's planar face has no zero-bound normal`, ErrUnsupported)
			}
			for _, l := range f.loops {
				for _, ce := range l.coedges {
					if _, ok := ce.edge.curve.(Line3); !ok {
						return nil, fmt.Errorf(`%w: this evaluator has no shared chording for a Body.Patch result's %T edge`,
							ErrUnsupported, ce.edge.curve)
					}
				}
			}
		}
		return nil, fmt.Errorf(`%w: this evaluator cannot triangulate the Body.Patch result`, ErrUnsupported)
	}
	vertices := b.Vertices()
	mesh := &Mesh{vertices: make([]r3.Vec, len(vertices))}
	classOf := make(map[*Vertex]int, len(vertices))
	store := make([]float64, len(vertices))
	budget := proofbound.NewWorkBudget(ctx)
	for i, v := range vertices {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		mesh.vertices[i] = v.position
		classOf[v] = i
		store[i] = v.bound.Base()
	}
	if _, err := tessellation.StoreMax(store); err != nil {
		return nil, err
	}
	var err error
	mesh.triangles, mesh.source, err = triangulateStitchFaces(ctx, faces, classOf)
	if err != nil {
		return nil, err
	}
	if err := orientBodyPatchFilledFaces(mesh, b, classOf, budget); err != nil {
		return nil, err
	}
	if pp.xform.IsReflection() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	if err := requireCapBlendFacetAreas(mesh, "Body.Patch sheet"); err != nil {
		return nil, err
	}
	if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := loftmesh.LoftCrossingAudit(budget, mesh.vertices, mesh.triangles); err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, ctxErr
			}
			return nil, fmt.Errorf(`%w: a Body.Patch sheet's facets are not proven embedded: %s`, ErrUnsupported, err)
		}
	}
	axial := make(map[*Face]float64, len(faces))
	for _, f := range faces {
		axial[f] = f.axialDelta
	}
	if err := composeFaceBounds(mesh, nil, axial, store, 0); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return mesh, nil
	}
	for i, tri := range mesh.triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		face := mesh.source[i]
		delta := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		delta = proofbound.AbsSumUpper(delta, face.axialDelta)
		if delta == 0 {
			continue
		}
		a, bb, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
			proofbound.PerturbedTriangleAreaAllow(a, bb, c, delta))
	}
	if proofbound.IsNonFinite(mesh.areaSlack) {
		return nil, fmt.Errorf(`%w: a Body.Patch result states no finite area bound`, ErrUnsupported)
	}
	return mesh, nil
}

// orientBodyPatchFilledFaces makes each new fill's mesh boundary run opposite
// its source face's mesh boundary. The source face already carries its
// feature's outward sense, including any reflection before Body.Patch. The
// patch's new plane frame can have the opposite local handedness, so its
// triangle winding is settled by the shared mesh edge, not that frame.
func orientBodyPatchFilledFaces(m *Mesh, b *Body, classOf map[*Vertex]int, budget *proofbound.WorkBudget) error {
	byFace := make(map[*Face][]int, len(b.Faces()))
	for i, f := range m.source {
		byFace[f] = append(byFace[f], i)
	}
	for _, f := range b.Faces() {
		if err := budget.Step(); err != nil {
			return err
		}
		if len(f.origins) != 1 || f.origins[0].producer != b.origin.producer || f.origins[0].Role != rolePatch {
			continue
		}
		if len(f.loops) == 0 || len(f.loops[0].coedges) == 0 {
			return fmt.Errorf(`%w: a Body.Patch fill has no boundary walk to orient`, ErrDegenerate)
		}
		ce := f.loops[0].coedges[0]
		start, end := classOf[ce.Start()], classOf[ce.End()]
		var source *Face
		for _, adjacent := range ce.edge.faces {
			if adjacent != f {
				source = adjacent
				break
			}
		}
		if source == nil {
			return fmt.Errorf(`%w: a Body.Patch fill has no source face on its rim`, ErrDegenerate)
		}
		patchSense := bodyPatchMeshEdgeSense(m, byFace[f], start, end)
		sourceSense := bodyPatchMeshEdgeSense(m, byFace[source], start, end)
		if patchSense == 0 || sourceSense == 0 {
			return fmt.Errorf(`%w: a Body.Patch rim has no matching source and fill mesh edges`, ErrUnsupported)
		}
		if patchSense != sourceSense {
			continue
		}
		for _, ti := range byFace[f] {
			m.triangles[ti][1], m.triangles[ti][2] = m.triangles[ti][2], m.triangles[ti][1]
		}
	}
	return nil
}

func bodyPatchMeshEdgeSense(m *Mesh, tris []int, a, b int) int {
	for _, ti := range tris {
		tri := m.triangles[ti]
		for j := range 3 {
			start, end := tri[j], tri[(j+1)%3]
			if start == a && end == b {
				return 1
			}
			if start == b && end == a {
				return -1
			}
		}
	}
	return 0
}
