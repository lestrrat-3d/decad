package decad

import (
	"context"
	"fmt"
	"math"
)

// This file is docs/surface-design.md §13.4's chain-fed tessellation
// increment: a chain-fed prism ribbon's own mesh, and
// docs/tessellation-design.md §1.2's manifold-with-boundary audit, which this
// sheet runs in the closed-mesh audit's place. A chain-fed feature mints no
// cap at all — §13's own decision, "no face to close anything" — so there is
// nothing this restatement drops the way tessellate_loft.go's own
// restatement drops the cap triangles by provenance range: every wall this
// file emits is a mesh face, and the ribbon's free boundary is both rims plus
// one sweep edge per free end (Table G).
//
// This increment tessellates a chain of LINE segments only, Table G's Plane
// row: each wall is already an exact planar quad the build placed directly on
// the recorded boundary, so there is no chord to choose and no tolerance for
// one to bind — the identical reading tessellate_loft.go's own doc comment
// states for a loft's exact restatement. A chain holding a curved
// (CircleSeg/ArcSeg fragment) or free-form segment builds a Cylinder or
// NURBSSurface wall this file has no chording arm for yet, staged for the
// increment that chords those walls too. Both ExtrudeChain and SweepChain's
// one-span straight reduction build through evalChainExtrudeContext, so both
// a chainPayload and the chainPayload a chainSweepPayload wraps reach this
// function unchanged.

// tessellateChain restates a chain-fed ribbon's own wall topology as a Mesh.
// Every wall Face is already an exact planar quad — four corner Vertex
// objects evalChainExtrudeContext placed directly on the recorded boundary —
// so the mesh is read straight off Face -> Loop -> CoEdge -> Vertex, with no
// chording decision of its own. A straight wall's own published
// Face.reversed is always false (buildWallGeometry's default-case return,
// prism_build.go), so this restatement's only degree of freedom is a
// REFLECTED placement: the walk order every wall's coedges are built in
// (bottomV[i] -> bottomV[i+1] -> topV[i+1] -> topV[i]) is fixed independent
// of the transform, so lifting it through a determinant-negative one flips
// handedness and the naive triangulation winds backward — exactly the case
// tessellateBodyContext's own prism path corrects with the identical final
// swap over its own 2D-chorded triangles.
func tessellateChain(ctx context.Context, b *Body, pp chainPayload) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	reflected := pp.xform.IsReflection()

	var mesh Mesh
	index := map[*Vertex]int{}
	var vertexStore []float64
	meshIndexOf := func(v *Vertex) int {
		if i, ok := index[v]; ok {
			return i
		}
		i := len(mesh.vertices)
		mesh.vertices = append(mesh.vertices, v.position)
		vertexStore = append(vertexStore, v.bound.Mag())
		index[v] = i
		return i
	}

	budget := newWorkBudget(ctx)
	for _, f := range b.Faces() {
		if err := budget.step(); err != nil {
			return nil, err
		}
		if _, ok := f.Surface().(Plane); !ok {
			return nil, fmt.Errorf(`%w: this evaluator has no chording arm for a chain-fed %T wall`, ErrUnsupported, f.Surface())
		}
		loops := f.Loops()
		if len(loops) != 1 {
			return nil, fmt.Errorf(`%w: a chain-fed wall face carries %d loops, not the one its own build mints`, ErrDegenerate, len(loops))
		}
		coedges := loops[0].CoEdges()
		if len(coedges) != 4 {
			return nil, fmt.Errorf(`%w: a chain-fed wall face carries %d coedges, not the four-corner quad its own build mints`, ErrDegenerate, len(coedges))
		}
		var corner [4]int
		var cornerDelta float64
		for i, ce := range coedges {
			v := ce.Start()
			corner[i] = meshIndexOf(v)
			cornerDelta = math.Max(cornerDelta, v.bound.Mag())
		}
		v0, v1, v2, v3 := corner[0], corner[1], corner[2], corner[3]
		tri1 := [3]int{v0, v1, v2}
		tri2 := [3]int{v0, v2, v3}
		if reflected {
			// The identical final swap tessellateBodyContext's own prism path
			// applies to its own 2D-chorded triangles, for the identical
			// reason (this file's own doc comment).
			tri1[1], tri1[2] = tri1[2], tri1[1]
			tri2[1], tri2[2] = tri2[2], tri2[1]
		}
		mesh.addTriangle(tri1, f)
		mesh.addTriangle(tri2, f)
		mesh.setFaceBound(f, upRound(cornerDelta+pp.sectionDelta))
	}
	if err := budget.err(); err != nil {
		return nil, err
	}

	mesh.areaSlack = meshStoreAreaAllow(&mesh, vertexStore)
	if pp.sectionDelta > 0 {
		walks := 0
		for _, chain := range pp.chains {
			walks += len(chain.Segments)
		}
		wallMove := productUpper(sectionDisplacementLength(pp.sectionDelta, walks), math.Abs(pp.z1-pp.z0))
		mesh.areaSlack = absSumUpper(mesh.areaSlack, wallMove)
	}

	// A chain-fed body is a BodySheet by construction, always (Table G), so
	// the manifold-with-boundary audit runs unconditionally rather than
	// branching on b.Kind() the way requireMeshAudit's generic dispatch does
	// for a payload that can build either kind.
	if err := requireSheetMesh(ctx, b, &mesh); err != nil {
		return nil, err
	}
	if err := requireSheetVertexLinks(ctx, &mesh); err != nil {
		return nil, err
	}
	// A sheet encloses no region, so there is no occupied volume to prove:
	// volSymDiff and symDiffOK stay at their zero value, the identical
	// omission tessellateBodyContext's own prism-sheet branch states
	// (docs/surface-design.md §10).
	return &mesh, nil
}
