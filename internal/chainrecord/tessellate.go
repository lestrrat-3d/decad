package chainrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file is docs/surface-design.md §13.4's chain-fed tessellation
// increment: a chain-fed prism ribbon's own mesh, and
// docs/tessellation-design.md §1.2's manifold-with-boundary audit, which this
// sheet runs in the closed-mesh audit's place. A chain-fed feature mints no
// cap at all — §13's own decision, "no face to close anything" — so there is
// nothing this restatement drops the way internal/loftmesh.RestateLoft's
// restatement drops the cap triangles by provenance range: every wall this
// file emits is a mesh face, and the ribbon's free boundary is both rims plus
// one sweep edge per free end (Table G).
//
// This increment tessellates a chain of LINE segments only, Table G's Plane
// row: each wall is already an exact planar quad the build placed directly on
// the recorded boundary, so there is no chord to choose and no tolerance for
// one to bind — the identical reading RestateLoft's own doc comment
// states for a loft's exact restatement. A chain holding a curved
// (CircleSeg/ArcSeg fragment) or free-form segment builds a Cylinder or
// NURBSSurface wall this file has no chording arm for yet, staged for the
// increment that chords those walls too. Both chain Extrude and the chain
// Sweep's one-span straight reduction build through Build, so both a Payload
// and the Payload a chain Sweep's payload wraps reach Tessellate unchanged.

// Tessellate restates body's own wall topology, whose payload is pp, as a
// mesh node. Every wall face is already an exact planar quad — four corner
// vertices Build placed directly on the recorded boundary — so the mesh is
// read straight off Face -> Loop -> CoEdge -> Vertex, with no chording
// decision of its own. A straight wall's own published topo.Face.Reversed is
// always false (prismrecord.BuildWallGeometry's default-case return,
// internal/prismrecord/build.go), so this restatement's only degree of freedom
// is a REFLECTED placement: the walk order every wall's coedges are built in
// (bottomV[i] -> bottomV[i+1] -> topV[i+1] -> topV[i]) is fixed independent
// of the transform, so lifting it through a determinant-negative one flips
// handedness and the naive triangulation winds backward — exactly the case
// the prism mesher (internal/prismtess) corrects with the identical final
// swap over its own 2D-chorded triangles. formatSurface names a non-planar
// wall's surface in its public form for the refusal.
func Tessellate(ctx context.Context, body *topo.Body, pp Payload,
	formatSurface func(surfacegeom.Surface) any) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	reflected := pp.Xform.IsReflection()

	mesh := new(topo.Mesh)
	index := map[*topo.Vertex]int{}
	var vertexStore []float64
	meshIndexOf := func(v *topo.Vertex) int {
		if i, ok := index[v]; ok {
			return i
		}
		i := len(mesh.Vertices)
		mesh.Vertices = append(mesh.Vertices, v.Position)
		vertexStore = append(vertexStore, v.Bound.Mag())
		index[v] = i
		return i
	}

	budget := proofbound.NewWorkBudget(ctx)
	for _, f := range topobuild.Faces(body) {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if _, ok := f.Surface.(surfacegeom.Plane); !ok {
			return nil, fmt.Errorf(`%w: this evaluator has no chording arm for a chain-fed %T wall`,
				decaderr.ErrUnsupported, formatSurface(f.Surface))
		}
		if len(f.Loops) != 1 {
			return nil, fmt.Errorf(`%w: a chain-fed wall face carries %d loops, not the one its own build mints`,
				decaderr.ErrDegenerate, len(f.Loops))
		}
		coedges := f.Loops[0].CoEdges
		if len(coedges) != 4 {
			return nil, fmt.Errorf(`%w: a chain-fed wall face carries %d coedges, not the four-corner quad its own build mints`,
				decaderr.ErrDegenerate, len(coedges))
		}
		var corner [4]int
		var cornerDelta float64
		for i, ce := range coedges {
			v := topobuild.CoEdgeStart(ce)
			corner[i] = meshIndexOf(v)
			cornerDelta = math.Max(cornerDelta, v.Bound.Mag())
		}
		v0, v1, v2, v3 := corner[0], corner[1], corner[2], corner[3]
		tri1 := [3]int{v0, v1, v2}
		tri2 := [3]int{v0, v2, v3}
		if reflected {
			// The identical final swap the prism mesher applies to its own
			// 2D-chorded triangles, for the identical reason (this
			// function's own doc comment).
			tri1[1], tri1[2] = tri1[2], tri1[1]
			tri2[1], tri2[2] = tri2[2], tri2[1]
		}
		topobuild.AddTriangle(mesh, tri1, f)
		topobuild.AddTriangle(mesh, tri2, f)
		topobuild.SetFaceBound(mesh, f, proofbound.UpRound(cornerDelta+pp.SectionDelta))
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}

	mesh.AreaSlack = tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, vertexStore)
	if pp.SectionDelta > 0 {
		walks := 0
		for _, chain := range pp.Chains {
			walks += len(chain.Segments)
		}
		wallMove := proofbound.ProductUpper(proofbound.SectionDisplacementLength(pp.SectionDelta, walks), math.Abs(pp.Z1-pp.Z0))
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, wallMove)
	}

	// A chain-fed body is a sheet by construction, always (Table G), so the
	// sheet audit runs unconditionally rather than branching on the body's
	// kind the way a payload that can build either kind does.
	if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
		return nil, err
	}
	// A sheet encloses no region, so there is no occupied volume to prove:
	// VolSymDiff and SymDiffOK stay at their zero value, the identical
	// omission the prism mesher's sheet branch states
	// (docs/surface-design.md §10).
	return mesh, nil
}
