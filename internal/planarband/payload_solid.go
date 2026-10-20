package planarband

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// PrismSolid lifts an all-LineSeg section through its frame and sweep
// levels. The caps reuse the tessellator's triangulation, which this function
// then proves: every cap triangle is exactly counterclockwise and, through the
// closed-mesh audit, the caps' boundary chain is the section loops, so the
// triangles tile the section exactly. Each triangle records its face from
// the roles in faceIndex: the start cap at z0, the end cap at z1, and
// side(loop, segment) for a wall; a missing role leaves Faces unset.
func PrismSolid(ctx context.Context, budget *proofbound.WorkBudget,
	pp prismrecord.Payload, faceIndex map[string]int) (planar.PlanarSolid, bool, error) {
	return planarsnapshot.PrismSolid(ctx, budget, planarsnapshot.PrismInput{
		Outer: pp.Profile.Outer, Holes: pp.Profile.Holes,
		Frame: pp.Frame, Transform: pp.Xform, Z0: pp.Z0, Z1: pp.Z1,
		SectionDelta: pp.SectionDelta, Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta,
		SurfaceResult: pp.SurfaceResult, CapStartRole: provenance.RoleCapStart, CapEndRole: provenance.RoleCapEnd,
	}, faceIndex, triangulation.Triangulate)
}

// FacetedSolid reads a Boolean's held mesh. A zero-bound mesh is its
// exact boundary. A translation-only placement reads the saved exact source
// mesh moved by the exact placement translation, after a reject-only check
// that every rebuilt vertex lies within the published bound of it. Any other
// positive-bound mesh is read as held, with its mesh bound returned as the
// displacement δ of §10.4.
func FacetedSolid(budget *proofbound.WorkBudget, pp facetedrecord.Payload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	return planarsnapshot.FacetedSolid(budget, planarsnapshot.FacetedInput{
		Verts: pp.Verts, Tris: pp.Tris, FaceOf: pp.FaceOf,
		ExactSourceVerts: pp.ExactSourceVerts, ExactSourceTris: pp.ExactSourceTris,
		MeshBound: pp.MeshBound, VolSymDiff: pp.VolSymDiff, Transform: pp.Xform,
	})
}
