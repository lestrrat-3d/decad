package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// tessellateTwoSidedDraft meshes each closed half through the ordinary draft
// tessellator, removes their middle caps, and identifies identical middle
// vertices. A mismatch in rim sampling leaves no watertight union to publish.
func tessellateTwoSidedDraft(ctx context.Context, body *Body, dp twoSidedDraftPayload, chord float64, verify Verification) (*Mesh, error) {
	budget := proofbound.NewWorkBudget(ctx)
	byRole := make(map[string]*Face)
	for _, face := range body.Faces() {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		for _, origin := range face.origins {
			byRole[origin.Role] = face
		}
	}
	parts := []draftPayload{dp.negative, dp.positive}
	var out Mesh
	vertexIndex := make(map[[3]float64]int)
	var bounds []float64
	for partIndex, payload := range parts {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		part, err := evalDraftContext(ctx, body.doc, body.origin.producer, payload)
		if err != nil {
			return nil, err
		}
		mesh, err := tessellateDraft(ctx, part, part.payload.(draftPayload), chord, verify)
		if err != nil {
			return nil, err
		}
		partBounds, err := mesh.vertexBounds()
		if err != nil {
			return nil, err
		}
		indices := make([]int, len(mesh.vertices))
		for i, v := range mesh.vertices {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			key := [3]float64{v.X, v.Y, v.Z}
			if found, ok := vertexIndex[key]; ok {
				indices[i] = found
				bounds[found] = math.Max(bounds[found], partBounds[i])
				continue
			}
			indices[i] = len(out.vertices)
			vertexIndex[key] = indices[i]
			out.vertices = append(out.vertices, v)
			bounds = append(bounds, partBounds[i])
		}
		internalCap := roleCapEnd
		if partIndex == 1 {
			internalCap = roleCapStart
		}
		for i, tri := range mesh.triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			face := mesh.source[i]
			if len(face.origins) == 0 {
				return nil, fmt.Errorf(`%w: a draft mesh facet has no source role`, ErrDegenerate)
			}
			role := face.origins[0].Role
			if role == internalCap {
				continue
			}
			if suffix, ok := strings.CutPrefix(role, "side("); ok {
				role = fmt.Sprintf("side(%d,%s", partIndex, suffix)
			}
			published := byRole[role]
			if published == nil {
				return nil, fmt.Errorf(`%w: the two-sided draft has no published face for role %q`, ErrDegenerate, role)
			}
			out.addTriangle([3]int{indices[tri[0]], indices[tri[1]], indices[tri[2]]}, published)
			if bound, ok := mesh.sourceBound(face); ok {
				out.setFaceBound(published, bound)
			} else {
				return nil, fmt.Errorf(`%w: a draft mesh source face has no displacement bound`, ErrDegenerate)
			}
		}
		out.areaSlack = proofbound.AbsSumUpper(out.areaSlack, mesh.areaSlack)
		out.volSymDiff = proofbound.AbsSumUpper(out.volSymDiff, mesh.volSymDiff)
		if partIndex == 0 {
			out.symDiffOK = mesh.symDiffOK
		} else {
			out.symDiffOK = out.symDiffOK && mesh.symDiffOK
		}
	}
	out.setVertexBounds(bounds)
	if err := liftTessellationError(tessellation.RequireClosedMesh(out.triangles)); err != nil {
		return nil, fmt.Errorf("two-sided draft middle rim: %w", err)
	}
	return &out, budget.Err()
}
