package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file is docs/draft-design.md Table DD row DD1: a draft body's
// tessellation and its occupied-volume proof. A draft body is built as the
// cap-loop band of docs/modify-reach-design.md §8.3 with no straight slab
// (draftPayload.Band), so it is meshed by the cap-loop chamfer tessellator
// over that same view (tessellate_capblend.go, docs/tessellation-reach-design.md
// §7) with four differences, each stated where it applies:
//
//   - there is no trimmed side wall, and the band's side ring is the near
//     cap's rim, written once (capblend.DraftNearRing) and read by both the near cap
//     and the wall band;
//   - a wall patch carries the prism's side(i, j) role, and the band has no
//     apex patch, because every reflex line-line corner is a miter
//     (capBlendPayload.offsetJoins);
//   - the slice-wise chord term integrates over the whole sweep height with
//     both ends' displacements (capblend.Payload.DraftBandHeightUpper);
//   - a subset draft's kept wall reads a zero setback (walkAmounts,
//     docs/draft-design.md §10.2) in the cap radius, the band's radius
//     rounding and the cap samples' motion, where a chamfer reads its one
//     setback.
//
// The occupied-volume proof is §7's admission and its two legs, unchanged: a
// sharp offset's corner loci are affine in the amount (docs/draft-design.md
// §2), so a band whose every corner is a line-line miter, convex or reflex,
// or an exactly G1 join, and a whole circle, reproduces the exact offset
// family slice by slice. Booleans, export, interference, mass properties and
// Patterned read the mesh this file returns (Table DD rows DD2–DD4, DD10,
// DD11, DD13).

// tessellateDraft meshes a draft body through its band view.
func tessellateDraft(ctx context.Context, b *Body, dp draftPayload, chord float64, verify Verification) (*Mesh, error) {
	if len(dp.Patches) == 0 || len(dp.BandDelta) == 0 {
		return nil, fmt.Errorf(`%w: the tapered extrude's payload states no wall band geometry`, ErrDegenerate)
	}
	return tessellateCapBlend(ctx, b, dp.Band(), chord, verify)
}

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
		for _, origin := range face.n.Origins {
			byRole[origin.Role] = face
		}
	}
	parts := []draftPayload{dp.Negative, dp.Positive}
	out := bornMesh(new(Mesh))
	vertexIndex := make(map[[3]float64]int)
	var bounds []float64
	for partIndex, payload := range parts {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		part, err := bodyResult(draftrecord.Build(ctx, envOf(body.n.Doc, body.n.Origin.Producer), payload))
		if err != nil {
			return nil, err
		}
		partPayload, ok := part.n.Payload.(draftPayload)
		if !ok {
			return nil, fmt.Errorf(`%w: a two-sided taper half lost its draft record`, ErrDegenerate)
		}
		mesh, err := tessellateDraft(ctx, part, partPayload, chord, verify)
		if err != nil {
			return nil, err
		}
		partBounds, err := topobuild.VertexBounds(&mesh.n)
		if err != nil {
			return nil, err
		}
		indices := make([]int, len(mesh.n.Vertices))
		for i, v := range mesh.n.Vertices {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			key := [3]float64{v.X, v.Y, v.Z}
			if found, ok := vertexIndex[key]; ok {
				indices[i] = found
				bounds[found] = math.Max(bounds[found], partBounds[i])
				continue
			}
			indices[i] = len(out.n.Vertices)
			vertexIndex[key] = indices[i]
			out.n.Vertices = append(out.n.Vertices, v)
			bounds = append(bounds, partBounds[i])
		}
		internalCap := provenance.RoleCapEnd
		if partIndex == 1 {
			internalCap = provenance.RoleCapStart
		}
		for i, tri := range mesh.n.Triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			face := mesh.n.Source[i]
			if len(face.Origins) == 0 {
				return nil, fmt.Errorf(`%w: a draft mesh facet has no source role`, ErrDegenerate)
			}
			role := face.Origins[0].Role
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
			topobuild.AddTriangle(&out.n, [3]int{indices[tri[0]], indices[tri[1]], indices[tri[2]]}, faceNode(published))
			if bound, ok := mesh.n.FaceBound[face]; ok {
				topobuild.SetFaceBound(&out.n, faceNode(published), bound)
			} else {
				return nil, fmt.Errorf(`%w: a draft mesh source face has no displacement bound`, ErrDegenerate)
			}
		}
		out.n.AreaSlack = proofbound.AbsSumUpper(out.n.AreaSlack, mesh.n.AreaSlack)
		out.n.VolSymDiff = proofbound.AbsSumUpper(out.n.VolSymDiff, mesh.n.VolSymDiff)
		if partIndex == 0 {
			out.n.SymDiffOK = mesh.n.SymDiffOK
		} else {
			out.n.SymDiffOK = out.n.SymDiffOK && mesh.n.SymDiffOK
		}
	}
	topobuild.SetVertexBounds(&out.n, bounds)
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(out.n.Triangles)); err != nil {
		return nil, fmt.Errorf("two-sided draft middle rim: %w", err)
	}
	return out, budget.Err()
}
