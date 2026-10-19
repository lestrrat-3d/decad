package draftrecord

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file is docs/draft-design.md Table DD row DD1: a draft body's
// tessellation and its occupied-volume proof. A draft body is built as the
// cap-loop band of docs/modify-reach-design.md §8.3 with no straight slab
// (Payload.Band), so it is meshed by the cap-loop chamfer tessellator over
// that same view (capblend.TessellateMesh, docs/tessellation-reach-design.md
// §7) with four differences, each stated where it applies:
//
//   - there is no trimmed side wall, and the band's side ring is the near
//     cap's rim, written once (capblend.DraftNearRing) and read by both the near cap
//     and the wall band;
//   - a wall patch carries the prism's side(i, j) role, and the band has no
//     apex patch, because every reflex line-line corner is a miter
//     (capblend.Payload.OffsetJoins);
//   - the slice-wise chord term integrates over the whole sweep height with
//     both ends' displacements (capblend.Payload.DraftBandHeightUpper);
//   - a subset draft's kept wall reads a zero setback (offset2d.DraftWalkAmounts,
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

// Tessellate meshes body, whose payload is dp, through its band view. The
// band view carries no loft source, so the cap-blend build's own proofs are
// the whole mesh proof. verifyAll decides whether the cap samples' motion,
// the per-vertex motion and the occupied-volume proof are built; the face
// bounds and AreaSlack are composed at every level.
func Tessellate(ctx context.Context, body *topo.Body, dp Payload, chord float64, verifyAll bool) (*topo.Mesh, error) {
	if len(dp.Patches) == 0 || len(dp.BandDelta) == 0 {
		return nil, fmt.Errorf(`%w: the tapered extrude's payload states no wall band geometry`, decaderr.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	mesh := new(topo.Mesh)
	if _, _, err := capblend.TessellateMesh(ctx, body, dp.Band(), chord, verifyAll, mesh); err != nil {
		return nil, err
	}
	return mesh, nil
}

// TessellateTwoSided meshes each closed half of body, whose payload is dp,
// through Tessellate, removes their middle caps, and identifies identical
// middle vertices. Each half is rebuilt under env. A mismatch in rim sampling
// leaves no watertight union to publish.
func TessellateTwoSided(ctx context.Context, body *topo.Body, env topo.Env, dp TwoSidedPayload,
	chord float64, verifyAll bool) (*topo.Mesh, error) {
	budget := proofbound.NewWorkBudget(ctx)
	byRole := make(map[string]*topo.Face)
	for _, face := range topobuild.Faces(body) {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		for _, origin := range face.Origins {
			byRole[origin.Role] = face
		}
	}
	parts := []Payload{dp.Negative, dp.Positive}
	out := new(topo.Mesh)
	vertexIndex := make(map[[3]float64]int)
	var bounds []float64
	for partIndex, payload := range parts {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		part, err := Build(ctx, env, payload)
		if err != nil {
			return nil, err
		}
		partPayload, ok := part.Payload.(Payload)
		if !ok {
			return nil, fmt.Errorf(`%w: a two-sided taper half lost its draft record`, decaderr.ErrDegenerate)
		}
		mesh, err := Tessellate(ctx, part, partPayload, chord, verifyAll)
		if err != nil {
			return nil, err
		}
		partBounds, err := topobuild.VertexBounds(mesh)
		if err != nil {
			return nil, err
		}
		indices := make([]int, len(mesh.Vertices))
		for i, v := range mesh.Vertices {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			key := [3]float64{v.X, v.Y, v.Z}
			if found, ok := vertexIndex[key]; ok {
				indices[i] = found
				bounds[found] = math.Max(bounds[found], partBounds[i])
				continue
			}
			indices[i] = len(out.Vertices)
			vertexIndex[key] = indices[i]
			out.Vertices = append(out.Vertices, v)
			bounds = append(bounds, partBounds[i])
		}
		internalCap := provenance.RoleCapEnd
		if partIndex == 1 {
			internalCap = provenance.RoleCapStart
		}
		for i, tri := range mesh.Triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			face := mesh.Source[i]
			if len(face.Origins) == 0 {
				return nil, fmt.Errorf(`%w: a draft mesh facet has no source role`, decaderr.ErrDegenerate)
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
				return nil, fmt.Errorf(`%w: the two-sided draft has no published face for role %q`, decaderr.ErrDegenerate, role)
			}
			topobuild.AddTriangle(out, [3]int{indices[tri[0]], indices[tri[1]], indices[tri[2]]}, published)
			bound, ok := mesh.FaceBound[face]
			if !ok {
				return nil, fmt.Errorf(`%w: a draft mesh source face has no displacement bound`, decaderr.ErrDegenerate)
			}
			topobuild.SetFaceBound(out, published, bound)
		}
		out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack, mesh.AreaSlack)
		out.VolSymDiff = proofbound.AbsSumUpper(out.VolSymDiff, mesh.VolSymDiff)
		if partIndex == 0 {
			out.SymDiffOK = mesh.SymDiffOK
		} else {
			out.SymDiffOK = out.SymDiffOK && mesh.SymDiffOK
		}
	}
	topobuild.SetVertexBounds(out, bounds)
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(out.Triangles)); err != nil {
		return nil, fmt.Errorf("two-sided draft middle rim: %w", err)
	}
	return out, budget.Err()
}
