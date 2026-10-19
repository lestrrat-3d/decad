package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/compositemesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// tessellateCompositeSweep assembles one indexed profile grid across the
// reduced spans. The station adapter admits recorded lines and circular walks
// and fitted splines whose span meshes expose the same profile parameter at
// each join. It maps those samples by profile index, never spatial proximity.
func tessellateCompositeSweep(ctx context.Context, b *Body, sp sweepPayload, chord float64, verify Verification) (*Mesh, error) {
	counts, profileSamples, freeformTarget, err := compositemesh.PlanProfileCounts(ctx, sp, chord, revolvetess.ResolveRevolveWithLimit)
	if err != nil {
		return nil, err
	}
	byRole := map[string]*topo.Face{}
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			byRole[origin.Role] = faceNode(f)
		}
	}
	mesh := bornMesh(&Mesh{})
	spanWork := sweeprecord.NewCompositeWork(sp.Prism.Profile)
	budget := proofbound.NewWorkBudget(ctx)
	assembly := compositemesh.NewAssembly(&mesh.n, byRole, len(sp.Spans), profileSamples,
		sp.ClosedComposite, sp.SurfaceResult, freeformTarget, budget,
		revolvetess.ResolveRevolveWithLimit, verify >= VerifyBoundary, verify >= VerifyAll)
	for i := range sp.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		span := sp.Spans[i]
		part, err := bodyResult(span.Build(ctx, envOf(b.n.Doc, b.n.Origin.Producer), spanWork))
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		// span.Build restores path cap names for topology pairing. The reduced
		// revolve tessellator needs its own cap names while it makes its proof.
		if span.Arc && span.ReverseArcCaps {
			sweeprecord.RestoreArcCapRoles(bodyNode(part), true)
		}
		var piece *Mesh
		if span.Arc {
			floor, floorErr := compositemesh.RevolveFloor(ctx, span.Revolve, counts, freeformTarget, revolvetess.ResolveRevolveWithLimit)
			if floorErr != nil {
				return nil, fmt.Errorf(`sweep path span %d: %w`, i, floorErr)
			}
			var node *topo.Mesh
			node, err = revolvetess.TessellateRevolveWithCountFloor(
				ctx, &part.n, span.Revolve, chord, revolvetess.Level(verify), nil, floor, freeformTarget)
			if err == nil {
				piece = bornMesh(&Mesh{n: *node})
			}
		} else {
			piece, err = tessellatePrismWithCountFloor(ctx, part, span.Prism, prismWallRole, chord, verify,
				func(loop, seg int) int { return counts[compositemesh.StationKey{Loop: loop, Segment: seg}] }, freeformTarget)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		vertexBounds, err := topobuild.VertexBounds(&piece.n)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		if err := assembly.Append(ctx, i, span, &piece.n, vertexBounds); err != nil {
			return nil, err
		}
	}
	if err := assembly.BeforeBodyAudit(); err != nil {
		return nil, err
	}
	if err := requireMeshAudit(ctx, sp.SurfaceResult, b, mesh); err != nil {
		return nil, err
	}
	if err := assembly.AfterBodyAudit(ctx); err != nil {
		return nil, err
	}
	return mesh, nil
}
