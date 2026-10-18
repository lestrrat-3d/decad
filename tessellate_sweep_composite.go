package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/compositemesh"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
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
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			byRole[origin.Role] = f
		}
	}
	mesh := bornMesh(&Mesh{})
	spanWork := sweeprecord.NewCompositeWork(sp.Prism.Profile)
	budget := proofbound.NewWorkBudget(ctx)
	coordinateBound := 0.0
	triangleSpan := []int{}
	stations := len(sp.Spans) + 1
	if sp.ClosedComposite {
		stations = len(sp.Spans)
	}
	join := make([][]int, stations)
	joinBound := make([][]float64, stations)
	for i := range join {
		join[i] = make([]int, profileSamples)
		joinBound[i] = make([]float64, profileSamples)
		for j := range join[i] {
			join[i][j] = -1
		}
	}
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
		vertexBounds, err := piece.vertexBounds()
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		start, end, spanCoordinateBound, err := compositemesh.SpanRings(ctx, span, &piece.n, profileSamples, freeformTarget, revolvetess.ResolveRevolveWithLimit)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		local := make([]int, len(piece.n.Vertices))
		for j := range local {
			local[j] = -1
		}
		maxMove := 0.0
		endSection := i + 1
		if sp.ClosedComposite && endSection == len(sp.Spans) {
			endSection = 0
		}
		for station := range profileSamples {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			for _, boundary := range []struct {
				section int
				index   int
			}{{i, start[station]}, {endSection, end[station]}} {
				shared := join[boundary.section][station]
				if shared < 0 {
					shared = len(mesh.n.Vertices)
					mesh.n.Vertices = append(mesh.n.Vertices, piece.n.Vertices[boundary.index])
					join[boundary.section][station] = shared
					joinBound[boundary.section][station] = vertexBounds[boundary.index]
				} else if mesh.n.Vertices[shared] != piece.n.Vertices[boundary.index] {
					delta2, ok := proofarith.DySquaredDistance3(
						mesh.n.Vertices[shared].X, mesh.n.Vertices[shared].Y, mesh.n.Vertices[shared].Z,
						piece.n.Vertices[boundary.index].X, piece.n.Vertices[boundary.index].Y, piece.n.Vertices[boundary.index].Z)
					if !ok {
						return nil, fmt.Errorf(`%w: a shared sweep station has no finite displacement`, ErrUnsupported)
					}
					move := proofarith.DySqrtUp(delta2)
					if proofbound.IsNonFinite(move) ||
						move > proofbound.AbsSumUpper(joinBound[boundary.section][station], vertexBounds[boundary.index]) {
						return nil, fmt.Errorf(`%w: adjacent sweep spans disagree on profile station %d at section %d`, ErrUnsupported, station, boundary.section)
					}
					maxMove = max(maxMove, move)
				}
				local[boundary.index] = shared
			}
		}
		for j, v := range piece.n.Vertices {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if local[j] < 0 {
				local[j] = len(mesh.n.Vertices)
				mesh.n.Vertices = append(mesh.n.Vertices, v)
			}
		}
		coordinateBound = max(coordinateBound, proofbound.AbsSumUpper(spanCoordinateBound, maxMove))
		for t, triangle := range piece.n.Triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			role, err := compositemesh.SpanFaceRole(piece.n.Source[t], i, span.ReverseArcCaps)
			if err != nil {
				return nil, err
			}
			if sp.ClosedComposite && (role == provenance.RoleCapStart || role == provenance.RoleCapEnd) ||
				role == provenance.RoleCapStart && (sp.SurfaceResult || i != 0) ||
				role == provenance.RoleCapEnd && (sp.SurfaceResult || i != len(sp.Spans)-1) {
				continue
			}
			face, ok := byRole[role]
			if !ok {
				return nil, fmt.Errorf(`%w: the composite sweep carries no face for role %q`, ErrDegenerate, role)
			}
			topobuild.AddTriangle(&mesh.n, [3]int{local[triangle[0]], local[triangle[1]], local[triangle[2]]}, faceNode(face))
			if len(mesh.n.Triangles) > compositemesh.MaxFacets {
				return nil, fmt.Errorf(`%w: the composite sweep mesh exceeds the fixed facet ceiling`, ErrUnsupported)
			}
			triangleSpan = append(triangleSpan, i)
			bound, ok := piece.sourceBound(piece.n.Source[t])
			if !ok {
				return nil, fmt.Errorf(`%w: sweep path span %d has no source bound for role %q`, ErrUnsupported, i, role)
			}
			topobuild.SetFaceBound(&mesh.n, faceNode(face), proofbound.AbsSumUpper(bound, maxMove))
		}
		if verify >= VerifyAll {
			if !piece.n.SymDiffOK {
				return nil, fmt.Errorf(`%w: sweep path span %d has no occupied-volume proof`, ErrUnsupported, i)
			}
			areaMove := revolvemesh.CoordinateAreaAllow(piece.n.Vertices, piece.n.Triangles, maxMove)
			volumeMove := proofbound.SweptVolumeAllow(maxMove,
				proofbound.PerturbedAreaUpper(piece.n.Vertices, piece.n.Triangles, maxMove))
			mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, piece.n.AreaSlack, areaMove)
			mesh.n.VolSymDiff = proofbound.AbsSumUpper(mesh.n.VolSymDiff, piece.n.VolSymDiff, volumeMove)
		}
	}
	if proofbound.IsNonFinite(mesh.n.Bound) || proofbound.IsNonFinite(coordinateBound) ||
		proofbound.IsNonFinite(mesh.n.AreaSlack) || proofbound.IsNonFinite(mesh.n.VolSymDiff) {
		return nil, fmt.Errorf(`%w: the composite sweep mesh has no finite proof bound`, ErrUnsupported)
	}
	mesh.n.CoordBound = coordinateBound
	if err := requireMeshAudit(ctx, sp.SurfaceResult, b, mesh); err != nil {
		return nil, err
	}
	if !sp.SurfaceResult {
		if err := topobuild.LiftAuditError(tessellation.RequireVertexLinks(ctx, len(mesh.n.Vertices), mesh.n.Triangles)); err != nil {
			return nil, err
		}
	}
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(auditBudget, mesh.n.Vertices, mesh.n.Triangles, coordinateBound)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := compositesweep.CrossSpanContactAudit(auditBudget, auditTris, mesh.n.Triangles, triangleSpan, coordinateBound); err != nil {
			return nil, err
		}
	}
	if !sp.SurfaceResult && tessellation.OrientationSign(mesh.n.Vertices, mesh.n.Triangles, r3.Vec{}) <= 0 {
		return nil, fmt.Errorf(`%w: the composite sweep mesh does not enclose a positive volume`, ErrUnsupported)
	}
	mesh.n.SymDiffOK = verify >= VerifyAll && !sp.SurfaceResult
	return mesh, nil
}
