package loftmirror

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// Union joins two mirrored Lofts that share one authenticated source into one
// faceted body, recording the join for the later cylinder routes. It reports
// false, with no error, when the pair is not that class.
func Union(ctx context.Context, op meshbool.OperationKind, env topo.Env, mesher verifiedmesh.Mesher,
	a, b *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpUnion || !a.Solid || !b.Solid {
		return nil, false, nil
	}
	lower, ok := a.Payload.(loftrecord.Payload)
	if !ok || lower.Xform != r3.Identity() {
		a, b = b, a
		lower, ok = a.Payload.(loftrecord.Payload)
	}
	upper, upperOK := b.Payload.(loftrecord.Payload)
	if !ok || !upperOK || lower.Xform != r3.Identity() ||
		!lower.AuthenticatedReconstruction || !upper.AuthenticatedReconstruction ||
		lower.SurfaceResult || upper.SurfaceResult || lower.SourceIdentity == nil ||
		lower.SourceIdentity != upper.SourceIdentity {
		return nil, false, nil
	}
	if !lower.MirrorAxialSource() || lower.Frame0 != upper.Frame0 ||
		lower.Frame1 != upper.Frame1 || lower.Plane0 != upper.Plane0 ||
		lower.Plane1 != upper.Plane1 || !slices.Equal(lower.Alignment, upper.Alignment) {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: loft end section has no reflection: %s", decaderr.ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, pair := range [][2]momentinput.Profile{{lower.Profile0, upper.Profile0},
		{lower.Profile1, upper.Profile1}} {
		same, err := momentinput.ExactProfileEqual(budget, pair[0], pair[1])
		if err != nil {
			return nil, true, err
		}
		if !same {
			return nil, false, nil
		}
	}
	// Both ideal Lofts use the same authenticated sections. Their XY-aligned
	// rules have z between the two section planes. The exact reflection fixes
	// the upper cap and maps the lower slab to the opposite side of it, so the
	// true interiors are disjoint and the ideal caps are identical.
	body, err := buildUnion(ctx, env, mesher, a, b, lower, upper)
	return body, true, err
}

func buildUnion(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	lowerBody, upperBody *topo.Body, lower, upper loftrecord.Payload) (*topo.Body, error) {
	n := len(lower.Verts) / 2
	if len(lower.Verts)%2 != 0 || n < 3 || len(upper.Verts) != len(lower.Verts) ||
		lower.Walls != 2*n || upper.Walls != lower.Walls ||
		lower.CapStartCount <= 0 || upper.CapStartCount != lower.CapStartCount ||
		len(lower.Tris) != len(upper.Tris) ||
		lower.Walls+lower.CapStartCount >= len(lower.Tris) {
		return nil, fmt.Errorf("%w: reflected lofts lack matching cap and wall stations", decaderr.ErrUnsupported)
	}
	for i := range n {
		if lower.Verts[n+i] != upper.Verts[n+i] {
			return nil, fmt.Errorf("%w: reflected loft cap stations differ", decaderr.ErrUnsupported)
		}
	}
	if !lower.MirrorCapTrianglesMatch(upper) {
		return nil, fmt.Errorf("%w: reflected loft cap triangles do not oppose", decaderr.ErrUnsupported)
	}
	lowerMesh, err := mesher.VerifyAll(ctx, lowerBody, 0.1)
	if err != nil {
		return nil, err
	}
	upperMesh, err := mesher.VerifyAll(ctx, upperBody, 0.1)
	if err != nil {
		return nil, err
	}
	if !meshMatchesPayload(lowerMesh, lower) ||
		!meshMatchesPayload(upperMesh, upper) ||
		!lowerMesh.BoundaryOK || !upperMesh.BoundaryOK ||
		!lowerMesh.SymDiffOK || !upperMesh.SymDiffOK {
		return nil, fmt.Errorf("%w: reflected loft source mesh has no matching boundary and volume proof", decaderr.ErrUnsupported)
	}
	_, pairDiameter, err := booloperand.PairChord(ctx, lowerBody, upperBody)
	if err != nil {
		return nil, err
	}
	lowerBound, upperBound := lowerMesh.Bound, upperMesh.Bound
	bound := max(lowerBound, upperBound)
	volumeGap := proofbound.AbsSumUpper(lowerMesh.VolSymDiff, upperMesh.VolSymDiff)
	// Each Loft area slack contains the nonnegative error of BOTH of its caps.
	// Removing two copies of the same true and held cap therefore adds at most
	// twice either source's entire area slack to the two wall-area errors.
	areaSlack := proofbound.AbsSumUpper(lowerMesh.AreaSlack, upperMesh.AreaSlack,
		proofbound.ProductUpper(2, max(lowerMesh.AreaSlack, upperMesh.AreaSlack)))
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || bound < 0 || volumeGap < 0 || areaSlack < 0 {
		return nil, fmt.Errorf("%w: reflected loft proof is not finite", decaderr.ErrUnsupported)
	}

	record := &facetedrecord.LoftMirrorJoinRecord{Lower: lowerBody, Upper: upperBody, StationCount: n}
	verts := append(append([]r3.Vec(nil), lower.Verts...), upper.Verts[:n]...)
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = lowerBound
		if i >= n {
			vertexBound[i] = upperBound
		}
		if i >= n && i < 2*n {
			vertexBound[i] = bound
		}
	}
	var tris [][3]int
	var src []int
	var groups []facetedrecord.Group
	groupOf := map[*topo.Face]int{}
	add := func(triangle [3]int, face *topo.Face, capRole string, half, wallIndex int) {
		group, exists := groupOf[face]
		if !exists {
			group = len(groups)
			groupOf[face] = group
			g := facetedrecord.Group{Origins: append([]provenance.Ref(nil), face.Origins...)}
			if capRole != "" {
				g.Origins = append(g.Origins, provenance.Ref{Producer: env.Ref, Role: capRole})
				g.Surface = face.Surface
				g.Planar = true
				g.NormalBound = face.NormalBound
				g.Reversed = face.Reversed
			}
			groups = append(groups, g)
		}
		tris = append(tris, triangle)
		src = append(src, group)
		if wallIndex >= 0 {
			record.WallGroups[half][wallIndex] = group
		} else if capRole == provenance.RoleCapStart {
			record.CapGroups[0] = group
		} else if capRole == provenance.RoleCapEnd {
			record.CapGroups[1] = group
		}
	}
	for half, item := range []struct {
		payload loftrecord.Payload
		mesh    *topo.Mesh
	}{
		{lower, lowerMesh}, {upper, upperMesh},
	} {
		record.WallGroups[half] = make([]int, item.payload.Walls)
		for i := range item.payload.Walls + item.payload.CapStartCount {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			triangle := item.payload.Tris[i]
			if half == 1 {
				for corner, old := range triangle {
					if old < n {
						triangle[corner] = 2*n + old
					}
				}
			}
			role, wallIndex := "", i
			if i >= item.payload.Walls {
				wallIndex = -1
				role = provenance.RoleCapStart
				if half == 1 {
					role = provenance.RoleCapEnd
				}
			}
			add(triangle, item.mesh.Source[i], role, half, wallIndex)
		}
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: reflected loft seam has an invalid vertex link: %s", decaderr.ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris); err != nil {
		return nil, err
	}
	capGroups := record.CapGroups
	fp := facetedrecord.Payload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: pairDiameter, ContactAudited: true,
		LoftMirrorJoin: record, MirrorCapGroups: &capGroups, Xform: r3.Identity()}
	return facetedrecord.Build(ctx, env, fp)
}

func meshMatchesPayload(mesh *topo.Mesh, lp loftrecord.Payload) bool {
	return slices.Equal(mesh.Vertices, lp.Verts) && slices.Equal(mesh.Triangles, lp.Tris) &&
		len(mesh.Source) == len(lp.Tris)
}
