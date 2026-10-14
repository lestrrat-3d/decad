package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftMirrorJoinRecord ties a sewn three-ring mesh back to two authenticated
// Lofts. The lower source occupies ring 0..1, its reflected copy ring 1..2.
// Wall groups stay distinct even when both sources use the same role string.
type loftMirrorJoinRecord struct {
	lower, upper *Body
	stationCount int
	wallGroups   [2][]int
	capGroups    [2]int
}

// A rigid copy keeps the source relation separate from its moved held mesh.
// The source record is never reauthenticated from a faceted surface.
type loftMirrorPlacementRecord struct {
	source *loftMirrorJoinRecord
	motion r3.Transform
}

func tryLoftMirrorUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion || !a.IsSolid() || !b.IsSolid() {
		return nil, false, nil
	}
	lower, ok := a.n.Payload.(loftPayload)
	if !ok || lower.xform != r3.Identity() {
		a, b = b, a
		lower, ok = a.n.Payload.(loftPayload)
	}
	upper, upperOK := b.n.Payload.(loftPayload)
	if !ok || !upperOK || lower.xform != r3.Identity() ||
		!lower.authenticatedReconstruction || !upper.authenticatedReconstruction ||
		lower.surfaceResult || upper.surfaceResult || lower.sourceIdentity == nil ||
		lower.sourceIdentity != upper.sourceIdentity {
		return nil, false, nil
	}
	if !loftMirrorAxialSource(lower) || lower.frame0 != upper.frame0 ||
		lower.frame1 != upper.frame1 || lower.plane0 != upper.plane0 ||
		lower.plane1 != upper.plane1 || !slices.Equal(lower.alignment, upper.alignment) {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: loft end section has no reflection: %s", ErrUnsupported, err)
	}
	if upper.xform != reflection {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, pair := range [][2]profileRecord{{lower.profile0, upper.profile0},
		{lower.profile1, upper.profile1}} {
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
	body, err := buildLoftMirrorUnion(ctx, d, ref, a, b, lower, upper)
	return body, true, err
}

func loftMirrorAxialSource(lp loftPayload) bool {
	for _, f := range []r3.Frame{lp.frame0, lp.frame1} {
		if f.U() != (r3.Vec{X: 1}) || f.V() != (r3.Vec{Y: 1}) ||
			f.Origin().X != 0 || f.Origin().Y != 0 {
			return false
		}
	}
	return lp.frame0.Origin().Z < lp.frame1.Origin().Z
}

func buildLoftMirrorUnion(ctx context.Context, d *Document, ref producerID,
	lowerBody, upperBody *Body, lower, upper loftPayload) (*Body, error) {
	n := len(lower.verts) / 2
	if len(lower.verts)%2 != 0 || n < 3 || len(upper.verts) != len(lower.verts) ||
		lower.walls != 2*n || upper.walls != lower.walls ||
		lower.capStartCount <= 0 || upper.capStartCount != lower.capStartCount ||
		len(lower.tris) != len(upper.tris) ||
		lower.walls+lower.capStartCount >= len(lower.tris) {
		return nil, fmt.Errorf("%w: reflected lofts lack matching cap and wall stations", ErrUnsupported)
	}
	for i := range n {
		if lower.verts[n+i] != upper.verts[n+i] {
			return nil, fmt.Errorf("%w: reflected loft cap stations differ", ErrUnsupported)
		}
	}
	if !loftMirrorCapTrianglesMatch(lower, upper) {
		return nil, fmt.Errorf("%w: reflected loft cap triangles do not oppose", ErrUnsupported)
	}
	lowerMesh, err := lowerBody.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		return nil, err
	}
	upperMesh, err := upperBody.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		return nil, err
	}
	if !loftMirrorMeshMatchesPayload(lowerMesh, lower) ||
		!loftMirrorMeshMatchesPayload(upperMesh, upper) ||
		!lowerMesh.BoundaryVerified() || !upperMesh.BoundaryVerified() ||
		!lowerMesh.VolumeVerified() || !upperMesh.VolumeVerified() {
		return nil, fmt.Errorf("%w: reflected loft source mesh has no matching boundary and volume proof", ErrUnsupported)
	}
	_, pairDiameter, err := pairChordTolerance(ctx, lowerBody, upperBody)
	if err != nil {
		return nil, err
	}
	lowerBound, upperBound := lowerMesh.Bound().Base(), upperMesh.Bound().Base()
	bound := max(lowerBound, upperBound)
	volumeGap := proofbound.AbsSumUpper(lowerMesh.n.VolSymDiff, upperMesh.n.VolSymDiff)
	// Each Loft area slack contains the nonnegative error of BOTH of its caps.
	// Removing two copies of the same true and held cap therefore adds at most
	// twice either source's entire area slack to the two wall-area errors.
	areaSlack := proofbound.AbsSumUpper(lowerMesh.n.AreaSlack, upperMesh.n.AreaSlack,
		proofbound.ProductUpper(2, max(lowerMesh.n.AreaSlack, upperMesh.n.AreaSlack)))
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || bound < 0 || volumeGap < 0 || areaSlack < 0 {
		return nil, fmt.Errorf("%w: reflected loft proof is not finite", ErrUnsupported)
	}

	record := &loftMirrorJoinRecord{lower: lowerBody, upper: upperBody, stationCount: n}
	verts := append(append([]r3.Vec(nil), lower.verts...), upper.verts[:n]...)
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
	var groups []facetGroup
	groupOf := map[*Face]int{}
	add := func(triangle [3]int, face *Face, capRole string, half, wallIndex int) {
		group, exists := groupOf[face]
		if !exists {
			group = len(groups)
			groupOf[face] = group
			g := facetGroup{origins: face.originRefs()}
			if capRole != "" {
				g.origins = append(g.origins, provenance.Ref{Producer: ref, Role: capRole})
				g.surface = face.n.Surface
				g.planar = true
				g.normalBound = face.n.NormalBound
				g.reversed = face.n.Reversed
			}
			groups = append(groups, g)
		}
		tris = append(tris, triangle)
		src = append(src, group)
		if wallIndex >= 0 {
			record.wallGroups[half][wallIndex] = group
		} else if capRole == provenance.RoleCapStart {
			record.capGroups[0] = group
		} else if capRole == provenance.RoleCapEnd {
			record.capGroups[1] = group
		}
	}
	for half, item := range []struct {
		payload loftPayload
		mesh    *Mesh
	}{
		{lower, lowerMesh}, {upper, upperMesh},
	} {
		record.wallGroups[half] = make([]int, item.payload.walls)
		for i := range item.payload.walls + item.payload.capStartCount {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			triangle := item.payload.tris[i]
			if half == 1 {
				for corner, old := range triangle {
					if old < n {
						triangle[corner] = 2*n + old
					}
				}
			}
			role, wallIndex := "", i
			if i >= item.payload.walls {
				wallIndex = -1
				role = provenance.RoleCapStart
				if half == 1 {
					role = provenance.RoleCapEnd
				}
			}
			add(triangle, item.mesh.source[i], role, half, wallIndex)
		}
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: reflected loft seam has an invalid vertex link: %s", ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris); err != nil {
		return nil, err
	}
	capGroups := record.capGroups
	fp := facetedPayload{verts: verts, tris: tris, src: src, groups: groups,
		vertexBound: vertexBound, meshBound: bound, volSymDiff: volumeGap,
		areaSlack: areaSlack, dPair: pairDiameter, contactAudited: true,
		loftMirrorJoin: record, mirrorCapGroups: &capGroups, xform: r3.Identity()}
	return buildFacetedBody(ctx, d, ref, fp)
}

func loftMirrorMeshMatchesPayload(mesh *Mesh, lp loftPayload) bool {
	return slices.Equal(mesh.n.Vertices, lp.verts) && slices.Equal(mesh.n.Triangles, lp.tris) &&
		len(mesh.source) == len(lp.tris)
}

func loftMirrorCapTrianglesMatch(lower, upper loftPayload) bool {
	start := lower.walls + lower.capStartCount
	indices := make(map[[3]int]int, len(lower.tris)-start)
	for _, triangle := range lower.tris[start:] {
		key, sign := facetedtopology.SortedTriangle(triangle)
		if indices[key] != 0 {
			return false
		}
		indices[key] = sign
	}
	for _, triangle := range upper.tris[start:] {
		key, sign := facetedtopology.SortedTriangle(triangle)
		if indices[key] != -sign {
			return false
		}
		delete(indices, key)
	}
	return len(indices) == 0
}
