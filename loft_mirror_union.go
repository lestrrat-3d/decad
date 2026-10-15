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
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftMirrorJoinRecord ties a sewn three-ring mesh back to two authenticated
// Lofts. The lower source occupies ring 0..1, its reflected copy ring 1..2.
// Wall groups stay distinct even when both sources use the same role string.
type loftMirrorJoinRecord struct {
	Lower, Upper *topo.Body
	StationCount int
	WallGroups   [2][]int
	CapGroups    [2]int
}

// A rigid copy keeps the source relation separate from its moved held mesh.
// The source record is never reauthenticated from a faceted surface.
type loftMirrorPlacementRecord struct {
	Source *loftMirrorJoinRecord
	Motion r3.Transform
}

func tryLoftMirrorUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion || !a.IsSolid() || !b.IsSolid() {
		return nil, false, nil
	}
	lower, ok := a.n.Payload.(loftPayload)
	if !ok || lower.Xform != r3.Identity() {
		a, b = b, a
		lower, ok = a.n.Payload.(loftPayload)
	}
	upper, upperOK := b.n.Payload.(loftPayload)
	if !ok || !upperOK || lower.Xform != r3.Identity() ||
		!lower.AuthenticatedReconstruction || !upper.AuthenticatedReconstruction ||
		lower.SurfaceResult || upper.SurfaceResult || lower.SourceIdentity == nil ||
		lower.SourceIdentity != upper.SourceIdentity {
		return nil, false, nil
	}
	if !loftMirrorAxialSource(lower) || lower.Frame0 != upper.Frame0 ||
		lower.Frame1 != upper.Frame1 || lower.Plane0 != upper.Plane0 ||
		lower.Plane1 != upper.Plane1 || !slices.Equal(lower.Alignment, upper.Alignment) {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: loft end section has no reflection: %s", ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, pair := range [][2]profileRecord{{lower.Profile0, upper.Profile0},
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
	body, err := buildLoftMirrorUnion(ctx, d, ref, a, b, lower, upper)
	return body, true, err
}

func loftMirrorAxialSource(lp loftPayload) bool {
	for _, f := range []r3.Frame{lp.Frame0, lp.Frame1} {
		if f.U() != (r3.Vec{X: 1}) || f.V() != (r3.Vec{Y: 1}) ||
			f.Origin().X != 0 || f.Origin().Y != 0 {
			return false
		}
	}
	return lp.Frame0.Origin().Z < lp.Frame1.Origin().Z
}

func buildLoftMirrorUnion(ctx context.Context, d *Document, ref producerID,
	lowerBody, upperBody *Body, lower, upper loftPayload) (*Body, error) {
	n := len(lower.Verts) / 2
	if len(lower.Verts)%2 != 0 || n < 3 || len(upper.Verts) != len(lower.Verts) ||
		lower.Walls != 2*n || upper.Walls != lower.Walls ||
		lower.CapStartCount <= 0 || upper.CapStartCount != lower.CapStartCount ||
		len(lower.Tris) != len(upper.Tris) ||
		lower.Walls+lower.CapStartCount >= len(lower.Tris) {
		return nil, fmt.Errorf("%w: reflected lofts lack matching cap and wall stations", ErrUnsupported)
	}
	for i := range n {
		if lower.Verts[n+i] != upper.Verts[n+i] {
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

	record := &loftMirrorJoinRecord{Lower: bodyNode(lowerBody), Upper: bodyNode(upperBody), StationCount: n}
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
	var groups []facetGroup
	groupOf := map[*topo.Face]int{}
	add := func(triangle [3]int, face *topo.Face, capRole string, half, wallIndex int) {
		group, exists := groupOf[face]
		if !exists {
			group = len(groups)
			groupOf[face] = group
			g := facetGroup{Origins: append([]provenance.Ref(nil), face.Origins...)}
			if capRole != "" {
				g.Origins = append(g.Origins, provenance.Ref{Producer: ref, Role: capRole})
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
		payload loftPayload
		mesh    *Mesh
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
			add(triangle, item.mesh.n.Source[i], role, half, wallIndex)
		}
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: reflected loft seam has an invalid vertex link: %s", ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris); err != nil {
		return nil, err
	}
	capGroups := record.CapGroups
	fp := facetedPayload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: pairDiameter, ContactAudited: true,
		LoftMirrorJoin: record, MirrorCapGroups: &capGroups, Xform: r3.Identity()}
	return buildFacetedBody(ctx, d, ref, fp)
}

func loftMirrorMeshMatchesPayload(mesh *Mesh, lp loftPayload) bool {
	return slices.Equal(mesh.n.Vertices, lp.Verts) && slices.Equal(mesh.n.Triangles, lp.Tris) &&
		len(mesh.n.Source) == len(lp.Tris)
}

func loftMirrorCapTrianglesMatch(lower, upper loftPayload) bool {
	start := lower.Walls + lower.CapStartCount
	indices := make(map[[3]int]int, len(lower.Tris)-start)
	for _, triangle := range lower.Tris[start:] {
		key, sign := facetedtopology.SortedTriangle(triangle)
		if indices[key] != 0 {
			return false
		}
		indices[key] = sign
	}
	for _, triangle := range upper.Tris[start:] {
		key, sign := facetedtopology.SortedTriangle(triangle)
		if indices[key] != -sign {
			return false
		}
		delete(indices, key)
	}
	return len(indices) == 0
}
