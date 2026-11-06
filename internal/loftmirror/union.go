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
	layout, ok := ringLayoutOf(lower)
	upperLayout, upperOK := ringLayoutOf(upper)
	if !ok || !upperOK || !slices.Equal(layout.sizes, upperLayout.sizes) ||
		len(upper.Verts) != len(lower.Verts) || upper.Walls != lower.Walls ||
		lower.CapStartCount <= 0 || upper.CapStartCount != lower.CapStartCount ||
		len(lower.Tris) != len(upper.Tris) ||
		lower.Walls+lower.CapStartCount >= len(lower.Tris) {
		return nil, fmt.Errorf("%w: reflected lofts lack matching cap and wall stations", decaderr.ErrUnsupported)
	}
	n := layout.stations
	for _, loop := range layout.loops {
		for _, index := range loop[1] {
			if lower.Verts[index] != upper.Verts[index] {
				return nil, fmt.Errorf("%w: reflected loft cap stations differ", decaderr.ErrUnsupported)
			}
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
	// Each upper start station joins the mesh after the lower vertices; each
	// end station is the shared middle-ring vertex of the same index.
	verts := append([]r3.Vec(nil), lower.Verts...)
	vertexBound := make([]float64, len(lower.Verts), len(lower.Verts)+n)
	for i := range vertexBound {
		vertexBound[i] = lowerBound
	}
	upperIndex := make([]int, len(upper.Verts))
	for i := range upperIndex {
		upperIndex[i] = i
	}
	for _, loop := range layout.loops {
		for _, index := range loop[0] {
			upperIndex[index] = len(verts)
			verts = append(verts, upper.Verts[index])
			vertexBound = append(vertexBound, upperBound)
		}
		for _, index := range loop[1] {
			vertexBound[index] = bound
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
					triangle[corner] = upperIndex[old]
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
	if err := auditSewnShell(ctx, verts, tris, lower, layout, upperIndex); err != nil {
		return nil, err
	}
	capGroups := record.CapGroups
	fp := facetedrecord.Payload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: pairDiameter, ContactAudited: true,
		LoftMirrorJoin: record, MirrorCapGroups: &capGroups, Xform: r3.Identity()}
	return facetedrecord.Build(ctx, env, fp)
}

// ringLayout is a Loft's held vertex layout: loftmesh.Assemble lifts each
// loop's start stations, then its end stations, loop by loop (outer first,
// then the holes), and emits two wall triangles per cell. loops holds each
// loop's start and end station indices in loop order; sizes their station
// counts; stations their total per ring.
type ringLayout struct {
	loops    [][2][]int
	sizes    []int
	stations int
}

// ringLayoutOf reads the layout from the payload's per-wall cell record. It
// reports false unless the cells name loops 0..k in order, every loop has at
// least three stations, and the vertex and wall counts agree.
func ringLayoutOf(pl loftrecord.Payload) (ringLayout, bool) {
	if len(pl.Cell) != pl.Walls || pl.Walls == 0 {
		return ringLayout{}, false
	}
	var layout ringLayout
	for _, cell := range pl.Cell {
		switch cell[0] {
		case len(layout.sizes):
			layout.sizes = append(layout.sizes, 1)
		case len(layout.sizes) - 1:
			layout.sizes[cell[0]]++
		default:
			return ringLayout{}, false
		}
	}
	offset := 0
	for i, cells := range layout.sizes {
		if cells%2 != 0 || cells/2 < 3 {
			return ringLayout{}, false
		}
		size := cells / 2
		layout.sizes[i] = size
		start, end := make([]int, size), make([]int, size)
		for k := range size {
			start[k], end[k] = offset+k, offset+size+k
		}
		layout.loops = append(layout.loops, [2][]int{start, end})
		layout.stations += size
		offset += 2 * size
	}
	if offset != len(pl.Verts) || len(layout.sizes) != 1+len(pl.Profile0.Holes) {
		return ringLayout{}, false
	}
	return layout, true
}

// auditSewnShell runs the crossing audit over the sewn shell. tris holds the
// lower walls, the lower start cap, the upper walls and the reflected upper
// start cap, in that order. Each loop's start stations bound the lower start
// cap, their reflected images (upperIndex) the upper cap, and its end stations
// form the common middle ring, so the shell is a banded loft with the middle
// rings as its interior section: the structured audit decides each planar
// outer cap as a whole (loftmesh.CapFamilyProof), holes included, and every
// other pair is tested exactly. A cap whose proof fails keeps its pairs in the
// exact test.
func auditSewnShell(ctx context.Context, verts []r3.Vec, tris [][3]int, lower loftrecord.Payload,
	layout ringLayout, upperIndex []int) error {
	walls, caps := lower.Walls, lower.CapStartCount
	half := walls + caps
	order := make([][3]int, 0, len(tris))
	order = append(order, tris[:walls]...)
	order = append(order, tris[half:half+walls]...)
	order = append(order, tris[walls:half]...)
	order = append(order, tris[half+walls:]...)
	structure := loftmesh.LoftAuditStructure{Walls: 2 * walls, CapStartCount: caps}
	for _, loop := range layout.loops {
		reflected := make([]int, len(loop[0]))
		for k, index := range loop[0] {
			reflected[k] = upperIndex[index]
		}
		structure.Loops0 = append(structure.Loops0, loop[0])
		structure.Loops1 = append(structure.Loops1, reflected)
		structure.Interior = append(structure.Interior, loop[1]...)
	}
	return loftmesh.LoftCrossingAuditBanded(proofbound.NewWorkBudget(ctx), verts, order, structure)
}

func meshMatchesPayload(mesh *topo.Mesh, lp loftrecord.Payload) bool {
	return slices.Equal(mesh.Vertices, lp.Verts) && slices.Equal(mesh.Triangles, lp.Tris) &&
		len(mesh.Source) == len(lp.Tris)
}
