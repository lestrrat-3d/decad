package loftcylinder

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// MirrorHalfShell is the transient held shell and cap map before root binds
// live sources, audits the exact blank, and publishes a fresh result.
type MirrorHalfShell struct {
	Payload     facetedrecord.Payload
	CapGroups   [2]int
	CapOutlines [2][]int
	Outer       [2][]int
	Inner       [2][]int
	WallGroups  [2]int
}

// AssembleMirrorJoin stitches the two authenticated circular Join halves.
func AssembleMirrorJoin(ctx context.Context, lower, upper facetedrecord.Payload,
	bridgeBound, bridgeVolume float64) (MirrorHalfShell, error) {
	lowCaps := lower.LoftCylinderJoin.CapGroups
	// Reflection preserves the upper outline's XY coordinates. Every middle
	// vertex of the reflected cap must match the original held vertex exactly.
	middle := lower.LoftCylinderJoin.Z1
	midVertices := make(map[r3.Vec]int)
	for i, v := range lower.Verts {
		if v.Z == middle {
			if _, exists := midVertices[v]; exists {
				return MirrorHalfShell{}, fmt.Errorf("%w: lower circular Join duplicates a middle vertex", decaderr.ErrUnsupported)
			}
			midVertices[v] = i
		}
	}
	if len(midVertices) < 3 {
		return MirrorHalfShell{}, fmt.Errorf("%w: circular Join has no complete middle outline", decaderr.ErrUnsupported)
	}
	vertexMap := make([]int, len(upper.Verts))
	verts := append([]r3.Vec(nil), lower.Verts...)
	vertexBound := append([]float64(nil), lower.VertexBound...)
	for i, v := range upper.Verts {
		if v.Z == middle {
			index, exists := midVertices[v]
			if !exists {
				return MirrorHalfShell{}, fmt.Errorf("%w: reflected cylinder caps do not match exactly", decaderr.ErrUnsupported)
			}
			vertexMap[i] = index
			vertexBound[index] = max(vertexBound[index],
				proofbound.AbsSumUpper(upper.VertexBound[i], bridgeBound))
			continue
		}
		if v.Z <= middle {
			return MirrorHalfShell{}, fmt.Errorf("%w: reflected cylinder slab crosses its middle plane", decaderr.ErrUnsupported)
		}
		vertexMap[i] = len(verts)
		verts = append(verts, v)
		vertexBound = append(vertexBound,
			proofbound.AbsSumUpper(upper.VertexBound[i], bridgeBound))
	}
	if !facetedtopology.CapsOppose(lower.Tris, lower.Src, upper.Tris, upper.Src,
		vertexMap, lowCaps[1], lowCaps[1]) {
		return MirrorHalfShell{}, fmt.Errorf("%w: reflected cylinder middle caps do not tile equally", decaderr.ErrUnsupported)
	}
	var tris [][3]int
	var src []int
	var groups []facetedrecord.Group
	capGroups := [2]int{-1, -1}
	for half, payload := range []facetedrecord.Payload{lower, upper} {
		// Reflection changes cap orientation and height, not its source
		// group ID. Both copies omit the original top cap group.
		omit, keep := lowCaps[1], lowCaps[0]
		groupMap := make(map[int]int)
		for i, tri := range payload.Tris {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return MirrorHalfShell{}, err
				}
			}
			oldGroup := payload.Src[i]
			if oldGroup == omit {
				continue
			}
			group, exists := groupMap[oldGroup]
			if !exists {
				if oldGroup < 0 || oldGroup >= len(payload.Groups) {
					return MirrorHalfShell{}, fmt.Errorf("%w: circular Join facet names no source group", decaderr.ErrUnsupported)
				}
				group = len(groups)
				groupMap[oldGroup] = group
				groups = append(groups, payload.Groups[oldGroup])
				if oldGroup == keep {
					capGroups[half] = group
				}
			}
			if half == 1 {
				for corner, index := range tri {
					tri[corner] = vertexMap[index]
				}
			}
			tris = append(tris, tri)
			src = append(src, group)
		}
	}
	if capGroups[0] < 0 || capGroups[1] < 0 {
		return MirrorHalfShell{}, fmt.Errorf("%w: mirrored cylinder has no exterior cap groups", decaderr.ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return MirrorHalfShell{}, fmt.Errorf("%w: mirrored cylinder seam has an invalid vertex link: %s", decaderr.ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		loftmesh.MaxLoftAuditTriangles, AuditMaxScans); err != nil {
		return MirrorHalfShell{}, err
	}
	bound := max(lower.MeshBound,
		proofbound.AbsSumUpper(upper.MeshBound, bridgeBound))
	volumeGap := proofbound.AbsSumUpper(lower.VolSymDiff, upper.VolSymDiff, bridgeVolume)
	areaSlack := proofbound.AbsSumUpper(lower.AreaSlack, upper.AreaSlack,
		proofbound.ProductUpper(2, max(lower.AreaSlack, upper.AreaSlack)))
	diameter := facetedrecord.SourceDiameterUpper(verts, bound)
	if bound > 0.1 || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(diameter) {
		return MirrorHalfShell{}, fmt.Errorf("%w: mirrored cylinder has no finite 0.1 mm proof", decaderr.ErrUnsupported)
	}
	lowOutlines := lower.LoftCylinderJoin.CapOutlines
	capOutlines := [2][]int{
		append([]int(nil), lowOutlines[0]...),
		make([]int, len(lowOutlines[0])),
	}
	for i, index := range lowOutlines[0] {
		if index < 0 || index >= len(vertexMap) {
			return MirrorHalfShell{}, fmt.Errorf("%w: mirrored cylinder lost a cap outline vertex", decaderr.ErrUnsupported)
		}
		capOutlines[1][i] = vertexMap[index]
	}
	return MirrorHalfShell{
		Payload: facetedrecord.Payload{
			Verts: verts, Tris: tris, Src: src, Groups: groups,
			VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
			AreaSlack: areaSlack, DPair: diameter, ContactAudited: true, Xform: r3.Identity(),
		},
		CapGroups: capGroups, CapOutlines: capOutlines,
	}, nil
}

// AssembleMirrorBore stitches the bored halves and maps their annular caps.
func AssembleMirrorBore(ctx context.Context, lower, upper facetedrecord.Payload,
	bridgeBound, bridgeVolume float64) (MirrorHalfShell, error) {
	lowCaps := lower.LoftCylinderBore.CapGroups
	middle := lower.LoftCylinderBore.Z1
	if lowCaps[0] < 0 || lowCaps[1] < 0 || lowCaps[0] == lowCaps[1] {
		return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore has no separate annular caps", decaderr.ErrUnsupported)
	}
	midVertices := make(map[r3.Vec]int)
	for i, vertex := range lower.Verts {
		if vertex.Z != middle {
			continue
		}
		if _, duplicate := midVertices[vertex]; duplicate {
			return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore duplicates a middle vertex", decaderr.ErrUnsupported)
		}
		midVertices[vertex] = i
	}
	vertexMap := make([]int, len(upper.Verts))
	verts := append([]r3.Vec(nil), lower.Verts...)
	vertexBound := append([]float64(nil), lower.VertexBound...)
	for i, vertex := range upper.Verts {
		if vertex.Z == middle {
			index, exists := midVertices[vertex]
			if !exists {
				return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore middle annuli do not match", decaderr.ErrUnsupported)
			}
			vertexMap[i] = index
			vertexBound[index] = max(vertexBound[index],
				proofbound.AbsSumUpper(upper.VertexBound[i], bridgeBound))
			continue
		}
		if vertex.Z <= middle {
			return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore crosses its middle plane", decaderr.ErrUnsupported)
		}
		vertexMap[i] = len(verts)
		verts = append(verts, vertex)
		vertexBound = append(vertexBound,
			proofbound.AbsSumUpper(upper.VertexBound[i], bridgeBound))
	}
	if !facetedtopology.CapsOppose(lower.Tris, lower.Src, upper.Tris, upper.Src,
		vertexMap, lowCaps[1], lowCaps[1]) {
		return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore middle annuli do not tile equally", decaderr.ErrUnsupported)
	}
	var tris [][3]int
	var src []int
	var groups []facetedrecord.Group
	capGroups := [2]int{-1, -1}
	wallGroups := [2]int{-1, -1}
	for half, payload := range []facetedrecord.Payload{lower, upper} {
		groupMap := make(map[int]int)
		for i, tri := range payload.Tris {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return MirrorHalfShell{}, err
				}
			}
			oldGroup := payload.Src[i]
			if oldGroup == lowCaps[1] {
				continue
			}
			group, exists := groupMap[oldGroup]
			if !exists {
				if oldGroup < 0 || oldGroup >= len(payload.Groups) {
					return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore facet lost its source group", decaderr.ErrUnsupported)
				}
				group = len(groups)
				groupMap[oldGroup] = group
				groups = append(groups, payload.Groups[oldGroup])
				if oldGroup == lowCaps[0] {
					capGroups[half] = group
				}
				if oldGroup == lower.LoftCylinderBore.WallGroup {
					wallGroups[half] = group
				}
			}
			if half == 1 {
				for corner, index := range tri {
					tri[corner] = vertexMap[index]
				}
			}
			tris = append(tris, tri)
			src = append(src, group)
		}
	}
	if capGroups[0] < 0 || capGroups[1] < 0 || wallGroups[0] < 0 || wallGroups[1] < 0 {
		return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore lost an exterior annulus", decaderr.ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore has an invalid vertex link: %s", decaderr.ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		loftmesh.MaxLoftAuditTriangles, AuditMaxScans); err != nil {
		return MirrorHalfShell{}, err
	}
	bound := max(lower.MeshBound,
		proofbound.AbsSumUpper(upper.MeshBound, bridgeBound))
	volumeGap := proofbound.AbsSumUpper(lower.VolSymDiff, upper.VolSymDiff, bridgeVolume)
	areaSlack := proofbound.AbsSumUpper(lower.AreaSlack, upper.AreaSlack,
		proofbound.ProductUpper(2, max(lower.AreaSlack, upper.AreaSlack)))
	diameter := facetedrecord.SourceDiameterUpper(verts, bound)
	if bound > 0.1 || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(diameter) {
		return MirrorHalfShell{}, fmt.Errorf("%w: reflected circular bore has no finite 0.1 mm proof", decaderr.ErrUnsupported)
	}
	copyLoop := func(loop []int, top bool) ([]int, error) {
		mapped := make([]int, len(loop))
		for i, vertex := range loop {
			if vertex < 0 || vertex >= len(vertexMap) {
				return nil, fmt.Errorf("%w: reflected circular bore has an incomplete cap map", decaderr.ErrUnsupported)
			}
			mapped[i] = vertex
			if top {
				mapped[i] = vertexMap[vertex]
			}
		}
		return mapped, nil
	}
	var outer, inner [2][]int
	for cap := range 2 {
		var err error
		outer[cap], err = copyLoop(lower.LoftCylinderBore.Outer[0], cap == 1)
		if err != nil {
			return MirrorHalfShell{}, err
		}
		inner[cap], err = copyLoop(lower.LoftCylinderBore.Inner[0], cap == 1)
		if err != nil {
			return MirrorHalfShell{}, err
		}
	}
	return MirrorHalfShell{
		Payload: facetedrecord.Payload{
			Verts: verts, Tris: tris, Src: src, Groups: groups,
			VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
			AreaSlack: areaSlack, DPair: diameter, ContactAudited: true, Xform: r3.Identity(),
		},
		CapGroups: capGroups, Outer: outer, Inner: inner, WallGroups: wallGroups,
	}, nil
}
