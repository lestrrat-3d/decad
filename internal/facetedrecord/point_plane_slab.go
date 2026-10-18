package facetedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// BuildPointPlaneSlab rebuilds a Split slab from its original point-loft fan.
func BuildPointPlaneSlab(ctx context.Context, env topo.Env,
	record PointPlaneSliceRecord) (*topo.Body, error) {
	base := record.Base
	if record.Lower != nil && record.Upper != nil &&
		PointPlaneOrder(base, record.Lower, record.Upper) != -1 {
		return nil, fmt.Errorf("%w: point-loft slab planes are not ordered", decaderr.ErrUnsupported)
	}
	n := len(base.Verts) - 1
	if n < 3 || len(base.Tris) <= n || len(base.Src) != len(base.Tris) {
		return nil, fmt.Errorf("%w: point-loft source has no complete fan", decaderr.ErrUnsupported)
	}
	lower, lowerRound, err := PointPlaneRing(ctx, base, record.Lower)
	if err != nil {
		return nil, err
	}
	upper, upperRound, err := PointPlaneRing(ctx, base, record.Upper)
	if err != nil {
		return nil, err
	}
	if record.Upper == nil {
		copy(upper, base.Verts[1:])
	}
	round := max(lowerRound, upperRound)
	hasApex := record.Lower == nil
	vertices := make([]r3.Vec, 0, 2*n+1)
	if hasApex {
		vertices = append(vertices, base.PointSection.Apex)
	} else {
		vertices = append(vertices, lower...)
	}
	upperOffset := len(vertices)
	vertices = append(vertices, upper...)
	groups := make([]Group, 0, len(base.Groups)+2)
	groups = append(groups, base.Groups[:len(base.Groups)-1]...)
	lowerGroup := len(groups)
	if !hasApex {
		groups = append(groups, PointPlaneCapGroup(env.Ref, provenance.RoleCapStart,
			record.Lower, base, false, record.Lower.FrameTowardFar))
	}
	upperGroup := len(groups)
	upperReversed := false
	if record.Upper != nil {
		upperReversed = !record.Upper.FrameTowardFar
	}
	groups = append(groups, PointPlaneCapGroup(env.Ref, provenance.RoleCapEnd,
		record.Upper, base, record.Upper == nil, upperReversed))
	triangles := make([][3]int, 0, 2*n+2*(len(base.Tris)-n))
	sources := make([]int, 0, cap(triangles))
	for i := range n {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		side := base.Tris[i]
		if side[0] != 0 || side[1] <= 0 || side[2] <= 0 {
			return nil, fmt.Errorf("%w: point-loft side has no apex fan", decaderr.ErrUnsupported)
		}
		b, c := side[1]-1, side[2]-1
		source := base.Src[i]
		if source < 0 || source >= len(base.Groups)-1 {
			return nil, fmt.Errorf("%w: point-loft side has no source role", decaderr.ErrUnsupported)
		}
		if hasApex {
			triangles = append(triangles, [3]int{0, upperOffset + b, upperOffset + c})
			sources = append(sources, source)
			continue
		}
		triangles = append(triangles,
			[3]int{b, upperOffset + b, upperOffset + c},
			[3]int{b, upperOffset + c, c})
		sources = append(sources, source, source)
	}
	for _, capTri := range base.Tris[n:] {
		if capTri[0] == 0 || capTri[1] == 0 || capTri[2] == 0 {
			return nil, fmt.Errorf("%w: point-loft far cap includes its apex", decaderr.ErrUnsupported)
		}
		mapIndex := func(v int, offset int) int { return offset + v - 1 }
		triangles = append(triangles, [3]int{
			mapIndex(capTri[0], upperOffset), mapIndex(capTri[1], upperOffset),
			mapIndex(capTri[2], upperOffset)})
		sources = append(sources, upperGroup)
		if !hasApex {
			triangles = append(triangles, [3]int{
				mapIndex(capTri[0], 0), mapIndex(capTri[2], 0), mapIndex(capTri[1], 0)})
			sources = append(sources, lowerGroup)
		}
	}
	if layout, ok := loftmesh.PointPlaneSlabCapLayout(base.Tris[:n], len(triangles), upperOffset, hasApex); ok {
		err = loftmesh.LoftCrossingAuditWithCaps(ctx, vertices, triangles, layout)
	} else {
		err = loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), vertices, triangles)
	}
	if err != nil {
		return nil, err
	}
	stretch := 1.0
	if record.Lower != nil {
		stretch = max(stretch, record.Lower.MapStretch)
	}
	if record.Upper != nil {
		stretch = max(stretch, record.Upper.MapStretch)
	}
	meshBound := proofbound.AbsSumUpper(proofbound.ProductUpper(stretch, base.MeshBound), round)
	if proofbound.IsNonFinite(meshBound) {
		return nil, fmt.Errorf("%w: planar split has no finite boundary bound", decaderr.ErrUnsupported)
	}
	areaRound, err := proofbound.PerturbedAreaUpperContext(ctx, vertices, triangles, round)
	if err != nil {
		return nil, err
	}
	volumeGap := proofbound.AbsSumUpper(base.VolSymDiff,
		proofbound.SweptVolumeAllow(round, areaRound))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts, base.Tris, 0)
	if err != nil {
		return nil, err
	}
	trueArea := proofbound.AbsSumUpper(baseArea, base.AreaSlack)
	// Each side patch is a subset of its original fan. The new caps are
	// projective images of the far cap; their area grows by at most L².
	capStretch := proofbound.ProductUpper(stretch, stretch)
	trueUpper := proofbound.ProductUpper(trueArea,
		proofbound.AbsSumUpper(1, proofbound.ProductUpper(2, capStretch)))
	areaSlack := proofbound.AbsSumUpper(trueUpper, areaRound)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: planar split has no finite occupied-volume or area proof", decaderr.ErrUnsupported)
	}
	vertexBounds := make([]float64, len(vertices))
	for i := range vertexBounds {
		vertexBounds[i] = meshBound
	}
	recordCopy := record
	return Build(ctx, env, Payload{
		Verts: vertices, Tris: triangles, Src: sources, Groups: groups,
		VertexBound: vertexBounds, MeshBound: meshBound,
		VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: base.DPair, Xform: r3.Identity(), PointPlane: &recordCopy,
		PointPlaneCapDisplacement: meshBound,
		PointPlaneSourceEligible:  true,
	})
}
