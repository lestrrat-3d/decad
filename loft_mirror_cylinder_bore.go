package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This record retains the certified private lower bore and the full shell's
// exterior and bore loops for a later outer-cap rewrite. Placement clears it.
type loftMirrorCylinderBoreRecord struct {
	Join       *loftMirrorCylinderJoinRecord
	LowerBore  *topo.Body
	CapGroups  [2]int
	Outer      [2][]int
	Inner      [2][]int
	WallGroups [2]int
}

// tryLoftMirrorCylinderBoreCut cuts the exact centered cylinder from both
// reflected source halves. The lower half keeps the authenticated circular
// Join record, so its existing bore proof supplies the inner wall and annular
// caps. Reflection then transfers that proof to the upper half.
func tryLoftMirrorCylinderBoreCut(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.LoftMirrorCylinderJoin == nil || !joined.ContactAudited ||
		joined.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := joined.LoftMirrorCylinderJoin
	if record.Blank == nil || record.Source == nil || record.LowerHalf == nil ||
		len(record.Pairs) == 0 || record.LowerHalf.Doc != documentNode(d) || record.Blank.Doc != documentNode(d) ||
		record.BridgeBound < 0 || record.BridgeVolume < 0 ||
		proofbound.IsNonFinite(record.BridgeBound) || proofbound.IsNonFinite(record.BridgeVolume) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its bore sources", ErrUnsupported)
	}
	lower, ok := record.LowerHalf.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderJoin == nil || !lower.ContactAudited ||
		lower.Xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its lower-half proof", ErrUnsupported)
	}
	blank, ok := record.Blank.Payload.(prismPayload)
	if !ok || !loftMirrorCylinderBlank(blank) ||
		lower.LoftCylinderJoin.Z0 != blank.Z0 ||
		!loftMirrorCylinderLevels(blank.Z0, lower.LoftCylinderJoin.Z1, blank.Z1) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its exact axial levels", ErrUnsupported)
	}
	source := &joinedMesh{Verts: joined.Verts, Tris: joined.Tris, Src: joined.Src}
	for cap, group := range record.CapGroups {
		level := blank.Z0
		if cap == 1 {
			level = blank.Z1
		}
		if group < 0 || group >= len(joined.Groups) ||
			!loftCylinderAuditCap(ctx, source, record.CapOutlines[cap], group, level, cap == 0) {
			return nil, true, fmt.Errorf("%w: mirrored circular Join lost an outer cap disk", ErrUnsupported)
		}
	}
	lowerBore, matched, err := tryLoftCylinderBoreCut(ctx, op, d, ref, bodyHandle(record.LowerHalf), tool)
	if err != nil || !matched {
		return lowerBore, matched, err
	}
	prism, ok := tool.n.Payload.(prismPayload)
	if !ok {
		return nil, true, fmt.Errorf("%w: mirrored circular bore lost its cutter source", ErrUnsupported)
	}
	toolEnd := new(big.Rat).Add(proofarith.FloatRat(prism.Frame.Origin().Z), proofarith.FloatRat(prism.Z1))
	if toolEnd.Cmp(proofarith.FloatRat(blank.Z1)) <= 0 {
		return nil, true, fmt.Errorf("%w: mirrored circular bore tool ends inside the upper half", ErrUnsupported)
	}
	body, err := buildLoftMirrorCylinderBore(ctx, d, ref, record, lowerBore)
	return body, true, err
}

func buildLoftMirrorCylinderBore(ctx context.Context, d *Document, ref producerID,
	record *loftMirrorCylinderJoinRecord, lowerBody *Body) (*Body, error) {
	lower, ok := lowerBody.n.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderBore == nil || !lower.ContactAudited ||
		lower.Xform != r3.Identity() || record.Source.Lower == nil {
		return nil, fmt.Errorf("%w: mirrored circular bore lost its lower annulus", ErrUnsupported)
	}
	loft, ok := record.Source.Lower.Payload.(loftPayload)
	if !ok || !loft.AuthenticatedReconstruction {
		return nil, fmt.Errorf("%w: mirrored circular bore lost its Loft source", ErrUnsupported)
	}
	reflection, err := r3.Reflection(loft.Frame1)
	if err != nil {
		return nil, fmt.Errorf("%w: mirrored circular bore has no source reflection: %s", ErrUnsupported, err)
	}
	upperBody, err := placePayload(ctx, lower, d, ref, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperBody.n.Payload.(facetedPayload)
	if !ok || !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) {
		return nil, fmt.Errorf("%w: reflected circular bore lacks a matching held shell", ErrUnsupported)
	}
	lowCaps := lower.LoftCylinderBore.CapGroups
	middle := lower.LoftCylinderBore.Z1
	if lowCaps[0] < 0 || lowCaps[1] < 0 || lowCaps[0] == lowCaps[1] {
		return nil, fmt.Errorf("%w: reflected circular bore has no separate annular caps", ErrUnsupported)
	}
	midVertices := make(map[r3.Vec]int)
	for i, vertex := range lower.Verts {
		if vertex.Z != middle {
			continue
		}
		if _, duplicate := midVertices[vertex]; duplicate {
			return nil, fmt.Errorf("%w: reflected circular bore duplicates a middle vertex", ErrUnsupported)
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
				return nil, fmt.Errorf("%w: reflected circular bore middle annuli do not match", ErrUnsupported)
			}
			vertexMap[i] = index
			vertexBound[index] = max(vertexBound[index],
				proofbound.AbsSumUpper(upper.VertexBound[i], record.BridgeBound))
			continue
		}
		if vertex.Z <= middle {
			return nil, fmt.Errorf("%w: reflected circular bore crosses its middle plane", ErrUnsupported)
		}
		vertexMap[i] = len(verts)
		verts = append(verts, vertex)
		vertexBound = append(vertexBound,
			proofbound.AbsSumUpper(upper.VertexBound[i], record.BridgeBound))
	}
	if !loftMirrorCylinderCapsOppose(lower, upper, vertexMap, lowCaps[1], lowCaps[1]) {
		return nil, fmt.Errorf("%w: reflected circular bore middle annuli do not tile equally", ErrUnsupported)
	}
	var tris [][3]int
	var src []int
	var groups []facetGroup
	capGroups := [2]int{-1, -1}
	wallGroups := [2]int{-1, -1}
	for half, payload := range []facetedPayload{lower, upper} {
		groupMap := make(map[int]int)
		for i, tri := range payload.Tris {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			oldGroup := payload.Src[i]
			if oldGroup == lowCaps[1] {
				continue
			}
			group, exists := groupMap[oldGroup]
			if !exists {
				if oldGroup < 0 || oldGroup >= len(payload.Groups) {
					return nil, fmt.Errorf("%w: reflected circular bore facet lost its source group", ErrUnsupported)
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
		return nil, fmt.Errorf("%w: reflected circular bore lost an exterior annulus", ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: reflected circular bore has an invalid vertex link: %s", ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, err
	}
	bound := max(lower.MeshBound,
		proofbound.AbsSumUpper(upper.MeshBound, record.BridgeBound))
	volumeGap := proofbound.AbsSumUpper(lower.VolSymDiff, upper.VolSymDiff, record.BridgeVolume)
	areaSlack := proofbound.AbsSumUpper(lower.AreaSlack, upper.AreaSlack,
		proofbound.ProductUpper(2, max(lower.AreaSlack, upper.AreaSlack)))
	diameter := facetedSourceDiameterUpper(verts, bound)
	if bound > 0.1 || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(diameter) {
		return nil, fmt.Errorf("%w: reflected circular bore has no finite 0.1 mm proof", ErrUnsupported)
	}
	copyLoop := func(loop []int, top bool) ([]int, error) {
		mapped := make([]int, len(loop))
		for i, vertex := range loop {
			if vertex < 0 || vertex >= len(vertexMap) {
				return nil, fmt.Errorf("%w: reflected circular bore has an incomplete cap map", ErrUnsupported)
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
		outer[cap], err = copyLoop(lower.LoftCylinderBore.Outer[0], cap == 1)
		if err != nil {
			return nil, err
		}
		inner[cap], err = copyLoop(lower.LoftCylinderBore.Inner[0], cap == 1)
		if err != nil {
			return nil, err
		}
	}
	fullBlank, ok := record.Blank.Payload.(prismPayload)
	if !ok || !loftMirrorCylinderBlank(fullBlank) {
		return nil, fmt.Errorf("%w: reflected circular bore lost its exact blank", ErrUnsupported)
	}
	mesh := &joinedMesh{Verts: verts, Tris: tris, Src: src}
	for cap, group := range capGroups {
		z := fullBlank.Z0
		if cap == 1 {
			z = fullBlank.Z1
		}
		if !loftCylinderAuditAnnulus(ctx, mesh, outer[cap], inner[cap], group, z, cap == 0) {
			return nil, fmt.Errorf("%w: reflected circular bore cap does not tile its annulus", ErrUnsupported)
		}
	}
	fp := facetedPayload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: diameter, ContactAudited: true, Xform: r3.Identity(),
		LoftMirrorCylinderBore: &loftMirrorCylinderBoreRecord{
			Join: record, LowerBore: bodyNode(lowerBody), CapGroups: capGroups,
			Outer: outer, Inner: inner, WallGroups: wallGroups,
		}}
	if err := loftMirrorCylinderCapOrigins(&fp, bodyHandle(record.Blank), bodyHandles(record.Pairs), capGroups); err != nil {
		return nil, err
	}
	if err := loftCylinderTagCaps(&fp, bodyHandle(record.Blank), capGroups,
		[2]float64{fullBlank.Z0, fullBlank.Z1}, ref); err != nil {
		return nil, err
	}
	loftCylinderTagBridgedTop(&fp, capGroups[1], []float64{record.BridgeBound})
	return buildFacetedBody(ctx, d, ref, fp)
}
