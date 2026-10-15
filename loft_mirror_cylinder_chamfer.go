package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// The mirrored bore retains a private lower-half bore. Chamfer only its
// exterior start cap, retain its middle annulus exactly, and reflect that
// audited half. Both exterior bands then have the same held construction.
func tryLoftMirrorCylinderCapChamfer(ctx context.Context, b *Body, fp facetedPayload,
	edges []*Edge, setback, delta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	record := fp.LoftMirrorCylinderBore
	if record == nil || !fp.ContactAudited || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	if asym != nil || delta != 0 || setback <= 0 || proofbound.IsNonFinite(setback) {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer needs one exact equal setback", ErrUnsupported)
	}
	if record.Join == nil || record.Join.Blank == nil || record.LowerBore == nil ||
		record.LowerBore.Doc != b.n.Doc || record.Join.Blank.Doc != b.n.Doc {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its authenticated source", ErrUnsupported)
	}
	lower, ok := record.LowerBore.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderBore == nil || !lower.ContactAudited || lower.Xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its lower bore", ErrUnsupported)
	}
	source, ok := loftCylinderChamferSourceOf(lower)
	if !ok {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its lower source", ErrUnsupported)
	}
	if err := loftCylinderChamferBridgeSources(&source); err != nil {
		return nil, true, err
	}
	start := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(setback))
	if start.Cmp(end) >= 0 || source.z0+setback == source.z0 || source.z1-setback == source.z1 {
		return nil, true, fmt.Errorf("%w: mirrored cap setback reaches the middle side section", ErrUnsupported)
	}
	if err := loftMirrorCylinderChamferSelect(ctx, b, fp, record, edges); err != nil {
		return nil, true, err
	}
	shifts, err := loftMirrorCylinderChamferUpperShifts(record, source)
	if err != nil {
		return nil, true, err
	}
	var half loftCylinderOneCapData
	ref := b.n.Doc.NextProducer
	reserve, err := loftMirrorCylinderChamferReflectionReserve(lower, record, setback)
	if err != nil {
		return nil, true, err
	}
	bridgeReserve, err := loftmesh.MirrorBridgeReserve(lower.Verts, lower.MeshBound,
		record.Join.BridgeBound, setback)
	if err != nil {
		return nil, true, err
	}
	totalReserve := proofbound.AbsSumUpper(reserve, bridgeReserve)
	allowance := math.Nextafter(0.1-totalReserve, math.Inf(-1))
	for i := 0; i < 4 && proofbound.AbsSumUpper(allowance, totalReserve) > 0.1; i++ {
		allowance = math.Nextafter(allowance, math.Inf(-1))
	}
	if allowance <= 0 || proofbound.AbsSumUpper(allowance, totalReserve) > 0.1 {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer has no reflection allowance", ErrUnsupported)
	}
	lowerBody, err := buildLoftCylinderOneCapChamfer(ctx, documentHandle(b.n.Doc), ref, lower, source,
		setback, allowance, shifts, &half)
	if err != nil {
		return nil, true, err
	}
	result, err := buildLoftMirrorCylinderCapChamfer(ctx, documentHandle(b.n.Doc), ref, fp, record,
		lowerBody, half, reserve, bridgeReserve, setback)
	return result, true, err
}

// Reserve the same certified rounding envelope used by placed faceted meshes
// for the reflection, with a second copy for any embedding correction. The
// assembled upper half must also pass the measured bound check below.
func loftMirrorCylinderChamferReflectionReserve(lower facetedPayload,
	record *loftMirrorCylinderBoreRecord, setback float64) (float64, error) {
	if record == nil || record.Join == nil || record.Join.Source == nil ||
		record.Join.Source.Lower == nil {
		return 0, fmt.Errorf("%w: mirrored cap chamfer lost its Loft frame", ErrUnsupported)
	}
	loft, ok := record.Join.Source.Lower.Payload.(loftPayload)
	if !ok || !loft.AuthenticatedReconstruction {
		return 0, fmt.Errorf("%w: mirrored cap chamfer lost its Loft frame", ErrUnsupported)
	}
	reflection, err := r3.Reflection(loft.Frame1)
	if err != nil {
		return 0, fmt.Errorf("%w: mirrored cap chamfer has no source reflection: %s", ErrUnsupported, err)
	}
	maxInput := 0.0
	for _, v := range lower.Verts {
		maxInput = max(maxInput, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	// The inset stays inside the old cap; adding the setback also covers the
	// new side stations and their held rounding.
	maxInput = proofbound.AbsSumUpper(maxInput, setback)
	tr := reflection.Translation()
	maxTrans := max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z))
	round := proofbound.RigidRoundAllow(maxInput, maxTrans)
	reserve := proofbound.AbsSumUpper(round, round)
	if reserve <= 0 || reserve >= 0.1 || proofbound.IsNonFinite(reserve) {
		return 0, fmt.Errorf("%w: mirrored cap chamfer reflection has no finite allowance", ErrUnsupported)
	}
	return reserve, nil
}

func loftMirrorCylinderChamferSelect(ctx context.Context, b *Body, fp facetedPayload,
	record *loftMirrorCylinderBoreRecord, edges []*Edge) error {
	mesh := &joinedMesh{Verts: fp.Verts, Tris: fp.Tris, Src: fp.Src, Groups: fp.Groups}
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				if caps[0] != nil {
					return fmt.Errorf("%w: mirrored start cap is split", ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
				if caps[1] != nil {
					return fmt.Errorf("%w: mirrored end cap is split", ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	if len(selected) != len(edges) {
		return fmt.Errorf("%w: mirrored cap chamfer repeats an edge", ErrUnsupported)
	}
	blank, ok := record.Join.Blank.Payload.(prismPayload)
	if !ok || !loftMirrorCylinderBlank(blank) || caps[0] == nil || caps[1] == nil {
		return fmt.Errorf("%w: mirrored bore lost its two exterior annuli", ErrUnsupported)
	}
	for cap, face := range caps {
		z := blank.Z0
		if cap == 1 {
			z = blank.Z1
		}
		if record.CapGroups[cap] < 0 || record.WallGroups[cap] < 0 ||
			!loftCylinderAuditAnnulus(ctx, mesh, record.Outer[cap], record.Inner[cap],
				record.CapGroups[cap], z, cap == 0) || len(face.n.Loops) != 2 ||
			!loftCylinderChamferCapWalk(face.n.Loops[0], fp.Verts, record.Outer[cap], cap == 0) {
			return fmt.Errorf("%w: mirrored bore cap %d has no audited outer loop", ErrUnsupported, cap)
		}
		for _, ce := range face.n.Loops[0].CoEdges {
			edge := ce.Edge
			if _, yes := selected[edge]; !yes {
				return fmt.Errorf("%w: mirrored chamfer needs both complete outer cap loops", ErrUnsupported)
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return fmt.Errorf("%w: mirrored chamfer selection includes a bore or side edge", ErrUnsupported)
	}
	return nil
}

func buildLoftMirrorCylinderCapChamfer(ctx context.Context, d *Document, ref producerID,
	old facetedPayload, record *loftMirrorCylinderBoreRecord, lowerBody *Body,
	half loftCylinderOneCapData, reflectionReserve, bridgeReserve, setback float64) (*Body, error) {
	lower, ok := lowerBody.n.Payload.(facetedPayload)
	if !ok || !lower.ContactAudited || lower.Xform != r3.Identity() ||
		record.Join.Source == nil || record.Join.Source.Lower == nil {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its audited lower half", ErrUnsupported)
	}
	loft, ok := record.Join.Source.Lower.Payload.(loftPayload)
	if !ok || !loft.AuthenticatedReconstruction {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its Loft source", ErrUnsupported)
	}
	reflection, err := r3.Reflection(loft.Frame1)
	if err != nil {
		return nil, fmt.Errorf("%w: mirrored cap chamfer has no source reflection: %s", ErrUnsupported, err)
	}
	upperBody, err := placePayload(ctx, lower, d, ref, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperBody.n.Payload.(facetedPayload)
	if !ok || !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) ||
		half.capGroups[0] < 0 || half.capGroups[1] < 0 || half.boreGroup < 0 {
		return nil, fmt.Errorf("%w: mirrored cap chamfer halves have no matching held shell", ErrUnsupported)
	}
	if upper.MeshBound > proofbound.AbsSumUpper(lower.MeshBound, reflectionReserve) {
		return nil, fmt.Errorf("%w: reflected cap chamfer exceeds its %g mm placement reserve "+
			"(lower %g, upper %g)", ErrUnsupported, reflectionReserve, lower.MeshBound, upper.MeshBound)
	}
	bridgeReach, err := loftMirrorCylinderChamferUpperCapBridge(ctx, old, upper, record, half, setback)
	if err != nil {
		return nil, err
	}
	if bridgeReach > bridgeReserve {
		return nil, fmt.Errorf("%w: reflected cap bridge %g exceeds its %g mm reserve",
			ErrUnsupported, bridgeReach, bridgeReserve)
	}
	middle := loft.Frame1.Origin().Z
	midVertices := make(map[r3.Vec]int)
	for i, vertex := range lower.Verts {
		if vertex.Z != middle {
			continue
		}
		if _, duplicate := midVertices[vertex]; duplicate {
			return nil, fmt.Errorf("%w: mirrored cap chamfer duplicates a middle station", ErrUnsupported)
		}
		midVertices[vertex] = i
	}
	mapUpper := make([]int, len(upper.Verts))
	verts := append([]r3.Vec(nil), lower.Verts...)
	vertexBound := append([]float64(nil), lower.VertexBound...)
	for i, vertex := range upper.Verts {
		if vertex.Z == middle {
			index, exists := midVertices[vertex]
			if !exists {
				return nil, fmt.Errorf("%w: mirrored cap chamfer middle annuli do not match", ErrUnsupported)
			}
			mapUpper[i] = index
			vertexBound[index] = max(vertexBound[index],
				proofbound.AbsSumUpper(upper.VertexBound[i], bridgeReach))
			continue
		}
		if vertex.Z <= middle {
			return nil, fmt.Errorf("%w: mirrored cap chamfer crosses its middle plane", ErrUnsupported)
		}
		mapUpper[i] = len(verts)
		verts = append(verts, vertex)
		vertexBound = append(vertexBound, proofbound.AbsSumUpper(upper.VertexBound[i], bridgeReach))
	}
	if !loftMirrorCylinderCapsOppose(lower, upper, mapUpper,
		half.capGroups[1], half.capGroups[1]) {
		return nil, fmt.Errorf("%w: mirrored cap chamfer middle annuli do not tile equally", ErrUnsupported)
	}
	if err := loftMirrorCylinderChamferSeam(ctx, lower, upper, half, middle); err != nil {
		return nil, err
	}
	var tris [][3]int
	var src []int
	var groups []facetGroup
	capGroups := [2]int{-1, -1}
	wallGroups := [2]int{-1, -1}
	for halfIndex, payload := range []facetedPayload{lower, upper} {
		groupMap := make(map[int]int)
		for i, tri := range payload.Tris {
			if i%256 == 0 && ctx.Err() != nil {
				return nil, ctx.Err()
			}
			oldGroup := payload.Src[i]
			if oldGroup == half.capGroups[1] {
				continue
			}
			group, exists := groupMap[oldGroup]
			if !exists {
				if oldGroup < 0 || oldGroup >= len(payload.Groups) {
					return nil, fmt.Errorf("%w: mirrored cap chamfer facet lost its source group", ErrUnsupported)
				}
				group = len(groups)
				groupMap[oldGroup] = group
				groups = append(groups, payload.Groups[oldGroup])
				if oldGroup == half.capGroups[0] {
					capGroups[halfIndex] = group
				}
				if oldGroup == half.boreGroup {
					wallGroups[halfIndex] = group
				}
			}
			if halfIndex == 1 {
				for corner, index := range tri {
					tri[corner] = mapUpper[index]
				}
			}
			tris = append(tris, tri)
			src = append(src, group)
		}
	}
	if capGroups[0] < 0 || capGroups[1] < 0 || wallGroups[0] < 0 || wallGroups[1] < 0 {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost an exterior annulus or bore wall", ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: mirrored cap chamfer has an invalid vertex link: %s", ErrUnsupported, err)
	}
	blank, ok := record.Join.Blank.Payload.(prismPayload)
	if !ok || !loftMirrorCylinderBlank(blank) {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its exact blank", ErrUnsupported)
	}
	var outer, inner [2][]int
	outer[0], inner[0] = half.outer[0], half.inner[0]
	outer[1] = make([]int, len(half.outer[0]))
	inner[1] = make([]int, len(half.inner[0]))
	for i, index := range half.outer[0] {
		outer[1][i] = mapUpper[index]
	}
	for i, index := range half.inner[0] {
		inner[1][i] = mapUpper[index]
	}
	mesh := &joinedMesh{Verts: verts, Tris: tris, Src: src, Groups: groups}
	for cap, group := range capGroups {
		z := blank.Z0
		if cap == 1 {
			z = blank.Z1
		}
		if !loftCylinderAuditAnnulus(ctx, mesh, outer[cap], inner[cap], group, z, cap == 0) {
			return nil, fmt.Errorf("%w: mirrored cap chamfer does not tile exterior annulus %d", ErrUnsupported, cap)
		}
	}
	if err := loftMirrorCylinderChamferBoreUnchanged(old, mesh, record.WallGroups, wallGroups); err != nil {
		return nil, err
	}
	bound := max(lower.MeshBound, proofbound.AbsSumUpper(upper.MeshBound, bridgeReach))
	volumeGap := proofbound.AbsSumUpper(lower.VolSymDiff, upper.VolSymDiff,
		record.Join.BridgeVolume)
	areaSlack := proofbound.AbsSumUpper(lower.AreaSlack, upper.AreaSlack,
		proofbound.ProductUpper(2, max(lower.AreaSlack, upper.AreaSlack)))
	diameter := facetedSourceDiameterUpper(verts, bound)
	if bound > 0.1 || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(diameter) {
		return nil, fmt.Errorf("%w: mirrored cap chamfer has no finite 0.1 mm proof "+
			"(lower %g, upper %g, volume %g, area %g, diameter %g)",
			ErrUnsupported, lower.MeshBound, upper.MeshBound, volumeGap, areaSlack, diameter)
	}
	result := facetedPayload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: diameter, ContactAudited: true, Xform: r3.Identity()}
	for cap, group := range capGroups {
		result.Groups[group].Origins = append([]provenance.Ref(nil), old.Groups[record.CapGroups[cap]].Origins...)
	}
	if err := loftCylinderTagCaps(&result, bodyHandle(record.Join.Blank), capGroups,
		[2]float64{blank.Z0, blank.Z1}, ref); err != nil {
		return nil, err
	}
	return buildFacetedBody(ctx, d, ref, result)
}

// Each half was audited before reflection. Their only possible geometric
// contact is on the middle plane: all other vertices lie on strict opposite
// sides. The opposed middle annuli are removed, and every remaining facet
// meets that plane only in a vertex or full edge of their shared two loops.
func loftMirrorCylinderChamferSeam(ctx context.Context, lower, upper facetedPayload,
	half loftCylinderOneCapData, middle float64) error {
	if half.capGroups[1] < 0 || len(half.outer[1]) < 3 || len(half.inner[1]) < 3 ||
		!loftcylinder.BoreRingSimple(ctx, lower.Verts, half.outer[1]) ||
		!loftcylinder.BoreRingSimple(ctx, lower.Verts, half.inner[1]) {
		return fmt.Errorf("%w: mirrored cap chamfer has no simple middle loops", ErrUnsupported)
	}
	nested, err := loftcylinder.ChamferRingNested(ctx, lower.Verts, half.outer[1], half.inner[1])
	if err != nil {
		return err
	}
	if !nested {
		return fmt.Errorf("%w: mirrored cap chamfer middle loops are not nested", ErrUnsupported)
	}
	vertices := make(map[int]struct{}, len(half.outer[1])+len(half.inner[1]))
	edges := make(map[[2]int]struct{}, len(vertices))
	for _, loop := range [][]int{half.outer[1], half.inner[1]} {
		for i, index := range loop {
			vertices[index] = struct{}{}
			edges[loftcylinder.ChamferEdge(index, loop[(i+1)%len(loop)])] = struct{}{}
		}
	}
	for side, payload := range []facetedPayload{lower, upper} {
		mesh := &joinedMesh{Verts: payload.Verts, Tris: payload.Tris,
			Src: payload.Src, Groups: payload.Groups}
		if !loftCylinderAuditAnnulus(ctx, mesh, half.outer[1], half.inner[1],
			half.capGroups[1], middle, side == 1) {
			return fmt.Errorf("%w: mirrored cap chamfer half %d has no tiled middle annulus",
				ErrUnsupported, side)
		}
		for i, tri := range payload.Tris {
			if i%256 == 0 && ctx.Err() != nil {
				return ctx.Err()
			}
			if payload.Src[i] == half.capGroups[1] {
				continue
			}
			var on []int
			for _, index := range tri {
				if index < 0 || index >= len(payload.Verts) {
					return fmt.Errorf("%w: mirrored cap chamfer has an invalid middle facet", ErrUnsupported)
				}
				z := payload.Verts[index].Z
				if side == 0 && z > middle || side == 1 && z < middle {
					return fmt.Errorf("%w: mirrored cap chamfer half crosses the middle plane", ErrUnsupported)
				}
				if z == middle {
					if _, ok := vertices[index]; !ok {
						return fmt.Errorf("%w: mirrored cap chamfer touches off its middle loop", ErrUnsupported)
					}
					on = append(on, index)
				}
			}
			if len(on) == 3 {
				return fmt.Errorf("%w: mirrored cap chamfer has a planar seam wall", ErrUnsupported)
			}
			if len(on) == 2 {
				if _, ok := edges[loftcylinder.ChamferEdge(on[0], on[1])]; !ok {
					return fmt.Errorf("%w: mirrored cap chamfer has an unmatched seam edge", ErrUnsupported)
				}
			}
		}
	}
	return nil
}

func loftMirrorCylinderChamferBoreUnchanged(old facetedPayload, mesh *joinedMesh,
	oldGroups, newGroups [2]int) error {
	type triangle [3]r3.Vec
	collect := func(verts []r3.Vec, tris [][3]int, src []int, groups [2]int) map[triangle]int {
		kept := make(map[triangle]int)
		for i, tri := range tris {
			if src[i] != groups[0] && src[i] != groups[1] {
				continue
			}
			key := triangle{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
			kept[key]++
		}
		return kept
	}
	a := collect(old.Verts, old.Tris, old.Src, oldGroups)
	b := collect(mesh.Verts, mesh.Tris, mesh.Src, newGroups)
	if len(a) != len(b) {
		return fmt.Errorf("%w: mirrored cap chamfer changed its bore wall", ErrUnsupported)
	}
	for tri, count := range a {
		if b[tri] != count {
			return fmt.Errorf("%w: mirrored cap chamfer changed a bore-wall facet", ErrUnsupported)
		}
	}
	return nil
}
