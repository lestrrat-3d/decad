package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sync"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// spiralToothPlacement names the immutable cone trim that produced a rigid
// copy. It is created only by facetedPayload.placed from an original trim.
type spiralToothPlacement struct {
	source *loftFacesTrimRecord
	motion r3.Transform
}

// r3.Rotation's Rodrigues evaluation can round its X basis length one float
// step below or above one. Keep that narrow case separate from the exact-axis
// class, because the sector certificate must charge its axial displacement.
func spiralNearAxisRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t == r3.Identity() ||
		t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EX.X >= math.Nextafter(1, 0) &&
		b.EX.X <= math.Nextafter(1, math.Inf(1)) &&
		b.EX.Y == 0 && b.EX.Z == 0 && b.EY.X == 0 && b.EZ.X == 0 &&
		b.EY.Y == b.EZ.Z && b.EY.Z == -b.EZ.Y
}

type spiralCombineSource struct {
	first *spiralFirstJoinSource
	tools []*Body
	moves []r3.Transform
}

type spiralCombinedTools struct {
	source *spiralCombineSource
	// owner maps each transient composite face to the live face it copies.
	owner map[*Face]*Face
}

type spiralCombinedAdmission struct {
	source  *spiralCombineSource
	roles   map[[2]*Face]spiralNearPairKind
	allowed map[[2]int]bool
	short   [2][4]spiralShortCapFamily
	groups  []map[int]int
}

// spiralCombinedDiameter selects six actual held vertices as a lower
// diameter witness for the transient tool operand. A subset's exact pair
// distance cannot exceed the full mesh diameter.
func spiralCombinedDiameter(ctx context.Context, verts []r3.Vec) (float64, bool, error) {
	if len(verts) == 0 {
		return 0, false, nil
	}
	witness := [6]r3.Vec{verts[0], verts[0], verts[0], verts[0], verts[0], verts[0]}
	for i, v := range verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		if !proofbound.FiniteVec(v) {
			return 0, false, nil
		}
		if v.X < witness[0].X {
			witness[0] = v
		}
		if v.X > witness[1].X {
			witness[1] = v
		}
		if v.Y < witness[2].Y {
			witness[2] = v
		}
		if v.Y > witness[3].Y {
			witness[3] = v
		}
		if v.Z < witness[4].Z {
			witness[4] = v
		}
		if v.Z > witness[5].Z {
			witness[5] = v
		}
	}
	return diameter.PointsContext(ctx, witness[:])
}

// spiralCombineFaceRoles binds the first tooth's authenticated face families
// to each copy by the preserved triangle ordering. It refuses any placement
// that changes the source group's patch partition or joins two copy faces.
func spiralCombineFaceRoles(ctx context.Context, blank, combined *Body,
	source *spiralCombineSource, faceID map[*Face]int) (*spiralCombinedAdmission, error) {
	first := source.tools[0]
	roles, short, ok, err := certifySpiralNearFaceRoles(ctx, blank, first, source.first)
	if err != nil || !ok {
		return nil, err
	}
	firstID := make(map[*Face]int)
	for i, face := range blank.Faces() {
		firstID[face] = i
	}
	for i, face := range first.Faces() {
		firstID[face] = len(blank.Faces()) + i
	}
	firstAdmission := &spiralFirstUnionAdmission{source: source.first, short: short, roles: roles}
	firstAllowed, ok := spiralFirstUnionPairs(blank, first, firstAdmission, firstID)
	if !ok {
		return nil, fmt.Errorf("%w: combined source has no first-tooth role table", ErrUnsupported)
	}
	firstPart, ok := first.payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: first spiral tooth lost its faceted source", ErrUnsupported)
	}
	allFaces := combined.Faces()
	combinedPart, ok := combined.payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: combined spiral tool lost its faceted source", ErrUnsupported)
	}
	result := &spiralCombinedAdmission{source: source,
		roles: make(map[[2]*Face]spiralNearPairKind), allowed: make(map[[2]int]bool),
		short: short, groups: make([]map[int]int, len(source.tools))}
	start := 0
	for copyIndex, tool := range source.tools {
		part, ok := tool.payload.(facetedPayload)
		if !ok {
			return nil, fmt.Errorf("%w: spiral copy lost its faceted source", ErrUnsupported)
		}
		copyFaces := tool.Faces()
		mapped := make(map[*Face]*Face)
		reverse := make(map[*Face]*Face)
		groupMap := make(map[int]int)
		if len(part.tris) != len(firstPart.tris) || start+len(part.tris) > len(combinedPart.tris) {
			return nil, fmt.Errorf("%w: combined tooth face layout changed", ErrUnsupported)
		}
		for j := range part.tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if part.src[j] != firstPart.src[j] ||
				firstPart.faceOf[j] < 0 || firstPart.faceOf[j] >= len(first.Faces()) ||
				part.faceOf[j] < 0 || part.faceOf[j] >= len(copyFaces) ||
				combinedPart.faceOf[start+j] < 0 || combinedPart.faceOf[start+j] >= len(allFaces) {
				return nil, fmt.Errorf("%w: copied spiral face lost source provenance", ErrUnsupported)
			}
			original := first.Faces()[firstPart.faceOf[j]]
			copyFace := copyFaces[part.faceOf[j]]
			compositeFace := allFaces[combinedPart.faceOf[start+j]]
			if owner := combinedPart.spiralTools.owner[compositeFace]; owner != copyFace {
				return nil, fmt.Errorf("%w: combined face owner changed", ErrUnsupported)
			}
			if prior := mapped[original]; prior != nil && prior != compositeFace {
				return nil, fmt.Errorf("%w: source face split across a copied tooth", ErrUnsupported)
			}
			if prior := reverse[compositeFace]; prior != nil && prior != original {
				return nil, fmt.Errorf("%w: distinct tooth roles merged", ErrUnsupported)
			}
			mapped[original], reverse[compositeFace] = compositeFace, original
			global, globalOK := faceID[compositeFace]
			local, localOK := firstID[original]
			if !globalOK || !localOK || local < len(blank.Faces()) {
				return nil, fmt.Errorf("%w: copied tooth lost a source group", ErrUnsupported)
			}
			groupMap[global] = local
		}
		for pair, role := range roles {
			copyFace := mapped[pair[1]]
			if copyFace == nil {
				return nil, fmt.Errorf("%w: copied tooth lost a near-face role", ErrUnsupported)
			}
			result.roles[[2]*Face{pair[0], copyFace}] = role
		}
		for pair := range firstAllowed {
			original := first.Faces()[pair[1]-len(blank.Faces())]
			copyFace := mapped[original]
			group, present := faceID[copyFace]
			if copyFace == nil || !present {
				return nil, fmt.Errorf("%w: copied tooth lost an allowed contact", ErrUnsupported)
			}
			result.allowed[[2]int{pair[0], group}] = true
		}
		start += len(part.tris)
		result.groups[copyIndex] = groupMap
	}
	if start != len(combinedPart.tris) || len(result.roles) != 14*len(source.tools) ||
		len(result.allowed) != 24*len(source.tools) {
		return nil, fmt.Errorf("%w: combined spiral face role count changed", ErrUnsupported)
	}
	return result, nil
}

func spiralValidateCombinedUnion(ctx context.Context, blank *Body, blankHeld *meshbool.BoolMesh,
	result *meshbool.StitchedMesh, admission *spiralCombinedAdmission) (bool, error) {
	if ctx == nil || blank == nil || blankHeld == nil || result == nil || admission == nil ||
		admission.source == nil || len(admission.groups) != len(admission.source.tools) {
		return false, nil
	}
	firstAdmission := &spiralFirstUnionAdmission{
		source: admission.source.first, short: admission.short,
		rootBoxes: new([2]*spiralRootFlankSourceBoxes),
	}
	for side := range firstAdmission.rootBoxes {
		boxes, ok, err := spiralPrepareRootFlankSource(ctx, firstAdmission.source, side)
		if err != nil || !ok {
			return false, err
		}
		firstAdmission.rootBoxes[side] = boxes
	}
	type verdict struct {
		accepted bool
		err      error
	}
	results := make([]verdict, len(admission.source.moves))
	jobs := make(chan int, len(results))
	for i := range results {
		jobs <- i
	}
	close(jobs)
	var workers sync.WaitGroup
	for range min(4, len(results)) {
		workers.Go(func() {
			for i := range jobs {
				results[i].accepted, results[i].err = spiralValidateCombinedCopy(
					ctx, blank, blankHeld, result, admission, firstAdmission, i)
			}
		})
	}
	workers.Wait()
	if err := ctx.Err(); err != nil {
		return false, err
	}
	for _, result := range results {
		if result.err != nil || !result.accepted {
			return false, result.err
		}
	}
	return true, nil
}

// Each copy gets private source IDs and inverse vertex/bound slices. The
// certified blank, source records and stitched triangles are read only.
func spiralValidateCombinedCopy(ctx context.Context, blank *Body, blankHeld *meshbool.BoolMesh,
	result *meshbool.StitchedMesh, admission *spiralCombinedAdmission,
	firstAdmission *spiralFirstUnionAdmission, copyIndex int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	first := admission.source.tools[0]
	blankFaces := len(blank.Faces())
	local := *result
	local.Src = make([]int, len(result.Src))
	for i, group := range result.Src {
		switch {
		case group >= 0 && group < blankFaces:
			local.Src[i] = group
		case admission.groups[copyIndex][group] >= blankFaces:
			local.Src[i] = admission.groups[copyIndex][group]
		default:
			local.Src[i] = -1
		}
	}
	radialExtra := 0.0
	if copyIndex != 0 {
		var ok bool
		local.Verts, local.VertexBound, local.Round, radialExtra, ok =
			spiralInverseHeldResult(ctx, blankHeld, first, result, admission.source.moves[copyIndex])
		if !ok {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			return false, nil
		}
	}
	return spiralValidateFirstUnion(ctx, blank, first,
		blankHeld, &local, firstAdmission, radialExtra)
}

// Inverse placement is used only for comparing output chains with their
// unplaced source curves. The certified output β is stretched and charged for
// inverse float rounding, the residual tooth transform, and the blank's
// slight radial scaling under the stored axis-rotation matrix.
func spiralInverseHeldResult(ctx context.Context, blankHeld *meshbool.BoolMesh,
	first *Body, held *meshbool.StitchedMesh, move r3.Transform) (
	[]r3.Vec, []float64, float64, float64, bool,
) {
	inv, err := move.Inverse()
	if err != nil {
		return nil, nil, 0, 0, false
	}
	forward, err := massmoment.PlacementRotation(move)
	if err != nil {
		return nil, nil, 0, 0, false
	}
	back, err := massmoment.PlacementRotation(inv)
	if err != nil {
		return nil, nil, 0, 0, false
	}
	// The admitted near-axis motion has no X/YZ cross terms. Keep the local
	// blank ring's held X coordinate exact, and charge the omitted one-ULP
	// axial inverse through the canonical back*forward residual below.
	back[0][0] = lfOne()
	back[0][1], back[0][2] = lfZero(), lfZero()
	charge, err := massmoment.MapChargeOf(back)
	if err != nil {
		return nil, nil, 0, 0, false
	}
	firstPart, ok := first.payload.(facetedPayload)
	if !ok || len(held.VertexBound) != len(held.Verts) || len(firstPart.verts) == 0 {
		return nil, nil, 0, 0, false
	}
	maxTooth, maxBlank := 0.0, 0.0
	for _, v := range firstPart.verts {
		maxTooth = max(maxTooth, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	for _, v := range blankHeld.Verts {
		maxBlank = max(maxBlank, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	toothRadius := proofbound.AbsSumUpper(proofbound.Radius3D(maxTooth), firstPart.meshBound)
	blankRadius := proofbound.AbsSumUpper(proofbound.Radius3D(maxBlank),
		facetproof.FacetBoundMax(blankHeld.Tris, blankHeld.VertexBound))
	deviation := new(big.Rat)
	for row := range 3 {
		for column := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				entry.Add(entry, lfMul(back[row][k], forward[k][column]))
			}
			if row == column {
				entry.Sub(entry, lfOne())
			}
			deviation.Add(deviation, lfAbs(entry))
		}
	}
	toothExtra := proofbound.ProductUpper(proofbound.RatFloatUp(deviation), toothRadius)
	blankExtra := proofbound.ProductUpper(charge.Stretch, blankRadius)
	radialExtra := blankExtra
	commonExtra := proofbound.AbsSumUpper(toothExtra, blankExtra)
	verts := make([]r3.Vec, len(held.Verts))
	beta := make([]float64, len(held.VertexBound))
	maxExtra := 0.0
	for i, v := range held.Verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, nil, 0, 0, false
			}
		}
		if !proofbound.FiniteVec(v) || proofbound.IsNonFinite(held.VertexBound[i]) {
			return nil, nil, 0, 0, false
		}
		verts[i] = inv.Apply(v)
		verts[i].X = v.X
		if !proofbound.FiniteVec(verts[i]) {
			return nil, nil, 0, 0, false
		}
		round := proofbound.RigidRoundAllow(max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z)), 0)
		extra := proofbound.AbsSumUpper(round, commonExtra)
		beta[i] = proofbound.AbsSumUpper(
			proofbound.ProductUpper(proofbound.AbsSumUpper(1, charge.Stretch), held.VertexBound[i]), extra)
		maxExtra = max(maxExtra, extra)
	}
	return verts, beta, proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, charge.Stretch), held.Round), maxExtra), radialExtra, true
}

// spiralPrepareCombinedUnion transports the first join's certified temporary
// cap mesh to every retained copy. The source-to-proxy volume and boundary
// charges are summed before the ordinary mesh Boolean can consume them.
func spiralPrepareCombinedUnion(ctx context.Context, op meshbool.OperationKind,
	blank, combined *Body, original *Mesh, pairTol float64) (*Mesh, *spiralCombineSource, error) {
	p, ok := combined.payload.(facetedPayload)
	if !ok || p.spiralTools == nil || op != meshbool.OpUnion {
		return original, nil, nil
	}
	meta := p.spiralTools.source
	if meta == nil || len(meta.tools) < 2 || len(meta.tools) != len(meta.moves) ||
		meta.first == nil || meta.first.tooth == nil {
		return nil, nil, fmt.Errorf("%w: combined spiral source is incomplete", ErrUnsupported)
	}
	first := meta.tools[0]
	firstMesh, _, err := tessellateBooleanOperand(ctx, first, pairTol)
	if err != nil {
		return nil, nil, err
	}
	faceID := make(map[*Face]int)
	for i, face := range first.Faces() {
		faceID[face] = len(blank.Faces()) + i
	}
	src, err := meshbool.SourceIDs(ctx, firstMesh.source, faceID)
	if err != nil {
		return nil, nil, err
	}
	proxy, certified, err := spiralOffsetToothMesh(ctx, blank, first, firstMesh, src, meta.first)
	if err != nil || !certified {
		return nil, nil, err
	}
	next := *original
	next.vertices = make([]r3.Vec, 0, len(original.vertices))
	next.vertexBound = make([]float64, 0, len(original.vertices))
	next.volSymDiff, next.areaSlack = 0, 0
	budget := proofbound.NewWorkBudget(ctx)
	// Each proxy shell is audited at its source and kept embedded by PlaceMesh.
	// A separate held-shell clearance proof below replaces the generic whole-
	// composite crossing audit, whose fixed triangle cap excludes 31 copies.
	proxyParts := make([]*Body, 0, len(meta.tools))
	for i, tool := range meta.tools {
		part, ok := tool.payload.(facetedPayload)
		if !ok || len(part.verts) != len(proxy.vertices) || len(part.tris) != len(proxy.triangles) ||
			len(part.src) != len(proxy.triangles) {
			return nil, nil, fmt.Errorf("%w: spiral copy mesh no longer matches source", ErrUnsupported)
		}
		for j, tri := range part.tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, nil, err
				}
			}
			if tri != proxy.triangles[j] {
				return nil, nil, fmt.Errorf("%w: spiral copy triangle correspondence was lost", ErrUnsupported)
			}
		}
		if i == 0 {
			proxyParts = append(proxyParts, &Body{payload: facetedPayload{
				verts: proxy.vertices, tris: proxy.triangles,
				vertexBound: make([]float64, len(proxy.vertices)),
			}})
			next.vertices = append(next.vertices, proxy.vertices...)
			next.vertexBound = append(next.vertexBound, proxy.vertexBound...)
			next.volSymDiff = proofbound.AbsSumUpper(next.volSymDiff, proxy.volSymDiff)
			next.areaSlack = proofbound.AbsSumUpper(next.areaSlack, proxy.areaSlack)
			continue
		}
		linear, err := massmoment.PlacementRotation(meta.moves[i])
		if err != nil {
			return nil, nil, err
		}
		charge, err := massmoment.MapChargeOf(linear)
		if err != nil {
			return nil, nil, err
		}
		placed, err := facetproof.PlaceMesh(ctx, budget, proxy.vertices, proxy.triangles,
			proxy.vertexBound, proxy.volSymDiff, proxy.areaSlack,
			charge.Stretch, charge.Volume, meta.moves[i])
		if err != nil {
			return nil, nil, err
		}
		// The clearance audit concerns the held proxy shells themselves. Bound
		// each placed held vertex against the exact image of the first held
		// proxy; its original source-shape bound belongs to the Boolean charge,
		// not to this crossing audit.
		placementBound := make([]float64, len(placed.Verts))
		for j, v := range placed.Verts {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, nil, err
				}
			}
			exact := meshbool.ExactRigidImage(meta.moves[i].Basis(), meta.moves[i].Translation(),
				proxy.vertices[j])
			placementBound[j] = proofbound.Radius3D(
				proofbound.RatFloatUp(meshbool.CoordDistance(exact, v)))
			if proofbound.IsNonFinite(placementBound[j]) {
				return nil, nil, fmt.Errorf("%w: placed spiral proxy has no finite held bound", ErrUnsupported)
			}
		}
		proxyParts = append(proxyParts, &Body{payload: facetedPayload{
			verts: placed.Verts, tris: placed.Tris, vertexBound: placementBound,
			meshBound: facetproof.FacetBoundMax(placed.Tris, placementBound),
		}})
		next.vertices = append(next.vertices, placed.Verts...)
		next.vertexBound = append(next.vertexBound, placed.VertexBound...)
		next.volSymDiff = proofbound.AbsSumUpper(next.volSymDiff, placed.VolSymDiff)
		next.areaSlack = proofbound.AbsSumUpper(next.areaSlack, placed.AreaSlack)
	}
	if len(next.vertices) != len(original.vertices) || len(next.vertexBound) != len(next.vertices) ||
		proofbound.IsNonFinite(next.volSymDiff) || proofbound.IsNonFinite(next.areaSlack) {
		return nil, nil, fmt.Errorf("%w: combined spiral proxy has no complete bound", ErrUnsupported)
	}
	next.faceBound = nil
	next.bound = 0
	next.setVertexBounds(next.vertexBound)
	next.coordBound = max(next.coordBound, next.bound)
	if !spiralToolSectorsSeparate(ctx, proxyParts, meta.moves, true) {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		return nil, nil, fmt.Errorf("%w: spiral proxy shells are not proven disjoint", ErrUnsupported)
	}
	if err := massmoment.AuditMesh(ctx, next.vertices, next.triangles, true); err != nil {
		return nil, nil, err
	}
	return &next, meta, nil
}

func spiralCombineBody(ctx context.Context, d *Document, ref producerID,
	blank *Body, tools []*Body) (*Body, error) {
	source, err := spiralCombineInputs(ctx, blank, tools)
	if err != nil {
		return nil, err
	}
	combined, err := spiralCombinedToolBody(ctx, d, ref, source)
	if err != nil {
		return nil, err
	}
	eval, err := evaluateBoolean(ctx, meshbool.OpUnion, blank, combined)
	if err != nil {
		return nil, asBooleanError(meshbool.OpUnion, err)
	}
	return buildFacetedBodyWithProof(ctx, d, ref, eval.payload, eval.audit, eval.volume, eval.volumeRat)
}

// spiralCombinedToolBody creates a transient, unregistered disjoint union of
// all input tools. Source patches and their bounds are copied, never blended;
// the pairwise separation certificate above proves its shells are separate.
func spiralCombinedToolBody(ctx context.Context, d *Document, ref producerID,
	source *spiralCombineSource) (*Body, error) {
	meta := &spiralCombinedTools{source: source, owner: make(map[*Face]*Face)}
	p := facetedPayload{xform: r3.Identity(), spiralTools: meta, contactAudited: true}
	firstPart, ok := source.tools[0].payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: first spiral tooth lost its faceted source", ErrUnsupported)
	}
	maxAbs := 0.0
	for copyIndex, tooth := range source.tools {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		part, ok := tooth.payload.(facetedPayload)
		if !ok || len(part.vertexBound) != len(part.verts) ||
			len(part.src) != len(part.tris) || len(part.faceOf) != len(part.tris) {
			return nil, fmt.Errorf("%w: spiral tool has no complete held-mesh proof", ErrUnsupported)
		}
		if copyIndex != 0 {
			if part.spiralTooth == nil || part.spiralTooth.source != source.first.tooth ||
				part.spiralTooth.motion != source.moves[copyIndex] || part.xform != source.moves[copyIndex] ||
				len(part.verts) != len(firstPart.verts) || len(part.tris) != len(firstPart.tris) {
				return nil, fmt.Errorf("%w: spiral copy lost its audited source", ErrUnsupported)
			}
			for j, tri := range part.tris {
				if j%256 == 0 {
					if err := ctx.Err(); err != nil {
						return nil, err
					}
				}
				if tri != firstPart.tris[j] {
					return nil, fmt.Errorf("%w: spiral copy changed the source facets", ErrUnsupported)
				}
			}
		}
		// The first audit proves the unplaced trim embedded. Each authenticated
		// copy was built by PlaceMesh, which keeps its held image embedded.
		if err := massmoment.AuditMesh(ctx, part.verts, part.tris,
			copyIndex != 0 || part.contactAudited); err != nil {
			return nil, err
		}
		vertexOffset, groupOffset := len(p.verts), len(p.groups)
		p.verts = append(p.verts, part.verts...)
		p.vertexBound = append(p.vertexBound, part.vertexBound...)
		p.groups = append(p.groups, part.groups...)
		for i, tri := range part.tris {
			p.tris = append(p.tris, [3]int{
				tri[0] + vertexOffset, tri[1] + vertexOffset, tri[2] + vertexOffset,
			})
			p.src = append(p.src, part.src[i]+groupOffset)
		}
		p.meshBound = max(p.meshBound, part.meshBound)
		p.volSymDiff = proofbound.AbsSumUpper(p.volSymDiff, part.volSymDiff)
		p.areaSlack = proofbound.AbsSumUpper(p.areaSlack, part.areaSlack)
		for _, v := range part.verts {
			maxAbs = max(maxAbs, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
		}
	}
	p.dPair = proofbound.ProductUpper(2, proofbound.AbsSumUpper(
		proofbound.Radius3D(maxAbs), p.meshBound))
	if proofbound.IsNonFinite(p.dPair) || proofbound.IsNonFinite(p.volSymDiff) ||
		proofbound.IsNonFinite(p.areaSlack) {
		return nil, fmt.Errorf("%w: combined tool has no finite measurement bounds", ErrUnsupported)
	}
	body, err := buildFacetedBody(ctx, d, ref, p)
	if err != nil {
		return nil, err
	}
	composite, ok := body.payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: combined spiral tool lost its faceted source", ErrBooleanFailed)
	}
	faces := body.Faces()
	triangle := 0
	for _, tooth := range source.tools {
		part, ok := tooth.payload.(facetedPayload)
		if !ok {
			return nil, fmt.Errorf("%w: spiral copy lost its faceted source", ErrBooleanFailed)
		}
		original := tooth.Faces()
		for i := range part.tris {
			if part.faceOf[i] < 0 || part.faceOf[i] >= len(original) ||
				composite.faceOf[triangle] < 0 || composite.faceOf[triangle] >= len(faces) {
				return nil, fmt.Errorf("%w: combined tool lost a source face", ErrBooleanFailed)
			}
			copyFace, liveFace := faces[composite.faceOf[triangle]], original[part.faceOf[i]]
			if prior, found := meta.owner[copyFace]; found && prior != liveFace {
				return nil, fmt.Errorf("%w: combined tool merged distinct source faces", ErrBooleanFailed)
			}
			meta.owner[copyFace] = liveFace
			triangle++
		}
	}
	return body, nil
}
