package decad

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type spiralCombinedAdmission struct {
	source  *spiralCombineSource
	roles   map[[2]*Face]spiralNearPairKind
	allowed map[[2]int]bool
	short   [2][4]spiralShortCapFamily
	groups  []map[int]int
}

// spiralCombineFaceRoles binds the first tooth's authenticated face families
// to each copy by the preserved triangle ordering. It refuses any placement
// that changes the source group's patch partition or joins two copy faces.
func spiralCombineFaceRoles(ctx context.Context, blank, combined *Body,
	source *spiralCombineSource, faceID map[*Face]int) (*spiralCombinedAdmission, error) {
	first := bodyHandle(source.Tools[0])
	roles, short, ok, err := certifySpiralNearFaceRoles(ctx, blank, first, source.First)
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
	firstAdmission := &spiralFirstUnionAdmission{source: source.First, short: short, roles: roles}
	firstAllowed, ok := spiralFirstUnionPairs(blank, first, firstAdmission, firstID)
	if !ok {
		return nil, fmt.Errorf("%w: combined source has no first-tooth role table", ErrUnsupported)
	}
	firstPart, ok := first.n.Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: first spiral tooth lost its faceted source", ErrUnsupported)
	}
	allFaces := combined.Faces()
	combinedPart, ok := combined.n.Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: combined spiral tool lost its faceted source", ErrUnsupported)
	}
	result := &spiralCombinedAdmission{source: source,
		roles: make(map[[2]*Face]spiralNearPairKind), allowed: make(map[[2]int]bool),
		short: short, groups: make([]map[int]int, len(source.Tools))}
	start := 0
	for copyIndex, tool := range source.Tools {
		part, ok := tool.Payload.(facetedPayload)
		if !ok {
			return nil, fmt.Errorf("%w: spiral copy lost its faceted source", ErrUnsupported)
		}
		copyFaces := bodyHandle(tool).Faces()
		mapped := make(map[*Face]*Face)
		reverse := make(map[*Face]*Face)
		groupMap := make(map[int]int)
		if len(part.Tris) != len(firstPart.Tris) || start+len(part.Tris) > len(combinedPart.Tris) {
			return nil, fmt.Errorf("%w: combined tooth face layout changed", ErrUnsupported)
		}
		for j := range part.Tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if part.Src[j] != firstPart.Src[j] ||
				firstPart.FaceOf[j] < 0 || firstPart.FaceOf[j] >= len(first.Faces()) ||
				part.FaceOf[j] < 0 || part.FaceOf[j] >= len(copyFaces) ||
				combinedPart.FaceOf[start+j] < 0 || combinedPart.FaceOf[start+j] >= len(allFaces) {
				return nil, fmt.Errorf("%w: copied spiral face lost source provenance", ErrUnsupported)
			}
			original := first.Faces()[firstPart.FaceOf[j]]
			copyFace := copyFaces[part.FaceOf[j]]
			compositeFace := allFaces[combinedPart.FaceOf[start+j]]
			if owner := combinedPart.SpiralTools.Owner[faceNode(compositeFace)]; owner != faceNode(copyFace) {
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
		start += len(part.Tris)
		result.groups[copyIndex] = groupMap
	}
	if start != len(combinedPart.Tris) || len(result.roles) != 14*len(source.Tools) ||
		len(result.allowed) != 24*len(source.Tools) {
		return nil, fmt.Errorf("%w: combined spiral face role count changed", ErrUnsupported)
	}
	return result, nil
}

func spiralValidateCombinedUnion(ctx context.Context, blank *Body, blankHeld *meshbool.BoolMesh,
	result *meshbool.StitchedMesh, admission *spiralCombinedAdmission) (bool, error) {
	if ctx == nil || blank == nil || blankHeld == nil || result == nil || admission == nil ||
		admission.source == nil || len(admission.groups) != len(admission.source.Tools) {
		return false, nil
	}
	firstAdmission := &spiralFirstUnionAdmission{
		source: admission.source.First, short: admission.short,
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
	results := make([]verdict, len(admission.source.Moves))
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
	first := bodyHandle(admission.source.Tools[0])
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
			spiralInverseHeldResult(ctx, blankHeld, first, result, admission.source.Moves[copyIndex],
				admission.source.Copies.Exact[copyIndex])
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
// unplaced source curves. The certified output β is charged for the float
// inverse's gap to the exact record's transpose and for its rounding.
func spiralInverseHeldResult(ctx context.Context, blankHeld *meshbool.BoolMesh,
	first *Body, held *meshbool.StitchedMesh, move r3.Transform, rec *exactrotation.Rotation) (
	[]r3.Vec, []float64, float64, float64, bool,
) {
	firstPart, ok := first.n.Payload.(facetedPayload)
	if !ok {
		return nil, nil, 0, 0, false
	}
	return spiralsector.InverseHeldResult(ctx, blankHeld, firstPart.Verts, firstPart.MeshBound, held, move, rec)
}

// spiralPrepareCombinedUnion transports the first join's certified temporary
// cap mesh to every retained copy. The source-to-proxy volume and boundary
// charges are summed before the ordinary mesh Boolean can consume them.
func spiralPrepareCombinedUnion(ctx context.Context, op meshbool.OperationKind,
	blank, combined *Body, original *Mesh, pairTol float64) (*Mesh, *spiralCombineSource, error) {
	p, ok := combined.n.Payload.(facetedPayload)
	if !ok || p.SpiralTools == nil || op != meshbool.OpUnion {
		return original, nil, nil
	}
	meta := p.SpiralTools.Source
	if meta == nil || len(meta.Tools) < 2 || len(meta.Tools) != len(meta.Moves) ||
		meta.First == nil || meta.First.Tooth == nil {
		return nil, nil, fmt.Errorf("%w: combined spiral source is incomplete", ErrUnsupported)
	}
	first := bodyHandle(meta.Tools[0])
	firstMesh, _, err := tessellateBooleanOperand(ctx, first, pairTol)
	if err != nil {
		return nil, nil, err
	}
	faceID := make(map[*topo.Face]int)
	for i, face := range first.Faces() {
		faceID[faceNode(face)] = len(blank.Faces()) + i
	}
	src, err := meshbool.SourceIDs(ctx, firstMesh.n.Source, faceID)
	if err != nil {
		return nil, nil, err
	}
	proxy, certified, err := spiralOffsetToothMesh(ctx, blank, first, firstMesh, src, meta.First)
	if err != nil || !certified {
		return nil, nil, err
	}
	placedParts, err := placedProxyShells(ctx, meta.Copies, proxy)
	if err != nil {
		return nil, nil, err
	}
	next, err := spiralCombinedProxyMesh(original, placedParts)
	if err != nil {
		return nil, nil, err
	}
	if err := massmoment.AuditMesh(ctx, next.n.Vertices, next.n.Triangles, true); err != nil {
		return nil, nil, err
	}
	return next, meta, nil
}

// spiralCombinedProxyMesh copies the combined operand's mesh and gives its
// restated face bounds a private map before writing any entries.
func spiralCombinedProxyMesh(original *Mesh, placedParts []facetproof.PlacedMesh) (*Mesh, error) {
	next := bornMesh(&Mesh{n: original.n})
	next.n.Vertices = make([]r3.Vec, 0, len(original.n.Vertices))
	next.n.VertexBound = make([]float64, 0, len(original.n.Vertices))
	next.n.VolSymDiff, next.n.AreaSlack = 0, 0
	for _, placed := range placedParts {
		next.n.Vertices = append(next.n.Vertices, placed.Verts...)
		next.n.VertexBound = append(next.n.VertexBound, placed.VertexBound...)
		next.n.VolSymDiff = proofbound.AbsSumUpper(next.n.VolSymDiff, placed.VolSymDiff)
		next.n.AreaSlack = proofbound.AbsSumUpper(next.n.AreaSlack, placed.AreaSlack)
	}
	if len(next.n.Vertices) != len(original.n.Vertices) || len(next.n.VertexBound) != len(next.n.Vertices) ||
		proofbound.IsNonFinite(next.n.VolSymDiff) || proofbound.IsNonFinite(next.n.AreaSlack) {
		return nil, fmt.Errorf("%w: combined spiral proxy has no complete bound", ErrUnsupported)
	}
	next.n.FaceBound = nil
	next.n.Bound = 0
	next.setVertexBounds(next.n.VertexBound)
	next.n.CoordBound = max(next.n.CoordBound, next.n.Bound)
	return next, nil
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
	return bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), eval.payload, eval.audit, measurementToInternal(eval.volume), eval.volumeRat))
}

// spiralCombinedToolBody creates a transient, unregistered disjoint union of
// all input tools. Source patches and their bounds are copied, never blended;
// the pairwise separation certificate above proves its shells are separate.
func spiralCombinedToolBody(ctx context.Context, d *Document, ref producerID,
	source *spiralCombineSource) (*Body, error) {
	meta := &spiralCombinedTools{Source: source, Owner: make(map[*topo.Face]*topo.Face)}
	p := facetedPayload{Xform: r3.Identity(), SpiralTools: meta, ContactAudited: true}
	firstPart, ok := source.Tools[0].Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: first spiral tooth lost its faceted source", ErrUnsupported)
	}
	maxAbs := 0.0
	for copyIndex, tooth := range source.Tools {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		part, ok := tooth.Payload.(facetedPayload)
		if !ok || len(part.VertexBound) != len(part.Verts) ||
			len(part.Src) != len(part.Tris) || len(part.FaceOf) != len(part.Tris) {
			return nil, fmt.Errorf("%w: spiral tool has no complete held-mesh proof", ErrUnsupported)
		}
		if copyIndex != 0 {
			if source.Copies == nil || source.Copies.Tools[copyIndex] != tooth ||
				part.CopyLineage == nil || part.CopyLineage.Source != source.Tools[0] ||
				part.CopyLineage.Motion != source.Moves[copyIndex] ||
				part.CopyLineage.Exact == nil || part.CopyLineage.Exact != source.Copies.Exact[copyIndex] ||
				len(part.Verts) != len(firstPart.Verts) || len(part.Tris) != len(firstPart.Tris) {
				return nil, fmt.Errorf("%w: spiral copy lost its audited source", ErrUnsupported)
			}
			for j, tri := range part.Tris {
				if j%256 == 0 {
					if err := ctx.Err(); err != nil {
						return nil, err
					}
				}
				if tri != firstPart.Tris[j] {
					return nil, fmt.Errorf("%w: spiral copy changed the source facets", ErrUnsupported)
				}
			}
		}
		// The first audit proves the unplaced trim embedded. Each authenticated
		// copy was built by PlaceMesh, which keeps its held image embedded.
		if err := massmoment.AuditMesh(ctx, part.Verts, part.Tris,
			copyIndex != 0 || part.ContactAudited); err != nil {
			return nil, err
		}
		vertexOffset, groupOffset := len(p.Verts), len(p.Groups)
		p.Verts = append(p.Verts, part.Verts...)
		p.VertexBound = append(p.VertexBound, part.VertexBound...)
		p.Groups = append(p.Groups, part.Groups...)
		for i, tri := range part.Tris {
			p.Tris = append(p.Tris, [3]int{
				tri[0] + vertexOffset, tri[1] + vertexOffset, tri[2] + vertexOffset,
			})
			p.Src = append(p.Src, part.Src[i]+groupOffset)
		}
		p.MeshBound = max(p.MeshBound, part.MeshBound)
		p.VolSymDiff = proofbound.AbsSumUpper(p.VolSymDiff, part.VolSymDiff)
		p.AreaSlack = proofbound.AbsSumUpper(p.AreaSlack, part.AreaSlack)
		for _, v := range part.Verts {
			maxAbs = max(maxAbs, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
		}
	}
	p.DPair = proofbound.ProductUpper(2, proofbound.AbsSumUpper(
		proofbound.Radius3D(maxAbs), p.MeshBound))
	if proofbound.IsNonFinite(p.DPair) || proofbound.IsNonFinite(p.VolSymDiff) ||
		proofbound.IsNonFinite(p.AreaSlack) {
		return nil, fmt.Errorf("%w: combined tool has no finite measurement bounds", ErrUnsupported)
	}
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), p))
	if err != nil {
		return nil, err
	}
	composite, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: combined spiral tool lost its faceted source", ErrBooleanFailed)
	}
	faces := body.Faces()
	triangle := 0
	for _, tooth := range source.Tools {
		part, ok := tooth.Payload.(facetedPayload)
		if !ok {
			return nil, fmt.Errorf("%w: spiral copy lost its faceted source", ErrBooleanFailed)
		}
		original := bodyHandle(tooth).Faces()
		for i := range part.Tris {
			if part.FaceOf[i] < 0 || part.FaceOf[i] >= len(original) ||
				composite.FaceOf[triangle] < 0 || composite.FaceOf[triangle] >= len(faces) {
				return nil, fmt.Errorf("%w: combined tool lost a source face", ErrBooleanFailed)
			}
			copyFace, liveFace := faces[composite.FaceOf[triangle]], original[part.FaceOf[i]]
			if prior, found := meta.Owner[faceNode(copyFace)]; found && prior != faceNode(liveFace) {
				return nil, fmt.Errorf("%w: combined tool merged distinct source faces", ErrBooleanFailed)
			}
			meta.Owner[faceNode(copyFace)] = faceNode(liveFace)
			triangle++
		}
	}
	return body, nil
}
