package spiralsector

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// CombinedAdmission carries the first tooth's certified face roles to every
// copy of a combined spiral tool: near-face roles, allowed held contacts, the
// short-cap families and, per copy, the map from combined held groups to the
// first tooth's groups.
type CombinedAdmission struct {
	Source  *facetedrecord.SpiralCombineSource
	Roles   map[[2]*topo.Face]meshbool.NearPairRole
	Allowed map[[2]int]bool
	Short   [2][4]ShortCapFamily
	Groups  []map[int]int
}

// CombineFaceRoles binds the first tooth's authenticated face families
// to each copy by the preserved triangle ordering. It refuses any placement
// that changes the source group's patch partition or joins two copy faces.
// faceID maps each blank and combined-tool face to its held group.
func CombineFaceRoles(ctx context.Context, blank, combined *topo.Body,
	source *facetedrecord.SpiralCombineSource, faceID map[*topo.Face]int) (*CombinedAdmission, error) {
	first := source.Tools[0]
	roles, short, ok, err := CertifyNearFaceRoles(ctx, blank, first, source.First)
	if err != nil || !ok {
		return nil, err
	}
	blankFaces := topobuild.Faces(blank)
	firstFaces := topobuild.Faces(first)
	firstID := make(map[*topo.Face]int)
	for i, face := range blankFaces {
		firstID[face] = i
	}
	for i, face := range firstFaces {
		firstID[face] = len(blankFaces) + i
	}
	firstAdmission := &FirstUnionAdmission{Source: source.First, Short: short, Roles: roles}
	firstAllowed, ok := FirstUnionPairs(blank, first, firstAdmission, firstID)
	if !ok {
		return nil, fmt.Errorf("%w: combined source has no first-tooth role table", decaderr.ErrUnsupported)
	}
	firstPart, ok := first.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, fmt.Errorf("%w: first spiral tooth lost its faceted source", decaderr.ErrUnsupported)
	}
	allFaces := topobuild.Faces(combined)
	combinedPart, ok := combined.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, fmt.Errorf("%w: combined spiral tool lost its faceted source", decaderr.ErrUnsupported)
	}
	result := &CombinedAdmission{Source: source,
		Roles: make(map[[2]*topo.Face]meshbool.NearPairRole), Allowed: make(map[[2]int]bool),
		Short: short, Groups: make([]map[int]int, len(source.Tools))}
	start := 0
	for copyIndex, tool := range source.Tools {
		part, ok := tool.Payload.(facetedrecord.Payload)
		if !ok {
			return nil, fmt.Errorf("%w: spiral copy lost its faceted source", decaderr.ErrUnsupported)
		}
		copyFaces := topobuild.Faces(tool)
		mapped := make(map[*topo.Face]*topo.Face)
		reverse := make(map[*topo.Face]*topo.Face)
		groupMap := make(map[int]int)
		if len(part.Tris) != len(firstPart.Tris) || start+len(part.Tris) > len(combinedPart.Tris) {
			return nil, fmt.Errorf("%w: combined tooth face layout changed", decaderr.ErrUnsupported)
		}
		for j := range part.Tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if part.Src[j] != firstPart.Src[j] ||
				firstPart.FaceOf[j] < 0 || firstPart.FaceOf[j] >= len(firstFaces) ||
				part.FaceOf[j] < 0 || part.FaceOf[j] >= len(copyFaces) ||
				combinedPart.FaceOf[start+j] < 0 || combinedPart.FaceOf[start+j] >= len(allFaces) {
				return nil, fmt.Errorf("%w: copied spiral face lost source provenance", decaderr.ErrUnsupported)
			}
			original := firstFaces[firstPart.FaceOf[j]]
			copyFace := copyFaces[part.FaceOf[j]]
			compositeFace := allFaces[combinedPart.FaceOf[start+j]]
			if owner := combinedPart.SpiralTools.Owner[compositeFace]; owner != copyFace {
				return nil, fmt.Errorf("%w: combined face owner changed", decaderr.ErrUnsupported)
			}
			if prior := mapped[original]; prior != nil && prior != compositeFace {
				return nil, fmt.Errorf("%w: source face split across a copied tooth", decaderr.ErrUnsupported)
			}
			if prior := reverse[compositeFace]; prior != nil && prior != original {
				return nil, fmt.Errorf("%w: distinct tooth roles merged", decaderr.ErrUnsupported)
			}
			mapped[original], reverse[compositeFace] = compositeFace, original
			global, globalOK := faceID[compositeFace]
			local, localOK := firstID[original]
			if !globalOK || !localOK || local < len(blankFaces) {
				return nil, fmt.Errorf("%w: copied tooth lost a source group", decaderr.ErrUnsupported)
			}
			groupMap[global] = local
		}
		for pair, role := range roles {
			copyFace := mapped[pair[1]]
			if copyFace == nil {
				return nil, fmt.Errorf("%w: copied tooth lost a near-face role", decaderr.ErrUnsupported)
			}
			result.Roles[[2]*topo.Face{pair[0], copyFace}] = role
		}
		for pair := range firstAllowed {
			original := firstFaces[pair[1]-len(blankFaces)]
			copyFace := mapped[original]
			group, present := faceID[copyFace]
			if copyFace == nil || !present {
				return nil, fmt.Errorf("%w: copied tooth lost an allowed contact", decaderr.ErrUnsupported)
			}
			result.Allowed[[2]int{pair[0], group}] = true
		}
		start += len(part.Tris)
		result.Groups[copyIndex] = groupMap
	}
	if start != len(combinedPart.Tris) || len(result.Roles) != 14*len(source.Tools) ||
		len(result.Allowed) != 24*len(source.Tools) {
		return nil, fmt.Errorf("%w: combined spiral face role count changed", decaderr.ErrUnsupported)
	}
	return result, nil
}

// ValidateCombinedUnion checks every copy's switched output curves against
// the first tooth's source seams, at most four copies at a time.
func ValidateCombinedUnion(ctx context.Context, blank *topo.Body, blankHeld *meshbool.BoolMesh,
	result *meshbool.StitchedMesh, admission *CombinedAdmission) (bool, error) {
	if ctx == nil || blank == nil || blankHeld == nil || result == nil || admission == nil ||
		admission.Source == nil || len(admission.Groups) != len(admission.Source.Tools) {
		return false, nil
	}
	firstAdmission := &FirstUnionAdmission{
		Source: admission.Source.First, Short: admission.Short,
		RootBoxes: new([2]*RootFlankSourceBoxes),
	}
	for side := range firstAdmission.RootBoxes {
		boxes, ok, err := PrepareRootFlankSource(ctx, firstAdmission.Source, side)
		if err != nil || !ok {
			return false, err
		}
		firstAdmission.RootBoxes[side] = boxes
	}
	type verdict struct {
		accepted bool
		err      error
	}
	results := make([]verdict, len(admission.Source.Moves))
	jobs := make(chan int, len(results))
	for i := range results {
		jobs <- i
	}
	close(jobs)
	var workers sync.WaitGroup
	for range min(4, len(results)) {
		workers.Go(func() {
			for i := range jobs {
				results[i].accepted, results[i].err = validateCombinedCopy(
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
// Inverse placement is used only for comparing output chains with their
// unplaced source curves. The certified output β is charged for the float
// inverse's gap to the exact record's transpose and for its rounding.
func validateCombinedCopy(ctx context.Context, blank *topo.Body, blankHeld *meshbool.BoolMesh,
	result *meshbool.StitchedMesh, admission *CombinedAdmission,
	firstAdmission *FirstUnionAdmission, copyIndex int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	first := admission.Source.Tools[0]
	blankFaces := len(topobuild.Faces(blank))
	local := *result
	local.Src = make([]int, len(result.Src))
	for i, group := range result.Src {
		switch {
		case group >= 0 && group < blankFaces:
			local.Src[i] = group
		case admission.Groups[copyIndex][group] >= blankFaces:
			local.Src[i] = admission.Groups[copyIndex][group]
		default:
			local.Src[i] = -1
		}
	}
	radialExtra := 0.0
	if copyIndex != 0 {
		firstPart, ok := first.Payload.(facetedrecord.Payload)
		if ok {
			local.Verts, local.VertexBound, local.Round, radialExtra, ok =
				InverseHeldResult(ctx, blankHeld, firstPart.Verts, firstPart.MeshBound, result,
					admission.Source.Moves[copyIndex], admission.Source.Copies.Exact[copyIndex])
		}
		if !ok {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			return false, nil
		}
	}
	return ValidateFirstUnion(ctx, blank, first, blankHeld, &local, firstAdmission, radialExtra)
}

// CombinedToolPayload assembles the faceted payload of a transient,
// unregistered disjoint union of all input tools. Source patches and their
// bounds are copied, never blended; the pairwise separation certificate of
// source.Copies proves its shells are separate. The payload's SpiralTools
// owner map stays empty until BindCombinedOwners reads the built body.
func CombinedToolPayload(ctx context.Context, source *facetedrecord.SpiralCombineSource) (facetedrecord.Payload, error) {
	meta := &facetedrecord.SpiralCombinedTools{Source: source, Owner: make(map[*topo.Face]*topo.Face)}
	p := facetedrecord.Payload{Xform: r3.Identity(), SpiralTools: meta, ContactAudited: true}
	firstPart, ok := source.Tools[0].Payload.(facetedrecord.Payload)
	if !ok {
		return facetedrecord.Payload{}, fmt.Errorf("%w: first spiral tooth lost its faceted source", decaderr.ErrUnsupported)
	}
	maxAbs := 0.0
	for copyIndex, tooth := range source.Tools {
		if err := ctx.Err(); err != nil {
			return facetedrecord.Payload{}, err
		}
		part, ok := tooth.Payload.(facetedrecord.Payload)
		if !ok || len(part.VertexBound) != len(part.Verts) ||
			len(part.Src) != len(part.Tris) || len(part.FaceOf) != len(part.Tris) {
			return facetedrecord.Payload{}, fmt.Errorf("%w: spiral tool has no complete held-mesh proof",
				decaderr.ErrUnsupported)
		}
		if copyIndex != 0 {
			if source.Copies == nil || source.Copies.Tools[copyIndex] != tooth ||
				part.CopyLineage == nil || part.CopyLineage.Source != source.Tools[0] ||
				part.CopyLineage.Motion != source.Moves[copyIndex] ||
				part.CopyLineage.Exact == nil || part.CopyLineage.Exact != source.Copies.Exact[copyIndex] ||
				len(part.Verts) != len(firstPart.Verts) || len(part.Tris) != len(firstPart.Tris) {
				return facetedrecord.Payload{}, fmt.Errorf("%w: spiral copy lost its audited source",
					decaderr.ErrUnsupported)
			}
			for j, tri := range part.Tris {
				if j%256 == 0 {
					if err := ctx.Err(); err != nil {
						return facetedrecord.Payload{}, err
					}
				}
				if tri != firstPart.Tris[j] {
					return facetedrecord.Payload{}, fmt.Errorf("%w: spiral copy changed the source facets",
						decaderr.ErrUnsupported)
				}
			}
		}
		// The first audit proves the unplaced trim embedded. Each authenticated
		// copy was built by PlaceMesh, which keeps its held image embedded.
		if err := massmoment.AuditMesh(ctx, part.Verts, part.Tris,
			copyIndex != 0 || part.ContactAudited); err != nil {
			return facetedrecord.Payload{}, err
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
		return facetedrecord.Payload{}, fmt.Errorf("%w: combined tool has no finite measurement bounds",
			decaderr.ErrUnsupported)
	}
	return p, nil
}

// BindCombinedOwners records, in meta.Owner, the live tool face each face of
// the built combined body copies, read through the preserved triangle order.
// It refuses when a composite face would copy two distinct source faces.
func BindCombinedOwners(body *topo.Body, meta *facetedrecord.SpiralCombinedTools) error {
	composite, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return fmt.Errorf("%w: combined spiral tool lost its faceted source", decaderr.ErrBooleanFailed)
	}
	faces := topobuild.Faces(body)
	triangle := 0
	for _, tooth := range meta.Source.Tools {
		part, ok := tooth.Payload.(facetedrecord.Payload)
		if !ok {
			return fmt.Errorf("%w: spiral copy lost its faceted source", decaderr.ErrBooleanFailed)
		}
		original := topobuild.Faces(tooth)
		for i := range part.Tris {
			if part.FaceOf[i] < 0 || part.FaceOf[i] >= len(original) ||
				composite.FaceOf[triangle] < 0 || composite.FaceOf[triangle] >= len(faces) {
				return fmt.Errorf("%w: combined tool lost a source face", decaderr.ErrBooleanFailed)
			}
			copyFace, liveFace := faces[composite.FaceOf[triangle]], original[part.FaceOf[i]]
			if prior, found := meta.Owner[copyFace]; found && prior != liveFace {
				return fmt.Errorf("%w: combined tool merged distinct source faces", decaderr.ErrBooleanFailed)
			}
			meta.Owner[copyFace] = liveFace
			triangle++
		}
	}
	return nil
}
