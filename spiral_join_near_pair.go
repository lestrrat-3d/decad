package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
)

// spiralNearPairKind is the source verdict for one otherwise undecidable
// held-face pair in the first curved tooth join. A caller must still audit
// the complete held result and refuse any near pair absent from this table.
type spiralNearPairKind uint8

const (
	spiralNearPairUnknown spiralNearPairKind = iota
	spiralNearPairCrossing
	spiralNearPairSeparated
)

// A straight cone meridian is affine in X. Strict signs at both recorded
// endpoints prove the same side over the entire finite generator. The far
// plane and apex checks ensure this sign describes the operative cone wall.
func spiralFiniteConeSide(cone *pointConeLimit, first, last planePoint, sign int) bool {
	if cone == nil || cone.Apex == nil || cone.Far == nil || cone.Slope == nil ||
		cone.Slope.Sign() <= 0 || cone.Far.Sign() >= 0 ||
		first.U <= 0 || last.U <= first.U || first.V <= 0 || last.V <= 0 ||
		lfF(first.U).Cmp(cone.Far) <= 0 || lfF(last.U).Cmp(cone.Apex) >= 0 {
		return false
	}
	residual := func(p planePoint) *big.Rat {
		return lfSub(lfAdd(lfF(p.U), lfMul(cone.Slope, lfF(p.V))), cone.Apex)
	}
	return residual(first).Sign() == sign && residual(last).Sign() == sign
}

// The two trims retain a LoftFaces wall where it meets a blank cap only if
// the point is outside the toe Cut and inside the heel Intersect. These exact
// meridian signs prove that fact over both finite blank outer caps. They also
// separate the matching tool-cap faces, the toe cap from the heel tool, and
// the entire finite blank root from both tool caps.
func spiralNearTrimSides(proof *spiralFirstJoinSource) bool {
	if proof == nil || proof.Blank == nil || proof.Tooth == nil ||
		proof.Tooth.Toe == nil || proof.Tooth.Heel == nil ||
		proof.ToeCapGap == nil || proof.HeelCapGap == nil ||
		proof.ToeCapGap.Sign() <= 0 || proof.HeelCapGap.Sign() <= 0 ||
		proof.ToeOffset == nil || proof.HeelOffset == nil ||
		proof.ToeOffset.Sign() >= 0 || proof.HeelOffset.Sign() <= 0 {
		return false
	}
	blank := proof.Blank
	return spiralFiniteConeSide(proof.Tooth.Toe, blank.ToeRoot, blank.ToeOuter, 1) &&
		spiralFiniteConeSide(proof.Tooth.Heel, blank.ToeRoot, blank.ToeOuter, -1) &&
		spiralFiniteConeSide(proof.Tooth.Toe, blank.Ded, blank.HeelOuter, 1) &&
		spiralFiniteConeSide(proof.Tooth.Heel, blank.Ded, blank.HeelOuter, -1) &&
		spiralFiniteConeSide(proof.Tooth.Toe, blank.ToeRoot, blank.Ded, 1) &&
		spiralFiniteConeSide(proof.Tooth.Heel, blank.ToeRoot, blank.Ded, -1)
}

func spiralUniqueFaceWithOrigin(body *Body, want provenance.Ref) *Face {
	if body == nil {
		return nil
	}
	var result *Face
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin != want {
				continue
			}
			if result != nil && result != face {
				return nil
			}
			result = face
		}
	}
	return result
}

// The original LoftFaces groups carry the producer identity that the two
// cone Booleans preserve on a surviving side. Do not infer it from a nearby
// polygon or from the result face's position in its face list.
func spiralOriginalSideRef(trim *loftFacesTrimRecord, band, segment int) (provenance.Ref, bool) {
	if trim == nil || band < 0 || band >= 8 || segment < 0 || segment >= 6 {
		return provenance.Ref{}, false
	}
	role := fmt.Sprintf("side(%d,%d)", band, segment)
	var result provenance.Ref
	found := false
	for _, group := range trim.Base.Groups {
		for _, origin := range group.Origins {
			if origin.Role != role {
				continue
			}
			if found && result != origin {
				return provenance.Ref{}, false
			}
			result, found = origin, true
		}
	}
	return result, found
}

func spiralCapGroupRef(groups []facetGroup, group int) (provenance.Ref, bool) {
	if group < 0 || group >= len(groups) || len(groups[group].Origins) != 1 {
		return provenance.Ref{}, false
	}
	return groups[group].Origins[0], true
}

// spiralNearFaceTable binds the fixed set of source roles for the first
// curved-tooth join to the live input faces. Geometry certificates for the
// nine crossings are checked by the caller before this table is used.
func spiralNearFaceTable(blank, tooth *Body, proof *spiralFirstJoinSource) (
	map[[2]*Face]spiralNearPairKind, bool,
) {
	if blank == nil || tooth == nil || proof == nil || proof.Blank == nil ||
		proof.Tooth == nil || proof.Blank.RootFace == nil ||
		proof.Blank.RootFace.Body != bodyNode(blank) {
		return nil, false
	}
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim != proof.Tooth {
		return nil, false
	}
	blankFace := func(edge int) *Face {
		return spiralUniqueFaceWithOrigin(blank, provenance.Ref{Producer: blank.n.Origin.Producer,
			Role: fmt.Sprintf("side(0,%d)", proof.Blank.EdgeIndex[edge])})
	}
	heel, root, toe := blankFace(2), blankFace(3), blankFace(4)
	if heel == nil || root == nil || toe == nil || faceNode(root) != proof.Blank.RootFace ||
		heel == root || heel == toe || root == toe {
		return nil, false
	}
	toeRef, okToe := spiralCapGroupRef(p.Groups, proof.Tooth.ToeCapGroup)
	heelRef, okHeel := spiralCapGroupRef(p.Groups, proof.Tooth.HeelCapGroup)
	if !okToe || !okHeel || toeRef == heelRef {
		return nil, false
	}
	toeTool, heelTool := spiralUniqueFaceWithOrigin(tooth, toeRef),
		spiralUniqueFaceWithOrigin(tooth, heelRef)
	if toeTool == nil || heelTool == nil || toeTool == heelTool {
		return nil, false
	}
	out := make(map[[2]*Face]spiralNearPairKind, 14)
	add := func(a, b *Face, kind spiralNearPairKind) bool {
		if a == nil || b == nil || a.n.Body != bodyNode(blank) || b.n.Body != bodyNode(tooth) ||
			kind == spiralNearPairUnknown {
			return false
		}
		pair := [2]*Face{a, b}
		if out[pair] != spiralNearPairUnknown {
			return false
		}
		out[pair] = kind
		return true
	}
	for _, side := range []int{0, 2} {
		for _, placement := range [...]struct {
			blank *Face
			band  int
		}{{heel, 7}, {toe, 1}, {root, 1}} {
			origin, found := spiralOriginalSideRef(proof.Tooth, placement.band, side)
			if !found || !add(placement.blank,
				spiralUniqueFaceWithOrigin(tooth, origin), spiralNearPairCrossing) {
				return nil, false
			}
		}
	}
	for _, side := range [...]int{3, 4, 5} {
		origin, found := spiralOriginalSideRef(proof.Tooth, 1, side)
		if !found || !add(toe, spiralUniqueFaceWithOrigin(tooth, origin), spiralNearPairCrossing) {
			return nil, false
		}
	}
	for _, pair := range [...][2]*Face{{heel, heelTool}, {toe, toeTool},
		{toe, heelTool}, {root, toeTool}, {root, heelTool}} {
		if !add(pair[0], pair[1], spiralNearPairSeparated) {
			return nil, false
		}
	}
	return out, len(out) == 14
}

// certifySpiralNearFaceRoles is a reject-only source classifier for the
// first unplaced curved-tooth Union. It never changes the generic proximity
// gate. The caller may use the returned roles only while the corresponding
// source-certified held output proves every crossing chain and every
// separated held face pair has no facet contact.
func certifySpiralNearFaceRoles(ctx context.Context, blank, tooth *Body,
	proof *spiralFirstJoinSource) (
	map[[2]*Face]spiralNearPairKind, [2][4]spiralsector.ShortCapFamily, bool, error,
) {
	var empty [2][4]spiralsector.ShortCapFamily
	if ctx == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Blank == nil || proof.Tooth == nil || proof.HeelSeam == nil ||
		!spiralNearTrimSides(proof) || !spiralsector.TrimSourceCurrent(proof.Tooth) {
		return nil, empty, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, empty, false, err
	}
	// The existing first-join proof covers two transverse flank/root
	// crossings over all bands. The short-cap certificate covers both
	// finite outer caps against flanks and source connectors. The toe seam
	// covers the one remaining cap/root-arc contact.
	short, ok, err := spiralsector.ShortCapFamilies(ctx, proof)
	if err != nil || !ok {
		return nil, empty, false, err
	}
	toeSeam, ok, err := spiralsector.CertifySpiralToeRootSeam(ctx, proof)
	if err != nil || !ok || toeSeam == nil {
		return nil, empty, false, err
	}
	for end := range short {
		band := 1
		if end == 1 {
			band = 7
		}
		for index, group := range [...]int{0, 2, 3, 5} {
			family := short[end][index]
			if family.Band != band || family.Group != group || len(family.Pieces) != 1 ||
				family.X.Lo == nil || family.X.Hi == nil ||
				family.X.Lo.Cmp(family.X.Hi) >= 0 {
				return nil, empty, false, nil
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, empty, false, err
	}
	roles, ok := spiralNearFaceTable(blank, tooth, proof)
	if !ok {
		return nil, empty, false, nil
	}
	return roles, short, true, nil
}
