package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func exactSpiralBlank(blank *Body) (*spiralBlankSource, bool) {
	for _, section := range revolveRootCandidates(blank) {
		source, ok := spiralsector.SixSideBlank(section)
		if ok {
			return source, true
		}
	}
	return nil, false
}

// certifySpiralFirstJoin proves that each of the two fitted flanks crosses
// the blank's recorded root meridian once and transversely through all eight
// continuous LoftFaces bands. It also proves that both root branches move
// strictly from toe to heel. The finite toe/heel transitions are retained as
// signed exact offsets for the cap-strip proof; they are not collapsed to a
// falsely shared ring.
func certifySpiralFirstJoin(ctx context.Context, blank, tooth *Body) (*spiralFirstJoinSource, bool, error) {
	if ctx == nil || blank == nil || tooth == nil {
		return nil, false, nil
	}
	section, ok := exactSpiralBlank(blank)
	if !ok {
		return nil, false, nil
	}
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim == nil || p.Xform != r3.Identity() {
		return nil, false, nil
	}
	trim := p.LoftFacesTrim
	if !spiralsector.TrimSourceCurrent(trim) || !facetedtopology.CapGroupsPresent(p.Src, p.FaceOf, len(p.Groups),
		trim.ToeCapGroup, trim.HeelCapGroup) {
		return nil, false, nil
	}
	m, b := section.Section.MeridianEquation()
	if !spiralsector.SourcePhysicalSector(trim, m, b) {
		return nil, false, nil
	}
	if !spiralsector.OtherSides(ctx, trim, m, b) {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		return nil, false, nil
	}
	toeX, ok := spiralsector.RootToolX(m, b, trim.Toe)
	if !ok {
		return nil, false, nil
	}
	heelX, ok := spiralsector.RootToolX(m, b, trim.Heel)
	if !ok || toeX.Cmp(heelX) >= 0 ||
		lfAdd(lfMul(m, toeX), b).Sign() <= 0 ||
		lfAdd(lfMul(m, heelX), b).Sign() <= 0 {
		return nil, false, nil
	}
	toeGap, ok := spiralsector.ToolOuterGap(trim.Toe, section.ToeRoot, section.ToeOuter, 1)
	if !ok {
		return nil, false, nil
	}
	heelGap, ok := spiralsector.ToolOuterGap(trim.Heel, section.Ded, section.HeelOuter, -1)
	if !ok {
		return nil, false, nil
	}
	proof := &spiralFirstJoinSource{Blank: section, Tooth: trim, RootM: m, RootB: b,
		ToeX: toeX, HeelX: heelX,
		ToeOffset:  lfSub(toeX, lfF(section.ToeRoot.U)),
		HeelOffset: lfSub(heelX, lfF(section.Ded.U)),
		ToeCapGap:  toeGap, HeelCapGap: heelGap}
	// The real finite root lies between the tool-clipped ends, with a small
	// overhang at each cap. The later cap proof charges both exact offsets.
	if proof.ToeOffset.Sign() >= 0 || proof.HeelOffset.Sign() <= 0 {
		return nil, false, nil
	}
	if !spiralsector.OriginalCapsAvoidToolRootCircles(trim, m, b, toeX, heelX) &&
		!spiralsector.TangentCapTouchesHeelCircle(trim, m, b, toeX, heelX) {
		return nil, false, nil
	}
	for side, group := range [...]int{0, 2} {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		fit, ok := trim.Source.Profile.Outer.Segments[group].(fitSplineSeg)
		if !ok {
			return nil, false, nil
		}
		work := freeform.NewFreeformWork()
		work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		spans, err := splinebezier.FitSplineBezierSpans(fit, work)
		if err != nil {
			return nil, false, err
		}
		if len(spans) != 14 {
			return nil, false, nil
		}
		proof.Flank[side] = spans[0]
		for band := range 8 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			for spanIndex, span := range spans {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				q := loftface.MeridianQRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
				if spanIndex == 0 {
					if !polynomial.BernsteinRowsCross(q) ||
						!loftface.CapXDecreases(trim.Maps[band], span) ||
						!loftface.CapXDecreases(trim.Maps[band+1], span) ||
						!polynomial.BernsteinRowsPositive(loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, q)) {
						return nil, false, nil
					}
					if band == 0 {
						proof.Crossings[side][0] = loftface.IsolateRoot(q[0])
					}
					proof.Crossings[side][band+1] = loftface.IsolateRoot(q[2])
				} else if !polynomial.BernsteinRowsPositive(q) {
					return nil, false, nil
				}
			}
		}
		toeRange := loftface.CapXRange(trim.Maps[0], spans[0], proof.Crossings[side][0])
		heelRange := loftface.CapXRange(trim.Maps[8], spans[0], proof.Crossings[side][8])
		if toeRange.Hi.Cmp(toeX) >= 0 || heelRange.Lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
		for end, targetX := range []*big.Rat{toeX, heelX} {
			junction, ok := spiralsector.IsolateJunction(ctx, trim.Maps, spans[0], m, b,
				proof.Crossings[side], targetX)
			if !ok {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				return nil, false, nil
			}
			proof.Junctions[side][end] = junction
		}
	}
	// The embedded source solid's boundary meets each tool/root circle only
	// at these two flank junctions. All other contour groups stay on one side.
	// The whole original cap 0 is below toeX. The whole cap-8 plane either
	// misses its heel tool/root circle (the strict branch), or, for a
	// cone-tangent end with a matching record, touches that circle at one
	// point with the circle on the source side of the plane
	// (spiralsector.TangentCapTouchesHeelCircle). Either way neither end-cap
	// interior can create an extra contact, even though cap 8 spans the
	// heelX coordinate.
	// Q_s and J make both junctions transverse. Circle membership therefore
	// changes at each junction. The long connecting arc crosses Y<0, where
	// the entire source is absent, so the short arc is inside the source.
	// The ordered exact Cut and Intersect define the operative tool-cone cap
	// as this source solid's trace on that cone. The retained cap group IDs
	// identify held facets only; they do not establish the true trace.
	seam, ok, err := spiralsector.CertifySpiralHeelRootSeam(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	proof.HeelSeam = seam
	return proof, true, nil
}

type spiralStarGroups struct {
	blankRoot, blankToe, blankHeel int
	toothToe, toothHeel            int
	toothRootToe, toothRootHeel    int
}

func spiralTrimFaceGroup(tooth *Body, sourceGroup int) int {
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok {
		return -1
	}
	return facetedtopology.TrimGroupFace(p.Src, p.FaceOf, len(tooth.Faces()), sourceGroup)
}

func spiralJoinGroups(blank, tooth *Body, proof *spiralFirstJoinSource) (spiralStarGroups, bool) {
	if blank == nil || tooth == nil || proof == nil || proof.Blank == nil || proof.Tooth == nil {
		return spiralStarGroups{}, false
	}
	offset := len(blank.Faces())
	toeBand := proof.Junctions[0][0].Band
	heelBand := proof.Junctions[0][1].Band
	if toeBand < 0 || heelBand < 0 ||
		proof.Junctions[1][0].Band != toeBand || proof.Junctions[1][1].Band != heelBand {
		return spiralStarGroups{}, false
	}
	toeCap := spiralTrimFaceGroup(tooth, proof.Tooth.ToeCapGroup)
	heelCap := spiralTrimFaceGroup(tooth, proof.Tooth.HeelCapGroup)
	toeRootRef, toeRefOK := spiralOriginalSideRef(proof.Tooth, toeBand, 4)
	heelRootRef, heelRefOK := spiralOriginalSideRef(proof.Tooth, heelBand, 4)
	if !toeRefOK || !heelRefOK || toeRootRef == heelRootRef {
		return spiralStarGroups{}, false
	}
	toeRootFace := spiralUniqueFaceWithOrigin(tooth, toeRootRef)
	heelRootFace := spiralUniqueFaceWithOrigin(tooth, heelRootRef)
	if toeRootFace == nil || heelRootFace == nil || toeRootFace == heelRootFace {
		return spiralStarGroups{}, false
	}
	toothFaces := tooth.Faces()
	toeRoot, heelRoot := -1, -1
	for i, face := range toothFaces {
		if face == toeRootFace {
			toeRoot = i
		}
		if face == heelRootFace {
			heelRoot = i
		}
	}
	if toeCap < 0 || heelCap < 0 || toeRoot < 0 || heelRoot < 0 {
		return spiralStarGroups{}, false
	}
	g := spiralStarGroups{
		blankRoot:     spiralBlankFaceGroup(blank, proof.Blank, 3),
		blankToe:      spiralBlankFaceGroup(blank, proof.Blank, 4),
		blankHeel:     spiralBlankFaceGroup(blank, proof.Blank, 2),
		toothToe:      offset + toeCap,
		toothHeel:     offset + heelCap,
		toothRootToe:  offset + toeRoot,
		toothRootHeel: offset + heelRoot,
	}
	for _, value := range [...]int{g.blankRoot, g.blankToe, g.blankHeel,
		g.toothToe, g.toothHeel, g.toothRootToe, g.toothRootHeel} {
		if value < 0 || value >= offset+len(tooth.Faces()) {
			return spiralStarGroups{}, false
		}
	}
	// These seven provenance groups must remain distinct. A role merger would
	// make the local cap/root proof unable to assign a held patch to a source.
	seen := map[int]bool{}
	for _, value := range [...]int{g.blankRoot, g.blankToe, g.blankHeel,
		g.toothToe, g.toothHeel, g.toothRootToe, g.toothRootHeel} {
		if seen[value] {
			return spiralStarGroups{}, false
		}
		seen[value] = true
	}
	return g, true
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
	map[[2]*topo.Face]meshbool.NearPairRole, bool,
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
	out := make(map[[2]*topo.Face]meshbool.NearPairRole, 14)
	add := func(a, b *Face, kind meshbool.NearPairRole) bool {
		if a == nil || b == nil || a.n.Body != bodyNode(blank) || b.n.Body != bodyNode(tooth) ||
			kind == meshbool.NearPairUnknown {
			return false
		}
		pair := [2]*topo.Face{&a.n, &b.n}
		if out[pair] != meshbool.NearPairUnknown {
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
				spiralUniqueFaceWithOrigin(tooth, origin), meshbool.NearPairCrossing) {
				return nil, false
			}
		}
	}
	for _, side := range [...]int{3, 4, 5} {
		origin, found := spiralOriginalSideRef(proof.Tooth, 1, side)
		if !found || !add(toe, spiralUniqueFaceWithOrigin(tooth, origin), meshbool.NearPairCrossing) {
			return nil, false
		}
	}
	for _, pair := range [...][2]*Face{{heel, heelTool}, {toe, toeTool},
		{toe, heelTool}, {root, toeTool}, {root, heelTool}} {
		if !add(pair[0], pair[1], meshbool.NearPairSeparated) {
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
	map[[2]*topo.Face]meshbool.NearPairRole, [2][4]spiralsector.ShortCapFamily, bool, error,
) {
	var empty [2][4]spiralsector.ShortCapFamily
	if ctx == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Blank == nil || proof.Tooth == nil || proof.HeelSeam == nil ||
		!spiralsector.NearTrimSides(proof) || !spiralsector.TrimSourceCurrent(proof.Tooth) {
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
