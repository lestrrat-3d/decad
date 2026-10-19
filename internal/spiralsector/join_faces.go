package spiralsector

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// StarGroups holds the seven held source groups of the first curved-tooth
// join. Blank groups are blank face indexes; tooth groups are offset by the
// blank's face count, as in the joint face order of a held Boolean.
type StarGroups struct {
	BlankRoot, BlankToe, BlankHeel int
	ToothToe, ToothHeel            int
	ToothRootToe, ToothRootHeel    int
}

// TrimFaceGroup returns the tooth face index that holds the facets of the
// trim source group, or -1 when the tooth carries no faceted payload or the
// group does not map to exactly one face.
func TrimFaceGroup(tooth *topo.Body, sourceGroup int) int {
	p, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok {
		return -1
	}
	return facetedtopology.TrimGroupFace(p.Src, p.FaceOf, len(topobuild.Faces(tooth)), sourceGroup)
}

// JoinGroups binds the certified first join's blank and tooth source roles
// to held group numbers. It refuses when any role is missing or two roles
// share a group.
func JoinGroups(blank, tooth *topo.Body, proof *spiralFirstJoinSource) (StarGroups, bool) {
	if blank == nil || tooth == nil || proof == nil || proof.Blank == nil || proof.Tooth == nil {
		return StarGroups{}, false
	}
	offset := len(topobuild.Faces(blank))
	toeBand := proof.Junctions[0][0].Band
	heelBand := proof.Junctions[0][1].Band
	if toeBand < 0 || heelBand < 0 ||
		proof.Junctions[1][0].Band != toeBand || proof.Junctions[1][1].Band != heelBand {
		return StarGroups{}, false
	}
	toeCap := TrimFaceGroup(tooth, proof.Tooth.ToeCapGroup)
	heelCap := TrimFaceGroup(tooth, proof.Tooth.HeelCapGroup)
	toeRootRef, toeRefOK := OriginalSideRef(proof.Tooth, toeBand, 4)
	heelRootRef, heelRefOK := OriginalSideRef(proof.Tooth, heelBand, 4)
	if !toeRefOK || !heelRefOK || toeRootRef == heelRootRef {
		return StarGroups{}, false
	}
	toeRootFace := UniqueFaceWithOrigin(tooth, toeRootRef)
	heelRootFace := UniqueFaceWithOrigin(tooth, heelRootRef)
	if toeRootFace == nil || heelRootFace == nil || toeRootFace == heelRootFace {
		return StarGroups{}, false
	}
	toothFaces := topobuild.Faces(tooth)
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
		return StarGroups{}, false
	}
	g := StarGroups{
		BlankRoot:     blankFaceGroup(blank, proof.Blank, 3),
		BlankToe:      blankFaceGroup(blank, proof.Blank, 4),
		BlankHeel:     blankFaceGroup(blank, proof.Blank, 2),
		ToothToe:      offset + toeCap,
		ToothHeel:     offset + heelCap,
		ToothRootToe:  offset + toeRoot,
		ToothRootHeel: offset + heelRoot,
	}
	for _, value := range [...]int{g.BlankRoot, g.BlankToe, g.BlankHeel,
		g.ToothToe, g.ToothHeel, g.ToothRootToe, g.ToothRootHeel} {
		if value < 0 || value >= offset+len(toothFaces) {
			return StarGroups{}, false
		}
	}
	// These seven provenance groups must remain distinct. A role merger would
	// make the local cap/root proof unable to assign a held patch to a source.
	seen := map[int]bool{}
	for _, value := range [...]int{g.BlankRoot, g.BlankToe, g.BlankHeel,
		g.ToothToe, g.ToothHeel, g.ToothRootToe, g.ToothRootHeel} {
		if seen[value] {
			return StarGroups{}, false
		}
		seen[value] = true
	}
	return g, true
}

// UniqueFaceWithOrigin returns the one face of body whose origins carry want,
// or nil when no face or more than one face carries it.
func UniqueFaceWithOrigin(body *topo.Body, want provenance.Ref) *topo.Face {
	if body == nil {
		return nil
	}
	var result *topo.Face
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
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

// OriginalSideRef returns the producer identity of the original LoftFaces
// side group for band and segment. The original LoftFaces groups carry the
// producer identity that the two cone Booleans preserve on a surviving side.
// Do not infer it from a nearby polygon or from the result face's position in
// its face list.
func OriginalSideRef(trim *loftFacesTrimRecord, band, segment int) (provenance.Ref, bool) {
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

func capGroupRef(groups []facetedrecord.Group, group int) (provenance.Ref, bool) {
	if group < 0 || group >= len(groups) || len(groups[group].Origins) != 1 {
		return provenance.Ref{}, false
	}
	return groups[group].Origins[0], true
}

// nearFaceTable binds the fixed set of source roles for the first
// curved-tooth join to the live input faces. Geometry certificates for the
// nine crossings are checked by the caller before this table is used.
func nearFaceTable(blank, tooth *topo.Body, proof *spiralFirstJoinSource) (
	map[[2]*topo.Face]meshbool.NearPairRole, bool,
) {
	if blank == nil || tooth == nil || proof == nil || proof.Blank == nil ||
		proof.Tooth == nil || proof.Blank.RootFace == nil ||
		proof.Blank.RootFace.Body != blank {
		return nil, false
	}
	p, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok || p.LoftFacesTrim != proof.Tooth {
		return nil, false
	}
	blankFace := func(edge int) *topo.Face {
		return UniqueFaceWithOrigin(blank, provenance.Ref{Producer: blank.Origin.Producer,
			Role: fmt.Sprintf("side(0,%d)", proof.Blank.EdgeIndex[edge])})
	}
	heel, root, toe := blankFace(2), blankFace(3), blankFace(4)
	if heel == nil || root == nil || toe == nil || root != proof.Blank.RootFace ||
		heel == root || heel == toe || root == toe {
		return nil, false
	}
	toeRef, okToe := capGroupRef(p.Groups, proof.Tooth.ToeCapGroup)
	heelRef, okHeel := capGroupRef(p.Groups, proof.Tooth.HeelCapGroup)
	if !okToe || !okHeel || toeRef == heelRef {
		return nil, false
	}
	toeTool, heelTool := UniqueFaceWithOrigin(tooth, toeRef), UniqueFaceWithOrigin(tooth, heelRef)
	if toeTool == nil || heelTool == nil || toeTool == heelTool {
		return nil, false
	}
	out := make(map[[2]*topo.Face]meshbool.NearPairRole, 14)
	add := func(a, b *topo.Face, kind meshbool.NearPairRole) bool {
		if a == nil || b == nil || a.Body != blank || b.Body != tooth ||
			kind == meshbool.NearPairUnknown {
			return false
		}
		pair := [2]*topo.Face{a, b}
		if out[pair] != meshbool.NearPairUnknown {
			return false
		}
		out[pair] = kind
		return true
	}
	for _, side := range []int{0, 2} {
		for _, placement := range [...]struct {
			blank *topo.Face
			band  int
		}{{heel, 7}, {toe, 1}, {root, 1}} {
			origin, found := OriginalSideRef(proof.Tooth, placement.band, side)
			if !found || !add(placement.blank,
				UniqueFaceWithOrigin(tooth, origin), meshbool.NearPairCrossing) {
				return nil, false
			}
		}
	}
	for _, side := range [...]int{3, 4, 5} {
		origin, found := OriginalSideRef(proof.Tooth, 1, side)
		if !found || !add(toe, UniqueFaceWithOrigin(tooth, origin), meshbool.NearPairCrossing) {
			return nil, false
		}
	}
	for _, pair := range [...][2]*topo.Face{{heel, heelTool}, {toe, toeTool},
		{toe, heelTool}, {root, toeTool}, {root, heelTool}} {
		if !add(pair[0], pair[1], meshbool.NearPairSeparated) {
			return nil, false
		}
	}
	return out, len(out) == 14
}

// CertifyNearFaceRoles is a reject-only source classifier for the
// first unplaced curved-tooth Union. It never changes the generic proximity
// gate. The caller may use the returned roles only while the corresponding
// source-certified held output proves every crossing chain and every
// separated held face pair has no facet contact.
func CertifyNearFaceRoles(ctx context.Context, blank, tooth *topo.Body,
	proof *spiralFirstJoinSource) (
	map[[2]*topo.Face]meshbool.NearPairRole, [2][4]ShortCapFamily, bool, error,
) {
	var empty [2][4]ShortCapFamily
	if ctx == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Blank == nil || proof.Tooth == nil || proof.HeelSeam == nil ||
		!NearTrimSides(proof) || !TrimSourceCurrent(proof.Tooth) {
		return nil, empty, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, empty, false, err
	}
	// The existing first-join proof covers two transverse flank/root
	// crossings over all bands. The short-cap certificate covers both
	// finite outer caps against flanks and source connectors. The toe seam
	// covers the one remaining cap/root-arc contact.
	short, ok, err := ShortCapFamilies(ctx, proof)
	if err != nil || !ok {
		return nil, empty, false, err
	}
	toeSeam, ok, err := CertifySpiralToeRootSeam(ctx, proof)
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
	roles, ok := nearFaceTable(blank, tooth, proof)
	if !ok {
		return nil, empty, false, nil
	}
	return roles, short, true, nil
}
