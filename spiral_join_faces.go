package decad

import "github.com/lestrrat-3d/decad/internal/facetedtopology"

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
