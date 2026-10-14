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
	return facetedtopology.TrimGroupFace(p.src, p.faceOf, len(tooth.Faces()), sourceGroup)
}

func spiralJoinGroups(blank, tooth *Body, proof *spiralFirstJoinSource) (spiralStarGroups, bool) {
	if blank == nil || tooth == nil || proof == nil || proof.blank == nil || proof.tooth == nil {
		return spiralStarGroups{}, false
	}
	offset := len(blank.Faces())
	toeBand := proof.junctions[0][0].band
	heelBand := proof.junctions[0][1].band
	if toeBand < 0 || heelBand < 0 ||
		proof.junctions[1][0].band != toeBand || proof.junctions[1][1].band != heelBand {
		return spiralStarGroups{}, false
	}
	toeCap := spiralTrimFaceGroup(tooth, proof.tooth.toeCapGroup)
	heelCap := spiralTrimFaceGroup(tooth, proof.tooth.heelCapGroup)
	toeRootRef, toeRefOK := spiralOriginalSideRef(proof.tooth, toeBand, 4)
	heelRootRef, heelRefOK := spiralOriginalSideRef(proof.tooth, heelBand, 4)
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
		blankRoot:     spiralBlankFaceGroup(blank, proof.blank, 3),
		blankToe:      spiralBlankFaceGroup(blank, proof.blank, 4),
		blankHeel:     spiralBlankFaceGroup(blank, proof.blank, 2),
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
