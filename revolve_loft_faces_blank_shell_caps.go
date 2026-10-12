package decad

import "context"

// The fresh source census supplies every occupied heel circle. Each circle
// splits its authenticated Revolve meridian before any retained-face cut is
// attempted. This is still a complete blank shell, not a Union result.
func revolveLoftFacesBlankShellWithCapRings(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, sectionIndex, cellIndex int,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	census, ok, err := revolveLoftFacesCensus(ctx, proof)
	if err != nil || !ok || census == nil || census.manifest == nil {
		return nil, false, err
	}
	var cuts []revolveLoftFacesShellCut
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		arc := pair.toolArc
		if arc == nil || arc.circle.tool != 1 || arc.circle.sourceMiss ||
			arc.circle.blankFace != pair.blankFace ||
			arc.circle.capFace != pair.trimmedFace ||
			arc.circle.span != pair.sectionIndex ||
			arc.circle.x == nil || arc.circle.radius == nil ||
			arc.circle.radius.Sign() <= 0 {
			return nil, false, nil
		}
		cuts = append(cuts, revolveLoftFacesShellCut{
			sectionIndex: pair.sectionIndex, x: arc.circle.x,
			radius: arc.circle.radius, arc: arc})
	}
	if len(cuts) == 0 {
		return nil, false, nil
	}
	return revolveLoftFacesBlankShellAtArcWithCuts(ctx, proof, census.manifest,
		sectionIndex, cellIndex, cuts)
}
