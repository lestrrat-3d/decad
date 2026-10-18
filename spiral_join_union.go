package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
)

// spiralFirstUnionAdmission is scoped to one exact blank and its first
// unplaced cone-trimmed curved tooth. The proxy mesh is temporary: public
// operands and their retained exact source records do not change.
type spiralFirstUnionAdmission struct {
	source    *spiralFirstJoinSource
	short     [2][4]spiralsector.ShortCapFamily
	roles     map[[2]*Face]spiralNearPairKind
	allowed   map[[2]int]bool
	rootBoxes *[2]*spiralsector.RootFlankSourceBoxes
}

// spiralPrepareFirstUnion authenticates source geometry before modifying a
// temporary held operand. Any failed source gate leaves generic Union in
// charge; cancellation and evaluator errors still propagate.
func spiralPrepareFirstUnion(ctx context.Context, op meshbool.OperationKind,
	blank, tooth *Body, held *Mesh, sourceIDs []int) (*Mesh, *spiralFirstUnionAdmission, error) {
	if ctx == nil || blank == nil || tooth == nil || held == nil ||
		op != meshbool.OpUnion {
		return held, nil, nil
	}
	if _, ok := exactSpiralBlank(blank); !ok {
		return held, nil, nil
	}
	payload, ok := tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim == nil {
		return held, nil, nil
	}
	source, ok, err := certifySpiralFirstJoin(ctx, blank, tooth)
	if err != nil || !ok {
		return held, nil, err
	}
	roles, short, ok, err := certifySpiralNearFaceRoles(ctx, blank, tooth, source)
	if err != nil || !ok {
		return held, nil, err
	}
	proxy, ok, err := spiralOffsetToothMesh(ctx, blank, tooth, held, sourceIDs, source)
	if err != nil || !ok {
		return held, nil, err
	}
	return proxy, &spiralFirstUnionAdmission{source: source, short: short,
		roles: roles}, nil
}

// spiralFirstUnionPairs lists the only source-face switches supported by the
// curved first-join certificate. Full FeatureRef matches bind the source
// flanks to live faces; source group numbers alone are insufficient after the
// two cone trims.
func spiralFirstUnionPairs(blank, tooth *Body, admission *spiralFirstUnionAdmission,
	faceID map[*Face]int) (map[[2]int]bool, bool) {
	if blank == nil || tooth == nil || admission == nil || admission.source == nil {
		return nil, false
	}
	proof := admission.source
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok {
		return nil, false
	}
	allowed := map[[2]int]bool{
		{groups.blankToe, groups.toothRootToe}:   true,
		{groups.blankHeel, groups.toothRootHeel}: true,
	}
	for pair, kind := range admission.roles {
		if kind != spiralNearPairCrossing {
			continue
		}
		a, aOK := faceID[pair[0]]
		b, bOK := faceID[pair[1]]
		if !aOK || !bOK {
			return nil, false
		}
		allowed[[2]int{a, b}] = true
	}
	for end := range 2 {
		blankCap := groups.blankToe
		if end == 1 {
			blankCap = groups.blankHeel
		}
		for index, segment := range [...]int{0, 2, 3, 5} {
			family := admission.short[end][index]
			if family.Group != segment || family.Band != proof.Junctions[0][end].Band ||
				family.Band != proof.Junctions[1][end].Band {
				return nil, false
			}
			origin, found := spiralOriginalSideRef(proof.Tooth, family.Band, segment)
			if !found {
				return nil, false
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			group, present := faceID[face]
			if face == nil || !present {
				return nil, false
			}
			allowed[[2]int{blankCap, group}] = true
		}
	}
	for band := 1; band < len(proof.Tooth.Maps)-1; band++ {
		for _, segment := range [...]int{0, 2} {
			origin, found := spiralOriginalSideRef(proof.Tooth, band, segment)
			if !found {
				return nil, false
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			group, present := faceID[face]
			if face == nil || !present {
				return nil, false
			}
			allowed[[2]int{groups.blankRoot, group}] = true
		}
	}
	return allowed, len(allowed) == 24
}

// spiralValidateFirstUnion runs after the ordinary stitched-solid audit. The
// source-pair gate permits only named near pairs, and this gate checks every
// resulting switched curve against its complete authenticated source seam.
// The unchanged mesh-Boolean and stitch volume proofs still apply.
func spiralValidateFirstUnion(ctx context.Context, blank, tooth *Body,
	blankHeld *meshbool.BoolMesh, result *meshbool.StitchedMesh,
	admission *spiralFirstUnionAdmission, radialExtra ...float64) (bool, error) {
	if ctx == nil || blank == nil || tooth == nil || blankHeld == nil ||
		result == nil || admission == nil || admission.source == nil {
		return false, nil
	}
	proof := admission.source
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok || proof.HeelSeam == nil {
		return false, nil
	}
	// The complete original Revolve cap faces are radial graphs. A bound on
	// their actual held facets then applies to every stitched subedge, with
	// the stitch displacement charged by the long-seam helpers.
	var radial [2]float64
	for end, edge := range [...]int{4, 2} {
		graph, err := spiralHeldBlankCellGraph(ctx, blank, blankHeld, proof.Blank, edge)
		if err != nil || !graph {
			return false, err
		}
		bound, accepted, err := spiralHeldBlankRadialAllowance(ctx,
			blank, blankHeld, proof.Blank, edge)
		if err != nil || !accepted {
			return false, err
		}
		radial[end] = bound
		if len(radialExtra) == 1 {
			radial[end] = proofbound.AbsSumUpper(radial[end], radialExtra[0])
		}
	}
	if _, ok, err := spiralHeldShortCapProof(ctx, result, blank, tooth,
		proof, admission.short); err != nil || !ok {
		return false, err
	}
	if _, ok, err := spiralsector.HeldHeelSeamProof(ctx, result,
		groups.blankHeel, groups.toothRootHeel, proof.HeelSeam, radial[1]); err != nil || !ok {
		return false, err
	}
	toeSeam, ok, err := spiralsector.CertifySpiralToeRootSeam(ctx, proof)
	if err != nil || !ok {
		return false, err
	}
	if _, ok, err := spiralsector.HeldToeSeamProof(ctx, result,
		groups.blankToe, groups.toothRootToe, toeSeam, radial[0]); err != nil || !ok {
		return false, err
	}
	for sourceSide, segment := range [...]int{0, 2} {
		origin, found := spiralOriginalSideRef(proof.Tooth,
			proof.Junctions[sourceSide][0].Band, segment)
		if !found {
			return false, nil
		}
		face := spiralUniqueFaceWithOrigin(tooth, origin)
		if face == nil {
			return false, nil
		}
		faceGroup := -1
		for i, live := range tooth.Faces() {
			if live == face {
				faceGroup = len(blank.Faces()) + i
			}
		}
		if faceGroup < 0 {
			return false, nil
		}
		chain, accepted, err := spiralsector.HeldHeelSwitch(ctx, result,
			groups.blankRoot, faceGroup)
		if err != nil || !accepted {
			return false, err
		}
		ordered := make([]r3.Vec, len(chain.Vertices))
		for i, vertex := range chain.Vertices {
			ordered[i] = result.Verts[vertex]
		}
		var prepared []*spiralsector.RootFlankSourceBoxes
		if admission.rootBoxes != nil {
			prepared = append(prepared, admission.rootBoxes[sourceSide])
		}
		if _, accepted, err := spiralsector.RootFlankWholeBound(ctx, proof,
			sourceSide, ordered, chain.MinBeta, prepared...); err != nil || !accepted {
			return false, err
		}
	}
	return ctx.Err() == nil, ctx.Err()
}
