package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// spiralFirstUnionAdmission is scoped to one exact blank and its first
// unplaced cone-trimmed curved tooth. The proxy mesh is temporary: public
// operands and their retained exact source records do not change.
type spiralFirstUnionAdmission struct {
	source  *spiralFirstJoinSource
	short   [2][4]spiralShortCapFamily
	roles   map[[2]*Face]spiralNearPairKind
	allowed map[[2]int]bool
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
	payload, ok := tooth.payload.(facetedPayload)
	if !ok || payload.loftFacesTrim == nil {
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
			if family.group != segment || family.band != proof.junctions[0][end].band ||
				family.band != proof.junctions[1][end].band {
				return nil, false
			}
			origin, found := spiralOriginalSideRef(proof.tooth, family.band, segment)
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
	for band := 1; band < len(proof.tooth.maps)-1; band++ {
		for _, segment := range [...]int{0, 2} {
			origin, found := spiralOriginalSideRef(proof.tooth, band, segment)
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

// spiralScopeWidenedContacts checks every exact held contact whose tooth
// facet uses the source-specific Gate allowance. A near-face role alone
// cannot cover a deeper contact hidden inside the Union result.
func spiralScopeWidenedContacts(ctx context.Context, blankHeld, toothHeld *meshbool.BoolMesh,
	memo *meshbool.ContactMemo, allowed map[[2]int]bool, ordinaryGate float64) (bool, error) {
	if ctx == nil || blankHeld == nil || toothHeld == nil || memo == nil ||
		len(allowed) != 24 || ordinaryGate <= 0 ||
		len(toothHeld.FacetBound) != len(toothHeld.Tris) ||
		len(blankHeld.Src) != len(blankHeld.Tris) ||
		len(toothHeld.Src) != len(toothHeld.Tris) ||
		len(blankHeld.Tris) > 1<<15 || len(toothHeld.Tris) > 1<<17 {
		return false, nil
	}
	high := 0
	for _, bound := range toothHeld.FacetBound {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return false, nil
		}
		if bound > ordinaryGate {
			high++
		}
	}
	// The exact contact classifier runs only after this bounded broad-phase
	// scan. A larger mesh takes the ordinary generic refusal path.
	if uint64(high)*uint64(len(blankHeld.Tris)) > 1<<24 {
		return false, nil
	}
	work := 0
	for j := range toothHeld.Tris {
		if toothHeld.FacetBound[j] <= ordinaryGate {
			continue
		}
		for i := range blankHeld.Tris {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			if !meshbool.BoxesWithin(blankHeld.Boxes[i], toothHeld.Boxes[j], 0) {
				continue
			}
			contact, err := memo.Classify(i, j)
			if err != nil {
				return false, err
			}
			if contact.Kind != meshbool.ContactNone &&
				!allowed[[2]int{blankHeld.Src[i], toothHeld.Src[j]}] {
				return false, nil
			}
		}
	}
	return ctx.Err() == nil, ctx.Err()
}

// spiralScopeOutputSwitches checks every stitched cross-operand edge, even
// when its input contact had a facet bound below the ordinary pair gate.
func spiralScopeOutputSwitches(ctx context.Context, held *meshbool.StitchedMesh,
	blankFaces int, allowed map[[2]int]bool) (bool, error) {
	if ctx == nil || held == nil || blankFaces <= 0 || len(allowed) != 24 ||
		len(held.Tris) > 1<<17 || len(held.Src) != len(held.Tris) {
		return false, nil
	}
	type edge struct{ a, b int }
	owners := make(map[edge][2]int, len(held.Tris)*3/2)
	counts := make(map[edge]uint8, len(held.Tris)*3/2)
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			if a < 0 || b < 0 || a >= len(held.Verts) || b >= len(held.Verts) || a == b {
				return false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := edge{a, b}
			if counts[key] >= 2 {
				return false, nil
			}
			pair := owners[key]
			pair[counts[key]] = held.Src[i]
			owners[key] = pair
			counts[key]++
		}
	}
	seen := 0
	for edge, pair := range owners {
		_ = edge
		if seen%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		seen++
		if counts[edge] != 2 {
			return false, nil
		}
		a, b := pair[0], pair[1]
		if a >= blankFaces && b < blankFaces {
			a, b = b, a
		}
		if a < blankFaces && b >= blankFaces && !allowed[[2]int{a, b}] {
			return false, nil
		}
	}
	return ctx.Err() == nil, ctx.Err()
}

// spiralValidateFirstUnion runs after the ordinary stitched-solid audit. The
// source-pair gate permits only named near pairs, and this gate checks every
// resulting switched curve against its complete authenticated source seam.
// The unchanged mesh-Boolean and stitch volume proofs still apply.
func spiralValidateFirstUnion(ctx context.Context, blank, tooth *Body,
	blankHeld *meshbool.BoolMesh, result *meshbool.StitchedMesh,
	admission *spiralFirstUnionAdmission) (bool, error) {
	if ctx == nil || blank == nil || tooth == nil || blankHeld == nil ||
		result == nil || admission == nil || admission.source == nil {
		return false, nil
	}
	proof := admission.source
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok || proof.heelSeam == nil {
		return false, nil
	}
	// The complete original Revolve cap faces are radial graphs. A bound on
	// their actual held facets then applies to every stitched subedge, with
	// the stitch displacement charged by the long-seam helpers.
	var radial [2]float64
	for end, edge := range [...]int{4, 2} {
		graph, err := spiralHeldBlankCellGraph(ctx, blank, blankHeld, proof.blank, edge)
		if err != nil || !graph {
			return false, err
		}
		bound, accepted, err := spiralHeldBlankRadialAllowance(ctx,
			blank, blankHeld, proof.blank, edge)
		if err != nil || !accepted {
			return false, err
		}
		radial[end] = bound
	}
	if _, ok, err := spiralHeldShortCapProof(ctx, result, blank, tooth,
		proof, admission.short); err != nil || !ok {
		return false, err
	}
	if _, ok, err := spiralHeldHeelSeamProof(ctx, result,
		groups.blankHeel, groups.toothRootHeel, proof.heelSeam, radial[1]); err != nil || !ok {
		return false, err
	}
	toeSeam, ok, err := certifySpiralToeRootSeam(ctx, proof)
	if err != nil || !ok {
		return false, err
	}
	if _, ok, err := spiralHeldToeSeamProof(ctx, result,
		groups.blankToe, groups.toothRootToe, toeSeam, radial[0]); err != nil || !ok {
		return false, err
	}
	for sourceSide, segment := range [...]int{0, 2} {
		origin, found := spiralOriginalSideRef(proof.tooth,
			proof.junctions[sourceSide][0].band, segment)
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
		chain, accepted, err := spiralHeldHeelSwitch(ctx, result,
			groups.blankRoot, faceGroup)
		if err != nil || !accepted {
			return false, err
		}
		ordered := make([]r3.Vec, len(chain.vertices))
		for i, vertex := range chain.vertices {
			ordered[i] = result.Verts[vertex]
		}
		if _, accepted, err := spiralRootFlankWholeBound(ctx, proof,
			sourceSide, ordered, chain.minBeta); err != nil || !accepted {
			return false, err
		}
	}
	return ctx.Err() == nil, ctx.Err()
}
