package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
)

// spiralHeldShortCapProof compares every complete short cap/flank or
// cap/connector switch with its authenticated source crossing. Global held
// group ids come from the original two operands' face order, not a gallery
// layout. Exact source-cell and held-edge boxes cover both curves.
func spiralHeldShortCapProof(ctx context.Context, held *meshbool.StitchedMesh,
	blank, tooth *Body, proof *spiralFirstJoinSource,
	short [2][4]spiralsector.ShortCapFamily) (float64, bool, error) {
	if ctx == nil || held == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Tooth == nil {
		return 0, false, nil
	}
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok {
		return 0, false, nil
	}
	roles := [...]int{0, 2, 3, 5}
	maxDistance := 0.0
	// Every switch below reads the same unchanged held mesh, so its edge
	// table is built once, at the first switch, and reused within this call.
	var edges map[spiralsector.HeldEdge]spiralsector.HeldEdgeOwners
	for end := range 2 {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		blankCap := groups.blankToe
		if end == 1 {
			blankCap = groups.blankHeel
		}
		for index, role := range roles {
			family := short[end][index]
			if family.Group != role || len(family.Pieces) != 1 ||
				family.Band < 0 || family.Band >= len(proof.Tooth.Maps)-1 ||
				family.Band != proof.Junctions[0][end].Band ||
				family.Band != proof.Junctions[1][end].Band {
				return 0, false, nil
			}
			origin, found := spiralOriginalSideRef(proof.Tooth, family.Band, role)
			if !found {
				return 0, false, nil
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			if face == nil {
				return 0, false, nil
			}
			toothFace := -1
			for i, live := range tooth.Faces() {
				if live == face {
					toothFace = i
				}
			}
			if toothFace < 0 {
				return 0, false, nil
			}
			if edges == nil {
				built, ok, err := spiralsector.HeldEdges(ctx, held)
				if err != nil || !ok {
					return 0, false, err
				}
				edges = built
			}
			chain, ok, err := spiralsector.HeldSwitchFrom(ctx, held, edges,
				blankCap, len(blank.Faces())+toothFace, false)
			if err != nil || !ok {
				return 0, false, err
			}
			ordered := make([]r3.Vec, len(chain.Vertices))
			for i, vertex := range chain.Vertices {
				ordered[i] = held.Verts[vertex]
			}
			distance, accepted, err := spiralsector.ShortHeldBound(ctx,
				proof, family, ordered, chain.MinBeta)
			if err != nil || !accepted {
				return distance, false, err
			}
			maxDistance = max(maxDistance, distance)
		}
	}
	return maxDistance, true, nil
}
