package decad

import (
	"context"
	"fmt"
)

// This file is docs/draft-design.md Table DD row DD1: a draft body's
// tessellation and its occupied-volume proof. A draft body is built as the
// cap-loop band of docs/modify-reach-design.md §8.3 with no straight slab
// (draftPayload.Band), so it is meshed by the cap-loop chamfer tessellator
// over that same view (tessellate_capblend.go, docs/tessellation-reach-design.md
// §7) with four differences, each stated where it applies:
//
//   - there is no trimmed side wall, and the band's side ring is the near
//     cap's rim, written once (capblend.DraftNearRing) and read by both the near cap
//     and the wall band;
//   - a wall patch carries the prism's side(i, j) role, and the band has no
//     apex patch, because every reflex line-line corner is a miter
//     (capBlendPayload.offsetJoins);
//   - the slice-wise chord term integrates over the whole sweep height with
//     both ends' displacements (capblend.Payload.DraftBandHeightUpper);
//   - a subset draft's kept wall reads a zero setback (walkAmounts,
//     docs/draft-design.md §10.2) in the cap radius, the band's radius
//     rounding and the cap samples' motion, where a chamfer reads its one
//     setback.
//
// The occupied-volume proof is §7's admission and its two legs, unchanged: a
// sharp offset's corner loci are affine in the amount (docs/draft-design.md
// §2), so a band whose every corner is a line-line miter, convex or reflex,
// or an exactly G1 join, and a whole circle, reproduces the exact offset
// family slice by slice. Booleans, export, interference, mass properties and
// Patterned read the mesh this file returns (Table DD rows DD2–DD4, DD10,
// DD11, DD13).

// tessellateDraft meshes a draft body through its band view.
func tessellateDraft(ctx context.Context, b *Body, dp draftPayload, chord float64, verify Verification) (*Mesh, error) {
	if len(dp.Patches) == 0 || len(dp.BandDelta) == 0 {
		return nil, fmt.Errorf(`%w: the tapered extrude's payload states no wall band geometry`, ErrDegenerate)
	}
	return tessellateCapBlend(ctx, b, dp.Band(), chord, verify)
}
