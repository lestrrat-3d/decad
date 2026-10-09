package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is docs/draft-design.md Table DD row DD1: a draft body's
// tessellation and its occupied-volume proof. A draft body is built as the
// cap-loop band of docs/modify-reach-design.md §8.3 with no straight slab
// (draftPayload.band), so it is meshed by the cap-loop chamfer tessellator
// over that same view (tessellate_capblend.go, docs/tessellation-reach-design.md
// §7) with three differences, each stated where it applies:
//
//   - there is no trimmed side wall, and the band's side ring is the near
//     cap's rim, written once (draftNearRing) and read by both the near cap
//     and the wall band;
//   - a wall patch carries the prism's side(i, j) role, and the band has no
//     apex patch, because every reflex line-line corner is a miter
//     (capBlendPayload.offsetJoins);
//   - the slice-wise chord term integrates over the whole sweep height with
//     both ends' displacements (draftBandHeightUpper).
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
	if len(dp.patches) == 0 || len(dp.bandDelta) == 0 {
		return nil, fmt.Errorf(`%w: the tapered extrude's payload states no wall band geometry`, ErrDegenerate)
	}
	return tessellateCapBlend(ctx, b, dp.band(), chord, verify)
}

// draftOccupiedVolumeAdmission is requireVolumeProvingPayload's question for a
// draft body: docs/tessellation-reach-design.md §7's admission over its band
// view, the same predicate tessellateCapBlend reads before it publishes.
func draftOccupiedVolumeAdmission(budget *proofbound.WorkBudget, dp draftPayload) (error, error) {
	return capBlendOccupiedVolumeAdmission(budget, dp.band())
}

// draftNearLevel is the near cap's level in a draft band view beside its own
// axial displacement: the cap no loop is chamfered on.
func draftNearLevel(cbp capBlendPayload) proofbound.BoundedScalar {
	if len(cbp.endLoops) > 0 {
		return proofbound.MeasuredScalar(cbp.z0, cbp.z0Delta)
	}
	return proofbound.MeasuredScalar(cbp.z1, cbp.z1Delta)
}

// draftBandHeightUpper is a proven upper bound on the axial extent of a draft
// body: the held sweep height with both ends' displacements and the
// subtraction's rounding. The band runs the whole of it, so it is the height
// the slice-wise chord term integrates over (tessellation.CapBlendChordVolume's
// band term), in place of the side setback, which carries neither end's
// displacement.
func draftBandHeightUpper(cbp capBlendPayload) float64 {
	h := proofbound.BoundedSub(proofbound.MeasuredScalar(cbp.z1, cbp.z1Delta), proofbound.MeasuredScalar(cbp.z0, cbp.z0Delta))
	return proofbound.AbsSumUpper(math.Abs(h.Value), h.Bound)
}

// draftNearRing writes one loop's side ring once, at the near level, as the
// near cap's rim. The near cap and the wall band read the same vertices, so
// sideLo and sideHi are one slice. Each vertex's motion is its station bound,
// its lift rounding and the near level's own displacement, as a side ring's is
// (capBlendSideRing).
func draftNearRing(budget *proofbound.WorkBudget, lm *capBlendLoopMesh, addVertex capBlendVertexWriter, motion *[]float64, proveVolume bool) error {
	lm.sideLo = make([]int, len(lm.sidePts))
	for j, p := range lm.sidePts {
		if err := budget.Step(); err != nil {
			return err
		}
		plane := proofbound.WalkEndBoundAllow(lm.sideBound[j])
		var round float64
		lm.sideLo[j], round = addVertex(p, lm.zLo.Value, plane)
		if proveVolume {
			*motion = append(*motion, proofbound.AbsSumUpper(plane, round, lm.zLo.Bound))
		}
	}
	lm.sideHi = lm.sideLo
	return nil
}
