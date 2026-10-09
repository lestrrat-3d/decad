package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the record of a draft body (docs/draft-design.md Table BD row
// BD1) and the offset amount its far section is built at (§8.1). The build
// itself is draft_build.go and the measurements draft_moments.go.

// draftPayload is the evaluator's own record of a tapered extrude: the near
// section P as recorded, the frame it lifts through, the sweep interval with
// each end's own axial displacement, which end P sits at, the taper in
// radians beside its conversion rounding, and the accumulated placement. The
// build fills in the rest from those: the offset amount d with its proven
// span, and the far section Q = P offset sharply by d with the displacement
// its built contour carries (docs/draft-design.md §2, §8.1).
type draftPayload struct {
	profile ProfileRecord
	frame   r3.Frame
	z0, z1  float64
	z0Delta float64
	z1Delta float64
	// nearStart is true when P sits at z0 (an Along extent) and false when it
	// sits at z1 (Against). The far section sits at the other end.
	nearStart bool
	// taper is the signed taper in radians and taperDelta its unit
	// conversion's rounding: the stated angle lies within taperDelta of it.
	taper, taperDelta float64
	xform             r3.Transform

	// d is the held offset amount h·tan α and dDelta the reach of every amount
	// the stated sweep and taper denote from it (draftOffset).
	d, dDelta float64
	// far is the far section Q as built, one loop per loop of profile in the
	// same order, and farDelta the largest contour displacement any of its
	// loops carries (capband.ContourDisplacement).
	far      ProfileRecord
	farDelta float64
	// patches is every wall patch's side(i, j) role beside the plane-local
	// geometry buildCapBand built it from, in loop order then walk order, and
	// bandDelta each loop's own far contour displacement keyed as the band view
	// keys it. Both are filled once by the build and handed to the band view
	// (band), so the tessellator (tessellate_draft.go) charges the same patch
	// geometry and contour displacement every reading of the body charged.
	// Both are plane-local and placement-invariant.
	patches   []capPatch
	bandDelta map[capBandKey]float64
}

// transform is the accumulated rigid placement.
func (dp draftPayload) transform() r3.Transform { return dp.xform }

// placed re-evaluates the same record under the composed motion: every gate
// the build runs is a fact of the record, so a placed body re-derives the
// identical far section and walls (docs/draft-design.md Table DD row DD12).
func (dp draftPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	dp.xform = composed
	return evalDraftContext(ctx, d, ref, dp)
}

// band is the capBlendPayload view the draft's walls are built through: every
// loop chamfered on the far cap, with the cap setback dc = d across the cap
// and the side setback ds = h down the whole sweep, so the band's side level
// is the sketch plane and no straight slab remains. The view's draft flag
// selects the sharp corner rule and the prism's roles and convexity
// (docs/draft-design.md §1, §7).
func (dp draftPayload) band() capBlendPayload {
	setback := capSetback{dc: dp.d, dcDelta: dp.dDelta, ds: dp.z1 - dp.z0}
	every := map[int]bool{}
	for li := range len(dp.profile.Holes) + 1 {
		every[li] = true
	}
	cbp := capBlendPayload{
		profile: dp.profile,
		frame:   dp.frame,
		z0:      dp.z0, z1: dp.z1,
		z0Delta: dp.z0Delta, z1Delta: dp.z1Delta,
		xform:     dp.xform,
		draft:     true,
		patches:   dp.patches,
		bandDelta: dp.bandDelta,
	}
	if dp.nearStart {
		cbp.end, cbp.endLoops = setback, every
	} else {
		cbp.start, cbp.startLoops = setback, every
	}
	return cbp
}

// levels returns the near level, its axial displacement, the far level and
// the far cap's material sense in the band's convention: +1 when the far
// section is the z0 cap, −1 when it is the z1 cap.
func (dp draftPayload) levels() (nearZ, nearDelta, farZ, matSign float64) {
	if dp.nearStart {
		return dp.z0, dp.z0Delta, dp.z1, -1
	}
	return dp.z1, dp.z1Delta, dp.z0, +1
}

// extentAlong is the through-all stop's reading of the draft body (stops.go):
// the band view's own patch extrema, which read the near section at the
// sketch plane and the far section at the far level, each beside its own
// displacement.
func (dp draftPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	return dp.band().extentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
}

// axialDelta is the larger sweep-level displacement a body-relative stop
// resolving against this body must preserve.
func (dp draftPayload) axialDelta() float64 { return math.Max(dp.z0Delta, dp.z1Delta) }

// draftOffset is §8.1's offset amount: d = h·tan α as a float, beside dDelta,
// the reach from d of the exact product of three enclosures — the sweep
// height h within its two ends' displacements, the taper within its
// conversion rounding, and tan over that taper span (proofbound.RadTanSpan).
// A taper whose cosine enclosure reaches zero has no tangent to enclose and is
// SD2, ErrDegenerate. d is never exact: the tangent enclosure has width.
func draftOffset(h, hDelta, taper, taperDelta float64) (float64, float64, error) {
	rh, rhd := proofarith.FloatRat(h), proofarith.FloatRat(hDelta)
	ra, rad := proofarith.FloatRat(taper), proofarith.FloatRat(taperDelta)
	if rh == nil || rhd == nil || ra == nil || rad == nil {
		return 0, 0, errDraftOffsetUnbounded
	}
	height := proofbound.Interval(new(big.Rat).Sub(rh, rhd), new(big.Rat).Add(rh, rhd))
	tan, ok := proofbound.RadTanSpan(proofbound.Interval(new(big.Rat).Sub(ra, rad), new(big.Rat).Add(ra, rad)))
	if !ok {
		return 0, 0, fmt.Errorf(`%w: the taper's cosine enclosure reaches zero, so the drafted walls lie at a right angle to the sweep and sweep no solid`, ErrDegenerate)
	}
	mid := new(big.Rat).Add(tan.Lo, tan.Hi)
	mid.Quo(mid, big.NewRat(2, 1))
	t, _ := mid.Float64()
	d := h * t
	dDelta := proofbound.IntervalFloatError(proofbound.IntervalMul(height, tan), d)
	if proofbound.IsNonFinite(d) || proofbound.IsNonFinite(dDelta) {
		return 0, 0, errDraftOffsetUnbounded
	}
	return d, dDelta, nil
}

// errDraftOffsetUnbounded is the refusal for an offset amount whose span
// cannot be enclosed: a non-finite height, taper or product. The drafted body
// exists, and only this evaluator cannot state its far section's
// displacement.
var errDraftOffsetUnbounded = fmt.Errorf(`%w: this evaluator cannot enclose the draft's offset amount h·tan α over the stated sweep and taper`, ErrUnsupported)
