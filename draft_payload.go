package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	Profile profileRecord
	Frame   r3.Frame
	Z0, Z1  float64
	Z0Delta float64
	Z1Delta float64
	// nearStart is true when P sits at z0 (an Along or forward stop extent)
	// and false when it sits at z1 (Against or backward stop). The far
	// section sits at the other end.
	NearStart bool
	// taper is the signed taper in radians and taperDelta its unit
	// conversion's rounding: the stated angle lies within taperDelta of it.
	Taper, TaperDelta float64
	Xform             r3.Transform
	// kept names the recorded segments of the walls a subset draft leaves
	// vertical (docs/draft-design.md §10.2), keyed by loop and segment; empty
	// for a tapered extrude or a draft of every wall, which moves them all.
	Kept map[offset2d.DraftWall]struct{}

	// d is the held offset amount h·tan α and dDelta the reach of every amount
	// the stated sweep and taper denote from it (draftOffset).
	D, DDelta float64
	// far is the far section Q as built, one loop per loop of profile in the
	// same order, and farDelta the largest contour displacement any of its
	// loops carries (capband.ContourDisplacement).
	Far      profileRecord
	FarDelta float64
	// patches is every wall patch's side(i, j) role beside the plane-local
	// geometry capblend.BuildBand built it from, in loop order then walk order, and
	// bandDelta each loop's own far contour displacement keyed as the band view
	// keys it. Both are filled once by the build and handed to the band view
	// (band), so the tessellator (tessellate_draft.go) charges the same patch
	// geometry and contour displacement every reading of the body charged.
	// Both are plane-local and placement-invariant.
	Patches   []capPatch
	BandDelta map[capBandKey]float64
}

// Transform is the accumulated rigid placement.
func (dp draftPayload) Transform() r3.Transform { return dp.Xform }

// Placed re-evaluates the same record under the composed motion: every gate
// the build runs is a fact of the record, so a placed body re-derives the
// identical far section and walls (docs/draft-design.md Table DD row DD12).
func (dp draftPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	dp.Xform = composed
	return placedNode(evalDraftContext(ctx, d, ref, dp))
}

// band is the capBlendPayload view the draft's walls are built through: every
// loop chamfered on the far cap, with the cap setback dc = d across the cap
// and the side setback ds = h down the whole sweep, so the band's side level
// is the sketch plane and no straight slab remains. The view's draft flag
// selects the sharp corner rule and the prism's roles and convexity, and its
// draftKept the walls a subset draft leaves in place (docs/draft-design.md §1,
// §7, §10.2).
func (dp draftPayload) Band() capBlendPayload {
	setback := capSetback{Dc: dp.D, DcDelta: dp.DDelta, Ds: dp.Z1 - dp.Z0}
	every := map[int]bool{}
	for li := range len(dp.Profile.Holes) + 1 {
		every[li] = true
	}
	cbp := capBlendPayload{
		Profile: dp.Profile,
		Frame:   dp.Frame,
		Z0:      dp.Z0, Z1: dp.Z1,
		Z0Delta: dp.Z0Delta, Z1Delta: dp.Z1Delta,
		Xform:     dp.Xform,
		Draft:     true,
		Patches:   dp.Patches,
		BandDelta: dp.BandDelta,
		DraftKept: dp.Kept,
	}
	if dp.NearStart {
		cbp.End, cbp.EndLoops = setback, every
	} else {
		cbp.Start, cbp.StartLoops = setback, every
	}
	return cbp
}

// levels returns the near level, its axial displacement, the far level and
// the far cap's material sense in the band's convention: +1 when the far
// section is the z0 cap, −1 when it is the z1 cap.
func (dp draftPayload) Levels() (nearZ, nearDelta, farZ, matSign float64) {
	if dp.NearStart {
		return dp.Z0, dp.Z0Delta, dp.Z1, -1
	}
	return dp.Z1, dp.Z1Delta, dp.Z0, +1
}

// ExtentAlong is the through-all stop's reading of the draft body (stops.go):
// the band view's own patch extrema, which read the near section at the
// sketch plane and the far section at the far level, each beside its own
// displacement.
func (dp draftPayload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	return dp.Band().ExtentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
}

// AuditExtent reads the analytic extent the composite sweep audit needs
// (compositesweep.ExtentReader).
func (dp draftPayload) AuditExtent(
	ctx context.Context, direction r3.Vec, work *freeform.FreeformWork,
) (float64, float64, float64, error) {
	return dp.Band().ExtentBoundedAlong(ctx, direction, work)
}

// AuditEndpointSupports reports that each draft slab stays between its two
// axial cap planes (compositesweep.EndpointSupporter).
func (dp draftPayload) AuditEndpointSupports() bool { return true }

// AxialDelta is the larger sweep-level displacement a body-relative stop
// resolving against this body must preserve.
func (dp draftPayload) AxialDelta() float64 { return math.Max(dp.Z0Delta, dp.Z1Delta) }

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
