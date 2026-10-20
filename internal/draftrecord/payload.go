package draftrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file is the record of a draft body (agent-docs/draft-design.md Table BD row
// BD1), the offset amount its far section is built at (§8.1), and the
// readings of the record alone that other code needs. The build itself is
// build.go and the measurements measure.go. Payload implements topo.Payload
// (agent-docs/evaluator-design.md §8).

// Payload is the evaluator's own record of a tapered extrude: the near
// section P as recorded, the frame it lifts through, the sweep interval with
// each end's own axial displacement, which end P sits at, the taper in
// radians beside its conversion rounding, and the accumulated placement. The
// build fills in the rest from those: the offset amount d with its proven
// span, and the far section Q = P offset sharply by d with the displacement
// its built contour carries (agent-docs/draft-design.md §2, §8.1).
type Payload struct {
	Profile momentinput.Profile
	Frame   r3.Frame
	Z0, Z1  float64
	Z0Delta float64
	Z1Delta float64
	// NearStart is true when P sits at z0 (an Along or forward stop extent)
	// and false when it sits at z1 (Against or backward stop). The far
	// section sits at the other end.
	NearStart bool
	// Taper is the signed taper in radians and TaperDelta its unit
	// conversion's rounding: the stated angle lies within TaperDelta of it.
	Taper, TaperDelta float64
	Xform             r3.Transform
	// Kept names the recorded segments of the walls a subset draft leaves
	// vertical (agent-docs/draft-design.md §10.2), keyed by loop and segment; empty
	// for a tapered extrude or a draft of every wall, which moves them all.
	Kept map[offset2d.DraftWall]struct{}

	// D is the held offset amount h·tan α and DDelta the reach of every amount
	// the stated sweep and taper denote from it (Offset).
	D, DDelta float64
	// Far is the far section Q as built, one loop per loop of profile in the
	// same order, and FarDelta the largest contour displacement any of its
	// loops carries (capband.ContourDisplacement).
	Far      momentinput.Profile
	FarDelta float64
	// Patches is every wall patch's side(i, j) role beside the plane-local
	// geometry capblend.BuildBand built it from, in loop order then walk order, and
	// BandDelta each loop's own far contour displacement keyed as the band view
	// keys it. Both are filled once by the build and handed to the band view
	// (Band), so the tessellator (Tessellate) charges the same patch
	// geometry and contour displacement every reading of the body charged.
	// Both are plane-local and placement-invariant.
	Patches   []capblend.Patch
	BandDelta map[capblend.BandKey]float64
}

// Transform is the accumulated rigid placement.
func (dp Payload) Transform() r3.Transform { return dp.Xform }

// Placed re-evaluates the same record under the composed motion: every gate
// the build runs is a fact of the record, so a placed body re-derives the
// identical far section and walls (agent-docs/draft-design.md Table DD row DD12).
func (dp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	dp.Xform = composed
	return Build(ctx, env, dp)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (agent-docs/evaluator-design.md §8): the root type this payload replaced.
func (Payload) SurveyClass() string { return "draftPayload" }

// Band is the capblend.Payload view the draft's walls are built through: every
// loop chamfered on the far cap, with the cap setback dc = d across the cap
// and the side setback ds = h down the whole sweep, so the band's side level
// is the sketch plane and no straight slab remains. The view's draft flag
// selects the sharp corner rule and the prism's roles and convexity, and its
// draftKept the walls a subset draft leaves in place (agent-docs/draft-design.md §1,
// §7, §10.2).
func (dp Payload) Band() capblend.Payload {
	setback := capblend.Setback{Dc: dp.D, DcDelta: dp.DDelta, Ds: dp.Z1 - dp.Z0}
	every := map[int]bool{}
	for li := range len(dp.Profile.Holes) + 1 {
		every[li] = true
	}
	cbp := capblend.Payload{
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

// Levels returns the near level, its axial displacement, the far level and
// the far cap's material sense in the band's convention: +1 when the far
// section is the z0 cap, −1 when it is the z1 cap.
func (dp Payload) Levels() (nearZ, nearDelta, farZ, matSign float64) {
	if dp.NearStart {
		return dp.Z0, dp.Z0Delta, dp.Z1, -1
	}
	return dp.Z1, dp.Z1Delta, dp.Z0, +1
}

// ExtentAlong is the through-all stop's reading of the draft body (stops.go):
// the band view's own patch extrema, which read the near section at the
// sketch plane and the far section at the far level, each beside its own
// displacement.
func (dp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	return dp.Band().ExtentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
}

// AuditExtent reads the analytic extent the composite sweep audit needs
// (compositesweep.ExtentReader).
func (dp Payload) AuditExtent(
	ctx context.Context, direction r3.Vec, work *freeform.FreeformWork,
) (float64, float64, float64, error) {
	return dp.Band().ExtentBoundedAlong(ctx, direction, work)
}

// AuditEndpointSupports reports that each draft slab stays between its two
// axial cap planes (compositesweep.EndpointSupporter).
func (dp Payload) AuditEndpointSupports() bool { return true }

// AxialDelta is the larger sweep-level displacement a body-relative stop
// resolving against this body must preserve.
func (dp Payload) AxialDelta() float64 { return math.Max(dp.Z0Delta, dp.Z1Delta) }

// MoverRecordRadius bounds every point of the draft body from its record.
// Every draft point lies on an end section or a ruling between them. Bound
// both section coordinate envelopes, then include the far contour's gap.
func (dp Payload) MoverRecordRadius() float64 {
	nearUpper, err := motionbound.DraftSectionCoordinateUpper(dp.Profile)
	if err != nil {
		return math.Inf(1)
	}
	farUpper, err := motionbound.DraftSectionCoordinateUpper(dp.Far)
	if err != nil {
		return math.Inf(1)
	}
	coordUpper := math.Max(nearUpper, proofbound.AbsSumUpper(farUpper, dp.FarDelta))
	zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(dp.Z0), math.Abs(dp.Z1)), dp.AxialDelta())
	return motionbound.LinearRecordRadius(dp.Frame, coordUpper, zUpper)
}

// CapPrisms is the near section at the near level and the recorded far
// section at the far level, each as a prism of zero height, beside the
// displacement their stations carry: the levels' and FarDelta.
func (dp Payload) CapPrisms() ([]prismrecord.Payload, float64) {
	nearZ, _, farZ, _ := dp.Levels()
	level := func(profile momentinput.Profile, z float64) prismrecord.Payload {
		return prismrecord.Payload{Profile: profile, Frame: dp.Frame, Z0: z, Z1: z, Xform: dp.Xform}
	}
	return []prismrecord.Payload{level(dp.Profile, nearZ), level(dp.Far, farZ)}, proofbound.AbsSumUpper(dp.AxialDelta(), dp.FarDelta)
}

// RequireNotReceiver is modify S3 for a draft body
// (agent-docs/draft-design.md Table DD row DD14) outside its bounded rim routes.
// It runs beside the cap-blend receiver check, so the refusal names the draft
// body rather than the generic class.
func RequireNotReceiver(payload topo.Payload, op string) error {
	switch payload.(type) {
	case Payload, TwoSidedPayload:
		return fmt.Errorf(`%w: this evaluator %s a straight prism, a revolve or their modify results; this draft body (a tapered extrude) is outside its bounded rim routes (modify S3)`, decaderr.ErrUnsupported, op)
	}
	return nil
}

// Offset is §8.1's offset amount: d = h·tan α as a float, beside dDelta,
// the reach from d of the exact product of three enclosures — the sweep
// height h within its two ends' displacements, the taper within its
// conversion rounding, and tan over that taper span (proofbound.RadTanSpan).
// A taper whose cosine enclosure reaches zero has no tangent to enclose and is
// SD2, ErrDegenerate. d is never exact: the tangent enclosure has width.
func Offset(h, hDelta, taper, taperDelta float64) (float64, float64, error) {
	rh, rhd := proofarith.FloatRat(h), proofarith.FloatRat(hDelta)
	ra, rad := proofarith.FloatRat(taper), proofarith.FloatRat(taperDelta)
	if rh == nil || rhd == nil || ra == nil || rad == nil {
		return 0, 0, errOffsetUnbounded
	}
	height := proofbound.Interval(new(big.Rat).Sub(rh, rhd), new(big.Rat).Add(rh, rhd))
	tan, ok := proofbound.RadTanSpan(proofbound.Interval(new(big.Rat).Sub(ra, rad), new(big.Rat).Add(ra, rad)))
	if !ok {
		return 0, 0, fmt.Errorf(`%w: the taper's cosine enclosure reaches zero, so the drafted walls lie at a right angle to the sweep and sweep no solid`, decaderr.ErrDegenerate)
	}
	mid := new(big.Rat).Add(tan.Lo, tan.Hi)
	mid.Quo(mid, big.NewRat(2, 1))
	t, _ := mid.Float64()
	d := h * t
	dDelta := proofbound.IntervalFloatError(proofbound.IntervalMul(height, tan), d)
	if proofbound.IsNonFinite(d) || proofbound.IsNonFinite(dDelta) {
		return 0, 0, errOffsetUnbounded
	}
	return d, dDelta, nil
}

// errOffsetUnbounded is the refusal for an offset amount whose span
// cannot be enclosed: a non-finite height, taper or product. The drafted body
// exists, and only this evaluator cannot state its far section's
// displacement.
var errOffsetUnbounded = fmt.Errorf(`%w: this evaluator cannot enclose the draft's offset amount h·tan α over the stated sweep and taper`, decaderr.ErrUnsupported)
