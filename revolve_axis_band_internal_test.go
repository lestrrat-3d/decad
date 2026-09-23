package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T90-T94: the revolve axis-band
// repair (revolve_axis.go's resolveAxisSide, revolve_build.go's
// revolveAxisAdmitBandCharge/revolveAxisAdmitVolumeCharge). Every fixture is
// a straight-edged rectangle in plane-local (u, v), U the axial coordinate
// and V the radial one, with every vertex an exact recorded coordinate —
// dipShaftBandProfile's own doc comment states which end carries the
// deliberate dip.
//
// The axis is hand-built as an axisLine2 rather than resolved from a
// SketchLine or ConstructionAxis: aVBound is set directly to the proven
// anchor uncertainty a tilted or offset axis would carry, which lets each
// fixture below land deliberately on one side of resolveAxisSide's own
// strict/charged split without fighting floating-point geometry for it.

// dipShaftRadius is every fixture's far (outer) edge in this file: no case
// needs a second radius, so it is a constant rather than a parameter every
// call site would otherwise repeat identically.
const dipShaftRadius = 5.0

// dipShaftBandProfile is a length x dipShaftRadius rectangle whose near
// (axis-side) edge sits at V = -dip instead of V = 0: a solid shaft revolved
// about an axis-aligned axis collapses that edge onto the axis (wallAxis, no
// face), so the body's own SNAPPED topology is a plain cylinder of
// dipShaftRadius and the stated length, while the recorded (UNSNAPPED)
// region's own area/moment integrals still carry the dip.
func dipShaftBandProfile(length, dip float64) ProfileRecord {
	const radius = dipShaftRadius
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: -dip}, End: Point2{U: length, V: -dip}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: length, V: -dip}, End: Point2{U: length, V: radius}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: length, V: radius}, End: Point2{U: 0, V: radius}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: radius}, End: Point2{U: 0, V: -dip}, TStart: 0, TEnd: 1},
	}}}
}

// axisAlignedFrame is the identity plane frame (world XY) every fixture below
// revolves in.
func axisAlignedFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

// fullTurnDenotation and quarterTurnDenotation are resolveAngularExtent's own
// FullRevolution/AngleExtent(Radians(pi/2), Along) denotations, restated
// directly (revolve.go), so a hand-built revolvePayload's sweep bound is the
// SAME tight, held-value-exact reading a real Revolve call produces — a
// zero-value den falls back to sweep()'s own conservative magnitude envelope
// (revolve_denotation.go), which would swamp every other bound in this file
// and defeat the isolation buildDipShaftBodyCharged exists for.
func fullTurnDenotation() sweepDenotation {
	return sweepDenotation{phi0: zeroAngleDenotation(), phi1: angleDenotation{rad: new(big.Rat), turn: big.NewRat(1, 1)}}
}

func quarterTurnDenotation() sweepDenotation {
	return sweepDenotation{phi0: zeroAngleDenotation(), phi1: angleDenotationFromValue(units.Radians(math.Pi / 2))}
}

// buildDipShaftBodyCharged builds the revolvePayload from an axis-ALIGNED,
// otherwise-exact axisFrame (every direction/anchor bound zero, so
// axisMoments' own use of aU/aV/dU/dV's bounds contributes nothing) carrying
// ONLY the admitted-band charge (radialAdmitAllow, axialExtentUpper) set by
// hand — the same shape resolveAxisSide itself produces whenever a boundary
// scan's own candidate (an arc's computed radius, not the axis) is what
// leaves the radial minimum unproven, isolated here from every OTHER source
// of bound so the containment assertion below tests THIS charge and no
// other.
func buildDipShaftBodyCharged(t *testing.T, profile ProfileRecord, full bool, phi1, radialAdmitAllow, axialExtentUpper float64) *Body {
	t.Helper()
	ax := axisFrame{dU: 1, dV: 0, radialAdmitAllow: radialAdmitAllow, axialExtentUpper: axialExtentUpper}
	den := quarterTurnDenotation()
	if full {
		den = fullTurnDenotation()
	}
	rp := revolvePayload{
		profile: profile,
		frame:   axisAlignedFrame(t),
		ax:      ax,
		phi0:    0, phi1: phi1,
		full:  full,
		den:   den,
		xform: r3.Identity(),
	}
	body, err := evalRevolveContext(t.Context(), New(), producerID(0), rp)
	require.NoError(t, err)
	return body
}

// T90: a dipped profile's published volume interval contains the volume its
// own face set encloses. radialAdmitAllow/axialExtentUpper are the interval
// resolveAxisSide's own charged path (survStraddle in its admission switch)
// carries forward whenever the boundary scan itself — never the axis, held
// zero-bound here — cannot prove the radial minimum clear of zero: the same
// mechanism as the investigation's own 100 mm/1e-7 mm shaft (T93 below is
// that exact fixture, which the strict half now refuses outright), scaled up
// so the gap this charge must cover (2π·dip²·L, the investigation's own
// formula) clears every OTHER analytic rounding term in this build by a
// comfortable margin, and the assertion below tests THIS charge rather than
// an incidental one.
func TestRevolveAxisBandVolumeContainsEnclosedVolume(t *testing.T) {
	t.Parallel()
	const length, dip = 1000.0, 1e-3
	profile := dipShaftBandProfile(length, dip)

	body := buildDipShaftBodyCharged(t, profile, true, 2*math.Pi, 2*dip, length)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, Approximate, vol.Exactness)
	volVal, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	volBound, err := vol.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Positive(t, volBound, "the admitted band must charge a nonzero volume bound")

	// The body's own SNAPPED topology is a plain cylinder: the near wall,
	// both endpoints within snapTol of the axis, contributes no face.
	enclosed := math.Pi * dipShaftRadius * dipShaftRadius * length
	require.LessOrEqual(t, math.Abs(enclosed-volVal), volBound,
		"the published volume interval must contain the volume the body's own faces enclose")
}

// T91: the partial-sweep area case, the sharper of the two measured
// consequences: a quarter sweep's cap loops are built from the SNAPPED
// profile while their published area is the Pappus engine's integral over
// the UNSNAPPED one.
func TestRevolveAxisBandPartialSweepAreaContainsEnclosedArea(t *testing.T) {
	t.Parallel()
	const length, dip = 1000.0, 1e-3
	profile := dipShaftBandProfile(length, dip)

	body := buildDipShaftBodyCharged(t, profile, false, math.Pi/2, 2*dip, length)

	// Both cap faces (the meridian wedge planes) carry the SAME admitted
	// band charge; each's own snapped-loop area is radius*length exactly.
	var found int
	for _, f := range body.Faces() {
		if f.origins[0].Role != roleCapStart && f.origins[0].Role != roleCapEnd {
			continue
		}
		found++
		ar, err := f.Area()
		require.NoError(t, err)
		require.Equal(t, Approximate, ar.Exactness)
		areaVal, err := ar.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		areaBound, err := ar.Bound.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.Positive(t, areaBound, "the admitted band must charge a nonzero cap area bound")

		enclosed := dipShaftRadius * length
		require.LessOrEqual(t, math.Abs(enclosed-areaVal), areaBound,
			"the published cap area interval must contain the area its own (snapped) loop encloses")
	}
	require.Equal(t, 2, found, "a partial sweep publishes exactly two cap faces")
}

// T92: an axis-aligned fixture's allowance is exactly zero, so the charge
// cannot silently widen the common case — every deterministic fixture in
// this tree revolves about an origin-anchored, axis-aligned axis, where the
// radial arithmetic is exact.
func TestRevolveAxisBandAllowanceZeroForExactAxis(t *testing.T) {
	t.Parallel()
	profile := dipShaftBandProfile(100, 0)
	line := axisLine2{dU: 1, dV: 0}
	ax, side, err := resolveAxisSide(t.Context(), profile, line, newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, 1.0, side)
	require.Zero(t, ax.radialAdmitAllow, "an exact axis proving the radial minimum non-negative charges nothing")
	require.Zero(t, revolveAxisAdmitBandCharge(ax))
	require.Zero(t, revolveAxisAdmitVolumeCharge(ax))
}

// T93: a profile whose radial minimum is proven negative, with an exact
// axis, is refused — the strict half of the repair: an admission gate may
// never rest on a tolerance. This is the exact fixture the investigation
// measured (a 100 mm shaft with a 1e-7 mm dip, axis-aligned): under the
// pre-repair tolerance it was silently admitted, and the strict gate refuses
// it outright now that the radial bound is proven zero.
func TestRevolveAxisBandRefusesProvenNegativeRadialMinimumUnderExactAxis(t *testing.T) {
	t.Parallel()
	profile := dipShaftBandProfile(100, 1e-7)
	line := axisLine2{dU: 1, dV: 0}
	_, _, err := resolveAxisSide(t.Context(), profile, line, newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate)
}

// T94: pinning the roff/zoff charge (defect #2). An axis anchored away from
// the origin, with a direction whose components are not exactly
// representable, commits real floating-point rounding computing roff =
// nU*aU+nV*aV — the same fault TestRevolvePayloadProvesSimpleChargesTheAxisOffsetShift
// pins one level up, only here it is resolveAxisSide's own build-time gate
// rather than a post-hoc review. The fixture mirrors that test's numbers
// exactly (aV = 1e10, a profile edge at v = aV-1e-7 held to v = aV-1e-7+5),
// which is far enough from the axis's own anchor that resolveAxisSide's
// coarse side classification straddles rather than resolving cleanly — the
// SAME rounding this test pins is what makes that straddle happen, so the
// bound the accompanying error names is asserted nonzero rather than the
// interval being read further.
func TestRevolveAxisBandChargesTheOffsetSubtraction(t *testing.T) {
	t.Parallel()
	const aV = 1e10
	const eps = 1e-7
	line := axisLine2{dU: 0.8, dV: 0.6, aV: aV}
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: aV - eps}, End: Point2{U: 0, V: aV - eps + 5}, TStart: 0, TEnd: 1},
	}}}
	_, _, err := resolveAxisSide(t.Context(), profile, line, newFreeformWork())
	require.Error(t, err)
	require.Contains(t, err.Error(), "does not decide which side")
	// The error's own stated bound is the roff charge itself: zero would mean
	// the offset subtraction's rounding went uncounted, exactly defect #2.
	msg := err.Error()
	require.NotContains(t, msg, "±0 mm", "the roff charge must be nonzero, never silently dropped")
}
