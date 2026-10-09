package decad

import (
	"context"
	"math"
	"math/big"
	"math/rand"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md's per-gate (G1-G6, §3.1)
// white-box test suite: each gate is isolated directly against
// admitPrismPair/tryPrismBoolean so a miss is confirmed precisely, without
// depending on whichever refusal the mesh path's own fallback happens to
// produce for a given geometry (apitest/prism_boolean_test.go covers that richer,
// public-API shape separately). A synthetic prismPayload is used where a gate
// is easiest isolated from one built directly (G1, G4): a live prismPayload
// can hold a free-form segment via Extrude since §10 P4b, but G4's own
// analytic-profile gate still refuses it (this file's own G4 tests cover
// that), and no evaluator path lets an operand answer with a
// non-prismPayload payload while still resembling one, so both gates are
// exercised against a value built directly.

// canonicalPrismFrame is the plane-local frame every synthetic payload below
// starts from: literal-zero U/V/origin, so its own N() is exactly (0,0,1)
// with no float rounding of its own to confuse a gate result with.
func canonicalPrismFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// synthLineLoop is placeholder single-segment geometry for a gate test that
// never reaches resolution (admitPrismPair reads only segment KIND, never
// shape) — every caller uses the same coordinates.
func synthLineLoop() LoopRecord {
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1},
	}}
}

// synthRectLoop is a proper closed, CCW rectangular loop — the shape G5/G6's
// full tryPrismBoolean tests need, since (unlike admitPrismPair's own gates)
// resolution actually arranges the operands' geometry through sketch.
func synthRectLoop(u0, v0, u1, v1 float64) LoopRecord {
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: u0, V: v0}, End: Point2{U: u1, V: v0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1, V: v0}, End: Point2{U: u1, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1, V: v1}, End: Point2{U: u0, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u0, V: v1}, End: Point2{U: u0, V: v0}, TStart: 0, TEnd: 1},
	}}
}

func synthDenseRectLoop(segmentsPerSide int) LoopRecord {
	point := func(i int) Point2 {
		switch {
		case i <= segmentsPerSide:
			return Point2{U: float64(i)}
		case i <= 2*segmentsPerSide:
			return Point2{U: float64(segmentsPerSide), V: float64(i - segmentsPerSide)}
		case i <= 3*segmentsPerSide:
			return Point2{U: float64(3*segmentsPerSide - i), V: float64(segmentsPerSide)}
		default:
			return Point2{V: float64(4*segmentsPerSide - i)}
		}
	}
	count := 4 * segmentsPerSide
	segs := make([]CurveSegment, count)
	for i := range count {
		segs[i] = LineSeg{Start: point(i), End: point(i + 1), TStart: 0, TEnd: 1}
	}
	return LoopRecord{Segments: segs}
}

func TestPrismBooleanGateG1RequiresBothOperandsPrismPayload(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: facetedPayload{}} // any non-prismPayload featurePayload

	_, _, ok := admitPrismPair(a, b)
	require.False(t, ok, "G1: a non-prismPayload operand must never admit")
}

// TestPrismBooleanGateG3RequiresCoDirectionalSharedAxisPlanes isolates G3's
// two arms (§3.1). Shown to fail, one deletion at a time: with
// admitPrismPairBudget's prismSharedAxisOf call deleted (the coplanar arm
// alone), "offset frame on the normal axis clears G3" went red; with
// prismSharedAxisOf's exact cross-product test deleted, "an in-plane origin
// component refuses" went red; with its placement comparison deleted,
// "placed along the normal stays outside the shared-axis arm" went red; with
// its U/V comparison deleted, "U/V bits one ulp apart refuse" went red.
func TestPrismBooleanGateG3RequiresCoDirectionalSharedAxisPlanes(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}

	t.Run("antiparallel normal (co-planar but not co-directional)", func(t *testing.T) {
		antiFrame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
		require.NoError(t, err)
		require.Equal(t, r3.NewVec(0, 0, -1), antiFrame.N())
		antiPP := pp
		antiPP.frame = antiFrame
		b := &Body{payload: antiPP}
		_, _, ok := admitPrismPair(a, b)
		require.False(t, ok)
	})

	t.Run("placed along the normal stays outside the shared-axis arm", func(t *testing.T) {
		shifted, err := r3.Translation(r3.Vec{Z: 3})
		require.NoError(t, err)
		shiftedPP := pp
		shiftedPP.xform = shifted
		b := &Body{payload: shiftedPP}
		_, _, ok := admitPrismPair(a, b)
		require.False(t, ok)
	})

	t.Run("offset frame on the normal axis clears G3", func(t *testing.T) {
		frameB, err := r3.NewFrame(r3.Vec{Z: -16}, frame.U(), frame.V())
		require.NoError(t, err)
		offsetPP := pp
		offsetPP.frame = frameB
		b := &Body{payload: offsetPP}
		pa, pb, ok := admitPrismPair(a, b)
		require.True(t, ok)
		require.Zero(t, prismZShift(pa, pb).Cmp(big.NewRat(-16, 1)), "G5's shift is the exact origin offset along N")
	})

	t.Run("an in-plane origin component refuses", func(t *testing.T) {
		frameB, err := r3.NewFrame(r3.Vec{X: 3, Z: -16}, frame.U(), frame.V())
		require.NoError(t, err)
		offsetPP := pp
		offsetPP.frame = frameB
		b := &Body{payload: offsetPP}
		_, _, ok := admitPrismPair(a, b)
		require.False(t, ok)
	})

	t.Run("U/V bits one ulp apart refuse", func(t *testing.T) {
		// r3.NewFrame normalises U = (0.7071067811865475, 0.7071067811865475, 0)
		// and (0.7071067811865476, 0.7071067811865476, 0) to the same bits, so
		// the pair is the one a caller meets: a tilted plane and its
		// sketch.CreateOffsetPlane by -16, whose plane frames hold U one ulp
		// apart, each rebuilt through r3.NewFrame the way Extrude records it.
		// The rebuilt frames hold equal U and N and V one ulp apart, with the
		// origin difference exactly along N, so the U/V comparison is the one
		// leg that refuses.
		w := sketch.NewWorld()
		tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 1))
		require.NoError(t, err)
		base, err := w.CreatePlaneFromFrame(tilted)
		require.NoError(t, err)
		below, err := w.CreateOffsetPlane(base, -16)
		require.NoError(t, err)
		planeA, err := base.Frame()
		require.NoError(t, err)
		planeB, err := below.Frame()
		require.NoError(t, err)
		require.NotEqual(t, planeA.U(), planeB.U(), "premise: the two plane frames' U differ in the stored bits")
		fa, err := r3.NewFrame(planeA.Origin(), planeA.U(), planeA.V())
		require.NoError(t, err)
		fb, err := r3.NewFrame(planeB.Origin(), planeB.U(), planeB.V())
		require.NoError(t, err)
		require.Equal(t, fa.N(), fb.N(), "premise: the rebuilt normals agree, so G3's normal check passes")
		require.NotEqual(t, [2]r3.Vec{fa.U(), fa.V()}, [2]r3.Vec{fb.U(), fb.V()}, "premise: the rebuilt U/V differ in the stored bits")
		d := proofarith.DvSub(proofarith.DyVec(fb.Origin()), proofarith.DyVec(fa.Origin()))
		require.True(t, proofarith.DvIsZero(proofarith.DvCross(d, proofarith.DyVec(fa.N()))), "premise: the origin difference lies exactly along N")
		tiltedA := pp
		tiltedA.frame = fa
		tiltedB := pp
		tiltedB.frame = fb
		_, _, ok := admitPrismPair(&Body{payload: tiltedA}, &Body{payload: tiltedB})
		require.False(t, ok)
	})

	t.Run("exactly co-directional and coplanar clears G3", func(t *testing.T) {
		b := &Body{payload: pp}
		_, _, ok := admitPrismPair(a, b)
		require.True(t, ok)
	})
}

func TestPrismBooleanGateG4RefusesNonAnalyticSegment(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	linePP := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	ellipsePP := linePP
	ellipsePP.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		EllipseSeg{Center: Point2{U: 20, V: 0}, Rx: units.Millimeters(4), Ry: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1},
	}}}

	a := &Body{payload: linePP}
	b := &Body{payload: ellipsePP}
	_, _, ok := admitPrismPair(a, b)
	require.False(t, ok, "G4: a non-analytic (ellipse) segment must never admit")

	// A line/circle/arc-only pair still clears G1-G4 (isolates G4).
	otherLinePP := linePP
	otherLinePP.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: Point2{U: 5, V: 5}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1},
	}}}
	c := &Body{payload: otherLinePP}
	_, _, ok = admitPrismPair(a, c)
	require.True(t, ok)
}

func TestPrismBooleanGateG5RequiresMatchingZInterval(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	t.Run("unequal heights from the same plane", func(t *testing.T) {
		pb := pa
		pb.z1 = 15
		require.False(t, prismUnionZIntervalMatches(pa, pb))
	})

	t.Run("matching interval, re-expressed through an offset frame", func(t *testing.T) {
		// B starts its own z0/z1 at [0, 10] in its own frame, whose origin sits
		// 3mm along the shared normal (G3's shared-axis arm): G5 must read the
		// SHIFTED interval [3, 13], not B's own unshifted one.
		offset, err := r3.NewFrame(r3.Vec{Z: 3}, frame.U(), frame.V())
		require.NoError(t, err)
		pb := pa
		pb.frame = offset
		require.False(t, prismUnionZIntervalMatches(pa, pb), "A's [0,10] must not match B's shifted [3,13]")

		paShiftedToMatch := pa
		paShiftedToMatch.z0, paShiftedToMatch.z1 = 3, 13
		require.True(t, prismUnionZIntervalMatches(paShiftedToMatch, pb))
	})
}

// TestPrismBooleanGateG5ShiftIsExactRational covers G5's comparisons over
// G3's shared-axis arm (§3.1): B's interval is lifted onto A's axis by the
// exact rational shift, never by a float sum. Shown to fail: with
// CutZIntervalSpans comparing the float sums tool.z0+shift and
// tool.z1+shift instead of the big.Rat lift, "a hair short refused" went red
// (the float sum 15.6 + 0.4 rounds onto 16).
func TestPrismBooleanGateG5ShiftIsExactRational(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	offsetBy := func(t *testing.T, z float64) r3.Frame {
		t.Helper()
		f, err := r3.NewFrame(r3.Vec{Z: z}, frame.U(), frame.V())
		require.NoError(t, err)
		return f
	}
	payload := func(f r3.Frame, z0, z1 float64) prismPayload {
		return prismPayload{
			profile: ProfileRecord{Outer: synthLineLoop()},
			frame:   f, z0: z0, z1: z1, xform: r3.Identity(),
		}
	}

	t.Run("meeting caps admitted", func(t *testing.T) {
		target := payload(frame, 0, 16)
		tool := payload(offsetBy(t, -16), 0, 32)
		require.True(t, prismplacement.CutZIntervalSpans(prismPlacementOf(target), prismPlacementOf(tool)))
	})

	t.Run("a hair short refused", func(t *testing.T) {
		target := payload(frame, 0, 16)
		tool := payload(offsetBy(t, 0.4), -1, 15.6)
		z1, shift := 15.6, 0.4
		require.Equal(t, 16.0, z1+shift, "premise: the float sum rounds onto the target's cap")
		require.Negative(t, new(big.Rat).Add(proofarith.FloatRat(z1), proofarith.FloatRat(shift)).Cmp(big.NewRat(16, 1)),
			"premise: the exact sum falls short of the target's cap")
		require.False(t, prismplacement.CutZIntervalSpans(prismPlacementOf(target), prismPlacementOf(tool)))
	})

	t.Run("union matches the shifted interval exactly", func(t *testing.T) {
		pa := payload(frame, 0, 10)
		require.True(t, prismUnionZIntervalMatches(pa, payload(offsetBy(t, -16), 16, 26)))
		require.False(t, prismUnionZIntervalMatches(pa, payload(offsetBy(t, -16), 16, 26.000000000000004)))
	})

	t.Run("intersect overlap over the shifted interval", func(t *testing.T) {
		pa := payload(frame, 0, 10)
		require.True(t, prismIntersectZIntervalOverlaps(pa, payload(offsetBy(t, -16), 20, 30)))
		require.False(t, prismIntersectZIntervalOverlaps(pa, payload(offsetBy(t, -16), 26, 30)))
	})
}

// TestPrismIntersectShiftedEndpointChargesItsRounding is §7's one new axial
// term: Intersect publishes B's shifted cap fl(0.1) + fl(0.3), which is no
// float, rounded once and charged into z1Delta. Shown to fail: with the
// rationalFloatError term deleted from prismIntersectEnd, z1Delta came back
// 0 and the require.Positive assertion went red.
func TestPrismIntersectShiftedEndpointChargesItsRounding(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	offset, err := r3.NewFrame(r3.Vec{Z: 0.1}, frame.U(), frame.V())
	require.NoError(t, err)
	a := &Body{payload: prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 1, xform: r3.Identity(),
	}}
	b := &Body{payload: prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(3, 3, 7, 7)},
		frame:   offset, z0: 0, z1: 0.3, xform: r3.Identity(),
	}}

	result, ok, err := tryPrismBoolean(t.Context(), meshbool.OpIntersect, a, b)
	require.NoError(t, err)
	require.True(t, ok, "a nested pair on shared-axis offset planes takes the analytic path")

	require.Equal(t, 0.1, result.z0, "B's shifted z0 is 0 + fl(0.1), itself a float")
	require.Zero(t, result.z0Delta)

	exact := new(big.Rat).Add(proofarith.FloatRat(0.3), proofarith.FloatRat(0.1))
	nearest, isFloat := exact.Float64()
	require.False(t, isFloat, "premise: fl(0.1) + fl(0.3) is no float")
	require.Equal(t, nearest, result.z1)
	require.Positive(t, result.z1Delta)
	// B's incoming z1Delta is 0, so the published term is the rounding charge
	// folded through proofbound.AbsSumUpper's outward rounding.
	require.GreaterOrEqual(t, result.z1Delta, proofarith.RationalFloatError(exact, result.z1))
	require.Equal(t, proofbound.AbsSumUpper(0, proofarith.RationalFloatError(exact, result.z1)), result.z1Delta)

	body, err := evalPrism(New(), producerID(0), result, freeform.NewFreeformWork())
	require.NoError(t, err)
	vol, err := body.Volume()
	require.NoError(t, err)
	value, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := vol.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	want := new(big.Rat).Mul(big.NewRat(16, 1), proofarith.FloatRat(0.3))
	gap := new(big.Rat).Sub(proofarith.FloatRat(value), want)
	require.LessOrEqual(t, gap.Abs(gap).Cmp(proofarith.FloatRat(bound)), 0,
		"the published volume bound contains the exact rational volume 16·fl(0.3)")
}

func TestPrismBooleanGateG6RestrictsUnionToHoleFreeOperands(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	holeFree := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	overlapping := holeFree
	overlapping.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
	holed := holeFree
	holed.profile = ProfileRecord{
		Outer: synthRectLoop(5, 5, 15, 15),
		Holes: []LoopRecord{synthRectLoop(8, 8, 9, 9)},
	}

	a := &Body{payload: holeFree}
	b := &Body{payload: holed}
	_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, a, b)
	require.NoError(t, err)
	require.False(t, ok, "G6: a holed operand must never admit a Union")

	c := &Body{payload: overlapping}
	_, ok, err = tryPrismBoolean(t.Context(), meshbool.OpUnion, a, c)
	require.NoError(t, err)
	require.True(t, ok, "a hole-free pair otherwise identical clears G6")
}

// TestPrismUnionReexpressedSplitChargesTheCrossing is A6's crossing charge
// (docs/general-boolean-design.md §3 A6) on a nonidentity re-expression: B's
// lower edge, rotated by θ = 0.01, crosses A's top edge at that angle, so
// B's re-expression rounding δ_B can move the crossing by δ_B/sin θ. The
// union builds, its sectionDelta covers δ_B/0.01 (sin 0.01 < 0.01), and its
// volume bound contains the exact residual against A ∪ B taken over
// math/big.Rat from the two records and B's stored placement. Shown to fail
// with prismSceneDelta.Merged's crossing term deleted.
func TestPrismUnionReexpressedSplitChargesTheCrossing(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	const theta = 0.01
	basis := r3.Basis{
		EX: r3.Vec{X: math.Cos(theta), Y: math.Sin(theta), Z: 0},
		EY: r3.Vec{X: -math.Sin(theta), Y: math.Cos(theta), Z: 0},
		EZ: r3.Vec{X: 0, Y: 0, Z: 1},
	}
	rotation, err := r3.FromBasis(basis, r3.Vec{})
	require.NoError(t, err)
	pb := pa
	// The lower long edge crosses A's upper edge at theta, so this fixture
	// reaches the trim-amplification path without relying on a degenerate
	// contact classification.
	pb.profile = ProfileRecord{Outer: synthRectLoop(-5, 9.9, 15, 11.9)}
	pb.xform = rotation
	_, _, admitted := admitPrismPair(&Body{payload: pa}, &Body{payload: pb})
	require.True(t, admitted, "the fixture must clear G1-G4 before the split guard runs")
	require.True(t, prismUnionZIntervalMatches(pa, pb), "the fixture must clear G5")

	reexpression, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.False(t, reexpression.Identity)
	scene, _, _, err := buildPrismScene(proofbound.NewWorkBudget(t.Context()), pa, pb, reexpression)
	require.NoError(t, err)
	profiles, err := prismProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)

	split := false
	for _, profile := range profiles {
		require.True(t, profile.Valid, "the shallow crossing must not depend on an invalid arrangement")
		for _, loop := range append([][]sketch.BoundaryEdge{profile.Outer}, profile.Holes...) {
			for _, edge := range loop {
				split = split || edge.Partial
			}
		}
	}
	require.True(t, split, "the overlapping rectangles must produce a split boundary")

	_, inB := prismSceneDelta{}.Incoming(pa.sectionDelta, pb.sectionDelta, reexpression.Delta)
	require.Positive(t, inB)

	result, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, &Body{payload: pa}, &Body{payload: pb})
	require.NoError(t, err)
	require.True(t, ok, "A6 charges the shallow crossing instead of rerouting it")
	require.GreaterOrEqual(t, result.sectionDelta, inB/0.01,
		"the crossing at sin θ < 0.01 can move by δ_B/sin θ")

	// A ∪ B = A + B − A ∩ B, with B's corners placed exactly through its
	// stored basis.
	basisOf := rotation.Basis()
	place := func(u, v float64) [2]*big.Rat {
		x := ratAddMul(prismRatOf(t, u), prismRatOf(t, basisOf.EX.X), prismRatOf(t, v), prismRatOf(t, basisOf.EY.X))
		y := ratAddMul(prismRatOf(t, u), prismRatOf(t, basisOf.EX.Y), prismRatOf(t, v), prismRatOf(t, basisOf.EY.Y))
		return [2]*big.Rat{x, y}
	}
	square := [][2]*big.Rat{ratXY(0, 0), ratXY(10, 0), ratXY(10, 10), ratXY(0, 10)}
	band := [][2]*big.Rat{place(-5, 9.9), place(15, 9.9), place(15, 11.9), place(-5, 11.9)}
	area := new(big.Rat).Add(ratPolygonArea(square), ratPolygonArea(band))
	area.Sub(area, ratPolygonArea(ratConvexClip(band, square)))
	truth := new(big.Rat).Mul(area, prismRatOf(t, 10))
	body, err := evalPrismContext(t.Context(), New(), 1, result, freeform.NewFreeformWork())
	require.NoError(t, err)
	residual := prismExactResidual(t, body.volume.Value.Base(), truth)
	require.LessOrEqualf(t, residual, body.volume.Bound.Base(),
		"the published volume bound %g must contain the true error %g", body.volume.Bound.Base(), residual)
}

// ratXY lifts an exact float point.
func ratXY(u, v float64) [2]*big.Rat {
	return [2]*big.Rat{new(big.Rat).SetFloat64(u), new(big.Rat).SetFloat64(v)}
}

// ratAddMul is a·b + c·d over rationals.
func ratAddMul(a, b, c, d *big.Rat) *big.Rat {
	x := new(big.Rat).Mul(a, b)
	return x.Add(x, new(big.Rat).Mul(c, d))
}

// ratPolygonArea is a simple polygon's signed area, exactly.
func ratPolygonArea(poly [][2]*big.Rat) *big.Rat {
	total := new(big.Rat)
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		total.Add(total, new(big.Rat).Sub(new(big.Rat).Mul(p[0], q[1]), new(big.Rat).Mul(q[0], p[1])))
	}
	return total.Mul(total, big.NewRat(1, 2))
}

// ratConvexClip clips poly by the counter-clockwise convex polygon clip
// (Sutherland–Hodgman), exactly: the test's own oracle for a convex overlap.
func ratConvexClip(poly, clip [][2]*big.Rat) [][2]*big.Rat {
	side := func(a, b, p [2]*big.Rat) *big.Rat {
		x := new(big.Rat).Mul(new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(p[1], a[1]))
		return x.Sub(x, new(big.Rat).Mul(new(big.Rat).Sub(b[1], a[1]), new(big.Rat).Sub(p[0], a[0])))
	}
	out := poly
	for i := range clip {
		a, b := clip[i], clip[(i+1)%len(clip)]
		in := out
		out = nil
		for j := range in {
			p, q := in[j], in[(j+1)%len(in)]
			sp, sq := side(a, b, p), side(a, b, q)
			if sp.Sign() >= 0 {
				out = append(out, p)
			}
			if sp.Sign()*sq.Sign() < 0 {
				t := new(big.Rat).Quo(sp, new(big.Rat).Sub(sp, sq))
				out = append(out, [2]*big.Rat{
					new(big.Rat).Add(p[0], new(big.Rat).Mul(t, new(big.Rat).Sub(q[0], p[0]))),
					new(big.Rat).Add(p[1], new(big.Rat).Mul(t, new(big.Rat).Sub(q[1], p[1]))),
				})
			}
		}
	}
	return out
}

// TestPrismUnionDisplacedSourceSplitChargesTheCrossing covers a chained union
// whose second re-expression is identity. The first union carries its own
// section displacement from re-expressing a containing operand. A shallow
// crossing (slope 0.01) in the second union can move by that displacement
// divided by the crossing sine, and A6 charges it: the union builds with a
// sectionDelta of at least δ_A/0.01. Shown to fail with
// prismSceneDelta.Merged's crossing term deleted.
func TestPrismUnionDisplacedSourceSplitChargesTheCrossing(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	inner := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(2, 2, 8, 8)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	const shift = 1e8
	translation, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	containing := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(-shift, 0, 10-shift, 10)},
		frame:   frame, z0: 0, z1: 10, xform: translation,
	}
	first, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, &Body{payload: inner}, &Body{payload: containing})
	require.NoError(t, err)
	require.True(t, ok, "the containing first union must resolve analytically")
	require.Positive(t, first.sectionDelta, "the nonidentity first union must carry its re-expression displacement")

	shallow := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: -5, V: 9.9}, End: Point2{U: 15, V: 10.1}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 15, V: 10.1}, End: Point2{U: 15, V: 12.1}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 15, V: 12.1}, End: Point2{U: -5, V: 11.9}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: -5, V: 11.9}, End: Point2{U: -5, V: 9.9}, TStart: 0, TEnd: 1},
		}}},
		frame: first.frame, z0: first.z0, z1: first.z1, xform: first.xform,
	}
	reexpression, err := prismcells.NewReexpression(prismPlacementOf(first), prismPlacementOf(shallow))
	require.NoError(t, err)
	require.True(t, reexpression.Identity, "the second union must take the identity re-expression path")

	scene, _, _, err := buildPrismScene(proofbound.NewWorkBudget(t.Context()), first, shallow, reexpression)
	require.NoError(t, err)
	profiles, err := prismProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)
	split, err := prismProfilesHaveSplitBoundary(proofbound.NewWorkBudget(t.Context()), profiles)
	require.NoError(t, err)
	require.True(t, split, "the shallow crossing must create a trimmed edge")

	second, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, &Body{payload: first}, &Body{payload: shallow})
	require.NoError(t, err)
	require.True(t, ok, "A6 charges the shallow crossing instead of rerouting it")
	require.GreaterOrEqual(t, second.sectionDelta, first.sectionDelta/0.01,
		"the crossing at sin θ < 0.01 can move by δ_A/sin θ")
}

// TestTryPrismBooleanSingleOpenSegmentIsUnresolvedForCutAndIntersect covers
// Cut and Intersect against a pair whose G1-G5 all pass but whose "loop" is a
// single open LineSeg (synthLineLoop's own shape — a placeholder G1-G4 never
// looks past the segment kind for): the private scene bounds no closed region
// at all, so §4.2's clean-nesting search finds no candidate profile and both
// ops fall through unresolved (§4.4), not admitted, exactly like any other
// topology this increment's resolution does not cover.
func TestTryPrismBooleanSingleOpenSegmentIsUnresolvedForCutAndIntersect(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: pp}

	for _, op := range []meshbool.OperationKind{meshbool.OpCut, meshbool.OpIntersect} {
		_, ok, err := tryPrismBoolean(t.Context(), op, a, b)
		require.NoError(t, err)
		require.False(t, ok)
	}
}

func TestPrismUnionArrangementCapRejectsLargeLineOnlyScene(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthDenseRectLoop(prismMaxArrangementSegments/8 + 1)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: pp}

	_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, a, b)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, ok)
}

func TestPrismUnionPreservesEndDisplacements(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame,
		z0:      0,
		z1:      10,
		z0Delta: 0.125,
		z1Delta: 0.75,
		xform:   r3.Identity(),
	}
	pb := pa
	pb.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
	pb.z0Delta = 0.5
	pb.z1Delta = 0.25

	result, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, &Body{payload: pa}, &Body{payload: pb})
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 0.5, result.z0Delta)
	require.Equal(t, 0.75, result.z1Delta)
}

func TestPrismProfilesContextWaitsForArrangementAfterCancellation(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	started := make(chan struct{})
	release := make(chan struct{})
	finished := make(chan struct{})
	profiles := func() []*sketch.Profile {
		close(started)
		<-release
		close(finished)
		return []*sketch.Profile{}
	}
	result := make(chan error, 1)
	go func() {
		_, err := prismProfilesContext(ctx, profiles)
		result <- err
	}()

	<-started
	cancel()
	select {
	case err := <-result:
		t.Fatalf("prismProfilesContext returned before arrangement finished: %v", err)
	case <-time.After(100 * time.Millisecond):
	}

	close(release)
	<-finished
	require.ErrorIs(t, <-result, context.Canceled)
}

// --- fu141 (docs/prism-boolean-design.md §9's RB9): the merged union loop's
// own recorded coordinates must join at every junction.

// requireMergedLoopSegmentsJoin is T1's own invariant, reused by T3: whenever
// a merge resolves (tryPrismBoolean's ok == true), every consecutive pair of
// WHOLE LineSeg segments in the merged Outer loop joins bit-exactly — the
// same property falsifyLoopJoins now proves before resolvePrismUnion returns.
// It reads the recorded coordinates directly (segs[i].End == segs[i+1].Start,
// wrap included), the same comparison loopJoinPointsAgree makes for a
// same-source pair.
func requireMergedLoopSegmentsJoin(t *testing.T, segs []CurveSegment) {
	t.Helper()
	n := len(segs)
	for i := range n {
		j := (i + 1) % n
		li, oki := segs[i].(LineSeg)
		lj, okj := segs[j].(LineSeg)
		if !oki || !okj || li.TStart != 0 || li.TEnd != 1 || lj.TStart != 0 || lj.TEnd != 1 {
			continue
		}
		require.Equal(t, li.End, lj.Start, "merged segment %d end must bit-exactly equal segment %d start", i, j)
	}
}

// synthGapRectLoop is synthRectLoop with one deliberate defect: seg 0's End
// and seg 1's Start name the SAME corner but differ by gap — a mismatch
// authored directly on the record, never computed by any merge. sketch's own
// arrangement still accepts the shape as one region on its proximity
// threshold (docs/sketch-seam-design.md), so the defect survives all the way
// to resolvePrismUnion's own recorded chain.
func synthGapRectLoop(u0, v0, u1, v1, gap float64) LoopRecord {
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: u0, V: v0}, End: Point2{U: u1, V: v0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1 + gap, V: v0}, End: Point2{U: u1, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1, V: v1}, End: Point2{U: u0, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u0, V: v1}, End: Point2{U: u0, V: v0}, TStart: 0, TEnd: 1},
	}}
}

// TestPrismUnionMergedLoopJunctionsClose is RB9's own guard, pinned directly
// against resolvePrismUnion's wiring rather than against a live cut-fragment
// fixture.
//
// The one LIVE-reachable way to produce this defect is an operand recorded as
// Partial cut fragments (drawn as overshooting lines sketch trims at both
// ends): buildPrismScene's walkOf interpolates each surviving fragment's
// walked endpoint independently, from its own entity's Start/End and T, so
// two fragments naming the same corner can round to bit-different floats
// before the merge ever sees them. That is fu157's class (#158): welding the
// private scene's own recorded junctions closes exactly that fixture's loop,
// so a test keyed on it would silently start asserting the wrong thing once
// that PR lands (apitest/prism_boolean_test.go's
// TestPrismUnionCutFragmentOperandRefusesNonClosingMerge is that fixture, and
// carries the same risk explicitly).
//
// This test instead authors the defect directly on a synthetic operand
// (bypassing RecordProfile's own seam checks, the same way the G1-G6 gate
// tests above do) — a corner sketch's own proximity threshold still accepts
// as one region, but whose recorded coordinates no merge computed and so
// never rounds into agreement. Nothing here depends on how any operand's
// Partial fragments get built, so the refusal half stays correct whichever
// PR lands first.
func TestPrismUnionMergedLoopJunctionsClose(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	const gap = 1e-9
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthGapRectLoop(0, 0, 10, 10, gap)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	pb := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(2, 2, 4, 4)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pa}
	b := &Body{payload: pb}

	_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, a, b)
	require.False(t, ok)
	require.ErrorIs(t, err, ErrUnrecordableProfile)
	require.Contains(t, err.Error(), "does not close")
	require.Contains(t, err.Error(), "10.000000001")

	// The invariant half, which must survive fu157's weld regardless of
	// landing order: a pair with no authored defect resolves, and its merged
	// loop DOES join bit-exactly at every whole junction.
	clean := pa
	clean.profile = ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)}
	res, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, &Body{payload: clean}, b)
	require.NoError(t, err)
	require.True(t, ok)
	requireMergedLoopSegmentsJoin(t, res.profile.Outer.Segments)
}

// TestPrismUnionCleanOperandsMergedLoopClosesExactly is the control: the
// ordinary class this guard must never fire on. Two operands whose own
// records carry no Partial fragment (drawn as four shared-point lines each,
// the live-reachable whole-edge shape) merge into a loop that closes
// bit-exactly, with no error and no displacement.
func TestPrismUnionCleanOperandsMergedLoopClosesExactly(t *testing.T) {
	t.Parallel()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	b := internalBoxBody(t, doc, 5, 5, 15, 15, 5)
	res, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, a, b)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, res.profile.Outer.Segments, 8)
	requireMergedLoopSegmentsJoin(t, res.profile.Outer.Segments)

	doc2 := New()
	c := internalBoxBody(t, doc2, 0, 0, 10, 10, 5)
	d := internalBoxBody(t, doc2, 2, 2, 4, 4, 5)
	res2, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, c, d)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, res2.profile.Outer.Segments, 4)
	requireMergedLoopSegmentsJoin(t, res2.profile.Outer.Segments)
	require.Zero(t, res2.sectionDelta)
}

// This is task fu143's own test suite: §7's fourth displacement source,
// δ_walk — a consumed source segment's own walked endpoint, computed rather
// than read off the record whenever that segment's recorded range narrows
// its entity's own natural domain.

// prismSplitLeftCellBody builds the rectangle [1,11]×[0,10] split by a fixed
// line through (5,-2)-(5,14) and extrudes the LEFT cell h mm — task fu143's
// own fixture (its investigation section 1). The left cell's bottom and top
// walls are Partial fragments of the rectangle's own bottom/top lines,
// recorded with the entity's full [1,11] Start/End and a narrowed
// TStart/TEnd denoting the split at u≈5, so the corner where they meet the
// right wall is a coordinate this evaluator's own scene construction
// computes, not one either wall's record states outright.
//
// Every one of those corners must come out the SAME coordinate on every host,
// or the loop buildPrismScene hands back no longer closes and RecordProfile
// refuses it before the charge under test is ever reached. Each corner is
// reached twice by separate arithmetic — once along each of the two walls that
// meet there, through lerp2's start + t·(end − start) — and a host that
// contracts that expression into a fused multiply-add rounds it once where a
// host without the fusion rounds it twice. So this fixture states the split
// line's own endpoints as (5,-2)-(5,14): its span is 16 and the rectangle's
// walls cut it at v = 0 and v = 10, making the recorded parameters 1/8 and
// 3/4 exactly. Every product and sum lerp2 then forms is representable, so
// both spellings return the identical corner and neither rounds at all. The
// bottom and top walls' own parameters, 0.4 and 0.6, are not exact binary
// fractions, but the value 1 + 0.4·10 sits a quarter of an ulp above 5 under
// either spelling, well inside the half ulp that rounds it back to 5.
//
// prismFixtureHeight is every fixture below's own sweep height: every test in
// this suite compares two operands and needs no other value.
const prismFixtureHeight = 10.0

func prismSplitLeftCellBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(1, 0, 11, 10)
	s.Fix(r.A)
	lo := s.CreatePoint(5, -2)
	hi := s.CreatePoint(5, 14)
	s.Fix(lo)
	s.Fix(hi)
	s.CreateLine(lo, hi)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var left *sketch.Profile
	for _, p := range s.Profiles() {
		minU, maxU := math.Inf(1), math.Inf(-1)
		for _, e := range p.Outer {
			for _, pt := range e.Polyline {
				minU = math.Min(minU, pt[0])
				maxU = math.Max(maxU, pt[0])
			}
		}
		if minU == 1 && maxU <= 5.0000001 {
			left = p
		}
	}
	require.NotNil(t, left, "the split rectangle's left cell must exist")

	body, err := doc.Extrude(s, left, Distance{D: units.Millimeters(prismFixtureHeight), Dir: Along})
	require.NoError(t, err)
	prismRequireSplitWallRange(t, body.payload.(prismPayload).profile)
	return body
}

// prismRequireSplitWallRange pins the recorded range of the split-left-cell
// fixture's own right wall to the two exact binary fractions its doc comment
// derives every corner's host independence from. A host whose sketch reports
// any other parameter must fail here, naming the fixture, rather than at
// whichever consumer first walks that parameter to a corner its neighbour
// does not share.
func prismRequireSplitWallRange(t *testing.T, p ProfileRecord) {
	t.Helper()
	for _, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		if !ok || ls.Start != (Point2{U: 5, V: -2}) || ls.End != (Point2{U: 5, V: 14}) {
			continue
		}
		require.Equal(t, 0.125, ls.TStart)
		require.Equal(t, 0.75, ls.TEnd)
		return
	}
	t.Fatal("the split-left-cell fixture's own right wall was not found")
}

// prismRectBody extrudes the axis-aligned rectangle (x0, y0)-(x1, y1)
// prismFixtureHeight mm — every segment WHOLE, TStart=0, TEnd=1, the
// every-caller-drawn-body shape.
func prismRectBody(t *testing.T, doc *Document, x0, y0, x1, y1 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(prismFixtureHeight), Dir: Along})
	require.NoError(t, err)
	return body
}

// prismRatOf lifts a float64 to the exact rational it is; a test fixture
// coordinate is always finite, so a non-finite input is a fixture bug.
func prismRatOf(t *testing.T, f float64) *big.Rat {
	t.Helper()
	r := new(big.Rat)
	require.NotNil(t, r.SetFloat64(f), "fixture coordinate must be finite")
	return r
}

// prismExactLineOnlyArea is the exact signed area of a line-only recorded
// outer loop (no holes), integrated over each segment's own RECORDED range
// via ratLerp — the same Green's-theorem boundary term moments.go
// accumulates, with no rounding at all. Every fixture below is a plain
// rectangle or a footprint difference of rectangles, so the outer loop alone
// is always line-only.
func prismExactLineOnlyArea(t *testing.T, p ProfileRecord) *big.Rat {
	t.Helper()
	total := new(big.Rat)
	for _, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		require.True(t, ok, "fixture must be line-only: %T", seg)
		u0 := ratLerp(ls.Start.U, ls.End.U, ls.TStart)
		v0 := ratLerp(ls.Start.V, ls.End.V, ls.TStart)
		u1 := ratLerp(ls.Start.U, ls.End.U, ls.TEnd)
		v1 := ratLerp(ls.Start.V, ls.End.V, ls.TEnd)
		term := new(big.Rat).Sub(new(big.Rat).Mul(u0, v1), new(big.Rat).Mul(u1, v0))
		total.Add(total, new(big.Rat).Mul(term, big.NewRat(1, 2)))
	}
	return total
}

// prismExactResidual is |reported − truth| taken entirely over rationals and
// rounded UP into a float64 — differencing against a float64 conversion of
// truth would fold half an ulp of the reported magnitude into the answer,
// which could flatter a bound that failed to contain the true error
// (apitest/prism_boolean_displacement_test.go's exactResidual documents the same
// point for the external test suite; this is its internal-package twin).
func prismExactResidual(t *testing.T, reported float64, truth *big.Rat) float64 {
	t.Helper()
	d := new(big.Rat).Sub(prismRatOf(t, reported), truth)
	d.Abs(d)
	f, exact := d.Float64()
	if !exact {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// prismBottomWallTEnd locates the split-left-cell fixture's own bottom wall
// (the full rectangle's [1,11] bottom line, Start=(1,0), End=(11,0)) and
// returns its recorded TEnd — the fraction §7's δ_walk charges a walked
// endpoint against.
func prismBottomWallTEnd(t *testing.T, p ProfileRecord) float64 {
	t.Helper()
	for _, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		if !ok {
			continue
		}
		if ls.Start == (Point2{U: 1, V: 0}) && ls.End == (Point2{U: 11, V: 0}) {
			return ls.TEnd
		}
	}
	t.Fatal("the split-left-cell fixture's own bottom wall was not found")
	return 0
}

// TestPrismSplitLeftCellFixtureWalksHostIndependently proves on THIS host the
// property the fixture's own doc comment derives, and which only another host
// could otherwise disprove: every corner the fixture's walls walk to is the
// same coordinate whether or not the host fuses lerp2's multiply and add.
// Each segment's endpoints are computed both ways — prismLerpSplit and
// prismLerpFused, this file's owner of lerp2's two readings — and compared
// exactly, so a future edit that reintroduces a parameter needing a rounding
// decision fails here rather than on whichever host makes that decision
// differently.
func TestPrismSplitLeftCellFixtureWalksHostIndependently(t *testing.T) {
	t.Parallel()
	doc := New()
	p := prismSplitLeftCellBody(t, doc).payload.(prismPayload).profile
	for i, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		require.Truef(t, ok, "the fixture is line-only: segment %d is a %T", i, seg)
		for _, at := range []float64{ls.TStart, ls.TEnd} {
			require.Equalf(t, prismLerpSplit(ls.Start, ls.End, at), prismLerpFused(ls.Start, ls.End, at),
				"segment %d walks to a different endpoint at t=%v when the host fuses the multiply-add", i, at)
		}
	}
}

// TestPrismUnionTrimmedSourceSegmentChargesItsWalkedEndpoint is fu143's own
// Union reproduction: operand A carries a Partial bottom-wall fragment of a
// wider rectangle line, operand B sits strictly inside A's own footprint, and
// the union must charge A's own walk displacement into its published
// sectionDelta and volume bound rather than publish Exact/zero over a section
// this union's own scene construction moved.
func TestPrismUnionTrimmedSourceSegmentChargesItsWalkedEndpoint(t *testing.T) {
	t.Parallel()
	const h = 10.0
	doc := New()
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)

	// Pin the fixture: if sketch's own cut parameter for this split ever
	// changes, this fixture must fail loudly rather than silently stop
	// testing anything.
	require.Equal(t, 0.4000000000000000222, prismBottomWallTEnd(t, pa.profile))

	b := prismRectBody(t, doc, 2, 2, 4, 8)

	u, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	pu, ok := u.payload.(prismPayload)
	require.True(t, ok, "the analytic reduction must own this pair")

	// The exact distance between the recorded corner (the right wall's own
	// u = 5, exactly) and the denoted corner the bottom wall's own TEnd
	// names (1 + TEnd·10), computed over math/big.Rat rather than typed as a
	// literal.
	tEnd := prismBottomWallTEnd(t, pa.profile)
	denotedCorner := new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(prismRatOf(t, tEnd), big.NewRat(10, 1)))
	minDelta := prismExactResidual(t, 5, denotedCorner)
	require.Positive(t, minDelta, "the fixture must actually disagree with itself, or it proves nothing")
	require.GreaterOrEqual(t, pu.sectionDelta, minDelta,
		"the union's own sectionDelta must charge at least the corner disagreement its own operand carries")

	// B sits strictly inside A's own footprint, so the union the two records
	// DENOTE is A's own recorded section swept h mm.
	truth := new(big.Rat).Mul(prismExactLineOnlyArea(t, pa.profile), prismRatOf(t, h))
	uv, err := u.Volume()
	require.NoError(t, err)
	residual := prismExactResidual(t, uv.Value.Base(), truth)
	require.Equal(t, Approximate, uv.Exactness)
	require.LessOrEqualf(t, residual, uv.Bound.Base(),
		"the published volume bound %g must contain the true error %g", uv.Bound.Base(), residual)
}

// TestPrismUnionChargesEachWalkExactlyOnce pins §7's composition
// δ = up(max(up(δ_A + δ_walkA), up(δ_B + δ_walkB + δ_reexpress)) + δ_cut) on
// the fixture where every term but one is zero: operand A owes a walk charge,
// operand B is drawn whole, the re-expression is the identity, and B sits
// strictly inside A so the merge cuts nothing. The published displacement must
// therefore be A's own walk charge and nothing more — charged ONCE, on A's own
// side of the max. A composition that also added a separate walk term outside
// the max would publish about twice this value and fail here.
func TestPrismUnionChargesEachWalkExactlyOnce(t *testing.T) {
	t.Parallel()
	doc := New()
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)
	require.Zero(t, pa.sectionDelta, "δ_A must be zero, or the fixture cannot isolate the walk charge")

	b := prismRectBody(t, doc, 2, 2, 4, 8) // strictly inside A's own footprint
	pb := b.payload.(prismPayload)
	require.Zero(t, pb.sectionDelta, "δ_B must be zero")

	reexpression, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, reexpression.Identity, "both operands share one frame with no placement between them")
	require.Zero(t, reexpression.Delta, "δ_reexpress must be zero")

	_, _, sceneDelta, err := buildPrismScene(proofbound.NewWorkBudget(t.Context()), pa, pb, reexpression)
	require.NoError(t, err)
	require.Positive(t, sceneDelta.A, "operand A's own trimmed walls must carry a walk charge")
	require.Zero(t, sceneDelta.B, "operand B is drawn whole, so δ_walkB is zero")

	u, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	pu, ok := u.payload.(prismPayload)
	require.True(t, ok, "the analytic reduction must own this pair")
	// §7's formula, term by term, with δ_cut = 0 because the merge cuts
	// nothing: every walk charge sits inside its own operand's fold.
	const cutDelta = 0.0
	want := proofbound.AbsSumUpper(
		max(
			proofbound.AbsSumUpper(pa.sectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(pb.sectionDelta, sceneDelta.B, reexpression.Delta),
		),
		cutDelta,
	)
	require.Equal(t, want, pu.sectionDelta,
		"with every other term zero the published displacement is A's own walk charge, folded in once")
	require.Less(t, pu.sectionDelta, proofbound.AbsSumUpper(want, sceneDelta.A),
		"a second, separate walk charge outside the max would roughly double the published displacement")
}

// TestPrismCutTrimmedTargetChargesItsWalkedEndpoint is fu143's Cut
// reproduction: the clean-nesting path's own target carries the same Partial
// bottom-wall fragment, and the tool is fully nested inside it.
func TestPrismCutTrimmedTargetChargesItsWalkedEndpoint(t *testing.T) {
	t.Parallel()
	const h = 10.0
	doc := New()
	target := prismSplitLeftCellBody(t, doc)
	ptarget := target.payload.(prismPayload)
	tool := prismRectBody(t, doc, 2, 2, 4, 8)

	got, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	pg, ok := got.payload.(prismPayload)
	require.True(t, ok, "the clean-nesting cut must build analytically")
	require.Positive(t, pg.sectionDelta, "the target's own walk charge must reach the cut's result")

	// Truth: the target's own denoted section, minus the tool's own 2×6
	// footprint (fully inside it), swept h mm.
	truthArea := new(big.Rat).Sub(prismExactLineOnlyArea(t, ptarget.profile), big.NewRat(12, 1))
	truth := new(big.Rat).Mul(truthArea, prismRatOf(t, h))
	gv, err := got.Volume()
	require.NoError(t, err)
	residual := prismExactResidual(t, gv.Value.Base(), truth)
	require.Equal(t, Approximate, gv.Exactness)
	require.LessOrEqualf(t, residual, gv.Bound.Base(),
		"the published volume bound %g must contain the true error %g", gv.Bound.Base(), residual)
}

// TestPrismIntersectTrimmedOperandChargesItsWalkedEndpoint is fu143's
// Intersect reproduction: a big outer box fully contains the split-left-cell
// fixture, so the nested operand's own walk charge must reach the result.
func TestPrismIntersectTrimmedOperandChargesItsWalkedEndpoint(t *testing.T) {
	t.Parallel()
	const h = 10.0
	doc := New()
	outer := prismRectBody(t, doc, 0, -1, 12, 11)
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)

	got, err := Intersect(t.Context(), outer, a)
	require.NoError(t, err)
	pg, ok := got.payload.(prismPayload)
	require.True(t, ok, "the clean-nesting intersect must build analytically")
	require.Positive(t, pg.sectionDelta, "the nested operand's own walk charge must reach the result")

	truth := new(big.Rat).Mul(prismExactLineOnlyArea(t, pa.profile), prismRatOf(t, h))
	gv, err := got.Volume()
	require.NoError(t, err)
	residual := prismExactResidual(t, gv.Value.Base(), truth)
	require.Equal(t, Approximate, gv.Exactness)
	require.LessOrEqualf(t, residual, gv.Bound.Base(),
		"the published volume bound %g must contain the true error %g", gv.Bound.Base(), residual)
}

// TestPrismBooleanWholeSourceSegmentsChargeNothing is the guard that §7's new
// term does not quietly retire the decidable zero case: two whole-segment
// boxes, the same pair TestPrismUnionWholeEdgeMergeStaysExact already covers
// from the outside, must publish a sectionDelta of exactly 0.0 on all three
// ops.
func TestPrismBooleanWholeSourceSegmentsChargeNothing(t *testing.T) {
	t.Parallel()
	t.Run("Union", func(t *testing.T) {
		doc := New()
		a := prismRectBody(t, doc, 0, 0, 10, 10)
		b := prismRectBody(t, doc, 2, 2, 8, 8)
		u, err := Union(t.Context(), a, b)
		require.NoError(t, err)
		pu, ok := u.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, 0.0, pu.sectionDelta)
	})

	t.Run("Cut", func(t *testing.T) {
		doc := New()
		target := prismRectBody(t, doc, 0, 0, 10, 10)
		tool := prismRectBody(t, doc, 2, 2, 8, 8)
		got, err := Cut(t.Context(), target, tool)
		require.NoError(t, err)
		pg, ok := got.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, 0.0, pg.sectionDelta)
	})

	t.Run("Intersect", func(t *testing.T) {
		doc := New()
		outer := prismRectBody(t, doc, 0, 0, 10, 10)
		inner := prismRectBody(t, doc, 2, 2, 8, 8)
		got, err := Intersect(t.Context(), outer, inner)
		require.NoError(t, err)
		pg, ok := got.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, 0.0, pg.sectionDelta)
	})
}

// TestWalkChargeOf is a table test over walkChargeOf itself: 0 for a whole
// segment of every admitted kind, a positive finite value for a trimmed one,
// and +Inf for a non-finite coordinate — never 0 for an unknown or uncertain
// case (this task's own risk: an absent bound must never read as a small
// one). The trimmed circular rows exercise the function directly; through the
// boolean they are unreachable, which
// TestPrismCircularWalkChargeImpliesRefusal pins.
func TestWalkChargeOf(t *testing.T) {
	t.Parallel()
	line := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}}
	arc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 5, V: 0},
		End:    Point2{U: 0, V: 5},
	}
	circle := CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(5), CCW: true}

	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{"whole LineSeg", func() CurveSegment { s := line; s.TStart, s.TEnd = 0, 1; return s }()},
		{"whole reversed LineSeg", func() CurveSegment { s := line; s.TStart, s.TEnd = 1, 0; return s }()},
		{"whole ArcSeg", func() CurveSegment { s := arc; s.TStart, s.TEnd = 0, 1; return s }()},
		{"whole reversed ArcSeg", func() CurveSegment { s := arc; s.TStart, s.TEnd = 1, 0; return s }()},
		{"whole CircleSeg", func() CurveSegment { s := circle; s.TStart, s.TEnd = 0, 1; return s }()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := boundarywalk.WalkOf(tc.seg, nil)
			require.NoError(t, err)
			got, err := walkChargeOf(tc.seg, w)
			require.NoError(t, err)
			require.Equal(t, 0.0, got)
		})
	}

	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{"trimmed LineSeg", func() CurveSegment { s := line; s.TStart, s.TEnd = 0, 0.4; return s }()},
		{"trimmed ArcSeg", func() CurveSegment { s := arc; s.TStart, s.TEnd = 0, 0.4; return s }()},
		{"trimmed CircleSeg", func() CurveSegment { s := circle; s.TStart, s.TEnd = 0, 0.4; return s }()},
		// One ulp short of the natural bound: the walk's own closed-ness
		// tolerance calls this circle closed, and the charge must still be
		// positive — wholeness is the recorded range, never that tolerance.
		{"CircleSeg one ulp short of whole", func() CurveSegment {
			s := circle
			s.TStart, s.TEnd = 0, math.Nextafter(1, 0)
			return s
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := boundarywalk.WalkOf(tc.seg, nil)
			require.NoError(t, err)
			got, err := walkChargeOf(tc.seg, w)
			require.NoError(t, err)
			require.Positive(t, got)
			require.False(t, math.IsInf(got, 0))
		})
	}

	t.Run("non-finite coordinate answers +Inf", func(t *testing.T) {
		bad := LineSeg{Start: Point2{U: math.NaN(), V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 0.4}
		w, err := boundarywalk.WalkOf(bad, nil)
		require.NoError(t, err)
		got, err := walkChargeOf(bad, w)
		require.NoError(t, err)
		require.True(t, math.IsInf(got, 1), "an absent bound must never read as a small one")
	})
}

// TestPrismCircularWalkChargeImpliesRefusal mechanises the reach §7 states for
// δ_walk: a positive charge is only ever computed over a trimmed LineSeg,
// because every circular carrier walkChargeOf could charge is one
// prismProfileHasTrimmedCircularSource refuses before buildPrismScene runs
// (§4.1). Over the circular ranges either side of that boundary, the charge
// being positive and the refusal firing must be the SAME condition — so a
// circular carrier admitted into a scene always charges zero, and the two
// answers cannot drift apart into a silently under-charged bound.
func TestPrismCircularWalkChargeImpliesRefusal(t *testing.T) {
	t.Parallel()
	arc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 5, V: 0},
		End:    Point2{U: 0, V: 5},
	}
	circle := CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(5), CCW: true}
	withRange := func(seg CurveSegment, tStart, tEnd float64) CurveSegment {
		switch s := seg.(type) {
		case ArcSeg:
			s.TStart, s.TEnd = tStart, tEnd
			return s
		case CircleSeg:
			// A CircleSeg's CCW flag must agree with its range order
			// (validateSegmentRange), so a reversed range is a CW circle.
			s.TStart, s.TEnd, s.CCW = tStart, tEnd, tStart < tEnd
			return s
		}
		t.Fatalf("unexpected segment kind %T", seg)
		return nil
	}

	for _, base := range []struct {
		kind string
		seg  CurveSegment
	}{{"ArcSeg", arc}, {"CircleSeg", circle}} {
		for _, rng := range []struct {
			name         string
			tStart, tEnd float64
			wantRefusal  bool
		}{
			{name: "whole", tStart: 0, tEnd: 1, wantRefusal: false},
			{name: "whole reversed", tStart: 1, tEnd: 0, wantRefusal: false},
			{name: "trimmed", tStart: 0, tEnd: 0.4, wantRefusal: true},
			{name: "one ulp short of whole", tStart: 0, tEnd: math.Nextafter(1, 0), wantRefusal: true},
		} {
			t.Run(base.kind+"/"+rng.name, func(t *testing.T) {
				seg := withRange(base.seg, rng.tStart, rng.tEnd)
				w, err := boundarywalk.WalkOf(seg, nil)
				require.NoError(t, err)
				charge, err := walkChargeOf(seg, w)
				require.NoError(t, err)

				profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{seg}}}
				refused, err := prismProfileHasTrimmedCircularSource(proofbound.NewWorkBudget(t.Context()), profile)
				require.NoError(t, err)

				require.Equal(t, rng.wantRefusal, refused,
					"the fixture's own premise: this range must be the refusal case it names")
				require.Equal(t, refused, charge > 0,
					"a circular carrier charges exactly when it is refused, so an admitted one charges zero")
				if !refused {
					require.Equal(t, 0.0, charge,
						"a circular carrier that reaches a scene contributes nothing to δ_walk")
				}
			})
		}
	}
}

// prismWalkEndpointResidualSq is the EXACT squared distance between one walked
// endpoint of a LineSeg and the endpoint its record denotes, taken entirely
// over math/big.Rat: lerp2's own float answer against ratLerp's exact one, per
// coordinate, squared and summed. Nothing here rounds, so a charge compared
// against it is compared against the true error and not against a second float
// estimate of it.
func prismWalkEndpointResidualSq(t *testing.T, s LineSeg, walkedU, walkedV, at float64) *big.Rat {
	t.Helper()
	exactU := ratLerp(s.Start.U, s.End.U, at)
	exactV := ratLerp(s.Start.V, s.End.V, at)
	require.NotNil(t, exactU)
	require.NotNil(t, exactV)
	du := new(big.Rat).Sub(prismRatOf(t, walkedU), exactU)
	dv := new(big.Rat).Sub(prismRatOf(t, walkedV), exactV)
	return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
}

// prismLerpFused and prismLerpSplit are this file's single owner of the two
// answers a conforming Go implementation may produce for lerp2's general arm
// start + t·(end − start), and every test here that needs either reading calls
// them rather than spelling the expression again.
//
// The spec (Floating-point operators) lets a target fuse the multiply and the
// add into one rounding, and the gc arm64 backend does exactly that: lerp2's
// general arm compiles to FSUBD followed by FMADDD there, while amd64 rounds
// the product and then the sum. Both helpers reproduce lerp2's natural-bound
// arms verbatim, so they differ from it only where it computes.
//
// prismLerpSplit's explicit float64 conversion is the barrier the spec names —
// it rounds the product to float64 precision and so forbids the fusion that
// would discard that rounding. Writing the sum without it does not state the
// unfused reading at all: the compiler is free to contract that spelling too,
// and on arm64 it does, which would leave a test comparing the fused answer
// against itself.
func prismLerpFused(start, end Point2, at float64) Point2 {
	if p, ok := prismLerpNaturalBound(start, end, at); ok {
		return p
	}
	return Point2{U: math.FMA(at, end.U-start.U, start.U), V: math.FMA(at, end.V-start.V, start.V)}
}

func prismLerpSplit(start, end Point2, at float64) Point2 {
	if p, ok := prismLerpNaturalBound(start, end, at); ok {
		return p
	}
	return Point2{U: start.U + float64(at*(end.U-start.U)), V: start.V + float64(at*(end.V-start.V))}
}

// prismLerpNaturalBound is lerp2's own t=0/t=1 arms, which return the record's
// coordinate verbatim and so round the same way on every target.
func prismLerpNaturalBound(start, end Point2, at float64) (Point2, bool) {
	switch at {
	case 0:
		return start, true
	case 1:
		return end, true
	}
	return Point2{}, false
}

// TestWalkChargeOfCoversLerpCancellation is the cancellation regression for
// §7's δ_walk: the charge a trimmed LineSeg owes must contain the EXACT
// rational residual of the endpoint lerp2 actually walked to, including when
// the carrier's own End − Start cancels and leaves the walked endpoint far
// smaller than the coordinates the rounding happened at.
//
// A Partial line fragment records its source line's full Start/End with a
// narrowed range (recordEdge, seam.go), so this shape is what an extruded
// profile carries whenever a sketch entity is cut near the sketch origin: the
// walked endpoint sits at ~0 while lerp2 rounds at the carrier's magnitude.
// Charging the walked endpoint's own envelope therefore under-charges without
// limit, which the premise assertion below pins per row — a row whose premise
// stops holding is a row that has stopped testing this defect, and it fails
// here rather than passing quietly.
//
// Every row's carrier is deliberately built so its own End − Start is NOT
// exactly representable (oneUlpDown below), because that subtraction is the
// only part of lerp2 whose rounding no target can fuse away. A symmetric
// carrier such as ±1e12 subtracts to an exactly representable difference, and
// a target that fuses the remaining multiply-add — the gc arm64 backend does;
// see prismLerpFused — then walks such a row to its exact endpoint, which
// leaves the row with nothing to discriminate. Every row is therefore also checked at both evaluations
// lerp2 is allowed to produce, not only at the one this target chose, so a
// row's discriminating power is proven here rather than assumed from the
// machine running the test.
//
// Every comparison is exact and squared, so no square root of the residual is
// ever taken and no float rounding can flatter a bound that failed to contain
// the true error.
func TestWalkChargeOfCoversLerpCancellation(t *testing.T) {
	t.Parallel()
	// A parameter one ulp wide about the carrier's midpoint: the walked
	// endpoints all but coincide with the plane origin while the carrier
	// reaches ±1e12.
	const nearHalf = 0.4999999999999999

	// oneUlpDown moves a carrier's End one ulp down, which is what stops the
	// carrier's own End − Start landing on a representable float.
	oneUlpDown := func(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }

	for _, tc := range []struct {
		name string
		seg  LineSeg
		// endpointOnlyUnderCharges says the walked-endpoint envelope alone
		// (survey2d.SegmentWalk.coordUpper, which is what the answer must NOT be
		// charged at) fails to contain this row's own residual.
		endpointOnlyUnderCharges bool
	}{
		{
			name: "cancelling carrier reaching a million kilometres",
			seg: LineSeg{
				Start:  Point2{U: 1e12, V: 0},
				End:    Point2{U: oneUlpDown(-1e12), V: 0},
				TStart: nearHalf,
				TEnd:   math.Nextafter(nearHalf, 1),
			},
			endpointOnlyUnderCharges: true,
		},
		{
			// No extreme coordinate is needed: a plain 200 mm carrier with a
			// 0.4 mm fragment centred on the sketch origin already escapes a
			// charge read off the fragment's own magnitude.
			name: "200 mm carrier, 0.4 mm fragment on the origin",
			seg: LineSeg{
				Start:  Point2{U: -100, V: 0},
				End:    Point2{U: oneUlpDown(100), V: 0},
				TStart: 0.499,
				TEnd:   0.501,
			},
			endpointOnlyUnderCharges: true,
		},
		{
			name: "200 mm carrier, 0.02 mm fragment on the origin",
			seg: LineSeg{
				Start:  Point2{U: -100, V: 0},
				End:    Point2{U: oneUlpDown(100), V: 0},
				TStart: 0.49995,
				TEnd:   0.50005,
			},
			endpointOnlyUnderCharges: true,
		},
		{
			name: "diagonal cancelling carrier moves both coordinates",
			seg: LineSeg{
				Start:  Point2{U: -1e6, V: -1e6},
				End:    Point2{U: oneUlpDown(1e6), V: oneUlpDown(1e6)},
				TStart: 0.4999999999,
				TEnd:   0.5000000001,
			},
			endpointOnlyUnderCharges: true,
		},
		{
			// The ordinary shape the split-left-cell fixture carries: no
			// cancellation, the fragment sits on its own carrier's scale.
			// It must still be covered, and its premise must NOT hold — the
			// answer may not have become a blanket inflation of every row.
			name: "ordinary fragment of a 1..11 carrier",
			seg: LineSeg{
				Start:  Point2{U: 1, V: 0},
				End:    Point2{U: 11, V: 0},
				TStart: 0,
				TEnd:   0.4000000000000000222,
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := boundarywalk.WalkOf(tc.seg, nil)
			require.NoError(t, err)
			charge, err := walkChargeOf(tc.seg, w)
			require.NoError(t, err)
			require.Positive(t, charge)
			require.False(t, math.IsInf(charge, 0))

			chargeSq := new(big.Rat).Mul(prismRatOf(t, charge), prismRatOf(t, charge))
			endpointOnly := proofbound.WalkEndpointAllow(w.CoordUpper)
			endpointOnlySq := new(big.Rat).Mul(prismRatOf(t, endpointOnly), prismRatOf(t, endpointOnly))

			// The three evaluations every row is judged at: the walk this
			// target actually performed, plus both answers lerp2's general arm
			// is allowed to produce. Judging all three is what keeps the row's
			// verdict — coverage AND premise — the same on a fused target as
			// on an unfused one.
			evaluations := [...]string{
				"as this target's own lerp2 evaluated it",
				"fused into one rounding, as the gc arm64 backend compiles it",
				"with the product rounded before the sum, as amd64 compiles it",
			}
			var premiseSeen [len(evaluations)]bool
			for _, end := range []struct {
				what string
				u, v float64
				at   float64
			}{
				{"start", w.StartU, w.StartV, tc.seg.TStart},
				{"end", w.EndU, w.EndV, tc.seg.TEnd},
			} {
				fused := prismLerpFused(tc.seg.Start, tc.seg.End, end.at)
				split := prismLerpSplit(tc.seg.Start, tc.seg.End, end.at)
				require.Truef(t, end.u == fused.U || end.u == split.U,
					"the %s endpoint's u (%g) must be one of the two answers lerp2 may give (fused %g, split %g) — an unmodelled evaluation is a row this table no longer judges",
					end.what, end.u, fused.U, split.U)
				require.Truef(t, end.v == fused.V || end.v == split.V,
					"the %s endpoint's v (%g) must be one of the two answers lerp2 may give (fused %g, split %g) — an unmodelled evaluation is a row this table no longer judges",
					end.what, end.v, fused.V, split.V)

				for i, walked := range [len(evaluations)]Point2{
					{U: end.u, V: end.v},
					fused,
					split,
				} {
					residualSq := prismWalkEndpointResidualSq(t, tc.seg, walked.U, walked.V, end.at)
					require.GreaterOrEqualf(t, chargeSq.Cmp(residualSq), 0,
						"the %s endpoint %s: the charge %g must contain its exact residual (squared residual %s)",
						end.what, evaluations[i], charge, residualSq.FloatString(40))
					if endpointOnlySq.Cmp(residualSq) < 0 {
						premiseSeen[i] = true
					}
				}
			}
			for i, seen := range premiseSeen {
				require.Equalf(t, tc.endpointOnlyUnderCharges, seen,
					"the premise, with the endpoint taken %s: charging the walked endpoint's own envelope (%g) instead of the carrier's must under-charge exactly on the rows this table says it does",
					evaluations[i], endpointOnly)
			}
		})
	}
}

// TestPrismBooleanTrimmedCircularSourceFallsBack is task fu143's own circular
// carrier row: a trimmed ArcSeg source segment (the arrangement's own arc
// through two cos/sin-computed points would move by more than a coordinate
// displacement can state, §4.1) refuses the analytic reduction before the
// scene is even built — ok=false, err=nil, the same silent §3.4 fallback
// every other entry-gate miss uses — rather than publish an under-charged
// bound for it.
func TestPrismBooleanTrimmedCircularSourceFallsBack(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	trimmedArc := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			ArcSeg{
				Center: Point2{U: 5, V: 5},
				Start:  Point2{U: 8, V: 5},
				End:    Point2{U: 5, V: 8},
				TStart: 0, TEnd: 0.4,
			},
			LineSeg{Start: Point2{U: 5, V: 8}, End: Point2{U: 2, V: 8}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 2, V: 8}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 8, V: 5}, TStart: 0, TEnd: 1},
		}}},
		frame: frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	whole := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 20, 20)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	for _, op := range []meshbool.OperationKind{meshbool.OpUnion, meshbool.OpCut, meshbool.OpIntersect} {
		_, ok, err := tryPrismBoolean(t.Context(), op, &Body{payload: trimmedArc}, &Body{payload: whole})
		require.NoError(t, err)
		require.False(t, ok, "%s over a trimmed circular source segment must fall back, never publish zero", op)
	}
}

// TestPrismProfileHasTrimmedCircularSourceReadsTheRecordedRange pins the
// refusal's own criterion on both circular kinds: wholeness is the recorded
// range compared exactly against 0 and 1, never the walk's closed-ness, which
// circularWalk decides within a tolerance of a full turn. A CircleSeg one ulp
// short of its natural bound walks as closed and is a trimmed carrier all the
// same, so the refusal must fire for it.
func TestPrismProfileHasTrimmedCircularSourceReadsTheRecordedRange(t *testing.T) {
	t.Parallel()
	circle := func(tStart, tEnd float64) CircleSeg {
		return CircleSeg{
			Center: Point2{U: 5, V: 5},
			Radius: units.Millimeters(10),
			CCW:    tStart < tEnd,
			TStart: tStart, TEnd: tEnd,
		}
	}
	arc := func(tStart, tEnd float64) ArcSeg {
		return ArcSeg{
			Center: Point2{U: 5, V: 5},
			Start:  Point2{U: 8, V: 5},
			End:    Point2{U: 5, V: 8},
			TStart: tStart, TEnd: tEnd,
		}
	}

	// The two near-whole circles below are the whole point of this test: both
	// are ranges the walk's own tolerance reads as a closed turn, so a refusal
	// that consulted the walk would let them through.
	for _, seg := range []CircleSeg{circle(0, math.Nextafter(1, 0)), circle(math.Nextafter(0, 1), 1)} {
		w, err := boundarywalk.WalkOf(seg, nil)
		require.NoError(t, err)
		require.True(t, w.Closed,
			"fixture [%v, %v] must be one circularWalk's own tolerance calls closed", seg.TStart, seg.TEnd)
	}

	for _, tc := range []struct {
		name    string
		seg     CurveSegment
		trimmed bool
	}{
		{"whole CircleSeg", circle(0, 1), false},
		{"whole reversed CircleSeg", circle(1, 0), false},
		{"CircleSeg one ulp short of 1", circle(0, math.Nextafter(1, 0)), true},
		{"CircleSeg one ulp past 0", circle(math.Nextafter(0, 1), 1), true},
		{"plainly trimmed CircleSeg", circle(0, 0.4), true},
		{"whole ArcSeg", arc(0, 1), false},
		{"whole reversed ArcSeg", arc(1, 0), false},
		{"trimmed ArcSeg", arc(0, 0.4), true},
		{"trimmed LineSeg carries no circular carrier", LineSeg{
			Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 0.4,
		}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{tc.seg}}}
			got, err := prismProfileHasTrimmedCircularSource(proofbound.NewWorkBudget(t.Context()), profile)
			require.NoError(t, err)
			require.Equal(t, tc.trimmed, got)
		})
	}
}

// TestPrismBooleanNearWholeCircleSourceFallsBack is the end-to-end half of the
// same row: a CircleSeg one ulp short of its natural bound must reroute every
// op to the mesh path, while the SAME pair with the bound recorded exactly is
// admitted analytically — so the fallback is caused by the trim itself and not
// by some unrelated miss elsewhere in the gate chain.
func TestPrismBooleanNearWholeCircleSourceFallsBack(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	circleOperand := func(tEnd float64) prismPayload {
		return prismPayload{
			profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{CircleSeg{
				Center: Point2{U: 10, V: 10},
				Radius: units.Millimeters(4),
				CCW:    true,
				TStart: 0, TEnd: tEnd,
			}}}},
			frame: frame, z0: 0, z1: 10, xform: r3.Identity(),
		}
	}
	box := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 20, 20)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	for _, op := range []meshbool.OperationKind{meshbool.OpUnion, meshbool.OpCut, meshbool.OpIntersect} {
		t.Run(op.String(), func(t *testing.T) {
			nearWhole := circleOperand(math.Nextafter(1, 0))
			_, ok, err := tryPrismBoolean(t.Context(), op, &Body{payload: box}, &Body{payload: nearWhole})
			require.NoError(t, err)
			require.False(t, ok, "a circle recorded one ulp short of whole must fall back, never publish zero")
		})
	}

	t.Run("the whole-circle control is admitted", func(t *testing.T) {
		_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpCut, &Body{payload: box}, &Body{payload: circleOperand(1)})
		require.NoError(t, err)
		require.True(t, ok, "the same pair with an exactly whole circle must reach the analytic result")
	})
}

// TestPrismUnionTrimmedSourceSplitBoundaryChargesTheCrossing pins A6 on a
// walk charge alone: the trimmed-source operand A from
// prismSplitLeftCellBody, unioned with a box that crosses its right wall
// square, builds analytically, and its sectionDelta covers A's walk
// charge. Shown to fail with resolvePrismUnion's former
// split-boundary reroute restored.
func TestPrismUnionTrimmedSourceSplitBoundaryChargesTheCrossing(t *testing.T) {
	t.Parallel()
	doc := New()
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)
	require.Zero(t, pa.sectionDelta, "the fixture's own displacement must come from the walk charge alone")

	b := prismRectBody(t, doc, 4, 3, 6, 7) // straddles A's right wall at u=5
	pb := b.payload.(prismPayload)

	reexpression, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, reexpression.Identity, "both operands share one frame with no placement between them")

	scene, _, sceneDelta, err := buildPrismScene(proofbound.NewWorkBudget(t.Context()), pa, pb, reexpression)
	require.NoError(t, err)
	require.Positive(t, sceneDelta.A, "operand A's own trimmed bottom/top walls must carry a walk charge")
	profiles, err := prismProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)
	split, err := prismProfilesHaveSplitBoundary(proofbound.NewWorkBudget(t.Context()), profiles)
	require.NoError(t, err)
	require.True(t, split, "the overlapping box must genuinely split A's own right wall")

	result, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, &Body{payload: pa}, &Body{payload: pb})
	require.NoError(t, err)
	require.True(t, ok, "A6 charges the crossing instead of rerouting it")
	require.GreaterOrEqual(t, result.sectionDelta, sceneDelta.A)
}

// This file is docs/prism-boolean-design.md §14 PR3's own required property
// test (§15): the crossing sub-case's analytic answer measured against the
// mesh path's own answer, on the same randomized pairs. It runs internally so
// the mesh path can be forced directly through evaluateBoolean, rather than
// raced against the analytic dispatch performBoolean already prefers.

// crossingDiscBody extrudes one coplanar disc of radius r centered at
// (cx, 0), swept by e. Two calls with different z-extents are this file's
// own fixture for a genuinely crossing (not nested) coplanar pair whose caps
// stay clear of each other — see TestPrismCrossingDiscPairsMatchMeshAnswer's
// own doc comment for why that clearance matters.
func crossingDiscBody(t *testing.T, doc *Document, cx, r float64, e Extent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, 0)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], e)
	require.NoError(t, err)
	return body
}

// crossingDiscPairFixture is one randomized trial's own parameters, drawn
// once and replayed identically against both the mesh path and the analytic
// path (and against both Cut and Intersect), so the two answers are about
// the exact same geometry.
type crossingDiscPairFixture struct {
	r1, r2, offset, targetH, toolHalf float64
}

// randomCrossingDiscPairFixtures draws n trials from a fixed seed: two radii
// in [5, 20] mm and a center offset in (|r1-r2|, r1+r2), margined by 0.5 mm on
// each side to stay clear of a near-tangent or near-nested crossing, so every
// drawn trial is a genuine two-point crossing this classifier's own scope
// covers. targetH/toolHalf follow this file's own cap-clearance shape.
func randomCrossingDiscPairFixtures(n int) []crossingDiscPairFixture {
	r := rand.New(rand.NewSource(1))
	fixtures := make([]crossingDiscPairFixture, 0, n)
	for len(fixtures) < n {
		r1 := 5 + r.Float64()*15
		r2 := 5 + r.Float64()*15
		lo := math.Abs(r1-r2) + 0.5
		hi := r1 + r2 - 0.5
		if hi <= lo {
			continue
		}
		offset := lo + r.Float64()*(hi-lo)
		targetH := 5 + r.Float64()*10
		toolHalf := targetH*1.5 + r.Float64()*5
		fixtures = append(fixtures, crossingDiscPairFixture{r1: r1, r2: r2, offset: offset, targetH: targetH, toolHalf: toolHalf})
	}
	return fixtures
}

// TestPrismCrossingDiscPairsMatchMeshAnswer is §14 PR3's own required
// property test: over a randomized set of crossing (not coincident-carrier)
// coplanar prism pairs, Cut and Intersect's analytic answer agrees with the
// mesh path's own answer within the mesh path's bound, and the analytic
// bound is the tighter of the two.
//
// Each pair's target sweeps [0, targetH] and its tool sweeps
// [-toolHalf, +toolHalf] with toolHalf > targetH, so the tool's own caps
// clear the target's on both sides — this pair's ONLY coplanar contact is the
// footprints' own genuine crossing, never a coincident cap. That is what lets
// evaluateBoolean below answer an ordinary chorded intersection instead of
// refusing the pair outright the way a coincident-cap pair would (§1's own
// "coplanar contact refuses outright" consequence, still standing for any
// pair outside this design's admitted class).
func TestPrismCrossingDiscPairsMatchMeshAnswer(t *testing.T) {
	t.Parallel()
	for i, fx := range randomCrossingDiscPairFixtures(8) {
		for _, op := range []meshbool.OperationKind{meshbool.OpIntersect, meshbool.OpCut} {
			doc := New()
			target := crossingDiscBody(t, doc, 0, fx.r1, Distance{D: units.Millimeters(fx.targetH), Dir: Along})
			tool := crossingDiscBody(t, doc, fx.offset, fx.r2, Symmetric{D: units.Millimeters(fx.toolHalf)})
			eval, err := evaluateBoolean(t.Context(), op, target, tool)
			require.NoErrorf(t, err, "trial %d op %v: mesh path", i, op)
			meshVol := eval.volume

			doc2 := New()
			target2 := crossingDiscBody(t, doc2, 0, fx.r1, Distance{D: units.Millimeters(fx.targetH), Dir: Along})
			tool2 := crossingDiscBody(t, doc2, fx.offset, fx.r2, Symmetric{D: units.Millimeters(fx.toolHalf)})
			pp, ok, err := tryPrismBoolean(t.Context(), op, target2, tool2)
			require.NoErrorf(t, err, "trial %d op %v: analytic path", i, op)
			require.Truef(t, ok, "trial %d op %v: the crossing classifier must admit this pair", i, op)
			analyticBody, err := evalPrismContext(t.Context(), doc2, doc2.nextProducerID(), pp, freeform.NewFreeformWork())
			require.NoError(t, err)
			analyticVol, err := analyticBody.Volume()
			require.NoError(t, err)

			diff := math.Abs(analyticVol.Value.Base() - meshVol.Value.Base())
			require.LessOrEqualf(t, diff, meshVol.Bound.Base(),
				"trial %d op %v: analytic %v and mesh %v (+-%v) disagree beyond the mesh bound",
				i, op, analyticVol.Value.Base(), meshVol.Value.Base(), meshVol.Bound.Base())
			require.Lessf(t, analyticVol.Bound.Base(), meshVol.Bound.Base(),
				"trial %d op %v: the analytic bound %v must be tighter than the mesh bound %v",
				i, op, analyticVol.Bound.Base(), meshVol.Bound.Base())
		}
	}
}

// This file is prism_overlap.go's own white-box test suite: the admission and
// decline mechanics apitest/prism_overlap_test.go's black-box Verify-level suite does
// not isolate on its own — the exactly-tangent decline, the arrangement cap,
// mid-reading cancellation, and the G1-G4-style regression fallbacks. It
// reuses prism_boolean_internal_test.go's canonicalPrismFrame/synthRectLoop
// synthetic-payload machinery.

// internalPolyPrismBody extrudes a closed polygon (one outer loop, no holes)
// from z=0 to z=h into a real Document, every vertex pinned at its authored
// coordinate — the internal-package twin of apitest/prism_overlap_test.go's
// polyPrismBody, needed here because prismOverlapVolume's per-cell
// measurement calls d.nextProducerID() and requires a live *Document.
func internalPolyPrismBody(t *testing.T, doc *Document, pts [][2]float64, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := doc.Extrude(s, profs[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestPrismOverlapVolumeDeclinesExactlyTangentPair is §4.5's own "what the
// reading declines" paragraph: two coplanar prisms whose sections meet along
// a shared wall but enclose no common area must never publish a zero-volume
// overlap. Whichever internal reason declines it — resolvePrismCrossingCells
// reporting unresolved (the coincident-carrier case its own header names) or
// resolved with an empty selected set — prismOverlapVolume's own contract is
// the same either way: ok=false, err=nil.
func TestPrismOverlapVolumeDeclinesExactlyTangentPair(t *testing.T) {
	t.Parallel()
	doc := New()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	pb := pa
	pb.profile = ProfileRecord{Outer: synthRectLoop(10, 0, 20, 10)}
	a := &Body{doc: doc, payload: pa}
	b := &Body{doc: doc, payload: pb}

	volume, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.False(t, ok, "a shared-wall contact must decline, never publish a zero-volume overlap")
	require.Equal(t, Measurement{}, volume)
}

// TestPrismOverlapVolumeArrangementCapRefuses is §9's RB7: a combined scene
// exceeding prismMaxArrangementSegments refuses with ErrUnsupported before
// s.Profiles runs, wrapped as meshbool.BooleanExpectedUnsupported exactly as
// evaluateAnalyticIntersect's own RB7 is, so measuredInterference reads it as
// interferenceUnsupportedPipeline and Verify reports Suspect rather than
// erroring.
func TestPrismOverlapVolumeArrangementCapRefuses(t *testing.T) {
	t.Parallel()
	doc := New()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthDenseRectLoop(prismMaxArrangementSegments/8 + 1)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{doc: doc, payload: pp}
	b := &Body{doc: doc, payload: pp}

	_, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.False(t, ok)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	expected, isExpected := asExpectedBoolean(err)
	require.True(t, isExpected, "RB7 must wrap as a booleanExpectedError, matching evaluateAnalyticIntersect")
	require.Equal(t, meshbool.BooleanExpectedUnsupported, expected.Kind)
}

// crossTeethPts and crossBarPts build the same two-disjoint-region U-and-bar
// shape as apitest/prism_overlap_test.go's own fixture, restated here so this
// internal-package file needs no cross-package helper.
var (
	crossUPts = [][2]float64{
		{0, 0}, {4, 0}, {4, 8}, {6, 8}, {6, 0}, {10, 0}, {10, 12}, {0, 12},
	}
	crossBarPts = [][2]float64{{-2, 2}, {12, 2}, {12, 5}, {-2, 5}}
)

// TestPrismOverlapVolumeCancellationLeavesDocumentUntouched is §15's
// cancellation row: a context canceled during the per-cell measurement
// returns ctx.Err() unchanged and leaves the document and both operands
// untouched — the reading never calls nextProducerID for a real step, appends a
// Step, retires an operand, or registers a body (§12's Interference row).
func TestPrismOverlapVolumeCancellationLeavesDocumentUntouched(t *testing.T) {
	t.Parallel()
	doc := New()
	u := internalPolyPrismBody(t, doc, crossUPts, 5)
	bar := internalPolyPrismBody(t, doc, crossBarPts, 5)
	beforeBodies := append([]*Body{}, doc.Bodies()...)
	beforeProducer := doc.nextProducer

	ctx := &internalCancelContext{Context: t.Context(), limit: 40}
	_, ok, err := prismOverlapVolume(ctx, u, bar)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, beforeBodies, doc.Bodies())
	require.Equal(t, beforeProducer, doc.nextProducer)
	require.NoError(t, doc.requireLive(u), "the operand must not be retired by a canceled reading")
	require.NoError(t, doc.requireLive(bar))
}

// TestPrismOverlapVolumeRegressionFallbacks is §15's regression row over
// prismOverlapVolume specifically: a non-coplanar pair and a pair carrying a
// free-form segment each take the exact silent-fallback path they take
// through tryPrismBoolean/admitPrismIntersectPair today — this reading
// shares that gate unchanged (Task 1) rather than restating it. A reflected operand whose image crosses
// A is measured, through the same gate and A6's crossing charge.
func TestPrismOverlapVolumeRegressionFallbacks(t *testing.T) {
	t.Parallel()
	doc := New()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	t.Run("non-coplanar pair", func(t *testing.T) {
		shifted, err := r3.Translation(r3.Vec{Z: 3})
		require.NoError(t, err)
		pb := pa
		pb.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
		pb.xform = shifted
		a := &Body{doc: doc, payload: pa}
		b := &Body{doc: doc, payload: pb}
		_, ok, err := prismOverlapVolume(t.Context(), a, b)
		require.NoError(t, err)
		require.False(t, ok, "G3: a non-coplanar pair must never admit")
	})

	t.Run("reflected crossing operand is measured", func(t *testing.T) {
		// B's image across x = 0 is [5, 15]², which crosses A in [5, 10]².
		// A reflection is a nonidentity re-expression, and
		// docs/general-boolean-design.md's A6 charges the square crossings it
		// can move instead of rerouting the pair.
		pb := pa
		pb.profile = ProfileRecord{Outer: synthRectLoop(-15, 5, -5, 15)}
		pb.xform = prismMirrorAcrossX(t, 0)
		a := &Body{doc: doc, payload: pa}
		b := &Body{doc: doc, payload: pb}
		got, ok, err := prismOverlapVolume(t.Context(), a, b)
		require.NoError(t, err)
		require.True(t, ok)
		require.InDelta(t, 250.0, got.Value.Base(), got.Bound.Base())
	})

	t.Run("free-form segment", func(t *testing.T) {
		pb := pa
		pb.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			EllipseSeg{Center: Point2{U: 20, V: 0}, Rx: units.Millimeters(4), Ry: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1},
		}}}
		a := &Body{doc: doc, payload: pa}
		b := &Body{doc: doc, payload: pb}
		_, ok, err := prismOverlapVolume(t.Context(), a, b)
		require.NoError(t, err)
		require.False(t, ok, "G4: a non-analytic (ellipse) segment must never admit")
	})
}

// TestPrismOverlapVolumeMatchesEvalPrismOnASingleCell is a targeted sanity
// check that prismOverlapVolume's per-cell measurement agrees with
// resolveAndBuildPrismIntersectCrossing's own crossing-sub-case answer for a
// pair whose overlap happens to be ONE region: both routes record the same
// single arrangement cell through the same recordEdge/edgeJoin/
// prismcells.CutDelta sequence and the same evalPrism math, so their published
// Value must agree exactly. Bound is not required to match bit for bit — the
// one-cell sum still charges proofbound.ExactSumRound's own accumulated-rounding term
// (§4.5's "The sum" paragraph), which is a legitimate, slightly more
// conservative outward rounding a single-term sum still commits, never a
// tighter one.
func TestPrismOverlapVolumeMatchesEvalPrismOnASingleCell(t *testing.T) {
	t.Parallel()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	b := internalBoxBody(t, doc, 5, 5, 15, 15, 5)

	want, ok, err := evaluateAnalyticIntersect(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok, "the single-region crossing sub-case must resolve")

	got, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, want.volume.Value, got.Value)
	require.Equal(t, want.volume.Exactness, got.Exactness)
	require.GreaterOrEqual(t, got.Bound.Base(), want.volume.Bound.Base(),
		"the sum's own accumulated rounding can only widen the bound, never narrow it")
	require.InDelta(t, 125.0, got.Value.Base(), 1e-9)
	require.Less(t, got.Bound.Base(), 1e-9)
}

// TestPrismOverlapVolumeSoundAgainstIndependentTriangleSum is §15's cell-sum
// soundness row, at the internal-package level: for a three-region comb
// fixture, the published interval must contain the exact area computed
// independently over each tooth as a plain axis-aligned rectangle, never a
// second float computation.
func TestPrismOverlapVolumeSoundAgainstIndependentTriangleSum(t *testing.T) {
	t.Parallel()
	doc := New()
	comb := [][2]float64{
		{0, 0}, {16, 0}, {16, 2}, {15, 2}, {15, 10}, {13, 10}, {13, 2},
		{9, 2}, {9, 10}, {7, 10}, {7, 2}, {3, 2}, {3, 10}, {1, 10}, {1, 2}, {0, 2},
	}
	bar := [][2]float64{{-2, 4}, {18, 4}, {18, 6}, {-2, 6}}
	const h = 3.0
	a := internalPolyPrismBody(t, doc, comb, h)
	b := internalPolyPrismBody(t, doc, bar, h)

	volume, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok)
	const want = 3 * 2 * 2 * h // three teeth, each 2x2 mm^2, times h
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base())
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
	require.Less(t, volume.Bound.Base(), 1e-9)
}

// The facing C shapes enclose a cell neither operand covers: the select-all
// paths decline the pair silently rather than fill it (§4.2, §4.4).
func TestPrismUnionEnclosedVoidIsUnresolved(t *testing.T) {
	doc := New()
	poly := func(pts [][2]float64, h float64) *Body {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		ps := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			ps[i] = s.CreatePoint(p[0], p[1])
			s.Fix(ps[i])
		}
		for i := range ps {
			s.CreateLine(ps[i], ps[(i+1)%len(ps)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
		require.NoError(t, err)
		return body
	}
	a := poly([][2]float64{{0, 0}, {6, 0}, {6, 3}, {3, 3}, {3, 7}, {6, 7}, {6, 10}, {0, 10}}, 1)
	b := poly([][2]float64{{5, -1}, {11, -1}, {11, 11}, {5, 11}, {5, 8}, {8, 8}, {8, 2}, {5, 2}}, 1)
	_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, a, b)
	require.NoError(t, err)
	require.False(t, ok)
	_, ok, err = tryPrismGroupUnion(t.Context(), a, b)
	require.NoError(t, err)
	require.False(t, ok)

	// The same pair under a taller block: A1's slab merge declines it too.
	block := internalBoxBodyAtZ(t, doc, 0.5, 4, 2.5, 6, 1, 1)
	stack, err := Union(t.Context(), a, block)
	require.NoError(t, err)
	_, ok, err = tryStackedUnion(t.Context(), stack, b)
	require.NoError(t, err)
	require.False(t, ok)
}
