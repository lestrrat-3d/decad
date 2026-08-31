package decad

import (
	"errors"
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCapBandMomentCoordUpperCoversOffsetBoundary is the regression for the
// unsound sweptMomentAllow input this fix corrects: bounds.go:407 documents
// coordUpper as "a PROVEN upper bound on |u|, |v| and |z| over the band's own
// material", but capBandMoment used to derive it from the ORIGINAL loop and
// the two axial levels alone — never from the offset cap boundary the band's
// material actually reaches.
//
// A centred, tiny hole (radius 0.01) under a symmetric ±0.5 mm sweep,
// chamfered by a setback (0.9) several times its own radius, reproduces the
// shortfall: chamfering WIDENS a hole by the setback, to radius rho+d = 0.91,
// while the pre-fix coordUpper — max(the original loop's own ~0 envelope,
// the 0.5 mm axial half-sweep) — never reaches past 0.5. This is the
// "genuinely under-covers" shape the audit isolated, distinct from the
// reviewer's off-centre-hole fixture (whose composed bound already enclosed
// through other slack).
//
// delta is set far larger than the record's own tiny contour rounding
// (loopContourDelta here is on the order of 1e-17, negligible against any
// O(1)-scale arithmetic rounding) so the sweptMomentAllow term this
// mechanism composes DOMINATES the band's ordinary arithmetic rounding
// rather than being masked by it — the same isolation
// TestCapBandVolumeBoundTracksTheFluxNotTheCancelledBand uses to show a
// mechanism is unsound on its own, without needing two platforms to
// disagree.
func TestCapBandMomentCoordUpperCoversOffsetBoundary(t *testing.T) {
	const rho, d, half = 0.01, 0.9, 0.5
	loop := LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: Point2{}, Radius: units.Millimeters(rho), CCW: false, TStart: 1, TEnd: 0},
	}}
	work := newFreeformWork()

	capBoundary, err := capLoopBoundary(t.Context(), loop, d)
	require.NoError(t, err)
	require.Len(t, capBoundary.Segments, 1)
	widened, ok := capBoundary.Segments[0].(CircleSeg)
	require.True(t, ok, "a whole-circle hole offsets into a whole circle")
	widenedRadius, err := widened.Radius.In(units.Millimeter)
	require.NoError(t, err)
	require.InDelta(t, rho+d, widenedRadius, 1e-9,
		"the premise: chamfering a hole widens it by the setback")

	cbp := capBlendPayload{d: d}
	capZ := half
	sideZ := capZ - d
	g := capPatchGeom{
		circular: true, sideRadius: rho, capRadius: rho + d,
		th0: 0, th1: 2 * math.Pi, capTh0: 0, capTh1: 2 * math.Pi,
		sweepCCW: false, wholeTurn: true,
		sideZ: sideZ, capZ: capZ,
	}

	// delta amplified well past the record's own rounding so the coordUpper
	// mechanism's own contribution — not masked by ordinary O(1)-scale
	// arithmetic rounding elsewhere in the band — decides the bound.
	const delta = 1e-6
	mu, mv, _, err := capBandMoment(t.Context(), loop, cbp, []capPatchGeom{g}, capZ, -1, delta, work)
	require.NoError(t, err)

	// The SAME area terms capBandMoment itself composes into areaUpper,
	// gathered independently here only to state the required minimum —
	// nothing about the coordUpper mechanism under test is re-derived.
	patchArea, patchAreaBound := patchAreaOf(g)
	capArea, err := loopEnclosedAreaContext(t.Context(), capBoundary)
	require.NoError(t, err)
	areaUpper := absSumUpper(patchArea, patchAreaBound, capArea.value, capArea.bound)

	oldCoordUpper := half // the pre-fix formula: original loop envelope (~0) vs the axial level
	trueCoordUpper := rho + d
	require.Less(t, oldCoordUpper, trueCoordUpper,
		"the premise: the pre-fix coordUpper never reaches the offset boundary's own radius")

	oldAllow := sweptMomentAllow(delta, areaUpper, oldCoordUpper)
	trueAllow := sweptMomentAllow(delta, areaUpper, trueCoordUpper)
	require.Less(t, oldAllow, trueAllow)

	required := (oldAllow + trueAllow) / 2
	require.GreaterOrEqual(t, mu.bound, required,
		"mu.bound %v must reach the offset boundary's own coordinate envelope, not just the pre-fix one", mu.bound)
	require.GreaterOrEqual(t, mv.bound, required,
		"mv.bound %v must reach the offset boundary's own coordinate envelope, not just the pre-fix one", mv.bound)
}

// TestLineCircleLocusSpeedUpperRefusesMomentaryFold pins fu144's own refusal
// path for a line meeting a circle: lineCircleLocusSpeedUpper answers false
// when the corner's own discriminant Δ(t) is zero AT the corner (t = 0, a
// genuine tangency there) but its own slope Δ1 is NOT — a momentary fold,
// where the two offset carriers touch only at the corner itself and then
// immediately stop meeting at all as the offset grows, so no real locus
// exists to bound a speed over. This is the "Δ0 = 0 but Δ1 != 0" case
// lineCircleLocusSpeedUpper's own doc comment sets apart from the tangent
// join's PERSISTENT Δ ≡ 0, which returns a finite answer instead
// (TestLineCircleLocusSpeedUpperExactAtPersistentTangency below).
//
// The public Chamfer path never reaches this: every tangent join this
// evaluator itself builds (a Fillet corner, fillet.go) lands on the
// persistent branch, bit for bit, and a corner recorded through sketch's own
// solver is never quite so exactly tangent that it can hit this one narrow
// non-persistent case — the same reason capblend_normal_internal_test.go
// covers its own certified enclosures directly rather than hunting for a
// public fixture that reaches them.
//
// The construction: a straight wall along the U axis (material-side normal
// (0, 1)) meets a circular wall centred at (0, -R), radius R, so the corner
// (0, 0) is the circle's own point of tangency with the line. Offsetting the
// line INTO material moves it to y = t (anchor + t·n), which moves AWAY from
// the centre; offsetting this circle with th1 > th0 shrinks its own radius
// to R - t. A line moving away from a shrinking circle's centre separates
// from it immediately once t > 0, so Δ0 = 0 but Δ1 = -4R != 0.
func TestLineCircleLocusSpeedUpperRefusesMomentaryFold(t *testing.T) {
	const radius = 1000.0
	line := sideWalk{segmentWalk: segmentWalk{
		startU: 0, startV: 0, endU: 10, endV: 0,
		tanInU: 1, tanInV: 0, tanOutU: 1, tanOutV: 0,
	}}
	circle := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: -radius, radius: radius,
		th0: math.Pi / 2, th1: math.Pi/2 + 0.5,
	}}

	for _, d := range []float64{0.001, 0.1, 1, 3} {
		speed, ok := lineCircleLocusSpeedUpper(line, circle, 0, d)
		require.False(t, ok, "d=%v: a momentary fold must refuse, not publish a speed", d)
		require.Zero(t, speed)
	}
}

// TestLineCircleLocusSpeedUpperExactAtPersistentTangency pins the closed
// form's other branch: a line permanently tangent to a circle under a
// consistent offset (every Fillet-built corner in this codebase) publishes
// the EXACT unit speed 1, not a refusal and not circleCircleLocusSpeedUpper's
// own looser bound.
//
// The construction mirrors the refusal case above but offsets the circle the
// OTHER way (th1 < th0, growing radius R + t): a line moving away from the
// centre at rate 1 then matches the circle's own radius growing at rate 1,
// so the two stay tangent — at (0, t) — for every offset amount, and the
// locus is that same straight line, ridden at unit speed.
func TestLineCircleLocusSpeedUpperExactAtPersistentTangency(t *testing.T) {
	const radius = 1000.0
	line := sideWalk{segmentWalk: segmentWalk{
		startU: 0, startV: 0, endU: 10, endV: 0,
		tanInU: 1, tanInV: 0, tanOutU: 1, tanOutV: 0,
	}}
	circle := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: -radius, radius: radius,
		th0: math.Pi/2 + 0.5, th1: math.Pi / 2, // th1 < th0: material outside, radius grows
	}}

	for _, d := range []float64{0.001, 0.1, 1, 3} {
		speed, ok := lineCircleLocusSpeedUpper(line, circle, 0, d)
		require.True(t, ok, "d=%v: a persistent tangency must publish a finite speed", d)
		require.InDelta(t, 1.0, speed, 1e-9)
	}
}

// TestCircleCircleLocusSpeedUpperRefusesNearParallelCorner covers
// circleCircleLocusSpeedUpper's own decorrelated Cramer solve directly,
// since no public two-circular-wall miter fixture reaches its refusal
// either (circleCircleLocusSpeedUpper's own doc comment states why: it
// cannot tell a persistent circle-circle tangency from a momentary one, and
// refuses on both).
//
// The construction: a circular wall centred at the origin meets a second
// circular wall centred twice its radius further down the same axis, so the
// two circles are EXTERNALLY tangent at (0, -radius) — the shared corner —
// and their own radial directions there are anti-parallel (still a zero
// cross product, the same singular configuration a parallel pair is).
func TestCircleCircleLocusSpeedUpperRefusesNearParallelCorner(t *testing.T) {
	const radius = 1000.0
	prev := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: 0, radius: radius,
		th0: -math.Pi / 2, th1: -math.Pi / 4,
	}}
	cur := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: -2 * radius, radius: radius,
		th0: math.Pi / 2, th1: math.Pi/2 + 0.3,
	}}

	for _, d := range []float64{0.001, 0.1, 1, 3} {
		speed, ok := circleCircleLocusSpeedUpper(prev, cur, 0, d, 0, -radius)
		require.False(t, ok, "d=%v: a near-parallel corner must refuse, not publish a speed", d)
		require.Zero(t, speed)
	}
}

// TestFixPatchOrientation pins fu153: fixPatchOrientation must not swallow
// its own Face.NormalAt error and leave face.reversed at whatever the caller
// constructed it with. Both cases build a fresh whole-turn Cone band patch
// (chamferedCircularBand, capblend_normal_internal_test.go), read the TRUE
// outward direction independently before touching face.reversed, then flip it
// so the function under test has real work to do.
func TestFixPatchOrientation(t *testing.T) {
	for _, tc := range []struct {
		name string
		// sample returns the point fixPatchOrientation is called with. The
		// "decides the sign" case reuses the build's own valid sample point;
		// "refuses an unreadable sample" uses the cone's own apex, where
		// NormalAt's radial direction is zero.
		sample func(band bandUnderTest, valid r3.Vec, cone Cone) r3.Vec
	}{
		{
			name:   "decides the sign",
			sample: func(_ bandUnderTest, valid r3.Vec, _ Cone) r3.Vec { return valid },
		},
		{
			name:   "refuses an unreadable sample",
			sample: func(_ bandUnderTest, _ r3.Vec, cone Cone) r3.Vec { return cone.Origin },
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			band := chamferedCircularBand(t, diskSection(0, 0, 30), 20, 5, nil)
			cone, ok := band.face.Surface().(Cone)
			require.True(t, ok, "a whole-turn band patch is a Cone")

			// The build's own sample point and reference direction, read back
			// independently: the plane-local basis is orthonormal
			// (docs/api-design.md's r3.Frame contract), so decomposing the
			// TRUE outward normal against it is a dot product, never a
			// matrix solve.
			valid := band.pl.point(band.geom.cU+band.geom.sideRadius, band.geom.cV, band.geom.sideZ)
			truth, err := band.face.NormalAt(valid)
			require.NoError(t, err, "the band built successfully, so its own sample point must read")
			eu, ev, en := band.pl.dir(1, 0, 0), band.pl.dir(0, 1, 0), band.pl.dir(0, 0, 1)
			refU, refV, refZ := truth.Value.Dot(eu), truth.Value.Dot(ev), truth.Value.Dot(en)

			before := band.face.reversed
			band.face.reversed = !before // give the function real work to do

			sample := tc.sample(band, valid, cone)
			err = fixPatchOrientation(band.face, band.pl, sample, refU, refV, refZ)

			if tc.name == "refuses an unreadable sample" {
				require.ErrorIs(t, err, ErrUnsupported)
				require.False(t, errors.Is(err, ErrDegenerate),
					"ErrDegenerate and ErrUnsupported are opposite existence claims; only one may ride along")
				require.Equal(t, !before, band.face.reversed,
					"a refusal must not half-decide before returning")
				return
			}
			require.NoError(t, err)
			require.Equal(t, before, band.face.reversed, "the function restores the correct sign")
			n, err := band.face.NormalAt(valid)
			require.NoError(t, err)
			ref := band.pl.dir(refU, refV, refZ)
			require.Positive(t, n.Value.Dot(ref))
		})
	}
}

// capBandCircle is one chamfered circular cap loop's band, evaluated at a
// caller-chosen sweep height. The loop, the setback and therefore the BAND
// ITSELF are identical at every height — a band is a frustum between the wall
// radius and its own offset, and nothing in it reads the far end of the sweep
// — while the flux the band is computed FROM scales with that height, because
// each of the two closing disks carries capZ times a whole section area.
func capBandCircle(t *testing.T, r, d, capZ float64) boundedScalar {
	t.Helper()
	loop := LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: Point2{}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1},
	}}
	cbp := capBlendPayload{
		profile:  ProfileRecord{Outer: loop},
		z0:       0,
		z1:       capZ,
		d:        d,
		endLoops: map[int]bool{0: true},
	}
	g := capPatchGeom{
		circular: true, sideRadius: r, capRadius: r - d,
		th0: 0, th1: 2 * math.Pi, capTh0: 0, capTh1: 2 * math.Pi,
		sweepCCW: true, wholeTurn: true,
		sideZ: capZ - d, capZ: capZ,
	}
	v, err := capBandVolume(t.Context(), loop, cbp, []capPatchGeom{g}, capZ, -1, 0)
	require.NoError(t, err)
	return v
}

// TestCapBandMassBoundsChargeInheritedCapLevel isolates the cap disk that
// both capBandVolume and capBandMoment use to close a chamfer band. A ToFace
// cap level is computed, so its inherited axial displacement must remain on
// that disk and on the derived side level. Otherwise both helpers treat the
// same computed cap as exact and can publish bounds for a different solid.
func TestCapBandMassBoundsChargeInheritedCapLevel(t *testing.T) {
	const capZ, d, capDelta = 1e12, 1e-3, 2.34375e-05
	loop := synthRectLoop(0, 0, 1, 1)
	for _, tc := range []struct {
		name    string
		matSign float64
		payload capBlendPayload
	}{
		{name: `start cap`, matSign: +1, payload: capBlendPayload{d: d, z0Delta: capDelta}},
		{name: `end cap`, matSign: -1, payload: capBlendPayload{d: d, z1Delta: capDelta}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sideZ := capZ + tc.matSign*d
			geom := []capPatchGeom{
				{sideA: Point2{U: 0, V: 0}, sideB: Point2{U: 1, V: 0}, capA: Point2{U: d, V: d}, capB: Point2{U: 1 - d, V: d}, sideZ: sideZ, capZ: capZ},
				{sideA: Point2{U: 1, V: 0}, sideB: Point2{U: 1, V: 1}, capA: Point2{U: 1 - d, V: d}, capB: Point2{U: 1 - d, V: 1 - d}, sideZ: sideZ, capZ: capZ},
				{sideA: Point2{U: 1, V: 1}, sideB: Point2{U: 0, V: 1}, capA: Point2{U: 1 - d, V: 1 - d}, capB: Point2{U: d, V: 1 - d}, sideZ: sideZ, capZ: capZ},
				{sideA: Point2{U: 0, V: 1}, sideB: Point2{U: 0, V: 0}, capA: Point2{U: d, V: 1 - d}, capB: Point2{U: d, V: d}, sideZ: sideZ, capZ: capZ},
			}
			withoutDelta := tc.payload
			withoutDelta.z0Delta = 0
			withoutDelta.z1Delta = 0

			volumeWith, err := capBandVolume(t.Context(), loop, tc.payload, geom, capZ, tc.matSign, 0)
			require.NoError(t, err)
			volumeWithout, err := capBandVolume(t.Context(), loop, withoutDelta, geom, capZ, tc.matSign, 0)
			require.NoError(t, err)
			require.Greater(t, volumeWith.bound, volumeWithout.bound,
				`the cap disk's inherited axial displacement must reach the band volume bound`)

			_, _, momentWith, err := capBandMoment(t.Context(), loop, tc.payload, geom, capZ, tc.matSign, 0, newFreeformWork())
			require.NoError(t, err)
			_, _, momentWithout, err := capBandMoment(t.Context(), loop, withoutDelta, geom, capZ, tc.matSign, 0, newFreeformWork())
			require.NoError(t, err)
			require.Greater(t, momentWith.bound, momentWithout.bound,
				`the cap disk's inherited axial displacement must reach the band first-moment bound`)
		})
	}
}

// TestCapBlendSetbackConversionCarriesAllDerivedSideLevels keeps a cap-loop
// chamfer from treating a non-millimetre setback as exact after it reaches the
// payload. Both cap senses derive the side level from that converted setback,
// and a later ThroughAll stop reads the payload's axialDelta.
func TestCapBlendSetbackConversionCarriesAllDerivedSideLevels(t *testing.T) {
	d, dDelta, err := magnitudeInBounded(units.Inches(0.1), units.Length, units.Millimeter, "the chamfer setback")
	require.NoError(t, err)
	require.Positive(t, dDelta, "the fixture's inch-to-millimetre conversion rounds")

	for _, tc := range []struct {
		name  string
		capZ  float64
		sideZ float64
		start map[int]bool
		end   map[int]bool
	}{
		{name: capNameStart, capZ: 0, sideZ: d, start: map[int]bool{0: true}},
		{name: capNameEnd, capZ: 10, sideZ: 10 - d, end: map[int]bool{0: true}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cbp := capBlendPayload{
				profile:    ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
				frame:      canonicalPrismFrame(t),
				z0:         0,
				z1:         10,
				xform:      r3.Identity(),
				d:          d,
				dDelta:     dDelta,
				startLoops: tc.start,
				endLoops:   tc.end,
			}
			body, err := evalCapBlendContext(t.Context(), New(), 0, cbp)
			require.NoError(t, err)
			require.GreaterOrEqual(t, cbp.axialDelta(), dDelta)

			side := 0
			for _, vertex := range body.Vertices() {
				position := vertex.Position()
				if position.Value.Z != tc.sideZ {
					continue
				}
				require.Equal(t, Approximate, position.Exactness)
				require.GreaterOrEqual(t, position.Bound.Mag(), dDelta)
				side++
			}
			require.Equal(t, 4, side, "the square has four derived side-level vertices")
		})
	}
}

// TestCapBandVolumeBoundTracksTheFluxNotTheCancelledBand is the regression
// check for the mechanism that made the published volume bound stop
// enclosing, and it needs no platform to disagree with another to show it.
// The band is a DIFFERENCE of two closing-disk fluxes, each of magnitude
// capZ times a whole section area, and the difference is smaller than either
// by the ratio of the sweep height to the setback. A budget scaled by that
// difference under-counts the rounding of the terms it came from by exactly
// that ratio.
//
// The two readings below hold the loop and the setback fixed and move only
// the sweep height. The band is the SAME SOLID either way — a frustum
// between the wall radius and its own offset, which reads nothing about the
// far end of the sweep — so the true value is one number and the two held
// values straddle it. Their disagreement is therefore a lower bound on the
// two errors added together, which is what makes the assertion below an
// enclosure requirement rather than a preference: any pair of bounds whose
// sum falls under that disagreement cannot both contain the true value.
func TestCapBandVolumeBoundTracksTheFluxNotTheCancelledBand(t *testing.T) {
	const R, d = 1e6, 1e-3
	lo := capBandCircle(t, R, d, 10)
	hi := capBandCircle(t, R, d, 1e5)

	disagreement := math.Abs(hi.value - lo.value)
	require.Greater(t, disagreement, 1.0,
		`the premise: cancelling a 1e5 sweep down to a 1e-3 band really does cost whole units`)
	require.GreaterOrEqual(t, lo.bound+hi.bound, disagreement,
		`two readings of ONE band that disagree by more than their bounds allow cannot both enclose it`)

	// The same fact stated the other way: a bound read off the cancelled band
	// would be identical at both heights, because the band is.
	require.Greater(t, hi.bound, 1e3*lo.bound,
		`the bound has to grow with the flux the band was cancelled out of`)
}

// TestPatchAreaOfChargesTheSideLevelRounding is the mechanism-level enclosure
// check behind TestCapBlendConeAreaEnclosesTheDenotedPatch, over BOTH of
// patchAreaOf's arms. A band's side level is the single float sum
// capZ + matSign*d, so a sweep tall enough to round that sum leaves the patch
// the build HOLDS a measurable distance from the patch the chamfer DENOTES;
// each arm reads the held level as an exact input, so its own arithmetic bound
// — the Cone arm's certified frustum bracket most of all, which lifts
// H = capZ - sideZ as an exact rational — speaks for the built patch alone
// unless the level's rounding is charged separately.
//
// Each reference is the SAME closed form the arm evaluates, taken at the
// DENOTED axial separation d and translated so the cap level sits at zero: an
// area is translation invariant, and the shift keeps 1e15-scale cancellation
// out of the test's own arithmetic.
func TestPatchAreaOfChargesTheSideLevelRounding(t *testing.T) {
	const capZ, d = 1e15, 0.2
	sideZ := capZ - d
	levelDelta := addRoundError(capZ, -d, sideZ)
	require.Greater(t, levelDelta, 0.0,
		"the premise: a 1e15 mm sweep really does round the side level")

	t.Run("cone", func(t *testing.T) {
		const R = 10.0
		capR := R - d
		g := capPatchGeom{
			circular: true, sideRadius: R, capRadius: capR,
			th0: 0, th1: 2 * math.Pi, capTh0: 0, capTh1: 2 * math.Pi,
			sweepCCW: true, wholeTurn: true,
			sideZ: sideZ, capZ: capZ, levelDelta: levelDelta,
		}
		area, bound := patchAreaOf(g)
		denoted := math.Pi * (R + capR) * math.Hypot(R-capR, d)
		residual := math.Abs(area - denoted)
		require.Greater(t, residual, 1.0, "the premise: whole mm^2 of it")
		require.GreaterOrEqual(t, bound, residual,
			"bound %v must enclose the %v mm^2 residual", bound, residual)
	})

	t.Run("plane", func(t *testing.T) {
		// A long straight wall's band patch, the shape whose residual the
		// patchAreaOf head comment measures: the cap-level chord is the wall
		// inset by the setback at both ends.
		const L = 3e6
		g := capPatchGeom{
			sideA: Point2{U: 0, V: 0}, sideB: Point2{U: L, V: 0},
			capA: Point2{U: d, V: d}, capB: Point2{U: L - d, V: d},
			sideZ: sideZ, capZ: capZ, levelDelta: levelDelta,
		}
		area, bound := patchAreaOf(g)

		// The same two-triangle sum, at the denoted separation.
		v0 := r3.NewVec(g.sideA.U, g.sideA.V, -d)
		v1 := r3.NewVec(g.sideB.U, g.sideB.V, -d)
		v2 := r3.NewVec(g.capB.U, g.capB.V, 0)
		v3 := r3.NewVec(g.capA.U, g.capA.V, 0)
		denoted := v1.Sub(v0).Cross(v2.Sub(v0)).Len()/2 + v2.Sub(v0).Cross(v3.Sub(v0)).Len()/2

		residual := math.Abs(area - denoted)
		require.Greater(t, residual, 1.0, "the premise: whole mm^2 of it")
		require.GreaterOrEqual(t, bound, residual,
			"bound %v must enclose the %v mm^2 residual", bound, residual)
	})
}

// TestPatchAreaOfEnclosesRoundedRadiusDifference is the mechanism-level
// enclosure check for the OTHER input the certified frustum bracket reads.
// ΔR is not a number the payload holds: patchAreaOf forms it as the float
// subtraction R1-R0, so a bracket lifting THAT value encloses the area of a
// frustum whose slant is √(fl(R1-R0)²+H²) instead of √((R1-R0)²+H²) — a patch
// neither held nor denoted.
//
// Only the outward arm of the cap offset can round that subtraction, and this
// geometry is one real body's: a chamfered circular hole of radius 9.011 mm
// with a 16.501 mm setback, whose cap contour offsets outward to 25.512 mm
// (TestCapBlendConeAreaEnclosesRoundedRadiusDifference builds it through
// Chamfer and reads these same numbers off the patch). Nothing here is at an
// extreme scale; the ratio of setback to radius is the whole of it.
//
// contourAllow is left at its zero on purpose. That term is the band's own
// built-versus-denoted contour displacement and speaks for a different
// difference entirely; on the built body it happens to run several times
// larger than this residual and would hide the arithmetic claim behind an
// allowance that never promised to cover it. What is asserted here is the
// claim patchAreaOf's own arithmetic makes: that the held area sits within
// the returned bound of the frustum sector its own held radii, level
// separation and true window describe.
func TestPatchAreaOfEnclosesRoundedRadiusDifference(t *testing.T) {
	const prec = 600
	// π to 100 decimals: the reference window is the true 2π, which is what
	// the patch's own capThAllow brackets the held fl(2π) against.
	const piDigits = "3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798"

	g := capPatchGeom{
		circular: true, sideRadius: 9.011281351443861, capRadius: 25.512209970360068,
		th0: 0, th1: 2 * math.Pi, capTh0: 0, capTh1: 2 * math.Pi,
		wholeTurn: true,
		sideZ:     23.49907138108379, capZ: 40,
		capThAllow: 2.449293598294707e-16,
	}
	ratOf := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	exactDR := new(big.Rat).Sub(ratOf(g.capRadius), ratOf(g.sideRadius))
	require.NotEqual(t, 0, ratOf(g.capRadius-g.sideRadius).Cmp(exactDR),
		"the premise: fl(R1-R0) really does round at this radius and setback")

	area, bound := patchAreaOf(g)

	bf := func(r *big.Rat) *big.Float { return new(big.Float).SetPrec(prec).SetRat(r) }
	pi, ok := new(big.Float).SetPrec(prec).SetString(piDigits)
	require.True(t, ok)
	H := ratOf(g.capZ - g.sideZ)
	slant := new(big.Float).SetPrec(prec).Sqrt(bf(new(big.Rat).Add(
		new(big.Rat).Mul(exactDR, exactDR),
		new(big.Rat).Mul(H, H),
	)))
	radii := bf(new(big.Rat).Add(ratOf(g.sideRadius), ratOf(g.capRadius)))
	ref, _ := new(big.Float).SetPrec(prec).Mul(pi,
		new(big.Float).SetPrec(prec).Mul(radii, slant)).Float64()

	residual := math.Abs(area - ref)
	require.Greater(t, residual, 0.0,
		"the premise: the held area really does sit off the frustum its own radii denote")
	require.GreaterOrEqual(t, bound, residual,
		"bound %v must enclose the %v mm^2 residual the rounded radius difference carries", bound, residual)
}

// TestConeFrustumAreaBracketEnclosesReference is coneFrustumAreaBracket's own
// soundness check, independent of any build. A = (dth/2)*(R0+R1)*slant is the
// frustum-sector formula patchAreaOf's Cone arm evaluates; dthAllow claims
// the true window sits within dthAllow of the held dth, so the certified
// bound must enclose the reference area computed at every trueDth the caller
// admits — dth-dthAllow, dth and dth+dthAllow — not merely at dth itself.
// The reference is built from the SAME exact-rational construction
// coneFrustumAreaBracket uses internally (big.Rat sums/differences, a
// 512-bit square root), so no float64 rounding on the test's own side can
// manufacture a false failure: what a real build hands this function is
// always a float64 dth and a float64 dthAllow, and this is the honest
// enclosure claim over the interval those two floats denote. This is the
// property that must never regress — the tightening this fix makes over
// conservativeValueError's old fallback is sound only where this still
// holds.
func TestConeFrustumAreaBracketEnclosesReference(t *testing.T) {
	const prec = 512
	ratOfFloat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	bfRat := func(r *big.Rat) *big.Float { return new(big.Float).SetPrec(prec).SetRat(r) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(a, b) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Add(a, b) }
	quo := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Quo(a, b) }
	sqrtF := func(a *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sqrt(a) }

	cases := []struct {
		name                  string
		R0, R1, H, dth, allow float64
		roundedDR             bool
	}{
		{name: "ordinary miter wall", R0: 10, R1: 9.5, H: 8, dth: 0.3, allow: 1e-9},
		{name: "whole-turn cylinder cap", R0: 10, R1: 9.5, H: 8, dth: 2 * math.Pi, allow: 1e-12},
		{name: "tiny apex sweep", R0: 0.5, R1: 0.5, H: 0.5, dth: 0.001, allow: 1e-6},
		{name: "huge radius, tiny setback", R0: 1e12, R1: 1e12 - 1e-3, H: 10, dth: math.Pi / 2, allow: 1e-15},
		{name: "zero allowance (a perfectly known window)", R0: 20, R1: 18, H: 5, dth: 1.0, allow: 0},
		{name: "equal radii, pure axial slant", R0: 15, R1: 15, H: 6, dth: 0.7, allow: 1e-8},
		// The outward arm of the cap offset, at millimetre scale: a chamfered
		// circular hole whose cap radius is R+d rather than the inward arm's
		// Sterbenz-exact R-d. The subtraction R1-R0 rounds here, so a bracket
		// lifting the CALLER's float difference brackets a frustum nobody
		// holds and excludes the held area's own residual.
		{
			name: "hole offset whose radius difference rounds",
			R0:   9.011281351443861, R1: 9.011281351443861 + 16.500928618916209,
			H: 16.500928618916209, dth: 2 * math.Pi, allow: 2.4492935982947064e-16,
			roundedDR: true,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			dR := tc.R1 - tc.R0
			H := tc.H
			exactDR := new(big.Rat).Sub(ratOfFloat(tc.R1), ratOfFloat(tc.R0))
			require.Equal(t, tc.roundedDR, ratOfFloat(dR).Cmp(exactDR) != 0,
				"the case's own premise about whether fl(R1-R0) rounds")

			held := tc.dth / 2 * (tc.R0 + tc.R1) * math.Hypot(dR, H)
			bound := coneFrustumAreaBracket(tc.R0, tc.R1, H, tc.dth, tc.allow, held)
			require.False(t, math.IsInf(bound, 1), "the bracket must build for well-formed finite inputs")

			// The reference slant is the EXACT R1-R0, the difference the held
			// radii denote, never the float subtraction the held area was
			// evaluated through.
			rDR, rH := exactDR, ratOfFloat(H)
			slant := sqrtF(add(mul(bfRat(rDR), bfRat(rDR)), mul(bfRat(rH), bfRat(rH))))
			radii := bfRat(new(big.Rat).Add(ratOfFloat(tc.R0), ratOfFloat(tc.R1)))
			dthRat, allowRat := ratOfFloat(tc.dth), ratOfFloat(tc.allow)
			for _, trueDthRat := range []*big.Rat{
				new(big.Rat).Sub(dthRat, allowRat),
				dthRat,
				new(big.Rat).Add(dthRat, allowRat),
			} {
				ref := quo(mul(bfRat(trueDthRat), mul(radii, slant)), bfRat(big.NewRat(2, 1)))
				refF, _ := ref.Float64()
				residual := math.Abs(held - refF)
				require.LessOrEqual(t, residual, bound,
					"trueDth=%v residual %v must not exceed bound %v", trueDthRat, residual, bound)
			}
		})
	}
}

// bandUnderTest is one chamfered circular cap loop read back as the survey
// reads it: the payload's own patch geometry, the Face DX7 samples, and the
// plane-local map both go through.
type bandUnderTest struct {
	face *Face
	pl   prismPayload
	geom capPatchGeom
}

// chamferedCircularBand chamfers one extruded section's end cap and returns
// its single CIRCULAR band patch. section draws the sketch the extrusion runs
// on, so one caller can hand it a whole disk (a cornerless loop, whose band is
// one whole turn) and another a quarter disk (a wall mitered at both ends,
// whose band patch reads a genuine window).
func chamferedCircularBand(t *testing.T, section func(*sketch.Sketch), h, d float64, motion *r3.Transform) bandUnderTest {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	section(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(Edges(CreatedBy(CapEnd(body))), units.Millimeters(d))
	require.NoError(t, err)
	if motion != nil {
		chamfered, err = chamfered.Placed(*motion)
		require.NoError(t, err)
	}

	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	roles := facesByRole(chamfered)
	var out bandUnderTest
	found := 0
	for _, patch := range cbp.patches {
		if !patch.geom.circular {
			continue
		}
		found++
		face := roles[patch.role]
		require.NotNil(t, face)
		require.Equal(t, KindCone, face.Surface().Kind())
		// The sweep levels the payload itself carries: the reading under test
		// takes its own levels from the patch geometry, so these only pin that
		// the survey's plane-local map is the payload's own.
		out = bandUnderTest{face: face, pl: cbp.prismLike(cbp.z0, cbp.z1), geom: patch.geom}
	}
	require.Equal(t, 1, found, "these sections each carry one circular wall")
	return out
}

func diskSection(cx, cy, r float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		c := s.CreatePoint(cx, cy)
		s.CreateCircle(c, r)
		s.Fix(c)
	}
}

// quarterDiskSection is the same shape quarterDiskBody builds for the external
// survey tests: one circular wall meeting a straight neighbour at a
// non-tangential miter at BOTH ends, so its band patch reads a quarter-turn
// window rather than a whole period.
//
// Unlike diskSection, this one is drawn at the sketch's own origin and carried
// out into the world by its PLACEMENT. The arc has to weld to a line at each
// end, and sketch judges such a weld against the section's own extent
// (geom.weldIdentEps times the source's extent), so a half-millimetre section
// drawn a metre out leaves that weld a handful of ulps of the coordinates it
// is welding — a margin each platform's own arithmetic can land either side
// of. A closed circle welds to nothing and so has no such corner.
func quarterDiskSection(cx, cy, r float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		o := s.CreatePoint(cx, cy)
		s.Fix(o)
		px := s.CreatePoint(cx+r, cy)
		py := s.CreatePoint(cx, cy+r)
		s.CreateLine(o, px)
		s.CreateLine(py, o)
		s.CreateArc(o, px, py)
	}
}

// chamferedRectFlatPatches chamfers a 100x60 rectangular plate's end cap loop
// and places it under motion, returning its four FLAT (Plane-tagged,
// non-circular) band patches read back as the survey reads them. A polygon's
// corners chamfer to Plane patches only — there is no circular wall to
// contribute a Cone — so every patch this returns exercises the flat arm.
func chamferedRectFlatPatches(t *testing.T, motion r3.Transform) []bandUnderTest {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(Edges(CreatedBy(CapEnd(body))), units.Millimeters(3))
	require.NoError(t, err)
	placed, err := chamfered.Placed(motion)
	require.NoError(t, err)

	cbp, ok := placed.payload.(capBlendPayload)
	require.True(t, ok)
	roles := facesByRole(placed)
	pl := cbp.prismLike(cbp.z0, cbp.z1)
	var out []bandUnderTest
	for _, patch := range cbp.patches {
		if patch.geom.circular {
			continue
		}
		face := roles[patch.role]
		require.NotNil(t, face)
		require.Equal(t, KindPlane, face.Surface().Kind())
		out = append(out, bandUnderTest{face: face, pl: pl, geom: patch.geom})
	}
	require.Len(t, out, 4, "a rectangle's four corners each chamfer to one flat patch")
	return out
}

// placedFarMotion is the placement chamferedRectFlatPatches' callers use to
// make a flat patch's own f.normalBound nonzero: a spin composed with a shift
// far from the world origin, the same motion
// TestCapPatchNormalRangeCoversWhatThePatchTakes uses to charge the circular
// arm's own displacement term.
func placedFarMotion(t *testing.T) r3.Transform {
	t.Helper()
	spin, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1000, 1000, 0))
	require.NoError(t, err)
	motion, err := spin.Then(shift)
	require.NoError(t, err)
	return motion
}

// TestCapPatchNormalRangeChargesAFlatPatchsDepartureOnce is the flat arm's own
// half of fu154: a placed flat patch's Face.NormalAt already composes the
// patch's departure from the Plane it publishes (normalMeasurement,
// topology.go), so capPatchNormalRange's flat branch must take that bound
// AS the allowance rather than add its own copy of it on top. An allowance
// charging the departure twice covers the patch just as soundly, but chases
// away the ~2x margin this test pins.
func TestCapPatchNormalRangeChargesAFlatPatchsDepartureOnce(t *testing.T) {
	motion := placedFarMotion(t)
	pull, ok := r3.NewVec(1, 0.3, 0.2).Normalize()
	require.True(t, ok)

	for i, band := range chamferedRectFlatPatches(t, motion) {
		t.Run(fmt.Sprintf("patch %d", i), func(t *testing.T) {
			require.Positive(t, band.face.normalBound,
				"a placed flat patch has a real departure from the plane it publishes")
			lo, hi, allow, ok := capPatchNormalRange(band.face, band.pl, band.geom, pull)
			require.True(t, ok)
			require.Equal(t, lo, hi, "a flat patch's own normal component does not vary with position")
			require.GreaterOrEqual(t, allow, band.face.normalBound, "the departure is still covered")
			require.Less(t, allow, 1.2*band.face.normalBound, "the departure is covered only once")
		})
	}
}

// TestCapPatchNormalRangeCoversWhatAFlatPatchTakes is the correctness half:
// the single allowance capPatchNormalRange reports for a flat patch, read at
// its one sampled corner, must still bound Face.NormalAt's own published
// bound everywhere else on the patch's quad — not only where it was sampled.
func TestCapPatchNormalRangeCoversWhatAFlatPatchTakes(t *testing.T) {
	motion := placedFarMotion(t)
	pull, ok := r3.NewVec(1, 0.3, 0.2).Normalize()
	require.True(t, ok)

	for i, band := range chamferedRectFlatPatches(t, motion) {
		t.Run(fmt.Sprintf("patch %d", i), func(t *testing.T) {
			_, _, allow, ok := capPatchNormalRange(band.face, band.pl, band.geom, pull)
			require.True(t, ok)

			// The quad's own walk order (capblend_geom.go): sideA -> sideB
			// along the original wall, sideB -> capB up the trailing slant,
			// capB -> capA along the cap contour, capA -> sideA down the
			// leading slant.
			g := band.geom
			corners := []r3.Vec{
				band.pl.point(g.sideA.U, g.sideA.V, g.sideZ),
				band.pl.point(g.sideB.U, g.sideB.V, g.sideZ),
				band.pl.point(g.capB.U, g.capB.V, g.capZ),
				band.pl.point(g.capA.U, g.capA.V, g.capZ),
			}
			points := append([]r3.Vec(nil), corners...)
			for i, c := range corners {
				points = append(points, c.Add(corners[(i+1)%len(corners)]).Scale(0.5))
			}
			for _, pt := range points {
				n, err := band.face.NormalAt(pt)
				require.NoError(t, err)
				bound, err := n.Bound.In(units.One)
				require.NoError(t, err)
				require.LessOrEqual(t, bound*pull.Len(), allow,
					"the reported allowance must cover every point of the patch, not only the one sampled")
			}
		})
	}
}

// componentAt reads the band's own published normal component against the pull
// at one azimuth of its window, beside the bound that reading publishes. It is
// the survey's own sampling arm, used here as an INDEPENDENT witness: the
// component of the patch's exact normal at that point lies within the returned
// bound of the returned value, so a range that excludes the whole bracket is
// excluding a value the patch really takes.
//
// The second bound is the ARM's own half of the same reading. Face.NormalAt
// publishes the arm's arithmetic proof composed with the face's own departure
// from its tag (capblend_departure.go), and only the first is what a reading
// that charged its arithmetic alone would have had — which is the figure the
// sample-displacement ratios below are measured against.
func (b bandUnderTest) componentAt(t *testing.T, theta float64, pull r3.Vec) (float64, float64, float64) {
	t.Helper()
	sin, cos := math.Sincos(theta)
	g := b.geom
	n, err := b.face.NormalAt(b.pl.point(g.cU+g.capRadius*cos, g.cV+g.capRadius*sin, g.capZ))
	require.NoError(t, err)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	return n.Value.Dot(pull), bound * pull.Len(), math.Max(0, bound-b.face.normalBound) * pull.Len()
}

// TestCapPatchNormalRangeCoversWhatThePatchTakes is the sampled half of the
// DX7 reading's proof. The survey recovers its A·cos+B·sin+C form from three
// readings taken at points it computes itself — a float sine and cosine of an
// azimuth, then two rounded maps into world space — and the azimuth those
// points really carry is not the one the recovery reads them as. That
// displacement scales like the frame origin's own rounding divided by the
// patch's radius, so a small band whose placement carries it far from the
// world origin makes it hundreds of times what any reading's own bound covers.
// A case that means to charge that displacement says how far out its patch has
// to sit, and the run checks that its centre really got there.
//
// Whatever it is, the range the survey reports must still hold every value the
// patch takes: each independently read component, widened by its own published
// bound, must sit inside [lo-allow, hi+allow].
func TestCapPatchNormalRangeCoversWhatThePatchTakes(t *testing.T) {
	spin, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1000, 1000, 0))
	require.NoError(t, err)
	// The same motion a metre out: what the reading pays for is the rounding of
	// the placed frame's own origin, so a section drawn at the sketch origin and
	// carried out here is displaced exactly as one drawn out there would be.
	spun, err := spin.Then(shift)
	require.NoError(t, err)
	pull, ok := r3.NewVec(1, 0.3, 0.2).Normalize()
	require.True(t, ok)

	for _, tc := range []struct {
		name string
		// overArmBounds is how many times the three sampled readings' OWN
		// published bounds the allowance must exceed. Those bounds are all a
		// reading that charged the arm alone ever had, and they say nothing about
		// where the point handed to the arm sits.
		overArmBounds  float64
		underArmBounds float64
		// centreFar is how far from the world origin the patch's own centre must
		// end up. A case charging the frame origin's rounding has to be placed
		// out there for the charge to mean anything.
		centreFar float64
		section   func(*sketch.Sketch)
		motion    *r3.Transform
	}{
		{
			// An axis-aligned band about the origin: the sample points land where
			// they were asked to, so there is nothing beyond the readings' own
			// arithmetic to pay for.
			name: `whole turn at the origin`, section: diskSection(0, 0, 12),
			underArmBounds: 10,
		},
		{
			// ulp(1000) over a radius of half a millimetre displaces each sample
			// by some 2e-13 of azimuth, which is two orders more than the
			// readings' own bounds cover.
			name: `whole turn far from the origin, placed`, section: diskSection(1000, 1000, 0.55), motion: &spin,
			overArmBounds: 50, centreFar: 1000,
		},
		{
			// A genuine window rather than a whole period, so the reported ends are
			// the window's own rather than the form's amplitude. No ratio is
			// claimed here: this patch is mitered, and the surface-departure bound
			// its readings carry dwarfs every arithmetic term on a band this small.
			// The section is drawn at the sketch origin and placed out here
			// (quarterDiskSection), which displaces the sample points exactly as
			// drawing it out here would.
			name: `quarter window far from the origin, placed`, section: quarterDiskSection(0, 0, 0.55), motion: &spun,
			centreFar: 1000,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			band := chamferedCircularBand(t, tc.section, 10, 0.2, tc.motion)
			lo, hi, allow, ok := capPatchNormalRange(band.face, band.pl, band.geom, pull)
			require.True(t, ok)
			require.GreaterOrEqual(t, allow, band.face.normalBound,
				"the circular arm composes the patch's own departure inside capPatchNormalRange")

			g := band.geom
			if tc.centreFar > 0 {
				require.Greater(t, band.pl.point(g.cU, g.cV, g.capZ).Len(), tc.centreFar,
					"this case charges the frame origin's rounding, so its patch must sit that far out")
			}
			arms := 0.0
			for _, off := range []float64{0, math.Pi / 2, math.Pi} {
				_, _, arm := band.componentAt(t, g.th0+off, pull)
				arms += arm
			}
			require.Positive(t, arms, "a Cone arm's own cosine and sine are not exact")
			if tc.overArmBounds > 0 {
				require.Greater(t, allow, tc.overArmBounds*arms,
					"the displaced sample points cost more than the readings' own bounds")
			}
			if tc.underArmBounds > 0 {
				require.Less(t, allow, tc.underArmBounds*arms,
					"an undisplaced band must not be charged for a displacement it has not got")
			}
			const samples = 512
			low, high := math.Inf(1), math.Inf(-1)
			for k := range samples + 1 {
				theta := g.th0 + (g.th1-g.th0)*float64(k)/samples
				v, bound, _ := band.componentAt(t, theta, pull)
				require.GreaterOrEqual(t, v+bound, lo-allow,
					"azimuth %v takes a component below the reported range", theta)
				require.LessOrEqual(t, v-bound, hi+allow,
					"azimuth %v takes a component above the reported range", theta)
				low, high = math.Min(low, v), math.Max(high, v)
			}
			// A range wide enough to hold anything would pass the containment
			// above for free. These witnesses come within the grid's own reach of
			// both ends, so the reported ends are the patch's own.
			require.Less(t, high-hi, 1e-4, "the reported maximum runs past what the patch reaches")
			require.Less(t, lo-low, 1e-4, "the reported minimum runs past what the patch reaches")
		})
	}
}

// TestHarmonicWindowRangeEnclosesInteriorExtremes is the reading half of the
// same proof. A window whose interior holds the form's stationary point takes
// its whole amplitude there, and a float64 evaluation of that extreme — a sine
// and a cosine at each end, an arctangent for the stationary point, a
// multiply-add at each candidate — can report an interval the exact extreme
// lies OUTSIDE of. The enclosure must bracket c±R, and must place its own
// reported ends outside every value the window actually takes.
func TestHarmonicWindowRangeEnclosesInteriorExtremes(t *testing.T) {
	for _, tc := range []struct {
		name       string
		a, b, c    *big.Rat
		width      *big.Rat
		wholeTurn  bool
		interior   bool
		tightBelow float64
	}{
		{
			name: `stationary point inside`, a: big.NewRat(3, 7), b: big.NewRat(-5, 11), c: big.NewRat(1, 3),
			width: big.NewRat(11, 2), interior: true, tightBelow: 1e-15,
		},
		{
			name: `endpoints only`, a: big.NewRat(1, 1), b: big.NewRat(0, 1), c: big.NewRat(0, 1),
			width: big.NewRat(1, 1), interior: false, tightBelow: 1e-15,
		},
		{
			name: `whole turn`, a: big.NewRat(2, 5), b: big.NewRat(9, 8), c: big.NewRat(-1, 4),
			width: big.NewRat(0, 1), wholeTurn: true, interior: true, tightBelow: 1e-15,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ext, ok := harmonicWindowRange(tc.a, tc.b, tc.c, tc.width, tc.wholeTurn)
			require.True(t, ok)
			require.LessOrEqual(t, ext.minLo.Cmp(ext.minHi), 0)
			require.LessOrEqual(t, ext.maxLo.Cmp(ext.maxHi), 0)
			require.Less(t, ratFloatUp(new(big.Rat).Sub(ext.minHi, ext.minLo)), tc.tightBelow)
			require.Less(t, ratFloatUp(new(big.Rat).Sub(ext.maxHi, ext.maxLo)), tc.tightBelow)

			amp, okAmp := intervalSqrt(pointInterval(ratAdd(ratMul(tc.a, tc.a), ratMul(tc.b, tc.b))))
			require.True(t, okAmp)
			if tc.interior {
				// The stationary points are reached, so both extremes are the
				// form's own amplitude about c and the enclosure must bracket them.
				trough := intervalSub(pointInterval(tc.c), amp)
				peak := intervalAdd(pointInterval(tc.c), amp)
				require.LessOrEqual(t, ext.minLo.Cmp(trough.hi), 0)
				require.GreaterOrEqual(t, ext.minHi.Cmp(trough.lo), 0)
				require.GreaterOrEqual(t, ext.maxHi.Cmp(peak.lo), 0)
				require.LessOrEqual(t, ext.maxLo.Cmp(peak.hi), 0)
			}

			// No azimuth of the window may take a value the reported enclosure
			// says is out of reach — including the stationary point itself,
			// where the float reading's own rounding used to escape.
			width := tc.width
			if tc.wholeTurn {
				width = ratMul(big.NewRat(2, 1), piUpper)
			}
			stationary, okStat := ratOf(math.Atan2(ratFloat(tc.b), ratFloat(tc.a)))
			require.True(t, okStat)
			for k := range 129 {
				phi := ratMul(width, big.NewRat(int64(k), 128))
				for _, at := range []*big.Rat{phi, new(big.Rat).Add(stationary, phi), new(big.Rat).Sub(stationary, phi)} {
					if at.Cmp(new(big.Rat)) < 0 || at.Cmp(width) > 0 {
						continue
					}
					sin, cos, okT := radSinCosInterval(at)
					require.True(t, okT)
					v := intervalAdd(intervalAdd(intervalScale(cos, tc.a), intervalScale(sin, tc.b)), pointInterval(tc.c))
					require.LessOrEqual(t, ext.minLo.Cmp(v.hi), 0, "a reachable value sits below the reported minimum")
					require.GreaterOrEqual(t, ext.maxHi.Cmp(v.lo), 0, "a reachable value sits above the reported maximum")
				}
			}
		})
	}
}

func ratFloat(q *big.Rat) float64 {
	f, _ := q.Float64()
	return f
}
