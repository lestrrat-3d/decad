package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

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
