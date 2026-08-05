package decad

import (
	"math"
	"math/big"
	"testing"

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
	}{
		{"ordinary miter wall", 10, 9.5, 8, 0.3, 1e-9},
		{"whole-turn cylinder cap", 10, 9.5, 8, 2 * math.Pi, 1e-12},
		{"tiny apex sweep", 0.5, 0.5, 0.5, 0.001, 1e-6},
		{"huge radius, tiny setback", 1e12, 1e12 - 1e-3, 10, math.Pi / 2, 1e-15},
		{"zero allowance (a perfectly known window)", 20, 18, 5, 1.0, 0},
		{"equal radii, pure axial slant", 15, 15, 6, 0.7, 1e-8},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			dR := tc.R1 - tc.R0
			H := tc.H
			held := tc.dth / 2 * (tc.R0 + tc.R1) * math.Hypot(dR, H)
			bound := coneFrustumAreaBracket(tc.R0, tc.R1, dR, H, tc.dth, tc.allow, held)
			require.False(t, math.IsInf(bound, 1), "the bracket must build for well-formed finite inputs")

			rDR, rH := ratOfFloat(dR), ratOfFloat(H)
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
