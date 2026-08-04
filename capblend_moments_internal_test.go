package decad

import (
	"math"
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
		th0: 0, th1: 2 * math.Pi, sweepCCW: true, wholeTurn: true,
		sideZ: capZ - d, capZ: capZ,
	}
	v, err := capBandVolume(t.Context(), loop, cbp, []capPatchGeom{g}, capZ, -1)
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
