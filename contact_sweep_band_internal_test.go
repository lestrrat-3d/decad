package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlanarReplayLowerGapChargesTravel reads the replay's lower gap inside a
// certified clear span directly. A sound sweep leaves no span whose margin
// falls below a pose rounding, so no public fixture can show the travel charge
// failing; this one does: deleting the charge reads the span's end gaps
// unreduced, 3 mm instead of 2 mm.
func TestPlanarReplayLowerGapChargesTravel(t *testing.T) {
	replay := planarReplay{travel: big.NewRat(4, 1), spans: []planarClearSpan{
		{from: new(big.Rat), to: big.NewRat(1, 2), left: big.NewRat(3, 1), right: big.NewRat(1, 1)},
		{from: big.NewRat(1, 2), to: big.NewRat(1, 1), axis: big.NewRat(1, 8)},
	}}
	// At 1/4: the left end's 3 mm less 4·(1/4), against the right end's
	// 1 mm less 4·(1/4).
	require.Zero(t, big.NewRat(2, 1).Cmp(replay.lowerGap(big.NewRat(1, 4))))
	// A hull-separated span reads its hull gap anywhere inside it.
	require.Zero(t, big.NewRat(1, 8).Cmp(replay.lowerGap(big.NewRat(3, 4))))
	require.Nil(t, (&planarReplay{}).lowerGap(big.NewRat(1, 2)))
}

// TestRotatingBracketDepthChargesTravelAndDeviation reads the replay inside a
// rotating impact bracket directly (bracketDepthWithin): a fraction f past the
// left edge lo replays when (f − lo)·T − g + deviation, the farthest the
// rounded pair can lie inside a separated one, fits PointResolution. A public
// fixture's bracket travel and pose rounding sit far below any useful
// resolution, so this one shows both charges failing: deleting the travel
// charge accepts f = 3/4 at resolution 3/4, and deleting the deviation
// accepts f = 1/2 at resolution −1/4. The left gap g is a credit, not a
// charge: deleting it only refuses more.
func TestRotatingBracketDepthChargesTravelAndDeviation(t *testing.T) {
	proof := sweepReplayProof{bracketLo: big.NewRat(1, 2), bracketHi: big.NewRat(1, 1),
		bracketGap: big.NewRat(1, 4), bracketTravel: big.NewRat(4, 1)}
	deviation := big.NewRat(1, 8)
	// (3/4 − 1/2)·4 − 1/4 + 1/8 = 7/8.
	require.True(t, proof.bracketDepthWithin(big.NewRat(3, 4), deviation, big.NewRat(7, 8)))
	require.False(t, proof.bracketDepthWithin(big.NewRat(3, 4), deviation, big.NewRat(3, 4)))
	// At the left edge the gap alone remains: −1/4 + 1/8 = −1/8.
	require.True(t, proof.bracketDepthWithin(big.NewRat(1, 2), deviation, new(big.Rat)))
	require.False(t, proof.bracketDepthWithin(big.NewRat(1, 2), deviation, big.NewRat(-1, 4)))
	// Past the right edge nothing replays.
	require.False(t, proof.bracketDepthWithin(big.NewRat(9, 8), deviation, big.NewRat(100, 1)))
	require.False(t, (&sweepReplayProof{bracketLo: big.NewRat(1, 2), bracketHi: big.NewRat(1, 1)}).
		bracketDepthWithin(big.NewRat(3, 4), deviation, big.NewRat(100, 1)))
}

// TestPlanarDepartureLowerGapIsBoundedByLateralClearance reads §10.6's lower
// gap off a proven departure: the 8 mm cube rests on a tray's floor 2⁻¹⁰ mm
// from its wall at x = 80 and rises at 100 mm/s. Its corners soon stand far
// above the floor, but the wall stays 2⁻¹⁰ mm away, so the departure's lower
// gap is the lateral clearance, exactly 2⁻¹⁰ mm: the wall's projection is a
// segment on x = 80 and the cube's path box ends at x = 80 − 2⁻¹⁰. Deleting
// the clearance publishes the height bound, about 6 mm.
func TestPlanarDepartureLowerGapIsBoundedByLateralClearance(t *testing.T) {
	doc := New()
	outer := internalBoxBody(t, doc, -90, -90, 90, 90, 50)
	lower, err := r3.Translation(r3.Vec{Z: -10})
	require.NoError(t, err)
	outer, err = outer.Placed(t.Context(), lower)
	require.NoError(t, err)
	inner := internalBoxBody(t, doc, -80, -80, 80, 80, 50)
	tray, err := Cut(t.Context(), outer, inner)
	require.NoError(t, err)
	cube := internalBoxBody(t, doc, 0, -4, 8, 4, 8)

	clearance := big.NewRat(1, 1<<10)
	at, err := r3.Translation(r3.Vec{X: 72 - 1.0/(1<<10)})
	require.NoError(t, err)
	zero := units.MillimetersPerSecond(0)
	still := units.RadiansPerSecond(0)
	cubePath := RigidDriftSegment{From: at,
		LinearVelocity:  QuantityVec{X: zero, Y: zero, Z: units.MillimetersPerSecond(100)},
		AngularVelocity: QuantityVec{X: still, Y: still, Z: still}, Duration: units.Seconds(1.0 / 16)}
	trayPath := RigidDriftSegment{From: r3.Identity(), LinearVelocity: QuantityVec{X: zero, Y: zero, Z: zero},
		AngularVelocity: QuantityVec{X: still, Y: still, Z: still}, Duration: units.Seconds(1.0 / 16)}
	for _, run := range []*rotationalPairSweep{
		bandRun(t, doc, tray, cube, trayPath, cubePath),
		bandRun(t, doc, cube, tray, cubePath, trayPath),
	} {
		until, ok, err := run.planarDepartureFraction(t.Context())
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, until.Cmp(big.NewRat(1, 1)))
		require.True(t, run.departure.support.local)
		// At the start of the sweep the height bound is the smaller term; at
		// its end the clearance is.
		early := big.NewRat(1, 1<<20)
		require.Positive(t, run.departure.lowerGap(early).Cmp(new(big.Rat)))
		require.Negative(t, run.departure.lowerGap(early).Cmp(clearance))
		require.Zero(t, run.departure.lowerGap(until).Cmp(clearance))
	}
}
