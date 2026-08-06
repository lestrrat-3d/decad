package decad

import (
	"math"
	"testing"

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
