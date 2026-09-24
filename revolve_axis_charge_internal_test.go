package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// This file pins docs/surface-intersection-design.md §7.1's one operational
// requirement on the fold: an ABSENT charge must leave every field the fold
// touches exactly as the uncharged walk left it. absSumUpper up-rounds every
// term it folds, so a fold taken unconditionally would widen an untrimmed
// revolve's every published bound by an ulp per term — and an untrimmed
// revolve has to read exactly as it read before the fold existed.

// chargeProbeAxis is a unit axis along +v anchored at the plane origin, with
// zero proven bounds on all four fields: the axis every T180-shaped fixture
// resolves, and the one that makes toAxisRhoBound answer zero so a widening
// shows up undiluted.
func chargeProbeAxis() axisFrame {
	return axisFrame{aU: 0, aV: 0, dU: 0, dV: 1}
}

func chargeProbeWalk(t *testing.T) segmentWalk {
	t.Helper()
	w, err := walkOf(LineSeg{
		Start:  Point2{U: 3, V: 0},
		End:    Point2{U: 3, V: 20},
		TStart: 0,
		TEnd:   1,
	}, nil)
	require.NoError(t, err)
	return w
}

func TestAxisFrameWalkChargeIsAbsentAtZero(t *testing.T) {
	ax := chargeProbeAxis()
	w := chargeProbeWalk(t)

	charged := ax.walkCharged(w, walkEndBound{}, walkEndBound{})

	// Every field the fold touches keeps the value the UNCHARGED derivation
	// gives it, with no up-round nudge of its own. The right-hand sides are
	// derived from the INPUT walk and the axis rather than read back off
	// another walkCharged call, so the comparison carries content rather than
	// restating the same expression twice.
	require.Equal(t, ax.toAxisRhoBound(w.startU, w.startV), charged.startVBound)
	require.Equal(t, ax.toAxisRhoBound(w.endU, w.endV), charged.endVBound)
	require.Equal(t, w.lengthBound, charged.lengthBound)
	require.Equal(t, w.lengthUpper, charged.lengthUpper)
	require.Equal(t, w.coordUpper, charged.coordUpper)
	require.Equal(t, ax.radialUpper(w.coordUpper), charged.axisRadiusUpper)
	require.Equal(t, productUpper(w.lengthUpper, ax.radialUpper(w.coordUpper)), charged.axisMomentUpper)

	// axisFrame.walk IS walkCharged with both charges absent, so an ordinary
	// revolve reaches exactly the values above and no other.
	require.Equal(t, charged, ax.walk(w))
}

func TestAxisFrameWalkChargeWidensEveryFieldItTouches(t *testing.T) {
	ax := chargeProbeAxis()
	w := chargeProbeWalk(t)
	plain := ax.walk(w)

	// A charge at the START end alone: the design's own per-endpoint rule, so
	// the other end's radial bound must not move while the shared length and
	// envelope figures must.
	charge := walkEndBound{u: 1e-9, v: 1e-9}
	startOnly := ax.walkCharged(w, charge, walkEndBound{})
	require.Greater(t, startOnly.startVBound, plain.startVBound)
	require.Equal(t, plain.endVBound, startOnly.endVBound, "a charge at one end never moves the other end's own radial bound")
	require.Greater(t, startOnly.lengthBound, plain.lengthBound)
	require.Greater(t, startOnly.lengthUpper, plain.lengthUpper)
	require.Greater(t, startOnly.coordUpper, plain.coordUpper)
	// The envelope has to widen with it, or walkAxisMoment's own math.Min
	// against conservativeValueError(value, axisMomentUpper) would clamp the
	// charge straight back off (§7.1).
	require.Greater(t, startOnly.axisMomentUpper, plain.axisMomentUpper)
	require.Greater(t, startOnly.axisRadiusUpper, plain.axisRadiusUpper)

	both := ax.walkCharged(w, charge, charge)
	require.Greater(t, both.endVBound, plain.endVBound)
	require.Greater(t, both.lengthBound, startOnly.lengthBound, "two moved ends move the chord by more than one does")
}
