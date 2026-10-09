package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/stretchr/testify/require"
)

// This file pins docs/surface-intersection-design.md §7.1's one operational
// requirement on the fold: an ABSENT charge must leave every field the fold
// touches exactly as the uncharged walk left it. proofbound.AbsSumUpper up-rounds every
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

func chargeProbeWalk(t *testing.T) survey2d.SegmentWalk {
	t.Helper()
	w, err := boundarywalk.WalkOf(lineSeg{
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

	charged := ax.walkCharged(w, proofbound.WalkEndBound{}, proofbound.WalkEndBound{})

	// Every field the fold touches keeps the value the UNCHARGED derivation
	// gives it, with no up-round nudge of its own. The right-hand sides are
	// derived from the INPUT walk and the axis rather than read back off
	// another walkCharged call, so the comparison carries content rather than
	// restating the same expression twice.
	require.Equal(t, ax.toAxisRhoBound(w.StartU, w.StartV), charged.StartVBound)
	require.Equal(t, ax.toAxisRhoBound(w.EndU, w.EndV), charged.EndVBound)
	require.Equal(t, w.LengthBound, charged.LengthBound)
	require.Equal(t, w.LengthUpper, charged.LengthUpper)
	require.Equal(t, w.CoordUpper, charged.CoordUpper)
	require.Equal(t, ax.radialUpper(w.CoordUpper), charged.AxisRadiusUpper)
	require.Equal(t, proofbound.ProductUpper(w.LengthUpper, ax.radialUpper(w.CoordUpper)), charged.AxisMomentUpper)

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
	charge := proofbound.WalkEndBound{U: 1e-9, V: 1e-9}
	startOnly := ax.walkCharged(w, charge, proofbound.WalkEndBound{})
	require.Greater(t, startOnly.StartVBound, plain.StartVBound)
	require.Equal(t, plain.EndVBound, startOnly.EndVBound, "a charge at one end never moves the other end's own radial bound")
	require.Greater(t, startOnly.LengthBound, plain.LengthBound)
	require.Greater(t, startOnly.LengthUpper, plain.LengthUpper)
	require.Greater(t, startOnly.CoordUpper, plain.CoordUpper)
	// The envelope has to widen with it, or walkAxisMoment's own math.Min
	// against proofbound.ConservativeValueError(value, axisMomentUpper) would clamp the
	// charge straight back off (§7.1).
	require.Greater(t, startOnly.AxisMomentUpper, plain.AxisMomentUpper)
	require.Greater(t, startOnly.AxisRadiusUpper, plain.AxisRadiusUpper)

	both := ax.walkCharged(w, charge, charge)
	require.Greater(t, both.EndVBound, plain.EndVBound)
	require.Greater(t, both.LengthBound, startOnly.LengthBound, "two moved ends move the chord by more than one does")
}
