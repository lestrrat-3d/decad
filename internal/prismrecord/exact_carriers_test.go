package prismrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func circleLoop(u, v, r float64, ccw bool) sectionrecord.LoopRecord {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: u, V: v}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}
	if !ccw {
		seg.CCW, seg.TStart, seg.TEnd = false, 1, 0
	}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{seg}}
}

func carrierExact(t *testing.T, pp prismrecord.Payload) []bool {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	out := make([]bool, 1+len(pp.Profile.Holes))
	for li := range out {
		exact, err := pp.CarrierExact(budget, li)
		require.NoError(t, err)
		out[li] = exact
	}
	return out
}

// TestCarrierExact checks which loops a listing names, and that a listing a copy left stale names none.
func TestCarrierExact(t *testing.T) {
	t.Parallel()
	plate := circleLoop(0, 0, 30, true)
	drawnHole := circleLoop(0, 0, 5, false)
	movedHole := circleLoop(-10.5, 18.186533479473216, 4, false)
	target := prismrecord.Payload{Profile: momentinput.Profile{Outer: plate, Holes: []sectionrecord.LoopRecord{drawnHole}}, Z1: 3, Xform: r3.Identity()}
	result := prismrecord.Payload{
		Profile:      momentinput.Profile{Outer: plate, Holes: []sectionrecord.LoopRecord{movedHole, drawnHole}},
		Z1:           3,
		Xform:        r3.Identity(),
		SectionDelta: 5e-16,
	}
	budget := proofbound.NewWorkBudget(t.Context())

	require.Equal(t, []bool{false, false, false}, carrierExact(t, result), "a displaced section with no listing states no carrier")
	require.Equal(t, []bool{true, true}, carrierExact(t, target), "an undisplaced section states every carrier")

	listing, err := prismrecord.ListExactCarriers(budget, result, target)
	require.NoError(t, err)
	require.NotNil(t, listing)
	result.ExactCarriers = listing
	require.Equal(t, []bool{true, false, true}, carrierExact(t, result), "the target's own loops keep exact carriers; the moved hole does not")

	stale := result
	stale.SectionDelta = proofbound.AbsSumUpper(result.SectionDelta, 1e-16)
	require.Equal(t, []bool{false, false, false}, carrierExact(t, stale), "a changed displacement leaves the listing stale")

	rewritten := result
	rewritten.Profile.Outer = circleLoop(0, 0, math.Nextafter(30, 31), true)
	require.Equal(t, []bool{false, false, true}, carrierExact(t, rewritten), "a rewritten loop is not the listed one")

	lifted, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	elsewhere := target
	elsewhere.Frame = lifted
	foreign, err := prismrecord.ListExactCarriers(budget, prismrecord.Payload{Profile: result.Profile, Xform: r3.Identity(), SectionDelta: 5e-16}, elsewhere)
	require.NoError(t, err)
	require.Nil(t, foreign, "an operand on another frame names other points")

	none, err := prismrecord.ListExactCarriers(budget, prismrecord.Payload{Profile: result.Profile, Xform: r3.Identity()}, target)
	require.NoError(t, err)
	require.Nil(t, none, "an undisplaced result needs no listing")
}
