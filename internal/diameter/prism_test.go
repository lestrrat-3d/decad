package diameter_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestPrismStationWitnessesLiftsLineEnds checks a 2x1 all-line rectangle on
// the XY frame lists both ends of each of its four walls as stations (every corner twice) and lifts every
// station to both cap levels, with a lift allowance that is non-negative and
// tiny.
//
// Shown to fail: lifting to {Z0, Z0} instead of {Z0, Z1} leaves no station at
// the upper level.
func TestPrismStationWitnessesLiftsLineEnds(t *testing.T) {
	t.Parallel()
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := [4]sectionrecord.Point2{{U: 0, V: 0}, {U: 2, V: 0}, {U: 2, V: 1}, {U: 0, V: 1}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	pp := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Frame:   xy, Xform: r3.Identity(), Z0: 0, Z1: 5,
	}

	pts, allow, ok, err := diameter.PrismStationWitnesses(
		proofbound.NewWorkBudget(t.Context()), pp, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.True(t, ok)
	require.GreaterOrEqual(t, allow, 0.0)
	require.Less(t, allow, 1e-12)

	atLevel := map[float64]int{}
	cornerSet := map[[2]float64]struct{}{}
	for _, c := range corners {
		cornerSet[[2]float64{c.U, c.V}] = struct{}{}
	}
	for _, p := range pts {
		atLevel[p.Z]++
		require.Contains(t, cornerSet, [2]float64{p.X, p.Y})
	}
	require.Len(t, pts, 16)
	require.Equal(t, map[float64]int{0: 8, 5: 8}, atLevel)
}
