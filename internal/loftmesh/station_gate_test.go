package loftmesh_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// parabolaSection is the region under B(s) = (4(1−s), 8s(1−s)), shifted by
// (du, 0), closed by the chord along v = 0. Its area is two thirds of the
// control triangle's: 16/3.
func parabolaSection(du float64) momentinput.Profile {
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u + du, V: v} }
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: pt(0, 0), End: pt(4, 0), TStart: 0, TEnd: 1},
		sectionrecord.NURBSSeg{
			Degree:  2,
			Control: []sectionrecord.Point2{pt(4, 0), pt(2, 4), pt(0, 0)},
			Knots:   []float64{0, 0, 0, 1, 1, 1},
			Weights: []float64{1, 1, 1},
			TStart:  0, TEnd: 1,
		},
	}}}
}

// TestLoftChordTargetReadsFeatureSize pins the chord target
// (agent-docs/loft-gear-bounds-design.md §5) on an exact LineSeg/NURBSSeg
// record lofted to a copy of itself: the record's own area is 16/3, the
// perimeter bound encloses the true boundary length within 1e-3 of it, and
// StationCapGate's target is ChordFraction·A/P to two ulps. Moving the
// record 10 of its own widths from the sketch origin leaves the target alone,
// which the coordinate-envelope rule would not.
func TestLoftChordTargetReadsFeatureSize(t *testing.T) {
	t.Parallel()
	const n = 200_000
	curve := 0.0
	for k := range n {
		s0, s1 := float64(k)/n, float64(k+1)/n
		curve += math.Hypot(4*(s1-s0), 8*(s1*(1-s1)-s0*(1-s0)))
	}
	pl0 := sectionrecord.PlaneRecord{Origin: r3.Vec{}, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	pl1 := sectionrecord.PlaneRecord{Origin: r3.NewVec(0, 0, 10), U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	read := func(du float64) float64 {
		p := parabolaSection(du)
		area, err := p.FalsifyLoftRecordedArea(16.0/3, freeform.NewFreeformWork())
		require.NoError(t, err)
		require.InEpsilon(t, 16.0/3, area, 1e-15, "the record's own area integral is the closed form")
		offsets, walks0, walks1, err := loftmesh.ValidateRecordWalks(p, p, pl0, pl1, nil,
			freeform.NewFreeformWork(), freeform.NewFreeformWork())
		require.NoError(t, err)
		perimeter := loftmesh.PerimeterUpper(p, walks0)
		require.GreaterOrEqual(t, perimeter, 4+curve, "the perimeter bound encloses the true boundary length")
		require.InEpsilon(t, 4+curve, perimeter, 1e-3)

		target, err := loftmesh.StationCapGate(p, p, [2]float64{area, area}, offsets, walks0, walks1)
		require.NoError(t, err)
		want := loftmesh.ChordFraction * area / perimeter
		require.InDelta(t, want, target, 2*(math.Nextafter(want, math.Inf(1))-want), "the target is fraction·A/P")
		return target
	}
	here, far := read(0), read(40)
	require.InEpsilon(t, here, far, 1e-9, "moving the section leaves its feature size alone")

	_, err := loftmesh.ChordTarget(0, 4+curve, 16.0/3, 4+curve)
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a zero area has no feature size")
}

// TestLoftStationWalkRefusesAtTheRaisedWorkLimit pins R7's raise
// (agent-docs/loft-gear-bounds-design.md §7) on the parabola record: after
// ValidateLoftRecords raises each fresh counter past FreeformWorkLimit, a
// station walk left half its own cost below the raised ceiling refuses
// there, spends its counter to exactly that ceiling, and names it.
//
// Shown to fail: without the raise in ValidateLoftRecords the counter stays
// at FreeformWorkLimit and the room assertion fails.
func TestLoftStationWalkRefusesAtTheRaisedWorkLimit(t *testing.T) {
	t.Parallel()
	p := parabolaSection(0)
	areas := [2]float64{16.0 / 3, 16.0 / 3}
	pl0 := sectionrecord.PlaneRecord{Origin: r3.Vec{}, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	pl1 := sectionrecord.PlaneRecord{Origin: r3.NewVec(0, 0, 10), U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	validate := func() ([]int, [][]survey2d.SegmentWalk, [][]survey2d.SegmentWalk, float64, *freeform.FreeformWork, *freeform.FreeformWork) {
		work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
		offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(p, p, pl0, pl1, nil, areas, work0, work1)
		require.NoError(t, err)
		return offsets, walks0, walks1, target, work0, work1
	}

	offsets, walks0, walks1, target, work0, work1 := validate()
	before0, before1 := work0.Spent, work1.Spent
	_, _, _, _, err := loftmesh.PairRecords(p, p, offsets, walks0, walks1, target, work0, work1) //nolint:dogsled // only the counters matter here.
	require.NoError(t, err)
	cost0, cost1 := work0.Spent-before0, work1.Spent-before1
	require.Positive(t, cost0)
	require.Positive(t, cost1)

	offsets, walks0, walks1, target, work0, work1 = validate()
	require.Greater(t, work0.WorkLimit(), freeform.FreeformWorkLimit+cost0, "the raise must leave room past the default ceiling")
	require.Greater(t, work1.WorkLimit(), freeform.FreeformWorkLimit+cost1, "the raise must leave room past the default ceiling")
	work0.Spent = work0.WorkLimit() - cost0/2
	work1.Spent = work1.WorkLimit() - cost1/2
	require.Greater(t, work0.Spent, freeform.FreeformWorkLimit, "the walk runs past the default ceiling")

	_, _, _, _, err = loftmesh.PairRecords(p, p, offsets, walks0, walks1, target, work0, work1) //nolint:dogsled // only the refusal matters here.
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	refused := work0
	if work1.Spent == work1.WorkLimit() {
		refused = work1
	}
	require.Equal(t, refused.WorkLimit(), refused.Spent, "the refused counter is spent to its raised limit")
	require.ErrorContains(t, err, fmt.Sprintf("fixed work budget of %d", refused.WorkLimit()),
		"the refusal names the raised ceiling, not the default")
}
