package prismcells_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestReflectedRewindReversesLoopAndChargesTrimmedLine(t *testing.T) {
	trimmed := prismcells.LineSeg{
		Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 20, V: 0}, TStart: 0, TEnd: 0.5,
	}
	loop := prismcells.LoopRecord{Segments: []prismcells.CurveSegment{
		trimmed,
		prismcells.LineSeg{Start: sectionrecord.Point2{U: 10, V: 0}, End: sectionrecord.Point2{U: 10, V: 5}, TStart: 0, TEnd: 1},
		prismcells.LineSeg{Start: sectionrecord.Point2{U: 10, V: 5}, End: sectionrecord.Point2{U: 0, V: 5}, TStart: 0, TEnd: 1},
		prismcells.LineSeg{Start: sectionrecord.Point2{U: 0, V: 5}, End: sectionrecord.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}
	reflect := func(p sectionrecord.Point2) (sectionrecord.Point2, error) {
		return sectionrecord.Point2{U: -p.U, V: p.V}, nil
	}
	got, charge, err := prismcells.RewindLoop(proofbound.NewWorkBudget(t.Context()), loop, reflect)
	require.NoError(t, err)
	require.Len(t, got.Segments, 4)
	require.Equal(t, prismcells.LineSeg{
		Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 0, V: 5}, TStart: 0, TEnd: 1,
	}, got.Segments[0])
	require.Equal(t, prismcells.LineSeg{
		Start: sectionrecord.Point2{U: -10, V: 0}, End: sectionrecord.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1,
	}, got.Segments[3])
	walk, err := boundarywalk.WalkOf(trimmed, nil)
	require.NoError(t, err)
	wantCharge, err := prismcells.WalkChargeOf(trimmed, walk)
	require.NoError(t, err)
	require.Positive(t, wantCharge)
	require.Equal(t, math.Float64bits(wantCharge), math.Float64bits(charge))
}

func TestGroupDisjointProofRejectsCrossingAndNestedRegions(t *testing.T) {
	for _, tc := range []struct {
		name    string
		regions []momentinput.Profile
		want    bool
	}{
		{name: "disjoint", regions: []momentinput.Profile{square(0, 2), square(4, 6)}, want: true},
		{name: "crossing", regions: []momentinput.Profile{square(0, 4), square(2, 6)}},
		{name: "nested", regions: []momentinput.Profile{square(0, 6), square(2, 4)}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			proven, _, err := prismcells.ProveGroupDisjoint(t.Context(), proofbound.NewWorkBudget(t.Context()), tc.regions)
			require.NoError(t, err)
			require.Equal(t, tc.want, proven)
		})
	}
}
