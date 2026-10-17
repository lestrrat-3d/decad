package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestMappedArcRadialTimeBandTwoExactTimeCrossings(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	start := s.CreatePoint(2, 0)
	end := s.CreatePoint(0, 2)
	arc := s.CreateArc(center, start, end)
	segment := arcSeg{Center: planePoint{}, Start: planePoint{U: 2},
		End: planePoint{V: 2}, TStart: 0, TEnd: 1}
	zero, one := lfZero(), lfOne()
	first := loftFaceMap{Constant: loftFaceRatVec{zero, zero, zero},
		Q0: loftFaceRatVec{zero, zero, zero},
		QU: loftFaceRatVec{zero, one, zero},
		QV: loftFaceRatVec{zero, zero, one},
		G0: one, GU: zero, GV: zero}
	second := loftFaceMap{Constant: loftFaceRatVec{one, zero, zero},
		Q0: loftFaceRatVec{zero, zero, zero},
		QU: loftFaceRatVec{zero, zero, one},
		QV: loftFaceRatVec{zero, big.NewRat(-1, 1), zero},
		G0: one, GU: zero, GV: zero}
	trim := &loftFacesTrimRecord{
		Source: &pointSectionRecord{
			Profile:   profileRecord{Outer: loopRecord{Segments: []curveSegment{segment}}},
			Source:    &pointSectionSource{Entities: []sketch.Entity{arc}},
			SourceBox: [4]float64{-3, 3, -3, 3},
		},
		Maps: []loftFaceMap{first, second},
	}
	meridian := revolveSectionSpan{Start: planePoint{V: 1.5}, End: planePoint{U: 1, V: 1.5}}
	_, certified := revolveloftfaces.MappedArcFiniteSide(context.Background(), &loftFacesTrimRecord{}, 0, meridian)
	require.False(t, certified)
	got, certified := revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.True(t, certified)
	require.Same(t, arc, got.Arc)
	require.Equal(t, 0, got.Segment)
	require.Equal(t, 0, got.Band)
	for i, want := range [...]string{"7/4", "-9/4", "7/4"} {
		require.Equal(t, want, got.Q[i].RatString())
	}
	require.Zero(t, got.X[0].Sign())
	require.Equal(t, 0, got.X[1].Cmp(lfOne()))
	segment.TStart, segment.TEnd = 1, 0
	trim.Source.Profile.Outer.Segments[0] = segment
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.True(t, certified)
	segment.Start.U = 2.1
	trim.Source.Profile.Outer.Segments[0] = segment
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.False(t, certified)
	segment.Start.U = 2
	trim.Source.Profile.Outer.Segments[0] = segment

	// A shear breaks the all-time radial identity even though the cap still
	// maps every sampled endpoint to a finite point.
	trim.Maps[1].QV[2] = big.NewRat(1, 100)
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.False(t, certified)
	trim.Maps[1].QV[2] = zero
	require.True(t, s.RemoveEntity(arc))
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.False(t, certified)
}
