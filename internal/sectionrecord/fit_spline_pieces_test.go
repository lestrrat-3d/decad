package sectionrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestFitSplinePieces(t *testing.T) {
	fit := func(n int) sectionrecord.FitSplineSeg {
		return sectionrecord.FitSplineSeg{Fit: make([]sectionrecord.Point2, n)}
	}
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{}, sectionrecord.ArcSeg{}, fit(15), fit(15),
		sectionrecord.SplineSeg{Control: make([]sectionrecord.Point2, 45)},
	}}
	require.Equal(t, uint64(28), sectionrecord.FitSplinePieces(loop))
	require.Equal(t, uint64(0), sectionrecord.FitSplinePieces(
		sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{}}}))
	require.Equal(t, uint64(30), sectionrecord.FitSplinePieces(loop, sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{fit(3)}}))
}
