package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftFitCircleBlendKeepsSourceFitCurve(t *testing.T) {
	circle := circleSeg{
		Center: sectionrecord.Point2{U: 0, V: 0}, Radius: units.Millimeters(1),
		CCW: true, TStart: 0.75, TEnd: 1,
	}
	fit := fitSplineSeg{
		Fit:    []sectionrecord.Point2{{U: 1, V: 0}, {U: 2, V: 1}},
		TStart: 0, TEnd: 1,
	}
	segments := []curveSegment{circle, fit}
	circleWalk, err := boundarywalk.WalkOf(circle, nil)
	require.NoError(t, err)
	fitWalk, err := boundarywalk.WalkOf(fit, freeform.NewFreeformWork())
	require.NoError(t, err)
	walks := []survey2d.SideWalk{{SegmentWalk: circleWalk}, {SegmentWalk: fitWalk}}

	blend, fitT, circleT, err := loftFitCircleBlend(segments, walks, 1, 0.05)
	require.NoError(t, err)
	require.Greater(t, fitT, fit.TStart)
	require.Less(t, fitT, fit.TEnd)
	require.Greater(t, circleT, circle.TStart)
	require.Less(t, circleT, circle.TEnd)
	require.InDelta(t, 0.05, math.Hypot(blend.FB.U-blend.Connector.(arcSeg).Center.U,
		blend.FB.V-blend.Connector.(arcSeg).Center.V), 1e-8)
	require.Equal(t, []sectionrecord.Point2{{U: 1, V: 0}, {U: 2, V: 1}}, fit.Fit)

	arrivingFit := fitSplineSeg{
		Fit:    []sectionrecord.Point2{{U: 2, V: -1}, {U: 1, V: 0}},
		TStart: 0, TEnd: 1,
	}
	leavingCircle := circleSeg{
		Center: sectionrecord.Point2{}, Radius: units.Millimeters(1),
		CCW: true, TStart: 0, TEnd: 0.25,
	}
	arrivingWalk, err := boundarywalk.WalkOf(arrivingFit, freeform.NewFreeformWork())
	require.NoError(t, err)
	leavingWalk, err := boundarywalk.WalkOf(leavingCircle, nil)
	require.NoError(t, err)
	blend, fitT, circleT, err = loftFitCircleBlend(
		[]curveSegment{arrivingFit, leavingCircle},
		[]survey2d.SideWalk{{SegmentWalk: arrivingWalk}, {SegmentWalk: leavingWalk}}, 1, 0.05,
	)
	require.NoError(t, err)
	require.Greater(t, fitT, arrivingFit.TStart)
	require.Less(t, fitT, arrivingFit.TEnd)
	require.Greater(t, circleT, leavingCircle.TStart)
	require.Less(t, circleT, leavingCircle.TEnd)
	require.InDelta(t, 0.05, math.Hypot(blend.FA.U-blend.Connector.(arcSeg).Center.U,
		blend.FA.V-blend.Connector.(arcSeg).Center.V), 1e-8)
}
