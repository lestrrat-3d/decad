package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftFitCircleRootCertificateEnclosesIdealContact(t *testing.T) {
	circle := circleSeg{
		Center: sectionrecord.Point2{}, Radius: units.Millimeters(1),
		CCW: true, TStart: 0.75, TEnd: 1,
	}
	fit := fitSplineSeg{
		Fit:    []sectionrecord.Point2{{U: 1, V: 0}, {U: 2, V: 1}},
		TStart: 0, TEnd: 1,
	}
	circleWalk, err := boundarywalk.WalkOf(circle, nil)
	require.NoError(t, err)
	fitWalk, err := boundarywalk.WalkOf(fit, freeform.NewFreeformWork())
	require.NoError(t, err)
	blend, fitT, _, err := loftFitCircleBlend(
		[]curveSegment{circle, fit},
		[]survey2d.SideWalk{{SegmentWalk: circleWalk}, {SegmentWalk: fitWalk}}, 1, 0.05,
	)
	require.NoError(t, err)
	cert, err := certifyLoftFitCircleRoot(t.Context(), fit, circle, blend.Connector.(arcSeg), fitT, 0.05, false, -1)
	require.NoError(t, err)
	require.Positive(t, cert.Param.Lo.Sign())
	require.Negative(t, new(big.Rat).Sub(cert.Param.Hi, big.NewRat(1, 1)).Sign())
	require.LessOrEqual(t, cert.Param.Lo.Cmp(cert.Param.Hi), 0)
	require.GreaterOrEqual(t, cert.SpeedUpper, math.Sqrt2)
	require.Positive(t, cert.FitFootU.Lo.Sign())
	require.Positive(t, cert.CircleFootU.Lo.Sign())

	reversedFit := fitSplineSeg{
		Fit:    []sectionrecord.Point2{{U: 2, V: 1}, {U: 1, V: 0}},
		TStart: 1, TEnd: 0,
	}
	reversedWalk, err := boundarywalk.WalkOf(reversedFit, freeform.NewFreeformWork())
	require.NoError(t, err)
	blend, fitT, _, err = loftFitCircleBlend(
		[]curveSegment{circle, reversedFit},
		[]survey2d.SideWalk{{SegmentWalk: circleWalk}, {SegmentWalk: reversedWalk}}, 1, 0.05,
	)
	require.NoError(t, err)
	cert, err = certifyLoftFitCircleRoot(t.Context(), reversedFit, circle, blend.Connector.(arcSeg), fitT, 0.05, false, -1)
	require.NoError(t, err)
	require.Positive(t, cert.Param.Lo.Sign())
	require.Negative(t, new(big.Rat).Sub(cert.Param.Hi, big.NewRat(1, 1)).Sign())
	require.GreaterOrEqual(t, cert.SpeedUpper, math.Sqrt2)
}
