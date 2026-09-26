package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChainGateDiameterRevolveNeverExceedsKnownCone(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pole := s.CreatePoint(0, 0)
	s.Fix(pole)
	s.CreateLine(pole, s.CreatePoint(4, 3))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	for _, tc := range []struct {
		name   string
		extent AngularExtent
		maxD   float64
	}{
		{"quarter", AngleExtent{A: units.Degrees(90), Dir: Along}, 5},
		{"three quarters", AngleExtent{A: units.Degrees(270), Dir: Along}, 6},
		{"full", FullRevolution{}, 6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := New()
			body, err := doc.RevolveChain(s, chains[0], axis, tc.extent)
			require.NoError(t, err)
			d, ok, err := bodyGateDiameter(t.Context(), body)
			require.NoError(t, err)
			require.True(t, ok)
			require.Greater(t, d, 0.0)
			require.LessOrEqual(t, d, tc.maxD)
			if tc.name != "full" {
				return
			}
			rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
			require.NoError(t, err)
			placed, err := body.Placed(t.Context(), rotation)
			require.NoError(t, err)
			placedD, ok, err := bodyGateDiameter(t.Context(), placed)
			require.NoError(t, err)
			require.True(t, ok)
			require.LessOrEqual(t, placedD, math.Nextafter(6, math.Inf(1)))
		})
	}
}

func TestChainGateDiameterCancelledBuildReturnsContextError(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for _, payload := range []featurePayload{chainPayload{}, chainLoftPayload{}, chainRevolvePayload{}} {
		body := &Body{payload: payload}
		_, _, err := bodyGateDiameter(ctx, body)
		require.ErrorIs(t, err, context.Canceled)
	}
}

func TestChainWalkEndpointAllowChargesComputedCircularEnds(t *testing.T) {
	for _, tc := range []struct {
		name    string
		segment CurveSegment
	}{
		{"circle fragment", CircleSeg{
			Center: Point2{}, Radius: units.Millimeters(1), CCW: true,
			TStart: 0.0625, TEnd: 0.1875,
		}},
		{"trimmed arc", ArcSeg{
			Center: Point2{}, Start: Point2{U: 1}, End: Point2{V: 1},
			TStart: 0.25, TEnd: 0.75,
		}},
		{"natural arc radial residual", ArcSeg{
			Center: Point2{}, Start: Point2{U: 1}, End: Point2{V: 1 + 1e-8},
			TStart: 0, TEnd: 1,
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			allow, ok, err := chainWalkEndpointAllow(t.Context(), []ChainRecord{{Segments: []CurveSegment{tc.segment}}})
			require.NoError(t, err)
			require.True(t, ok)
			require.Positive(t, allow)
			walk, err := walkOf(tc.segment, newFreeformWork())
			require.NoError(t, err)
			require.GreaterOrEqual(t, allow, walkEndBoundAllow(walk.startBound))
			require.GreaterOrEqual(t, allow, walkEndBoundAllow(walk.endBound))
			require.GreaterOrEqual(t, allow, arcNaturalEndRadialUpper(tc.segment))
		})
	}
}

func TestChainWalkEndpointAllowKeepsExactArcEndsAtZero(t *testing.T) {
	arc := ArcSeg{Center: Point2{}, Start: Point2{U: 1}, End: Point2{V: 1}, TEnd: 1}
	require.Zero(t, arcNaturalEndRadialUpper(arc))
	allow, ok, err := chainWalkEndpointAllow(t.Context(), []ChainRecord{{Segments: []CurveSegment{arc}}})
	require.NoError(t, err)
	require.True(t, ok)
	require.Zero(t, allow)
}
