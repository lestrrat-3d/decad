package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftQuarterWallOrderRealSketchPair(t *testing.T) {
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 10)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, radius := range [2]float64{5, 5.5} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		right := s.CreatePoint(radius, 0)
		top := s.CreatePoint(0, radius)
		s.CreateLine(center, right)
		s.CreateArc(center, right, top)
		s.CreateLine(top, center)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		found := s.Profiles()
		require.Len(t, found, 1)
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0],
		sections[1], profiles[1])
	require.NoError(t, err)

	makePrism := func(t *testing.T, xMin float64) *Body {
		s, createErr := world.CreateSketch(world.XY())
		require.NoError(t, createErr)
		rect := s.CreateRectangle(xMin, 1, 6, 3)
		s.Fix(rect.A)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		found := s.Profiles()
		require.Len(t, found, 1)
		prism, buildErr := Extrude(t.Context(), New(), s, found[0],
			Distance{D: units.Millimeters(12), Dir: Along})
		require.NoError(t, buildErr)
		return prism
	}
	for _, test := range []struct {
		name  string
		xMin  float64
		admit bool
	}{
		{name: "ordered_crossings", xMin: 2, admit: true},
		{name: "left_edge_enters_arc", xMin: 5.8},
	} {
		t.Run(test.name, func(t *testing.T) {
			prism := makePrism(t, test.xMin)
			var steps uint64
			operation := &proofbound.WorkBudget{
				StepFn: func() error { steps++; return t.Context().Err() },
				ErrFn:  t.Context().Err,
			}
			wall := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
			source, ok, sourceErr := certifyLoftSourceSpanCaps(t.Context(), wall, prism, loft)
			require.NoError(t, sourceErr)
			require.True(t, ok)
			proof, ordered, orderErr := certifyLoftQuarterWallOrder(t.Context(), wall, source)
			require.NoError(t, orderErr)
			require.Equal(t, test.admit, ordered)
			require.Less(t, steps, uint64(4096))
			if !ordered {
				return
			}
			require.Same(t, prism, proof.prism)
			require.Same(t, loft, proof.loft)
			require.Equal(t, "5", proof.radius[0].RatString())
			require.Equal(t, "11/2", proof.radius[1].RatString())
			require.NotEmpty(t, proof.wallFaces[0])
			require.NotEmpty(t, proof.wallFaces[1])
			pp := prism.n.Payload.(prismPayload)
			lower := pp.Profile.Outer.Segments[proof.lower].(sectionrecord.LineSeg)
			upper := pp.Profile.Outer.Segments[proof.upper].(sectionrecord.LineSeg)
			require.Equal(t, float64(1), lower.Start.V)
			require.Equal(t, float64(3), upper.Start.V)
			require.Equal(t, "0", source.levels[0].RatString())
			require.Equal(t, "10", source.levels[1].RatString())
			require.Equal(t, "12", source.levels[2].RatString())
		})
	}

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err := certifyLoftQuarterWallOrder(canceled,
		proofbound.NewWorkBudget(canceled), loftSourceSpanCaps{})
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
