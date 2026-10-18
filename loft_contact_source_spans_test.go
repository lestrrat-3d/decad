package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftCurvedSourceCapsRealIndependentWedgeRefusesWithoutBoundaryProof(t *testing.T) {
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
		require.Equal(t, 1, len(found))
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	lp := loft.n.Payload.(loftPayload)
	for _, loop := range []sectionrecord.LoopRecord{lp.Profile0.Outer, lp.Profile1.Outer} {
		require.Equal(t, 3, len(loop.Segments))
		var lines, arcs int
		for _, segment := range loop.Segments {
			switch segment.(type) {
			case sectionrecord.LineSeg:
				lines++
			case sectionrecord.ArcSeg:
				arcs++
			}
		}
		require.Equal(t, 2, lines)
		require.Equal(t, 1, arcs)
	}

	prismSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.Zero(t, prism.n.Origin.Producer)
	require.Zero(t, loft.n.Origin.Producer)
	require.False(t, prism.n.Doc == loft.n.Doc)
	certificate, certified, certErr := certifyLoftSourceSpanCaps(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		prism, loft)
	require.NoError(t, certErr)
	require.True(t, certified)
	require.True(t, certificate.prism == prism && certificate.loft == loft)
	require.Equal(t, [3]string{"0", "10", "12"}, [3]string{
		certificate.levels[0].RatString(), certificate.levels[1].RatString(),
		certificate.levels[2].RatString(),
	})
	require.Positive(t, certificate.visits)
	require.Less(t, certificate.visits, uint64(4096))
	for cap := range 2 {
		require.Equal(t, 4, len(certificate.prismCaps[cap]))
		require.Equal(t, 3, len(certificate.loftCaps[cap]))
		var arcsWithWalls int
		for _, span := range certificate.loftCaps[cap] {
			require.True(t, span.body == loft)
			require.Equal(t, float64(0), span.tStart)
			require.Equal(t, float64(1), span.tEnd)
			if _, arc := span.curve.(sectionrecord.ArcSeg); arc {
				require.Positive(t, len(span.wallFaces))
				arcsWithWalls++
			}
		}
		require.Equal(t, 1, arcsWithWalls)
	}
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			source, admitted, sourceErr := certifyLoftSourcePartition(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, sourceErr)
			require.False(t, admitted)
			require.Zero(t, source.sourceVisits)

			boundary, accepted, boundaryErr := certifyLoftOutputBoundary(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, boundaryErr)
			require.False(t, accepted)
			require.Zero(t, len(boundary.shell.vertices))
		})
	}
}
