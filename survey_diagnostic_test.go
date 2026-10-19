package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRunSurveysReportsUndecidedCupWall(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) curveSegment {
		return lineSeg{
			Start: planePoint{U: u0, V: v0}, End: planePoint{U: u1, V: v1},
			TStart: 0, TEnd: 1,
		}
	}
	outer := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		line(0, 0, 100, 0), line(100, 0, 100, 60),
		line(100, 60, 0, 60), line(0, 60, 0, 0),
	}}}
	cavity, err := offset2d.OffsetProfile(proofbound.NewWorkBudget(t.Context()), outer, 1, 5, shellTol)
	require.NoError(t, err)
	for i, segment := range cavity.Outer.Segments {
		moved := segment.(lineSeg)
		moved.Start.U += 0.25
		moved.End.U += 0.25
		cavity.Outer.Segments[i] = moved
	}
	bad := cupView{
		Outer: outer, Cavity: cavity,
		ZOuter: 0, ZCav: 5, ZOpen: 20, Thickness: 5, Sense: cupwall.Inward,
	}
	record, err := bad.Payload(t.Context())
	require.NoError(t, err)
	body := bornBody(&Body{n: topo.Body{Payload: record}})
	results, diags, err := runSurveys(proofbound.NewWorkBudget(t.Context()), body, verifyConfig{
		Wall: &wallSpec{Tool: units.Millimeters(1)}, ToolMM: 1, AllowRad: 15 * math.Pi / 180,
	})
	require.NoError(t, err)
	require.Nil(t, results.Wall.reading)
	require.Len(t, diags, 1)
	require.Equal(t, DiagUndecidedWall, diags[0].Code)
	require.Equal(t, Suspect, diags[0].Status)
	require.NotContains(t, diags[0].Message, "facetedPayload")
}
