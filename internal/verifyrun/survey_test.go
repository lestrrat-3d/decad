package verifyrun

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/verifyoption"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRunSurveysReportsUndecidedCupWall(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1},
			TStart: 0, TEnd: 1,
		}
	}
	outer := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(0, 0, 100, 0), line(100, 0, 100, 60),
		line(100, 60, 0, 60), line(0, 60, 0, 0),
	}}}
	cavity, err := offset2d.OffsetProfile(proofbound.NewWorkBudget(t.Context()), outer, 1, 5, shellTol)
	require.NoError(t, err)
	for i, segment := range cavity.Outer.Segments {
		moved := segment.(sectionrecord.LineSeg)
		moved.Start.U += 0.25
		moved.End.U += 0.25
		cavity.Outer.Segments[i] = moved
	}
	bad := cupwall.View{
		Outer: outer, Cavity: cavity,
		ZOuter: 0, ZCav: 5, ZOpen: 20, Thickness: 5, Sense: cupwall.Inward,
	}
	record, err := bad.Payload(t.Context())
	require.NoError(t, err)
	body := &topo.Body{Payload: record}
	results, diags, err := runSurveys(proofbound.NewWorkBudget(t.Context()), body, verifyConfig{
		Wall: &verifyoption.WallSpec{Tool: units.Millimeters(1)}, ToolMM: 1, AllowRad: 15 * math.Pi / 180,
	})
	require.NoError(t, err)
	require.Nil(t, results.Wall.reading)
	require.Len(t, diags, 1)
	require.Equal(t, vocab.DiagUndecidedWall, diags[0].Code)
	require.Equal(t, vocab.Suspect, diags[0].Status)
	require.NotContains(t, diags[0].Message, "facetedPayload")
}
