package verifyrun

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
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

func TestBodyChecksKeepCheapRequestsSerial(t *testing.T) {
	// These records exercise scheduling only; real producer/consumer coverage
	// lives in TestParallelBodyWallSurveysPreserveReadingsAndOrder.
	prism := prismrecord.Payload{Profile: momentinput.Profile{
		Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{TEnd: 1}}},
	}}
	body := topo.Body{Solid: true, Payload: prism}
	other := topo.Body{Solid: true, Payload: prism}
	bodies := []*topo.Body{&body, &other}
	wall := verifyConfig{Wall: &verifyoption.WallSpec{Tool: units.Millimeters(1)}}
	ctx := orderedwork.WithWorkers(t.Context(), 12)
	require.Equal(t, orderedwork.MaxEachWorkers, bodyCheckWorkers(ctx, bodies, wall))
	require.Equal(t, 1, bodyCheckWorkers(orderedwork.WithWorkers(ctx, 1), bodies, wall))
	require.Equal(t, 1, bodyCheckWorkers(ctx, bodies, verifyConfig{}))
	require.Equal(t, 1, bodyCheckWorkers(ctx, bodies[:1], wall))

	other.Kind = topo.BodySheet
	require.Equal(t, 1, bodyCheckWorkers(ctx, bodies, wall))
	other.Kind = topo.BodySolid
	displaced := prism
	displaced.SectionDelta = 1e-9
	other.Payload = displaced
	require.Equal(t, 1, bodyCheckWorkers(ctx, bodies, wall))
	other.Payload = nil
	require.Equal(t, 1, bodyCheckWorkers(ctx, bodies, wall))
	freeform := prism
	freeform.Profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{sectionrecord.SplineSeg{}}}}
	other.Payload = freeform
	require.Equal(t, 1, bodyCheckWorkers(ctx, bodies, wall))
}

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
