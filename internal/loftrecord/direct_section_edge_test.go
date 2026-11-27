package loftrecord

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestDirectSectionLineWithinEdge(t *testing.T) {
	frame0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	frame1, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	profile := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.LineSeg{Start: momentinput.Point2{U: 0}, End: momentinput.Point2{U: 2}, TEnd: 1},
	}}}
	body := new(topo.Body)
	body.Origin.Producer = 1
	p := Payload{Profile0: profile, Profile1: profile, Frame0: frame0, Frame1: frame1,
		AuthenticatedReconstruction: true, Xform: r3.Identity(), Alignment: []int{0}}
	p.SourceIdentity = &SourceIdentity{Origin: 1, direct: sealDirectSections(body, p)}
	edge := func(a, b r3.Vec) *topo.Edge {
		return &topo.Edge{RetainedCurve: true, Curve: surfacegeom.Line3{},
			Start: &topo.Vertex{Position: a, Bound: units.Millimeters(0)},
			End:   &topo.Vertex{Position: b, Bound: units.Millimeters(0)}}
	}
	for _, tc := range []struct {
		name string
		a, b r3.Vec
		want bool
	}{
		{"contained", r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0), true},
		{"reversed target", r3.NewVec(3, 0, 0), r3.NewVec(-1, 0, 0), true},
		{"same endpoints", r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), true},
		{"short target", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), false},
		{"offset", r3.NewVec(-1, math.SmallestNonzeroFloat64, 0), r3.NewVec(3, math.SmallestNonzeroFloat64, 0), false},
		{"degenerate", r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 0), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := p.DirectSectionLineWithinEdge(t.Context(), body, 0, 0, edge(tc.a, tc.b), proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
	wrong := edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0))
	wrong.Start.Bound = units.Millimeters(1e-9)
	got, err := p.DirectSectionLineWithinEdge(t.Context(), body, 0, 0, wrong, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	got, err = p.DirectSectionLineWithinEdge(t.Context(), new(topo.Body), 0, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	got, err = p.DirectSectionLineWithinEdge(t.Context(), body, 1, 0, edge(r3.NewVec(-1, 0, 1), r3.NewVec(3, 0, 1)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, got)
	got, err = p.DirectSectionLineWithinEdge(t.Context(), body, 2, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	got, err = p.DirectSectionLineWithinEdge(t.Context(), body, 0, 1,
		edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	modified := p
	modified.ConstructionDelta = 1e-12
	got, err = modified.DirectSectionLineWithinEdge(t.Context(), body, 0, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = p.DirectSectionLineWithinEdge(canceled, body, 0, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(canceled))
	require.ErrorIs(t, err, context.Canceled)
	_, err = p.DirectSectionLineWithinEdge(nil, body, 0, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context())) //nolint:staticcheck // Exercise the nil-context refusal.
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = p.DirectSectionLineWithinEdge(t.Context(), body, 0, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), nil)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	budget := &proofbound.WorkBudget{StepFn: func() error { return proofbound.ErrWallWorkBudget }, ErrFn: func() error { return nil }}
	_, err = p.DirectSectionLineWithinEdge(t.Context(), body, 0, 0, edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), budget)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	rotated0, err := r3.NewFrame(r3.NewVec(4, 5, 6), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	rotated1, err := r3.NewFrame(r3.NewVec(5, 5, 6), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	rotated := p
	rotated.Frame0, rotated.Frame1 = rotated0, rotated1
	rotated.SourceIdentity = &SourceIdentity{Origin: body.Origin.Producer, direct: sealDirectSections(body, rotated)}
	got, err = rotated.DirectSectionLineWithinEdge(t.Context(), body, 0, 0,
		edge(r3.NewVec(4, 4, 6), r3.NewVec(4, 8, 6)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, got)
	got, err = rotated.DirectSectionLineWithinEdge(t.Context(), body, 0, 0,
		edge(r3.NewVec(4, 4, math.Nextafter(6, 7)), r3.NewVec(4, 8, math.Nextafter(6, 7))),
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	partial := p
	partial.Profile0 = momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.LineSeg{Start: momentinput.Point2{U: 0}, End: momentinput.Point2{U: 2}, TStart: 0.25, TEnd: 1},
	}}}
	partial.SourceIdentity = &SourceIdentity{Origin: body.Origin.Producer, direct: sealDirectSections(body, partial)}
	got, err = partial.DirectSectionLineWithinEdge(t.Context(), body, 0, 0,
		edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	two := p
	two.Profile0 = momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.LineSeg{Start: momentinput.Point2{U: 0}, End: momentinput.Point2{U: 2}, TEnd: 1},
		momentinput.LineSeg{Start: momentinput.Point2{U: 0, V: 1}, End: momentinput.Point2{U: 2, V: 1}, TEnd: 1},
	}}}
	two.SourceIdentity = &SourceIdentity{Origin: body.Origin.Producer, direct: sealDirectSections(body, two)}
	got, err = two.DirectSectionLineWithinEdge(t.Context(), body, 0, 1,
		edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got, "a different matching source segment cannot certify this one")
	got, err = two.DirectSectionLineWithinEdge(t.Context(), body, 0, 0,
		edge(r3.NewVec(-1, 0, 0), r3.NewVec(3, 0, 0)), proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, got)
	for _, tc := range []struct {
		name   string
		points [2]r3.Vec
		want   bool
	}{
		{"within", [2]r3.Vec{{X: 0.25}, {X: 1.75}}, true},
		{"reversed", [2]r3.Vec{{X: 1.75}, {X: 0.25}}, true},
		{"outside", [2]r3.Vec{{X: -0.01}, {X: 1}}, false},
		{"off line", [2]r3.Vec{{X: 0.25, Y: math.SmallestNonzeroFloat64}, {X: 1}}, false},
		{"same point", [2]r3.Vec{{X: 1}, {X: 1}}, false},
		{"nonfinite", [2]r3.Vec{{X: math.Inf(1)}, {X: 1}}, false},
	} {
		t.Run("points "+tc.name, func(t *testing.T) {
			got, err := p.DirectSectionLineContainsPoints(t.Context(), body, 0, 0, tc.points, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
	points := [2]r3.Vec{{X: 0.25}, {X: 1.75}}
	got, err = modified.DirectSectionLineContainsPoints(t.Context(), body, 0, 0, points, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
	_, err = p.DirectSectionLineContainsPoints(nil, body, 0, 0, points, proofbound.NewWorkBudget(t.Context())) //nolint:staticcheck // Exercise the nil-context refusal.
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = p.DirectSectionLineContainsPoints(t.Context(), body, 0, 0, points, nil)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = p.DirectSectionLineContainsPoints(canceled, body, 0, 0, points, proofbound.NewWorkBudget(canceled))
	require.ErrorIs(t, err, context.Canceled)
	_, err = p.DirectSectionLineContainsPoints(t.Context(), body, 0, 0, points, budget)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	for _, tc := range []struct {
		name   string
		points [2]r3.Vec
		want   bool
	}{
		{"endpoints", [2]r3.Vec{{}, {X: 2}}, true},
		{"reversed endpoints", [2]r3.Vec{{X: 2}, {}}, true},
		{"partial", [2]r3.Vec{{}, {X: 1}}, false},
		{"offset", [2]r3.Vec{{}, {X: 2, Y: math.SmallestNonzeroFloat64}}, false},
		{"nonfinite", [2]r3.Vec{{}, {X: math.Inf(1)}}, false},
	} {
		t.Run("matches "+tc.name, func(t *testing.T) {
			got, err := p.DirectSectionLineMatchesPoints(t.Context(), body, 0, 0, tc.points, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
