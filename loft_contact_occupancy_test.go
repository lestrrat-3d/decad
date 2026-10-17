package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestLoftContactOccupancyReadsVaryingRecordedWalls(t *testing.T) {
	world := sketch.NewWorld()
	upperPlane, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), upperPlane} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		x1 := 12.0
		if i == 1 {
			x1 = 13
		}
		rect := s.CreateRectangle(8, 4, x1, 6)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := New().Loft(t.Context(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	lp := loft.n.Payload.(loftPayload)
	require.True(t, lp.AuthenticatedReconstruction)
	require.NotNil(t, lp.SourceIdentity)
	require.False(t, lp.SurfaceResult)
	require.Zero(t, lp.ConstructionDelta)
	require.Empty(t, lp.Profile0.Holes)
	require.Empty(t, lp.Profile1.Holes)
	require.Equal(t, lp.Frame0.U(), lp.Frame1.U())
	require.Equal(t, lp.Frame0.V(), lp.Frame1.V())
	for _, offset := range lp.Alignment {
		require.Zero(t, offset)
	}

	blank := internalBoxBody(t, New(), 0, 0, 10, 10, 5)
	pp := blank.n.Payload.(prismPayload)
	require.Zero(t, pp.SectionDelta)
	require.Empty(t, pp.BlendSegs)
	require.Equal(t, lp.Frame0.U(), pp.Frame.U())
	require.Equal(t, lp.Frame0.V(), pp.Frame.V())
	var carrier [2]sectionrecord.Point2
	found := 0
	for _, raw := range pp.Profile.Outer.Segments {
		line, ok := raw.(sectionrecord.LineSeg)
		require.True(t, ok)
		if line.Start.U == 10 && line.End.U == 10 {
			carrier = [2]sectionrecord.Point2{line.Start, line.End}
			found++
		}
	}
	require.Equal(t, 1, found)
	// Orient the real source wall toward increasing carrier v.
	if carrier[0].V > carrier[1].V {
		carrier[0], carrier[1] = carrier[1], carrier[0]
	}
	require.Equal(t, sectionrecord.Point2{U: 10, V: 0}, carrier[0])
	require.Equal(t, sectionrecord.Point2{U: 10, V: 10}, carrier[1])

	bottom, top := make([]sectionrecord.LineSeg, 4), make([]sectionrecord.LineSeg, 4)
	require.Len(t, lp.Profile0.Outer.Segments, len(bottom))
	require.Len(t, lp.Profile1.Outer.Segments, len(top))
	for i := range bottom {
		var ok bool
		bottom[i], ok = lp.Profile0.Outer.Segments[i].(sectionrecord.LineSeg)
		require.True(t, ok)
		top[i], ok = lp.Profile1.Outer.Segments[i].(sectionrecord.LineSeg)
		require.True(t, ok)
	}
	check := func(lower, upper []sectionrecord.LineSeg, line [2]sectionrecord.Point2) bool {
		t.Helper()
		edges, seams, sides, admitted, err := loftcontact.CertifyCarrierOccupancy(t.Context(),
			proofbound.NewWorkBudget(t.Context()), lower, upper, line)
		require.NoError(t, err)
		if !admitted {
			return false
		}
		require.Len(t, sides, 4)
		require.NotEqual(t, edges[0], edges[1])
		for _, height := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 2), big.NewRat(1, 1)} {
			_, low, ok := seams[0].At(height)
			require.True(t, ok)
			_, high, ok := seams[1].At(height)
			require.True(t, ok)
			require.Equal(t, "2/5", low.RatString())
			require.Equal(t, "3/5", high.RatString())
		}
		for _, edge := range edges {
			require.NotEqual(t, sides[edge], sides[(edge+1)%len(sides)])
		}
		return true
	}
	require.True(t, check(bottom, top, carrier))

	// A cap-level convexity check alone would accept both sections of this
	// rotating pairing. Its moving quadrilateral collapses at mid-height.
	rotated := make([]sectionrecord.LineSeg, len(bottom))
	for i := range rotated {
		rotated[i] = bottom[(i+2)%len(bottom)]
	}
	require.True(t, check(bottom, bottom, carrier))
	require.True(t, check(rotated, rotated, carrier))
	_, _, _, admitted, err := loftcontact.CertifyCarrierOccupancy(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottom, rotated, carrier)
	require.NoError(t, err)
	require.False(t, admitted)

	tangent := carrier
	tangent[0].U, tangent[1].U = 8, 8
	_, _, _, admitted, err = loftcontact.CertifyCarrierOccupancy(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottom, top, tangent)
	require.NoError(t, err)
	require.False(t, admitted)

	broken := append([]sectionrecord.LineSeg(nil), top...)
	broken[0].End.U++
	_, _, _, admitted, err = loftcontact.CertifyCarrierOccupancy(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottom, broken, carrier)
	require.NoError(t, err)
	require.False(t, admitted)

	exhausted := &proofbound.WorkBudget{StepFn: func() error { return context.DeadlineExceeded }}
	_, _, _, admitted, err = loftcontact.CertifyCarrierOccupancy(t.Context(), exhausted, bottom, top, carrier)
	require.ErrorIs(t, err, context.DeadlineExceeded)
	require.False(t, admitted)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, _, admitted, err = loftcontact.CertifyCarrierOccupancy(canceled,
		proofbound.NewWorkBudget(canceled), bottom, top, carrier)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
}
