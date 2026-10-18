package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// The source pair comes from the real public Loft producer. The contact
// proof consumes its recorded lines, not hand-written surrogate sections.
func TestLoftContactSeamReadsVaryingRecordedLines(t *testing.T) {
	world := sketch.NewWorld()
	top, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), top} {
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
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	pl := loft.n.Payload.(loftPayload)
	require.True(t, pl.AuthenticatedReconstruction)
	for _, offset := range pl.Alignment {
		require.Zero(t, offset)
	}
	require.Len(t, pl.Profile0.Outer.Segments, 4)
	require.Len(t, pl.Profile1.Outer.Segments, 4)

	carrier := [2]sectionrecord.Point2{{U: 10, V: 0}, {U: 10, V: 10}}
	budget := proofbound.NewWorkBudget(t.Context())
	parameters := map[string]int{}
	var changedLineParameter bool
	for i, lowerSegment := range pl.Profile0.Outer.Segments {
		lower, ok := lowerSegment.(sectionrecord.LineSeg)
		require.True(t, ok)
		upper, ok := pl.Profile1.Outer.Segments[i].(sectionrecord.LineSeg)
		require.True(t, ok)
		require.Equal(t, 0.0, lower.TStart)
		require.Equal(t, 1.0, lower.TEnd)
		seam, admitted, err := loftcontact.CertifyLineSeam(t.Context(), budget,
			[2]sectionrecord.Point2{lower.Start, lower.End},
			[2]sectionrecord.Point2{upper.Start, upper.End}, carrier)
		require.NoError(t, err)
		if !admitted {
			continue
		}
		s0, v0, ok := seam.At(big.NewRat(0, 1))
		require.True(t, ok)
		s1, v1, ok := seam.At(big.NewRat(1, 1))
		require.True(t, ok)
		require.Equal(t, v0.RatString(), v1.RatString())
		parameters[v0.RatString()]++
		changedLineParameter = changedLineParameter || s0.Cmp(s1) != 0
	}
	require.Equal(t, map[string]int{"2/5": 1, "3/5": 1}, parameters)
	require.True(t, changedLineParameter)

	_, admitted, err := loftcontact.CertifyLineSeam(t.Context(), budget,
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 12, V: 4}},
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 13, V: 4}},
		[2]sectionrecord.Point2{{U: 12, V: 0}, {U: 12, V: 10}})
	require.NoError(t, err)
	require.False(t, admitted)

	// Both caps cross inside the carrier, but the moving crossing rises above
	// its upper endpoint at intermediate height.
	_, admitted, err = loftcontact.CertifyLineSeam(t.Context(), budget,
		[2]sectionrecord.Point2{{U: -1, V: 0}, {U: 1, V: 1.5}},
		[2]sectionrecord.Point2{{U: -1, V: -2}, {U: 3, V: 9}},
		[2]sectionrecord.Point2{{U: 0, V: 0}, {U: 0, V: 1}})
	require.NoError(t, err)
	require.False(t, admitted)

	_, admitted, err = loftcontact.CertifyLineSeam(t.Context(), budget,
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 12, V: 4}},
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 13, V: 4}},
		[2]sectionrecord.Point2{{U: math.NaN(), V: 0}, {U: 10, V: 10}})
	require.NoError(t, err)
	require.False(t, admitted)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, admitted, err = loftcontact.CertifyLineSeam(canceled, budget,
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 12, V: 4}},
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 13, V: 4}}, carrier)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)

	exhausted := &proofbound.WorkBudget{StepFn: func() error { return context.DeadlineExceeded }}
	_, admitted, err = loftcontact.CertifyLineSeam(t.Context(), exhausted,
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 12, V: 4}},
		[2]sectionrecord.Point2{{U: 8, V: 4}, {U: 13, V: 4}}, carrier)
	require.ErrorIs(t, err, context.DeadlineExceeded)
	require.False(t, admitted)
}
