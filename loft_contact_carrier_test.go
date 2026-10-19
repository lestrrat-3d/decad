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

func TestLoftContactOrderReadsVaryingRecordedLines(t *testing.T) {
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
	require.NotNil(t, pl.SourceIdentity)
	require.Len(t, pl.Profile0.Outer.Segments, 4)
	require.Len(t, pl.Profile1.Outer.Segments, 4)

	carrier := [2]sectionrecord.Point2{{U: 10, V: 0}, {U: 10, V: 10}}
	var bottoms, tops [][2]sectionrecord.Point2
	for i, raw := range pl.Profile0.Outer.Segments {
		lower, ok := raw.(sectionrecord.LineSeg)
		require.True(t, ok)
		upper, ok := pl.Profile1.Outer.Segments[i].(sectionrecord.LineSeg)
		require.True(t, ok)
		bottom := [2]sectionrecord.Point2{lower.Start, lower.End}
		upperEnds := [2]sectionrecord.Point2{upper.Start, upper.End}
		seam, admitted, err := loftcontact.CertifyLineSeam(t.Context(),
			proofbound.NewWorkBudget(t.Context()), bottom, upperEnds, carrier)
		require.NoError(t, err)
		if !admitted {
			continue
		}
		_, v, ok := seam.At(big.NewRat(0, 1))
		require.True(t, ok)
		if v.Cmp(big.NewRat(2, 5)) == 0 {
			bottoms = append([][2]sectionrecord.Point2{bottom}, bottoms...)
			tops = append([][2]sectionrecord.Point2{upperEnds}, tops...)
		} else {
			bottoms = append(bottoms, bottom)
			tops = append(tops, upperEnds)
		}
	}
	require.Len(t, bottoms, 2)
	require.Len(t, tops, 2)
	seams, admitted, err := loftcontact.CertifyOrderedLineSeams(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottoms[0], tops[0],
		bottoms[1], tops[1], carrier)
	require.NoError(t, err)
	require.True(t, admitted)
	for _, height := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 2), big.NewRat(1, 1)} {
		_, lower, ok := seams[0].At(height)
		require.True(t, ok)
		_, upper, ok := seams[1].At(height)
		require.True(t, ok)
		require.Negative(t, lower.Cmp(upper))
	}
	_, admitted, err = loftcontact.CertifyOrderedLineSeams(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottoms[1], tops[1],
		bottoms[0], tops[0], carrier)
	require.NoError(t, err)
	require.False(t, admitted)
	_, admitted, err = loftcontact.CertifyOrderedLineSeams(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottoms[0], tops[0],
		bottoms[0], tops[0], carrier)
	require.NoError(t, err)
	require.False(t, admitted)

	// Both moving seams stay individually transverse, but their order swaps.
	bottomA := [2]sectionrecord.Point2{{U: 8, V: 4}, {U: 12, V: 4}}
	topA := [2]sectionrecord.Point2{{U: 8, V: 6}, {U: 13, V: 6}}
	bottomB := [2]sectionrecord.Point2{{U: 8, V: 6}, {U: 12, V: 6}}
	topB := [2]sectionrecord.Point2{{U: 8, V: 4}, {U: 13, V: 4}}
	for _, source := range [][2][2]sectionrecord.Point2{{bottomA, topA}, {bottomB, topB}} {
		_, ok, err := loftcontact.CertifyLineSeam(t.Context(),
			proofbound.NewWorkBudget(t.Context()), source[0], source[1], carrier)
		require.NoError(t, err)
		require.True(t, ok)
	}
	_, admitted, err = loftcontact.CertifyOrderedLineSeams(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottomA, topA, bottomB, topB, carrier)
	require.NoError(t, err)
	require.False(t, admitted)

	// Endpoint order agrees, but a moving tilted wall crosses the other
	// seam between the caps. Sampling only the two sections would miss it.
	flat := [2]sectionrecord.Point2{{U: -1, V: 0.625}, {U: 1, V: 0.625}}
	tiltedBottom := [2]sectionrecord.Point2{{U: -1, V: 0}, {U: 1, V: 1.5}}
	tiltedTop := [2]sectionrecord.Point2{{U: -1, V: 1}, {U: 3, V: 0}}
	midCarrier := [2]sectionrecord.Point2{{U: 0, V: 0}, {U: 0, V: 1}}
	for _, source := range [][2][2]sectionrecord.Point2{{flat, flat}, {tiltedBottom, tiltedTop}} {
		_, ok, err := loftcontact.CertifyLineSeam(t.Context(),
			proofbound.NewWorkBudget(t.Context()), source[0], source[1], midCarrier)
		require.NoError(t, err)
		require.True(t, ok)
	}
	_, admitted, err = loftcontact.CertifyOrderedLineSeams(t.Context(),
		proofbound.NewWorkBudget(t.Context()), flat, flat, tiltedBottom, tiltedTop, midCarrier)
	require.NoError(t, err)
	require.False(t, admitted)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, admitted, err = loftcontact.CertifyOrderedLineSeams(canceled,
		proofbound.NewWorkBudget(canceled), bottoms[0], tops[0], bottoms[1], tops[1], carrier)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
}

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
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
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
	edges, seams, sides, admitted, err := loftcontact.CertifyCarrierOccupancy(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottom, rotated, carrier)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Zero(t, edges)
	require.Zero(t, seams)
	require.Nil(t, sides)

	tangent := carrier
	tangent[0].U, tangent[1].U = 8, 8
	edges, seams, sides, admitted, err = loftcontact.CertifyCarrierOccupancy(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottom, top, tangent)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Zero(t, edges)
	require.Zero(t, seams)
	require.Nil(t, sides)

	broken := append([]sectionrecord.LineSeg(nil), top...)
	broken[0].End.U++
	edges, seams, sides, admitted, err = loftcontact.CertifyCarrierOccupancy(t.Context(),
		proofbound.NewWorkBudget(t.Context()), bottom, broken, carrier)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Zero(t, edges)
	require.Zero(t, seams)
	require.Nil(t, sides)

	exhausted := &proofbound.WorkBudget{StepFn: func() error { return context.DeadlineExceeded }}
	edges, seams, sides, admitted, err = loftcontact.CertifyCarrierOccupancy(t.Context(), exhausted, bottom, top, carrier)
	require.ErrorIs(t, err, context.DeadlineExceeded)
	require.False(t, admitted)
	require.Zero(t, edges)
	require.Zero(t, seams)
	require.Nil(t, sides)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	edges, seams, sides, admitted, err = loftcontact.CertifyCarrierOccupancy(canceled,
		proofbound.NewWorkBudget(canceled), bottom, top, carrier)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
	require.Zero(t, edges)
	require.Zero(t, seams)
	require.Nil(t, sides)
}
