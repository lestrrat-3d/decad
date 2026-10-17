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
	loft, err := New().Loft(t.Context(), sections[0], profiles[0], sections[1], profiles[1])
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
