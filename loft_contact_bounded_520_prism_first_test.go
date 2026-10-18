package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftBoundedProfilesRealRightXPrismFirst(t *testing.T) {
	prism, loft := loftBounded520Operands(t)
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	checkLoftBoundedCapCertificates(t, boundary.source)
	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(520, 1)))
	require.Less(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
	t.Logf("totalOutputVisits=%d", proof.totalOutputVisits)
}

func TestLoftBoundedProfilesRealRightXLoftFirst(t *testing.T) {
	prism, loft := loftBounded520Operands(t)
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), loft, prism, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	checkLoftBoundedCapCertificates(t, boundary.source)
	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(520, 1)))
	require.Less(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
	t.Logf("totalOutputVisits=%d", proof.totalOutputVisits)
}

func loftBounded520Operands(t *testing.T) (*Body, *Body) {
	t.Helper()
	world := sketch.NewWorld()
	base := world.XY()
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, coords := range [2][4]float64{
		{8, 4, 12, 6}, {8, 4, 13, 6},
	} {
		plane := base
		if i == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		rect := s.CreateRectangle(coords[0], coords[1], coords[2], coords[3])
		s.Fix(rect.A)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	d := New()
	loft, err := Loft(t.Context(), d, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(0, 0, 10, 10)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), d, prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	return prism, loft
}

func checkLoftBoundedCapCertificates(t *testing.T, source loftSourcePartition) {
	t.Helper()
	require.Greater(t, source.sourceVisits, uint64(0))
	require.LessOrEqual(t, source.sourceVisits, uint64(4_096))
	require.True(t, source.prepared[0].scene != source.prepared[1].scene)
	for capIndex, cap := range source.prepared {
		require.True(t, cap.scene != nil)
		require.Equal(t, capIndex, cap.capIndex)
		require.True(t, cap.level == source.capLevels[capIndex])
		require.Equal(t, 3, len(cap.raw))
		for _, cell := range cap.raw {
			require.True(t, cell != nil)
			require.True(t, cell.Sketch() == cap.scene)
			require.False(t, cell.IsStale())
		}
	}
	t.Logf("sourceVisits=%d", source.sourceVisits)
}

func TestLoftBoundedProfilesGroundingCancelsInFlight(t *testing.T) {
	world := sketch.NewWorld()
	scene, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	for i := range 8 {
		scene.CreatePoint(float64(i), 0)
	}
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 3 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	err = groundLoftPartitionScene(budget, scene)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 3, steps)
	for i, point := range scene.Points() {
		require.Equal(t, i < 2, point.IsFixed(), "point %d", i)
	}
}
