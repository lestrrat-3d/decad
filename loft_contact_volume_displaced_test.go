package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftVolumeComposeRealDisplacedNonjunction(t *testing.T) {
	world := sketch.NewWorld()
	epsilon := math.Ldexp(1, -52)
	epsilonRat := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 52))
	seamWorld := new(big.Rat).Add(big.NewRat(1, 1), epsilonRat)
	seamHeld, exact := seamWorld.Float64()
	require.True(t, exact)
	require.Equal(t, 1+epsilon, seamHeld)
	wantDisplaced := new(big.Rat).Add(big.NewRat(5, 2), epsilonRat)
	displacedHeld, exact := wantDisplaced.Float64()
	require.False(t, exact)
	require.Equal(t, float64(2.5), displacedHeld)
	frame, err := r3.NewFrame(r3.Vec{X: 1 + epsilon}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)

	doc := New()
	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	prismRect := prismSketch.CreateRectangle(-9, 0, 0, 10)
	prismSketch.Fix(prismRect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), doc, prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)

	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{base, upper} {
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		right := 1.0
		if i == 1 {
			right = 1.5
		}
		rect := s.CreateRectangle(-0.5, 4, right, 6)
		s.Fix(rect.A)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)

	pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{
		{loftOperandPrismFirst, prism, loft},
		{loftOperandLoftFirst, loft, prism},
	} {
		t.Run(tc.name, func(t *testing.T) {
			operation := proofbound.NewWorkBudget(t.Context())
			boundary, admitted, err := certifyLoftOutputBoundary(
				t.Context(), tc.first, tc.second, operation)
			require.NoError(t, err, "boundary")
			require.True(t, admitted, "boundary")
			proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
			require.NoError(t, err, "composer")
			require.True(t, admitted, "composer")
			require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(460, 1)))
			displaced := false
			for _, vertex := range boundary.shell.vertices {
				if vertex.point.u.Cmp(big.NewRat(3, 2)) == 0 &&
					vertex.point.v.Cmp(big.NewRat(6, 1)) == 0 {
					require.Equal(t, wantDisplaced.RatString(), vertex.world[0].RatString())
					require.Equal(t, displacedHeld, vertex.held.X)
					require.Positive(t, vertex.bound)
					displaced = true
				}
			}
			require.True(t, displaced, "nonjunction displacement")
			require.Greater(t, proof.symDiffUpper, float64(0))
			require.Less(t, proof.symDiffUpper, float64(460))
			require.Positive(t, boundary.source.sourceVisits)
			require.LessOrEqual(t, boundary.source.sourceVisits, uint64(4096))
			require.LessOrEqual(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
			require.LessOrEqual(t, len(boundary.shell.vertices), 512)
			require.LessOrEqual(t, len(boundary.shell.triangles), 512)
			t.Logf("source visits=%d output visits=%d facets=%d bound=%g",
				boundary.source.sourceVisits, proof.totalOutputVisits,
				len(boundary.shell.triangles), proof.symDiffUpper)
		})
	}

	t.Run("charged refusal", func(t *testing.T) {
		operation := proofbound.NewWorkBudget(t.Context())
		boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft, operation)
		require.NoError(t, err)
		require.True(t, admitted)
		limited := proofbound.NewWallWorkBudgetWithOperation(16, operation)
		proof, admitted, err := loftVolumeCompose(t.Context(), limited, pp, lp, boundary)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
		require.False(t, admitted)
		require.Zero(t, proof.totalOutputVisits)
	})

	t.Run("inflight cancellation", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		defer cancel()
		parent := proofbound.NewWorkBudget(ctx)
		composing := false
		var composeVisits uint64
		operation := &proofbound.WorkBudget{
			StepFn: func() error {
				if composing {
					composeVisits++
					if composeVisits == 32 {
						cancel()
					}
				}
				return parent.Step()
			},
			ErrFn: parent.Err,
		}
		boundary, admitted, err := certifyLoftOutputBoundary(ctx, prism, loft, operation)
		require.NoError(t, err)
		require.True(t, admitted)
		composing = true
		proof, admitted, err := loftVolumeCompose(ctx, operation, pp, lp, boundary)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, admitted)
		require.Zero(t, proof.totalOutputVisits)
		require.Equal(t, uint64(32), composeVisits)
	})
}
