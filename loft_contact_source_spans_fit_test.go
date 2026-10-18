package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftFitSplineSourceCapsRealWedge(t *testing.T) {
	world, base, top := wedgePlanes(t)
	lowerSketch, lowerProfile := wedgeSplineSketch(t, world, base)
	upperSketch, upperProfile := wedgeSplineSketch(t, world, top)
	loft, err := Loft(t.Context(), New(), lowerSketch, lowerProfile, upperSketch, upperProfile)
	require.NoError(t, err)

	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.False(t, prism.n.Doc == loft.n.Doc)
	require.Equal(t, prism.n.Origin.Producer, loft.n.Origin.Producer)

	budget := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
	spans, accepted, err := certifyLoftSourceSpanCaps(t.Context(), budget, prism, loft)
	require.NoError(t, err)
	require.True(t, accepted)
	require.True(t, spans.prism == prism && spans.loft == loft)
	require.Positive(t, spans.visits)
	require.Less(t, spans.visits, uint64(4096))
	lp := loft.n.Payload.(loftPayload)
	for cap := range 2 {
		want := lp.Profile0.Outer.Segments
		role := provenance.RoleCapStart
		if cap == 1 {
			want = lp.Profile1.Outer.Segments
			role = provenance.RoleCapEnd
		}
		require.Len(t, spans.loftCaps[cap], 3)
		var fitCount int
		for i, span := range spans.loftCaps[cap] {
			require.True(t, span.body == loft)
			require.Equal(t, i, span.segment)
			require.Equal(t, loft.n.Origin.Producer, span.capFace.Producer)
			require.Equal(t, role, span.capFace.Role)
			require.NotEmpty(t, span.wallFaces)
			for _, face := range span.wallFaces {
				require.Equal(t, loft.n.Origin.Producer, face.Producer)
			}
			fit, isFit := span.curve.(sectionrecord.FitSplineSeg)
			if !isFit {
				continue
			}
			fitCount++
			require.Len(t, fit.Fit, 5)
			require.Equal(t, want[i], span.curve)
			require.Equal(t, float64(0), span.tStart)
			require.Equal(t, float64(1), span.tEnd)
			require.False(t, span.reversed)
		}
		require.Equal(t, 1, fitCount)
	}
	for _, order := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, admitted, sourceErr := certifyLoftSourcePartition(t.Context(), order[0], order[1],
			proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, sourceErr)
		require.False(t, admitted)
		boundary, outputAdmitted, outputErr := certifyLoftOutputBoundaryForDirectShell(t.Context(),
			order[0], order[1], proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, outputErr)
		require.False(t, outputAdmitted)
		require.Empty(t, boundary.shell.triangles)
	}

	shortBudget := proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context()))
	partial, accepted, err := certifyLoftSourceSpanCaps(t.Context(), shortBudget, prism, loft)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.False(t, accepted)
	require.True(t, partial.prism == nil && partial.loft == nil)

	canceled, cancel := context.WithCancel(t.Context())
	steps := 0
	operation := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 16 {
				cancel()
			}
			return canceled.Err()
		},
		ErrFn: canceled.Err,
	}
	partial, accepted, err = certifyLoftSourceSpanCaps(canceled,
		proofbound.NewWallWorkBudgetWithOperation(4096, operation), prism, loft)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, accepted)
	require.True(t, partial.prism == nil && partial.loft == nil)
	require.Equal(t, 16, steps)
}

func TestLoftFitSplineSourcePreflightRefusesOversizedInput(t *testing.T) {
	fit := sectionrecord.FitSplineSeg{Fit: make([]sectionrecord.Point2, 65), TStart: 0, TEnd: 1}
	budget := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
	visits, accepted, err := loftSourceFitPreflight(budget, nil,
		[]sectionrecord.CurveSegment{fit}, nil)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.False(t, accepted)
	require.Zero(t, visits)
}
