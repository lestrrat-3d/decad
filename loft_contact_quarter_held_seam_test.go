package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftQuarterOriginalHeldY1RealProducer(t *testing.T) {
	ctx := t.Context()
	world := sketch.NewWorld()
	doc := New()
	upper, err := world.CreateOffsetPlane(world.XY(), 10)
	require.NoError(t, err)

	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, radius := range [2]float64{5, 5.5} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		right := s.CreatePoint(radius, 0)
		top := s.CreatePoint(0, radius)
		s.CreateLine(center, right)
		s.CreateArc(center, right, top)
		s.CreateLine(top, center)
		_, createErr = s.Solve(ctx)
		require.NoError(t, createErr)
		found := s.Profiles()
		require.Len(t, found, 1)
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	loft, err := Loft(ctx, doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	require.Same(t, &doc.n, loft.n.Doc)

	prismSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(ctx)
	require.NoError(t, err)
	prismProfiles := prismSketch.Profiles()
	require.Len(t, prismProfiles, 1)
	require.True(t, prismProfiles[0].Valid)
	prism, err := Extrude(ctx, doc, prismSketch, prismProfiles[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.Same(t, &doc.n, prism.n.Doc)

	lp, ok := loft.n.Payload.(loftPayload)
	require.True(t, ok)
	require.True(t, lp.AuthenticatedReconstruction)
	require.NotNil(t, lp.SourceIdentity)
	require.NotEmpty(t, lp.Proof)
	pp, ok := prism.n.Payload.(prismPayload)
	require.True(t, ok)
	carrierCount, carrierY := 0, float64(0)
	for _, raw := range pp.Profile.Outer.Segments {
		line, isLine := raw.(sectionrecord.LineSeg)
		if !isLine || line.Start.V != 1 || line.End.V != 1 {
			continue
		}
		require.True(t, line.Start.U == 2 && line.End.U == 6 ||
			line.Start.U == 6 && line.End.U == 2)
		carrierCount++
		carrierY = line.Start.V
	}
	require.Equal(t, 1, carrierCount)
	arcCount, arcSegment := 0, -1
	for i, raw := range lp.Profile0.Outer.Segments {
		if _, isArc := raw.(sectionrecord.ArcSeg); isArc {
			arcCount++
			arcSegment = i
		}
	}
	require.Equal(t, 1, arcCount)

	var visits uint64
	operation := &proofbound.WorkBudget{
		StepFn: func() error { visits++; return ctx.Err() },
		ErrFn:  ctx.Err,
	}
	budget := proofbound.NewWallWorkBudgetWithOperation(65_536, operation)
	report, accepted, err := loftcontact.CertifyQuarterHeldSeam(ctx, budget, &loft.n, arcSegment, carrierY)
	require.NoError(t, err)
	if !accepted {
		t.Fatalf("held y=1 refused: reason=%s cell=%d side=%d triangle=%d stations=%d pieces=%d visits=%d verts=%d tris=%d",
			report.Reason, report.Cell, report.Side, report.Triangle, report.Stations,
			report.Pieces, visits, len(lp.Verts), len(lp.Tris))
	}
	require.LessOrEqual(t, len(lp.Verts), 512)
	require.LessOrEqual(t, len(lp.Tris), 512)
	require.Greater(t, report.Stations, 2)
	require.GreaterOrEqual(t, report.Pieces, 2)
	require.Greater(t, visits, uint64(0))
	require.LessOrEqual(t, visits, uint64(65_536))
	require.True(t, report.BottomX.Cmp(big.NewRat(2, 1)) > 0)
	require.True(t, report.BottomX.Cmp(big.NewRat(6, 1)) < 0)
	require.True(t, report.TopX.Cmp(report.BottomX) > 0)
	require.True(t, report.TopX.Cmp(big.NewRat(6, 1)) < 0)
	t.Logf("held y=1: verts=%d tris=%d stations=%d pieces=%d visits=%d bottom_bits=%d/%d top_bits=%d/%d",
		len(lp.Verts), len(lp.Tris), report.Stations, report.Pieces, visits,
		report.BottomX.Num().BitLen(), report.BottomX.Denom().BitLen(),
		report.TopX.Num().BitLen(), report.TopX.Denom().BitLen())

	t.Run("facet limit refuses before scan", func(t *testing.T) {
		oversized := lp
		oversized.Verts = make([]r3.Vec, 513)
		loft.n.Payload = oversized
		defer func() { loft.n.Payload = lp }()
		steps := 0
		one := &proofbound.WorkBudget{
			StepFn: func() error { steps++; return nil },
			ErrFn:  func() error { return nil },
		}
		got, ok, err := loftcontact.CertifyQuarterHeldSeam(t.Context(), one, &loft.n, arcSegment, carrierY)
		require.NoError(t, err)
		require.False(t, ok)
		require.Equal(t, "facet_limit", got.Reason)
		require.Equal(t, 1, steps) // live-body check only; no mesh visit
	})

	t.Run("altered original role refuses", func(t *testing.T) {
		altered := lp
		altered.Side = append([]uint8(nil), lp.Side...)
		changed := false
		for i, role := range lp.Cell {
			if lp.Segment[role[0]][role[1]] == arcSegment {
				altered.Side[i] = 2
				changed = true
				break
			}
		}
		require.True(t, changed)
		loft.n.Payload = altered
		defer func() { loft.n.Payload = lp }()
		got, ok, err := loftcontact.CertifyQuarterHeldSeam(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			&loft.n, arcSegment, carrierY)
		require.NoError(t, err)
		require.False(t, ok)
		require.Equal(t, "wall_role", got.Reason)
	})

	t.Run("shared work limit refuses", func(t *testing.T) {
		_, ok, err := loftcontact.CertifyQuarterHeldSeam(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context())),
			&loft.n, arcSegment, carrierY)
		require.False(t, ok)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	})

	t.Run("in flight cancellation stops", func(t *testing.T) {
		cancelCtx, cancel := context.WithCancel(t.Context())
		defer cancel()
		steps := 0
		operation := &proofbound.WorkBudget{
			StepFn: func() error {
				steps++
				if steps == 100 {
					cancel()
				}
				return cancelCtx.Err()
			},
			ErrFn: cancelCtx.Err,
		}
		_, ok, err := loftcontact.CertifyQuarterHeldSeam(cancelCtx,
			proofbound.NewWallWorkBudgetWithOperation(65_536, operation), &loft.n, arcSegment, carrierY)
		require.False(t, ok)
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, 100, steps)
	})

	t.Run("live copied producer refuses", func(t *testing.T) {
		copied, copyErr := Copy(t.Context(), loft)
		require.NoError(t, copyErr)
		copyPayload, payloadOK := copied.n.Payload.(loftPayload)
		require.True(t, payloadOK)
		require.True(t, copyPayload.AuthenticatedReconstruction)
		require.Same(t, lp.SourceIdentity, copyPayload.SourceIdentity)
		require.NotEqual(t, loft.n.Origin.Producer, copied.n.Origin.Producer)
		require.True(t, copied.n.Doc.IsLive(&copied.n))
		got, ok, readErr := loftcontact.CertifyQuarterHeldSeam(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			&copied.n, arcSegment, carrierY)
		require.NoError(t, readErr)
		require.False(t, ok)
		require.Equal(t, "source_identity", got.Reason)
	})

	t.Run("rewrite markers refuse", func(t *testing.T) {
		altered := lp
		altered.BlendSegs = []map[int]struct{}{{arcSegment: {}}}
		loft.n.Payload = altered
		defer func() { loft.n.Payload = lp }()
		got, ok, readErr := loftcontact.CertifyQuarterHeldSeam(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			&loft.n, arcSegment, carrierY)
		require.NoError(t, readErr)
		require.False(t, ok)
		require.Equal(t, "source_identity", got.Reason)
	})

	t.Run("retired original refuses", func(t *testing.T) {
		require.NoError(t, Remove(loft))
		got, ok, readErr := loftcontact.CertifyQuarterHeldSeam(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			&loft.n, arcSegment, carrierY)
		require.NoError(t, readErr)
		require.False(t, ok)
		require.Equal(t, "live_original", got.Reason)
	})
}
