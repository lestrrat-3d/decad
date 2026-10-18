package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftQuarterHeldPairRealProducerGate(t *testing.T) {
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
	prismSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(ctx)
	require.NoError(t, err)
	found := prismSketch.Profiles()
	require.Len(t, found, 1)
	require.True(t, found[0].Valid)
	prism, err := Extrude(ctx, doc, prismSketch, found[0], Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.Same(t, &doc.n, loft.n.Doc)
	require.Same(t, &doc.n, prism.n.Doc)
	lp, ok := loft.n.Payload.(loftPayload)
	require.True(t, ok)
	pp, ok := prism.n.Payload.(prismPayload)
	require.True(t, ok)
	arc := -1
	for i, raw := range lp.Profile0.Outer.Segments {
		if _, yes := raw.(sectionrecord.ArcSeg); yes {
			require.Equal(t, -1, arc)
			arc = i
		}
	}
	require.NotEqual(t, -1, arc)
	carriers := map[float64]int{}
	for _, raw := range pp.Profile.Outer.Segments {
		line, yes := raw.(sectionrecord.LineSeg)
		if !yes || line.Start.V != line.End.V {
			continue
		}
		require.True(t, line.Start.U == 2 && line.End.U == 6 || line.Start.U == 6 && line.End.U == 2)
		carriers[line.Start.V]++
	}
	require.Equal(t, map[float64]int{1: 1, 3: 1}, carriers)
	var visits uint64
	operation := &proofbound.WorkBudget{StepFn: func() error { visits++; return ctx.Err() }, ErrFn: ctx.Err}
	budget := proofbound.NewWallWorkBudgetWithOperation(65_536, operation)
	for _, y := range []float64{1, 3} {
		report, accepted, readErr := loftcontact.CertifyQuarterHeldSeam(ctx, budget, &loft.n, arc, y)
		require.NoError(t, readErr)
		if !accepted {
			t.Fatalf("held y=%g refused: reason=%s cell=%d side=%d triangle=%d stations=%d pieces=%d visits=%d verts=%d tris=%d",
				y, report.Reason, report.Cell, report.Side, report.Triangle, report.Stations,
				report.Pieces, visits, len(lp.Verts), len(lp.Tris))
		}
		t.Logf("held y=%g: stations=%d pieces=%d visits=%d", y, report.Stations, report.Pieces, visits)
	}
	beforePair := visits
	pair, accepted, err := loftcontact.CertifyQuarterHeldPair(ctx, budget, &prism.n, &loft.n)
	require.NoError(t, err)
	if !accepted {
		t.Fatalf("held pair refused: reason=%s visits=%d", pair.Reason, visits)
	}
	require.Equal(t, 4, pair.Y1.Pieces)
	require.Equal(t, 8, pair.Y3.Pieces)
	t.Logf("held pair accepted: cumulative visits=%d", visits)
	pairVisits := visits - beforePair

	refusesPrism := func(t *testing.T, name string, alter func(*prismPayload)) {
		t.Helper()
		t.Run(name, func(t *testing.T) {
			changed := pp
			changed.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), pp.Profile.Outer.Segments...)
			alter(&changed)
			prism.n.Payload = changed
			defer func() { prism.n.Payload = pp }()
			got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
				proofbound.NewWorkBudget(t.Context()), &prism.n, &loft.n)
			require.NoError(t, readErr)
			require.False(t, accepted)
			require.Equal(t, "prism_record", got.Reason)
		})
	}
	refusesPrism(t, "missing edge", func(p *prismPayload) {
		p.Profile.Outer.Segments = p.Profile.Outer.Segments[:3]
	})
	refusesPrism(t, "extra edge", func(p *prismPayload) {
		p.Profile.Outer.Segments = append(p.Profile.Outer.Segments, p.Profile.Outer.Segments[0])
	})
	refusesPrism(t, "partial edge", func(p *prismPayload) {
		line := p.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
		line.TEnd = 0.5
		p.Profile.Outer.Segments[0] = line
	})
	refusesPrism(t, "duplicate carrier", func(p *prismPayload) {
		for i, raw := range p.Profile.Outer.Segments {
			line := raw.(sectionrecord.LineSeg)
			if line.Start.V == 3 && line.End.V == 3 {
				line.Start.V, line.End.V = 1, 1
				p.Profile.Outer.Segments[i] = line
			}
		}
	})
	refusesPrism(t, "swapped carriers", func(p *prismPayload) {
		for i, raw := range p.Profile.Outer.Segments {
			line := raw.(sectionrecord.LineSeg)
			line.Start.V = 4 - line.Start.V
			line.End.V = 4 - line.End.V
			p.Profile.Outer.Segments[i] = line
		}
	})
	refusesPrism(t, "reversed loop", func(p *prismPayload) {
		for i, raw := range p.Profile.Outer.Segments {
			line := raw.(sectionrecord.LineSeg)
			line.TStart, line.TEnd = line.TEnd, line.TStart
			p.Profile.Outer.Segments[i] = line
		}
	})
	refusesPrism(t, "rewrite displacement", func(p *prismPayload) { p.SectionDelta = 0.01 })
	refusesPrism(t, "placed prism", func(p *prismPayload) {
		shift, shiftErr := r3.Translation(r3.Vec{X: 1})
		require.NoError(t, shiftErr)
		p.Xform = shift
	})

	t.Run("altered loft cap", func(t *testing.T) {
		changed := lp
		changed.Plane1.Origin.Z = 11
		loft.n.Payload = changed
		defer func() { loft.n.Payload = lp }()
		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWorkBudget(t.Context()), &prism.n, &loft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "loft_arc", got.Reason)
	})
	t.Run("altered loft wall role", func(t *testing.T) {
		changed := lp
		changed.Side = append([]uint8(nil), lp.Side...)
		for i, role := range lp.Cell {
			if lp.Segment[role[0]][role[1]] == arc {
				changed.Side[i] = 2
				break
			}
		}
		loft.n.Payload = changed
		defer func() { loft.n.Payload = lp }()
		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWorkBudget(t.Context()), &prism.n, &loft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "y1.wall_role", got.Reason)
	})
	t.Run("held order on an interior wall band", func(t *testing.T) {
		changed := lp
		changed.Verts = append([]r3.Vec(nil), lp.Verts...)
		for i, v := range changed.Verts {
			if v.Y > 2.75 && v.Y < 3.25 {
				changed.Verts[i].X = 6
			}
		}
		loft.n.Payload = changed
		defer func() { loft.n.Payload = lp }()
		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWorkBudget(t.Context()), &prism.n, &loft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "held_order", got.Reason)
	})
	t.Run("one visit short", func(t *testing.T) {
		_, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(pairVisits-1, proofbound.NewWorkBudget(t.Context())),
			&prism.n, &loft.n)
		require.False(t, accepted)
		require.ErrorIs(t, readErr, proofbound.ErrWallWorkBudget)
	})
	t.Run("in flight cancellation", func(t *testing.T) {
		cancelCtx, cancel := context.WithCancel(t.Context())
		defer cancel()
		steps := 0
		operation := &proofbound.WorkBudget{StepFn: func() error {
			steps++
			if steps == 100 {
				cancel()
			}
			return cancelCtx.Err()
		}, ErrFn: cancelCtx.Err}
		_, accepted, readErr := loftcontact.CertifyQuarterHeldPair(cancelCtx, operation, &prism.n, &loft.n)
		require.False(t, accepted)
		require.ErrorIs(t, readErr, context.Canceled)
		require.Equal(t, 100, steps)
	})
	t.Run("another document with the same producer", func(t *testing.T) {
		other := New()
		prism.n.Doc = &other.n
		defer func() { prism.n.Doc = &doc.n }()
		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWorkBudget(t.Context()), &prism.n, &loft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "live_pair", got.Reason)
	})
	t.Run("copied prism", func(t *testing.T) {
		copied, copyErr := Copy(t.Context(), prism)
		require.NoError(t, copyErr)
		defer func() { require.NoError(t, Remove(copied)) }()
		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWorkBudget(t.Context()), &copied.n, &loft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "live_pair", got.Reason)
	})
	t.Run("prism removed during scan", func(t *testing.T) {
		steps := 0
		operation := &proofbound.WorkBudget{StepFn: func() error {
			steps++
			if steps == 3000 {
				return Remove(prism)
			}
			return nil
		}, ErrFn: func() error { return nil }}
		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(), operation, &prism.n, &loft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "live_pair", got.Reason)
		require.Greater(t, steps, 3000)
	})
	t.Run("copied prism after original removal", func(t *testing.T) {
		copyDoc := New()
		copyLoft, buildErr := Loft(t.Context(), copyDoc,
			sections[0], profiles[0], sections[1], profiles[1])
		require.NoError(t, buildErr)
		original, buildErr := Extrude(t.Context(), copyDoc, prismSketch, found[0],
			Distance{D: units.Millimeters(12), Dir: Along})
		require.NoError(t, buildErr)
		copied, copyErr := Copy(t.Context(), original)
		require.NoError(t, copyErr)
		require.NoError(t, Remove(original))
		originalRecord, recordOK := original.n.Payload.(prismPayload)
		require.True(t, recordOK)
		copiedRecord, recordOK := copied.n.Payload.(prismPayload)
		require.True(t, recordOK)
		require.Equal(t, originalRecord.Frame, copiedRecord.Frame)
		require.Equal(t, originalRecord.Xform, copiedRecord.Xform)
		require.Equal(t, originalRecord.Z0, copiedRecord.Z0)
		require.Equal(t, originalRecord.Z1, copiedRecord.Z1)
		require.Equal(t, originalRecord.Profile.Outer.Segments, copiedRecord.Profile.Outer.Segments)

		live := copyDoc.n.Live()
		require.Len(t, live, 2)
		require.Same(t, &copyLoft.n, live[0])
		require.Same(t, &copied.n, live[1])
		require.Equal(t, 0, int(copyLoft.n.Origin.Producer))
		require.Equal(t, 1, int(original.n.Origin.Producer))
		require.Equal(t, 2, int(copied.n.Origin.Producer))

		got, accepted, readErr := loftcontact.CertifyQuarterHeldPair(t.Context(),
			proofbound.NewWorkBudget(t.Context()), &copied.n, &copyLoft.n)
		require.NoError(t, readErr)
		require.False(t, accepted)
		require.Equal(t, "live_pair", got.Reason)
	})
}
