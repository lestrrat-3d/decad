package decad

import (
	"context"
	"errors"
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

const (
	quarterSeamSourceStage = "source"
	quarterSeamOutputStage = "output"
)

func quarterSeamRealPair(ctx context.Context, t *testing.T, radii [2]float64) (*Body, *Body) {
	t.Helper()
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 10)
	if err != nil {
		t.Fatal(err)
	}
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for capIndex, radius := range radii {
		plane := world.XY()
		if capIndex == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		if createErr != nil {
			t.Fatal(createErr)
		}
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		right := s.CreatePoint(radius, 0)
		top := s.CreatePoint(0, radius)
		s.CreateLine(center, right)
		s.CreateArc(center, right, top)
		s.CreateLine(top, center)
		if _, createErr = s.Solve(ctx); createErr != nil {
			t.Fatal(createErr)
		}
		found := s.Profiles()
		if len(found) != 1 || !found[0].Valid {
			t.Fatalf("Loft cap %d: profiles=%d", capIndex, len(found))
		}
		sections[capIndex], profiles[capIndex] = s, found[0]
	}
	doc := New()
	loft, err := Loft(ctx, doc, sections[0], profiles[0], sections[1], profiles[1])
	if err != nil {
		t.Fatal(err)
	}
	prismSketch, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	if _, err = prismSketch.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	found := prismSketch.Profiles()
	if len(found) != 1 || !found[0].Valid {
		t.Fatalf("prism profiles=%d", len(found))
	}
	prism, err := Extrude(ctx, doc, prismSketch, found[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	return prism, loft
}

func TestQuarterSourceHeldY1RealProducerGate(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	var sourceVisits, outputVisits uint64
	sourceOperation := &proofbound.WorkBudget{
		StepFn: func() error { sourceVisits++; return ctx.Err() }, ErrFn: ctx.Err,
	}
	outputOperation := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return ctx.Err() }, ErrFn: ctx.Err,
	}
	proof, accepted, err := loftQuarterSourceHeldY1(ctx, prism, loft,
		sourceOperation, outputOperation)
	if err != nil || !accepted || proof.Bound == nil ||
		proof.Bound.Cmp(big.NewRat(1, 4)) != 0 || proof.Pieces != 4 {
		t.Fatalf("real y=1 seam: accepted=%t reason=%s err=%v bound=%v pieces=%d source=%d output=%d",
			accepted, proof.Reason, err, proof.Bound, proof.Pieces, sourceVisits, outputVisits)
	}
	if sourceVisits >= 4096 || outputVisits >= 65_536 {
		t.Fatalf("work: source=%d output=%d", sourceVisits, outputVisits)
	}
	t.Logf("real y=1 seam: source=%d output=%d", sourceVisits, outputVisits)
}

func TestQuarterSourceHeldY3RealProducerGate(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	var sourceVisits, outputVisits uint64
	source := &proofbound.WorkBudget{
		StepFn: func() error { sourceVisits++; return ctx.Err() }, ErrFn: ctx.Err,
	}
	output := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return ctx.Err() }, ErrFn: ctx.Err,
	}
	proof, accepted, err := loftQuarterSourceHeldY3(ctx, prism, loft, source, output)
	if err != nil || !accepted || proof.Bound == nil ||
		proof.Bound.Cmp(big.NewRat(1, 4)) != 0 || proof.Pieces != 8 {
		t.Fatalf("real y=3 seam: accepted=%t reason=%s err=%v bound=%v pieces=%d source=%d output=%d",
			accepted, proof.Reason, err, proof.Bound, proof.Pieces, sourceVisits, outputVisits)
	}
	if sourceVisits >= 4096 || outputVisits >= 65_536 {
		t.Fatalf("work: source=%d output=%d", sourceVisits, outputVisits)
	}
	t.Logf("real y=3 seam: source=%d output=%d pieces=%d", sourceVisits, outputVisits, proof.Pieces)
}

func TestQuarterSourceHeldPairRealProducerGate(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	var sourceVisits, outputVisits uint64
	source := &proofbound.WorkBudget{
		StepFn: func() error { sourceVisits++; return ctx.Err() }, ErrFn: ctx.Err,
	}
	output := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return ctx.Err() }, ErrFn: ctx.Err,
	}
	proof, accepted, err := loftQuarterSourceHeldPair(ctx, prism, loft, source, output)
	if err != nil || !accepted || proof.Y1.Bound == nil || proof.Y3.Bound == nil ||
		proof.Y1.Bound.Cmp(big.NewRat(1, 4)) != 0 ||
		proof.Y3.Bound.Cmp(big.NewRat(1, 4)) != 0 ||
		proof.Y1.Pieces != 4 || proof.Y3.Pieces != 8 || proof.Slabs != 11 {
		t.Fatalf("real paired seam: accepted=%t reason=%s err=%v y1=%d y3=%d slabs=%d source=%d output=%d",
			accepted, proof.Reason, err, proof.Y1.Pieces, proof.Y3.Pieces,
			proof.Slabs, sourceVisits, outputVisits)
	}
	if sourceVisits >= 4096 || outputVisits >= 65_536 {
		t.Fatalf("paired work: source=%d output=%d", sourceVisits, outputVisits)
	}
	t.Logf("real paired seam: source=%d output=%d slabs=%d", sourceVisits, outputVisits, proof.Slabs)
}

func TestQuarterSourceHeldPairReverseOrder(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	proof, accepted, err := loftQuarterSourceHeldPair(ctx, loft, prism,
		proofbound.NewWorkBudget(ctx), proofbound.NewWorkBudget(ctx))
	if err != nil || !accepted || proof.Y1.Bound == nil || proof.Y3.Bound == nil ||
		proof.Y1.Pieces != 4 || proof.Y3.Pieces != 8 || proof.Slabs != 11 {
		t.Fatalf("reverse paired seam: accepted=%t reason=%s err=%v y1=%d y3=%d slabs=%d",
			accepted, proof.Reason, err, proof.Y1.Pieces, proof.Y3.Pieces, proof.Slabs)
	}
}

func TestQuarterSourceHeldPairRefusals(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	limitErr := errors.New("operation budget exhausted")
	for _, tc := range []struct {
		name, stage string
		limit       uint64
	}{
		{name: quarterSeamSourceStage, stage: quarterSeamSourceStage, limit: 3340},
		{name: quarterSeamOutputStage, stage: quarterSeamOutputStage, limit: 7000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var sourceVisits, outputVisits uint64
			source := &proofbound.WorkBudget{StepFn: func() error {
				if tc.stage == quarterSeamSourceStage && sourceVisits >= tc.limit {
					return limitErr
				}
				sourceVisits++
				return nil
			}, ErrFn: func() error { return nil }}
			output := &proofbound.WorkBudget{StepFn: func() error {
				if tc.stage == quarterSeamOutputStage && outputVisits >= tc.limit {
					return limitErr
				}
				outputVisits++
				return nil
			}, ErrFn: func() error { return nil }}
			got, accepted, err := loftQuarterSourceHeldPair(ctx, prism, loft, source, output)
			if accepted || !errors.Is(err, limitErr) || got.Y1.Bound != nil ||
				got.Y3.Bound != nil || got.Slabs != 0 {
				t.Fatalf("limited paired %s: accepted=%t err=%v slabs=%d",
					tc.name, accepted, err, got.Slabs)
			}
			if tc.stage == quarterSeamSourceStage && outputVisits != 0 {
				t.Fatalf("output started after source refusal: visits=%d", outputVisits)
			}
		})
	}
	canceled, cancel := context.WithCancel(ctx)
	var outputVisits uint64
	source := &proofbound.WorkBudget{
		StepFn: func() error { return nil }, ErrFn: func() error { return nil },
	}
	output := &proofbound.WorkBudget{StepFn: func() error {
		outputVisits++
		if outputVisits == 7000 {
			cancel()
		}
		return nil
	}, ErrFn: func() error { return nil }}
	got, accepted, err := loftQuarterSourceHeldPair(canceled, prism, loft, source, output)
	if accepted || !errors.Is(err, context.Canceled) || got.Y1.Bound != nil ||
		got.Y3.Bound != nil || got.Slabs != 0 || outputVisits != 7000 {
		t.Fatalf("paired cancellation: accepted=%t err=%v slabs=%d output=%d",
			accepted, err, got.Slabs, outputVisits)
	}
	if err := Remove(prism); err != nil {
		t.Fatal(err)
	}
	got, accepted, err = loftQuarterSourceHeldPair(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), proofbound.NewWorkBudget(ctx))
	if err != nil || accepted || got.Y1.Bound != nil || got.Y3.Bound != nil || got.Slabs != 0 {
		t.Fatalf("retired paired source: accepted=%t err=%v slabs=%d", accepted, err, got.Slabs)
	}
}

func TestQuarterSourceHeldPairWrongSource(t *testing.T) {
	ctx := t.Context()
	for _, tc := range []struct {
		name  string
		radii [2]float64
	}{
		{name: "radius", radii: [2]float64{5, 6}},
		{name: "junction", radii: [2]float64{5, 2}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			prism, loft := quarterSeamRealPair(ctx, t, tc.radii)
			var outputVisits uint64
			output := &proofbound.WorkBudget{
				StepFn: func() error { outputVisits++; return nil }, ErrFn: func() error { return nil },
			}
			got, accepted, err := loftQuarterSourceHeldPair(ctx, prism, loft,
				proofbound.NewWorkBudget(ctx), output)
			if err != nil || accepted || got.Y1.Bound != nil || got.Y3.Bound != nil ||
				got.Slabs != 0 || outputVisits != 0 {
				t.Fatalf("wrong paired %s: accepted=%t err=%v slabs=%d output=%d",
					tc.name, accepted, err, got.Slabs, outputVisits)
			}
		})
	}
}

func TestQuarterSourceHeldY3ReverseOrder(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	proof, accepted, err := loftQuarterSourceHeldY3(ctx, loft, prism,
		proofbound.NewWorkBudget(ctx), proofbound.NewWorkBudget(ctx))
	if err != nil || !accepted || proof.Bound == nil ||
		proof.Bound.Cmp(big.NewRat(1, 4)) != 0 || proof.Pieces != 8 {
		t.Fatalf("reverse y=3 seam: accepted=%t reason=%s err=%v pieces=%d",
			accepted, proof.Reason, err, proof.Pieces)
	}
}

func TestQuarterSourceHeldY3Refusals(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	limitErr := errors.New("operation budget exhausted")
	for _, tc := range []struct {
		name, stage string
		limit       uint64
	}{
		{name: quarterSeamSourceStage, stage: quarterSeamSourceStage, limit: 3000},
		{name: quarterSeamOutputStage, stage: quarterSeamOutputStage, limit: 6000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var sourceVisits, outputVisits uint64
			source := &proofbound.WorkBudget{StepFn: func() error {
				if tc.stage == quarterSeamSourceStage && sourceVisits >= tc.limit {
					return limitErr
				}
				sourceVisits++
				return nil
			}, ErrFn: func() error { return nil }}
			output := &proofbound.WorkBudget{StepFn: func() error {
				if tc.stage == quarterSeamOutputStage && outputVisits >= tc.limit {
					return limitErr
				}
				outputVisits++
				return nil
			}, ErrFn: func() error { return nil }}
			got, accepted, err := loftQuarterSourceHeldY3(ctx, prism, loft, source, output)
			if accepted || !errors.Is(err, limitErr) || got.Bound != nil || got.Pieces != 0 {
				t.Fatalf("limited %s: accepted=%t err=%v pieces=%d", tc.name, accepted, err, got.Pieces)
			}
			if tc.stage == quarterSeamSourceStage && outputVisits != 0 {
				t.Fatalf("output started after source refusal: visits=%d", outputVisits)
			}
		})
	}
	canceled, cancel := context.WithCancel(ctx)
	var outputVisits uint64
	source := &proofbound.WorkBudget{
		StepFn: func() error { return nil }, ErrFn: func() error { return nil },
	}
	output := &proofbound.WorkBudget{StepFn: func() error {
		outputVisits++
		if outputVisits == 6000 {
			cancel()
		}
		return nil
	}, ErrFn: func() error { return nil }}
	got, accepted, err := loftQuarterSourceHeldY3(canceled, prism, loft, source, output)
	if accepted || !errors.Is(err, context.Canceled) || got.Bound != nil || got.Pieces != 0 ||
		outputVisits != 6000 {
		t.Fatalf("in-flight cancellation: accepted=%t err=%v pieces=%d output=%d",
			accepted, err, got.Pieces, outputVisits)
	}
	if err := Remove(prism); err != nil {
		t.Fatal(err)
	}
	got, accepted, err = loftQuarterSourceHeldY3(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), proofbound.NewWorkBudget(ctx))
	if err != nil || accepted || got.Bound != nil || got.Pieces != 0 {
		t.Fatalf("retired source: accepted=%t err=%v pieces=%d", accepted, err, got.Pieces)
	}
}

func TestQuarterSourceHeldY3WrongRealRadius(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 6})
	var outputVisits uint64
	output := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return nil }, ErrFn: func() error { return nil },
	}
	got, accepted, err := loftQuarterSourceHeldY3(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), output)
	if err != nil || accepted || got.Bound != nil || got.Pieces != 0 || outputVisits != 0 {
		t.Fatalf("wrong real radius: accepted=%t err=%v pieces=%d output=%d",
			accepted, err, got.Pieces, outputVisits)
	}
}

func TestQuarterSourceHeldY3MissingRealJunction(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 2})
	var outputVisits uint64
	output := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return nil }, ErrFn: func() error { return nil },
	}
	got, accepted, err := loftQuarterSourceHeldY3(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), output)
	if err != nil || accepted || got.Bound != nil || got.Pieces != 0 || outputVisits != 0 {
		t.Fatalf("missing real cap junction: accepted=%t err=%v pieces=%d output=%d",
			accepted, err, got.Pieces, outputVisits)
	}
}

func TestQuarterSourceHeldY1Refusals(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 5.5})
	limitErr := errors.New("operation budget exhausted")
	for _, tc := range []struct {
		name        string
		sourceLimit uint64
		outputLimit uint64
	}{
		{name: quarterSeamSourceStage, sourceLimit: 3000, outputLimit: 65_536},
		{name: quarterSeamOutputStage, sourceLimit: 4096, outputLimit: 6000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var sourceVisits, outputVisits uint64
			source := &proofbound.WorkBudget{StepFn: func() error {
				if sourceVisits >= tc.sourceLimit {
					return limitErr
				}
				sourceVisits++
				return nil
			}, ErrFn: func() error { return nil }}
			output := &proofbound.WorkBudget{StepFn: func() error {
				if outputVisits >= tc.outputLimit {
					return limitErr
				}
				outputVisits++
				return nil
			}, ErrFn: func() error { return nil }}
			got, accepted, err := loftQuarterSourceHeldY1(ctx, prism, loft, source, output)
			if accepted || !errors.Is(err, limitErr) || got.Bound != nil || got.Pieces != 0 {
				t.Fatalf("limited %s: accepted=%t reason=%s err=%v pieces=%d",
					tc.name, accepted, got.Reason, err, got.Pieces)
			}
			if tc.name == quarterSeamSourceStage && outputVisits != 0 {
				t.Fatalf("output started after source refusal: visits=%d", outputVisits)
			}
		})
	}
	canceled, cancel := context.WithCancel(ctx)
	var sourceVisits, outputVisits uint64
	source := &proofbound.WorkBudget{
		StepFn: func() error { sourceVisits++; return nil }, ErrFn: func() error { return nil },
	}
	output := &proofbound.WorkBudget{StepFn: func() error {
		outputVisits++
		if outputVisits == 6000 {
			cancel()
		}
		return nil
	}, ErrFn: func() error { return nil }}
	got, accepted, err := loftQuarterSourceHeldY1(canceled, prism, loft, source, output)
	if accepted || !errors.Is(err, context.Canceled) || got.Bound != nil || got.Pieces != 0 ||
		outputVisits != 6000 {
		t.Fatalf("in-flight cancellation: accepted=%t err=%v pieces=%d source=%d output=%d",
			accepted, err, got.Pieces, sourceVisits, outputVisits)
	}
	if err := Remove(prism); err != nil {
		t.Fatal(err)
	}
	got, accepted, err = loftQuarterSourceHeldY1(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), proofbound.NewWorkBudget(ctx))
	if err != nil || accepted || got.Bound != nil || got.Pieces != 0 {
		t.Fatalf("retired source: accepted=%t err=%v pieces=%d", accepted, err, got.Pieces)
	}
}

func TestQuarterSourceHeldY1MissingRealJunction(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 2})
	var outputVisits uint64
	output := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return nil }, ErrFn: func() error { return nil },
	}
	got, accepted, err := loftQuarterSourceHeldY1(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), output)
	if err != nil || accepted || got.Bound != nil || got.Pieces != 0 || outputVisits != 0 {
		t.Fatalf("missing real cap junction: accepted=%t err=%v pieces=%d output=%d",
			accepted, err, got.Pieces, outputVisits)
	}
}

func TestQuarterSourceHeldY1WrongRealRadius(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(ctx, t, [2]float64{5, 6})
	var outputVisits uint64
	output := &proofbound.WorkBudget{
		StepFn: func() error { outputVisits++; return nil }, ErrFn: func() error { return nil },
	}
	got, accepted, err := loftQuarterSourceHeldY1(ctx, prism, loft,
		proofbound.NewWorkBudget(ctx), output)
	if err != nil || accepted || got.Bound != nil || got.Pieces != 0 || outputVisits != 0 {
		t.Fatalf("wrong real radius: accepted=%t err=%v pieces=%d output=%d",
			accepted, err, got.Pieces, outputVisits)
	}
}

func TestQuarterRootBracketPreflight(t *testing.T) {
	den := new(big.Int).Lsh(big.NewInt(1), 4093)
	num := new(big.Int).Mul(big.NewInt(5), den)
	num.Add(num, big.NewInt(1))
	x := new(big.Rat).SetFrac(num, den)
	if 2*x.Num().BitLen() > 8192 || 2*x.Denom().BitLen() > 8192 {
		t.Fatal("boundary fixture fails the old square-only test")
	}
	one := big.NewRat(1, 1)
	if quarterRootBracketFits(x, x, one, one, big.NewRat(117, 4)) {
		t.Fatal("oversized comparison cross product was accepted")
	}
	if !quarterRootBracketFits(big.NewRat(5, 1), big.NewRat(11, 2),
		one, one, big.NewRat(117, 4)) {
		t.Fatal("ordinary quarter-root bracket was refused")
	}
}

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

func TestLoftQuarterJunctionOneRealCase(t *testing.T) {
	ctx := t.Context()
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 10)
	if err != nil {
		t.Fatal(err)
	}
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, radius := range [2]float64{5, 5.5} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, e := world.CreateSketch(plane)
		if e != nil {
			t.Fatal(e)
		}
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		right := s.CreatePoint(radius, 0)
		top := s.CreatePoint(0, radius)
		s.CreateLine(center, right)
		s.CreateArc(center, right, top)
		s.CreateLine(top, center)
		if _, e = s.Solve(ctx); e != nil {
			t.Fatal(e)
		}
		found := s.Profiles()
		if len(found) != 1 || !found[0].Valid {
			t.Fatalf("producer cap %d: profiles=%d", cap, len(found))
		}
		sections[cap], profiles[cap] = s, found[0]
	}
	doc := New()
	loft, err := Loft(ctx, doc, sections[0], profiles[0],
		sections[1], profiles[1])
	if err != nil {
		t.Fatal(err)
	}
	prismSketch, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	if _, err = prismSketch.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	prismProfiles := prismSketch.Profiles()
	if len(prismProfiles) != 1 || !prismProfiles[0].Valid {
		t.Fatalf("prism profiles=%d", len(prismProfiles))
	}
	prism, err := Extrude(ctx, doc, prismSketch, prismProfiles[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}

	var total uint64
	operation := &proofbound.WorkBudget{
		StepFn: func() error { total++; return ctx.Err() },
		ErrFn:  ctx.Err,
	}
	wall := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
	source, ok, err := certifyLoftSourceSpanCaps(ctx, wall, prism, loft)
	if err != nil || !ok {
		t.Fatalf("source authentication: ok=%t err=%v steps=%d", ok, err, total)
	}
	work := &loftQuarterWork{budget: wall}
	order, ok, err := certifyLoftCapSourceOrder(ctx, wall, source, work)
	t.Logf("quarter source counts: total=%d decad=%d raw=%d proof=%d pairs=%d",
		total, total-work.raw-work.proof-work.pairs,
		work.raw, work.proof, work.pairs)
	if work.declined {
		d := work.decline
		t.Logf("first raw decline: predicate=%s cap=%d profile=%d loop=%d edge=%d left_source=%d left_kind=%T left_tagged=%t left_is_loft=%t left_segment=%d right_source=%d right_kind=%T right_tagged=%t right_is_loft=%t right_segment=%d",
			d.Predicate, work.cap, d.Profile, d.Loop, d.Edge,
			d.LeftSourceIndex, d.LeftEntity, work.leftTagged,
			work.leftTag.IsB, work.leftTag.Segment,
			d.RightSourceIndex, d.RightEntity, work.rightTagged,
			work.rightTag.IsB, work.rightTag.Segment)
	}
	if err != nil || !ok {
		t.Fatalf("quarter cap proof: ok=%t err=%v", ok, err)
	}
	if work.pairs != 24 {
		t.Fatalf("original pair scans=%d, want 24", work.pairs)
	}
	for cap := range 2 {
		if len(order.quarter[cap]) != 2 {
			t.Fatalf("cap %d: junctions=%d, want 2", cap, len(order.quarter[cap]))
		}
		seen := map[float64]bool{}
		for _, event := range order.quarter[cap] {
			line := event.line.curve.(sectionrecord.LineSeg)
			if line.Start.V != line.End.V || event.appearances == 0 {
				t.Fatalf("cap %d: unverified source occurrence", cap)
			}
			y := line.Start.V
			var want *big.Rat
			switch {
			case cap == 0 && y == 1:
				want = big.NewRat(24, 1)
			case cap == 0 && y == 3:
				want = big.NewRat(16, 1)
			case cap == 1 && y == 1:
				want = big.NewRat(117, 4)
			case cap == 1 && y == 3:
				want = big.NewRat(85, 4)
			default:
				t.Fatalf("cap %d: unexpected line y=%g", cap, y)
			}
			x, yBox := event.witness.Point()
			xlo, xhi := x.Bounds()
			ylo, yhi := yBox.Bounds()
			if xlo == nil || xhi == nil || ylo == nil || yhi == nil ||
				xlo.Sign() <= 0 || ylo.Cmp(new(big.Rat).SetFloat64(y)) > 0 ||
				yhi.Cmp(new(big.Rat).SetFloat64(y)) < 0 ||
				new(big.Rat).Mul(xlo, xlo).Cmp(want) > 0 ||
				new(big.Rat).Mul(xhi, xhi).Cmp(want) < 0 {
				t.Fatalf("cap %d y=%g: root bracket misses x squared=%s", cap, y, want)
			}
			if seen[y] {
				t.Fatalf("cap %d y=%g: duplicate root", cap, y)
			}
			seen[y] = true
		}
	}
	wallOrder, ordered, orderErr := certifyLoftQuarterWallOrder(ctx, wall, source)
	if orderErr != nil || !ordered || wallOrder.prism != prism ||
		wallOrder.loft != loft || wallOrder.arcSegment < 0 ||
		len(wallOrder.wallFaces[0]) == 0 ||
		len(wallOrder.wallFaces[0]) != len(wallOrder.wallFaces[1]) {
		t.Fatalf("quarter wall order: accepted=%t err=%v steps=%d",
			ordered, orderErr, total)
	}
	t.Logf("quarter source and wall order: total=%d remaining=%d wall_faces=%d",
		total, 4096-total, len(wallOrder.wallFaces[0]))
	for _, operands := range [][2]*Body{{prism, loft}, {loft, prism}} {
		total = 0
		envelope, certified, e := certifyLoftQuarterSourceEnvelope(ctx,
			operands[0], operands[1], operation)
		if e != nil || !certified || envelope.visits != total ||
			envelope.visits >= 4096 || envelope.wall.prism != prism ||
			envelope.wall.loft != loft || envelope.wall.arcSegment != wallOrder.arcSegment ||
			envelope.wall.lower != wallOrder.lower || envelope.wall.upper != wallOrder.upper {
			t.Fatalf("shared quarter source: certified=%t err=%v visits=%d steps=%d",
				certified, e, envelope.visits, total)
		}
		for cap := range envelope.order.quarter {
			if len(envelope.order.quarter[cap]) != 2 {
				t.Fatalf("cap %d: envelope junctions=%d", cap,
					len(envelope.order.quarter[cap]))
			}
		}
		total = 0
		wall = proofbound.NewWallWorkBudgetWithOperation(4096, operation)
		_, accepted, e := certifyLoftSourcePartition(ctx,
			operands[0], operands[1], wall)
		if e != nil || accepted {
			t.Fatalf("private source consumer: accepted=%t err=%v steps=%d",
				accepted, e, total)
		}
		total = 0
		wall = proofbound.NewWallWorkBudgetWithOperation(4096, operation)
		_, accepted, e = certifyLoftOutputBoundary(ctx,
			operands[0], operands[1], wall)
		if e != nil || accepted {
			t.Fatalf("private output consumer: accepted=%t err=%v steps=%d",
				accepted, e, total)
		}
	}
	limitErr := errors.New("source operation budget exhausted")
	var limited uint64
	limitedOperation := &proofbound.WorkBudget{
		StepFn: func() error {
			if limited >= 3000 {
				return limitErr
			}
			limited++
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, certified, err := certifyLoftQuarterSourceEnvelope(ctx, prism, loft,
		limitedOperation)
	if !errors.Is(err, limitErr) || certified || limited != 3000 {
		t.Fatalf("limited source: certified=%t err=%v steps=%d", certified, err, limited)
	}
	canceled, cancel := context.WithCancel(ctx)
	cancel()
	_, certified, err = certifyLoftQuarterSourceEnvelope(canceled, prism, loft,
		proofbound.NewWorkBudget(canceled))
	if !errors.Is(err, context.Canceled) || certified {
		t.Fatalf("canceled source: certified=%t err=%v", certified, err)
	}
	if err := Remove(prism); err != nil {
		t.Fatal(err)
	}
	for _, operands := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, certified, err := certifyLoftQuarterSourceEnvelope(ctx,
			operands[0], operands[1], proofbound.NewWorkBudget(ctx))
		if err != nil || certified {
			t.Fatalf("retired source: certified=%t err=%v", certified, err)
		}
	}
}

func TestLoftQuarterWallOrderRealSketchPair(t *testing.T) {
	world := sketch.NewWorld()
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
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		found := s.Profiles()
		require.Len(t, found, 1)
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0],
		sections[1], profiles[1])
	require.NoError(t, err)

	makePrism := func(t *testing.T, xMin float64) *Body {
		s, createErr := world.CreateSketch(world.XY())
		require.NoError(t, createErr)
		rect := s.CreateRectangle(xMin, 1, 6, 3)
		s.Fix(rect.A)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		found := s.Profiles()
		require.Len(t, found, 1)
		prism, buildErr := Extrude(t.Context(), New(), s, found[0],
			Distance{D: units.Millimeters(12), Dir: Along})
		require.NoError(t, buildErr)
		return prism
	}
	for _, test := range []struct {
		name  string
		xMin  float64
		admit bool
	}{
		{name: "ordered_crossings", xMin: 2, admit: true},
		{name: "left_edge_enters_arc", xMin: 5.8},
	} {
		t.Run(test.name, func(t *testing.T) {
			prism := makePrism(t, test.xMin)
			var steps uint64
			operation := &proofbound.WorkBudget{
				StepFn: func() error { steps++; return t.Context().Err() },
				ErrFn:  t.Context().Err,
			}
			wall := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
			source, ok, sourceErr := certifyLoftSourceSpanCaps(t.Context(), wall, prism, loft)
			require.NoError(t, sourceErr)
			require.True(t, ok)
			proof, ordered, orderErr := certifyLoftQuarterWallOrder(t.Context(), wall, source)
			require.NoError(t, orderErr)
			require.Equal(t, test.admit, ordered)
			require.Less(t, steps, uint64(4096))
			if !ordered {
				return
			}
			require.Same(t, prism, proof.prism)
			require.Same(t, loft, proof.loft)
			require.Equal(t, "5", proof.radius[0].RatString())
			require.Equal(t, "11/2", proof.radius[1].RatString())
			require.NotEmpty(t, proof.wallFaces[0])
			require.NotEmpty(t, proof.wallFaces[1])
			pp := prism.n.Payload.(prismPayload)
			lower := pp.Profile.Outer.Segments[proof.lower].(sectionrecord.LineSeg)
			upper := pp.Profile.Outer.Segments[proof.upper].(sectionrecord.LineSeg)
			require.Equal(t, float64(1), lower.Start.V)
			require.Equal(t, float64(3), upper.Start.V)
			require.Equal(t, "0", source.levels[0].RatString())
			require.Equal(t, "10", source.levels[1].RatString())
			require.Equal(t, "12", source.levels[2].RatString())
		})
	}

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err := certifyLoftQuarterWallOrder(canceled,
		proofbound.NewWorkBudget(canceled), loftSourceSpanCaps{})
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
