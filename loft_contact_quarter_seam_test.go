package decad

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

const (
	quarterSeamSourceStage = "source"
	quarterSeamOutputStage = "output"
)

func quarterSeamRealPair(t *testing.T, radii [2]float64) (*Body, *Body) {
	t.Helper()
	ctx := t.Context()
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 5.5})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 5.5})
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

func TestQuarterSourceHeldY3ReverseOrder(t *testing.T) {
	ctx := t.Context()
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 5.5})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 5.5})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 6})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 2})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 5.5})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 2})
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
	prism, loft := quarterSeamRealPair(t, [2]float64{5, 6})
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
