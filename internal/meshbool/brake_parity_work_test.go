package meshbool

import (
	"errors"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestBrakeParityWorkBudgetAndUnanchoredRefusal(t *testing.T) {
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		size := 2*(uint64(unsafe.Sizeof(proofbound.WorkBudget{}))+256) + 128
		if short {
			size--
		}
		if err := meter.ReserveBytes(t.Context(), proofbound.BrakeReplayBytes-size); err != nil {
			t.Fatal(err)
		}
		budget, err := NewReplayPairCutsBudget(meter)
		if err != nil {
			t.Fatal(err)
		}
		work, err := newBrakeParityWork(t.Context(), budget)
		if short && (!errors.Is(err, decaderr.ErrUnsupported) || work != nil) {
			t.Fatalf("one-short: %v", err)
		}
		if !short && (err != nil || work == nil) {
			t.Fatalf("exact: %v", err)
		}
	}
	budget, err := NewReplayPairCutsBudget(proofbound.NewBrakeReplayBudget())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := classifyRegionReplay(t.Context(), CutRegion{}, nil, budget); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("unanchored: %v", err)
	}
}

func TestBrakeCutterRefusesNoContactEdge(t *testing.T) {
	budget, err := NewReplayPairCutsBudget(proofbound.NewBrakeReplayBudget())
	if err != nil {
		t.Fatal(err)
	}
	tri := [3]proof.Xpt{proof.XptOf(r3.Vec{}), proof.XptOf(r3.Vec{X: 1}), proof.XptOf(r3.Vec{Y: 1})}
	_, err = cutTriangleSharedWork(t.Context(), tri, proof.XptOf(r3.Vec{Z: 1}), nil, budget)
	if !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("empty contact accepted: %v", err)
	}
}

func TestBrakeStaticRefusalsDoNotAllocate(t *testing.T) {
	ctx := t.Context()
	budget, err := NewReplayPairCutsBudget(proofbound.NewBrakeReplayBudget())
	if err != nil {
		t.Fatal(err)
	}
	cutter := &brakeTriCutter{}
	for _, run := range []func() error{
		func() error { _, err := cutter.cutInterned(ctx, [3]int{}); return err },
		func() error { _, err := classifyRegionReplay(ctx, CutRegion{}, nil, budget); return err },
	} {
		err := run()
		if !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("unexpected refusal: %v", err)
		}
		if got := testing.AllocsPerRun(100, func() {
			if run() == nil {
				panic("missing refusal")
			}
		}); got != 0 {
			t.Fatalf("static refusal allocated %.0f objects", got)
		}
	}
	if !errors.Is(errBrakeParityRays, decaderr.ErrBooleanFailed) {
		t.Fatal("parity refusal lost its cause")
	}
}

func TestBrakeTypedContactRefusalsDoNotAllocate(t *testing.T) {
	cutter := &brakeTriCutter{}
	budget := &PairCutsBudget{}
	cases := []struct {
		run   func() error
		reset func()
		want  error
	}{
		{cutter.branchContactRefusal, func() { cutter.contactRefusal = BooleanExpectedError{} }, ErrUnclassifiableContact(`intersection curves branch on a facet`)},
		{budget.boundaryContactRefusal, func() { budget.contactRefusal = BooleanExpectedError{} }, ErrUnclassifiableContact(`an operand facet touches the other operand's boundary`)},
	}
	for i, c := range cases {
		err := c.run()
		var expected *BooleanExpectedError
		if !errors.As(err, &expected) || expected.Kind != BooleanExpectedContact || expected.Operand != -1 ||
			!errors.Is(err, decaderr.ErrUnsupported) || err.Error() != c.want.Error() {
			t.Fatalf("contact refusal %d did not preserve its type or message", i)
		}
		if got := testing.AllocsPerRun(100, func() {
			c.reset()
			if c.run() == nil {
				panic("missing refusal")
			}
		}); got != 0 {
			t.Fatalf("contact refusal %d allocated %.0f objects", i, got)
		}
	}
}

func TestBrakeParityAdapterRecordLimits(t *testing.T) {
	ctx := t.Context()
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := uint64(6)
		if short {
			remaining--
		}
		if err := meter.ReserveRecords(ctx, proofbound.BrakeReplayRecords-remaining); err != nil {
			t.Fatal(err)
		}
		budget, err := NewReplayPairCutsBudget(meter)
		if err != nil {
			t.Fatal(err)
		}
		work, err := newBrakeParityWork(ctx, budget)
		if short && (!errors.Is(err, decaderr.ErrUnsupported) || work != nil) {
			t.Fatalf("one-short: %v", err)
		}
		if !short && (err != nil || work == nil) {
			t.Fatalf("exact: %v", err)
		}
	}
}

func TestBrakeParityTreeRecordLimits(t *testing.T) {
	ctx := t.Context()
	build := func(meter *proofbound.ReplayBudget) error {
		mesh := &ParityMesh{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}}, Tris: [][3]int{{0, 1, 2}}}
		_, err := replayParityTree(ctx, mesh, 2, 0, 1, meter.WorkBudget(ctx))
		return err
	}
	measure := proofbound.NewBrakeReplayBudget()
	if err := build(measure); err != nil {
		t.Fatal(err)
	}
	_, cost := measure.Usage()
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := cost
		if short {
			remaining--
		}
		if err := meter.ReserveRecords(ctx, proofbound.BrakeReplayRecords-remaining); err != nil {
			t.Fatal(err)
		}
		err := build(meter)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short tree: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact tree: %v", err)
		}
	}
}
