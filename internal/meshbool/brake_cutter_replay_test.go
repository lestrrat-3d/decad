package meshbool

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestBrakeCutterConstructorByteLimit(t *testing.T) {
	ctx := t.Context()
	meter := proofbound.NewBrakeReplayBudget()
	budget, err := NewReplayPairCutsBudget(meter)
	if err != nil {
		t.Fatal(err)
	}
	cutter, err := newBrakeTriCutter(ctx, 5, budget)
	if err != nil {
		t.Fatal(err)
	}
	if cutter.replayVert == nil || cutter.replaySeen == nil || cutter.Work == nil || cutter.Reserve == nil {
		t.Fatal("incomplete cutter")
	}
	cost := meter.Bytes()
	for _, short := range []bool{false, true} {
		m := proofbound.NewBrakeReplayBudget()
		remaining := cost
		if short {
			remaining--
		}
		if err := m.ReserveBytes(ctx, proofbound.BrakeReplayBytes-remaining); err != nil {
			t.Fatal(err)
		}
		b, err := NewReplayPairCutsBudget(m)
		if err != nil {
			t.Fatal(err)
		}
		_, err = newBrakeTriCutter(ctx, 5, b)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short constructor error: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact constructor error: %v", err)
		}
	}
}

func TestBrakeCutterRegionsMatchOrdinary(t *testing.T) {
	fixtures := cutFixtureFacets(t, 4)
	for i, f := range fixtures {
		want, err := CutTriangle(t.Context(), f.xtri, f.normal, f.segs)
		if err != nil {
			t.Fatalf("fixture %d ordinary: %v", i, err)
		}
		budget, err := NewReplayPairCutsBudget(proofbound.NewBrakeReplayBudget())
		if err != nil {
			t.Fatal(err)
		}
		got, err := cutTriangleSharedWork(t.Context(), f.xtri, f.normal, f.segs, budget)
		if err != nil {
			t.Fatalf("fixture %d replay: %v", i, err)
		}
		requireSameRegionValues(t, want, got, "replay")
	}
}

func TestBrakeCutterRefusesIncompleteState(t *testing.T) {
	cutter := &brakeTriCutter{}
	_, err := cutter.cut(t.Context(), [3]proof.Xpt{}, proof.Xpt{}, nil)
	if !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("incomplete cutter error: %v", err)
	}
	if err := cutter.reserveProduct(1, 1, 0); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("missing reservation error: %v", err)
	}
	if err := cutter.reserveArray(1, 1); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("missing budget error: %v", err)
	}
}

func TestBrakeCutterConstructorRecordLimits(t *testing.T) {
	ctx := t.Context()
	measure := proofbound.NewBrakeReplayBudget()
	budget, err := NewReplayPairCutsBudget(measure)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := newBrakeTriCutter(ctx, 5, budget); err != nil {
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
		b, err := NewReplayPairCutsBudget(meter)
		if err != nil {
			t.Fatal(err)
		}
		_, err = newBrakeTriCutter(ctx, 5, b)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact: %v", err)
		}
	}
}

func TestBrakeCutterRefusesDeferredChainProbe(t *testing.T) {
	cutter := &brakeTriCutter{Verts: []CutVert{{P2: Xp2{deferredRats: true}}, {}}}
	chain := ChainPath{Verts: []int{0, 1}}
	_, _, err := cutter.chainProbe(chain)
	if !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("deferred probe: %v", err)
	}
	if allocs := testing.AllocsPerRun(100, func() {
		if _, _, err := cutter.chainProbe(chain); err == nil {
			panic("missing refusal")
		}
	}); allocs != 0 {
		t.Fatalf("deferred probe refusal allocated %.0f objects", allocs)
	}
}

func TestBrakeCutterRefusesDeferredVertex(t *testing.T) {
	cutter := &brakeTriCutter{}
	_, err := cutter.addVertShared(Xp2{deferredRats: true}, proof.Xpt{}, true)
	if !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("deferred vertex: %v", err)
	}
}

func TestBrakeCutterRefusesMismatchedBudget(t *testing.T) {
	ctx := t.Context()
	for _, kind := range []string{brakeTestWork, "vertices", "edges"} {
		t.Run(kind, func(t *testing.T) {
			first := proofbound.NewBrakeReplayBudget()
			budget, err := NewReplayPairCutsBudget(first)
			if err != nil {
				t.Fatal(err)
			}
			cutter, err := newBrakeTriCutter(ctx, 0, budget)
			if err != nil {
				t.Fatal(err)
			}
			other := proofbound.NewBrakeReplayBudget()
			switch kind {
			case brakeTestWork:
				cutter.Work.ReplayID = other
			case "vertices":
				cutter.replayVert.budget = other
			case "edges":
				cutter.replaySeen.budget = other
			}
			_, err = cutter.cut(ctx, [3]proof.Xpt{}, proof.Xpt{}, nil)
			if !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("mismatched budget: %v", err)
			}
		})
	}
}
