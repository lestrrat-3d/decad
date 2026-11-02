package meshbool

import (
	"context"
	"errors"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestSpatialBoxTreeWorkKeepsEveryCandidate(t *testing.T) {
	ctx := t.Context()
	boxes := make([][2]r3.Vec, 137)
	for i := range boxes {
		p := r3.Vec{X: float64(i * 53 % 31), Y: float64(i * 17 % 29), Z: float64(i * 37 % 23)}
		boxes[i] = [2]r3.Vec{p, p.Add(r3.Vec{X: 1, Y: 2, Z: 3})}
	}
	boxes[3][0].X = math.NaN()
	boxes[7] = [2]r3.Vec{{X: math.Inf(-1)}, {X: math.Inf(1)}}
	budget := proofbound.NewBrakeReplayBudget().WorkBudget(ctx)
	tree, err := newSpatialBoxTreeWork(boxes, budget)
	if err != nil {
		t.Fatal(err)
	}
	stack := make([]int, 0, len(tree.nodes))
	for sample := range 256 {
		p := r3.Vec{X: float64(sample*7%37) - 3, Y: float64(sample*11%31) - 3, Z: float64(sample*13%29) - 3}
		query := [2]r3.Vec{p, p.Add(r3.Vec{X: 5, Y: 4, Z: 3})}
		var want, got []int
		for i, box := range boxes {
			if BoxesOverlap(query, box) {
				want = append(want, i)
			}
		}
		if err := tree.visitOverlapsWork(query, budget.Step, stack, func(i int) error { got = append(got, i); return nil }); err != nil {
			t.Fatal(err)
		}
		slices.Sort(got)
		if !slices.Equal(want, got) {
			t.Fatalf("query %d candidate mismatch: want=%d got=%d", sample, len(want), len(got))
		}
	}
}

func TestSpatialBoxTreeWorkLimitsAndCancellation(t *testing.T) {
	ctx := t.Context()
	boxes := make([][2]r3.Vec, 67)
	for i := range boxes {
		p := r3.Vec{X: float64(i * 31 % 67), Y: float64(i % 3)}
		boxes[i] = [2]r3.Vec{p, p}
	}
	before := slices.Clone(boxes)
	run := func(meter *proofbound.ReplayBudget) (heldBoxTree, error) {
		work := meter.WorkBudget(ctx)
		if err := work.Reserve(5 * uint64(len(boxes))); err != nil {
			return heldBoxTree{}, err
		}
		return newSpatialBoxTreeWork(boxes, work)
	}
	measured := proofbound.NewReplayTrialBudget()
	if _, err := run(measured); err != nil {
		t.Fatal(err)
	}
	w, r := measured.Usage()
	z := measured.Bytes()
	for _, resource := range []string{"exact", "work", "records", "bytes"} {
		t.Run(resource, func(t *testing.T) {
			work, records, bytes := w, r, z
			switch resource {
			case "work":
				work--
			case "records":
				records--
			case "bytes":
				bytes--
			}
			meter, err := proofbound.NewReplayBudgetLimits(work, records, bytes)
			if err != nil {
				t.Fatal(err)
			}
			tree, err := run(meter)
			if resource == "exact" {
				gotW, gotR := meter.Usage()
				if err != nil || len(tree.facets) != len(boxes) || gotW != w || gotR != r || meter.Bytes() != z {
					t.Fatalf("exact: work=%d records=%d bytes=%d err=%v", gotW, gotR, meter.Bytes(), err)
				}
				return
			}
			var detail *proofbound.WorkLimitError
			want := map[string]string{"work": "multiway_work", "records": "multiway_records", "bytes": "replay_bytes"}[resource]
			if !errors.As(err, &detail) || detail.Resource != want || len(tree.nodes) != 0 {
				t.Fatalf("one-short %s: %v", resource, err)
			}
		})
	}
	cancelCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	steps := 0
	work := proofbound.NewReplayTrialBudget().WorkBudget(cancelCtx)
	work.StepFn = func() error {
		steps++
		if steps == 2*len(boxes)+16 {
			cancel()
		}
		return cancelCtx.Err()
	}
	tree, err := newSpatialBoxTreeWork(boxes, work)
	if !errors.Is(err, context.Canceled) || len(tree.nodes) != 0 {
		t.Fatalf("mid-partition cancellation: steps=%d err=%v", steps, err)
	}
	if !slices.Equal(before, boxes) {
		t.Fatal("building or refusing the index changed source boxes")
	}
}
