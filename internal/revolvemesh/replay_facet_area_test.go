package revolvemesh

import (
	"context"
	"errors"
	"math"
	"math/rand"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayDyadicSqrtOracle(t *testing.T) {
	rng := rand.New(rand.NewSource(929))
	for i := range 1000 {
		a := math.Ldexp(rng.Float64(), rng.Intn(1900)-950)
		b := math.Ldexp(rng.Float64(), rng.Intn(1900)-950)
		da, db := proof.MustDyOf(a), proof.MustDyOf(b)
		d := proof.DyAdd(proof.DyMul(da, da), proof.DyMul(db, db))
		for _, down := range []bool{true, false} {
			got, err := proof.ReplaySqrtDyadic(t.Context(), d, down, proofbound.NewReplayTrialBudget())
			if err != nil {
				t.Fatalf("case %d: %v", i, err)
			}
			want := proof.DySqrtUp(d)
			if down {
				want = proof.DySqrtDown(d)
			}
			if got != want {
				t.Fatalf("case %d down=%v got=%g want=%g", i, down, got, want)
			}
		}
	}
}

func TestReplayFacetAuditTriangleMatches(t *testing.T) {
	cases := [][]r3.Vec{
		{{X: 1, Y: 2, Z: 3}, {X: 4, Y: -3, Z: 2}, {X: -2, Y: 4, Z: 7}},
		{{X: math.Ldexp(1, -30), Y: 4, Z: -7}, {X: 3.25, Y: -.5, Z: 12}, {X: -5.125, Y: 2.75, Z: .125}},
		{{}, {X: 1}, {Y: 1}},
	}
	for i, verts := range cases {
		want, ok := NewRevolveAuditTri(verts, [3]int{0, 1, 2})
		if !ok {
			t.Fatalf("case %d ordinary refused", i)
		}
		b := proofbound.NewReplayTrialBudget()
		got, err := newRevolveAuditTriReplay(t.Context(), verts, [3]int{0, 1, 2}, b)
		if err != nil {
			t.Fatalf("case %d: %v", i, err)
		}
		if !reflect.DeepEqual(got, want) {
			t.Fatalf("case %d facet audit differs", i)
		}
		if err := requireRevolveFacetAreaReplay(t.Context(), got, i, 0, b); err != nil {
			t.Fatalf("case %d area: %v", i, err)
		}
		w, r := b.Usage()
		t.Logf("case %d work=%d records=%d bytes=%d", i, w, r, b.Bytes())
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := newRevolveAuditTriReplay(ctx, cases[0], [3]int{0, 1, 2}, proofbound.NewReplayTrialBudget())
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation: %v", err)
	}
}

func TestReplayFacetAuditTriangleBudget(t *testing.T) {
	verts := []r3.Vec{{X: 1, Y: 2, Z: 3}, {X: 4, Y: -3, Z: 2}, {X: -2, Y: 4, Z: 7}}
	run := func(b *proofbound.ReplayBudget) error {
		_, err := newRevolveAuditTriReplay(t.Context(), verts, [3]int{0, 1, 2}, b)
		return err
	}
	measured := proofbound.NewReplayTrialBudget()
	if err := run(measured); err != nil {
		t.Fatal(err)
	}
	work, records := measured.Usage()
	bytes := measured.Bytes()
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		fail                 bool
	}{
		{replayBudgetTestExact, proofbound.ReplayTrialWork - work, proofbound.ReplayTrialRecords - records, proofbound.ReplayTrialBytes - bytes, false},
		{"work short", proofbound.ReplayTrialWork - work + 1, 0, 0, true},
		{"records short", 0, proofbound.ReplayTrialRecords - records + 1, 0, true},
		{"bytes short", 0, 0, proofbound.ReplayTrialBytes - bytes + 1, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b := proofbound.NewReplayTrialBudget()
			if err := b.SpendWork(t.Context(), tc.work); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveRecords(t.Context(), tc.records); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveBytes(t.Context(), tc.bytes); err != nil {
				t.Fatal(err)
			}
			err := run(b)
			if (err != nil) != tc.fail {
				t.Fatalf("fail=%v err=%v", tc.fail, err)
			}
		})
	}
}
