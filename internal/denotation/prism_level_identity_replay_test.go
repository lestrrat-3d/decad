package denotation

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const replayTestCaseExact = "exact"

func TestReplayLevelIdentity(t *testing.T) {
	d := LevelDenotation{exact: big.NewRat(3, 2), held: 1.5}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (bool, error) {
		s, ok, err := CaptureReplayLevelIdentity(ctx, d, b)
		if err != nil || !ok {
			return false, err
		}
		return s.Matches(ctx, d, b)
	}
	b := proofbound.NewReplayTrialBudget()
	if ok, err := run(t.Context(), b); err != nil || !ok {
		t.Fatalf("match=%t error=%v", ok, err)
	}
	w, r := b.Usage()
	z := b.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, z uint64
		pass    bool
	}{
		{replayTestCaseExact, w, r, z, true}, {"work", w - 1, r, z, false},
		{"records", w, r - 1, z, false}, {"bytes", w, r, z - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.z)
			if err != nil {
				t.Fatal(err)
			}
			ok, err := run(t.Context(), meter)
			if tc.pass && (err != nil || !ok) || !tc.pass && (err == nil || ok) {
				t.Fatalf("match=%t error=%v", ok, err)
			}
		})
	}
	s, ok, err := CaptureReplayLevelIdentity(t.Context(), d, b)
	if err != nil || !ok {
		t.Fatal("capture failed")
	}
	replaced := d
	replaced.exact = new(big.Rat).Set(d.exact)
	if ok, _ := s.Matches(t.Context(), replaced, b); ok {
		t.Fatal("equal rational replacement accepted")
	}
	d.exact.SetInt64(2)
	if ok, _ := s.Matches(t.Context(), d, b); ok {
		t.Fatal("rational mutation accepted")
	}
	d.exact.SetFrac64(3, 2)
	d.delta = math.Copysign(0, -1)
	if ok, _ := s.Matches(t.Context(), d, b); ok {
		t.Fatal("signed zero mutation accepted")
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err := CaptureReplayLevelIdentity(ctx, d, b); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel error=%v", err)
	}
}
