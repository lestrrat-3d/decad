package tessellation

import (
	"context"
	"errors"
	"math/rand"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayCloseCancel struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	polls           atomic.Int64
	cancel          context.CancelFunc
}

func (c *replayCloseCancel) Err() error {
	if c.polls.Add(1) == 20 {
		c.cancel()
	}
	return c.Context.Err()
}

func TestReplayRequireClosedMesh(t *testing.T) {
	tetra := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	cases := [][][3]int{nil, tetra, tetra[:3], append(append([][3]int(nil), tetra...), tetra[0]), {{0, 0, 1}}}
	random := rand.New(rand.NewSource(42))
	for range 100 {
		tris := make([][3]int, random.Intn(12))
		for j := range tris {
			for k := range 3 {
				tris[j][k] = random.Intn(6)
			}
		}
		cases = append(cases, tris)
	}
	for i, tris := range cases {
		want := RequireClosedMesh(tris)
		got := ReplayRequireClosedMesh(context.Background(), tris, proofbound.NewReplayTrialBudget())
		if (want == nil) != (got == nil) {
			t.Fatalf("case %d triangles=%d differs", i, len(tris))
		}
	}
	baseline := proofbound.NewReplayTrialBudget()
	if err := ReplayRequireClosedMesh(context.Background(), tetra, baseline); err != nil {
		t.Fatal(err)
	}
	w, r := baseline.Usage()
	bytes := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{replayTestCaseExact, w, r, bytes, true}, {replayTestCaseWorkShort, w - 1, r, bytes, false}, {replayTestCaseRecordsShort, w, r - 1, bytes, false}, {replayTestCaseBytesShort, w, r, bytes - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if err != nil {
				t.Fatal(err)
			}
			err = ReplayRequireClosedMesh(context.Background(), tetra, m)
			if (err == nil) != tc.pass {
				t.Fatalf("error=%v", err)
			}
		})
	}
	failed := proofbound.NewReplayTrialBudget()
	var audit *AuditError
	if err := ReplayRequireClosedMesh(context.Background(), tetra[:3], failed); !errors.As(err, &audit) || audit.Sentinel != Degenerate {
		t.Fatalf("open mesh error=%v", err)
	}
	fw, fr := failed.Usage()
	fb := failed.Bytes()
	failureExact, err := proofbound.NewReplayBudgetLimits(fw, fr, fb)
	if err != nil {
		t.Fatal(err)
	}
	if err := ReplayRequireClosedMesh(context.Background(), tetra[:3], failureExact); !errors.As(err, &audit) || audit.Sentinel != Degenerate {
		t.Fatalf("exact open mesh error=%v", err)
	}
	failureShort, err := proofbound.NewReplayBudgetLimits(fw, fr-1, fb)
	if err != nil {
		t.Fatal(err)
	}
	if err := ReplayRequireClosedMesh(context.Background(), tetra[:3], failureShort); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("one-short refusal error=%v", err)
	}
	base, cancel := context.WithCancel(context.Background())
	defer cancel()
	late := &replayCloseCancel{Context: base, cancel: cancel}
	if err := ReplayRequireClosedMesh(late, tetra, proofbound.NewReplayTrialBudget()); err == nil || late.polls.Load() < 20 {
		t.Fatalf("late cancellation polls=%d error=%v", late.polls.Load(), err)
	}
}
