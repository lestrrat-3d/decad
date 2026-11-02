package momentinput

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func TestReplayAnalyticProfileViews(t *testing.T) {
	source := []Profile{{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{End: sectionrecord.Point2{U: 2}}}}}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		v, err := ReplayAnalyticProfileViews(ctx, source, b)
		if err != nil {
			return err
		}
		if len(v) != 1 || !sectionrecord.IdenticalRecord(v[0].Outer, source[0].Outer) {
			t.Fatal("profile view differs")
		}
		return nil
	}
	base := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), base); err != nil {
		t.Fatal(err)
	}
	w, r := base.Usage()
	z := base.Bytes()
	for i := range 4 {
		ww, rr, zz := w+1, r+1, z+1
		switch i {
		case 1:
			ww--
		case 2:
			rr--
		case 3:
			zz--
		}
		b, err := proofbound.NewReplayBudgetLimits(ww, rr, zz)
		if err != nil {
			t.Fatal(err)
		}
		if err = b.ReserveBundle(t.Context(), 1, 1, 1); err != nil {
			t.Fatal(err)
		}
		err = run(t.Context(), b)
		if (i == 0) != (err == nil) {
			t.Fatalf("limit %d error=%v", i, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}
