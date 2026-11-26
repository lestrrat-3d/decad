package decad

// CI: isolated-tests

import (
	"context"
	"errors"
	"runtime"
	"slices"
	"strings"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestRunStepsSecondRunSeesFullHistory checks that a run leaves no step index on the operations: after one run, a
// serial operation and a second run, every step of the second run and a reader outside it see the whole history
// as pre-run, and an empty second run keeps the operation order.
func TestRunStepsSecondRunSeesFullHistory(t *testing.T) {
	doc := New()
	two := []Step{
		{Name: "a", Build: func(ctx context.Context) error { return stepsBoxes(ctx, doc, 0, 1) }},
		{Name: "b", Build: func(ctx context.Context) error { return stepsBoxes(ctx, doc, 100, 1) }},
	}
	require.NoError(t, RunSteps(t.Context(), doc, two, WithStepWorkers(1)))
	for _, op := range doc.operations {
		require.Equal(t, -1, op.step, "a finished run leaves no step index on an operation")
	}
	_, err := stepsBox(t.Context(), doc, 300, 0, 310, 10) // a serial operation between the runs
	require.NoError(t, err)
	before := doc.OperationHistory().Operations()
	require.Len(t, before, 5)

	var in0, in1, outside int
	second := []Step{
		{Name: "a2", Build: func(ctx context.Context) error {
			in0 = len(doc.historyAt(ctx).operations)
			outside = len(doc.OperationHistory().Operations())
			return nil
		}},
		{Name: "b2", Build: func(ctx context.Context) error {
			in1 = len(doc.historyAt(ctx).operations)
			return nil
		}},
	}
	require.NoError(t, RunSteps(t.Context(), doc, second, WithStepWorkers(1)))
	require.Equal(t, 5, in0, "step 0 view of the pre-run history")
	require.Equal(t, 5, in1, "step 1 view of the pre-run history")
	require.Equal(t, 5, outside, "outside view of the pre-run history")
	require.True(t, slices.Equal(before, doc.OperationHistory().Operations()),
		"an empty second run must not reorder the history")
}

// TestRunStepsDigestIgnoresStep checks that a body's digest does not depend on the step that built it: the box
// built as step 1, digested while the run is active, equals the same box built serially after the same other box.
func TestRunStepsDigestIgnoresStep(t *testing.T) {
	plain := New()
	_, err := stepsBox(t.Context(), plain, 100, 0, 110, 10)
	require.NoError(t, err)
	second, err := stepsBox(t.Context(), plain, 0, 0, 10, 10)
	require.NoError(t, err)
	want, err := second.Digest(t.Context())
	require.NoError(t, err)

	doc := New()
	var got Digest
	steps := []Step{
		{Name: "other", Build: func(ctx context.Context) error {
			_, err := stepsBox(ctx, doc, 100, 0, 110, 10)
			return err
		}},
		{Name: "box", Build: func(ctx context.Context) error {
			b, err := stepsBox(ctx, doc, 0, 0, 10, 10)
			if err != nil {
				return err
			}
			b.digest.Store(nil)
			got, err = b.Digest(ctx)
			return err
		}},
	}
	require.NoError(t, RunSteps(t.Context(), doc, steps, WithStepWorkers(1)))
	require.Equal(t, want, got)
}

// TestRunStepsInfillSkinRefusesPreRunBody checks that Infill with a skin cannot retire a body that was live before
// the run, and that the refusal fails the run.
func TestRunStepsInfillSkinRefusesPreRunBody(t *testing.T) {
	doc := New()
	block, err := stepsBox(t.Context(), doc, 5, 5, 15, 15)
	require.NoError(t, err)
	before := len(doc.Bodies())
	var stepErr error
	steps := []Step{{Name: "infill", Build: func(ctx context.Context) error {
		_, stepErr = Infill(block, GyroidInfill, units.Millimeters(10), units.Scalar(0.6)).
			Resolution(8).Skin(units.Millimeters(1)).Do(ctx)
		return nil // the step swallows the refusal
	}}}
	runErr := RunSteps(t.Context(), doc, steps)
	require.ErrorIs(t, stepErr, ErrStepScope)
	require.ErrorIs(t, runErr, ErrStepScope, "a swallowed refusal still fails the run")
	require.True(t, doc.n.IsLive(&block.n), "a step must not retire a pre-run body")
	require.Len(t, doc.Bodies(), before, "the refused call leaves no body behind")
}

// TestRunStepsDraftMeshPrismRefusesPreRunBody is the same check for the mesh prism route of Draft, and shows the
// route still works on a body the step built.
func TestRunStepsDraftMeshPrismRefusesPreRunBody(t *testing.T) {
	doc := New()
	block, err := stepsMeshBlock(t.Context(), doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 8))
	require.NoError(t, err)
	var stepErr error
	refused := []Step{{Name: "draft", Build: func(ctx context.Context) error {
		_, stepErr = stepsDraftBlock(ctx, block)
		return nil
	}}}
	runErr := RunSteps(t.Context(), doc, refused)
	require.ErrorIs(t, stepErr, ErrStepScope)
	require.ErrorIs(t, runErr, ErrStepScope)
	require.True(t, doc.n.IsLive(&block.n), "a step must not retire a pre-run body")
	require.Len(t, doc.Bodies(), 1)

	var drafted, own *Body
	own1 := []Step{{Name: "own", Build: func(ctx context.Context) error {
		var err error
		if own, err = stepsMeshBlock(ctx, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 8)); err != nil {
			return err
		}
		drafted, err = stepsDraftBlock(ctx, own)
		return err
	}}}
	require.NoError(t, RunSteps(t.Context(), doc, own1))
	require.False(t, doc.n.IsLive(&own.n), "the step retires the block it built")
	require.True(t, doc.n.IsLive(&drafted.n))
}

// TestRunStepsSwallowedScopeRefusalFailsRun checks that each ErrStepScope refusal of stepGate also fails the run,
// so a step that ignores the refusal cannot produce a build that differs from a serial one.
func TestRunStepsSwallowedScopeRefusalFailsRun(t *testing.T) {
	move, err := r3.Translation(r3.Vec{X: 1, Y: 1, Z: 1})
	require.NoError(t, err)
	t.Run("input built by another step", func(t *testing.T) {
		doc := New()
		var shared *Body
		var refusal error
		steps := []Step{
			{Name: "owner", Build: func(ctx context.Context) error {
				var err error
				shared, err = stepsBox(ctx, doc, 0, 0, 10, 10)
				return err
			}},
			{Name: "reader", Build: func(ctx context.Context) error {
				mine, err := stepsBox(ctx, doc, 5, 0, 15, 10)
				if err != nil {
					return err
				}
				_, refusal = Union(mine, shared).Do(ctx)
				return nil
			}},
		}
		runErr := RunSteps(t.Context(), doc, steps, WithStepWorkers(1))
		require.ErrorIs(t, refusal, ErrStepScope)
		require.ErrorIs(t, runErr, ErrStepScope)
	})
	t.Run("consuming a pre-run body", func(t *testing.T) {
		doc := New()
		pre, err := stepsBox(t.Context(), doc, 0, 0, 10, 10)
		require.NoError(t, err)
		var refusal error
		steps := []Step{{Name: "consumer", Build: func(ctx context.Context) error {
			_, refusal = Transform(pre, move).Do(ctx)
			return nil
		}}}
		runErr := RunSteps(t.Context(), doc, steps)
		require.ErrorIs(t, refusal, ErrStepScope)
		require.ErrorIs(t, runErr, ErrStepScope)
		require.True(t, doc.n.IsLive(&pre.n))
	})
	t.Run("consuming a body of another step", func(t *testing.T) {
		doc := New()
		var shared *Body
		var refusal error
		steps := []Step{
			{Name: "owner", Build: func(ctx context.Context) error {
				var err error
				shared, err = stepsBox(ctx, doc, 0, 0, 10, 10)
				return err
			}},
			{Name: "thief", Build: func(ctx context.Context) error {
				refusal = Remove(shared).Do(ctx)
				return nil
			}},
		}
		runErr := RunSteps(t.Context(), doc, steps, WithStepWorkers(1))
		require.ErrorIs(t, refusal, ErrStepScope)
		require.ErrorIs(t, runErr, ErrStepScope)
	})
	t.Run("a finished step's ctx", func(t *testing.T) {
		doc := New()
		var first context.Context
		var refusal error
		steps := []Step{
			{Name: "first", Build: func(ctx context.Context) error {
				first = ctx //nolint:fatcontext // the test keeps a finished step's ctx on purpose
				return nil
			}},
			{Name: "second", Build: func(context.Context) error {
				_, refusal = stepsBox(first, doc, 0, 0, 10, 10)
				return nil
			}},
		}
		runErr := RunSteps(t.Context(), doc, steps, WithStepWorkers(1))
		require.ErrorIs(t, refusal, ErrStepScope)
		require.ErrorIs(t, runErr, ErrStepScope)
	})
}

// TestRunStepsPanicBecomesError checks that a panic in one Build is returned as that step's error, that the other
// steps' work is kept and the run is ended.
func TestRunStepsPanicBecomesError(t *testing.T) {
	doc := New()
	steps := []Step{
		{Name: "boom", Build: func(context.Context) error { panic("boom") }},
		{Name: "fine", Build: func(ctx context.Context) error { return stepsBoxes(ctx, doc, 100, 1) }},
	}
	err := runStepsWithin(t, doc, steps, WithStepWorkers(1))
	require.Error(t, err)
	require.Contains(t, err.Error(), `step "boom"`)
	require.Contains(t, err.Error(), "panicked: boom")
	require.Nil(t, doc.n.ActiveRun(), "EndRun still runs")
	require.Len(t, doc.Bodies(), 1, "the other step's work is kept")
}

// TestRunStepsGoexitBecomesError checks that a Build that ends its goroutine with runtime.Goexit is reported as an
// error, and that the dispatcher still hands the remaining steps to the single worker.
func TestRunStepsGoexitBecomesError(t *testing.T) {
	doc := New()
	steps := []Step{
		{Name: "exit", Build: func(context.Context) error { runtime.Goexit(); return nil }},
		{Name: "fine", Build: func(ctx context.Context) error { return stepsBoxes(ctx, doc, 100, 1) }},
		{Name: "exit2", Build: func(context.Context) error { runtime.Goexit(); return nil }},
	}
	err := runStepsWithin(t, doc, steps, WithStepWorkers(1))
	require.Error(t, err)
	require.Contains(t, err.Error(), `step "exit"`)
	require.Contains(t, err.Error(), `step "exit2"`)
	require.True(t, strings.Contains(err.Error(), "Goexit"))
	require.False(t, errors.Is(err, ErrStepScope))
	require.Nil(t, doc.n.ActiveRun())
	require.Len(t, doc.Bodies(), 1)
}

// runStepsWithin runs RunSteps and fails the test when it does not return within a minute.
func runStepsWithin(t *testing.T, doc *Document, steps []Step, opts ...RunStepsOption) error {
	t.Helper()
	done := make(chan error, 1)
	go func() { done <- RunSteps(t.Context(), doc, steps, opts...) }()
	select {
	case err := <-done:
		return err
	case <-time.After(time.Minute):
		require.FailNow(t, "RunSteps did not return")
		return nil
	}
}

// TestRunStepsFullSnapshotRestoreFailsRun checks that restoring a snapshot taken outside the run, which would wipe
// the steps' work, marks the run failed.
func TestRunStepsFullSnapshotRestoreFailsRun(t *testing.T) {
	doc := New()
	snap := doc.snapshot(t.Context())
	steps := []Step{{Name: "restore", Build: func(ctx context.Context) error {
		doc.restore(snap)
		return nil
	}}}
	require.ErrorIs(t, RunSteps(t.Context(), doc, steps), ErrStepScope)
}

// TestRunStepsPrivateToolRemovalNeedsNoGate checks that retiring a private tool body, which never enters the
// live set and carries no step tag, is no scope fault inside a step.
func TestRunStepsPrivateToolRemovalNeedsNoGate(t *testing.T) {
	doc := New()
	steps := []Step{{Name: "tools", Build: func(ctx context.Context) error {
		toolCtx, _ := privateConstructionContext(ctx)
		tool, err := stepsBox(toolCtx, doc, 0, 0, 10, 10)
		if err != nil {
			return err
		}
		removeTools(ctx, []*Body{tool})
		return nil
	}}}
	require.NoError(t, RunSteps(t.Context(), doc, steps))
	require.Empty(t, doc.Bodies())
}

// TestRunStepsFullSnapshotRestoreKeepsStepWork checks that a full snapshot restored while steps run fails the run
// and leaves what the steps built in place.
func TestRunStepsFullSnapshotRestoreKeepsStepWork(t *testing.T) {
	doc := New()
	snap := doc.snapshot(t.Context())
	var built *Body
	steps := []Step{{Name: "restore", Build: func(ctx context.Context) error {
		var err error
		if built, err = stepsBox(ctx, doc, 0, 0, 10, 10); err != nil {
			return err
		}
		doc.restore(snap)
		return nil
	}}}
	require.ErrorIs(t, RunSteps(t.Context(), doc, steps), ErrStepScope)
	require.Equal(t, []*Body{built}, doc.Bodies())
	require.Len(t, doc.OperationHistory().Operations(), 1)
}

// TestRunStepsSnapshotRestoreAfterRunIsNoop checks that a step snapshot restored after its run ended, as a
// goroutine that outlives its step would, changes nothing.
func TestRunStepsSnapshotRestoreAfterRunIsNoop(t *testing.T) {
	doc := New()
	var snap *documentSnapshot
	steps := []Step{{Name: "snap", Build: func(ctx context.Context) error {
		if _, err := stepsBox(ctx, doc, 0, 0, 10, 10); err != nil {
			return err
		}
		snap = doc.snapshot(ctx)
		return nil
	}}}
	require.NoError(t, RunSteps(t.Context(), doc, steps))
	before := slices.Clone(doc.Bodies())
	ops := len(doc.OperationHistory().Operations())
	doc.restore(snap)
	require.Equal(t, before, doc.Bodies())
	require.Len(t, doc.OperationHistory().Operations(), ops)
}

// TestRunStepsStaleRunContextMessage checks that a step ctx kept from an earlier run, used while a later run is
// active, is refused with a message that names the previous run.
func TestRunStepsStaleRunContextMessage(t *testing.T) {
	doc := New()
	var kept context.Context
	require.NoError(t, RunSteps(t.Context(), doc, []Step{{Name: "keep", Build: func(ctx context.Context) error {
		kept = ctx //nolint:fatcontext // the test keeps the step ctx to use it in a later run
		return nil
	}}}))
	var refusal error
	err := RunSteps(t.Context(), doc, []Step{{Name: "late", Build: func(context.Context) error {
		_, refusal = stepsBox(kept, doc, 0, 0, 10, 10)
		return nil
	}}})
	require.ErrorIs(t, refusal, ErrStepScope)
	require.ErrorIs(t, err, ErrStepScope)
	require.Contains(t, refusal.Error(), "previous run")
	require.NotContains(t, refusal.Error(), "without its step's context")
}
