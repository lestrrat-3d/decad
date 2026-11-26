package decad

// CI: isolated-tests

import (
	"context"
	"fmt"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestRunStepsIDsInBlock checks every identity a step's bodies carry against its block: producers inside
// [P+kB, P+(k+1)B), and every level and curve id the step's bodies carry that a body live before the run does not
// inherit inside (L+kB, L+(k+1)B] and (C+kB, C+(k+1)B]. It also checks the counters after the run.
func TestRunStepsIDsInBlock(t *testing.T) {
	const block = 1 << 20
	doc := New()
	pre, err := stepsBox(t.Context(), doc, 0, 0, 10, 10)
	require.NoError(t, err)
	preLevels, preCurves := stepIDRanges(pre)
	p0, l0, c0 := int(doc.n.NextProducer), int(doc.n.NextLevel), int(doc.n.NextCurve)
	steps := make([]Step, 3)
	for i := range steps {
		steps[i] = Step{Name: stepsLabel(i), Build: func(ctx context.Context) error {
			return stepsBoxes(ctx, doc, 100*float64(i+1), 2)
		}}
	}
	require.NoError(t, RunSteps(t.Context(), doc, steps, WithStepWorkers(3), WithStepIdentities(block)))
	require.Equal(t, p0+3*block, int(doc.n.NextProducer))
	require.Equal(t, l0+3*block, int(doc.n.NextLevel))
	require.Equal(t, c0+3*block, int(doc.n.NextCurve))
	require.Nil(t, doc.n.ActiveRun(), "the run ended")
	require.Empty(t, doc.n.StepTag, "the step tags are dropped")
	bodies := doc.Bodies()
	require.Len(t, bodies, 1+3*2)
	require.Same(t, pre, bodies[0])
	for i, b := range bodies[1:] {
		k := i / 2
		producer := int(b.n.Origin.Producer)
		require.GreaterOrEqual(t, producer, p0+k*block, "body %d producer", i)
		require.Less(t, producer, p0+(k+1)*block, "body %d producer", i)
		levels, curves := stepIDRanges(b)
		require.NotEmpty(t, levels)
		require.NotEmpty(t, curves)
		for id := range levels {
			require.True(t, id > l0+k*block && id <= l0+(k+1)*block, "body %d level %d", i, id)
		}
		for id := range curves {
			require.True(t, id > c0+k*block && id <= c0+(k+1)*block, "body %d curve %d", i, id)
		}
	}
	for id := range preLevels {
		require.LessOrEqual(t, id, l0)
	}
	for id := range preCurves {
		require.LessOrEqual(t, id, c0)
	}
}

// TestRunStepsReadWithoutStepFailsRun checks that a producer read, a level mint and a curve mint made without a
// step's ctx while a run is active mark the run failed, and that the counters stay as they were.
func TestRunStepsReadWithoutStepFailsRun(t *testing.T) {
	read := func(ctx context.Context, doc *Document) {
		_ = doc.n.ProducerAt(ctx)
	}
	mintLevel := func(ctx context.Context, doc *Document) {
		require.Zero(t, doc.n.MintLevel(ctx, r3.Vec{}, r3.Vec{}).ID)
	}
	mintCurve := func(ctx context.Context, doc *Document) {
		require.Zero(t, doc.n.MintCurve(ctx).ID)
	}
	for name, call := range map[string]func(context.Context, *Document){"producer": read, "level": mintLevel, "curve": mintCurve} {
		doc := New()
		level, curve, producer := doc.n.NextLevel, doc.n.NextCurve, doc.n.NextProducer
		err := RunSteps(t.Context(), doc, []Step{{Name: "reader", Build: func(context.Context) error {
			call(t.Context(), doc) //nolint:contextcheck // the call is made without the step's ctx on purpose
			return nil
		}}}, WithStepWorkers(1), WithStepIdentities(8))
		require.ErrorIs(t, err, ErrStepScope, name)
		require.Equal(t, producer+8, doc.n.NextProducer, name)
		require.Equal(t, level+8, doc.n.NextLevel, name)
		require.Equal(t, curve+8, doc.n.NextCurve, name)
	}
}

// TestRunStepsTransientIDsFromStepCursor checks that a query inside a step seeds its transient ids from the
// step's cursors, so they differ from the document's counters and cannot equal the step's committed ids.
func TestRunStepsTransientIDsFromStepCursor(t *testing.T) {
	doc := New()
	const block = 1 << 20
	var seeds [2]struct{ level, curve int }
	steps := make([]Step, 2)
	for i := range steps {
		steps[i] = Step{Name: stepsLabel(i), Build: func(ctx context.Context) error {
			if _, err := stepsBox(ctx, doc, 0, 0, 10, 10); err != nil {
				return err
			}
			s := doc.n.SeedsAt(ctx)
			seeds[i].level, seeds[i].curve = int(s.Level), int(s.Curve)
			if q := doc.n.SeedsAt(doc.queryContext(ctx)); q != s {
				return fmt.Errorf("a query's source starts at %v, want the step's cursor %v", q, s)
			}
			return nil
		}}
	}
	require.NoError(t, RunSteps(t.Context(), doc, steps, WithStepWorkers(2), WithStepIdentities(block)))
	for i, s := range seeds {
		require.Greater(t, s.level, i*block, "step %d level seed is at or above its block", i)
		require.LessOrEqual(t, s.level, (i+1)*block)
		require.Greater(t, s.curve, i*block)
		require.LessOrEqual(t, s.curve, (i+1)*block)
	}
}

// TestRunStepsEndRaceWithQueries checks that RunSteps' end-of-run counter write is safe against a Verify and an
// IntersectionVolume made without a step's ctx on other goroutines while the run goes on and ends.
func TestRunStepsEndRaceWithQueries(t *testing.T) {
	doc := New()
	a, err := stepsBox(t.Context(), doc, 0, 0, 10, 10)
	require.NoError(t, err)
	b, err := stepsBox(t.Context(), doc, 5, 5, 15, 15)
	require.NoError(t, err)
	stop := make(chan struct{})
	var wg sync.WaitGroup
	for range 2 {
		wg.Go(func() {
			for {
				select {
				case <-stop:
					return
				default:
				}
				_, _ = doc.Verify(t.Context())
				_, _ = doc.IntersectionVolume(t.Context(), a, b)
			}
		})
	}
	steps := make([]Step, 4)
	for i := range steps {
		steps[i] = Step{Name: stepsLabel(i), Build: func(ctx context.Context) error {
			return stepsBoxes(ctx, doc, 100*float64(i+1), 1)
		}}
	}
	err = RunSteps(t.Context(), doc, steps, WithStepWorkers(4))
	close(stop)
	wg.Wait()
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2+4)
}

// TestRunStepsSnapshotRestoreIsStepLocal checks that a composite's rollback inside a step undoes only that
// step's own publications while another step publishes beside it.
func TestRunStepsSnapshotRestoreIsStepLocal(t *testing.T) {
	doc := New()
	var kept *Body
	steps := []Step{
		{Name: "rollback", Build: func(ctx context.Context) error {
			var err error
			if kept, err = stepsBox(ctx, doc, 0, 0, 10, 10); err != nil {
				return err
			}
			producer := doc.n.ProducerAt(ctx)
			snap := doc.snapshot(ctx)
			scratch, err := stepsBox(ctx, doc, 20, 0, 30, 10)
			if err != nil {
				return err
			}
			if _, err := Copy(kept).Do(ctx); err != nil {
				return err
			}
			if err := Remove(scratch).Do(ctx); err != nil {
				return err
			}
			doc.restore(snap)
			if got := doc.n.ProducerAt(ctx); got != producer {
				return fmt.Errorf("the step's producer cursor is %d after the rollback, want %d", got, producer)
			}
			return nil
		}},
		{Name: "beside", Build: func(ctx context.Context) error {
			return stepsBoxes(ctx, doc, 100, 3)
		}},
	}
	require.NoError(t, RunSteps(t.Context(), doc, steps, WithStepWorkers(2)))
	bodies := doc.Bodies()
	require.Len(t, bodies, 1+3, "the first step's body and the other step's three")
	require.Same(t, kept, bodies[0])
	require.Len(t, doc.OperationHistory().Operations(), 1+3*2)
	require.Len(t, doc.producers, 1+3*2, "no producer entry of the rolled-back operations is left")
}

// TestRunStepsDispatchOrderDoesNotChangeResults starts the steps in reverse order with one worker and requires
// the same ids and geometry as the ordinary order.
func TestRunStepsDispatchOrderDoesNotChangeResults(t *testing.T) {
	run := func(workers int) []stepsPrint {
		doc := New()
		steps := make([]Step, 4)
		for i := range steps {
			steps[i] = Step{Name: stepsLabel(i), Build: func(ctx context.Context) error {
				return stepsBoxes(ctx, doc, 100*float64(i+1), 2)
			}}
		}
		require.NoError(t, RunSteps(t.Context(), doc, steps, WithStepWorkers(workers)))
		return stepsPrints(t, doc)
	}
	want := run(1)
	saved := stepDispatchOrder
	t.Cleanup(func() { stepDispatchOrder = saved })
	stepDispatchOrder = func(n int) []int {
		order := make([]int, n)
		for i := range order {
			order[i] = n - 1 - i
		}
		return order
	}
	require.Equal(t, want, run(1), "reverse start order, one worker")
	require.Equal(t, want, run(4), "reverse start order, four workers")
}
