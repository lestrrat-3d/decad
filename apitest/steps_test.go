package apitest_test

// CI: isolated-tests

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// TestRunStepsDeterministic runs the fixture with one worker and with four, twice, and requires every body to
// agree in origin, face origins, volume and tessellation bits, in Bodies() order.
func TestRunStepsDeterministic(t *testing.T) {
	var first []rsPrint
	for i, workers := range []int{1, 4, 4} {
		doc, _, steps := rsFixture(t)
		require.NoError(t, decad.RunSteps(t.Context(), doc, steps, decad.WithStepWorkers(workers)))
		got, err := rsPrintBodies(t.Context(), doc.Bodies())
		require.NoError(t, err)
		if i == 0 {
			first = got
			require.Greater(t, len(first), 10)
			continue
		}
		rsRequireSame(t, first, got)
	}
}

// TestRunStepsOrder checks Bodies(), the operation list and the container lists after a run: the bodies live
// before it first, then step 0's in commit order, and so on, for every worker count.
func TestRunStepsOrder(t *testing.T) {
	for _, workers := range []int{1, 4} {
		doc, plate, steps := rsFixture(t)
		before := len(doc.OperationHistory().Operations())
		require.NoError(t, decad.RunSteps(t.Context(), doc, steps, decad.WithStepWorkers(workers)))
		bodies := doc.Bodies()
		require.Same(t, plate, bodies[0], "the pre-run body stays first")
		ops := doc.OperationHistory().Operations()
		require.Greater(t, len(ops), before)
		require.Equal(t, doc.Root().Bodies(), bodies, "the root container lists bodies in Bodies() order")
		// Every operation's outputs appear in Bodies() in the order of the operations (outputs that a later
		// operation consumed are gone, so compare the surviving ones).
		position := map[*decad.Body]int{}
		for i, b := range bodies {
			position[b] = i
		}
		last := -1
		for _, op := range ops[before:] {
			for _, out := range op.Outputs() {
				if i, ok := position[out]; ok {
					require.Greater(t, i, last, "operation outputs follow operation order")
					last = i
				}
			}
		}
	}
}

// TestRunStepsView checks that a step sees the bodies live before the run and its own, never another step's: a
// ThroughAll sweep in one step is not stopped by a body the other step built.
func TestRunStepsView(t *testing.T) {
	for _, workers := range []int{1, 2} {
		doc, _, steps := rsFixture(t)
		outer := t.Context()
		var inside struct{ bodies, operations int }
		steps = append(steps, decad.Step{Name: "outside-view", Build: func(ctx context.Context) error {
			if _, err := rsBlock(ctx, doc, 700, 0, 710, 10, 0, rsHeight(4)); err != nil {
				return err
			}
			// A call without the step's ctx reads the document as it stood when the run began.
			inside.bodies = len(doc.Bodies())
			inside.operations = len(doc.OperationHistory().Operations())
			rep, err := doc.Verify(outer)
			if err != nil {
				return err
			}
			if len(rep.Bodies) != 1 {
				return fmt.Errorf("a Verify from outside the steps saw %d bodies, want the plate alone", len(rep.Bodies))
			}
			return nil
		}})
		require.NoError(t, decad.RunSteps(t.Context(), doc, steps, decad.WithStepWorkers(workers)))
		require.Equal(t, 1, inside.bodies, "Bodies() from outside the steps sees the pre-run set")
		require.Equal(t, 1, inside.operations, "OperationHistory() from outside the steps sees the pre-run set")
		bodies := doc.Bodies()
		var through *decad.Body
		for _, b := range bodies {
			v, err := b.Volume()
			require.NoError(t, err)
			if v.Value.Base() == 2000 {
				through = b
			}
		}
		require.NotNil(t, through, "the ThroughAll body spans the plate's bottom to its sketch plane, 10 x 10 x 20")
	}
}

// TestRunStepsMisuse runs one rule of the step contract per subtest and checks the refusal and that the refused
// call changed nothing.
func TestRunStepsMisuse(t *testing.T) {
	// refused runs one step that makes the call and requires ErrStepScope from the call. A call that cannot name
	// its step, consumes a body live before the run or reads or consumes another step's body also fails the run,
	// even when the step ignores the refusal. Only a refused nested RunSteps is the step's to handle.
	refused := func(t *testing.T, failsRun bool, call func(ctx context.Context, doc *decad.Document, plate *decad.Body) error) {
		t.Helper()
		doc := decad.New()
		plate, err := rsBlock(t.Context(), doc, 0, 0, 40, 40, 0, rsHeight(4))
		require.NoError(t, err)
		var inner error
		err = decad.RunSteps(t.Context(), doc, []decad.Step{{Name: "misuse", Build: func(ctx context.Context) error {
			inner = call(ctx, doc, plate)
			return nil
		}}})
		require.ErrorIs(t, inner, decad.ErrStepScope)
		if failsRun {
			require.ErrorIs(t, err, decad.ErrStepScope, "a refused call fails the run even when the step ignores it")
		} else {
			require.NoError(t, err)
		}
		require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused call leaves the document unchanged")
	}
	t.Run("outer context", func(t *testing.T) {
		outer := t.Context()
		refused(t, true, func(_ context.Context, doc *decad.Document, _ *decad.Body) error {
			_, err := rsBlock(outer, doc, 100, 0, 120, 20, 0, rsHeight(4))
			return err
		})
	})
	t.Run("consume a pre-run body", func(t *testing.T) {
		refused(t, true, func(ctx context.Context, _ *decad.Document, plate *decad.Body) error {
			move, err := rsShift(1, 1, 1)
			if err != nil {
				return err
			}
			_, err = decad.Transform(plate, move).Do(ctx)
			return err
		})
	})
	t.Run("remove a pre-run body", func(t *testing.T) {
		refused(t, true, func(ctx context.Context, _ *decad.Document, plate *decad.Body) error {
			return decad.Remove(plate).Do(ctx)
		})
	})
	t.Run("container mutation", func(t *testing.T) {
		refused(t, true, func(_ context.Context, doc *decad.Document, _ *decad.Body) error {
			_, err := doc.Root().NewChild("parts")
			return err
		})
	})
	t.Run("parameter read", func(t *testing.T) {
		refused(t, true, func(_ context.Context, doc *decad.Document, _ *decad.Body) error {
			_, err := doc.Param("width")
			return err
		})
	})
	t.Run("nested RunSteps", func(t *testing.T) {
		refused(t, false, func(ctx context.Context, doc *decad.Document, _ *decad.Body) error {
			return decad.RunSteps(ctx, doc, []decad.Step{{Name: "inner", Build: func(context.Context) error { return nil }}})
		})
	})
	t.Run("another step's body", func(t *testing.T) {
		doc := decad.New()
		plate, err := rsBlock(t.Context(), doc, 0, 0, 40, 40, 0, rsHeight(4))
		require.NoError(t, err)
		handoff := make(chan *decad.Body)
		done := make(chan struct{})
		var inner error
		err = decad.RunSteps(t.Context(), doc, []decad.Step{
			{Name: "builder", Build: func(ctx context.Context) error {
				b, err := rsBlock(ctx, doc, 100, 0, 120, 20, 0, rsHeight(4))
				if err != nil {
					return err
				}
				handoff <- b
				<-done
				return nil
			}},
			{Name: "borrower", Build: func(ctx context.Context) error {
				defer close(done)
				b := <-handoff
				_, inner = decad.Copy(b).Do(ctx)
				return nil
			}},
		}, decad.WithStepWorkers(2))
		require.ErrorIs(t, inner, decad.ErrStepScope)
		require.ErrorIs(t, err, decad.ErrStepScope, "a refusal the step ignores still fails the run")
		require.Len(t, doc.Bodies(), 2, "the plate and the builder's block")
		require.Same(t, plate, doc.Bodies()[0])
	})
	t.Run("step context after its run", func(t *testing.T) {
		doc := decad.New()
		var kept context.Context
		require.NoError(t, decad.RunSteps(t.Context(), doc, []decad.Step{{Name: "keep", Build: func(ctx context.Context) error {
			kept = ctx //nolint:fatcontext // the test keeps the step ctx to use it after the run
			return nil
		}}}))
		_, err := rsBlock(kept, doc, 0, 0, 10, 10, 0, rsHeight(4))
		require.ErrorIs(t, err, decad.ErrStepScope)
		require.Empty(t, doc.Bodies())
	})
}

// TestRunStepsRemoveOwnBody checks that a step may retire a body it built.
func TestRunStepsRemoveOwnBody(t *testing.T) {
	doc := decad.New()
	require.NoError(t, decad.RunSteps(t.Context(), doc, []decad.Step{{Name: "scratch", Build: func(ctx context.Context) error {
		scratch, err := rsBlock(ctx, doc, 0, 0, 10, 10, 0, rsHeight(4))
		if err != nil {
			return err
		}
		if _, err := rsBlock(ctx, doc, 20, 0, 30, 10, 0, rsHeight(4)); err != nil {
			return err
		}
		return decad.Remove(scratch).Do(ctx)
	}}}))
	require.Len(t, doc.Bodies(), 1)
}

// TestRunStepsIdentities checks that a step that uses up its identity block fails with ErrStepIdentities, leaves
// the document unchanged, and does not disturb another step.
func TestRunStepsIdentities(t *testing.T) {
	doc := decad.New()
	var small, large error
	err := decad.RunSteps(t.Context(), doc, []decad.Step{{Name: "small", Build: func(ctx context.Context) error {
		_, small = rsBlock(ctx, doc, 0, 0, 10, 10, 0, rsHeight(4))
		return small
	}}}, decad.WithStepIdentities(4))
	require.ErrorIs(t, err, decad.ErrStepIdentities)
	require.ErrorIs(t, small, decad.ErrStepIdentities)
	require.Empty(t, doc.Bodies(), "the operation that ran out leaves the document unchanged")
	err = decad.RunSteps(t.Context(), doc, []decad.Step{{Name: "large", Build: func(ctx context.Context) error {
		_, large = rsBlock(ctx, doc, 0, 0, 10, 10, 0, rsHeight(4))
		return large
	}}}, decad.WithStepIdentities(1<<20))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
}

// TestRunStepsOptions checks the option errors.
func TestRunStepsOptions(t *testing.T) {
	doc := decad.New()
	noop := []decad.Step{{Name: "noop", Build: func(context.Context) error { return nil }}}
	require.ErrorIs(t, decad.RunSteps(t.Context(), doc, noop, decad.WithStepWorkers(0)), decad.ErrDegenerate)
	require.ErrorIs(t, decad.RunSteps(t.Context(), doc, noop, decad.WithStepIdentities(0)), decad.ErrDegenerate)
	require.ErrorIs(t, decad.RunSteps(t.Context(), doc, append(noop, noop...), decad.WithStepIdentities(1<<62)),
		decad.ErrDegenerate, "a block total past the identity range")
	require.ErrorIs(t, decad.RunSteps(t.Context(), nil, noop), decad.ErrDegenerate)
	//nolint:staticcheck // a nil ctx is the case under test
	require.ErrorIs(t, decad.RunSteps(nil, doc, noop), decad.ErrDegenerate)
	require.ErrorIs(t, decad.RunSteps(t.Context(), doc, []decad.Step{{Name: "no build"}}), decad.ErrDegenerate)
	require.NoError(t, decad.RunSteps(t.Context(), doc, nil))
	require.NoError(t, decad.RunSteps(t.Context(), doc, noop), "a failed option left the document able to run steps")
}

// TestRunStepsErrors checks that every step runs after one fails and that the failures come back joined in step
// order, for one worker and several, and that a canceled ctx stops steps that have not started.
func TestRunStepsErrors(t *testing.T) {
	errFirst, errThird := errors.New("first failed"), errors.New("third failed")
	for _, workers := range []int{1, 3} {
		doc := decad.New()
		ran := make([]bool, 4)
		steps := []decad.Step{
			{Name: "a", Build: func(context.Context) error { ran[0] = true; return errFirst }},
			{Name: "b", Build: func(ctx context.Context) error {
				ran[1] = true
				_, err := rsBlock(ctx, doc, 0, 0, 10, 10, 0, rsHeight(4))
				return err
			}},
			{Name: "c", Build: func(context.Context) error { ran[2] = true; return errThird }},
			{Name: "d", Build: func(context.Context) error { ran[3] = true; return nil }},
		}
		err := decad.RunSteps(t.Context(), doc, steps, decad.WithStepWorkers(workers))
		require.ErrorIs(t, err, errFirst)
		require.ErrorIs(t, err, errThird)
		require.Equal(t, []bool{true, true, true, true}, ran, "every step ran")
		require.Len(t, doc.Bodies(), 1, "the passing step's body is kept")
		msg := err.Error()
		require.Less(t, strings.Index(msg, errFirst.Error()), strings.Index(msg, errThird.Error()), "step order")
		require.Contains(t, msg, `"a"`)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	started := false
	err := decad.RunSteps(ctx, decad.New(), []decad.Step{{Name: "late", Build: func(context.Context) error {
		started = true
		return nil
	}}})
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, started)
}

// TestRunStepsMatchesAlone builds each fixture step alone on a fresh document holding the same pre-run plate and
// requires its bodies to equal, in volume and tessellation bits, the bodies the same step made in the full run.
func TestRunStepsMatchesAlone(t *testing.T) {
	doc, _, steps := rsFixture(t)
	require.NoError(t, decad.RunSteps(t.Context(), doc, steps, decad.WithStepWorkers(4)))
	full, err := rsPrintBodies(t.Context(), doc.Bodies())
	require.NoError(t, err)
	at := 1 // full[0] is the pre-run plate
	for i := range steps {
		aloneDoc, _, aloneSteps := rsFixture(t)
		require.NoError(t, decad.RunSteps(t.Context(), aloneDoc, aloneSteps[i:i+1]))
		alone, err := rsPrintBodies(t.Context(), aloneDoc.Bodies())
		require.NoError(t, err)
		for j, want := range alone[1:] {
			got := full[at+j]
			got.Origin, want.Origin = decad.FeatureRef{}, decad.FeatureRef{}
			got.Faces, want.Faces = 0, 0
			require.Equal(t, want, got, "step %q body %d", steps[i].Name, j)
		}
		at += len(alone) - 1
	}
	require.Equal(t, len(full), at, "the steps' bodies add up to the full run's")
}
