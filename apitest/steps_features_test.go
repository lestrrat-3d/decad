package apitest_test

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// TestRunStepsFeaturesMatchSerial runs every feature that builds private tool bodies (Rib, Thread with and without
// a lead-in, Emboss, the wedge chamfer, an Extrude to a tilted face) as a step, with one worker and four, and
// requires each feature's bodies to equal, in volume and tessellation bits, those of the same feature built
// serially on a plain document, with no tool body left live.
func TestRunStepsFeaturesMatchSerial(t *testing.T) {
	features := rsFeatures()
	var want []rsPrint
	for _, f := range features {
		doc := decad.New()
		target, err := f.target(t.Context(), doc)
		require.NoError(t, err, f.name)
		require.NoError(t, f.apply(t.Context(), doc, target), f.name)
		prints, err := rsPrintBodies(t.Context(), doc.Bodies())
		require.NoError(t, err, f.name)
		require.NotEmpty(t, prints, f.name)
		want = append(want, prints...)
	}
	for _, workers := range []int{1, 4} {
		doc := decad.New()
		steps := make([]decad.Step, len(features))
		for i, f := range features {
			steps[i] = decad.Step{Name: f.name, Build: func(ctx context.Context) error {
				target, err := f.target(ctx, doc)
				if err != nil {
					return err
				}
				return f.apply(ctx, doc, target)
			}}
		}
		require.NoError(t, decad.RunSteps(t.Context(), doc, steps, decad.WithStepWorkers(workers)))
		got, err := rsPrintBodies(t.Context(), doc.Bodies())
		require.NoError(t, err)
		require.Len(t, got, len(want), "%d workers: a tool body leaked or a result is missing", workers)
		for i := range want {
			got[i].Origin, want[i].Origin = decad.FeatureRef{}, decad.FeatureRef{}
			got[i].Faces, want[i].Faces = 0, 0
		}
		rsRequireSame(t, want, got)
	}
}

// TestRunStepsFeaturesRefusePreRunBody runs each feature on a body that was live before the run. A feature that
// consumes its body fails the run with ErrStepScope and leaves only the pre-run body. An Extrude to a tilted face
// only reads its stop, so it succeeds, leaves the stop and adds its result.
func TestRunStepsFeaturesRefusePreRunBody(t *testing.T) {
	for _, f := range rsFeatures() {
		t.Run(f.name, func(t *testing.T) {
			doc := decad.New()
			target, err := f.target(t.Context(), doc)
			require.NoError(t, err)
			before := len(doc.Bodies())
			var stepErr error
			err = decad.RunSteps(t.Context(), doc, []decad.Step{{Name: f.name, Build: func(ctx context.Context) error {
				stepErr = f.apply(ctx, doc, target)
				return nil // the step swallows the refusal
			}}})
			if f.readsOnly {
				require.NoError(t, stepErr)
				require.NoError(t, err)
				require.Len(t, doc.Bodies(), before+1, "the stop and the result")
				return
			}
			require.ErrorIs(t, stepErr, decad.ErrStepScope)
			require.ErrorIs(t, err, decad.ErrStepScope, "a swallowed refusal still fails the run")
			require.Equal(t, []*decad.Body{target}, doc.Bodies(), "only the pre-run body is left, no tool body")
		})
	}
}
