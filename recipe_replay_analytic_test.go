package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §14 PR4's replay regression
// guard: recipe-replay-design §8's own contract already allows the analytic
// upgrade with no wire change ("A later evaluator MUST reproduce ... one
// produced body per step ... measurements valid under its own
// Exactness/Bound. It need not reproduce v1's internal payload."). This
// proves that promise for an admitted coplanar Union step: the stored
// OpUnion step's wire shape survives an encode/decode round trip unchanged,
// and re-issuing the feature calls the decoded step names ("replay" —
// docs/recipe-replay-design.md: decad has no replay entry point that
// rebuilds a document from a recipe value, so a caller reads the steps back
// and re-issues them) reproduces the identical Volume/Exactness/Bound the
// analytic dispatch (docs/prism-boolean-design.md §4.2's select-all/merge
// path) already proves for the original.

// buildAdmittedUnionDocument builds prism_boolean_test.go's own
// TestPrismUnionTwoBoxesSharingCapPlaneBuildsAnalyticPrism fixture — two
// coplanar, overlapping boxes sharing a cap plane, (0,0)-(10,10) and
// (5,5)-(15,15), both height 10 — and unions them through the analytic
// select-all/merge path (§4.2): the merged footprint is 100 + 100 - 25 (the
// 5x5 overlap) = 175 mm^2, times the shared height 10 mm = 1750 mm^3,
// Approximate.
func buildAdmittedUnionDocument(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 5, 5, 15, 15, 10)
	union, err := decad.Union(a, b)
	require.NoError(t, err)
	return doc, union
}

func TestRecipeReplayAdmittedUnionStepMatchesOriginal(t *testing.T) {
	doc, union := buildAdmittedUnionDocument(t)
	original, err := union.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1750.0, volumeMM(t, original), 1e-9)

	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 3)
	require.Equal(t, decad.OpUnion, recipe.Steps[2].Op)

	data, err := json.Marshal(recipe)
	require.NoError(t, err)
	var decoded decad.Recipe
	require.NoError(t, json.Unmarshal(data, &decoded))
	require.Equal(t, recipe, decoded,
		`the stored OpUnion step's wire shape survives an encode/decode round trip unchanged`)
	require.Equal(t, decad.OpUnion, decoded.Steps[2].Op)
	require.Equal(t, []decad.StepRef{0, 1}, decoded.Steps[2].Inputs)

	// "Replay": re-issue the feature calls the decoded steps name, exactly as
	// recipe-replay-design.md documents a caller must (decad has no replay
	// entry point that rebuilds a document from a recipe value).
	_, replayed := buildAdmittedUnionDocument(t)
	got, err := replayed.Volume()
	require.NoError(t, err)
	require.Equal(t, original.Value, got.Value, `the replayed step reproduces the original's Volume value`)
	require.Equal(t, original.Exactness, got.Exactness, `the replayed step reproduces the original's Exactness`)
	require.Equal(t, original.Bound, got.Bound, `the replayed step reproduces the original's Bound`)
}
