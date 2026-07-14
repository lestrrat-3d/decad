package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// This is a deliberate white-box exception to the external-test-package rule:
// no shipped feature records a selector into a Document until fillet/chamfer/
// shell land (docs/evaluator-design.md §11 increment 5), so the clone seam —
// cloneStep/cloneSelectors keeping Recipe a value when a Step carries a
// query — is unreachable from the external API today. Exporting an ingress
// just to test it would add the very aliasing footgun the clone defends
// against. Everything reachable externally is tested in selector_test.go.

func TestCloneStepIsolatesCallerHeldSelectors(t *testing.T) {
	// A feature call records the caller's query by cloning the step; mutating
	// the caller's query afterwards must not change the recorded step.
	q := Edges(Convex()).Exactly(4)
	recorded := cloneStep(Step{Op: OpFillet, Selectors: []Selector{q}})

	require.NotSame(t, q, recorded.Selectors[0],
		`the recorded query is a fresh value, never the caller's pointer`)

	q.preds[0] = Concave()
	q.AtLeast(1)
	got, ok := recorded.Selectors[0].(*EdgeQuery)
	require.True(t, ok)
	require.Equal(t, []EdgePredicate{Convex()}, got.preds,
		`the recorded predicates are the ones given at the call`)
	require.Equal(t, cardinality{kind: cardExactly, n: 4}, got.card,
		`the recorded cardinality assertion is the one given at the call`)
}

func TestRecipeSelectorsDoNotEscapeTheDocument(t *testing.T) {
	// Mutating a returned Recipe's selector must not change the document's
	// own record.
	doc := &Document{steps: []Step{{
		Op:        OpShell,
		Inputs:    []StepRef{0},
		Selectors: []Selector{Faces(Planar()).Exactly(1)},
	}}}

	stolen := doc.Recipe()
	sq, ok := stolen.Steps[0].Selectors[0].(*FaceQuery)
	require.True(t, ok)
	require.NotSame(t, doc.steps[0].Selectors[0], stolen.Steps[0].Selectors[0],
		`Recipe() hands out a fresh query, never the document's pointer`)

	sq.preds[0] = Cylindrical()
	sq.AtLeast(3)

	fresh, ok := doc.Recipe().Steps[0].Selectors[0].(*FaceQuery)
	require.True(t, ok)
	require.Equal(t, []FacePredicate{Planar()}, fresh.preds,
		`the document's record is isolated from caller mutation`)
	require.Equal(t, cardinality{kind: cardExactly, n: 1}, fresh.card)
}
