package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func TestRecipeWireCanonicalEmptyEnvelope(t *testing.T) {
	for _, recipe := range []decad.Recipe{
		{},
		{Steps: []decad.Step{}},
	} {
		data, err := json.Marshal(recipe)
		require.NoError(t, err)
		require.JSONEq(t,
			`{"format":"decad.recipe","version":2,"steps":[]}`,
			string(data),
		)
		require.Equal(t,
			`{"format":"decad.recipe","version":2,"steps":[]}`,
			string(data),
			`canonical field order and empty-array normalization are stable`,
		)
	}
}

func TestRecipeWireLegacyV1Compatibility(t *testing.T) {
	for _, input := range []string{
		`{"steps":[]}`,
		`{"steps":null}`,
	} {
		var recipe decad.Recipe
		require.NoError(t, json.Unmarshal([]byte(input), &recipe))
		require.NotNil(t, recipe.Steps)
		require.Empty(t, recipe.Steps)

		data, err := json.Marshal(recipe)
		require.NoError(t, err)
		require.Equal(t,
			`{"format":"decad.recipe","version":2,"steps":[]}`,
			string(data),
			`legacy version 1 always re-encodes canonically`,
		)
	}
}

func TestRecipeWireRejectsUnknownAndDuplicateFields(t *testing.T) {
	tests := []string{
		`{"steps":[],"future_field":true}`,
		`{"format":"decad.recipe","version":1,"steps":[],"future_field":true}`,
		`{"steps":[],"steps":[]}`,
		`{"format":"decad.recipe","format":"decad.recipe","version":1,"steps":[]}`,
		`{"format":"decad.recipe","version":1,"steps":[{"op":"union","op":"cut"}]}`,
	}
	for _, input := range tests {
		var recipe decad.Recipe
		err := json.Unmarshal([]byte(input), &recipe)
		require.Error(t, err, input)
		require.ErrorIs(t, err, decad.ErrInvalidRecipe, input)
		var recipeErr *decad.RecipeError
		require.ErrorAs(t, err, &recipeErr, input)
		require.Equal(t, -1, recipeErr.StepIndex, input)
		require.Equal(t, "$", recipeErr.Path, input)
	}
}

func TestRecipeWireRejectsInvalidEnvelopes(t *testing.T) {
	tests := []struct {
		name      string
		input     string
		target    error
		notTarget error
	}{
		{name: "missing steps", input: `{}`, target: decad.ErrInvalidRecipe},
		{name: "format only", input: `{"format":"decad.recipe","steps":[]}`, target: decad.ErrInvalidRecipe},
		{name: "version only", input: `{"version":1,"steps":[]}`, target: decad.ErrInvalidRecipe},
		{name: "wrong format", input: `{"format":"other","version":1,"steps":[]}`, target: decad.ErrInvalidRecipe},
		{name: "unknown version", input: `{"format":"decad.recipe","version":3,"steps":[]}`, target: decad.ErrUnsupportedRecipeVersion},
		{name: "null version", input: `{"format":"decad.recipe","version":null,"steps":[]}`, target: decad.ErrInvalidRecipe, notTarget: decad.ErrUnsupportedRecipeVersion},
		{name: "unknown version before null steps", input: `{"format":"decad.recipe","version":3,"steps":null}`, target: decad.ErrUnsupportedRecipeVersion, notTarget: decad.ErrInvalidRecipe},
		{name: "null versioned steps", input: `{"format":"decad.recipe","version":1,"steps":null}`, target: decad.ErrInvalidRecipe},
		{name: "non-array steps", input: `{"format":"decad.recipe","version":1,"steps":{}}`, target: decad.ErrInvalidRecipe},
		{name: "trailing value", input: `{"steps":[]} true`},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			original := decad.Recipe{Steps: []decad.Step{{Op: decad.OpUnion}}}
			recipe := original
			err := json.Unmarshal([]byte(test.input), &recipe)
			require.Error(t, err)
			if test.target != nil {
				require.ErrorIs(t, err, test.target)
			}
			if test.notTarget != nil {
				require.NotErrorIs(t, err, test.notTarget)
			}
			require.Equal(t, original, recipe, `failed decode leaves the receiver unchanged`)
		})
	}
}
