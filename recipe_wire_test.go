package decad_test

import (
	"encoding/json"
	"maps"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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

// validLoftStep builds a structurally valid (wire-shape only, never
// geometrically evaluated by this PR) OpLoft step: a square LineSeg profile
// on one plane paired with the same square on a second, parallel plane, plus
// a nonzero per-loop alignment offset — exercising every LoftOpts field.
func validLoftStep() decad.Step {
	square := decad.ProfileRecord{
		Outer: decad.LoopRecord{
			Segments: []decad.CurveSegment{
				decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: decad.Point2{U: 1, V: 0}, End: decad.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: decad.Point2{U: 1, V: 1}, End: decad.Point2{U: 0, V: 1}, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: decad.Point2{U: 0, V: 1}, End: decad.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
			},
		},
	}
	plane0 := decad.PlaneRecord{U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	plane1 := decad.PlaneRecord{Origin: r3.NewVec(0, 0, 1), U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}

	return decad.Step{
		Op:      decad.OpLoft,
		Profile: square,
		Plane:   plane0,
		Opts: decad.LoftOpts{
			Profile2:  square,
			Plane2:    plane1,
			Alignment: []int{2},
		},
	}
}

// TestLoftOptsRoundTripsThroughVersion2Envelope covers docs/loft-design.md
// §13's recipe/replay requirement: a LoftOpts payload, including a nonzero
// Alignment, survives a full Recipe encode/decode round trip field for
// field, and the envelope it round-trips through is canonical version 2.
func TestLoftOptsRoundTripsThroughVersion2Envelope(t *testing.T) {
	step := validLoftStep()
	recipe := decad.Recipe{Steps: []decad.Step{step}}

	data, err := json.Marshal(recipe)
	require.NoError(t, err)

	var envelope struct {
		Format  string `json:"format"`
		Version int    `json:"version"`
	}
	require.NoError(t, json.Unmarshal(data, &envelope))
	require.Equal(t, "decad.recipe", envelope.Format)
	require.Equal(t, 2, envelope.Version)

	var decoded decad.Recipe
	require.NoError(t, json.Unmarshal(data, &decoded))
	require.Equal(t, recipe, decoded)

	decodedOpts, ok := decoded.Steps[0].Opts.(decad.LoftOpts)
	require.True(t, ok)
	require.Equal(t, step.Opts.(decad.LoftOpts).Profile2, decodedOpts.Profile2)
	require.Equal(t, step.Opts.(decad.LoftOpts).Plane2, decodedOpts.Plane2)
	require.Equal(t, []int{2}, decodedOpts.Alignment)
}

// TestLoftOptsRequiresProfile2AndPlane2 covers replay design §2.2: both
// second-section payload fields decode through presence-aware pointer wire
// fields, so a missing key and an explicit JSON null are both rejected —
// neither ever decodes into a zero-value ProfileRecord/PlaneRecord.
func TestLoftOptsRequiresProfile2AndPlane2(t *testing.T) {
	step := validLoftStep()
	buf, err := json.Marshal(step)
	require.NoError(t, err)

	var fields map[string]json.RawMessage
	require.NoError(t, json.Unmarshal(buf, &fields))
	var opts map[string]json.RawMessage
	require.NoError(t, json.Unmarshal(fields["opts"], &opts))

	tests := []struct {
		name  string
		field string
		mode  string // "missing" or "null"
	}{
		{name: "missing profile2", field: "profile2", mode: "missing"},
		{name: "null profile2", field: "profile2", mode: "null"},
		{name: "missing plane2", field: "plane2", mode: "missing"},
		{name: "null plane2", field: "plane2", mode: "null"},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			mutated := make(map[string]json.RawMessage, len(opts))
			maps.Copy(mutated, opts)
			if test.mode == "missing" {
				delete(mutated, test.field)
			} else {
				mutated[test.field] = json.RawMessage(`null`)
			}
			optsWire, err := json.Marshal(mutated)
			require.NoError(t, err)

			stepFields := make(map[string]json.RawMessage, len(fields))
			maps.Copy(stepFields, fields)
			stepFields["opts"] = optsWire
			stepWire, err := json.Marshal(stepFields)
			require.NoError(t, err)

			var decoded decad.Step
			err = json.Unmarshal(stepWire, &decoded)
			require.Error(t, err)
			require.ErrorContains(t, err, test.field)

			var recipe decad.Recipe
			err = json.Unmarshal([]byte(`{"steps":[`+string(stepWire)+`]}`), &recipe)
			require.ErrorIs(t, err, decad.ErrInvalidRecipe)
			require.ErrorContains(t, err, test.field)
		})
	}
}

// TestLoftOptsAlignmentAbsentMeansZeroOffsetPerLoop covers loft design §2:
// omitting WithLoftAlignment's payload means every loop's offset is 0, never
// distinguished from an explicit all-zero list.
func TestLoftOptsAlignmentAbsentMeansZeroOffsetPerLoop(t *testing.T) {
	step := validLoftStep()
	step.Opts = decad.LoftOpts{
		Profile2: step.Opts.(decad.LoftOpts).Profile2,
		Plane2:   step.Opts.(decad.LoftOpts).Plane2,
	}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	require.NotContains(t, string(buf), "alignment")

	var decoded decad.Step
	require.NoError(t, json.Unmarshal(buf, &decoded))
	decodedOpts, ok := decoded.Opts.(decad.LoftOpts)
	require.True(t, ok)
	require.Nil(t, decodedOpts.Alignment)
}

// TestRecipeWireVersion1RejectsLoftStep covers replay design §2.1: version 1
// is the pre-loft vocabulary, so a version-1 (and legacy unversioned)
// envelope carrying a "loft" step is invalid, never reinterpreted with
// version-2 rules.
func TestRecipeWireVersion1RejectsLoftStep(t *testing.T) {
	step := validLoftStep()
	stepWire, err := json.Marshal(step)
	require.NoError(t, err)

	tests := []struct {
		name  string
		input string
	}{
		{name: "legacy unversioned", input: `{"steps":[` + string(stepWire) + `]}`},
		{name: "explicit version 1", input: `{"format":"decad.recipe","version":1,"steps":[` + string(stepWire) + `]}`},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var recipe decad.Recipe
			err := json.Unmarshal([]byte(test.input), &recipe)
			require.ErrorIs(t, err, decad.ErrInvalidRecipe)
			require.NotErrorIs(t, err, decad.ErrUnsupportedRecipeVersion)
		})
	}
}

// TestRecipeWireUnsupportedVersionRejectsBeforeLoftDispatch covers replay
// design §2.1's decoder-compatibility rule stated for a version-1-only
// decoder against a version-2 envelope, generalized one version up for this
// (version 1 + 2 capable) decoder: an envelope declaring an unsupported
// version is rejected by the version gate alone, before any step — loft
// included — is decoded. The fixture's loft step is deliberately incomplete
// (no profile/plane/opts, which would otherwise fail its own shape gate) so
// a passing test proves the version check ran first.
func TestRecipeWireUnsupportedVersionRejectsBeforeLoftDispatch(t *testing.T) {
	input := `{"format":"decad.recipe","version":3,"steps":[{"op":"loft"}]}`
	var recipe decad.Recipe
	err := json.Unmarshal([]byte(input), &recipe)
	require.ErrorIs(t, err, decad.ErrUnsupportedRecipeVersion)
	require.NotErrorIs(t, err, decad.ErrInvalidRecipe)
}

// TestRecipeWireLoftRequiresVersion2 covers replay design §2.1: a version-2
// envelope admits a "loft" step; the same step decoded under legacy/version-1
// rejects.
func TestRecipeWireLoftRequiresVersion2(t *testing.T) {
	step := validLoftStep()
	recipe := decad.Recipe{Steps: []decad.Step{step}}
	data, err := json.Marshal(recipe)
	require.NoError(t, err)

	var decoded decad.Recipe
	require.NoError(t, json.Unmarshal(data, &decoded))
	require.Len(t, decoded.Steps, 1)
	require.Equal(t, decad.OpLoft, decoded.Steps[0].Op)
}
