package decad

import (
	"encoding/json"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestDefaultRecipeDecodeLimits(t *testing.T) {
	require.Equal(t, recipeDecodeLimits{
		MaxBytes:            16 << 20,
		MaxDepth:            32,
		MaxSteps:            4_096,
		MaxLoops:            65_536,
		MaxSegments:         262_144,
		MaxCurvePoints:      1_048_576,
		MaxCurveScalars:     1_048_576,
		MaxSelectors:        16_384,
		MaxPredicates:       131_072,
		MaxRoleBytes:        256,
		MaxTotalStringBytes: 1 << 20,
	}, defaultRecipeDecodeLimits())
}

func TestRecipeDecodePreflightCollectionLimits(t *testing.T) {
	tests := []struct {
		name      string
		set       func(*recipeDecodeLimits)
		exact     string
		over      string
		path      string
		stepIndex int
	}{
		{
			name:      "steps",
			set:       func(l *recipeDecodeLimits) { l.MaxSteps = 1 },
			exact:     `{"steps":[{}]}`,
			over:      `{"steps":[{},{}]}`,
			path:      "steps[1]",
			stepIndex: 1,
		},
		{
			name:      "loops",
			set:       func(l *recipeDecodeLimits) { l.MaxLoops = 1 },
			exact:     `{"steps":[{"profile":{"outer":{"segments":[]}}}]}`,
			over:      `{"steps":[{"profile":{"outer":{"segments":[]},"holes":[{"segments":[]}]}}]}`,
			path:      "steps[0].profile.holes[0]",
			stepIndex: 0,
		},
		{
			name:      "segments",
			set:       func(l *recipeDecodeLimits) { l.MaxSegments = 1 },
			exact:     `{"steps":[{"profile":{"outer":{"segments":[{}]}}}]}`,
			over:      `{"steps":[{"profile":{"outer":{"segments":[{},{}]}}}]}`,
			path:      "steps[0].profile.outer.segments[1]",
			stepIndex: 0,
		},
		{
			name:      "curve points",
			set:       func(l *recipeDecodeLimits) { l.MaxCurvePoints = 1 },
			exact:     `{"steps":[{"profile":{"outer":{"segments":[{"kind":"spline","control":[{}]}]}}}]}`,
			over:      `{"steps":[{"profile":{"outer":{"segments":[{"kind":"spline","control":[{},{}]}]}}}]}`,
			path:      "steps[0].profile.outer.segments[0].control[1]",
			stepIndex: 0,
		},
		{
			name:      "curve scalars",
			set:       func(l *recipeDecodeLimits) { l.MaxCurveScalars = 1 },
			exact:     `{"steps":[{"profile":{"outer":{"segments":[{"kind":"nurbs","knots":[0]}]}}}]}`,
			over:      `{"steps":[{"profile":{"outer":{"segments":[{"kind":"nurbs","knots":[0,1]}]}}}]}`,
			path:      "steps[0].profile.outer.segments[0].knots[1]",
			stepIndex: 0,
		},
		{
			name:      "selectors",
			set:       func(l *recipeDecodeLimits) { l.MaxSelectors = 1 },
			exact:     `{"steps":[{"selectors":[{"kind":"edges","preds":[]}]}]}`,
			over:      `{"steps":[{"selectors":[{"kind":"edges","preds":[]}],"extent":{"kind":"to_face","face":{"kind":"faces","preds":[]}}}]}`,
			path:      "steps[0].extent.face",
			stepIndex: 0,
		},
		{
			name: "nested selectors",
			set:  func(l *recipeDecodeLimits) { l.MaxSelectors = 2 },
			exact: `{"steps":[{"extent":{"kind":"to_face","face":{"kind":"faces","preds":[]}},
				"angular":{"kind":"to_face_angular","face":{"kind":"faces","preds":[]}}}]}`,
			over: `{"steps":[{"extent":{"kind":"to_face","face":{"kind":"faces","preds":[]}},
				"angular":{"kind":"to_face_angular","face":{"kind":"faces","preds":[]}},
				"axis":{"kind":"edge_axis","edge":{"kind":"edges","preds":[]}}}]}`,
			path:      "steps[0].axis.edge",
			stepIndex: 0,
		},
		{
			name:      "predicates",
			set:       func(l *recipeDecodeLimits) { l.MaxPredicates = 1 },
			exact:     `{"steps":[{"selectors":[{"kind":"edges","preds":[{}]}]}]}`,
			over:      `{"steps":[{"selectors":[{"kind":"edges","preds":[{},{}]}]}]}`,
			path:      "steps[0].selectors[0].preds[1]",
			stepIndex: 0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			limits := defaultRecipeDecodeLimits()
			test.set(&limits)
			require.NoError(t, preflightRecipeJSON([]byte(test.exact), limits))
			requireRecipeLimitPath(t, preflightRecipeJSON([]byte(test.over), limits), test.path, test.stepIndex)
		})
	}
}

func TestRecipeDecodePreflightByteDepthAndStringLimits(t *testing.T) {
	t.Run("bytes", func(t *testing.T) {
		exact := []byte(`{"steps":[]}`)
		limits := defaultRecipeDecodeLimits()
		limits.MaxBytes = int64(len(exact))
		require.NoError(t, preflightRecipeJSON(exact, limits))
		limits.MaxBytes--
		requireRecipeLimitPath(t, preflightRecipeJSON(exact, limits), "$", -1)
	})

	t.Run("depth", func(t *testing.T) {
		limits := defaultRecipeDecodeLimits()
		limits.MaxDepth = 2
		require.NoError(t, preflightRecipeJSON([]byte(`{"steps":[]}`), limits))
		requireRecipeLimitPath(t, preflightRecipeJSON([]byte(`{"steps":[{}]}`), limits), "steps[0]", 0)
	})

	t.Run("role bytes", func(t *testing.T) {
		limits := defaultRecipeDecodeLimits()
		limits.MaxRoleBytes = 1
		exact := []byte(`{"steps":[{"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":"x"}}]}]}]}`)
		over := []byte(`{"steps":[{"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":"xx"}}]}]}]}`)
		require.NoError(t, preflightRecipeJSON(exact, limits))
		requireRecipeLimitPath(t, preflightRecipeJSON(over, limits),
			"steps[0].selectors[0].preds[0].ref.role", 0)
	})

	t.Run("total string bytes", func(t *testing.T) {
		limits := defaultRecipeDecodeLimits()
		limits.MaxTotalStringBytes = 1
		require.NoError(t, preflightRecipeJSON([]byte(`{"steps":[{"op":"x"}]}`), limits))
		requireRecipeLimitPath(t, preflightRecipeJSON([]byte(`{"steps":[{"op":"xx"}]}`), limits),
			"steps[0].op", 0)
	})
}

func TestRecipeUnmarshalStructuralCollectionLimits(t *testing.T) {
	original := Recipe{Steps: []Step{{Op: OpUnion}}}

	t.Run("inputs exact limit", func(t *testing.T) {
		var got Recipe
		err := json.Unmarshal([]byte(`{"steps":[{"op":"extrude"},{"op":"extrude","inputs":[0]}]}`), &got)
		require.NoError(t, err)
		require.Equal(t, []StepRef{0}, got.Steps[1].Inputs)
	})

	t.Run("inputs one over", func(t *testing.T) {
		got := original
		err := json.Unmarshal([]byte(`{"steps":[{"op":"extrude"},{"op":"extrude","inputs":[0,0]}]}`), &got)
		requireRecipeLimitPath(t, err, "steps[1].inputs[1]", 1)
		require.Equal(t, original, got)
	})

	t.Run("values exact limit", func(t *testing.T) {
		var got Recipe
		err := json.Unmarshal([]byte(`{"steps":[{"op":"fillet","values":["1 mm"]}]}`), &got)
		require.NoError(t, err)
		require.Len(t, got.Steps[0].Values, 1)
	})

	t.Run("values one over", func(t *testing.T) {
		got := original
		err := json.Unmarshal([]byte(`{"steps":[{"op":"fillet","values":["1 mm","2 mm"]}]}`), &got)
		requireRecipeLimitPath(t, err, "steps[0].values[1]", 0)
		require.Equal(t, original, got)
	})
}

func TestRecipeUnmarshalRejectsBeforeChangingDestination(t *testing.T) {
	original := Recipe{Steps: []Step{{Op: OpUnion}}}

	t.Run("preflight failure", func(t *testing.T) {
		got := original
		over := []byte(`{"steps":[` + strings.Repeat(`null,`, defaultRecipeDecodeLimits().MaxSteps) + `null]}`)

		err := json.Unmarshal(over, &got)
		requireRecipeLimitPath(t, err, "steps[4096]", 4096)
		require.Equal(t, original, got)
	})

	t.Run("typed decode failure", func(t *testing.T) {
		got := original
		err := json.Unmarshal([]byte(`{"steps":[{"op":"cut"},{"op":"extrude","extent":{"kind":"helical"}}]}`), &got)
		require.Error(t, err)
		require.Equal(t, original, got)
	})
}

func TestRecipeUnmarshalPreservesDestinationWhenStepsAbsent(t *testing.T) {
	original := Recipe{Steps: []Step{{Op: OpUnion}}}

	for _, test := range []struct {
		name  string
		input string
	}{
		{name: "null", input: `null`},
		{name: "empty object", input: `{}`},
	} {
		t.Run(test.name, func(t *testing.T) {
			got := original
			require.NoError(t, json.Unmarshal([]byte(test.input), &got))
			require.Equal(t, original, got)
		})
	}
}

func TestRecipeUnmarshalDefaultByteLimit(t *testing.T) {
	limits := defaultRecipeDecodeLimits()
	prefix := `{"steps":[],"padding":"`
	suffix := `"}`
	padding := int(limits.MaxBytes) - len(prefix) - len(suffix) + 1
	over := []byte(prefix + strings.Repeat("x", padding) + suffix)

	var got Recipe
	requireRecipeLimitPath(t, json.Unmarshal(over, &got), "$", -1)
}

func requireRecipeLimitPath(t *testing.T, err error, path string, stepIndex int) {
	t.Helper()
	require.ErrorIs(t, err, ErrResourceLimit)
	var recipeErr *RecipeError
	require.ErrorAs(t, err, &recipeErr)
	require.Equal(t, path, recipeErr.Path)
	require.Equal(t, stepIndex, recipeErr.StepIndex)
	require.Same(t, ErrResourceLimit, recipeErr.Kind)
}
