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
		name  string
		set   func(*recipeDecodeLimits)
		exact string
		over  string
	}{
		{
			name:  "steps",
			set:   func(l *recipeDecodeLimits) { l.MaxSteps = 1 },
			exact: `{"steps":[{}]}`,
			over:  `{"steps":[{},{}]}`,
		},
		{
			name:  "loops",
			set:   func(l *recipeDecodeLimits) { l.MaxLoops = 1 },
			exact: `{"steps":[{"profile":{"outer":{"segments":[]}}}]}`,
			over:  `{"steps":[{"profile":{"outer":{"segments":[]},"holes":[{"segments":[]}]}}]}`,
		},
		{
			name:  "segments",
			set:   func(l *recipeDecodeLimits) { l.MaxSegments = 1 },
			exact: `{"steps":[{"profile":{"outer":{"segments":[{}]}}}]}`,
			over:  `{"steps":[{"profile":{"outer":{"segments":[{},{}]}}}]}`,
		},
		{
			name:  "curve points",
			set:   func(l *recipeDecodeLimits) { l.MaxCurvePoints = 1 },
			exact: `{"steps":[{"profile":{"outer":{"segments":[{"kind":"spline","control":[{}]}]}}}]}`,
			over:  `{"steps":[{"profile":{"outer":{"segments":[{"kind":"spline","control":[{},{}]}]}}}]}`,
		},
		{
			name:  "curve scalars",
			set:   func(l *recipeDecodeLimits) { l.MaxCurveScalars = 1 },
			exact: `{"steps":[{"profile":{"outer":{"segments":[{"kind":"nurbs","knots":[0]}]}}}]}`,
			over:  `{"steps":[{"profile":{"outer":{"segments":[{"kind":"nurbs","knots":[0,1]}]}}}]}`,
		},
		{
			name:  "selectors",
			set:   func(l *recipeDecodeLimits) { l.MaxSelectors = 1 },
			exact: `{"steps":[{"selectors":[{"kind":"edges","preds":[]}]}]}`,
			over:  `{"steps":[{"selectors":[{"kind":"edges","preds":[]}],"extent":{"kind":"to_face","face":{"kind":"faces","preds":[]}}}]}`,
		},
		{
			name: "nested selectors",
			set:  func(l *recipeDecodeLimits) { l.MaxSelectors = 2 },
			exact: `{"steps":[{"extent":{"kind":"to_face","face":{"kind":"faces","preds":[]}},
				"angular":{"kind":"to_face_angular","face":{"kind":"faces","preds":[]}}}]}`,
			over: `{"steps":[{"extent":{"kind":"to_face","face":{"kind":"faces","preds":[]}},
				"angular":{"kind":"to_face_angular","face":{"kind":"faces","preds":[]}},
				"axis":{"kind":"edge_axis","edge":{"kind":"edges","preds":[]}}}]}`,
		},
		{
			name:  "predicates",
			set:   func(l *recipeDecodeLimits) { l.MaxPredicates = 1 },
			exact: `{"steps":[{"selectors":[{"kind":"edges","preds":[{}]}]}]}`,
			over:  `{"steps":[{"selectors":[{"kind":"edges","preds":[{},{}]}]}]}`,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			limits := defaultRecipeDecodeLimits()
			test.set(&limits)
			require.NoError(t, preflightRecipeJSON([]byte(test.exact), limits))
			require.ErrorIs(t, preflightRecipeJSON([]byte(test.over), limits), ErrResourceLimit)
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
		require.ErrorIs(t, preflightRecipeJSON(exact, limits), ErrResourceLimit)
	})

	t.Run("depth", func(t *testing.T) {
		limits := defaultRecipeDecodeLimits()
		limits.MaxDepth = 2
		require.NoError(t, preflightRecipeJSON([]byte(`{"steps":[]}`), limits))
		require.ErrorIs(t, preflightRecipeJSON([]byte(`{"steps":[{}]}`), limits), ErrResourceLimit)
	})

	t.Run("role bytes", func(t *testing.T) {
		limits := defaultRecipeDecodeLimits()
		limits.MaxRoleBytes = 1
		exact := []byte(`{"steps":[{"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":"x"}}]}]}]}`)
		over := []byte(`{"steps":[{"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":"xx"}}]}]}]}`)
		require.NoError(t, preflightRecipeJSON(exact, limits))
		require.ErrorIs(t, preflightRecipeJSON(over, limits), ErrResourceLimit)
	})

	t.Run("total string bytes", func(t *testing.T) {
		limits := defaultRecipeDecodeLimits()
		limits.MaxTotalStringBytes = 1
		require.NoError(t, preflightRecipeJSON([]byte(`{"steps":[{"op":"x"}]}`), limits))
		require.ErrorIs(t, preflightRecipeJSON([]byte(`{"steps":[{"op":"xx"}]}`), limits), ErrResourceLimit)
	})
}

func TestRecipeUnmarshalRejectsBeforeChangingDestination(t *testing.T) {
	original := Recipe{Steps: []Step{{Op: OpUnion}}}
	got := original
	over := []byte(`{"steps":[` + strings.Repeat(`null,`, defaultRecipeDecodeLimits().MaxSteps) + `null]}`)

	err := json.Unmarshal(over, &got)
	require.ErrorIs(t, err, ErrResourceLimit)
	require.Equal(t, original, got)
}

func TestRecipeUnmarshalDefaultByteLimit(t *testing.T) {
	limits := defaultRecipeDecodeLimits()
	prefix := `{"steps":[],"padding":"`
	suffix := `"}`
	padding := int(limits.MaxBytes) - len(prefix) - len(suffix) + 1
	over := []byte(prefix + strings.Repeat("x", padding) + suffix)

	var got Recipe
	require.ErrorIs(t, json.Unmarshal(over, &got), ErrResourceLimit)
}
