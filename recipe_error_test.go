package decad_test

import (
	"encoding/json"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func requireRecipeDecodePath(t *testing.T, input, path string, step int, causes ...error) {
	t.Helper()

	var recipe decad.Recipe
	err := json.Unmarshal([]byte(input), &recipe)
	require.Error(t, err)

	var recipeErr *decad.RecipeError
	require.ErrorAs(t, err, &recipeErr)
	require.Equal(t, step, recipeErr.StepIndex)
	require.Equal(t, path, recipeErr.Path)
	require.Equal(t, decad.ErrInvalidRecipe, recipeErr.Kind)
	require.ErrorIs(t, err, decad.ErrInvalidRecipe)
	for _, cause := range causes {
		require.ErrorIs(t, err, cause)
	}
	require.Contains(t, err.Error(), path)
}

func TestRecipeDecodeErrorPathIncludesProfilePosition(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [
			{"op": "union"},
			{"op": "cut"},
			{
				"op": "extrude",
				"profile": {
					"outer": {"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}]},
					"holes": [
						{"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}]},
						{"segments": [
							{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1},
							{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1},
							{"kind": "helix"}
						]}
					]
				}
			}
		]
	}`, "steps[2].profile.holes[1].segments[2].kind", 2)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"profile": {
				"outer": {"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}, {"kind": "helix"}]}
			}
		}]
	}`, "steps[0].profile.outer.segments[1].kind", 0)
}

func TestRecipeDecodeErrorPathIncludesSelectorPosition(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "shell",
			"selectors": [
				{"kind": "edges", "preds": []},
				{"kind": "faces", "preds": [
					{"kind": "planar"},
					{"kind": "warp"}
				]}
			]
		}]
	}`, "steps[0].selectors[1].preds[1].kind", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "fillet",
			"selectors": [{
				"kind": "edges",
				"preds": [{"kind": "created_by", "ref": {"step": 0}}]
			}]
		}]
	}`, "steps[0].selectors[0].preds[0].ref.role", 0)
}

func TestRecipeDecodeErrorPathIncludesPredicateRefLeaf(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "fillet",
			"selectors": [
				{"kind": "edges", "preds": []},
				{"kind": "edges", "preds": [{"kind": "created_by", "ref": {"step": 1.5, "role": "capStart"}}]}
			]
		}]
	}`, "steps[0].selectors[1].preds[0].ref.step", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "fillet",
			"selectors": [
				{"kind": "edges", "preds": []},
				{"kind": "edges", "preds": [{"kind": "created_by", "ref": {"step": 0, "role": ""}}]}
			]
		}]
	}`, "steps[0].selectors[1].preds[0].ref.role", 0, decad.ErrDegenerate)
}

func TestRecipeDecodeErrorPathIncludesSemanticSegmentField(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"profile": {
				"outer": {"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0.5, "t_end": 0.5}]}
			}
		}]
	}`, "steps[0].profile.outer.segments[0].t_end", 0, decad.ErrDegenerate)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"profile": {
				"outer": {"segments": [{"kind": "circle", "center": {"u": 0, "v": 0}, "radius": "1 mm", "ccw": false, "t_start": 0, "t_end": 1}]}
			}
		}]
	}`, "steps[0].profile.outer.segments[0].ccw", 0, decad.ErrDegenerate)
}

func TestRecipeDecodeErrorPathIncludesNestedClosedFields(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"extent": {
				"kind": "two_sided",
				"one": {
					"kind": "to_face",
					"body": 0,
					"face": {
						"kind": "faces",
						"preds": [{"kind": "planar"}, {"kind": "warp"}]
					},
					"offset": "0 mm"
				},
				"two": {"kind": "through_all_side"}
			}
		}]
	}`, "steps[0].extent.one.face.preds[1].kind", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "revolve",
			"axis": {
				"kind": "edge_axis",
				"body": 0,
				"edge": {
					"kind": "edges",
					"preds": [{"kind": "convex"}, {"kind": "warp"}]
				}
			}
		}]
	}`, "steps[0].axis.edge.preds[1].kind", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "revolve",
			"angular": {
				"kind": "two_sided_angle",
				"one": {"kind": "angle_side", "a": "30 deg"},
				"two": {"kind": "warp"}
			}
		}]
	}`, "steps[0].angular.two.kind", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{"op": "extrude", "opts": {"kind": "warp"}}]
	}`, "steps[0].opts.kind", 0)
}

func TestRecipeDecodeErrorPathIncludesEmptyExtentLeaf(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "revolve",
			"angular": {"kind": "full_revolution", "a": "90 deg"}
		}]
	}`, "steps[0].angular.a", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"extent": {
				"kind": "two_sided",
				"one": {"kind": "through_all_side", "d": "2 mm"},
				"two": {"kind": "distance_side", "d": "5 mm"}
			}
		}]
	}`, "steps[0].extent.one.d", 0)
}

func TestRecipeDecodeErrorPathIncludesPrimitiveArrayIndex(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{"op": "union", "inputs": [0, "bad"]}]
	}`, "steps[0].inputs[1]", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{"op": "fillet", "values": ["bad"]}]
	}`, "steps[0].values[0]", 0)
}

func TestRecipeDecodeErrorPathIncludesCustomScalarAndInnerArray(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"profile": {
				"outer": {"segments": [{"kind": "circle", "center": {"u": 0, "v": 0}, "radius": "bad", "ccw": true, "t_start": 0, "t_end": 1}]}
			}
		}]
	}`, "steps[0].profile.outer.segments[0].radius", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"profile": {
				"outer": {"segments": [{
					"kind": "nurbs",
					"degree": 2,
					"control": [{"u": 0, "v": 0}, {"u": "bad", "v": 0}, {"u": 2, "v": 0}],
					"knots": [0, 0, 0, 1, 1, 1],
					"weights": [1, 1, 1],
					"t_start": 0,
					"t_end": 1
				}]}
			}
		}]
	}`, "steps[0].profile.outer.segments[0].control[1].u", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"extent": {"kind": "distance", "d": "bad", "dir": "along"}
		}]
	}`, "steps[0].extent.d", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "fillet",
			"selectors": [{
				"kind": "edges",
				"preds": [{"kind": "longer_than", "l": "bad"}]
			}]
		}]
	}`, "steps[0].selectors[0].preds[0].l", 0)

	requireRecipeDecodePath(t, `{
		"steps": [{"op": "extrude", "opts": {"kind": "extrude", "taper": "bad"}}]
	}`, "steps[0].opts.taper", 0)
}

func TestRecipeDecodeErrorPathIncludesStepField(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{"op": "union"}, {"op": "warp"}]
	}`, "steps[1].op", 1)

	requireRecipeDecodePath(t, `{
		"steps": [{"inputs": [0]}]
	}`, "steps[0].op", 0)

	requireRecipeDecodePath(t, `{"steps": "bad"}`, "steps", -1)
}

func TestRecipeDecodeErrorPreservesSpecificCause(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "fillet",
			"selectors": [{"kind": "edges", "preds": [], "exactly": 0}]
		}]
	}`, "steps[0].selectors[0].exactly", 0, decad.ErrDegenerate)
}

func TestLoopRecordDecodeErrorIncludesSegmentIndex(t *testing.T) {
	var loop decad.LoopRecord
	err := json.Unmarshal([]byte(`{
		"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}, {"kind": "helix"}]
	}`), &loop)
	require.Error(t, err)
	require.Contains(t, err.Error(), "segments[1].kind")
	require.False(t, errors.Is(err, decad.ErrInvalidRecipe),
		`a standalone loop error has no recipe root and is not a RecipeError`)
}
