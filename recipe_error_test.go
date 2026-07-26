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

func validRecipeStepJSON(t *testing.T, op decad.OpKind, mutate func(map[string]json.RawMessage)) string {
	t.Helper()

	raw, err := json.Marshal(validCodecStep(op))
	require.NoError(t, err)
	var fields map[string]json.RawMessage
	require.NoError(t, json.Unmarshal(raw, &fields))
	if mutate != nil {
		mutate(fields)
	}
	raw, err = json.Marshal(fields)
	require.NoError(t, err)
	return string(raw)
}

func validRecipeJSON(t *testing.T, steps ...string) string {
	t.Helper()
	rawSteps := make([]json.RawMessage, len(steps))
	for i, step := range steps {
		rawSteps[i] = json.RawMessage(step)
	}
	raw, err := json.Marshal(struct {
		Steps []json.RawMessage `json:"steps"`
	}{Steps: rawSteps})
	require.NoError(t, err)
	return string(raw)
}

func TestRecipeDecodeErrorPathIncludesProfilePosition(t *testing.T) {
	profileWithBadHelix := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{
			"outer": {"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}]},
			"holes": [
				{"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}]},
				{"segments": [
					{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1},
					{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1},
					{"kind": "helix"}
				]}
			]
		}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t,
		validRecipeStepJSON(t, decad.OpUnion, nil),
		validRecipeStepJSON(t, decad.OpCut, nil),
		profileWithBadHelix,
	), "steps[2].profile.holes[1].segments[2].kind", 2)

	profileWithBadOuterSegment := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{
			"outer": {"segments": [{"kind": "line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}, "t_start": 0, "t_end": 1}, {"kind": "helix"}]}
		}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, profileWithBadOuterSegment), "steps[0].profile.outer.segments[1].kind", 0)
}

func TestRecipeDecodeErrorPathIncludesSelectorPosition(t *testing.T) {
	shellWithBadSelector := validRecipeStepJSON(t, decad.OpShell, func(fields map[string]json.RawMessage) {
		fields["selectors"] = json.RawMessage(`[{"kind": "faces", "preds": [{"kind": "planar"}, {"kind": "warp"}]}]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, shellWithBadSelector), "steps[0].selectors[0].preds[1].kind", 0)

	filletWithBadRef := validRecipeStepJSON(t, decad.OpFillet, func(fields map[string]json.RawMessage) {
		fields["selectors"] = json.RawMessage(`[{"kind": "edges", "preds": [{"kind": "created_by", "ref": {"step": 0}}]}]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, filletWithBadRef), "steps[0].selectors[0].preds[0].ref.role", 0)
}

func TestRecipeDecodeErrorPathIncludesPredicateRefLeaf(t *testing.T) {
	filletWithBadStepRef := validRecipeStepJSON(t, decad.OpFillet, func(fields map[string]json.RawMessage) {
		fields["selectors"] = json.RawMessage(`[{"kind": "edges", "preds": [{"kind": "convex"}, {"kind": "created_by", "ref": {"step": 1.5, "role": "capStart"}}]}]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, filletWithBadStepRef), "steps[0].selectors[0].preds[1].ref.step", 0)

	filletWithBadRole := validRecipeStepJSON(t, decad.OpFillet, func(fields map[string]json.RawMessage) {
		fields["selectors"] = json.RawMessage(`[{"kind": "edges", "preds": [{"kind": "convex"}, {"kind": "created_by", "ref": {"step": 0, "role": ""}}]}]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, filletWithBadRole), "steps[0].selectors[0].preds[1].ref.role", 0, decad.ErrDegenerate)
}

func TestRecipeDecodeErrorPathIncludesSemanticSegmentField(t *testing.T) {
	profileWithDegenerateLine := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{"outer":{"segments":[{"kind":"line","start":{"u":0,"v":0},"end":{"u":1,"v":0},"t_start":0.5,"t_end":0.5}]}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, profileWithDegenerateLine), "steps[0].profile.outer.segments[0].t_end", 0, decad.ErrDegenerate)

	profileWithDegenerateCircle := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{"outer":{"segments":[{"kind":"circle","center":{"u":0,"v":0},"radius":"1 mm","ccw":false,"t_start":0,"t_end":1}]}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, profileWithDegenerateCircle), "steps[0].profile.outer.segments[0].ccw", 0, decad.ErrDegenerate)
}

func TestRecipeDecodeErrorPathIncludesNestedClosedFields(t *testing.T) {
	extrudeWithBadFaceSelector := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["extent"] = json.RawMessage(`{"kind":"two_sided","one":{"kind":"to_face","body":0,"face":{"kind":"faces","preds":[{"kind":"planar"},{"kind":"warp"}]},"offset":"0 mm"},"two":{"kind":"through_all_side"}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadFaceSelector), "steps[0].extent.one.face.preds[1].kind", 0)

	revolveWithBadEdgeSelector := validRecipeStepJSON(t, decad.OpRevolve, func(fields map[string]json.RawMessage) {
		fields["axis"] = json.RawMessage(`{"kind":"edge_axis","body":0,"edge":{"kind":"edges","preds":[{"kind":"convex"},{"kind":"warp"}]}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, revolveWithBadEdgeSelector), "steps[0].axis.edge.preds[1].kind", 0)

	revolveWithBadAngular := validRecipeStepJSON(t, decad.OpRevolve, func(fields map[string]json.RawMessage) {
		fields["angular"] = json.RawMessage(`{"kind":"two_sided_angle","one":{"kind":"angle_side","a":"30 deg"},"two":{"kind":"warp"}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, revolveWithBadAngular), "steps[0].angular.two.kind", 0)

	extrudeWithBadOpts := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["opts"] = json.RawMessage(`{"kind":"warp"}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadOpts), "steps[0].opts.kind", 0)
}

func TestRecipeDecodeErrorPathIncludesEmptyExtentLeaf(t *testing.T) {
	revolveWithUnexpectedAngularField := validRecipeStepJSON(t, decad.OpRevolve, func(fields map[string]json.RawMessage) {
		fields["angular"] = json.RawMessage(`{"kind":"full_revolution","a":"90 deg"}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, revolveWithUnexpectedAngularField), "steps[0].angular.a", 0)

	extrudeWithUnexpectedExtentField := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["extent"] = json.RawMessage(`{"kind":"two_sided","one":{"kind":"through_all_side","d":"2 mm"},"two":{"kind":"distance_side","d":"5 mm"}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithUnexpectedExtentField), "steps[0].extent.one.d", 0)
}

func TestRecipeDecodeErrorPathIncludesPrimitiveArrayIndex(t *testing.T) {
	unionWithBadInput := validRecipeStepJSON(t, decad.OpUnion, func(fields map[string]json.RawMessage) {
		fields["inputs"] = json.RawMessage(`[0, "bad"]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, unionWithBadInput), "steps[0].inputs[1]", 0)

	filletWithBadValue := validRecipeStepJSON(t, decad.OpFillet, func(fields map[string]json.RawMessage) {
		fields["values"] = json.RawMessage(`["bad"]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, filletWithBadValue), "steps[0].values[0]", 0)
}

func TestRecipeDecodeErrorPathIncludesCustomScalarAndInnerArray(t *testing.T) {
	extrudeWithBadRadius := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{"outer":{"segments":[{"kind":"circle","center":{"u":0,"v":0},"radius":"bad","ccw":true,"t_start":0,"t_end":1}]}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadRadius), "steps[0].profile.outer.segments[0].radius", 0)

	extrudeWithBadControl := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{"outer":{"segments":[{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":"bad","v":0},{"u":2,"v":0}],"knots":[0,0,0,1,1,1],"weights":[1,1,1],"t_start":0,"t_end":1}]}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadControl), "steps[0].profile.outer.segments[0].control[1].u", 0)

	extrudeWithBadDistance := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["extent"] = json.RawMessage(`{"kind":"distance","d":"bad","dir":"along"}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadDistance), "steps[0].extent.d", 0)

	filletWithBadPredicate := validRecipeStepJSON(t, decad.OpFillet, func(fields map[string]json.RawMessage) {
		fields["selectors"] = json.RawMessage(`[{"kind":"edges","preds":[{"kind":"longer_than","l":"bad"}]}]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, filletWithBadPredicate), "steps[0].selectors[0].preds[0].l", 0)

	extrudeWithBadTaper := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["opts"] = json.RawMessage(`{"kind":"extrude","taper":"bad"}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadTaper), "steps[0].opts.taper", 0)
}

func TestRecipeDecodeErrorPathFollowsReturnedCustomFieldError(t *testing.T) {
	extrudeWithBadCircleFields := validRecipeStepJSON(t, decad.OpExtrude, func(fields map[string]json.RawMessage) {
		fields["profile"] = json.RawMessage(`{"outer":{"segments":[{"kind":"circle","ccw":"bad","center":{"u":0,"v":0},"radius":"bad","t_start":0,"t_end":1}]}}`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, extrudeWithBadCircleFields), "steps[0].profile.outer.segments[0].radius", 0)
}

func TestRecipeDecodeErrorPathIncludesStepField(t *testing.T) {
	requireRecipeDecodePath(t, validRecipeJSON(t,
		validRecipeStepJSON(t, decad.OpUnion, nil),
		`{"op":"warp"}`,
	), "steps[1].op", 1)

	requireRecipeDecodePath(t, `{
		"steps": [{"inputs": [0]}]
	}`, "steps[0].op", 0)

	requireRecipeDecodePath(t, `{"steps": "bad"}`, "steps", -1)
}

func TestRecipeDecodeErrorPathUsesStepForForbiddenAxis(t *testing.T) {
	requireRecipeDecodePath(t, `{
		"steps": [{
			"op": "extrude",
			"extent": {"kind": "distance", "d": "1 mm", "dir": "along"},
			"axis": {"kind": "sketch_line", "start": {"u": 0, "v": 0}, "end": {"u": 1, "v": 0}}
		}]
	}`, "steps[0]", 0)
}

func TestRecipeDecodeErrorPreservesSpecificCause(t *testing.T) {
	filletWithDegenerateSelector := validRecipeStepJSON(t, decad.OpFillet, func(fields map[string]json.RawMessage) {
		fields["selectors"] = json.RawMessage(`[{"kind":"edges","preds":[],"exactly":0}]`)
	})
	requireRecipeDecodePath(t, validRecipeJSON(t, filletWithDegenerateSelector), "steps[0].selectors[0].exactly", 0, decad.ErrDegenerate)
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
