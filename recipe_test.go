package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Compile-time sealing: the two extent tiers are disjoint — a side is never a
// standalone extent and vice versa. (TwoSided{One: Distance{}} does not
// compile, which is the §8.1 guarantee; it cannot be asserted in a test that
// must itself compile.)
var (
	_ decad.Extent     = decad.Distance{}
	_ decad.Extent     = decad.ThroughAll{}
	_ decad.Extent     = decad.Symmetric{}
	_ decad.Extent     = decad.TwoSided{}
	_ decad.SideExtent = decad.DistanceSide{}
	_ decad.SideExtent = decad.ThroughAllSide{}
	_ decad.BodyRef    = decad.StepRef(0)
	_ decad.StepOpts   = decad.ExtrudeOpts{}
)

func validCodecStep(op decad.OpKind) decad.Step {
	profile := decad.ProfileRecord{
		Outer: decad.LoopRecord{
			Segments: []decad.CurveSegment{
				decad.CircleSeg{
					Radius: units.Millimeters(1),
					CCW:    true,
					TEnd:   1,
				},
			},
		},
	}
	plane := decad.PlaneRecord{
		U: r3.NewVec(1, 0, 0),
		V: r3.NewVec(0, 1, 0),
	}
	placement := decad.TransformRecord{
		EX: r3.NewVec(1, 0, 0),
		EY: r3.NewVec(0, 1, 0),
		EZ: r3.NewVec(0, 0, 1),
	}

	switch op {
	case decad.OpExtrude:
		return decad.Step{
			Op:      op,
			Profile: profile,
			Plane:   plane,
			Extent:  decad.Distance{D: units.Millimeters(1), Dir: decad.Along},
			Opts:    decad.ExtrudeOpts{Taper: units.Degrees(0)},
		}
	case decad.OpRevolve:
		return decad.Step{
			Op:      op,
			Profile: profile,
			Plane:   plane,
			Angular: decad.FullRevolution{},
			Axis:    decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}},
		}
	case decad.OpUnion, decad.OpCut, decad.OpIntersect:
		return decad.Step{Op: op, Inputs: []decad.StepRef{0, 1}}
	case decad.OpFillet:
		return decad.Step{
			Op:        op,
			Inputs:    []decad.StepRef{0},
			Selectors: []decad.Selector{decad.Edges()},
			Values:    []units.Value{units.Millimeters(1)},
		}
	case decad.OpChamfer:
		return decad.Step{
			Op:        op,
			Inputs:    []decad.StepRef{0},
			Selectors: []decad.Selector{decad.Edges()},
			Values:    []units.Value{units.Millimeters(1)},
		}
	case decad.OpShell:
		return decad.Step{
			Op:        op,
			Inputs:    []decad.StepRef{0},
			Selectors: []decad.Selector{decad.Faces()},
			Opts:      decad.ShellOpts{Sense: decad.Inward},
			Values:    []units.Value{units.Millimeters(1)},
		}
	case decad.OpPlaced:
		return decad.Step{Op: op, Inputs: []decad.StepRef{0}, Placement: placement}
	case decad.OpDuplicate:
		return decad.Step{Op: op, Inputs: []decad.StepRef{0}}
	case decad.OpPlacedCopy:
		return decad.Step{Op: op, Inputs: []decad.StepRef{0}, Placement: placement}
	default:
		return decad.Step{Op: op}
	}
}

func rejectStepMutationBothWays(
	t *testing.T,
	op decad.OpKind,
	mutateStep func(*decad.Step),
	mutateWire func(map[string]json.RawMessage),
) {
	t.Helper()

	step := validCodecStep(op)
	mutateStep(&step)
	_, err := json.Marshal(step)
	require.Error(t, err, `the in-memory shape must refuse to encode`)

	buf, err := json.Marshal(validCodecStep(op))
	require.NoError(t, err)
	var fields map[string]json.RawMessage
	require.NoError(t, json.Unmarshal(buf, &fields))
	mutateWire(fields)
	buf, err = json.Marshal(fields)
	require.NoError(t, err)
	var decoded decad.Step
	require.Error(t, json.Unmarshal(buf, &decoded), `the wire shape must refuse to decode`)
}

func TestStepOperationShapes(t *testing.T) {
	ops := []decad.OpKind{
		decad.OpExtrude,
		decad.OpRevolve,
		decad.OpUnion,
		decad.OpCut,
		decad.OpIntersect,
		decad.OpFillet,
		decad.OpChamfer,
		decad.OpShell,
		decad.OpPlaced,
		decad.OpDuplicate,
		decad.OpPlacedCopy,
	}
	for _, op := range ops {
		t.Run(op.String(), func(t *testing.T) {
			step := validCodecStep(op)
			buf, err := json.Marshal(step)
			require.NoError(t, err)
			var got decad.Step
			require.NoError(t, json.Unmarshal(buf, &got))
			require.Equal(t, step, got)
		})
	}
}

func TestStepOperationShapeRejectsMissingAndExtraFields(t *testing.T) {
	tests := []struct {
		op           decad.OpKind
		missingStep  func(*decad.Step)
		missingField string
		extraStep    func(*decad.Step)
		extraField   string
		extraWire    json.RawMessage
	}{
		{
			op:           decad.OpExtrude,
			missingStep:  func(s *decad.Step) { s.Profile = decad.ProfileRecord{} },
			missingField: "profile",
			extraStep:    func(s *decad.Step) { s.Values = []units.Value{units.Millimeters(1)} },
			extraField:   "values",
			extraWire:    json.RawMessage(`[]`),
		},
		{
			op:           decad.OpRevolve,
			missingStep:  func(s *decad.Step) { s.Axis = nil },
			missingField: "axis",
			extraStep:    func(s *decad.Step) { s.Extent = decad.Distance{D: units.Millimeters(1), Dir: decad.Along} },
			extraField:   "extent",
			extraWire:    json.RawMessage(`null`),
		},
		{
			op:           decad.OpUnion,
			missingStep:  func(s *decad.Step) { s.Inputs = s.Inputs[:1] },
			missingField: "inputs",
			extraStep:    func(s *decad.Step) { s.Profile = validCodecStep(decad.OpExtrude).Profile },
			extraField:   "profile",
			extraWire:    json.RawMessage(`{}`),
		},
		{
			op:           decad.OpCut,
			missingStep:  func(s *decad.Step) { s.Inputs = nil },
			missingField: "inputs",
			extraStep:    func(s *decad.Step) { s.Opts = decad.ExtrudeOpts{Taper: units.Degrees(0)} },
			extraField:   "opts",
			extraWire:    json.RawMessage(`null`),
		},
		{
			op:           decad.OpIntersect,
			missingStep:  func(s *decad.Step) { s.Inputs = nil },
			missingField: "inputs",
			extraStep:    func(s *decad.Step) { s.Selectors = []decad.Selector{decad.Edges()} },
			extraField:   "selectors",
			extraWire:    json.RawMessage(`[]`),
		},
		{
			op:           decad.OpFillet,
			missingStep:  func(s *decad.Step) { s.Selectors = nil },
			missingField: "selectors",
			extraStep:    func(s *decad.Step) { s.Opts = decad.ExtrudeOpts{Taper: units.Degrees(0)} },
			extraField:   "opts",
			extraWire:    json.RawMessage(`null`),
		},
		{
			op:           decad.OpChamfer,
			missingStep:  func(s *decad.Step) { s.Values = nil },
			missingField: "values",
			extraStep:    func(s *decad.Step) { s.Placement = validCodecStep(decad.OpPlaced).Placement },
			extraField:   "placement",
			extraWire:    json.RawMessage(`null`),
		},
		{
			op:           decad.OpShell,
			missingStep:  func(s *decad.Step) { s.Opts = nil },
			missingField: "opts",
			extraStep:    func(s *decad.Step) { s.Extent = decad.Distance{D: units.Millimeters(1), Dir: decad.Along} },
			extraField:   "extent",
			extraWire:    json.RawMessage(`null`),
		},
		{
			op:           decad.OpPlaced,
			missingStep:  func(s *decad.Step) { s.Placement = decad.TransformRecord{} },
			missingField: "placement",
			extraStep:    func(s *decad.Step) { s.Values = []units.Value{units.Millimeters(1)} },
			extraField:   "values",
			extraWire:    json.RawMessage(`[]`),
		},
		{
			op:           decad.OpDuplicate,
			missingStep:  func(s *decad.Step) { s.Inputs = nil },
			missingField: "inputs",
			extraStep:    func(s *decad.Step) { s.Placement = validCodecStep(decad.OpPlaced).Placement },
			extraField:   "placement",
			extraWire:    json.RawMessage(`null`),
		},
		{
			op:           decad.OpPlacedCopy,
			missingStep:  func(s *decad.Step) { s.Placement = decad.TransformRecord{} },
			missingField: "placement",
			extraStep:    func(s *decad.Step) { s.Opts = decad.ExtrudeOpts{Taper: units.Degrees(0)} },
			extraField:   "opts",
			extraWire:    json.RawMessage(`null`),
		},
	}

	for _, test := range tests {
		t.Run(test.op.String()+"/missing", func(t *testing.T) {
			rejectStepMutationBothWays(
				t,
				test.op,
				test.missingStep,
				func(fields map[string]json.RawMessage) { delete(fields, test.missingField) },
			)
		})
		t.Run(test.op.String()+"/extra", func(t *testing.T) {
			rejectStepMutationBothWays(
				t,
				test.op,
				test.extraStep,
				func(fields map[string]json.RawMessage) { fields[test.extraField] = test.extraWire },
			)
		})
	}
}

func TestStepAndRecipeRejectUnknownOperationFields(t *testing.T) {
	var step decad.Step
	err := json.Unmarshal([]byte(`{"op":"union","inputs":[0,1],"unexpected":true}`), &step)
	require.ErrorContains(t, err, `unknown field "unexpected"`)

	var recipe decad.Recipe
	err = json.Unmarshal([]byte(`{"steps":[{"op":"union","inputs":[0,1],"unexpected":true}]}`), &recipe)
	require.ErrorIs(t, err, decad.ErrInvalidRecipe)
	require.ErrorContains(t, err, `unknown field "unexpected"`)
}

func TestStepAndRecipeRejectDuplicateJSONFields(t *testing.T) {
	tests := []struct {
		name  string
		input string
	}{
		{
			name:  "step operation",
			input: `{"op":"union","op":"cut"}`,
		},
		{
			name:  "nested selector",
			input: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","kind":"faces","preds":[]}],"values":["1 mm"]}`,
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var step decad.Step
			err := json.Unmarshal([]byte(test.input), &step)
			require.Error(t, err)
			require.Contains(t, err.Error(), "duplicate recipe field")

			var recipe decad.Recipe
			err = json.Unmarshal([]byte(`{"steps":[`+test.input+`]}`), &recipe)
			require.ErrorIs(t, err, decad.ErrInvalidRecipe)
			require.Contains(t, err.Error(), "duplicate recipe field")
		})
	}
}

func TestStepShapeGateRunsBeforeTypedPayloadDecoding(t *testing.T) {
	tests := []struct {
		name string
		step string
		want string
	}{
		{
			name: "forbidden profile",
			step: `{"op":"union","inputs":[0,1],"profile":{"outer":{"segments":[{"kind":"unknown"}]}}}`,
			want: `the "union" op forbids a profile`,
		},
		{
			name: "missing profile before malformed extent",
			step: `{"op":"extrude","extent":{"kind":"distance"}}`,
			want: `the "extrude" op requires a non-empty profile`,
		},
		{
			name: "missing profile before malformed angular extent",
			step: `{"op":"revolve","angular":{"kind":"angle_extent"}}`,
			want: `the "revolve" op requires a non-empty profile`,
		},
		{
			name: "missing values before malformed edge selector",
			step: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"parallel_to"}]}]}`,
			want: `the "fillet" op requires exactly 1 values`,
		},
		{
			name: "missing options before malformed face selector",
			step: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"normal_to"}]}],"values":["1 mm"]}`,
			want: `the "shell" op requires its matching options`,
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var step decad.Step
			err := json.Unmarshal([]byte(test.step), &step)
			require.ErrorContains(t, err, test.want)

			var recipe decad.Recipe
			err = json.Unmarshal([]byte(`{"steps":[`+test.step+`]}`), &recipe)
			require.ErrorIs(t, err, decad.ErrInvalidRecipe)
			require.ErrorContains(t, err, test.want)
		})
	}
}

func TestStepOperationShapeRejectsWrongCountsAndTypes(t *testing.T) {
	tests := []struct {
		name       string
		op         decad.OpKind
		mutateStep func(*decad.Step)
		mutateWire func(map[string]json.RawMessage)
	}{
		{
			name:       "extrude duplicate inputs",
			op:         decad.OpExtrude,
			mutateStep: func(s *decad.Step) { s.Inputs = []decad.StepRef{0, 0} },
			mutateWire: func(fields map[string]json.RawMessage) { fields["inputs"] = json.RawMessage(`[0,0]`) },
		},
		{
			name:       "revolve duplicate inputs",
			op:         decad.OpRevolve,
			mutateStep: func(s *decad.Step) { s.Inputs = []decad.StepRef{0, 0} },
			mutateWire: func(fields map[string]json.RawMessage) { fields["inputs"] = json.RawMessage(`[0,0]`) },
		},
		{
			name:       "union duplicate inputs",
			op:         decad.OpUnion,
			mutateStep: func(s *decad.Step) { s.Inputs = []decad.StepRef{0, 0} },
			mutateWire: func(fields map[string]json.RawMessage) { fields["inputs"] = json.RawMessage(`[0,0]`) },
		},
		{
			name:       "cut duplicate inputs",
			op:         decad.OpCut,
			mutateStep: func(s *decad.Step) { s.Inputs = []decad.StepRef{0, 0} },
			mutateWire: func(fields map[string]json.RawMessage) { fields["inputs"] = json.RawMessage(`[0,0]`) },
		},
		{
			name:       "intersect duplicate inputs",
			op:         decad.OpIntersect,
			mutateStep: func(s *decad.Step) { s.Inputs = []decad.StepRef{0, 0} },
			mutateWire: func(fields map[string]json.RawMessage) { fields["inputs"] = json.RawMessage(`[0,0]`) },
		},
		{
			name:       "fillet selectors",
			op:         decad.OpFillet,
			mutateStep: func(s *decad.Step) { s.Selectors = append(s.Selectors, decad.Edges()) },
			mutateWire: func(fields map[string]json.RawMessage) {
				fields["selectors"] = json.RawMessage(`[{"kind":"edges","preds":[]},{"kind":"edges","preds":[]}]`)
			},
		},
		{
			name:       "chamfer values",
			op:         decad.OpChamfer,
			mutateStep: func(s *decad.Step) { s.Values = append(s.Values, units.Millimeters(2)) },
			mutateWire: func(fields map[string]json.RawMessage) { fields["values"] = json.RawMessage(`["1 mm","2 mm"]`) },
		},
		{
			name:       "shell inputs",
			op:         decad.OpShell,
			mutateStep: func(s *decad.Step) { s.Inputs = append(s.Inputs, 1) },
			mutateWire: func(fields map[string]json.RawMessage) { fields["inputs"] = json.RawMessage(`[0,1]`) },
		},
		{
			name:       "extrude options",
			op:         decad.OpExtrude,
			mutateStep: func(s *decad.Step) { s.Opts = decad.ShellOpts{Sense: decad.Inward} },
			mutateWire: func(fields map[string]json.RawMessage) {
				fields["opts"] = json.RawMessage(`{"kind":"shell","sense":"inward"}`)
			},
		},
		{
			name:       "shell options",
			op:         decad.OpShell,
			mutateStep: func(s *decad.Step) { s.Opts = decad.ExtrudeOpts{Taper: units.Degrees(0)} },
			mutateWire: func(fields map[string]json.RawMessage) {
				fields["opts"] = json.RawMessage(`{"kind":"extrude","taper":"0 deg"}`)
			},
		},
		{
			name:       "fillet selector",
			op:         decad.OpFillet,
			mutateStep: func(s *decad.Step) { s.Selectors = []decad.Selector{decad.Faces()} },
			mutateWire: func(fields map[string]json.RawMessage) {
				fields["selectors"] = json.RawMessage(`[{"kind":"faces","preds":[]}]`)
			},
		},
		{
			name:       "chamfer selector",
			op:         decad.OpChamfer,
			mutateStep: func(s *decad.Step) { s.Selectors = []decad.Selector{decad.Faces()} },
			mutateWire: func(fields map[string]json.RawMessage) {
				fields["selectors"] = json.RawMessage(`[{"kind":"faces","preds":[]}]`)
			},
		},
		{
			name:       "shell selector",
			op:         decad.OpShell,
			mutateStep: func(s *decad.Step) { s.Selectors = []decad.Selector{decad.Edges()} },
			mutateWire: func(fields map[string]json.RawMessage) {
				fields["selectors"] = json.RawMessage(`[{"kind":"edges","preds":[]}]`)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			rejectStepMutationBothWays(t, test.op, test.mutateStep, test.mutateWire)
		})
	}
}

func TestRecipeRoundTrip(t *testing.T) {
	// A real recorded profile makes the step's payload the genuine article.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof, plane, err := decad.RecordProfile(s, s.Profiles()[0])
	require.NoError(t, err)

	motion, err := r3.Translation(r3.NewVec(0, 0, 25))
	require.NoError(t, err)
	placement, err := decad.RecordTransform(motion)
	require.NoError(t, err)

	recipe := decad.Recipe{Steps: []decad.Step{
		{
			Op:      decad.OpExtrude,
			Profile: prof,
			Plane:   plane,
			Extent:  decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			Opts:    decad.ExtrudeOpts{Taper: units.Degrees(0)},
		},
		{
			Op:        decad.OpPlaced,
			Inputs:    []decad.StepRef{0},
			Placement: placement,
		},
	}}

	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `a recipe is a value: it round-trips field for field`)
}

func TestExtentCodec(t *testing.T) {
	// Every extent variant round-trips through a step, nested sides included.
	for _, e := range []decad.Extent{
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.Distance{D: units.Inches(1), Dir: decad.Against},
		decad.ThroughAll{Dir: decad.Against},
		decad.Symmetric{D: units.Millimeters(8), FullLength: true},
		decad.TwoSided{One: decad.DistanceSide{D: units.Millimeters(3)}, Two: decad.ThroughAllSide{}},
	} {
		step := validCodecStep(decad.OpExtrude)
		step.Extent = e
		buf, err := json.Marshal(step)
		require.NoError(t, err, `%T should encode`, e)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got), `%T should decode`, e)
		require.Equal(t, e, got.Extent, `%T should round-trip`, e)
	}

	// The set is closed: unknown and missing tags are rejected.
	var step decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"helical"}}`), &step))
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"d":"3 mm"}}`), &step))

	// Absent fields are malformed, never silently a zero value or Along.
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"distance","d":"3 mm"}}`), &step),
		`a distance extent with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"distance","dir":"along"}}`), &step),
		`a distance extent with no d is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"through_all"}}`), &step),
		`a through-all extent with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"symmetric"}}`), &step),
		`a symmetric extent with no d is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"two_sided","one":{"kind":"distance_side"},"two":{"kind":"through_all_side"}}}`), &step),
		`a distance side with no d is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"two_sided","one":{"kind":"distance"},"two":{"kind":"distance_side"}}}`), &step),
		`a standalone extent is not a side: the side codec rejects its tag`)
}

func TestDirectionAndOpKindCodec(t *testing.T) {
	// Directions and op kinds travel by name, so a renumbered constant can
	// never silently reinterpret an old recipe.
	buf, err := json.Marshal(decad.Step{Op: decad.OpCut, Inputs: []decad.StepRef{2, 5}})
	require.NoError(t, err)
	require.Contains(t, string(buf), `"op":"cut"`)
	require.Contains(t, string(buf), `"inputs":[2,5]`, `Cut's inputs order [target, tool] is preserved`)

	var d decad.Direction
	require.Error(t, d.UnmarshalText([]byte("upwards")), `an unknown direction name is rejected`)
	var k decad.OpKind
	require.Error(t, k.UnmarshalText([]byte("teleport")), `an unknown op name is rejected`)
	_, err = decad.Direction(42).MarshalText()
	require.Error(t, err, `an out-of-range direction refuses to encode`)
	_, err = decad.OpKind(42).MarshalText()
	require.Error(t, err, `an out-of-range op kind refuses to encode`)
}

func TestStepCodecPreservesExplicitNullSlices(t *testing.T) {
	var nullStep decad.Step
	require.NoError(t, json.Unmarshal([]byte(`{"op":"union","inputs":null,"values":null}`), &nullStep))
	require.Nil(t, nullStep.Inputs)
	require.Nil(t, nullStep.Values)

	var emptyStep decad.Step
	require.NoError(t, json.Unmarshal([]byte(`{"op":"union","inputs":[],"values":[]}`), &emptyStep))
	require.NotNil(t, emptyStep.Inputs)
	require.NotNil(t, emptyStep.Values)
}

func TestStepOptsCodec(t *testing.T) {
	step := validCodecStep(decad.OpExtrude)
	step.Opts = decad.ExtrudeOpts{Taper: units.Degrees(-3)}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, step.Opts, got.Opts, `a signed taper records exactly`)

	var bad decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","opts":{"kind":"warp"}}`), &bad))
	require.Error(t, json.Unmarshal([]byte(`{"extent":{"kind":"through_all","dir":"along"}}`), &bad),
		`a step with no op is malformed, never silently an extrude`)
}

func TestStepOptsCodecRequiresPayloadFields(t *testing.T) {
	var step decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","opts":{"kind":"extrude"}}`), &step),
		`extrude options require taper`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","opts":{"kind":"extrude","taper":null}}`), &step),
		`an explicit null taper is not a present value`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","opts":{"kind":"shell"}}`), &step),
		`shell options require sense`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","opts":{"kind":"shell","sense":null}}`), &step),
		`an explicit null sense is not a present value`)
}

func TestStepOptsCodecAcceptsPresentPayloadFields(t *testing.T) {
	base := validCodecStep(decad.OpExtrude)
	raw, err := json.Marshal(base)
	require.NoError(t, err)
	var fields map[string]json.RawMessage
	require.NoError(t, json.Unmarshal(raw, &fields))
	fields["opts"] = json.RawMessage(`{"kind":"extrude","taper":"0 deg"}`)
	raw, err = json.Marshal(fields)
	require.NoError(t, err)
	var extrude decad.Step
	require.NoError(t, json.Unmarshal(raw, &extrude))
	require.Equal(t, decad.ExtrudeOpts{Taper: units.Degrees(0)}, extrude.Opts)

	base = validCodecStep(decad.OpShell)
	raw, err = json.Marshal(base)
	require.NoError(t, err)
	fields = nil
	require.NoError(t, json.Unmarshal(raw, &fields))
	fields["opts"] = json.RawMessage(`{"kind":"shell","sense":"outward"}`)
	raw, err = json.Marshal(fields)
	require.NoError(t, err)
	var shell decad.Step
	require.NoError(t, json.Unmarshal(raw, &shell))
	require.Equal(t, decad.ShellOpts{Sense: decad.Outward}, shell.Opts)
}

func TestStepOptsCodecRejectsInvalidPayloadFields(t *testing.T) {
	var step decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","opts":{"kind":"extrude","taper":"not-a-value"}}`), &step))
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","opts":{"kind":"shell","sense":"sideways"}}`), &step))
}

func TestPlacementKeyingCodec(t *testing.T) {
	// The Placement keying is presence-aware and bidirectional: the placing
	// ops REQUIRE a nonzero placement, every other op FORBIDS the field.
	motion, err := r3.Translation(r3.NewVec(0, 0, 25))
	require.NoError(t, err)
	placement, err := decad.RecordTransform(motion)
	require.NoError(t, err)
	identity, err := decad.RecordTransform(r3.Identity())
	require.NoError(t, err)

	// A placed copy with a real motion round-trips, and so does one under the
	// identity motion — r3.Identity() is a valid (nonzero) placement.
	for _, p := range []decad.TransformRecord{placement, identity} {
		step := decad.Step{Op: decad.OpPlacedCopy, Inputs: []decad.StepRef{0}, Placement: p}
		buf, err := json.Marshal(step)
		require.NoError(t, err, `a placed copy with a valid placement encodes`)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got), `a placed copy with a valid placement decodes`)
		require.Equal(t, step, got, `a placed copy round-trips field for field`)
	}

	// A duplicate carries NO placement and round-trips: the field is absent.
	dup := decad.Step{Op: decad.OpDuplicate, Inputs: []decad.StepRef{0}}
	buf, err := json.Marshal(dup)
	require.NoError(t, err)
	require.NotContains(t, string(buf), `"placement"`, `a duplicate records no placement`)
	var gotDup decad.Step
	require.NoError(t, json.Unmarshal(buf, &gotDup))
	require.Equal(t, dup, gotDup, `a duplicate round-trips with no placement`)

	// Decode: a placing op with no placement is malformed; a forbidding op with
	// a present-but-zero placement is malformed.
	var bad decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"placed_copy","inputs":[0]}`), &bad),
		`a placed copy with no placement is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"placed","inputs":[0]}`), &bad),
		`a placed op with no placement is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"duplicate","inputs":[0],"placement":{}}`), &bad),
		`a duplicate with a present zero placement is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"duplicate","inputs":[0],"placement":null}`), &bad),
		`a duplicate with an explicit null placement is present, so malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"duplicate","inputs":[0],"placement":{"ex":[1,0,0],"ey":[0,1,0],"ez":[0,0,1],"t":[0,0,25]}}`), &bad),
		`a duplicate with a nonzero placement is malformed`)
	// The key being present at all forbids it on any non-placing op, whatever
	// its op: an explicit null placement on an extrude is malformed too, not
	// silently read as absent.
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","inputs":[0],"placement":null}`), &bad),
		`an extrude with an explicit null placement is present, so malformed`)

	// Marshal: an in-memory placing op with a zero placement, and a forbidding
	// op with a nonzero placement, both refuse to encode.
	_, err = json.Marshal(decad.Step{Op: decad.OpPlacedCopy, Inputs: []decad.StepRef{0}})
	require.Error(t, err, `an in-memory placed copy with a zero placement names no placement to record`)
	_, err = json.Marshal(decad.Step{Op: decad.OpDuplicate, Inputs: []decad.StepRef{0}, Placement: placement})
	require.Error(t, err, `an in-memory duplicate with a placement is keyed wrong`)
}

func TestRecipePointerForms(t *testing.T) {
	// The sealed sets use value receivers, so pointer forms satisfy the
	// interfaces; the codecs normalize them to values and reject nil.
	step := validCodecStep(decad.OpExtrude)
	step.Extent = &decad.TwoSided{One: &decad.DistanceSide{D: units.Millimeters(2)}, Two: decad.ThroughAllSide{}}
	step.Opts = &decad.ExtrudeOpts{Taper: units.Degrees(1)}
	buf, err := json.Marshal(step)
	require.NoError(t, err, `pointer variant forms should encode like their values`)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, decad.TwoSided{One: decad.DistanceSide{D: units.Millimeters(2)}, Two: decad.ThroughAllSide{}}, got.Extent)
	require.Equal(t, decad.ExtrudeOpts{Taper: units.Degrees(1)}, got.Opts)

	nilExtent := validCodecStep(decad.OpExtrude)
	nilExtent.Extent = (*decad.Distance)(nil)
	_, err = json.Marshal(nilExtent)
	require.Error(t, err, `a nil extent pointer names no extent to record`)
	nilOpts := validCodecStep(decad.OpExtrude)
	nilOpts.Opts = (*decad.ExtrudeOpts)(nil)
	_, err = json.Marshal(nilOpts)
	require.Error(t, err, `nil step options name nothing to record`)
}
