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
			Values:  []units.Value{units.Millimeters(10)},
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
		step := decad.Step{Op: decad.OpExtrude, Extent: e}
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

func TestStepOptsCodec(t *testing.T) {
	step := decad.Step{Op: decad.OpExtrude, Opts: decad.ExtrudeOpts{Taper: units.Degrees(-3)}}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, step.Opts, got.Opts, `a signed taper records exactly`)

	var bad decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","opts":{"kind":"warp"}}`), &bad))
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges"}]}`), &bad),
		`selector tags are rejected until the query types land`)
	require.Error(t, json.Unmarshal([]byte(`{"extent":{"kind":"through_all","dir":"along"}}`), &bad),
		`a step with no op is malformed, never silently an extrude`)
}

func TestRecipePointerForms(t *testing.T) {
	// The sealed sets use value receivers, so pointer forms satisfy the
	// interfaces; the codecs normalize them to values and reject nil.
	step := decad.Step{
		Op:     decad.OpExtrude,
		Extent: &decad.TwoSided{One: &decad.DistanceSide{D: units.Millimeters(2)}, Two: decad.ThroughAllSide{}},
		Opts:   &decad.ExtrudeOpts{Taper: units.Degrees(1)},
	}
	buf, err := json.Marshal(step)
	require.NoError(t, err, `pointer variant forms should encode like their values`)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, decad.TwoSided{One: decad.DistanceSide{D: units.Millimeters(2)}, Two: decad.ThroughAllSide{}}, got.Extent)
	require.Equal(t, decad.ExtrudeOpts{Taper: units.Degrees(1)}, got.Opts)

	_, err = json.Marshal(decad.Step{Op: decad.OpExtrude, Extent: (*decad.Distance)(nil)})
	require.Error(t, err, `a nil extent pointer names no extent to record`)
	_, err = json.Marshal(decad.Step{Op: decad.OpExtrude, Opts: (*decad.ExtrudeOpts)(nil)})
	require.Error(t, err, `nil step options name nothing to record`)
}
