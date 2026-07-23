package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Compile-time sealing: the two angular tiers are disjoint — a side is never
// a standalone angular extent and vice versa, and neither set satisfies the
// linear one. (TwoSidedAngle{One: AngleExtent{}} and Step{Extent:
// AngleExtent{}} do not compile, which is the §8.1 guarantee; it cannot be
// asserted in a test that must itself compile.)
var (
	_ decad.AngularExtent = decad.AngleExtent{}
	_ decad.AngularExtent = decad.FullRevolution{}
	_ decad.AngularExtent = decad.SymmetricAngle{}
	_ decad.AngularExtent = decad.TwoSidedAngle{}
	_ decad.SideAngular   = decad.AngleSide{}
)

func TestAngularExtentCodec(t *testing.T) {
	// Every angular variant round-trips through a step, nested sides included.
	for _, a := range []decad.AngularExtent{
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along},
		decad.AngleExtent{A: units.Radians(1), Dir: decad.Against},
		decad.FullRevolution{},
		decad.SymmetricAngle{A: units.Degrees(45)},
		decad.SymmetricAngle{A: units.Degrees(120), FullLength: true},
		decad.TwoSidedAngle{One: decad.AngleSide{A: units.Degrees(30)}, Two: decad.AngleSide{A: units.Degrees(60)}},
	} {
		step := validCodecStep(decad.OpRevolve)
		step.Angular = a
		buf, err := json.Marshal(step)
		require.NoError(t, err, `%T should encode`, a)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got), `%T should decode`, a)
		require.Equal(t, a, got.Angular, `%T should round-trip`, a)
	}

	// The set is closed: unknown and missing tags are rejected.
	var step decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"helical"}}`), &step))
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"a":"90 deg"}}`), &step))

	// The linear and angular sets stay disjoint on the wire too: neither
	// codec accepts the other's tags.
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"distance","d":"3 mm","dir":"along"}}`), &step),
		`a linear extent is not an angular extent: the angular codec rejects its tag`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"angle_extent","a":"90 deg","dir":"along"}}`), &step),
		`an angular extent is not a linear extent: the linear codec rejects its tag`)

	// Absent fields are malformed, never silently a zero angle or Along.
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"angle_extent","a":"90 deg"}}`), &step),
		`an angle extent with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"angle_extent","dir":"along"}}`), &step),
		`an angle extent with no a is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"symmetric_angle"}}`), &step),
		`a symmetric angle extent with no a is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"two_sided_angle","one":{"kind":"angle_side"},"two":{"kind":"angle_side","a":"30 deg"}}}`), &step),
		`an angle side with no a is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"two_sided_angle","one":{"kind":"angle_side","a":"30 deg"}}}`), &step),
		`a two-sided angle extent with a missing side is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"two_sided_angle","one":{"kind":"angle_extent","a":"30 deg","dir":"along"},"two":{"kind":"angle_side","a":"30 deg"}}}`), &step),
		`a standalone angular extent is not a side: the side codec rejects its tag`)
}

func TestEmptyExtentVariantCodec(t *testing.T) {
	for _, test := range []struct {
		name string
		step decad.Step
		bad  string
	}{
		{
			name: "full revolution",
			step: decad.Step{Op: decad.OpRevolve, Angular: decad.FullRevolution{}},
			bad:  `{"op":"revolve","angular":{"kind":"full_revolution","a":"90 deg","dir":"against"}}`,
		},
		{
			name: "through-all side",
			step: decad.Step{
				Op: decad.OpExtrude,
				Extent: decad.TwoSided{
					One: decad.ThroughAllSide{},
					Two: decad.DistanceSide{D: units.Millimeters(5)},
				},
			},
			bad: `{"op":"extrude","extent":{"kind":"two_sided","one":{"kind":"through_all_side","d":"2 mm"},"two":{"kind":"distance_side","d":"5 mm"}}}`,
		},
	} {
		t.Run(test.name, func(t *testing.T) {
			buf, err := json.Marshal(test.step)
			require.NoError(t, err)
			var got decad.Step
			require.NoError(t, json.Unmarshal(buf, &got))
			require.Equal(t, test.step, got)

			var bad decad.Step
			err = json.Unmarshal([]byte(test.bad), &bad)
			require.Error(t, err)
			require.Contains(t, err.Error(), "unknown field")
		})
	}
}

func TestAngularExtentPointerForms(t *testing.T) {
	// The sealed sets use value receivers, so pointer forms satisfy the
	// interfaces; the codecs normalize them to values recursively — nested
	// sides included — so no caller-owned pointer survives into a recorded
	// value.
	for _, tc := range []struct {
		ptr  decad.AngularExtent
		want decad.AngularExtent
	}{
		{ptr: &decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}, want: decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}},
		{ptr: &decad.FullRevolution{}, want: decad.FullRevolution{}},
		{ptr: &decad.SymmetricAngle{A: units.Degrees(45), FullLength: true}, want: decad.SymmetricAngle{A: units.Degrees(45), FullLength: true}},
		{
			ptr:  &decad.TwoSidedAngle{One: &decad.AngleSide{A: units.Degrees(15)}, Two: decad.AngleSide{A: units.Degrees(75)}},
			want: decad.TwoSidedAngle{One: decad.AngleSide{A: units.Degrees(15)}, Two: decad.AngleSide{A: units.Degrees(75)}},
		},
	} {
		step := validCodecStep(decad.OpRevolve)
		step.Angular = tc.ptr
		buf, err := json.Marshal(step)
		require.NoError(t, err, `pointer variant %T should encode like its value`, tc.ptr)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, tc.want, got.Angular, `%T should round-trip to its value form`, tc.ptr)
	}

	// A nil variant pointer names no extent to record: rejected, and
	// branchable via errors.Is against ErrDegenerate — never a string match.
	for _, a := range []decad.AngularExtent{
		(*decad.AngleExtent)(nil),
		(*decad.FullRevolution)(nil),
		(*decad.SymmetricAngle)(nil),
		(*decad.TwoSidedAngle)(nil),
		decad.TwoSidedAngle{One: (*decad.AngleSide)(nil), Two: decad.AngleSide{A: units.Degrees(5)}},
	} {
		step := validCodecStep(decad.OpRevolve)
		step.Angular = a
		_, err := json.Marshal(step)
		require.ErrorIs(t, err, decad.ErrDegenerate, `a nil pointer inside %T is ErrDegenerate`, a)
	}
}

func TestStepExtentKeying(t *testing.T) {
	// The core §6.2 one-of contract: at most one of extent and angular,
	// each keyed to its op — enforced on both wire directions.
	t.Run("both extents rejected on marshal", func(t *testing.T) {
		step := validCodecStep(decad.OpExtrude)
		step.Angular = decad.FullRevolution{}
		_, err := json.Marshal(step)
		require.Error(t, err)
	})
	t.Run("angular on a non-revolve op rejected on marshal", func(t *testing.T) {
		step := validCodecStep(decad.OpExtrude)
		step.Angular = decad.FullRevolution{}
		_, err := json.Marshal(step)
		require.Error(t, err)
	})
	t.Run("linear on a non-extrude op rejected on marshal", func(t *testing.T) {
		step := validCodecStep(decad.OpRevolve)
		step.Extent = decad.Distance{D: units.Millimeters(5), Dir: decad.Along}
		_, err := json.Marshal(step)
		require.Error(t, err)
	})
	t.Run("both extents rejected on unmarshal", func(t *testing.T) {
		var s decad.Step
		err := json.Unmarshal([]byte(`{"op":"extrude","extent":{"kind":"distance","d":"5 mm","dir":"along"},"angular":{"kind":"full_revolution"}}`), &s)
		require.Error(t, err)
	})
	t.Run("angular on a non-revolve op rejected on unmarshal", func(t *testing.T) {
		var s decad.Step
		err := json.Unmarshal([]byte(`{"op":"extrude","angular":{"kind":"full_revolution"}}`), &s)
		require.Error(t, err)
	})
	t.Run("linear on a non-extrude op rejected on unmarshal", func(t *testing.T) {
		var s decad.Step
		err := json.Unmarshal([]byte(`{"op":"revolve","extent":{"kind":"distance","d":"5 mm","dir":"along"}}`), &s)
		require.Error(t, err)
	})
}
