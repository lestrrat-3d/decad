package decad_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Compile-time sealing: the query types are the Selector variants, and each
// implements its own selector interface (core §9). The variants seal in with
// pointer receivers — a value EdgeQuery is not a Selector, so there is no
// value form to record; the clone helpers deep-copy the pointer instead.
var (
	_ decad.Selector     = (*decad.EdgeQuery)(nil)
	_ decad.Selector     = (*decad.FaceQuery)(nil)
	_ decad.EdgeSelector = (*decad.EdgeQuery)(nil)
	_ decad.FaceSelector = (*decad.FaceQuery)(nil)
)

func TestSelectorCodec(t *testing.T) {
	// Every selector variant, carrying every predicate the vocabulary
	// defines, round-trips through a Step field for field.
	ref := decad.FeatureRef{Step: 2, Role: roleCapStart}
	for _, sel := range []decad.Selector{
		decad.Edges(),
		decad.Faces(),
		decad.Edges(decad.Convex(), decad.ParallelTo(r3.NewVec(0, 0, 1))),
		decad.Edges(
			decad.Convex(),
			decad.Concave(),
			decad.ParallelTo(r3.NewVec(1, 0, 0)),
			decad.LongerThan(units.Millimeters(5)),
			decad.CreatedBy(ref),
			decad.Circular(),
		).Exactly(4),
		decad.Edges(decad.Circular()).AtLeast(1),
		decad.Faces(
			decad.Planar(),
			decad.Cylindrical(),
			decad.NormalTo(r3.NewVec(0, 1, 0)),
			decad.FaceCreatedBy(ref),
		).Exactly(1),
		decad.Faces(decad.Planar()).AtLeast(2),
	} {
		step := validCodecStep(decad.OpFillet)
		if _, ok := sel.(*decad.FaceQuery); ok {
			step = validCodecStep(decad.OpShell)
		}
		step.Selectors = []decad.Selector{sel}
		buf, err := json.Marshal(step)
		require.NoError(t, err, `%T should encode`, sel)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got), `%T should decode`, sel)
		require.Equal(t, step, got, `a step carrying a %T round-trips field for field`, sel)
	}

	// A later cardinality assertion replaces the earlier one.
	step := validCodecStep(decad.OpFillet)
	step.Selectors = []decad.Selector{decad.Edges(decad.Convex()).Exactly(4).AtLeast(1)}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	require.Contains(t, string(buf), `"at_least":1`)
	require.NotContains(t, string(buf), `"exactly"`)
}

func TestSelectorCodecRejections(t *testing.T) {
	// The set is closed: unknown and missing tags are rejected, at the
	// selector level and inside each predicate tier.
	var step decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"vertices","preds":[]}],"values":["1 mm"]}`), &step),
		`an unknown selector kind is rejected`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"preds":[]}],"values":["1 mm"]}`), &step),
		`a selector with no kind tag is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"glowing"}]}],"values":["1 mm"]}`), &step),
		`an unknown edge predicate kind is rejected`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"convex"}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`), &step),
		`the predicate tiers are disjoint: an edge predicate is not a face predicate`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{}]}],"values":["1 mm"]}`), &step),
		`a predicate with no kind tag is malformed`)

	// Absent fields are malformed, never silently zeroed.
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges"}],"values":["1 mm"]}`), &step),
		`a query with no preds is malformed, never silently match-all`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"parallel_to"}]}],"values":["1 mm"]}`), &step),
		`a parallel-to predicate with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"longer_than"}]}],"values":["1 mm"]}`), &step),
		`a longer-than predicate with no l is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by"}]}],"values":["1 mm"]}`), &step),
		`a created-by predicate with no ref is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":1}}]}],"values":["1 mm"]}`), &step),
		`a provenance ref with no role is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"normal_to"}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`), &step),
		`a normal-to predicate with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"role":"capEnd"}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`), &step),
		`a provenance ref with no step is malformed`)
	for _, data := range []string{
		`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":""}}]}],"values":["1 mm"]}`,
		`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-1,"role":"capStart"}}]}],"values":["1 mm"]}`,
		`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":0,"role":""}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`,
		`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-1,"role":"capStart"}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`,
	} {
		err := json.Unmarshal([]byte(data), &step)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	}
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[],"exactly":4,"at_least":1}],"values":["1 mm"]}`), &step),
		`a query carries at most one cardinality assertion`)

	// A zero-value predicate names nothing: only the constructors build one.
	badEdge := validCodecStep(decad.OpFillet)
	badEdge.Selectors = []decad.Selector{decad.Edges(decad.EdgePredicate{})}
	_, err := json.Marshal(badEdge)
	require.Error(t, err, `a zero-value edge predicate refuses to encode`)
	badFace := validCodecStep(decad.OpShell)
	badFace.Selectors = []decad.Selector{decad.Faces(decad.FacePredicate{})}
	_, err = json.Marshal(badFace)
	require.Error(t, err, `a zero-value face predicate refuses to encode`)
}

func TestSelectorCodecRejectsUnknownNestedFields(t *testing.T) {
	for _, tc := range []struct {
		name string
		data string
	}{
		{
			name: "selector field",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[],"unexpected":true}],"values":["1 mm"]}`,
		},
		{
			name: "predicate field",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"convex","unexpected":true}]}],"values":["1 mm"]}`,
		},
		{
			name: "provenance field",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":"capStart","unexpected":true}}]}],"values":["1 mm"]}`,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var step decad.Step
			require.Error(t, json.Unmarshal([]byte(tc.data), &step))
		})
	}
}

func TestSelectorCodecRejectsOutOfRangeNegativeProvenanceSteps(t *testing.T) {
	for _, tc := range []struct {
		name string
		data string
	}{
		{
			name: "edge created-by",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-9223372036854775809,"role":"capStart"}}]}],"values":["1 mm"]}`,
		},
		{
			name: "face created-by",
			data: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-9223372036854775809,"role":"capStart"}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var step decad.Step
			err := json.Unmarshal([]byte(tc.data), &step)
			require.ErrorIs(t, err, decad.ErrDegenerate)
		})
	}
}

func TestSelectorCodecRejectsNegativeProvenanceStepNumberForms(t *testing.T) {
	for _, tc := range []struct {
		name string
		data string
	}{
		{
			name: "edge created-by decimal",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-1.0,"role":"capStart"}}]}],"values":["1 mm"]}`,
		},
		{
			name: "edge created-by exponent",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-1e0,"role":"capStart"}}]}],"values":["1 mm"]}`,
		},
		{
			name: "edge created-by uppercase exponent",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-1E+0,"role":"capStart"}}]}],"values":["1 mm"]}`,
		},
		{
			name: "face created-by decimal",
			data: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-1.0,"role":"capStart"}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`,
		},
		{
			name: "face created-by exponent",
			data: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-1e0,"role":"capStart"}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`,
		},
		{
			name: "face created-by uppercase exponent",
			data: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-1E+0,"role":"capStart"}}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var step decad.Step
			err := json.Unmarshal([]byte(tc.data), &step)
			require.ErrorIs(t, err, decad.ErrDegenerate)
		})
	}
}

func TestSelectorCodecAcceptsNegativeZeroProvenanceSteps(t *testing.T) {
	ref := decad.FeatureRef{Step: 0, Role: roleCapStart}
	for _, tc := range []struct {
		name string
		data string
		want decad.Step
	}{
		{
			name: "edge created-by",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-0,"role":"capStart"}}]}],"values":["1 mm"]}`,
			want: decad.Step{Op: decad.OpFillet, Inputs: []decad.StepRef{0}, Selectors: []decad.Selector{decad.Edges(decad.CreatedBy(ref))}, Values: []units.Value{units.Millimeters(1)}},
		},
		{
			name: "face created-by",
			data: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-0,"role":"capStart"}}]}],"values":["1 mm"],"opts":{"kind":"shell","sense":"inward"}}`,
			want: decad.Step{Op: decad.OpShell, Inputs: []decad.StepRef{0}, Selectors: []decad.Selector{decad.Faces(decad.FaceCreatedBy(ref))}, Values: []units.Value{units.Millimeters(1)}, Opts: decad.ShellOpts{Sense: decad.Inward}},
		},
		{
			name: "edge created-by exponent",
			data: `{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-0e10,"role":"capStart"}}]}],"values":["1 mm"]}`,
			want: decad.Step{Op: decad.OpFillet, Inputs: []decad.StepRef{0}, Selectors: []decad.Selector{decad.Edges(decad.CreatedBy(ref))}, Values: []units.Value{units.Millimeters(1)}},
		},
		{
			name: "face created-by exponent",
			data: `{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-0E+10,"role":"capStart"}}]}],"values":["1 mm"],"opts":{"kind":"shell","sense":"inward"}}`,
			want: decad.Step{Op: decad.OpShell, Inputs: []decad.StepRef{0}, Selectors: []decad.Selector{decad.Faces(decad.FaceCreatedBy(ref))}, Values: []units.Value{units.Millimeters(1)}, Opts: decad.ShellOpts{Sense: decad.Inward}},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var got decad.Step
			require.NoError(t, json.Unmarshal([]byte(tc.data), &got))
			require.Equal(t, tc.want, got)
		})
	}
}

func TestNilSelectorPointersAreBranchable(t *testing.T) {
	// A typed nil query pointer follows the same branchable contract as the
	// other sealed sets: errors.Is(err, ErrDegenerate).
	badEdge := validCodecStep(decad.OpFillet)
	badEdge.Selectors = []decad.Selector{(*decad.EdgeQuery)(nil)}
	_, err := json.Marshal(badEdge)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	badFace := validCodecStep(decad.OpShell)
	badFace.Selectors = []decad.Selector{(*decad.FaceQuery)(nil)}
	_, err = json.Marshal(badFace)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestSelectorResolutionRejectsNilInputs(t *testing.T) {
	// A nil body has no topology to select from, and a typed nil query names
	// no query: both are branchable ErrDegenerate, never a silent zero-match.
	_, err := decad.Edges(decad.Convex()).SelectEdges(nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Faces(decad.Planar()).SelectFaces(nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	var eq *decad.EdgeQuery
	_, err = eq.SelectEdges(nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	var fq *decad.FaceQuery
	_, err = fq.SelectFaces(nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestSelectorConstructorsDoNotAliasCallerSlices(t *testing.T) {
	// Edges/Faces copy the variadic predicate slice, so a caller mutating its
	// own slice after the call cannot rewrite the query it handed out.
	preds := []decad.EdgePredicate{decad.Convex()}
	q := decad.Edges(preds...)
	preds[0] = decad.Concave()
	require.Equal(t, decad.Edges(decad.Convex()), q,
		`the query holds the predicates given at construction`)
}

// outwardNormal reads a planar face's proven outward normal; NormalAt ignores
// the point for a plane, so any point serves.
func outwardNormal(t *testing.T, f *decad.Face) r3.Vec {
	t.Helper()
	nm, err := f.NormalAt(r3.NewVec(0, 0, 0))
	require.NoError(t, err)
	return nm.Value
}

func TestFacingPredicate(t *testing.T) {
	body := holePlateBody(t) // 100×60×8 plate, hole; caps at z=0 (capStart) and z=8 (capEnd)

	t.Run("PicksOneCapWhereNormalToPicksTwo", func(t *testing.T) {
		both, err := decad.Faces(decad.NormalTo(zAxis)).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, both, 2, `NormalTo matches both caps, either sense`)

		top, err := decad.Faces(decad.Planar(), decad.Facing(zAxis)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, top, 1, `Facing(+z) is the single upward-facing cap`)
		require.Positive(t, outwardNormal(t, top[0]).Dot(zAxis),
			`the matched cap's outward normal points ALONG +z`)
	})

	t.Run("OppositeSensePicksTheOtherCap", func(t *testing.T) {
		bottom, err := decad.Faces(decad.Planar(), decad.Facing(zAxis.Scale(-1))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, bottom, 1)
		require.Negative(t, outwardNormal(t, bottom[0]).Dot(zAxis),
			`Facing(-z) is the downward-facing cap`)

		// The two senses select disjoint single faces.
		top, err := decad.Faces(decad.Facing(zAxis)).SelectFaces(body)
		require.NoError(t, err)
		require.NotEqual(t, top[0].Origins(), bottom[0].Origins(),
			`Facing(+z) and Facing(-z) are different caps`)
	})

	t.Run("PicksOneSideWall", func(t *testing.T) {
		// The x=100 wall faces +x; the x=0 wall faces -x. NormalTo(x) matches
		// both, Facing(x) only the one whose material-leaving normal is +x.
		xWall, err := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, xWall[0]).Dot(r3.NewVec(1, 0, 0)))
	})

	t.Run("NonPlanarNeverMatches", func(t *testing.T) {
		// The hole cylinder is not planar, and no direction faces it.
		_, err := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)), decad.Cylindrical()).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})

	t.Run("HugeFiniteDirectionParallelToNoWallMatchesNothing", func(t *testing.T) {
		// A finite but huge 45° direction (parallel to no axis-aligned wall's
		// normal) must match nothing. Taking its length squares its components
		// to +Inf, so a scale-blind parallel test would wrongly report it
		// parallel to every wall; the scaled direction keeps it honest.
		_, err := decad.Faces(decad.Facing(r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a huge finite 45° direction is parallel to no box wall's normal`)

		// The +x wall still matches when the huge direction really is +x.
		xWall, err := decad.Faces(decad.Facing(r3.NewVec(math.MaxFloat64, 0, 0))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, xWall[0]).Dot(r3.NewVec(1, 0, 0)),
			`a huge +x direction still faces the +x wall`)
	})

	t.Run("SubnormalDirectionStillNamesTheRay", func(t *testing.T) {
		// A subnormal +x direction must still face the +x wall. Its
		// infinity-norm m is that same subnormal, so 1/m overflows to +Inf and
		// v.Scale(1/m) would yield {+Inf, NaN, NaN}; scaling by component
		// division keeps m/m = 1, a finite unit-∞-norm {1, 0, 0}.
		xWall, err := decad.Faces(decad.Facing(r3.NewVec(math.SmallestNonzeroFloat64, 0, 0))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, xWall[0]).Dot(r3.NewVec(1, 0, 0)),
			`a subnormal +x direction still faces the +x wall`)
	})

	t.Run("DegenerateDirectionRejectedAtResolve", func(t *testing.T) {
		_, err := decad.Faces(decad.Facing(r3.NewVec(0, 0, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrDegenerate, `a zero direction is rejected, as NormalTo's is`)
		_, err = decad.Faces(decad.Facing(r3.NewVec(math.NaN(), 0, 1))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNotFinite, `a non-finite direction is rejected`)
	})
}

func TestFacingCodec(t *testing.T) {
	// A facing predicate round-trips through a Step field for field, and its
	// dir payload is required — an absent dir is malformed, never a zero vector.
	step := validCodecStep(decad.OpShell)
	step.Selectors = []decad.Selector{decad.Faces(decad.Planar(), decad.Facing(zAxis)).Exactly(1)}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	require.Contains(t, string(buf), `"kind":"facing"`)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, step, got, `a step carrying a facing predicate round-trips`)

	require.Error(t, json.Unmarshal(
		[]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"facing"}]}]}`), &got),
		`a facing predicate with no dir is malformed`)
}

// TestMissingDirErrorWording pins the human-readable missing-dir decode error
// for each direction-bearing face predicate. The wire token and the display
// name differ deliberately (normal_to -> normal-to), so this guards against the
// raw tag leaking back into the message a caller reads.
func TestMissingDirErrorWording(t *testing.T) {
	for _, tc := range []struct {
		kind string
		want string
	}{
		{"normal_to", `decad: a normal-to predicate requires dir`},
		{"facing", `decad: a facing predicate requires dir`},
	} {
		t.Run(tc.kind, func(t *testing.T) {
			var got decad.Step
			err := json.Unmarshal(
				[]byte(`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"`+tc.kind+`"}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`),
				&got)
			require.ErrorContains(t, err, tc.want)
		})
	}
}

// TestMalformedDirErrorWording pins the OTHER decode-error path — a dir that is
// present but malformed (a non-vector) — for the same predicates. Like the
// missing-dir message, it must render the display name, never the raw wire tag,
// so NormalTo's established wording is unchanged.
func TestMalformedDirErrorWording(t *testing.T) {
	for _, tc := range []struct {
		kind string
		want string
	}{
		{"normal_to", `decad: failed to decode normal-to predicate`},
		{"facing", `decad: failed to decode facing predicate`},
	} {
		t.Run(tc.kind, func(t *testing.T) {
			var got decad.Step
			err := json.Unmarshal(
				[]byte(`{"op":"shell","inputs":[0],"selectors":[{"kind":"faces","preds":[{"kind":"`+tc.kind+`","dir":"bad"}]}],"opts":{"kind":"shell","sense":"inward"},"values":["1 mm"]}`),
				&got)
			require.ErrorContains(t, err, tc.want)
		})
	}
}

func TestCapHelpers(t *testing.T) {
	body := holePlateBody(t)

	t.Run("CapStartAndCapEndNameTheRightFaces", func(t *testing.T) {
		require.Equal(t,
			decad.FeatureRef{Step: body.Origin().Step, Role: roleCapStart},
			decad.CapStart(body), `CapStart pairs the body's own step with the fixed role`)
		require.Equal(t,
			decad.FeatureRef{Step: body.Origin().Step, Role: roleCapEnd},
			decad.CapEnd(body))

		start, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Contains(t, start[0].Origins(), decad.CapStart(body))
		require.Negative(t, outwardNormal(t, start[0]).Dot(zAxis), `the start cap faces down (−z)`)

		end, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, end[0]).Dot(zAxis), `the end cap faces up (+z)`)

		// CapStart names the same cap Facing(−z) selects by geometry.
		down, err := decad.Faces(decad.Facing(zAxis.Scale(-1))).SelectFaces(body)
		require.NoError(t, err)
		require.Equal(t, start[0].Origins(), down[0].Origins())
	})

	t.Run("MatchesNothingWhereTheStepMintsNoCap", func(t *testing.T) {
		// A full revolution mints no caps, so the well-formed ref matches nothing.
		rev := annularRevolveBody(t)
		_, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(rev))).SelectFaces(rev)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})
}

// A finite but extreme direction must not overflow or underflow the parallel
// test that NormalTo and edge ParallelTo share. holePlateBody is a 100×60×8
// plate whose every planar face normal and straight edge is axis-aligned, so a
// 45° direction is normal to no face and parallel to no edge — the huge case
// used to read cross <= eps*la*lb as finite <= +Inf and match everything, the
// subnormal case used to underflow the reference length to 0 and match nothing.
func TestParallelDirsExtremeDirections(t *testing.T) {
	body := holePlateBody(t)

	huge := math.MaxFloat64
	tiny := math.SmallestNonzeroFloat64

	t.Run("NormalToHuge45MatchesNothing", func(t *testing.T) {
		_, err := decad.Faces(decad.NormalTo(r3.NewVec(huge, huge, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a huge 45° direction is normal to no face of the plate`)
	})

	t.Run("NormalToSubnormal45MatchesNothing", func(t *testing.T) {
		_, err := decad.Faces(decad.NormalTo(r3.NewVec(tiny, tiny, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a subnormal 45° direction is normal to no face`)
	})

	t.Run("EdgeParallelToHuge45MatchesNothing", func(t *testing.T) {
		_, err := decad.Edges(decad.ParallelTo(r3.NewVec(huge, huge, 0))).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a huge 45° direction is parallel to no straight edge`)
	})

	// A huge direction that REALLY is an axis normal still matches: normalizing
	// must preserve the true parallel, not just suppress the false one. +z is
	// the two caps' normal.
	t.Run("NormalToHugeZStillMatchesBothCaps", func(t *testing.T) {
		faces, err := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, huge))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2, `NormalTo(+z) matches both caps regardless of magnitude`)
	})

	// A subnormal +z direction likewise still names the caps once the length no
	// longer underflows to zero.
	t.Run("NormalToSubnormalZStillMatchesBothCaps", func(t *testing.T) {
		faces, err := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, tiny))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2)
	})

	// The threat model requires a genuinely parallel direction to match at ANY
	// magnitude — the edge ParallelTo false-miss path, where a subnormal used to
	// underflow the length to 0 and match nothing. +x is the direction of the
	// plate's four axis-aligned edges (two per cap). Compare against the unit
	// direction so the expected set is the geometry's own, not a hard-coded count.
	unitX, err := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).SelectEdges(body)
	require.NoError(t, err)
	require.NotEmpty(t, unitX, `sanity: the plate has edges parallel to +x`)

	t.Run("EdgeParallelToSubnormalXStillMatches", func(t *testing.T) {
		edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(tiny, 0, 0))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, len(unitX),
			`a subnormal +x direction is parallel to the same edges as unit +x`)
	})

	t.Run("EdgeParallelToHugeXStillMatches", func(t *testing.T) {
		edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(huge, 0, 0))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, len(unitX),
			`a huge +x direction is parallel to the same edges as unit +x`)
	})
}

// roleCapStart is the provenance role of an extrude/revolve start cap
// (docs/evaluator-design.md §3), shared by the provenance assertions across
// this package's tests.
const roleCapStart = "capStart"

// holePlateBody extrudes a 100×60 plate with a Ø20 hole at (70, 30) by 8 mm:
// 7 faces (4 planar sides + 1 hole cylinder + 2 caps) and 14 edges (12 box
// lines + 2 hole circles) — the known set every predicate below is checked
// against.
func holePlateBody(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// annularRevolveBody revolves the annular rectangle a full turn about the
// sketch u axis: 4 faces (2 cylinders + 2 planar rings) and 4 circular edges.
func annularRevolveBody(t *testing.T) *decad.Body {
	t.Helper()
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestSelectEdgesPredicates(t *testing.T) {
	body := holePlateBody(t)

	t.Run("NoPredicatesMatchesEverything", func(t *testing.T) {
		edges, err := decad.Edges().SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 14)
	})
	t.Run("Convex", func(t *testing.T) {
		edges, err := decad.Edges(decad.Convex()).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 12, `the box edges are convex; the hole edges are not`)
		for _, e := range edges {
			require.True(t, e.IsConvex())
		}
	})
	t.Run("Concave", func(t *testing.T) {
		edges, err := decad.Edges(decad.Concave()).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 2, `the hole's two rim edges are concave`)
		for _, e := range edges {
			require.False(t, e.IsConvex())
		}
	})
	t.Run("Circular", func(t *testing.T) {
		edges, err := decad.Edges(decad.Circular()).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 2)
		for _, e := range edges {
			require.IsType(t, decad.Circle3{}, e.Curve())
		}
	})
	t.Run("ParallelTo", func(t *testing.T) {
		// Either sense matches, and a circular edge has no direction, so the
		// hole rims never match.
		edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, -2))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 4, `the four vertical box edges`)
		edges, err = decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 4)
	})
	t.Run("LongerThan", func(t *testing.T) {
		// Strictly longer, judged on Edge.Length(): the four 100 mm box
		// edges and the two 2π·10 ≈ 62.8 mm hole rims clear 61 mm.
		edges, err := decad.Edges(decad.LongerThan(units.Millimeters(61))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 6)
		edges, err = decad.Edges(decad.LongerThan(units.Millimeters(90))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 4)
		// Strict: nothing is longer than itself.
		edges, err = decad.Edges(decad.LongerThan(units.Millimeters(60))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 6, `the 60 mm edges are not strictly longer than 60 mm`)
	})
	t.Run("CreatedBy", func(t *testing.T) {
		// An edge is created by the role that created a face it bounds: the
		// bottom cap's boundary is its four box edges plus the hole rim.
		ref := decad.FeatureRef{Step: body.Origin().Step, Role: roleCapStart}
		edges, err := decad.Edges(decad.CreatedBy(ref)).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 5)
	})
	t.Run("Conjunction", func(t *testing.T) {
		ref := decad.FeatureRef{Step: body.Origin().Step, Role: roleCapStart}
		edges, err := decad.Edges(decad.CreatedBy(ref), decad.Circular()).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 1, `the bottom hole rim is the one circular capStart edge`)
		edges, err = decad.Edges(decad.Convex(), decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.LongerThan(units.Millimeters(90))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 4)
	})
}

func TestSelectFacesPredicates(t *testing.T) {
	body := holePlateBody(t)

	t.Run("NoPredicatesMatchesEverything", func(t *testing.T) {
		faces, err := decad.Faces().SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 7)
	})
	t.Run("Planar", func(t *testing.T) {
		faces, err := decad.Faces(decad.Planar()).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 6)
	})
	t.Run("Cylindrical", func(t *testing.T) {
		faces, err := decad.Faces(decad.Cylindrical()).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		cyl, ok := faces[0].Surface().(decad.Cylinder)
		require.True(t, ok)
		require.True(t, cyl.Radius.Equal(units.Millimeters(10), 1e-9))
	})
	t.Run("NormalTo", func(t *testing.T) {
		// Parallel either sense: ±z matches both caps; a non-planar face
		// never matches.
		faces, err := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 5))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2)
		faces, err = decad.Faces(decad.NormalTo(r3.NewVec(-1, 0, 0))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2, `the x=0 and x=100 side walls`)
	})
	t.Run("FaceCreatedBy", func(t *testing.T) {
		ref := decad.FeatureRef{Step: body.Origin().Step, Role: "capEnd"}
		faces, err := decad.Faces(decad.FaceCreatedBy(ref)).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		require.Equal(t, []decad.FeatureRef{ref}, faces[0].Origins())
	})
	t.Run("Conjunction", func(t *testing.T) {
		faces, err := decad.Faces(decad.Planar(), decad.NormalTo(r3.NewVec(0, 0, 1))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2)
	})
}

func TestSelectorResolutionOnRevolvedBody(t *testing.T) {
	// Resolution reads only the public topology, so a revolved body's faces
	// and edges select exactly like a prism's.
	body := annularRevolveBody(t)

	faces, err := decad.Faces(decad.Cylindrical()).SelectFaces(body)
	require.NoError(t, err)
	require.Len(t, faces, 2, `outer and inner walls of the annular cylinder`)

	faces, err = decad.Faces(decad.Planar()).SelectFaces(body)
	require.NoError(t, err)
	require.Len(t, faces, 2, `the two annular rings`)

	// The revolve axis is the world x axis, so the ring normals are ±x.
	faces, err = decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).SelectFaces(body)
	require.NoError(t, err)
	require.Len(t, faces, 2)

	edges, err := decad.Edges(decad.Circular()).SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, edges, 4, `every edge of the annular cylinder is a latitude circle`)

	// A circle has no direction, so nothing is parallel to anything here.
	_, err = decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).SelectEdges(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	edges, err = decad.Edges(decad.Convex()).SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, edges, 4, `every rectangle corner turns convex, so every rim edge is convex`)

	// Provenance works on revolve roles too.
	_, err = decad.Edges(decad.CreatedBy(decad.FeatureRef{Step: body.Origin().Step, Role: roleCapStart})).SelectEdges(body)
	require.ErrorIs(t, err, decad.ErrNoMatch, `a full revolution has no caps`)
}

func TestSelectorCardinality(t *testing.T) {
	body := holePlateBody(t)

	t.Run("ExactlySucceeds", func(t *testing.T) {
		edges, err := decad.Edges(decad.Circular()).Exactly(2).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 2)
		faces, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 1)
	})
	t.Run("AtLeastSucceeds", func(t *testing.T) {
		edges, err := decad.Edges(decad.Convex()).AtLeast(1).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 12)
	})
	t.Run("ExactlyMissIsErrCardinality", func(t *testing.T) {
		_, err := decad.Edges(decad.Convex()).Exactly(4).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrCardinality)
		_, err = decad.Faces(decad.Planar()).Exactly(1).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrCardinality)
	})
	t.Run("AtLeastMissIsErrCardinality", func(t *testing.T) {
		_, err := decad.Edges(decad.Circular()).AtLeast(3).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrCardinality)
	})
	t.Run("AssertedZeroIsErrCardinality", func(t *testing.T) {
		// ErrCardinality takes precedence at zero matches (core §12).
		_, err := decad.Edges(decad.Circular(), decad.ParallelTo(r3.NewVec(1, 0, 0))).Exactly(1).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrCardinality)
		require.NotErrorIs(t, err, decad.ErrNoMatch)
	})
	t.Run("UnassertedZeroIsErrNoMatch", func(t *testing.T) {
		_, err := decad.Edges(decad.Circular(), decad.ParallelTo(r3.NewVec(1, 0, 0))).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrNoMatch)
		_, err = decad.Faces(decad.NormalTo(r3.NewVec(1, 1, 1))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})
}

func TestSelectorResolutionIsDeterministic(t *testing.T) {
	// The result keeps the topology accessors' order, so a recipe replay
	// selects identically: two resolutions return the same entities in the
	// same order.
	body := holePlateBody(t)
	e1, err := decad.Edges(decad.Convex()).SelectEdges(body)
	require.NoError(t, err)
	e2, err := decad.Edges(decad.Convex()).SelectEdges(body)
	require.NoError(t, err)
	require.Equal(t, e1, e2)
	f1, err := decad.Faces(decad.Planar()).SelectFaces(body)
	require.NoError(t, err)
	f2, err := decad.Faces(decad.Planar()).SelectFaces(body)
	require.NoError(t, err)
	require.Equal(t, f1, f2)
}

func TestSelectorProvenanceSurvivesFaceMerge(t *testing.T) {
	// A rectangle authored with a midpoint on its bottom edge: the two
	// collinear side walls coalesce into ONE face carrying both roles, and
	// FaceCreatedBy matches on ANY of them (core §6.1) — as does CreatedBy
	// through the merged face's edges.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	m := s.CreatePoint(50, 0)
	b := s.CreatePoint(100, 0)
	c := s.CreatePoint(100, 60)
	d := s.CreatePoint(0, 60)
	s.CreateLine(a, m)
	s.CreateLine(m, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	var merged *decad.Face
	for _, f := range body.Faces() {
		if len(f.Origins()) == 2 {
			merged = f
		}
	}
	require.NotNil(t, merged)
	for _, ref := range merged.Origins() {
		faces, err := decad.Faces(decad.FaceCreatedBy(ref)).SelectFaces(body)
		require.NoError(t, err)
		require.Equal(t, []*decad.Face{merged}, faces, `either contributing role selects the merged face`)

		edges, err := decad.Edges(decad.CreatedBy(ref)).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, 4, `the merged face's edges are created by either role`)
	}
}

func TestSelectorPredicateParameterGates(t *testing.T) {
	body := holePlateBody(t)

	t.Run("ZeroDirection", func(t *testing.T) {
		_, err := decad.Edges(decad.ParallelTo(r3.Vec{})).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Faces(decad.NormalTo(r3.Vec{})).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NonFiniteDirection", func(t *testing.T) {
		_, err := decad.Edges(decad.ParallelTo(r3.NewVec(math.NaN(), 0, 0))).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrNotFinite)
		_, err = decad.Faces(decad.NormalTo(r3.NewVec(0, math.Inf(1), 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNotFinite)
	})
	t.Run("LongerThanKind", func(t *testing.T) {
		_, err := decad.Edges(decad.LongerThan(units.Degrees(5))).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrUnitKind)
	})
	t.Run("LongerThanNegative", func(t *testing.T) {
		_, err := decad.Edges(decad.LongerThan(units.Millimeters(-1))).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	})
	t.Run("FeatureRef", func(t *testing.T) {
		for _, ref := range []decad.FeatureRef{
			{Step: 0, Role: ""},
			{Step: -1, Role: roleCapStart},
		} {
			_, err := decad.Edges(decad.CreatedBy(ref)).SelectEdges(body)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.NotErrorIs(t, err, decad.ErrNoMatch)

			_, err = decad.Faces(decad.FaceCreatedBy(ref)).SelectFaces(body)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.NotErrorIs(t, err, decad.ErrNoMatch)
		}
	})
	t.Run("ZeroValuePredicate", func(t *testing.T) {
		// Only the constructors build a predicate; the zero value names no
		// kind and is malformed input at resolve.
		_, err := decad.Edges(decad.EdgePredicate{}).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Faces(decad.FacePredicate{}).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

func TestSelectorRejectsNonPositiveCardinality(t *testing.T) {
	body := holePlateBody(t)

	// A zero or negative assertion would let "matches nothing" read as
	// success: rejected at resolve.
	_, err := decad.Edges(decad.Circular()).Exactly(0).SelectEdges(body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Faces(decad.Planar()).AtLeast(0).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Edges(decad.Circular()).AtLeast(-1).SelectEdges(body)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// And on both wire directions, with the same branchable identity.
	step := validCodecStep(decad.OpFillet)
	step.Selectors = []decad.Selector{decad.Edges(decad.Circular()).Exactly(0)}
	_, err = json.Marshal(step)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	var s decad.Step
	err = json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"edges","preds":[],"exactly":0}],"values":["1 mm"]}`), &s)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	err = json.Unmarshal([]byte(`{"op":"fillet","inputs":[0],"selectors":[{"kind":"faces","preds":[],"at_least":-2}],"values":["1 mm"]}`), &s)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
