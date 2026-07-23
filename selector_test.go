package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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
		step := decad.Step{Op: decad.OpFillet, Inputs: []decad.StepRef{0}, Selectors: []decad.Selector{sel}}
		buf, err := json.Marshal(step)
		require.NoError(t, err, `%T should encode`, sel)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got), `%T should decode`, sel)
		require.Equal(t, step, got, `a step carrying a %T round-trips field for field`, sel)
	}

	// A later cardinality assertion replaces the earlier one.
	step := decad.Step{Op: decad.OpFillet, Selectors: []decad.Selector{decad.Edges(decad.Convex()).Exactly(4).AtLeast(1)}}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	require.Contains(t, string(buf), `"at_least":1`)
	require.NotContains(t, string(buf), `"exactly"`)
}

func TestSelectorCodecRejections(t *testing.T) {
	// The set is closed: unknown and missing tags are rejected, at the
	// selector level and inside each predicate tier.
	var step decad.Step
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"vertices","preds":[]}]}`), &step),
		`an unknown selector kind is rejected`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"preds":[]}]}`), &step),
		`a selector with no kind tag is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"glowing"}]}]}`), &step),
		`an unknown edge predicate kind is rejected`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"convex"}]}]}`), &step),
		`the predicate tiers are disjoint: an edge predicate is not a face predicate`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[{}]}]}`), &step),
		`a predicate with no kind tag is malformed`)

	// Absent fields are malformed, never silently zeroed.
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges"}]}`), &step),
		`a query with no preds is malformed, never silently match-all`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"parallel_to"}]}]}`), &step),
		`a parallel-to predicate with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"longer_than"}]}]}`), &step),
		`a longer-than predicate with no l is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"created_by"}]}]}`), &step),
		`a created-by predicate with no ref is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":1}}]}]}`), &step),
		`a provenance ref with no role is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"normal_to"}]}]}`), &step),
		`a normal-to predicate with no dir is malformed`)
	require.Error(t, json.Unmarshal([]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"role":"capEnd"}}]}]}`), &step),
		`a provenance ref with no step is malformed`)
	for _, data := range []string{
		`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":0,"role":""}}]}]}`,
		`{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-1,"role":"capStart"}}]}]}`,
		`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":0,"role":""}}]}]}`,
		`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-1,"role":"capStart"}}]}]}`,
	} {
		err := json.Unmarshal([]byte(data), &step)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	}
	require.Error(t, json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[],"exactly":4,"at_least":1}]}`), &step),
		`a query carries at most one cardinality assertion`)

	// A zero-value predicate names nothing: only the constructors build one.
	_, err := json.Marshal(decad.Step{Op: decad.OpFillet, Selectors: []decad.Selector{decad.Edges(decad.EdgePredicate{})}})
	require.Error(t, err, `a zero-value edge predicate refuses to encode`)
	_, err = json.Marshal(decad.Step{Op: decad.OpShell, Selectors: []decad.Selector{decad.Faces(decad.FacePredicate{})}})
	require.Error(t, err, `a zero-value face predicate refuses to encode`)
}

func TestSelectorCodecRejectsOutOfRangeNegativeProvenanceSteps(t *testing.T) {
	for _, tc := range []struct {
		name string
		data string
	}{
		{
			name: "edge created-by",
			data: `{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-9223372036854775809,"role":"capStart"}}]}]}`,
		},
		{
			name: "face created-by",
			data: `{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-9223372036854775809,"role":"capStart"}}]}]}`,
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
			data: `{"op":"fillet","selectors":[{"kind":"edges","preds":[{"kind":"created_by","ref":{"step":-0,"role":"capStart"}}]}]}`,
			want: decad.Step{Op: decad.OpFillet, Selectors: []decad.Selector{decad.Edges(decad.CreatedBy(ref))}},
		},
		{
			name: "face created-by",
			data: `{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"face_created_by","ref":{"step":-0,"role":"capStart"}}]}]}`,
			want: decad.Step{Op: decad.OpShell, Selectors: []decad.Selector{decad.Faces(decad.FaceCreatedBy(ref))}},
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
	_, err := json.Marshal(decad.Step{Op: decad.OpFillet, Selectors: []decad.Selector{(*decad.EdgeQuery)(nil)}})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = json.Marshal(decad.Step{Op: decad.OpShell, Selectors: []decad.Selector{(*decad.FaceQuery)(nil)}})
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
