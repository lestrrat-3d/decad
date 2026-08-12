package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §14 PR4's replay regression
// guard: recipe-replay-design §8's own contract already allows the analytic
// upgrade with no wire change ("A later evaluator MUST reproduce ... one
// produced body per step ... measurements valid under its own
// Exactness/Bound. It need not reproduce v1's internal payload."). This
// proves that promise for an admitted coplanar Union step, and for the
// read-only OpIntersect the interference reading runs over an admitted
// coplanar pair: the stored steps' wire shape survives an encode/decode round
// trip unchanged, and re-issuing the feature calls the DECODED steps name
// reproduces the identical Volume/Exactness/Bound.
//
// "Replay" here is the caller-side loop docs/recipe-replay-design.md
// describes: decad has no replay entry point that rebuilds a document from a
// recipe value, so a caller reads the steps back and re-issues them.
// replayRecipe below is that caller, and every coordinate, extent and operand
// it feeds the public API comes out of the decoded value — never out of the
// literals the original document was built from.
// TestRecipeReplayPerturbedUnionSectionChangesVolume is the negative control
// proving it.

// buildAdmittedUnionDocument builds prism_boolean_test.go's own
// TestPrismUnionTwoBoxesSharingCapPlaneBuildsAnalyticPrism fixture — two
// coplanar, overlapping boxes sharing a cap plane, (0,0)-(10,10) and
// (5,5)-(15,15), both height 10 — and unions them through the analytic
// select-all/merge path (§4.2): the merged footprint is 100 + 100 - 25 (the
// 5x5 overlap) = 175 mm^2, times the shared height 10 mm = 1750 mm^3,
// Approximate.
func buildAdmittedUnionDocument(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 5, 5, 15, 15, 10)
	union, err := decad.Union(a, b)
	require.NoError(t, err)
	return doc, union
}

// replayRecipe re-issues a recipe's steps into a fresh document through the
// public API, driving every call from the recipe VALUE it is handed: an
// extrude rebuilds its sketch from the step's own recorded corners and passes
// the step's own Extent straight through, and a union keys its operands off
// the step's own Inputs. It returns the produced body of each step, indexed by
// StepRef, so a consumed operand's slot still names the step that produced it.
func replayRecipe(t *testing.T, r decad.Recipe) (*decad.Document, []*decad.Body) {
	t.Helper()
	doc := decad.New()
	bodies := make([]*decad.Body, len(r.Steps))
	for i, step := range r.Steps {
		switch step.Op {
		case decad.OpExtrude:
			bodies[i] = replayExtrude(t, doc, step)
		case decad.OpUnion:
			require.Len(t, step.Inputs, 2, `a union step records exactly two operands`)
			a, b := bodies[step.Inputs[0]], bodies[step.Inputs[1]]
			require.NotNil(t, a, `union operand A names an earlier step`)
			require.NotNil(t, b, `union operand B names an earlier step`)
			union, err := decad.Union(a, b)
			require.NoError(t, err)
			bodies[i] = union
		default:
			t.Fatalf(`this replay driver covers extrude and union steps, got %q`, step.Op)
		}
	}
	return doc, bodies
}

// replayExtrude re-issues one recorded extrude. Every corner it draws is a
// recorded LineSeg endpoint, and it walks the loop's chain — segment i's End
// is segment i+1's Start — so a displaced endpoint of either kind reaches the
// rebuilt sketch rather than being silently ignored.
func replayExtrude(t *testing.T, doc *decad.Document, step decad.Step) *decad.Body {
	t.Helper()
	require.Equal(t, decad.PlaneRecord{U: r3.Vec{X: 1}, V: r3.Vec{Y: 1}}, step.Plane,
		`this replay driver rebuilds a sketch on the world XY plane`)
	require.Empty(t, step.Profile.Holes, `this replay driver rebuilds a single-loop section`)
	segments := step.Profile.Outer.Segments
	require.NotEmpty(t, segments)

	lines := make([]decad.LineSeg, len(segments))
	for i, seg := range segments {
		line, ok := seg.(decad.LineSeg)
		require.Truef(t, ok, `this replay driver rebuilds line-only sections, got %T`, seg)
		lines[i] = line
	}
	for i, line := range lines {
		require.Equal(t, lines[(i+1)%len(lines)].Start, line.End,
			`a recorded loop's segment ends where the next one starts`)
	}

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := make([]*sketch.Point, len(lines))
	for i, line := range lines {
		corners[i] = s.CreatePoint(line.Start.U, line.Start.V)
	}
	s.Fix(corners[0])
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%len(corners)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1, `the rebuilt corners close exactly one region`)

	body, err := doc.Extrude(s, s.Profiles()[0], step.Extent)
	require.NoError(t, err)
	return body
}

// decodedCorner reads corner i of a recorded single-loop section: the start of
// its i-th segment.
func decodedCorner(t *testing.T, step decad.Step, i int) decad.Point2 {
	t.Helper()
	segments := step.Profile.Outer.Segments
	require.Less(t, i, len(segments))
	line, ok := segments[i].(decad.LineSeg)
	require.True(t, ok)
	return line.Start
}

// moveRecordedCorner displaces corner i of a recorded single-loop section by
// (du, dv) millimetres. The corner IS segment i's start and the previous
// segment's end, so both recorded endpoints move together and the loop stays
// closed — which is what makes the perturbed record a legal input to the same
// replay driver, and its differing volume evidence that the driver reads the
// record.
func moveRecordedCorner(t *testing.T, step *decad.Step, i int, du, dv float64) {
	t.Helper()
	segments := step.Profile.Outer.Segments
	require.Less(t, i, len(segments))
	start, ok := segments[i].(decad.LineSeg)
	require.True(t, ok)
	prev, ok := segments[(i-1+len(segments))%len(segments)].(decad.LineSeg)
	require.True(t, ok)
	require.Equal(t, prev.End, start.Start)

	moved := decad.Point2{U: start.Start.U + du, V: start.Start.V + dv}
	start.Start = moved
	prev.End = moved
	segments[i] = start
	segments[(i-1+len(segments))%len(segments)] = prev
}

func TestRecipeReplayAdmittedUnionStepMatchesOriginal(t *testing.T) {
	doc, union := buildAdmittedUnionDocument(t)
	original, err := union.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1750.0, volumeMM(t, original), 1e-9)

	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 3)
	require.Equal(t, decad.OpUnion, recipe.Steps[2].Op)

	data, err := json.Marshal(recipe)
	require.NoError(t, err)
	var decoded decad.Recipe
	require.NoError(t, json.Unmarshal(data, &decoded))
	require.Equal(t, recipe, decoded,
		`the stored OpUnion step's wire shape survives an encode/decode round trip unchanged`)
	require.Equal(t, decad.OpUnion, decoded.Steps[2].Op)
	require.Equal(t, []decad.StepRef{0, 1}, decoded.Steps[2].Inputs)

	// Replay: re-issue the feature calls the DECODED steps name, exactly as
	// recipe-replay-design.md documents a caller must. Nothing below reads the
	// literals buildAdmittedUnionDocument used.
	_, replayed := replayRecipe(t, decoded)
	require.Len(t, replayed, 3, `the replay produces one body per decoded step`)
	got, err := replayed[2].Volume()
	require.NoError(t, err)
	require.Equal(t, original.Value, got.Value, `the replayed step reproduces the original's Volume value`)
	require.Equal(t, original.Exactness, got.Exactness, `the replayed step reproduces the original's Exactness`)
	require.Equal(t, original.Bound, got.Bound, `the replayed step reproduces the original's Bound`)
}

// TestRecipeReplayPerturbedUnionSectionChangesVolume is the negative control
// for the test above: displacing one recorded corner of the decoded recipe
// before the replay must change the replayed volume. A replay that re-ran the
// original literals instead of the decoded record would report 1750 mm^3 here
// too, and so would prove nothing about the decoded value.
//
// The displaced corner is box A's own (0,0), moved 3 mm along u. That shears a
// 3 x 10 right triangle — 15 mm^2 — off A's 100 mm^2 footprint and leaves the
// 5x5 overlap with box B untouched, so the merged footprint is
// 85 + 100 - 25 = 160 mm^2 and the union's volume 1600 mm^3.
func TestRecipeReplayPerturbedUnionSectionChangesVolume(t *testing.T) {
	doc, union := buildAdmittedUnionDocument(t)
	original, err := union.Volume()
	require.NoError(t, err)

	data, err := json.Marshal(doc.Recipe())
	require.NoError(t, err)
	var decoded decad.Recipe
	require.NoError(t, json.Unmarshal(data, &decoded))

	require.Equal(t, decad.Point2{}, decodedCorner(t, decoded.Steps[0], 0),
		`the displaced corner is box A's own (0,0)`)
	moveRecordedCorner(t, &decoded.Steps[0], 0, 3, 0)

	_, replayed := replayRecipe(t, decoded)
	require.Len(t, replayed, 3, `the replay produces one body per decoded step`)
	got, err := replayed[2].Volume()
	require.NoError(t, err)
	require.InDelta(t, 1750.0, volumeMM(t, original), 1e-9)
	require.InDelta(t, 1600.0, volumeMM(t, got), 1e-9,
		`the replay derives its section from the decoded record, so a displaced corner moves the volume`)
}

// TestRecipeReplayAdmittedInterferencePairMatchesOriginal is the same guard
// over the reading docs/prism-boolean-design.md §14 PR4 actually rewires:
// interference.go's measuredInterference now resolves an admitted coplanar,
// co-directional prism pair through the read-only analytic OpIntersect
// dispatch. Replaying the recorded pair must not reach a different
// interference verdict than the live document did. The fixture is
// interference_test.go's own admitted pair — a (0,0)-(20,20) height 10
// container and a (5,5)-(9,10) height 15 nested prism sharing its base plane,
// overlapping over 4 x 5 x 10 = 200 mm^3.
func TestRecipeReplayAdmittedInterferencePairMatchesOriginal(t *testing.T) {
	doc := decad.New()
	container := boxBody(t, doc, 0, 0, 20, 20, 10)
	nested := boxBody(t, doc, 5, 5, 9, 10, 15)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	original := report.Interferences[0]
	require.Same(t, container, original.A)
	require.Same(t, nested, original.B)
	require.InDelta(t, 200.0, original.Volume.Value.Base(), 1e-9)

	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	data, err := json.Marshal(recipe)
	require.NoError(t, err)
	var decoded decad.Recipe
	require.NoError(t, json.Unmarshal(data, &decoded))
	require.Equal(t, recipe, decoded,
		`the stored extrude steps' wire shape survives an encode/decode round trip unchanged`)

	replayDoc, replayed := replayRecipe(t, decoded)
	require.Len(t, replayed, 2)
	replayReport, err := replayDoc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, replayReport.Status)
	require.Len(t, replayReport.Interferences, 1)
	got := replayReport.Interferences[0]
	require.Same(t, replayed[0], got.A)
	require.Same(t, replayed[1], got.B)
	require.Equal(t, original.Volume.Value, got.Volume.Value,
		`the replayed pair reproduces the original's interference volume`)
	require.Equal(t, original.Volume.Exactness, got.Volume.Exactness,
		`the replayed pair reproduces the original's interference Exactness`)
	require.Equal(t, original.Volume.Bound, got.Volume.Bound,
		`the replayed pair reproduces the original's interference Bound`)
}
