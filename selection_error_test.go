package decad_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// wrappedFaceSelector embeds *decad.FaceQuery, so it promotes the unexported
// sealed selector() marker and satisfies decad.FaceSelector even though decad
// owns the vocabulary — the standard Go embedding bypass of a pointer-receiver
// seal. Its own SelectFaces overrides the promoted one to resolve to no face.
type wrappedFaceSelector struct {
	*decad.FaceQuery
}

func (wrappedFaceSelector) SelectFaces(*decad.Body) ([]*decad.Face, error) {
	return nil, nil
}

// wrappedEdgeSelector is the edge analog: it embeds *decad.EdgeQuery to promote
// the seal and overrides SelectEdges to resolve to no edge.
type wrappedEdgeSelector struct {
	*decad.EdgeQuery
}

func (wrappedEdgeSelector) SelectEdges(*decad.Body) ([]*decad.Edge, error) {
	return nil, nil
}

// zAxis is the sweep/normal direction the rendering examples key on.
var zAxis = r3.NewVec(0, 0, 1)

func TestSelectorKindString(t *testing.T) {
	// The error's kind field is singular, naming the entity — deliberately
	// distinct from a query's plural prefix (core §9).
	require.Equal(t, "edge", decad.EdgeSelectorKind.String())
	require.Equal(t, "face", decad.FaceSelectorKind.String())
}

func TestQueryStringRendering(t *testing.T) {
	// A FeatureRef whose role itself holds parentheses and a comma: the role
	// is quoted, so it stays unambiguous inside the created_by payload.
	sideRef := decad.FeatureRef{Step: 2, Role: "side(0,1)"}
	capRef := decad.FeatureRef{Step: 3, Role: "capStart"}

	cases := []struct {
		name string
		got  string
		want string
	}{
		{"EdgesEmpty", decad.Edges().String(), "edges()"},
		{"FacesEmpty", decad.Faces().String(), "faces()"},
		{
			"EdgeConjunctionExactly",
			decad.Edges(decad.Convex(), decad.ParallelTo(zAxis)).Exactly(4).String(),
			"edges(convex, parallel_to(0,0,1)).exactly(4)",
		},
		{
			"FaceConjunctionExactly",
			decad.Faces(decad.Planar(), decad.NormalTo(zAxis)).Exactly(1).String(),
			"faces(planar, normal_to(0,0,1)).exactly(1)",
		},
		{
			"AtLeastSuffix",
			decad.Edges(decad.Concave()).AtLeast(1).String(),
			"edges(concave).at_least(1)",
		},
		{
			"CircularCircularCylindrical",
			decad.Faces(decad.Cylindrical()).String(),
			"faces(cylindrical)",
		},
		{
			"LongerThanValue",
			decad.Edges(decad.Circular(), decad.LongerThan(units.Millimeters(10))).String(),
			"edges(circular, longer_than(10 mm))",
		},
		{
			"CreatedByQuotedRole",
			decad.Edges(decad.CreatedBy(sideRef)).String(),
			`edges(created_by(2:"side(0,1)"))`,
		},
		{
			"FaceCreatedByRole",
			decad.Faces(decad.FaceCreatedBy(capRef)).String(),
			`faces(face_created_by(3:"capStart"))`,
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			require.Equal(t, c.want, c.got)
		})
	}
}

func TestQueryStringNegativeZeroNormalized(t *testing.T) {
	// -0.0 == 0.0, so two value-equal directions must render identically;
	// FormatFloat would otherwise write a negative zero as "-0". The
	// normalization is what makes "equal recorded queries render identically"
	// hold (core §9).
	plus := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1)))
	neg := decad.Faces(decad.NormalTo(r3.NewVec(math.Copysign(0, -1), math.Copysign(0, -1), 1)))
	require.Equal(t, "faces(normal_to(0,0,1))", plus.String())
	require.Equal(t, plus.String(), neg.String())
}

func TestQueryStringNonFiniteIsTotal(t *testing.T) {
	// Rendering is total: an unresolved query still prints, a non-finite
	// component read as the formatter writes it (core §9).
	q := decad.Edges(decad.ParallelTo(r3.NewVec(math.NaN(), math.Inf(1), math.Inf(-1))))
	require.Equal(t, "edges(parallel_to(NaN,+Inf,-Inf))", q.String())
}

func TestQueryStringDecodeRoundTrip(t *testing.T) {
	// A query and its decoded round-trip render identically: the Recipe JSON
	// codec is the round-trip channel (core §9/§6.2).
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)
	sel := decad.Edges(decad.Convex(), decad.ParallelTo(zAxis)).Exactly(4)
	before := sel.String()

	_, err := box.Fillet(sel, units.Millimeters(1))
	require.NoError(t, err)

	steps := doc.Recipe().Steps
	step := steps[len(steps)-1]
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Len(t, got.Selectors, 1)
	q, ok := got.Selectors[0].(*decad.EdgeQuery)
	require.True(t, ok)
	require.Equal(t, before, q.String())
}

func TestSelectionErrorUnassertedZero(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)

	// A box has no circular edges: an unasserted query that matches nothing is
	// ErrNoMatch, wrapped in a SelectionError.
	_, err := decad.Edges(decad.Circular()).SelectEdges(box)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, decad.EdgeSelectorKind, se.Kind)
	require.Equal(t, "edges(circular)", se.Query)
	require.Same(t, box, se.Body)
	require.Equal(t, "any", se.Expected)
	require.Equal(t, 0, se.Actual)
	require.Len(t, se.Residuals, 1)
	require.Equal(t, "circular", se.Residuals[0].Predicate)
	require.Equal(t, 0, se.Residuals[0].Remaining)
	require.Contains(t, se.Error(), "circular")
}

func TestSelectionErrorCardinalityMiss(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)

	// A box has twelve convex edges; asserting ninety-nine fails the
	// assertion, and the SelectionError reports the actual count.
	_, err := decad.Edges(decad.Convex()).Exactly(99).SelectEdges(box)
	require.ErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, "edges(convex).exactly(99)", se.Query)
	require.Equal(t, "exactly 99", se.Expected)
	require.Equal(t, 12, se.Actual)
	require.Len(t, se.Residuals, 1)
	require.Equal(t, 12, se.Residuals[0].Remaining)
}

func TestSelectionErrorResidualNamesEmptyingClause(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)

	// The conjunction is evaluated cumulatively in query order: parallel_to(z)
	// leaves the four vertical edges, then circular empties the set — the
	// residual funnel names which clause is at fault.
	_, err := decad.Edges(decad.ParallelTo(zAxis), decad.Circular()).Exactly(1).SelectEdges(box)
	require.ErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Len(t, se.Residuals, 2)
	require.Equal(t, "parallel_to(0,0,1)", se.Residuals[0].Predicate)
	require.Equal(t, 4, se.Residuals[0].Remaining)
	require.Equal(t, "circular", se.Residuals[1].Predicate)
	require.Equal(t, 0, se.Residuals[1].Remaining)
}

func TestSelectionErrorFaceQuery(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)

	// A box has no cylindrical faces.
	_, err := decad.Faces(decad.Cylindrical()).SelectFaces(box)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, decad.FaceSelectorKind, se.Kind)
	require.Equal(t, "faces(cylindrical)", se.Query)
	require.Equal(t, 0, se.Actual)
}

func TestImplicitExactlyOneStopFailedAssertionPassesThrough(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// The query's own explicit assertion fails first: a plate has six planar
	// faces, not three. The caller gets that SelectionError unchanged — its
	// Expected reflects the caller's own assertion, never overwritten to
	// "exactly 1" (core §9).
	_, err = doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Planar()).Exactly(3)})
	require.ErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, "exactly 3", se.Expected)
	require.Equal(t, 6, se.Actual)
}

func TestImplicitExactlyOneStopSuccessfulManyRewritten(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// An unasserted query that resolves to several faces succeeds at
	// SelectFaces, and the stop — which needs one — rewrites it to Expected
	// "exactly 1" / Actual the resolved count, preserving the query rendering.
	_, err = doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Planar())})
	require.ErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, "exactly 1", se.Expected)
	require.Equal(t, 6, se.Actual)
	require.Equal(t, "faces(planar)", se.Query)
}

func TestImplicitExactlyOneStopSatisfiedAssertionRewritten(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// A satisfied .Exactly(6) resolves to six faces; the stop turns that into
	// Expected "exactly 1" / Actual 6, exactly as an unasserted many-match
	// would — a satisfied assertion whose count is not one is still the
	// implicit exactly-one's to own (core §9).
	_, err = doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Planar()).Exactly(6)})
	require.ErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, "exactly 1", se.Expected)
	require.Equal(t, 6, se.Actual)
}

func TestImplicitExactlyOneStopUnassertedZeroRewritten(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// An unasserted resolution that matched nothing is ErrNoMatch from
	// SelectFaces; the stop rewrites it into ErrCardinality with Expected
	// "exactly 1" and Actual 0 — no longer ErrNoMatch.
	_, err = doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Cylindrical())})
	require.ErrorIs(t, err, decad.ErrCardinality)
	require.NotErrorIs(t, err, decad.ErrNoMatch)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, "exactly 1", se.Expected)
	require.Equal(t, 0, se.Actual)
}

func TestImplicitExactlyOneStopForeignSelectorRejected(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// decad owns the selector vocabulary: a foreign FaceSelector that only
	// satisfies the interface by embedding *decad.FaceQuery to promote the
	// sealed marker is malformed input, rejected as ErrDegenerate — never a
	// plain untyped cardinality error that errors.As could not reach.
	foreign := wrappedFaceSelector{FaceQuery: decad.Faces(decad.Planar())}
	_, err = doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: foreign})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.NotErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.NotErrorAs(t, err, &se)
}

func TestImplicitExactlyOneEdgeAxis(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	es, ep := plateSketch(t)
	host, err := doc.Extrude(es, ep, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	t.Run("FailedAssertionPassesThrough", func(t *testing.T) {
		// The selector's own .Exactly(2) fails against the host's many edges;
		// the axis gets that SelectionError unchanged.
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges().Exactly(2)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrCardinality)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, decad.EdgeSelectorKind, se.Kind)
		require.Equal(t, "exactly 2", se.Expected)
	})
	t.Run("SuccessfulManyRewritten", func(t *testing.T) {
		// An unasserted query matches every edge; the axis needs one and
		// rewrites to Expected "exactly 1".
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges()}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrCardinality)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, "exactly 1", se.Expected)
		require.Positive(t, se.Actual)
	})
	t.Run("UnassertedZeroRewritten", func(t *testing.T) {
		// A plate has no circular edges: zero matches, rewritten to Expected
		// "exactly 1" / Actual 0, no longer ErrNoMatch.
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges(decad.Circular())}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrCardinality)
		require.NotErrorIs(t, err, decad.ErrNoMatch)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, "exactly 1", se.Expected)
		require.Equal(t, 0, se.Actual)
	})
	t.Run("ForeignSelectorRejected", func(t *testing.T) {
		// A foreign EdgeSelector that satisfies the interface only by embedding
		// *decad.EdgeQuery to promote the sealed marker is malformed input,
		// rejected as ErrDegenerate — never a plain untyped cardinality error.
		foreign := wrappedEdgeSelector{EdgeQuery: decad.Edges()}
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: foreign}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrCardinality)
		var se *decad.SelectionError
		require.NotErrorAs(t, err, &se)
	})
}
