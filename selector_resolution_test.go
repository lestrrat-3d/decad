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
	_, err = json.Marshal(decad.Step{Op: decad.OpFillet, Selectors: []decad.Selector{decad.Edges(decad.Circular()).Exactly(0)}})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	var s decad.Step
	err = json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"edges","preds":[],"exactly":0}]}`), &s)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	err = json.Unmarshal([]byte(`{"op":"fillet","selectors":[{"kind":"faces","preds":[],"at_least":-2}]}`), &s)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
