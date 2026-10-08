package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers the query union of docs/api-design.md §9: EdgeQuery.Or and
// FaceQuery.Or add branches, an entity matches when every clause of one branch
// holds, and the cardinality assertion counts the union's distinct entities.
//
// Legs shown to fail before these fixtures were accepted: matching only the
// first branch sends the quarter-turn shells, the union cardinality test and
// the prism edge unions red; appending an entity once per matching branch,
// instead of once, sends the duplicate test red (8 edges rather than 4);
// enforcing the cardinality on the first branch's count alone sends the
// quarter-turn shells, the union cardinality test and the prism edge unions
// red; dropping the ".or" rendering sends the String, slice-copy, union
// cardinality and error-message assertions red; dropping the branch label from
// Error sends both emptied-branch messages red; and evaluating each branch's
// residuals from the previous branch's survivors, instead of the whole face
// list, sends the union cardinality test's residuals red.

// Every revolve here takes quarterTurn (surface_revolve_test.go) about the
// sketch's u axis, so one angular cap lies in the XY plane and the other in
// the XZ plane.

// quarterTurnVolume is Pappus over a quarter turn: V = (π/2)·∫ρ dA.
func quarterTurnVolume(q float64) units.Value { return units.CubicMillimeters(math.Pi / 2 * q) }

// angularCaps names both angular caps of b by provenance, one branch each.
func angularCaps(b *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.FaceCreatedBy(decad.CapStart(b))).Or(decad.FaceCreatedBy(decad.CapEnd(b)))
}

func TestSelectorUnionShellsQuarterTurnRevolve(t *testing.T) {
	t.Parallel()
	t.Run("ring", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, quarterTurn)
		// The caps are not coplanar, and the meridian's two end faces are
		// planar too: no conjunction of geometric predicates names exactly
		// the caps.
		planar, err := decad.Faces(decad.Planar()).SelectFaces(ring)
		require.NoError(t, err)
		require.Len(t, planar, 4, `two angular caps and two meridian end faces`)
		normalZ, err := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))).SelectFaces(ring)
		require.NoError(t, err)
		require.Len(t, normalZ, 1, `only the start cap lies in the XY plane`)

		caps, err := angularCaps(ring).Exactly(2).SelectFaces(ring)
		require.NoError(t, err)
		require.Len(t, caps, 2)
		for _, f := range caps {
			// Each cap is the meridian rectangle itself: 20 × 5.
			area, err := f.Area()
			require.NoError(t, err)
			decadtest.Measures(t, "cap area", area, units.SquareMillimeters(100))
		}

		shelled, err := ring.Shell(t.Context(), angularCaps(ring).Exactly(2), units.Millimeters(1))
		require.NoError(t, err)
		require.True(t, shelled.IsSolid())
		requireManifold(t, shelled)
		// The wall is the meridian less its 1 mm erosion z ∈ [1, 19],
		// ρ ∈ [6, 9]: ∫ρ dA = 75/2·20 − 45/2·18 = 345, as the half-turn ring.
		decadtest.MeasuresVolume(t, shelled, quarterTurnVolume(345))
		require.ElementsMatch(t, []float64{5, 6, 9, 10}, cylinderRadii(t, shelled))
	})
	t.Run("solid cylinder", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, quarterTurn)
		shelled, err := cyl.Shell(t.Context(), angularCaps(cyl).Exactly(2), units.Millimeters(2))
		require.NoError(t, err)
		requireManifold(t, shelled)
		// The axis walk grows no wall: the cavity is z ∈ [2, 18], ρ ∈ [0, 8],
		// so ∫ρ dA = 100/2·20 − 64/2·16 = 488.
		decadtest.MeasuresVolume(t, shelled, quarterTurnVolume(488))
	})
}

func TestSelectorUnionCardinalityCountsTheUnion(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ring := revolveMeridian(t, doc, ringMeridian, quarterTurn)

	q := angularCaps(ring).Exactly(1)
	_, err := q.SelectFaces(ring)
	require.ErrorIs(t, err, decad.ErrCardinality)
	var se *decad.SelectionError
	require.ErrorAs(t, err, &se)
	require.Equal(t, decad.FaceSelectorKind, se.Kind)
	require.Equal(t, q.String(), se.Query)
	require.Contains(t, se.Query, `).or(face_created_by(`)
	require.Contains(t, se.Error(), se.Query, `the message names the whole union`)
	require.Equal(t, "exactly 1", se.Expected)
	require.Equal(t, 2, se.Actual)
	// Each branch restarts from the ring's six faces and keeps its one cap.
	require.Len(t, se.Residuals, 2)
	require.Equal(t, 0, se.Residuals[0].Branch)
	require.Equal(t, 1, se.Residuals[0].Remaining)
	require.Equal(t, 1, se.Residuals[1].Branch)
	require.Equal(t, 1, se.Residuals[1].Remaining)

	// The assertion counts the union whether it is stated before or after Or.
	caps, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(ring))).Exactly(2).
		Or(decad.FaceCreatedBy(decad.CapEnd(ring))).SelectFaces(ring)
	require.NoError(t, err)
	require.Len(t, caps, 2)
}

func TestSelectorUnionEdgesOnAPrism(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)
	xAxis := r3.NewVec(1, 0, 0)

	// edgeDir is an edge's start-to-end vector in stored coordinates.
	edgeDir := func(e *decad.Edge) r3.Vec {
		return e.End().Position().Value.Sub(e.Start().Position().Value)
	}
	vertical := func(d r3.Vec) bool { return d.X == 0 && d.Y == 0 }
	alongX := func(d r3.Vec) bool { return d.Y == 0 && d.Z == 0 }

	t.Run("two directions", func(t *testing.T) {
		edges, err := decad.Edges(decad.ParallelTo(zAxis)).Or(decad.ParallelTo(xAxis)).Exactly(8).SelectEdges(box)
		require.NoError(t, err)
		var nVertical, nAlongX int
		for _, e := range edges {
			d := edgeDir(e)
			length, err := e.Length()
			require.NoError(t, err)
			switch {
			case vertical(d):
				nVertical++
				decadtest.Measures(t, "vertical edge length", length, units.Millimeters(5))
			case alongX(d):
				nAlongX++
				decadtest.Measures(t, "x edge length", length, units.Millimeters(10))
			default:
				t.Fatalf("edge direction %v is neither vertical nor along x", d)
			}
		}
		require.Equal(t, 4, nVertical)
		require.Equal(t, 4, nAlongX)
	})
	t.Run("keeps topology order", func(t *testing.T) {
		got, err := decad.Edges(decad.ParallelTo(xAxis)).Or(decad.ParallelTo(zAxis)).SelectEdges(box)
		require.NoError(t, err)
		var want []*decad.Edge
		for _, e := range box.Edges() {
			if d := edgeDir(e); vertical(d) || alongX(d) {
				want = append(want, e)
			}
		}
		require.Len(t, want, 8)
		require.Equal(t, want, got, `the union keeps Body.Edges() order, not branch order`)
	})
	t.Run("duplicates count once", func(t *testing.T) {
		// Every vertical edge is convex, so both branches match all four.
		edges, err := decad.Edges(decad.ParallelTo(zAxis)).Or(decad.ParallelTo(zAxis), decad.Convex()).
			Exactly(4).SelectEdges(box)
		require.NoError(t, err)
		require.Len(t, edges, 4)
		seen := map[*decad.Edge]struct{}{}
		for _, e := range edges {
			seen[e] = struct{}{}
		}
		require.Len(t, seen, 4)
		// The box's twelve edges are all convex: the union is the whole box.
		all, err := decad.Edges(decad.Convex()).Or(decad.ParallelTo(zAxis)).Exactly(12).SelectEdges(box)
		require.NoError(t, err)
		require.Len(t, all, 12)
	})
	t.Run("empty branch matches everything", func(t *testing.T) {
		all, err := decad.Edges(decad.Circular()).Or().Exactly(12).SelectEdges(box)
		require.NoError(t, err)
		require.Len(t, all, 12)
	})
}

func TestSelectorUnionErrorNamesEachEmptiedBranch(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)

	t.Run("one branch empties", func(t *testing.T) {
		_, err := decad.Edges(decad.ParallelTo(zAxis)).Or(decad.Convex(), decad.Circular()).Exactly(5).SelectEdges(box)
		require.ErrorIs(t, err, decad.ErrCardinality)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, 4, se.Actual)
		require.Equal(t, []decad.PredicateResidual{
			{Branch: 0, Predicate: "parallel_to(0,0,1)", Remaining: 4},
			{Branch: 1, Predicate: "convex", Remaining: 12},
			{Branch: 1, Predicate: "circular", Remaining: 0},
		}, se.Residuals)
		require.Contains(t, se.Error(), "; the clause circular of branch 1 matched none")
		require.NotContains(t, se.Error(), "of branch 0")
	})
	t.Run("every branch empties", func(t *testing.T) {
		_, err := decad.Edges(decad.Circular()).Or(decad.Free()).SelectEdges(box)
		require.ErrorIs(t, err, decad.ErrNoMatch)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, "edges(circular).or(free)", se.Query)
		require.Equal(t, "any", se.Expected)
		require.Equal(t, 0, se.Actual)
		require.Contains(t, se.Error(),
			"; the clause circular of branch 0 matched none; the clause free of branch 1 matched none")
	})
	t.Run("predicates of a later branch are validated", func(t *testing.T) {
		_, err := decad.Edges(decad.Convex()).Or(decad.ParallelTo(r3.Vec{})).SelectEdges(box)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

func TestSelectorUnionString(t *testing.T) {
	t.Parallel()
	capStart := decad.FeatureRef{Role: roleCapStart}
	capEnd := decad.FeatureRef{Role: roleCapEnd}
	cases := []struct {
		name string
		got  string
		want string
	}{
		{
			"FaceCapsExactly",
			decad.Faces(decad.FaceCreatedBy(capStart)).Or(decad.FaceCreatedBy(capEnd)).Exactly(2).String(),
			`faces(face_created_by(0:"capStart")).or(face_created_by(0:"capEnd")).exactly(2)`,
		},
		{
			// The cardinality renders last, after every branch, whatever the
			// call order.
			"CardinalityBeforeOr",
			decad.Edges(decad.Convex()).AtLeast(1).Or(decad.Circular(), decad.Free()).String(),
			"edges(convex).or(circular, free).at_least(1)",
		},
		{
			"ThreeBranches",
			decad.Edges(decad.ParallelTo(zAxis)).Or(decad.Circular()).Or(decad.Concave()).String(),
			"edges(parallel_to(0,0,1)).or(circular).or(concave)",
		},
		{
			"EmptyBranches",
			decad.Faces().Or().String(),
			"faces().or()",
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			require.Equal(t, c.want, c.got)
		})
	}
}

func TestSelectorUnionCopiesCallerSlice(t *testing.T) {
	t.Parallel()
	preds := []decad.EdgePredicate{decad.Circular()}
	q := decad.Edges(decad.Convex()).Or(preds...)
	preds[0] = decad.Concave()
	require.Equal(t, "edges(convex).or(circular)", q.String())
}
