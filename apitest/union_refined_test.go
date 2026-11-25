package apitest_test

// CI: isolated-tests

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireUnionRefinedEqualsPair builds the join both ways at each bound and
// requires the same mesh, readings and warnings.
func requireUnionRefinedEqualsPair(t *testing.T, fitted bool, targets []float64) {
	t.Helper()
	for _, target := range targets {
		wantDoc, want, err := unionThenRefine(t, fitted, units.Millimeters(target))
		require.NoError(t, err, "target %g", target)

		doc, hub, lugs := unionRefinedSet(t, fitted)
		before := len(doc.OperationHistory().Operations())
		got, err := decad.UnionRefined(t.Context(), append([]*decad.Body{hub}, lugs...), units.Millimeters(target))
		require.NoError(t, err, "target %g", target)

		require.Equal(t, fingerprintUnionRefined(t, want), fingerprintUnionRefined(t, got), "target %g", target)
		require.True(t, len(doc.Bodies()) == 1 && doc.Bodies()[0] == got, "target %g", target)
		require.Len(t, wantDoc.Bodies(), 1)
		require.LessOrEqual(t, refineSectorFloor(t, got), math.Max(target, refineSectorFloor(t, want)))
		// One Union entry and no Refine entry; the pair has both.
		requireOperationKinds(t, doc, before, decad.OperationUnion)
		pairOps := wantDoc.OperationHistory().Operations()
		require.Equal(t, decad.OperationUnion, pairOps[len(pairOps)-2].Kind())
		require.Equal(t, decad.OperationRefine, pairOps[len(pairOps)-1].Kind())
	}
}

func TestUnionRefinedEqualsUnionThenRefineOnASectorJoin(t *testing.T) {
	// Below, at and above the 0.1 mm the default join holds, and one bound the
	// finer caps still meet.
	requireUnionRefinedEqualsPair(t, false, []float64{0.02, 0.03, 0.05, 0.09, 0.1, 0.5})
}

func TestUnionRefinedEqualsUnionThenRefineWhenTheAuditForcesARetry(t *testing.T) {
	// The fitted join's first attempt meets a cap and root contact in the
	// crossing audit and builds again at a quarter of the sag allowance, so
	// the equal mesh shows the direct build ran the same audit.
	requireUnionRefinedEqualsPair(t, true, []float64{0.08, 0.5})
}

func TestUnionRefinedKeepsLaterOperationsAtTheRefinedBound(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	got, err := decad.UnionRefined(t.Context(), append([]*decad.Body{hub}, lugs[:4]...), units.Millimeters(0.03))
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, got), 0.03)
	// A serial union of one more lug keeps the refined ceiling, as after Refine.
	five, err := decad.Union(t.Context(), got, lugs[4])
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, five), 0.03)
	require.Contains(t, doc.Bodies(), five)
}

func TestUnionRefinedRefusesABoundTheJoinCannotMeetAndChangesNothing(t *testing.T) {
	bound := units.Millimeters(0.001)
	_, pairBody, pairErr := unionThenRefine(t, false, bound)
	require.ErrorIs(t, pairErr, decad.ErrUnsupported)
	require.True(t, pairBody == nil)

	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.UnionRefined(t.Context(), append([]*decad.Body{hub}, lugs...), bound)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "mm boundary proof")
	require.Contains(t, err.Error(), "call Union, then Refine")
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)

	// The fallback the message names: Union keeps the 0.1 mm join, as the
	// pair does when its Refine refuses.
	fallback, err := decad.Union(t.Context(), hub, lugs...)
	require.NoError(t, err)
	wantDoc, hub2, lugs2 := unionRefinedOperands(t)
	want, err := decad.Union(t.Context(), hub2, lugs2...)
	require.NoError(t, err)
	require.Equal(t, fingerprintUnionRefined(t, want), fingerprintUnionRefined(t, fallback))
	require.True(t, len(wantDoc.Bodies()) == 1 && wantDoc.Bodies()[0] == want)
}

func TestUnionRefinedRefusesAPairAtABoundTheJoinCannotMeet(t *testing.T) {
	// A hub and one lug join as a pair at 0.1 mm, and Refine refuses 0.006 mm.
	// UnionRefined refuses the pair up front, and changes nothing, rather than
	// return that 0.1 mm join as if it met the bound.
	bound := units.Millimeters(0.006)
	pairDoc, pairHub, pairLugs := unionRefinedOperands(t)
	joined, err := decad.Union(t.Context(), pairHub, pairLugs[0])
	require.NoError(t, err)
	refined, pairErr := decad.Refine(t.Context(), joined, bound)
	require.ErrorIs(t, pairErr, decad.ErrUnsupported)
	require.True(t, refined == nil)
	require.Contains(t, pairDoc.Bodies(), joined)

	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.UnionRefined(t.Context(), []*decad.Body{hub, lugs[0]}, bound)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "call Union, then Refine")
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}

func TestUnionRefinedRefusesAJoinFoldedTwoBodiesAtATime(t *testing.T) {
	// A serial join is no blank and teeth: the earlier join stands in for the
	// blank, so the sector route does not take the set.
	doc, hub, lugs := unionRefinedOperands(t)
	joined, err := decad.Union(t.Context(), hub, lugs[:3]...)
	require.NoError(t, err)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.UnionRefined(t.Context(), append([]*decad.Body{joined}, lugs[3:]...), units.Millimeters(0.03))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "call Union, then Refine")
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}

func TestUnionRefinedRefusesASetThatIsNoSectorJoinAndChangesNothing(t *testing.T) {
	// Three overlapping prisms join analytically, not as a sector join.
	doc := decad.New()
	bodies := []*decad.Body{
		xCylinder(t, doc, 0, 10, 0, 0, 2),
		xCylinder(t, doc, 8, 10, 0, 0, 2),
		xCylinder(t, doc, 16, 10, 0, 0, 2),
	}
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.UnionRefined(t.Context(), bodies, units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "call Union, then Refine")
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}

func TestUnionRefinedRejectsABadBoundBeforeBuilding(t *testing.T) {
	cases := []struct {
		name  string
		bound units.Value
		want  error
	}{
		{"an angle", units.Radians(0.05), decad.ErrUnitKind},
		{"NaN", units.Millimeters(math.NaN()), decad.ErrNotFinite},
		{"below zero", units.Millimeters(-0.05), decad.ErrNegativeMagnitude},
		{"exactly zero", units.Millimeters(0), decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			doc, hub, lugs := unionRefinedOperands(t)
			operands := doc.Bodies()
			history := len(doc.OperationHistory().Operations())
			got, err := decad.UnionRefined(t.Context(), append([]*decad.Body{hub}, lugs...), tc.bound)
			require.ErrorIs(t, err, tc.want)
			require.True(t, got == nil)
			requireUnionRefinedUnchanged(t, doc, operands, history)
		})
	}
}

func TestUnionRefinedRejectsNilContextAndTooFewBodies(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	//nolint:staticcheck // a nil context is the case under test
	_, err := decad.UnionRefined(nil, append([]*decad.Body{hub}, lugs...), units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.UnionRefined(t.Context(), []*decad.Body{hub}, units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.UnionRefined(t.Context(), nil, units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}

func TestUnionRefinedRefusesRepeatedOperands(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	_, err := decad.UnionRefined(t.Context(), append([]*decad.Body{hub, lugs[0]}, lugs...), units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}

func TestUnionRefinedStopsOnACanceledContext(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err := decad.UnionRefined(ctx, append([]*decad.Body{hub}, lugs...), units.Millimeters(0.03))
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}
