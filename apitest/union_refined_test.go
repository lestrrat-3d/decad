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
		got, err := decad.Union(append([]*decad.Body{hub}, lugs...)...).RefineTo(units.Millimeters(target)).Do(t.Context())
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
	// A bound the finer caps meet, and the 0.1 mm the default join holds, where
	// the pair's Refine leaves the join as it is.
	requireUnionRefinedEqualsPair(t, false, []float64{0.03, 0.1})
}

func TestUnionRefinedEqualsUnionThenRefineWhenTheAuditForcesARetry(t *testing.T) {
	// The fitted join's first attempt meets a cap and root contact in the
	// crossing audit and builds again at a quarter of the sag allowance, so
	// the equal mesh shows the direct build ran the same audit.
	requireUnionRefinedEqualsPair(t, true, []float64{0.08})
}

func TestUnionRefinedKeepsLaterOperationsAtTheRefinedBound(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	got, err := decad.Union(append([]*decad.Body{hub}, lugs[:4]...)...).RefineTo(units.Millimeters(0.03)).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, got), 0.03)
	// A serial union of one more lug keeps the refined ceiling, as after Refine.
	five, err := decad.Union(got, lugs[4]).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, five), 0.03)
	require.Contains(t, doc.Bodies(), five)
}

func TestUnionRefinedRefusesABoundTheJoinCannotMeetAndChangesNothing(t *testing.T) {
	// Refine of the Union at this bound refuses the same way; see TestRefineSectorJoinRefusals.
	bound := units.Millimeters(0.001)
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.Union(append([]*decad.Body{hub}, lugs...)...).RefineTo(bound).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "mm boundary proof")
	require.Contains(t, err.Error(), "call Union, then Refine")
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)

	// The fallback the message names: Union keeps the 0.1 mm join, as the
	// pair does when its Refine refuses.
	fallback, err := decad.Union(append([]*decad.Body{hub}, lugs...)...).Do(t.Context())
	require.NoError(t, err)
	wantDoc, hub2, lugs2 := unionRefinedOperands(t)
	want, err := decad.Union(append([]*decad.Body{hub2}, lugs2...)...).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, fingerprintUnionRefined(t, want), fingerprintUnionRefined(t, fallback))
	require.True(t, len(wantDoc.Bodies()) == 1 && wantDoc.Bodies()[0] == want)
}

func TestUnionRefinedRefusesAJoinFoldedTwoBodiesAtATime(t *testing.T) {
	// A serial join is no blank and teeth: the earlier join stands in for the
	// blank, so the sector route does not take the set.
	doc, hub, lugs := unionRefinedOperands(t)
	joined, err := decad.Union(append([]*decad.Body{hub}, lugs[:3]...)...).Do(t.Context())
	require.NoError(t, err)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.Union(append([]*decad.Body{joined}, lugs[3:]...)...).RefineTo(units.Millimeters(0.03)).Do(t.Context())
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
	got, err := decad.Union(bodies...).RefineTo(units.Millimeters(0.05)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "call Union, then Refine")
	require.True(t, got == nil)
	requireUnionRefinedUnchanged(t, doc, operands, history)
}

// TestUnionRefinedRefusesBeforeBuilding runs every refusal UnionRefined makes before it builds anything on one
// operand set. A refusal changes nothing, so each case checks the document is as it was, and the set serves the next.
func TestUnionRefinedRefusesBeforeBuilding(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	all := append([]*decad.Body{hub}, lugs...)

	t.Run("a nil context and too few bodies", func(t *testing.T) {
		//nolint:staticcheck // a nil context is the case under test
		_, err := decad.Union(all...).RefineTo(units.Millimeters(0.05)).Do(nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Union(hub).RefineTo(units.Millimeters(0.05)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Union(nil...).RefineTo(units.Millimeters(0.05)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		requireUnionRefinedUnchanged(t, doc, operands, history)
	})
	t.Run("repeated operands", func(t *testing.T) {
		_, err := decad.Union(append([]*decad.Body{hub, lugs[0]}, lugs...)...).RefineTo(units.Millimeters(0.05)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		requireUnionRefinedUnchanged(t, doc, operands, history)
	})
	t.Run("a canceled context", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		got, err := decad.Union(all...).RefineTo(units.Millimeters(0.03)).Do(ctx)
		require.ErrorIs(t, err, context.Canceled)
		require.True(t, got == nil)
		requireUnionRefinedUnchanged(t, doc, operands, history)
	})
	badBounds := []struct {
		name  string
		bound units.Value
		want  error
	}{
		{"an angle", units.Radians(0.05), decad.ErrUnitKind},
		{"NaN", units.Millimeters(math.NaN()), decad.ErrNotFinite},
		{"below zero", units.Millimeters(-0.05), decad.ErrNegativeMagnitude},
		{"exactly zero", units.Millimeters(0), decad.ErrDegenerate},
	}
	for _, tc := range badBounds {
		t.Run("a bad bound: "+tc.name, func(t *testing.T) {
			got, err := decad.Union(all...).RefineTo(tc.bound).Do(t.Context())
			require.ErrorIs(t, err, tc.want)
			require.True(t, got == nil)
			requireUnionRefinedUnchanged(t, doc, operands, history)
		})
	}
	// A hub and one lug join as a pair at 0.1 mm, and Refine refuses 0.006 mm.
	// UnionRefined refuses the pair up front, and changes nothing, rather than
	// return that 0.1 mm join as if it met the bound.
	t.Run("a pair at a bound the join cannot meet", func(t *testing.T) {
		bound := units.Millimeters(0.006)
		got, err := decad.Union(hub, lugs[0]).RefineTo(bound).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "call Union, then Refine")
		require.True(t, got == nil)
		requireUnionRefinedUnchanged(t, doc, operands, history)

		// The pair's own Union and Refine run last, as the Union consumes the set.
		joined, err := decad.Union(hub, lugs[0]).Do(t.Context())
		require.NoError(t, err)
		refined, pairErr := decad.Refine(joined, bound).Do(t.Context())
		require.ErrorIs(t, pairErr, decad.ErrUnsupported)
		require.True(t, refined == nil)
		require.Contains(t, doc.Bodies(), joined)
	})
}
