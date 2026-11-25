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

func TestUnionRefinedEqualsUnionThenRefineOnASectorJoin(t *testing.T) {
	// Below, at and above the 0.1 mm the default join holds, and one bound the
	// finer caps still meet.
	for _, target := range []float64{0.02, 0.03, 0.05, 0.09, 0.1, 0.5} {
		wantDoc, want, err := unionThenRefine(t, units.Millimeters(target))
		require.NoError(t, err, "target %g", target)

		doc, hub, lugs := unionRefinedOperands(t)
		before := len(doc.OperationHistory().Operations())
		got, err := decad.UnionRefined(t.Context(), units.Millimeters(target), hub, lugs...)
		require.NoError(t, err, "target %g", target)

		require.Equal(t, fingerprintUnionRefined(t, want), fingerprintUnionRefined(t, got), "target %g", target)
		require.Equal(t, []*decad.Body{got}, doc.Bodies())
		require.Len(t, wantDoc.Bodies(), 1)
		require.LessOrEqual(t, refineSectorFloor(t, got), math.Max(target, refineSectorFloor(t, want)))
		// One Union entry and no Refine entry; the pair has both.
		requireOperationKinds(t, doc, before, decad.OperationUnion)
		pairOps := wantDoc.OperationHistory().Operations()
		require.Equal(t, decad.OperationUnion, pairOps[len(pairOps)-2].Kind())
		require.Equal(t, decad.OperationRefine, pairOps[len(pairOps)-1].Kind())
	}
}

func TestUnionRefinedKeepsLaterOperationsAtTheRefinedBound(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	got, err := decad.UnionRefined(t.Context(), units.Millimeters(0.03), hub, lugs[:4]...)
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
	_, pairBody, pairErr := unionThenRefine(t, bound)
	require.ErrorIs(t, pairErr, decad.ErrUnsupported)
	require.Nil(t, pairBody)

	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	got, err := decad.UnionRefined(t.Context(), bound, hub, lugs...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "mm boundary proof")
	require.Nil(t, got)
	require.Equal(t, operands, doc.Bodies())
	require.Len(t, doc.OperationHistory().Operations(), history)

	// The fallback the doc comment names: Union keeps the 0.1 mm join, as the
	// pair does when its Refine refuses.
	fallback, err := decad.Union(t.Context(), hub, lugs...)
	require.NoError(t, err)
	wantDoc, hub2, lugs2 := unionRefinedOperands(t)
	want, err := decad.Union(t.Context(), hub2, lugs2...)
	require.NoError(t, err)
	require.Equal(t, fingerprintUnionRefined(t, want), fingerprintUnionRefined(t, fallback))
	require.Equal(t, []*decad.Body{want}, wantDoc.Bodies())
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
			got, err := decad.UnionRefined(t.Context(), tc.bound, hub, lugs...)
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, got)
			require.Equal(t, operands, doc.Bodies())
			require.Len(t, doc.OperationHistory().Operations(), history)
		})
	}
}

func TestUnionRefinedRejectsNilContextAndTooFewBodies(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	//nolint:staticcheck // a nil context is the case under test
	_, err := decad.UnionRefined(nil, units.Millimeters(0.05), hub, lugs...)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.UnionRefined(t.Context(), units.Millimeters(0.05), hub)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, operands, doc.Bodies())
}

func TestUnionRefinedStopsOnACanceledContext(t *testing.T) {
	doc, hub, lugs := unionRefinedOperands(t)
	operands := doc.Bodies()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err := decad.UnionRefined(ctx, units.Millimeters(0.03), hub, lugs...)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, got)
	require.Equal(t, operands, doc.Bodies())
}

func TestUnionRefinedOfAPairIsUnionThenRefine(t *testing.T) {
	// Two bodies take the pair route, where the join is built at 0.1 mm and
	// Refine rebuilds it, so the pair and UnionRefined agree trivially.
	build := func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
		doc, hub, lugs := unionRefinedOperands(t)
		return doc, hub, lugs[0]
	}
	_, hubA, lugA := build(t)
	joined, err := decad.Union(t.Context(), hubA, lugA)
	require.NoError(t, err)
	want, wantErr := decad.Refine(t.Context(), joined, units.Millimeters(0.05))

	docB, hubB, lugB := build(t)
	got, gotErr := decad.UnionRefined(t.Context(), units.Millimeters(0.05), hubB, lugB)
	if wantErr != nil {
		require.EqualError(t, gotErr, wantErr.Error())
		require.NotNil(t, got, "the union stays committed beside the Refine error")
		require.Contains(t, docB.Bodies(), got)
		return
	}
	require.NoError(t, gotErr)
	require.Equal(t, fingerprintUnionRefined(t, want), fingerprintUnionRefined(t, got))
}

func TestUnionRefinedKeepsTheUnionWhenRefineRefusesANonMeshJoin(t *testing.T) {
	// Three overlapping prisms join analytically, so Refine refuses the result.
	build := func() (*decad.Document, []*decad.Body) {
		doc := decad.New()
		return doc, []*decad.Body{
			xCylinder(t, doc, 0, 10, 0, 0, 2),
			xCylinder(t, doc, 8, 10, 0, 0, 2),
			xCylinder(t, doc, 16, 10, 0, 0, 2),
		}
	}
	_, pair := build()
	joined, err := decad.Union(t.Context(), pair[0], pair[1:]...)
	require.NoError(t, err)
	_, wantErr := decad.Refine(t.Context(), joined, units.Millimeters(0.05))
	require.Error(t, wantErr)

	doc, bodies := build()
	got, err := decad.UnionRefined(t.Context(), units.Millimeters(0.05), bodies[0], bodies[1:]...)
	require.EqualError(t, err, wantErr.Error())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotNil(t, got)
	require.Equal(t, []*decad.Body{got}, doc.Bodies())
	volume, err := got.Volume()
	require.NoError(t, err)
	want, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(want.Value.Base()), math.Float64bits(volume.Value.Base()))
}
