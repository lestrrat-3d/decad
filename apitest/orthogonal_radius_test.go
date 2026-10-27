package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-wall-design.md §4's public-surface tests for Verify's concave-radius survey of
// a faceted solid every face of which squares to a world axis.

func TestOrthogonalConcaveRadiusIsAbsentFromABlockAndZeroFromAnL(t *testing.T) {
	t.Parallel()
	blockDoc := decad.New()
	_ = blockIn(t, blockDoc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	report, err := blockDoc.Verify(t.Context(), decad.WithConcaveRadius())
	require.NoError(t, err)
	require.Equal(t, decad.ScalarAbsent, report.Bodies[0].ConcaveRadius.Outcome, "a block has no concave edge")
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)

	lDoc := decad.New()
	a := blockIn(t, lDoc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := blockIn(t, lDoc, r3.NewVec(0, 0, 0), r3.NewVec(4, 10, 4))
	_, err = decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	report, err = lDoc.Verify(t.Context(), decad.WithConcaveRadius())
	require.NoError(t, err)
	radius := report.Bodies[0].ConcaveRadius
	require.Equal(t, decad.ScalarMeasured, radius.Outcome, "the L's inside corner is a concave edge")
	require.NotNil(t, radius.Minimum)
	require.Zero(t, radius.Minimum.Value.Base(), "every edge is sharp")
	require.Zero(t, radius.Minimum.Bound.Base())
	// A shelled cup has concave edges where the floor meets the walls.
	cupDoc := decad.New()
	block := blockIn(t, cupDoc, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 20))
	_, err = decad.Shell(t.Context(), block, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1))
	require.NoError(t, err)
	report, err = cupDoc.Verify(t.Context(), decad.WithConcaveRadius())
	require.NoError(t, err)
	require.Equal(t, decad.ScalarMeasured, report.Bodies[0].ConcaveRadius.Outcome)
}
