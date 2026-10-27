package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B8's tests: a result of the orthogonal Boolean
// keeps exact planes, so Fillet, Chamfer and Shell take it.

// lBracket is the union of a 10 x 4 x 4 bar and a 4 x 10 x 4 bar sharing a corner: an L of 160 + 160 − 64 = 256 mm³.
func lBracket(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(4, 10, 4))
	out, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	requireVolume(t, out, 256)
	return out
}

func TestOrthogonalBooleanResultTakesAFilletAndAChamfer(t *testing.T) {
	t.Parallel()
	outerCorner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(0, 0, 0)))
	const r = 1.0
	filleted, err := decad.Fillet(t.Context(), lBracket(t), outerCorner, units.Millimeters(r))
	require.NoError(t, err)
	volume, err := filleted.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(256-(1-math.Pi/4)*r*r*4)), volume.Bound.Base()+1e-12)
	// The reflex corner at (4, 4) takes a concave fillet, which adds material.
	inner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(4, 4, 0)))
	filled, err := decad.Fillet(t.Context(), lBracket(t), inner, units.Millimeters(r))
	require.NoError(t, err)
	fv, err := filled.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(fv.Value.Base()-(256+(1-math.Pi/4)*r*r*4)), fv.Bound.Base()+1e-12)
	chamfered, err := decad.Chamfer(t.Context(), lBracket(t), outerCorner, units.Millimeters(r))
	require.NoError(t, err)
	cv, err := chamfered.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(cv.Value.Base()-(256-r*r/2*4)), cv.Bound.Base()+1e-12)
}

func TestOrthogonalBooleanResultTakesAShell(t *testing.T) {
	t.Parallel()
	out, err := decad.Shell(t.Context(), lBracket(t), decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The cavity is the L eroded by 1 mm on every side but the open top: two 8 x 2 and 2 x 8 legs less their shared
	// 2 x 2 corner, 28 mm² from the floor at 1 up through the top, 3 mm: 84 mm³, leaving 172 mm³ of wall.
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-172), volume.Bound.Base()+1e-9)
	require.Len(t, out.Shells(), 1)
}
