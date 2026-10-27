package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B7's public-surface test: a faceted operand and an
// extruded one.

func TestOrthogonalBooleanTakesAnExtrudedPartnerForAMeshSolid(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stl := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	extruded := boxBody(t, doc, 10, 0, 20, 10, 10) // x 10 to 20, y 0 to 10, z 0 to 10: touches the block at a face
	out, err := decad.Union(t.Context(), stl, extruded)
	require.NoError(t, err)
	requireVolume(t, out, 2000)
	require.Len(t, out.Faces(), 6)
	require.Len(t, doc.Bodies(), 1)
}
