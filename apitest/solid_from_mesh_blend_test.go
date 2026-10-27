package apitest_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md §3's tests for Fillet and Chamfer on a body
// SolidFromMesh made, whose faces square to a world axis keep their exact planes.

// blockCornerEdge is the edge along z at the (4, 3) corner of the 4 x 3 x 2 block.
func blockCornerEdge() decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(4, 3, 0)))
}

func TestSolidFromMeshFaceTagsAreExactPlanes(t *testing.T) {
	t.Parallel()
	_, body := solidFromBlock(t)
	for _, f := range body.Faces() {
		plane, ok := f.Surface().(decad.Plane)
		require.True(t, ok, "a face square to an axis keeps its plane, not %T", f.Surface())
		n, err := f.NormalAt(plane.Frame.Origin())
		require.NoError(t, err)
		require.InDelta(t, 1, math.Abs(n.Value.X)+math.Abs(n.Value.Y)+math.Abs(n.Value.Z), 1e-15, "an axis")
		require.InDelta(t, 0, n.Bound.Base(), 0, "with no departure from the exact plane")
	}
}

func TestSolidFromMeshFilletsAnEdge(t *testing.T) {
	t.Parallel()
	_, body := solidFromBlock(t)
	const r = 0.5
	out, err := decad.Fillet(t.Context(), body, blockCornerEdge(), units.Millimeters(r))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// A quarter-round fillet along the edge's 2 mm removes (1 − π/4)·r² of the section.
	want := 24 - (1-math.Pi/4)*r*r*2
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-12)
	require.Less(t, volume.Bound.Base(), 0.01, "the bound means something")
}

func TestSolidFromMeshChamfersAnEdge(t *testing.T) {
	t.Parallel()
	_, body := solidFromBlock(t)
	out, err := decad.Chamfer(t.Context(), body, blockCornerEdge(), units.Millimeters(0.5))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The chamfer cuts a right triangle of legs 0.5 along 2 mm: 0.25 mm³.
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-23.75), volume.Bound.Base()+1e-12)
}

func TestSolidFromMeshTiltedFacesStayFaceted(t *testing.T) {
	t.Parallel()
	// The unit corner tetrahedron has three faces square to an axis and one slanted, x + y + z = 1.
	facets := importTetrahedron()
	m, err := decad.ReadSTL(t.Context(), bytes.NewReader(importBinarySTL(facets[:])))
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	planes, faceted := 0, 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Plane); ok {
			planes++
		} else {
			faceted++
		}
	}
	require.Equal(t, 3, planes, "the three axis-square faces keep their planes")
	require.Equal(t, 1, faceted, "the slanted face has no exact float plane, so it stays Faceted")
	// The edge along z at the origin ends on the slanted face, which carries no plane to blend against.
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(0, 0, 0)))
	_, err = decad.Fillet(t.Context(), body, edge, units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
