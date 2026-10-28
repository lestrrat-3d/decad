package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/retained-edge-blend-design.md §4.8's tests: Fillet of straight mesh edges that meet at
// corners of three perpendicular faces, where three filleted edges end in a sphere octant.

func TestFilletOfThreeEdgesAtABoxCornerEndsInASphere(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	corner := r3.NewVec(0, 0, 10)
	three := decad.Edges(decad.EndpointAt(corner), decad.ParallelTo(x)).
		Or(decad.EndpointAt(corner), decad.ParallelTo(y)).Or(decad.EndpointAt(corner), decad.ParallelTo(z))
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	out, err := decad.Fillet(t.Context(), box, three, units.Millimeters(2))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	// Each edge loses (L − r) r²(1 − π/4) beyond the corner cube, which loses r³ less the sphere octant πr³/6.
	k := 1 - math.Pi/4
	want := 4000 - (18*4*k*2 + 8*4*k + 8 - 8*math.Pi/6)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 0.1)
	require.Len(t, out.Faces(), 10, "six faces, a strip for each edge and the sphere")
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(0.3, 0.3, 9.7):   decad.PointOutside,
		r3.NewVec(1.2, 1.2, 8.8):   decad.PointInside,
		r3.NewVec(10, 10, 5):       decad.PointInside,
		r3.NewVec(10, 0.2, 9.8):    decad.PointOutside,
		r3.NewVec(0.2, 10, 9.8):    decad.PointOutside,
		r3.NewVec(0.2, 0.2, 5):     decad.PointOutside,
		r3.NewVec(19.5, 19.5, 9.5): decad.PointInside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestFilletOfEveryEdgeOfAMeshBoxRoundsAllEightCorners(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	out, err := decad.Fillet(t.Context(), box, decad.Edges(), units.Millimeters(2))
	require.NoError(t, err)
	// Twelve edges of length L − 2r (four each of 20, 20 and 10) and eight corner cubes.
	k := 1 - math.Pi/4
	want := 4000 - 152*4*k - 8*(8-8*math.Pi/6)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 0.5)
	require.Len(t, out.Faces(), 26, "six faces, twelve strips and eight spheres")
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestFilletOfEdgesAtACornerRefusesAnEdgeShorterThanTwoRadii(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 3))
	_, err := decad.Fillet(t.Context(), box, decad.Edges(), units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}
