package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/subd-design.md's public-surface tests for SubDMesh.

// cubeCage is the cube from -1 to 1 on each axis, six quads wound counter-clockwise seen from outside.
func cubeCage() decad.SubDCage {
	return decad.SubDCage{
		Vertices: []r3.Vec{
			{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1},
			{X: -1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: -1, Y: 1, Z: 1},
		},
		Faces: [][]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}},
	}
}

func TestSubDMeshOfACubeIsTheCatmullClarkSurface(t *testing.T) {
	t.Parallel()
	m, err := decad.SubDMesh(t.Context(), cubeCage(), 1)
	require.NoError(t, err)
	// One round on the cube, by the rules worked by hand: the 8 corners move to (±5/9)³, the
	// 6 face points sit at the face centres, and the 12 edge points are three quarters of the
	// way out from the axis of the edge's middle, e.g. (3/4, 3/4, 0) for the edge along z.
	var want []r3.Vec
	for _, x := range []float64{-5.0 / 9, 5.0 / 9} {
		for _, y := range []float64{-5.0 / 9, 5.0 / 9} {
			for _, z := range []float64{-5.0 / 9, 5.0 / 9} {
				want = append(want, r3.Vec{X: x, Y: y, Z: z})
			}
		}
	}
	for _, s := range []float64{-1, 1} {
		want = append(want, r3.Vec{X: s}, r3.Vec{Y: s}, r3.Vec{Z: s})
	}
	for _, a := range []float64{-0.75, 0.75} {
		for _, b := range []float64{-0.75, 0.75} {
			want = append(want, r3.Vec{X: a, Y: b}, r3.Vec{X: a, Z: b}, r3.Vec{Y: a, Z: b})
		}
	}
	got := m.Vertices()
	require.Len(t, got, 26, "8 + 12 + 6 points")
	for _, w := range want {
		found := 0
		for _, g := range got {
			if g.Equal(w, 1e-12) {
				found++
			}
		}
		require.Equal(t, 1, found, "the point %v appears once", w)
	}
	report := m.Report()
	require.True(t, report.Closed)
	require.False(t, report.Inverted, "wound outward")
	require.Equal(t, 48, report.Triangles, "24 quads of two triangles")
	volume, err := m.Volume()
	require.NoError(t, err)
	// Worked out by an independent calculation: the level-1 surface of the 8 mm³ cube holds 10/3.
	require.InDelta(t, 10.0/3, volume.Value.Base(), 1e-9)
}

func TestSubDMeshRefinesToTheLimitFromInside(t *testing.T) {
	t.Parallel()
	counts := map[int]struct {
		vertices, triangles int
		volume              float64
	}{
		2: {98, 192, 2.800937647355113},
		3: {386, 768, 2.664624789679461},
	}
	prev := math.Inf(1)
	for _, level := range []int{1, 2, 3} {
		m, err := decad.SubDMesh(t.Context(), cubeCage(), level)
		require.NoError(t, err)
		require.True(t, m.Report().Closed, "level %d", level)
		require.False(t, m.Report().Inverted)
		volume, err := m.Volume()
		require.NoError(t, err)
		require.Less(t, volume.Value.Base(), prev, "each round pulls the surface in")
		prev = volume.Value.Base()
		box := m.Bounds()
		// The first round leaves the face points on the cage's faces; the later rounds pull them in.
		if level == 1 {
			require.LessOrEqual(t, box.Max.X, 1.0, "the surface lies inside the cage")
		} else {
			require.Less(t, box.Max.X, 1.0, "the surface lies inside the cage")
		}
		require.True(t, box.Min.Equal(box.Max.Scale(-1), 1e-12), "and keeps the cage's symmetry")
		if want, ok := counts[level]; ok {
			require.Len(t, m.Vertices(), want.vertices)
			require.Len(t, m.Triangles(), want.triangles)
			require.InDelta(t, want.volume, volume.Value.Base(), 1e-9)
		}
	}
}

func TestSubDMeshKeepsAnOpenCagesBoundary(t *testing.T) {
	t.Parallel()
	square := decad.SubDCage{
		Vertices: []r3.Vec{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}, {X: 0, Y: 1}},
		Faces:    [][]int{{0, 1, 2, 3}},
	}
	m, err := decad.SubDMesh(t.Context(), square, 1)
	require.NoError(t, err)
	// Boundary corners move to (6P + A + B)/8, boundary edges keep their midpoints, and the face
	// point is the centre.
	want := []r3.Vec{
		{X: 1.0 / 8, Y: 1.0 / 8}, {X: 7.0 / 8, Y: 1.0 / 8}, {X: 7.0 / 8, Y: 7.0 / 8}, {X: 1.0 / 8, Y: 7.0 / 8},
		{X: 0.5, Y: 0.5}, {X: 0.5}, {X: 1, Y: 0.5}, {X: 0.5, Y: 1}, {Y: 0.5},
	}
	got := m.Vertices()
	require.Len(t, got, len(want))
	for _, w := range want {
		found := false
		for _, g := range got {
			found = found || g.Equal(w, 1e-12)
		}
		require.True(t, found, "the point %v", w)
	}
	require.False(t, m.Report().Closed, "an open cage gives an open mesh")
	require.Equal(t, 8, m.Report().Triangles)
}

func TestSubDMeshRefusals(t *testing.T) {
	t.Parallel()
	with := func(edit func(*decad.SubDCage)) error {
		cage := cubeCage()
		edit(&cage)
		_, err := decad.SubDMesh(t.Context(), cage, 1)
		return err
	}
	require.NoError(t, with(func(*decad.SubDCage) {}))
	for name, edit := range map[string]func(*decad.SubDCage){
		"no faces":          func(c *decad.SubDCage) { c.Faces = nil },
		"a two-sided face":  func(c *decad.SubDCage) { c.Faces[0] = []int{0, 1} },
		"a missing vertex":  func(c *decad.SubDCage) { c.Faces[0] = []int{0, 3, 2, 99} },
		"a repeated vertex": func(c *decad.SubDCage) { c.Faces[0] = []int{0, 3, 3, 1} },
		"two faces agree on a direction": func(c *decad.SubDCage) {
			c.Faces[1] = []int{7, 6, 5, 4} // wound inward: shares edges with its neighbours the same way
		},
		"an edge in three faces": func(c *decad.SubDCage) { c.Faces = append(c.Faces, []int{0, 1, 2}) },
	} {
		require.ErrorIs(t, with(edit), decad.ErrDegenerate, name)
	}
	require.ErrorIs(t, with(func(c *decad.SubDCage) { c.Vertices[3].X = math.NaN() }), decad.ErrNotFinite)
	for _, levels := range []int{0, -1, 9} {
		_, err := decad.SubDMesh(t.Context(), cubeCage(), levels)
		require.ErrorIs(t, err, decad.ErrDegenerate, "levels %d", levels)
	}
	_, err := decad.SubDMesh(nil, cubeCage(), 1) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// Two squares that meet only at a corner leave that corner on four boundary edges.
	bowtie := decad.SubDCage{
		Vertices: []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}, {X: -1}, {X: -1, Y: -1}, {Y: -1}},
		Faces:    [][]int{{0, 1, 2, 3}, {0, 6, 5, 4}},
	}
	_, err = decad.SubDMesh(t.Context(), bowtie, 1)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
