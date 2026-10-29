package trimrebuild_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/trimrebuild"
	"github.com/lestrrat-3d/decad/internal/trimtest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cubeSplice returns the splice of an inward-facing cube of the given side at lo, whose vertex indices start at base.
// Its facets wind into the cube (outward from the material that surrounds it).
func cubeSplice(t *testing.T, lo, side, bound float64, base int) trimrebuild.Splice {
	t.Helper()
	var s trimrebuild.Splice
	for i := range 8 {
		s.Verts = append(s.Verts, r3.NewVec(lo+side*float64(i&1), lo+side*float64(i>>1&1), lo+side*float64(i>>2&1)))
		s.Bounds = append(s.Bounds, bound)
	}
	// Corner indices of the six faces, each listed so that the first three corners give the face plane.
	faces := [][4]int{{0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}}
	for _, q := range faces {
		a, b, c := s.Verts[q[0]], s.Verts[q[1]], s.Verts[q[2]]
		// The group's plane normal points out of the cube; the facets wind the opposite way.
		out := b.Sub(a).Cross(c.Sub(a))
		centre := r3.NewVec(lo+side/2, lo+side/2, lo+side/2)
		if out.Dot(a.Sub(centre)) < 0 {
			q[1], q[3] = q[3], q[1]
		}
		g := trimtest.PlaneGroup(t, "cavity", s.Verts[q[0]], s.Verts[q[1]], s.Verts[q[2]])
		g.Reversed = true
		i0, i1, i2, i3 := base+q[0], base+q[1], base+q[2], base+q[3]
		// Reverse the outward winding so the void's facets face into the cavity.
		s.Faces = append(s.Faces, trimrebuild.NewFace{Group: g, Tris: [][3]int{{i0, i2, i1}, {i0, i3, i2}}})
	}
	return s
}

func outerCube(t *testing.T) (facetedrecord.Payload, int) {
	t.Helper()
	_, fp := trimtest.Prism(t, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, 0, 10)
	return fp, len(fp.Verts)
}

func TestBuildSplicesInnerCubeVoid(t *testing.T) {
	fp, base := outerCube(t)
	keep := make([]int, len(fp.Tris))
	for i := range keep {
		keep[i] = i
	}
	s := cubeSplice(t, 1, 8, 1e-6, base)
	s.Keep = keep
	body, err := trimrebuild.Build(t.Context(), trimtest.Env(), fp, s)
	require.NoError(t, err)
	require.Len(t, body.Lumps, 1)
	require.Len(t, body.Lumps[0].Shells, 2)
	require.False(t, body.Lumps[0].Shells[0].Void)
	require.True(t, body.Lumps[0].Shells[1].Void)

	got := body.Payload.(facetedrecord.Payload)
	require.GreaterOrEqual(t, got.VolSymDiff, fp.VolSymDiff)
	require.Greater(t, got.VolSymDiff, fp.VolSymDiff, "the new vertices' bounds are charged")
	require.Greater(t, got.AreaSlack, fp.AreaSlack)

	vol := body.Volume.Value.Base()
	require.InDelta(t, 1000-512, vol, body.Volume.Bound.Base()+1e-9)
	require.Greater(t, body.Volume.Bound.Base(), 0.0)
	require.Less(t, math.Abs(vol-(1000-512)), 1e-6)
}

func TestBuildRefusesFoldedVoid(t *testing.T) {
	fp, base := outerCube(t)
	keep := make([]int, len(fp.Tris))
	for i := range keep {
		keep[i] = i
	}
	// The inner cube spans 5..13, so it pokes through the outer cube's x=10, y=10 and z=10 faces.
	s := cubeSplice(t, 5, 8, 0, base)
	s.Keep = keep
	_, err := trimrebuild.Build(t.Context(), trimtest.Env(), fp, s)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "(trim TG5)")
	require.ErrorContains(t, err, "folds at facets")
}

func TestBuildRefusesMismatchedBounds(t *testing.T) {
	fp, _ := outerCube(t)
	_, err := trimrebuild.Build(t.Context(), trimtest.Env(), fp, trimrebuild.Splice{Verts: []r3.Vec{{}}})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
