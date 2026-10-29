package trimshell_test

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facemesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/trimshell"
	"github.com/lestrrat-3d/decad/internal/trimtest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func voidVertices(t *testing.T, b *topo.Body) []*topo.Vertex {
	t.Helper()
	require.Len(t, b.Lumps, 1)
	require.Len(t, b.Lumps[0].Shells, 2)
	void := b.Lumps[0].Shells[1]
	require.True(t, void.Void)
	seen := map[*topo.Vertex]struct{}{}
	var out []*topo.Vertex
	for _, f := range void.Faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				for _, v := range []*topo.Vertex{ce.Edge.Start, ce.Edge.End} {
					if _, ok := seen[v]; !ok {
						seen[v] = struct{}{}
						out = append(out, v)
					}
				}
			}
		}
	}
	return out
}

func TestClosedBevelledPlate(t *testing.T) {
	b, fp := trimtest.BevelledPlate(t)
	got, matched, err := trimshell.Closed(t.Context(), trimtest.Env(), b, fp, trimshell.Call{Inward: true, T: 1})
	require.NoError(t, err)
	require.True(t, matched)

	s2 := math.Sqrt2
	want := []r3.Vec{}
	for _, x := range []float64{1, 39} {
		for _, p := range [][2]float64{{1, 1}, {19, 1}, {19, 19}, {1 + s2, 19}, {1, 19 - s2}} {
			want = append(want, r3.NewVec(x, p[0], p[1]))
		}
	}
	verts := voidVertices(t, got)
	require.Len(t, verts, len(want))
	for _, w := range want {
		best := math.Inf(1)
		var bound float64
		for _, v := range verts {
			if d := v.Position.Sub(w).Len(); d < best {
				best, bound = d, v.Bound.Base()
			}
		}
		// The closed form is rounded to float64 here, so allow its own rounding on top of the vertex bound.
		require.LessOrEqual(t, best, bound+1e-13, "void vertex near %v", w)
	}

	void := got.Lumps[0].Shells[1]
	require.Len(t, void.Faces, 7)
	bevelled := 0
	for _, f := range void.Faces {
		require.True(t, f.RetainedTag)
		_, isPlane := f.Surface.(surfacegeom.Plane)
		require.True(t, isPlane)
		require.NotEmpty(t, f.Origins)
		if f.NormalBound > 0 {
			bevelled++
		}
	}
	require.Equal(t, 1, bevelled, "only the bevel's offset plane is not a float plane")

	require.InDelta(t, 15920-38*323, got.Volume.Value.Base(), got.Volume.Bound.Base()+1e-9)
	require.Greater(t, got.Volume.Bound.Base(), 0.0)
}

func TestClosedDropsBevelAtThickness4(t *testing.T) {
	b, fp := trimtest.BevelledPlate(t)
	_, matched, err := trimshell.Closed(t.Context(), trimtest.Env(), b, fp, trimshell.Call{Inward: true, T: 4})
	require.True(t, matched)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "(trim TG6)")
}

func TestClosedDeclines(t *testing.T) {
	b, fp := trimtest.BevelledPlate(t)
	got, matched, err := trimshell.Closed(t.Context(), trimtest.Env(), b, fp, trimshell.Call{Inward: false, T: 1})
	require.NoError(t, err)
	require.False(t, matched)
	require.Nil(t, got)

	l, lfp := trimtest.LSection(t)
	got, matched, err = trimshell.Closed(t.Context(), trimtest.Env(), l, lfp, trimshell.Call{Inward: true, T: 1})
	require.NoError(t, err)
	require.False(t, matched, "a concave edge declines")
	require.Nil(t, got)

	p, pfp := trimtest.Pyramid(t)
	_, matched, err = trimshell.Closed(t.Context(), trimtest.Env(), p, pfp, trimshell.Call{Inward: true, T: 1})
	require.NoError(t, err)
	require.False(t, matched, "a receiver that does not lift declines")
}

func TestClosedRefusesInexactThickness(t *testing.T) {
	b, fp := trimtest.BevelledPlate(t)
	for _, c := range []trimshell.Call{{Inward: true, T: 1, TDelta: 1e-9}, {Inward: true, T: math.NaN()}} {
		_, matched, err := trimshell.Closed(t.Context(), trimtest.Env(), b, fp, c)
		require.True(t, matched)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "(trim TG7)")
	}
}

func TestOrientationGateRefusesFlippedLoop(t *testing.T) {
	n := [3]*big.Rat{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 1)}
	ccw := facemesh.Loop{
		Points: []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(4, 4, 0), r3.NewVec(0, 4, 0)},
		Bounds: []float64{0, 0, 0, 0},
	}
	require.NoError(t, trimshell.CheckOrientation(3, n, ccw))

	cw := facemesh.Loop{Points: slices.Clone(ccw.Points), Bounds: ccw.Bounds}
	slices.Reverse(cw.Points)
	err := trimshell.CheckOrientation(3, n, cw)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "drops face 3")
	require.ErrorContains(t, err, "(trim TG4)")

	// An area smaller than perimeter times the corner bound is not positive beyond the bound.
	thin := facemesh.Loop{Points: ccw.Points, Bounds: []float64{3, 3, 3, 3}}
	require.ErrorContains(t, trimshell.CheckOrientation(3, n, thin), "(trim TG4)")
}
