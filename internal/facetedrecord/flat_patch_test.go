package facetedrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// flatPatchBound and flatPatchLengthBound are the published bounds of every vertex and every edge length of
// flatPatch.
const flatPatchBound, flatPatchLengthBound = 1e-9, 1e-8

// flatPatch is a 4 × 2 rectangle of a band's flat patch on the tag z = 0, one corner lifted off the tag by lift,
// each vertex bound by flatPatchBound and each edge's length by flatPatchLengthBound, with the given tilt.
func flatPatch(t *testing.T, lift, tilt float64) *topo.Face {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	corners := []r3.Vec{{}, {X: 4}, {X: 4, Y: 2, Z: lift}, {Y: 2}}
	vs := make([]*topo.Vertex, len(corners))
	for i, c := range corners {
		vs[i] = &topo.Vertex{Position: c, Bound: units.Millimeters(flatPatchBound)}
	}
	loop := &topo.Loop{Outer: true}
	for i := range vs {
		a, b := vs[i], vs[(i+1)%len(vs)]
		e := &topo.Edge{Start: a, End: b, Length: b.Position.Sub(a.Position).Len(), LengthBound: flatPatchLengthBound,
			Curve: surfacegeom.Line3{}}
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: e, Forward: true})
	}
	return &topo.Face{Surface: surfacegeom.Plane{Frame: frame}, FlatPatch: true, NormalBound: tilt,
		Loops: []*topo.Loop{loop}}
}

func TestFlatPatchExtent(t *testing.T) {
	t.Parallel()
	const tilt, bound, lengthBound = 1e-6, flatPatchBound, flatPatchLengthBound
	perimeter := 12 + 4*lengthBound

	t.Run("the nearest corner plus the tilt over half the perimeter", func(t *testing.T) {
		t.Parallel()
		// Three corners lie on the tag; the lifted one does not set the extent.
		got, ok := facetedrecord.FlatPatchExtent(flatPatch(t, 1e-7, tilt))
		require.True(t, ok)
		want := bound + tilt*perimeter/2
		require.GreaterOrEqual(t, got, want)
		require.InDelta(t, want, got, 1e-15)
	})
	t.Run("a tilt past one reads as one", func(t *testing.T) {
		t.Parallel()
		got, ok := facetedrecord.FlatPatchExtent(flatPatch(t, 0, 1.5))
		require.True(t, ok)
		require.InDelta(t, bound+perimeter/2, got, 1e-12)
	})
	t.Run("a face the claim does not cover states none", func(t *testing.T) {
		t.Parallel()
		cases := map[string]func(f *topo.Face){
			"not a flat patch":   func(f *topo.Face) { f.FlatPatch = false },
			"no tilt":            func(f *topo.Face) { f.NormalBound = 0 },
			"an infinite tilt":   func(f *topo.Face) { f.NormalBound = math.Inf(1) },
			"a retained tag":     func(f *topo.Face) { f.RetainedTag = true },
			"an axial delta":     func(f *topo.Face) { f.HasAxialDelta = true },
			"a curved surface":   func(f *topo.Face) { f.Surface = surfacegeom.Cylinder{} },
			"two loops":          func(f *topo.Face) { f.Loops = append(f.Loops, f.Loops[0]) },
			"an unbounded edge":  func(f *topo.Face) { f.Loops[0].CoEdges[0].Edge.LengthUnbounded = true },
			"an infinite length": func(f *topo.Face) { f.Loops[0].CoEdges[1].Edge.LengthBound = math.Inf(1) },
			"a NaN bound": func(f *topo.Face) {
				f.Loops[0].CoEdges[2].Edge.Start.Bound = units.Millimeters(math.NaN())
			},
		}
		for name, mutate := range cases {
			f := flatPatch(t, 0, tilt)
			mutate(f)
			_, ok := facetedrecord.FlatPatchExtent(f)
			require.False(t, ok, name)
		}
	})
}

func TestSourceTagKeepsTiltedFlatPatch(t *testing.T) {
	t.Parallel()
	const tilt = 1e-6
	t.Run("a flat patch keeps its tag as an oblique plane", func(t *testing.T) {
		t.Parallel()
		f := flatPatch(t, 0, tilt)
		extent, ok := facetedrecord.FlatPatchExtent(f)
		require.True(t, ok)
		g, ok := facetedrecord.SourceTag(f, false)
		require.True(t, ok)
		require.True(t, g.Retained)
		require.Equal(t, tilt, g.NormalBound)
		require.Equal(t, extent, g.ExtentDelta)
	})
	t.Run("any other built face that states a tilt keeps none", func(t *testing.T) {
		t.Parallel()
		f := flatPatch(t, 0, tilt)
		f.FlatPatch = false
		g, ok := facetedrecord.SourceTag(f, false)
		require.False(t, ok)
		require.Nil(t, g.Surface)
	})
}
