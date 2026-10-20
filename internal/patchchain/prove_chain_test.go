package patchchain_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// squareChain is a closed four-line chain through the given corners.
func squareChain(corners [4]r3.Vec) []*topo.Edge {
	var verts [4]*topo.Vertex
	for i, c := range corners {
		verts[i] = &topo.Vertex{Position: c}
	}
	edges := make([]*topo.Edge, 4)
	for i := range edges {
		edges[i] = &topo.Edge{Curve: surfacegeom.Line3{}, Start: verts[i], End: verts[(i+1)%4]}
	}
	return edges
}

func flatSquare() [4]r3.Vec {
	return [4]r3.Vec{{}, {X: 4}, {X: 4, Y: 4}, {Y: 4}}
}

func TestProveChainPlane(t *testing.T) {
	t.Parallel()

	t.Run("exact planar chain", func(t *testing.T) {
		t.Parallel()
		level, bound, held, err := patchchain.ProveChainPlane(squareChain(flatSquare()))
		require.NoError(t, err)
		require.Equal(t, denotation.LevelToken{}, level)
		require.Zero(t, bound)
		require.False(t, held)
	})

	refusals := map[string]func() []*topo.Edge{
		"corner off the plane": func() []*topo.Edge {
			c := flatSquare()
			c[2].Z = 1
			return squareChain(c)
		},
		"bounded vertex": func() []*topo.Edge {
			edges := squareChain(flatSquare())
			edges[0].Start.Bound = units.Millimeters(1e-3)
			return edges
		},
		"bounded line length": func() []*topo.Edge {
			edges := squareChain(flatSquare())
			edges[1].LengthBound = 1e-9
			return edges
		},
		"unbounded line length": func() []*topo.Edge {
			edges := squareChain(flatSquare())
			edges[1].LengthUnbounded = true
			return edges
		},
		"unbounded circle length": func() []*topo.Edge {
			v := &topo.Vertex{Position: r3.Vec{X: 1}}
			return []*topo.Edge{{Curve: surfacegeom.Circle3{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(1)},
				Start: v, End: v, LengthUnbounded: true}}
		},
		"circle off the chain's plane": func() []*topo.Edge {
			v := &topo.Vertex{Position: r3.Vec{X: 1}}
			return []*topo.Edge{{Curve: surfacegeom.Circle3{Center: r3.Vec{Z: 1}, Axis: r3.Vec{X: 1},
				Radius: units.Millimeters(1)}, Start: v, End: v}}
		},
	}
	for name, build := range refusals {
		_, _, held, err := patchchain.ProveChainPlane(build())
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.False(t, held, name)
	}

	t.Run("lone closed circle is admitted despite its length bound", func(t *testing.T) {
		t.Parallel()
		v := &topo.Vertex{Position: r3.Vec{X: 1}}
		circle := &topo.Edge{Curve: surfacegeom.Circle3{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(1)},
			Start: v, End: v, LengthBound: 1e-9}
		_, _, held, err := patchchain.ProveChainPlane([]*topo.Edge{circle})
		require.NoError(t, err)
		require.False(t, held)
	})

	t.Run("lone arc edge pair is admitted from its carrier plane", func(t *testing.T) {
		t.Parallel()
		a, b := &topo.Vertex{Position: r3.Vec{X: 1}}, &topo.Vertex{Position: r3.Vec{X: -1}}
		carrier := surfacegeom.Arc3{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(1)}
		edges := []*topo.Edge{
			{Curve: carrier, Start: a, End: b, LengthBound: 1e-9},
			{Curve: carrier, Start: b, End: a, LengthBound: 1e-9},
		}
		_, _, held, err := patchchain.ProveChainPlane(edges)
		require.NoError(t, err)
		require.False(t, held)
	})

	t.Run("shared level arm admits a bounded chain", func(t *testing.T) {
		t.Parallel()
		level := denotation.LevelToken{ID: 7}
		edges := squareChain(flatSquare())
		edges[0].Start.Bound = units.Millimeters(2e-3)
		for _, e := range edges {
			e.Level = level
			e.Start.Level, e.End.Level = level, level
		}
		got, bound, held, err := patchchain.ProveChainPlane(edges)
		require.NoError(t, err)
		require.True(t, held)
		require.Equal(t, level, got)
		require.InDelta(t, 2e-3, bound, 1e-15)
	})

	t.Run("level arm needs every edge on the same level", func(t *testing.T) {
		t.Parallel()
		level := denotation.LevelToken{ID: 7}
		edges := squareChain(flatSquare())
		edges[0].Start.Bound = units.Millimeters(2e-3)
		for _, e := range edges {
			e.Start.Level, e.End.Level = level, level
		}
		_, _, held, err := patchchain.ProveChainPlane(edges)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.False(t, held)
	})
}
