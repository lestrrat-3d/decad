package modifyselect_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func vertexAt(x, y, z float64) *topo.Vertex { return &topo.Vertex{Position: r3.Vec{X: x, Y: y, Z: z}} }

func TestEdgeContext(t *testing.T) {
	t.Parallel()

	shared := vertexAt(1, 2, 3)
	tests := []struct {
		name string
		edge *topo.Edge
		want string
	}{
		{"nil edge", nil, `selected edge[4]`},
		{"no start vertex", &topo.Edge{End: vertexAt(1, 0, 0)}, `selected edge[4]`},
		{"no end vertex", &topo.Edge{Start: vertexAt(1, 0, 0)}, `selected edge[4]`},
		{"straight", &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexAt(0, 0, 0), End: vertexAt(1.5, 0, -2)},
			`selected edge[4] from (0,0,0) to (1.5,0,-2)`},
		{"closed circle", &topo.Edge{Curve: surfacegeom.Circle3{Center: r3.Vec{X: 5}, Radius: units.Millimeters(2)},
			Start: shared, End: shared},
			`selected edge[4] closed circle through (1,2,3), centre (5,0,0), radius 2 mm`},
		{"ellipse arc", &topo.Edge{Curve: surfacegeom.Ellipse3{Center: r3.Vec{Z: 1},
			SemiMajor: units.Millimeters(3), SemiMinor: units.Millimeters(2)},
			Start: vertexAt(0, 0, 0), End: vertexAt(1, 1, 1)},
			`selected edge[4] ellipse arc from (0,0,0) to (1,1,1), centre (0,0,1), semi-axes 3 mm and 2 mm`},
		{"closed ellipse", &topo.Edge{Curve: surfacegeom.Ellipse3{Center: r3.Vec{Z: 1},
			SemiMajor: units.Millimeters(3), SemiMinor: units.Millimeters(2)}, Start: shared, End: shared},
			`selected edge[4] closed ellipse through (1,2,3), centre (0,0,1), semi-axes 3 mm and 2 mm`},
		{"closed through one vertex", &topo.Edge{Curve: surfacegeom.Line3{}, Start: shared, End: shared},
			`selected edge[4] closed through (1,2,3)`},
		{"distinct vertices at one position", &topo.Edge{Curve: surfacegeom.Line3{},
			Start: vertexAt(1, 2, 3), End: vertexAt(1, 2, 3)},
			`selected edge[4] from (1,2,3) to (1,2,3)`},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, modifyselect.EdgeContext(4, tc.edge))
		})
	}
}

func TestEdgesContext(t *testing.T) {
	t.Parallel()

	edges := []*topo.Edge{
		{Curve: surfacegeom.Line3{}, Start: vertexAt(0, 0, 0), End: vertexAt(1, 0, 0)},
		nil,
		{Curve: surfacegeom.Line3{}, Start: vertexAt(0, 1, 0), End: vertexAt(0, 2, 0)},
	}
	require.Equal(t, `selected edge[0] from (0,0,0) to (1,0,0); selected edge[1]; selected edge[2] from (0,1,0) to (0,2,0)`,
		modifyselect.EdgesContext(edges))
	require.Empty(t, modifyselect.EdgesContext(nil))
}
