package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func retainedFace(s surfacegeom.Surface) *topo.Face {
	return &topo.Face{Surface: s, RetainedTag: true}
}

func cyl(axis r3.Vec) surfacegeom.Cylinder {
	return surfacegeom.Cylinder{Axis: axis, Radius: units.Millimeters(3)}
}

func TestUnreadReason(t *testing.T) {
	t.Parallel()
	plain := func(k surfacegeom.SurfaceKind) tagFace { return tagFace{kind: k, retained: true} }
	cylX := tagFace{kind: surfacegeom.KindCylinder, retained: true, cylinderDir: r3.NewVec(1, 0, 0)}
	cylZ := tagFace{kind: surfacegeom.KindCylinder, retained: true, cylinderDir: r3.NewVec(0, 0, 1)}

	tests := []struct {
		name  string
		faces []tagFace
		want  string
	}{
		{"cylinders with non-parallel axes", []tagFace{cylX, cylZ},
			"two retained cylinders whose axes are not parallel: the blend is a canal surface (retained-edge-blend SR1)"},
		{"cylinders with parallel axes", []tagFace{cylZ, cylZ},
			"an edge between a retained cylinder and a retained cylinder"},
		{"cone", []tagFace{plain(surfacegeom.KindPlane), plain(surfacegeom.KindCone)},
			"an edge on a retained cone, which no class reads (retained-edge-blend SR1)"},
		{"torus", []tagFace{plain(surfacegeom.KindTorus), cylZ},
			"an edge on a retained torus, which no class reads (retained-edge-blend SR1)"},
		{"sphere", []tagFace{plain(surfacegeom.KindSphere), plain(surfacegeom.KindPlane)},
			"an edge between a retained sphere and a retained plane"},
		{"plane and plane", []tagFace{plain(surfacegeom.KindPlane), plain(surfacegeom.KindPlane)},
			"an edge between a retained plane and a retained plane"},
		{"faceted face", []tagFace{plain(surfacegeom.KindPlane), {kind: surfacegeom.KindFaceted}},
			"a face with no retained surface (faceted)"},
		{"NURBS face", []tagFace{{kind: surfacegeom.KindNURBS}, plain(surfacegeom.KindPlane)},
			"a face with a free-form surface (NURBS), which has no retained analytic surface"},
		{"face without a retained tag", []tagFace{{kind: surfacegeom.KindPlane}, plain(surfacegeom.KindPlane)},
			"a face whose surface is not a retained tag"},
		{"tag with a stated departure", []tagFace{{kind: surfacegeom.KindPlane, retained: true, departs: true},
			plain(surfacegeom.KindPlane)},
			"a face whose retained tag states a departure from the true surface (retained-edge-blend SR1)"},
		{"tag that is not its revolve record", []tagFace{{kind: surfacegeom.KindCylinder, retained: true, notDenoted: true},
			plain(surfacegeom.KindPlane)},
			"a face whose tag is not exactly the surface its revolve record denotes (retained-edge-blend SR1)"},
		{"one face", []tagFace{plain(surfacegeom.KindPlane)},
			"the edge borders 1 faces, and a class reads an edge between exactly two"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, unreadReason(tc.faces))
		})
	}
}

func TestUnreadReadsTagsOfSelectedEdges(t *testing.T) {
	t.Parallel()
	crossing := &topo.Edge{Faces: []*topo.Face{
		retainedFace(cyl(r3.NewVec(1, 0, 0))), retainedFace(cyl(r3.NewVec(0, 0, 1)))}}
	faceted := &topo.Edge{Faces: []*topo.Face{
		retainedFace(surfacegeom.Plane{}), {Surface: surfacegeom.Faceted{}}}}

	require.Empty(t, Unread(nil))
	require.Equal(t,
		"no retained blend class reads selected edge[0]: two retained cylinders whose axes are not parallel: "+
			"the blend is a canal surface (retained-edge-blend SR1)",
		Unread([]*topo.Edge{crossing, faceted}))
	require.Equal(t,
		"no retained blend class reads selected edge[0]: a face with no retained surface (faceted)",
		Unread([]*topo.Edge{faceted}))
}
