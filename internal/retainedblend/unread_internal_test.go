package retainedblend

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
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
			"an edge between two retained cylinders with parallel axes"},
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
			"a face whose tag is not proven to be exactly the surface its revolve record denotes (retained-edge-blend SR1)"},
		{"three faces", []tagFace{{}, {}, {}},
			"the edge borders 3 faces, and a class reads an edge between exactly two"},
		{"unmapped kind", []tagFace{plain(surfacegeom.SurfaceKind(99)), plain(surfacegeom.KindPlane)},
			"an edge between a retained surface of kind 99 and a retained plane"},
		{"one face", []tagFace{plain(surfacegeom.KindPlane)},
			"the edge borders 1 face, and a class reads an edge between exactly two"},
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

// exactSphereRecord is the revolve record of a sphere of radius 2 centred at
// the origin about the z axis, every leaf a point.
func exactSphereRecord(t *testing.T) *surfacenormal.Revolved {
	t.Helper()
	pt := func(n int64) proofbound.RatInterval { return proofbound.Interval(big.NewRat(n, 1), big.NewRat(n, 1)) }
	vec := func(x, y, z int64) proofbound.IvVec3 { return proofbound.IvVec3{pt(x), pt(y), pt(z)} }
	return &surfacenormal.Revolved{
		Origin: vec(0, 0, 0), Basis: [3]proofbound.IvVec3{vec(1, 0, 0), vec(0, 1, 0), vec(0, 0, 1)},
		Circular: true, Centre: [2]proofbound.RatInterval{pt(0), pt(0)}, Radius: pt(2), Valid: true,
	}
}

func TestUnreadFaceTags(t *testing.T) {
	t.Parallel()
	denoted := &surfacenormal.Revolved{}
	plane := retainedFace(surfacegeom.Plane{})
	edge := func(f *topo.Face) *topo.Edge { return &topo.Edge{Faces: []*topo.Face{f, plane}} }

	t.Run("a face with a nil surface has no retained surface", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, "no retained blend class reads selected edge[0]: a face with no retained surface (faceted)",
			Unread([]*topo.Edge{edge(&topo.Face{RetainedTag: true})}))
	})
	t.Run("a sphere equal to its revolve record is named by its kinds", func(t *testing.T) {
		t.Parallel()
		f := retainedFace(surfacegeom.Sphere{Radius: units.Millimeters(2)})
		f.Denoted = exactSphereRecord(t)
		require.Equal(t,
			"no retained blend class reads selected edge[0]: an edge between a retained sphere and a retained plane",
			Unread([]*topo.Edge{edge(f)}))
	})
	t.Run("a sphere whose record is not exact is named as such", func(t *testing.T) {
		t.Parallel()
		f := retainedFace(surfacegeom.Sphere{Radius: units.Millimeters(2)})
		f.Denoted = denoted
		require.Contains(t, Unread([]*topo.Edge{edge(f)}),
			"a face whose tag is not proven to be exactly the surface its revolve record denotes")
	})
	t.Run("a plane or cylinder whose record is not proven is named as such", func(t *testing.T) {
		t.Parallel()
		for _, surf := range []surfacegeom.Surface{surfacegeom.Plane{}, surfacegeom.Cylinder{}} {
			f := retainedFace(surf)
			f.Denoted = denoted // not Valid, so the tag is not proven to be the record
			require.Contains(t, Unread([]*topo.Edge{edge(f)}),
				"a face whose tag is not proven to be exactly the surface its revolve record denotes")
		}
	})
	t.Run("a nil edge is named", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, "no retained blend class reads selected edge[0]: it is not an edge", Unread([]*topo.Edge{nil}))
	})
	t.Run("a cone with a revolve record is named a cone", func(t *testing.T) {
		t.Parallel()
		f := retainedFace(surfacegeom.Cone{})
		f.Denoted = denoted
		require.Equal(t,
			"no retained blend class reads selected edge[0]: an edge on a retained cone, which no class reads (retained-edge-blend SR1)",
			Unread([]*topo.Edge{edge(f)}))
	})
	t.Run("a torus with a revolve record is named a torus", func(t *testing.T) {
		t.Parallel()
		f := retainedFace(surfacegeom.Torus{})
		f.Denoted = denoted
		require.Contains(t, Unread([]*topo.Edge{edge(f)}), "an edge on a retained torus")
	})
}
