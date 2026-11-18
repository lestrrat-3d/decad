package booleaneval_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ratIv is the point interval at the float v.
func ratIv(v float64) proofbound.RatInterval {
	return proofbound.PointInterval(new(big.Rat).SetFloat64(v))
}

// ivVec is the point interval vector at v.
func ivVec(v r3.Vec) proofbound.IvVec3 {
	return proofbound.IvVec3{ratIv(v.X), ratIv(v.Y), ratIv(v.Z)}
}

// zRevolved is the record of a revolve about the world Z axis whose meridian
// runs from (z0, r0) to (z1, r1) in axial and radial coordinates.
func zRevolved(z0, r0, z1, r1 float64) *surfacenormal.Revolved {
	return &surfacenormal.Revolved{
		Origin: ivVec(r3.Vec{}),
		Basis:  [3]proofbound.IvVec3{ivVec(r3.Vec{X: 1}), ivVec(r3.Vec{Y: 1}), ivVec(r3.Vec{Z: 1})},
		Ends:   [][2][2]proofbound.RatInterval{{{ratIv(z0), ratIv(r0)}, {ratIv(z1), ratIv(r1)}}},
		Valid:  true,
	}
}

// zPlane is a Plane tag at level z with U = +X and V = +Y.
func zPlane(t *testing.T, z float64) surfacegeom.Plane {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return surfacegeom.Plane{Frame: f}
}

// TestRetainedPlaneOfDenotedRevolve checks which retained faces name their
// plane to the slab (agent-docs/faceted-source-retention-design.md §11): a
// Plane tag with no denoted patch, and a denoted revolve patch whose tag is
// exactly the plane it sweeps. A tag one ulp off the record's level, a record
// whose meridian is not perpendicular to the axis, a cap record, an invalid
// record, a tag that states a normal departure and a face that is not retained
// name none. Shown to fail with the denoted check removed (every denoted face
// named its tag) and with every denoted face refused (the exact one read false).
func TestRetainedPlaneOfDenotedRevolve(t *testing.T) {
	t.Parallel()
	const z = -3.8
	exact := zRevolved(z, 3.25, z, 20)
	face := func(mod func(f *topo.Face)) *topo.Face {
		f := &topo.Face{Surface: zPlane(t, z), RetainedTag: true, Denoted: exact}
		mod(f)
		return f
	}
	capRecord := *exact
	capRecord.Cap = true
	invalid := *exact
	invalid.Valid = false
	for _, tc := range []struct {
		name string
		face *topo.Face
		want bool
	}{
		{"a tag exactly its record's plane", face(func(*topo.Face) {}), true},
		{"a retained plane with no record", face(func(f *topo.Face) { f.Denoted = nil }), true},
		{"a tag one ulp off its record's level", face(func(f *topo.Face) {
			f.Surface = zPlane(t, math.Nextafter(z, 0))
		}), false},
		{"a record whose meridian is not perpendicular to the axis", face(func(f *topo.Face) {
			f.Denoted = zRevolved(z, 3.25, z+1, 20)
		}), false},
		{"a cap record", face(func(f *topo.Face) { f.Denoted = &capRecord }), false},
		{"an invalid record", face(func(f *topo.Face) { f.Denoted = &invalid }), false},
		{"a tag that states a normal departure", face(func(f *topo.Face) { f.NormalBound = 1e-9 }), false},
		{"a face that is not retained", face(func(f *topo.Face) { f.RetainedTag = false }), false},
		{"a cylinder tag on a planar record", face(func(f *topo.Face) {
			f.Surface = surfacegeom.Cylinder{Axis: r3.Vec{Z: 1}}
		}), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			m := &topo.Mesh{Source: []*topo.Face{tc.face, tc.face}}
			planeOf := booleaneval.RetainedPlaneOf(m)
			for facet := range 2 {
				frame, delta, ok := planeOf(facet)
				require.Equal(t, tc.want, ok, "facet %d", facet)
				if !tc.want {
					continue
				}
				require.Zero(t, delta)
				require.Equal(t, tc.face.Surface.(surfacegeom.Plane).Frame, frame)
			}
			_, _, ok := planeOf(2)
			require.False(t, ok, "a facet past the source record names no plane")
		})
	}
}
