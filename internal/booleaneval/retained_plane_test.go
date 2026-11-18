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
	return framePlane(t, r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
}

// framePlane is a Plane tag with the given frame.
func framePlane(t *testing.T, origin, u, v r3.Vec) surfacegeom.Plane {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return surfacegeom.Plane{Frame: f}
}

// withRecord returns the record of the planar ring at level, radii 3.25 to
// 20, changed by mod.
func withRecord(level float64, mod func(r *surfacenormal.Revolved)) *surfacenormal.Revolved {
	r := zRevolved(level, 3.25, level, 20)
	mod(r)
	return r
}

// TestRetainedPlaneOfDenotedRevolve checks which retained faces name their
// plane to the slab (agent-docs/faceted-source-retention-design.md §11): a
// Plane tag with no denoted patch, and a denoted revolve patch whose tag is
// exactly the plane it sweeps, including about an axis whose origin is not the
// world origin. Each refused record fails one condition of
// stitchflux.TagIsDenoted: the tag's level, its tilt, the meridian's ends, a
// second segment's level, the record's origin, a leaf that is no point, a basis
// that is not orthonormal, a cap, validity. A tag that states a normal departure
// and a face that is not retained name none either. Shown to fail with the
// denoted check removed (every denoted face named its tag), with every denoted
// face refused (the exact ones read false), and with each of TagIsDenoted's
// leaf, basis, end-level, tilt and origin checks removed in turn (that case
// read true).
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
		{"a tag at the record's level, tilted off its axis", face(func(f *topo.Face) {
			f.Surface = framePlane(t, r3.Vec{Z: z}, r3.Vec{X: 0.6, Z: 0.8}, r3.Vec{Y: 1})
		}), false},
		{"a second meridian segment at another level", face(func(f *topo.Face) {
			f.Denoted = withRecord(z, func(r *surfacenormal.Revolved) {
				r.Ends = append(r.Ends, [2][2]proofbound.RatInterval{{ratIv(z + 1), ratIv(20)}, {ratIv(z + 1), ratIv(22)}})
			})
		}), false},
		{"a record whose axis origin is off the world origin", face(func(f *topo.Face) {
			f.Denoted = withRecord(z, func(r *surfacenormal.Revolved) {
				r.Origin = ivVec(r3.Vec{Z: 1})
			})
		}), false},
		{"the same origin with the ends at the tag's level", face(func(f *topo.Face) {
			f.Denoted = withRecord(z-1, func(r *surfacenormal.Revolved) {
				r.Origin = ivVec(r3.Vec{Z: 1})
			})
		}), true},
		{"a widened leaf", face(func(f *topo.Face) {
			f.Denoted = withRecord(z, func(r *surfacenormal.Revolved) {
				r.Ends[0][1][1] = proofbound.Interval(big.NewRat(20, 1), big.NewRat(21, 1))
			})
		}), false},
		{"a basis that is not orthonormal", face(func(f *topo.Face) {
			f.Denoted = withRecord(z, func(r *surfacenormal.Revolved) {
				r.Basis[0] = ivVec(r3.Vec{X: 1, Y: 1})
			})
		}), false},
		{"a cap record", face(func(f *topo.Face) { f.Denoted = &capRecord }), false},
		{"an invalid record", face(func(f *topo.Face) { f.Denoted = &invalid }), false},
		{"a tag that states a normal departure", face(func(f *topo.Face) { f.NormalBound = 1e-9 }), false},
		{"a face that is not retained", face(func(f *topo.Face) { f.RetainedTag = false }), false},
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

// TestPrepBoolMeshReplayChargesDenotedPlanes checks that a bounded replay
// charges the denoted-plane decision once per distinct denoted Plane face. A
// tetrahedron whose four facets map to two retained Plane faces is prepared
// twice: with each face carrying an exact denoted record, and with none. Both
// read the same slabs, so the work differs by exactly two decisions and the
// records by two memo entries. Shown to fail with the charge removed.
func TestPrepBoolMeshReplayChargesDenotedPlanes(t *testing.T) {
	t.Parallel()
	usage := func(t *testing.T, denoted bool) (uint64, uint64) {
		t.Helper()
		faces := [2]*topo.Face{}
		for i := range faces {
			faces[i] = &topo.Face{Surface: zPlane(t, 0), RetainedTag: true}
			if denoted {
				faces[i].Denoted = zRevolved(0, 0, 0, 1)
			}
		}
		m := &topo.Mesh{
			Vertices:    []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}},
			Triangles:   [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
			Source:      []*topo.Face{faces[0], faces[0], faces[1], faces[1]},
			VertexBound: []float64{0, 0, 0, 0},
		}
		budget, err := proofbound.NewReplayBudgetLimits(proofbound.ReplayTrialWork, proofbound.ReplayTrialRecords,
			proofbound.ReplayTrialBytes)
		require.NoError(t, err)
		bm, err := booleaneval.PrepBoolMeshReplay(t.Context(), m, []int{0, 0, 1, 1}, true, budget)
		require.NoError(t, err)
		require.Zero(t, bm.SlabBound[0], "the facet in the tag plane reads a zero slab")
		return budget.Usage()
	}
	work, records := usage(t, true)
	plainWork, plainRecords := usage(t, false)
	require.Equal(t, plainWork+2*booleaneval.DenotedPlaneWork, work)
	require.Equal(t, plainRecords+2, records)
}
