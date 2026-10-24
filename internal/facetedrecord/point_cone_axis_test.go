package facetedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The outside witness charges the mesh-bound move at (1+k)·meshBound·|A|↑/c
// about a general axis (agent-docs/revolve-axis-design.md §5.4). About
// (1, 2, 0) with c = 10 and k = 1, the vertex (4, 0, 0) has s = 4 and σ = 8,
// so q = 1.2 and its margin is just under 0.2. The move is 0.2·√5·meshBound.
func TestPointConeOutsideWitnessChargesAxisLength(t *testing.T) {
	t.Parallel()
	axis, ok := coneaxis.New(r3.Vec{X: 1, Y: 2})
	require.True(t, ok)
	cone := &facetedrecord.PointConeLimit{Axis: axis, Apex: big.NewRat(10, 1), Slope: big.NewRat(1, 1)}
	base := facetedrecord.Payload{Verts: []r3.Vec{{}, {X: 4}}, MeshBound: 0.3}
	require.True(t, facetedrecord.PointConeHasOutsideWitness(base, cone), "0.2 > 0.06·√5")
	base.MeshBound = 0.6
	require.False(t, facetedrecord.PointConeHasOutsideWitness(base, cone), "0.2 < 0.12·√5")
}

// A canonical limit has one constant set. A limit about (2, 0, 0) with the
// A-scaled apex 2a has a sample set built from 2a and a bound set built from
// a↑ = a, which equals the canonical set of apex a in every triangle bound.
func TestPointConeLimitConstantSets(t *testing.T) {
	t.Parallel()
	canonical := &facetedrecord.PointConeLimit{Axis: coneaxis.X(), Apex: big.NewRat(5, 1), Slope: big.NewRat(1, 1)}
	require.Same(t, canonical.Constants(), canonical.BoundConstants())

	axis, ok := coneaxis.New(r3.Vec{X: 2})
	require.True(t, ok)
	scaled := &facetedrecord.PointConeLimit{Axis: axis, Apex: big.NewRat(10, 1), Slope: big.NewRat(1, 1)}
	require.NotSame(t, scaled.Constants(), scaled.BoundConstants())
	tri := [3]r3.Vec{{X: 4, Y: -1, Z: 2}, {X: 5, Y: 1, Z: 3}, {X: 4.5, Y: 0.5, Z: 2.5}}
	geom := pointcone.CellGeometry(tri, coneaxis.X())
	want, _, _, err := pointcone.TriangleBound(geom, canonical.BoundConstants())
	require.NoError(t, err)
	got, _, _, err := pointcone.TriangleBound(geom, scaled.BoundConstants())
	require.NoError(t, err)
	require.Equal(t, want, got)
	doubled, _, _, err := pointcone.TriangleBound(geom, scaled.Constants())
	require.NoError(t, err)
	require.Greater(t, doubled, got, "the sample set reads the A-scaled apex 2a")
}
