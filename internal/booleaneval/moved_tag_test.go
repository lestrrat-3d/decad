package booleaneval_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// requireExactSum asserts got is the exact rational sum o + s.
func requireExactSum(t *testing.T, o, s, got float64) {
	t.Helper()
	sum := new(big.Rat).Add(new(big.Rat).SetFloat64(o), new(big.Rat).SetFloat64(s))
	require.Zero(t, sum.Cmp(new(big.Rat).SetFloat64(got)), "%v + %v is not exactly %v", o, s, got)
}

// requireBare asserts g keeps the provenance of src and no tag.
func requireBare(t *testing.T, src, g facetedrecord.Group) {
	t.Helper()
	require.Equal(t, facetedrecord.Group{Origins: src.Origins, Source: src.Source}, g)
}

func TestMovedGroup(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{r3.NewVec(44, 30, 10), r3.NewVec(56, 30, 18)}
	s := booleaneval.SnapShift(r3.NewVec(0.0010544506, 0.0006516862, -0.0048694890))
	shifts := map[int]booleaneval.PieceShift{0: {Move: s}, 2: {Move: s, Grow: 0.001}}
	cyl := facetedrecord.Group{
		Origins:  []provenance.Ref{{Role: provenance.RoleBody}},
		Source:   &facetedrecord.SourcePatch{Operand: 1, Ordinal: 3},
		Surface:  surfacegeom.Cylinder{Origin: r3.NewVec(50, 30, 10), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(6)},
		Retained: true,
	}

	t.Run("a translated piece carries its tag moved by the exact shift", func(t *testing.T) {
		t.Parallel()
		g := booleaneval.MovedGroup(cyl, 0, true, shifts, verts, 0.01)
		moved, ok := g.Surface.(surfacegeom.Cylinder)
		require.True(t, ok)
		requireExactSum(t, 50, s.X, moved.Origin.X)
		requireExactSum(t, 30, s.Y, moved.Origin.Y)
		requireExactSum(t, 10, s.Z, moved.Origin.Z)
		require.Equal(t, r3.NewVec(0, 0, 1), moved.Axis)
		require.Equal(t, 6.0, moved.Radius.Base())
		require.True(t, g.Retained)
		require.Zero(t, g.NormalBound, "an exact translation adds no departure, so a blend reads the tag")
		require.Equal(t, cyl.Origins, g.Origins)
		require.Same(t, cyl.Source, g.Source)
	})

	t.Run("a translated plane drops its cap record", func(t *testing.T) {
		t.Parallel()
		frame, err := r3.NewFrame(r3.NewVec(30, 0, 18), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		top := facetedrecord.Group{
			Surface: surfacegeom.Plane{Frame: frame}, Retained: true, Planar: true,
			CapRecord: &topo.CapRecord{Frame: frame, Xform: r3.Identity(), Z: 18}, HasAxialDelta: true,
		}
		g := booleaneval.MovedGroup(top, 0, true, shifts, verts, 0.01)
		plane, ok := g.Surface.(surfacegeom.Plane)
		require.True(t, ok)
		requireExactSum(t, 18, s.Z, plane.Frame.Origin().Z)
		require.Zero(t, g.NormalBound)
		require.Nil(t, g.CapRecord, "the record states the unmoved plane")
		require.False(t, g.HasAxialDelta)
	})

	t.Run("a piece no shift names keeps its tag", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, cyl, booleaneval.MovedGroup(cyl, 1, true, shifts, verts, 0.01))
	})

	t.Run("a grown piece keeps no tag", func(t *testing.T) {
		t.Parallel()
		requireBare(t, cyl, booleaneval.MovedGroup(cyl, 2, true, shifts, verts, 0.01))
	})

	t.Run("a face spanning pieces keeps no tag", func(t *testing.T) {
		t.Parallel()
		requireBare(t, cyl, booleaneval.MovedGroup(cyl, booleaneval.SpanningPieces, true, shifts, verts, 0.01))
	})

	t.Run("a face with no facet keeps no tag", func(t *testing.T) {
		t.Parallel()
		requireBare(t, cyl, booleaneval.MovedGroup(cyl, 0, false, shifts, verts, 0.01))
	})

	t.Run("a denoted tag keeps no tag", func(t *testing.T) {
		t.Parallel()
		denoted := cyl
		denoted.Denoted = &surfacenormal.Revolved{}
		requireBare(t, denoted, booleaneval.MovedGroup(denoted, 0, true, shifts, verts, 0.01))
	})

	t.Run("an unretained tag keeps no tag", func(t *testing.T) {
		t.Parallel()
		built := cyl
		built.Retained = false
		requireBare(t, built, booleaneval.MovedGroup(built, 0, true, shifts, verts, 0.01))
	})

	t.Run("a cone keeps no tag", func(t *testing.T) {
		t.Parallel()
		cone := cyl
		cone.Surface = surfacegeom.Cone{Origin: r3.NewVec(50, 30, 10), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(6), HalfAngle: units.Degrees(10)}
		requireBare(t, cone, booleaneval.MovedGroup(cone, 0, true, shifts, verts, 0.01))
	})
}

func TestFacePieces(t *testing.T) {
	t.Parallel()
	// Two separate tetrahedra, 100 mm apart. Face a covers the first, face b
	// the second, and face c one facet of each.
	tet := func(dx float64) []r3.Vec {
		return []r3.Vec{{X: dx}, {X: dx + 1}, {X: dx, Y: 1}, {X: dx, Z: 1}}
	}
	a, b, c := &topo.Face{}, &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{
		Vertices:  append(tet(0), tet(100)...),
		Triangles: [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}, {4, 6, 5}, {4, 5, 7}, {5, 6, 7}, {4, 7, 6}},
		Source:    []*topo.Face{a, a, a, c, b, b, b, c},
	}
	got := booleaneval.FacePieces(m)
	require.Equal(t, map[*topo.Face]int{a: 0, b: 1, c: booleaneval.SpanningPieces}, got)
}
