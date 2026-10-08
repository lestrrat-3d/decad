package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/general-boolean-design.md §9's A1 brep rows: a
// Union whose boss is flush with the plate's outline, or crosses it, builds an
// analytic brep body that every brep consumer reads.

// requireOneWallFace asserts exactly one planar face of body faces n and
// reads the given area exactly.
func requireOneWallFace(t *testing.T, body *decad.Body, n r3.Vec, area float64) {
	t.Helper()
	faces, err := decad.Faces(decad.Planar(), decad.Facing(n)).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	got, err := faces[0].Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, got.Exactness)
	require.Equal(t, area, got.Value.Base())
}

func requireEveryEdgeOnTwoFaces(t *testing.T, body *decad.Body) {
	t.Helper()
	require.Len(t, body.Lumps(), 1)
	for _, e := range body.Edges() {
		require.Len(t, e.Faces(), 2)
	}
}

// TestStackedUnionFlushCornerBoss stands a 10 mm square boss in the plate's
// corner, sharing its walls x = 20 and y = −20: each shared wall is one
// planar face of 550 mm², the plate's 400 and the boss's 150 together. The
// body is exact: 17500 mm³, 5400 mm², 9 faces, watertight, Sound, with a
// proven 5 mm clearance to a box standing off the shared wall x = 20.
func TestStackedUnionFlushCornerBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := boxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Faces(), 9)
	requireEveryEdgeOnTwoFaces(t, got)
	requireOneWallFace(t, got, r3.NewVec(1, 0, 0), 550)
	requireOneWallFace(t, got, r3.NewVec(0, -1, 0), 550)
	requireOneWallFace(t, got, r3.NewVec(0, 0, -1), 1600)

	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 17500.0, volumeMM(t, volume))
	area, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Equal(t, 5400.0, area.Value.Base())
	box, err := got.Bounds()
	require.NoError(t, err)
	require.Equal(t, 20.0, box.Max.X)
	require.Equal(t, 25.0, box.Max.Z)
	requireMeshWatertightAt(t, got, 0.05)

	// A box standing 5 mm off the shared wall x = 20, and one 5 mm above the
	// boss's top: both gaps read through the brep's own carriers.
	off := boxBody(t, doc, 25, -20, 30, 20, 10)
	above := boxBodyAtZ(t, doc, 10, -20, 20, -10, 30, 5)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Interferences)
	gaps := map[*decad.Body]decad.Measurement{}
	for _, row := range report.Clearances {
		switch got {
		case row.A:
			gaps[row.B] = row.Gap
		case row.B:
			gaps[row.A] = row.Gap
		}
	}
	for other, want := range map[*decad.Body]float64{off: 5, above: 5} {
		gap, found := gaps[other]
		require.True(t, found, "the union has a clearance row")
		require.LessOrEqual(t, math.Abs(gap.Value.Base()-want), gap.Bound.Base()+1e-12)
	}
}

// TestStackedUnionFlushEdgeBoss stands the boss in the middle of the plate's
// edge x = 20: that wall is one T-shaped face of 550 mm², the body has 10
// faces and the same exact volume and area.
func TestStackedUnionFlushEdgeBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := boxBodyAtZ(t, doc, 10, -5, 20, 5, 10, 15)
	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Faces(), 10)
	requireEveryEdgeOnTwoFaces(t, got)
	requireOneWallFace(t, got, r3.NewVec(1, 0, 0), 550)
	requireOneWallFace(t, got, r3.NewVec(0, 0, -1), 1600)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 17500.0, volumeMM(t, volume))
	area, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, 5400.0, area.Value.Base())
	requireMeshWatertightAt(t, got, 0.05)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestStackedUnionCrossingBossStanding stands a Ø10 boss on the plate's top
// with its centre 2 mm inside the wall x = 20, so its footprint crosses the
// outline. The boss's wall splits into two cylinder pieces at the two
// crossings; the floor is the plate less the disc's inside part and the
// ceiling the disc's outside part. Volume: the plate plus the whole boss.
// Area: the plate's 1600 + 4·400, less the disc's inside part on top, plus
// that inside part's complement under the overhang, the 25π top and the
// 150π wall — 4800 + 150π + 2·S with S = 25·acos(0.4) − 2·√21.
func TestStackedUnionCrossingBossStanding(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := circleBodyAtZ(t, doc, 18, 5, 10, 15)
	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Faces(), 10)
	requireEveryEdgeOnTwoFaces(t, got)
	cylinders, err := decad.Faces(decad.Cylindrical()).Exactly(2).SelectFaces(got)
	require.NoError(t, err)
	require.Len(t, cylinders, 2)

	volume, err := got.Volume()
	require.NoError(t, err)
	want := 16000 + 25*15*math.Pi
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	require.Less(t, boundMM3(t, volume), 1e-9)
	area, err := got.Area()
	require.NoError(t, err)
	segment := 25*math.Acos(0.4) - 2*math.Sqrt(21)
	wantArea := 4800 + 150*math.Pi + 2*segment
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base())
	box, err := got.Bounds()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(box.Max.X-23), box.Bound.Base())
	requireMeshWatertightAt(t, got, 0.05)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)

	// A second union on the brep result takes the mesh path: a brep operand
	// is outside class A, and a co-directional pair is outside class B.
	second := boxBodyAtZ(t, doc, -20, 10, -10, 20, 25, 5)
	chained, err := decad.Union(t.Context(), got, second)
	require.NoError(t, err)
	volume, err = chained.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want-500), boundMM3(t, volume))
}

// TestStackedUnionBrepOperandChain unions the flush corner boss result with
// a second boss flush in the opposite corner, then a third on the plate's
// edge, each through the public API: every result is analytic, exact, and
// Sound, and every flush wall is one planar face.
func TestStackedUnionBrepOperandChain(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	first := boxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
	one, err := decad.Union(t.Context(), plate, first)
	require.NoError(t, err)
	second := boxBodyAtZ(t, doc, -20, 10, -10, 20, 10, 15)
	two, err := decad.Union(t.Context(), one, second)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(two))
	require.Len(t, two.Faces(), 12)
	requireEveryEdgeOnTwoFaces(t, two)
	requireOneWallFace(t, two, r3.NewVec(0, 0, -1), 1600)
	volume, err := two.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 19000.0, volumeMM(t, volume))
	area, err := two.Area()
	require.NoError(t, err)
	require.Equal(t, 6000.0, area.Value.Base())
	requireMeshWatertightAt(t, two, 0.05)

	third := boxBodyAtZ(t, doc, -5, 10, 5, 20, 10, 15)
	three, err := decad.Union(t.Context(), two, third)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(three))
	require.Len(t, three.Faces(), 16)
	volume, err = three.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 20500.0, volumeMM(t, volume))
	requireMeshWatertightAt(t, three, 0.05)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Len(t, doc.Bodies(), 1, "each union consumes both operands")
}
