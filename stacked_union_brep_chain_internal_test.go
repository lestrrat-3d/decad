package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/general-boolean-design.md §9's A1 brep operand
// rows: a co-directional Union whose operand is an A1 result reads that
// result's stack, arranges every record reaching each slab and interface,
// and keeps the merged slabs as its own stack.

// internalWallFaceAt finds the one planar face of a brep record whose frame
// normal is n at level along it.
func internalWallFaceAt(t *testing.T, bp brepPayload, n r3.Vec, level float64) brepFace {
	t.Helper()
	var found []brepFace
	for _, f := range bp.faces {
		if f.planar() && f.frame.N() == n && f.z0 == level {
			found = append(found, f)
		}
	}
	require.Len(t, found, 1, "one planar face faces %v at %v", n, level)
	return found[0]
}

// internalFlushCornerUnion is the flush corner boss result of
// stacked_union_brep_internal_test.go: a 40×40×10 plate with a 10 mm boss
// standing in its (20, −20) corner.
func internalFlushCornerUnion(t *testing.T, doc *Document) *Body {
	t.Helper()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalBoxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	bp := requireBrep(t, got)
	require.NotNil(t, bp.stack, "an A1 result keeps its stack")
	return got
}

// TestStackedUnionBrepOperandSecondBoss unions the flush corner boss result
// with a second boss flush in the opposite corner, in either operand order.
// The upper slab holds both bosses apart, the interface arranges the plate
// against both, and every flush wall is one L-shaped face. 16000 + 1500 +
// 1500 mm³ and 1600 + 1400 + 2·100 + 4·550 + 4·150 mm², exactly. Shown to
// fail with stackedUnionOperandOf's brepPayload arm deleted (the pair took
// the mesh path and refused the coplanar contact) and with slabRegions'
// several-loop merge replaced by the one-loop merge (the apart bosses then
// missed).
func TestStackedUnionBrepOperandSecondBoss(t *testing.T) {
	t.Parallel()
	for _, swap := range []bool{false, true} {
		t.Run(map[bool]string{false: "brep first", true: "brep second"}[swap], func(t *testing.T) {
			t.Parallel()
			doc := New()
			first := internalFlushCornerUnion(t, doc)
			second := internalBoxBodyAtZ(t, doc, -20, 10, -10, 20, 10, 15)
			a, b := first, second
			if swap {
				a, b = second, first
			}
			payload, ok, err := tryStackedUnion(t.Context(), a, b)
			require.NoError(t, err)
			require.True(t, ok)
			bp, isBrep := payload.(brepPayload)
			require.True(t, isBrep, "got %T", payload)
			require.Len(t, bp.faces, 12)
			require.Len(t, bp.stack.slabs, 2)
			require.Len(t, bp.stack.slabs[0].regions, 1)
			require.Len(t, bp.stack.slabs[1].regions, 2, "the upper slab holds both bosses")
			require.Zero(t, bp.stack.delta)
			for _, n := range []r3.Vec{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}} {
				wall := internalWallFaceAt(t, bp, n, 20)
				require.Len(t, wall.region.Outer.Segments, 6, "the flush wall facing %v is one L-shaped face", n)
			}
			got, err := Union(t.Context(), a, b)
			require.NoError(t, err)
			requireClosedTopology(t, got)
			require.Len(t, got.Faces(), 12)
			require.Equal(t, Exact, got.volume.Exactness)
			require.Equal(t, 19000.0, got.volume.Value.Base())
			require.Equal(t, 6000.0, got.area.Value.Base())
			floors := 0
			floorLevel := bp.stack.slabs[0].z1
			for _, f := range bp.faces {
				if f.planar() && f.outward && f.z0 == floorLevel && f.frame == bp.faces[0].frame {
					floors++
					area, err := loopSignedAreaCB(f.region.Outer)
					require.NoError(t, err)
					require.Equal(t, 1400.0, area)
				}
			}
			require.Equal(t, 1, floors, "one floor, the plate less both corners")
			mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// TestStackedUnionBrepOperandThirdBoss unions the two-boss result with a
// third boss standing on the plate's edge y = 20: the interface at z = 10
// arranges four records (the plate, both bosses of the result's upper slab,
// and the new boss), and the volume is 16000 + 3·1500. Shown to fail with
// the interface scene restricted to two records (a miss).
func TestStackedUnionBrepOperandThirdBoss(t *testing.T) {
	t.Parallel()
	doc := New()
	first := internalFlushCornerUnion(t, doc)
	second := internalBoxBodyAtZ(t, doc, -20, 10, -10, 20, 10, 15)
	two, err := Union(t.Context(), first, second)
	require.NoError(t, err)
	third := internalBoxBodyAtZ(t, doc, -5, 10, 5, 20, 10, 15)
	got, err := Union(t.Context(), two, third)
	require.NoError(t, err)
	bp := requireBrep(t, got)
	require.Len(t, bp.stack.slabs[1].regions, 3)
	require.Equal(t, Exact, got.volume.Exactness)
	require.Equal(t, 20500.0, got.volume.Value.Base())
	// Three top caps, the floor, the bottom, and 4 + 4 + 4 + 4 walls, with
	// the y = 20 wall one face across the plate and two bosses.
	require.Len(t, got.Faces(), 16)
	wall := internalWallFaceAt(t, bp, r3.NewVec(0, 1, 0), 20)
	require.Len(t, wall.region.Outer.Segments, 10, "the y = 20 wall carries the plate and two bosses")
	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}

// TestStackedUnionBrepOperandOverlappingBoss unions the flush corner boss
// result with a second boss overlapping the first in the upper slab: the
// slab's merge is one loop, the two bosses' outline, and the floor is the
// plate less that outline. Volume 16000 + 1500 + 1500 − 5·5·15 = 18625.
func TestStackedUnionBrepOperandOverlappingBoss(t *testing.T) {
	t.Parallel()
	doc := New()
	first := internalFlushCornerUnion(t, doc)
	second := internalBoxBodyAtZ(t, doc, 5, -15, 15, -5, 10, 15)
	got, err := Union(t.Context(), first, second)
	require.NoError(t, err)
	bp := requireBrep(t, got)
	require.Len(t, bp.stack.slabs[1].regions, 1, "overlapping bosses merge into one region")
	require.Equal(t, Exact, got.volume.Exactness)
	require.Equal(t, 18625.0, got.volume.Value.Base())
	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}

// TestClassifyRegionsReadsEachRecord arranges a plate with two bosses flush
// in two corners, the plate and one boss as operand A's two regions and the
// other boss as operand B, and reads each cell's membership per record: the
// plate-only cells sum to 1400 mm², each boss's cell is inside that boss and
// the plate and outside the other boss, and the shared walls between the
// plate and its own boss resolve as spans. Shown to fail with
// CoincidentEdgesRegions partnering across operands only (unresolved).
func TestClassifyRegionsReadsEachRecord(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10).payload.(prismPayload).profile
	boss1 := internalBoxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15).payload.(prismPayload).profile
	boss2 := internalBoxBodyAtZ(t, doc, -20, 10, -10, 20, 10, 15).payload.(prismPayload).profile
	budget := proofbound.NewWorkBudget(t.Context())
	s, tags, _, err := buildPrismSceneRegions(budget, []ProfileRecord{plate, boss1}, []ProfileRecord{boss2}, &prismReexpression{identity: true})
	require.NoError(t, err)
	profiles, err := prismCellProfiles(t.Context(), budget, s)
	require.NoError(t, err)
	reading, ok, err := prismcells.CoincidentEdgesRegions(budget, tags, profiles)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, reading.Spans, 4, "each boss shares two walls with the plate")
	matter, resolved, err := prismcells.ClassifyRegions(budget, tags, profiles)
	require.NoError(t, err)
	require.True(t, resolved)
	inPlate := matter[prismcells.RegionKey{IsB: false, Region: 0}]
	inBoss1 := matter[prismcells.RegionKey{IsB: false, Region: 1}]
	inBoss2 := matter[prismcells.RegionKey{IsB: true, Region: 0}]
	plateOnly, boss1Cells, boss2Cells := 0.0, 0, 0
	for i, p := range profiles {
		switch {
		case inPlate[i] && !inBoss1[i] && !inBoss2[i]:
			plateOnly += p.Area
		case inPlate[i] && inBoss1[i] && !inBoss2[i]:
			boss1Cells++
			require.InDelta(t, 100, p.Area, 1e-9)
		case inPlate[i] && !inBoss1[i] && inBoss2[i]:
			boss2Cells++
			require.InDelta(t, 100, p.Area, 1e-9)
		default:
			t.Fatalf("cell %d is in no record's expected combination", i)
		}
	}
	require.InDelta(t, 1400, plateOnly, 1e-9)
	require.Equal(t, 1, boss1Cells)
	require.Equal(t, 1, boss2Cells)
}

// TestStackedUnionBrepOperandMisses pins the brep operands the build hands
// to the mesh path: a class-B result keeps no stack, and a crossing round
// boss result's stack carries the crossings' displacement (B5).
func TestStackedUnionBrepOperandMisses(t *testing.T) {
	t.Parallel()
	t.Run("class-B result", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, 0, 0, 40, 20, 10)
		pin := internalPinAlongX(t, doc, 30, 20)
		classB, err := Union(t.Context(), plate, pin)
		require.NoError(t, err)
		require.Nil(t, requireBrep(t, classB).stack)
		boss := internalBoxBodyAtZ(t, doc, 0, 0, 10, 10, 10, 5)
		_, ok, err := tryStackedUnion(t.Context(), classB, boss)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("displaced stack", func(t *testing.T) {
		t.Parallel()
		plate, round := internalBossOnPlate(t, 18, 10, 15)
		crossing, err := Union(t.Context(), plate, round)
		require.NoError(t, err)
		require.Positive(t, requireBrep(t, crossing).stack.delta)
		second := internalBoxBodyAtZ(t, plate.doc, -20, 10, -10, 20, 10, 15)
		_, ok, err := tryStackedUnion(t.Context(), crossing, second)
		require.NoError(t, err)
		require.False(t, ok)
		// The mesh path refuses the second boss's coplanar contact, as it
		// refuses any boss standing on a faceted body.
		_, err = Union(t.Context(), crossing, second)
		var be *BooleanError
		require.ErrorAs(t, err, &be)
		require.Equal(t, BooleanUnsupportedContact, be.Code)
	})
}
