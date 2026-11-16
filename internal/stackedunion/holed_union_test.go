package stackedunion_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// holedJoin is the union prismboolean hands stackedunion.Join for two prisms
// in the world XY frame when a region has holes: brepOnly set.
func holedJoin(t *testing.T, a, b prismrecord.Payload) (topo.Payload, bool, error) {
	t.Helper()
	va, ok := stackedunion.OperandFromPayload(a)
	require.True(t, ok)
	vb, ok := stackedunion.OperandFromPayload(b)
	require.True(t, ok)
	shift := new(big.Rat)
	levels, ok := stackedrecord.UnionLevels(va.Slabs, vb.Slabs, shift)
	require.True(t, ok)
	require.GreaterOrEqual(t, len(levels), 3)
	reexpress, err := prismcells.NewReexpression(va.Proxy.PlacementOperand(), vb.Proxy.PlacementOperand())
	require.NoError(t, err)
	return stackedunion.Join(t.Context(), proofbound.NewWorkBudget(t.Context()), va, vb, reexpress, levels, shift, true)
}

func holedPrism(t *testing.T, profile momentinput.Profile, z0, z1 float64) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return prismrecord.Payload{Profile: profile, Frame: frame, Z0: z0, Z1: z1, Xform: r3.Identity()}
}

// cwSquare is a clockwise square loop, a hole's winding.
func cwSquare(u0, v0, u1, v1 float64) sectionrecord.LoopRecord {
	corners := []sectionrecord.Point2{{U: u0, V: v0}, {U: u0, V: v1}, {U: u1, V: v1}, {U: u1, V: v0}}
	segments := make([]sectionrecord.CurveSegment, len(corners))
	for i, corner := range corners {
		segments[i] = sectionrecord.LineSeg{Start: corner, End: corners[(i+1)%len(corners)], TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segments}
}

func holedCircle(cx, cy, r float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: cx, V: cy}, Radius: units.Millimeters(r), CCW: true, TEnd: 1},
	}}}
}

// sectorHole is a clockwise annular sector about (5, 5): its inner arc runs
// radius 5 between (3, 4) and (4, 3) offsets, its outer arc radius 10 between
// (6, 8) and (8, 6). Every arc end is on its radius exactly.
func sectorHole() sectionrecord.LoopRecord {
	c := sectionrecord.Point2{U: 5, V: 5}
	pt := func(du, dv float64) sectionrecord.Point2 { return sectionrecord.Point2{U: 5 + du, V: 5 + dv} }
	innerA, innerB := pt(3, 4), pt(4, 3)
	outerA, outerB := pt(6, 8), pt(8, 6)
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: c, Start: innerA, End: innerB, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: innerB, End: outerB, TEnd: 1},
		sectionrecord.ArcSeg{Center: c, Start: outerA, End: outerB, TStart: 1, TEnd: 0},
		sectionrecord.LineSeg{Start: outerA, End: innerA, TEnd: 1},
	}}
}

// TestJoinHoledExactLines unions a 40 × 40 plate over [0, 5] whose hole
// [-5, 5]² has its top wall on the bottom wall of a block [-8, 8] × [5, 15]
// over [4, 6]. The span is exact lines, so every face and the stack carry a
// zero delta, and the hole stays in the slabs the plate reaches. A block
// [-5, 5] wide, whose side walls continue the hole's, is a miss: the top
// face's ring writes the collinear wall edges as one edge the two walls
// cannot pair with.
func TestJoinHoledExactLines(t *testing.T) {
	t.Parallel()
	plate := holedPrism(t, momentinput.Profile{Outer: rectRecord(-20, -20, 20, 20).Outer,
		Holes: []sectionrecord.LoopRecord{cwSquare(-5, -5, 5, 5)}}, 0, 5)
	block := holedPrism(t, rectRecord(-8, 5, 8, 15), 4, 6)
	payload, ok, err := holedJoin(t, plate, block)
	require.NoError(t, err)
	require.True(t, ok)
	brep, isBrep := payload.(breprecord.Payload)
	require.True(t, isBrep)
	require.NotNil(t, brep.Stack)
	require.Zero(t, brep.Stack.Delta)
	for i, face := range brep.Faces {
		require.Zero(t, face.Delta, "face %d", i)
	}
	require.Len(t, brep.Stack.Slabs, 3)
	require.Len(t, brep.Stack.Slabs[0].Regions[0].Holes, 1)
	require.Len(t, brep.Stack.Slabs[1].Regions[0].Holes, 1)
	require.Empty(t, brep.Stack.Slabs[2].Regions[0].Holes)
}

// TestJoinHoledChargePublished unions a plate whose annular-sector hole has
// its inner arc on radius 5 about (5, 5) exactly with a boss circle of the
// next float above 5. The shared span is one ulp wide, and every face and the
// stack carry at least that width.
func TestJoinHoledChargePublished(t *testing.T) {
	t.Parallel()
	r := math.Nextafter(5, math.Inf(1))
	gap := r - 5
	require.Positive(t, gap)
	plate := holedPrism(t, momentinput.Profile{Outer: rectRecord(-30, -30, 30, 30).Outer,
		Holes: []sectionrecord.LoopRecord{sectorHole()}}, 0, 5)
	boss := holedPrism(t, holedCircle(5, 5, r), 2, 8)
	payload, ok, err := holedJoin(t, plate, boss)
	require.NoError(t, err)
	require.True(t, ok)
	brep, isBrep := payload.(breprecord.Payload)
	require.True(t, isBrep)
	require.NotNil(t, brep.Stack)
	require.GreaterOrEqual(t, brep.Stack.Delta, gap)
	for i, face := range brep.Faces {
		require.GreaterOrEqual(t, face.Delta, gap, "face %d", i)
	}
}

// TestJoinHoledRejects covers the silent misses of a holed union: the record
// pre-filter, a span sketch does not report, and a hole-free union that keeps
// its earlier route.
func TestJoinHoledRejects(t *testing.T) {
	t.Parallel()
	holedPlate := func() prismrecord.Payload {
		return holedPrism(t, momentinput.Profile{Outer: rectRecord(-30, -30, 30, 30).Outer,
			Holes: []sectionrecord.LoopRecord{sectorHole()}}, 0, 5)
	}
	t.Run("no candidate", func(t *testing.T) {
		t.Parallel()
		plate := holedPrism(t, momentinput.Profile{Outer: rectRecord(-20, -20, 20, 20).Outer,
			Holes: []sectionrecord.LoopRecord{cwSquare(-5, -5, 5, 5)}}, 0, 5)
		block := holedPrism(t, rectRecord(12, 12, 18, 18), 4, 6)
		payload, ok, err := holedJoin(t, plate, block)
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, payload)
	})
	t.Run("no span", func(t *testing.T) {
		t.Parallel()
		boss := holedPrism(t, holedCircle(5, 5, 5+1e-11), 2, 8)
		payload, ok, err := holedJoin(t, holedPlate(), boss)
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, payload)
	})
	t.Run("hole-free", func(t *testing.T) {
		t.Parallel()
		plate := holedPrism(t, rectRecord(0, 0, 20, 20), -3, 2)
		boss := holedPrism(t, rectRecord(15, 5, 25, 15), -3, 1)
		payload, ok, err := holedJoin(t, plate, boss)
		require.NoError(t, err)
		require.True(t, ok)
		brep, isBrep := payload.(breprecord.Payload)
		require.True(t, isBrep)
		require.NotNil(t, brep.Stack)
	})
}
