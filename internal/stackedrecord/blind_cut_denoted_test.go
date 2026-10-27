package stackedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The plate is 20x20 over [0, 12] on the XY plane. Every tool is drawn on the
// plane z=12 and swept down, so its inner end is 12 plus its Z0.
const plateTop = 12.0

func rectRecord(u0, v0, u1, v1 float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segments := make([]sectionrecord.CurveSegment, len(corners))
	for i, corner := range corners {
		segments[i] = sectionrecord.LineSeg{Start: corner, End: corners[(i+1)%len(corners)], TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

func xyFrame(t *testing.T, z float64) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

func plateRecord(t *testing.T) prismrecord.Payload {
	t.Helper()
	return prismrecord.Payload{Profile: rectRecord(0, 0, 20, 20), Frame: xyFrame(t, 0),
		Z0: 0, Z1: plateTop, Xform: r3.Identity()}
}

// toolRecord is a pocket tool over [z0, 0] below z=12 whose lower level
// carries delta and, when denoted is not nil, records that denotation.
func toolRecord(t *testing.T, u0, u1, z0, delta float64, denoted *big.Rat) prismrecord.Payload {
	t.Helper()
	return prismrecord.Payload{Profile: rectRecord(u0, 2, u1, 18), Frame: xyFrame(t, plateTop),
		Z0: z0, Z0Delta: delta, Z0Denoted: prismplacement.DenoteLevel(denoted, z0, delta),
		Xform: r3.Identity()}
}

// converted is the tool a Distance of depth Against records: the negated
// held magnitude, its conversion rounding and the exact rescale.
func converted(t *testing.T, u0, u1 float64, depth units.Value) prismrecord.Payload {
	t.Helper()
	m, delta, err := extent.MagnitudeInBounded(depth, units.Length, units.Millimeter, "depth")
	require.NoError(t, err)
	return toolRecord(t, u0, u1, -m, delta, extent.ScaledExact(extent.ExactConversion(depth, units.Millimeter), -1, 1))
}

// floorDenoted is the level a depth leaves below the plate top.
func floorDenoted(depth units.Value) *big.Rat {
	return new(big.Rat).Sub(big.NewRat(int64(plateTop), 1), extent.ExactConversion(depth, units.Millimeter))
}

// firstPocket cuts a 0.2 in pocket at u in [2, 8] into the plate.
func firstPocket(t *testing.T) stackedrecord.Payload {
	t.Helper()
	sp, ok, err := stackedrecord.BuildBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()),
		plateRecord(t), converted(t, 2, 8, units.Inches(0.2)))
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, sp.Slabs, 2)
	require.Positive(t, sp.Slabs[0].Z1Delta, "the floor level carries a displacement")
	require.Zero(t, sp.Slabs[0].DenotedZ1().Cmp(floorDenoted(units.Inches(0.2))))
	require.Zero(t, sp.Slabs[1].DenotedZ0().Cmp(floorDenoted(units.Inches(0.2))))
	return sp
}

func applyBlindCut(t *testing.T, sp stackedrecord.Payload, tool prismrecord.Payload) (stackedrecord.Payload, bool) {
	t.Helper()
	got, ok, err := stackedrecord.ApplyBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()), sp, sp.OuterPrism(), tool)
	require.NoError(t, err)
	return got, ok
}

func TestApplyBlindCutDenotedInterface(t *testing.T) {
	t.Parallel()
	t.Run("equal denoted levels reuse the interface", func(t *testing.T) {
		t.Parallel()
		sp := firstPocket(t)
		got, ok := applyBlindCut(t, sp, converted(t, 12, 18, units.Inches(0.2)))
		require.True(t, ok)
		require.Len(t, got.Slabs, 2)
		require.Equal(t, sp.Slabs[0].Z1, got.Slabs[0].Z1)
		require.Len(t, got.Interfaces[0].LowerExposed, 2)
		require.NoError(t, stackedrecord.Falsify(t.Context(), stackedrecord.Record{Slabs: got.Slabs, Interfaces: got.Interfaces}))
	})
	t.Run("equal held levels with another denotation decline", func(t *testing.T) {
		t.Parallel()
		tool := converted(t, 12, 18, units.Millimeters(5.08))
		require.Equal(t, converted(t, 12, 18, units.Inches(0.2)).Z0, tool.Z0, "the two depths hold one float")
		_, ok := applyBlindCut(t, firstPocket(t), tool)
		require.False(t, ok)
	})
	t.Run("displaced tool without a denotation declines", func(t *testing.T) {
		t.Parallel()
		tool := converted(t, 12, 18, units.Inches(0.2))
		tool.Z0Denoted = prismplacement.LevelDenotation{}
		_, ok := applyBlindCut(t, firstPocket(t), tool)
		require.False(t, ok)
	})
	t.Run("displaced interface without a denotation declines", func(t *testing.T) {
		t.Parallel()
		sp := firstPocket(t)
		sp.Slabs[0].Z1Denoted, sp.Slabs[1].Z0Denoted = prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{}
		_, ok := applyBlindCut(t, sp, converted(t, 12, 18, units.Inches(0.2)))
		require.False(t, ok)
	})
	t.Run("one side of the interface without a denotation declines", func(t *testing.T) {
		t.Parallel()
		sp := firstPocket(t)
		sp.Slabs[1].Z0Denoted = prismplacement.LevelDenotation{}
		_, ok := applyBlindCut(t, sp, converted(t, 12, 18, units.Inches(0.2)))
		require.False(t, ok)
	})
	t.Run("interface sides denoting two levels decline and fail the audit", func(t *testing.T) {
		t.Parallel()
		sp := firstPocket(t)
		above := sp.Slabs[1]
		// The held float itself lies within the displacement, so this second
		// record is a claim the denotation check cannot disprove alone.
		sp.Slabs[1].Z0Denoted = prismplacement.DenoteLevel(new(big.Rat).SetFloat64(above.Z0), above.Z0, above.Z0Delta)
		require.NotNil(t, sp.Slabs[1].DenotedZ0())
		_, ok := applyBlindCut(t, sp, converted(t, 12, 18, units.Inches(0.2)))
		require.False(t, ok)
		err := stackedrecord.Falsify(t.Context(), stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "record two denoted levels for one interface")
	})
}

func TestApplyBlindCutDenotedSplitOrder(t *testing.T) {
	t.Parallel()
	// A tool whose held end lies 1e-6 below the floor, inside the lower slab,
	// with a displacement wide enough to denote a level on either side.
	sp := firstPocket(t)
	floor := floorDenoted(units.Inches(0.2))
	held := sp.Slabs[0].Z1 - plateTop - 1e-6
	toolAt := func(offset *big.Rat) prismrecord.Payload {
		level := new(big.Rat).Sub(floor, big.NewRat(int64(plateTop), 1))
		return toolRecord(t, 12, 18, held, 1e-5, level.Add(level, offset))
	}
	t.Run("denoted below the floor splits", func(t *testing.T) {
		t.Parallel()
		got, ok := applyBlindCut(t, sp, toolAt(big.NewRat(-2, 1_000_000)))
		require.True(t, ok)
		require.Len(t, got.Slabs, 3)
		want := new(big.Rat).Add(floor, big.NewRat(-2, 1_000_000))
		require.Zero(t, got.Slabs[0].DenotedZ1().Cmp(want))
		require.Zero(t, got.Slabs[1].DenotedZ0().Cmp(want))
	})
	t.Run("denoted above the floor declines", func(t *testing.T) {
		t.Parallel()
		_, ok := applyBlindCut(t, sp, toolAt(big.NewRat(1, 1_000_000_000)))
		require.False(t, ok)
	})
}

func TestBuildBlindCutDenotedOrder(t *testing.T) {
	t.Parallel()
	// The plate's top is held at 12 but denotes 12 - 1e-6 within its
	// displacement; a tool ending 1e-7 below z=12 is inside the held interval
	// and above the denoted top.
	plate := plateRecord(t)
	plate.Z1Delta = 1e-5
	tool := toolRecord(t, 2, 8, -1e-7, 0, nil)
	// An undenoted top is known only to its displacement, so a tool end
	// inside that window proves no pocket; one clear of it does.
	t.Run("an undenoted top within its displacement of the tool end declines", func(t *testing.T) {
		t.Parallel()
		_, ok, err := stackedrecord.BuildBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()), plate, tool)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("an undenoted top clear of the tool end admits the held order", func(t *testing.T) {
		t.Parallel()
		// The tool also rises clear of the top's displacement.
		cleared := toolRecord(t, 2, 8, -1e-4, 0, nil)
		cleared.Z1 = 1e-4
		_, ok, err := stackedrecord.BuildBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()), plate, cleared)
		require.NoError(t, err)
		require.True(t, ok)
	})
	t.Run("a denoted top below the tool end declines", func(t *testing.T) {
		t.Parallel()
		denoted := plate
		denoted.Z1Denoted = prismplacement.DenoteLevel(
			new(big.Rat).Sub(big.NewRat(int64(plateTop), 1), big.NewRat(1, 1_000_000)), plate.Z1, plate.Z1Delta)
		require.NotNil(t, denoted.DenotedZ1())
		_, ok, err := stackedrecord.BuildBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()), denoted, tool)
		require.NoError(t, err)
		require.False(t, ok)
	})
}

func TestApplyBlindCutUndenotedInnerEnd(t *testing.T) {
	t.Parallel()
	// The first pocket's floor denotes 12 - 0.2 in. A tool whose end records
	// no denotation is known only to its displacement, 1e-5: an end 1e-6 below
	// or above the floor's float may lie on either side of the floor, and one
	// 1e-4 away lies inside the slab on its side whatever its displacement.
	sp := firstPocket(t)
	for _, tc := range []struct {
		name   string
		offset float64
		splits bool
	}{
		{"within the displacement below the floor declines", 1e-6, false},
		{"within the displacement above the floor declines", -1e-6, false},
		{"clear of the floor splits the lower slab", 1e-4, true},
		{"clear of the floor splits the upper slab", -1e-4, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			tool := toolRecord(t, 12, 18, sp.Slabs[0].Z1-plateTop-tc.offset, 1e-5, nil)
			got, ok := applyBlindCut(t, sp, tool)
			require.Equal(t, tc.splits, ok)
			if tc.splits {
				require.Len(t, got.Slabs, 3)
				split := 0
				if tc.offset < 0 {
					split = 1
				}
				require.Nil(t, got.Slabs[split].DenotedZ1(), "an undenoted end records no level")
			}
		})
	}
}

func TestUnionLevelsReadRanges(t *testing.T) {
	t.Parallel()
	// The lower operand's top holds 5.08 with a 1e-15 displacement and no
	// denotation; the upper operand starts one ulp above it, inside that
	// displacement, so the held order is no proven order.
	lower := []stackedrecord.Slab{{Z0: 0, Z1: 5.08, Z1Delta: 1e-15}}
	for _, tc := range []struct {
		name  string
		upper stackedrecord.Slab
		ok    bool
	}{
		{"distinct floats within a displacement", stackedrecord.Slab{Z0: 5.080000000000001, Z1: 10}, false},
		{"distinct floats clear of the displacement", stackedrecord.Slab{Z0: 5.09, Z1: 10}, true},
		{"equal floats, one displaced", stackedrecord.Slab{Z0: 5.08, Z1: 10}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok := stackedrecord.UnionLevels(lower, []stackedrecord.Slab{tc.upper}, new(big.Rat))
			require.Equal(t, tc.ok, ok)
		})
	}
	t.Run("equal floats denoting one level merge", func(t *testing.T) {
		t.Parallel()
		exact := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
		d := prismplacement.DenoteLevel(exact, 5.08, 1e-15)
		a := []stackedrecord.Slab{{Z0: 0, Z1: 5.08, Z1Delta: 1e-15, Z1Denoted: d}}
		b := []stackedrecord.Slab{{Z0: 5.08, Z0Delta: 1e-15, Z0Denoted: d, Z1: 10}}
		levels, ok := stackedrecord.UnionLevels(a, b, new(big.Rat))
		require.True(t, ok)
		require.Len(t, levels, 3)
	})
}

func TestBuildBlindCutUndenotedCap(t *testing.T) {
	t.Parallel()
	// The plate's bottom holds 0 with a 1e-5 displacement and no denotation.
	// A pocket from the top whose floor lies 1e-6 above that float may break
	// through the bottom, so no pocket is proven; one 1e-4 above it is.
	plate := plateRecord(t)
	plate.Z0Delta = 1e-5
	for _, tc := range []struct {
		name  string
		floor float64
		ok    bool
	}{
		{"floor within the bottom's displacement declines", 1e-6, false},
		{"floor clear of the bottom's displacement builds", 1e-4, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			tool := toolRecord(t, 2, 8, tc.floor-plateTop, 0, nil)
			_, ok, err := stackedrecord.BuildBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()), plate, tool)
			require.NoError(t, err)
			require.Equal(t, tc.ok, ok)
		})
	}
}
