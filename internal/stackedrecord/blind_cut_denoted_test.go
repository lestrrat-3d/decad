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
	t.Run("an undenoted top admits the held order", func(t *testing.T) {
		t.Parallel()
		_, ok, err := stackedrecord.BuildBlindCut(t.Context(), proofbound.NewWorkBudget(t.Context()), plate, tool)
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
