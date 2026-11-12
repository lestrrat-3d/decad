package parts_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const (
	nameCarrier = "carrier"
	nameShaft   = "shaft"
	nameBearing = "bearing"
	nameHousing = "housing"
	nameNut     = "nut"
	nameBolt    = "bolt"
	nameCup     = "cup"
	nameFloor   = "floor"
)

func must[T any](v T, err error) T {
	if err != nil {
		panic(err)
	}
	return v
}

// collarSet is a carrier plate (z 16..20, bore radius 3), a shaft (radius 2,
// z 0..30) with a radius 5 collar at z 12..14, and a bearing (radius 6, bore
// 2.5, z 8..11) between the collar and the lower end. The collar is wider than
// the carrier's bore and the bearing's bore, so the shaft leaves only downward
// and only once the bearing has gone.
func collarSet(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	cut := func(b, tool *decad.Body) *decad.Body {
		out, err := decad.Cut(t.Context(), b, tool)
		require.NoError(t, err)
		return out
	}
	carrier := cut(block(t, doc, 0, 0, 20, 20, 16, 4), cylinder(t, doc, 10, 3, 15, 6))
	shaft, err := decad.Union(t.Context(), cylinder(t, doc, 10, 2, 0, 30), cylinder(t, doc, 10, 5, 12, 2))
	require.NoError(t, err)
	bearing := cut(cylinder(t, doc, 10, 6, 8, 3), cylinder(t, doc, 10, 2.5, 7, 5))
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameCarrier, Kind: parts.Manufactured, Body: carrier},
		parts.Part{Name: nameShaft, Kind: parts.Manufactured, Body: shaft},
		parts.Part{Name: nameBearing, Kind: parts.Purchased, Body: bearing})
	mustJoin(t, s,
		parts.Joint{Name: "shaft~carrier", Kind: parts.PinInBore, A: nameShaft, B: nameCarrier, Motion: parts.Turns},
		parts.Joint{Name: "shaft~bearing", Kind: parts.PinInBore, A: nameShaft, B: nameBearing, Motion: parts.Turns})
	return s
}

func offsetZ(t *testing.T, ex *parts.Exploded, name string) float64 {
	t.Helper()
	return partByName(t, ex, name).Offset.Z
}

func TestDerivedCollarShaft(t *testing.T) {
	s := collarSet(t)
	ex, err := s.Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameCarrier))
	require.NoError(t, err)
	bearing, shaft, carrier := partByName(t, ex, nameBearing), partByName(t, ex, nameShaft), partByName(t, ex, nameCarrier)

	// The bearing leaves first, downward, and the shaft follows the same way. The shaft, removed last,
	// sits nearest the carrier and the bearing ends beyond it.
	require.Equal(t, "removal 1", bearing.Stage)
	require.Equal(t, "removal 2", shaft.Stage)
	require.InDelta(t, -1, bearing.Dir.Z, 1e-9)
	require.InDelta(t, -1, shaft.Dir.Z, 1e-9)
	require.Equal(t, r3.Vec{}, carrier.Offset)
	require.Less(t, offsetZ(t, ex, nameShaft), 0.0)
	require.Less(t, offsetZ(t, ex, nameBearing), offsetZ(t, ex, nameShaft))
	for _, p := range []parts.ExplodedPart{bearing, shaft} {
		require.InDelta(t, 0, p.Offset.X, 1e-9, p.Name)
		require.InDelta(t, 0, p.Offset.Y, 1e-9, p.Name)
	}
	// The shaft clears the carrier, and the bearing the shaft's moved box, by the gap: the round removed
	// last runs first.
	require.Equal(t, []string{"removal 2", "removal 1"}, ex.Stages)
	require.Empty(t, ex.Overlaps)

	// The same query twice gives the same offsets.
	again, err := s.Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameCarrier))
	require.NoError(t, err)
	for _, p := range ex.Parts {
		require.Equal(t, p.Offset, partByName(t, again, p.Name).Offset, p.Name)
	}
}

// pressFitSet is a plate with a bore 0.0005 mm wider than the pin in it, a gap
// far below twice the mesh bound.
func pressFitSet(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlateR(t, doc, 2.5005)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 2.5, -5, 20)})
	return s
}

func TestDerivedPressFitDoesNotDeadlock(t *testing.T) {
	ex, err := pressFitSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	pin := partByName(t, ex, namePin)
	require.InDelta(t, 1, pin.Dir.Z, 1e-9)
	require.Greater(t, pin.Offset.Z, 15.0)
	require.InDelta(t, 0, pin.Offset.X, 1e-9)
}

// capturedSet is a housing with a closed pocket, a nut in the pocket that does
// not fit the bores, and a base under the housing. A T-bolt (head in a second
// pocket, stem through a bore) joins the same housing.
func capturedSet(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	cut := func(b, tool *decad.Body) *decad.Body {
		out, err := decad.Cut(t.Context(), b, tool)
		require.NoError(t, err)
		return out
	}
	housing := block(t, doc, 0, 0, 20, 20, 0, 10)
	housing = cut(housing, block(t, doc, 7, 7, 13, 13, 3, 4))
	housing = cut(housing, cylinder(t, doc, 10, 2, -1, 5))
	housing = cut(housing, cylinder(t, doc, 10, 2, 6, 5))
	nut := cut(block(t, doc, 8, 8, 12, 12, 4, 2), cylinder(t, doc, 10, 1, 3, 4))
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "base", Kind: parts.Manufactured, Body: block(t, doc, -5, -5, 25, 25, -5, 5)},
		parts.Part{Name: nameHousing, Kind: parts.Manufactured, Body: housing},
		parts.Part{Name: nameNut, Kind: parts.Purchased, Body: nut})
	return s
}

func TestDerivedCapturedNutGroupsWithHousing(t *testing.T) {
	ex, err := capturedSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround("base"))
	require.NoError(t, err)
	h, n := partByName(t, ex, nameHousing), partByName(t, ex, nameNut)
	require.Equal(t, h.Stage, n.Stage)
	require.Equal(t, h.Offset, n.Offset)
	require.InDelta(t, 1, h.Dir.Z, 1e-9)
	require.Greater(t, h.Offset.Z, 0.0)
}

// tBoltSet is a cup with a closed pocket holding the head of a T-bolt whose
// stem passes through a narrower bore: head and cup block each other whichever
// way they move.
func tBoltSet(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	cup := must(decad.Cut(t.Context(), block(t, doc, 0, 0, 20, 20, 0, 12), block(t, doc, 6, 6, 14, 14, 3, 5)))
	cup = must(decad.Cut(t.Context(), cup, cylinder(t, doc, 10, 2, -1, 5)))
	cup = must(decad.Cut(t.Context(), cup, cylinder(t, doc, 10, 2, 7, 6)))
	bolt := must(decad.Union(t.Context(), block(t, doc, 7, 7, 13, 13, 4, 3), cylinder(t, doc, 10, 1, 0.5, 16.5)))
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameFloor, Kind: parts.Manufactured, Body: block(t, doc, -5, -5, 25, 25, -6, 6)},
		parts.Part{Name: nameCup, Kind: parts.Manufactured, Body: cup},
		parts.Part{Name: nameBolt, Kind: parts.Manufactured, Body: bolt})
	return s
}

func TestDerivedMutualBlockingGroupsAndTerminates(t *testing.T) {
	ex, err := tBoltSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameFloor))
	require.NoError(t, err)
	cup, bolt := partByName(t, ex, nameCup), partByName(t, ex, nameBolt)
	require.Equal(t, cup.Stage, bolt.Stage)
	require.Equal(t, cup.Offset, bolt.Offset)
	require.Greater(t, cup.Offset.Z, 0.0)
	require.Len(t, ex.Stages, 1)
}

func TestDerivedNoFreeDirectionNamesBlockers(t *testing.T) {
	// With the housing as the ground, the captured nut cannot leave in any direction.
	_, err := capturedSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameHousing))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, `part "nut" has no free direction`)
	require.ErrorContains(t, err, `by "housing"`)
}

func TestDerivedOffByDefault(t *testing.T) {
	ex, err := collarSet(t).Explode(t.Context(), parts.WithExplodeGround(nameCarrier))
	require.NoError(t, err)
	require.Equal(t, []string{""}, ex.Stages)
	for _, p := range ex.Parts {
		require.Equal(t, "", p.Stage, p.Name)
	}
	_, err = collarSet(t).Explode(t.Context(), parts.WithRemoval(nameShaft, parts.Removal{Group: "g"}))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "WithDerivedExplode")
}

func TestDerivedExplicitStagesRunFirst(t *testing.T) {
	ex, err := collarSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameCarrier),
		parts.WithExplodeStages(parts.Stage{Name: "bearing off", Parts: []string{nameBearing}, Clear: parts.ClearAll,
			Direction: r3.Vec{Z: -1}}))
	require.NoError(t, err)
	require.Equal(t, []string{"bearing off", "removal 1"}, ex.Stages)
	require.Equal(t, "bearing off", partByName(t, ex, nameBearing).Stage)
	shaft := partByName(t, ex, nameShaft)
	require.Equal(t, "removal 1", shaft.Stage)
	require.InDelta(t, -1, shaft.Dir.Z, 1e-9)
	require.Less(t, shaft.Offset.Z, offsetZ(t, ex, nameBearing))
}

func TestDerivedRemovalConstraints(t *testing.T) {
	ground := parts.WithExplodeGround(nameCarrier)
	derive := parts.WithDerivedExplode()
	explode := func(opts ...parts.ExplodeOption) (*parts.Exploded, error) {
		return collarSet(t).Explode(t.Context(), append([]parts.ExplodeOption{derive, ground}, opts...)...)
	}

	t.Run("after is honoured", func(t *testing.T) {
		ex, err := explode(parts.WithRemoval(nameShaft, parts.Removal{After: []string{nameBearing}}))
		require.NoError(t, err)
		require.Equal(t, "removal 2", partByName(t, ex, nameShaft).Stage)
	})
	t.Run("before is honoured", func(t *testing.T) {
		ex, err := explode(parts.WithRemoval(nameBearing, parts.Removal{Before: []string{nameShaft}}))
		require.NoError(t, err)
		require.Equal(t, "removal 1", partByName(t, ex, nameBearing).Stage)
	})
	t.Run("before that contradicts the blocking is a conflict", func(t *testing.T) {
		_, err := explode(parts.WithRemoval(nameShaft, parts.Removal{Before: []string{nameBearing}}))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, `part "bearing" must leave after "shaft"`)
	})
	t.Run("order cycle", func(t *testing.T) {
		_, err := explode(
			parts.WithRemoval(nameShaft, parts.Removal{After: []string{nameBearing}}),
			parts.WithRemoval(nameBearing, parts.Removal{After: []string{nameShaft}}))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "cycle")
	})
	t.Run("direction is honoured", func(t *testing.T) {
		ex, err := explode(parts.WithRemoval(nameBearing, parts.Removal{Direction: r3.Vec{Z: -7}}))
		require.NoError(t, err)
		require.InDelta(t, -1, partByName(t, ex, nameBearing).Dir.Z, 1e-9)
	})
	t.Run("direction that is blocked is an error", func(t *testing.T) {
		_, err := explode(parts.WithRemoval(nameBearing, parts.Removal{Direction: r3.Vec{Z: 1}}))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, `part "bearing"`)
		require.ErrorContains(t, err, `"carrier", "shaft"`)
	})
	t.Run("group moves together", func(t *testing.T) {
		ex, err := explode(
			parts.WithRemoval(nameBearing, parts.Removal{Group: "stack"}),
			parts.WithRemoval(nameShaft, parts.Removal{Group: "stack"}))
		require.NoError(t, err)
		b, sh := partByName(t, ex, nameBearing), partByName(t, ex, nameShaft)
		require.Equal(t, b.Stage, sh.Stage)
		require.Equal(t, b.Offset, sh.Offset)
		require.Less(t, b.Offset.Z, 0.0)
	})
	t.Run("group with different directions", func(t *testing.T) {
		_, err := explode(
			parts.WithRemoval(nameBearing, parts.Removal{Group: "stack", Direction: r3.Vec{Z: -1}}),
			parts.WithRemoval(nameShaft, parts.Removal{Group: "stack", Direction: r3.Vec{Z: 1}}))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "different directions")
	})
	t.Run("bad names", func(t *testing.T) {
		for _, r := range []struct {
			part string
			r    parts.Removal
		}{
			{"nobody", parts.Removal{}},
			{nameShaft, parts.Removal{After: []string{"nobody"}}},
			{nameShaft, parts.Removal{Before: []string{nameCarrier}}}, // the ground
			{nameCarrier, parts.Removal{}},
			{nameShaft, parts.Removal{After: []string{nameShaft}}},
		} {
			_, err := explode(parts.WithRemoval(r.part, r.r))
			require.ErrorIs(t, err, decad.ErrDegenerate, "%s %v", r.part, r.r)
		}
	})
	t.Run("non-finite direction", func(t *testing.T) {
		_, err := explode(parts.WithRemoval(nameShaft, parts.Removal{Direction: r3.Vec{X: math.NaN()}}))
		require.ErrorIs(t, err, decad.ErrNotFinite)
	})
}
