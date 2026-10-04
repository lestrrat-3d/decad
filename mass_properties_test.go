package decad_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestMassPropertiesSourceBox(t *testing.T) {
	doc := decad.New()
	box := boxBodyAtZ(t, doc, 0, 0, 10, 10, 10, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	got, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, units.Mass, got.Mass.Value.Kind())
	require.Equal(t, units.Length, got.Center.Bound.Kind())
	require.InDelta(t, 1, got.Mass.Value.Base(), 1e-14)
	exactMass := new(big.Rat).Mul(new(big.Rat).SetFloat64(density.Mag()), big.NewRat(1000, 1))
	requireReadingCovers(t, got.Mass, exactMass)
	require.InDelta(t, 5, got.Center.Value.X, 1e-12)
	require.InDelta(t, 5, got.Center.Value.Y, 1e-12)
	require.InDelta(t, 15, got.Center.Value.Z, 1e-12)
	for _, entry := range []decad.Measurement{got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ} {
		require.Equal(t, units.MomentOfInertia, entry.Value.Kind())
		require.InDelta(t, 100.0/6, entry.Value.Base(), 1e-12)
		exactInertia := new(big.Rat).Mul(exactMass, big.NewRat(50, 3))
		requireReadingCovers(t, entry, exactInertia)
		require.Less(t, entry.Bound.Base(), entry.Value.Base())
	}
	for _, entry := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		require.Equal(t, units.MomentOfInertia, entry.Value.Kind())
		require.Equal(t, decad.Exact, entry.Exactness)
		require.Zero(t, entry.Value.Base())
		require.Zero(t, entry.Bound.Base())
	}

	translation, err := r3.Translation(r3.NewVec(3, -4, 7))
	require.NoError(t, err)
	placed, err := box.Placed(t.Context(), translation)
	require.NoError(t, err)
	shifted, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 8, shifted.Center.Value.X, 1e-12)
	require.InDelta(t, 1, shifted.Center.Value.Y, 1e-12)
	require.InDelta(t, 22, shifted.Center.Value.Z, 1e-12)
	require.Equal(t, got.Mass, shifted.Mass)
	require.Equal(t, got.Inertia, shifted.Inertia)
	// Placing the box retired its source. MassProperties remains a read-only
	// geometry query on that source, just like Volume and Centroid.
	again, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)

	// A non-cube distinguishes the three tensor entries and checks that an
	// exact cardinal rotation reorders them without changing mass.
	rectangular := boxBody(t, decad.New(), 0, 0, 20, 10, 30)
	base, err := rectangular.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 6, base.Mass.Value.Base(), 1e-13)
	require.InDelta(t, 500, base.Inertia.XX.Value.Base(), 1e-11)
	require.InDelta(t, 650, base.Inertia.YY.Value.Base(), 1e-11)
	require.InDelta(t, 250, base.Inertia.ZZ.Value.Base(), 1e-11)
	quarterTurn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 1, 0),
		EY: r3.NewVec(-1, 0, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(50, 0, 0))
	require.NoError(t, err)
	turned, err := rectangular.Placed(t.Context(), quarterTurn)
	require.NoError(t, err)
	rotated, err := turned.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, base.Mass, rotated.Mass)
	require.Equal(t, base.Inertia.XX, rotated.Inertia.YY)
	require.Equal(t, base.Inertia.YY, rotated.Inertia.XX)
	require.Equal(t, base.Inertia.ZZ, rotated.Inertia.ZZ)
}

func requireReadingCovers(t *testing.T, reading decad.Measurement, exact *big.Rat) {
	t.Helper()
	held := new(big.Rat).SetFloat64(reading.Value.Base())
	bound := new(big.Rat).SetFloat64(reading.Bound.Base())
	deviation := new(big.Rat).Abs(new(big.Rat).Sub(exact, held))
	require.LessOrEqual(t, deviation.Cmp(bound), 0)
}

func TestMassPropertiesRefusals(t *testing.T) {
	box := boxBody(t, decad.New(), 0, 0, 10, 10, 10)
	for _, input := range []struct {
		name    string
		density units.Value
		want    error
	}{
		{"wrong kind", units.Kilograms(1), decad.ErrUnitKind},
		{"negative", units.KilogramsPerCubicMillimeter(-1), decad.ErrNegativeMagnitude},
		{"zero", units.KilogramsPerCubicMillimeter(0), decad.ErrDegenerate},
		{"nonfinite", units.KilogramsPerCubicMillimeter(math.Inf(1)), decad.ErrNotFinite},
	} {
		t.Run(input.name, func(t *testing.T) {
			got, err := box.MassProperties(t.Context(), input.density)
			require.ErrorIs(t, err, input.want)
			require.Equal(t, decad.MassProperties{}, got)
		})
	}
	var missing *decad.Body
	got, err := missing.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, decad.MassProperties{}, got)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err = box.MassProperties(ctx, units.KilogramsPerCubicMillimeter(1))
	require.True(t, errors.Is(err, context.Canceled))
	require.Equal(t, decad.MassProperties{}, got)

	ball := ballBody(t, decad.New(), 10)
	got, err = ball.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, decad.MassProperties{}, got)
}
