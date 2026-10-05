package decad_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFacetedMassPropertiesUnionAndRefusals(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -5, -5, 5, 5, 0, 10)
	cap := boxBodyAtZ(t, doc, -2, -2, 2, 2, 8, 4)
	union, err := decad.Union(t.Context(), base, cap)
	require.NoError(t, err)
	mesh, err := union.Tessellate(t.Context(), units.Millimeters(1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Zero(t, mesh.Bound().Base())
	density := units.KilogramsPerCubicMillimeter(.001)
	got, err := union.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, units.Mass, got.Mass.Value.Kind())
	require.Equal(t, units.Length, got.Center.Bound.Kind())
	rho := new(big.Rat).SetFloat64(density.Mag())
	wantMass := new(big.Rat).Mul(rho, big.NewRat(1032, 1))
	requireReadingCovers(t, got.Mass, wantMass)
	require.Equal(t, 0.0, got.Center.Value.X)
	require.Equal(t, 0.0, got.Center.Value.Y)
	wantCenter := big.NewRat(223, 43)
	centerError := new(big.Rat).Sub(wantCenter, new(big.Rat).SetFloat64(got.Center.Value.Z))
	centerError.Abs(centerError)
	require.LessOrEqual(t, centerError.Cmp(new(big.Rat).SetFloat64(got.Center.Bound.Base())), 0)

	// Inclusion-exclusion of three axis-aligned boxes independently checks
	// the mesh integration. The overlap occupies z=[8,10].
	boxIzz := func(volume, width int64) *big.Rat {
		factor := new(big.Rat).Mul(big.NewRat(width, 1), big.NewRat(width, 1))
		factor.Mul(factor, big.NewRat(2, 12))
		factor.Mul(factor, big.NewRat(volume, 1))
		return new(big.Rat).Mul(rho, factor)
	}
	wantIzz := new(big.Rat).Add(boxIzz(1000, 10), boxIzz(64, 4))
	wantIzz.Sub(wantIzz, boxIzz(32, 4))
	requireReadingCovers(t, got.Inertia.ZZ, wantIzz)
	boxIxx := func(volume, width, height, centerZ int64) *big.Rat {
		dims := new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(width, 1), big.NewRat(width, 1)),
			new(big.Rat).Mul(big.NewRat(height, 1), big.NewRat(height, 1)))
		dims.Quo(dims, big.NewRat(12, 1))
		shift := new(big.Rat).Sub(big.NewRat(centerZ, 1), wantCenter)
		dims.Add(dims, new(big.Rat).Mul(shift, shift))
		return new(big.Rat).Mul(new(big.Rat).Mul(rho, big.NewRat(volume, 1)), dims)
	}
	wantIxx := new(big.Rat).Add(boxIxx(1000, 10, 10, 5), boxIxx(64, 4, 4, 10))
	wantIxx.Sub(wantIxx, boxIxx(32, 4, 2, 9))
	requireReadingCovers(t, got.Inertia.XX, wantIxx)
	requireReadingCovers(t, got.Inertia.YY, wantIxx)
	for _, component := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		requireReadingCovers(t, component, big.NewRat(0, 1))
	}
	again, err := union.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)

	shift, err := r3.Translation(r3.Vec{X: .1})
	require.NoError(t, err)
	placed, err := union.Placed(t.Context(), shift)
	require.NoError(t, err)
	reading, err := placed.MassProperties(t.Context(), density)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, decad.MassProperties{}, reading)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	reading, err = union.MassProperties(canceled, density)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, decad.MassProperties{}, reading)
}

func TestFacetedMassPropertiesLumpsAndVoid(t *testing.T) {
	density := units.KilogramsPerCubicMillimeter(.001)
	rho := new(big.Rat).SetFloat64(density.Mag())
	t.Run("disjoint lumps", func(t *testing.T) {
		doc := decad.New()
		a := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
		b := boxBodyAtZ(t, doc, 20, 20, 30, 30, 0, 10)
		union, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		require.Len(t, union.Lumps(), 2)
		got, err := union.MassProperties(t.Context(), density)
		require.NoError(t, err)
		requireReadingCovers(t, got.Mass, new(big.Rat).Mul(rho, big.NewRat(2000, 1)))
		require.Equal(t, r3.Vec{X: 15, Y: 15, Z: 5}, got.Center.Value)
		for _, reading := range []decad.Measurement{got.Inertia.XX, got.Inertia.YY} {
			requireReadingCovers(t, reading,
				new(big.Rat).Mul(rho, big.NewRat(700000, 3)))
		}
		requireReadingCovers(t, got.Inertia.ZZ,
			new(big.Rat).Mul(rho, big.NewRat(1300000, 3)))
		requireReadingCovers(t, got.Inertia.XY,
			new(big.Rat).Mul(rho, big.NewRat(-200000, 1)))
		requireReadingCovers(t, got.Inertia.XZ, big.NewRat(0, 1))
		requireReadingCovers(t, got.Inertia.YZ, big.NewRat(0, 1))
	})
	t.Run("internal void", func(t *testing.T) {
		doc := decad.New()
		outer := boxBodyAtZ(t, doc, 0, 0, 20, 20, 0, 8)
		tool := boxBodyAtZ(t, doc, 8, 8, 12, 12, 2, 4)
		hollow, err := decad.Cut(t.Context(), outer, tool)
		require.NoError(t, err)
		got, err := hollow.MassProperties(t.Context(), density)
		require.NoError(t, err)
		requireReadingCovers(t, got.Mass, new(big.Rat).Mul(rho, big.NewRat(3136, 1)))
		require.Equal(t, r3.Vec{X: 10, Y: 10, Z: 4}, got.Center.Value)
		wantIxx := new(big.Rat).Mul(rho, big.NewRat(370688, 3))
		requireReadingCovers(t, got.Inertia.XX, wantIxx)
		requireReadingCovers(t, got.Inertia.YY, wantIxx)
		wantIzz := new(big.Rat).Mul(rho, big.NewRat(639488, 3))
		requireReadingCovers(t, got.Inertia.ZZ, wantIzz)
	})
}
