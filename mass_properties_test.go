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

const testNegative = "negative"

func TestMassPropertiesTrianglePrism(t *testing.T) {
	doc := decad.New()
	sketch, profile := polygonSketch(t, [][2]float64{{0, 0}, {6, 0}, {0, 8}})
	body, err := doc.Extrude(sketch, profile, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(0.001)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 0.24, got.Mass.Value.Base(), 1e-14)
	require.InDelta(t, 2, got.Center.Value.X, 1e-14)
	require.InDelta(t, 8.0/3, got.Center.Value.Y, 1e-14)
	require.InDelta(t, 5, got.Center.Value.Z, 1e-14)
	densityExact := new(big.Rat).SetFloat64(density.Mag())
	massExact := new(big.Rat).Mul(densityExact, big.NewRat(240, 1))
	for _, entry := range []struct {
		reading decad.Measurement
		factor  *big.Rat
	}{
		{got.Inertia.XX, big.NewRat(107, 9)},
		{got.Inertia.YY, big.NewRat(31, 3)},
		{got.Inertia.ZZ, big.NewRat(50, 9)},
		{got.Inertia.XY, big.NewRat(4, 3)},
		{got.Inertia.XZ, big.NewRat(0, 1)},
		{got.Inertia.YZ, big.NewRat(0, 1)},
	} {
		exact := new(big.Rat).Mul(massExact, entry.factor)
		requireReadingCovers(t, entry.reading, exact)
	}
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 1, 0),
		EY: r3.NewVec(-1, 0, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(30, 0, 0))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), turn)
	require.NoError(t, err)
	rotated, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got.Mass, rotated.Mass)
	require.Equal(t, got.Inertia.XX, rotated.Inertia.YY)
	require.Equal(t, got.Inertia.YY, rotated.Inertia.XX)
	require.InDelta(t, -got.Inertia.XY.Value.Base(), rotated.Inertia.XY.Value.Base(), 1e-14)
}

func TestMassPropertiesCircularPrism(t *testing.T) {
	body := circleProfile(t, 3, 10)
	got, err := body.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	mass := math.Pi * 9 * 10 * 0.001
	require.InDelta(t, mass, got.Mass.Value.Base(), 1e-14)
	require.InDelta(t, mass*(27+100)/12, got.Inertia.XX.Value.Base(), 1e-13)
	require.InDelta(t, mass*(27+100)/12, got.Inertia.YY.Value.Base(), 1e-13)
	require.InDelta(t, mass*9/2, got.Inertia.ZZ.Value.Base(), 1e-13)
	require.InDelta(t, 0, got.Inertia.XY.Value.Base(), 1e-14)
}

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

func TestMassPropertiesSourceSphere(t *testing.T) {
	ball := ballBody(t, decad.New(), 5)
	density := units.KilogramsPerCubicMillimeter(0.001)
	got, err := ball.MassProperties(t.Context(), density)
	require.NoError(t, err)
	wantMass := 4 * math.Pi * 125 * density.Base() / 3
	wantInertia := 2 * wantMass * 25 / 5
	require.InDelta(t, wantMass, got.Mass.Value.Base(), 1e-14)
	require.InDelta(t, wantInertia, got.Inertia.XX.Value.Base(), 1e-13)
	require.Equal(t, got.Inertia.XX, got.Inertia.YY)
	require.Equal(t, got.Inertia.XX, got.Inertia.ZZ)
	for _, mixed := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		require.Equal(t, decad.Exact, mixed.Exactness)
		require.Zero(t, mixed.Value.Base())
		require.Zero(t, mixed.Bound.Base())
	}
	pi, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937510582097494459230781640628620899")
	require.True(t, ok)
	exactMass := new(big.Rat).Mul(new(big.Rat).SetFloat64(density.Mag()), big.NewRat(500, 3))
	exactMass.Mul(exactMass, pi)
	requireReadingCovers(t, got.Mass, exactMass)
	exactInertia := new(big.Rat).Mul(exactMass, big.NewRat(10, 1))
	requireReadingCovers(t, got.Inertia.XX, exactInertia)

	pose, err := r3.RotationAround(r3.NewVec(20, -3, 7), r3.NewVec(1, 1, 1), units.Degrees(37))
	require.NoError(t, err)
	placed, err := ball.Placed(t.Context(), pose)
	require.NoError(t, err)
	rotated, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got.Mass, rotated.Mass)
	require.Equal(t, got.Inertia, rotated.Inertia)
	center := pose.Apply(r3.Vec{})
	require.InDelta(t, center.X, rotated.Center.Value.X, rotated.Center.Bound.Base())
	require.InDelta(t, center.Y, rotated.Center.Value.Y, rotated.Center.Bound.Base())
	require.InDelta(t, center.Z, rotated.Center.Value.Z, rotated.Center.Bound.Base())
	again, err := ball.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	partial, err := ball.MassProperties(canceled, density)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, decad.MassProperties{}, partial)
}

func TestMassPropertiesRevolvedCylinder(t *testing.T) {
	doc := decad.New()
	sketch, profile := solidSketch(t)
	body, err := doc.Revolve(sketch, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	pi, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937510582097494459230781640628620899")
	require.True(t, ok)
	mass := new(big.Rat).Mul(new(big.Rat).SetFloat64(density.Mag()), big.NewRat(640, 1))
	mass.Mul(mass, pi)
	requireReadingCovers(t, got.Mass, mass)
	transverse := new(big.Rat).Mul(mass, big.NewRat(73, 3)) // (3·8² + 10²)/12
	axial := new(big.Rat).Mul(mass, big.NewRat(32, 1))      // 8²/2
	requireReadingCovers(t, got.Inertia.XX, axial)
	requireReadingCovers(t, got.Inertia.YY, transverse)
	requireReadingCovers(t, got.Inertia.ZZ, transverse)
	for _, mixed := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		require.Equal(t, decad.Exact, mixed.Exactness)
		require.Zero(t, mixed.Value.Base())
		require.Zero(t, mixed.Bound.Base())
	}
	require.InDelta(t, 5, got.Center.Value.X, got.Center.Bound.Base())
	require.InDelta(t, 0, got.Center.Value.Y, got.Center.Bound.Base())
	require.InDelta(t, 0, got.Center.Value.Z, got.Center.Bound.Base())

	cardinal, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 1, 0), EY: r3.NewVec(-1, 0, 0), EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(2, 3, 4))
	require.NoError(t, err)
	placed, err := body.PlacedCopy(t.Context(), cardinal)
	require.NoError(t, err)
	turned, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got.Mass, turned.Mass)
	require.Equal(t, got.Inertia.XX, turned.Inertia.YY)
	require.Equal(t, got.Inertia.YY, turned.Inertia.XX)
	require.Equal(t, got.Inertia.ZZ, turned.Inertia.ZZ)
	require.InDelta(t, 2, turned.Center.Value.X, turned.Center.Bound.Base())
	require.InDelta(t, 8, turned.Center.Value.Y, turned.Center.Bound.Base())
	require.InDelta(t, 4, turned.Center.Value.Z, turned.Center.Bound.Base())
	again, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)

	sheet, err := doc.Revolve(sketch, profile, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	reading, err := sheet.MassProperties(t.Context(), density)
	require.ErrorIs(t, err, decad.ErrNotSolid)
	require.Equal(t, decad.MassProperties{}, reading)
	obliquePose, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Degrees(37))
	require.NoError(t, err)
	oblique, err := body.PlacedCopy(t.Context(), obliquePose)
	require.NoError(t, err)
	// An oblique placement leaves the source-cylinder path for the general
	// revolve path, whose rotation keeps the mass and, about Z, the
	// transverse ZZ.
	reading, err = oblique.MassProperties(t.Context(), density)
	require.NoError(t, err)
	requireReadingCovers(t, reading.Mass, mass)
	requireReadingCovers(t, reading.Inertia.ZZ, transverse)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	reading, err = body.MassProperties(canceled, density)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, decad.MassProperties{}, reading)
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
		{testNegative, units.KilogramsPerCubicMillimeter(-1), decad.ErrNegativeMagnitude},
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
}
