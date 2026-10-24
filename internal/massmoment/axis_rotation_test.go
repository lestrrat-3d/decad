package massmoment_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const oraclePrec = 300

func bf(v float64) *big.Float { return new(big.Float).SetPrec(oraclePrec).SetFloat64(v) }

func mulf(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(oraclePrec).Mul(a, b) }
func subf(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(oraclePrec).Sub(a, b) }
func addf(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(oraclePrec).Add(a, b) }

// fittedRotation is the exact rotation about the unit vector along a whose
// angle is the one M itself turns by: cos = (tr M - 1)/2 and sin = u·axial(M),
// normalized so that cos² + sin² = 1. It is a candidate for the exact rotation
// Q that AxisRotation's charge covers, built in 300-bit arithmetic.
func fittedRotation(a r3.Vec, rot [3][3]*big.Rat) [3][3]*big.Float {
	norm := new(big.Float).SetPrec(oraclePrec)
	for _, c := range []float64{a.X, a.Y, a.Z} {
		norm.Add(norm, mulf(bf(c), bf(c)))
	}
	norm.Sqrt(norm)
	u := [3]*big.Float{}
	for i, c := range []float64{a.X, a.Y, a.Z} {
		u[i] = new(big.Float).SetPrec(oraclePrec).Quo(bf(c), norm)
	}
	var m [3][3]*big.Float
	for i := range 3 {
		for j := range 3 {
			m[i][j] = new(big.Float).SetPrec(oraclePrec).SetRat(rot[i][j])
		}
	}
	half := bf(0.5)
	cos := mulf(subf(addf(addf(m[0][0], m[1][1]), m[2][2]), bf(1)), half)
	axial := [3]*big.Float{mulf(subf(m[2][1], m[1][2]), half), mulf(subf(m[0][2], m[2][0]), half),
		mulf(subf(m[1][0], m[0][1]), half)}
	sin := addf(addf(mulf(u[0], axial[0]), mulf(u[1], axial[1])), mulf(u[2], axial[2]))
	n := new(big.Float).SetPrec(oraclePrec).Sqrt(addf(mulf(cos, cos), mulf(sin, sin)))
	cos = new(big.Float).SetPrec(oraclePrec).Quo(cos, n)
	sin = new(big.Float).SetPrec(oraclePrec).Quo(sin, n)
	oneMinus := subf(bf(1), cos)
	skew := [3][3]*big.Float{
		{bf(0), new(big.Float).Neg(u[2]), u[1]},
		{u[2], bf(0), new(big.Float).Neg(u[0])},
		{new(big.Float).Neg(u[1]), u[0], bf(0)},
	}
	var q [3][3]*big.Float
	for i := range 3 {
		for j := range 3 {
			v := addf(mulf(mulf(u[i], u[j]), oneMinus), mulf(sin, skew[i][j]))
			if i == j {
				v = addf(v, cos)
			}
			q[i][j] = v
		}
	}
	return q
}

// displacement is the Frobenius norm of M − Q for the fitted exact rotation Q
// about a, in float64. The Frobenius norm is at least the spectral norm, so a
// charge covering it covers the maximum of |Mp − Qp| over unit p.
func displacement(t *testing.T, m r3.Transform, a r3.Vec) float64 {
	t.Helper()
	rot, err := massmoment.PlacementRotation(m)
	require.NoError(t, err)
	q := fittedRotation(a, rot)
	sum := new(big.Float).SetPrec(oraclePrec)
	for i := range 3 {
		for j := range 3 {
			mv := new(big.Float).SetPrec(oraclePrec).SetRat(rot[i][j])
			d := subf(mv, q[i][j])
			sum.Add(sum, mulf(d, d))
		}
	}
	out, _ := sum.Sqrt(sum).Float64()
	return out
}

func normalize(v r3.Vec) r3.Vec {
	n := math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
	return r3.Vec{X: v.X / n, Y: v.Y / n, Z: v.Z / n}
}

func rotation(t *testing.T, axis r3.Vec, radians float64) r3.Transform {
	t.Helper()
	m, err := r3.Rotation(axis, units.Radians(radians))
	require.NoError(t, err)
	return m
}

// tilted returns the unit axis a moved by tilt (a relative size) toward its
// in-plane normal about e_z: a coordinate-plane axis with a zero z component.
func tilted(a r3.Vec, tilt float64) r3.Vec {
	unit := normalize(a)
	return normalize(r3.Vec{X: unit.X - tilt*unit.Y, Y: unit.Y + tilt*unit.X, Z: unit.Z})
}

func axisOf(t *testing.T, v r3.Vec) coneaxis.Axis {
	t.Helper()
	axis, ok := coneaxis.New(v)
	require.True(t, ok)
	return axis
}

func TestAxisRotationCanonicalIsXAxisRotation(t *testing.T) {
	t.Parallel()
	x := coneaxis.X()
	for _, m := range []r3.Transform{r3.Identity(), rotation(t, r3.Vec{X: 1}, math.Pi/3),
		rotation(t, r3.Vec{X: 1}, 2*math.Pi*5/16)} {
		charge, eps, ok := massmoment.AxisRotation(m, x)
		require.True(t, ok)
		require.True(t, massmoment.XAxisRotation(m))
		want, err := massmoment.XAxisPlacementCharge(m)
		require.NoError(t, err)
		require.Equal(t, want, charge)
		require.Equal(t, charge.Stretch, eps, "the canonical charge is exactly the stretch")
	}
	shift, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	for name, m := range map[string]r3.Transform{
		"about Z": rotation(t, r3.Vec{Z: 1}, math.Pi/3), "translated": shift,
	} {
		_, _, ok := massmoment.AxisRotation(m, x)
		require.False(t, ok, name)
	}
}

func TestAxisRotationAdmitsRoundedRotationsAndCoversTheirDisplacement(t *testing.T) {
	t.Parallel()
	for _, a := range []r3.Vec{{X: 1, Y: 2}, {X: 3, Y: 4}, {X: 1, Z: 2}, {Y: 1}} {
		for _, angle := range []struct {
			name    string
			radians float64
		}{{"pi", math.Pi}, {"pi/2", math.Pi / 2}, {"pi/3", math.Pi / 3}} {
			m := rotation(t, normalize(a), angle.radians)
			charge, eps, ok := massmoment.AxisRotation(m, axisOf(t, a))
			require.True(t, ok, "%v by %s", a, angle.name)
			require.GreaterOrEqual(t, eps, charge.Stretch)
			got := displacement(t, m, a)
			require.LessOrEqual(t, got, eps, "%v by %s: displacement %g exceeds the charge %g", a, angle.name, got, eps)
		}
	}
}

// A rotation about an axis a few units in the last place off A moves the
// stretch charge e only a little, but its displacement from the exact rotation
// about A is dominated by the drift. The charge must carry the s_φ leg.
func TestAxisRotationChargesDriftLeg(t *testing.T) {
	t.Parallel()
	a := r3.Vec{X: 1, Y: 2}
	m := rotation(t, tilted(a, 0x1p-48), math.Pi)
	charge, eps, ok := massmoment.AxisRotation(m, axisOf(t, a))
	require.True(t, ok)
	got := displacement(t, m, a)
	withoutDrift := charge.Stretch + (math.Pi/2)*charge.Stretch
	require.Greater(t, got, withoutDrift, "the fixture's drift must exceed the stretch-only charge")
	require.LessOrEqual(t, got, eps)
	require.Greater(t, eps, 1.2*withoutDrift)
}

func TestAxisRotationDriftLimit(t *testing.T) {
	t.Parallel()
	a := r3.Vec{X: 1, Y: 2}
	axis := axisOf(t, a)
	// The tilt of 2^-40 moves the axis image by a sine near 1.4·2^-40.
	_, _, ok := massmoment.AxisRotation(rotation(t, tilted(a, 0x1p-40), math.Pi), axis)
	require.False(t, ok, "a tilt of 2^-40 must decline")
	_, _, ok = massmoment.AxisRotation(rotation(t, tilted(a, 0x1p-48), math.Pi), axis)
	require.True(t, ok, "a tilt of 2^-48 must pass")
	// The limit is a bound on the proven sine: 2^-46 is 64 ulp of 1.
	require.Equal(t, 64*0x1p-52, massmoment.DriftSinLimit)
}

func TestAxisRotationDeclines(t *testing.T) {
	t.Parallel()
	a := r3.Vec{X: 1, Y: 2}
	axis := axisOf(t, a)
	shift, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	mirror, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	reflect, err := r3.Reflection(mirror)
	require.NoError(t, err)
	for name, m := range map[string]r3.Transform{
		"translated": shift,
		"reflected":  reflect,
		// Perpendicular to A in the plane: an obtuse copy sending A to -A.
		"obtuse":   rotation(t, normalize(r3.Vec{X: 2, Y: -1}), math.Pi),
		"about X":  rotation(t, r3.Vec{X: 1}, math.Pi/3),
		"about Z":  rotation(t, r3.Vec{Z: 1}, math.Pi),
		"zero":     {},
		"coarse A": rotation(t, tilted(a, 0.01), math.Pi/2),
	} {
		_, _, ok := massmoment.AxisRotation(m, axis)
		require.False(t, ok, name)
	}
	// A rotation about A is still admitted for a longer A: the test is
	// homogeneous in |A|.
	_, _, ok := massmoment.AxisRotation(rotation(t, normalize(a), math.Pi/3), axisOf(t, r3.Vec{X: 6, Y: 12}))
	require.True(t, ok)
	// The identity has A mapped to itself.
	charge, eps, ok := massmoment.AxisRotation(r3.Identity(), axis)
	require.True(t, ok)
	require.Equal(t, massmoment.MapCharge{}, charge)
	require.Zero(t, eps)
}
