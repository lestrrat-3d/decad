package spiralsector_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestNearAxisRotation(t *testing.T) {
	t.Parallel()
	aboutX, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(30))
	require.NoError(t, err)
	aboutZ, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)

	require.True(t, spiralsector.NearAxisRotation(aboutX))
	require.False(t, spiralsector.NearAxisRotation(r3.Identity()))
	require.False(t, spiralsector.NearAxisRotation(aboutZ))
	require.False(t, spiralsector.NearAxisRotation(shift))
}

func TestAxisCopyMapPreservesConeTangentAndStrictGap(t *testing.T) {
	t.Parallel()
	record, _, err := exactrotation.New(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(37))
	require.NoError(t, err)
	m, ok := spiralsector.AxisCopyMap(record)
	require.True(t, ok)
	turn := func(v [2]*big.Rat) [2]*big.Rat {
		return [2]*big.Rat{
			new(big.Rat).Add(new(big.Rat).Mul(m[1][1], v[0]), new(big.Rat).Mul(m[1][2], v[1])),
			new(big.Rat).Add(new(big.Rat).Mul(m[2][1], v[0]), new(big.Rat).Mul(m[2][2], v[1])),
		}
	}
	dot := func(a, b [2]*big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]),
			new(big.Rat).Mul(a[1], b[1]))
	}
	e := [2]*big.Rat{big.NewRat(3, 5), big.NewRat(4, 5)}
	p := [2]*big.Rat{big.NewRat(6, 5), big.NewRat(8, 5)}
	turnedE, turnedP := turn(e), turn(p)
	require.Zero(t, m[0][0].Cmp(big.NewRat(1, 1)))
	require.Zero(t, dot(turnedE, turnedE).Cmp(big.NewRat(1, 1)))
	require.Zero(t, dot(turnedP, turnedP).Cmp(big.NewRat(4, 1)))
	require.Zero(t, dot(turnedE, turnedP).Cmp(big.NewRat(2, 1)))
	// With apex 10 and slope 2, x=6 is the tangent contact. At x=5 the
	// same radial point has a strict one-unit source-side gap after rotation.
	contact := new(big.Rat).Add(big.NewRat(6, 1),
		new(big.Rat).Mul(big.NewRat(2, 1), dot(turnedE, turnedP)))
	gap := new(big.Rat).Sub(big.NewRat(10, 1),
		new(big.Rat).Add(big.NewRat(5, 1),
			new(big.Rat).Mul(big.NewRat(2, 1), dot(turnedE, turnedP))))
	require.Zero(t, contact.Cmp(big.NewRat(10, 1)))
	require.Zero(t, gap.Cmp(big.NewRat(1, 1)))

	other, _, err := exactrotation.New(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(37))
	require.NoError(t, err)
	_, ok = spiralsector.AxisCopyMap(other)
	require.False(t, ok)
}
