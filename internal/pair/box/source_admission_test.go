package box_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSignedAxisTransformAdmission(t *testing.T) {
	t.Run("invalid with finite translation", func(t *testing.T) {
		xform := r3.Transform{}
		require.False(t, xform.IsValid())
		require.True(t, proofbound.FiniteVec(xform.Translation()))
		require.False(t, box.SignedAxisTransform(xform))
		// This case cannot distinguish || from && in the initial guard:
		// CardinalBasis also rejects the zero basis. More generally, a
		// signed cardinal basis and finite translation imply r3.IsValid.
	})

	t.Run("identity", func(t *testing.T) {
		require.True(t, box.SignedAxisTransform(r3.Identity()))
	})

	for _, tc := range []struct {
		name  string
		basis r3.Basis
	}{
		{"signed permutation", r3.Basis{
			EX: r3.Vec{Y: -1}, EY: r3.Vec{Z: 1}, EZ: r3.Vec{X: -1},
		}},
		{"reflection", r3.Basis{
			EX: r3.Vec{X: -1}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1},
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			xform, err := r3.FromBasis(tc.basis, r3.Vec{X: 1, Y: -2, Z: 3})
			require.NoError(t, err)
			require.True(t, xform.IsValid())
			require.True(t, box.SignedAxisTransform(xform))
		})
	}

	t.Run("valid noncardinal rotation", func(t *testing.T) {
		xform, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(15))
		require.NoError(t, err)
		require.True(t, xform.IsValid())
		require.False(t, box.SignedAxisTransform(xform))
	})
}
