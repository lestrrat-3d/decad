package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The signed mesh volume must lie within the body's analytic interval plus
// the occupied-volume allowance that VerifyAll publishes. This caught a
// reversed quarter-circle integration while the mesh itself was closed.
func TestShellStackedBossMeshVolumeProof(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name            string
		closed, reverse bool
	}{
		{name: "top open"},
		{name: "closed cavity", closed: true},
		{name: "closed reversed cavity", closed: true, reverse: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			var plate, boss *Body
			if tc.reverse {
				boss = internalBoxBody(t, doc, -5, -4, 3, 6, 5)
				plate = internalBoxBodyAtZ(t, doc, -12, -10, 12, 10, 5, 5)
			} else {
				plate = internalBoxBody(t, doc, -12, -10, 12, 10, 5)
				boss = internalBoxBodyAtZ(t, doc, -5, -4, 3, 6, 5, 5)
			}
			stack, err := Union(t.Context(), plate, boss)
			require.NoError(t, err)
			var shell *Body
			if tc.closed {
				shell, err = stack.Shell(t.Context(), nil, units.Millimeters(1), WithNoOpenings())
			} else {
				shell, err = stack.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(stack))).Exactly(1),
					units.Millimeters(1))
			}
			require.NoError(t, err)
			v, err := shell.Volume()
			require.NoError(t, err)
			mesh, err := tessellateContext(t.Context(), shell, units.Millimeters(0.1), VerifyAll)
			require.NoError(t, err)
			require.True(t, mesh.boundaryOK)
			require.True(t, mesh.symDiffOK)
			delta := new(big.Rat).Sub(internalMeshVolumeRat(mesh), new(big.Rat).SetFloat64(v.Value.Base()))
			delta.Abs(delta)
			deltaHeld, _ := delta.Float64()
			require.LessOrEqual(t, deltaHeld, proofbound.AbsSumUpper(mesh.volSymDiff, v.Bound.Base()))
		})
	}
}
