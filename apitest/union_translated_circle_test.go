package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestUnionTranslatedEqualCirclePrisms(t *testing.T) {
	ctx := t.Context()
	doc := decad.New(decad.ExactOnly())
	a := historyZCylinder(t, doc, 0, 0, 1, 0, 2)
	b, err := decad.Copy(ctx, a)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{Z: 2})
	require.NoError(t, err)
	b, err = decad.Transform(ctx, b, shift)
	require.NoError(t, err)
	before := len(doc.OperationHistory().Operations())
	result, err := decad.Union(ctx, a, b)
	require.NoError(t, err)
	require.Empty(t, result.OperationDiagnostics())
	for _, face := range result.Faces() {
		require.NotEqual(t, decad.KindFaceted, face.Surface().Kind())
	}
	volume, err := result.Volume()
	require.NoError(t, err)
	require.InDelta(t, 4*math.Pi, volume.Value.Base(), volume.Bound.Base()+1e-12)
	mesh, err := result.Tessellate(ctx, units.Millimeters(.01), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	ops := doc.OperationHistory().Operations()
	require.Len(t, ops, before+1)
	require.Equal(t, decad.OperationUnion, ops[before].Kind())
	require.Len(t, ops[before].Inputs(), 2)
	require.Same(t, a, ops[before].Inputs()[0].Body)
	require.Same(t, b, ops[before].Inputs()[1].Body)
	require.Equal(t, decad.InputConsumed, ops[before].Inputs()[0].Use)
	require.Equal(t, decad.InputConsumed, ops[before].Inputs()[1].Use)
	require.Len(t, ops[before].Outputs(), 1)
	require.Same(t, result, ops[before].Outputs()[0])
}
