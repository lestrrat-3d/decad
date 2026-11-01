package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestTranslatedBlindCutKeepsThinFloor(t *testing.T) {
	doc := decad.New(decad.ExactOnly())
	target := boxBody(t, doc, 470, -5, 478, 5, 20)
	tool := historyZCylinder(t, doc, 0, 0, 1.1, 0, 20)
	u := r3.NewVec(0.024051868856736578, -0.31446392750578867, 0.94896467052368849)
	v := r3.NewVec(0.007967648549189436, 0.94926942345028398, 0.31436297218177989)
	frameA, err := r3.NewFrame(r3.NewVec(229.99678960557387, 257.82946876781881, 108.21070606486229), u, v)
	require.NoError(t, err)
	frameB, err := r3.NewFrame(r3.NewVec(241.39737544366702, 108.77356713007498, 558.01995989309057), u, v)
	require.NoError(t, err)
	xa, err := r3.FromFrame(frameA)
	require.NoError(t, err)
	xb, err := r3.FromFrame(frameB)
	require.NoError(t, err)
	target, err = decad.Transform(t.Context(), target, xa)
	require.NoError(t, err)
	tool, err = decad.Transform(t.Context(), tool, xb)
	require.NoError(t, err)
	for _, operation := range []func() (*decad.Body, error){
		func() (*decad.Body, error) {
			a, err := decad.Copy(t.Context(), target)
			require.NoError(t, err)
			b, err := decad.Copy(t.Context(), tool)
			require.NoError(t, err)
			return decad.Intersect(t.Context(), a, b)
		},
		func() (*decad.Body, error) {
			a, err := decad.Copy(t.Context(), target)
			require.NoError(t, err)
			b, err := decad.Copy(t.Context(), tool)
			require.NoError(t, err)
			return decad.Union(t.Context(), a, b)
		},
	} {
		other, err := operation()
		if err != nil {
			require.ErrorIs(t, err, decad.ErrUnsupported)
			continue
		}
		require.NotEqual(t, "decad.stackedPrismPayload", decadtest.PayloadClass(other))
	}
	result, err := decad.Cut(t.Context(), target, tool)
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(result))
	volume, err := result.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1600-20*math.Pi*1.21, volume.Value.Base(), volume.Bound.Base()+1e-6)
	ops := doc.OperationHistory().Operations()
	op := ops[len(ops)-1]
	require.Equal(t, decad.OperationCut, op.Kind())
	require.Len(t, op.Inputs(), 2)
	require.Same(t, target, op.Inputs()[0].Body)
	require.Same(t, tool, op.Inputs()[1].Body)
	require.Equal(t, decad.InputConsumed, op.Inputs()[0].Use)
	require.Equal(t, decad.InputConsumed, op.Inputs()[1].Use)
	require.Len(t, op.Outputs(), 1)
	require.Same(t, result, op.Outputs()[0])
	none, err := result.Tessellate(t.Context(), units.Millimeters(.12), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.True(t, none.BoundaryVerified())
	require.False(t, none.VolumeVerified())
	boundary, err := result.Tessellate(t.Context(), units.Millimeters(.12), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, boundary.BoundaryVerified())
	require.False(t, boundary.VolumeVerified())
	all, err := result.Tessellate(t.Context(), units.Millimeters(.12), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, all.BoundaryVerified())
	require.True(t, all.VolumeVerified())
	require.True(t, slices.Equal(none.Vertices(), boundary.Vertices()))
	require.True(t, slices.Equal(none.Vertices(), all.Vertices()))
	require.True(t, slices.Equal(none.Triangles(), boundary.Triangles()))
	require.True(t, slices.Equal(none.Triangles(), all.Triangles()))
	require.True(t, slices.Equal(none.SourceFaces(), boundary.SourceFaces()))
	require.True(t, slices.Equal(none.SourceFaces(), all.SourceFaces()))
}
