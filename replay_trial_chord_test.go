package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestReplayTrialChordRealPublicProducers(t *testing.T) {
	ctx := t.Context()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 3, 4, 5)
	b := internalBoxBody(t, doc, 8, -3, 12, 3, 4)
	chord, diameter, err := booloperand.PairChordReplay(ctx, &a.n, &b.n, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	ordinary, ordinaryDiameter, err := booloperand.PairChord(ctx, &a.n, &b.n)
	require.NoError(t, err)
	require.Equal(t, ordinary, chord)
	require.Equal(t, ordinaryDiameter, diameter)
	require.True(t, chord > 0 && !math.IsInf(chord, 0) && !math.IsNaN(chord))
	require.True(t, diameter > 0 && !math.IsInf(diameter, 0) && !math.IsNaN(diameter))
	for i, body := range []*Body{a, b} {
		mesh, err := body.Tessellate(ctx, units.Millimeters(chord), WithVerification(VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
		volume, err := body.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-[]float64{60, 96}[i]), volume.Bound.Base())
	}
	cylinder := internalRevolvedCylinderBody(t, doc, 3, 12)
	raw := internalBoxBody(t, doc, 0, -4, 4, 4, 3)
	move, err := r3.Translation(r3.Vec{Z: 1.5})
	require.NoError(t, err)
	tool, err := Transform(ctx, raw, move)
	require.NoError(t, err)
	cut, err := Cut(ctx, cylinder, tool)
	require.NoError(t, err)
	_, ok := cut.n.Payload.(facetedrecord.Payload)
	require.True(t, ok, "first Cut did not produce a faceted payload")
	boundedShare, err := booloperand.ChordShareReplay(ctx, &cut.n, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.Equal(t, booloperand.ChordShare(ctx, &cut.n), boundedShare)
}
