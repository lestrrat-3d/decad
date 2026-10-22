package apitest_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestUnionManyArrangesThreePrismsOnce(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 4, 4, 2)
	b := boxBody(t, doc, 2, 2, 6, 6, 2)
	c := boxBody(t, doc, 5, -1, 9, 3, 2)

	tools := []*decad.Body{b, c}
	joined, err := decad.Union(t.Context(), a, tools...)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
	require.False(t, tookMeshPath(joined))
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-86), boundMM3(t, volume))
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestUnionManyChainsMixedHeightPrisms(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 4, 4, 2)
	b := boxBody(t, doc, 2, 2, 6, 6, 2)
	c := boxBodyAtZ(t, doc, 5, -1, 9, 3, 1, 2)

	joined, err := decad.Union(t.Context(), a, b, c)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-87), boundMM3(t, volume))
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestUnionManyRefusalKeepsEveryOperandLive(t *testing.T) {
	doc := decad.New(decad.ExactOnly())
	a := boxBody(t, doc, 0, 0, 4, 4, 2)
	b := boxBody(t, doc, 2, 2, 6, 6, 2)
	// The first pair joins. The third box touches that result only along an
	// edge, so the second pair has no solid contact certificate.
	c := boxBodyAtZ(t, doc, 6, 6, 8, 8, 1, 2)
	original := doc.Bodies()

	_, err := decad.Union(t.Context(), a, b, c)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, original, doc.Bodies())

	_, err = decad.Union(t.Context(), a, b, b)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, original, doc.Bodies())

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = decad.Union(ctx, a, b, c)
	require.True(t, errors.Is(err, context.Canceled))
	require.Equal(t, original, doc.Bodies())
}

func TestUnionManyKeepsSeparatePrismLumps(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 2, 2, 2)
	b := boxBody(t, doc, 4, 3, 6, 5, 2)
	c := boxBody(t, doc, 8, 6, 10, 8, 2)
	joined, err := decad.Union(t.Context(), a, b, c)
	require.NoError(t, err)
	require.Len(t, joined.Lumps(), 3)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-24), boundMM3(t, volume))
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}

func TestUnionManyRefusesSharedLineCarrier(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 2, 2, 2)
	b := boxBody(t, doc, 4, 0, 6, 2, 2)
	c := boxBody(t, doc, 8, 0, 10, 2, 2)
	original := doc.Bodies()
	_, err := decad.Union(t.Context(), a, b, c)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, original, doc.Bodies())
}

func TestUnionNeedsAtLeastTwoBodies(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 2, 2, 2)
	_, err := decad.Union(t.Context(), a)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, []*decad.Body{a}, doc.Bodies())
}
