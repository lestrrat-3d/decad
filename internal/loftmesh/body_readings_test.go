package loftmesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func finiteReadings() loftmesh.BodyReadings {
	return loftmesh.BodyReadings{
		Volume: 8, VolumeBound: 1e-9, Area: 24,
		Centroid: r3.Vec{X: 1, Y: 1, Z: 1}, CentroidBound: 1e-9,
		BoundsMin: r3.Vec{}, BoundsMax: r3.Vec{X: 2, Y: 2, Z: 2}, BoundsBound: 1e-9,
	}
}

func TestValidateBodyReadings(t *testing.T) {
	require.NoError(t, loftmesh.ValidateBodyReadings(finiteReadings()))

	inf := math.Inf(1)
	nan := math.NaN()
	cases := []struct {
		name string
		edit func(*loftmesh.BodyReadings)
		msg  string
	}{
		{"volume", func(r *loftmesh.BodyReadings) { r.Volume = inf }, "the loft's volume measurement is not finite"},
		{"volume bound", func(r *loftmesh.BodyReadings) { r.VolumeBound = nan }, "the loft's volume measurement is not finite"},
		{"area", func(r *loftmesh.BodyReadings) { r.Area = nan }, "the loft's area value is not finite"},
		{"centroid", func(r *loftmesh.BodyReadings) { r.Centroid.Z = inf }, "the loft's centroid measurement is not finite"},
		{"centroid bound", func(r *loftmesh.BodyReadings) { r.CentroidBound = inf }, "the loft's centroid measurement is not finite"},
		{"bounds min", func(r *loftmesh.BodyReadings) { r.BoundsMin.Y = nan }, "the loft's bounds measurement is not finite"},
		{"bounds max", func(r *loftmesh.BodyReadings) { r.BoundsMax.X = inf }, "the loft's bounds measurement is not finite"},
		{"bounds bound", func(r *loftmesh.BodyReadings) { r.BoundsBound = inf }, "the loft's bounds measurement is not finite"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			r := finiteReadings()
			tc.edit(&r)
			err := loftmesh.ValidateBodyReadings(r)
			require.ErrorIs(t, err, decaderr.ErrNotFinite)
			require.EqualError(t, err, "decad: non-finite value: "+tc.msg)
		})
	}
}
