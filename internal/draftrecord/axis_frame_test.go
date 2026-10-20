package draftrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestExactSignedAxisFrame(t *testing.T) {
	t.Parallel()

	frame := func(u, v r3.Vec) r3.Frame {
		f, err := r3.NewFrame(r3.Vec{}, u, v)
		require.NoError(t, err)
		return f
	}
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	neg := func(v r3.Vec) r3.Vec { return v.Scale(-1) }
	for name, f := range map[string]r3.Frame{
		"x y": frame(x, y), "y x": frame(y, x), "x z": frame(x, z), "z y": frame(z, y), "y z": frame(y, z),
		"negative x": frame(neg(x), y), "negative y": frame(x, neg(y)), "negative z": frame(neg(z), x),
	} {
		require.True(t, draftrecord.ExactSignedAxisFrame(f), name)
	}
	diagonal := math.Sqrt2 / 2
	for name, f := range map[string]r3.Frame{
		"turned about z": frame(r3.Vec{X: diagonal, Y: diagonal}, r3.Vec{X: -diagonal, Y: diagonal}),
		"tilted v":       frame(x, r3.Vec{Y: diagonal, Z: diagonal}),
		"tilted u":       frame(r3.Vec{X: diagonal, Z: diagonal}, y),
		"nearly x":       frame(r3.NewVec(1, 1e-9, 0), r3.NewVec(-1e-9, 1, 0)),
		"u nearly x":     frame(r3.NewVec(1, 1e-9, 0), z),
		"u nearly y":     frame(r3.NewVec(1e-9, 1, 0), z),
		"u nearly z":     frame(r3.NewVec(0, 1e-9, 1), x),
	} {
		require.False(t, draftrecord.ExactSignedAxisFrame(f), name)
	}
}
