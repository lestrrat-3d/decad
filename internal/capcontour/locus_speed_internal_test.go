package capcontour

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestLineWallFrameEnclosesDenotedWall pins that the frame
// LineCircleLocusSpeedUpper reads holds the wall the record denotes: e and n
// enclose the unit direction and normal of the exact endpoint difference, and
// the anchor encloses the exact lerp of a trimmed start.
//
// Shown to fail: with lineWallFrameOf taking the exact unit vector of the held
// tangent (TanInU, TanInV) as e and the held start as an exact point, e misses
// the denoted direction, and the trimmed wall's anchor misses its denoted
// start.
func TestLineWallFrameEnclosesDenotedWall(t *testing.T) {
	t.Parallel()
	eu, ev := exactWallUnit()
	frame, ok := lineWallFrameOf(roundedTangentWall(t, 0))
	require.True(t, ok)
	requireEncloses(t, frame.e.U, eu, `e.u`)
	requireEncloses(t, frame.e.V, ev, `e.v`)
	requireEncloses(t, frame.n.U, new(big.Float).Neg(ev), `n.u`)
	requireEncloses(t, frame.n.V, eu, `n.v`)

	// The start at t = 1/3 is 24 + (−24 − 1e−15)/3, 32 − 32/3 as reals.
	const tStart = 1.0 / 3
	trimmed := roundedTangentWall(t, tStart)
	frame, ok = lineWallFrameOf(trimmed)
	require.True(t, ok)
	rt := new(big.Rat).SetFloat64(tStart)
	lerp := func(a, b float64) *big.Float {
		ra := new(big.Rat).SetFloat64(a)
		x := new(big.Rat).Mul(new(big.Rat).Sub(new(big.Rat).SetFloat64(b), ra), rt)
		return new(big.Float).SetPrec(400).SetRat(x.Add(x, ra))
	}
	requireEncloses(t, frame.anchor.U, lerp(24, -1e-15), `anchor u`)
	requireEncloses(t, frame.anchor.V, lerp(32, 0), `anchor v`)
}
