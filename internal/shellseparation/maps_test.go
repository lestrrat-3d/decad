package shellseparation

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func ratMatrix(m [3][3]int64) [3][3]*big.Rat {
	var out [3][3]*big.Rat
	for i := range m {
		for j := range m[i] {
			out[i][j] = big.NewRat(m[i][j], 1)
		}
	}
	return out
}

func TestExactXRotationStructure(t *testing.T) {
	quarter := ratMatrix([3][3]int64{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}})
	require.True(t, exactXRotation(quarter))

	aboutY := ratMatrix([3][3]int64{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}})
	require.False(t, exactXRotation(aboutY), "an X cross term")

	scaled := quarter
	scaled[0][0] = big.NewRat(1, 1)
	scaled[0][0].Add(scaled[0][0], new(big.Rat).SetFloat64(0x1p-50))
	require.False(t, exactXRotation(scaled), "an X scale beyond 2^-52")

	// An exact X rotation fixes X, so the X scale must be 1 exactly.
	for _, tiny := range []float64{0x1p-60, 0x1p-52} {
		off := quarter
		off[0][0] = new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).SetFloat64(tiny))
		require.False(t, exactXRotation(off), "an X scale of 1 + %g is malformed", tiny)
		off[0][0] = new(big.Rat).Sub(big.NewRat(1, 1), new(big.Rat).SetFloat64(tiny))
		require.False(t, exactXRotation(off), "an X scale of 1 - %g is malformed", tiny)
	}

	skew := quarter
	skew[2][1] = big.NewRat(2, 1)
	require.False(t, exactXRotation(skew), "off-diagonal Y/Z terms must be opposite")
}

func TestAxialDriftReadsExactMaps(t *testing.T) {
	// The float move of copy 1 has X scale near 0, so reading it as the true
	// map charges a large drift. The exact map has X scale 1 and charges none.
	about, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(90))
	require.NoError(t, err)
	shell := Shell{Verts: []r3.Vec{{X: 3, Y: 1, Z: 1}, {X: 4}, {X: 5, Y: 2}},
		Tris: [][3]int{{0, 1, 2}}, VertexBound: make([]float64, 3)}
	moves := []r3.Transform{r3.Identity(), about}
	exact := [][3][3]*big.Rat{{}, ratMatrix([3][3]int64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}})}

	drift, ok := sourceAxialDrift(shell, moves, nil, 1)
	require.True(t, ok)
	require.Greater(t, drift, 1.0)

	drift, ok = sourceAxialDrift(shell, moves, exact, 1)
	require.True(t, ok)
	require.Zero(t, drift)
}

func TestTransferableMovesReadsExactMaps(t *testing.T) {
	// The float move is a Y rotation, which float transfer refuses. The exact
	// entry is an X rotation, so transfer is allowed: the maps decide.
	about, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(90))
	require.NoError(t, err)
	moves := []r3.Transform{r3.Identity(), about}
	quarter := [][3][3]*big.Rat{{}, ratMatrix([3][3]int64{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}})}
	require.False(t, transferableMoves(moves, nil))
	require.True(t, transferableMoves(moves, quarter))

	aboutY := [][3][3]*big.Rat{{}, ratMatrix([3][3]int64{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}})}
	require.False(t, transferableMoves(moves, aboutY))
}
