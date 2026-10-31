package crossbore

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestStationSampleError compares one station's held band points with its
// base cell's exact rows: rounded rows read within rounding, and a moved inner
// row reads the move.
func TestStationSampleError(t *testing.T) {
	t.Parallel()
	const R, r, d, j = 10.0, 3.0, 1.5, 77
	pose := Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{-1, -1, 1}, Shift: r3.Vec{X: 40}}
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(ToolStations, 1))
	base, err := proveToolBase(R, r, d, j, gap)
	require.NoError(t, err)
	o, b := BandEnds(R, r, d, j)
	held := func(i int) r3.Vec { return pose.World(BandRow(o, b, i)) }
	require.Less(t, stationSampleError(&base.ends, held, false, false, pose), 1e-12,
		"the held rows are the exact rows rounded")

	const moved = 1e-4
	shifted := func(i int) r3.Vec {
		p := held(i)
		if i == ToolRows/2 {
			p.X += moved
		}
		return p
	}
	require.GreaterOrEqual(t, stationSampleError(&base.ends, shifted, false, false, pose), moved,
		"the comparison reads the moved inner row")
}
