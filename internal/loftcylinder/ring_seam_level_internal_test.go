package loftcylinder

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireXYEqual(t *testing.T, want, got XY) {
	t.Helper()
	require.Zero(t, want.X.Cmp(got.X))
	require.Zero(t, want.Y.Cmp(got.Y))
}

// TestRingSeamLevelMatchesDirect checks that the per-level seam record holds
// exactly what the direct CrossingBox, connector slice and sector calls give
// for the same circle and level, for a circle whose crossing box succeeds and
// one whose crossing box refuses.
func TestRingSeamLevelMatchesDirect(t *testing.T) {
	t.Parallel()
	xy := func(x, y, den int64) XY { return XY{big.NewRat(x, den), big.NewRat(y, den)} }
	up := Connector{a: [2]XY{xy(2, 0, 1), xy(5, 1, 3)}, d: [2]XY{xy(2, 0, 1), xy(5, 1, 3)},
		f: [2]XY{xy(7, 1, 1), xy(22, 5, 3)}}
	down := Connector{a: [2]XY{xy(2, 0, 1), xy(5, -1, 3)}, d: [2]XY{xy(2, 0, 1), xy(5, -1, 3)},
		f: [2]XY{xy(7, -1, 1), xy(22, -5, 3)}}
	circle := SourceCircle{Z0: big.NewRat(0, 1), Z1: big.NewRat(1, 1), SafeBottom: big.NewRat(0, 1),
		SafeTop: big.NewRat(1, 1), Radius: big.NewRat(7, 2), GapAxis: xy(1, 0, 1),
		Connector: [2]Connector{down, up}}
	refusing := circle
	refusing.GapAxis = xy(-1, 0, 1)

	rng := rand.New(rand.NewPCG(97, 101))
	for ci, c := range []SourceCircle{circle, refusing} {
		sawTrue, sawFalse := false, false
		for _, level := range []*big.Rat{big.NewRat(1, 4), big.NewRat(1, 3), big.NewRat(1, 2), big.NewRat(5, 8)} {
			seam := ringSeamLevelOf(RingToothLevel{Circle: c, Level: level})
			require.True(t, seam.built)
			boxes, err := c.CrossingBox(level, level)
			require.Equal(t, err == nil, seam.sectorOK)
			if err == nil {
				for k := range boxes {
					requireXYEqual(t, boxes[k].lo, seam.sector[k].lo)
					requireXYEqual(t, boxes[k].hi, seam.sector[k].hi)
				}
			}
			for i, connector := range c.Connector {
				tt := new(big.Rat).Quo(new(big.Rat).Sub(level, c.Z0), new(big.Rat).Sub(c.Z1, c.Z0))
				requireXYEqual(t, sourceLerp(connector.a[0], connector.a[1], tt), seam.connectors[i][0])
				requireXYEqual(t, sourceLerp(connector.d[0], connector.d[1], tt), seam.connectors[i][1])
				requireXYEqual(t, sourceLerp(connector.f[0], connector.f[1], tt), seam.connectors[i][2])
			}

			var tris [][3]proofarith.Xpt
			for i := -20; i <= 20; i++ {
				m := RingMappedPoint(proofarith.XptOf(r3.Vec{X: 3.5, Y: 0.05 * float64(i), Z: 0}), level)
				tris = append(tris, [3]proofarith.Xpt{m, m, m})
			}
			point := func() proofarith.Xpt {
				radius := 3 + rng.Float64()
				angle := (rng.Float64() - 0.5) * 0.8
				p := proofarith.XptOf(r3.Vec{X: radius * math.Cos(angle), Y: radius * math.Sin(angle), Z: 0})
				return RingMappedPoint(p, level)
			}
			for range 20 {
				tris = append(tris, [3]proofarith.Xpt{point(), point(), point()})
			}
			for _, tri := range tris {
				want := c.ContainsTriangleSector(level, level, tri)
				require.Equal(t, want, seam.sectorOK && triangleInSector(seam.sector, level, level, tri))
				sawTrue = sawTrue || want
				sawFalse = sawFalse || !want
			}
		}
		if ci == 0 {
			require.True(t, sawTrue)
			require.True(t, sawFalse)
		}
	}
}
