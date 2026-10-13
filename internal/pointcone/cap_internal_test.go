package pointcone

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// capTestFan subdivides a small point-loft fan, whose far cap lies off the
// cone axis, rounds times.
func capTestFan(t *testing.T, rounds int) ([]r3.Vec, [][3]int) {
	t.Helper()
	const ring = 8
	verts := []r3.Vec{{X: -1, Y: 3, Z: 3}, {X: 3, Y: 3, Z: 3}}
	for i := range ring {
		a := 2 * math.Pi * float64(i) / ring
		verts = append(verts, r3.Vec{X: 3 + 0.1*float64(i%3), Y: 3 + math.Cos(a), Z: 3 + math.Sin(a)})
	}
	var tris [][3]int
	for i := range ring {
		a, b := 2+i, 2+(i+1)%ring
		tris = append(tris, [3]int{0, a, b}, [3]int{1, a, b})
	}
	far, caps, _, _, err := Subdivide(t.Context(), verts, tris, make([]int, len(tris)), rounds)
	require.NoError(t, err)
	return far, caps
}

// capTestRandom is n random cells over off-axis vertices.
func capTestRandom(r *rand.Rand, n int) ([]r3.Vec, [][3]int) {
	far := make([]r3.Vec, 700)
	for i := range far {
		far[i] = r3.Vec{X: 0.1 + 6*r.Float64(), Y: 0.5 + 4*r.Float64(), Z: 0.5 + 4*r.Float64()}
	}
	caps := make([][3]int, n)
	for i := range caps {
		caps[i] = [3]int{r.IntN(len(far)), r.IntN(len(far)), r.IntN(len(far))}
	}
	return far, caps
}

func capTestCones() []*Consts {
	var out []*Consts
	for _, p := range [][2]*big.Rat{
		{big.NewRat(3, 1), big.NewRat(1, 1)},
		{big.NewRat(15, 2), big.NewRat(1, 1)},
		{big.NewRat(12, 1), big.NewRat(1, 1)},
		{big.NewRat(1, 1000), big.NewRat(1000, 1)},
	} {
		out = append(out, ConstantsOf(p[0], p[1]))
	}
	return out
}

func requireSameGeom(t *testing.T, want, got []CellGeom) {
	t.Helper()
	require.Len(t, got, len(want))
	for k := range want {
		require.Equal(t, math.Float64bits(want[k].xMin), math.Float64bits(got[k].xMin), "cell %d xMin", k)
		require.Equal(t, math.Float64bits(want[k].rhoMin), math.Float64bits(got[k].rhoMin), "cell %d rhoMin", k)
		require.Equal(t, math.Float64bits(want[k].reach), math.Float64bits(got[k].reach), "cell %d reach", k)
		require.Equal(t, math.Float64bits(want[k].h), math.Float64bits(got[k].h), "cell %d h", k)
	}
}

func TestPointConeCapParallelMatchesSerial(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(21, 22))
	type fan struct {
		name string
		far  []r3.Vec
		caps [][3]int
	}
	var fans []fan
	for _, rounds := range []int{3, 4} {
		far, caps := capTestFan(t, rounds)
		fans = append(fans, fan{"subdivide", far, caps})
	}
	far, caps := capTestRandom(r, 1500)
	fans = append(fans, fan{"random", far, caps})
	for _, f := range fans {
		serial := capGeometry(f.far, f.caps, 1)
		requireSameGeom(t, serial, capGeometry(f.far, f.caps, 4))
		for _, c := range capTestCones() {
			wantBound, wantLip, wantErr := capBound(serial, c, 1)
			gotBound, gotLip, gotErr := capBound(serial, c, 4)
			require.NoError(t, wantErr, f.name)
			require.NoError(t, gotErr, f.name)
			require.Equal(t, math.Float64bits(wantBound), math.Float64bits(gotBound), f.name)
			require.Equal(t, math.Float64bits(wantLip), math.Float64bits(gotLip), f.name)
		}
	}
}

func TestPointConeCapBoundReportsTheFirstFailingCell(t *testing.T) {
	t.Parallel()
	far, caps := capTestRandom(rand.New(rand.NewPCG(23, 24)), 1500)
	geom := capGeometry(far, caps, 1)
	geom[900].rhoMin = 0
	geom[1200].xMin = -1e6
	c := ConstantsOf(big.NewRat(3, 1), big.NewRat(1, 1))
	_, _, wantErr := capBound(geom, c, 1)
	require.ErrorContains(t, wantErr, "reaches the axis")
	for range 20 {
		_, _, gotErr := capBound(geom, c, 4)
		require.EqualError(t, gotErr, wantErr.Error())
	}
}

func TestPointConeCapSerialPathOnNonFiniteInput(t *testing.T) {
	t.Parallel()
	far, caps := capTestRandom(rand.New(rand.NewPCG(25, 26)), 1500)
	geom := capGeometry(far, caps, 1)
	require.True(t, geomFinite(geom))
	geom[900].rhoMin = 0
	geom[1000].xMin = math.NaN()
	require.False(t, geomFinite(geom))
	c := ConstantsOf(big.NewRat(3, 1), big.NewRat(1, 1))
	_, _, wantErr := capBound(geom, c, 1)
	require.ErrorContains(t, wantErr, "reaches the axis")
	_, _, gotErr := capBound(geom, c, 4)
	require.EqualError(t, gotErr, wantErr.Error())

	nan := append([]r3.Vec(nil), far...)
	used := caps[len(caps)-1][0]
	nan[used] = r3.Vec{X: math.NaN()}
	require.Panics(t, func() { capGeometry(nan, caps[len(caps)-1:], 1) }, "serial path is the old code")
	require.Panics(t, func() { capGeometry(nan, caps, 1) })
	require.Panics(t, func() { capGeometry(nan, caps, 4) }, "a non-finite vertex takes the serial path")
}
