package loftmesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// fanFixture builds triangles around the centers in centers (vertex indices
// 0..len(centers)-1) over an integer rim grid. Small coordinates make
// duplicate directions, exact antipodes and collinear rims frequent. Each
// center gets count random triangles over its rim; a triangle is wound at
// random and may be a non-star corner pair.
func fanFixture(rng *rand.Rand, centers []r3.Vec, rimSize, count int, span float64) ([]r3.Vec, [][3]int) {
	verts := append([]r3.Vec(nil), centers...)
	rim := make([]int, rimSize)
	for k := range rim {
		rim[k] = len(verts)
		verts = append(verts, r3.Vec{
			X: float64(rng.IntN(2*int(span)+1) - int(span)),
			Y: float64(rng.IntN(2*int(span)+1) - int(span)),
			Z: centers[0].Z,
		})
	}
	var tris [][3]int
	for c := range centers {
		for n := 0; n < count; {
			a, b := rim[rng.IntN(rimSize)], rim[rng.IntN(rimSize)]
			if a == b || verts[a] == verts[c] || verts[b] == verts[c] {
				continue
			}
			tri := [3]int{c, a, b}
			if rng.IntN(2) == 0 {
				tri[1], tri[2] = tri[2], tri[1]
			}
			tris = append(tris, tri)
			n++
		}
	}
	return verts, tris
}

// checkFanTable compares the table with CoplanarIsolatedSharedVertex on every
// pair that shares one vertex and returns how many pairs the table decided
// and how many of those were true.
func checkFanTable(t *testing.T, name string, data *LoftAuditData, tris [][3]int) (int, int) {
	t.Helper()
	if data.fans == nil {
		return 0, 0
	}
	decided, certified := 0, 0
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if count != 1 {
				continue
			}
			got, ok := data.fans.vertexCertificate(i, j, shared[0])
			if !ok {
				continue
			}
			want := CoplanarIsolatedSharedVertex(data, tris[i], tris[j], i, j, shared[0])
			require.Equal(t, want, got, "%s pair %d,%d", name, i, j)
			decided++
			if got {
				certified++
			}
		}
	}
	return decided, certified
}

func TestLoftAuditFansMatchCoplanarCertificate(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(61, 67))
	cases := []struct {
		name    string
		centers []r3.Vec
		rim     int
		count   int
		span    float64
		fans    int
	}{
		{"grid-rim", []r3.Vec{{Z: 2}}, 24, 40, 3, 1},
		{"grid-rim-wide", []r3.Vec{{Z: -1}}, 40, 70, 8, 1},
		{"31-triangles", []r3.Vec{{Z: 0}}, 20, 31, 4, 0},
		{"32-triangles", []r3.Vec{{Z: 0}}, 20, 32, 4, 1},
		{"two-fans-shared-rim", []r3.Vec{{Z: 1}, {X: 1, Y: 0.5, Z: 1}}, 22, 36, 3, 2},
		{"offset-center", []r3.Vec{{X: 0.5, Y: -0.25, Z: 3}}, 30, 50, 5, 1},
	}
	for _, c := range cases {
		verts, tris := fanFixture(rng, c.centers, c.rim, c.count, c.span)
		for name, build := range map[string]func([]r3.Vec, [][3]int) *LoftAuditData{
			"full": NewLoftAuditData, "compact": newCompactLoftAuditData,
		} {
			data := build(verts, tris)
			if c.fans == 0 {
				require.Nil(t, data.fans, "%s %s", c.name, name)
				continue
			}
			require.NotNil(t, data.fans, "%s %s", c.name, name)
			require.Len(t, data.fans.fans, c.fans, "%s %s", c.name, name)
			decided, certified := checkFanTable(t, c.name+" "+name, data, tris)
			require.Positive(t, decided, "%s %s", c.name, name)
			require.Positive(t, certified, "%s %s", c.name, name)
			require.Less(t, certified, decided, "%s %s: no false pair", c.name, name)
		}
	}
}

func TestLoftAuditFansRefuseUnsafeFans(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	verts, tris := fanFixture(rng, []r3.Vec{{Z: 2}}, 24, 40, 3)

	// A table is built only for non-exact lifts.
	exact := NewLoftAuditDataWithExact(verts, perturbedExact(verts), tris)
	require.Nil(t, exact.fans)

	// Mixed projection axes: no fan.
	data := NewLoftAuditData(verts, tris)
	require.NotNil(t, data.fans)
	data.certAxes = append([]uint8(nil), data.certAxes...)
	data.certAxes[0] = (data.certAxes[0] + 1) % 3
	require.Nil(t, newLoftAuditFans(data, tris))

	// A tilted triangle at the center leaves the fan; the pair is undecided.
	tilted := append([]r3.Vec(nil), verts...)
	tilted = append(tilted, r3.Vec{X: 5, Y: 5, Z: 9})
	tris2 := append(append([][3]int(nil), tris...), [3]int{0, 1, len(tilted) - 1})
	data2 := NewLoftAuditData(tilted, tris2)
	require.NotNil(t, data2.fans)
	last := len(tris2) - 1
	require.Equal(t, int32(-1), data2.fans.fan[last][0])
	_, ok := data2.fans.vertexCertificate(0, last, 0)
	require.False(t, ok)

	// A rim vertex projected onto the center disables the fan.
	onto := append([]r3.Vec(nil), verts...)
	onto = append(onto, verts[0])
	tris3 := append([][3]int(nil), tris...)
	for k := range 3 {
		tris3 = append(tris3, [3]int{0, 1 + k, len(onto) - 1})
	}
	require.Nil(t, NewLoftAuditData(onto, tris3).fans)
}
