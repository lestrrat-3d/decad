package explodeblock_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeblock"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// translate moves every vertex of m by off.
func translate(m explodeblock.Mesh, off r3.Vec) explodeblock.Mesh {
	vs := make([]r3.Vec, len(m.Vertices))
	for i, v := range m.Vertices {
		vs[i] = v.Add(off)
	}
	return explodeblock.Mesh{Vertices: vs, Triangles: m.Triangles, Bound: m.Bound}
}

// boltedFlange is a wide flange (an annulus, z 0..6) with boltCount identical
// bolts standing on it, each through an identical washer under its head: one
// big mesh and many small repeated ones, the shape of a real assembly.
const boltCount = 8

func boltedFlange() []explodeblock.Mesh {
	flange := revolve([][2]float64{{12, 0}, {40, 0}, {40, 6}, {12, 6}}, 0.02)
	bolt := revolve([][2]float64{
		{0, 6}, {2, 6}, {2, 9}, {4, 9}, {4, 12}, {0, 12},
	}, 0.01)
	washer := revolve([][2]float64{{2.1, 6}, {3.9, 6}, {3.9, 7}, {2.1, 7}}, 0.01)
	meshes := []explodeblock.Mesh{flange}
	for k := range boltCount {
		a := 2 * math.Pi * float64(k) / boltCount
		off := r3.NewVec(26*math.Cos(a), 26*math.Sin(a), 0)
		meshes = append(meshes, translate(bolt, off), translate(washer, off))
	}
	return meshes
}

// instances groups the meshes and places one projection per class along dir.
func instances(t *testing.T, meshes []explodeblock.Mesh, dir r3.Vec) []explodeblock.Instance {
	t.Helper()
	shares, classes := explodeblock.Group(meshes)
	projs := make([]*explodeblock.Projection, len(classes))
	for i, c := range classes {
		p, err := explodeblock.Project(c, dir)
		require.NoError(t, err)
		projs[i] = p
	}
	items := make([]explodeblock.Instance, len(meshes))
	for i, s := range shares {
		items[i] = explodeblock.Instance{P: projs[s.Class], Offset: s.Offset}
	}
	return items
}

// The pair tests read only the grid cells inside the other mesh's shadow, skip
// a mover triangle that every triangle of the other mesh stands behind, and
// reuse their clipping buffers. The ceilings sit between the work done and the
// work done without each of these.
func TestTableWorkOnRepeatedParts(t *testing.T) {
	meshes := boltedFlange()
	_, classes := explodeblock.Group(meshes)
	require.Len(t, classes, 3) // flange, bolt, washer

	items := instances(t, meshes, up)
	tab := explodeblock.BuildTable(items, 4)
	serial := explodeblock.BuildTable(items, 1)
	require.Equal(t, serial.Work(), tab.Work())
	for a := range items {
		for b := range items {
			require.Equal(t, serial.At(a, b), tab.At(a, b), "pair %d %d", a, b)
		}
	}

	// Up: a bolt's shank passes through its washer's bore, so the bolt leaves
	// freely; the head stands over the washer; every bolt and washer stands
	// over the flange.
	var above []int
	for k := range boltCount {
		bolt, washer := 1+2*k, 2+2*k
		require.Empty(t, tab.Blockers(bolt), "bolt %d", k)
		require.Equal(t, []int{bolt}, tab.Blockers(washer), "washer %d", k)
		above = append(above, bolt, washer)
	}
	require.Equal(t, above, tab.Blockers(0))

	w := tab.Work()
	// Reading every cell a dilated mover triangle spans, inside the other
	// mesh's shadow or not, reads about 177k cells; testing the mover triangles
	// every washer and bolt stands behind reads about 13k cells and 77k
	// triangle pairs.
	require.LessOrEqual(t, w.Cells, int64(6000))
	require.LessOrEqual(t, w.Candidates, int64(40000))

	// One worker reuses its buffers over every pair: a buffer per pair costs
	// about 220 allocations, a buffer per clip about 25k.
	allocs := testing.AllocsPerRun(3, func() { explodeblock.BuildTable(items, 1) })
	require.LessOrEqual(t, allocs, float64(64))
}
