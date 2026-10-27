package apitest_test

import (
	"errors"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B16's public-surface differential test: random solids of unit
// cubes are read in as mesh solids and run through Union, Cut, Intersect, Shell and Offset, each volume compared with
// the same operation done cube by cube. internal/orthoshell's own differential test checks the grid operations; this one
// checks the way the document's operations use them.

const voxelSide = 5

// differentialOp is the Boolean a differential test runs.
type differentialOp int

const (
	opUnion differentialOp = iota
	opCut
	opIntersect
)

type cubes [voxelSide][voxelSide][voxelSide]bool

func (c *cubes) at(x, y, z int) bool {
	if x < 0 || y < 0 || z < 0 || x >= voxelSide || y >= voxelSide || z >= voxelSide {
		return false
	}
	return c[x][y][z]
}

func (c *cubes) count() int {
	n := 0
	for x := range voxelSide {
		for y := range voxelSide {
			for z := range voxelSide {
				if c[x][y][z] {
					n++
				}
			}
		}
	}
	return n
}

// randomCubes unites a few boxes and carves up to three cubes out again.
func randomCubes(rng *rand.Rand) *cubes {
	var c cubes
	for range 1 + rng.IntN(3) {
		lo := [3]int{rng.IntN(voxelSide - 1), rng.IntN(voxelSide - 1), rng.IntN(voxelSide - 1)}
		var hi [3]int
		for a := range 3 {
			hi[a] = lo[a] + 2 + rng.IntN(voxelSide-lo[a]-1)
		}
		for x := lo[0]; x < hi[0]; x++ {
			for y := lo[1]; y < hi[1]; y++ {
				for z := lo[2]; z < hi[2]; z++ {
					c[x][y][z] = true
				}
			}
		}
	}
	for range rng.IntN(4) {
		c[rng.IntN(voxelSide)][rng.IntN(voxelSide)][rng.IntN(voxelSide)] = false
	}
	return &c
}

// mesh is the closed outward mesh of the cubes, ok false when an edge is not in two triangles.
func (c *cubes) mesh() (verts []r3.Vec, tris [][3]int, ok bool) {
	index := map[[3]int]int{}
	vertex := func(p [3]int) int {
		if i, found := index[p]; found {
			return i
		}
		index[p] = len(verts)
		verts = append(verts, r3.NewVec(float64(p[0]), float64(p[1]), float64(p[2])))
		return len(verts) - 1
	}
	for x := range voxelSide {
		for y := range voxelSide {
			for z := range voxelSide {
				if !c[x][y][z] {
					continue
				}
				cell := [3]int{x, y, z}
				for a := range 3 {
					for _, s := range []int{-1, 1} {
						next := cell
						next[a] += s
						if c.at(next[0], next[1], next[2]) {
							continue
						}
						plane := cell[a]
						if s > 0 {
							plane++
						}
						a1, a2 := (a+1)%3, (a+2)%3
						corner := func(d1, d2 int) int {
							var p [3]int
							p[a], p[a1], p[a2] = plane, cell[a1]+d1, cell[a2]+d2
							return vertex(p)
						}
						q := [4]int{corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)}
						if s < 0 {
							q = [4]int{q[0], q[3], q[2], q[1]}
						}
						tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
					}
				}
			}
		}
	}
	uses := map[[2]int]int{}
	for _, t := range tris {
		for k := range 3 {
			a, b := t[k], t[(k+1)%3]
			uses[[2]int{min(a, b), max(a, b)}]++
		}
	}
	for _, n := range uses {
		if n != 2 {
			return nil, nil, false
		}
	}
	return verts, tris, len(tris) > 0
}

// solid reads the cubes into doc as a mesh solid; ok is false for a set the document does not take as a solid.
func (c *cubes) solid(t *testing.T, doc *decad.Document) (*decad.Body, bool) {
	t.Helper()
	v, tris, ok := c.mesh()
	if !ok {
		return nil, false
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	b, err := decad.SolidFromMesh(t.Context(), doc, m)
	if err != nil {
		require.True(t, errors.Is(err, decad.ErrNotSolid) || errors.Is(err, decad.ErrDegenerate) || errors.Is(err, decad.ErrUnsupported), "%v", err)
		return nil, false
	}
	return b, true
}

// dilated and eroded count the cubes of the set grown or shrunk by the cube of half-side one, on a grid a cube larger each
// way so that a growth past the set's own grid is counted.
func (c *cubes) grown() int {
	n := 0
	for x := -1; x <= voxelSide; x++ {
		for y := -1; y <= voxelSide; y++ {
			for z := -1; z <= voxelSide; z++ {
				found := false
				for dx := -1; dx <= 1 && !found; dx++ {
					for dy := -1; dy <= 1 && !found; dy++ {
						for dz := -1; dz <= 1 && !found; dz++ {
							found = c.at(x+dx, y+dy, z+dz)
						}
					}
				}
				if found {
					n++
				}
			}
		}
	}
	return n
}

func (c *cubes) eroded() int {
	n := 0
	for x := range voxelSide {
		for y := range voxelSide {
			for z := range voxelSide {
				all := true
				for dx := -1; dx <= 1; dx++ {
					for dy := -1; dy <= 1; dy++ {
						for dz := -1; dz <= 1; dz++ {
							all = all && c.at(x+dx, y+dy, z+dz)
						}
					}
				}
				if all {
					n++
				}
			}
		}
	}
	return n
}

func volumeOf(t *testing.T, b *decad.Body) (float64, float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	return v.Value.Base(), v.Bound.Base()
}

func TestMeshSolidBooleansAgreeWithTheCubeByCubeBoolean(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(21, 22))
	checked, exact := 0, 0
	for range 45 {
		a, b := randomCubes(rng), randomCubes(rng)
		for _, op := range []differentialOp{opUnion, opCut, opIntersect} {
			doc := decad.New()
			ba, okA := a.solid(t, doc)
			bb, okB := b.solid(t, doc)
			if !okA || !okB {
				continue
			}
			want := 0
			for x := range voxelSide {
				for y := range voxelSide {
					for z := range voxelSide {
						switch op {
						case opUnion:
							if a[x][y][z] || b[x][y][z] {
								want++
							}
						case opCut:
							if a[x][y][z] && !b[x][y][z] {
								want++
							}
						default:
							if a[x][y][z] && b[x][y][z] {
								want++
							}
						}
					}
				}
			}
			var out *decad.Body
			var err error
			switch op {
			case opUnion:
				out, err = decad.Union(t.Context(), ba, bb)
			case opCut:
				out, err = decad.Cut(t.Context(), ba, bb)
			default:
				out, err = decad.Intersect(t.Context(), ba, bb)
			}
			if err != nil {
				// An empty result, or cubes that meet only along an edge, which no solid is: refused, never a wrong volume.
				continue
			}
			checked++
			got, bound := volumeOf(t, out)
			require.InDelta(t, float64(want), got, bound+1e-9, "op %d of %d and %d cubes", op, a.count(), b.count())
			if bound == 0 {
				exact++
			}
		}
	}
	require.Greater(t, checked, 50)
	require.Greater(t, exact, checked*9/10, "the grid route answers nearly every pair exactly; the rest take the mesh route and stay within its bound")
}

func TestMeshSolidShellAndOffsetAgreeWithTheCubeByCubeErosionAndDilation(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(23, 24))
	shelled, grown, shrunk := 0, 0, 0
	for range 90 {
		c := randomCubes(rng)
		doc := decad.New()
		b, ok := c.solid(t, doc)
		if !ok {
			continue
		}
		// A closed inward shell one cube thick keeps the cubes the erosion takes away.
		if out, err := decad.Shell(t.Context(), b, nil, units.Millimeters(1), decad.WithNoOpenings()); err == nil {
			shelled++
			got, bound := volumeOf(t, out)
			require.InDelta(t, float64(c.count()-c.eroded()), got, bound+1e-9, "shell of %d cubes", c.count())
		}
		doc = decad.New()
		b, _ = c.solid(t, doc)
		if out, err := decad.Offset(t.Context(), b, units.Millimeters(1)); err == nil {
			grown++
			got, bound := volumeOf(t, out)
			require.InDelta(t, float64(c.grown()), got, bound+1e-9, "growing %d cubes", c.count())
		}
		doc = decad.New()
		b, _ = c.solid(t, doc)
		if out, err := decad.Offset(t.Context(), b, units.Millimeters(1), decad.WithOffsetSide(decad.OffsetNegative)); err == nil {
			shrunk++
			got, bound := volumeOf(t, out)
			require.InDelta(t, float64(c.eroded()), got, bound+1e-9, "shrinking %d cubes", c.count())
		}
	}
	require.Greater(t, shelled, 4)
	require.Greater(t, grown, 25)
	require.Greater(t, shrunk, 12)
}
