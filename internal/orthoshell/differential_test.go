package orthoshell_test

import (
	"errors"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file checks the grid operations against a brute force on a voxel grid: a solid of unit cubes on a small integer
// grid is meshed, each operation is run on the mesh, and the result is compared with the same operation done cube by
// cube. The shapes are random with a fixed seed, so a failure repeats.

const gridSide = 6 // the voxel grid is gridSide cubes along each axis

// voxels is a set of unit cubes: v[x][y][z] holds the cube from (x, y, z) to (x+1, y+1, z+1).
type voxels [gridSide][gridSide][gridSide]bool

func (v *voxels) at(x, y, z int) bool {
	if x < 0 || y < 0 || z < 0 || x >= gridSide || y >= gridSide || z >= gridSide {
		return false
	}
	return v[x][y][z]
}

func (v *voxels) count() int {
	n := 0
	for x := range gridSide {
		for y := range gridSide {
			for z := range gridSide {
				if v[x][y][z] {
					n++
				}
			}
		}
	}
	return n
}

// random fills a few boxes of random size into an empty set.
func randomVoxels(rng *rand.Rand) *voxels {
	var v voxels
	for range 1 + rng.IntN(3) {
		// Boxes at least three cubes across where the grid has room, so a solid holds cubes that erode and a cavity can form.
		lo := [3]int{rng.IntN(gridSide - 2), rng.IntN(gridSide - 2), rng.IntN(gridSide - 2)}
		hi := [3]int{}
		for a := range 3 {
			hi[a] = lo[a] + 3 + rng.IntN(gridSide-lo[a]-2)
		}
		for x := lo[0]; x < hi[0]; x++ {
			for y := lo[1]; y < hi[1]; y++ {
				for z := lo[2]; z < hi[2]; z++ {
					v[x][y][z] = true
				}
			}
		}
	}
	// Carve a few cubes out again: a notch on the surface, a slot or a cavity wholly inside.
	for range rng.IntN(4) {
		v[rng.IntN(gridSide)][rng.IntN(gridSide)][rng.IntN(gridSide)] = false
	}
	return &v
}

// combine is the cube by cube Boolean of two sets.
func (v *voxels) combine(o *voxels, op orthoshell.Op) *voxels {
	var out voxels
	for x := range gridSide {
		for y := range gridSide {
			for z := range gridSide {
				a, b := v[x][y][z], o[x][y][z]
				switch op {
				case orthoshell.Union:
					out[x][y][z] = a || b
				case orthoshell.Cut:
					out[x][y][z] = a && !b
				default:
					out[x][y][z] = a && b
				}
			}
		}
	}
	return &out
}

// shrink is the set eroded by the cube of half-side one: a cube stays when every cube within one step is in the set.
func (v *voxels) shrink() *voxels {
	var out voxels
	for x := range gridSide {
		for y := range gridSide {
			for z := range gridSide {
				all := true
				for dx := -1; dx <= 1; dx++ {
					for dy := -1; dy <= 1; dy++ {
						for dz := -1; dz <= 1; dz++ {
							all = all && v.at(x+dx, y+dy, z+dz)
						}
					}
				}
				out[x][y][z] = all
			}
		}
	}
	return &out
}

// mesh is the closed outward boundary mesh of the set, one quad of two triangles for each cube face that has no cube
// behind it, vertices shared by coordinate; ok is false when an edge is not in exactly two triangles, so the set
// has cubes that meet only along an edge or at a corner.
func (v *voxels) mesh() (verts []r3.Vec, tris [][3]int, face [][4]int, ok bool) {
	index := map[[3]int]int{}
	vertex := func(p [3]int) int {
		if i, found := index[p]; found {
			return i
		}
		index[p] = len(verts)
		verts = append(verts, r3.Vec{X: float64(p[0]), Y: float64(p[1]), Z: float64(p[2])})
		return len(verts) - 1
	}
	for x := range gridSide {
		for y := range gridSide {
			for z := range gridSide {
				if !v[x][y][z] {
					continue
				}
				cell := [3]int{x, y, z}
				for a := range 3 {
					for _, s := range []int{-1, 1} {
						next := cell
						next[a] += s
						if v.at(next[0], next[1], next[2]) {
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
						// Counter-clockwise in (a1, a2) is outward along +a; the other way round faces -a.
						q := [4]int{corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)}
						if s < 0 {
							q = [4]int{q[0], q[3], q[2], q[1]}
						}
						tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
						face = append(face, q)
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
			return nil, nil, nil, false
		}
	}
	return verts, tris, face, true
}

// declines reports an error the grid route is allowed to give for a shape it does not take, never a wrong answer.
func declines(err error) bool {
	return errors.Is(err, orthoshell.ErrUnsupported) || errors.Is(err, orthoshell.ErrDegenerate)
}

// closedAndAxisSquare requires a mesh to be closed with every edge in two triangles and every triangle square to an axis.
func closedAndAxisSquare(t *testing.T, verts []r3.Vec, tris [][3]int) {
	t.Helper()
	uses := map[[2]int]int{}
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		n := b.Sub(a).Cross(c.Sub(a))
		zeros := 0
		for _, comp := range []float64{n.X, n.Y, n.Z} {
			if comp == 0 {
				zeros++
			}
		}
		require.GreaterOrEqual(t, zeros, 2, "a triangle square to an axis")
		for k := range 3 {
			i, j := tri[k], tri[(k+1)%3]
			uses[[2]int{min(i, j), max(i, j)}]++
		}
	}
	for edge, n := range uses {
		require.Equal(t, 2, n, "edge %v", edge)
	}
}

func TestCombineAgreesWithTheCubeByCubeBoolean(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	checked := 0
	for range 250 {
		a, b := randomVoxels(rng), randomVoxels(rng)
		va, ta, _, okA := a.mesh()
		vb, tb, _, okB := b.mesh()
		if !okA || !okB {
			continue
		}
		for _, op := range []orthoshell.Op{orthoshell.Union, orthoshell.Cut, orthoshell.Intersect} {
			want := a.combine(b, op)
			verts, tris, err := orthoshell.Combine(va, ta, vb, tb, op)
			if err != nil {
				require.True(t, declines(err), "%v", err)
				continue
			}
			checked++
			require.InDelta(t, float64(want.count()), volume(verts, tris), 1e-9, "op %d on %d and %d cubes", op, a.count(), b.count())
		}
		overlap, bound, err := orthoshell.OverlapVolume(va, ta, vb, tb)
		if err == nil {
			require.InDelta(t, float64(a.combine(b, orthoshell.Intersect).count()), overlap, bound+1e-9)
		}
	}
	require.Greater(t, checked, 100, "most random pairs are admitted")
}

func TestOffsetAgreesWithTheCubeByCubeDilationAndErosion(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 4))
	grown, shrunk := 0, 0
	for range 250 {
		a := randomVoxels(rng)
		v, tris, _, ok := a.mesh()
		if !ok {
			continue
		}
		// The set is inside the voxel grid with room only inward, so the grown set may leave it: grow a copy placed in
		// the middle of a bigger grid by counting against a bigger oracle.
		big := bigGrow(a)
		if verts, out, err := orthoshell.Offset(v, tris, 1); err == nil {
			grown++
			closedAndAxisSquare(t, verts, out)
			require.InDelta(t, float64(big), volume(verts, out), 1e-9, "dilating %d cubes", a.count())
		} else {
			require.True(t, declines(err), "%v", err)
		}
		if verts, out, err := orthoshell.Offset(v, tris, -1); err == nil {
			shrunk++
			closedAndAxisSquare(t, verts, out)
			require.InDelta(t, float64(a.shrink().count()), volume(verts, out), 1e-9, "eroding %d cubes", a.count())
		} else {
			require.True(t, declines(err) || a.shrink().count() == 0, "%v", err)
		}
	}
	require.Greater(t, grown, 40)
	require.Greater(t, shrunk, 15)
}

// bigGrow is the number of cubes in the set dilated by one cube, counted on a grid a cube larger each way.
func bigGrow(v *voxels) int {
	n := 0
	for x := -1; x <= gridSide; x++ {
		for y := -1; y <= gridSide; y++ {
			for z := -1; z <= gridSide; z++ {
				found := false
				for dx := -1; dx <= 1 && !found; dx++ {
					for dy := -1; dy <= 1 && !found; dy++ {
						for dz := -1; dz <= 1 && !found; dz++ {
							found = v.at(x+dx, y+dy, z+dz)
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

func TestWallsAgreeWithTheCubeByCubeErosionAndDilation(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 6))
	inward, outward := 0, 0
	for range 250 {
		a := randomVoxels(rng)
		v, tris, _, ok := a.mesh()
		if !ok {
			continue
		}
		none := make([]bool, len(tris))
		// Inward walls of thickness one are the set less its erosion, and outward walls the dilation less the set.
		if verts, out, err := orthoshell.Walls(v, tris, none, 1, false); err == nil {
			inward++
			closedAndAxisSquare(t, verts, out)
			require.InDelta(t, float64(a.count()-a.shrink().count()), volume(verts, out), 1e-9, "inward walls of %d cubes", a.count())
		}
		if verts, out, err := orthoshell.Walls(v, tris, none, 1, true); err == nil {
			outward++
			closedAndAxisSquare(t, verts, out)
			require.InDelta(t, float64(bigGrow(a)-a.count()), volume(verts, out), 1e-9, "outward walls of %d cubes", a.count())
		}
	}
	require.Greater(t, inward, 8, "a set with a cavity inside its walls is common enough")
	require.Greater(t, outward, 40)
}

func TestUndercutsAgreeWithTheCubeByCubeColumns(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 8))
	checked := 0
	for range 250 {
		a := randomVoxels(rng)
		v, tris, faces, ok := a.mesh()
		if !ok {
			continue
		}
		for axis := range 3 {
			blocked, err := orthoshell.Undercuts(v, tris, axis)
			if err != nil {
				require.True(t, declines(err), "%v", err)
				continue
			}
			checked++
			for i, q := range faces {
				// The quad's two triangles are 2i and 2i+1. A face square to the pull axis is blocked when a cube lies
				// beyond it, along its outward normal, in the column over it; any other face is never blocked.
				p0, p1, p2 := v[q[0]], v[q[1]], v[q[2]]
				n := p1.Sub(p0).Cross(p2.Sub(p0))
				comp := [3]float64{n.X, n.Y, n.Z}
				want := false
				if comp[axis] != 0 {
					// The cube the face belongs to is on the side opposite its normal.
					// The face's lower corner is the minimum of its four corners.
					low := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
					for _, k := range q {
						low[0], low[1], low[2] = math.Min(low[0], v[k].X), math.Min(low[1], v[k].Y), math.Min(low[2], v[k].Z)
					}
					cell := [3]int{int(low[0]), int(low[1]), int(low[2])}
					step := 1
					if comp[axis] < 0 {
						step = -1
						// A face turned toward -axis lies on the cube's low side: the column goes down from that cube.
					} else {
						cell[axis]-- // a face turned toward +axis lies on the high side: the cube is one below the plane
					}
					for k := cell[axis] + step; k >= -1 && k <= gridSide; k += step {
						c := cell
						c[axis] = k
						if a.at(c[0], c[1], c[2]) {
							want = true
						}
					}
				}
				require.Equal(t, want, blocked[2*i], "face %d of a %d cube set along axis %d", i, a.count(), axis)
				require.Equal(t, want, blocked[2*i+1])
			}
		}
	}
	require.Greater(t, checked, 100)
}

func TestSectionAreaAgreesWithTheCubeCountOfTheLayer(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(9, 10))
	checked := 0
	for range 250 {
		a := randomVoxels(rng)
		v, tris, _, ok := a.mesh()
		if !ok {
			continue
		}
		for axis := range 3 {
			ia, iv := (axis+1)%3, (axis+2)%3
			for layer := range gridSide {
				want := 0
				for x := range gridSide {
					for y := range gridSide {
						for z := range gridSide {
							if a[x][y][z] && [3]int{x, y, z}[axis] == layer {
								want++
							}
						}
					}
				}
				loops, err := orthoshell.SectionLoops(v, tris, axis, float64(layer)+0.5, ia, iv)
				if err != nil {
					require.True(t, declines(err), "%v", err)
					continue
				}
				checked++
				// An outer loop runs counter-clockwise and a hole clockwise, so the shoelace areas add up to the layer's cubes.
				area := 0.0
				for _, loop := range loops {
					for i, p := range loop {
						q := loop[(i+1)%len(loop)]
						area += (p[0]*q[1] - q[0]*p[1]) / 2
					}
				}
				require.InDelta(t, float64(want), area, 1e-9, "axis %d layer %d of %d cubes", axis, layer, a.count())
			}
		}
	}
	require.Greater(t, checked, 500)
}

func TestConcaveEdgeAgreesWithTheEdgeScan(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 12))
	concave, convexOnly := 0, 0
	for range 400 {
		a := randomVoxels(rng)
		v, tris, _, ok := a.mesh()
		if !ok {
			continue
		}
		// A grid edge is concave when three of the four cubes round it are solid: the material turns inward there.
		want := false
		for axis := range 3 {
			u, w := (axis+1)%3, (axis+2)%3
			for k := range gridSide {
				for cu := 0; cu <= gridSide; cu++ {
					for cw := 0; cw <= gridSide; cw++ {
						n := 0
						for du := -1; du <= 0; du++ {
							for dw := -1; dw <= 0; dw++ {
								var cell [3]int
								cell[axis], cell[u], cell[w] = k, cu+du, cw+dw
								if a.at(cell[0], cell[1], cell[2]) {
									n++
								}
							}
						}
						want = want || n == 3
					}
				}
			}
		}
		got, err := orthoshell.HasConcaveEdge(v, tris)
		if err != nil {
			require.True(t, declines(err), "%v", err)
			continue
		}
		require.Equal(t, want, got, "a set of %d cubes", a.count())
		if got {
			concave++
		} else {
			convexOnly++
		}
	}
	require.Greater(t, concave, 50)
	require.Greater(t, convexOnly, 20, "a single box has no concave edge")
}

func TestMinWallAgreesWithTheShortestRunOfCubes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 14))
	checked, thin := 0, 0
	for range 400 {
		a := randomVoxels(rng)
		v, tris, _, ok := a.mesh()
		if !ok {
			continue
		}
		// The thinnest wall is the shortest maximal run of solid cubes along any line of cubes parallel to an axis.
		want := math.Inf(1)
		for axis := range 3 {
			for p := range gridSide {
				for q := range gridSide {
					run := 0
					for k := 0; k <= gridSide; k++ {
						var c [3]int
						c[axis], c[(axis+1)%3], c[(axis+2)%3] = k, p, q
						if a.at(c[0], c[1], c[2]) {
							run++
							continue
						}
						if run > 0 {
							want = math.Min(want, float64(run))
						}
						run = 0
					}
				}
			}
		}
		got, bound, err := orthoshell.MinWall(v, tris)
		if err != nil {
			require.True(t, declines(err), "%v", err)
			continue
		}
		checked++
		require.InDelta(t, want, got, bound+1e-9, "a set of %d cubes", a.count())
		if want == 1 {
			thin++
		}
	}
	require.Greater(t, checked, 200)
	require.Greater(t, thin, 20, "carving leaves cubes of one thick often enough")
}

// faceRegions groups the quads of a meshed set into planar faces: quads on one plane facing one way that share an edge
// there. Each region is the quad indices, the axis and sign of its normal and the cubes just behind it.
type faceRegion struct {
	quads  []int
	axis   int
	sign   int
	behind [][3]int
}

func (v *voxels) regions(verts []r3.Vec, face [][4]int) []faceRegion {
	type key struct {
		axis, sign int
		plane      float64
	}
	type quad struct {
		key    key
		low    [3]float64
		behind [3]int
	}
	quads := make([]quad, len(face))
	for i, q := range face {
		p0, p1, p2 := verts[q[0]], verts[q[1]], verts[q[2]]
		n := p1.Sub(p0).Cross(p2.Sub(p0))
		comp := [3]float64{n.X, n.Y, n.Z}
		axis := 0
		for a := range 3 {
			if comp[a] != 0 {
				axis = a
			}
		}
		sign := 1
		if comp[axis] < 0 {
			sign = -1
		}
		low := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
		for _, k := range q {
			low[0], low[1], low[2] = math.Min(low[0], verts[k].X), math.Min(low[1], verts[k].Y), math.Min(low[2], verts[k].Z)
		}
		behind := [3]int{int(low[0]), int(low[1]), int(low[2])}
		if sign > 0 {
			behind[axis]-- // a face turned toward +axis is the top of the cube below its plane
		}
		quads[i] = quad{key: key{axis, sign, [3]float64{verts[q[0]].X, verts[q[0]].Y, verts[q[0]].Z}[axis]}, low: low, behind: behind}
	}
	seen := make([]bool, len(quads))
	var out []faceRegion
	for i := range quads {
		if seen[i] {
			continue
		}
		region := faceRegion{axis: quads[i].key.axis, sign: quads[i].key.sign}
		stack := []int{i}
		seen[i] = true
		for len(stack) > 0 {
			j := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			region.quads = append(region.quads, j)
			region.behind = append(region.behind, quads[j].behind)
			for k := range quads {
				if seen[k] || quads[k].key != quads[j].key {
					continue
				}
				// Neighbouring cubes on the plane differ by one step along one of the two other axes.
				d := 0
				for a := range 3 {
					d += int(math.Abs(float64(quads[k].behind[a] - quads[j].behind[a])))
				}
				if d == 1 {
					seen[k] = true
					stack = append(stack, k)
				}
			}
		}
		out = append(out, region)
	}
	return out
}

func TestPressAgreesWithTheCubeByCubeSlab(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(15, 16))
	outward, inward := 0, 0
	for range 300 {
		a := randomVoxels(rng)
		v, tris, face, ok := a.mesh()
		if !ok {
			continue
		}
		regions := a.regions(v, face)
		region := regions[rng.IntN(len(regions))]
		mark := make([]bool, len(tris))
		for _, q := range region.quads {
			mark[2*q], mark[2*q+1] = true, true
		}
		// The slab behind a face is the cubes just under it, and the slab in front the cubes just out from it.
		front := 0
		occupied := false
		for _, c := range region.behind {
			c[region.axis] += region.sign
			if a.at(c[0], c[1], c[2]) {
				occupied = true
			}
			front++
		}
		if verts, out, err := orthoshell.Press(v, tris, mark, 1); err == nil {
			outward++
			require.False(t, occupied, "a face pressed into material is refused")
			closedAndAxisSquare(t, verts, out)
			require.InDelta(t, float64(a.count()+front), volume(verts, out), 1e-9, "pressing a face of %d cubes out", a.count())
		} else {
			require.True(t, declines(err), "%v", err)
		}
		if verts, out, err := orthoshell.Press(v, tris, mark, -1); err == nil {
			inward++
			closedAndAxisSquare(t, verts, out)
			require.InDelta(t, float64(a.count()-len(region.behind)), volume(verts, out), 1e-9, "pressing a face of %d cubes in", a.count())
		} else {
			require.True(t, declines(err), "%v", err)
		}
	}
	require.Greater(t, outward, 30)
	require.Greater(t, inward, 30)
}
