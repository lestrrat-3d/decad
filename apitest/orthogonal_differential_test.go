package apitest_test

import (
	"errors"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
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

func TestVerifySurveysAgreeWithTheCubeByCubeRunsAndColumns(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(25, 26))
	walls, blocked, free := 0, 0, 0
	for range 90 {
		c := randomCubes(rng)
		doc := decad.New()
		if _, ok := c.solid(t, doc); !ok {
			continue
		}
		report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(0.5)),
			decad.WithPullDirection(r3.NewVec(0, 0, 1)))
		require.NoError(t, err)
		body := report.Bodies[0]
		// The thinnest wall is the shortest maximal run of solid cubes along any line of cubes parallel to an axis.
		wantWall := float64(voxelSide + 1)
		for axis := range 3 {
			for p := range voxelSide {
				for q := range voxelSide {
					run := 0
					for k := 0; k <= voxelSide; k++ {
						var cell [3]int
						cell[axis], cell[(axis+1)%3], cell[(axis+2)%3] = k, p, q
						if c.at(cell[0], cell[1], cell[2]) {
							run++
							continue
						}
						if run > 0 {
							wantWall = min(wantWall, float64(run))
						}
						run = 0
					}
				}
			}
		}
		if body.Wall.Minimum != nil {
			walls++
			got, err := body.Wall.Minimum.Value.In(units.Millimeter)
			require.NoError(t, err)
			bound, err := body.Wall.Minimum.Bound.In(units.Millimeter)
			require.NoError(t, err)
			require.InDelta(t, wantWall, got, bound+1e-9, "thinnest wall of %d cubes", c.count())
		}
		// A pull along +z is blocked when some column of cubes holds a solid cube, an empty one and a solid one again.
		want := false
		for x := range voxelSide {
			for y := range voxelSide {
				seenSolid, seenGap := false, false
				for z := range voxelSide {
					switch {
					case c[x][y][z] && seenGap:
						want = true
					case c[x][y][z]:
						seenSolid = true
					case seenSolid:
						seenGap = true
					}
				}
			}
		}
		if body.Undercut.Coverage == decad.CoverageComplete {
			if want {
				blocked++
				require.NotEmpty(t, body.Undercut.Faces, "an undercut in a column of %d cubes", c.count())
			} else {
				free++
				require.Empty(t, body.Undercut.Faces, "no undercut in a set of %d cubes", c.count())
			}
		}
	}
	require.Greater(t, walls, 40)
	require.Greater(t, blocked, 4)
	require.Greater(t, free, 25)
}

func TestQuarterTurnAgreesWithTheTurnedCubes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(27, 28))
	checked := 0
	for range 30 {
		c := randomCubes(rng)
		doc := decad.New()
		b, ok := c.solid(t, doc)
		if !ok {
			continue
		}
		about := r3.NewVec(float64(rng.IntN(voxelSide)), float64(rng.IntN(voxelSide)), 0)
		turns := 1 + rng.IntN(3)
		out, err := decad.QuarterTurn(t.Context(), b, r3.NewVec(0, 0, 1), about, turns)
		if err != nil {
			continue
		}
		checked++
		got, bound := volumeOf(t, out)
		require.InDelta(t, float64(c.count()), got, bound+1e-9, "a turn keeps the volume")
		require.Zero(t, bound, "a quarter turn is exact")
		// A cube at (x, y, z) goes to the cube whose centre is its centre turned about the axis, a quarter turn at a time
		// counter-clockwise: (dx, dy) to (-dy, dx).
		for x := range voxelSide {
			for y := range voxelSide {
				dx, dy := float64(x)+0.5-about.X, float64(y)+0.5-about.Y
				for range turns {
					dx, dy = -dy, dx
				}
				for z := range voxelSide {
					p := r3.NewVec(about.X+dx, about.Y+dy, float64(z)+0.5)
					at, err := out.LocatePoint(t.Context(), p, units.Millimeters(0.05))
					require.NoError(t, err)
					want := decad.PointOutside
					if c[x][y][z] {
						want = decad.PointInside
					}
					require.Equal(t, want, at, "cube %d,%d,%d after %d turns about %v", x, y, z, turns, about)
				}
			}
		}
	}
	require.Greater(t, checked, 12)
}

func TestStretchAgreesWithTheStretchedCubes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(29, 30))
	factors := []float64{2, 3, 0.5, -1, -2}
	checked := 0
	for range 40 {
		c := randomCubes(rng)
		doc := decad.New()
		b, ok := c.solid(t, doc)
		if !ok {
			continue
		}
		var f [3]float64
		for a := range 3 {
			f[a] = factors[rng.IntN(len(factors))]
		}
		anchor := r3.NewVec(float64(rng.IntN(voxelSide)), float64(rng.IntN(voxelSide)), float64(rng.IntN(voxelSide)))
		out, err := decad.Stretch(t.Context(), b, anchor, units.Scalar(f[0]), units.Scalar(f[1]), units.Scalar(f[2]))
		if err != nil {
			continue
		}
		checked++
		got, bound := volumeOf(t, out)
		want := float64(c.count()) * math.Abs(f[0]*f[1]*f[2])
		require.InDelta(t, want, got, bound+1e-9, "stretching %d cubes by %v", c.count(), f)
		// The centre of the cube at (x, y, z) goes to the anchor plus each of its offsets times its factor, and the cube
		// it stood in is wholly the stretched set's box there, so the point is inside when the cube was solid.
		for range 40 {
			x, y, z := rng.IntN(voxelSide), rng.IntN(voxelSide), rng.IntN(voxelSide)
			p := r3.NewVec(
				anchor.X+(float64(x)+0.5-anchor.X)*f[0],
				anchor.Y+(float64(y)+0.5-anchor.Y)*f[1],
				anchor.Z+(float64(z)+0.5-anchor.Z)*f[2])
			at, err := out.LocatePoint(t.Context(), p, units.Millimeters(0.05))
			require.NoError(t, err)
			wantAt := decad.PointOutside
			if c[x][y][z] {
				wantAt = decad.PointInside
			}
			require.Equal(t, wantAt, at, "cube %d,%d,%d stretched by %v about %v", x, y, z, f, anchor)
		}
	}
	require.Greater(t, checked, 20)
}

func TestCircularPatternAgreesWithTheUnionOfTurnedCubes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(31, 32))
	checked := 0
	for range 40 {
		c := randomCubes(rng)
		doc := decad.New()
		b, ok := c.solid(t, doc)
		if !ok {
			continue
		}
		count := []int{2, 4}[rng.IntN(2)]
		centre := r3.NewVec(float64(rng.IntN(voxelSide+1)), float64(rng.IntN(voxelSide+1)), 0)
		out, err := decad.Patterned(t.Context(), b, decad.CircularPattern{Center: centre, Axis: r3.NewVec(0, 0, 1), Count: count})
		if err != nil {
			// A union whose kept cubes meet only along an edge, or too many cubes: refused, never a wrong volume.
			continue
		}
		checked++
		// The cubes covered by the cube set and its copies turned 360/count degrees at a time about the axis: a cube turns to
		// the cube whose centre is the turned centre.
		covered := map[[3]int]bool{}
		for x := range voxelSide {
			for y := range voxelSide {
				for z := range voxelSide {
					if !c[x][y][z] {
						continue
					}
					dx, dy := float64(x)+0.5-centre.X, float64(y)+0.5-centre.Y
					for range count {
						covered[[3]int{int(math.Floor(centre.X + dx)), int(math.Floor(centre.Y + dy)), z}] = true
						for range 4 / count {
							dx, dy = -dy, dx
						}
					}
				}
			}
		}
		got, bound := volumeOf(t, out)
		require.InDelta(t, float64(len(covered)), got, bound+1e-9, "%d turned copies of %d cubes about %v", count, c.count(), centre)
	}
	require.Greater(t, checked, 12)
}

func TestMeshSolidWithAPrismAgreesWithTheCubeByCubeBoolean(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(33, 34))
	checked, exact := 0, 0
	for range 60 {
		c := randomCubes(rng)
		// A rectangular prism on integer coordinates, lifted to a random height: the mixed route of B7 reads it beside the
		// mesh solid.
		lo := [3]int{rng.IntN(voxelSide - 1), rng.IntN(voxelSide - 1), rng.IntN(voxelSide - 1)}
		var hi [3]int
		for a := range 3 {
			hi[a] = lo[a] + 1 + rng.IntN(voxelSide-lo[a])
		}
		for _, op := range []differentialOp{opUnion, opCut, opIntersect} {
			doc := decad.New()
			mesh, ok := c.solid(t, doc)
			if !ok {
				continue
			}
			block := decadtest.NewBlock(t, doc, float64(lo[0]), float64(lo[1]), float64(hi[0]), float64(hi[1]), units.Millimeters(float64(hi[2]-lo[2])))
			lift, err := r3.Translation(r3.NewVec(0, 0, float64(lo[2])))
			require.NoError(t, err)
			prism, err := decad.Transform(t.Context(), block, lift)
			require.NoError(t, err)
			want := 0
			for x := range voxelSide {
				for y := range voxelSide {
					for z := range voxelSide {
						inPrism := x >= lo[0] && x < hi[0] && y >= lo[1] && y < hi[1] && z >= lo[2] && z < hi[2]
						switch op {
						case opUnion:
							if c[x][y][z] || inPrism {
								want++
							}
						case opCut:
							if c[x][y][z] && !inPrism {
								want++
							}
						default:
							if c[x][y][z] && inPrism {
								want++
							}
						}
					}
				}
			}
			var out *decad.Body
			switch op {
			case opUnion:
				out, err = decad.Union(t.Context(), mesh, prism)
			case opCut:
				out, err = decad.Cut(t.Context(), mesh, prism)
			default:
				out, err = decad.Intersect(t.Context(), mesh, prism)
			}
			if err != nil {
				continue
			}
			checked++
			got, bound := volumeOf(t, out)
			require.InDelta(t, float64(want), got, bound+1e-9, "op %d of %d cubes and a prism %v to %v", op, c.count(), lo, hi)
			if bound == 0 {
				exact++
			}
		}
	}
	require.Greater(t, checked, 60)
	require.Greater(t, exact, checked*8/10, "the mixed route answers nearly every pair exactly")
}

func TestMirroredCopyAndUnionAgreeWithTheMirroredCubes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(35, 36))
	checked := 0
	for range 40 {
		c := randomCubes(rng)
		doc := decad.New()
		b, ok := c.solid(t, doc)
		if !ok {
			continue
		}
		// Mirror across the plane x = k, k a whole number from 0 to the grid's side: the cube at x goes to the cube at
		// 2k - 1 - x, along each of y and z unchanged.
		k := rng.IntN(voxelSide + 1)
		frame, err := r3.NewFrame(r3.NewVec(float64(k), 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		image, err := decad.MirroredCopy(t.Context(), b, decad.MirrorFrame{Frame: frame})
		require.NoError(t, err)
		got, bound := volumeOf(t, image)
		require.InDelta(t, float64(c.count()), got, bound+1e-9, "a mirror keeps the volume")
		both, err := decad.Union(t.Context(), b, image)
		if err != nil {
			continue // cubes that meet only along an edge are refused, never a wrong volume
		}
		checked++
		covered := map[[3]int]bool{}
		for x := range voxelSide {
			for y := range voxelSide {
				for z := range voxelSide {
					if c[x][y][z] {
						covered[[3]int{x, y, z}] = true
						covered[[3]int{2*k - 1 - x, y, z}] = true
					}
				}
			}
		}
		got, bound = volumeOf(t, both)
		require.InDelta(t, float64(len(covered)), got, bound+1e-9, "a set of %d cubes joined to its mirror at x = %d", c.count(), k)
	}
	require.Greater(t, checked, 15)
}
