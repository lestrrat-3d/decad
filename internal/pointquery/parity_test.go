package pointquery

import (
	"errors"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// ratObliqueParity is the normalised big.Rat form of obliqueParity, the
// reference the dyadic scan must match. It forms the ray parameter and the
// meeting point and tests the meeting point against each edge directly. It
// also returns the index of the ladder direction that decided the query, or
// len(obliqueDirections) when none did.
func ratObliqueParity(budget *proofbound.WorkBudget, p r3.Vec, verts []r3.Vec, tris [][3]int) (bool, bool, int, error) {
	point := exactVec(p)
	converted := make([]ratVec, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return false, false, 0, err
		}
		converted[i] = exactVec(v)
	}
	for depth, values := range obliqueDirections {
		dir := ratVec{
			new(big.Rat).SetInt64(values[0]),
			new(big.Rat).SetInt64(values[1]),
			new(big.Rat).SetInt64(values[2]),
		}
		crossings := 0
		ambiguous := false
		for _, tri := range tris {
			if err := budget.Step(); err != nil {
				return false, false, 0, err
			}
			a, b, c := converted[tri[0]], converted[tri[1]], converted[tri[2]]
			normal := cross(subtract(b, a), subtract(c, a))
			denom := dot(normal, dir)
			numerator := dot(normal, subtract(a, point))
			if denom.Sign() == 0 {
				if numerator.Sign() == 0 {
					ambiguous = true
					break
				}
				continue
			}
			parameter := new(big.Rat).Quo(numerator, denom)
			if parameter.Sign() <= 0 {
				continue
			}
			intersection := ratVec{}
			for i := range intersection {
				intersection[i] = new(big.Rat).Add(point[i], new(big.Rat).Mul(parameter, dir[i]))
			}
			vertices := [3]ratVec{a, b, c}
			inside := true
			grazes := false
			for i := range vertices {
				sign := dot(cross(subtract(vertices[(i+1)%3], vertices[i]),
					subtract(intersection, vertices[i])), normal).Sign()
				if sign < 0 {
					inside = false
					break
				}
				if sign == 0 {
					grazes = true
				}
			}
			if !inside {
				continue
			}
			if grazes {
				ambiguous = true
				break
			}
			crossings++
		}
		if !ambiguous {
			return crossings%2 == 1, true, depth, nil
		}
	}
	return false, false, len(obliqueDirections), nil
}

var errStopParity = errors.New("stop")

// countingBudget counts every step and fails the failAt-th one; failAt 0
// never fails.
func countingBudget(failAt int) (*proofbound.WorkBudget, *int) {
	steps := new(int)
	return &proofbound.WorkBudget{
		StepFn: func() error {
			*steps++
			if *steps == failAt {
				return errStopParity
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}, steps
}

// parityOutcome is the bounded record two scans must agree on.
type parityOutcome struct {
	Inside, Resolved, Failed bool
	Steps                    int
}

// parityMesh is one fixture mesh with the query points to ask about it.
type parityMesh struct {
	name   string
	verts  []r3.Vec
	tris   [][3]int
	points []r3.Vec
}

// grazingPoints samples count anchors among the vertices, edge midpoints and
// facet centroids. It returns each anchor, and a point a quarter or a half of
// a random ladder direction d before it, whose ray along d passes through
// the anchor.
func grazingPoints(rng *rand.Rand, verts []r3.Vec, tris [][3]int, count int) []r3.Vec {
	var anchors []r3.Vec
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		anchors = append(anchors, a, a.Add(b).Scale(0.5), b.Add(c).Scale(0.5),
			a.Add(b).Add(c).Scale(1.0/3))
	}
	var points []r3.Vec
	for range count {
		anchor := anchors[rng.IntN(len(anchors))]
		points = append(points, anchor)
		values := obliqueDirections[rng.IntN(len(obliqueDirections))]
		dir := r3.NewVec(float64(values[0]), float64(values[1]), float64(values[2]))
		points = append(points, anchor.Sub(dir.Scale([]float64{0.25, 0.5}[rng.IntN(2)])))
	}
	return points
}

// parityMeshes builds the fixture families: axis, quarter-turned and tilted
// unit-square blocks with their voxel centres, randomly turned cubes, random
// tetrahedra, a tetrahedron with a facet parallel to the first ladder
// direction, a block carrying a degenerate triangle, spheres and slivers.
func parityMeshes(rng *rand.Rand, trials int) []parityMesh {
	var out []parityMesh
	add := func(name string, verts []r3.Vec, tris [][3]int, extra ...r3.Vec) {
		points := append(append([]r3.Vec(nil), extra...), grazingPoints(rng, verts, tris, 12)...)
		points = append(points, meshQueryPoints(rng, verts, 4)...)
		out = append(out, parityMesh{name: name, verts: verts, tris: tris, points: points})
	}
	verts, tris := blockSurface(2)
	add("block", verts, tris, voxelCentres(2)...)
	verts3, tris3 := blockSurface(3)
	about := r3.NewVec(1, 2, 0)
	add("quarter-turned block", quarterTurned(verts3, about), tris3, quarterTurned(voxelCentres(3), about)...)
	centre := r3.NewVec(1, 1, 1)
	add("tilted block", tilted(verts, centre, r3.NewVec(1, 2, 2), 0.3), tris,
		tilted(voxelCentres(2), centre, r3.NewVec(1, 2, 2), 0.3)...)
	plane := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 1), r3.NewVec(0, 0, -1)}
	planeTris := [][3]int{{0, 1, 2}, {0, 3, 1}, {1, 3, 2}, {2, 3, 0}}
	add("parallel facet", plane, planeTris,
		r3.NewVec(0.25, 0.25, 0.5), r3.NewVec(2, 1, 3), r3.NewVec(-1, -1, -2), r3.NewVec(0.1, 0.1, -0.2))
	degenerate := append(append([]r3.Vec(nil), verts...), r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1), r3.NewVec(2, 2, 2))
	degenerateTris := append(append([][3]int(nil), tris...), [3]int{len(verts), len(verts) + 1, len(verts) + 2})
	add("degenerate", degenerate, degenerateTris, voxelCentres(2)...)
	cubeVerts, cubeTris := blockSurface(1)
	for range trials {
		axis := r3.NewVec(rng.Float64()-0.5, rng.Float64()-0.5, rng.Float64()-0.5)
		if axis.Len() < 0.1 {
			axis = r3.NewVec(0, 0, 1)
		}
		shift := r3.NewVec(float64(rng.IntN(5))-2, float64(rng.IntN(5))*0.25, rng.Float64())
		turned := tilted(cubeVerts, r3.NewVec(0.5, 0.5, 0.5), axis, rng.Float64()*6)
		for i := range turned {
			turned[i] = turned[i].Add(shift)
		}
		add("turned cube", turned, cubeTris, r3.NewVec(0.5, 0.5, 0.5).Add(shift))
		tet := make([]r3.Vec, 4)
		for i := range tet {
			if rng.IntN(2) == 0 {
				tet[i] = r3.NewVec(float64(rng.IntN(9))/4, float64(rng.IntN(9))/4, float64(rng.IntN(9))/4)
				continue
			}
			tet[i] = r3.NewVec(rng.Float64()*2, rng.Float64()*2, rng.Float64()*2)
		}
		add("tetrahedron", tet, [][3]int{{0, 1, 2}, {0, 3, 1}, {1, 3, 2}, {2, 3, 0}},
			tet[0].Add(tet[1]).Add(tet[2]).Add(tet[3]).Scale(0.25))
		sphereVerts, sphereTris := uvSphere(3+rng.IntN(5), 3+rng.IntN(6), 0.5+rng.Float64()*2)
		add("sphere", sphereVerts, sphereTris, r3.NewVec(0, 0, 0))
		sliverVerts, sliverTris := sliverMesh(rng, 1+rng.IntN(12), 1)
		add("slivers", sliverVerts, sliverTris)
	}
	return out
}

// tilted rotates every point by angle radians about axis through centre.
func tilted(points []r3.Vec, centre, axis r3.Vec, angle float64) []r3.Vec {
	rotation, err := r3.RotationAround(centre, axis, units.Radians(angle))
	if err != nil {
		panic(err)
	}
	out := make([]r3.Vec, len(points))
	for i, p := range points {
		out[i] = rotation.Apply(p)
	}
	return out
}

// TestObliqueParityMatchesRatReference holds the dyadic scan to the big.Rat
// reference: the same inside and resolved answers and the same budget step
// count. With the budget failing at the first, a middle or the last of those
// steps, the dyadic scan must stop at that step. The fixture must reach every
// outcome and decide queries past the first ladder direction. Under the race
// detector it runs 1 random trial instead of 5.
func TestObliqueParityMatchesRatReference(t *testing.T) {
	t.Parallel()
	trials := 5
	if raceDetector {
		trials = 1
	}
	rng := rand.New(rand.NewPCG(31, 37))
	depths := make([]int, len(obliqueDirections)+1)
	var insides, outsides int
	run := func(failAt int, scan func(*proofbound.WorkBudget) (bool, bool, error)) parityOutcome {
		budget, steps := countingBudget(failAt)
		inside, resolved, err := scan(budget)
		if err != nil {
			require.ErrorIs(t, err, errStopParity)
		}
		return parityOutcome{Inside: inside, Resolved: resolved, Failed: err != nil, Steps: *steps}
	}
	for _, mesh := range parityMeshes(rng, trials) {
		for _, p := range mesh.points {
			depth := 0
			reference := func(budget *proofbound.WorkBudget) (bool, bool, error) {
				inside, resolved, d, err := ratObliqueParity(budget, p, mesh.verts, mesh.tris)
				depth = d
				return inside, resolved, err
			}
			dyadic := func(budget *proofbound.WorkBudget) (bool, bool, error) {
				return obliqueParity(budget, p, mesh.verts, mesh.tris)
			}
			want := run(0, reference)
			require.Equal(t, want, run(0, dyadic), "%s point %v", mesh.name, p)
			depths[depth]++
			switch {
			case want.Resolved && want.Inside:
				insides++
			case want.Resolved:
				outsides++
			}
			// A budget step is the only source of an error, so a scan that
			// fails at step k ≤ want.Steps stops there with no answer.
			for _, failAt := range []int{1, 1 + want.Steps/2, want.Steps} {
				require.Equal(t, parityOutcome{Failed: true, Steps: failAt}, run(failAt, dyadic),
					"%s point %v failing at step %d", mesh.name, p, failAt)
			}
		}
	}
	t.Logf("decided at ladder depth (last = exhausted): %v; inside %d, outside %d", depths, insides, outsides)
	require.Positive(t, insides, "some query is inside")
	require.Positive(t, outsides, "some query is outside")
	require.Positive(t, depths[len(obliqueDirections)], "some query exhausts the ladder")
	require.Positive(t, depths[1]+depths[2]+depths[3], "some query is decided past the first direction")
}

// blockSurface triangulates the surface of the box [0, n]^3 into unit
// squares, two triangles each, split along the square's diagonal. A voxel
// centre's six axis rays each meet such a diagonal, so every centre takes
// the oblique ladder.
func blockSurface(n int) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var tris [][3]int
	vec := func(c [3]float64) r3.Vec { return r3.NewVec(c[0], c[1], c[2]) }
	for k := range 3 {
		u, v := (k+1)%3, (k+2)%3
		for _, side := range []int{0, n} {
			for i := range n {
				for j := range n {
					var p0, p1, p2, p3 [3]float64
					p0[k], p0[u], p0[v] = float64(side), float64(i), float64(j)
					p1, p2, p3 = p0, p0, p0
					p1[u]++
					p2[u]++
					p2[v]++
					p3[v]++
					base := len(verts)
					verts = append(verts, vec(p0), vec(p1), vec(p2), vec(p3))
					if side == 0 {
						tris = append(tris, [3]int{base, base + 2, base + 1}, [3]int{base, base + 3, base + 2})
						continue
					}
					tris = append(tris, [3]int{base, base + 1, base + 2}, [3]int{base, base + 2, base + 3})
				}
			}
		}
	}
	return verts, tris
}

// voxelCentres lists the centre of every unit voxel of the box [0, n]^3.
func voxelCentres(n int) []r3.Vec {
	var out []r3.Vec
	for x := range n {
		for y := range n {
			for z := range n {
				out = append(out, r3.NewVec(float64(x)+0.5, float64(y)+0.5, float64(z)+0.5))
			}
		}
	}
	return out
}

// quarterTurned turns every point a quarter turn counter-clockwise about the
// z axis through about. The fixture swaps coordinates so the turn stays exact.
func quarterTurned(points []r3.Vec, about r3.Vec) []r3.Vec {
	out := make([]r3.Vec, len(points))
	for i, p := range points {
		out[i] = r3.NewVec(about.X-(p.Y-about.Y), about.Y+(p.X-about.X), p.Z)
	}
	return out
}

func benchmarkObliqueParity(b *testing.B, points, verts []r3.Vec, tris [][3]int) {
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		for _, p := range points {
			_, _, err := ObliqueParity(b.Context(), p, verts, tris)
			require.NoError(b, err)
		}
	}
}

func BenchmarkObliqueParity(b *testing.B) {
	verts, tris := blockSurface(3)
	centres := voxelCentres(3)
	about := r3.NewVec(1, 2, 0)
	b.Run("quarter-turned", func(b *testing.B) {
		benchmarkObliqueParity(b, quarterTurned(centres, about), quarterTurned(verts, about), tris)
	})
	centre, axis := r3.NewVec(1.5, 1.5, 1.5), r3.NewVec(1, 2, 2)
	b.Run("tilted", func(b *testing.B) {
		benchmarkObliqueParity(b, tilted(centres, centre, axis, 0.3), tilted(verts, centre, axis, 0.3), tris)
	})
}
