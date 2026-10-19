package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// groupFixture is a box set with the family of each box, as the grouped S8
// count sees one: -1 for a box without a proven patch.
type groupFixture struct {
	name   string
	boxes  [][2]r3.Vec
	family []int
}

func (g groupFixture) scan() groupedTreeScan {
	return newGroupedTreeScan(g.boxes, allMembers(len(g.boxes)), g.family)
}

// fanTriangles is the triangle count of one fan.
const fanTriangles = 600

// addFan appends the triangle boxes of a flat fan of radius 5 around (cx, cy)
// at height z, all in family fam.
func (g *groupFixture) addFan(cx, cy, z float64, fam int) {
	for k := range fanTriangles {
		a0, a1 := 2*math.Pi*float64(k)/fanTriangles, 2*math.Pi*float64(k+1)/fanTriangles
		p := [3]r3.Vec{
			r3.NewVec(cx, cy, z),
			r3.NewVec(cx+5*math.Cos(a0), cy+5*math.Sin(a0), z),
			r3.NewVec(cx+5*math.Cos(a1), cy+5*math.Sin(a1), z),
		}
		lo, hi := p[0], p[0]
		for _, v := range p[1:] {
			lo = r3.NewVec(min(lo.X, v.X), min(lo.Y, v.Y), min(lo.Z, v.Z))
			hi = r3.NewVec(max(hi.X, v.X), max(hi.Y, v.Y), max(hi.Z, v.Z))
		}
		g.boxes = append(g.boxes, [2]r3.Vec{lo, hi})
		g.family = append(g.family, fam)
	}
}

// addWalls appends n random boxes of family fam, centred in [-5,5]^2 and
// zLo..zHi with half-extents up to half in x and y and halfZ in z.
func (g *groupFixture) addWalls(rng *rand.Rand, n int, zLo, zHi, half, halfZ float64, fam int) {
	for range n {
		c := r3.NewVec(10*rng.Float64()-5, 10*rng.Float64()-5, zLo+(zHi-zLo)*rng.Float64())
		h := r3.NewVec(half*rng.Float64(), half*rng.Float64(), halfZ*rng.Float64())
		g.boxes = append(g.boxes, [2]r3.Vec{c.Sub(h), c.Add(h)})
		g.family = append(g.family, fam)
	}
}

// addRandom appends n random boxes in [0,10]x[0,10]x[0,1] whose family cycles
// through fams.
func (g *groupFixture) addRandom(rng *rand.Rand, n int, fams []int) {
	for k := range n {
		c := r3.NewVec(10*rng.Float64(), 10*rng.Float64(), rng.Float64())
		h := r3.NewVec(0.3*rng.Float64(), 0.3*rng.Float64(), 0.3*rng.Float64())
		g.boxes = append(g.boxes, [2]r3.Vec{c.Sub(h), c.Add(h)})
		g.family = append(g.family, fams[k%len(fams)])
	}
}

func groupFixtures() []groupFixture {
	rng := rand.New(rand.NewPCG(97, 101))
	var a, b, c, d, e groupFixture
	// (a) two flat fans with walls between them: walls miss both fans' planes.
	a.name = "walls between fans"
	a.addFan(0, 0, 0, 0)
	a.addFan(1, 1, 4, 1)
	a.addWalls(rng, 3000, 1, 3, 0.5, 0.5, -1)
	// (b) the same, with walls that reach the fan at z = 0.
	b.name = "walls reach a fan"
	b.addFan(0, 0, 0, 0)
	b.addFan(1, 1, 4, 1)
	b.addWalls(rng, 3000, 0, 4, 0.5, 1, -1)
	// (c) random boxes in three families.
	c.name = "three random families"
	c.addRandom(rng, 4200, []int{-1, 0, 1})
	// (d) one family, -1 only.
	d.name = "only -1"
	d.addRandom(rng, 4200, []int{-1})
	// (e) two proven families and no -1 group.
	e.name = "no -1 group"
	e.addRandom(rng, 4200, []int{0, 1})
	return []groupFixture{a, b, c, d, e}
}

func TestGroupedCountParallelMatchesVisit(t *testing.T) {
	keeps := map[string]func(i, j int) bool{
		"every": keepEveryPair,
		"mod3":  func(i, j int) bool { return (i+j)%3 != 0 },
	}
	for _, fixture := range groupFixtures() {
		t.Run(fixture.name, func(t *testing.T) {
			scan := fixture.scan()
			require.GreaterOrEqual(t, scan.members(), loftTreeParallelMin)
			f := len(fixture.boxes)
			full, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, f, keepEveryPair, math.MaxUint64, 1)
			require.NoError(t, err)
			total := full.scanned
			require.Greater(t, total, uint64(1000))
			require.Positive(t, full.count)
			limits := []uint64{math.MaxUint64, total, total - 1, total / 2, 1000, 0}
			for name, keep := range keeps {
				t.Run(name, func(t *testing.T) {
					for _, limit := range limits {
						want, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, f, keep, limit, 1)
						require.NoError(t, err)
						got, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, f, keep, limit, 4)
						require.NoError(t, err)
						require.Equal(t, want, got, "limit %d", limit)
					}
					// A budget that fails part-way fails at the same step with
					// the same error as the serial pass.
					for _, k := range []uint64{0, 1, total / 2, total - 1, total} {
						var wantSteps, gotSteps uint64
						want, wantErr := sweepCandidateCountsWorkers(countingBudget(k, &wantSteps), scan, f, keep,
							math.MaxUint64, 1)
						got, gotErr := sweepCandidateCountsWorkers(countingBudget(k, &gotSteps), scan, f, keep,
							math.MaxUint64, 4)
						require.Equal(t, want, got, "budget %d", k)
						require.Equal(t, wantErr, gotErr, "budget %d", k)
						require.Equal(t, wantSteps, gotSteps, "budget %d", k)
					}
				})
			}
		})
	}
}

func TestGroupedCountDispatch(t *testing.T) {
	// sweepCandidateCounts picks the worker count from the member total, and
	// below loftTreeParallelMin the explicit worker count is ignored.
	fixture := groupFixtures()[2]
	small := groupFixture{boxes: fixture.boxes[:loftTreeParallelMin-1], family: fixture.family[:loftTreeParallelMin-1]}
	for _, f := range []groupFixture{fixture, small} {
		scan := f.scan()
		want, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, len(f.boxes), keepEveryPair,
			math.MaxUint64, 1)
		require.NoError(t, err)
		for _, workers := range []int{2, 4} {
			got, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, len(f.boxes), keepEveryPair,
				math.MaxUint64, workers)
			require.NoError(t, err)
			require.Equal(t, want, got)
		}
		got, err := sweepCandidateCounts(unlimitedBudget(t), scan, len(f.boxes), keepEveryPair, math.MaxUint64)
		require.NoError(t, err)
		require.Equal(t, want, got)
	}
}

func TestCrossTasksPartitionCross(t *testing.T) {
	scan := groupFixtures()[1].scan()
	require.Equal(t, []int{-1, 0, 1}, scan.keys)
	a, b := &scan.groups[0], &scan.groups[1]

	// cross is groupedTreeScan.visit's cross, counting units and pairs.
	var wantUnits uint64
	wantPairs := map[[2]int]int{}
	var cross func(ai, bi int)
	cross = func(ai, bi int) {
		wantUnits++
		na, nb := a.nodes[ai], b.nodes[bi]
		if !meshbool.BoxesOverlap(na.box, nb.box) {
			return
		}
		if na.left < 0 && nb.left < 0 {
			for _, ia := range a.order[na.start:na.end] {
				for _, ib := range b.order[nb.start:nb.end] {
					wantUnits++
					i, j := int(ia), int(ib)
					if !meshbool.BoxesOverlap(a.boxes[i], b.boxes[j]) {
						continue
					}
					wantPairs[[2]int{min(i, j), max(i, j)}]++
				}
			}
			return
		}
		if nb.left < 0 || (na.left >= 0 && na.end-na.start >= nb.end-nb.start) {
			cross(na.left, bi)
			cross(na.right, bi)
			return
		}
		cross(ai, nb.left)
		cross(ai, nb.right)
	}
	cross(0, 0)
	require.Greater(t, wantUnits, uint64(1000))
	require.NotEmpty(t, wantPairs)

	for _, want := range []int{1, 2, 7, 64} {
		tasks, used := crossTasks(a, b, want)
		require.GreaterOrEqual(t, len(tasks), want)
		units := used
		pairs := map[[2]int]int{}
		for _, task := range tasks {
			w := crossWalker{a: a, b: b, limit: math.MaxUint64, fn: func(i, j int) { pairs[[2]int{i, j}]++ }}
			require.False(t, w.walk(task[0], task[1]))
			units += w.scanned
		}
		require.Equal(t, wantUnits, units, "want %d tasks", want)
		require.Equal(t, wantPairs, pairs, "want %d tasks", want)
	}
}
