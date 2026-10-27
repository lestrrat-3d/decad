package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// radialLugFixture is a hub with lugs around it, as boxes: the shape of a
// sector join's audit. Each lug has two flat caps, each its own proven
// family, and wall boxes of family -1 between them; the hub is family -1 too.
// Most rows therefore meet many family trees whose roots lie far away.
func radialLugFixture(lugs, capBoxes, wallBoxes, hubBoxes int) groupFixture {
	rng := rand.New(rand.NewPCG(5, 7))
	var g groupFixture
	g.name = "radial lugs"
	add := func(c r3.Vec, h r3.Vec, fam int) {
		g.boxes = append(g.boxes, [2]r3.Vec{c.Sub(h), c.Add(h)})
		g.family = append(g.family, fam)
	}
	for range hubBoxes {
		a := 2 * math.Pi * rng.Float64()
		r := 3 + 2*rng.Float64()
		add(r3.NewVec(r*math.Cos(a), r*math.Sin(a), 4*rng.Float64()), r3.NewVec(0.3, 0.3, 0.3), -1)
	}
	for k := range lugs {
		theta := 2 * math.Pi * float64(k) / float64(lugs)
		d := r3.NewVec(math.Cos(theta), math.Sin(theta), 0)
		n := r3.NewVec(-math.Sin(theta), math.Cos(theta), 0)
		// The axis-aligned half extents of a rectangle with half sides hd
		// along d, hn along n and hz along z.
		half := func(hd, hn, hz float64) r3.Vec {
			return r3.NewVec(math.Abs(d.X)*hd+math.Abs(n.X)*hn, math.Abs(d.Y)*hd+math.Abs(n.Y)*hn, hz)
		}
		for side := range 2 {
			offset := n.Scale(float64(2*side-1) * 0.5)
			for range capBoxes {
				c := d.Scale(5 + 5*rng.Float64()).Add(offset).Add(r3.NewVec(0, 0, 1+2*rng.Float64()))
				add(c, half(0.2*rng.Float64(), 1e-3, 0.2*rng.Float64()), 2*k+side)
			}
		}
		for range wallBoxes {
			c := d.Scale(5 + 5*rng.Float64()).Add(n.Scale(rng.Float64() - 0.5))
			c.Z = 1 + 2*float64(rng.IntN(2))
			add(c, half(0.2*rng.Float64(), 0.2*rng.Float64(), 1e-3), -1)
		}
	}
	return g
}

// BenchmarkGroupedRowFill measures the parallel row fill of a sector join's
// candidate list over a grouped scan with many proven families.
func BenchmarkGroupedRowFill(b *testing.B) {
	g := radialLugFixture(40, 300, 40, 800)
	scan := g.scan()
	members := allMembers(len(g.boxes))
	sc, err := sweepCandidateCountsWorkers(proofbound.NewWorkBudget(b.Context()), scan, len(g.boxes),
		keepEveryPair, math.MaxUint64, 1)
	if err != nil || sc.exceeded || sc.count == 0 {
		b.Fatalf("count: %v %v %d", err, sc.exceeded, sc.count)
	}
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		c, err := sweepCandidatesRowsWorkers(proofbound.NewWorkBudget(b.Context()), scan, members, sc.starts,
			sc.count, sc.scanned, 4)
		if err != nil || uint64(len(c.js)) != sc.count {
			b.Fatalf("fill: %v %d", err, len(c.js))
		}
	}
}
