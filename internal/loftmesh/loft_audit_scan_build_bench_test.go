package loftmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
)

// twistedRibbonBoxes is the triangle boxes of a twisted gear-like ribbon:
// sections stacked step apart in z, each a closed profile of points vertices
// on a wavy circle with teeth lobes, each section turned twist radians from
// the one below, and every quad between two sections split into two
// triangles. With 686 sections of 328 points it has the size and shape of the
// TestLoftSectionsTwistedLongRibbonIsOneVerifiedLoft mesh's wall.
func twistedRibbonBoxes(sections, points, teeth int, step, twist float64) [][2]r3.Vec {
	at := func(s, p int) r3.Vec {
		a := 2*math.Pi*float64(p%points)/float64(points) + twist*float64(s)
		r := 20 + 1.5*math.Sin(float64(teeth)*2*math.Pi*float64(p%points)/float64(points))
		return r3.NewVec(r*math.Cos(a), r*math.Sin(a), step*float64(s))
	}
	verts := make([]r3.Vec, 0, sections*points)
	for s := range sections {
		for p := range points {
			verts = append(verts, at(s, p))
		}
	}
	boxes := make([][2]r3.Vec, 0, 2*(sections-1)*points)
	for s := range sections - 1 {
		for p := range points {
			a, b := s*points+p, s*points+(p+1)%points
			c, d := a+points, b+points
			boxes = append(boxes, meshbool.TriBox(verts, [3]int{a, b, d}), meshbool.TriBox(verts, [3]int{a, d, c}))
		}
	}
	return boxes
}

// BenchmarkPairScanBuild measures newPairScan, and each scan it builds, over
// the twisted ribbon at the audit's own ceiling.
func BenchmarkPairScanBuild(b *testing.B) {
	boxes := twistedRibbonBoxes(686, 328, 68, 0.26, 0.004)
	members := allMembers(len(boxes))
	const limit = 2_048_000_000
	b.Run("pair", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			_ = newPairScan(boxes, members, limit)
		}
	})
	b.Run("grid", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			if _, ok := newGridScan(boxes, members, limit); !ok {
				b.Fatal("the ribbon grid must fit the limit")
			}
		}
	})
	b.Run("tree", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			_ = newTreeScan(boxes, members)
		}
	})
}
