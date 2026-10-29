package loftmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
)

// lobedTubeBoxes is the triangle boxes of a closed lobed tube: sections
// stacked step apart in z, each a closed profile of points vertices on a wavy
// circle with lobes lobes, each section turned twist radians from the one
// below, and every quad between two sections split into two triangles.
func lobedTubeBoxes(sections, points, lobes int, step, twist float64) [][2]r3.Vec {
	at := func(s, p int) r3.Vec {
		a := 2*math.Pi*float64(p%points)/float64(points) + twist*float64(s)
		r := 20 + 1.5*math.Sin(float64(lobes)*2*math.Pi*float64(p%points)/float64(points))
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
// a lobed tube at the twisted loft route's ceilings: 763 sections of 328
// points are 499,872 triangles, just under twistMaxTriangles (500_000), and
// the limit is twistMaxScans (4096 * twistMaxTriangles). Both constants live
// in internal/smoothloft/twisted_mesh.go, which this package cannot import.
func BenchmarkPairScanBuild(b *testing.B) {
	boxes := lobedTubeBoxes(763, 328, 24, 0.26, 0.004)
	members := allMembers(len(boxes))
	const limit = 4096 * 500_000
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
				b.Fatal("the tube grid must fit the limit")
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
