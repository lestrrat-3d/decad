package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var splitPLatticeBenchmarkSink *decad.MeshBody

// BenchmarkSplitPLatticeOneCellThinWall measures the public Split P lattice
// constructor for one 10 mm cell and a level 15% of the field's 1.8 maximum.
func BenchmarkSplitPLatticeOneCellThinWall(b *testing.B) {
	spec := decad.LatticeSpec{
		Min: r3.Vec{}, Max: r3.Vec{X: 10, Y: 10, Z: 10},
		Cell: units.Millimeters(10), Level: units.Scalar(0.27), Resolution: 48,
	}
	ctx := b.Context()
	b.ReportAllocs()
	for range b.N {
		mesh, err := decad.SplitPLattice(ctx, spec)
		if err != nil {
			b.Fatal(err)
		}
		splitPLatticeBenchmarkSink = mesh
	}
}
