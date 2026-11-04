package proof

import (
	"testing"

	"github.com/lestrrat-3d/r3"
)

var orientBenchSink int

// BenchmarkOrientSignExpansion compares the filtered expansion with the exact
// determinant on near-coplanar and exactly coplanar quadruples.
func BenchmarkOrientSignExpansion(b *testing.B) {
	var nonzero, coplanar [][4]r3.Vec
	for _, q := range nearCoplanarQuads(20000) {
		s := OrientSignExact(XptOf(q[0]), XptOf(q[1]), XptOf(q[2]), XptOf(q[3]))
		switch s {
		case 0:
			coplanar = append(coplanar, q)
		default:
			nonzero = append(nonzero, q)
		}
	}
	for _, set := range []struct {
		name  string
		quads [][4]r3.Vec
	}{{"nonzero", nonzero}, {"coplanar", coplanar}} {
		for _, impl := range []struct {
			name string
			fn   func(a, b, c, d r3.Vec) int
		}{
			{"exact-determinant", func(a, b, c, d r3.Vec) int {
				return OrientSignExact(XptOf(a), XptOf(b), XptOf(c), XptOf(d))
			}},
			{"expansion", func(a, b, c, d r3.Vec) int {
				s, _ := OrientSignExpansion(a, b, c, d)
				return s
			}},
		} {
			b.Run(set.name+"/"+impl.name, func(b *testing.B) {
				b.ReportAllocs()
				for i := range b.N {
					q := &set.quads[i%len(set.quads)]
					orientBenchSink += impl.fn(q[0], q[1], q[2], q[3])
				}
			})
		}
	}
}
