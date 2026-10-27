package proof_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

var orientBenchSink int

// BenchmarkOrientSignExpansion compares the expansion with the previous
// implementation on admitted near-coplanar quadruples with a nonzero sign,
// the inputs the loft crossing audit mostly hands it, and on exactly
// coplanar ones, which always build the full expansion.
func BenchmarkOrientSignExpansion(b *testing.B) {
	var nonzero, coplanar [][4]r3.Vec
	for _, q := range nearCoplanarQuads(20000) {
		s, ok := proof.LegacyOrientSignExpansion(q[0], q[1], q[2], q[3])
		switch {
		case !ok:
		case s == 0:
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
			fn   func(a, b, c, d r3.Vec) (int, bool)
		}{
			{"legacy", proof.LegacyOrientSignExpansion},
			{"current", proof.OrientSignExpansion},
		} {
			b.Run(set.name+"/"+impl.name, func(b *testing.B) {
				b.ReportAllocs()
				for i := range b.N {
					q := &set.quads[i%len(set.quads)]
					s, _ := impl.fn(q[0], q[1], q[2], q[3])
					orientBenchSink += s
				}
			})
		}
	}
}
