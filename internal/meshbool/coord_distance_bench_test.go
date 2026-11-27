package meshbool_test

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func coordDistanceBenchInputs(exact bool) ([]proof.Xpt, []r3.Vec) {
	r := rand.New(rand.NewPCG(9, 9))
	pts := make([]proof.Xpt, 1024)
	vs := make([]r3.Vec, 1024)
	for i := range pts {
		v := r3.Vec{X: r.Float64()*100 - 50, Y: r.Float64()*100 - 50, Z: r.Float64()*100 - 50}
		vs[i] = v
		pts[i] = proof.XptOf(v)
		if !exact {
			w := v
			w.X += 1e-12
			pts[i] = proof.XptOf(w)
		}
	}
	return pts, vs
}

func BenchmarkCoordDistance(b *testing.B) {
	for _, c := range []struct {
		name  string
		exact bool
	}{{"exact", true}, {"off", false}} {
		pts, vs := coordDistanceBenchInputs(c.exact)
		b.Run(c.name+"/reference", func(b *testing.B) {
			b.ReportAllocs()
			for i := range b.N {
				referenceCoordDistance(pts[i%1024], vs[i%1024])
			}
		})
		b.Run(c.name+"/current", func(b *testing.B) {
			b.ReportAllocs()
			for i := range b.N {
				meshbool.CoordDistance(pts[i%1024], vs[i%1024])
			}
		})
	}
}
