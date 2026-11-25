package meshbool_test

import (
	"math"
	"math/rand/v2"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// parityStarPrism is a closed star prism along z with caps at z = 0 and
// z = height. Its cap facets project to segments on a line of constant v when
// a ray sweeps x, and to segments on a line of constant u when it sweeps y.
func parityStarPrism(n int, height float64, rng *rand.Rand) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	for _, z := range []float64{0, height} {
		verts = append(verts, r3.NewVec(0, 0, z))
		for j := range n {
			a := 2 * math.Pi * float64(j) / float64(n)
			r := 2.0
			if j%2 == 1 {
				r = 3 + 0.1*rng.Float64()
			}
			verts = append(verts, r3.NewVec(r*math.Cos(a), r*math.Sin(a), z))
		}
	}
	m := n + 1
	var tris [][3]int
	for j := range n {
		a, b := 1+j, 1+(j+1)%n
		tris = append(tris, [3]int{0, b, a}, [3]int{m, m + a, m + b})
		tris = append(tris, [3]int{a, b, m + a}, [3]int{b, m + b, m + a})
	}
	return verts, tris
}

// parityPrismPoints returns query points for a prism of the given height:
// random, on a cap plane, sharing a vertex's y and z with a random x, sharing
// a vertex's x and z with a random y, and exactly on vertices.
func parityPrismPoints(verts []r3.Vec, height float64, count int, rng *rand.Rand) []proof.Xpt {
	var points []proof.Xpt
	for k := range count {
		var p r3.Vec
		switch k % 5 {
		case 0:
			p = r3.NewVec(rng.Float64()*8-4, rng.Float64()*8-4, rng.Float64()*3-0.75)
		case 1:
			p = r3.NewVec(rng.Float64()*8-4, rng.Float64()*8-4, []float64{0, height}[rng.IntN(2)])
		case 2:
			v := verts[rng.IntN(len(verts))]
			p = r3.NewVec(rng.Float64()*8-4, v.Y, v.Z)
		case 3:
			v := verts[rng.IntN(len(verts))]
			p = r3.NewVec(v.X, rng.Float64()*8-4, v.Z)
		default:
			p = verts[rng.IntN(len(verts))]
		}
		points = append(points, proof.XptOf(p))
	}
	return points
}

func errorText(err error) string {
	if err == nil {
		return ""
	}
	return err.Error()
}
