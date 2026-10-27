package facetedtopology

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// topologyCase is one input to build: a mesh, its per-facet face and each
// face's planarity.
type topologyCase struct {
	name      string
	verts     []r3.Vec
	tris      [][3]int
	facetFace []int
	planar    map[int]bool
}

// cylinderCase is a closed capped cylinder of n sectors and m rings. Its caps
// are planar fans around a centre of degree n; the side is cut into a grid of
// bands × sectors curved faces, so some boundary vertices join more than two
// boundary edges.
func cylinderCase(n, m, bands, sectors int) topologyCase {
	c := topologyCase{name: fmt.Sprintf("cylinder-%dx%d", n, m), planar: map[int]bool{0: true, 1: true}}
	at := func(r, s int) int { return r*n + s%n }
	for r := 0; r <= m; r++ {
		for s := range n {
			angle := 2 * math.Pi * float64(s) / float64(n)
			c.verts = append(c.verts, r3.NewVec(math.Cos(angle), math.Sin(angle), float64(r)/float64(m)))
		}
	}
	bottom, top := len(c.verts), len(c.verts)+1
	c.verts = append(c.verts, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))
	for s := range n {
		c.tris = append(c.tris, [3]int{bottom, at(0, s+1), at(0, s)})
		c.facetFace = append(c.facetFace, 0)
		c.tris = append(c.tris, [3]int{top, at(m, s), at(m, s+1)})
		c.facetFace = append(c.facetFace, 1)
	}
	for r := range m {
		for s := range n {
			face := 2 + (r*bands/m)*sectors + s*sectors/n
			a, b, cc, d := at(r, s), at(r, s+1), at(r+1, s+1), at(r+1, s)
			c.tris = append(c.tris, [3]int{a, b, cc}, [3]int{a, cc, d})
			c.facetFace = append(c.facetFace, face, face)
		}
	}
	return c
}

// sphereCase is a closed UV sphere whose poles are fans of degree n. With
// faces > 1 its latitude bands are separate curved faces.
func sphereCase(n, m, faces int) topologyCase {
	c := topologyCase{name: fmt.Sprintf("sphere-%dx%d-%d", n, m, faces), planar: map[int]bool{}}
	c.verts = append(c.verts, r3.NewVec(0, 0, -1))
	for r := 1; r < m; r++ {
		phi := math.Pi * float64(r) / float64(m)
		for s := range n {
			theta := 2 * math.Pi * float64(s) / float64(n)
			c.verts = append(c.verts, r3.NewVec(math.Sin(phi)*math.Cos(theta), math.Sin(phi)*math.Sin(theta), -math.Cos(phi)))
		}
	}
	north := len(c.verts)
	c.verts = append(c.verts, r3.NewVec(0, 0, 1))
	at := func(r, s int) int { return 1 + (r-1)*n + s%n }
	face := func(r int) int { return r * faces / m }
	for s := range n {
		c.tris = append(c.tris, [3]int{0, at(1, s+1), at(1, s)})
		c.facetFace = append(c.facetFace, face(0))
		c.tris = append(c.tris, [3]int{north, at(m-1, s), at(m-1, s+1)})
		c.facetFace = append(c.facetFace, face(m-1))
	}
	for r := 1; r+1 < m; r++ {
		for s := range n {
			a, b, cc, d := at(r, s), at(r, s+1), at(r+1, s+1), at(r+1, s)
			c.tris = append(c.tris, [3]int{a, b, cc}, [3]int{a, cc, d})
			c.facetFace = append(c.facetFace, face(r), face(r))
		}
	}
	return c
}

func BenchmarkBuild(b *testing.B) {
	for _, c := range []topologyCase{
		cylinderCase(16, 4, 2, 2),
		cylinderCase(128, 80, 4, 4),
		sphereCase(96, 64, 4),
	} {
		bounds := make([]float64, len(c.verts))
		b.Run(fmt.Sprintf("%s/%d", c.name, len(c.tris)), func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				result, err := Build(b.Context(), c.verts, c.tris, c.facetFace, c.planar, bounds)
				require.NoError(b, err)
				require.NotEmpty(b, result.Edges)
			}
		})
	}
}
