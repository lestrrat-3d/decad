// Package trimtest builds hand-made retained faceted bodies for the trim-graph
// packages' tests: prisms and a pyramid whose every group is a retained plane
// with an exact frame and zero vertex bounds.
package trimtest

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Producer is the producer id every kit body carries.
const Producer provenance.Producer = 9

// Env is the topology environment the kit builds with.
func Env() topo.Env {
	return topo.Env{Doc: &topo.Document{}, Ref: Producer, Alloc: topotest.Alloc{}}
}

// PlaneGroup is a retained plane group through p0, p1, p2, whose outward normal is (p1-p0)x(p2-p0).
func PlaneGroup(t *testing.T, role string, p0, p1, p2 r3.Vec) facetedrecord.Group {
	t.Helper()
	f, err := r3.NewFrame(p0, p1.Sub(p0), p2.Sub(p0))
	require.NoError(t, err)
	return facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: Producer, Role: role}},
		Surface:  surfacegeom.Plane{Frame: f},
		Retained: true,
		Planar:   true,
	}
}

// axisGroup is a retained plane group on the frame (o, u, v), whose outward normal is u x v.
func axisGroup(t *testing.T, role string, o, u, v r3.Vec) facetedrecord.Group {
	t.Helper()
	return PlaneGroup(t, role, o, o.Add(u), o.Add(v))
}

// Build builds the body of fp with the kit's environment.
func Build(t *testing.T, fp facetedrecord.Payload) *topo.Body {
	t.Helper()
	b, err := facetedrecord.Build(t.Context(), Env(), fp)
	require.NoError(t, err)
	return b
}

// BuildPayload builds fp and returns the body with the payload it holds, whose FaceOf is filled.
func BuildPayload(t *testing.T, fp facetedrecord.Payload) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	b := Build(t, fp)
	built, ok := b.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	return b, built
}

// Prism is the extrusion along x, from x0 to x1, of a section given as (y, z) points in counter-clockwise order. The
// side face of section edge i carries the role "side(i)", and the end faces "capEnd" (x1) and "capStart" (x0).
func Prism(t *testing.T, section [][2]float64, x0, x1 float64) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	n := len(section)
	var fp facetedrecord.Payload
	for _, x := range []float64{x0, x1} {
		for _, p := range section {
			fp.Verts = append(fp.Verts, r3.NewVec(x, p[0], p[1]))
			fp.VertexBound = append(fp.VertexBound, 0)
		}
	}
	vec := func(i int) r3.Vec { return fp.Verts[i] }
	addTri := func(g, a, b, c int) {
		fp.Tris = append(fp.Tris, [3]int{a, b, c})
		fp.Src = append(fp.Src, g)
	}
	for i := range n {
		j := (i + 1) % n
		a, b, c, d := i, j, j+n, i+n
		fp.Groups = append(fp.Groups, PlaneGroup(t, "side("+string(rune('0'+i))+")", vec(a), vec(b), vec(d)))
		addTri(i, a, b, c)
		addTri(i, a, c, d)
	}
	end, start := len(fp.Groups), len(fp.Groups)+1
	fp.Groups = append(fp.Groups, axisGroup(t, "capEnd", r3.NewVec(x1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
	fp.Groups = append(fp.Groups, axisGroup(t, "capStart", r3.NewVec(x0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(0, 1, 0)))
	for k := 1; k < n-1; k++ {
		addTri(end, n, n+k, n+k+1)
		addTri(start, 0, k+1, k)
	}
	fp.MeshBound, fp.DPair = 0, 1
	fp.DPair = r3.NewVec(x1-x0, 40, 40).Len()
	return BuildPayload(t, fp)
}

// BevelledPlate is the 45 degree bevelled plate: a 40 x 20 x 20 plate whose y=0, z=20 edge is cut by the plane
// z - y = 18. Its volume is 40 * (400 - 2) = 15920.
func BevelledPlate(t *testing.T) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	return Prism(t, [][2]float64{{0, 0}, {20, 0}, {20, 20}, {2, 20}, {0, 18}}, 0, 40)
}

// LSection is a concave L prism: a 20 x 20 square with the (10..20, 10..20) quarter removed, 40 long.
func LSection(t *testing.T) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	return Prism(t, [][2]float64{{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}}, 0, 40)
}

// Pyramid is a square pyramid with base 10 x 10 and apex (5, 5, 8): its apex has valence 4.
func Pyramid(t *testing.T) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	var fp facetedrecord.Payload
	fp.Verts = []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(10, 10, 0), r3.NewVec(0, 10, 0), r3.NewVec(5, 5, 8)}
	fp.VertexBound = make([]float64, 5)
	fp.Groups = []facetedrecord.Group{
		PlaneGroup(t, "base", fp.Verts[0], fp.Verts[3], fp.Verts[1]),
	}
	fp.Tris = [][3]int{{0, 3, 2}, {0, 2, 1}}
	fp.Src = []int{0, 0}
	for i := range 4 {
		j := (i + 1) % 4
		fp.Groups = append(fp.Groups, PlaneGroup(t, "wall", fp.Verts[i], fp.Verts[j], fp.Verts[4]))
		fp.Tris = append(fp.Tris, [3]int{i, j, 4})
		fp.Src = append(fp.Src, i+1)
	}
	fp.DPair = 20
	return BuildPayload(t, fp)
}
