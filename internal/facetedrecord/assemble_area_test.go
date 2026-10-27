package facetedrecord

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// torusMesh is a closed outward torus of around × rings quads, two facets
// each, with one source group per quarter turn: the ring-of-rings vertex
// layout a revolved faceted body holds. Vertex bounds vary around the tube.
func torusMesh(around, rings int) Mesh {
	var m Mesh
	for i := range around {
		u := 2 * math.Pi * float64(i) / float64(around)
		for j := range rings {
			v := 2 * math.Pi * float64(j) / float64(rings)
			r := 10 + 3*math.Cos(v)
			m.Verts = append(m.Verts, r3.NewVec(r*math.Cos(u), r*math.Sin(u), 3*math.Sin(v)))
			m.VertexBound = append(m.VertexBound, 1e-9*float64(1+j%5))
		}
	}
	for range 4 {
		m.Groups = append(m.Groups, Group{Origins: []provenance.Ref{{Producer: 3, Role: provenance.RoleBody}}})
	}
	at := func(i, j int) int { return (i%around)*rings + j%rings }
	for i := range around {
		for j := range rings {
			a, b, c, d := at(i, j), at(i+1, j), at(i+1, j+1), at(i, j+1)
			m.Tris = append(m.Tris, [3]int{a, b, c}, [3]int{a, c, d})
			group := 4 * i / around
			m.Src = append(m.Src, group, group)
		}
	}
	return m
}

// TestAssembleBodyAreaMatchesPerFacetSums holds every face's and the body's
// area reading to the per-facet loops AssembleBody once ran twice: each
// facet's area term and evaluation charge, summed in facet order per face and
// again over the body. Values and bounds must match bit for bit.
func TestAssembleBodyAreaMatchesPerFacetSums(t *testing.T) {
	t.Parallel()
	var cubes Mesh
	addCube(&cubes, 0, 4, false, 3)
	addCube(&cubes, 1, 2, true, 4)
	cubes.AreaSlack = 1e-7
	thin := torusMesh(7, 5)
	thin.AreaSlack = 3e-12
	fixtures := map[string]Mesh{"torus": torusMesh(24, 12), "thin-torus": thin, "cube-with-void": cubes}
	for name, m := range fixtures {
		body, faceOf, _, err := AssembleBody(t.Context(),
			topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}, m, nil, measurement.Measurement{}, nil)
		require.NoError(t, err, name)
		faces := topobuild.Faces(body)
		area := make([]float64, len(faces))
		slop := make([]float64, len(faces))
		facets := make([][]int, len(faces))
		areaF, termSlop := 0.0, 0.0
		for i, tri := range m.Tris {
			a, b, c := m.Verts[tri[0]], m.Verts[tri[1]], m.Verts[tri[2]]
			k := faceOf[i]
			area[k] += b.Sub(a).Cross(c.Sub(a)).Len() / 2
			slop[k] = proofbound.AbsSumUpper(slop[k], proofbound.FacetAreaTermSlop(a, b, c))
			facets[k] = append(facets[k], i)
		}
		for _, tri := range m.Tris {
			a, b, c := m.Verts[tri[0]], m.Verts[tri[1]], m.Verts[tri[2]]
			areaF += b.Sub(a).Cross(c.Sub(a)).Len() / 2
			termSlop = proofbound.AbsSumUpper(termSlop, proofbound.FacetAreaTermSlop(a, b, c))
		}
		// These fixtures list faces in component order, the order the body
		// sums its faces' geometric terms in.
		bodyGeom := 0.0
		bad := 0
		for k, f := range faces {
			geom, err := FaceAreaGeom(proofbound.NewWorkBudget(t.Context()), f, m.Verts, m.VertexBound, m.Tris, facets[k])
			require.NoError(t, err, name)
			bodyGeom = proofbound.AbsSumUpper(bodyGeom, geom)
			want := proofbound.AbsSumUpper(geom, m.AreaSlack, proofbound.SumSlop(len(facets[k]), area[k]), slop[k])
			if math.Float64bits(f.Area) != math.Float64bits(area[k]) ||
				math.Float64bits(f.AreaBound) != math.Float64bits(want) {
				bad++
			}
		}
		require.Zero(t, bad, "%s: faces whose area or area bound differs", name)
		require.Equal(t, math.Float64bits(areaF), math.Float64bits(body.Area.Value.Base()), name)
		wantBound := proofbound.AbsSumUpper(bodyGeom, m.AreaSlack, proofbound.SumSlop(len(m.Tris), areaF), termSlop)
		require.Equal(t, math.Float64bits(wantBound), math.Float64bits(body.Area.Bound.Base()), name)
	}
}

func BenchmarkAssembleBodyTorus(b *testing.B) {
	m := torusMesh(96, 48)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	b.ReportAllocs()
	for b.Loop() {
		if _, _, _, err := AssembleBody(b.Context(), env, m, nil, measurement.Measurement{}, nil); err != nil {
			b.Fatal(err)
		}
	}
}
