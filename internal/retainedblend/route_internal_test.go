package retainedblend

import (
	"fmt"
	"maps"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// exactSum is the exact rational sum of float terms.
func exactSum(terms ...float64) *big.Rat {
	total := new(big.Rat)
	for _, t := range terms {
		total.Add(total, new(big.Rat).SetFloat64(t))
	}
	return total
}

// TestSumDown holds the lower sum of a group's area terms to the exact sum.
// One plus three quarters of an ulp rounds up to the next float, which a
// rounded sum would publish as a lower bound past the exact one.
func TestSumDown(t *testing.T) {
	t.Parallel()
	for name, terms := range map[string][]float64{
		"rounds up":   {1, 3 * 0x1p-54},
		"rounds down": {1, 0x1p-54},
		"no terms":    nil,
		"zero terms":  {0, 0},
	} {
		got := sumDown(terms)
		require.GreaterOrEqual(t, got, 0.0, name)
		require.LessOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(exactSum(terms...)), 0, name)
	}
}

// TestGeomShortfall holds the charged shortfall to the exact sum, over the old
// body's groups, of how far each group's old terms exceed its new ones. A
// group the new body lacks charges all of its old terms, and a group whose
// terms grew charges nothing.
func TestGeomShortfall(t *testing.T) {
	t.Parallel()
	// Group 0's difference is 1 + 2^-54, which rounds down to 1.
	oldGeom := map[int]float64{0: 1 + 0x1p-52, 1: 0.25, 2: 0.5, 3: 0}
	newGeom := map[int]float64{0: 3 * 0x1p-54, 1: 0.75, 4: 2}
	want := new(big.Rat).Sub(new(big.Rat).SetFloat64(oldGeom[0]), new(big.Rat).SetFloat64(newGeom[0]))
	want.Add(want, new(big.Rat).SetFloat64(oldGeom[2]))
	got := geomShortfall(oldGeom, newGeom)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(want), 0)
	// The charge stays within a few ulps of the exact shortfall.
	slack := new(big.Rat).Mul(want, big.NewRat(1, 1<<48))
	require.LessOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(new(big.Rat).Add(want, slack)), 0)

	require.Zero(t, geomShortfall(map[int]float64{0: 1}, map[int]float64{0: 1, 1: 1}))
}

// boundedCube is the cube [0, 10]³ as a retained faceted payload, one planar
// group per side, every vertex bound 1/1024 so each face charges a geometric
// area term.
func boundedCube(t *testing.T) facetedrecord.Payload {
	t.Helper()
	var fp facetedrecord.Payload
	for i := range 8 {
		fp.Verts = append(fp.Verts, r3.NewVec(10*float64(i&1), 10*float64(i>>1&1), 10*float64(i>>2&1)))
		fp.VertexBound = append(fp.VertexBound, 0x1p-10)
	}
	sides := [][3]int{{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
		{2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}}
	for i, tri := range sides {
		if i%2 == 0 {
			a, b, c := fp.Verts[tri[0]], fp.Verts[tri[1]], fp.Verts[tri[2]]
			f, err := r3.NewFrame(a, b.Sub(a), c.Sub(a))
			require.NoError(t, err)
			fp.Groups = append(fp.Groups, facetedrecord.Group{
				Origins: []provenance.Ref{{Producer: 9, Role: fmt.Sprintf("side%d", i/2)}},
				Surface: surfacegeom.Plane{Frame: f}, Retained: true, Planar: true,
			})
		}
		fp.Tris = append(fp.Tris, tri)
		fp.Src = append(fp.Src, i/2)
	}
	fp.MeshBound = 0x1p-10
	fp.AreaSlack = 0x1p-20
	fp.DPair = r3.NewVec(20, 20, 20).Len()
	return fp
}

// TestBuildCharged builds a payload against old terms by group: a group whose
// old terms exceed the new ones by an exact half charges at least that half to
// the slack and to the area bound, and old terms the new body reaches charge
// nothing.
func TestBuildCharged(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	fp := boundedCube(t)
	plain, err := buildCharged(t.Context(), env, fp, nil)
	require.NoError(t, err)
	built, ok := plain.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	require.Equal(t, fp.AreaSlack, built.AreaSlack)
	newGeom, err := groupGeom(t.Context(), faceIndexOf(plain), built, false)
	require.NoError(t, err)
	require.Len(t, newGeom, 6)
	require.Positive(t, newGeom[2], "a side charges a geometric term")

	oldGeom := maps.Clone(newGeom)
	oldGeom[2] += 0.5
	charged, err := buildCharged(t.Context(), env, fp, oldGeom)
	require.NoError(t, err)
	got, ok := charged.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	shortfall := new(big.Rat).Sub(rat(oldGeom[2]), rat(newGeom[2]))
	require.GreaterOrEqual(t, rat(got.AreaSlack).Cmp(new(big.Rat).Add(rat(fp.AreaSlack), shortfall)), 0)
	require.GreaterOrEqual(t, rat(charged.Area.Bound.Base()).Cmp(new(big.Rat).Add(rat(plain.Area.Bound.Base()), shortfall)), 0)

	reached, err := buildCharged(t.Context(), env, fp, newGeom)
	require.NoError(t, err)
	require.Equal(t, plain.Area.Bound.Base(), reached.Area.Bound.Base())
}
