package facetproof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func rationalFacetDeterminant(verts []r3.Vec, tri [3]int) *big.Rat {
	var coordinates [3][3]*big.Rat
	for i, vertex := range tri {
		v := verts[vertex]
		coordinates[i] = [3]*big.Rat{
			new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z),
		}
	}
	sum := new(big.Rat)
	for axis := range 3 {
		j, k := (axis+1)%3, (axis+2)%3
		cross := new(big.Rat).Sub(
			new(big.Rat).Mul(coordinates[1][j], coordinates[2][k]),
			new(big.Rat).Mul(coordinates[1][k], coordinates[2][j]))
		sum.Add(sum, new(big.Rat).Mul(coordinates[0][axis], cross))
	}
	return sum
}

func TestAuditFacetDeterminantsMatchRationalFormula(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(193, 197))
	exponents := []int{-400, -160, -1, 0, 63, 160, 400}
	for sample := range 35 {
		verts := make([]r3.Vec, 4)
		for i := range verts {
			verts[i] = r3.Vec{
				X: math.Ldexp(rng.Float64()-0.5, exponents[sample%len(exponents)]),
				Y: math.Ldexp(rng.Float64()-0.5, exponents[(sample+2)%len(exponents)]),
				Z: math.Ldexp(rng.Float64()-0.5, exponents[(sample+4)%len(exponents)]),
			}
		}
		tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
		var determinants [4]*big.Rat
		total := new(big.Rat)
		for i, tri := range tris {
			determinants[i] = rationalFacetDeterminant(verts, tri)
			total.Add(total, determinants[i])
		}
		require.NotZero(t, total.Sign(), "sample %d: tetrahedron has no volume", sample)
		if total.Sign() < 0 {
			total.Neg(total)
			for i, tri := range tris {
				tris[i] = [3]int{tri[0], tri[2], tri[1]}
				determinants[i].Neg(determinants[i])
			}
		}
		var moments [3]*big.Rat
		for axis := range 3 {
			moments[axis] = new(big.Rat)
		}
		for i, tri := range tris {
			var sums [3]big.Rat
			for _, vertex := range tri {
				v := verts[vertex]
				for axis, coordinate := range [3]float64{v.X, v.Y, v.Z} {
					sums[axis].Add(&sums[axis], new(big.Rat).SetFloat64(coordinate))
				}
			}
			for axis := range 3 {
				moments[axis].Add(moments[axis], new(big.Rat).Mul(determinants[i], &sums[axis]))
			}
		}
		for _, workers := range []int{1, 4} {
			audit, err := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, tris, workers)
			require.NoError(t, err, "sample %d, workers %d", sample, workers)
			require.Len(t, audit.Det, len(tris))
			for i, want := range determinants {
				require.Zero(t, want.Cmp(audit.Det[i].Rat()), "sample %d, facet %d", sample, i)
			}
			require.Zero(t, new(big.Rat).Quo(total, big.NewRat(6, 1)).Cmp(audit.CompVol[0]),
				"sample %d: component volume differs", sample)
			got, err := facetproof.FirstMomentsFromAudit(proofbound.NewWorkBudget(t.Context()), audit, tris)
			require.NoError(t, err)
			for axis := range 3 {
				require.Zero(t, moments[axis].Cmp(got[axis]), "sample %d, moment axis %d", sample, axis)
			}
		}
	}
}

// TestAuditFacetDeterminantsCoverEveryFacet runs the multi-component meshes
// back to back, every parallel audit before the serial ones, so a determinant
// left over from an earlier audit or a facet skipped deep in a long component
// shows up as a wrong entry.
func TestAuditFacetDeterminantsCoverEveryFacet(t *testing.T) {
	t.Parallel()
	cases := auditCases(t)
	for _, workers := range []int{4, 1} {
		for _, c := range cases {
			audit, err := facetproof.AuditFacetedMeshWorkers(t.Context(), c.verts, c.tris, workers)
			require.NoError(t, err, "%s, workers %d", c.name, workers)
			require.Len(t, audit.Det, len(c.tris), "%s, workers %d", c.name, workers)
			for fi, tri := range c.tris {
				require.Zero(t, rationalFacetDeterminant(c.verts, tri).Cmp(audit.Det[fi].Rat()),
					"%s, workers %d, facet %d", c.name, workers, fi)
			}
			for ci, members := range audit.Members {
				total := new(big.Rat)
				for _, fi := range members {
					total.Add(total, audit.Det[fi].Rat())
				}
				require.Zero(t, total.Quo(total, big.NewRat(6, 1)).Cmp(audit.CompVol[ci]),
					"%s, workers %d, component %d", c.name, workers, ci)
			}
		}
	}
}
