package sweepmitre_test

import (
	"math/big"
	"math/rand"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/stretchr/testify/require"
)

func TestOrientSignMatchesRationalDeterminant(t *testing.T) {
	rng := rand.New(rand.NewSource(23))
	point := func() sweeparc.RatVec {
		var p sweeparc.RatVec
		for axis := range p {
			den := rng.Int63n(1000) + 1
			if rng.Intn(4) == 0 {
				den = 1 << uint(rng.Intn(53))
			}
			p[axis] = big.NewRat(rng.Int63n(20001)-10000, den)
		}
		return p
	}
	for trial := range 300 {
		a, b, c, d := point(), point(), point(), point()
		if trial%7 == 0 {
			d = c
		}
		want := sweeparc.Dot(sweeparc.Cross(sweeparc.Sub(b, a), sweeparc.Sub(c, a)), sweeparc.Sub(d, a)).Sign()
		require.Equal(t, want, sweepmitre.OrientSign(a, b, c, d), "trial %d", trial)
	}
}

func TestTriangleAreasMatchRationalConstruction(t *testing.T) {
	rng := rand.New(rand.NewSource(17))
	exact := make([]sweeparc.RatVec, 0, 300)
	tris := make([][3]int, 0, 100)
	for i := range 100 {
		tris = append(tris, [3]int{3 * i, 3*i + 1, 3*i + 2})
		for vertex := range 3 {
			var p sweeparc.RatVec
			for axis := range 3 {
				num := rng.Int63n(200001) - 100000
				den := rng.Int63n(100000) + 1
				if i%4 == 0 {
					den = 1 << uint(rng.Intn(55))
				}
				p[axis] = big.NewRat(num, den)
			}
			if i%10 == 0 && vertex == 2 {
				p = exact[len(exact)-1]
			}
			exact = append(exact, p)
		}
	}

	got, err := sweepmitre.TriangleAreas(t.Context(), exact, tris)
	require.NoError(t, err)
	require.Len(t, got, len(tris))
	quarter := big.NewRat(1, 4)
	for i, tri := range tris {
		n := sweeparc.Cross(sweeparc.Sub(exact[tri[1]], exact[tri[0]]), sweeparc.Sub(exact[tri[2]], exact[tri[0]]))
		q := sweeparc.Dot(n, n)
		q.Mul(q, quarter)
		want := [2]*big.Rat{
			proofarith.FloatRat(proofbound.RatSqrtDown(q)),
			proofarith.FloatRat(proofbound.RatSqrtUp(q)),
		}
		for end := range 2 {
			require.Equal(t, want[end].RatString(), got[i][end].RatString(), "triangle %d end %d", i, end)
		}
	}
}
