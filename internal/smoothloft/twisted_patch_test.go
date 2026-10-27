package smoothloft

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func twistTestPoint(x, y int64) freeform.RatPoint {
	return freeform.RatPoint{U: rint(x), V: rint(y)}
}

// twistTestLinePatch is the patch of a line from a to b that is fixed in
// height: both height spans see the same segment.
func twistTestLinePatch(a, b freeform.RatPoint) twistPatch {
	span := twistLineSpan(a, b)
	return twistPatchOf(span, span, twistTestZeroSlopes(), twistTestZeroSlopes(), rint(1))
}

func twistTestZeroSlopes() twistSpan {
	z := twistTestPoint(0, 0)
	return twistSpan{z, z, z, z}
}

func TestTwistedPatchPolarProof(t *testing.T) {
	t.Parallel()
	zero := twistTestZeroSlopes()
	prove := func(a, b freeform.RatPoint) error {
		span := twistLineSpan(a, b)
		patch := twistPatchOf(span, span, zero, zero, rint(1))
		return patch.provePolarPositive(t.Context(), proofbound.NewWorkBudget(t.Context()))
	}
	// A counterclockwise edge seen from the origin is admitted.
	require.NoError(t, prove(twistTestPoint(4, -2), twistTestPoint(4, 2)))
	// The same edge walked clockwise, and an edge through the origin, refuse.
	require.ErrorIs(t, prove(twistTestPoint(4, 2), twistTestPoint(4, -2)), decaderr.ErrUnsupported)
	require.ErrorIs(t, prove(twistTestPoint(-4, -2), twistTestPoint(4, 2)), decaderr.ErrUnsupported)
}

func TestTwistedPatchEvaluatesRecordedSections(t *testing.T) {
	t.Parallel()
	a, b := twistTestPoint(4, -2), twistTestPoint(4, 2)
	c, d := twistTestPoint(0, 5), twistTestPoint(-1, 7)
	first, second := twistLineSpan(a, b), twistLineSpan(c, d)
	// With zero slopes the blend still meets both sections at its ends.
	zero := twistTestZeroSlopes()
	patch := twistPatchOf(first, second, zero, zero, rint(1))
	for _, tc := range []struct {
		u, t int64
		want freeform.RatPoint
	}{{0, 0, a}, {1, 0, b}, {0, 1, c}, {1, 1, d}} {
		got := patch.At(rint(tc.u), rint(tc.t))
		require.Zero(t, got.U.Cmp(tc.want.U))
		require.Zero(t, got.V.Cmp(tc.want.V))
	}
}

func TestTwistedSlopesUseExactSpacing(t *testing.T) {
	t.Parallel()
	points := []freeform.RatPoint{twistTestPoint(0, 0), twistTestPoint(1, 0), twistTestPoint(4, 0)}
	zs := []*big.Rat{rint(0), rint(1), rint(3)}
	slopes := twistSlopes(points, zs)
	require.Zero(t, slopes[0].U.Cmp(rint(1)))
	require.Zero(t, slopes[1].U.Cmp(big.NewRat(4, 3)))
	require.Zero(t, slopes[2].U.Cmp(big.NewRat(3, 2)))
}

func TestTwistedCountForMeetsLimit(t *testing.T) {
	t.Parallel()
	limit := big.NewRat(1, 300)
	for _, x := range []*big.Rat{rint(0), big.NewRat(1, 2), rint(1), big.NewRat(37, 10)} {
		n := twistCountFor(x, limit)
		require.GreaterOrEqual(t, n, 1)
		require.LessOrEqual(t, div(x, mul(rint(8), rint(int64(n*n)))).Cmp(limit), 0)
		if n > 1 {
			require.Greater(t, div(x, mul(rint(8), rint(int64((n-1)*(n-1))))).Cmp(limit), 0)
		}
	}
}

func TestTwistedBoundsOfLinePatch(t *testing.T) {
	t.Parallel()
	b := twistTestLinePatch(twistTestPoint(4, -2), twistTestPoint(4, 2)).bounds()
	require.Zero(t, b.muu.Sign())
	require.Zero(t, b.mtt.Sign())
	require.Zero(t, b.mut.Sign())
	require.Zero(t, b.bu.Cmp(rint(4)))
}

func TestTwistedPolarCoefficientsMatchDerivativeProduct(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(17, 29))
	coordinate := func(sample int) *big.Rat {
		num := big.NewInt(int64(rng.IntN(2001) - 1000))
		num.Lsh(num, uint((sample%4)*40))
		den := big.NewInt(int64(2*rng.IntN(7) + 1))
		den.Lsh(den, uint(rng.IntN(20)))
		return new(big.Rat).SetFrac(num, den)
	}
	for sample := range 48 {
		var patch twistPatch
		for i := range patch {
			for j := range patch[i] {
				patch[i][j] = freeform.RatPoint{U: coordinate(sample), V: coordinate(sample)}
			}
		}
		if sample == 0 {
			patch = twistTestLinePatch(twistTestPoint(0, 0), twistTestPoint(0, 0))
		}
		if sample == 1 {
			patch = twistTestLinePatch(twistTestPoint(4, -2), twistTestPoint(4, 2))
		}
		var before [4][4][2]string
		for i := range patch {
			for j, point := range patch[i] {
				before[i][j] = [2]string{point.U.RatString(), point.V.RatString()}
			}
		}
		want := twistPolarDerivativeProduct(patch)
		got := patch.polarCoefficients()
		for i := range got {
			for j := range got[i] {
				require.Zero(t, got[i][j].Cmp(want[i][j]), "case %d coefficient (%d,%d)", sample, i, j)
			}
		}
		for i := range patch {
			for j, point := range patch[i] {
				require.Equal(t, before[i][j][0], point.U.RatString(), "case %d input U (%d,%d)", sample, i, j)
				require.Equal(t, before[i][j][1], point.V.RatString(), "case %d input V (%d,%d)", sample, i, j)
			}
		}
	}
}

// twistPolarDerivativeProduct multiplies the patch by its literal cubic
// derivative in Bernstein form, without combining antisymmetric cross terms.
func twistPolarDerivativeProduct(p twistPatch) [6][7]*big.Rat {
	var coeff [6][7]*big.Rat
	for k := range coeff {
		for l := range coeff[k] {
			coeff[k][l] = new(big.Rat)
		}
	}
	var d [3][4]freeform.RatPoint
	for m := range d {
		for n := range d[m] {
			d[m][n] = twistScale(twistSub(p[m+1][n], p[m][n]), rint(3))
		}
	}
	for i := range p {
		for m := range d {
			wu := big.NewRat(twistBinomial[3][i]*twistBinomial[2][m], twistBinomial[5][i+m])
			for j := range p[i] {
				for n := range d[m] {
					wt := big.NewRat(twistBinomial[3][j]*twistBinomial[3][n], twistBinomial[6][j+n])
					cross := sub(mul(p[i][j].U, d[m][n].V), mul(p[i][j].V, d[m][n].U))
					coeff[i+m][j+n].Add(coeff[i+m][j+n], mul(cross, mul(wu, wt)))
				}
			}
		}
	}
	return coeff
}
