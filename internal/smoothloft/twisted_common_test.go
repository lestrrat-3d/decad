package smoothloft

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// The tests in this file compare the integer evaluation over a patch's common
// denominator (twistCommon) with the big.Rat evaluation it replaced, kept
// below as twistRat* copies, on exact values.

// twistTestSamples is n, or n/4 under the race detector.
func twistTestSamples(n int) int {
	if raceDetector {
		return n / 4
	}
	return n
}

// twistRatPolarCoefficients is polarCoefficients over big.Rat arithmetic.
func twistRatPolarCoefficients(p twistPatch) [6][7]*big.Rat {
	pairs := [6][2]int{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}
	var products [6][7]*big.Rat
	for k, pair := range pairs {
		for l := range products[k] {
			products[k][l] = new(big.Rat)
		}
		a, b := p[pair[0]], p[pair[1]]
		for j := range a {
			for n := range b {
				wt := big.NewRat(twistBinomial[3][j]*twistBinomial[3][n], twistBinomial[6][j+n])
				cross := sub(mul(a[j].U, b[n].V), mul(a[j].V, b[n].U))
				products[k][j+n].Add(products[k][j+n], mul(cross, wt))
			}
		}
	}
	three, six, nine := rint(3), rint(6), rint(9)
	fifth, tenth := big.NewRat(1, 5), big.NewRat(1, 10)
	var coeff [6][7]*big.Rat
	for l := range coeff[0] {
		a := mul(three, products[0][l])
		b := mul(six, products[1][l])
		c := add(mul(three, products[2][l]), mul(nine, products[3][l]))
		d := mul(six, products[4][l])
		e := mul(three, products[5][l])
		coeff[0][l] = a
		coeff[1][l] = mul(add(a, b), fifth)
		coeff[2][l] = mul(add(b, c), tenth)
		coeff[3][l] = mul(add(c, d), tenth)
		coeff[4][l] = mul(add(d, e), fifth)
		coeff[5][l] = e
	}
	return coeff
}

// twistRatProvePolarPositive is provePolarPositive over
// twistRatPolarCoefficients.
func twistRatProvePolarPositive(ctx context.Context, p twistPatch, budget *proofbound.WorkBudget) error {
	const maxDepth = 12
	var check func(twistPatch, int) error
	check = func(cell twistPatch, depth int) error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := budget.Step(); err != nil {
			return err
		}
		coeff := twistRatPolarCoefficients(cell)
		for _, corner := range [...][2]int{{0, 0}, {5, 0}, {0, 6}, {5, 6}} {
			if coeff[corner[0]][corner[1]].Sign() <= 0 {
				return fmt.Errorf("%w: twisted loft section has no increasing polar angle", decaderr.ErrUnsupported)
			}
		}
		positive := true
		for _, row := range coeff {
			for _, c := range row {
				if c.Sign() <= 0 {
					positive = false
				}
			}
		}
		if positive {
			return nil
		}
		if depth == maxDepth {
			return fmt.Errorf("%w: twisted loft has no positive polar-angle proof", decaderr.ErrUnsupported)
		}
		left, right := cell.split(depth%2 == 0)
		if err := check(left, depth+1); err != nil {
			return err
		}
		return check(right, depth+1)
	}
	return check(p, 0)
}

func twistRatL1(a freeform.RatPoint) *big.Rat { return add(abs(a.U), abs(a.V)) }

// twistRatBounds is bounds over big.Rat arithmetic.
func twistRatBounds(p twistPatch) twistBounds {
	b := twistBounds{bu: new(big.Rat), bt: new(big.Rat), muu: new(big.Rat), mtt: new(big.Rat),
		mut: new(big.Rat), radius: new(big.Rat)}
	for i := range p {
		for j := range p[i] {
			b.radius = maxRat(b.radius, twistRatL1(p[i][j]))
			if i < 3 {
				b.bu = maxRat(b.bu, mul(rint(3), twistRatL1(twistSub(p[i+1][j], p[i][j]))))
			}
			if j < 3 {
				b.bt = maxRat(b.bt, mul(rint(3), twistRatL1(twistSub(p[i][j+1], p[i][j]))))
			}
			if i < 2 {
				second := twistAdd(twistSub(p[i+2][j], twistScale(p[i+1][j], rint(2))), p[i][j])
				b.muu = maxRat(b.muu, mul(rint(6), twistRatL1(second)))
			}
			if j < 2 {
				second := twistAdd(twistSub(p[i][j+2], twistScale(p[i][j+1], rint(2))), p[i][j])
				b.mtt = maxRat(b.mtt, mul(rint(6), twistRatL1(second)))
			}
			if i < 3 && j < 3 {
				mixed := twistSub(twistSub(p[i+1][j+1], p[i+1][j]), twistSub(p[i][j+1], p[i][j]))
				b.mut = maxRat(b.mut, mul(rint(9), twistRatL1(mixed)))
			}
		}
	}
	return b
}

// twistRatRow is the patch's curve at height b/nt over big.Rat arithmetic.
func twistRatRow(p twistPatch, b, nt int64) [4]freeform.RatPoint {
	bt := twistBernstein3(big.NewRat(b, nt))
	var row [4]freeform.RatPoint
	for j := range p {
		row[j] = freeform.RatPoint{U: new(big.Rat), V: new(big.Rat)}
		for l := range p[j] {
			row[j] = twistAdd(row[j], twistScale(p[j][l], bt[l]))
		}
	}
	return row
}

// twistRatStation is one held station of buildTwistedMesh over big.Rat
// arithmetic: the rounded X, Y of the row's point at a/nu plus the origin,
// and the exact planar rounding when both are finite.
func twistRatStation(row [4]freeform.RatPoint, a, nu int64, ox, oy *big.Rat) (float64, float64, *big.Rat) {
	bu := twistBernstein3(big.NewRat(a, nu))
	pt := freeform.RatPoint{U: new(big.Rat), V: new(big.Rat)}
	for j := range row {
		pt = twistAdd(pt, twistScale(row[j], bu[j]))
	}
	x, y := add(pt.U, ox), add(pt.V, oy)
	xf, _ := x.Float64()
	yf, _ := y.Float64()
	if math.IsInf(xf, 0) || math.IsInf(yf, 0) {
		return xf, yf, nil
	}
	return xf, yf, add(abs(sub(x, rf(xf))), abs(sub(y, rf(yf))))
}

// twistTestSource draws coordinates whose denominators cover the shapes the
// route meets: floats, floats over 3 and 9 (the thirds and slopes of
// twistPatchOf), small and large odd factors (fit-spline solves), integers,
// zeros, negatives, and huge and subnormal floats.
type twistTestSource struct{ rng *rand.Rand }

func (s twistTestSource) float() float64 {
	switch s.rng.IntN(10) {
	case 0:
		return 0
	case 1:
		return float64(s.rng.IntN(41) - 20)
	case 2:
		return math.Ldexp(s.rng.Float64()-0.5, -1030) // subnormal
	case 3:
		return math.Ldexp(s.rng.Float64()-0.5, 900)
	default:
		return (s.rng.Float64() - 0.5) * 40
	}
}

func (s twistTestSource) coordinate() *big.Rat {
	q := rf(s.float())
	switch s.rng.IntN(6) {
	case 0:
		return q.Quo(q, rint([...]int64{3, 9, 27, 5, 7}[s.rng.IntN(5)]))
	case 1:
		den := new(big.Int).SetUint64(s.rng.Uint64())
		den.Lsh(den, 64).Or(den, new(big.Int).SetUint64(s.rng.Uint64()))
		den.Lsh(den, 32).Or(den, new(big.Int).SetUint64(s.rng.Uint64()>>32))
		den.SetBit(den, 0, 1)
		return q.Quo(q, new(big.Rat).SetFrac(den, new(big.Int).Lsh(big.NewInt(1), 150)))
	default:
		return q
	}
}

func (s twistTestSource) point() freeform.RatPoint {
	return freeform.RatPoint{U: s.coordinate(), V: s.coordinate()}
}

// patch is either a free control net or one built as the route builds it,
// from section spans, Z slopes and an exact float height.
func (s twistTestSource) patch() twistPatch {
	if s.rng.IntN(2) == 0 {
		var p twistPatch
		for i := range p {
			for j := range p[i] {
				p[i][j] = s.point()
			}
		}
		return p
	}
	var c0, c1, m0, m1 twistSpan
	for j := range 4 {
		c0[j], c1[j], m0[j], m1[j] = s.point(), s.point(), s.point(), s.point()
	}
	return twistPatchOf(c0, c1, m0, m1, rf(0.26+s.rng.Float64()))
}

// twistTestPositivePatch is a counterclockwise line patch about the origin
// with every control perturbed by up to scale/2 in each coordinate, so a
// small scale proves at once, a larger one needs subdivision, and the
// largest refuses.
func twistTestPositivePatch(rng *rand.Rand, scale float64) twistPatch {
	p := twistTestLinePatch(twistTestPoint(4, -2), twistTestPoint(4, 2))
	for i := range p {
		for j := range p[i] {
			p[i][j] = freeform.RatPoint{U: add(p[i][j].U, rf((rng.Float64()-0.5)*scale)),
				V: add(p[i][j].V, rf((rng.Float64()-0.5)*scale))}
		}
	}
	return p
}

func requireTwistBoundsEqual(t *testing.T, want, got twistBounds, sample int) {
	t.Helper()
	for _, pair := range [...][2]*big.Rat{{want.bu, got.bu}, {want.bt, got.bt}, {want.muu, got.muu},
		{want.mtt, got.mtt}, {want.mut, got.mut}, {want.radius, got.radius}} {
		require.Zero(t, pair[0].Cmp(pair[1]), "sample %d: bound %s, want %s", sample, pair[1], pair[0])
	}
}

func TestTwistedCommonMatchesRationalPatchAlgebra(t *testing.T) {
	t.Parallel()
	src := twistTestSource{rng: rand.New(rand.NewPCG(5, 2655))}
	for sample := range twistTestSamples(200) {
		p := src.patch()
		c := p.overCommon()
		require.Positive(t, c.d.Sign())
		for i := range p {
			for j := range p[i] {
				for k, x := range [2]*big.Rat{p[i][j].U, p[i][j].V} {
					got := new(big.Rat).SetFrac(&c.q[i][j][k], c.d)
					require.Zero(t, got.Cmp(x), "sample %d: coordinate (%d,%d,%d)", sample, i, j, k)
					// d is the least common denominator: it divides every
					// common denominator, so no coordinate may be over less.
					require.Zero(t, new(big.Int).Rem(c.d, x.Denom()).Sign())
				}
			}
		}
		want := twistRatPolarCoefficients(p)
		got := p.polarCoefficients()
		for k := range want {
			for l := range want[k] {
				require.Zero(t, want[k][l].Cmp(got[k][l]), "sample %d: coefficient (%d,%d)", sample, k, l)
			}
		}
		requireTwistBoundsEqual(t, twistRatBounds(p), p.bounds(), sample)
	}
}

func TestTwistedCommonLeastDenominator(t *testing.T) {
	t.Parallel()
	// Denominators 2^3·3, 2·9 and 5 have least common multiple 2^3·9·5.
	var p twistPatch
	for i := range p {
		for j := range p[i] {
			p[i][j] = freeform.RatPoint{U: big.NewRat(1, 24), V: big.NewRat(7, 18)}
		}
	}
	p[2][3].U = big.NewRat(-3, 5)
	require.Zero(t, p.overCommon().d.Cmp(big.NewInt(360)))
}

func TestTwistedCommonProofMatchesRationalProof(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 2655))
	verdicts := map[string]int{}
	for sample := range 120 {
		p := twistTestPositivePatch(rng, []float64{0.5, 5, 7, 12}[sample%4])
		var wantSteps, gotSteps int
		count := func(n *int) *proofbound.WorkBudget {
			return &proofbound.WorkBudget{StepFn: func() error { *n++; return nil }, ErrFn: t.Context().Err}
		}
		want := twistRatProvePolarPositive(t.Context(), p, count(&wantSteps))
		got := p.provePolarPositive(t.Context(), count(&gotSteps))
		require.Equal(t, wantSteps, gotSteps, "sample %d", sample)
		if want == nil {
			require.NoError(t, got, "sample %d", sample)
		} else {
			require.EqualError(t, got, want.Error(), "sample %d", sample)
			require.True(t, errors.Is(got, decaderr.ErrUnsupported))
		}
		key := "proved"
		if want != nil {
			key = want.Error()
		}
		if want == nil && wantSteps > 1 {
			key = "proved after split"
		}
		verdicts[key]++
	}
	// The samples reach a proof at once, a proof after subdivision, and a
	// corner refusal.
	require.Len(t, verdicts, 3, "%v", verdicts)
}

func TestTwistedCommonStationsMatchRationalStations(t *testing.T) {
	t.Parallel()
	src := twistTestSource{rng: rand.New(rand.NewPCG(23, 2655))}
	for sample := range twistTestSamples(40) {
		oxf, oyf := src.float(), src.float()
		ox, oy := rf(oxf), rf(oyf)
		o := twistOriginOf(ox, oy)
		nt := int64(1 + src.rng.IntN(32))
		b := src.rng.Int64N(nt + 1)
		// One ring crosses several patches, so the kept rounding compares
		// fractions over different denominators.
		if sample == 0 {
			// Stations past the float range round to +Inf on both paths.
			ox, oy = rf(math.MaxFloat64), rf(math.MaxFloat64)
			o = twistOriginOf(ox, oy)
		}
		var rho twistRoundMax
		want := new(big.Rat)
		for range 3 {
			p := src.patch()
			if sample == 0 {
				p = twistTestLinePatch(freeform.RatPoint{U: rf(math.MaxFloat64), V: rint(1)},
					freeform.RatPoint{U: rint(1), V: rf(math.MaxFloat64)})
			}
			nu := int64(1 + src.rng.IntN(64))
			curve := p.overCommon().curveAt(twistBernsteinScaled(b, nt), int(nu), int(nt), o)
			var x, y big.Int
			row := twistRatRow(p, b, nt)
			for a := range nu + 1 {
				wantX, wantY, wantErr := twistRatStation(row, a, nu, ox, oy)
				curve.point(twistBernsteinScaled(a, nu), &x, &y)
				gotX, gotY := proof.FracFloat64(&x, &curve.den), proof.FracFloat64(&y, &curve.den)
				require.Equal(t, math.Float64bits(wantX), math.Float64bits(gotX), "sample %d station %d", sample, a)
				require.Equal(t, math.Float64bits(wantY), math.Float64bits(gotY), "sample %d station %d", sample, a)
				if wantErr == nil {
					continue
				}
				rho.keep(&x, &y, &curve.den, gotX, gotY)
				want = maxRat(want, wantErr)
			}
		}
		require.Zero(t, want.Cmp(rho.rat()), "sample %d: rounding %s, want %s", sample, rho.rat(), want)
	}
}

// twistBenchPatches are patches as the route builds them: float section
// controls, Z slopes over the exact float spacing of 0.26 mm levels.
func twistBenchPatches(n int) []twistPatch {
	rng := rand.New(rand.NewPCG(3, 2655))
	zs := make([]*big.Rat, n+2)
	for i := range zs {
		zs[i] = rf(float64(i) * 0.26)
	}
	spans := make([]twistSpan, len(zs))
	for i := range spans {
		angle := float64(i) * 0.26 / (49.5 / (2 * math.Pi))
		for j := range spans[i] {
			x, y := 7+rng.Float64(), -2+float64(j)*4/3+rng.Float64()/10
			spans[i][j] = freeform.RatPoint{U: rf(x*math.Cos(angle) - y*math.Sin(angle)),
				V: rf(x*math.Sin(angle) + y*math.Cos(angle))}
		}
	}
	tangents := make([]twistSpan, len(zs))
	for j := range 4 {
		points := make([]freeform.RatPoint, len(zs))
		for i := range spans {
			points[i] = spans[i][j]
		}
		for i, m := range twistSlopes(points, zs) {
			tangents[i][j] = m
		}
	}
	patches := make([]twistPatch, n)
	for i := range patches {
		patches[i] = twistPatchOf(spans[i+1], spans[i+2], tangents[i+1], tangents[i+2], sub(zs[i+2], zs[i+1]))
	}
	return patches
}

// twistBenchRat names the big.Rat reference in the benchmarks.
const twistBenchRat = "rat"

func BenchmarkTwistedPatchProof(b *testing.B) {
	patches := twistBenchPatches(16)
	for _, impl := range []string{twistBenchRat, "common"} {
		b.Run("impl="+impl, func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; b.Loop(); i++ {
				p := patches[i%len(patches)]
				budget := proofbound.NewWorkBudget(b.Context())
				var err error
				if impl == twistBenchRat {
					err = twistRatProvePolarPositive(b.Context(), p, budget)
				} else {
					err = p.provePolarPositive(b.Context(), budget)
				}
				require.NoError(b, err)
			}
		})
	}
}

func BenchmarkTwistedPatchBounds(b *testing.B) {
	patches := twistBenchPatches(16)
	for _, impl := range []string{twistBenchRat, "common"} {
		b.Run("impl="+impl, func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; b.Loop(); i++ {
				p := patches[i%len(patches)]
				if impl == twistBenchRat {
					_ = twistRatBounds(p)
				} else {
					_ = p.bounds()
				}
			}
		})
	}
}

// BenchmarkTwistedStations evaluates one ring band of 32 stations on one
// patch, with its rounding, as buildTwistedMesh does per patch and ring.
func BenchmarkTwistedStations(b *testing.B) {
	const nu, nt, band = 32, 8, 3
	patches := twistBenchPatches(16)
	commons := make([]*twistCommon, len(patches))
	for i, p := range patches {
		commons[i] = p.overCommon()
	}
	ox, oy := rf(0), rf(0)
	o := twistOriginOf(ox, oy)
	for _, impl := range []string{twistBenchRat, "common"} {
		b.Run("impl="+impl, func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; b.Loop(); i++ {
				k := i % len(patches)
				if impl == twistBenchRat {
					rho := new(big.Rat)
					row := twistRatRow(patches[k], band, nt)
					for a := range int64(nu) {
						_, _, e := twistRatStation(row, a, nu, ox, oy)
						rho = maxRat(rho, e)
					}
					continue
				}
				var rho twistRoundMax
				var x, y big.Int
				curve := commons[k].curveAt(twistBernsteinScaled(band, nt), nu, nt, o)
				for a := range int64(nu) {
					curve.point(twistBernsteinScaled(a, nu), &x, &y)
					xf, yf := proof.FracFloat64(&x, &curve.den), proof.FracFloat64(&y, &curve.den)
					rho.keep(&x, &y, &curve.den, xf, yf)
				}
				_ = rho.rat()
			}
		})
	}
}
