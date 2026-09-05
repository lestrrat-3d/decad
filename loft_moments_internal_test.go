package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// addBoxFaces folds a closed, outward-oriented axis-aligned box [0,a]x[0,b]x
// [0,c] into the accumulator as 12 wall triangles (2 per face, split along
// a fixed diagonal, every triangle wound so (B-A)x(C-A) points outward).
func addBoxFaces(m *loftMassAccumulator, a, b, c float64) {
	p := func(x, y, z float64) r3.Vec { return r3.NewVec(x, y, z) }
	p000, p100 := p(0, 0, 0), p(a, 0, 0)
	p010, p110 := p(0, b, 0), p(a, b, 0)
	p001, p101 := p(0, 0, c), p(a, 0, c)
	p011, p111 := p(0, b, c), p(a, b, c)

	tris := [][3]r3.Vec{
		{p000, p010, p110}, {p000, p110, p100}, // bottom, -z
		{p001, p101, p111}, {p001, p111, p011}, // top, +z
		{p000, p100, p101}, {p000, p101, p001}, // front, -y
		{p010, p111, p110}, {p010, p011, p111}, // back, +y
		{p000, p011, p010}, {p000, p001, p011}, // left, -x
		{p100, p111, p101}, {p100, p110, p111}, // right, +x
	}
	for _, tri := range tris {
		m.add(tri[0], tri[1], tri[2], true)
	}
}

// TestLoftMassAccumulatorBoxIsExact reproduces a closed box's closed-form
// volume and all three centroid coordinates exactly: every dimension is a
// power of two, so every intermediate rational is exactly representable in
// float64 (docs/loft-design.md §8, required test).
func TestLoftMassAccumulatorBoxIsExact(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	addBoxFaces(m, 2, 2, 2)

	vol := m.volume(nil, nil)
	require.Equal(t, 8.0, vol.Value.Base())
	require.Equal(t, Exact, vol.Exactness)
	require.Equal(t, 0.0, vol.Bound.Base())

	c, err := m.centroid(nil, nil)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(1, 1, 1), c.Value)
	require.Equal(t, Exact, c.Exactness)
	require.Equal(t, 0.0, c.Bound.Base())

	box, ok := m.bounds()
	require.True(t, ok)
	require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
	require.Equal(t, r3.NewVec(2, 2, 2), box.Max)
	require.Equal(t, Exact, box.Exactness)
	require.Equal(t, 0.0, box.Bound.Base())
}

// TestLoftMassAccumulatorBoxAnchoredElsewhereStillExact proves the anchor
// shift itself introduces no rounding: an anchor away from the origin still
// reproduces the exact volume and centroid, since every subtraction is
// carried out over rationals (docs/loft-design.md §8's anchor discipline).
func TestLoftMassAccumulatorBoxAnchoredElsewhereStillExact(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(-4, 8, 0.5), 0, 0, 0)
	addBoxFaces(m, 2, 2, 2)

	vol := m.volume(nil, nil)
	require.Equal(t, 8.0, vol.Value.Base())
	require.Equal(t, Exact, vol.Exactness)

	c, err := m.centroid(nil, nil)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(1, 1, 1), c.Value)
	require.Equal(t, Exact, c.Exactness)
}

// TestLoftMassAccumulatorVolumeApproximate is a single tetrahedron (O at the
// anchor, A=(1,0,0), B=(0,1,0), C=(0,0,1)) whose only volume-contributing
// face is ABC: vol6 = A.(BxC) = 1, so Volume = 1/6, which float64 cannot
// represent exactly — Approximate, with the proven single-rounding bound
// (docs/loft-design.md §8).
func TestLoftMassAccumulatorVolumeApproximate(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	m.add(r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1), false)

	vol := m.volume(nil, nil)
	wantRat := big.NewRat(1, 6)
	wantFloat, exact := wantRat.Float64()
	require.False(t, exact, "1/6 must not be representable in float64 for this test to mean anything")
	require.Equal(t, wantFloat, vol.Value.Base())
	require.Equal(t, Approximate, vol.Exactness)
	require.Equal(t, rationalFloatError(wantRat, wantFloat), vol.Bound.Base())
	require.Greater(t, vol.Bound.Base(), 0.0)
}

// TestLoftMassAccumulatorCentroidApproximate combines two tetrahedra sharing
// the anchor vertex O — vol6_1=1 (A1=(1,0,0),B1=(0,1,0),C1=(0,0,1)) and
// vol6_2=2 (A2=(1,0,0),B2=(-1,2,0),C2=(0,0,1)) — into one accumulator. The
// combined centroid is a genuine volume-weighted average
// (vol6_1*sum1 + vol6_2*sum2)/(4*(vol6_1+vol6_2)); by construction its X and
// Y coordinates land on a denominator of 12 (an irreducible factor of 3),
// which float64 cannot represent, while Z lands on 3/12 = 1/4, which it can.
// Centroid must therefore be Approximate, bounded by radius3D of the worst
// per-coordinate rounding, and Exact only when every coordinate rounds
// exactly (docs/loft-design.md §8).
func TestLoftMassAccumulatorCentroidApproximate(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	m.add(r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1), false)
	m.add(r3.NewVec(1, 0, 0), r3.NewVec(-1, 2, 0), r3.NewVec(0, 0, 1), false)

	wantVol6 := big.NewRat(3, 1)
	wantMomX := big.NewRat(1, 1)
	wantMomY := big.NewRat(5, 1)
	wantMomZ := big.NewRat(3, 1)
	denom := new(big.Rat).Mul(big.NewRat(4, 1), wantVol6)
	wantCX := new(big.Rat).Quo(wantMomX, denom)
	wantCY := new(big.Rat).Quo(wantMomY, denom)
	wantCZ := new(big.Rat).Quo(wantMomZ, denom)

	fx, exactX := wantCX.Float64()
	fy, exactY := wantCY.Float64()
	fz, exactZ := wantCZ.Float64()
	require.False(t, exactX, "1/12 must not be representable in float64 for this test to mean anything")
	require.False(t, exactY, "5/12 must not be representable in float64 for this test to mean anything")
	require.True(t, exactZ, "1/4 must be representable in float64 for this test to mean anything")

	c, err := m.centroid(nil, nil)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(fx, fy, fz), c.Value)
	require.Equal(t, Approximate, c.Exactness)

	bx := rationalFloatError(wantCX, fx)
	by := rationalFloatError(wantCY, fy)
	bz := rationalFloatError(wantCZ, fz)
	require.Equal(t, 0.0, bz)
	require.Equal(t, radius3D(math.Max(bx, by)), c.Bound.Base())
	require.Greater(t, c.Bound.Base(), 0.0)
}

// TestLoftMassAccumulatorCentroidZeroVolumeDegenerate proves a zero net
// volume has no centroid: two opposing tetrahedra of equal and opposite
// signed volume cancel exactly.
func TestLoftMassAccumulatorCentroidZeroVolumeDegenerate(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	m.add(r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1), false)
	m.add(r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), false) // reversed winding

	_, err := m.centroid(nil, nil)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestLoftMassAccumulatorAreaApproximateBoundedByReference builds two wall
// triangles whose exact areas are irrational (sqrt(2)/2 and sqrt(13)) plus a
// cap contribution of the non-representable exact rational 5/3, then checks
// the reported bound against a 256-bit high-precision reference sum — never
// merely asserting the bound is present (docs/loft-design.md §8, required
// test).
func TestLoftMassAccumulatorAreaApproximateBoundedByReference(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	m.add(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 1), true) // area sqrt(2)/2
	m.add(r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(0, 2, 3), true) // area sqrt(13)
	capArea := big.NewRat(5, 3)

	area := m.area(capArea)
	require.Equal(t, Approximate, area.Exactness)
	require.Greater(t, area.Bound.Base(), 0.0)

	const prec = 256
	sqrt := func(x int64) *big.Float {
		return new(big.Float).SetPrec(prec).Sqrt(new(big.Float).SetPrec(prec).SetInt64(x))
	}
	two := sqrt(2)
	half := new(big.Float).SetPrec(prec).Quo(two, new(big.Float).SetPrec(prec).SetInt64(2))
	thirteen := sqrt(13)
	capF := new(big.Float).SetPrec(prec).SetRat(capArea)

	ref := new(big.Float).SetPrec(prec)
	ref.Add(half, thirteen)
	ref.Add(ref, capF)

	held := new(big.Float).SetPrec(prec).SetFloat64(area.Value.Base())
	diff := new(big.Float).SetPrec(prec).Sub(ref, held)
	diff.Abs(diff)
	diffF, _ := diff.Float64()

	require.LessOrEqual(t, diffF, area.Bound.Base(),
		"the proven bound must cover the gap to a high-precision reference sum")
}

// referenceTriangleArea returns one triangle's true area at prec bits. The
// edge vectors, their cross product and its squared norm are taken over exact
// rationals — a float64 IS a rational — so the only inexactness anywhere is
// the closing square root, computed at far more precision than float64 holds.
func referenceTriangleArea(a, b, c r3.Vec, prec uint) *big.Float {
	u := xsub(xptOf(b), xptOf(a))
	v := xsub(xptOf(c), xptOf(a))
	w := xcross(u, v)
	q := xdotRat(w, w)
	q.Quo(q, big.NewRat(4, 1))
	return new(big.Float).SetPrec(prec).Sqrt(new(big.Float).SetPrec(prec).SetRat(q))
}

// TestLoftMassAccumulatorAreaBoundEnclosesTrueError is the enclosure
// assertion the reported bound exists to earn: for every row the published
// |held - true| must be at or under the published Bound, with the truth
// computed at 400 bits.
//
// The table is deliberately weighted toward SLIVERS — the perpendicular offset
// over the base length runs from 1 down to 1e-9 — because a thin triangle is
// where a float cross product cancels and a bound scaled off the held total
// stops enclosing anything. docs/loft-design.md Table B splits every wall quad
// along a diagonal, so a short loft over long recorded LineSegs produces
// exactly these aspect ratios as its ordinary walls, and no Table S gate
// refuses a thin-but-valid triangle or caps coordinate magnitude — hence the
// large-coordinate row too.
func TestLoftMassAccumulatorAreaBoundEnclosesTrueError(t *testing.T) {
	t.Parallel()
	// Two orthonormal, non-axis-aligned directions: dir.perp is exactly 0 and
	// both have unit length, so a row's aspect really is its offset over its
	// base and no coordinate is a special case of the arithmetic.
	dir := r3.NewVec(0.6, 0.8, 0)
	perp := r3.NewVec(-0.48, 0.36, 0.8)
	require.Equal(t, 0.0, dir.Dot(perp))

	// sliver spans base along dir and offsets its apex base*aspect along perp,
	// planted a third of the way along so no two edges share a length.
	sliver := func(origin r3.Vec, base, aspect float64) [3]r3.Vec {
		return [3]r3.Vec{
			origin,
			origin.Add(dir.Scale(base)),
			origin.Add(dir.Scale(base * 0.37)).Add(perp.Scale(base * aspect)),
		}
	}

	origin := r3.NewVec(12.5, -7.25, 3.5)
	far := r3.NewVec(1e10, -1e10, 1e10)
	for _, tc := range []struct {
		name string
		tri  [3]r3.Vec
	}{
		{name: "aspect 1", tri: sliver(origin, 100, 1)},
		{name: "aspect 1e-2", tri: sliver(origin, 100, 1e-2)},
		{name: "aspect 1e-3", tri: sliver(origin, 100, 1e-3)},
		{name: "aspect 1e-6", tri: sliver(origin, 100, 1e-6)},
		{name: "aspect 1e-9", tri: sliver(origin, 100, 1e-9)},
		{name: "large coordinates aspect 1e-3", tri: sliver(far, 8192, 1e-3)},
		{name: "large coordinates aspect 1e-6", tri: sliver(far, 8192, 1e-6)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
			m.add(tc.tri[0], tc.tri[1], tc.tri[2], true)
			area := m.area()

			require.Equal(t, Approximate, area.Exactness,
				"a wall triangle's area is a square root of a rational and is never Exact")
			require.Greater(t, area.Bound.Base(), 0.0)

			const prec = 400
			ref := referenceTriangleArea(tc.tri[0], tc.tri[1], tc.tri[2], prec)
			require.Equal(t, 1, ref.Sign(), "the fixture must have positive area")

			held := new(big.Float).SetPrec(prec).SetFloat64(area.Value.Base())
			diff := new(big.Float).SetPrec(prec).Sub(ref, held)
			diff.Abs(diff)
			// Round the measured error UPWARD, so the assertion can never pass
			// on the reference's own rounding.
			diffF, acc := diff.Float64()
			if acc == big.Below {
				diffF = math.Nextafter(diffF, math.Inf(1))
			}

			refF, _ := ref.Float64()
			t.Logf("true=%.17g held=%.17g error=%.17g bound=%.17g",
				refF, area.Value.Base(), diffF, area.Bound.Base())
			require.LessOrEqual(t, diffF, area.Bound.Base(),
				"the reported bound must ENCLOSE the true error, not estimate it")
		})
	}
}

// TestLoftMassAccumulatorAreaBoundEnclosesSliverSum repeats the enclosure
// assertion over a MANY-triangle wall set, so the summation loop's own slop is
// under test beside the per-triangle brackets: 64 slivers at aspect 1e-5 and
// 1e-6, spread over a growing coordinate range with long bases, plus a
// non-representable exact rational cap contribution. Charging every triangle
// at the held total's scale does not cover this set either — the per-triangle
// enclosure widths are what carry it.
func TestLoftMassAccumulatorAreaBoundEnclosesSliverSum(t *testing.T) {
	t.Parallel()
	dir := r3.NewVec(0.6, 0.8, 0)
	perp := r3.NewVec(-0.48, 0.36, 0.8)

	const prec = 400
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	ref := new(big.Float).SetPrec(prec)
	for i := range 64 {
		origin := r3.NewVec(100*float64(i), -25*float64(i), 3.5)
		base := 1e4 + 100*float64(i)
		aspect := math.Pow(10, -float64(5+i%2))
		a := origin
		b := origin.Add(dir.Scale(base))
		c := origin.Add(dir.Scale(base * 0.37)).Add(perp.Scale(base * aspect))
		m.add(a, b, c, true)
		ref.Add(ref, referenceTriangleArea(a, b, c, prec))
	}

	capArea := big.NewRat(5, 3)
	ref.Add(ref, new(big.Float).SetPrec(prec).SetRat(capArea))

	area := m.area(capArea)
	require.Equal(t, Approximate, area.Exactness)

	held := new(big.Float).SetPrec(prec).SetFloat64(area.Value.Base())
	diff := new(big.Float).SetPrec(prec).Sub(ref, held)
	diff.Abs(diff)
	diffF, acc := diff.Float64()
	if acc == big.Below {
		diffF = math.Nextafter(diffF, math.Inf(1))
	}

	refF, _ := ref.Float64()
	t.Logf("true=%.17g held=%.17g error=%.17g bound=%.17g",
		refF, area.Value.Base(), diffF, area.Bound.Base())
	require.LessOrEqual(t, diffF, area.Bound.Base(),
		"the reported bound must ENCLOSE the true error over a whole wall set")
}

// addLoftWalls folds the wall triangles of a loft between two congruent
// planar loops into the accumulator, in profile segment order and split the
// way docs/loft-design.md Table B splits every wall quad — along the
// bottom-start/top-end diagonal, so segment i contributes (b_i, b_i+1, t_i+1)
// then (b_i, t_i+1, t_i). bottom and top must list corresponding points.
func addLoftWalls(m *loftMassAccumulator, bottom, top []r3.Vec) {
	for i := range bottom {
		j := (i + 1) % len(bottom)
		m.add(bottom[i], bottom[j], top[j], true)
		m.add(bottom[i], top[j], top[i], true)
	}
}

// TestLoftMassAccumulatorAreaBoundSurvivesSaturatedScale is the regression for
// the wall summation's own SCALE saturating while the summed value does not.
//
// wallAreaAbs diverges upward from wallAreaSum by one upRound per term, so it
// can reach +Inf while wallAreaSum is still finite — and sumSlop reports 0 for
// a non-finite absSum, which is the whole of the wall loop's summation cover.
// With every triangle's own area exactly representable the enclosure slack is
// 0 too, so an unguarded area() publishes a zero bound and Exact over a value
// that has silently swallowed four whole triangles.
//
// The witness is two congruent rectangles of width 2^-27 and height
// 2^27 − 2^-26 on the planes z = 0 and z = 2^996: four long wall triangles of
// area exactly MaxFloat64/4 and four short ones of area exactly 2^968, just
// under half an ulp of MaxFloat64. Their float sum saturates at MaxFloat64
// while the true total runs past it.
func TestLoftMassAccumulatorAreaBoundSurvivesSaturatedScale(t *testing.T) {
	t.Parallel()
	width := math.Ldexp(1, -27)
	height := math.Ldexp(1, 27) - math.Ldexp(1, -26)
	lift := math.Ldexp(1, 996)

	corners := func(z float64) []r3.Vec {
		return []r3.Vec{
			r3.NewVec(0, 0, z),
			r3.NewVec(height, 0, z),
			r3.NewVec(height, width, z),
			r3.NewVec(0, width, z),
		}
	}

	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	addLoftWalls(m, corners(0), corners(lift))

	// The fixture is only meaningful while it really does saturate the scale
	// without saturating the sum, and while no triangle's own bracket has any
	// width of its own to carry the bound.
	require.Equal(t, 8, m.wallTerms)
	require.True(t, math.IsInf(m.wallAreaAbs, 1),
		"the fixture must drive the summation scale to +Inf")
	require.False(t, math.IsInf(m.wallAreaSum, 1),
		"the fixture must leave the summed value finite")
	require.Equal(t, 0.0, m.wallAreaSlack,
		"every triangle area here is exactly representable, so the enclosure slack must be 0")
	require.Equal(t, 0.0, sumSlop(m.wallTerms, m.wallAreaAbs),
		"sumSlop reports nothing for a saturated scale — the hole this test guards")

	area := m.area()
	require.Equal(t, math.MaxFloat64, area.Value.Base(),
		"the naive float sum saturates at MaxFloat64")

	// The truth, at 400 bits: four times MaxFloat64/4 plus four times 2^968.
	const prec = 400
	ref := new(big.Float).SetPrec(prec)
	quarter := new(big.Float).SetPrec(prec).SetFloat64(math.MaxFloat64 / 4)
	short := new(big.Float).SetPrec(prec).SetFloat64(math.Ldexp(1, 968))
	for range 4 {
		ref.Add(ref, quarter)
		ref.Add(ref, short)
	}

	held := new(big.Float).SetPrec(prec).SetFloat64(area.Value.Base())
	diff := new(big.Float).SetPrec(prec).Sub(ref, held)
	diff.Abs(diff)
	diffF, acc := diff.Float64()
	if acc == big.Below {
		diffF = math.Nextafter(diffF, math.Inf(1))
	}
	require.Greater(t, diffF, 0.0, "the fixture must actually lose mass")
	t.Logf("held=%.17g error=%.17g bound=%.17g", area.Value.Base(), diffF, area.Bound.Base())

	require.LessOrEqual(t, diffF, area.Bound.Base(),
		"the reported bound must ENCLOSE the true error even where the summation scale saturated")

	// Once the scale has overflowed there is no finite proven scale left, and
	// the true error here already exceeds MaxFloat64's own ulp by orders of
	// magnitude — any finite substitute would be a guess.
	require.True(t, math.IsInf(area.Bound.Base(), 1),
		"a saturated summation scale must publish an infinite bound, never a finite guess")
	require.Equal(t, Approximate, area.Exactness)
}

// TestLoftMassAccumulatorAreaNeverExact pins docs/loft-design.md §8's rule
// itself — "Area is never Exact" — rather than any single input that breaks a
// derived Exactness. A triangle's own area is a square root of a rational and
// is generically irrational, so the published reading is the CONSTANT
// Approximate and no arithmetic on the proven bound decides it.
//
// The table is keyed on the BOUND REGIME rather than on a witness, because a
// bound-derived Exactness fails wherever the bound arithmetic runs out of
// scale to state, and that happens at BOTH ends of float range. Each row whose
// bound is zero would read Exact under a derived Exactness — asserted per row,
// so the test fails loudly if the derivation returns — and three independent
// mechanisms reach a zero bound here: an empty wall set, a representable cap
// rational with no wall at all, and a subnormal wall triangle whose summation
// term underflows. The +Inf regime at the far end is pinned by
// TestLoftMassAccumulatorAreaBoundSurvivesSaturatedScale, which asserts the
// same constant.
func TestLoftMassAccumulatorAreaNeverExact(t *testing.T) {
	t.Parallel()
	// square is a unit square's corners on the plane z, in walk order.
	square := func(z float64) []r3.Vec {
		return []r3.Vec{
			r3.NewVec(0, 0, z), r3.NewVec(1, 0, z),
			r3.NewVec(1, 1, z), r3.NewVec(0, 1, z),
		}
	}

	for _, tc := range []struct {
		name      string
		build     func(*loftMassAccumulator)
		caps      []*big.Rat
		zeroBound bool
	}{
		{
			// An ordinary loft: two congruent unit squares three apart, every
			// wall triangle's own area representable and the whole set well
			// conditioned. Nothing here is near a float limit.
			name:  "ordinary well-conditioned wall set",
			build: func(m *loftMassAccumulator) { addLoftWalls(m, square(0), square(3)) },
			caps:  []*big.Rat{big.NewRat(1, 1), big.NewRat(1, 1)},
		},
		{
			name:      "no triangle and no cap",
			build:     func(*loftMassAccumulator) {},
			zeroBound: true,
		},
		{
			name:      "representable cap rational with no wall",
			build:     func(*loftMassAccumulator) {},
			caps:      []*big.Rat{big.NewRat(4, 1)},
			zeroBound: true,
		},
		{
			// The smallest positive area float64 holds: |u x v|/2 is exactly
			// 2^-1074, so the per-triangle bracket has zero width, and the
			// summation term scaled off that magnitude underflows to zero.
			name: "subnormal wall triangle",
			build: func(m *loftMassAccumulator) {
				m.add(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, math.Ldexp(1, -1073), 0), true)
			},
			zeroBound: true,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
			tc.build(m)
			area := m.area(tc.caps...)

			require.Equal(t, Approximate, area.Exactness,
				"a loft's Area is never Exact, whatever the bound arithmetic produced")

			if !tc.zeroBound {
				require.Greater(t, area.Bound.Base(), 0.0)
				return
			}
			require.Equal(t, 0.0, area.Bound.Base(),
				"this row exists to drive the bound to zero")
			require.Equal(t, Exact, exactnessOf(area.Bound.Base()),
				"a bound-derived Exactness reads Exact here — the derivation this rule forbids")
		})
	}
}

// TestLoftMassAccumulatorBoundsEmpty proves bounds reports false before any
// triangle has been added.
func TestLoftMassAccumulatorBoundsEmpty(t *testing.T) {
	t.Parallel()
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
	_, ok := m.bounds()
	require.False(t, ok)
}

// TestLoftMassAccumulatorVolumeChordedTermReadsMatchedDeltaNotSagitta pins
// a10-plan.md Part 3 PR 9 Task 1's own soundness fix: bounds.go's
// chordedBoundaryVolumeAllow (and chordedBoundaryMomentAllow /
// chordedBoundarySeamAllow inside computeLoftChordedAllow) must be composed
// with the build's own PARAMETER-MATCHED sectionMatchedDelta — NEVER
// sectionDelta, the build's own MAX SAGITTA (a SET-distance). The
// chord-to-curve half of that matched term coincides with the sagitta on a
// circular-only build (loftCircularCellStations' own doc comment: a circular
// cell's own sagitta discharges that half exactly), so the two differ there
// only by the delta leg §5.2's matchedDelta row adds — but a free-form cell
// can carry a chord-to-curve half strictly LARGER than its own sagitta
// (spline_sagitta_test.go's own overshoot-net counterexample: sagitta
// exactly 0, matchedDelta about 0.384), so composing with sectionDelta alone
// would silently understate a genuine chord-to-curve volume displacement —
// unsound in exactly the direction CLAUDE.md forbids.
//
// FALSIFICATION: reverting loft_moments.go's volume()/centroid() and
// computeLoftChordedAllow to read m.sectionDelta wherever they now read
// m.sectionMatchedDelta turns this test red — verified by hand during
// review (git stash the fix, rerun, confirm failure; restore). The fixture
// is potent BECAUSE sectionDelta and sectionMatchedDelta are given
// deliberately different values (0.001 vs 0.5) that a circular-only build
// could never produce, so no bit-identical-arc-fixture reason lets a caller
// swap them silently.
func TestLoftMassAccumulatorVolumeChordedTermReadsMatchedDeltaNotSagitta(t *testing.T) {
	t.Parallel()
	// Two stations per side: station 0 is the one curved cell under test;
	// station 1 stands in for the next segment's own first station — the
	// same "closing, non-curved station" shape
	// TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap already uses
	// — so cell 0 (j=0, matchedDelta[0] > 0) is charged and cell 1 (j=1,
	// matchedDelta[1] == 0, the wraparound back to station 0) is skipped.
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 1, 1)
	// wHi is displaced off the vLo/vHi/wLo plane (z=1.3, not 1) so the two
	// triangles m.add folds in below carry a genuinely nonzero signed
	// tetrahedron sum — a coplanar (T=0) quad would give vol6 == 0 and no
	// centroid to publish, which is not what this test is checking.
	wHi := r3.NewVec(1, 1, 1.3)
	verts := []r3.Vec{vLo, vHi, wLo, wHi}
	vIdx := [][]int{{0, 1}}
	wIdx := [][]int{{2, 3}}
	arcUpperV := []float64{1.01, 0}
	arcUpperW := []float64{1.1, 0} // wHi.Sub(wLo).Len() = sqrt(1.09) ~= 1.044; must stay below its own arc-length claim

	const sectionDelta = 0.001      // stands in for the build's own max sagitta
	const sectionMatchedDelta = 0.5 // stands in for its own max matchedDelta, deliberately far larger
	pairs := []loftLoopPair{{
		v: make([]Point2, 2), w: make([]Point2, 2),
		arcUpperV: arcUpperV, arcUpperW: arcUpperW,
		matchedDelta: []float64{sectionMatchedDelta, 0},
		// No arm placed these stations, so neither side carries a
		// constant-speed claim: +Inf is the absence of a tangent-deviation
		// energy proof, which cellChordCurveAreaAllow spends as its
		// premise-free arm (this fixture asserts on the VOLUME leg anyway).
		tangentEnergyV: []float64{math.Inf(1), math.Inf(1)},
		tangentEnergyW: []float64{math.Inf(1), math.Inf(1)},
	}}
	// anchor sits off every held vertex so neither triangle's own first
	// vertex cancels the tetrahedron term trivially (a vLo-anchored sum
	// would read vol6 == 0 regardless of geometry, since A-anchor is then
	// the zero vector for both of these triangles).
	anchor := r3.NewVec(-1, -1, -1)

	// delta is 0: the accumulator below carries no placement displacement
	// either, so the composed matchedDelta (docs/loft-design.md §5.2) is the
	// chord-to-curve half alone and the two sites agree.
	chorded, err := computeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, sectionMatchedDelta, 0, 2.0, false)
	require.NoError(t, err, "this fixture's cap plane offset is derivable")

	m := newLoftMassAccumulator(anchor, 0, sectionDelta, sectionMatchedDelta)
	m.chorded = chorded
	m.add(vLo, vHi, wHi, true)
	m.add(vLo, wHi, wLo, true)

	tris := [][3]int{{0, 1, 3}, {0, 3, 2}}
	vol := m.volume(verts, tris)

	// The reference this fixture must FALSIFY: chordedBoundaryVolumeAllow
	// composed with sectionDelta (the sagitta) rather than
	// sectionMatchedDelta — what a buggy caller reading the wrong field
	// would publish.
	wrongTerm := chordedBoundaryVolumeResidualAllow(
		sectionDelta, chorded.wallAreaUpper, chorded.capVolumeUpper, chorded.seamAllow,
	)
	rightTerm := chordedBoundaryVolumeResidualAllow(
		sectionMatchedDelta, chorded.wallAreaUpper, chorded.capVolumeUpper, chorded.seamAllow,
	)
	require.Greater(t, rightTerm, wrongTerm, "the fixture must actually distinguish the two candidate bounds")

	require.GreaterOrEqual(t, vol.Bound.Base(), rightTerm,
		"Volume's own Bound must compose the matchedDelta-keyed chordedBoundaryVolumeAllow term")
	require.Greater(t, vol.Bound.Base(), wrongTerm,
		"Volume's own Bound must exceed what composing the sagitta-keyed term alone would publish")

	// The moment leg's identical composition, checked at the scalar level
	// (chordedBoundaryMomentAllow itself, the SAME helper centroid() calls
	// with m.sectionMatchedDelta): this fixture's own volume/allowance ratio
	// is too extreme for m.centroid() to publish a positive S12 clearance
	// (an expected, unrelated refusal on a synthetic 2-triangle patch this
	// small), so the moment leg is checked directly rather than through the
	// full accumulator call.
	wrongMoment := chordedBoundaryMomentResidualAllow(
		sectionDelta, chorded.wallAreaUpper, chorded.capVolumeUpper,
		chorded.seamAllow, chorded.maxTwistOffsetUpper, m.coordUpper,
	)
	rightMoment := chordedBoundaryMomentResidualAllow(
		sectionMatchedDelta, chorded.wallAreaUpper, chorded.capVolumeUpper,
		chorded.seamAllow, chorded.maxTwistOffsetUpper, m.coordUpper,
	)
	require.Greater(t, rightMoment, wrongMoment, "the fixture must actually distinguish the two candidate moment terms")
}

// TestComputeLoftChordedAllowChargesTheHeldStationDisplacement pins
// docs/loft-design.md §5.2's matchedDelta row on every chorded leg: that row's
// certified source is absSumUpper(sectionDelta, delta), and the section closes
// with the exact failure this test falsifies — "Reading matchedDelta as
// sectionDelta alone leaves the computed station's own displacement uncharged
// on every chorded leg."
//
// The sagitta bounds the IDEAL chord between the two points the record denotes.
// The chord the build DRAWS joins two HELD stations, each displaced by delta
// from those points, so every leg keyed on the matched term — the wall upper,
// the ruled area leg, the cap-area tube, and the seam allowance with the
// posUpper it widens — must charge the SUM. Reachability needs neither an
// interior station nor a placement: delta > 0 is enough, and an unplaced loft
// over two same-kind ArcSeg profiles reaches it as soon as the settled chord
// count produces a computed station.
//
// FALSIFICATION: replace computeLoftChordedAllow's own per-cell
// chordCellDeltaUpper(p.matchedDelta[j], delta) with p.matchedDelta[j], or its
// cap-area/seam matchedDelta argument with the chord-to-curve half alone, and
// the matching leg below turns red — verified by hand (apply the shim, rerun,
// confirm failure, restore). The fixture is potent BECAUSE delta is the same
// order as the chord-to-curve half rather than the ulp-scale term a real build
// carries, so no leg can absorb the missing charge as rounding.
func TestComputeLoftChordedAllowChargesTheHeldStationDisplacement(t *testing.T) {
	t.Parallel()
	// The same one-curved-cell shape
	// TestLoftMassAccumulatorVolumeChordedTermReadsMatchedDeltaNotSagitta
	// uses: station 1 stands in for the next segment's own first station, so
	// cell 0 is charged and the wraparound cell 1 is skipped.
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 1, 1)
	wHi := r3.NewVec(1, 1, 1.3)
	verts := []r3.Vec{vLo, vHi, wLo, wHi}
	vIdx := [][]int{{0, 1}}
	wIdx := [][]int{{2, 3}}

	const chordToCurve = 0.5 // the cell's own sagitta half of the matched row
	const delta = 0.25       // the held stations' own displacement
	pairs := []loftLoopPair{{
		v: make([]Point2, 2), w: make([]Point2, 2),
		arcUpperV:      []float64{1.01, 0},
		arcUpperW:      []float64{1.1, 0},
		matchedDelta:   []float64{chordToCurve, 0},
		tangentEnergyV: []float64{math.Inf(1), math.Inf(1)},
		tangentEnergyW: []float64{math.Inf(1), math.Inf(1)},
	}}
	anchor := r3.NewVec(-1, -1, -1)

	matched := chordCellDeltaUpper(chordToCurve, delta)
	require.Greater(t, matched, chordToCurve, "the composition must actually widen the term it replaces")

	got, err := computeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, matched, delta, 2.0, false)
	require.NoError(t, err, "this fixture's cap plane offset is derivable")
	// What the sagitta-alone reading publishes: the identical call with the
	// station displacement dropped from both the per-cell and the build-wide
	// argument.
	sagittaOnly, err := computeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, chordToCurve, 0, 2.0, false)
	require.NoError(t, err, "the sagitta-alone reference reads the same derivable assembly")

	require.Greater(t, got.wallAreaUpper, sagittaOnly.wallAreaUpper,
		"the wall upper must charge the held station displacement")
	require.Greater(t, got.areaExcess, sagittaOnly.areaExcess,
		"the ruled area leg must charge the held station displacement")
	require.Greater(t, got.capAreaExcess, sagittaOnly.capAreaExcess,
		"the cap-area tube must charge the matched term, never the sagitta alone")
	require.Greater(t, got.seamAllow, sagittaOnly.seamAllow,
		"the seam allowance must charge the held station displacement")
	require.Greater(t, got.capVolumeUpper, sagittaOnly.capVolumeUpper,
		"the cap volume leg inherits the widened cap-area term")
	// The exact corrections and the offset leg read no displacement at all,
	// so the widening above is the matched term's and not blanket inflation.
	require.Zero(t, got.twistVolumeCorrection.Cmp(sagittaOnly.twistVolumeCorrection),
		"the twist volume correction reads no displacement term")
	require.Equal(t, sagittaOnly.twistVolumeUpper, got.twistVolumeUpper,
		"the occupied-volume twist measure reads no displacement term")
	for axis := range got.twistMomentCorrection {
		require.Zero(t, got.twistMomentCorrection[axis].Cmp(sagittaOnly.twistMomentCorrection[axis]),
			"the twist moment correction reads no displacement term on axis %d", axis)
	}
	require.Equal(t, sagittaOnly.maxTwistOffsetUpper, got.maxTwistOffsetUpper,
		"the twist offset leg reads no displacement term")

	// And the published measurement the legs feed: Volume's own chorded term
	// must dominate what the sagitta-alone reading would publish.
	m := newLoftMassAccumulator(anchor, delta, chordToCurve, matched)
	m.chorded = got
	m.add(vLo, vHi, wHi, true)
	m.add(vLo, wHi, wLo, true)
	tris := [][3]int{{0, 1, 3}, {0, 3, 2}}
	vol := m.volume(verts, tris)

	wrongTerm := chordedBoundaryVolumeResidualAllow(
		chordToCurve, sagittaOnly.wallAreaUpper, sagittaOnly.capVolumeUpper, sagittaOnly.seamAllow,
	)
	rightTerm := chordedBoundaryVolumeResidualAllow(
		matched, got.wallAreaUpper, got.capVolumeUpper, got.seamAllow,
	)
	require.Greater(t, rightTerm, wrongTerm, "the fixture must actually distinguish the two candidate bounds")
	require.GreaterOrEqual(t, vol.Bound.Base(), rightTerm,
		"Volume's own Bound must compose the matched-term chorded allowance")
	require.Greater(t, vol.Bound.Base(), wrongTerm,
		"Volume's own Bound must exceed what the sagitta-alone reading would publish")
}

// underivableCapOffsetPairs is the one-cell chorded pairing the two subtests
// below share: a single positive-matchedDelta cell on each side, so the wall
// loop runs and both caps carry a strictly positive sectionDisplacementArea,
// leaving the cap plane offset as the only term under test. The two tangent
// energies are +Inf, which costs cellChordCurveAreaAllow its sharp arm and
// nothing else (loft_build.go's perCellTangentEnergy).
func underivableCapOffsetPairs(matched float64) []loftLoopPair {
	return []loftLoopPair{{
		v: make([]Point2, 2), w: make([]Point2, 2),
		arcUpperV:      []float64{1.01, 0},
		arcUpperW:      []float64{1.1, 0},
		matchedDelta:   []float64{matched, 0},
		tangentEnergyV: []float64{math.Inf(1), math.Inf(1)},
		tangentEnergyW: []float64{math.Inf(1), math.Inf(1)},
	}}
}

// TestLoftCapOffsetUnderivableRefuses is docs/loft-design.md Table S row S14
// for §5.2's cap planeOffsetUpper term, decided in the CONSTRUCTION arm §4's
// gate-order paragraph assigns it: an assembly stating no proven distance from
// the mass anchor to a held cap1 vertex must REFUSE, never publish a finite
// substitute and never a zero (§5.2's own closing rule).
//
// Both subtests assert on the sentinel and on the WHOLE return being the zero
// value, so no leg reaches a consumer at all. Neither weakens any numeric
// bound: the fixture is built for this row and shares no assertion with the
// calibrated wedges.
func TestLoftCapOffsetUnderivableRefuses(t *testing.T) {
	t.Parallel()
	const matched = 0.5
	anchor := r3.NewVec(0, 0, 0)
	vIdx, wIdx := [][]int{{0, 1}}, [][]int{{2, 3}}

	t.Run("a cap1 distance past the float64 range refuses instead of publishing zero", func(t *testing.T) {
		// Both cap1 vertices sit a hair beyond MaxFloat64 from the anchor, so
		// ratSqrtUp's outward step off MaxFloat64 lands on +Inf for each and
		// the minimum over the cap never leaves +Inf.
		for _, v := range []r3.Vec{r3.NewVec(math.MaxFloat64, 0, 1), r3.NewVec(math.MaxFloat64, -1, 1)} {
			d2 := ratSquaredDistance3(anchor.X, anchor.Y, anchor.Z, v.X, v.Y, v.Z)
			require.NotNil(t, d2, "the coordinates are finite, so the squared distance is an exact rational")
			require.True(t, math.IsInf(ratSqrtUp(d2), 1), "no float64 upper bound on this distance exists")
		}

		// What a substituted zero would publish in its place, at this
		// fixture's own cap-area allowance: capAreaVolumeAllow takes its
		// planeOffsetUpper <= 0 arm and answers exactly 0 for a strictly
		// positive area gap, the SMALLEST possible number standing in for a
		// quantity no derivation states. The proven-underivable reading
		// answers +Inf for the same area gap, which is what a refusal must
		// stand on.
		capAreaAllow := sectionDisplacementArea(matched, 1, 1.1)
		require.Positive(t, capAreaAllow, "the fixture's cap carries a real area gap to fold into a volume")
		require.Zero(t, capAreaVolumeAllow(0, capAreaAllow), "a zero plane offset publishes a zero cap volume gap")
		require.True(t, math.IsInf(capAreaVolumeAllow(math.Inf(1), capAreaAllow), 1),
			"the underivable plane offset's own answer is +Inf, never that zero")

		verts := []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
			r3.NewVec(math.MaxFloat64, 0, 1), r3.NewVec(math.MaxFloat64, -1, 1),
		}
		got, err := computeLoftChordedAllow(underivableCapOffsetPairs(matched), vIdx, wIdx, verts, anchor, matched, 0, 2.0, false)
		require.ErrorIs(t, err, errLoftCapOffsetUnderivable, "S14 refuses the underivable cap plane offset")
		require.ErrorIs(t, err, ErrUnsupported, "the row's sentinel is ErrUnsupported, a derivation gap and not a shape rule")
		require.Equal(t, loftChordedAllow{}, got, "a refused build publishes no leg at all")
	})

	t.Run("a NaN cap1 vertex refuses instead of panicking", func(t *testing.T) {
		// A NaN coordinate has no exact rational, so ratSquaredDistance3
		// answers nil — which ratSqrtUp dereferences. The same refusal arm
		// covers it, so the reading never reaches that dereference.
		verts := []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
			r3.NewVec(math.NaN(), 0, 1), r3.NewVec(1, 1, 1.3),
		}
		require.Nil(t, ratSquaredDistance3(anchor.X, anchor.Y, anchor.Z, verts[2].X, verts[2].Y, verts[2].Z),
			"a NaN coordinate states no squared distance")

		var got loftChordedAllow
		var err error
		require.NotPanics(t, func() {
			got, err = computeLoftChordedAllow(underivableCapOffsetPairs(matched), vIdx, wIdx, verts, anchor, matched, 0, 2.0, false)
		}, "an unreadable cap1 coordinate is a refusal, never a nil dereference")
		require.ErrorIs(t, err, errLoftCapOffsetUnderivable, "S14 refuses the unreadable cap plane offset")
		require.ErrorIs(t, err, ErrUnsupported, "the row's sentinel is ErrUnsupported")
		require.Equal(t, loftChordedAllow{}, got, "a refused build publishes no leg at all")
	})
}

// TestComputeLoftChordedAllowTwistAreaSumsEveryChordedCell pins
// loftChordedAllow.twistAreaAllow (docs/tessellation-reach-design.md §4): the
// wall's HELD-TO-BILINEAR area leg, summed over exactly the chorded cells the
// other legs walk, through the same absSumUpper chain.
//
// Area's own bound never reads it — areaCorrection has already MOVED
// Area.Value onto the bilinear patches, so charging the gap there would count
// it twice — while the tessellation must, because the mesh holds the
// uncorrected held triangles. That makes this field's own arithmetic the only
// place the leg is checked, and the sum, not a maximum, is the correct shape:
// a mesh's area error accumulates over cells where a single point's departure
// does not.
//
// The gate is the same one every other per-cell leg uses, the cell's own
// proven chord-to-curve half, never a segment kind: the zero subtest drives
// matchedDelta to 0 on both cells with the geometry untouched, so a leg
// accumulated outside that gate would show up as a positive sum on a build
// that charges nothing else.
func TestComputeLoftChordedAllowTwistAreaSumsEveryChordedCell(t *testing.T) {
	t.Parallel()
	// A genuinely twisted quad: wHi sits off the plane of the other three
	// corners, so T = vLo - vHi - wLo + wHi is nonzero and each cell's own
	// held-to-bilinear gap is positive rather than the exact zero a planar
	// cell would give.
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 1, 1)
	wHi := r3.NewVec(1, 1, 1.3)
	verts := []r3.Vec{vLo, vHi, wLo, wHi}
	vIdx := [][]int{{0, 1}}
	wIdx := [][]int{{2, 3}}
	anchor := r3.NewVec(-1, -1, -1)
	// Each side's arc-length claim must stay at or above the chord it
	// subtends: |vHi-vLo| = 1 and |wHi-wLo| = sqrt(1.09) ~= 1.044.
	arcUpperV := []float64{1.01, 1.01}
	arcUpperW := []float64{1.1, 1.1}
	pairsWith := func(matched []float64) []loftLoopPair {
		return []loftLoopPair{{
			v: make([]Point2, 2), w: make([]Point2, 2),
			arcUpperV: arcUpperV, arcUpperW: arcUpperW,
			matchedDelta:   matched,
			tangentEnergyV: []float64{math.Inf(1), math.Inf(1)},
			tangentEnergyW: []float64{math.Inf(1), math.Inf(1)},
		}}
	}

	t.Run("sums both charged cells", func(t *testing.T) {
		chorded, err := computeLoftChordedAllow(pairsWith([]float64{0.5, 0.5}), vIdx, wIdx, verts, anchor, 0.5, 0, 2.0, false)
		require.NoError(t, err)

		// The loop walks cell 0 as (v0, v1, w0, w1) and cell 1 as the wrap
		// back, (v1, v0, w1, w0) — the same quad with its two ends swapped.
		cell0 := cellTwistAreaAllow(vLo, vHi, wLo, wHi)
		cell1 := cellTwistAreaAllow(vHi, vLo, wHi, wLo)
		require.Positive(t, cell0, "the fixture's own twist must make each cell's leg positive")
		require.Positive(t, cell1)
		require.Equal(t, absSumUpper(absSumUpper(0, cell0), cell1), chorded.twistAreaAllow,
			"twistAreaAllow is the per-cell sum through the same rounding chain, never a maximum")
		require.Greater(t, chorded.twistAreaAllow, cell0,
			"a two-cell build must publish more than either cell alone")
	})

	t.Run("charges nothing where no cell is chorded", func(t *testing.T) {
		chorded, err := computeLoftChordedAllow(pairsWith([]float64{0, 0}), vIdx, wIdx, verts, anchor, 0, 0, 2.0, false)
		require.NoError(t, err)
		require.Zero(t, chorded.twistAreaAllow,
			"a LineSeg-only build's held triangle pair IS its own boundary, so it charges no held-to-bilinear gap")
	})
}
