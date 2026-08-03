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
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
	addBoxFaces(m, 2, 2, 2)

	vol := m.volume()
	require.Equal(t, 8.0, vol.Value.Base())
	require.Equal(t, Exact, vol.Exactness)
	require.Equal(t, 0.0, vol.Bound.Base())

	c, err := m.centroid()
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
	m := newLoftMassAccumulator(r3.NewVec(-4, 8, 0.5))
	addBoxFaces(m, 2, 2, 2)

	vol := m.volume()
	require.Equal(t, 8.0, vol.Value.Base())
	require.Equal(t, Exact, vol.Exactness)

	c, err := m.centroid()
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
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
	m.add(r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1), false)

	vol := m.volume()
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
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
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

	c, err := m.centroid()
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
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
	m.add(r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1), false)
	m.add(r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), false) // reversed winding

	_, err := m.centroid()
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestLoftMassAccumulatorAreaApproximateBoundedByReference builds two wall
// triangles whose exact areas are irrational (sqrt(2)/2 and sqrt(13)) plus a
// cap contribution of the non-representable exact rational 5/3, then checks
// the reported bound against a 256-bit high-precision reference sum — never
// merely asserting the bound is present (docs/loft-design.md §8, required
// test).
func TestLoftMassAccumulatorAreaApproximateBoundedByReference(t *testing.T) {
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
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
	q := xdot(w, w)
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
			m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
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
	dir := r3.NewVec(0.6, 0.8, 0)
	perp := r3.NewVec(-0.48, 0.36, 0.8)

	const prec = 400
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
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

// TestLoftMassAccumulatorBoundsEmpty proves bounds reports false before any
// triangle has been added.
func TestLoftMassAccumulatorBoundsEmpty(t *testing.T) {
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
	_, ok := m.bounds()
	require.False(t, ok)
}
