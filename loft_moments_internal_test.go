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

// TestLoftMassAccumulatorBoundsEmpty proves bounds reports false before any
// triangle has been added.
func TestLoftMassAccumulatorBoundsEmpty(t *testing.T) {
	m := newLoftMassAccumulator(r3.NewVec(0, 0, 0))
	_, ok := m.bounds()
	require.False(t, ok)
}
