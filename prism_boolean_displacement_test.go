package decad_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §7's own test obligations (§15's
// exactness rows): the merged section's displacement from the section the two
// operands jointly denote, and its reach into every measurement evalPrism
// publishes. §4.1's re-expression is where that displacement is committed, so
// each case here places operand B and reads the result against the union its
// two operands' OWN recorded coordinates denote, computed over exact rationals.

// ratOf lifts a float64 to the exact rational it is.
func ratOf(f float64) *big.Rat {
	r := new(big.Rat)
	require := r.SetFloat64(f)
	if require == nil {
		panic("non-finite coordinate in a test fixture")
	}
	return r
}

// trueBoxUnionVolume is the EXACT volume of the union of two axis-aligned
// boxes over one plane, both swept h, computed over the corner coordinates
// taken exactly — the answer the operands denote, whatever the evaluator's own
// arithmetic rounds to.
func trueBoxUnionVolume(a, b [4]float64, h float64) *big.Rat {
	span := func(lo, hi float64) *big.Rat { return new(big.Rat).Sub(ratOf(hi), ratOf(lo)) }
	overlap := func(alo, ahi, blo, bhi float64) *big.Rat {
		lo, hi := ratOf(alo), ratOf(ahi)
		if ratOf(blo).Cmp(lo) > 0 {
			lo = ratOf(blo)
		}
		if ratOf(bhi).Cmp(hi) < 0 {
			hi = ratOf(bhi)
		}
		d := new(big.Rat).Sub(hi, lo)
		if d.Sign() < 0 {
			return new(big.Rat)
		}
		return d
	}
	areaA := new(big.Rat).Mul(span(a[0], a[2]), span(a[1], a[3]))
	areaB := new(big.Rat).Mul(span(b[0], b[2]), span(b[1], b[3]))
	shared := new(big.Rat).Mul(
		overlap(a[0], a[2], b[0], b[2]),
		overlap(a[1], a[3], b[1], b[3]),
	)
	area := new(big.Rat).Sub(new(big.Rat).Add(areaA, areaB), shared)
	return new(big.Rat).Mul(area, ratOf(h))
}

// ratFloat is the rational read as the float64 nearest it, for a difference
// against a reported value.
func ratFloat(r *big.Rat) float64 {
	f, _ := r.Float64()
	return f
}

// placedFar translates b by shift along +X, keeping the receiver retired
// exactly as Placed always does.
func placedFar(t *testing.T, b *decad.Body, shift float64) *decad.Body {
	t.Helper()
	m, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	out, err := b.Placed(m)
	require.NoError(t, err)
	return out
}

// TestPrismUnionFarPlacementReturnsTheTrueUnion pins the two reproductions the
// PR 111 audit measured. Two boxes over [0,10]² and [5,15]², each swept 10 mm
// and placed by the SAME pure translation, have a union of exactly 1750 mm³
// whatever that translation is: the two operands do not move relative to each
// other, so the union's own shape cannot depend on where the pair sits.
//
// Re-expressing operand B through world space rather than through the composed
// relative map lost that: at a translation of 1e16 one ulp is 2 mm, B's [5,15]
// re-expressed as [4,16], and the reported solid was 1900 mm³ and a millimetre
// longer than operand B itself; at 2^55 the same pair reported 1700 mm³,
// dropping material genuinely inside B.
func TestPrismUnionFarPlacementReturnsTheTrueUnion(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shift float64
	}{
		{name: "origin", shift: 0},
		{name: "1e16 — the 1900 mm³ reproduction", shift: 1e16},
		{name: "2^55 — the 1700 mm³ reproduction", shift: 36028797018963968},
		{name: "2^56", shift: 72057594037927936},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			a := placedFar(t, boxBody(t, doc, 0, 0, 10, 10, 10), tc.shift)
			b := placedFar(t, boxBody(t, doc, 5, 5, 15, 15, 10), tc.shift)
			bBox, err := b.Bounds()
			require.NoError(t, err)

			got, err := decad.Union(a, b)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

			vol, err := got.Volume()
			require.NoError(t, err)
			truth := trueBoxUnionVolume([4]float64{0, 0, 10, 10}, [4]float64{5, 5, 15, 15}, 10)
			residual := math.Abs(volumeMM(t, vol) - ratFloat(truth))
			require.Equal(t, 1750.0, volumeMM(t, vol))
			require.LessOrEqual(t, residual, vol.Bound.Base())

			// Both operands share one frame and one placement, so §7's decidable
			// zero case holds: B's fields are copied verbatim and the merged
			// section carries no displacement at all.
			require.Equal(t, decad.Exact, vol.Exactness)
			require.Equal(t, units.CubicMillimeters(0), vol.Bound)

			// The union of two solids reaches exactly as far as the further of
			// them: a box running past operand B's own is a wrong SOLID, not a
			// wrong measurement.
			box, err := got.Bounds()
			require.NoError(t, err)
			require.Equal(t, bBox.Max.X, box.Max.X)
			require.Equal(t, decad.Exact, box.Exactness)
		})
	}
}

// TestPrismUnionReExpressionDisplacementBoundEnclosesTheError is §7's
// Approximate arm. Operand B is drawn far from the origin and placed back over
// operand A, so its re-expression into A's frame is a genuine rigid-transform
// recomputation at that magnitude rather than a copy: the merged section then
// carries a displacement, and every measurement of it must carry a bound that
// encloses the difference from the section the two operands denote.
//
// The bound must also SCALE with the placement magnitude — the rounding is an
// ulp at that magnitude — which is what fails if a later change drops the
// displacement term and leaves only the arithmetic bounds behind.
func TestPrismUnionReExpressionDisplacementBoundEnclosesTheError(t *testing.T) {
	shifts := []float64{1e3, 1e6, 1e9, 1e12}
	bounds := make([]float64, len(shifts))
	for i, shift := range shifts {
		t.Run(units.Millimeters(shift).String(), func(t *testing.T) {
			doc := decad.New()
			a := boxBody(t, doc, 0, 0, 10, 10, 10)
			// B's own record sits at the far magnitude along u; its placement
			// brings it back over A, so the pair overlaps while the
			// re-expression runs at the far magnitude's own ulp.
			lo, hi := 5-shift, 15-shift
			b := placedFar(t, boxBody(t, doc, lo, 5, hi, 15, 10), shift)

			got, err := decad.Union(a, b)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

			vol, err := got.Volume()
			require.NoError(t, err)
			// The section B denotes in A's frame is its own recorded corners
			// carried by the exact translation — never the 5 and 15 the fixture
			// wrote them from.
			truth := trueBoxUnionVolume(
				[4]float64{0, 0, 10, 10},
				[4]float64{lo + shift, 5, hi + shift, 15},
				10,
			)
			residual := math.Abs(volumeMM(t, vol) - ratFloat(truth))
			require.Positive(t, vol.Bound.Base())
			require.Equal(t, decad.Approximate, vol.Exactness)
			require.LessOrEqual(t, residual, vol.Bound.Base(),
				"the reported bound must enclose the re-expression's own error")

			// Every other reading of the same section carries it too (§7).
			area, err := got.Area()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, area.Exactness)
			require.Positive(t, area.Bound.Base())
			centroid, err := got.Centroid()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, centroid.Exactness)
			require.Positive(t, centroid.Bound.Base())
			box, err := got.Bounds()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, box.Exactness)
			require.Positive(t, box.Bound.Base())

			bounds[i] = vol.Bound.Base()
		})
	}
	for i := 1; i < len(bounds); i++ {
		require.Greater(t, bounds[i], bounds[i-1],
			"the displacement is an ulp at the placement magnitude, so its bound grows with it")
	}
}

// TestPrismUnionRoundedReExpressionStaysWithinItsBound is the same arm on a
// section whose corners are not representable at the placement magnitude, so
// none of the fixture's own arithmetic cancels and the re-expression is the
// whole of what puts the merged section where it is. The reported volume must
// still sit inside its own bound of the union the two operands denote, and that
// bound must be the DISPLACEMENT's — orders above anything the section's own
// 10 mm-scale arithmetic can produce.
func TestPrismUnionRoundedReExpressionStaysWithinItsBound(t *testing.T) {
	const shift = 1e12
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	lo, hi := 5.1-shift, 15.1-shift
	b := placedFar(t, boxBody(t, doc, lo, 5.1, hi, 15.1, 10), shift)

	got, err := decad.Union(a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

	vol, err := got.Volume()
	require.NoError(t, err)
	truth := trueBoxUnionVolume(
		[4]float64{0, 0, 10, 10},
		[4]float64{lo + shift, 5.1, hi + shift, 15.1},
		10,
	)
	residual := math.Abs(volumeMM(t, vol) - ratFloat(truth))
	require.LessOrEqual(t, residual, vol.Bound.Base())
	require.Equal(t, decad.Approximate, vol.Exactness)
	// An ordinary prism of this size reports a bound near 1e-13 mm³: a bound
	// this large exists only because the section displacement is in it.
	require.Greater(t, vol.Bound.Base(), 1e-3)
}

// TestPrismUnionDisplacedSectionRefusesTheSectionRewrites pins what a payload
// carrying a section displacement is NOT allowed to do quietly: a modify op
// rewrites the recorded section (modify §2) and has no proven displacement of
// its own to publish, so it refuses rather than returning a body whose bounds
// speak only for the record.
func TestPrismUnionDisplacedSectionRefusesTheSectionRewrites(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	const shift = 1e9
	b := placedFar(t, boxBody(t, doc, 5-shift, 5, 15-shift, 15, 10), shift)
	got, err := decad.Union(a, b)
	require.NoError(t, err)

	_, err = got.Fillet(verticalConvexEdge(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "proven displacement")
}

// TestPrismUnionDisplacedSectionDownstreamReadings pins what the rest of the
// evaluator does with a payload whose section carries a displacement: the mesh
// carries it in its own proven bound, and the wall survey — which answers with
// a bare reading and no bound to widen — leaves the question undecided rather
// than measuring the recorded section as though it were the denoted one.
func TestPrismUnionDisplacedSectionDownstreamReadings(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	const shift = 1e9
	b := placedFar(t, boxBody(t, doc, 5-shift, 5, 15-shift, 15, 10), shift)
	got, err := decad.Union(a, b)
	require.NoError(t, err)
	box, err := got.Bounds()
	require.NoError(t, err)
	displacement := box.Bound.Base()
	require.Positive(t, displacement)

	mesh, err := got.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	require.GreaterOrEqual(t, mesh.Bound().Base(), displacement,
		"the mesh vertices sit on the recorded section, so its displacement is in the mesh's own bound")

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
}
