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

// exactResidual is |reported − truth| taken entirely over rationals and rounded
// UP into a float64. Differencing against ratFloat instead would fold half an
// ulp of the reported magnitude into the answer — at 1750 mm³ that is 1.1e-13,
// the very scale the bounds under test live at — so a bound that failed to
// contain its error could be flattered into passing.
func exactResidual(reported float64, truth *big.Rat) float64 {
	d := new(big.Rat).Sub(ratOf(reported), truth)
	d.Abs(d)
	f, exact := d.Float64()
	if !exact {
		f = math.Nextafter(f, math.Inf(1))
	}
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

			// Both operands share one frame and one placement, so the
			// re-expression contributes nothing. The merge still SPLITS four
			// walls, and each fragment's endpoints are named by a cut parameter
			// sketch rounded for this pair, so §7's δ_cut stands: the reading is
			// Approximate, and its bound is the cut displacement's alone —
			// far below anything a placement could add.
			require.Equal(t, decad.Approximate, vol.Exactness)
			require.Positive(t, vol.Bound.Base())
			require.Less(t, vol.Bound.Base(), 1e-9)

			// The union of two solids reaches exactly as far as the further of
			// them: a box running past operand B's own is a wrong SOLID, not a
			// wrong measurement.
			box, err := got.Bounds()
			require.NoError(t, err)
			require.Equal(t, bBox.Max.X, box.Max.X)
			require.Equal(t, decad.Approximate, box.Exactness)
		})
	}
}

// TestPrismUnionCutDisplacementBoundEnclosesTheError is §7's δ_cut arm, on the
// fixture that found it: two boxes over [0,10]² and [5.1,15.1]², each swept
// 10 mm, drawn on one plane with no placement at all. The re-expression is the
// identity and neither operand carries a displacement, so every term §7 charged
// before δ_cut is zero — and the merge still splits four walls, whose surviving
// fragments name their endpoints by parameters sketch rounded for this pair.
//
// The corner at u = 5.1 is where it shows. sketch's cut parameter on the
// 10 mm wall reads 0.51000000000000000888, whose exact lerp lands at
// 5.1000000000000000888 while the operand's own corner is
// 5.0999999999999996447 — a walk that does not close, 4.4e-16 out. Over the
// whole octagon that displaces the region area by 2.2e-15 mm², and the volume
// by 2.2e-14 mm³, against a moments-engine bound of 1.0e-13 mm³ that speaks
// only for its own arithmetic. Charging nothing for the cut published
// 1.0e-13 mm³ for a true error of 1.26e-13 mm³ — a bound 17% SHORT of the
// error it claimed to cover, which is the one thing a bound may never be.
//
// The truth here is computed over math/big.Rat from the operands' own float
// coordinates, so it is the volume the two records denote and not a second
// float answer.
func TestPrismUnionCutDisplacementBoundEnclosesTheError(t *testing.T) {
	// The sweep height only scales the reading, so containment must hold at
	// every one of them: 10 mm is the investigation's own fixture, 25 mm proves
	// the charge is not a constant tuned to it.
	for _, h := range []float64{10, 25} {
		t.Run(units.Millimeters(h).String(), func(t *testing.T) {
			doc := decad.New()
			a := boxBody(t, doc, 0, 0, 10, 10, h)
			b := boxBody(t, doc, 5.1, 5.1, 15.1, 15.1, h)

			got, err := decad.Union(a, b)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

			vol, err := got.Volume()
			require.NoError(t, err)
			truth := trueBoxUnionVolume(
				[4]float64{0, 0, 10, 10},
				[4]float64{5.1, 5.1, 15.1, 15.1},
				h,
			)
			residual := exactResidual(volumeMM(t, vol), truth)
			require.Positive(t, residual, "the fixture is chosen so the reported value is NOT the denoted one")
			require.Equal(t, decad.Approximate, vol.Exactness)
			require.LessOrEqualf(t, residual, vol.Bound.Base(),
				"the published volume bound %g mm³ must contain the true error %g mm³",
				vol.Bound.Base(), residual)

			// The bound is still a per-mechanism one and not a blanket
			// widening: an ordinary prism of this size reports an arithmetic
			// bound near 1e-13 mm³, and the cut displacement is charged at the
			// section's own 10 mm scale.
			require.Less(t, vol.Bound.Base(), 1e-9)

			// Every other reading of the same section carries the displacement.
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
		})
	}
}

// TestPrismUnionWholeEdgeMergeStaysExact is the other side of the same arm: the
// zero case §7 keeps. Operand B sits strictly inside operand A, so the
// arrangement splits nothing, every survivor is a whole edge carrying the
// entity's own recorded endpoints, and no cut parameter exists to charge.
func TestPrismUnionWholeEdgeMergeStaysExact(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 2, 2, 8, 8, 10)

	got, err := decad.Union(a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 1000.0, volumeMM(t, vol))
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, units.CubicMillimeters(0), vol.Bound)
}

// TestPrismUnionReExpressionDisplacementBoundEnclosesTheError is §7's
// Approximate arm. Operand B is drawn far from the origin and placed exactly
// over operand A, so its re-expression into A's frame is a genuine
// rigid-transform recomputation at that magnitude rather than a copy. B stays
// strictly inside A, so the arrangement leaves every boundary whole and the
// merged section carries a displacement every measurement must report.
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
			// brings it strictly inside A, so the pair has no split boundary
			// while the re-expression runs at the far magnitude's own ulp.
			lo, hi := 2-shift, 8-shift
			b := placedFar(t, boxBody(t, doc, lo, 2, hi, 8, 10), shift)

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
				[4]float64{lo + shift, 2, hi + shift, 8},
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

// TestPrismUnionRoundedReExpressionStaysWithinItsBound pins the same arm at a
// large placement magnitude. B stays strictly inside A, so no split boundary
// hides the re-expression behind the mesh path. The reported volume must still
// sit inside its own bound of the union the two operands denote, and that bound
// must be the DISPLACEMENT's — orders above anything the section's own
// 10 mm-scale arithmetic can produce.
func TestPrismUnionRoundedReExpressionStaysWithinItsBound(t *testing.T) {
	const shift = 1e12
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	lo, hi := 2-shift, 8-shift
	b := placedFar(t, boxBody(t, doc, lo, 2, hi, 8, 10), shift)

	got, err := decad.Union(a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

	vol, err := got.Volume()
	require.NoError(t, err)
	truth := trueBoxUnionVolume(
		[4]float64{0, 0, 10, 10},
		[4]float64{lo + shift, 2, hi + shift, 8},
		10,
	)
	residual := math.Abs(volumeMM(t, vol) - ratFloat(truth))
	require.LessOrEqual(t, residual, vol.Bound.Base())
	require.Equal(t, decad.Approximate, vol.Exactness)
	// An ordinary prism of this size reports a bound near 1e-13 mm³: a bound
	// this large exists only because the section displacement is in it.
	require.Greater(t, vol.Bound.Base(), 1e-3)
}

// TestPrismUnionChainedSecondOperandReexpressionAccumulatesDisplacement
// exercises the public chained-Union path where the approximate first result
// is operand B of a second nonidentity union. B's existing displacement passes
// unchanged through the rigid map and the new coordinate rounding adds to it.
func TestPrismUnionChainedSecondOperandReexpressionAccumulatesDisplacement(t *testing.T) {
	doc := decad.New()
	const firstShift = 1e8
	inside := boxBody(t, doc, 2, 2, 8, 8, 10)
	outer := placedFar(t, boxBody(t, doc, -firstShift, 0, 10-firstShift, 10, 10), firstShift)
	first, err := decad.Union(inside, outer)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(first), "the analytic reduction must own the first union")
	firstVolume, err := first.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, firstVolume.Exactness)
	require.Positive(t, firstVolume.Bound.Base())

	const secondShift = 3e8
	insideAgain := placedFar(t, boxBody(t, doc, 2-secondShift, 2, 8-secondShift, 8, 10), secondShift)
	chained, err := decad.Union(insideAgain, first)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(chained), "the analytic reduction must own the chained union")
	volume, err := chained.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.Greater(t, volume.Bound.Base(), firstVolume.Bound.Base(),
		"B's existing displacement and its new re-expression allowance must both reach the result")

	_, err = chained.Fillet(decad.Edges().AtLeast(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
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
	b := placedFar(t, boxBody(t, doc, 2-shift, 2, 8-shift, 8, 10), shift)
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
	b := placedFar(t, boxBody(t, doc, 2-shift, 2, 8-shift, 8, 10), shift)
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
