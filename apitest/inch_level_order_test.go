package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// pocketToolFromTop extrudes a rectangle downward from the plane z=top by
// depth, so the tool's inner end sits at top-depth. An inch depth is a
// converted magnitude and carries a level displacement.
func pocketToolFromTop(t *testing.T, doc *decad.Document, x0, y0, x1, y1, top float64, depth units.Value) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), top)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: depth, Dir: decad.Against})
	require.NoError(t, err)
	return body
}

// requireIntendedNestedPockets cuts the outer then the inner pocket from a
// 20x20x10 plate and requires either a refusal or the denoted solid: the
// outer pocket 16x16 wide and outerDepth deep, the inner one inside it.
// outerMM is the denoted outer depth in millimetres; innerMM the inner one.
func requireIntendedNestedPockets(t *testing.T, outerDepth, innerDepth units.Value, outerMM, innerMM float64) {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 10)
	outer := pocketToolFromTop(t, doc, 2, 2, 18, 18, 10, outerDepth)
	inner := pocketToolFromTop(t, doc, 6, 6, 14, 14, 10, innerDepth)

	step, err := decad.Cut(t.Context(), plate, outer)
	if err != nil {
		t.Logf("first cut refused: %v", err)
		return
	}
	result, err := decad.Cut(t.Context(), step, inner)
	if err != nil {
		t.Logf("second cut refused: %v", err)
		return
	}
	// The denoted solid removes the union of the two pockets; the inner one
	// lies inside the outer footprint, so it adds only depth beyond outerMM.
	want := 20.0*20*10 - 16*16*outerMM
	wantFaces := 11
	if innerMM > outerMM {
		want -= 8 * 8 * (innerMM - outerMM)
		wantFaces = 16
	}
	volume, err := result.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), volume.Bound.Base()+1e-9)
	require.Len(t, result.Lumps(), 1)
	if !tookMeshPath(result) {
		require.Len(t, result.Faces(), wantFaces)
	}
	requireBodyWatertight(t, result)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestInchPocketLevelsCoincidingAfterRounding(t *testing.T) {
	t.Parallel()
	// 0.2 in and 5.08 mm denote the same depth; their held floats can
	// differ by one ulp.
	requireIntendedNestedPockets(t, units.Inches(0.2), units.Millimeters(5.08), 5.08, 5.08)
	requireIntendedNestedPockets(t, units.Millimeters(5.08), units.Inches(0.2), 5.08, 5.08)
	requireIntendedNestedPockets(t, units.Inches(0.2), units.Inches(0.2), 5.08, 5.08)
}

func TestInchPocketLevelsInvertedAfterRounding(t *testing.T) {
	t.Parallel()
	// 0.3 in = 7.62 mm: the inner pocket is denoted deeper than the outer
	// one by three ulps of the inch conversion, or shallower; the order
	// must follow the denotation or the call must refuse.
	for _, tc := range []struct {
		name         string
		outer, inner units.Value
		outerMM, mm  float64
	}{
		{"inner deeper by 1e-9", units.Inches(0.3), units.Millimeters(7.62 + 1e-9), 7.62, 7.62 + 1e-9},
		{"inner shallower by 1e-9", units.Inches(0.3), units.Millimeters(7.62 - 1e-9), 7.62, 7.62 - 1e-9},
		{"inch pair 0.3 vs 0.3+eps", units.Inches(0.3), units.Inches(0.3 + 1e-12), 7.62, 7.62 + 25.4e-12},
	} {
		t.Run(tc.name, func(t *testing.T) {
			requireIntendedNestedPockets(t, tc.outer, tc.inner, tc.outerMM, tc.mm)
		})
	}
}

// TestInchPocketLevelsWithinOneUlp nests a millimetre pocket against an inch
// pocket whose denoted depth fl(x)*Inch.Factor/Millimeter.Factor lies within an ulp of the
// millimetre float on either side. The floor levels 10-depth then round to
// the same or to inverted floats, so topology must follow the exact
// denotation, not the held floats; a refusal is also acceptable.
func TestInchPocketLevelsWithinOneUlp(t *testing.T) {
	t.Parallel()
	for _, x := range []float64{0.3, 0.2, 0.125, 0.7, 0.35} {
		// The denotation rescales the float magnitude by the float unit factors.
		denoted := new(big.Rat).SetFloat64(x)
		denoted.Mul(denoted, new(big.Rat).SetFloat64(units.Inch.Factor()))
		denoted.Quo(denoted, new(big.Rat).SetFloat64(units.Millimeter.Factor()))
		down, _ := denoted.Float64()
		up := math.Nextafter(down, math.Inf(1))
		if new(big.Rat).SetFloat64(down).Cmp(denoted) > 0 {
			down, up = math.Nextafter(down, math.Inf(-1)), down
		}
		for _, m := range []float64{down, up} {
			for _, inchFirst := range []bool{true, false} {
				cmp := new(big.Rat).SetFloat64(m).Cmp(denoted)
				t.Run("", func(t *testing.T) {
					outerV, innerV := units.Inches(x), units.Millimeters(m)
					if !inchFirst {
						outerV, innerV = innerV, outerV
					}
					requireNestedByDenotation(t, outerV, innerV, inchFirst, cmp)
				})
			}
		}
	}
}

// requireNestedByDenotation is requireIntendedNestedPockets with the depth
// order given as a sign: cmp is the sign of (millimetre depth - inch depth).
func requireNestedByDenotation(t *testing.T, outerDepth, innerDepth units.Value, inchOuter bool, cmp int) {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 10)
	outer := pocketToolFromTop(t, doc, 2, 2, 18, 18, 10, outerDepth)
	inner := pocketToolFromTop(t, doc, 6, 6, 14, 14, 10, innerDepth)
	step, err := decad.Cut(t.Context(), plate, outer)
	if err != nil {
		t.Logf("first cut refused: %v", err)
		return
	}
	result, err := decad.Cut(t.Context(), step, inner)
	if err != nil {
		t.Logf("second cut refused: %v", err)
		return
	}
	innerDeeper := (inchOuter && cmp > 0) || (!inchOuter && cmp < 0)
	wantFaces := 11
	if innerDeeper {
		wantFaces = 16
	}
	require.Len(t, result.Lumps(), 1)
	if tookMeshPath(result) {
		// The mesh path claims no denoted topology below its tolerance.
		return
	}
	require.Len(t, result.Faces(), wantFaces)
	requireBodyWatertight(t, result)
}

// TestInchPocketUnionLevelsFollowDenotation unions two pocketed plates whose
// pocket floors denote the same level, one through an inch depth and one
// through millimetres, and requires the denoted volume or a refusal.
func TestInchPocketUnionLevelsFollowDenotation(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plateA := boxBody(t, doc, 0, 0, 20, 20, 10)
	plateB := boxBody(t, doc, 10, 0, 30, 20, 10)
	toolA := pocketToolFromTop(t, doc, 2, 2, 8, 18, 10, units.Inches(0.2))
	toolB := pocketToolFromTop(t, doc, 22, 2, 28, 18, 10, units.Millimeters(5.08))
	a, err := decad.Cut(t.Context(), plateA, toolA)
	require.NoError(t, err)
	b, err := decad.Cut(t.Context(), plateB, toolB)
	require.NoError(t, err)
	result, err := decad.Union(t.Context(), a, b)
	if err != nil {
		t.Logf("union refused: %v", err)
		return
	}
	// Plates overlap on x 10..20, a 10x20x10 block; pockets lie outside it.
	want := (20.0*20*10)*2 - 10*20*10 - 2*(6*16*5.08)
	volume, err := result.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), volume.Bound.Base()+1e-9)
	require.Len(t, result.Lumps(), 1)
	requireBodyWatertight(t, result)
}
