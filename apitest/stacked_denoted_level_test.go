package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// denotedTop is the plate top the pocket tools start from. The pocket floors
// sit at denotedTop minus a depth, which is not a float for the depths below,
// so every floor carries the rounding of that difference as a displacement.
const denotedTop = 12.0

// pocketBelow extrudes a rectangle drawn on the plane z=top by e.
func pocketBelow(t *testing.T, doc *decad.Document, x0, y0, x1, y1 float64, e decad.Extent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), denotedTop)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], e)
	require.NoError(t, err)
	return body
}

// down is a Distance extent from the plate top into the plate.
func down(depth units.Value) decad.Extent {
	return decad.Distance{D: depth, Dir: decad.Against}
}

// denotedDepth is the depth in millimetres a length denotes: its float
// magnitude rescaled exactly by the float unit factors.
func denotedDepth(v units.Value) *big.Rat {
	d := new(big.Rat).SetFloat64(v.Mag())
	d.Mul(d, new(big.Rat).SetFloat64(v.Unit().Factor()))
	return d.Quo(d, new(big.Rat).SetFloat64(units.Millimeter.Factor()))
}

// pocketedVolume is the 20x20x12 plate less area mm² of pocket per depth.
func pocketedVolume(area int64, depths ...*big.Rat) *big.Rat {
	removed := new(big.Rat)
	for _, d := range depths {
		removed.Add(removed, d)
	}
	removed.Mul(removed, big.NewRat(area, 1))
	return removed.Sub(big.NewRat(20*20*12, 1), removed)
}

// requireRationalVolumeCovered requires the published volume's bound to cover
// the exact rational volume want.
func requireRationalVolumeCovered(t *testing.T, b *decad.Body, want *big.Rat) decad.Measurement {
	t.Helper()
	volume, err := b.Volume()
	require.NoError(t, err)
	residual := new(big.Rat).Sub(new(big.Rat).SetFloat64(volumeMM(t, volume)), want)
	residual.Abs(residual)
	bound := new(big.Rat).SetFloat64(boundMM3(t, volume))
	require.LessOrEqual(t, residual.Cmp(bound), 0, "the volume bound covers the denoted volume")
	return volume
}

// requireDenotedPocketPair cuts two pockets from a 20x20x12 plate in order and
// requires the analytic stacked result with the given face count and volume.
func requireDenotedPocketPair(t *testing.T, plate, first, second *decad.Body, faces int, want *big.Rat) {
	t.Helper()
	part, err := decad.Cut(t.Context(), plate, first)
	require.NoError(t, err)
	require.False(t, tookMeshPath(part))
	firstVolume, err := part.Volume()
	require.NoError(t, err)
	require.Positive(t, boundMM3(t, firstVolume), "the first floor carries a level displacement")
	part, err = decad.Cut(t.Context(), part, second)
	require.NoError(t, err)
	require.False(t, tookMeshPath(part), "equal denoted levels share one interface")
	require.Len(t, part.Lumps(), 1)
	require.Len(t, part.Faces(), faces)
	volume := requireRationalVolumeCovered(t, part, want)
	require.Equal(t, decad.Approximate, volume.Exactness)
	mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
	report, err := part.Document().Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestDenotedLevelFixtures pins what the tests below depend on: 0.125 in
// and 3.175 mm denote one depth whose floor below z=12 is not a float; 0.2 in
// converts with a rounding, and half of 0.4 in denotes 0.2 in.
func TestDenotedLevelFixtures(t *testing.T) {
	t.Parallel()
	inch, mm := denotedDepth(units.Inches(0.125)), denotedDepth(units.Millimeters(3.175))
	require.Zero(t, inch.Cmp(mm))
	floor := new(big.Rat).Sub(big.NewRat(int64(denotedTop), 1), mm)
	_, exact := floor.Float64()
	require.False(t, exact, "the floor level rounds, so the stack displaces it")
	_, exact = denotedDepth(units.Inches(0.2)).Float64()
	require.False(t, exact, "0.2 in converts with a rounding")
	half := new(big.Rat).Quo(denotedDepth(units.Inches(0.4)), big.NewRat(2, 1))
	require.Zero(t, half.Cmp(denotedDepth(units.Inches(0.2))))
}

func TestDenotedLevelPocketsShareOneInterface(t *testing.T) {
	t.Parallel()
	exactDepth := denotedDepth(units.Millimeters(3.175))
	inchDepth := denotedDepth(units.Inches(0.2))
	for _, tc := range []struct {
		name          string
		first, second decad.Extent
		depth         *big.Rat
	}{
		{"millimetre first", down(units.Millimeters(3.175)), down(units.Inches(0.125)), exactDepth},
		{"inch first", down(units.Inches(0.125)), down(units.Millimeters(3.175)), exactDepth},
		{"converted inch twice", down(units.Inches(0.2)), down(units.Inches(0.2)), inchDepth},
		{"symmetric then distance", decad.Symmetric{D: units.Inches(0.4), FullLength: true}, down(units.Inches(0.2)), inchDepth},
		{"distance then two-sided", down(units.Inches(0.2)),
			decad.TwoSided{One: decad.DistanceSide{D: units.Millimeters(3)}, Two: decad.DistanceSide{D: units.Inches(0.2)}}, inchDepth},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 20, 20, denotedTop)
			// Two 6x16 pockets beside each other leave one floor level with
			// two exposed floors.
			first := pocketBelow(t, doc, 2, 2, 8, 18, tc.first)
			second := pocketBelow(t, doc, 12, 2, 18, 18, tc.second)
			requireDenotedPocketPair(t, plate, first, second, 16, pocketedVolume(2*6*16, tc.depth))
		})
	}
}

func TestDenotedLevelNestedPocketEndsOnTheFloor(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name         string
		outer, inner units.Value
	}{
		{"millimetre outer", units.Millimeters(3.175), units.Inches(0.125)},
		{"inch outer", units.Inches(0.125), units.Millimeters(3.175)},
		{"converted inch twice", units.Inches(0.2), units.Inches(0.2)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 20, 20, denotedTop)
			// The inner pocket lies inside the outer one and ends on its
			// floor, so the solid is the outer pocket alone.
			outer := pocketBelow(t, doc, 2, 2, 18, 18, down(tc.outer))
			inner := pocketBelow(t, doc, 6, 6, 14, 14, down(tc.inner))
			requireDenotedPocketPair(t, plate, outer, inner, 11, pocketedVolume(16*16, denotedDepth(tc.outer)))
		})
	}
}

// TestDenotedLevelEqualFloatsDifferentRationalsDecline cuts a 0.2 in pocket
// beside a 5.08 mm one. Both depths hold the same float, but 0.2 in denotes
// fl(0.2)*fl(25.4), which is not 5.08 mm, so the floors are distinct planes
// and the analytic stack declines the shared interface.
func TestDenotedLevelEqualFloatsDifferentRationalsDecline(t *testing.T) {
	t.Parallel()
	held, err := units.Inches(0.2).In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 5.08, held, "both depths hold one float")
	require.NotZero(t, denotedDepth(units.Inches(0.2)).Cmp(denotedDepth(units.Millimeters(5.08))))
	for _, tc := range []struct {
		name          string
		first, second units.Value
	}{
		{"millimetre first", units.Millimeters(5.08), units.Inches(0.2)},
		{"inch first", units.Inches(0.2), units.Millimeters(5.08)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 20, 20, denotedTop)
			first := pocketBelow(t, doc, 2, 2, 8, 18, down(tc.first))
			second := pocketBelow(t, doc, 12, 2, 18, 18, down(tc.second))
			part, err := decad.Cut(t.Context(), plate, first)
			require.NoError(t, err)
			require.False(t, tookMeshPath(part))
			part, err = decad.Cut(t.Context(), part, second)
			require.NoError(t, err)
			require.True(t, tookMeshPath(part), "distinct denoted floors never share one interface")
			requireRationalVolumeCovered(t, part, pocketedVolume(6*16,
				denotedDepth(units.Inches(0.2)), denotedDepth(units.Millimeters(5.08))))
		})
	}
}

// TestDenotedLevelSplitInterfaceIsReused splits a stack at a converted level
// and then ends a third pocket at that same denoted level.
func TestDenotedLevelSplitInterfaceIsReused(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, denotedTop)
	deep := pocketBelow(t, doc, 2, 2, 6, 18, down(units.Millimeters(5)))
	part, err := decad.Cut(t.Context(), plate, deep)
	require.NoError(t, err)
	first := pocketBelow(t, doc, 8, 2, 12, 18, down(units.Inches(0.2)))
	second := pocketBelow(t, doc, 14, 2, 18, 18, down(units.Inches(0.2)))
	inch := denotedDepth(units.Inches(0.2))
	requireDenotedPocketPair(t, part, first, second, 21, pocketedVolume(4*16, big.NewRat(5, 1), inch, inch))
}
