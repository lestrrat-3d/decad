package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// crossingHolePlateExtent extrudes the 20x20 plate from the XY plane by e and
// cuts the 4x4 through hole at x -4..0, y -2..2 with a box that overshoots
// both caps.
func crossingHolePlateExtent(t *testing.T, doc *decad.Document, e decad.Extent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-10, -10, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], e)
	require.NoError(t, err)
	part, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, -4, -2, 0, 2, -20, 40))
	require.NoError(t, err)
	require.False(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
	return part
}

// crossingDenotedVolume is the plate of the given denoted height less its
// 4x4 hole (384 mm^2 of material) less 13 mm^2 per millimetre of pocket depth
// inside the plate (crossingPocketTool).
func crossingDenotedVolume(height, depth *big.Rat) *big.Rat {
	v := new(big.Rat).Mul(height, big.NewRat(384, 1))
	return v.Sub(v, new(big.Rat).Mul(depth, big.NewRat(13, 1)))
}

func sumRat(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }

// TestCrossingBlindPocketDenotedCap cuts a pocket from the XY plane across
// the plate's through hole, ending at a plate cap that both bodies reach
// through a unit conversion. 0.2 in is not a float in millimetres, so both
// levels carry a displacement; they denote one rational, so the crossing
// route treats them as one level. 0.125 in converts exactly to the float
// 3.175 mm, so that pair is undisplaced and builds through the held levels
// alone.
func TestCrossingBlindPocketDenotedCap(t *testing.T) {
	t.Parallel()
	inch := denotedDepth(units.Inches(0.2))
	_, exact := inch.Float64()
	require.False(t, exact, "0.2 in converts with a rounding, so its level is displaced")
	_, exact = denotedDepth(units.Inches(0.125)).Float64()
	require.True(t, exact, "0.125 in converts exactly")
	require.Zero(t, denotedDepth(units.Inches(0.125)).Cmp(denotedDepth(units.Millimeters(3.175))))
	three := big.NewRat(3, 1)
	twoSided := func(along, against units.Value) decad.Extent {
		return decad.TwoSided{One: decad.DistanceSide{D: along}, Two: decad.DistanceSide{D: against}}
	}
	for _, tc := range []struct {
		name          string
		plate         decad.Extent
		depth         units.Value
		dir           decad.Direction
		height, inner *big.Rat
	}{
		{"two-sided-top", twoSided(units.Inches(0.2), units.Millimeters(3)), units.Inches(0.2), decad.Along,
			sumRat(three, inch), inch},
		{"two-sided-bottom", twoSided(units.Millimeters(3), units.Inches(0.2)), units.Inches(0.2), decad.Against,
			sumRat(three, inch), inch},
		{"symmetric-top", decad.Symmetric{D: units.Inches(0.4), FullLength: true}, units.Inches(0.2), decad.Along,
			sumRat(inch, inch), inch},
		{"symmetric-bottom", decad.Symmetric{D: units.Inches(0.4), FullLength: true}, units.Inches(0.2), decad.Against,
			sumRat(inch, inch), inch},
		{"inch-plate-millimetre-pocket", twoSided(units.Inches(0.125), units.Millimeters(3)), units.Millimeters(3.175),
			decad.Along, sumRat(three, denotedDepth(units.Millimeters(3.175))), denotedDepth(units.Millimeters(3.175))},
		{"millimetre-plate-inch-pocket", twoSided(units.Millimeters(3.175), units.Millimeters(3)), units.Inches(0.125),
			decad.Along, sumRat(three, denotedDepth(units.Millimeters(3.175))), denotedDepth(units.Millimeters(3.175))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := crossingHolePlateExtent(t, doc, tc.plate)
			part, err := decad.Cut(t.Context(), plate, crossingPocketTool(t, doc, 0, tc.depth, tc.dir))
			require.NoError(t, err)
			require.False(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
			requireEveryEdgeOnTwoFaces(t, part)
			requireRationalVolumeCovered(t, part, crossingDenotedVolume(tc.height, tc.inner))
			mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			requireWatertight(t, mesh)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}

// TestCrossingBlindPocketDenotedCapDeclines pairs 0.2 in with 5.08 mm. Both
// hold one float, but 0.2 in denotes fl(0.2)*fl(25.4), which is not 5.08 mm,
// so the pocket's end and the plate's cap are two planes a displacement
// apart and the crossing route does not build.
func TestCrossingBlindPocketDenotedCapDeclines(t *testing.T) {
	t.Parallel()
	held, err := units.Inches(0.2).In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 5.08, held, "both lengths hold one float")
	inch, mm := denotedDepth(units.Inches(0.2)), denotedDepth(units.Millimeters(5.08))
	require.NotZero(t, inch.Cmp(mm))
	three := big.NewRat(3, 1)
	for _, tc := range []struct {
		name          string
		plate, pocket units.Value
		height, inner *big.Rat
	}{
		{"inch-plate-millimetre-pocket", units.Inches(0.2), units.Millimeters(5.08), sumRat(three, inch), inch},
		{"millimetre-plate-inch-pocket", units.Millimeters(5.08), units.Inches(0.2), sumRat(three, mm), inch},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := crossingHolePlateExtent(t, doc,
				decad.TwoSided{One: decad.DistanceSide{D: tc.plate}, Two: decad.DistanceSide{D: units.Millimeters(3)}})
			part, err := decad.Cut(t.Context(), plate, crossingPocketTool(t, doc, 0, tc.pocket, decad.Along))
			require.NoError(t, err)
			require.True(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
			// 0.2 in denotes the lower of the two levels, so the pocket
			// removes 13 mm^2 per millimetre up to it in both orders.
			requireRationalVolumeCovered(t, part, crossingDenotedVolume(tc.height, tc.inner))
		})
	}
}
