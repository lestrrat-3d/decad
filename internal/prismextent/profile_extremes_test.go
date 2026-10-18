package prismextent_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// squareLoop is a counterclockwise line square [x0, x1] x [y0, y1].
func squareLoop(x0, y0, x1, y1 float64) sectionrecord.LoopRecord {
	corners := [4]sectionrecord.Point2{{U: x0, V: y0}, {U: x1, V: y0}, {U: x1, V: y1}, {U: x0, V: y1}}
	segs := make([]sectionrecord.CurveSegment, 4)
	for i := range segs {
		segs[i] = sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func requireExtremeContains(t *testing.T, held, bound float64, truth *big.Rat) {
	t.Helper()
	gap := new(big.Rat).Sub(proofarith.FloatRat(held), truth)
	gap.Abs(gap)
	require.LessOrEqual(t, gap.Cmp(proofarith.FloatRat(bound)), 0,
		"the published extreme %g ± %g misses %s", held, bound, truth.RatString())
}

func TestProfileBoundaryExtremesChargeComputedEndpoints(t *testing.T) {
	t.Parallel()
	cosEighth := proofbound.MustRatDecimal("0.923879532511286756128183189396788286822416625863642486115097")
	sinEighth := proofbound.MustRatDecimal("0.382683432365089771728459984030398866761344562485627041433800")
	arc := sectionrecord.ArcSeg{
		Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{V: 1}, TStart: .25, TEnd: .75,
	}
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: .0625, TEnd: .1875}
	for _, tc := range []struct {
		name string
		seg  sectionrecord.CurveSegment
	}{{"arc", arc}, {"circle", circle}} {
		t.Run(tc.name, func(t *testing.T) {
			profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{
				Segments: []sectionrecord.CurveSegment{tc.seg},
			}}
			lo, hi, bound, err := prismextent.ProfileBoundaryExtremes(
				t.Context(), profile, 1, 0, freeform.NewFreeformWork(), nil)
			require.NoError(t, err)
			require.Positive(t, bound)
			requireExtremeContains(t, hi, bound, cosEighth)
			requireExtremeContains(t, lo, bound, sinEighth)
		})
	}

	line := sectionrecord.LineSeg{
		Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: .1}, TStart: .3, TEnd: 1,
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{line},
	}}
	lo, _, bound, err := prismextent.ProfileBoundaryExtremes(
		t.Context(), profile, 1, 0, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	require.Positive(t, bound)
	truth := new(big.Rat).Mul(proofarith.FloatRat(.1), proofarith.FloatRat(.3))
	requireExtremeContains(t, lo, bound, truth)
}

func TestProfileBoundaryExtremesReadsHoles(t *testing.T) {
	// The hole lies outside the outer square on purpose: the function does
	// not validate the profile, so only a reader that visits the hole loops
	// reaches u = 4.
	profile := momentinput.Profile{
		Outer: squareLoop(0, 0, 2, 2),
		Holes: []sectionrecord.LoopRecord{squareLoop(3, 3, 4, 4)},
	}
	lo, hi, bound, err := prismextent.ProfileBoundaryExtremes(t.Context(), profile, 1, 0, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, 0.0, lo)
	require.Equal(t, 4.0, hi)
	require.Equal(t, 0.0, bound)

	nanLo, nanHi, nanBound, err := prismextent.ProfileBoundaryExtremes(t.Context(), profile, math.NaN(), 0, freeform.NewFreeformWork(), nil)
	require.Zero(t, nanLo+nanHi+nanBound)
	require.Error(t, err)

	other := momentinput.Profile{Outer: squareLoop(0, 0, 1, 1)}
	walks := &momentinput.ProfileWalks{Profile: other}
	misLo, misHi, misBound, err := prismextent.ProfileBoundaryExtremes(t.Context(), profile, 1, 0, freeform.NewFreeformWork(), walks)
	require.Zero(t, misLo+misHi+misBound)
	require.ErrorIs(t, err, momentinput.ErrResolvedWalksMismatch)
}
