package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/sheet-metal-design.md's public-surface tests for SheetMetal.

// sheetArgs are the constructor arguments of SheetMetal, with the optional K-factor.
type sheetArgs struct {
	thickness, innerRadius, width units.Value
	legs, bends                   []units.Value
	k                             *units.Value
}

// sheetArgsOf is the strip of 2 mm sheet, inside radius 3 mm and width 10 mm over the given legs (mm) and bends
// (degrees).
func sheetArgsOf(legs []float64, bends []float64) sheetArgs {
	a := sheetArgs{
		thickness:   units.Millimeters(2),
		innerRadius: units.Millimeters(3),
		width:       units.Millimeters(10),
	}
	for _, l := range legs {
		a.legs = append(a.legs, units.Millimeters(l))
	}
	for _, b := range bends {
		a.bends = append(a.bends, units.Degrees(b))
	}
	return a
}

// command is the SheetMetal command of the arguments, with KFactor called when k is set.
func (a sheetArgs) command(doc *decad.Document) *decad.SheetMetalCommand {
	c := decad.SheetMetal(doc, a.thickness, a.innerRadius, a.width, a.legs, a.bends)
	if a.k != nil {
		c.KFactor(*a.k)
	}
	return c
}

func sheetCommand(doc *decad.Document, legs []float64, bends []float64) *decad.SheetMetalCommand {
	return sheetArgsOf(legs, bends).command(doc)
}

// stripVolume is thickness · width · the length of the centreline: the legs and each bend's arc at the
// centreline radius InnerRadius + Thickness/2 = 4.
func stripVolume(legs []float64, bends []float64) float64 {
	length := 0.0
	for _, l := range legs {
		length += l
	}
	for _, b := range bends {
		length += math.Abs(b) * math.Pi / 180 * 4
	}
	return 2 * 10 * length
}

func TestSheetMetalBuildsAnLBracket(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	legs, bends := []float64{30, 20}, []float64{90}
	body, report, err := sheetCommand(doc, legs, bends).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-stripVolume(legs, bends)), volume.Bound.Base()+1e-9)
	box, err := body.Bounds()
	require.NoError(t, err)
	// The first leg runs along +X from the origin, the bend turns it up +Y: the outside of the bend reaches
	// x = 30 + 4 + 1 and the leg ends at y = 4 + 20, with the strip 1 mm either side of its centreline.
	require.True(t, box.Min.Equal(r3.NewVec(0, -1, 0), 1e-9), "min %v", box.Min)
	require.True(t, box.Max.Equal(r3.NewVec(35, 24, 10), 1e-9), "max %v", box.Max)
	// The flat pattern: 50 mm of legs and a quarter turn at the neutral radius 3 + 0.44 · 2.
	allowance := math.Pi / 2 * (3 + 0.44*2)
	require.Len(t, report.BendAllowances, 1)
	require.InDelta(t, allowance, report.BendAllowances[0].Base(), 1e-12)
	require.InDelta(t, 50+allowance, report.FlatLength.Base(), 1e-12)
	inside, err := body.LocatePoint(t.Context(), r3.NewVec(15, 0, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside, "the first leg")
	hollow, err := body.LocatePoint(t.Context(), r3.NewVec(15, 5, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, hollow, "above the first leg is empty")
}

func TestSheetMetalBuildsAChannelAndAZBend(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		legs, bends []float64
		min, max    r3.Vec
	}{
		// A channel: both bends turn the same way. The first leg 0..20 along +X, up 40 along +Y between the two
		// bends, and back along -X.
		"channel": {[]float64{20, 40, 20}, []float64{90, 90}, r3.NewVec(0, -1, 0), r3.NewVec(25, 49, 10)},
		// A Z: the bends turn opposite ways, the second leg rising and the third heading on in +X.
		"z bend": {[]float64{10, 15, 10}, []float64{90, -90}, r3.NewVec(0, -1, 0), r3.NewVec(10+4+4+10+0, 4+15+4+1, 10)},
	} {
		doc := decad.New()
		body, report, err := sheetCommand(doc, tc.legs, tc.bends).Do(t.Context())
		require.NoError(t, err, name)
		volume, err := body.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-stripVolume(tc.legs, tc.bends)), volume.Bound.Base()+1e-9, name)
		box, err := body.Bounds()
		require.NoError(t, err)
		require.True(t, box.Min.Equal(tc.min, 1e-6), "%s min %v", name, box.Min)
		require.True(t, box.Max.Equal(tc.max, 1e-6), "%s max %v", name, box.Max)
		require.Len(t, report.BendAllowances, 2, name)
	}
}

func TestSheetMetalBuildsBendsOfAnyAngle(t *testing.T) {
	t.Parallel()
	for _, bends := range [][]float64{{37}, {-53}, {37, -53}, {135}, {120, -120}} {
		legs := make([]float64, len(bends)+1)
		for i := range legs {
			legs[i] = 12.5 + float64(i)
		}
		body, _, err := sheetCommand(decad.New(), legs, bends).Do(t.Context())
		require.NoError(t, err, "%v", bends)
		volume, err := body.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-stripVolume(legs, bends)), volume.Bound.Base()+1e-6, "%v", bends)
	}
}

func TestSheetMetalFlatLengthFollowsTheKFactor(t *testing.T) {
	t.Parallel()
	_, report, err := sheetCommand(decad.New(), []float64{30, 20}, []float64{90}).
		KFactor(units.Scalar(0.5)).Do(t.Context())
	require.NoError(t, err)
	require.InDelta(t, 50+math.Pi/2*(3+0.5*2), report.FlatLength.Base(), 1e-12)
}

func TestSheetMetalRefusals(t *testing.T) {
	t.Parallel()
	good := func() sheetArgs { return sheetArgsOf([]float64{30, 20}, []float64{90}) }
	k := func(v units.Value) func(*sheetArgs) { return func(s *sheetArgs) { s.k = &v } }
	cases := map[string]struct {
		edit func(*sheetArgs)
		want error
	}{
		"zero thickness":   {func(s *sheetArgs) { s.thickness = units.Millimeters(0) }, decad.ErrDegenerate},
		"negative radius":  {func(s *sheetArgs) { s.innerRadius = units.Millimeters(-1) }, decad.ErrNegativeMagnitude},
		"width kind":       {func(s *sheetArgs) { s.width = units.Degrees(5) }, decad.ErrUnitKind},
		"k factor range":   {k(units.Scalar(1.5)), decad.ErrDegenerate},
		"k factor kind":    {k(units.Millimeters(1)), decad.ErrUnitKind},
		"missing bend":     {func(s *sheetArgs) { s.bends = nil }, decad.ErrDegenerate},
		"no legs":          {func(s *sheetArgs) { s.legs, s.bends = nil, nil }, decad.ErrDegenerate},
		"zero leg":         {func(s *sheetArgs) { s.legs[0] = units.Millimeters(0) }, decad.ErrDegenerate},
		"flat bend":        {func(s *sheetArgs) { s.bends[0] = units.Degrees(0) }, decad.ErrDegenerate},
		"half turn":        {func(s *sheetArgs) { s.bends[0] = units.Degrees(180) }, decad.ErrDegenerate},
		"bend kind":        {func(s *sheetArgs) { s.bends[0] = units.Millimeters(1) }, decad.ErrUnitKind},
		"non-finite angle": {func(s *sheetArgs) { s.bends[0] = units.Degrees(math.NaN()) }, decad.ErrNotFinite},
	}
	for name, tc := range cases {
		doc := decad.New()
		args := good()
		tc.edit(&args)
		_, _, err := args.command(doc).Do(t.Context())
		require.ErrorIs(t, err, tc.want, name)
		require.Empty(t, doc.Bodies(), "%s builds nothing", name)
	}
	// Four right-angle bends of 30 mm legs bring the fifth leg back over the first: the strip closes on itself
	// and its outline is no simple loop.
	doc := decad.New()
	_, _, err := sheetCommand(doc, []float64{30, 30, 30, 30, 30}, []float64{90, 90, 90, 90}).Do(t.Context())
	require.Error(t, err)
	t.Logf("%v", err)
	require.Empty(t, doc.Bodies())
	_, _, err = good().command(doc).Do(nil) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = good().command(nil).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
