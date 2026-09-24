package decad_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T200-T206 for Body.Offset (§17):
// the patch arm's translation, the prism arm's certified section offset, and
// the refusal set of Table R rows R37-R41. T207, the internal exact-generation
// gate, is offset_internal_test.go's, as are the whole-interval certification
// and the one-region payload shape that lets §17.2 drop Thicken's
// source-and-offset nesting audit.

// offsetPrismSheet builds a 100x60 mm rectangle surface-extruded 10 mm Along —
// the profile-fed prism sheet §17.1 admits — and returns it with its document.
func offsetPrismSheet(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

// offsetCircleSheet builds a whole-circle prism sheet of the given radius over
// 10 mm — §17.1's second admitted section shape.
func offsetCircleSheet(t *testing.T, doc *decad.Document, radius float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.CreateCircle(c, radius)
	s.Fix(c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return sheet
}

// TestOffsetPatchPositive is T200: a 100x60 mm Document.Patch offset positive
// 2 mm. The result is the source's own face translated along the sketch
// plane's normal, so its Area is the source's reading — value AND bound — and
// the source itself stays live.
func TestOffsetPatchPositive(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	sourceArea, err := patch.Area()
	require.NoError(t, err)
	sourceBox, err := patch.Bounds()
	require.NoError(t, err)

	moved, err := patch.Offset(t.Context(), units.Millimeters(2))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, moved.Kind())
	require.False(t, moved.IsSolid())
	require.Len(t, moved.Faces(), 1)
	decadtest.MeasuresArea(t, moved, units.SquareMillimeters(6000), decadtest.Exactly())
	movedArea, err := moved.Area()
	require.NoError(t, err)
	require.Equal(t, sourceArea, movedArea,
		`a rigid motion moves no area: the offset publishes the source's reading, value and bound`)
	decadtest.MeasuresBounds(t, moved, r3.NewVec(0, 0, 2), r3.NewVec(100, 60, 2), decadtest.Exactly())
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(moved)
	require.NoError(t, err)
	require.Len(t, free, 4)
	_, err = moved.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = moved.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	// The receiver stays LIVE and unread: Offset depends on it, never consumes
	// it (§17.1).
	require.Len(t, doc.Bodies(), 2)
	require.Contains(t, doc.Bodies(), patch)
	againArea, err := patch.Area()
	require.NoError(t, err)
	require.Equal(t, sourceArea, againArea)
	againBox, err := patch.Bounds()
	require.NoError(t, err)
	require.Equal(t, sourceBox, againBox)
}

// TestOffsetPatchNegative is T201: the same patch offset negative 2 mm, with
// both results and their shared source live in one document at once.
func TestOffsetPatchNegative(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)

	up, err := patch.Offset(t.Context(), units.Millimeters(2), decad.WithOffsetSide(decad.OffsetPositive))
	require.NoError(t, err)
	down, err := patch.Offset(t.Context(), units.Millimeters(2),
		decad.WithOffsetSide(decad.OffsetPositive), decad.WithOffsetSide(decad.OffsetNegative))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, down.Kind())
	require.Len(t, down.Faces(), 1)
	decadtest.MeasuresArea(t, down, units.SquareMillimeters(6000), decadtest.Exactly())
	decadtest.MeasuresBounds(t, down, r3.NewVec(0, 0, -2), r3.NewVec(100, 60, -2), decadtest.Exactly())
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(down)
	require.NoError(t, err)
	require.Len(t, free, 4)

	bodies := doc.Bodies()
	require.Len(t, bodies, 3)
	require.Contains(t, bodies, patch)
	require.Contains(t, bodies, up)
	require.Contains(t, bodies, down)
}

// TestOffsetPrismRectangle is T202: a 100x60 mm prism sheet over 10 mm offset
// positive 3 mm. Each convex corner grows a radius-3 mm arc, so the result
// carries eight walls and its perimeter is the straight runs plus one whole
// circle of radius 3.
func TestOffsetPrismRectangle(t *testing.T) {
	t.Parallel()
	doc, sheet := offsetPrismSheet(t)
	grown, err := sheet.Offset(t.Context(), units.Millimeters(3))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, grown.Kind())
	require.Len(t, grown.Faces(), 8)
	var planar, curved int
	for _, f := range grown.Faces() {
		if _, ok := f.Surface().(decad.Plane); ok {
			planar++
			continue
		}
		curved++
	}
	require.Equal(t, 4, planar)
	require.Equal(t, 4, curved)
	free, err := decad.Edges(decad.Free()).Exactly(16).SelectEdges(grown)
	require.NoError(t, err)
	require.Len(t, free, 16)

	area, err := grown.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	requirePiLinearEnclosed(t, area, 3200, 60)
	_, err = grown.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = grown.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	decadtest.MeasuresBounds(t, grown, r3.NewVec(-3, -3, 0), r3.NewVec(103, 63, 10), decadtest.Exactly())

	require.Len(t, doc.Bodies(), 2)
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(3200), decadtest.Exactly())
}

// TestOffsetPairCannotDoubleCount is the receiver-stays-live decision's own
// obligation (§17.1): leaving the source live is safe only if no reading can
// count the pair's material twice. Neither body claims any. Volume and Centroid
// are ErrNotSolid on each BY KIND, and Union, Cut and Intersect refuse a sheet
// operand permanently (Table X), so no shipped operation combines the two into
// one body or one quantity. Area stays per body: reading both is reading two
// surfaces, which is what the pair IS.
func TestOffsetPairCannotDoubleCount(t *testing.T) {
	t.Parallel()
	doc, sheet := offsetPrismSheet(t)
	grown, err := sheet.Offset(t.Context(), units.Millimeters(3))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2)

	for _, body := range []*decad.Body{sheet, grown} {
		_, err := body.Volume()
		require.ErrorIs(t, err, decad.ErrNotSolid)
		_, err = body.Centroid()
		require.ErrorIs(t, err, decad.ErrNotSolid)
		require.False(t, body.IsSolid())
	}
	for _, tc := range []struct {
		name string
		call func() (*decad.Body, error)
	}{
		{"union", func() (*decad.Body, error) { return decad.Union(t.Context(), sheet, grown) }},
		{"cut", func() (*decad.Body, error) { return decad.Cut(t.Context(), sheet, grown) }},
		{"intersect", func() (*decad.Body, error) { return decad.Intersect(t.Context(), sheet, grown) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := tc.call()
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Len(t, doc.Bodies(), 2, `a refused boolean registers and retires nothing`)
		})
	}
}

// TestOffsetPrismCircle is T203: a radius-10 mm whole-circle prism sheet over
// 10 mm, offset 2 mm each way. Both results are one concentric wall.
func TestOffsetPrismCircle(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		side   decad.OffsetSide
		area   float64
		radius float64
	}{
		{"outward", decad.OffsetPositive, 240, 12},
		{"inward", decad.OffsetNegative, 160, 8},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			sheet := offsetCircleSheet(t, doc, 10)
			result, err := sheet.Offset(t.Context(), units.Millimeters(2), decad.WithOffsetSide(tc.side))
			require.NoError(t, err)

			require.Equal(t, decad.BodySheet, result.Kind())
			require.Len(t, result.Faces(), 1)
			area, err := result.Area()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, area.Exactness,
				`an offset circle's perimeter carries the rational pi enclosure, never a zero bound`)
			require.Positive(t, area.Bound.Base())
			requirePiLinearEnclosed(t, area, 0, tc.area)
			decadtest.MeasuresBounds(t, result, r3.NewVec(-tc.radius, -tc.radius, 0),
				r3.NewVec(tc.radius, tc.radius, 10), decadtest.Exactly())
			require.Len(t, doc.Bodies(), 2)
		})
	}
}

// TestOffsetPrismNeckRefusal is T204: §17.2's 4 mm-wide neck eroded 3 mm makes
// its two offset walls cross, which the shared crossing audit names (R38).
//
// Shown to fail: drop auditOffsetSectionBudget from the prism arm and the
// whole-interval certification refuses the same neck instead, with the less
// specific "the offset interval contains a possible nonadjacent contact" —
// which turns the message assertion below red. That is the gate order §16.2
// fixes and §17.2 inherits, asserted rather than assumed.
func TestOffsetPrismNeckRefusal(t *testing.T) {
	t.Parallel()
	doc, sheet := thickenNeckSheet(t)
	_, err := sheet.Offset(t.Context(), units.Millimeters(3), decad.WithOffsetSide(decad.OffsetNegative))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "crosses itself"), err.Error())
	require.Len(t, doc.Bodies(), 1)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(1320), decadtest.Exactly())
}

// TestOffsetPrismNarrowNeckAdmitted pins that the neck's refusal is the
// crossing, not the shape: the same section eroded 1.5 mm builds, so T204's
// refusal cannot be read as a blanket refusal of a concave loop.
func TestOffsetPrismNarrowNeckAdmitted(t *testing.T) {
	t.Parallel()
	_, sheet := thickenNeckSheet(t)
	eroded, err := sheet.Offset(t.Context(), units.Millimeters(1.5), decad.WithOffsetSide(decad.OffsetNegative))
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, eroded.Kind())
	require.Len(t, eroded.Faces(), 16, `twelve eroded walks plus one arc at each of the four reflex corners`)
	area, err := eroded.Area()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, area, 1080, 30)
}

// TestOffsetSecondOffset is T205: an offset result is an ordinary receiver, and
// §17.1's own section-shape gate decides it. An offset rectangle carries corner
// arcs and is no longer the axis-parallel line class (R37); an offset circle is
// again one whole CircleSeg and offsets again.
//
// The radius-13 leg is also the group's tightest bound, so it is the fixture
// that isolates the arc perimeter's rational π enclosure. Shown to fail:
// dropping circularLengthInterval's refinement of a circular walk's length
// bound leaves a bound that no longer covers the reading's own float error and
// the 260π enclosure goes red, while T202's and T203's own readings keep enough
// slack to survive the same deletion.
func TestOffsetSecondOffset(t *testing.T) {
	t.Parallel()
	t.Run("rectangle", func(t *testing.T) {
		doc, sheet := offsetPrismSheet(t)
		grown, err := sheet.Offset(t.Context(), units.Millimeters(3))
		require.NoError(t, err)
		_, err = grown.Offset(t.Context(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.True(t, strings.Contains(err.Error(), "axis-parallel"), err.Error())
		require.Len(t, doc.Bodies(), 2)
	})
	t.Run("circle", func(t *testing.T) {
		doc := decad.New()
		sheet := offsetCircleSheet(t, doc, 10)
		twelve, err := sheet.Offset(t.Context(), units.Millimeters(2))
		require.NoError(t, err)
		thirteen, err := twelve.Offset(t.Context(), units.Millimeters(1))
		require.NoError(t, err)
		require.Len(t, thirteen.Faces(), 1)
		area, err := thirteen.Area()
		require.NoError(t, err)
		requirePiLinearEnclosed(t, area, 0, 260)
		decadtest.MeasuresBounds(t, thirteen, r3.NewVec(-13, -13, 0), r3.NewVec(13, 13, 10),
			decadtest.Exactly())
		require.Len(t, doc.Bodies(), 3)
	})
}

// TestOffsetStagedFamilies is T206: every receiver §17.1 does not admit is R37,
// and each one stays live and readable with the document's body set unchanged.
func TestOffsetStagedFamilies(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	w := sketch.NewWorld()
	cs, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := cs.CreatePoint(10, 0)
	b := cs.CreatePoint(10, 40)
	cs.Fix(a)
	cs.CreateLine(a, b)
	_, err = cs.Solve(t.Context())
	require.NoError(t, err)
	chain := cs.Chains()[0]

	ribbon, err := doc.ExtrudeChain(cs, chain, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	shell, err := doc.RevolveChain(cs, chain, axis, decad.FullRevolution{})
	require.NoError(t, err)

	ps, pp := plateSketch(t)
	flat, err := doc.Patch(t.Context(), ps, pp)
	require.NoError(t, err)
	doubled, err := flat.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(4))
	require.NoError(t, err)

	walls, bottom, _ := stitchBoxSheets(t, doc, 10)
	stitched, err := decad.Stitch(t.Context(), walls, bottom)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, stitched.Kind())

	ss, sp := plateSketch(t)
	solid, err := doc.Extrude(ss, sp, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	before := doc.Bodies()
	// Each refusal is read by its own TEXT, not only by errors.Is: R37 covers a
	// solid and a sheet family alike, so the sentinel alone cannot say which
	// gate spoke.
	for _, tc := range []struct {
		name string
		body *decad.Body
		want string
	}{
		{"chain ribbon", ribbon, "no admitted Offset generator"},
		{"chain revolve shell", shell, "no admitted Offset generator"},
		{"Body.Patch result", doubled, "no admitted Offset generator"},
		{"stitched sheet", stitched, "no admitted Offset generator"},
		{"solid", solid, "requires a sheet body"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := tc.body.Offset(t.Context(), units.Millimeters(1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Contains(t, err.Error(), tc.want)
			require.Equal(t, before, doc.Bodies())
			_, err = tc.body.Area()
			require.NoError(t, err)
		})
	}
	decadtest.MeasuresArea(t, ribbon, units.SquareMillimeters(400), decadtest.Exactly())
	free, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(shell)
	require.NoError(t, err)
	require.Len(t, free, 2)
}

// TestOffsetRefusals is Table R's R40 and R41 plus the sealed option tier: a
// bad magnitude keeps magnitudeIn's own sentinel, an option this package did
// not mint is ErrDegenerate, and a retired receiver is ErrRetiredBody. None of
// them registers or retires anything.
func TestOffsetRefusals(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	// Three of the five share ErrDegenerate, so each case also reads the text
	// that says WHICH gate refused it.
	for _, tc := range []struct {
		name  string
		value units.Value
		opts  []decad.OffsetOption
		want  error
		says  string
	}{
		{"zero", units.Millimeters(0), nil, decad.ErrDegenerate, "zero distance names no offset surface"},
		{"negative magnitude", units.Millimeters(-1), nil, decad.ErrNegativeMagnitude, "the offset distance"},
		{"wrong kind", units.SquareMillimeters(2), nil, decad.ErrUnitKind, "the offset distance"},
		{"nil option", units.Millimeters(2), []decad.OffsetOption{nil},
			decad.ErrDegenerate, "not a decad offset option"},
		{"unknown side", units.Millimeters(2), []decad.OffsetOption{decad.WithOffsetSide(100)},
			decad.ErrDegenerate, "unknown offset side"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := patch.Offset(t.Context(), tc.value, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Contains(t, err.Error(), tc.says)
			require.Len(t, doc.Bodies(), 1)
		})
	}

	// R41: Thicken consumes its receiver, so an offset of that retired sheet
	// refuses on liveness rather than on any geometry.
	solid, err := patch.Thicken(t.Context(), units.Millimeters(2))
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())
	_, err = patch.Offset(t.Context(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

// TestOffsetPatchConversionIsRefused is R39's patch leg: a distance that
// rescales to millimetres with a nonzero displacement would move every
// published coordinate off the translation the call denotes, and patchPayload
// carries no term to charge it against, so the call refuses rather than
// publishing a zero-bound reading of a body it cannot place exactly.
func TestOffsetPatchConversionIsRefused(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	_, err = patch.Offset(t.Context(), units.Inches(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "conversion"), err.Error())
	require.Len(t, doc.Bodies(), 1)

	// The same distance in millimetres converts by a factor of one, reports a
	// zero displacement, and builds: the refusal is the conversion's doing.
	moved, err := patch.Offset(t.Context(), units.Millimeters(2.54))
	require.NoError(t, err)
	decadtest.MeasuresBounds(t, moved, r3.NewVec(0, 0, 2.54), r3.NewVec(100, 60, 2.54),
		decadtest.Exactly())
}

// TestOffsetPrismCarriesTheSourceAxialDisplacement is §17.3's axial bound row:
// the result's Bounds carries the SOURCE's z0Delta/z1Delta unchanged, because
// the offset replaces the section and nothing else. The fixture's height is
// stated in inches, so its rescale to millimetres commits a rounding the source
// already publishes, and the offset must publish the same one rather than a
// zero bound on a level it cannot represent.
//
// Shown to fail: zeroing the two axial displacements as the payload is rebuilt
// drops this reading's Bound to zero and turns it Exact, which the strictly
// positive assertion and the equality below both catch.
func TestOffsetPrismCarriesTheSourceAxialDisplacement(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Inches(0.1), Dir: decad.Along},
		decad.WithSurfaceResult())
	require.NoError(t, err)
	sourceBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, sourceBox.Exactness)
	require.Positive(t, sourceBox.Bound.Base())

	grown, err := sheet.Offset(t.Context(), units.Millimeters(3))
	require.NoError(t, err)
	box, err := grown.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, box.Exactness,
		`the source's axial displacement travels with the payload; a zero bound would claim a level this build cannot represent`)
	require.Positive(t, box.Bound.Base())
	require.Equal(t, sourceBox.Bound, box.Bound,
		`§17.3: the axial terms are the source's, unchanged — the offset replaces the section and nothing else`)
	require.Equal(t, r3.NewVec(-3, -3, 0), box.Min)
	require.Equal(t, r3.NewVec(103, 63, 2.54), box.Max)
}
