package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T31-T36 public coverage for
// stitch_flux.go's per-surface flux integral: the `Plane` and `Cylinder`
// arms that let `Stitch` close a boundary holding a curved face. T37 and T38
// are internal-only (stitch_internal_test.go): a genuinely closed
// `NURBSSurface`-holding set and a hand-built nonzero-`normalBound` face are
// both unreachable through the public seam, for reasons each test records.

// annularSketchRange is annularSketch generalized to an arbitrary radial
// band [vLo, vHi] and axial span [0, uLen], clear of the revolve axis —
// T31's own fixture with T33's family swept across it.
func annularSketchRange(t *testing.T, uLen, vLo, vHi float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, vLo, uLen, vHi)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// annularRevolveSheet builds annularSketchRange's profile as a closed
// full-revolution surface sheet: 2 `Cylinder` walls, 2 `Plane` annuli, no
// free edge (Table W).
func annularRevolveSheet(t *testing.T, uLen, vLo, vHi float64) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := annularSketchRange(t, uLen, vLo, vHi)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

// TestStitchAnnularRevolveSheetClosesToATube is T31: annularSketch's own
// full-revolution surface sheet, stitched alone, closes to a solid whose
// volume, area and centroid this file's flux arms now compute — the
// increment's own headline case.
func TestStitchAnnularRevolveSheetClosesToATube(t *testing.T) {
	t.Parallel()
	doc, sheet := annularRevolveSheet(t, 10, 5, 15)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 4)
	decadtest.HasSurfaceKinds(t, sheet, map[decad.SurfaceKind]int{
		decad.KindCylinder: 2,
		decad.KindPlane:    2,
	})

	solid, err := decad.Stitch(sheet)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(2000*math.Pi))
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(800*math.Pi))
	decadtest.MeasuresCentroid(t, solid, r3.NewVec(5, 0, 0))

	vol, err := solid.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness, "a term carrying pi can never claim Exact")
	require.Greater(t, vol.Bound.Base(), 0.0)

	cen, err := solid.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, cen.Exactness)

	_, err = decad.Edges(decad.Free()).SelectEdges(solid)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	require.Len(t, doc.Bodies(), 1)
	require.Same(t, solid, doc.Bodies()[0])
}

// TestStitchAnnularSolidMatchesTheRevolveEngine is T32: the
// independent-producer cross-check. The same profile built as a `BodySolid`
// through `Revolve` with no option is an entirely different code path — the
// Pappus-based analytic solid evaluator, never stitch_flux.go's own
// arithmetic — so agreement here does not depend on this increment's own
// derivation being right.
func TestStitchAnnularSolidMatchesTheRevolveEngine(t *testing.T) {
	t.Parallel()
	_, sheet := annularRevolveSheet(t, 10, 5, 15)
	stitched, err := decad.Stitch(sheet)
	require.NoError(t, err)

	s2, p2 := annularSketchRange(t, 10, 5, 15)
	solidDoc := decad.New()
	direct, err := solidDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	stitchedVol, err := stitched.Volume()
	require.NoError(t, err)
	directVol, err := direct.Volume()
	require.NoError(t, err)
	decadtest.Agree(t, "volume", stitchedVol, directVol)

	stitchedCen, err := stitched.Centroid()
	require.NoError(t, err)
	directCen, err := direct.Centroid()
	require.NoError(t, err)
	tol := stitchedCen.Bound.Base() + directCen.Bound.Base() + 1e-9
	require.InDelta(t, directCen.Value.X, stitchedCen.Value.X, tol)
	require.InDelta(t, directCen.Value.Y, stitchedCen.Value.Y, tol)
	require.InDelta(t, directCen.Value.Z, stitchedCen.Value.Z, tol)
}

// TestStitchCurvedVolumeBoundEncloses is T33: the bound leg, over a family
// of radii and heights, each checked against its own Pappus-hand analytic
// volume and centroid.
func TestStitchCurvedVolumeBoundEncloses(t *testing.T) {
	t.Parallel()
	cases := []struct {
		uLen, vLo, vHi float64
	}{
		{10, 5, 15},
		{3, 1, 2},
		{50, 20, 21},
		{1, 100, 101},
	}
	for _, c := range cases {
		_, sheet := annularRevolveSheet(t, c.uLen, c.vLo, c.vHi)
		solid, err := decad.Stitch(sheet)
		require.NoError(t, err)

		area := c.vHi*c.vHi - c.vLo*c.vLo // pi * area factor, Pappus: V = pi*(vHi^2-vLo^2)*uLen
		wantVol := math.Pi * area * c.uLen
		decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(wantVol))
		decadtest.MeasuresCentroid(t, solid, r3.NewVec(c.uLen/2, 0, 0))
	}
}

// TestStitchCurvedVolumeBoundWidensWhenPlaced is the placement allowance's
// own shown-to-fail leg: a placed curved solid's own Bound must be strictly
// larger than the identical unplaced shape's, since a rigid motion rounds
// every coordinate the flux arms read. Deleting sweptVolumeAllow/
// sweptMomentAllow from stitchCurvedMass collapses this to an equality and
// this test goes red — recorded in the PR body.
func TestStitchCurvedVolumeBoundWidensWhenPlaced(t *testing.T) {
	t.Parallel()
	_, sheet := annularRevolveSheet(t, 10, 5, 15)
	unplaced, err := decad.Stitch(sheet)
	require.NoError(t, err)
	unplacedVol, err := unplaced.Volume()
	require.NoError(t, err)
	unplacedCen, err := unplaced.Centroid()
	require.NoError(t, err)

	motion, err := r3.Translation(r3.NewVec(3, -4, 9))
	require.NoError(t, err)
	rot, err := r3.RotationAround(r3.NewVec(3, -4, 9), r3.NewVec(0, 0, 1), units.Degrees(17))
	require.NoError(t, err)
	composed, err := motion.Then(rot)
	require.NoError(t, err)

	placed, err := unplaced.Placed(composed)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, placed.Kind())

	placedVol, err := placed.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, placedVol.Exactness)
	require.Greater(t, placedVol.Bound.Base(), unplacedVol.Bound.Base(),
		"a placed curved solid's volume bound must widen over the unplaced one")
	decadtest.MeasuresVolume(t, placed, units.CubicMillimeters(2000*math.Pi), decadtest.WithinRel(units.Scalar(1e-6)))

	placedCen, err := placed.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, placedCen.Exactness)
	require.Greater(t, placedCen.Bound.Base(), unplacedCen.Bound.Base(),
		"a placed curved solid's centroid bound must widen over the unplaced one")
}

// TestStitchPatchCappedTubeStaysUnsupported is T34: an extruded tube capped
// on both rims by Body.Patch structurally succeeds — this design's own
// motivating "walls, then cap, then stitch" flow (§6.1) — but Rule S
// refuses the stitch that would close it, since a bodyPatchPayload carries
// no non-self-intersection proof of its own (Body.Patch proves its own
// chains simple in their own plane, never the whole assembled boundary's).
// This is a real, visible gap this increment leaves open, not a corner
// case: a later increment's own Rule P is what closes it.
func TestStitchPatchCappedTubeStaysUnsupported(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	p := s.Profiles()[0]

	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 2)

	patched, err := sheet.Patch(decad.Edges(decad.Free()).Exactly(2))
	require.NoError(t, err)
	_, err = decad.Edges(decad.Free()).SelectEdges(patched)
	require.ErrorIs(t, err, decad.ErrNoMatch, "Body.Patch itself closes the tube")

	before := doc.Bodies()
	_, err = decad.Stitch(patched)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}

// T35 (two curved sheets from different features, refused by Rule S's
// single-source restriction) is internal-only: docs/surface-design.md §6.2's
// Table J admits a free Line3 edge alone (J5 is undecidable for Circle3,
// Arc3, NURBSCurve and FacetedCurve, since none of them carries a bound
// field to ask "zero bound" of — stitch_weld.go's own doc comment). So no
// two curved rims from different features ever weld into a closed set
// through the public seam at all — confirmed directly: a surface-extruded
// cylinder wall plus two Document.Patch discs at its own exact end planes,
// bit-identical circles at bit-identical planes, still leaves all four rims
// free after Stitch. Rule S's own single-source-body restriction is
// pinned instead by TestStitchRuleSRefusesMultipleSourceBodies
// (stitch_internal_test.go), on a hand-built face set the same way
// TestStitchOrientationRefusesMobiusAssembly pins deriveStitchOrientation.

// TestStitchTorusFaceStaysUnsupported is T36: offAxisSemicircleSketch's own
// full-revolution surface sheet closes with one Cylinder wall (this
// increment's own landed arm) and one Torus wall (not landed) and no free
// edge. Rule S admits it — a full-turn revolvePayload clear of the axis —
// so the refusal this test pins is the sealed switch's own default arm for
// KindTorus, not a Rule S or vertex-link refusal; stitch_flux.go's own
// dispatch never reaches a face it cannot classify without first deciding
// every OTHER face admits.
func TestStitchTorusFaceStaysUnsupported(t *testing.T) {
	t.Parallel()
	s, p := offAxisSemicircleSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Len(t, sheet.Faces(), 2)
	decadtest.HasSurfaceKinds(t, sheet, map[decad.SurfaceKind]int{
		decad.KindCylinder: 1,
		decad.KindTorus:    1,
	})
	_, err = decad.Edges(decad.Free()).SelectEdges(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	before := doc.Bodies()
	_, err = decad.Stitch(sheet)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}
