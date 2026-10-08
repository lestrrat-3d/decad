package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T31-T36, T46-T47 and T50-T52
// public coverage for stitch_flux.go's per-surface flux integral: the
// `Plane`, `Cylinder`, `Cone` and `Sphere` arms that let `Stitch` close a
// boundary holding a curved face. T37, T38, T48 and T49 are internal-only
// (stitch_internal_test.go): a genuinely closed
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

	solid, err := decad.Stitch(t.Context(), sheet)
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
	stitched, err := decad.Stitch(t.Context(), sheet)
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
		solid, err := decad.Stitch(t.Context(), sheet)
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
	unplaced, err := decad.Stitch(t.Context(), sheet)
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

	placed, err := unplaced.Placed(t.Context(), composed)
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

// TestStitchPatchCappedTubeClosesToASolid is T34: an extruded tube capped
// on both rims by Body.Patch, in one call, then stitched — this design's
// own motivating "walls, then cap, then stitch" flow (§6.1). Rule P admits
// the closure: the receiver's own prismPayload proves simple under Rule S
// (a plain Distance extrude, sectionDelta zero), and each of the two new
// chains is the receiver's own COMPLETE end rim under its own shared level
// id, minted once per end by prism_build.go's evalPrismContext regardless
// of the end's own bound.
func TestStitchPatchCappedTubeClosesToASolid(t *testing.T) {
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

	patched, err := sheet.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(2))
	require.NoError(t, err)
	_, err = decad.Edges(decad.Free()).SelectEdges(patched)
	require.ErrorIs(t, err, decad.ErrNoMatch, "Body.Patch itself closes the tube")

	solid, err := decad.Stitch(t.Context(), patched)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(1000*math.Pi))
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(400*math.Pi))
	decadtest.MeasuresCentroid(t, solid, r3.NewVec(0, 0, 5))

	vol, err := solid.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness, "a term carrying pi can never claim Exact")
	require.Greater(t, vol.Bound.Base(), 0.0)

	_, err = decad.Edges(decad.Free()).SelectEdges(solid)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	require.Len(t, doc.Bodies(), 1)
	require.Same(t, solid, doc.Bodies()[0])
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

// halfTorusAnalytics is the by-hand closed form for offAxisSemicircleSketch's
// own shape: a straight chord at rho=Major (the tube's own equatorial
// diameter, revolving into a Cylinder wall) and a semicircular arc bulging
// outward from Major to Major+Minor and back (revolving into a Torus wall
// spanning exactly the tube's own outer half, phi in [-pi/2, pi/2]).
// Volume and area are Pappus's theorem over the 2D half-disc profile
// directly (a route independent of stitch_flux.go's own per-face flux
// arms, matching how frustumShellAnalytics stays independent of the Cone
// arm): a half-disc of radius Minor has area (pi/2)*Minor^2 and its own
// centroid sits (4*Minor)/(3*pi) from the flat diameter, on the bulge side,
// so Volume = 2*pi*(Major + (4*Minor)/(3*pi))*((pi/2)*Minor^2) — expanding
// the (4*Minor)/(3*pi) term against the *pi factor it multiplies removes pi
// from that half entirely, matching torusFaceFluxAndMoment's own K_F
// closed form once the sibling Cylinder wall's own INWARD-facing K_F
// (Cylinder's own K_F identity always faces away from its axis; here that
// direction points OUT of the solid, so it subtracts rather than adds — the
// solid occupies rho >= Major, not rho <= Major) is folded in. Area is the
// straight chord's own cylinder wall (2*pi*Major*(2*Minor), the chord's own
// length being the tube's own diameter 2*Minor) plus the arc's own Torus
// wall lateral area (2*pi*Minor*(Major*pi + 2*Minor), the standard partial
// torus-zone area over a pi-wide window). The axial centroid is exactly the
// generatrix's own chord midpoint (offAxisSemicircleSketchGeneral's u0),
// independent of Major and Minor, so the caller asserts that directly
// rather than through a formula here — checked by hand against the
// divergence-theorem moment integral while landing this test: the
// Cylinder wall's own first moment along the revolve axis is exactly zero
// (a cylinder's outward normal never has an axial component), and the
// Torus wall's own axial moment reduces to exactly u0 times the total
// volume once its own (1-Axis_i^2) coefficient (torusFaceFluxAndMoment's
// own doc comment) vanishes for the axis-aligned component.
func halfTorusAnalytics(major, minor float64) (volume, area float64) {
	halfDiscArea := (math.Pi / 2) * minor * minor
	halfDiscCentroidOffset := (4 * minor) / (3 * math.Pi)
	volume = 2 * math.Pi * (major + halfDiscCentroidOffset) * halfDiscArea
	cylinderArea := 2 * math.Pi * major * (2 * minor)
	torusArea := 2 * math.Pi * minor * (major*math.Pi + 2*minor)
	area = cylinderArea + torusArea
	return volume, area
}

// offAxisSemicircleSketchGeneral generalizes offAxisSemicircleSketch to an
// arbitrary Major (the chord's own axial-clear radius) and Minor (the
// arc's own radius) centered at axial position u0 — T53's own fixture with
// T55's family swept across it.
func offAxisSemicircleSketchGeneral(t *testing.T, u0, major, minor float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(u0-minor, major)
	s.Fix(o)
	end := s.CreatePoint(u0+minor, major)
	c := s.CreatePoint(u0, major)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// halfTorusRevolveSheet builds offAxisSemicircleSketchGeneral's profile as
// a closed full-revolution surface sheet: one Cylinder wall (the chord) and
// one Torus wall (the arc), no free edge.
func halfTorusRevolveSheet(t *testing.T, u0, major, minor float64) *decad.Body {
	t.Helper()
	s, p := offAxisSemicircleSketchGeneral(t, u0, major, minor)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return sheet
}

// TestStitchTorusRevolveSheetClosesToASolid is T53: offAxisSemicircleSketch's
// own full-revolution surface sheet (u0=5, Major=10, Minor=5 in
// offAxisSemicircleSketchGeneral's own parametrization — the identical
// points TestStitchTorusFaceStaysUnsupported built) — one Cylinder wall and
// one Torus wall spanning exactly the tube's own outer half, phi in
// [-pi/2, pi/2] — closes to a solid whose volume, area and centroid the
// Torus arm now computes, checked against halfTorusAnalytics's own
// independent Pappus derivation. This replaces
// TestStitchTorusFaceStaysUnsupported: the refusal that test pinned (the
// sealed switch's own then-missing KindTorus arm) is retired, and its shard
// row moves to this test's name.
func TestStitchTorusRevolveSheetClosesToASolid(t *testing.T) {
	t.Parallel()
	s, p := offAxisSemicircleSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 2)
	decadtest.HasSurfaceKinds(t, sheet, map[decad.SurfaceKind]int{
		decad.KindCylinder: 1,
		decad.KindTorus:    1,
	})
	_, err = decad.Edges(decad.Free()).SelectEdges(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	solid, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	wantVol, wantArea := halfTorusAnalytics(10, 5)
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(wantVol))
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(wantArea))
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

// TestStitchTorusSolidMatchesTheRevolveEngine is T54: the
// independent-producer cross-check, the Torus arm's own sibling of
// TestStitchAnnularSolidMatchesTheRevolveEngine,
// TestStitchConicalSolidMatchesTheRevolveEngine and
// TestStitchSphereSolidMatchesTheRevolveEngine. The same profile built as a
// BodySolid through Revolve with no option is the Pappus-based analytic
// solid evaluator, never stitch_flux.go's own arithmetic, so agreement here
// does not depend on this PR's own derivation being right.
func TestStitchTorusSolidMatchesTheRevolveEngine(t *testing.T) {
	t.Parallel()
	sheet := halfTorusRevolveSheet(t, 5, 10, 5)
	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)

	s2, p2 := offAxisSemicircleSketchGeneral(t, 5, 10, 5)
	solidDoc := decad.New()
	direct, err := solidDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	stitchedVol, err := stitched.Volume()
	require.NoError(t, err)
	directVol, err := direct.Volume()
	require.NoError(t, err)
	decadtest.Agree(t, "volume", stitchedVol, directVol)

	// The centroid comparison below is real but WEAK for this fixture,
	// checked directly while landing this test: the direct revolve
	// engine's own centroid bound comes out about 80 mm wide for this
	// 20 mm-wide solid — nowhere near the roughly 460 mm the Sphere arm's
	// own cross-check hits for a profile touching the axis at both poles
	// (this fixture stays clear of the axis, so it does not suffer that
	// failure mode), but still far too wide to be decisive on its own. So
	// this leg only catches a gross blunder (a wrong sign, an
	// order-of-magnitude error); the tight, decisive centroid proof against
	// the hand-derived analytic value is
	// TestStitchTorusRevolveSheetClosesToASolid (T53) and
	// TestStitchTorusVolumeBoundEncloses (T55), both Bound-tight and both
	// shown to fail against a broken moment formula.
	stitchedCen, err := stitched.Centroid()
	require.NoError(t, err)
	directCen, err := direct.Centroid()
	require.NoError(t, err)
	tol := stitchedCen.Bound.Base() + directCen.Bound.Base() + 1e-9
	require.InDelta(t, directCen.Value.X, stitchedCen.Value.X, tol)
	require.InDelta(t, directCen.Value.Y, stitchedCen.Value.Y, tol)
	require.InDelta(t, directCen.Value.Z, stitchedCen.Value.Z, tol)
}

// TestStitchTorusVolumeBoundEncloses is T55: T53's fixture swept across a
// family of Major/Minor radii and axial centers, each checked against its
// own halfTorusAnalytics closed form.
func TestStitchTorusVolumeBoundEncloses(t *testing.T) {
	t.Parallel()
	cases := []struct {
		u0, major, minor float64
	}{
		{5, 10, 5},
		{0, 20, 3},
		{-15, 6, 6},
		{100, 50, 1},
	}
	for _, c := range cases {
		sheet := halfTorusRevolveSheet(t, c.u0, c.major, c.minor)
		solid, err := decad.Stitch(t.Context(), sheet)
		require.NoError(t, err)

		wantVol, wantArea := halfTorusAnalytics(c.major, c.minor)
		decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(wantVol))
		decadtest.MeasuresArea(t, solid, units.SquareMillimeters(wantArea))
		decadtest.MeasuresCentroid(t, solid, r3.NewVec(c.u0, 0, 0))
	}
}

// trapezoidFrustumSketch builds a trapezoid profile clear of the revolve
// axis whose two non-radial sides both lean off it — (0, vLo0)-(uLen, vLo1)
// and (0, vHi0)-(uLen, vHi1) — rather than running parallel to it the way
// annularSketchRange's rectangle does. Revolved a full turn, those two
// sides sweep two Cone walls of different half-angles (T46's own fixture),
// never Cylinder walls, as long as vLo0 != vLo1 and vHi0 != vHi1.
func trapezoidFrustumSketch(t *testing.T, uLen, vLo0, vHi0, vLo1, vHi1 float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p1 := s.CreatePoint(0, vLo0)
	p2 := s.CreatePoint(uLen, vLo1)
	p3 := s.CreatePoint(uLen, vHi1)
	p4 := s.CreatePoint(0, vHi0)
	s.Fix(p1)
	s.CreateLine(p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p4)
	s.CreateLine(p4, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// frustumSheet builds trapezoidFrustumSketch's profile as a closed
// full-revolution surface sheet: 2 Cone walls, 2 Plane annuli, no free
// edge (Table W), the frustum-shell analogue of annularRevolveSheet.
func frustumSheet(t *testing.T, uLen, vLo0, vHi0, vLo1, vHi1 float64) (*decad.Document, *decad.Body) { //nolint:unparam // The independent frustum dimensions make this fixture reusable beyond the current call sites.
	t.Helper()
	s, p := trapezoidFrustumSketch(t, uLen, vLo0, vHi0, vLo1, vHi1)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

// frustumShellAnalytics is the by-hand Pappus-style closed form for the
// frustum shell trapezoidFrustumSketch describes: the outer and inner
// radii are each linear in u, v(u) = a + b*(u/uLen), so
// integral_0^uLen v(u)^2 du = uLen*(a^2 + a*b + b^2/3) (the standard
// integral of a linear function squared over a unit interval, scaled) and
// integral_0^uLen u*v(u)^2 du = uLen^2*(a^2/2 + 2*a*b/3 + b^2/4), the same
// substitution weighted by u for the first moment. Lateral area is the
// standard frustum formula pi*(r1+r2)*slant, slant = hypot(deltaR, uLen).
func frustumShellAnalytics(uLen, vLo0, vHi0, vLo1, vHi1 float64) (volume, area, centroidX float64) {
	aO, bO := vHi0, vHi1-vHi0
	aI, bI := vLo0, vLo1-vLo0
	iOuter := uLen * (aO*aO + aO*bO + bO*bO/3)
	iInner := uLen * (aI*aI + aI*bI + bI*bI/3)
	volume = math.Pi * (iOuter - iInner)

	jOuter := uLen * uLen * (aO*aO/2 + 2*aO*bO/3 + bO*bO/4)
	jInner := uLen * uLen * (aI*aI/2 + 2*aI*bI/3 + bI*bI/4)
	centroidX = (jOuter - jInner) / (iOuter - iInner)

	slantOuter := math.Hypot(bO, uLen)
	slantInner := math.Hypot(bI, uLen)
	lateralOuter := math.Pi * (vHi0 + vHi1) * slantOuter
	lateralInner := math.Pi * (vLo0 + vLo1) * slantInner
	annulus0 := math.Pi * (vHi0*vHi0 - vLo0*vLo0)
	annulus1 := math.Pi * (vHi1*vHi1 - vLo1*vLo1)
	area = lateralOuter + lateralInner + annulus0 + annulus1
	return volume, area, centroidX
}

// TestStitchConicalRevolveSheetCloses is T46: trapezoidFrustumSketch's own
// full-revolution surface sheet, stitched alone, closes to a solid whose
// volume, area and centroid the Cone arm now computes against the
// hand-derived frustumShellAnalytics closed form.
func TestStitchConicalRevolveSheetCloses(t *testing.T) {
	t.Parallel()
	const uLen, vLo0, vHi0, vLo1, vHi1 = 10.0, 5.0, 15.0, 8.0, 12.0
	doc, sheet := frustumSheet(t, uLen, vLo0, vHi0, vLo1, vHi1)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 4)
	decadtest.HasSurfaceKinds(t, sheet, map[decad.SurfaceKind]int{
		decad.KindCone:  2,
		decad.KindPlane: 2,
	})

	solid, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	wantVol, wantArea, wantCX := frustumShellAnalytics(uLen, vLo0, vHi0, vLo1, vHi1)
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(wantVol))
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(wantArea))
	decadtest.MeasuresCentroid(t, solid, r3.NewVec(wantCX, 0, 0))

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

// TestStitchConicalSolidMatchesTheRevolveEngine is T47: the
// independent-producer cross-check, TestStitchAnnularSolidMatchesTheRevolveEngine's
// Cone-arm sibling. The same profile built as a BodySolid through Revolve
// with no option is the Pappus-based analytic solid evaluator, never
// stitch_flux.go's own arithmetic, so agreement here is the strongest
// available proof: it does not depend on this PR's own derivation being
// right, only on the two independent producers agreeing.
func TestStitchConicalSolidMatchesTheRevolveEngine(t *testing.T) {
	t.Parallel()
	const uLen, vLo0, vHi0, vLo1, vHi1 = 10.0, 5.0, 15.0, 8.0, 12.0
	_, sheet := frustumSheet(t, uLen, vLo0, vHi0, vLo1, vHi1)
	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)

	s2, p2 := trapezoidFrustumSketch(t, uLen, vLo0, vHi0, vLo1, vHi1)
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

// semicircleSketchAt is semicircleSketch generalized to an arbitrary
// diameter length and starting position along the axis — T50's own fixture
// with T52's family swept across it. The diameter still runs along the
// sketch's own U axis, collinear with uAxis, so every fixture this builds
// stays on-origin and axis-aligned in the sense boundedCircleRadius's own
// doc comment names, exactly as T50 itself is.
func semicircleSketchAt(t *testing.T, u0, diameter float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(u0, 0)
	s.Fix(o)
	end := s.CreatePoint(u0+diameter, 0)
	c := s.CreatePoint(u0+diameter/2, 0)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// sphereRevolveSheet builds semicircleSketchAt's profile as a closed
// full-revolution surface sheet: a single, boundary-less Sphere face, no
// free edge (T6/T50's own shape).
func sphereRevolveSheet(t *testing.T, u0, diameter float64) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := semicircleSketchAt(t, u0, diameter)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

// TestStitchSphereRevolveSheetClosesToABall is T50: semicircleSketch's own
// full-revolution surface sheet (T6's fixture) closes to a solid whose
// volume, area and centroid the Sphere arm now computes. This replaces
// TestStitchClosedCurvedSheetIsUnsupported (stitch_test.go): T6's own
// "stitching it alone is ErrUnsupported" half is retired, and its shard row
// moves to this test's name.
func TestStitchSphereRevolveSheetClosesToABall(t *testing.T) {
	t.Parallel()
	doc, sheet := sphereRevolveSheet(t, 0, 10)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 1)
	decadtest.HasSurfaceKinds(t, sheet, map[decad.SurfaceKind]int{
		decad.KindSphere: 1,
	})
	// The premise the Sphere arm's own doc comment rests on: the face
	// carries no boundary loop at all, and the whole body carries no edge or
	// vertex either.
	require.Empty(t, sheet.Faces()[0].Loops())
	require.Empty(t, sheet.Edges())
	require.Empty(t, sheet.Vertices())

	solid, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(4.0/3.0*math.Pi*125))
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(100*math.Pi))
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

// TestStitchSphereSolidMatchesTheRevolveEngine is T51: the
// independent-producer cross-check, the Sphere arm's own sibling of
// TestStitchAnnularSolidMatchesTheRevolveEngine and
// TestStitchConicalSolidMatchesTheRevolveEngine. The same profile built as a
// BodySolid through Revolve with no option is the Pappus-based analytic
// solid evaluator, never stitch_flux.go's own arithmetic, so agreement here
// does not depend on this PR's own derivation being right.
func TestStitchSphereSolidMatchesTheRevolveEngine(t *testing.T) {
	t.Parallel()
	_, sheet := sphereRevolveSheet(t, 0, 10)
	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)

	s2, p2 := semicircleSketchAt(t, 0, 10)
	solidDoc := decad.New()
	direct, err := solidDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	stitchedVol, err := stitched.Volume()
	require.NoError(t, err)
	directVol, err := direct.Volume()
	require.NoError(t, err)
	decadtest.Agree(t, "volume", stitchedVol, directVol)

	// The centroid comparison below is real but genuinely weak for THIS
	// shape: the direct revolve engine's own centroid bound for a profile
	// touching the axis at both poles comes out on the order of a hundred
	// millimetres for this 5 mm sphere (checked directly while landing this
	// test), an existing property of the unrelated Pappus-based solid
	// evaluator, not of this arm — every Cylinder/Cone cross-check fixture
	// stays clear of the axis and does not hit it. So this leg still catches
	// a gross blunder (a wrong sign, an order-of-magnitude error), but the
	// tight, decisive centroid proof against the hand-derived analytic value
	// is TestStitchSphereRevolveSheetClosesToABall (T50) and
	// TestStitchSphereVolumeBoundEncloses (T52), both `Bound`-tight and both
	// shown to fail against a broken moment formula.
	stitchedCen, err := stitched.Centroid()
	require.NoError(t, err)
	directCen, err := direct.Centroid()
	require.NoError(t, err)
	tol := stitchedCen.Bound.Base() + directCen.Bound.Base() + 1e-9
	require.InDelta(t, directCen.Value.X, stitchedCen.Value.X, tol)
	require.InDelta(t, directCen.Value.Y, stitchedCen.Value.Y, tol)
	require.InDelta(t, directCen.Value.Z, stitchedCen.Value.Z, tol)
}

// TestStitchSphereVolumeBoundEncloses is T52: T50's fixture swept across a
// family of radii and off-origin diameters (the generating semicircle's own
// centre moved along the axis), each checked against its own analytic
// (4/3)*pi*R^3 volume and its own centre as centroid.
func TestStitchSphereVolumeBoundEncloses(t *testing.T) {
	t.Parallel()
	cases := []struct {
		u0, diameter float64
	}{
		{0, 10},
		{0, 6},
		{-20, 200},
		{5, 2},
	}
	for _, c := range cases {
		_, sheet := sphereRevolveSheet(t, c.u0, c.diameter)
		solid, err := decad.Stitch(t.Context(), sheet)
		require.NoError(t, err)

		r := c.diameter / 2
		wantVol := 4.0 / 3.0 * math.Pi * r * r * r
		decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(wantVol))
		decadtest.MeasuresCentroid(t, solid, r3.NewVec(c.u0+r, 0, 0))
	}
}

// revolvedPolygonTruth is the exact volume and centroid of the solid a
// recorded polygon on the world XY plane sweeps in a full turn about the
// world X axis, by Pappus over the recorded vertices: V = 2π∫v dA and
// x̄ = ∫u·v dA / ∫v dA, each a polygon moment in exact rationals. Only π
// rounds, at normalOraclePrec.
func revolvedPolygonTruth(t *testing.T, pts [][2]float64) (*big.Float, onVec) {
	t.Helper()
	sA, sV, sUV := new(big.Rat), new(big.Rat), new(big.Rat)
	two := big.NewRat(2, 1)
	for i := range pts {
		j := (i + 1) % len(pts)
		x0, y0 := new(big.Rat).SetFloat64(pts[i][0]), new(big.Rat).SetFloat64(pts[i][1])
		x1, y1 := new(big.Rat).SetFloat64(pts[j][0]), new(big.Rat).SetFloat64(pts[j][1])
		c := new(big.Rat).Sub(new(big.Rat).Mul(x0, y1), new(big.Rat).Mul(x1, y0))
		sA.Add(sA, c)
		sV.Add(sV, new(big.Rat).Mul(new(big.Rat).Add(y0, y1), c))
		k := new(big.Rat).Add(new(big.Rat).Mul(x0, y1), new(big.Rat).Mul(x1, y0))
		k.Add(k, new(big.Rat).Mul(two, new(big.Rat).Mul(x0, y0)))
		k.Add(k, new(big.Rat).Mul(two, new(big.Rat).Mul(x1, y1)))
		sUV.Add(sUV, new(big.Rat).Mul(k, c))
	}
	if sA.Sign() < 0 {
		sV.Neg(sV)
		sUV.Neg(sUV)
	}
	firstV := new(big.Rat).Quo(sV, big.NewRat(6, 1))
	firstUV := new(big.Rat).Quo(sUV, big.NewRat(24, 1))
	pi, _, err := big.ParseFloat(oraclePiDigits, 10, normalOraclePrec, big.ToNearestEven)
	require.NoError(t, err)
	vol := new(big.Float).SetPrec(normalOraclePrec).Mul(pi, new(big.Float).SetPrec(normalOraclePrec).SetRat(firstV))
	vol.Mul(vol, onF(2))
	xbar := new(big.Float).SetPrec(normalOraclePrec).SetRat(new(big.Rat).Quo(firstUV, firstV))
	return vol, onVec{xbar, onF(0), onF(0)}
}

// requireStitchedMassEncloses requires solid's Volume and Centroid to sit
// within their own bounds of the exact readings.
func requireStitchedMassEncloses(t *testing.T, solid *decad.Body, vol *big.Float, cen onVec) {
	t.Helper()
	v, err := solid.Volume()
	require.NoError(t, err)
	gap := new(big.Float).SetPrec(normalOraclePrec).Sub(onF(v.Value.Base()), vol)
	volGap, _ := gap.Abs(gap).Float64()
	require.LessOrEqualf(t, volGap, v.Bound.Base(), "the volume sits %g off the record's, outside its bound %g", volGap, v.Bound.Base())
	c, err := solid.Centroid()
	require.NoError(t, err)
	cenGap, _ := onOf(c.Value).sub(cen).norm().Float64()
	require.LessOrEqualf(t, cenGap, c.Bound.Base(), "the centroid sits %g off the record's, outside its bound %g", cenGap, c.Bound.Base())
}

// TestStitchRefusesRevolveTagOffItsRecord stitches full-turn revolve sheets
// whose wall tags are not the surfaces their records denote: a side climbing
// 5e-10 over a unit run, tagged a Cylinder, and a side leaning 5e-10 off
// perpendicular, tagged a Plane. The flux arms integrate the tag, so Stitch
// refuses rather than publish the tag's volume (docs/surface-design.md §6.4,
// stitch_flux.go's stitchFluxTagsDenoted).
//
// Shown to fail: without that gate Stitch published volumes 2.1e-9 and
// 1.6e-9 mm³ off the record's, under bounds of 7.0e-15 and 5.7e-15, and
// centroids 1.1e-10 and 4.2e-10 mm off under bounds of 2.8e-15 and 2.6e-15.
func TestStitchRefusesRevolveTagOffItsRecord(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		pts  [][2]float64
	}{
		{"near parallel", [][2]float64{{0, 1}, {1, 1}, {1, 2}, {0, 2 + 5e-10}}},
		{"near perpendicular", [][2]float64{{0, 1}, {1, 1}, {1 + 5e-10, 2}, {0, 2}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, p := polygonSketch(t, tc.pts)
			sheet, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
			require.NoError(t, err)
			solid, err := decad.Stitch(t.Context(), sheet)
			if err == nil {
				vol, cen := revolvedPolygonTruth(t, tc.pts)
				requireStitchedMassEncloses(t, solid, vol, cen)
			}
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

// TestStitchDecimalFrustumEnclosesItsRecord stitches a trapezoid drawn at
// decimal coordinates, two Cylinder and two Cone walls. Its walls ARE the
// surfaces their records denote, but
// each Cone tag's apex is the float z − ρ·Δz/Δρ of its walk, which rounds;
// the Cone arm admits the face and charges how far that apex sits from the
// record's (coneApexDeparture), so the volume and centroid enclose the
// record's own.
func TestStitchDecimalFrustumEnclosesItsRecord(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0.1, 0.2}, {1.1, 0.2}, {0.7, 0.9}, {0.3, 0.9}}
	s, p := polygonSketch(t, pts)
	sheet, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	decadtest.HasSurfaceKinds(t, sheet, map[decad.SurfaceKind]int{decad.KindCone: 2, decad.KindCylinder: 2})
	solid, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)
	vol, cen := revolvedPolygonTruth(t, pts)
	requireStitchedMassEncloses(t, solid, vol, cen)
}
