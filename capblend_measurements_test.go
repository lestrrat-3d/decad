package decad_test

import (
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the area-bound regression suite for patchAreaOf's Cone arm
// (docs/modify-reach-design.md §8.4): before this fix, that arm bounded its
// own closed form with conservativeValueError's unconditional |held|+envelope
// fallback — the documented last resort for a reading no certified bracket
// admits — even though a bracket exists for every factor of the frustum-
// sector formula A = (Δθ/2)·(R0+R1)·√(ΔR²+H²). The published bound then sat
// at up to 15% of the body's own area (a 20x39 mm cylinder cap-chamfer band)
// or 3.8x an apex patch's own value (a reflex-corner star), for a VALUE that
// was already exact to the closed form. coneFrustumAreaBracket
// (capblend_moments.go) replaces the fallback with a certified interval
// wherever one can be built; these tests pin the result tight rather than
// merely present.
//
// Tightness is only half of what a bound owes, so the suite carries the other
// half beside it: a bound may only ever shrink to something that still
// ENCLOSES the residual against the patch the chamfer DENOTES, and the
// side-level rounding a tall sweep commits is the one displacement no
// arithmetic reading of the held patch can see.

// starPrismBody builds an n-point star (alternating outer/inner radius),
// extruded by h: a cap-loop chamfer on it builds n Plane band patches plus n
// reflex apex Cone patches — the same patch mix a gear's own root fillets
// make, without needing a gear profile generator.
func starPrismBody(t *testing.T, points int, ro, ri, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	n := 2 * points
	pts := make([]*sketch.Point, n)
	for i := range n {
		r := ro
		if i%2 == 1 {
			r = ri
		}
		th := 2 * math.Pi * float64(i) / float64(n)
		pts[i] = s.CreatePoint(r*math.Cos(th), r*math.Sin(th))
		s.Fix(pts[i])
	}
	for i := range n {
		s.CreateLine(pts[i], pts[(i+1)%n])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendCircularRimAreaBoundIsTight is the cylinder reproduction the
// defect was reported against: a r10 h8 disk with a 0.5 mm cap chamfer. The
// body's whole reported area bound used to sit at 165.84 mm^2 on a
// 1112.245 mm^2 value (15% of it), entirely on the single chamferCap Cone
// band — every other face was already tight. The value itself was already
// exact to the closed form (a straight cylinder wall plus two disks plus a
// frustum band), so only the bound was loose.
func TestCapBlendCircularRimAreaBoundIsTight(t *testing.T) {
	const R, H, d = 10.0, 8.0, 0.5
	body := circleProfile(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	Rc := R - d
	slant := math.Hypot(R-Rc, d)
	wantArea := math.Pi*R*R + 2*math.Pi*R*(H-d) + math.Pi*(R+Rc)*slant + math.Pi*Rc*Rc

	area, err := chamfered.Area()
	require.NoError(t, err)
	require.InDelta(t, wantArea, area.Value.Mag(), 1e-9)
	require.LessOrEqual(t, area.Bound.Mag(), 1e-9,
		`the body's own area bound must be tight, not the 165.84 mm^2 the unconditional fallback published`)

	band := faceWithRole(t, chamfered, `chamferCap(end,0,0)`)
	bandArea, err := band.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, bandArea.Bound.Mag(), 1e-9,
		`the band patch's own bound must be tight (it used to carry the body's whole 165.84 mm^2)`)
}

// TestCapBlendApexPatchAreaBoundIsTight is the reflex-corner reproduction: a
// 6-point star (ro 10, ri 6, h 6, chamfer 0.5) builds 12 Plane band patches
// (already tight before this fix) plus 6 reflex apex Cone patches, which used
// to publish a bound 3.8x their own value. Every chamferCap face's bound must
// now be tight relative to its own value — the same fix that tightens the
// cylinder's whole-turn band tightens a non-whole-turn Cone patch's frustum
// term identically, since patchAreaOf reads capThAllow the same way in both
// cases.
func TestCapBlendApexPatchAreaBoundIsTight(t *testing.T) {
	const ro, ri, h, d = 10.0, 6.0, 6.0, 0.5
	body := starPrismBody(t, 6, ro, ri, h)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	checked := 0
	for _, f := range chamfered.Faces() {
		isCapPatch := false
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				isCapPatch = true
			}
		}
		if !isCapPatch {
			continue
		}
		area, err := f.Area()
		require.NoError(t, err)
		require.Greater(t, area.Value.Mag(), 0.0)
		checked++
		require.LessOrEqual(t, area.Bound.Mag(), 1e-9*area.Value.Mag(),
			`face with role(s) %v: bound %v must be tight against its own value %v (an apex patch used to publish 3.8x)`,
			f.Origins(), area.Bound.Mag(), area.Value.Mag())
	}
	require.Equal(t, 18, checked, "12 wall patches plus 6 reflex apex patches")

	bodyArea, err := chamfered.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, bodyArea.Bound.Mag(), 1e-8,
		`the star's whole-body area bound must be tight, not the 3.97 mm^2 six apex patches used to carry`)
}

// TestCapBlendConeAreaEnclosesTheDenotedPatch is the enclosure regression that
// bounds the tightening above: a band's side level is the single float sum
// capZ + matSign*d, and a sweep tall enough to round that sum leaves the patch
// the build HOLDS a measurable distance from the patch the chamfer DENOTES.
// The certified frustum bracket lifts H = capZ - sideZ as an exact rational —
// it brackets the held patch — so without a charge for the level's own
// rounding it published 5.81e-14 mm^2 against a 2.32 mm^2 residual here.
//
// A r10 disk swept 1e15 mm with a 0.2 mm cap chamfer is the shape that shows
// it: ulp(1e15) is 0.125, so the side level lands 0.05 mm from where the
// chamfer puts it and the band the build holds is 0.25 mm tall instead of
// 0.2 mm. Nothing refuses the body — SX13's axial collapse gate reads the two
// levels as separate, which they are — so the reading has to be honest about
// the gap instead.
func TestCapBlendConeAreaEnclosesTheDenotedPatch(t *testing.T) {
	const R, H, d = 10.0, 1e15, 0.2
	body := circleProfile(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	band := faceWithRole(t, chamfered, `chamferCap(end,0,0)`)
	area, err := band.Area()
	require.NoError(t, err)

	// The frustum sector the chamfer denotes: the same two radii, at the
	// DENOTED 0.2 mm axial separation rather than the held 0.25 mm one.
	Rc := R - d
	denoted := math.Pi * (R + Rc) * math.Hypot(R-Rc, d)
	residual := math.Abs(area.Value.Mag() - denoted)
	require.Greater(t, residual, 1.0,
		`the premise: the rounded side level really does move whole mm^2 here`)
	require.GreaterOrEqual(t, area.Bound.Mag(), residual,
		`the published bound %v must enclose the %v mm^2 the rounded side level moves`,
		area.Bound.Mag(), residual)
}

// tallPlateWithDiskHole extrudes a 100x100 plate with a disk hole of radius r
// centred at (50, 50) by h. The height is the one thing plateWithDiskHole
// fixes: a hole cap-loop chamfer whose setback exceeds the hole's own radius
// needs a sweep taller than that setback, or SX7 refuses the band for running
// past the far end.
func tallPlateWithDiskHole(t *testing.T, r, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(outer.A)
	s.CreateCircle(s.CreatePoint(50, 50), r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-disk region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendConeAreaEnclosesRoundedRadiusDifference is the reachability
// half of the ΔR enclosure regression whose arithmetic half is
// TestPatchAreaOfEnclosesRoundedRadiusDifference: it builds through Chamfer
// the very patch that test states by hand, so the geometry the bound is
// judged on is a body a caller can actually make.
//
// Only one arm of the cap offset can round the ΔR = R1-R0 subtraction
// patchAreaOf's Cone arm evaluates. The inward arm's cap radius is R-d with
// 0<d<R, which is Sterbenz-exact; the reachable route is the outward arm — a
// clockwise, material-outside circular wall, so a hole or a concave arc,
// whose cap radius is R+d — and it begins rounding at a setback-to-radius
// ratio just above 1. This body is an ordinary millimetre-scale countersink
// at that ratio: a 9.011 mm hole with a 16.501 mm setback, whose 25.512 mm
// cap radius puts fl(R1-R0) an ulp from the difference the two held radii
// denote.
//
// The published bound asserted here is the whole composed one, contourAllow
// among its terms, so it is the looser statement of the two — the built
// contour's own displacement runs several times the arithmetic residual on
// this body. The tight claim is the one the arithmetic makes, and the
// internal test judges that alone.
func TestCapBlendConeAreaEnclosesRoundedRadiusDifference(t *testing.T) {
	const R, d, h = 9.011281351443861, 16.500928618916209, 40.0
	body := tallPlateWithDiskHole(t, R, h)
	q := decad.Edges(decad.CreatedBy(decad.CapEnd(body)), decad.Circular())
	matched, err := q.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, matched, 1, "the hole loop's single whole-circle edge")

	chamfered, err := body.Chamfer(q, units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	// The band's own two radii and axial run, formed exactly as the build
	// forms them: a hole is walked clockwise, so its cap contour offsets
	// OUTWARD to R+d, and the side level is the cap level moved d into the
	// material.
	R0 := R
	R1 := R + d
	capZ := h
	sideZ := capZ - d
	H := capZ - sideZ

	exactDR := new(big.Rat).Sub(new(big.Rat).SetFloat64(R1), new(big.Rat).SetFloat64(R0))
	require.NotEqual(t, 0, new(big.Rat).SetFloat64(R1-R0).Cmp(exactDR),
		`the premise: fl(R1-R0) really does round at this radius and setback`)

	band := capBandPatch(t, chamfered)
	area, err := band.Area()
	require.NoError(t, err)

	// A(true) = (2π/2)·(R0+R1)·√(ΔR²+H²) over the held radii, at 600 bits with
	// the true π rather than the float64 the build swept through — the same
	// window the patch's own capThAllow already brackets.
	const prec = 600
	const piDigits = "3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798"
	bf := func(r *big.Rat) *big.Float { return new(big.Float).SetPrec(prec).SetRat(r) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(a, b) }
	pi, ok := new(big.Float).SetPrec(prec).SetString(piDigits)
	require.True(t, ok)
	slantSq := new(big.Rat).Add(
		new(big.Rat).Mul(exactDR, exactDR),
		new(big.Rat).Mul(new(big.Rat).SetFloat64(H), new(big.Rat).SetFloat64(H)),
	)
	slant := new(big.Float).SetPrec(prec).Sqrt(bf(slantSq))
	radii := bf(new(big.Rat).Add(new(big.Rat).SetFloat64(R0), new(big.Rat).SetFloat64(R1)))
	ref, _ := mul(pi, mul(radii, slant)).Float64()

	residual := math.Abs(area.Value.Mag() - ref)
	require.Greater(t, residual, 0.0,
		`the premise: the held area really does sit off the frustum its own radii denote`)
	require.GreaterOrEqual(t, area.Bound.Mag(), residual,
		`the published bound %v must enclose the %v mm^2 the rounded radius difference moves`,
		area.Bound.Mag(), residual)
}

// capBandPatch returns the body's one chamferCap patch face, the shape a
// single chamfered whole-circle loop builds.
func capBandPatch(t *testing.T, b *decad.Body) *decad.Face {
	t.Helper()
	var found []*decad.Face
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				found = append(found, f)
				break
			}
		}
	}
	require.Len(t, found, 1, "one chamfered loop builds one whole-turn band patch")
	return found[0]
}

// TestCapBlendCircularRimVerifyArea is the cylinder case through the public
// Verify surface: at the default tolerance, the tightened area bound must
// clear the gate the loose 165.84 mm^2 fallback used to fail (required
// 1.112 mm^2 at rel=1e-3).
func TestCapBlendCircularRimVerifyArea(t *testing.T) {
	const R, H, d = 10.0, 8.0, 0.5
	body := circleProfile(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	report, err := chamfered.Document().Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	for _, diag := range report.Diagnostics {
		require.NotEqual(t, decad.ReadingArea, diag.Reading,
			`the area reading must clear the default tolerance now: %+v`, diag)
	}
}

// This file is the A8 regression suite for the cap-loop chamfer's CENTROID
// (docs/modify-reach-design.md §8.4's closed-form first moments): before this
// fix, evalCapBlendContext never computed a first moment at all — it
// published an area-weighted average of each face's own first-loop-start
// VERTEX, bounded only by the body's own bounding box. On the ask's own
// cylinder r10 h8 chamfer 0.5mm fixture that reported (9.8725, 0, 2.3314)
// against the true (0, 0, 3.9881863539) mm, 10.01 mm out, with a 22.96 mm
// bound; the fix computes the real first moments in closed form and reports
// a value within float64 precision of the true one.

// TestCapBlendCircularRimCentroidIsClosedForm is the A8 reproduction, digit
// for digit: a cone frustum's centroid is closed-form, so no sampling is
// needed. The true value is 10.01 mm from what the OLD estimate published;
// this pins the corrected value AND checks Document.Verify now reads Sound
// at the default tolerance, the ask's own acceptance criterion.
func TestCapBlendCircularRimCentroidIsClosedForm(t *testing.T) {
	const R, H, d = 10.0, 8.0, 0.5
	body := circleProfile(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	R1 := R - d
	slabV := math.Pi * R * R * (H - d)
	frusV := math.Pi * d / 3 * (R*R + R*R1 + R1*R1)
	slabCz := (H - d) / 2
	frusCzLocal := d / 4 * (R*R + 2*R*R1 + 3*R1*R1) / (R*R + R*R1 + R1*R1)
	frusCz := (H - d) + frusCzLocal
	wantCz := (slabV*slabCz + frusV*frusCz) / (slabV + frusV)

	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0.0, centroid.Value.X, 1e-9)
	require.InDelta(t, 0.0, centroid.Value.Y, 1e-9)
	require.InDelta(t, wantCz, centroid.Value.Z, 1e-9,
		`the true value is 10.01 mm from what the old estimate published`)
	require.LessOrEqual(t, centroid.Bound.Mag(), 1e-6,
		`the old estimate's bound was 22.96 mm`)

	doc := chamfered.Document()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status,
		`the cylinder document must read Sound at the default tolerance: %+v`, report.Diagnostics)
	require.True(t, report.Trustworthy())
}

// TestCapBlendPlateCentroidIsExactRational is the 100x60 plate h20 chamfer 5
// fixture — an all-Plane band, whose per-patch flux is already exact
// rational (patchRawFlux's own Plane arm), so its first moment is too
// (exactPlanePatchMoment). A rectangular section's own centroid stays at its
// own (u, v) centre under a UNIFORM inward offset (every side moves in by
// the same amount, so the offset rectangle keeps the same centre) —
// (50, 30) for every cross-section, slab and band alike — so only the Z
// coordinate moves, to a value derived independently from the linear-offset
// family the plate's own volume test already uses.
func TestCapBlendPlateCentroidIsExactRational(t *testing.T) {
	_, box := capBlendBox(t)
	const d = 5.0
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	const wantCz = 1595.0 / 164.0 // derived independently, see PR description
	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 50.0, centroid.Value.X, 1e-9)
	require.InDelta(t, 30.0, centroid.Value.Y, 1e-9)
	require.InDelta(t, wantCz, centroid.Value.Z, 1e-9)
	require.LessOrEqual(t, centroid.Bound.Mag(), 1e-6,
		`an all-Plane band's centroid should be exact-rational end to end`)
}

// TestCapBlendHoleLoopCentroid chamfers a HOLE loop directly — the same
// countersink fixture TestCapBlendHoleLoopChamferVolume checks for volume —
// and checks the centroid against an independent closed form: the outer
// slab's centroid less the widened-hole void's own (subtracted) moment. This
// is what catches the per-loop sign and the band's own -matSign*orient
// composition specifically for a hole (li != 0): a wrong sign there flips
// the void from subtracted to added, which volume alone already guards
// against but centroid's own vector sign could still miss independently.
func TestCapBlendHoleLoopCentroid(t *testing.T) {
	const L, H, cx, cy, R, d = 100.0, 10.0, 50.0, 50.0, 10.0, 2.0
	_, box := plateWithDiskHole(t, cx, cy, R)
	q := decad.Edges(decad.CreatedBy(decad.CapEnd(box)), decad.Circular())
	matched, err := q.SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, matched, 1, "the hole loop's single whole-circle edge")

	chamfered, err := box.Chamfer(q, units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	const wantCz = 4.993980401607336 // derived independently, see PR description
	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
	require.InDelta(t, cx, centroid.Value.X, 1e-6)
	require.InDelta(t, cy, centroid.Value.Y, 1e-6)
	require.InDelta(t, wantCz, centroid.Value.Z, 1e-6)
	require.LessOrEqual(t, centroid.Bound.Mag(), 1e-3)
}

// TestCapBlendStartCapCentroidMirrorsEndCap is the centroid sibling of
// TestCapBlendStartCapVolumeMatchesEndCap: chamfering the START cap must give
// the same Z-symmetric result as chamfering the END cap (measured from the
// opposite end), catching the -matSign half of capBandMoment's own sign pair
// — the half no single-cap fixture can see (docs/modify-reach-design.md
// §8.4's "Signs").
func TestCapBlendStartCapCentroidMirrorsEndCap(t *testing.T) {
	for _, tc := range []struct {
		name  string
		build func(t *testing.T) *decad.Body
		d     float64
	}{
		{`rectangular box`, func(t *testing.T) *decad.Body { _, b := capBlendBox(t); return b }, 5},
		{`partially-swept arc (quarter disk)`, func(t *testing.T) *decad.Body { return quarterDiskBody(t, 45, 15) }, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			endBody := tc.build(t)
			endChamfered, err := endBody.Chamfer(capLoopEdgesOn(endBody, true), units.Millimeters(tc.d))
			require.NoError(t, err)
			endCentroid, err := endChamfered.Centroid()
			require.NoError(t, err)
			endBounds, err := endChamfered.Bounds()
			require.NoError(t, err)

			startBody := tc.build(t)
			startChamfered, err := startBody.Chamfer(capLoopEdgesOn(startBody, false), units.Millimeters(tc.d))
			require.NoError(t, err)
			startCentroid, err := startChamfered.Centroid()
			require.NoError(t, err)
			startBounds, err := startChamfered.Bounds()
			require.NoError(t, err)

			// The start-chamfered body is the end-chamfered one reflected
			// through the sweep's own mid-height (both share the same
			// [z0, z1] interval and the same in-plane section), so X and Y
			// match directly and Z mirrors about the shared mid-height.
			midZ := (endBounds.Min.Z + endBounds.Max.Z) / 2
			require.InDelta(t, startBounds.Min.Z, endBounds.Min.Z, 1e-9)
			require.InDelta(t, startBounds.Max.Z, endBounds.Max.Z, 1e-9)

			require.InDelta(t, endCentroid.Value.X, startCentroid.Value.X, 1e-6)
			require.InDelta(t, endCentroid.Value.Y, startCentroid.Value.Y, 1e-6)
			mirroredEndZ := 2*midZ - endCentroid.Value.Z
			require.InDelta(t, mirroredEndZ, startCentroid.Value.Z, 1e-6)
		})
	}
}

// TestCapBlendReflexCornerCentroidBoundIsTight is the star's own reproduction
// of the gear's reflex root corners (the same fixture
// TestCapBlendApexPatchAreaBoundIsTight uses): before this fix, a body this
// size (14-21 mm across) published a centroid bound of 17.22 mm — larger
// than the body itself. The general Cone arm's own bound
// (conservativeValueError against its structural envelope) is a documented
// loose term, but it must still be a decisive, order-of-magnitude
// improvement, never above the old figure (the math.Min ceiling), and it
// must still ENCLOSE the true centroid.
func TestCapBlendReflexCornerCentroidBoundIsTight(t *testing.T) {
	const ro, ri, h, d = 10.0, 6.0, 6.0, 0.5
	body := starPrismBody(t, 6, ro, ri, h)
	// The receiver's own centroid is close to, but not exactly, the
	// chamfered body's true one (the chamfer removes a shallow band near
	// one cap) — a coarse sanity check on the VALUE, independent of the
	// tight bound assertion below.
	receiver, err := body.Centroid()
	require.NoError(t, err)

	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
	require.Less(t, centroid.Bound.Mag(), 17.2237,
		`must be a decisive improvement over the pre-fix bound`)
	require.InDelta(t, receiver.Value.X, centroid.Value.X, 1.0)
	require.InDelta(t, receiver.Value.Y, centroid.Value.Y, 1.0)

	// Never above the geometry-net ceiling — proven sound regardless of the
	// general Cone arm's own (documented loose) envelope.
	bounds, err := chamfered.Bounds()
	require.NoError(t, err)
	net := 0.0
	for _, x := range []float64{bounds.Min.X, bounds.Max.X} {
		for _, y := range []float64{bounds.Min.Y, bounds.Max.Y} {
			for _, z := range []float64{bounds.Min.Z, bounds.Max.Z} {
				corner := r3.NewVec(x, y, z)
				if dd := corner.Sub(centroid.Value).Len(); dd > net {
					net = dd
				}
			}
		}
	}
	net += bounds.Bound.Mag()
	require.LessOrEqual(t, centroid.Bound.Mag(), net)
}

// This file is the PLACEMENT half of a band patch's surface-departure bound
// (docs/modify-reach-design.md §8.3). The other half — a non-tangential miter
// corner, which leaves the two directrices sweeping different windows — is
// covered in capblend_survey_test.go. This one covers the half that has nothing
// to do with the windows at all: every world coordinate the build emits is a
// ROUNDED image of the plane-local number it denotes, and the roundings are
// independent, so a band whose two windows genuinely coincide still leaves its
// own `Cone` tag once it is placed. A bound read off the plane-local windows
// alone is identical placed or not and publishes a zero there, which is an
// assertion rather than a measurement.
//
// Every reading below is taken from HELD numbers only: a ruling's own two
// endpoint vertices, a directrix's own centre, and the tag's own origin. Each
// difference is between two coordinates within a factor of two of one another,
// so the subtraction is exact and the reconstruction carries no cancellation of
// its own to be confused with the departure it measures. Nothing is sampled by
// finite differences and nothing is evaluated at a point the test computed.

// bandRulingEnd is one corner of a chamfer band's built ruled surface: the
// vertex itself, the directrix passing through it, and the straight ruling
// leaving it. The two together span the built surface's own tangent plane
// there, which is what the patch's published `Cone` is judged against.
type bandRulingEnd struct {
	at      r3.Vec
	center  r3.Vec
	axis    r3.Vec
	rulingA r3.Vec
	rulingB r3.Vec
}

// tangentPlaneNormal is the built surface's own unit normal at this corner: the
// directrix tangent (axis crossed into the radial arm) crossed into the ruling.
// The sign is taken from the published normal, which may decide it — the two
// are far closer than a right angle everywhere this file reads them, so the
// sign is never the thing under test.
func (e bandRulingEnd) tangentPlaneNormal(t *testing.T, published r3.Vec) r3.Vec {
	t.Helper()
	tangent := e.axis.Cross(e.at.Sub(e.center))
	n, ok := tangent.Cross(e.rulingB.Sub(e.rulingA)).Normalize()
	require.True(t, ok, "the built ruled patch has a tangent plane at its own corner")
	if n.Dot(published) < 0 {
		n = n.Scale(-1)
	}
	return n
}

// generatorSine is how far the published straight ruling leaves the published
// cone's own generator through this corner — zero exactly when the ruling IS a
// generator, which is what an unrounded build gives and what the `Cone` tag
// claims.
func (e bandRulingEnd) generatorSine(origin r3.Vec) float64 {
	ruling := e.rulingB.Sub(e.rulingA)
	generator := e.at.Sub(origin)
	return ruling.Cross(generator).Len() / (ruling.Len() * generator.Len())
}

// bandRulingEnds reads every corner of one circular band patch off its OWN
// public boundary: two `Arc3` directrices and two `Line3` rulings, paired
// through the boundary vertices they share.
func bandRulingEnds(t *testing.T, f *decad.Face) []bandRulingEnd {
	t.Helper()
	type arcEdge struct {
		arc        decad.Arc3
		start, end *decad.Vertex
	}
	var arcs []arcEdge
	var lines []*decad.Edge
	for _, ce := range f.Loops()[0].CoEdges() {
		e := ce.Edge()
		switch c := e.Curve().(type) {
		case decad.Arc3:
			arcs = append(arcs, arcEdge{arc: c, start: e.Start(), end: e.End()})
		case decad.Line3:
			lines = append(lines, e)
		}
	}
	require.Len(t, arcs, 2, "a circular band patch runs between two circular directrices")
	require.Len(t, lines, 2, "and is bounded by two straight rulings")

	holder := func(v *decad.Vertex) (arcEdge, bool) {
		for _, a := range arcs {
			if a.start == v || a.end == v {
				return a, true
			}
		}
		return arcEdge{}, false
	}
	var out []bandRulingEnd
	for _, ln := range lines {
		a, okA := holder(ln.Start())
		b, okB := holder(ln.End())
		require.True(t, okA && okB, "each ruling joins one directrix to the other")
		for _, side := range []struct {
			v *decad.Vertex
			a arcEdge
		}{{ln.Start(), a}, {ln.End(), b}} {
			out = append(out, bandRulingEnd{
				at:      side.v.Position().Value,
				center:  side.a.arc.Center,
				axis:    side.a.arc.Axis,
				rulingA: ln.Start().Position().Value,
				rulingB: ln.End().Position().Value,
			})
		}
	}
	return out
}

// bandQuadCorners reads one FLAT band patch's four built corners off its own
// public boundary, in walk order: the wall's side-level segment, one ruling,
// the cap-level segment the offset denotes, and the other ruling. Consecutive
// triples give the four corner normals of the surface the build rules between
// them, which are also the normals of the four triangles either diagonal
// splits the quad into — so nothing here picks a triangulation, and nothing
// re-derives a production formula.
func bandQuadCorners(t *testing.T, f *decad.Face) []r3.Vec {
	t.Helper()
	require.Len(t, f.Loops(), 1)
	co := f.Loops()[0].CoEdges()
	require.Len(t, co, 4, "a flat band patch is a quad")
	out := make([]r3.Vec, 0, 4)
	for _, ce := range co {
		require.IsType(t, decad.Line3{}, ce.Edge().Curve(), "every edge of a flat band patch is straight")
		out = append(out, ce.Start().Position().Value)
	}
	return out
}

// bandFlatPatches is every FLAT chamfer band patch of a chamfered body — the
// role prefix is what separates them from the body's other planar faces, which
// the chamfer only trimmed.
func bandFlatPatches(t *testing.T, b *decad.Body) []*decad.Face {
	t.Helper()
	var out []*decad.Face
	for _, f := range b.Faces() {
		if f.Surface().Kind() != decad.KindPlane {
			continue
		}
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				out = append(out, f)
				break
			}
		}
	}
	require.NotEmpty(t, out)
	return out
}

// bandConePatches is every circular chamfer band patch of a chamfered body.
func bandConePatches(t *testing.T, b *decad.Body) []*decad.Face {
	t.Helper()
	var out []*decad.Face
	for _, f := range b.Faces() {
		if f.Surface().Kind() != decad.KindCone {
			continue
		}
		out = append(out, f)
	}
	require.NotEmpty(t, out)
	return out
}

// tangentFilletChamfer is the finding's own fixture: an l x w rectangle swept
// h, all four lateral corners filleted to radius r — so every join between a
// straight wall and a circular one is TANGENT and the two directrices of every
// band patch sweep the same window — then chamfered on its end cap.
//
// The section is drawn at the sketch ORIGIN and carried out by the placement.
// Drawing a small section at large sketch coordinates leaves the arrangement's
// own weld band a handful of ulps of margin, which each platform's arithmetic
// can land either side of; a placement displaces the built coordinates exactly
// as drawing it out there would, and welds nothing.
func tangentFilletChamfer(t *testing.T, motion *r3.Transform) *decad.Body {
	t.Helper()
	const l, w, h, r, d = 8.0, 6.0, 4.0, 1.0, 0.25
	sk := sketch.NewWorld()
	s, err := sk.CreateSketch(sk.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, l, w)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	rounded, err := box.Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()), units.Millimeters(r))
	require.NoError(t, err)
	chamfered, err := rounded.Chamfer(capLoopEdges(rounded), units.Millimeters(d))
	require.NoError(t, err)
	if motion == nil {
		return chamfered
	}
	placed, err := chamfered.Placed(*motion)
	require.NoError(t, err)
	return placed
}

// tangentBandMotions is the three placements this file reads the same band
// under: none, a rotation about the world origin, and that rotation carried far
// out. The rounding of a placed coordinate scales with its own magnitude, so
// the departure the middle case shows is small and the far one's is large,
// which is exactly the difference a plane-local bound cannot see.
func tangentBandMotions(t *testing.T) (r3.Transform, r3.Transform) {
	t.Helper()
	spin, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1e5, -3e5, 7e4))
	require.NoError(t, err)
	far, err := spin.Then(shift)
	require.NoError(t, err)
	return spin, far
}

// TestCapBlendPlacedTangentBandNormalCarriesItsOwnBound is this file's central
// claim. A tangent-join band's own windows coincide, so its patches carry no
// window skew at all — and its built ruled surface still leaves the `Cone` it
// publishes, because the rulings' endpoints, the directrices' centres and the
// tag's own origin are separately rounded images of the placed frame. The
// published `Face.NormalAt` bound must enclose that departure at every corner
// of every patch, under every placement.
func TestCapBlendPlacedTangentBandNormalCarriesItsOwnBound(t *testing.T) {
	spin, far := tangentBandMotions(t)
	for _, tc := range []struct {
		name   string
		motion *r3.Transform
	}{
		{name: `unplaced`},
		{name: `rotated about the origin`, motion: &spin},
		{name: `rotated and placed far out`, motion: &far},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := tangentFilletChamfer(t, tc.motion)
			checked := 0
			for _, f := range bandConePatches(t, body) {
				cone, ok := f.Surface().(decad.Cone)
				require.True(t, ok)
				for _, end := range bandRulingEnds(t, f) {
					published, err := f.NormalAt(end.at)
					require.NoError(t, err)
					built := end.tangentPlaneNormal(t, published.Value)
					gap := built.Sub(published.Value).Len()
					require.LessOrEqual(t, gap, published.Bound.Mag(),
						"the built ruled surface's normal is %v from the published %v, past the bound %v it publishes (generator sine %v)",
						built, published.Value, published.Bound.Mag(), end.generatorSine(cone.Origin))
					checked++
				}
			}
			require.Equal(t, 16, checked, "four tangent fillets, two rulings each, two ends per ruling")
		})
	}
}

// flatCornerDefectSq is one corner of a flat band patch's own built surface,
// measured against the `Plane` the patch publishes: the exact squared sine
// between the two directions, taken over rationals from the three held corners
// that meet there and the tag's own held frame normal. The build fixes that
// plane through THREE of the four corners, so the corner normals are the one
// reading that can see the fourth leave it.
func flatCornerDefectSq(corners []r3.Vec, i int, published r3.Vec) *big.Rat {
	at := ratVecOf(corners[i])
	next := ratVecSub(ratVecOf(corners[(i+1)%len(corners)]), at)
	prev := ratVecSub(ratVecOf(corners[(i+len(corners)-1)%len(corners)]), at)
	return perpDefectSq(published, ratVecCross(next, prev))
}

// TestCapBlendPlacedFlatPatchNormalCarriesItsOwnBound is the tangent band's
// claim on the band's OTHER patch kind. A straight wall's offset stays parallel
// to it in exact arithmetic, so the surface a flat patch DENOTES really is its
// tag's plane — and the four corners the build emits are each rounded once
// more, while the tag is fixed through only three of them, so the quad the
// build actually rules still leaves that plane. The published `Face.NormalAt`
// bound must enclose the departure at every corner of every patch, under every
// placement.
func TestCapBlendPlacedFlatPatchNormalCarriesItsOwnBound(t *testing.T) {
	spin, far := tangentBandMotions(t)
	for _, tc := range []struct {
		name   string
		motion *r3.Transform
	}{
		{name: `unplaced`},
		{name: `rotated about the origin`, motion: &spin},
		{name: `rotated and placed far out`, motion: &far},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := tangentFilletChamfer(t, tc.motion)
			checked := 0
			for _, f := range bandFlatPatches(t, body) {
				corners := bandQuadCorners(t, f)
				for i, corner := range corners {
					published, err := f.NormalAt(corner)
					require.NoError(t, err)
					requireBoundCoversDefect(t, published.Bound.Mag(),
						flatCornerDefectSq(corners, i, published.Value), tc.name)
					checked++
				}
			}
			require.Equal(t, 16, checked, "four straight walls, four corners each")
		})
	}
}

// TestCapBlendPlacedFlatPatchDepartureGrowsWithPlacement is the flat patch's
// half of the causal path below: the SAME wall, whose plane-local offset is
// byte for byte the same under every placement, rules a quad that leaves its
// own `Plane` tag by orders more once the placement carries it far out. A bound
// asserted from the plane-local offset alone would be zero in all three rows.
func TestCapBlendPlacedFlatPatchDepartureGrowsWithPlacement(t *testing.T) {
	spin, far := tangentBandMotions(t)
	worst := func(motion *r3.Transform) (float64, float64) {
		body := tangentFilletChamfer(t, motion)
		gap, bound := 0.0, 0.0
		for _, f := range bandFlatPatches(t, body) {
			corners := bandQuadCorners(t, f)
			for i, corner := range corners {
				published, err := f.NormalAt(corner)
				require.NoError(t, err)
				sq, _ := flatCornerDefectSq(corners, i, published.Value).Float64()
				gap = math.Max(gap, math.Sqrt(sq))
				bound = math.Max(bound, published.Bound.Mag())
			}
		}
		return gap, bound
	}

	flatGap, flatBound := worst(nil)
	spunGap, spunBound := worst(&spin)
	farGap, farBound := worst(&far)

	// Audited: unplaced a gap of 0 under a bound of 1.1e-16; rotated at the
	// origin 1.8e-15 under 4.0e-15; rotated and far out 1.0e-10 under 2.1e-10.
	require.Less(t, flatGap, 1e-15,
		"an unplaced build's quad is planar to within its own arithmetic")
	require.Greater(t, farGap, 1e-13, "a placed build's leaves its own tag")
	require.Greater(t, farGap, 1e3*spunGap, "by orders more once the placement carries it far out")
	require.GreaterOrEqual(t, spunBound, flatBound)
	require.Greater(t, farBound, 1e3*spunBound)
	require.Greater(t, farBound, farGap)
}

// TestCapBlendPlacedTangentBandDepartureGrowsWithPlacement pins the causal path
// the bound above has to charge for. The SAME band, whose plane-local windows
// are byte for byte the same under every placement, leaves its `Cone` tag by
// orders more once it is placed — measured two independent ways, on held
// coordinates alone: the built tangent plane's own normal against the published
// one, and the published straight ruling against the published cone's own
// generator through the same corner. A bound derived from the windows would be
// unmoved across these three rows, and zero in all of them.
func TestCapBlendPlacedTangentBandDepartureGrowsWithPlacement(t *testing.T) {
	spin, far := tangentBandMotions(t)
	worst := func(motion *r3.Transform) (float64, float64, float64) {
		body := tangentFilletChamfer(t, motion)
		normalGap, generator, bound := 0.0, 0.0, 0.0
		for _, f := range bandConePatches(t, body) {
			cone, ok := f.Surface().(decad.Cone)
			require.True(t, ok)
			for _, end := range bandRulingEnds(t, f) {
				published, err := f.NormalAt(end.at)
				require.NoError(t, err)
				built := end.tangentPlaneNormal(t, published.Value)
				normalGap = math.Max(normalGap, built.Sub(published.Value).Len())
				generator = math.Max(generator, end.generatorSine(cone.Origin))
				bound = math.Max(bound, published.Bound.Mag())
			}
		}
		return normalGap, generator, bound
	}

	_, flatGen, flatBound := worst(nil)
	spunGap, spunGen, spunBound := worst(&spin)
	farGap, farGen, farBound := worst(&far)

	// Audited: unplaced generator sine 6.3e-16 and bound 1.5e-15; rotated at the
	// origin 3.2e-15 and 1.2e-14; rotated and far out 1.6e-10 and 2.7e-10, with
	// the built surface's own normal 7.4e-11 from the published one there.
	require.Less(t, flatGen, 1e-14,
		"an unplaced build's rulings are the cone's own generators to within its own arithmetic")
	require.Greater(t, farGen, 1e3*spunGen,
		"a placed build's leave the generator by orders more once the placement carries the band far out")
	require.Greater(t, farGap, 1e-11, "and the built surface's own normal leaves the published one with it")
	require.Greater(t, farGap, 1e3*spunGap, "at the same scaling")
	// The published bound has to follow, or a decision taken against the
	// published direction is decided against a direction the surface has not
	// got. Bounds that did not move across these rows would be the defect this
	// file exists for.
	require.Greater(t, spunBound, flatBound)
	require.Greater(t, farBound, 1e3*spunBound)
	require.Greater(t, farBound, farGap)
}

// This file is the STRAIGHT-SLANT RULED PATCH defect's own test suite
// (docs/modify-reach-design.md §8.3): a circular cap-loop-chamfer wall that
// meets a neighbour at a non-tangential miter corner has a CAP-level
// directrix (the offset arc, trimmed at the mitered corner feet) whose
// angular window differs from its SIDE-level directrix (the wall's own
// recorded sweep, untrimmed) — capblend_geom.go/capblend_moments.go used to
// read the side window for both, which moved Body.Volume() by up to 0.7% at
// a 7.24e-10 published bound. Affected exactly when a circular wall meets a
// non-tangential neighbour; unaffected and covered here too: a whole-turn
// circle, tangent junctions, and reflex corners.

// TestCapBlendCapLevelArcLengthMatchesGeometry is defect-2's public shape: a
// cap-level Arc3 edge's own Length() must equal Radius times the angle
// between its own Start() and End() (about the Arc3's own Center), within
// its own Bound — reference-free, since Start/End/Center/Radius are all
// public readings of the SAME edge, never a second construction.
// capblend_geom.go used to hold this edge's length as
// capRadius*|w.th1-w.th0| — the WALL's own full recorded sweep — rather than
// the trimmed cap-level sweep between the actual corner feet the edge runs
// between, so the held length disagreed with the edge's own endpoints.
func TestCapBlendCapLevelArcLengthMatchesGeometry(t *testing.T) {
	const R, H, d = 60.0, 20.0, 4.0
	body := quarterDiskBody(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	checked := 0
	for _, e := range chamfered.Edges() {
		arc, ok := e.Curve().(decad.Arc3)
		if !ok {
			continue
		}
		start := e.Start().Position().Value
		end := e.End().Position().Value
		if start.Z != H || end.Z != H {
			continue // not the chamfer band's cap-level directrix
		}
		checked++

		center := arc.Center
		radius := arc.Radius.Mag()
		a0 := math.Atan2(start.Y-center.Y, start.X-center.X)
		a1 := math.Atan2(end.Y-center.Y, end.X-center.X)
		want := radius * math.Abs(a1-a0)

		length, err := e.Length()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(length.Value.Mag()-want), length.Bound.Mag(),
			`cap-level arc length %v mm must enclose radius*angle(start,end) = %v mm within its own bound %v mm`,
			length.Value.Mag(), want, length.Bound.Mag())
	}
	require.Equal(t, 1, checked, `one cap-level arc: the chamfered quarter-circle wall`)
}

// simpson1D is a fixed-resolution composite Simpson's rule integrator, test
// code only: every reference below judges a residual of several mm^3 against
// a numerical integral, so float64 quadrature at this resolution (relative
// error many orders of magnitude below what it is judging) is more than
// enough — this is an independent REFERENCE, not a proof obligation the
// production code carries.
func simpson1D(f func(float64) float64, a, b float64, n int) float64 {
	if n%2 != 0 {
		n++
	}
	h := (b - a) / float64(n)
	sum := f(a) + f(b)
	for i := 1; i < n; i++ {
		x := a + float64(i)*h
		if i%2 == 0 {
			sum += 2 * f(x)
		} else {
			sum += 4 * f(x)
		}
	}
	return sum * h / 3
}

// sectorArea is a symmetric circular sector's (radius R, full angle phi)
// own area once its boundary — the arc AND the two straight radii through
// the centre — is eroded inward by t: the closed form the task's own design
// note states, A(t) = 2*integral[asin(t/(R-t)), phi/2] of
// ((R-t)^2 - (t/sin(theta))^2)/2 dtheta.
func sectorArea(R, phi, t float64) float64 {
	if t <= 0 {
		return R * R * phi / 2
	}
	lo := math.Asin(t / (R - t))
	hi := phi / 2
	if lo >= hi {
		return 0
	}
	f := func(th float64) float64 {
		s := t / math.Sin(th)
		return ((R-t)*(R-t) - s*s) / 2
	}
	return 2 * simpson1D(f, lo, hi, 4000)
}

// sectorErosionVolume is the erosion-family reference volume for a straight
// prism over a symmetric circular sector (radius R, full angle phi), swept
// h and chamfered by setback d on one cap: the straight slab below the band
// plus the band's own volume as the integral, over the eroded depth t in
// [0, d], of the sector's own area at that erosion.
func sectorErosionVolume(R, phi, h, d float64) float64 {
	slab := sectorArea(R, phi, 0) * (h - d)
	band := simpson1D(func(t float64) float64 { return sectorArea(R, phi, t) }, 0, d, 4000)
	return slab + band
}

// segmentArea is a minor circular segment's (radius R, chord at v = chordV)
// own area once its boundary is eroded inward by t: the standard closed form
// for a circular segment's area, r^2*acos(h/r) - h*sqrt(r^2-h^2), with the
// eroded radius r = R-t and the eroded chord height h = chordV+t.
func segmentArea(R, chordV, t float64) float64 {
	r := R - t
	h := chordV + t
	if h >= r {
		return 0
	}
	return r*r*math.Acos(h/r) - h*math.Sqrt(r*r-h*h)
}

// segmentErosionVolume is the erosion-family reference volume for a straight
// prism over a minor circular segment (radius R, chord at v = chordV), swept
// h and chamfered by setback d on one cap.
func segmentErosionVolume(R, chordV, h, d float64) float64 {
	slab := segmentArea(R, chordV, 0) * (h - d)
	band := simpson1D(func(t float64) float64 { return segmentArea(R, chordV, t) }, 0, d, 4000)
	return slab + band
}

// circularSegmentBody extrudes the minor circular segment cut from a
// radius-r disk (centred at the origin) by the chord v = chordV — a straight
// chord meeting the arc at two non-tangential miter corners — by h.
func circularSegmentBody(t *testing.T, r, chordV, h float64) *decad.Body {
	t.Helper()
	x0 := math.Sqrt(r*r - chordV*chordV)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	p1 := s.CreatePoint(-x0, chordV)
	p2 := s.CreatePoint(x0, chordV)
	s.CreateLine(p1, p2)
	s.CreateArc(o, p2, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// circularSectorBody extrudes a symmetric circular sector — two straight
// radii from the origin at angle 0 and phi, closed by the arc between their
// far ends — by h: a single circular wall meeting a straight neighbour at a
// non-tangential miter corner at BOTH its own ends, generalizing
// quarterDiskBody to any full angle phi.
func circularSectorBody(t *testing.T, r, phi, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(r, 0)
	py := s.CreatePoint(r*math.Cos(phi), r*math.Sin(phi))
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendErosionFamilyVolumeBoundEncloses is defect-1's own value check
// (docs/modify-reach-design.md §8.3): the published volume judged against the
// erosion-family integral A(0)*(H-d) + integral[0,d] of A(t) dt, an
// INDEPENDENT reference derived from the section alone (never from the
// evaluator's own construction) for shapes whose circular wall meets a
// straight neighbour at a non-tangential miter on both ends — the quarter
// disk, a minor circular segment cut by a chord well off-centre, and two
// sectors near the setback's own limit (an audited PR-122 repro: the setback
// approaching the section's own inradius makes the cap window close hard
// against the side one — capblend_moments.go's chordLocusResidualAllow judges
// exactly this residual). Every residual is the gap between the BUILT
// (straight-ruled-patch) solid and the exact miter-locus solid the erosion
// family denotes (docs/modify-reach-design.md §8.3's own scope note,
// capblend_moments.go); the published Bound must enclose it.
func TestCapBlendErosionFamilyVolumeBoundEncloses(t *testing.T) {
	for _, tc := range []struct {
		name    string
		build   func(t *testing.T) *decad.Body
		erosion func() float64
		d       float64
	}{
		{
			name:    `quarter disk`,
			build:   func(t *testing.T) *decad.Body { return quarterDiskBody(t, 60, 20) },
			erosion: func() float64 { return sectorErosionVolume(60, math.Pi/2, 20, 4) },
			d:       4,
		},
		{
			name:    `minor circular segment, chord v=30`,
			build:   func(t *testing.T) *decad.Body { return circularSegmentBody(t, 60, 30, 20) },
			erosion: func() float64 { return segmentErosionVolume(60, 30, 20, 4) },
			d:       4,
		},
		{
			name:    `quarter disk near the setback limit`,
			build:   func(t *testing.T) *decad.Body { return circularSectorBody(t, 10, math.Pi/2, 4.2) },
			erosion: func() float64 { return sectorErosionVolume(10, math.Pi/2, 4.2, 4.13) },
			d:       4.13,
		},
		{
			name:    `wide sector near the setback limit`,
			build:   func(t *testing.T) *decad.Body { return circularSectorBody(t, 10, 2.7, 4.953329) },
			erosion: func() float64 { return sectorErosionVolume(10, 2.7, 4.953329, 4.928686) },
			d:       4.928686,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := tc.build(t)
			chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(tc.d))
			require.NoError(t, err)
			vol, err := chamfered.Volume()
			require.NoError(t, err)

			erosion := tc.erosion()
			residual := math.Abs(vol.Value.Mag() - erosion)
			require.LessOrEqual(t, residual, vol.Bound.Mag(),
				`the published volume bound (%v mm^3) must enclose the residual (%v mm^3) against the erosion-family reference (held %v mm^3, erosion %v mm^3)`,
				vol.Bound.Mag(), residual, vol.Value.Mag(), erosion)
		})
	}
}

// TestCapBlendChordLocusVolumeAllowScalesSweptTermToFlux is the units-mismatch
// regression for bounds.go's chordLocusVolumeAllow (PR-122 review): the
// function composes envelopeSlack (a difference of two patchRawFlux results —
// raw FLUX, three times a volume) with sweptVolumeAllow(patchDeviation,
// areaUpper) (already a VOLUME), and capBandVolume divides the composed sum
// by 3 exactly once (capblend_moments.go's boundedQuotient at its own single
// division site). Composing the two without first scaling the volume term up
// to flux charged it at a third of its proven size — a shortfall that grows
// as the setback approaches the section's own inradius, which is exactly the
// case pinned here (the audited PR-122 wide-sector repro, near the setback
// limit). Pre-fix (swept charged as a bare volume) this body's published
// bound was 2383.4693377126996 mm^3; correctly scaled it was
// 5796.19196853153 mm^3 at the time of that fix.
//
// It reads 5675.434906518183 mm^3 now: chordLocusResidualAllow feeds this
// patch's own patchAreaOf(g) result into chordLocusVolumeAllow as areaUpper,
// and patchAreaOf's Cone arm has since traded its unconditional envelope for
// a certified interval bracket on the frustum-sector area formula wherever
// one can be built (capThAllow, capblend_contour.go), so areaUpper is
// tighter and this term shrinks with it — soundly, since a smaller PROVEN
// upper bound on the same area still bounds the same swept volume. This test
// still fails if the term regresses to the under-scaled (pre-PR-122) reading.
func TestCapBlendChordLocusVolumeAllowScalesSweptTermToFlux(t *testing.T) {
	body := circularSectorBody(t, 10, 2.7, 4.953329)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(4.928686))
	require.NoError(t, err)
	vol, err := chamfered.Volume()
	require.NoError(t, err)

	const preFixBound = 2383.4693377126996
	require.Greater(t, vol.Bound.Mag(), preFixBound,
		`the published bound (%v mm^3) must exceed the pre-fix under-scaled reading (%v mm^3): the swept-volume term must be scaled to flux before capBandVolume's own /3, not composed as a bare volume`,
		vol.Bound.Mag(), preFixBound)
	require.InDelta(t, 5675.434906518183, vol.Bound.Mag(), 0.01,
		`the published bound (%v mm^3) must match the correctly-scaled reading`, vol.Bound.Mag())

	erosion := sectorErosionVolume(10, 2.7, 4.953329, 4.928686)
	residual := math.Abs(vol.Value.Mag() - erosion)
	require.LessOrEqual(t, residual, vol.Bound.Mag(),
		`the published bound (%v mm^3) must still enclose the erosion-family residual (%v mm^3)`,
		vol.Bound.Mag(), residual)
}

// roundedRectBody extrudes an l x w rectangle by h and fillets its four
// lateral corners to radius r — a TRUE tangent fillet, so every join between
// a straight wall and a circular wall is tangential.
func roundedRectBody(t *testing.T, l, w, h, r float64) *decad.Body {
	t.Helper()
	sk := sketch.NewWorld()
	s, err := sk.CreateSketch(sk.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, l, w)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	lateral := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	rounded, err := box.Fillet(lateral, units.Millimeters(r))
	require.NoError(t, err)
	return rounded
}

// roundedRectErosionVolume is the erosion-family reference for a rounded
// rectangle (fillet radius r, tangent corners): the eroded section at depth
// t is a smaller rounded rectangle, (l-2t) x (w-2t) with fillet radius
// (r-t) — a closed form, no numerical integration needed, since a tangent
// fillet's own offset family is elementary.
func roundedRectErosionVolume(l, w, h, r, d float64) float64 {
	area0 := l*w - (4-math.Pi)*r*r
	slab := area0 * (h - d)
	rectBand := d * (l*w - (l+w)*d + (4.0/3.0)*d*d)
	filletBand := (4 - math.Pi) / 3 * (r*r*r - (r-d)*(r-d)*(r-d))
	return slab + rectBand - filletBand
}

// roundedRectHoleBody extrudes plateWithRectHole's own 100x100 plate with a
// side x side square hole (100 mm plate, 10 mm sweep, both fixed by that
// helper) and then fillets the hole's own four lateral edges to radius
// rho — a TRUE tangent fillet, so the D-HOLE this leaves (like a rounded
// rectangle's, but on a CW hole loop rather than an outer one) meets every
// straight wall and its neighbouring round wall tangentially.
func roundedRectHoleBody(t *testing.T, side, rho float64) *decad.Body {
	t.Helper()
	plate := plateWithRectHole(t, side)
	hole := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave())
	rounded, err := plate.Fillet(hole, units.Millimeters(rho))
	require.NoError(t, err)
	return rounded
}

// roundedRectHoleErosionVolume is the erosion-family reference for
// roundedRectHoleBody, BOTH the plate's own outer loop and the hole's rim
// chamfered together: the outer boundary shrinks (L-2t) x (L-2t) exactly as
// a plain rectangle's own band already does (TestCapBlendChamferPolygonLoop's
// closed form), while the rounded-rectangle hole WIDENS as the chamfer erodes
// into the material — its own side and fillet radius both growing by t, the
// same tangent-offset family a rounded rectangle's OWN outer erosion uses
// (roundedRectErosionVolume), just widening rather than shrinking.
func roundedRectHoleErosionVolume(plateL, side, rho, h, d float64) float64 {
	holeArea := func(t float64) float64 {
		s, r := side+2*t, rho+t
		return s*s - (4-math.Pi)*r*r
	}
	area0 := plateL*plateL - holeArea(0)
	slab := area0 * (h - d)
	rectBand := d * (plateL*plateL - 2*plateL*d + (4.0/3.0)*d*d)
	holeSideBand := d * (side*side + 2*side*d + (4.0/3.0)*d*d)
	holeFilletBand := (4 - math.Pi) / 3 * ((rho+d)*(rho+d)*(rho+d) - rho*rho*rho)
	holeBand := holeSideBand - holeFilletBand
	return slab + rectBand - holeBand
}

// TestCapBlendTangentJunctionVolumeUnaffected is the fix's own negative
// space: docs/modify-reach-design.md §8.3 states the defect is affected
// exactly when a circular wall meets a neighbour at a NON-tangential miter
// corner, and unaffected at a tangent junction — the cap-level and side-level
// directrices coincide there in the real numbers, so a fix that narrowed or
// widened the window unconditionally, rather than reading the actual offset
// corner feet, would show up here as a multi-mm^3 residual against the same
// closed forms these shapes already have (a rounded rectangle's fillet band,
// a D-hole's own widening) — the same order of magnitude
// TestCapBlendErosionFamilyVolumeBoundEncloses's miter cases show, never
// merely a rounding-level one.
func TestCapBlendTangentJunctionVolumeUnaffected(t *testing.T) {
	t.Run(`rounded rectangle`, func(t *testing.T) {
		const l, w, h, r, d = 100.0, 60.0, 20.0, 15.0, 4.0
		body := roundedRectBody(t, l, w, h, r)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		want := roundedRectErosionVolume(l, w, h, r, d)
		require.InDelta(t, want, vol.Value.Mag(), 1e-6)
	})

	t.Run(`D-hole`, func(t *testing.T) {
		const plateL, side, rho, h, d = 100.0, 20.0, 3.0, 10.0, 2.0
		body := roundedRectHoleBody(t, side, rho)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		want := roundedRectHoleErosionVolume(plateL, side, rho, h, d)
		require.InDelta(t, want, vol.Value.Mag(), 1e-6)
	})
}

// TestCapBlendTangentJunctionAndWholeTurnBoundsStayTight pins the two
// PR-122 audit reproductions defect-3 fixed: a shape whose every circular
// wall meets its neighbour tangentially (so the two directrices share one
// window, `chordLocusResidualAllow` contributes nothing, and the ONLY thing
// left in play is the arithmetic-rounding envelope itself), and the
// cornerless whole-turn circle. `patchRawFlux`'s poly/cross terms used to
// read the absolute plane-local levels z0, z1 rather than the band's own z
// origin, which left the published Bound scale with those absolute levels
// instead of the band's own small axial extent — a restructure sound enough
// to still enclose the residual (Bound only ever WIDENS, never wrongly
// shrinks) but far looser than it needs to be on a body that never asked for
// the cap-loop chamfer's own general non-tangent-miter machinery at all. The
// ceilings below are well above the measured value (so ordinary arithmetic
// reordering cannot flake this) and well below the values a bound regressed
// back to reading the absolute levels would publish (audited at
// 28386.2821001117 and 1.450768679e-08 respectively).
func TestCapBlendTangentJunctionAndWholeTurnBoundsStayTight(t *testing.T) {
	t.Run(`tangent-fillet plate`, func(t *testing.T) {
		const l, w, h, r, d = 100.0, 60.0, 20.0, 15.0, 4.0
		body := roundedRectBody(t, l, w, h, r)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 20000.0,
			`the tangent-junction bound (%v mm^3) must stay close to the pre-PR-122 reading (15600.0000000006 mm^3), not the audited regression (28386.2821001117 mm^3)`,
			vol.Bound.Mag())
	})

	t.Run(`whole-turn circle`, func(t *testing.T) {
		body := circleProfile(t, 60, 20)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(4))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 5e-9,
			`the whole-turn bound (%v mm^3) must stay close to the pre-PR-122 reading (2.743387239e-09 mm^3), not the audited regression (1.450768679e-08 mm^3)`,
			vol.Bound.Mag())
	})
}

// dHoleBody extrudes a 100x100 plate with a "D" shaped hole: a circle of
// radius r centred at (cx, cy), cut by a chord between the points at
// phi0Deg and phi1Deg (degrees, measured from the centre) — a straight wall
// meeting a circular wall at two non-tangential miter corners, on a HOLE
// loop rather than an outer one. minor picks which of the chord's two arcs
// is the hole's own wall: true keeps the SHORT (<180 degrees) arc, false
// the LONG (>180 degrees) one — the major-arc case is PR-122's own audited
// branch-crossing repro (capWallSweep's raw Atan2 lands the offset foot's
// angle a full turn from the wall's own recorded th0).
func dHoleBody(t *testing.T, cx, cy, r, phi0Deg, phi1Deg float64, minor bool) *decad.Body {
	t.Helper()
	phi0 := phi0Deg * math.Pi / 180
	phi1 := phi1Deg * math.Pi / 180
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(outer.A)
	o := s.CreatePoint(cx, cy)
	p0 := s.CreatePoint(cx+r*math.Cos(phi0), cy+r*math.Sin(phi0))
	p1 := s.CreatePoint(cx+r*math.Cos(phi1), cy+r*math.Sin(phi1))
	s.CreateLine(p0, p1)
	if minor {
		s.CreateArc(o, p0, p1)
	} else {
		s.CreateArc(o, p1, p0)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof, `the rectangle-with-D-hole region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendDHoleLoopBoundStaysTight is the fix's own hole-loop floor:
// TestCapBlendTangentJunctionAndWholeTurnBoundsStayTight above pins the
// tight-bound property for an outer-loop tangent fillet and a whole-turn
// circle, but neither is a HOLE loop and neither has a genuinely
// non-tangential miter corner, so neither could have caught PR-122's own
// defect (a D-hole whose wall is the MAJOR arc: capWallSweep's raw Atan2
// puts the offset foot's angle a full 2*pi from the wall's own recorded
// th0, so chordLocusResidualAllow's windowSkewMax read close to 2*pi
// instead of the corner's own small miter skew, and the published bound
// inflated by orders of magnitude on a patch whose true residual is
// negligible). The major-arc case below is that exact repro, audited at a
// pre-fix bound of 22972.28860508683 mm^3 (windowSkewMax == 2*pi, up to
// float rounding) against a post-fix bound of 7157.192745203181 mm^3; the
// minor-arc case is the same hole family with the wall on the SHORT arc
// instead, which never crosses branches and never needed the fix, given a
// floor of its own so a future change to either patch cannot silently
// regress it unnoticed.
func TestCapBlendDHoleLoopBoundStaysTight(t *testing.T) {
	t.Run(`major-arc hole (branch-crossing repro)`, func(t *testing.T) {
		body := dHoleBody(t, 50, 50, 10, 30, 150, false)
		chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(body))), units.Millimeters(2))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 10000.0,
			`the major-arc D-hole bound (%v mm^3) must stay close to the post-fix reading (7157.192745203181 mm^3), not the pre-fix branch-mismatch regression (22972.28860508683 mm^3)`,
			vol.Bound.Mag())
	})

	t.Run(`minor-arc hole`, func(t *testing.T) {
		body := dHoleBody(t, 50, 50, 10, 30, 150, true)
		chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(body))), units.Millimeters(2))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 5000.0,
			`the minor-arc D-hole bound (%v mm^3) must stay close to the audited reading (2733.0793750625394 mm^3)`,
			vol.Bound.Mag())
	})
}
