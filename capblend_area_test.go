package decad_test

import (
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
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
