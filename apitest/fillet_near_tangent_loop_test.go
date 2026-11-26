package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// hexCircum is the radius of the circle roundedHexPrism's corners lie on,
// and hexRound the radius its corners are rounded to.
const hexCircum, hexRound = 2.51, 0.8

// roundedHexPrism extrudes h of a regular hexagon whose corners lie on a
// circle of radius hexCircum about origin, one flat parallel to X, with every
// corner rounded by sketch.CreateFillet to radius hexRound. The sides are held
// by their distance from the origin and their 60 degree turns, so the solver
// places each tangent point by rounding: the record's line–arc joins at the
// 30 and 60 degree directions are tangent only to the solver's precision.
func roundedHexPrism(t *testing.T, doc *decad.Document, origin r3.Vec, h float64) *decad.Body {
	t.Helper()
	s := shortStepSketch(t, origin, r3.Vec{X: 1}, r3.Vec{Y: 1})
	addRoundedHex(t, s, hexCircum, hexRound)
	body, err := decad.Extrude(t.Context(), doc, s, shortStepProfile(t, s), decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// addRoundedHex draws roundedHexPrism's outline into s.
func addRoundedHex(t *testing.T, s *sketch.Sketch, circum, round float64) {
	t.Helper()
	var corners [6]*sketch.Point
	for i := range corners {
		a := float64(i) * math.Pi / 3
		corners[i] = s.CreatePoint(circum*math.Cos(a), circum*math.Sin(a))
	}
	var sides [6]*sketch.Line
	for i := range sides {
		sides[i] = s.CreateLine(corners[i], corners[(i+1)%6])
		s.AddConstraint(sketch.NewDistancePointLine(s.Origin(), sides[i], circum*math.Sqrt(3)/2))
	}
	s.AddConstraint(sketch.NewHorizontal(sides[1]))
	for i := range 5 {
		s.AddConstraint(sketch.NewAngle(sides[i], sides[i+1], 60))
	}
	for i, c := range corners {
		_, err := sketch.CreateFillet(s, sides[(i+5)%6], sides[i], c, round)
		require.NoError(t, err)
	}
}

// roundedHexStrip is Table CF's a₁ (the perimeter) of roundedHexPrism's
// outline and the strip volume a₁·J₁ + a₂·J₂ its loop fillet of radius r
// sweeps, with a₂ = −π for an outer loop and +π for a hole or a boss root,
// whose material lies outside the outline (agent-docs/modify-design.md Part L
// §5.1).
func roundedHexStrip(circum, round, r float64, outside bool) float64 {
	half := math.Pi / 6
	a1 := 6*(circum-2*round*math.Tan(half)) + 2*math.Pi*round
	a2 := -math.Pi
	if outside {
		a2 = math.Pi
	}
	return a1*r*r*(1-math.Pi/4) + a2*r*r*r*(5.0/3-math.Pi/2)
}

// roundedHexFilletVolume is Pappus's closed form for roundedHexPrism of
// height h with one cap loop filleted by r: the rounded hexagon's area times
// the height, less the fillet's spandrel (area r²(1 − π/4), centroid
// r(10 − 3π)/(12 − 3π) in from each wall) swept along the six straight runs
// and round the six corners.
func roundedHexFilletVolume(circum, round, h, r float64) float64 {
	half := math.Pi / 6 // half of each corner's 60 degree turn
	area := 3*math.Sqrt(3)/2*circum*circum - 6*round*round*(math.Tan(half)-half)
	straight := 6 * (circum - 2*round*math.Tan(half))
	spandrel := r * r * (1 - math.Pi/4)
	inset := r * (10 - 3*math.Pi) / (12 - 3*math.Pi)
	return area*h - spandrel*(straight+2*math.Pi*(round-inset))
}

// TestFilletLoopNearTangentJoins fillets the start cap's loop of
// roundedHexPrism (corner circle 2.51, corner rounds 0.8, 3 mm tall) by
// 0.6. Each line–arc join of the loop is tangent only to the sketch solver's
// precision, so the loop fillet reads it as an LF5′ join and charges its
// sliver (agent-docs/modify-design.md Part L §5.5). The volume matches
// roundedHexFilletVolume within the published bound.
func TestFilletLoopNearTangentJoins(t *testing.T) {
	t.Parallel()
	const circum, round, h, r = 2.51, 0.8, 3.0, 0.6
	doc := decad.New()
	body := roundedHexPrism(t, doc, r3.Vec{}, h)
	got, err := decad.Fillet(body, decad.Edges(decad.OuterLoopOf(decad.CapStart(body))), units.Millimeters(r)).Do(t.Context())
	require.NoError(t, err)

	want := roundedHexFilletVolume(circum, round, h, r)

	vol, err := got.Volume()
	require.NoError(t, err)
	bound := boundMM3(t, vol)
	t.Logf("filleted %.12f ± %g, want %.12f", volumeMM(t, vol), bound, want)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound+1e-9)
	require.Less(t, bound, 0.01, `a ceiling on the published bound, not a pin`)
	requireBodyWatertight(t, got)
}

// tangentArcPlateFilletVolume is the start cap loop of tangentArcPlate
// filleted by r, against Table CF's closed form: the plate's perimeter times
// J₁ plus a₂·J₂, a₂ = −(2 + π/2) from its two right-angle miters and its
// half turn of arcs (agent-docs/modify-design.md Part L §5.1).
func TestFilletLoopNearTangentArcJoins(t *testing.T) {
	t.Parallel()
	const r = 0.5
	doc := decad.New()
	body := tangentArcPlate(t, doc)
	before, err := body.Volume()
	require.NoError(t, err)
	got, err := decad.Fillet(body, decad.Edges(decad.OuterLoopOf(decad.CapStart(body))), units.Millimeters(r)).Do(t.Context())
	require.NoError(t, err)

	const halfWidth, cornerR, endR = 10.0, 4.5, 22.0
	alpha := math.Atan2(math.Sqrt((endR-cornerR)*(endR-cornerR)-(halfWidth-cornerR)*(halfWidth-cornerR)), halfWidth-cornerR)
	perimeter := 2*halfWidth + 2*20 + 2*cornerR*alpha + endR*(math.Pi-2*alpha)
	a2 := -(2 + math.Pi/2)
	j1, j2 := r*r*(1-math.Pi/4), r*r*r*(5.0/3-math.Pi/2)
	want := volumeMM(t, before) - (perimeter*j1 + a2*j2)

	vol, err := got.Volume()
	require.NoError(t, err)
	bound := boundMM3(t, vol) + boundMM3(t, before)
	t.Logf("filleted %.12f ± %g, want %.12f", volumeMM(t, vol), bound, want)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound+1e-9)
	require.Less(t, bound, 0.01, `a ceiling on the published bound, not a pin`)
	requireBodyWatertight(t, got)
}

// TestFilletLoopNearTangentCut cuts a 12 × 10 × 4 arm, both cap loops
// chamfered by 0.5, with roundedHexPrism standing 5 tall from z = 1 and its
// start cap loop filleted by 0.6: a pocket whose floor outline is rounded.
// The tool passes through the arm's top face inside its chamfer band, so the
// cut removes the filleted tool's part below z = 4 (roundedHexFilletVolume of
// height 3) within the two bodies' published bounds.
func TestFilletLoopNearTangentCut(t *testing.T) {
	t.Parallel()
	const circum, round, r = 2.51, 0.8, 0.6
	ctx := t.Context()
	doc := decad.New()
	arm := boxBodyAtZ(t, doc, -6, -5, 6, 5, 0, 4)
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(arm))).Or(decad.OuterLoopOf(decad.CapEnd(arm)))
	arm, err := decad.Chamfer(arm, loops, units.Millimeters(0.5)).Do(ctx)
	require.NoError(t, err)
	before, err := arm.Volume()
	require.NoError(t, err)

	tool := roundedHexPrism(t, doc, r3.Vec{Z: 1}, 5)
	tool, err = decad.Fillet(tool, decad.Edges(decad.OuterLoopOf(decad.CapStart(tool))), units.Millimeters(r)).Do(ctx)
	require.NoError(t, err)
	got, err := decad.Cut(arm, tool).Do(ctx)
	require.NoError(t, err)

	vol, err := got.Volume()
	require.NoError(t, err)
	want := volumeMM(t, before) - roundedHexFilletVolume(circum, round, 3, r)
	bound := boundMM3(t, vol) + boundMM3(t, before)
	t.Logf("arm %.12f ± %g, cut %.12f ± %g, want %.12f", volumeMM(t, before), boundMM3(t, before), volumeMM(t, vol), boundMM3(t, vol), want)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound)
	require.Positive(t, bound)
	require.Less(t, bound, 1.0, `a ceiling on the published bound, not a pin`)
	requireBodyWatertight(t, got)
}

// requireFilletVolumeNear asserts got's published volume encloses want within its
// bound plus before's (the receiver's own bound, when want reads it) and
// that the published bound stays under ceiling.
func requireFilletVolumeNear(t *testing.T, got *decad.Body, want, extraBound, ceiling float64) {
	t.Helper()
	vol, err := got.Volume()
	require.NoError(t, err)
	bound := boundMM3(t, vol) + extraBound
	t.Logf("volume %.12f ± %g, want %.12f", volumeMM(t, vol), bound, want)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound+1e-9)
	require.Less(t, bound, ceiling, `a ceiling on the published bound, not a pin`)
	requireBodyWatertight(t, got)
}

// TestFilletLoopNearTangentHole fillets the start cap's hole loop of a
// 12 × 10 × 3 plate whose sketch holds roundedHexPrism's outline as a hole:
// the loop walks the arcs clockwise with material outside, so each sketched
// join is an LF5′ join of a hole rim, and the strip is Table CF's with
// a₂ = +π.
func TestFilletLoopNearTangentHole(t *testing.T) {
	t.Parallel()
	const circum, round, h, r = 2.51, 0.8, 3.0, 0.6
	ctx := t.Context()
	doc := decad.New()
	s := shortStepSketch(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	a, b, c, d := s.CreatePoint(-6, -5), s.CreatePoint(6, -5), s.CreatePoint(6, 5), s.CreatePoint(-6, 5)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	addRoundedHex(t, s, circum, round)
	_, err := s.Solve(ctx)
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	plate, err := decad.Extrude(ctx, doc, s, prof, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	before, err := plate.Volume()
	require.NoError(t, err)
	capFace := planeFacing(t, plate, r3.NewVec(0, 0, -1), r3.NewVec(0, 0, 0))
	var hole *decad.Loop
	for _, l := range capFace.Loops() {
		if !l.IsOuter() {
			hole = l
		}
	}
	require.NotNil(t, hole)
	got, err := decad.Fillet(plate, loopEdgeQuery(hole), units.Millimeters(r)).Do(ctx)
	require.NoError(t, err)
	requireFilletVolumeNear(t, got, volumeMM(t, before)-roundedHexStrip(circum, round, r, true), boundMM3(t, before), 0.01)
}

// TestFilletLoopNearTangentBossRoot unites roundedHexPrism, 3 tall, onto a
// 12 × 10 × 3 plate and fillets the boss root, the plate top's inner loop:
// the walls rise off the face (σ = +1), so the band fills the concave corner
// and adds Table CF's strip with a₂ = +π.
func TestFilletLoopNearTangentBossRoot(t *testing.T) {
	t.Parallel()
	const circum, round, r = 2.51, 0.8, 0.6
	ctx := t.Context()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, -6, -5, 6, 5, 0, 3)
	boss := roundedHexPrism(t, doc, r3.Vec{Z: 3}, 3)
	joined, err := decad.Union(plate, boss).Do(ctx)
	require.NoError(t, err)
	before, err := joined.Volume()
	require.NoError(t, err)
	top := planeFacing(t, joined, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 3))
	var root *decad.Loop
	for _, l := range top.Loops() {
		if !l.IsOuter() {
			root = l
		}
	}
	require.NotNil(t, root)
	got, err := decad.Fillet(joined, loopEdgeQuery(root), units.Millimeters(r)).Do(ctx)
	require.NoError(t, err)
	requireFilletVolumeNear(t, got, volumeMM(t, before)+roundedHexStrip(circum, round, r, true), boundMM3(t, before), 0.01)
}

// TestFilletLoopNearTangentInchRadius fillets roundedHexPrism's start cap
// loop by 0.025 in, a radius whose conversion to millimetres is inexact, so
// every near-tangent and restatement term reads the radius over its
// conversion span.
func TestFilletLoopNearTangentInchRadius(t *testing.T) {
	t.Parallel()
	const circum, round, h = 2.51, 0.8, 3.0
	doc := decad.New()
	body := roundedHexPrism(t, doc, r3.Vec{}, h)
	radius := units.Inches(0.025)
	r, err := radius.In(units.Millimeter)
	require.NoError(t, err)
	got, err := decad.Fillet(body, decad.Edges(decad.OuterLoopOf(decad.CapStart(body))), radius).Do(t.Context())
	require.NoError(t, err)
	requireFilletVolumeNear(t, got, roundedHexFilletVolume(circum, round, h, r), 0, 0.01)
}
