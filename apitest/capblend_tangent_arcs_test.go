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

// tangentArcPlate extrudes 8 mm of a 20 mm wide plate whose straight sides
// turn into R4.5 corners joined by one R22 end arc, every join a sketch
// tangent constraint. The solver places the four tangent points of the arcs
// by rounding, so the record holds the corner arcs' far ends off the circle
// each Start states and the corner–end arc joins only near-tangent.
func tangentArcPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	const halfWidth, cornerR, endR, sideEnd, top = 10.0, 4.5, 22.0, -10.0, 10.0
	s := shortStepSketch(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	cR := r3.Vec{X: halfWidth - cornerR, Y: sideEnd}
	cB := r3.Vec{Y: sideEnd + math.Sqrt((endR-cornerR)*(endR-cornerR)-(halfWidth-cornerR)*(halfWidth-cornerR))}
	pR := cB.Add(cR.Sub(cB).Scale(endR / cR.Sub(cB).Len()))
	tr, tl := s.CreatePoint(halfWidth, top), s.CreatePoint(-halfWidth, top)
	sR, sL := s.CreatePoint(halfWidth, sideEnd), s.CreatePoint(-halfWidth, sideEnd)
	s.CreateLine(tl, tr)
	right, left := s.CreateLine(sR, tr), s.CreateLine(tl, sL)
	cRp, cLp := s.CreatePoint(cR.X, cR.Y), s.CreatePoint(-cR.X, cR.Y)
	cBp := s.CreatePoint(cB.X, cB.Y)
	pRp, pLp := s.CreatePoint(pR.X, pR.Y), s.CreatePoint(-pR.X, pR.Y)
	arcR := s.CreateArc(cRp, pRp, sR)
	arcL := s.CreateArc(cLp, sL, pLp)
	arcB := s.CreateArc(cBp, pLp, pRp)
	s.AddConstraint(
		sketch.NewVertical(right), sketch.NewVertical(left),
		sketch.NewRadius(arcR, cornerR), sketch.NewTangent(right, arcR),
		sketch.NewRadius(arcL, cornerR), sketch.NewTangent(left, arcL),
		sketch.NewRadius(arcB, endR), sketch.NewTangentCircles(arcB, arcR, true), sketch.NewTangentCircles(arcB, arcL, true),
	)
	body, err := decad.Extrude(t.Context(), doc, s, shortStepProfile(t, s), decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendCutAfterTangentArcChamfer chamfers both cap loops of
// tangentArcPlate and cuts a 4 mm box out of its middle, clear of both
// bands. The chamfer's occupied-volume proof restates each off-radius corner
// arc on the centre that puts both its ends on one circle and charges the
// near-tangent joins (agent-docs/tessellation-reach-design.md §7.1), so the
// cut composes the chamfered body and removes exactly the 4 × 4 × 3 mm the
// box overlaps.
func TestCapBlendCutAfterTangentArcChamfer(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := decad.New()
	body := tangentArcPlate(t, doc)
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Or(decad.OuterLoopOf(decad.CapEnd(body)))
	chamfered, err := decad.Chamfer(body, loops, units.Millimeters(0.5)).Do(ctx)
	require.NoError(t, err)
	before, err := chamfered.Volume()
	require.NoError(t, err)

	box := boxBodyAtZ(t, doc, -2, -2, 2, 2, -1, 4)
	got, err := decad.Cut(chamfered, box).Do(ctx)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	want := volumeMM(t, before) - 48
	bound := boundMM3(t, vol) + boundMM3(t, before)
	t.Logf("chamfered %.12f ± %g, cut %.12f ± %g, want %.12f", volumeMM(t, before), boundMM3(t, before), volumeMM(t, vol), boundMM3(t, vol), want)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound)
	require.Positive(t, bound)
	require.Less(t, bound, 1.0, `a ceiling on the published bound, not a pin`)
	requireBodyWatertight(t, got)
}
