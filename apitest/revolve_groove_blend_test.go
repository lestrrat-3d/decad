package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers agent-docs/general-boolean-design.md "Common-axis full
// revolutions": a retaining-ring groove cut into a stepped shaft keeps an
// exact meridian, so the modify ops that rewrite the meridian take it. Every
// fixture is drawn on the XY plane and revolved a full turn about the u axis,
// so a meridian point (u, v) is the axis coordinate z = u and the radius
// ρ = v, and Pappus turns a meridian's ∫ρ dA into the volume 2π·∫ρ dA.
//
// Legs shown to fail before these fixtures were accepted: recording the cut
// meridian through MergeWithChain, which charges the shaft rim's rounded cut
// parameters, refuses all three with "cannot integrate a region over a
// meridian carrying its own section displacement".

// grooveShaftMeridian is a stepped shaft: a radius-12 journal over
// z ∈ [0, 30] and a radius-8 journal over z ∈ [30, 80].
var grooveShaftMeridian = [][2]float64{{0, 0}, {80, 0}, {80, 8}, {30, 8}, {30, 12}, {0, 12}}

// grooveRingMeridian is the ring a retaining-ring groove removes: 2 mm wide
// and 1 mm deep into the radius-8 journal.
var grooveRingMeridian = [][2]float64{{60, 7}, {62, 7}, {62, 9}, {60, 9}}

// groovedShaftQ is the grooved shaft's ∫ρ dA: 144/2·30 + 64/2·50 less the
// groove's (64 − 49)/2·2.
const groovedShaftQ = 72*30 + 32*50 - 15.0

// groovedShaft cuts the groove from the shaft. Both are full turns about the
// same axis, so the Cut arranges the two meridians in one Sketch scene. The
// groove's walls cross the radius-8 rim at (60, 8) and (62, 8), at cut
// parameters 0.36 and 0.4 along the rim, neither a binary64 value.
func groovedShaft(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	shaft := revolveMeridian(t, doc, grooveShaftMeridian, decad.FullRevolution{})
	ring := revolveMeridian(t, doc, grooveRingMeridian, decad.FullRevolution{})
	grooved, err := decad.Cut(t.Context(), shaft, ring)
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, grooved, fullTurnVolume(groovedShaftQ))
	return grooved
}

func TestRevolveGrooveCornerBlends(t *testing.T) {
	t.Parallel()
	// The three concave junctions are the shoulder's inner corner (30, 8)
	// and the groove floor's corners (60, 7) and (62, 7). Each blend adds a
	// region at its corner whose centroid sits the same distance c from the
	// corner along both legs, so it lies at ρ = 8 + c for the shoulder and
	// ρ = 7 + c for each groove corner.
	t.Run("fillet", func(t *testing.T) {
		t.Parallel()
		const r = 0.3
		doc := decad.New()
		grooved := groovedShaft(t, doc)
		filleted, err := decad.Fillet(t.Context(), grooved, concaveJunctions().Exactly(3), units.Millimeters(r))
		require.NoError(t, err)
		requireManifold(t, filleted)
		require.Len(t, blendFaces(t, filleted, "fillet"), 3)
		// The spandrel's area is (1 − π/4)r² and its first moment about
		// either leg k·r times that (spandrel).
		a, m, _ := spandrel(r)
		c := m / a
		decadtest.MeasuresVolume(t, filleted, fullTurnVolume(groovedShaftQ+a*(8+c)+2*a*(7+c)))
		decadtest.IsSound(t, doc)
	})
	t.Run("chamfer", func(t *testing.T) {
		t.Parallel()
		const d = 0.3
		doc := decad.New()
		grooved := groovedShaft(t, doc)
		chamfered, err := decad.Chamfer(t.Context(), grooved, concaveJunctions().Exactly(3), units.Millimeters(d))
		require.NoError(t, err)
		requireManifold(t, chamfered)
		require.Len(t, blendFaces(t, chamfered, "chamfer"), 3)
		// Each chamfer adds a right triangle of legs d: area d²/2, centroid
		// d/3 from the corner along each leg.
		a, c := d*d/2, d/3
		decadtest.MeasuresVolume(t, chamfered, fullTurnVolume(groovedShaftQ+a*(8+c)+2*a*(7+c)))
		decadtest.IsSound(t, doc)
	})
	t.Run("closed shell", func(t *testing.T) {
		t.Parallel()
		// The 0.5 mm cavity is the meridian's erosion cut back to the axis:
		// z ∈ [0.5, 29.5], ρ ∈ [0, 11.5] (29·11.5²/2) beside
		// z ∈ [29.5, 79.5], ρ ∈ [0, 7.5] (50·7.5²/2). The shoulder's concave
		// corner (30, 8) adds the square z ∈ [29.5, 30], ρ ∈ [7.5, 8] less
		// the quarter disk of radius 0.5 about it, whose centroid sits
		// 2/(3π) below ρ = 8. The groove takes away every point within 0.5
		// of it: z ∈ [59.5, 62.5], ρ ∈ [7, 7.5], then z ∈ [60, 62],
		// ρ ∈ [6.5, 7], then a quarter disk below each floor corner, centroid
		// 2/(3π) below ρ = 7.
		const t5 = 0.5
		quarter := math.Pi * t5 * t5 / 4
		drop := 4 * t5 / (3 * math.Pi)
		cavity := 29*11.5*11.5/2 + 50*7.5*7.5/2 +
			t5*(64-7.5*7.5)/2 - quarter*(8-drop) -
			(3*(7.5*7.5-49)/2 + 2*(49-6.5*6.5)/2 + 2*quarter*(7-drop))
		doc := decad.New()
		grooved := groovedShaft(t, doc)
		shelled, err := decad.Shell(t.Context(), grooved, nil, units.Millimeters(t5), decad.WithNoOpenings())
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(groovedShaftQ-cavity))
		decadtest.IsSound(t, doc)
	})
}
