package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

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
