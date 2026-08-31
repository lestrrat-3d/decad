package decad_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the cap-blend band's DX7/DX8 surveys read through the public
// API (docs/modify-reach-design.md §8.3, Table DX). Everything here turns on
// ONE fact about a circular band patch at a non-tangential miter corner: the
// build rules it with straight Line3 rulings between two directrices that
// sweep DIFFERENT angular windows, so the surface it carries is not the Cone
// it publishes — it is not even developable, and its own normal turns along a
// single ruling, which a cone's never does. Every reading taken off that Cone
// therefore owes a bound, and the two readings that cannot state one refuse.

// miteredConePatch returns the single circular chamfer band patch of a
// chamfered quarter disk, with the two arcs and two rulings its own boundary
// carries: the side-level directrix (the wall's own recorded sweep), the
// cap-level one (the offset arc, trimmed at the mitered corner feet), and the
// two straight rulings between them.
type miteredConePatch struct {
	body                 *decad.Body
	face                 *decad.Face
	sideArc, capArc      decad.Arc3
	sideZ, capZ          float64
	rulings              [2][2]r3.Vec // [i] = {side-level end, cap-level end}
	sideAngles, capAngle [2]float64
}

func chamferedQuarterDiskPatch(t *testing.T, r, h, d float64) miteredConePatch {
	t.Helper()
	body := quarterDiskBody(t, r, h)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	out := miteredConePatch{body: chamfered}
	found := 0
	for _, f := range chamfered.Faces() {
		if f.Surface().Kind() != decad.KindCone {
			continue
		}
		found++
		out.face = f
		var lines []*decad.Edge
		for _, ce := range f.Loops()[0].CoEdges() {
			e := ce.Edge()
			switch c := e.Curve().(type) {
			case decad.Arc3:
				if e.Start().Position().Value.Z == h {
					out.capArc, out.capZ = c, h
					continue
				}
				out.sideArc, out.sideZ = c, e.Start().Position().Value.Z
			case decad.Line3:
				lines = append(lines, e)
			}
		}
		require.Len(t, lines, 2, "a mitered band patch is bounded by two straight rulings")
		for i, ln := range lines {
			a, b := ln.Start().Position().Value, ln.End().Position().Value
			if a.Z == h {
				a, b = b, a
			}
			out.rulings[i] = [2]r3.Vec{a, b}
		}
	}
	require.Equal(t, 1, found, "the quarter disk's single circular wall builds one band patch")

	for i := range out.rulings {
		s, c := out.rulings[i][0], out.rulings[i][1]
		out.sideAngles[i] = math.Atan2(s.Y-out.sideArc.Center.Y, s.X-out.sideArc.Center.X)
		out.capAngle[i] = math.Atan2(c.Y-out.capArc.Center.Y, c.X-out.capArc.Center.X)
	}
	return out
}

// pointAt evaluates the RULED surface the build actually assembles, at
// parameters (u, v): the straight ruling at u, walked from its side-level end
// (v = 0) to its cap-level end (v = 1). Both directrices are read off the
// patch's OWN public boundary — arc centre, radius and level, and the two
// rulings' endpoints — so nothing here re-derives a production formula; it
// re-reads the surface §8.3 says the build rules.
func (m miteredConePatch) pointAt(u, v float64) r3.Vec {
	sideTh := m.sideAngles[0] + u*(m.sideAngles[1]-m.sideAngles[0])
	capTh := m.capAngle[0] + u*(m.capAngle[1]-m.capAngle[0])
	sr := m.sideArc.Radius.Mag()
	cr := m.capArc.Radius.Mag()
	side := r3.NewVec(m.sideArc.Center.X+sr*math.Cos(sideTh), m.sideArc.Center.Y+sr*math.Sin(sideTh), m.sideZ)
	top := r3.NewVec(m.capArc.Center.X+cr*math.Cos(capTh), m.capArc.Center.Y+cr*math.Sin(capTh), m.capZ)
	return side.Add(top.Sub(side).Scale(v))
}

// trueNormalAt is the ruled surface's own outward unit normal at (u, v), by
// central differences on pointAt — an independent numerical reading, never the
// evaluator's own closed form. The face's published normal decides only the
// SIGN, which it may: the two are far closer than a right angle everywhere
// this file samples, so the sign is never the thing under test.
func (m miteredConePatch) trueNormalAt(t *testing.T, u, v float64) (r3.Vec, r3.Vec) {
	t.Helper()
	const h = 1e-6
	du := m.pointAt(math.Min(1, u+h), v).Sub(m.pointAt(math.Max(0, u-h), v))
	dv := m.pointAt(u, math.Min(1, v+h)).Sub(m.pointAt(u, math.Max(0, v-h)))
	n, ok := du.Cross(dv).Normalize()
	require.True(t, ok, "the ruled patch has a tangent plane at (%v, %v)", u, v)

	published, err := m.face.NormalAt(m.pointAt(u, v))
	require.NoError(t, err)
	if n.Dot(published.Value) < 0 {
		n = n.Scale(-1)
	}
	return n, published.Value
}

// TestCapBlendMiteredPatchNormalCarriesItsOwnBound is the fix's central claim:
// a mitered band patch's Face.NormalAt is the CONE's normal, and the surface
// the build assembled is not that cone, so the reading is published
// Approximate with a bound that encloses how far the two can differ — over the
// whole patch, not merely at the point sampled to derive it. A zero bound
// there asserts a direction the built surface does not have.
func TestCapBlendMiteredPatchNormalCarriesItsOwnBound(t *testing.T) {
	for _, tc := range []struct {
		name    string
		r, h, d float64
	}{
		{`ordinary setback`, 100, 20, 0.5},
		{`moderate setback`, 60, 20, 4},
		{`widest admitted setback`, 10, 20, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := chamferedQuarterDiskPatch(t, tc.r, tc.h, tc.d)

			sample, err := m.face.NormalAt(m.rulings[0][1])
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, sample.Exactness,
				"a ruled patch's normal is not the Cone's own")
			require.Greater(t, sample.Bound.Mag(), 0.0)

			worst := 0.0
			for i := range 11 {
				for j := range 11 {
					u, v := float64(i)/10, float64(j)/10
					truth, published := m.trueNormalAt(t, u, v)
					gap := truth.Sub(published).Len()
					worst = math.Max(worst, gap)
					n, err := m.face.NormalAt(m.pointAt(u, v))
					require.NoError(t, err)
					require.LessOrEqual(t, gap, n.Bound.Mag(),
						"at (%v, %v) the built surface's normal is %v from the published %v, past its own bound",
						u, v, truth, published)
				}
			}
			require.Greater(t, worst, 0.0, "the two surfaces genuinely differ")
		})
	}
}

// TestCapBlendUnmiteredPatchNormalCarriesNoWindowSkew pins the other side of
// the same rule: where a band patch's two directrices sweep ONE window there is
// no ANGULAR skew to bound, so the patch may not pay for the mitered one's.
// What is left on an unplaced, axis-aligned build of these shapes is the
// rounding of the coordinates themselves, which is a rounding-scale term rather
// than a geometric one (capblend_departure_test.go reads the same term on a
// placed band, where it is orders larger). A whole turn (no corner trims it at
// all) and a rectangular loop's flat patches are the shipped cases.
func TestCapBlendUnmiteredPatchNormalCarriesNoWindowSkew(t *testing.T) {
	for _, tc := range []struct {
		name  string
		build func(t *testing.T) *decad.Body
		d     float64
		kind  decad.SurfaceKind
	}{
		{`whole circle`, func(t *testing.T) *decad.Body { return circleProfile(t, 20, 10) }, 2, decad.KindCone},
		{`rectangular loop`, func(t *testing.T) *decad.Body { _, b := capBlendBox(t); return b }, 5, decad.KindPlane},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := tc.build(t)
			chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(tc.d))
			require.NoError(t, err)
			checked := 0
			for _, f := range chamfered.Faces() {
				if f.Surface().Kind() != tc.kind {
					continue
				}
				p := f.Loops()[0].CoEdges()[0].Start().Position().Value
				n, err := f.NormalAt(p)
				require.NoError(t, err)
				requireRoundingScaleNormalBound(t, n)
				checked++
			}
			require.Positive(t, checked)
		})
	}
}

// TestCapBlendMiteredUndercutIsNotPassed is the unsound PASS this fix exists
// for. At the widest setback the offset admits, the built patch's own normal
// leans 25 degrees off the Cone's, so a pull tilted just inside the Cone's own
// clearance opposes the surface the body actually has while the Cone says it
// does not. The survey used to read the Cone alone and answer the proven
// all-clear — a wrong Sound, not a coarse reading. The Cone patch now stays
// undecided and unlisted; any independent proven face remains a violation.
// TestUndercutCapBlendReceiverWallsBoundedLikePatches proves DX7's extended
// §12 cell (docs/modify-reach-design.md): a cap-blend body's receiver walls
// read through the SAME bounded three-valued rule as a chamfer patch, rather
// than a raw float range. The receiver here is fu155's own square (edge
// tangents exactly (3,9), (-9,3), (-3,-9), (9,-3),
// TestUndercutExactlyPerpendicularWallIsNotOpposed): before the fix,
// chamfering its cap loop listed 3 faces — the genuine patch plus the two
// mismeasured receiver walls exactly perpendicular and exactly antiparallel
// to the pull.
func TestUndercutCapBlendReceiverWallsBoundedLikePatches(t *testing.T) {
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {3, 9}, {-6, 12}, {-9, 3}})
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(1))
	require.NoError(t, err)

	report, err := chamfered.Document().Verify(t.Context(), decad.WithPullDirection(r3.NewVec(3, 9, 0)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	br := report.Bodies[0]
	require.Len(t, br.Undercuts, 1, "only the chamfer patch, not either receiver wall")

	f := br.Undercuts[0]
	n, err := f.NormalAt(f.Loops()[0].CoEdges()[0].Start().Position().Value)
	require.NoError(t, err)
	require.InDelta(t, math.Sqrt2/2, n.Value.Z, 1e-12, "the listed face is the 45-degree chamfer patch")
	unit, ok := r3.NewVec(3, 9, 0).Normalize()
	require.True(t, ok)
	require.InDelta(t, -math.Sqrt2/2, n.Value.Dot(unit), 1e-12)

	undercutDiags := 0
	for _, d := range report.Diagnostics {
		if d.Code == decad.DiagUndercut {
			undercutDiags++
		}
	}
	require.Equal(t, 1, undercutDiags, "one DiagUndercut for the one proven listing, no diagnostic for a receiver wall")
}

func TestCapBlendMiteredUndercutIsNotPassed(t *testing.T) {
	const r, h, d = 10.0, 20.0, 4.0
	m := chamferedQuarterDiskPatch(t, r, h, d)

	// A pull tilted 44 degrees off the sweep axis, away from the patch's own
	// mid azimuth: the Cone's normal keeps a positive component along it
	// everywhere on the window (44 < the Cone's own 45 degree taper), so the
	// Cone reading clears the patch outright.
	const tilt = 44 * math.Pi / 180
	pull := r3.NewVec(-math.Sin(tilt)*math.Cos(math.Pi/4), -math.Sin(tilt)*math.Sin(math.Pi/4), math.Cos(tilt))

	truth, published := m.trueNormalAt(t, 0, 1)
	require.Greater(t, published.Dot(pull), 0.0, "the published Cone normal does not oppose this pull")
	require.Less(t, truth.Dot(pull), 0.0, "the surface the build assembled does oppose it")

	report, err := m.body.Document().Verify(t.Context(), decad.WithPullDirection(pull))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotContains(t, report.Bodies[0].Undercuts, m.face,
		"the straddling Cone patch is not itself a proven undercut")
	require.True(t, hasDiagnostic(report, decad.DiagUndercut))
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
	require.Equal(t, decad.Violating, report.Bodies[0].Status)
}

// TestCapBlendUndecidedPatchKeepsProvenUndercut covers the mixed per-face
// outcome. The circular patch's bounded range is undecided for a pull along
// +x, but the final flat chamfer patch provably opposes it. The undecided
// patch must not erase that listed violation.
func TestCapBlendUndecidedPatchKeepsProvenUndercut(t *testing.T) {
	m := chamferedQuarterDiskPatch(t, 100, 20, 0.5)
	plane := faceWithRole(t, m.body, "chamferCap(end,0,2)")
	require.Equal(t, decad.KindPlane, plane.Surface().Kind())
	p := plane.Loops()[0].CoEdges()[0].Start().Position().Value
	n, err := plane.NormalAt(p)
	require.NoError(t, err)
	requireRoundingScaleFlatBound(t, n)
	require.InDelta(t, -math.Sqrt2/2, n.Value.Dot(r3.NewVec(1, 0, 0)), 1e-12)

	report, err := m.body.Document().Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 0)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Contains(t, report.Bodies[0].Undercuts, plane)
	require.True(t, hasDiagnostic(report, decad.DiagUndercut))
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
	require.Equal(t, decad.Violating, report.Bodies[0].Status)
}

// TestCapBlendWholeTurnUndercutRespectsNormalBound covers the OTHER reading a
// band patch's published bound governs, on the band that carries no window skew
// at all. A whole-turn Cone patch publishes its normal
// with the arithmetic bound its own arm earned (normal_bound.go), and a caller
// may pull along a direction the patch's own tangent cannot be separated from
// within that bound: the exact taper and the float cosine and sine the arm
// took of the held half angle disagree by less than the bound, so which side
// of zero the patch's minimum component falls on is not decided by the
// reading. A survey that kept only the sampled value would answer that pull
// with a positive claim — the proven all-clear or a listed violation — and be
// right only by luck. It must be undecided instead.
func TestCapBlendWholeTurnUndercutRespectsNormalBound(t *testing.T) {
	body := circleProfile(t, 20, 10)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(2))
	require.NoError(t, err)

	var cone *decad.Face
	for _, f := range chamfered.Faces() {
		if f.Surface().Kind() == decad.KindCone {
			require.Nil(t, cone, "a whole-turn band builds one Cone patch")
			cone = f
		}
	}
	require.NotNil(t, cone)

	// The patch's own published normal at one azimuth, which fixes the whole
	// band: a Cone's normal keeps one axial component and turns its radial
	// part with the azimuth, so the band's minimum component against a pull p
	// is p.Z*n.Z - hypot(p.X, p.Y)*hypot(n.X, n.Y).
	n, err := cone.NormalAt(cone.Loops()[0].CoEdges()[0].Start().Position().Value)
	require.NoError(t, err)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Positive(t, bound, "a Cone arm's own cosine and sine are not exact")

	// A pull whose lateral share matches that same ratio makes the band's
	// minimum component zero — the tangent the reading cannot resolve.
	radial := math.Hypot(n.Value.X, n.Value.Y)
	axial := math.Abs(n.Value.Z)
	pull := r3.NewVec(axial/radial, 0, math.Copysign(1, n.Value.Z))
	unit, ok := pull.Normalize()
	require.True(t, ok)
	minimum := unit.Z*n.Value.Z - math.Hypot(unit.X, unit.Y)*radial
	require.Less(t, math.Abs(minimum), bound,
		"the pull must sit inside the patch's own bound of its tangent, not merely near it")

	report, err := chamfered.Document().Verify(t.Context(), decad.WithPullDirection(pull))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotContains(t, report.Bodies[0].Undercuts, cone,
		"a tangent the reading cannot resolve is not a proven violation")
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut),
		"nor is it a proven all-clear")
}

// TestCapBlendFlatPatchUndercutRespectsNormalBound is the same claim on the
// single-sample arm. A flat band patch is read from ONE Face.NormalAt, whose
// bound is just as real and carries its own departure term the same way, and a pull perpendicular to the direction that
// reading names leaves the patch's true component on neither proven side of
// zero.
func TestCapBlendFlatPatchUndercutRespectsNormalBound(t *testing.T) {
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)

	patch := faceWithRole(t, chamfered, "chamferCap(end,0,0)")
	require.Equal(t, decad.KindPlane, patch.Surface().Kind())
	n, err := patch.NormalAt(patch.Loops()[0].CoEdges()[0].Start().Position().Value)
	require.NoError(t, err)
	requireRoundingScaleFlatBound(t, n)

	// Perpendicular to the published normal, so the patch's own component is
	// zero to within the bound that normal carries.
	pull := r3.NewVec(n.Value.Y, -n.Value.X, 0)
	unit, ok := pull.Normalize()
	require.True(t, ok)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Less(t, math.Abs(n.Value.Dot(unit)), bound)

	report, err := chamfered.Document().Verify(t.Context(), decad.WithPullDirection(pull))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotContains(t, report.Bodies[0].Undercuts, patch)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}

// requireRoundingScaleFlatBound checks a flat band patch's computed normal,
// which carries Face.NormalAt's own arithmetic proof composed with the
// rounding-scale departure the patch's four independently rounded corners
// leave (capblend_departure.go). Both terms are rounding-scale on the unplaced
// builds this file reads; capblend_departure_test.go reads the same patch
// placed, where the second is orders larger.
func requireRoundingScaleFlatBound(t *testing.T, n decad.VecMeasurement) {
	t.Helper()
	require.Equal(t, decad.Approximate, n.Exactness)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.Less(t, bound, 1e-14)
}

// requireRoundingScaleNormalBound keeps a band with no window skew distinct
// from a mitered patch, whose bound is a geometric fraction of a radian. An
// axis-aligned normal can be Exact; every other normal carries only
// Face.NormalAt's arithmetic bound and the rounding-scale departure an unplaced
// build's own coordinates leave.
func requireRoundingScaleNormalBound(t *testing.T, n decad.VecMeasurement) {
	t.Helper()
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Less(t, bound, 1e-14)
	switch n.Exactness {
	case decad.Exact:
		require.Zero(t, bound)
	case decad.Approximate:
		require.Positive(t, bound)
	default:
		require.Fail(t, `unknown normal exactness`, `%v`, n.Exactness)
	}
}

// TestCapBlendUndercutStillDecidedOnOrdinaryBand keeps the fix reject-only: an
// ordinary chamfer's band is mitered too, and its patches must still be
// CLEARED outright rather than swept into the undecided answer above. Its own
// departure from its Cone is a fraction of a degree, nowhere near the reading
// it would have to cross to leave the pull's clearance.
func TestCapBlendUndercutStillDecidedOnOrdinaryBand(t *testing.T) {
	m := chamferedQuarterDiskPatch(t, 100, 20, 0.5)
	n, err := m.face.NormalAt(m.rulings[0][1])
	require.NoError(t, err)
	// Audited at 0.009722243846487153 — a little over half a degree, and far
	// below the trivial 2 a refused derivation publishes.
	require.Less(t, n.Bound.Mag(), 0.05, "an ordinary setback's departure is small")

	report, err := m.body.Document().Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotNil(t, report.Bodies[0].Undercuts, "the band is still decided")
	require.Empty(t, report.Bodies[0].Undercuts, "and pulling toward the chamfered cap frees it")
	require.False(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}

// TestCapBlendMiteredUndercutStillProven keeps the other half of DX7's answer
// on a mitered band: where the widened reading falls ENTIRELY below zero,
// every point of the ruled patch opposes the pull whatever the patch's own
// departure did, so the band is still a proven undercut rather than a body the
// bound made undecidable.
func TestCapBlendMiteredUndercutStillProven(t *testing.T) {
	m := chamferedQuarterDiskPatch(t, 100, 20, 0.5)
	report, err := m.body.Document().Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, -1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Contains(t, report.Bodies[0].Undercuts, m.face,
		"pulling away from the chamfered cap catches the band patch itself")
	require.Equal(t, decad.Violating, report.Bodies[0].Status)
	require.False(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}

// TestCapBlendMinRadiusUndecidedOnMiteredBand covers DX8. The reduction to the
// receiver's own section rests on every patch being a Plane or a Cone, whose
// ruling direction is straight and whose azimuthal radius never falls below
// the wall's own. A mitered patch is neither: its normal TURNS along a single
// ruling — pinned here from its own boundary — so it is not developable and
// carries curvature the receiver's section never sees. The survey reports that
// as undecided rather than as the proven absence of a concave feature.
func TestCapBlendMinRadiusUndecidedOnMiteredBand(t *testing.T) {
	m := chamferedQuarterDiskPatch(t, 10, 20, 4)

	atSide, _ := m.trueNormalAt(t, 0, 0)
	atCap, _ := m.trueNormalAt(t, 0, 1)
	require.Less(t, atSide.Dot(atCap), 1-1e-6,
		"the normal turns along one straight ruling, which no cone's does")

	report, err := m.body.Document().Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Nil(t, report.Bodies[0].MinRadius)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedMinRadius))
}

// TestCapBlendMinRadiusUndecidedOnCircularBand covers DX8: a whole-turn band
// has one window, so capPatchWindowSkew never rules it out, but its single
// Cone patch's own tag is only enclosed — the held half-angle's cosine and
// sine are never exact — so the patch is not proven to be the surface it
// publishes even unplaced, and the survey refuses rather than answering.
func TestCapBlendMinRadiusUndecidedOnCircularBand(t *testing.T) {
	body := circleProfile(t, 20, 10)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(2))
	require.NoError(t, err)
	report, err := chamfered.Document().Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Nil(t, report.Bodies[0].MinRadius)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedMinRadius))
	require.Equal(t, decad.Suspect, report.Bodies[0].Status)
}

// TestCapBlendMinRadiusUndecidedOnPlacedBand covers DX8's placement gate
// (docs/modify-reach-design.md §8.3): the same band that answers unplaced
// (TestCapBlendMinRadiusMatchesUnchamferedSection's receiver-matching case)
// is undecided once placed away from the world origin, because the
// placement's own independent rounding of every emitted coordinate leaves
// the built patch off its published tag by a proven, nonzero amount.
func TestCapBlendMinRadiusUndecidedOnPlacedBand(t *testing.T) {
	_, box := plateWithDiskHole(t, 50, 50, 10)
	chamfered, err := box.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(box)), decad.LongerThan(units.Millimeters(50))), units.Millimeters(3))
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(1e6, -2.5e5, 8e4))
	require.NoError(t, err)
	motion, err := rot.Then(trans)
	require.NoError(t, err)
	placed, err := chamfered.Placed(motion)
	require.NoError(t, err)

	report, err := placed.Document().Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Nil(t, report.Bodies[0].MinRadius)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedMinRadius))
	require.Equal(t, decad.Suspect, report.Bodies[0].Status)

	// The geometry that forces the refusal: a placed band patch's own
	// NormalAt bound at one of its own boundary vertices is not zero — the
	// built patch's own published numbers do not put it on its tag.
	var patch *decad.Face
	for _, f := range placed.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				patch = f
				break
			}
		}
		if patch != nil {
			break
		}
	}
	require.NotNil(t, patch, "the placed body still carries its chamfer patches")
	v := patch.Loops()[0].CoEdges()[0].Start().Position().Value
	n, err := patch.NormalAt(v)
	require.NoError(t, err)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Greater(t, bound, 1e-12,
		"a placed patch's own published normal does not sit exactly on its tag")
}

func hasDiagnostic(report *decad.Report, code decad.DiagnosticCode) bool {
	for _, d := range report.Diagnostics {
		if d.Code == code {
			return true
		}
	}
	return false
}

// TestCapBlendReflexApexUndercutSurvey is DX7 over the apex patch, through the
// public Verify API. A pull opposing the apex patch alone must list it: the
// role collision that put a wall patch on the apex patch's role made the
// survey read the wall's face AND the wall's geometry, so the apex patch was
// reported for no pull at all. The (0, 0, 1) case is the proven-free reading —
// the band's own normals all tilt toward the chamfered end, so pulling that
// way frees every patch, and the empty list is only correct while the apex
// patch's normal really does point that way.
func TestCapBlendReflexApexUndercutSurvey(t *testing.T) {
	body := reflexLBody(t)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(3))
	require.NoError(t, err)
	apex := apexPatchOf(t, chamfered, "chamferCap(end,")
	doc := chamfered.Document()

	// (-1, -1, 1) opposes the apex patch's inward-and-up normal over its own
	// quarter turn and no other patch in the band: every wall patch's normal
	// is exactly perpendicular to it or better.
	opposing, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(-1, -1, 1)))
	require.NoError(t, err)
	require.Len(t, opposing.Bodies, 1)
	var patches []*decad.Face
	apexReported := false
	for _, f := range opposing.Bodies[0].Undercuts {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				patches = append(patches, f)
			}
		}
		if f == apex {
			apexReported = true
		}
	}
	require.True(t, apexReported, "the apex patch opposes this pull and must be listed")
	require.Equal(t, []*decad.Face{apex}, patches, "and it is the only patch that does")
	require.Equal(t, decad.Violating, opposing.Bodies[0].Status)

	// Pulling toward the chamfered end frees the whole band, apex included.
	free, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	for _, f := range free.Bodies[0].Undercuts {
		for _, o := range f.Origins() {
			require.False(t, strings.HasPrefix(o.Role, "chamferCap("), "role %s", o.Role)
		}
	}
	p := apex.Loops()[0].CoEdges()[0].Start().Position().Value
	n, err := apex.NormalAt(p)
	require.NoError(t, err)
	require.Greater(t, n.Value.Z, 0.0, "the apex patch is free of that pull because it tilts with it")
}

// TestCapBlendUndercutSurvey checks DX7 through the public Verify API: a
// straight-down pull (0,0,-1) is caught as an undercut by the end-cap
// chamfer bevels (they tilt outward-and-up, so extracting downward passes
// through the wider unchamfered wall below — the same reasoning
// prismUndercuts already applies to an ordinary wall), while a straight-up
// pull sees no undercut from them.
func TestCapBlendUndercutSurvey(t *testing.T) {
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)
	doc := chamfered.Document()

	down, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, -1)))
	require.NoError(t, err)
	require.Len(t, down.Bodies, 1)
	require.NotEmpty(t, down.Bodies[0].Undercuts, "pulling away from the chamfered end catches its bevels")
	for _, f := range down.Bodies[0].Undercuts {
		found := false
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				found = true
			}
		}
		require.True(t, found, "every reported undercut is one of the chamfer patches")
	}

	up, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	require.Len(t, up.Bodies, 1)
	for _, f := range up.Bodies[0].Undercuts {
		for _, o := range f.Origins() {
			require.False(t, strings.HasPrefix(o.Role, "chamferCap("), "pulling toward the chamfered end frees its bevels")
		}
	}
}

// TestCapBlendMinRadiusMatchesUnchamferedSection checks DX8: chamfering this
// plate's cap loop also chamfers its hole rim into a whole-turn Cone patch,
// whose own held half-angle only encloses a cosine and sine rather than
// fixing them exactly — so the patch is not proven to be the surface it
// publishes even unplaced, and the survey now refuses instead of publishing
// an answer the patch set does not support, while the receiver's own reading
// is unaffected.
func TestCapBlendMinRadiusMatchesUnchamferedSection(t *testing.T) {
	_, box := plateWithDiskHole(t, 50, 50, 10)
	doc := box.Document()
	before, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, before.Bodies, 1)
	require.NotNil(t, before.Bodies[0].MinRadius)

	chamfered, err := box.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(box)), decad.LongerThan(units.Millimeters(50))), units.Millimeters(3))
	require.NoError(t, err)
	after, err := chamfered.Document().Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, after.Bodies, 1)
	require.Nil(t, after.Bodies[0].MinRadius)
	require.True(t, hasDiagnostic(after, decad.DiagUndecidedMinRadius))
	require.Equal(t, decad.Suspect, after.Bodies[0].Status)
}

// TestCapBlendMinRadiusStillAnsweredOnPlaneOnlyBand keeps a case where DX8
// still answers: a rectangle's sharp cap-loop corners meet two straight
// walls directly, with no separate vertex patch at all, so every patch of
// the band is an axis-aligned `Plane` whose own stamped departure is an
// exact zero unplaced — the survey still returns the receiver's own proven
// absence of a concave feature with no refusal diagnostic.
func TestCapBlendMinRadiusStillAnsweredOnPlaneOnlyBand(t *testing.T) {
	_, box := capBlendBox(t)
	doc := box.Document()
	before, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, before.Bodies, 1)
	require.Nil(t, before.Bodies[0].MinRadius, "a plain rectangle has no concave feature")

	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)
	after, err := chamfered.Document().Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, after.Bodies, 1)
	require.Nil(t, after.Bodies[0].MinRadius, "a proven absence, not a refusal")
	require.False(t, hasDiagnostic(after, decad.DiagUndecidedMinRadius))
	require.Equal(t, decad.Sound, after.Bodies[0].Status)
}
