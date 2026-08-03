package decad_test

import (
	"encoding/json"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// capBlendBox extrudes the 100x60 plate by filletBoxHeight, the same
// receiver filletBox builds, for cap-loop chamfer tests.
func capBlendBox(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	return filletBox(t)
}

// capLoopEdges selects every rim edge of body's end cap — a complete cap
// loop, RX1's second class.
func capLoopEdges(body *decad.Body) *decad.EdgeQuery {
	return decad.Edges(decad.CreatedBy(decad.CapEnd(body)))
}

// circleProfile extrudes a disk of radius r by height h — a single closed
// circular cap loop with no corner at all.
func circleProfile(t *testing.T, r, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.CreateCircle(c, r)
	s.Fix(c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestCapBlendChamferPolygonLoop(t *testing.T) {
	_, box := capBlendBox(t)
	const d = 5.0
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	vol, err := chamfered.Volume()
	require.NoError(t, err)
	// Independent closed form: the base prism minus the top d, plus the
	// band's exact area-integral over the linear offset family.
	const L, W, H = 100.0, 60.0, filletBoxHeight
	slab := L * W * (H - d)
	band := d * ((L * W) - (L+W)*d + (4.0/3.0)*d*d)
	want := slab + band
	require.InDelta(t, want, vol.Value.Mag(), 1e-2)

	area, err := chamfered.Area()
	require.NoError(t, err)
	require.Greater(t, area.Value.Mag(), 0.0)
}

func TestCapBlendChamferCircularRim(t *testing.T) {
	const R, H, d = 30.0, 20.0, 5.0
	disk := circleProfile(t, R, H)
	chamfered, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	vol, err := chamfered.Volume()
	require.NoError(t, err)
	R1 := R - d
	frustum := math.Pi * d / 3 * (R*R + R*R1 + R1*R1)
	slab := math.Pi * R * R * (H - d)
	want := slab + frustum
	require.InDelta(t, want, vol.Value.Mag(), 1e-1)

	area, err := chamfered.Area()
	require.NoError(t, err)
	require.Greater(t, area.Value.Mag(), 0.0)
}

func TestCapBlendPartialLoopSelectionRefused(t *testing.T) {
	_, box := capBlendBox(t)
	// The 100x60 plate's end-cap loop has two edges parallel to X and two
	// parallel to Y; selecting only the X-parallel pair is a proper,
	// non-empty part of the loop.
	q := decad.Edges(decad.CreatedBy(decad.CapEnd(box)), decad.ParallelTo(r3.NewVec(1, 0, 0)))
	matched, err := q.SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, matched, 2, `a proper subset of the four-edge loop`)
	_, err = box.Chamfer(q, units.Millimeters(5))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestCapBlendMixedCapAndLateralSelectionRefused(t *testing.T) {
	_, box := capBlendBox(t)
	verticalMatched, err := verticalEdges().SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, verticalMatched, 4, `the box's four lateral edges`)
	// LongerThan(15mm) matches every lateral edge (20mm) AND every cap rim
	// edge (100mm/60mm) — a genuine mix of the two classes.
	q := decad.Edges(decad.LongerThan(units.Millimeters(15)))
	matched, err := q.SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, matched, 12)
	_, err = box.Chamfer(q, units.Millimeters(5))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestCapBlendLateralOnlySelectionUsesBasePath(t *testing.T) {
	_, box := capBlendBox(t)
	_, err := box.Chamfer(verticalEdges(), units.Millimeters(5))
	require.NoError(t, err)
}

// bothCapLoops selects every rim edge of BOTH complete cap loops of a
// filletBox-shaped receiver: every edge longer than the box's own height
// (20mm) is a rim edge (length 100 or 60), and every lateral edge (length
// exactly 20mm) is excluded — the union of the two complete loops, RX1's
// "spanning two complete cap loops" case, without needing an OR combinator
// the selector vocabulary does not have.
func bothCapLoops() *decad.EdgeQuery {
	return decad.Edges(decad.LongerThan(units.Millimeters(21)))
}

func TestCapBlendOppositeBandsMeetingRefused(t *testing.T) {
	_, box := capBlendBox(t)
	q := bothCapLoops()
	edges, err := q.SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, edges, 8, `both caps' complete loops`)
	// filletBoxHeight is 20; a 10mm setback on both caps of the SAME loop
	// selection reaches exactly the midpoint — refused (SX7).
	_, err = box.Chamfer(q, units.Millimeters(10))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestCapBlendBothCapsBuildsWhenClear(t *testing.T) {
	_, box := capBlendBox(t)
	q := bothCapLoops()
	chamfered, err := box.Chamfer(q, units.Millimeters(5))
	require.NoError(t, err)
	requireManifold(t, chamfered)
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Greater(t, vol.Value.Mag(), 0.0)
	require.Less(t, vol.Value.Mag(), 100*60*filletBoxHeight)
}

func TestCapBlendCarrierCollapseRefused(t *testing.T) {
	const R, H = 4.0, 20.0
	disk := circleProfile(t, R, H)
	// A setback at or beyond the radius drops the circular carrier (SX6).
	_, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(R))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestCapBlendReflexCornerBuilds(t *testing.T) {
	// An L-shaped section has one reflex corner; its cap-loop chamfer needs
	// a Cone-with-apex patch there.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(0, 0), s.CreatePoint(40, 0), s.CreatePoint(40, 20),
		s.CreatePoint(20, 20), s.CreatePoint(20, 40), s.CreatePoint(0, 40),
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	s.Fix(pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)

	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(3))
	require.NoError(t, err)
	requireManifold(t, chamfered)
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Greater(t, vol.Value.Mag(), 0.0)
}

func TestCapBlendHoleLoopNestingPreserved(t *testing.T) {
	_, box := plateWithDiskHole(t, 50, 50, 10)
	// Chamfer only the outer loop's end cap, leaving the hole loop untouched.
	q := decad.Edges(decad.CreatedBy(decad.CapEnd(box)), decad.LongerThan(units.Millimeters(50)))
	chamfered, err := box.Chamfer(q, units.Millimeters(3))
	require.NoError(t, err)
	requireManifold(t, chamfered)
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Greater(t, vol.Value.Mag(), 0.0)
}

// TestCapBlendRecipeRoundTrips checks the cap-loop chamfer records the same
// Step shape a lateral chamfer does (op, receiver input, unresolved selector,
// distance, no options) and that it JSON round-trips exactly — the same
// check TestChamferRecipeAndRetire runs for the base path. The evaluator
// re-derives the same classification and geometry from that Step alone
// (deterministic selector resolution + closed-form gates), which is what
// makes a replay reproduce the same seeds, expansion and result.
func TestCapBlendRecipeRoundTrips(t *testing.T) {
	const d = 5.0
	doc, box := capBlendBox(t)
	body, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())

	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	step := recipe.Steps[1]
	require.Equal(t, decad.OpChamfer, step.Op)
	require.Equal(t, []decad.StepRef{0}, step.Inputs)
	require.Len(t, step.Selectors, 1)
	require.Len(t, step.Values, 1)
	require.True(t, step.Values[0].Equal(units.Millimeters(d), 1e-12))
	require.Nil(t, step.Opts, `a cap-loop chamfer Step takes no options this increment`)

	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded cap-loop chamfer recipe round-trips`)
}

func TestCapBlendFailedCallLeavesReceiverLiveAndRecipeUnchanged(t *testing.T) {
	doc, box := capBlendBox(t)
	before := doc.Recipe()
	q := decad.Edges(decad.CreatedBy(decad.CapStart(box)), decad.CreatedBy(decad.CapEnd(box)))
	_, err := box.Chamfer(q, units.Millimeters(10)) // reaches SX7
	require.Error(t, err)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestCapBlendSX10RefusesFurtherModify(t *testing.T) {
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)

	_, err = chamfered.Chamfer(verticalEdges(), units.Millimeters(1))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	_, err = chamfered.Fillet(verticalEdges(), units.Millimeters(1))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	_, err = chamfered.Shell(decad.Faces(decad.FaceCreatedBy(decad.CapStart(chamfered))), units.Millimeters(1))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// capLoopEdgesOn selects every rim edge of the named cap (start or end).
func capLoopEdgesOn(body *decad.Body, end bool) *decad.EdgeQuery {
	if end {
		return decad.Edges(decad.CreatedBy(decad.CapEnd(body)))
	}
	return decad.Edges(decad.CreatedBy(decad.CapStart(body)))
}

// patchNormalsByRolePrefix returns every chamferCap face's own outward
// normal (via Face.NormalAt at a point recovered from its own boundary
// vertex), keyed by role — used to check orientation directly rather than
// trust the closed-form derivation alone.
func patchNormalsByRolePrefix(t *testing.T, body *decad.Body, prefix string) map[string]r3.Vec {
	t.Helper()
	out := map[string]r3.Vec{}
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if !strings.HasPrefix(o.Role, prefix) {
				continue
			}
			loops := f.Loops()
			require.NotEmpty(t, loops)
			coedges := loops[0].CoEdges()
			require.NotEmpty(t, coedges)
			p := coedges[0].Start().Position().Value
			n, err := f.NormalAt(p)
			require.NoError(t, err)
			out[o.Role] = n.Value
		}
	}
	return out
}

// TestCapBlendPlanePatchNormalOutwardBothCaps checks the sign of the Z
// component of every straight-wall chamfer patch's outward normal on the
// rectangular box, for BOTH the start and the end cap: since a start-cap
// band and an end-cap band tilt toward OPPOSITE caps (the exterior end of
// each band), the two cases must read opposite Z signs — the asymmetry a
// naive "the Plane's own u x v is always outward" assumption misses.
func TestCapBlendPlanePatchNormalOutwardBothCaps(t *testing.T) {
	for _, tc := range []struct {
		name    string
		end     bool
		wantPos bool // true: normal Z component must be positive
	}{
		{"end cap tilts toward +Z (the chamfered end)", true, true},
		{"start cap tilts toward -Z (the chamfered end)", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, box := capBlendBox(t)
			chamfered, err := box.Chamfer(capLoopEdgesOn(box, tc.end), units.Millimeters(5))
			require.NoError(t, err)
			prefix := "chamferCap(end,"
			if !tc.end {
				prefix = "chamferCap(start,"
			}
			normals := patchNormalsByRolePrefix(t, chamfered, prefix)
			require.Len(t, normals, 4, "one Plane patch per rectangle wall")
			for role, n := range normals {
				if tc.wantPos {
					require.Greater(t, n.Z, 0.0, "role %s", role)
				} else {
					require.Less(t, n.Z, 0.0, "role %s", role)
				}
				require.InDelta(t, 1.0, n.Len(), 1e-9, "role %s: NormalAt must be unit", role)
			}
		})
	}
}

// TestCapBlendConePatchNormalOutwardBothCaps is the circular-rim analog of
// the Plane check: the cone chamfer patch's normal must tilt toward whichever
// cap is chamfered, for both caps.
func TestCapBlendConePatchNormalOutwardBothCaps(t *testing.T) {
	for _, tc := range []struct {
		name    string
		end     bool
		wantPos bool
	}{
		{"end cap", true, true},
		{"start cap", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const R, H = 30.0, 20.0
			disk := circleProfile(t, R, H)
			chamfered, err := disk.Chamfer(capLoopEdgesOn(disk, tc.end), units.Millimeters(5))
			require.NoError(t, err)
			prefix := "chamferCap(end,"
			if !tc.end {
				prefix = "chamferCap(start,"
			}
			normals := patchNormalsByRolePrefix(t, chamfered, prefix)
			require.Len(t, normals, 1, "one whole-turn Cone patch")
			for role, n := range normals {
				if tc.wantPos {
					require.Greater(t, n.Z, 0.0, "role %s", role)
				} else {
					require.Less(t, n.Z, 0.0, "role %s", role)
				}
				require.InDelta(t, 1.0, n.Len(), 1e-9, "role %s: NormalAt must be unit", role)
			}
		})
	}
}

// TestCapBlendReflexApexNormalOutward checks the reflex corner's Cone-apex
// patch: its outward normal must point AWAY from the corner (into the
// former void the chamfer fills) and, for an end-cap chamfer, toward +Z.
func TestCapBlendReflexApexNormalOutward(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(0, 0), s.CreatePoint(40, 0), s.CreatePoint(40, 20),
		s.CreatePoint(20, 20), s.CreatePoint(20, 40), s.CreatePoint(0, 40),
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	s.Fix(pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(capLoopEdgesOn(body, true), units.Millimeters(3))
	require.NoError(t, err)

	normals := patchNormalsByRolePrefix(t, chamfered, "chamferCap(end,")
	// The reflex corner is at (20, 20); its apex patch's outward normal must
	// point away from that corner (a positive dot with the vector from the
	// corner to the sample point) and, exactly like a regular wall patch,
	// TOWARD the chamfered end: the offset carrier is an erosion (a subset of
	// the original material) at a reflex corner as much as a convex one, so
	// material retreats toward the cap there too
	// (capblend_geom.go's fixPatchOrientation comment).
	found := false
	for _, f := range chamfered.Faces() {
		for _, o := range f.Origins() {
			if !strings.HasPrefix(o.Role, "chamferCap(end,") {
				continue
			}
			loops := f.Loops()
			if len(loops[0].CoEdges()) != 3 {
				continue // the reflex apex patch is the only 3-edge patch
			}
			found = true
			p := loops[0].CoEdges()[0].Start().Position().Value
			// The corner's own WORLD position is read off the topology
			// itself (the apex vertex, shared by slantOut's end) rather than
			// assumed from the sketch's local (u, v) — the sketch plane's
			// frame axes need not align with world (X, Y, Z).
			corner := loops[0].CoEdges()[1].End().Position().Value
			away := p.Sub(corner)
			n := normals[o.Role]
			require.Greater(t, n.Dot(away), 0.0, "outward from the corner")
			require.Greater(t, n.Z, 0.0, "toward the chamfered end")
		}
	}
	require.True(t, found, "the reflex corner must produce an apex patch")
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

// TestCapBlendMinRadiusMatchesUnchamferedSection checks DX8: the receiver's
// own MinRadius survey (an unchamfered plate with a hole) must equal the
// chamfered body's own MinRadius reading exactly — the new patches never
// tighten it, per capBlendMinRadius's own proof.
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
	require.NotNil(t, after.Bodies[0].MinRadius)
	require.InDelta(t, before.Bodies[0].MinRadius.Value.Mag(), after.Bodies[0].MinRadius.Value.Mag(), 1e-9)
}

// TestCapBlendStartCapVolumeMatchesEndCap is a regression check for the
// start/end orientation asymmetry TestCapBlendPlanePatchNormalOutwardBothCaps
// also covers: chamfering either cap of the same straight prism by the same
// distance must give the SAME volume (the loop offset and the band's own
// closed-form integral do not depend on which end the band sits at).
func TestCapBlendStartCapVolumeMatchesEndCap(t *testing.T) {
	_, endBox := capBlendBox(t)
	endChamfered, err := endBox.Chamfer(capLoopEdgesOn(endBox, true), units.Millimeters(5))
	require.NoError(t, err)
	endVol, err := endChamfered.Volume()
	require.NoError(t, err)

	_, startBox := capBlendBox(t)
	startChamfered, err := startBox.Chamfer(capLoopEdgesOn(startBox, false), units.Millimeters(5))
	require.NoError(t, err)
	startVol, err := startChamfered.Volume()
	require.NoError(t, err)

	require.InDelta(t, endVol.Value.Mag(), startVol.Value.Mag(), 1e-6)
}

// TestCapBlendHoleLoopChamferVolume chamfers a HOLE loop directly (not just
// leaving it untouched beside a chamfered outer loop, as
// TestCapBlendHoleLoopNestingPreserved does) and checks the result against
// an independent closed form: a countersink widens the hole at the cap from
// its own radius R to R+d, REMOVING material — the outer prism's volume less
// a straight cylinder over the unchanged run and a widening frustum over the
// band.
func TestCapBlendHoleLoopChamferVolume(t *testing.T) {
	const L, H, cx, cy, R, d = 100.0, 10.0, 50.0, 50.0, 10.0, 2.0
	_, box := plateWithDiskHole(t, cx, cy, R)
	q := decad.Edges(decad.CreatedBy(decad.CapEnd(box)), decad.Circular())
	matched, err := q.SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, matched, 1, "the hole loop's single whole-circle edge")

	chamfered, err := box.Chamfer(q, units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)
	vol, err := chamfered.Volume()
	require.NoError(t, err)

	outer := L * L * H
	R1 := R + d
	holeStraight := math.Pi * R * R * (H - d)
	holeFrustum := math.Pi * d / 3 * (R*R + R*R1 + R1*R1)
	want := outer - holeStraight - holeFrustum
	require.InDelta(t, want, vol.Value.Mag(), 1e-1)
}

func TestCapBlendBooleanReceiverRefusedSX9(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 3, 3, 13, 7, 6)
	// Lift b off the z=0/z=10 planes a's caps occupy, so the union's contact
	// is a genuine interior crossing rather than a coplanar cap tangency.
	shift, err := r3.Translation(r3.NewVec(0, 0, 2))
	require.NoError(t, err)
	b, err = b.Placed(shift)
	require.NoError(t, err)
	union, err := decad.Union(a, b)
	require.NoError(t, err)
	_, err = union.Chamfer(decad.Edges(decad.Convex()).AtLeast(1), units.Millimeters(1))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
