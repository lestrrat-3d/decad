package decad_test

import (
	"encoding/json"
	"fmt"
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

// TestCapBlendCarrierCollapseRefused pins SX6's sentinel
// (docs/modify-reach-design.md Table SX): eroding a radius-R circle by R or
// more leaves the empty set, so the cap contour the caller asked for does not
// exist and the refusal is ErrDegenerate — the same existence test §4 puts in
// stage 5. The shared offset raises this as Shell's own S11a ErrUnsupported,
// which is correct where a shell body exists and only a trimmed-offset kernel
// is missing; the cap-blend call site is the one that translates it, and
// shell_test.go's own "outward offset erasing a hole is S11a" subtest is the
// other half — it still reads ErrUnsupported, which a global re-sentinelling
// of errOffsetDrop would have broken.
func TestCapBlendCarrierCollapseRefused(t *testing.T) {
	for _, d := range []float64{4.0, 5.0} {
		t.Run(fmt.Sprintf("d=%g", d), func(t *testing.T) {
			const R, H = 4.0, 20.0
			disk := circleProfile(t, R, H)
			_, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(d))
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.NotErrorIs(t, err, decad.ErrUnsupported,
				`SX6 answers to one sentinel; the shell's S11a must not ride along`)
		})
	}
}

// reflexLBody extrudes the L-shaped section — five convex corners and ONE
// reflex corner at (20, 20), walked counter-clockwise — by reflexLHeight. Its
// cap-loop chamfer is the only case that mints a Cone-with-apex patch.
const (
	reflexLHeight = 20.0
	reflexLArea   = 1200.0
	reflexLPerim  = 160.0
)

func reflexLBody(t *testing.T) *decad.Body {
	t.Helper()
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
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(reflexLHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// reflexLChamferVolume is the L section's own cap-chamfered volume, derived
// independently of the evaluator: the straight slab plus the integral of the
// eroded section's area over the setback. Eroding the L by t keeps a polygon
// whose area is A - P*t + t^2 * sum(f_i) over its corners, with f = tan(45
// degrees) = 1 at each of the five convex (miter) corners and f = -pi/4 at the
// reflex one, whose erosion rounds off a quarter disk of radius t rather than
// mitering. Integrating that over [0, d] gives the band.
func reflexLChamferVolume(d float64) float64 {
	corner := 5 - math.Pi/4
	band := reflexLArea*d - reflexLPerim*d*d/2 + corner*d*d*d/3
	return reflexLArea*(reflexLHeight-d) + band
}

// apexPatchOf returns the reflex corner's Cone-with-apex patch — the one
// chamferCap face on the named cap whose loop is the three-coedge arc/slant/
// slant triangle — and asserts there is exactly one.
func apexPatchOf(t *testing.T, b *decad.Body, prefix string) *decad.Face {
	t.Helper()
	var found []*decad.Face
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if !strings.HasPrefix(o.Role, prefix) {
				continue
			}
			if len(f.Loops()[0].CoEdges()) == 3 {
				found = append(found, f)
			}
		}
	}
	require.Len(t, found, 1, "exactly one reflex-corner apex patch")
	return found[0]
}

func TestCapBlendReflexCornerBuilds(t *testing.T) {
	// An L-shaped section has one reflex corner; its cap-loop chamfer needs
	// a Cone-with-apex patch there.
	body := reflexLBody(t)

	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(3))
	require.NoError(t, err)
	requireManifold(t, chamfered)
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Greater(t, vol.Value.Mag(), 0.0)
	// The apex patch's own angular window and flux orientation both enter this
	// number, so the independent closed form pins them.
	require.InDelta(t, reflexLChamferVolume(3), vol.Value.Mag(), 1e-6)
}

// TestCapBlendReflexApexPatchRolesDistinct pins Table BX row BX3's `p` as the
// patch's own order in the result's `capBlendPayload`: an apex patch is keyed
// by a CORNER and a wall patch by a WALL, and capOffsetJoins puts corner i at
// walks[i].start, so keying either role by that index puts two faces on one
// role string. Every last-wins reader then collapses onto whichever face was
// built second — facesByRole, FaceCreatedBy, and the survey's role lookup.
func TestCapBlendReflexApexPatchRolesDistinct(t *testing.T) {
	body := reflexLBody(t)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(3))
	require.NoError(t, err)

	count := map[string]int{}
	for _, f := range chamfered.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				count[o.Role]++
			}
		}
	}
	// Six walls plus one reflex apex.
	require.Len(t, count, 7)
	for role, n := range count {
		require.Equal(t, 1, n, "role %s must name exactly one face", role)
	}
}

// TestCapBlendReflexApexArcLength pins the apex patch's cap-level connector.
// The inward offset's reflex corner closes with a CLOCKWISE arc of radius d
// spanning the corner's own reflex turn — a quarter turn here, 3*pi/2 mm long
// — never the counter-clockwise complement, which is the arc on the far side
// of the corner and three times as long.
func TestCapBlendReflexApexArcLength(t *testing.T) {
	const d = 3.0
	body := reflexLBody(t)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	apex := apexPatchOf(t, chamfered, "chamferCap(end,")
	var arcs int
	for _, ce := range apex.Loops()[0].CoEdges() {
		e := ce.Edge()
		if _, ok := e.Curve().(decad.Arc3); !ok {
			continue
		}
		arcs++
		length, err := e.Length()
		require.NoError(t, err)
		require.InDelta(t, d*math.Pi/2, length.Value.Mag(), 1e-9)
		// Both feet sit a full setback from the corner, and the arc between
		// them is the shorter way around.
		start := ce.Start().Position().Value
		end := ce.End().Position().Value
		require.InDelta(t, d*math.Sqrt2, start.Sub(end).Len(), 1e-9, "a quarter-turn chord")
	}
	require.Equal(t, 1, arcs, "the apex patch's one cap-level arc")
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
// patch against a reference derived from the topology alone, never from the
// builder's own formula.
//
// The offset is an EROSION, and at a reflex corner the eroded boundary is the
// arc of radius d about the corner with the surviving material radially
// OUTSIDE it — the sector the arc cuts off is exactly what the chamfer
// removes. So this patch's cone has the VOID inside and the solid outside,
// and its outward normal points radially INWARD, at the corner's own axis,
// while tilting toward the chamfered cap like every other patch in the band.
//
// The reference is built from the apex vertex, the arc foot and the cap face's
// own outward normal: with dc = ds = d the cone stands at 45 degrees, so the
// outward normal is (capNormal - radialUnit)/sqrt(2) exactly. The check is a
// vector identity rather than a pair of sign predicates, so an inverted normal
// fails it — and so does the radially-outward reading, which is EXACTLY
// perpendicular to the truth here and passes every sign test written against a
// cone ruling.
func TestCapBlendReflexApexNormalOutward(t *testing.T) {
	for _, tc := range []struct {
		name string
		end  bool
	}{
		{"end cap", true},
		{"start cap", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const d = 3.0
			body := reflexLBody(t)
			chamfered, err := body.Chamfer(capLoopEdgesOn(body, tc.end), units.Millimeters(d))
			require.NoError(t, err)

			prefix := "chamferCap(end,"
			capRole := roleCapEnd
			if !tc.end {
				prefix = "chamferCap(start,"
				capRole = roleCapStart
			}
			apex := apexPatchOf(t, chamfered, prefix)

			// The cap face's own outward normal is the band's axial reference;
			// it is independently pinned by the plane- and cone-patch tests.
			capFace := faceWithRole(t, chamfered, capRole)
			capN, err := capFace.NormalAt(capFace.Loops()[0].CoEdges()[0].Start().Position().Value)
			require.NoError(t, err)
			axis := capN.Value

			// The apex vertex is the ORIGINAL corner, held where the receiver
			// had it; the arc's own start is one foot of the offset connector.
			coedges := apex.Loops()[0].CoEdges()
			corner := coedges[1].End().Position().Value
			p := coedges[0].Start().Position().Value
			v := p.Sub(corner)
			require.InDelta(t, d, v.Dot(axis), 1e-9, "one setback along the cap's own outward sense")
			radial, ok := v.Sub(axis.Scale(v.Dot(axis))).Normalize()
			require.True(t, ok)

			want, ok := axis.Sub(radial).Normalize()
			require.True(t, ok)
			n, err := apex.NormalAt(p)
			require.NoError(t, err)
			require.InDelta(t, 1.0, n.Value.Dot(want), 1e-9, "the apex patch's outward normal")
			// Restated as the two facts the identity carries, so a failure
			// names which half moved.
			require.Less(t, n.Value.Dot(radial), 0.0, "toward the corner's own axis")
			require.Greater(t, n.Value.Dot(axis), 0.0, "toward the chamfered end")
		})
	}
}

// faceWithRole returns the single face carrying role on b.
func faceWithRole(t *testing.T, b *decad.Body, role string) *decad.Face {
	t.Helper()
	var found []*decad.Face
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if o.Role == role {
				found = append(found, f)
			}
		}
	}
	require.Len(t, found, 1, "exactly one %s face", role)
	return found[0]
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

// plateWithRectHole extrudes a 100×100 plate with a side×side square hole
// centred at (50, 50), by 10 mm — a straight prism whose section is a
// rectangle with a POLYGONAL hole, so the hole loop is a clockwise walk of
// four straight walls and four corners.
func plateWithRectHole(t *testing.T, side float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(outer.A)
	s.CreateRectangle(50-side/2, 50-side/2, 50+side/2, 50+side/2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-square-hole region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendPolygonalHoleChamferVolume is the hole-loop case
// TestCapBlendHoleLoopChamferVolume cannot reach. A hole is walked CLOCKWISE,
// so every patch built from that walk faces into the band, while the band's
// two closing disks are absolute areas and so describe the loop's region read
// counter-clockwise. A whole-turn cone hides the mismatch — its own flux does
// not depend on which way round the circle was walked — but a polygonal hole's
// straight walls and quarter-turn apex cones both do, and each sign is
// independent of the other.
//
// The reference is derived from the section alone, never from the evaluator:
// at depth h below the chamfered cap the section is the original eroded by h,
// so the outer contributes (L − 2h)² and the hole dilates to
// side² + 4·side·h + π·h² (its four corners round off to quarter disks of
// radius h). Integrating each over [0, d] and adding the straight slab below
// gives the whole volume in closed form.
func TestCapBlendPolygonalHoleChamferVolume(t *testing.T) {
	const L, H, side, d = 100.0, 10.0, 20.0, 2.0
	box := plateWithRectHole(t, side)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	// The hole's four corners are reflex under the inward offset, so the band
	// really does mint apex cones — without them this case degenerates into
	// the straight-wall one and stops testing what it is here for.
	apexes := 0
	for _, f := range chamfered.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") && len(f.Loops()[0].CoEdges()) == 3 {
				apexes++
			}
		}
	}
	require.Equal(t, 4, apexes, `one apex cone per corner of the dilating hole`)

	vol, err := chamfered.Volume()
	require.NoError(t, err)
	straight := (L*L - side*side) * (H - d)
	outerBand := (L*L*L - (L-2*d)*(L-2*d)*(L-2*d)) / 6
	holeBand := side*side*d + 2*side*d*d + math.Pi*d*d*d/3
	require.InDelta(t, straight+outerBand-holeBand, vol.Value.Mag(), 1e-6)
}

// TestCapBlendThroughAllStopsAtBuiltExtent is Table DX row DX5: a through-all
// sweep reads the stop body's extent as an EXACT endpoint, so a cap-blend
// body must report the extent its own patches attain. Padding the receiver
// prism's extent by the setback overruns the sweep by exactly that setback,
// silently and with no diagnostic — the sweep runs past a body that ends at
// the sketch plane's own 20 mm.
func TestCapBlendThroughAllStopsAtBuiltExtent(t *testing.T) {
	const height, d = 20.0, 5.0
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	chamfered, err := plate.Chamfer(capLoopEdges(plate), units.Millimeters(d))
	require.NoError(t, err)

	// The chamfer offsets INTO the material at the cap and leaves [z0, z1]
	// alone, so every vertex the build made still sits within the receiver's
	// own sweep.
	zHi := math.Inf(-1)
	for _, v := range chamfered.Vertices() {
		zHi = math.Max(zHi, v.Position().Value.Z)
	}
	require.InDelta(t, height, zHi, 1e-9)

	pin, err := doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, pin, 20*20*height)
	requireBounds(t, pin, 120, 0, 0, 140, 20, height)
}

// TestCapBlendThroughAllBehindPlaneRefused is the same reading's other half:
// "is this body in the sweep's path" is decided by the same extent, so a
// padded one puts a body the sweep never reaches into the path. A chamfered
// plate lying entirely behind the sketch plane must leave the sweep with no
// stop at all rather than build a body in empty space and record a dependency
// on a plate it never meets.
func TestCapBlendThroughAllBehindPlaneRefused(t *testing.T) {
	const height, d, drop = 20.0, 5.0, 24.0
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	chamfered, err := plate.Chamfer(capLoopEdges(plate), units.Millimeters(d))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(0, 0, -drop))
	require.NoError(t, err)
	behind, err := chamfered.Placed(shift)
	require.NoError(t, err)

	zHi := math.Inf(-1)
	for _, v := range behind.Vertices() {
		zHi = math.Max(zHi, v.Position().Value.Z)
	}
	require.InDelta(t, height-drop, zHi, 1e-9, "the whole body sits behind the sketch plane")

	before := doc.Recipe()
	_, err = doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Along})
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Recipe())
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

// TestCapBlendBoundsAreAttainedNotPadded checks §8.4's "bounds from patch
// extrema, not a loose box": the cap contour offsets INTO the material and
// [z0, z1] is unchanged, so a cap-loop chamfer of the 100x60x20 plate holds
// exactly the receiver's own extent whatever the setback is. Box.Min and
// Box.Max are positions and Bound is the absolute error on them, so a box
// widened outward by d has an error of d and may not report zero.
func TestCapBlendBoundsAreAttainedNotPadded(t *testing.T) {
	for _, d := range []float64{1, 5, 9} {
		t.Run(fmt.Sprintf("d=%v", d), func(t *testing.T) {
			_, box := capBlendBox(t)
			chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
			require.NoError(t, err)

			bounds, err := chamfered.Bounds()
			require.NoError(t, err)
			require.Equal(t, r3.NewVec(0, 0, 0), bounds.Min)
			require.Equal(t, r3.NewVec(100, 60, filletBoxHeight), bounds.Max)
			require.Equal(t, decad.Exact, bounds.Exactness)
			require.Zero(t, bounds.Bound.Mag())

			// The same extent read independently off the body's own vertices:
			// every face of this body is flat, so each extreme is attained at a
			// vertex, and the box may not sit outside that hull.
			lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
			hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
			for _, e := range chamfered.Edges() {
				for _, v := range []*decad.Vertex{e.Start(), e.End()} {
					p := v.Position().Value
					lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
					hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
				}
			}
			require.Equal(t, lo, bounds.Min, `the box's Min is a point the body holds`)
			require.Equal(t, hi, bounds.Max, `the box's Max is a point the body holds`)
		})
	}
}

// TestCapBlendNestingRefusalKeepsDegenerate checks §4 stage 6's sentinel
// discipline on the base S9 row the cap-loop chamfer inherits. A 60x60 plate
// holding a radius-14 hole at (30, 30), chamfered by 12, offsets the outer
// loop inward to a 36x36 rectangle and grows the hole to radius 26, which
// swallows it: the two loops stay strictly disjoint, so the audit reaches S9
// and decides the nesting is broken. No such body exists, which is the
// opposite existence claim to SX7/SX12's, so the refusal answers to
// ErrDegenerate and not to ErrUnsupported.
func TestCapBlendNestingRefusalKeepsDegenerate(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 60, 60)
	s.Fix(outer.A)
	s.CreateCircle(s.CreatePoint(30, 30), 14)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the plate-with-disk region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
	require.NoError(t, err)
	before := doc.Recipe()

	_, err = body.Chamfer(capLoopEdges(body), units.Millimeters(12))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate, `an S9 nesting refusal keeps its own sentinel`)
	require.NotErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Recipe())
}

// TestCapBlendUndercutOrderIsDeterministic checks that Table BX row BX3's
// "deterministic patch order" survives into the DX7 survey's public output. A
// straight-down pull catches all four end-cap bevels of the plate, so the
// reported sequence IS the payload's own patch order; a caller may diff or
// golden-test Report.Bodies[i].Undercuts, so repeated calls must agree.
func TestCapBlendUndercutOrderIsDeterministic(t *testing.T) {
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)
	doc := chamfered.Document()

	want := []string{
		`chamferCap(end,0,0)`,
		`chamferCap(end,0,1)`,
		`chamferCap(end,0,2)`,
		`chamferCap(end,0,3)`,
	}
	for range 20 {
		rep, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, -1)))
		require.NoError(t, err)
		require.Len(t, rep.Bodies, 1)
		got := make([]string, 0, len(rep.Bodies[0].Undercuts))
		for _, f := range rep.Bodies[0].Undercuts {
			require.Len(t, f.Origins(), 1)
			got = append(got, f.Origins()[0].Role)
		}
		require.Equal(t, want, got, `the same body under the same pull reports the same faces in the same order`)
	}
}

// TestCapBlendConePatchKeepsTaperAtHugeRadius pins the surface KIND a chamfer
// band patch is built with against what its two stored radii ARE, never against
// how close they are. A 1e12 mm disk chamfered by 1 micron offsets its cap
// contour to a radius that differs from the wall's, so the patch is a cone —
// but the difference is a millionth of a part in the radius, which any relative
// tolerance on the radial change reads as zero. Calling that a cylinder throws
// away the taper the chamfer exists to create, and the DX7 undercut survey then
// answers about a shape nobody asked for: a cylinder's normal has no axial
// component at all, so the bevel reads as free of a pull it plainly opposes.
//
// The volume and area readings are checked here as well, because changing a
// surface kind may move them: they are integrated from the patch's own recorded
// radii rather than from its carrier surface, so both must stand unchanged.
func TestCapBlendConePatchKeepsTaperAtHugeRadius(t *testing.T) {
	const R, H, d = 1e12, 10.0, 1e-3
	disk := circleProfile(t, R, H)
	chamfered, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	patch := faceWithRole(t, chamfered, `chamferCap(end,0,0)`)
	cone, ok := patch.Surface().(decad.Cone)
	require.True(t, ok, `a band patch whose two radii differ is a cone, got %T`, patch.Surface())
	// dc = ds = d stands every chamfer cone at 45 degrees in exact arithmetic;
	// at this radius the offset rounds to the nearest 8 ulps of 1e12, so the
	// held half-angle sits near it without reaching it. What matters is that it
	// is a real taper and not the zero a cylinder would carry.
	require.InDelta(t, math.Pi/4, cone.HalfAngle.Mag(), 0.05)

	// The two mass readings are integrated from the recorded radii, so the kind
	// decision must leave them exactly where the closed form puts them. Both are
	// judged against the body's own proven bound.
	capRadius := R - d
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	wantVol := math.Pi*R*R*(H-d) + math.Pi*d/3*(R*R+R*capRadius+capRadius*capRadius)
	require.InDelta(t, wantVol, vol.Value.Mag(), vol.Bound.Mag())

	area, err := chamfered.Area()
	require.NoError(t, err)
	slant := math.Hypot(R-capRadius, d)
	wantArea := math.Pi*R*R + 2*math.Pi*R*(H-d) + math.Pi*(R+capRadius)*slant + math.Pi*capRadius*capRadius
	require.InDelta(t, wantArea, area.Value.Mag(), area.Bound.Mag())

	// DX7 through the public API: the band tilts toward the chamfered end, so
	// pulling away from it is an undercut — the same reading the ordinary-scale
	// plate gives in TestCapBlendUndercutSurvey.
	rep, err := chamfered.Document().Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, -1)))
	require.NoError(t, err)
	require.Len(t, rep.Bodies, 1)
	require.Equal(t, []*decad.Face{patch}, rep.Bodies[0].Undercuts,
		`the tapered band opposes a pull away from the chamfered end`)
}

// TestCapBlendUnrepresentableRadialChangeRefused is the other half of the same
// rule. Where the setback is so small beside the radius that `R - d` rounds back
// onto `R`, the cap contour this evaluator would build is the original circle
// and the band's patch really is a cylinder — a different solid from the one the
// caller asked for. The call refuses rather than return it: the body exists (its
// taper is real, just finer than float64 names at that radius), so the sentinel
// is ErrUnsupported, and the receiver and recipe are untouched.
func TestCapBlendUnrepresentableRadialChangeRefused(t *testing.T) {
	const R, H, d = 1e12, 10.0, 1e-9
	require.Equal(t, R, R-d, `the premise: this setback is below the radius's own float64 spacing`)
	disk := circleProfile(t, R, H)
	doc := disk.Document()
	before := doc.Recipe()

	_, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(d))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{disk}, doc.Bodies())
}

// TestCapBlendPatchFacesReportTheirOwnArea checks that every constructed patch
// Face carries the area its own geometry has. An unset Face.area is a zero value
// with a zero bound, which the public reading publishes as an EXACT zero — not a
// missing answer but a wrong one, asserted as fact about a face the body's own
// area sum meanwhile counts in full. The two readings must agree, and neither
// may claim Exact: a plane patch's area is a float cross product, a norm and a
// sum, and a cone patch's carries a square root.
func TestCapBlendPatchFacesReportTheirOwnArea(t *testing.T) {
	const d = 5.0
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)

	// The plate's four bevels are trapezoids between the original rim and the
	// rim offset inward by d, slanted over the setback: an independent closed
	// form per wall, derived from the section alone.
	slant := math.Hypot(d, d)
	want := map[string]float64{
		`chamferCap(end,0,0)`: (100 + (100 - 2*d)) / 2 * slant,
		`chamferCap(end,0,1)`: (60 + (60 - 2*d)) / 2 * slant,
		`chamferCap(end,0,2)`: (100 + (100 - 2*d)) / 2 * slant,
		`chamferCap(end,0,3)`: (60 + (60 - 2*d)) / 2 * slant,
	}
	var faceSum float64
	seen := 0
	for _, f := range chamfered.Faces() {
		a, err := f.Area()
		require.NoError(t, err)
		faceSum += a.Value.Mag()
		role := f.Origins()[0].Role
		w, ok := want[role]
		if !ok {
			continue
		}
		seen++
		require.InDelta(t, w, a.Value.Mag(), 1e-9, `role %s`, role)
		require.Greater(t, a.Bound.Mag(), 0.0, `role %s: a float-computed area is never Exact`, role)
		require.Equal(t, decad.Approximate, a.Exactness, `role %s`, role)
	}
	require.Equal(t, len(want), seen, `every bevel patch was inspected`)

	// A caller adding up the faces and a caller asking the body must be told the
	// same thing about the same surface.
	area, err := chamfered.Area()
	require.NoError(t, err)
	require.InDelta(t, area.Value.Mag(), faceSum, area.Bound.Mag()+1e-9)
}

// smallSkewSection extrudes the CCW loop (0,1), (0.5,0.5), (0.5,0), (1,0),
// (1,0.5), (0.51,0.51) by h. Its footprint spans a full unit in x and y while
// its sweep is a thousandth of that, so the bounding box is extremely flat — the
// shape whose farthest corner from an interior point is one of the six corners
// that are neither Min nor Max.
func smallSkewSection(t *testing.T, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	coords := [][2]float64{{0, 1}, {0.5, 0.5}, {0.5, 0}, {1, 0}, {1, 0.5}, {0.51, 0.51}}
	pts := make([]*sketch.Point, len(coords))
	for i, c := range coords {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	s.Fix(pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendCentroidBoundEncloses checks the centroid bound for what a bound
// IS — an enclosure — rather than for a particular value. The reported bound is
// the reach of the body's own Bounds box from the estimate, and the true
// centroid may sit anywhere in that box, so the bound must cover every point of
// it. p -> |p - estimate| is convex, so its maximum over the box is attained at
// a CORNER: reading only Min and Max leaves six corners unexamined, and on a box
// far wider than it is tall the farthest corner is among exactly those six.
func TestCapBlendCentroidBoundEncloses(t *testing.T) {
	const h, d = 1e-3, 1e-7
	body := smallSkewSection(t, h)
	// The receiver's own centroid stands in for the chamfered body's true one.
	// The band removes a boundary sliver of relative volume about 1e-10 here, so
	// the true centroid moves by far less than the margin under test.
	receiver, err := body.Centroid()
	require.NoError(t, err)

	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
	bound := centroid.Bound.Mag()

	require.LessOrEqual(t, centroid.Value.Sub(receiver.Value).Len(), bound,
		`the bound must enclose the true centroid, not merely be near the estimate`)

	// And the property the bound claims outright: every point the Bounds box
	// holds is within it, corners included.
	bounds, err := chamfered.Bounds()
	require.NoError(t, err)
	for _, x := range []float64{bounds.Min.X, bounds.Max.X} {
		for _, y := range []float64{bounds.Min.Y, bounds.Max.Y} {
			for _, z := range []float64{bounds.Min.Z, bounds.Max.Z} {
				corner := r3.NewVec(x, y, z)
				require.LessOrEqual(t, corner.Sub(centroid.Value).Len(), bound,
					`corner %v of the bounds box the true centroid may occupy`, corner)
			}
		}
	}
}
