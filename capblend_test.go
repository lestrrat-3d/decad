package decad_test

import (
	"encoding/json"
	"errors"
	"fmt"
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
// closed-form integral do not depend on which end the band sits at). This is
// REFERENCE-FREE — it needs no independent formula, only that mirroring the
// band does not change what it measures — so it catches capblend_geom.go's
// side-window/cap-window defect (docs/modify-reach-design.md §8.3) even
// without a value to check the result against: the quarter-disk case's
// circular wall meets its two line neighbours at a non-tangential miter on
// BOTH ends, so a Cone patch integrated over the wrong (shared, wall-only)
// window there reads a DIFFERENT residual against the true miter locus on the
// start cap than on the end cap (the two bands are not mirror images of the
// SAME error), and the two volumes disagree.
func TestCapBlendStartCapVolumeMatchesEndCap(t *testing.T) {
	for _, tc := range []struct {
		name  string
		build func(t *testing.T) *decad.Body
		d     float64
	}{
		{`rectangular box`, func(t *testing.T) *decad.Body { _, b := capBlendBox(t); return b }, 5},
		{`partially-swept arc (quarter disk)`, func(t *testing.T) *decad.Body { return quarterDiskBody(t, 60, 20) }, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			endBody := tc.build(t)
			endChamfered, err := endBody.Chamfer(capLoopEdgesOn(endBody, true), units.Millimeters(tc.d))
			require.NoError(t, err)
			endVol, err := endChamfered.Volume()
			require.NoError(t, err)

			startBody := tc.build(t)
			startChamfered, err := startBody.Chamfer(capLoopEdgesOn(startBody, false), units.Millimeters(tc.d))
			require.NoError(t, err)
			startVol, err := startChamfered.Volume()
			require.NoError(t, err)

			require.InDelta(t, endVol.Value.Mag(), startVol.Value.Mag(), 1e-6)
		})
	}
}

// quarterDiskBody extrudes the quarter disk (0,0)->(r,0), a CCW arc about the
// origin from (r,0) to (0,r), (0,r)->(0,0), by h — a single circular wall
// meeting a straight neighbour at a non-tangential miter corner at BOTH its
// own ends (docs/modify-reach-design.md §8.3's affected shape).
func quarterDiskBody(t *testing.T, r, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(r, 0)
	py := s.CreatePoint(0, r)
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

// volumeRefPrec and piRef are the reference arithmetic the enclosure check
// below judges against: 400 bits of significand and pi to 62 decimal digits
// (the same expansion Go's own math.Pi constant carries), so the reference is
// nearer the true volume than a float64 can represent and the residual the
// test measures is the SHIPPED value's, not the reference's.
const volumeRefPrec = 400

const piRef = `3.14159265358979323846264338327950288419716939937510582097494459`

// exactChamferedDiskVolume is the volume of a radius-r disk swept h and
// chamfered by d on one cap, in reference arithmetic: the straight slab
// pi*r^2*(h-d) plus the band frustum pi*d/3*(r^2 + r*rc + rc^2) with
// rc = r-d. It restates TestCapBlendConePatchKeepsTaperAtHugeRadius's own
// closed form, evaluated without float64 rounding anywhere.
func exactChamferedDiskVolume(t *testing.T, r, h, d float64) *big.Float {
	t.Helper()
	pi, ok := new(big.Float).SetPrec(volumeRefPrec).SetString(piRef)
	require.True(t, ok)
	bf := func(x float64) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).SetFloat64(x) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Mul(a, b) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Add(a, b) }
	sub := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Sub(a, b) }

	R, H, D := bf(r), bf(h), bf(d)
	rc := sub(R, D)
	slab := mul(pi, mul(mul(R, R), sub(H, D)))
	sum := add(add(mul(R, R), mul(R, rc)), mul(rc, rc))
	band := new(big.Float).SetPrec(volumeRefPrec).Quo(mul(mul(pi, D), sum), bf(3))
	return add(slab, band)
}

// TestCapBlendVolumeBoundEnclosesExactVolume is the promise the whole package
// makes about a Measurement, checked where it is hardest to keep: the true
// value lies inside [Value-Bound, Value+Bound]. The reference is computed in
// 400-bit arithmetic rather than in float64, so what the residual measures is
// the shipped reading's own error and nothing else — a float64 reference at
// these magnitudes carries a residual of its own within an ulp of the one
// under test.
//
// The configurations are the ones the band's cancellation is worst at. A
// chamfer band is a difference of two flux terms of magnitude sweep-height
// times section-area, so the reading's error scales with the SWEEP while the
// band it lands in scales with the SETBACK; the first two rows below run that
// ratio to 1e4 and 1e8. The third is ordinary scale, where the same
// composition must not have become loose.
func TestCapBlendVolumeBoundEnclosesExactVolume(t *testing.T) {
	for _, tc := range []struct {
		name    string
		r, h, d float64
	}{
		{`a 1e12 disk with a 1e-3 setback under a 10 mm sweep`, 1e12, 10, 1e-3},
		{`a 1e6 disk with a 1e-3 setback under a 1e5 mm sweep`, 1e6, 1e5, 1e-3},
		{`ordinary scale`, 30, 20, 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			disk := circleProfile(t, tc.r, tc.h)
			chamfered, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(tc.d))
			require.NoError(t, err)
			vol, err := chamfered.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, vol.Exactness,
				`a flux sum through a trig closed form is never Exact`)

			held := new(big.Float).SetPrec(volumeRefPrec).SetFloat64(vol.Value.Mag())
			residual := new(big.Float).SetPrec(volumeRefPrec).Sub(held, exactChamferedDiskVolume(t, tc.r, tc.h, tc.d))
			got, _ := new(big.Float).Abs(residual).Float64()
			require.LessOrEqual(t, got, vol.Bound.Mag(),
				`the published bound must contain the true volume, not merely look small`)
		})
	}
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

// TestCapBlendUnrepresentableAxialChangeRefused is SX13's axial half, the
// sibling of the radial refusal above on the band's OTHER directrix. The band
// runs between the cap contour at the cap level and the original loop at the
// side level, and the setback displaces both. Under a sweep tall enough that
// `z1 - d` rounds back onto `z1`, the axial displacement is the one that
// vanishes: both contours land on one level and every patch of the band comes
// out flat in the cap plane.
//
// What makes this worth a refusal is the SURFACE, not the volume. The volume the
// collapsed body reports is the correctly rounded volume of the true chamfered
// solid — the real chamfer here removes about 2e-18 mm³ — and the shell stays
// watertight. But a Plane carries its normal with no bound, so each of those four
// flat faces asserts the chamfer's 45-degree taper as a fact about geometry that
// has none, and the DX7 undercut survey reads that assertion straight off the
// surface. The requested body exists, so the sentinel is ErrUnsupported and never
// ErrDegenerate, and the receiver and recipe are untouched.
//
// SX7's band-reach gate cannot catch this and is not meant to: it refuses a
// setback so LARGE beside the sweep that the band passes the far end, while this
// one is smaller than the sweep by twenty-one orders of magnitude.
func TestCapBlendUnrepresentableAxialChangeRefused(t *testing.T) {
	const H, d = 1e12, 1e-9
	require.Equal(t, H, H-d, `the premise: this setback is below the sweep level's own float64 spacing`)

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	tower, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(H), Dir: decad.Along})
	require.NoError(t, err)
	before := doc.Recipe()

	_, err = tower.Chamfer(capLoopEdges(tower), units.Millimeters(d))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{tower}, doc.Bodies())

	// The same tower with a setback the level can name still builds, so the gate
	// refuses the collapse and not the shape.
	chamfered, err := tower.Chamfer(capLoopEdges(tower), units.Millimeters(1e-3))
	require.NoError(t, err)
	requireManifold(t, chamfered)
}

// TestCapBlendPlanePatchVolumeIsExact checks §8.4's opening rule — "report each
// exactly representable result as Exact" — on the shape that can actually keep
// it. Every patch of an all-line cap loop's band is a Plane, whose flux is the
// tetrahedron identity: a polynomial in the payload's own float coordinates, so
// its exact value is a rational and the only rounding in the whole reading is the
// final one into a float64.
//
// The setback is chosen so the true answer IS a float64. A w×h rectangle's
// cap-loop chamfer removes `(w+h)·d² − (4/3)·d³`, so a `d` divisible by 3 clears
// the thirds; d=3 on the 100×60×20 plate leaves 120000 − 1404 = 118596 exactly.
// Evaluating the same identity in floats cannot pass this: a triple product
// cancels, so its only honest budget is an envelope of the absolute terms it was
// built from, and a positive bound over an exact value publishes Approximate.
//
// The `Cone` row is the other half of the rule. A circular wall's flux passes
// through math.Sincos, which no rational carries, so a band holding one stays
// Approximate however exactly its Plane patches integrate — the win is confined
// to all-Plane cap loops and must not leak past them.
func TestCapBlendPlanePatchVolumeIsExact(t *testing.T) {
	for _, tc := range []struct {
		name string
		d    float64
		want float64
	}{
		{`d=3 clears the thirds`, 3, 118596},
		{`d=6 clears them too`, 6, 114528},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, box := capBlendBox(t)
			const L, W, H = 100.0, 60.0, filletBoxHeight
			require.Equal(t, tc.want, L*W*H-((L+W)*tc.d*tc.d-(4.0/3.0)*tc.d*tc.d*tc.d),
				`the premise: this setback's true volume is a float64`)

			chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(tc.d))
			require.NoError(t, err)
			vol, err := chamfered.Volume()
			require.NoError(t, err)
			require.Equal(t, tc.want, vol.Value.Mag())
			require.Equal(t, 0.0, vol.Bound.Mag(),
				`an exact rational tetrahedron sum commits only its final rounding, and here that rounding is zero`)
			require.Equal(t, decad.Exact, vol.Exactness)
		})
	}

	t.Run(`a Cone patch keeps the band Approximate`, func(t *testing.T) {
		disk := circleProfile(t, 30, 20)
		chamfered, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(3))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness,
			`a flux term through math.Sincos is never Exact`)
		require.Greater(t, vol.Bound.Mag(), 0.0)
	})
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
// IS — an enclosure of the TRUE centroid — rather than for a particular value.
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
}

// TestCapBlendCentroidNeverExceedsGeometryNet is docs/modify-reach-design.md
// §8.4 PR B's required test 10: the published bound is a math.Min against
// the geometric safety net (the body's own Bounds box reach from the
// centroid, corners included, plus the box's own Bound), so it can never
// exceed that net — proving the math.Min really is a CEILING, not merely a
// tie-breaker. Before PR B the estimate WAS that net exactly (so the two
// always coincided); this pins the inequality directly now that the
// closed-form formula bound usually wins it.
func TestCapBlendCentroidNeverExceedsGeometryNet(t *testing.T) {
	const R, H, d = 10.0, 8.0, 0.5
	body := circleProfile(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
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

	require.LessOrEqual(t, centroid.Bound.Mag(), net,
		`the published centroid bound must never exceed the geometry-net ceiling`)
}

// TestCapBlendBandReachRefusalKeepsSX6Degenerate pins §4's stage order for the
// two rows that can both fire on one call. SX6 (stage 5) says the cap contour
// the caller named is EMPTY, so no such body exists at any sweep height; SX7
// (stage 6) says the bands meet, so a body exists that this evaluator cannot
// merge. Those are opposite existence claims, and §4's one-sentinel rule is
// what keeps the same nonexistent body from reporting both: SX6 precedes SX7.
//
// A radius-4 disk eroded by 4 leaves nothing whatever the sweep is, so both
// halves of this test name the same nonexistent body and both must answer
// ErrDegenerate. The 20mm sweep is the case TestCapBlendCarrierCollapseRefused
// already covers; the 4mm sweep is the one where SX7's own condition (reach >=
// height) is ALSO satisfied, and deciding it first would let the sweep height
// alone pick the sentinel.
func TestCapBlendBandReachRefusalKeepsSX6Degenerate(t *testing.T) {
	for _, height := range []float64{4.0, 20.0} {
		t.Run(fmt.Sprintf("H=%g", height), func(t *testing.T) {
			disk := circleProfile(t, 4, height)
			_, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(4))
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.NotErrorIs(t, err, decad.ErrUnsupported,
				`SX6 precedes SX7, so an empty cap contour keeps its own sentinel however short the sweep`)
		})
	}

	// The other side of the same order: a contour that exists and a band that
	// does not fit is still SX7's own ErrUnsupported. Eroding a radius-30 disk
	// by 4 leaves a radius-26 circle, so nothing about SX6 applies.
	disk := circleProfile(t, 30, 4)
	_, err := disk.Chamfer(capLoopEdges(disk), units.Millimeters(4))
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrDegenerate)
}

// TestCapBlendCapLevelEdgesReportFiniteLengths is defect-2's public shape: a
// cap-level edge is a real edge of a real body and Edge.Length() is a public
// reading of it, so it must report the length the edge HAS with a proven bound
// on it. An infinite value is a wrong answer that selector.go's LongerThan
// reads straight off the raw field, and an infinite bound beside a finite value
// bounds nothing at all.
//
// The four slant edges of a right-angled corner are the closed form: the miter
// foot sits a full setback from each of the two walls, hence d*sqrt(2) from the
// corner in the plane, and the apex sits one setback below it, so the slant is
// d*sqrt(3).
func TestCapBlendCapLevelEdgesReportFiniteLengths(t *testing.T) {
	const d = 5.0
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)

	slants := 0
	for _, e := range chamfered.Edges() {
		length, err := e.Length()
		require.NoError(t, err)
		require.False(t, math.IsInf(length.Value.Mag(), 0), `an edge length is a measurement, never an infinity`)
		require.False(t, math.IsInf(length.Bound.Mag(), 0), `an infinite bound bounds nothing`)
		if math.Abs(length.Value.Mag()-d*math.Sqrt(3)) < 1e-9 {
			slants++
		}
	}
	require.Equal(t, 4, slants, `one slant edge per corner of the rectangular cap loop`)

	// The propagation the raw field drives: LongerThan compares e.length
	// directly, so an infinite length matched every threshold a caller could
	// name.
	_, err = decad.Edges(decad.LongerThan(units.Millimeters(1e6))).SelectEdges(chamfered)
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// rightTriangleBody extrudes the 12-9-15 right triangle, walked
// counter-clockwise from the right angle at the origin. Its inward offset has a
// closed form in EXACT rationals at every corner, which is what lets a test
// state the denoted cap contour rather than approximate it: the two legs are
// axis aligned, and the hypotenuse's direction (-12, 9) has exact length 15, so
// the three unit normals are (0, 1), (-3/5, -4/5) and (-1, 0) as REALS. The
// feet are then (t, t), (12 - 3t, t) and (t, 9 - 2t) for a setback t, exactly.
//
// Nothing about that is exact in float64: normalize2 divides by the hypot, so
// the offset carriers the build intersects hold rounded directions, and the
// miter it solves for lands some ulps off the rational point above.
func rightTriangleBody(t *testing.T, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{s.CreatePoint(0, 0), s.CreatePoint(12, 0), s.CreatePoint(0, 9)}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, p := range pts {
		s.Fix(p)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	// The closed form below is a fact about THESE coordinates, so the test
	// states them rather than trusting the solver to have left them alone.
	corners := map[[2]float64]bool{}
	for _, v := range body.Vertices() {
		p := v.Position().Value
		if p.Z != 0 {
			continue
		}
		corners[[2]float64{p.X, p.Y}] = true
	}
	require.Equal(t, map[[2]float64]bool{{0, 0}: true, {12, 0}: true, {0, 9}: true}, corners)
	return body
}

// TestCapBlendCapContourVertexBoundEncloses is defect-3's property, and it is
// stated as ENCLOSURE rather than as a particular bound: a cap-level vertex
// sits where a float miter solve put it, and whatever bound it publishes must
// cover the distance from there to the point the offset DENOTES. A zero bound
// publishes Exact, which asserts that distance is nothing at all.
//
// The truth is taken over exact rationals from rightTriangleBody's closed form,
// never from a second float evaluation, so the assertion is about the denoted
// contour and not about two roundings agreeing.
func TestCapBlendCapContourVertexBoundEncloses(t *testing.T) {
	const height = 20.0
	for _, d := range []float64{0.1, 0.3, 0.7, 1.3, 2.5, 2.9} {
		t.Run(fmt.Sprintf("d=%v", d), func(t *testing.T) {
			body := rightTriangleBody(t, height)
			chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
			require.NoError(t, err)

			rd := new(big.Rat).SetFloat64(d)
			feet := [3][2]*big.Rat{
				{new(big.Rat).Set(rd), new(big.Rat).Set(rd)},
				{new(big.Rat).Sub(big.NewRat(12, 1), new(big.Rat).Mul(big.NewRat(3, 1), rd)), new(big.Rat).Set(rd)},
				{new(big.Rat).Set(rd), new(big.Rat).Sub(big.NewRat(9, 1), new(big.Rat).Mul(big.NewRat(2, 1), rd))},
			}

			checked := 0
			for _, v := range chamfered.Vertices() {
				p := v.Position()
				if p.Value.Z != height {
					continue // a side-level corner, at its own recorded (u, v)
				}
				checked++
				gap := math.Inf(1)
				for _, foot := range feet {
					gap = math.Min(gap, ratDistance2D(p.Value.X, p.Value.Y, foot[0], foot[1]))
				}
				require.LessOrEqual(t, gap, p.Bound.Mag(),
					`vertex %v must publish a bound covering its distance to the denoted foot`, p.Value)
			}
			require.Equal(t, 3, checked, `one cap-level foot per corner`)
		})
	}
}

// ratDistance2D is the distance from a float64 point to an exact rational one,
// rounded upward through a 200-bit square root so the test never asks the
// bound to cover its own arithmetic.
func ratDistance2D(x, y float64, wantU, wantV *big.Rat) float64 {
	du := new(big.Rat).Sub(new(big.Rat).SetFloat64(x), wantU)
	dv := new(big.Rat).Sub(new(big.Rat).SetFloat64(y), wantV)
	sum := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	root := new(big.Float).SetPrec(200).Sqrt(new(big.Float).SetPrec(200).SetRat(sum))
	out, _ := root.Float64()
	return out
}

// TestCapBlendContourHeldExtentCarriesItsDisplacement is the same term where
// the payload's own directional reading picks it up. Along a world axis of an
// axis-aligned plate the extreme is always held by a RECORDED coordinate — the
// cap contour offsets inward, so it never holds an in-plane extreme — and the
// box is exact. Tilt the body and one axis reads both the plane and the sweep
// at once; there the 45-degree band can carry the contour past the trimmed
// straight level, and the box may not then claim its faces are exact.
func TestCapBlendContourHeldExtentCarriesItsDisplacement(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	var pts []*sketch.Point
	for i := range 6 {
		a := 2 * math.Pi * float64(i) / 6
		pts = append(pts, s.CreatePoint(100*math.Cos(a), 100*math.Sin(a)))
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
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(5))
	require.NoError(t, err)

	upright, err := chamfered.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, upright.Exactness, `every world-axis extreme is a recorded coordinate here`)
	require.Zero(t, upright.Bound.Mag())

	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(0.4))
	require.NoError(t, err)
	tilted, err := chamfered.Placed(rot)
	require.NoError(t, err)
	tiltedBounds, err := tilted.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, tiltedBounds.Exactness,
		`an extreme held by the computed cap contour is known only to that contour's displacement`)
	require.Positive(t, tiltedBounds.Bound.Mag())

	// Every vertex the body holds still lies inside the reported box widened by
	// its own bound — the claim the bound actually makes.
	slack := tiltedBounds.Bound.Mag()
	for _, v := range tilted.Vertices() {
		p := v.Position().Value
		require.GreaterOrEqual(t, p.X, tiltedBounds.Min.X-slack)
		require.GreaterOrEqual(t, p.Y, tiltedBounds.Min.Y-slack)
		require.GreaterOrEqual(t, p.Z, tiltedBounds.Min.Z-slack)
		require.LessOrEqual(t, p.X, tiltedBounds.Max.X+slack)
		require.LessOrEqual(t, p.Y, tiltedBounds.Max.Y+slack)
		require.LessOrEqual(t, p.Z, tiltedBounds.Max.Z+slack)
	}
}

// rightTriangleBodyLegs is rightTriangleBody with caller-chosen leg lengths:
// a right triangle with legs a (along x) and b (along y), right angle at the
// origin, extruded by h.
func rightTriangleBodyLegs(t *testing.T, a, b, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{s.CreatePoint(0, 0), s.CreatePoint(a, 0), s.CreatePoint(0, b)}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, p := range pts {
		s.Fix(p)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// exactWedgeChamferReadings is the high-precision (400-bit) reference for a
// right triangle's cap-loop chamfer: legs a, b (right angle at the origin),
// swept h, chamfered by setback d on one cap. Offsetting all three sides of a
// triangle inward by t produces a similar triangle, homothetic to the
// original about the incenter with ratio (r-t)/r — r the inradius, a
// classical fact about triangles independent of float64's own
// offset-intersection arithmetic — so the cap contour's area at setback t is
// exactly A0*((r-t)/r)^2, and the band between the original and offset
// triangles is a pyramid frustum of height d: d/3*(A0+A1+sqrt(A0*A1)).
func exactWedgeChamferReadings(t *testing.T, a, b, h, d float64) (capArea, volume *big.Float) {
	t.Helper()
	const prec = 400
	bf := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	mul := func(x, y *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(x, y) }
	add := func(x, y *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Add(x, y) }
	sub := func(x, y *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sub(x, y) }
	quo := func(x, y *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Quo(x, y) }
	sqrt := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sqrt(x) }

	A, B, H, D := bf(a), bf(b), bf(h), bf(d)
	c := sqrt(add(mul(A, A), mul(B, B)))
	A0 := quo(mul(A, B), bf(2))
	s := quo(add(add(A, B), c), bf(2))
	r := quo(A0, s)
	k := quo(sub(r, D), r)
	A1 := mul(A0, mul(k, k))
	band := quo(mul(D, add(add(A0, A1), sqrt(mul(A0, A1)))), bf(3))
	vol := add(mul(A0, sub(H, D)), band)
	return A1, vol
}

// TestCapBlendWedgeAreaAndVolumeBoundsEncloseTrueError is the counterexample
// that shows the missing cap-contour displacement term is a real defect, not
// a theoretical one. An extreme-aspect-ratio right triangle wedge's offset
// feet round far enough from the point they denote that the area and volume
// bounds — built only from the arithmetic that measured the BUILT contour,
// never from how far that contour sits from the one the offset DENOTES —
// fall short of the true residual against a 400-bit reference.
func TestCapBlendWedgeAreaAndVolumeBoundsEncloseTrueError(t *testing.T) {
	const a, b, h, d = 9e4, 3e6, 50000.0, 13500.0
	body := rightTriangleBodyLegs(t, a, b, h)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	wantArea, wantVol := exactWedgeChamferReadings(t, a, b, h, d)

	capEnd := faceWithRole(t, chamfered, roleCapEnd)
	area, err := capEnd.Area()
	require.NoError(t, err)
	heldArea := new(big.Float).SetPrec(400).SetFloat64(area.Value.Mag())
	areaResidual, _ := new(big.Float).Abs(new(big.Float).SetPrec(400).Sub(heldArea, wantArea)).Float64()
	require.LessOrEqual(t, areaResidual, area.Bound.Mag(),
		`the published cap face area bound must enclose the true residual against the denoted (homothetic) contour`)

	vol, err := chamfered.Volume()
	require.NoError(t, err)
	heldVol := new(big.Float).SetPrec(400).SetFloat64(vol.Value.Mag())
	volResidual, _ := new(big.Float).Abs(new(big.Float).SetPrec(400).Sub(heldVol, wantVol)).Float64()
	require.LessOrEqual(t, volResidual, vol.Bound.Mag(),
		`the published volume bound must enclose the true residual against the denoted (homothetic) band`)
}

// TestCapBlendSideLevelCarriesSetbackRounding pins the cap-blend half of the
// computed-level rule: a chamfered end pulls its straight side level in by the
// setback, that float sum rounds, and the side walls built over the level
// publish the rounding rather than claiming the level the setback denotes. The
// cap-level feet keep their own, much smaller, offset-solve bound, so the two
// levels are proven apart rather than by one blanket stamp.
func TestCapBlendSideLevelCarriesSetbackRounding(t *testing.T) {
	const (
		height = 1e12
		d      = 1e-3
		// fl(1e12 − 1e-3) against the level it denotes.
		rounding = 2.34375e-05
	)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(d))
	require.NoError(t, err)

	side := 0
	for _, v := range chamfered.Vertices() {
		p := v.Position()
		switch p.Value.Z {
		case 0:
			require.Equal(t, decad.Exact, p.Exactness, `the untouched end stays exact`)
		case height:
			// A cap-level foot: bounded by its own offset solve, not by the
			// setback sum.
			require.Equal(t, decad.Approximate, p.Exactness)
		default:
			require.Equal(t, decad.Approximate, p.Exactness, `a vertex at the setback level is not exact`)
			bound, err := p.Bound.In(units.Millimeter)
			require.NoError(t, err)
			require.InDelta(t, rounding, bound, 1e-12)
			side++
		}
	}
	require.Equal(t, 4, side, `the square section has four side-level vertices`)
}

// TestCapBlendInheritsComputedCapLevelBound keeps a cap blend from laundering
// the stop displacement of its chamfered end cap back into an exact level.
func TestCapBlendInheritsComputedCapLevelBound(t *testing.T) {
	const (
		plateHeight = 1e12
		shortBy     = 1e-3
		heldStop    = 999999999999.9990234375
		rounding    = 2.34375e-05
	)
	s, plateProfile, pinProfile := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(plateHeight), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := doc.Extrude(s, pinProfile, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(-shortBy),
	})
	require.NoError(t, err)

	chamfered, err := pin.Chamfer(
		decad.Edges(decad.CreatedBy(decad.CapEnd(pin))),
		units.Millimeters(shortBy),
	)
	require.NoError(t, err)

	top := 0
	for _, vertex := range chamfered.Vertices() {
		position := vertex.Position()
		if position.Value.Z != heldStop {
			continue
		}
		require.Equal(t, decad.Approximate, position.Exactness)
		bound, err := position.Bound.In(units.Millimeter)
		require.NoError(t, err)
		require.GreaterOrEqual(t, bound, rounding)
		top++
	}
	require.Equal(t, 4, top, `the chamfered end cap keeps its four bounded vertices`)

	bounds, err := chamfered.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, bounds.Exactness)
	require.GreaterOrEqual(t, bounds.Bound.Mag(), rounding)
}

// TestCapBlendComputedStopMassBounds checks the mass-property half of the
// computed-cap-level contract. A ToFace stop carries its own axial
// displacement into both the chamfer band and the straight slab, so volume
// and centroid bounds must enclose the solid at the stop level the operation
// denotes rather than only the rounded level the build holds.
func TestCapBlendComputedStopMassBounds(t *testing.T) {
	const (
		plateHeight = 1e12
		shortBy     = 1e-3
		side        = 1.0
	)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	plateRect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(plateRect.A)
	s.CreateRectangle(120, 0, 121, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var plateProfile, pinProfile *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Area > 1000 {
			plateProfile = profile
			continue
		}
		pinProfile = profile
	}
	require.NotNil(t, plateProfile)
	require.NotNil(t, pinProfile)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(plateHeight), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := doc.Extrude(s, pinProfile, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(-shortBy),
	})
	require.NoError(t, err)

	chamfered, err := pin.Chamfer(capLoopEdges(pin), units.Millimeters(shortBy))
	require.NoError(t, err)

	wantVolume, wantCentroidZ := exactComputedStopSquareChamfer(t, side, plateHeight, -shortBy, shortBy)
	volume, err := chamfered.Volume()
	require.NoError(t, err)
	heldVolume := new(big.Float).SetPrec(volumeRefPrec).SetFloat64(volume.Value.Mag())
	volumeResidual, _ := new(big.Float).Abs(new(big.Float).SetPrec(volumeRefPrec).Sub(heldVolume, wantVolume)).Float64()
	require.Greater(t, volumeResidual, 0.0, `the rounded stop must move the held volume from the denoted solid`)
	require.GreaterOrEqual(t, volume.Bound.Mag(), volumeResidual,
		`the published volume bound %v must enclose the %v mm^3 residual at the denoted stop level`, volume.Bound.Mag(), volumeResidual)

	centroid, err := chamfered.Centroid()
	require.NoError(t, err)
	heldCentroidZ := new(big.Float).SetPrec(volumeRefPrec).SetFloat64(centroid.Value.Z)
	centroidResidual, _ := new(big.Float).Abs(new(big.Float).SetPrec(volumeRefPrec).Sub(heldCentroidZ, wantCentroidZ)).Float64()
	require.Greater(t, centroidResidual, 0.0, `the rounded stop must move the held centroid from the denoted solid`)
	require.GreaterOrEqual(t, centroid.Bound.Mag(), centroidResidual,
		`the published centroid bound %v must enclose the %v mm residual at the denoted stop level`, centroid.Bound.Mag(), centroidResidual)
}

// exactComputedStopSquareChamfer is a 400-bit reference for a square prism
// with its end cap chamfered after a ToFace stop. The stop offset remains in
// reference arithmetic so this does not collapse the denoted level back onto
// the float64 level the evaluator holds.
func exactComputedStopSquareChamfer(t *testing.T, side, plateHeight, stopOffset, d float64) (volume, centroidZ *big.Float) {
	t.Helper()
	bf := func(x float64) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).SetFloat64(x) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Mul(a, b) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Add(a, b) }
	sub := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Sub(a, b) }
	quo := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(volumeRefPrec).Quo(a, b) }

	L, H, D := bf(side), add(bf(plateHeight), bf(stopOffset)), bf(d)
	area := mul(L, L)
	straightHeight := sub(H, D)
	slabVolume := mul(area, straightHeight)
	slabMoment := quo(mul(slabVolume, straightHeight), bf(2))

	bandFactor := add(sub(area, mul(bf(2), mul(L, D))), quo(mul(bf(4), mul(D, D)), bf(3)))
	bandVolume := mul(D, bandFactor)
	bandMomentFactor := add(sub(quo(area, bf(2)), quo(mul(bf(4), mul(L, D)), bf(3))), mul(D, D))
	bandMoment := add(mul(straightHeight, bandVolume), mul(mul(D, D), bandMomentFactor))

	volume = add(slabVolume, bandVolume)
	centroidZ = quo(add(slabMoment, bandMoment), volume)
	return volume, centroidZ
}

// TestCapBlendWholeCircleInheritsComputedCapLevelBound covers the cap seam
// vertex that a whole-circle chamfer builds instead of corner vertices.
func TestCapBlendWholeCircleInheritsComputedCapLevelBound(t *testing.T) {
	const (
		plateHeight = 1e12
		shortBy     = 1e-3
		heldStop    = 999999999999.9990234375
		rounding    = 2.34375e-05
	)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	plate := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(plate.A)
	center := s.CreatePoint(130, 10)
	s.Fix(center)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var plateProfile, diskProfile *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Area > 1000 {
			plateProfile = profile
		} else {
			diskProfile = profile
		}
	}
	require.NotNil(t, plateProfile)
	require.NotNil(t, diskProfile)

	doc := decad.New()
	plateBody, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(plateHeight), Dir: decad.Along})
	require.NoError(t, err)
	disk, err := doc.Extrude(s, diskProfile, decad.ToFace{
		Body:   plateBody,
		Face:   capEndFace(plateBody),
		Offset: units.Millimeters(-shortBy),
	})
	require.NoError(t, err)

	chamfered, err := disk.Chamfer(
		decad.Edges(decad.CreatedBy(decad.CapEnd(disk))),
		units.Millimeters(shortBy),
	)
	require.NoError(t, err)

	top := 0
	for _, vertex := range chamfered.Vertices() {
		position := vertex.Position()
		if position.Value.Z != heldStop {
			continue
		}
		require.Equal(t, decad.Approximate, position.Exactness)
		bound, err := position.Bound.In(units.Millimeter)
		require.NoError(t, err)
		require.GreaterOrEqual(t, bound, rounding)
		top++
	}
	require.Equal(t, 1, top, `the whole-circle cap seam keeps its level bound`)
}

// concentricDiskWithHole extrudes an annulus — a disk of radius R holding a
// concentric hole of radius r, both centred at the origin — by h. Both loops
// are whole circles, so chamfering the end cap mints exactly two whole-turn
// Cone patches, one per loop.
func concentricDiskWithHole(t *testing.T, R, r, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, R)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 1 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the annulus region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendFarPlacementOrientationRefusal pins fu153: fixPatchOrientation
// used to drop Face.NormalAt's own error and leave a patch's outward
// orientation whatever it was constructed with. A holed disk's end-cap
// chamfer placed 2^60 mm along one plane axis rounds the build's own
// orientation sample onto the cone axis, so NormalAt refuses there and the
// build must refuse too (SX15, docs/modify-reach-design.md §4) rather than
// publish a hole patch whose outward side was never checked.
func TestCapBlendFarPlacementOrientationRefusal(t *testing.T) {
	const R, r, h, d = 4.0, 1.5, 6.0, 0.5
	body := concentricDiskWithHole(t, R, r, h)
	chamfered, err := body.Chamfer(capLoopEdgesOn(body, true), units.Millimeters(d))
	require.NoError(t, err)

	far, err := r3.Translation(r3.NewVec(math.Ldexp(1, 60), 0, 0))
	require.NoError(t, err)
	_, err = chamfered.Placed(far)
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.False(t, errors.Is(err, decad.ErrDegenerate),
		`ErrDegenerate and ErrUnsupported are opposite existence claims; only one may ride along`)

	// The near placement is unaffected: the build's orientation sample stays
	// far from the cone axis, so this is the hole patch's outward normal as
	// fixPatchOrientation decides it when NormalAt succeeds.
	near, err := r3.Translation(r3.NewVec(1000, 0, 0))
	require.NoError(t, err)
	nearBody, err := chamfered.Placed(near)
	require.NoError(t, err)

	// The hole's cap-level radius grows from r to r+d (a countersink widens a
	// hole), while the outer wall's cap-level radius shrinks from R to R-d —
	// the hole patch is the smaller of the two chamferCap(end,...) Cone
	// patches.
	var holePatch *decad.Face
	holeRadius := math.Inf(1)
	for _, f := range nearBody.Faces() {
		isBand := false
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(end,") {
				isBand = true
			}
		}
		if !isBand || f.Surface().Kind() != decad.KindCone {
			continue
		}
		p := f.Loops()[0].CoEdges()[0].Start().Position().Value
		radius := math.Hypot(p.X-1000, p.Y)
		if radius < holeRadius {
			holeRadius = radius
			holePatch = f
		}
	}
	require.NotNil(t, holePatch)
	require.Less(t, holeRadius, R-d, `the hole patch's cap radius stays under the outer wall's`)

	capRadius := r + d
	n, err := holePatch.NormalAt(r3.NewVec(1000, capRadius, h))
	require.NoError(t, err)
	require.InDelta(t, 0.0, n.Value.X, 1e-12)
	require.InDelta(t, -0.7071067811865476, n.Value.Y, 1e-12)
	require.InDelta(t, 0.7071067811865475, n.Value.Z, 1e-12)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Less(t, bound, 1e-12)
}
