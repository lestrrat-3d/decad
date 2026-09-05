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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
	doc, box := capBlendBox(t)
	before := doc.Recipe()
	q := decad.Edges(decad.CreatedBy(decad.CapStart(box)), decad.CreatedBy(decad.CapEnd(box)))
	_, err := box.Chamfer(q, units.Millimeters(10)) // reaches SX7
	require.Error(t, err)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestCapBlendSX10RefusesFurtherModify(t *testing.T) {
	t.Parallel()
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
