package decad_test

import (
	"encoding/json"
	"math"
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
