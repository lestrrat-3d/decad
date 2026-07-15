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

// shellBoxHeight is the sweep height of the plate every shell test hollows.
const shellBoxHeight = 20.0

// shellBox extrudes the 100×60 plate by shellBoxHeight — a straight prism whose
// two caps are the faces a shell removes.
func shellBox(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// bothCaps selects a prism's two caps: planar and normal to the z sweep axis.
func bothCaps() *decad.FaceQuery {
	return decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1)))
}

// topCap selects a prism's end cap by its role.
func topCap(b *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.FaceCreatedBy(decad.FeatureRef{Step: b.Origin().Step, Role: roleCapEnd}))
}

func TestShellTubeInwardBox(t *testing.T) {
	const th = 5.0
	h := shellBoxHeight
	doc, box := shellBox(t)

	// Both caps removed, inward — a tube: a prism over the annular section
	// {Outer: 100×60, Hole: 90×50}.
	body, err := box.Shell(bothCaps(), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume = (A_P − A_Q)·h, Q = P ⊖ t, both exact.
	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := (aP - aQ) * h
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Bound.Equal(units.CubicMillimeters(0), 1e-12), `a shell introduces no bound`)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// A tube is a prismPayload: 4 outer walls + 4 cavity walls + 2 rim annuli.
	require.Len(t, body.Faces(), 10)

	// It tessellates (a prismPayload does), and Verify reads it Sound.
	mesh, err := body.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotNil(t, mesh)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())

	// The wall survey reads the uniform wall thickness exactly.
	report, err = doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.True(t, br.MinWallThickness.Value.Equal(units.Millimeters(th), 1e-9),
		`the tube wall is the shell thickness, got %s`, br.MinWallThickness.Value)

	// The recipe records the shell intent and round-trips.
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	require.Equal(t, decad.OpShell, recipe.Steps[1].Op)
	require.Equal(t, decad.ShellOpts{Sense: decad.Inward}, recipe.Steps[1].Opts)
	require.Len(t, recipe.Steps[1].Selectors, 1)
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded shell recipe round-trips`)
}

func TestShellTubeInwardCylinder(t *testing.T) {
	const R, th = 20.0, 4.0
	h := 12.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), R)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	disk, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	body, err := disk.Shell(bothCaps(), units.Millimeters(th))
	require.NoError(t, err)
	requireManifold(t, body)

	// A cylindrical tube: outer cylinder, inner cylinder, two rim annuli.
	wantVol := (math.Pi*R*R - math.Pi*(R-th)*(R-th)) * h
	vol, err := body.Volume()
	require.NoError(t, err)
	got, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, got, 1e-9)

	cyl := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cyl++
		}
	}
	require.Equal(t, 2, cyl, `a cylindrical tube has an outer and an inner cylinder`)
}

func TestShellCupInwardBox(t *testing.T) {
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)

	// One cap removed, inward — a cup (cupPayload): the outer prism over P on
	// [z0, z1] and the cavity prism over Q = P ⊖ t on [z0 + t, z1].
	body, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume = A_P·h − A_Q·(h − t).
	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := aP*h - aQ*(h-th)
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Bound.Equal(units.CubicMillimeters(0), 1e-12))
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// Faces: 4 outer walls + 4 cavity walls + capStart + shellCap + rim = 11.
	require.Len(t, body.Faces(), 11)
	roles := shellRoleSet(body)
	require.Contains(t, roles, "capStart")
	require.Contains(t, roles, "shellCap")
	require.Contains(t, roles, "rim(0)")
	require.Contains(t, roles, "side(0,0)")
	require.Contains(t, roles, "shellSide(0,0)")

	// A shelled body has no void — the opening joins inner and outer into one
	// shell (docs/modify-design.md §8).
	require.False(t, body.IsSolid() && anyVoid(body), `a cup has no void shell`)
	for _, sh := range body.Shells() {
		require.False(t, sh.IsVoid())
	}
}

func TestShellCupOutwardBox(t *testing.T) {
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)

	body, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume = A_Q·(h + t) − A_P·h, Q = P ⊕ t. The outward dilation ROUNDS the
	// four convex corners with quarter arcs of radius t, so A_Q is the 110×70
	// box less the (4 − π)t² the rounds cut away (§7).
	aP := 100.0 * 60.0
	aQ := (100+2*th)*(60+2*th) - (4-math.Pi)*th*th
	wantVol := aQ*(h+th) - aP*h
	vol, err := body.Volume()
	require.NoError(t, err)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)
	// Outer walls: 4 straight + 4 rounded-corner cylinders (8). Cavity walls: 4
	// (the rectangular P). Plus capStart, shellCap, rim.
	require.Len(t, body.Faces(), 15)
}

func TestShellCupPlacedComposes(t *testing.T) {
	const th = 5.0
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	want, err := cup.Volume()
	require.NoError(t, err)

	// Body.Placed re-evaluates the cupPayload under the composed motion; volume
	// is invariant under a rigid move.
	motion, err := r3.Translation(r3.NewVec(10, 20, 30))
	require.NoError(t, err)
	moved, err := cup.Placed(motion)
	require.NoError(t, err)
	require.True(t, moved.IsSolid())
	requireManifold(t, moved)
	got, err := moved.Volume()
	require.NoError(t, err)
	require.True(t, got.Value.Equal(want.Value, 1e-9), `a rigid move preserves volume`)
	bounds, err := moved.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 10.0, bounds.Min.X, 1e-9, `the placed cup's box shifts by the motion`)
}

func TestShellRefusals(t *testing.T) {
	t.Run("holed both caps is S12 unsupported", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 100, 60)
		s.Fix(rect.A)
		s.CreateCircle(s.CreatePoint(70, 30), 10)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		var prof *sketch.Profile
		for _, p := range s.Profiles() {
			if len(p.Holes) == 1 {
				prof = p
			}
		}
		require.NotNil(t, prof)
		doc := decad.New()
		box, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
		require.NoError(t, err)
		_, err = box.Shell(bothCaps(), units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrUnsupported, `1 + k lumps has no prismPayload`)
	})

	t.Run("t at or past the inradius is S10 degenerate", func(t *testing.T) {
		_, box := shellBox(t)
		// Inradius of a 100×60 rectangle is 30; a 35 mm inward wall eats it.
		_, err := box.Shell(bothCaps(), units.Millimeters(35))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("t past the sweep height on a kept cap is S10 degenerate", func(t *testing.T) {
		_, box := shellBox(t)
		// 25 mm < inradius 30 (the section survives) but ≥ the 20 mm sweep, so
		// the kept cap's floor eats the cavity.
		_, err := box.Shell(topCap(box), units.Millimeters(25))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("a topology-changing offset is S11 unsupported", func(t *testing.T) {
		_, box := shellBox(t)
		rounded, err := box.Fillet(verticalEdges(), units.Millimeters(2))
		require.NoError(t, err)
		// A 5 mm inward wall erodes the 2 mm corner arcs past zero — a dropped
		// feature, which needs a trimmed-offset kernel.
		_, err = rounded.Shell(bothCaps(), units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("removing a side wall is S2 unsupported", func(t *testing.T) {
		_, box := shellBox(t)
		wall := decad.Faces(decad.FaceCreatedBy(decad.FeatureRef{Step: box.Origin().Step, Role: "side(0,0)"}))
		_, err := box.Shell(wall, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("a query matching nothing is loud", func(t *testing.T) {
		_, box := shellBox(t)
		_, err := box.Shell(decad.Faces(decad.Cylindrical()), units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})

	t.Run("a zero thickness is S14 degenerate", func(t *testing.T) {
		_, box := shellBox(t)
		_, err := box.Shell(bothCaps(), units.Millimeters(0))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

func TestShellCupDownstreamStaging(t *testing.T) {
	const th = 5.0

	t.Run("MinWallThickness reads Suspect", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Nil(t, br.MinWallThickness, `the cup's stacked section has no single-height reading`)
		require.Equal(t, decad.Suspect, br.Status)
		require.False(t, report.Trustworthy())
	})

	t.Run("a lone cup verifies Sound", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status, `a lone cup has no pairs to clear`)
		require.True(t, report.Trustworthy())
	})

	t.Run("a box-disjoint cup pair is Sound, but WithClearances invokes the kernel and reads Suspect", func(t *testing.T) {
		doc, box1 := shellBox(t)
		cup1, err := box1.Shell(topCap(box1), units.Millimeters(th))
		require.NoError(t, err)
		// Move the first cup far clear, then build a second cup at the origin —
		// two live, box-disjoint cups in one document.
		far, err := r3.Translation(r3.NewVec(500, 0, 0))
		require.NoError(t, err)
		_, err = cup1.Placed(far)
		require.NoError(t, err)
		s2, p2 := plateSketch(t)
		box2, err := doc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
		require.NoError(t, err)
		_, err = box2.Shell(topCap(box2), units.Millimeters(th))
		require.NoError(t, err)
		require.Len(t, doc.Bodies(), 2)

		// The cheap box test proves the pair disjoint; the kernel is never
		// invoked, so both cups verify Sound.
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status, `box-disjoint cups need no kernel`)

		// WithClearances asks for the gap, which invokes the kernel; it has no
		// cupPayload case, so the pair reads Suspect — never a fabricated pass.
		report, err = doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.Equal(t, decad.Suspect, report.Status, `an invoked cup pair is staged`)
		require.False(t, report.Trustworthy())
	})
}

// shellRoleSet gathers every provenance role on a body's faces.
func shellRoleSet(b *decad.Body) map[string]struct{} {
	roles := map[string]struct{}{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			roles[o.Role] = struct{}{}
		}
	}
	return roles
}

// anyVoid reports whether any shell of the body bounds a void.
func anyVoid(b *decad.Body) bool {
	for _, sh := range b.Shells() {
		if sh.IsVoid() {
			return true
		}
	}
	return false
}
