package apitest_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// shellBoxBossBand is the root transition the 2 mm inward shell of the
// 60×40×20 box with its 20×20×10 boss cuts away beside the boss
// (TestShellStackedBossKeepsThicknessAtRoot).
var shellBoxBossBand = 800 + 64.0/3 - 80*math.Pi

// boxBossOpening names the face the boss shell removes.
type boxBossOpening int

const (
	openBossTop boxBossOpening = iota
	openBoxBottom
	openNone
)

func (o boxBossOpening) String() string {
	return [...]string{"boss top open", "box bottom open", "closed"}[o]
}

// shellBoxBoss is the 2 mm inward shell of the box-and-boss Union, with the
// boss top, the box bottom or no face removed, and its volume.
func shellBoxBoss(t *testing.T, opening boxBossOpening) (*decad.Document, *decad.Body, float64) {
	t.Helper()
	doc := decad.New()
	stack, err := decad.Union(t.Context(), boxBody(t, doc, 0, 0, 60, 40, 20),
		boxBodyAtZ(t, doc, 20, 10, 40, 30, 20, 10))
	require.NoError(t, err)
	var shell *decad.Body
	// The cavity is the 56×36 plate inset, the 16×16 boss inset and the band.
	var cavity float64
	switch opening {
	case openBossTop:
		shell, err = decad.Shell(t.Context(), stack,
			decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1), units.Millimeters(2))
		cavity = 56*36*16 + 16*16*10
	case openBoxBottom:
		shell, err = decad.Shell(t.Context(), stack,
			decad.Faces(decad.FaceCreatedBy(decad.CapStart(stack))).Exactly(1), units.Millimeters(2))
		cavity = 56*36*18 + 16*16*8
	default:
		shell, err = decad.Shell(t.Context(), stack, nil, units.Millimeters(2), decad.WithNoOpenings())
		cavity = 56*36*16 + 16*16*8
	}
	require.NoError(t, err)
	want := 52000 - cavity - shellBoxBossBand
	requireVolumeNear(t, shell, want)
	return doc, shell, want
}

// boxVerticals selects the four 20 mm vertical edges of the 60×40 box.
func boxVerticals() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex(),
		decad.LongerThan(units.Millimeters(15))).Exactly(4)
}

// requireBlendVerified checks the blended body's verified mesh, and that a
// default Verify of doc reads Sound, with every reading of each body's
// report bounded below its tolerance limit.
func requireBlendVerified(t *testing.T, doc *decad.Document, b *decad.Body) *decad.Mesh {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status, "diagnostics: %+v", report.Diagnostics)
	for _, readings := range report.Bodies {
		require.NotNil(t, readings.Region)
		for _, reading := range []struct {
			name  string
			bound units.Value
			limit *units.Value
		}{
			{"Area", readings.Area.Bound, readings.Area.Tolerance.Limit},
			{"Bounds", readings.Bounds.Bound, readings.Bounds.Tolerance.Limit},
			{"Volume", readings.Region.Volume.Bound, readings.Region.Volume.Tolerance.Limit},
			{"Centroid", readings.Region.Centroid.Bound, readings.Region.Centroid.Tolerance.Limit},
		} {
			require.NotNil(t, reading.limit, reading.name)
			require.Less(t, reading.bound.Base(), reading.limit.Base(), reading.name)
		}
	}
	return mesh
}

// A stacked boss shell's root band meshes within the requested tolerance.
// Each straight root edge's band runs from one root corner's miter to the
// next in a single cell, so its ring count is set by that cell's twist
// (agent-docs/loop-fillet-design.md §7.1), and the mesh Bound, every term of
// which the band charges, stays at or below tol.
func TestShellBossTessellateWithinTolerance(t *testing.T) {
	t.Parallel()
	_, shell, _ := shellBoxBoss(t, openBossTop)
	for _, tol := range []float64{0.1, 0.01} {
		mesh, err := shell.Tessellate(t.Context(), units.Millimeters(tol))
		require.NoError(t, err)
		require.LessOrEqual(t, mesh.Bound().Base(), tol, "tol %g", tol)
	}
}

// A Fillet or Chamfer of the outer box verticals of a stacked boss shell
// runs on the shell's faceted view. A fillet of radius r at a corner whose
// walls are t thick leaves √2·t − (√2−1)·r of wall on the corner's
// diagonal: 1.586 mm for r = 3 and t = 2. The cavity is untouched, so the
// shell loses the four corner pieces only, (1 − π/4)·r² for a fillet and
// d²/2 for a chamfer per unit length, over the 20 mm edges. The view's root
// band meshes within its chord, so every blended result's readings stay
// below their relative limits and Verify reads it Sound.
//
// A fillet of radius 3 on the bottom-open shell's verticals refuses: the
// 2 mm rim at the open end is an end face whose cavity loop lies within the
// blend's reach of the corner (retained-edge-blend SR5). Radius 1.5 builds.
func TestShellBossThenBlendBoxVerticals(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		opening boxBossOpening
		r       float64
	}{{openBossTop, 3}, {openBoxBottom, 1.5}, {openNone, 3}} {
		t.Run("fillet "+tc.opening.String(), func(t *testing.T) {
			t.Parallel()
			doc, shell, before := shellBoxBoss(t, tc.opening)
			filleted, err := decad.Fillet(t.Context(), shell, boxVerticals(), units.Millimeters(tc.r))
			require.NoError(t, err)
			requireVolumeNear(t, filleted, before-4*(1-math.Pi/4)*tc.r*tc.r*20)
			mesh := requireBlendVerified(t, doc, filleted)
			if tc.r != 3 {
				return
			}
			// At the (60, 0) corner the fillet axis is (57, 3). On the corner's
			// diagonal, 2.26 mm from the axis is wall, 3.54 mm is cut away,
			// and (57.5, 2.5) is the untouched cavity.
			require.True(t, meshEncloses(mesh, r3.NewVec(58.6, 1.4, 10)), "the wall behind the fillet is material")
			require.False(t, meshEncloses(mesh, r3.NewVec(59.5, 0.5, 10)), "the corner piece is cut away")
			require.False(t, meshEncloses(mesh, r3.NewVec(57.5, 2.5, 10)), "the cavity is untouched")
		})
	}
	t.Run("chamfer top", func(t *testing.T) {
		t.Parallel()
		doc, shell, before := shellBoxBoss(t, openBossTop)
		chamfered, err := decad.Chamfer(t.Context(), shell, boxVerticals(), units.Millimeters(3))
		require.NoError(t, err)
		requireVolumeNear(t, chamfered, before-4*4.5*20)
		requireBlendVerified(t, doc, chamfered)
	})
}

// A fillet of radius 7 > √2·t/(√2−1) = 6.83 would cut through the 2 mm wall
// into the cavity at each corner, so the strip check refuses it, naming a
// point on the cavity wall within 7 mm of a corner.
func TestShellBossThenFilletRefusesCavityBreakthrough(t *testing.T) {
	t.Parallel()
	_, shell, before := shellBoxBoss(t, openBossTop)
	_, err := decad.Fillet(t.Context(), shell, boxVerticals(), units.Millimeters(7))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	const at = "the blend meets another facet of the body at {"
	msg := err.Error()
	i := strings.Index(msg, at)
	require.GreaterOrEqual(t, i, 0, msg)
	var p r3.Vec
	_, scanErr := fmt.Sscanf(msg[i+len(at):], "%g %g %g}", &p.X, &p.Y, &p.Z)
	require.NoError(t, scanErr, msg)
	// The cavity spans [2, 58]×[2, 38]×[2, 18].
	require.True(t, p.X >= 2 && p.X <= 58 && p.Y >= 2 && p.Y <= 38 && p.Z >= 2 && p.Z <= 18, msg)
	require.LessOrEqual(t, min(p.X-2, 58-p.X), 7.0, msg)
	require.LessOrEqual(t, min(p.Y-2, 38-p.Y), 7.0, msg)
	requireVolumeNear(t, shell, before)
}

// A second Shell of the stacked boss shell, here opening its bottom faces,
// has no route, so it refuses and leaves the body as it was (brep-modify
// SB11).
func TestShellBossThenShellRefuses(t *testing.T) {
	t.Parallel()
	_, shell, before := shellBoxBoss(t, openBossTop)
	bottom := decad.Faces(decad.Facing(r3.NewVec(0, 0, -1)), decad.NormalTo(r3.NewVec(0, 0, 1)))
	_, err := decad.Shell(t.Context(), shell, bottom, units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "this evaluator does not modify a stacked shell's rounded cavity")
	requireVolumeNear(t, shell, before)
}

// The opening's outer rim edge along x at y = 10 fillets on the view: the
// blend is a radius-1 cylinder about the line y = 11, z = 29, so a point
// 0.71 mm from that axis inside the corner stays and one 1.20 mm away goes.
// The fillet removes the corner piece (1 − π/4)·20 = 4.29 mm³, and the
// result's volume bound sits far enough below it for the removed volume to
// tell the fillet from no fillet.
func TestShellBossThenFilletOpeningRim(t *testing.T) {
	t.Parallel()
	doc, shell, _ := shellBoxBoss(t, openBossTop)
	rim := decad.Edges(decad.Convex(), decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(20, 10, 30))).Exactly(1)
	filleted, err := decad.Fillet(t.Context(), shell, rim, units.Millimeters(1))
	require.NoError(t, err)
	found := false
	for _, f := range filleted.Faces() {
		c, ok := f.Surface().(decad.Cylinder)
		if !ok || !c.Radius.Equal(units.Millimeters(1), 1e-12) {
			continue
		}
		found = true
		require.InDelta(t, 1, math.Abs(c.Axis.X), 1e-12)
		require.InDelta(t, 11, c.Origin.Y, 1e-12)
		require.InDelta(t, 29, c.Origin.Z, 1e-12)
	}
	require.True(t, found, "the fillet wall is a radius-1 cylinder")
	corner := (1 - math.Pi/4) * 20
	vol, err := filleted.Volume()
	require.NoError(t, err)
	require.Less(t, vol.Bound.Base(), corner/2, "the volume bound resolves the corner piece")
	requireRemoved(t, shell, filleted, corner)
	mesh := requireBlendVerified(t, doc, filleted)
	require.True(t, meshEncloses(mesh, r3.NewVec(30, 10.5, 29.5)), "material inside the fillet")
	require.False(t, meshEncloses(mesh, r3.NewVec(30, 10.15, 29.85)), "the rim corner is cut away")
}

// The outer verticals of a shelled blind pocket fillet the same way: the
// 10×10×10 block with its 4×4×4 pocket, shelled 1 mm with the top removed,
// loses (1 − π/4) per unit length at r = 1 over the four 10 mm edges.
func TestShellPocketThenFilletOuterVerticals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pocket, err := decad.Cut(t.Context(), boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10),
		boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4))
	require.NoError(t, err)
	shell, err := decad.Shell(t.Context(), pocket,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pocket))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	before := 504 + 26*math.Pi/3
	requireVolumeNear(t, shell, before)
	verticals := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex(),
		decad.LongerThan(units.Millimeters(9))).Exactly(4)
	filleted, err := decad.Fillet(t.Context(), shell, verticals, units.Millimeters(1))
	require.NoError(t, err)
	requireVolumeNear(t, filleted, before-4*(1-math.Pi/4)*10)
	mesh := requireBlendVerified(t, doc, filleted)
	// The (10, 10) corner's fillet axis is (9, 9).
	require.True(t, meshEncloses(mesh, r3.NewVec(9.5, 9.5, 5)), "the wall behind the fillet is material")
	require.False(t, meshEncloses(mesh, r3.NewVec(9.9, 9.9, 5)), "the corner piece is cut away")
}

// requireRemoved checks that after is before less want, within the two
// volume readings' bounds.
func requireRemoved(t *testing.T, before, after *decad.Body, want float64) {
	t.Helper()
	vb, err := before.Volume()
	require.NoError(t, err)
	va, err := after.Volume()
	require.NoError(t, err)
	got := vb.Value.Base() - va.Value.Base()
	require.LessOrEqual(t, math.Abs(got-want), vb.Bound.Base()+va.Bound.Base(),
		"removed %v must lie within the bounds of %v", got, want)
}

// A two-distance chamfer on the faceted view: the bottom-open shell's
// plate-top edge along x at y = 0 and the boss-top edge along x at y = 10,
// with d = 1 on the +z face and 0.5 on the side. Each removes a 1×0.5
// triangle along its 60 or 20 mm length, 0.25·80 mm³. At x = 30 the plate
// edge's bevel runs from (y, z) = (1, 20) to (0, 19.5): (0.4, 19.9) lies
// above it and is cut away, and (0.1, 19.4) lies below it and stays, which
// it would not if the side took the 1 mm setback.
func TestShellBossThenAsymmetricChamfer(t *testing.T) {
	t.Parallel()
	_, shell, _ := shellBoxBoss(t, openBoxBottom)
	ex := r3.NewVec(1, 0, 0)
	edges := decad.Edges(decad.Convex(), decad.ParallelTo(ex), decad.EndpointAt(r3.NewVec(0, 0, 20))).
		Or(decad.Convex(), decad.ParallelTo(ex), decad.EndpointAt(r3.NewVec(20, 10, 30))).Exactly(2)
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)), decad.NormalTo(r3.NewVec(0, 0, 1)))
	chamfered, err := decad.Chamfer(t.Context(), shell, edges, units.Millimeters(1),
		decad.WithAsymmetricChamfer(top, units.Millimeters(0.5)))
	require.NoError(t, err)
	requireRemoved(t, shell, chamfered, 0.25*80)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	require.False(t, meshEncloses(mesh, r3.NewVec(30, 0.4, 19.9)), "the bevel cuts 1 mm into the top face")
	require.True(t, meshEncloses(mesh, r3.NewVec(30, 0.1, 19.4)), "the bevel cuts 0.5 mm down the side")
}

// The other stacked shells take the faceted view as well: the counterbore
// shell's 10 mm plate verticals at r = 1, the round boss shell's 5 mm plate
// verticals at r = 3.3 (√2 − (√2−1)·3.3 = 0.05 mm of its 1 mm wall stays on
// the diagonal), and the round boss shell's outer rim, radius 4 at z = 10, at
// r = 0.5. A rim fillet removes the corner piece (1 − π/4)·r² swept round the
// circle through its centroid, r·(10 − 3π)/(3·(4 − π)) inside the rim, and
// its wall is a torus of major radius 3.5 and minor radius 0.5 at z = 9.5.
func TestShellThenFilletOtherStackedShells(t *testing.T) {
	t.Parallel()
	ez := r3.NewVec(0, 0, 1)
	roundShell := func(t *testing.T) *decad.Body {
		t.Helper()
		stack := roundBossFromSketch(t, decad.New(), 10, 10)
		shell, err := decad.Shell(t.Context(), stack,
			decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1), units.Millimeters(1))
		require.NoError(t, err)
		return shell
	}
	t.Run("counterbore verticals", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		part, err := decad.Cut(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 0, 3, 0, 10))
		require.NoError(t, err)
		part, err = decad.Cut(t.Context(), part, circleBodyAtZ(t, doc, 0, 5, 7, 3))
		require.NoError(t, err)
		shell, err := decad.Shell(t.Context(), part, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(part))).Exactly(1),
			units.Millimeters(1))
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), shell,
			decad.Edges(decad.Convex(), decad.ParallelTo(ez)).Exactly(4), units.Millimeters(1))
		require.NoError(t, err)
		requireRemoved(t, shell, filleted, 4*(1-math.Pi/4)*10)
	})
	t.Run("round boss verticals", func(t *testing.T) {
		t.Parallel()
		shell := roundShell(t)
		filleted, err := decad.Fillet(t.Context(), shell,
			decad.Edges(decad.Convex(), decad.ParallelTo(ez)).Exactly(4), units.Millimeters(3.3))
		require.NoError(t, err)
		requireRemoved(t, shell, filleted, 4*(1-math.Pi/4)*3.3*3.3*5)
	})
	t.Run("round boss rim", func(t *testing.T) {
		t.Parallel()
		shell := roundShell(t)
		const r = 0.5
		filleted, err := decad.Fillet(t.Context(), shell,
			decad.Edges(decad.Circular(), decad.EndpointAt(r3.NewVec(14, 10, 10))).Exactly(1), units.Millimeters(r))
		require.NoError(t, err)
		centroid := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
		requireRemoved(t, shell, filleted, 2*math.Pi*(4-centroid)*(1-math.Pi/4)*r*r)
		found := false
		for _, f := range filleted.Faces() {
			tor, ok := f.Surface().(decad.Torus)
			if !ok || !tor.Minor.Equal(units.Millimeters(r), 1e-12) {
				continue
			}
			found = true
			require.True(t, tor.Major.Equal(units.Millimeters(3.5), 1e-12), "major %s", tor.Major)
			require.InDelta(t, 9.5, tor.Center.Z, 1e-12)
		}
		require.True(t, found, "the rim fillet's wall is a torus of minor radius 0.5")
	})
}
