package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/modify-reach-design.md §9.3's side opening: a revolve
// shell that removes a connected run of generated side faces builds §9.2's
// open-chain wall region and sweeps it. Fixtures are revolve_shell_test.go's:
// a meridian point (u, v) is z = u, ρ = v, and Pappus turns the wall region's
// ∫ρ dA into the volume.
//
// Legs shown to fail (each changed or deleted, the fixture watched go red,
// then restored): halving the opening end's normal segment sends every
// building volume red; dropping requireOpeningRim's right-angle test lets the
// cone's acute opening build; dropping its length test leaves the short tread
// to the offset's generic S11a drop. The one-axis-end ordering gate is not
// exhibited: on these meridians the S11a drop gate fires first.

// sideFace names the one generated side face of b whose surface pick
// accepts, by that face's own role.
func sideFace(t *testing.T, b *decad.Body, pick func(decad.Surface) bool) *decad.FaceQuery {
	t.Helper()
	var hit *decad.Face
	for _, f := range b.Faces() {
		if !pick(f.Surface()) {
			continue
		}
		require.Nil(t, hit, `exactly one face matches`)
		hit = f
	}
	require.NotNil(t, hit)
	return decad.Faces(decad.FaceCreatedBy(hit.Origins()[0]))
}

// cylinderOfRadius10 picks a cylindrical face of radius 10.
func cylinderOfRadius10() func(decad.Surface) bool {
	const r = 10.0
	return func(s decad.Surface) bool {
		c, ok := s.(decad.Cylinder)
		if !ok {
			return false
		}
		got, err := c.Radius.In(units.Millimeter)
		return err == nil && got == r
	}
}

// planeAtU picks a planar face whose frame origin lies at x = u.
func planeAtU(u float64) func(decad.Surface) bool {
	return func(s decad.Surface) bool {
		p, ok := s.(decad.Plane)
		return ok && math.Abs(p.Frame.N().X) > 0.5 && p.Frame.Origin().X == u
	}
}

func TestRevolveShellSideOpening(t *testing.T) {
	t.Parallel()
	t.Run("full ring without its outer skin", func(t *testing.T) {
		t.Parallel()
		// The ring z ∈ [0, 20], ρ ∈ [5, 10] loses its ρ = 10 skin. Inward
		// 1 mm, the wall is the meridian less the core z ∈ [1, 19],
		// ρ ∈ [6, 10]: ∫ρ dA = 750 − 576 = 174, a channel open outward.
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		shelled, err := ring.Shell(t.Context(), sideFace(t, ring, cylinderOfRadius10()), units.Millimeters(1))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{shelled}, doc.Bodies())
		requireManifold(t, shelled)
		require.Len(t, shelled.Lumps(), 1)
		require.Len(t, shelled.Shells(), 1, `the opening leaves no void`)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(174))
		// The receiver's ρ = 5 skin, the cavity's ρ = 6 wall, and the two
		// rims of the opening, each a band on the removed skin's radius.
		require.ElementsMatch(t, []float64{5, 6, 10, 10}, cylinderRadii(t, shelled))
		requireMeshWatertightAt(t, shelled, 0.05)
	})
	t.Run("full ring outward", func(t *testing.T) {
		t.Parallel()
		// Outward 1 mm: two end strips ρ ∈ [5, 10] (37.5 each), the ρ ∈ [4, 5]
		// strip under the core (90), and two quarter disks of radius 1 at the
		// corners (5π/4 − 1/3 each).
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		shelled, err := ring.Shell(t.Context(), sideFace(t, ring, cylinderOfRadius10()), units.Millimeters(1),
			decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(165-2.0/3+5*math.Pi/2))
	})
	t.Run("solid cylinder without one end", func(t *testing.T) {
		t.Parallel()
		// The solid cylinder z ∈ [0, 20], ρ ≤ 10 loses its z = 20 end: a cup
		// whose kept chain runs from the opening down to the axis, so its
		// offset ends on the axis at z = 2. π·(10²·20 − 8²·18).
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		shelled, err := cyl.Shell(t.Context(), sideFace(t, cyl, planeAtU(20)), units.Millimeters(2))
		require.NoError(t, err)
		requireManifold(t, shelled)
		require.Len(t, shelled.Shells(), 1)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(1000-576))
		require.ElementsMatch(t, []float64{8, 10}, cylinderRadii(t, shelled), `no wall grows along the axis`)
		requireMeshWatertightAt(t, shelled, 0.05)
	})
	t.Run("solid cylinder outward", func(t *testing.T) {
		t.Parallel()
		// Outward 2 mm: the skin strip ρ ∈ [10, 12] (440), the end slab
		// z ∈ [−2, 0] (100) and the quarter disk of radius 2 at the corner
		// (10π + 8/3).
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		shelled, err := cyl.Shell(t.Context(), sideFace(t, cyl, planeAtU(20)), units.Millimeters(2),
			decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(540+8.0/3+10*math.Pi))
	})
	t.Run("half ring with both caps and its outer skin", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		var outer *decad.Face
		for _, f := range ring.Faces() {
			if cylinderOfRadius10()(f.Surface()) {
				outer = f
			}
		}
		require.NotNil(t, outer)
		sel := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))).Or(decad.FaceCreatedBy(outer.Origins()[0]))
		shelled, err := ring.Shell(t.Context(), sel, units.Millimeters(1))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(174))
		// The two rim bands at the angular openings are the wall region.
		for _, ref := range []decad.FeatureRef{decad.CapStart(shelled), decad.CapEnd(shelled)} {
			rims, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(shelled)
			require.NoError(t, err)
			area, err := rims[0].Area()
			require.NoError(t, err)
			decadtest.Measures(t, "rim band area", area, units.SquareMillimeters(28))
		}
	})
	t.Run("placed", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		shelled, err := ring.Shell(t.Context(), sideFace(t, ring, cylinderOfRadius10()), units.Millimeters(1))
		require.NoError(t, err)
		rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
		require.NoError(t, err)
		placed, err := shelled.Placed(t.Context(), rotation)
		require.NoError(t, err)
		require.Len(t, placed.Faces(), len(shelled.Faces()))
		decadtest.MeasuresVolume(t, placed, fullTurnVolume(174))
	})
}

func TestRevolveShellSideOpeningRefusals(t *testing.T) {
	t.Parallel()
	t.Run("two separate runs", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), decad.Faces(decad.Planar()), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `one connected run`)
	})
	t.Run("every side face", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), decad.Faces(), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `not all of them`)
	})
	t.Run("an opening at an acute corner", func(t *testing.T) {
		t.Parallel()
		// The cone's base disk meets its slant at an acute corner, so the
		// slant's normal segment would leave the base plane.
		doc := decad.New()
		cone := revolveMeridian(t, doc, coneMeridian, decad.FullRevolution{})
		_, err := cone.Shell(t.Context(), sideFace(t, cone, planeAtU(0)), units.Millimeters(1))
		requireShellRefused(t, doc, cone, err, decad.ErrUnsupported, `right angle`)
	})
	t.Run("a removed walk shorter than the wall", func(t *testing.T) {
		t.Parallel()
		// A step whose 2 mm tread at ρ = 10 is the opening: the 3 mm rim at
		// the tread's outer end would run past the tread.
		doc := decad.New()
		step := revolveMeridian(t, doc, [][2]float64{{0, 5}, {20, 5}, {20, 10}, {18, 10}, {18, 11}, {0, 11}}, decad.FullRevolution{})
		_, err := step.Shell(t.Context(), sideFace(t, step, cylinderOfRadius10()), units.Millimeters(3))
		requireShellRefused(t, doc, step, err, decad.ErrUnsupported, `no longer than the shell thickness`)
	})
	t.Run("a run between two kept walks on the axis", func(t *testing.T) {
		t.Parallel()
		// The solid cylinder's skin lies between its two end disks, so the
		// two disks would each keep a wall of their own.
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		_, err := cyl.Shell(t.Context(), sideFace(t, cyl, cylinderOfRadius10()), units.Millimeters(1))
		requireShellRefused(t, doc, cyl, err, decad.ErrUnsupported, `two wall regions`)
	})
}
