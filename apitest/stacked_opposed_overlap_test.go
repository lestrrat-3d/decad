package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// opposedPockets cuts a 20 x 20 x 10 plate with a top pocket over z 6..10
// and a bottom pocket over z 0..6, in either order. Each rectangle is
// (x0, y0, x1, y1).
func opposedPockets(t *testing.T, doc *decad.Document, top, bottom [4]float64, topFirst bool) (*decad.Body, error) {
	t.Helper()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	topTool := boxBodyAtZ(t, doc, top[0], top[1], top[2], top[3], 6, 4)
	bottomTool := boxBodyAtZ(t, doc, bottom[0], bottom[1], bottom[2], bottom[3], 0, 6)
	first, second := topTool, bottomTool
	if !topFirst {
		first, second = bottomTool, topTool
	}
	part, err := decad.Cut(t.Context(), plate, first)
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part))
	return decad.Cut(t.Context(), part, second)
}

// TestStackedOpposedOverlapPockets cuts pockets from opposite faces that
// meet at one level and overlap there in plan. Each volume is the plate's
// 4000 mm^3 less the two pockets' boxes, which share no volume: the pockets
// meet only in the plane z = 6.
func TestStackedOpposedOverlapPockets(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		top    [4]float64
		bottom [4]float64
		faces  int
		volume float64
	}{
		// Same y extent: the walls on y = -2 and y = 2 are each one Z-shaped
		// face, the floor is x -6..-4 and the ceiling x 0..1.
		{name: "overlap", top: [4]float64{-6, -2, -2, 2}, bottom: [4]float64{-4, -2, 1, 2},
			faces: 14, volume: 4000 - 4*4*4 - 5*4*6},
		// Corners crossing: L-shaped floor and ceiling meeting the through
		// square at two proper crossings.
		{name: "partial-overlap", top: [4]float64{-6, -6, -2, -2}, bottom: [4]float64{-4, -4, 0, 1},
			faces: 16, volume: 4000 - 4*4*4 - 4*5*6},
	} {
		for _, topFirst := range []bool{true, false} {
			order := map[bool]string{true: "top-first", false: "bottom-first"}[topFirst]
			t.Run(tc.name+"/"+order, func(t *testing.T) {
				t.Parallel()
				doc := decad.New()
				part, err := opposedPockets(t, doc, tc.top, tc.bottom, topFirst)
				require.NoError(t, err)
				require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(part))
				requireEveryEdgeOnTwoFaces(t, part)
				require.Len(t, part.Faces(), tc.faces)
				volume, err := part.Volume()
				require.NoError(t, err)
				require.Equal(t, decad.Exact, volume.Exactness)
				require.Equal(t, tc.volume, volumeMM(t, volume))
				mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1),
					decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, mesh.VolumeVerified())
				requireWatertight(t, mesh)
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				require.Equal(t, decad.Sound, report.Status)
			})
		}
	}
}

// TestStackedOpposedTouchingPocketsRefuse keeps opposed pockets that only
// touch off every analytic route: along an edge the floor and ceiling would
// put four faces on one edge, and at a corner or a tangency the body would
// meet itself at one point. Under ExactOnly the mesh path refuses the contact.
func TestStackedOpposedTouchingPocketsRefuse(t *testing.T) {
	t.Parallel()
	box := func(x0, y0, x1, y1, z0, h float64) func(*testing.T, *decad.Document) *decad.Body {
		return func(t *testing.T, doc *decad.Document) *decad.Body { return boxBodyAtZ(t, doc, x0, y0, x1, y1, z0, h) }
	}
	round := func(x, r, z0, h float64) func(*testing.T, *decad.Document) *decad.Body {
		return func(t *testing.T, doc *decad.Document) *decad.Body { return circleBodyAtZ(t, doc, x, r, z0, h) }
	}
	for _, tc := range []struct {
		name        string
		top, bottom func(*testing.T, *decad.Document) *decad.Body
	}{
		{name: "edge", top: box(-6, -2, -2, 2, 6, 4), bottom: box(-2, -2, 2, 2, 0, 6)},
		{name: "corner", top: box(-6, -2, -2, 2, 6, 4), bottom: box(-2, 2, 2, 6, 0, 6)},
		// Circles of radius 1 about x = -3 and x = -1 touch at (-2, 0); the
		// circle about x = -1 touches the rectangle's edge x = -2 there too.
		{name: "tangent-circles", top: round(-3, 1, 6, 4), bottom: round(-1, 1, 0, 6)},
		{name: "tangent-rectangle-circle", top: box(-6, -2, -2, 2, 6, 4), bottom: round(-1, 1, 0, 6)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			plate := boxBody(t, doc, -10, -10, 10, 10, 10)
			part, err := decad.Cut(t.Context(), plate, tc.top(t, doc))
			require.NoError(t, err)
			require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part))
			_, err = decad.Cut(t.Context(), part, tc.bottom(t, doc))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			var be *decad.BooleanError
			require.ErrorAs(t, err, &be)
			require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
		})
	}
}

// TestStackedOpposedOverlapGates keeps overlapping opposed pockets this
// route does not state exactly on the mesh path, which refuses their
// coplanar contact under ExactOnly: a round pocket that passes through the
// rectangle's corners (-2, ±2) tangent to its edges y = ±2 there, and a
// pocket moved in its plane, whose re-expression carries a section
// displacement the planar walls cannot state.
func TestStackedOpposedOverlapGates(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		bottom func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{name: "round", bottom: func(t *testing.T, doc *decad.Document) *decad.Body {
			return circleBodyAtZ(t, doc, -2, 2, 0, 6)
		}},
		{name: "moved", bottom: func(t *testing.T, doc *decad.Document) *decad.Body {
			tool := boxBodyAtZ(t, doc, -4.1, -2, 0.9, 2, 0, 6)
			move, err := r3.Translation(r3.NewVec(0.1, 0, 0))
			require.NoError(t, err)
			placed, err := decad.Transform(t.Context(), tool, move)
			require.NoError(t, err)
			return placed
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			plate := boxBody(t, doc, -10, -10, 10, 10, 10)
			part, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, -6, -2, -2, 2, 6, 4))
			require.NoError(t, err)
			_, err = decad.Cut(t.Context(), part, tc.bottom(t, doc))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			var be *decad.BooleanError
			require.ErrorAs(t, err, &be)
			require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
		})
	}
}

// TestStackedOpposedSeparatePocketsStayStacked keeps opposed pockets the
// private scene proves apart on the stacked payload.
func TestStackedOpposedSeparatePocketsStayStacked(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	part, err := opposedPockets(t, doc, [4]float64{-6, -2, -2, 2}, [4]float64{2, -2, 6, 2}, true)
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part))
	volume, err := part.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, float64(4000-4*4*4-4*4*6), volumeMM(t, volume))
}

// TestStackedOpposedOverlapDisplacedLevelDeclines keeps overlapping opposed
// pockets that meet at a displaced level off the A8 BRep. On a 12 mm plate a
// top pocket 0.125 in deep has its floor at 12 − fl(3.175), which is no
// float, and a bottom tool sketched on z = −3.175 and extruded 12 mm ends at
// that same rational. The stacked interface arm shares that interface
// (separated pockets build analytically), but the A8 faces state held
// levels, so overlapping pockets there take the mesh path, which refuses the
// coplanar contact under ExactOnly.
func TestStackedOpposedOverlapDisplacedLevelDeclines(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		bottom  [4]float64
		refused bool
	}{
		{name: "separate", bottom: [4]float64{2, -2, 6, 2}},
		{name: "overlap", bottom: [4]float64{-4, -2, 1, 2}, refused: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			plate := boxBody(t, doc, -10, -10, 10, 10, denotedTop)
			part, err := decad.Cut(t.Context(), plate, pocketBelow(t, doc, -6, -2, -2, 2, down(units.Inches(0.125))))
			require.NoError(t, err)
			require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part))
			first, err := part.Volume()
			require.NoError(t, err)
			require.Positive(t, boundMM3(t, first), "the floor carries a level displacement")
			bottom := boxBodyAtZ(t, doc, tc.bottom[0], tc.bottom[1], tc.bottom[2], tc.bottom[3], -3.175, denotedTop)
			part, err = decad.Cut(t.Context(), part, bottom)
			if !tc.refused {
				require.NoError(t, err)
				require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part),
					"equal denoted levels share the displaced interface")
				return
			}
			require.ErrorIs(t, err, decad.ErrUnsupported)
			var be *decad.BooleanError
			require.ErrorAs(t, err, &be)
			require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
		})
	}
}
