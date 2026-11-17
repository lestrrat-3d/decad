package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// obliqueWall is a window side of spokedHubDisc: a plane face whose normal lies in the XY plane.
type obliqueWall struct {
	face   *decad.Face
	plane  decad.Plane
	bound  float64
	normal r3.Vec
}

// obliqueWalls lists the faces of body that are analytic planes with a normal in the XY plane, with NormalAt's
// bound at the plane's origin.
func obliqueWalls(t *testing.T, body *decad.Body) []obliqueWall {
	t.Helper()
	var out []obliqueWall
	for _, f := range body.Faces() {
		plane, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(plane.Frame.N().Z) > 1e-9 {
			continue
		}
		n, err := f.NormalAt(plane.Frame.Origin())
		require.NoError(t, err)
		out = append(out, obliqueWall{face: f, plane: plane, bound: n.Bound.Base(), normal: n.Value})
	}
	return out
}

// trueSideNormal is the outward unit normal of the window side that lies on the line through the origin at 45
// degrees, offset 2 toward the wall's own side: the side's tag normal is (+-1, +-1)/sqrt2, and the true wall is
// the plane n . x = 2 for the matching sign.
func trueSideNormal(n r3.Vec) r3.Vec {
	return r3.Vec{X: math.Copysign(math.Sqrt2/2, n.X), Y: math.Copysign(math.Sqrt2/2, n.Y)}
}

// TestStackedViewObliqueWallTag builds the spoked hub disc with a Ø14 root-filleted boss. The union's record
// carries a section displacement, so its faceted view rewrites each of the four windows' two oblique sides between
// canonical vertices. The view keeps each side's Plane tag and states its tilt (charged-window-chamfer-design §2),
// so a side is a decad.Plane with a positive NormalAt bound that covers the distance to the true wall's normal,
// and its rim with a cap is a Line3.
//
// Shown red: without the view's tilt entry the sides read Faceted and the rims FacetedCurve.
func TestStackedViewObliqueWallTag(t *testing.T) {
	t.Parallel()
	doc, disc := spokedHubDisc(t)
	part, err := bossOnTop(t, disc, 14, 0.5)
	require.NoError(t, err)

	walls := obliqueWalls(t, part)
	require.Len(t, walls, 8, "four windows with two oblique sides each")
	for _, w := range walls {
		n := w.plane.Frame.N()
		want := trueSideNormal(n)
		require.Greater(t, w.bound, 1e-15, "the stated tilt is far above NormalAt's own rounding")
		require.Less(t, w.bound, 1e-9)
		// The bound covers the tag normal's distance from the true wall's.
		require.LessOrEqual(t, n.Sub(want).Len(), w.bound)
		// The tag passes through the true wall within the same order of magnitude: its offset from the true plane.
		require.InDelta(t, 2, math.Abs(n.Dot(w.plane.Frame.Origin())), 1e-9)

		var rims int
		for _, e := range w.face.Edges() {
			if _, line := e.Curve().(decad.Line3); !line {
				continue
			}
			for _, other := range e.Faces() {
				if p, ok := other.Surface().(decad.Plane); ok && math.Abs(p.Frame.N().Z) > 0.5 {
					rims++
				}
			}
		}
		require.Equal(t, 2, rims, "a Line3 rim with each cap")
	}

	// The mesh Cut below retires part, so the Chamfer subtest reads its own copy.
	chamferPart, err := decad.Copy(t.Context(), part)
	require.NoError(t, err)

	t.Run("the uncharged twin states no tilt", func(t *testing.T) {
		t.Parallel()
		twin := obliqueWalls(t, spokedDisc(t))
		require.Len(t, twin, 8)
		for _, w := range twin {
			require.Less(t, w.bound, 1e-15, "a prism's side carries only NormalAt's rounding")
		}
	})

	t.Run("a Chamfer of a side rim alone refuses at its tangent junction", func(t *testing.T) {
		t.Parallel()
		w := walls[0]
		for _, e := range w.face.Edges() {
			if _, line := e.Curve().(decad.Line3); !line {
				continue
			}
			for _, other := range e.Faces() {
				p, ok := other.Surface().(decad.Plane)
				if !ok || p.Frame.N().Z > -0.5 {
					continue
				}
				edge := decad.Edges(decad.EndpointAt(e.Start().Position().Value), decad.EndpointAt(e.End().Position().Value)).Exactly(1)
				_, err := decad.Chamfer(t.Context(), chamferPart, edge, units.Millimeters(0.5))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				// The side's rim ends at a tangent junction with a corner round. A closed chamfer chain reads the
				// oblique wall (charged-window-chamfer-design §3), so the selection is one edge of a chain whose
				// continuation is not selected.
				require.ErrorContains(t, err, "whose continuation is not selected")
				return
			}
		}
		require.Fail(t, "no bottom rim")
	})

	t.Run("a mesh Cut away from the windows keeps the sides tagged", func(t *testing.T) {
		t.Parallel()
		pocket := boxBodyAtZ(t, doc, 19.5, -0.5, 20.5, 0.5, 1, 2)
		cut, err := decad.Cut(t.Context(), part, pocket)
		require.NoError(t, err)
		// The pocket's own four walls are exact planes with no stated tilt.
		var kept []obliqueWall
		for _, w := range obliqueWalls(t, cut) {
			if w.bound > 1e-15 {
				kept = append(kept, w)
			}
		}
		require.Len(t, kept, 8)
		byOrigin := map[r3.Vec]obliqueWall{}
		for _, w := range walls {
			byOrigin[w.plane.Frame.Origin()] = w
		}
		for _, w := range kept {
			before, ok := byOrigin[w.plane.Frame.Origin()]
			require.True(t, ok, "a kept side has the tag origin of one built before the Cut")
			require.Equal(t, before.bound, w.bound)
			require.Equal(t, before.plane.Frame.N(), w.plane.Frame.N())
		}
	})
}
