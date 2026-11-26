package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// wallSelector names the one face of body that wall is, by the first origin role that no other face carries.
func wallSelector(t *testing.T, body *decad.Body, wall obliqueWall) decad.FaceSelector {
	t.Helper()
	for _, o := range wall.face.Origins() {
		sel := decad.Faces(decad.FaceCreatedBy(o))
		faces, err := sel.SelectFaces(body)
		if err == nil && len(faces) == 1 {
			return sel
		}
	}
	require.Fail(t, "no origin names this wall alone")
	return nil
}

// TestObliqueWallReaders refuses the readers that take a plane face's tag as the plane when the wall states that
// its true patch may lie off the tag (ExtentDelta; charged-window-chamfer-design §4): Hole's planar site, and
// Boss and Emboss through resolvePlanarFace. The uncharged twin, the same window side of the disc alone, builds.
func TestObliqueWallReaders(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	// A point of each wall's plane, 2.5 above its bottom edge, on the straight stretch of the side.
	at := func(w obliqueWall) r3.Vec { return w.plane.Frame.Origin().Add(r3.Vec{Z: 2.5}) }
	// fixture builds a root-filleted boss hub, and apart from it the disc alone, each from its own document.
	fixture := func(t *testing.T) (part *decad.Body, charged obliqueWall, disc *decad.Body, twin obliqueWall) {
		t.Helper()
		_, hub := spokedHubDisc(t)
		part, err := bossOnTop(t, hub, 14, 0.5)
		require.NoError(t, err)
		_, disc = spokedHubDisc(t)
		return part, obliqueWalls(t, part)[0], disc, obliqueWalls(t, disc)[0]
	}
	const refusal = "its tag is an oblique wall of a stacked solid"

	t.Run("Hole", func(t *testing.T) {
		t.Parallel()
		part, charged, disc, twin := fixture(t)
		cmd := func(body *decad.Body, w obliqueWall) *decad.HoleCommand {
			return decad.Hole(body, wallSelector(t, body, w), at(w), mm(1), decad.BlindHole{Depth: mm(1)})
		}
		_, err := cmd(part, charged).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "Hole cannot read this planar face as its plane")
		require.ErrorContains(t, err, refusal)

		out, err := cmd(disc, twin).Do(t.Context())
		require.NoError(t, err)
		require.True(t, out.IsSolid())
	})

	t.Run("Boss", func(t *testing.T) {
		t.Parallel()
		part, charged, disc, twin := fixture(t)
		cmd := func(body *decad.Body, w obliqueWall) *decad.BossCommand {
			return decad.Boss(body, wallSelector(t, body, w), at(w), mm(1), mm(1))
		}
		_, err := cmd(part, charged).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "Boss cannot read this planar face as its plane")
		require.ErrorContains(t, err, refusal)

		out, err := cmd(disc, twin).Do(t.Context())
		require.NoError(t, err)
		require.True(t, out.IsSolid())
	})

	t.Run("Emboss", func(t *testing.T) {
		t.Parallel()
		part, charged, disc, twin := fixture(t)
		emboss := func(body *decad.Body, w obliqueWall) (*decad.Body, error) {
			world := sketch.NewWorld()
			plane, err := world.CreatePlaneFromFrame(w.plane.Frame)
			require.NoError(t, err)
			s, err := world.CreateSketch(plane)
			require.NoError(t, err)
			c := s.CreatePoint(0, 2.5)
			s.Fix(c)
			s.CreateCircle(c, 0.5)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return decad.Emboss(body, s, s.Profiles(), wallSelector(t, body, w), decad.EmbossRaised, mm(0.5)).Do(t.Context())
		}
		_, err := emboss(part, charged)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "Emboss cannot read this planar face as its plane")
		require.ErrorContains(t, err, refusal)

		out, err := emboss(disc, twin)
		require.NoError(t, err)
		require.True(t, out.IsSolid())
	})
}
