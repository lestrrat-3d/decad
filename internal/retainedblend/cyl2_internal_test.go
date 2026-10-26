package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// mixedWalls is a concave R5 edge between a bore wall (radius 6 about (14, 10),
// material outside) and a post wall (radius 3 about (19, 10), material inside).
// Their offset circles are 6 - r and 3 + r from the axes, so a large r leaves
// them not meeting. The public API does not reach that refusal. Two walls of the
// same kind offset to R + r (or R - r) each, and |R1 - R2| < s < R1 + R2 keeps
// those circles meeting for every r. A post through a bore wall is a mixed pair,
// but its ends are convex corners of the post's top face while the edge is
// concave, so the edge refuses at its ends first.
func mixedWalls(t *testing.T) (c cylCyl, k0 r3.Vec, o, u, w [2]r3.Vec) {
	t.Helper()
	axis := r3.NewVec(0, 0, 1)
	c = cylCyl{axis: axis, scale: 40}
	c.wall[0] = cylWall{origin: r3.NewVec(14, 10, 0), radius: 6, matOutside: true}
	c.wall[1] = cylWall{origin: r3.NewVec(19, 10, 0), radius: 3}
	// The generator where the circles meet, nearer y = 0.
	x := 14 + (25.0+36-9)/10
	k0 = r3.NewVec(x, 10-math.Sqrt(36-(x-14)*(x-14)), 0)
	for k := range 2 {
		o[k] = c.wall[k].axisPoint(axis, k0)
		u[k] = k0.Sub(o[k]).Scale(1 / c.wall[k].radius)
	}
	// Each wall's face leaves the corner along the part of its circle that
	// borders the air: the bore outside the post, the post inside the bore.
	w[0] = axis.Cross(u[0]).Scale(-1)
	w[1] = axis.Cross(u[1]).Scale(-1)
	return c, k0, o, u, w
}

func TestCylCylSectionNoBallTouchesBothWalls(t *testing.T) {
	t.Parallel()
	c, k0, o, u, w := mixedWalls(t)

	t.Run("a ball that fits is tangent to both walls", func(t *testing.T) {
		t.Parallel()
		const r = 1.0
		s, err := c.solveSection(blendSpec{fillet: true, r: r}, k0, o, u, w, false, 1e-3, 1e-9)
		require.NoError(t, err)
		// Inside the bore at 6 - r from its axis, outside the post at 3 + r.
		require.InDelta(t, 6-r, s.centre.Sub(o[0]).Len(), 1e-12)
		require.InDelta(t, 3+r, s.centre.Sub(o[1]).Len(), 1e-12)
	})

	t.Run("a ball too large for the gap between the offset circles", func(t *testing.T) {
		t.Parallel()
		// Offset radii 1 and 8 about axes 5 apart: the circles are nested.
		_, err := c.solveSection(blendSpec{fillet: true, r: 5}, k0, o, u, w, false, 1e-3, 1e-9)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "no ball of that radius touches both faces")
	})
}
