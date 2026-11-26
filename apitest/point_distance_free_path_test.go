package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestDistanceToPointFreePath measures from points in a bore of a large mesh Boolean result, where the body's
// whole symmetric-difference bound dwarfs any ball about the point, so only a free path proves the point outside
// (agent-docs/point-containment-design.md §2, "Free path"). The distance then reads within the mesh bound of the
// exact one, before and after a Transform.
func TestDistanceToPointFreePath(t *testing.T) {
	t.Parallel()
	sc := newTCScene()
	link := tcLink(t, sc, false)
	bore := sc.prism(t, r3.NewVec(0, 11, 0), tcX, tcZ, []tcSeg{
		tcArc(0, 0, 2.8, 0, -2.8, 0), tcArc(0, 0, -2.8, 0, 2.8, 0),
	}, tcAlong(22))
	body := tcCut(t, link, bore)
	turn, err := r3.Rotation(r3.NewVec(0, -1, 0), units.Degrees(70))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(30, -5, 14))
	require.NoError(t, err)
	placed, err := turn.Then(shift)
	require.NoError(t, err)
	moves := []struct {
		name string
		xf   r3.Transform
	}{{"as built", r3.Identity()}, {"rotated and moved", placed}}
	for _, m := range moves {
		// Both subtests mutate body's document through Copy and Transform, and a
		// Document is not safe for concurrent mutation, so they run in sequence.
		t.Run(m.name, func(t *testing.T) {
			b, err := decad.Copy(body).Do(t.Context())
			require.NoError(t, err)
			if m.name != "as built" {
				b, err = decad.Transform(b, m.xf).Do(t.Context())
				require.NoError(t, err)
			}
			// Points in the bore, 0.3 and 1.3 from its wall of radius 2.8.
			for _, at := range []struct {
				p    r3.Vec
				want float64
			}{{r3.NewVec(0, 0, 2.5), 0.3}, {r3.NewVec(-1.5, 2, 0), 1.3}} {
				d, err := b.DistanceToPoint(t.Context(), m.xf.Apply(at.p), units.Millimeters(0.05))
				require.NoError(t, err)
				require.InDelta(t, at.want, d.Value.Base(), d.Bound.Base(), "at %v", at.p)
				require.Less(t, d.Bound.Base(), 0.05, "at %v", at.p)
			}
		})
	}
}
