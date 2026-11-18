package offset2d_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// cornerFillet rounds the corner of the line loop through pts that starts at
// at, with FilletAtCorner.
func cornerFillet(t *testing.T, at [2]float64, swap bool, pts ...[2]float64) *offset2d.Blend {
	t.Helper()
	loops, err := offset2d.ProfileCornerLoopsBudget(proofbound.NewWorkBudget(t.Context()),
		momentinput.Profile{Outer: draftLoop(pts...)})
	require.NoError(t, err)
	require.Len(t, loops, 1)
	for ci, w := range loops[0].Walks {
		if w.StartU == at[0] && w.StartV == at[1] {
			b, err := offset2d.FilletAtCorner(loops[0].Walks, ci, 0.3, sectionaudit.Tolerance, swap)
			require.NoError(t, err)
			return b
		}
	}
	require.Fail(t, "no corner", "at %v", at)
	return nil
}

// TestFilletAtCornerAgreesAcrossFaces rounds one solid corner as two end
// faces of a route E edge hold it (agent-docs/brep-modify-design.md §5.3 step
// 2). G0 is the face x ∈ [−3.5, 3.5], y ∈ [0, 3.6]. G1 holds the same corner
// (−3.5, 0) through a frame that exchanges the axes, (u, v) = (y, x), so it
// reflects, and its region runs on to x = 7. G0's loop arrives at the corner
// along the wall x = −3.5 and G1's along the cap y = 0, so G0 swaps and G1
// does not: both read the cap's walk arriving. The two blends are mirror
// images bit for bit, though the cap's far ends differ: read through a far
// end, the two cap feet part by an ulp.
func TestFilletAtCornerAgreesAcrossFaces(t *testing.T) {
	t.Parallel()
	g0 := cornerFillet(t, [2]float64{-3.5, 0}, true,
		[2]float64{-3.5, 0}, [2]float64{3.5, 0}, [2]float64{3.5, 3.6}, [2]float64{-3.5, 3.6})
	g1 := cornerFillet(t, [2]float64{0, -3.5}, false,
		[2]float64{0, -3.5}, [2]float64{3.6, -3.5}, [2]float64{3.6, 7}, [2]float64{0, 7})
	mirror := func(p sectionrecord.Point2) sectionrecord.Point2 { return sectionrecord.Point2{U: p.V, V: p.U} }

	// G0 arrives on the wall and leaves on the cap; G1 the other way round.
	require.Equal(t, g0.FA, mirror(g1.FB))
	require.Equal(t, g0.FB, mirror(g1.FA))
	a0, ok := g0.Connector.(sectionrecord.ArcSeg)
	require.True(t, ok)
	a1, ok := g1.Connector.(sectionrecord.ArcSeg)
	require.True(t, ok)
	require.Equal(t, a0.Center, mirror(a1.Center))
	// The feet lie r from the corner along each carrier and the centre r off
	// both: the cap's foot at x = −3.2 and the wall's at y = 0.3.
	require.InDelta(t, -3.2, g0.FB.U, 1e-15)
	require.Equal(t, 0.0, g0.FB.V)
	require.Equal(t, -3.5, g0.FA.U)
	require.InDelta(t, 0.3, g0.FA.V, 1e-15)
	// Each loop walks its connector in its own sense: from the arriving
	// walk's foot to the leaving walk's.
	require.Equal(t, g0.FA, arcFrom(a0))
	require.Equal(t, g1.FA, arcFrom(a1))
}

// arcFrom is the point an arc segment walks from.
func arcFrom(a sectionrecord.ArcSeg) sectionrecord.Point2 {
	if a.TStart < a.TEnd {
		return a.Start
	}
	return a.End
}
