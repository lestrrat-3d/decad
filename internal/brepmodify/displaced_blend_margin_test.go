package brepmodify

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// These are Table PD's per-corner margins (agent-docs/brep-modify-design.md
// §4.4) on crafted numbers: each refuses a reading within its margin and
// passes one clear of it. Shown-to-fail: deleting the comparison in
// requireTurnMargin, requireRootMargin or requireSenseMargin, the radius
// scaling of PD5c's rounding allowance, or the widening in widenDisplaced,
// turns the matching refusal below into a pass.

func TestDisplacedTurnMargin(t *testing.T) {
	t.Parallel()
	tol := sectionaudit.Tolerance
	theta := 1e-6
	err := requireTurnMargin(tol+theta/2, theta, 0)
	require.ErrorIs(t, err, errPD)
	require.Equal(t, "PD5a", pdRow(err))
	require.NoError(t, requireTurnMargin(-(tol+2*theta), theta, 0))
	require.NoError(t, requireTurnMargin(0.5, 0, 0), `a line–line corner needs only S4's margin`)
}

func TestDisplacedRootMargin(t *testing.T) {
	t.Parallel()
	const delta = 1e-6
	near := [][2]float64{{1, 0}, {-1 - delta/2, 0}}
	err := requireRootMargin(near, 0, 0, delta)
	require.ErrorIs(t, err, errPD)
	require.Equal(t, "PD5b", pdRow(err))
	require.NoError(t, requireRootMargin([][2]float64{{1, 0}, {-1 - 3*delta, 0}}, 0, 0, delta))
	require.NoError(t, requireRootMargin([][2]float64{{1, 0}}, 0, 0, delta), `a single root decides nothing`)
	require.NoError(t, requireRootMargin([][2]float64{{1, 0}, {1, 0}}, 0, 0, delta), `a double root decides nothing`)
}

func TestDisplacedSenseMargin(t *testing.T) {
	t.Parallel()
	const r, theta = 2.0, 1e-6
	err := requireSenseMargin(r*theta/2, r, theta)
	require.ErrorIs(t, err, errPD)
	require.Equal(t, "PD5c", pdRow(err))
	require.NoError(t, requireSenseMargin(-r, r, theta))

	// The rounding allowance scales with the radius: at r = 1000 a reading
	// of 2⁻⁴⁹·r is past any fixed 2⁻⁵⁰ but within r·2⁻⁴⁸.
	const big = 1000.0
	err = requireSenseMargin(math.Ldexp(big, -49), big, 0)
	require.ErrorIs(t, err, errPD)
	require.NoError(t, requireSenseMargin(math.Ldexp(big, -46), big, 0))
}

// TestDisplacedCutbackWidening is PD6 on the D-flat section with a
// displacement of 1e-3: the line walk's two claims fit its length by 3δ,
// which S6 passes as recorded and refuses once each displaced end grows by
// 2δ'.
func TestDisplacedCutbackWidening(t *testing.T) {
	t.Parallel()
	const delta = 1e-3
	y := math.Sqrt(51)
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: y, V: 7}, End: sectionrecord.Point2{U: -y, V: 7}, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: -y, V: 7}, End: sectionrecord.Point2{U: y, V: 7}, TStart: 0, TEnd: 1},
	}}}
	budget := proofbound.NewWorkBudget(t.Context())
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	require.NoError(t, err)
	require.Len(t, loops, 1)
	require.Len(t, loops[0].Walks, 2)
	c := SectionCarriers{Delta: delta, Radius: [][]float64{{0, 10}}, Center: [][]sectionrecord.Point2{{{}, {}}}}
	exact, err := c.carrierLoops(loops)
	require.NoError(t, err)
	line := exact[0].Walks[0]
	require.False(t, line.IsCircular())
	// Corner 0 leaves along the line, corner 1 arrives on it.
	half := (line.Length - sectionaudit.Tolerance*line.Length - 3*delta) / 2
	blendAt := []map[int]*offset2d.Blend{{
		0: {CutbackA: 0.1, CutbackB: half},
		1: {CutbackA: half, CutbackB: 0.1},
	}}
	require.NoError(t, offset2d.AuditRewriteBudget(budget, profile, profile, exact, blendAt))
	err = offset2d.AuditRewriteBudget(budget, profile, profile, exact, c.widenDisplaced(exact, blendAt))
	require.ErrorIs(t, err, sectionaudit.ErrUnsupported)
	require.ErrorContains(t, err, `setback`)
}

// TestDisplacedCarrierCircle is PD3's three legs on one arc of radius 10
// about the origin from (−6, 8) to (6, 8): a centre off the carrier's, an
// endpoint outside the annulus, and a displacement of half the radius each
// refuse, and the arc on its carrier passes.
func TestDisplacedCarrierCircle(t *testing.T) {
	t.Parallel()
	arc := sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: -6, V: 8}, End: sectionrecord.Point2{U: 6, V: 8}, TStart: 0, TEnd: 1}
	origin := sectionrecord.Point2{}
	require.NoError(t, requireCarrierCircle(arc, origin, 10, 1e-12))
	require.ErrorContains(t, requireCarrierCircle(arc, sectionrecord.Point2{U: math.Nextafter(0, 1)}, 10, 1e-12), `centre`)
	require.ErrorContains(t, requireCarrierCircle(arc, origin, 10.5, 1e-12), `farther than the displacement`)
	require.ErrorContains(t, requireCarrierCircle(arc, origin, 10, 5), `half`)
}
