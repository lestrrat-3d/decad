package prismcells

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// V1 (§5.3): an operand's own recorded vertex lying exactly on another
// operand's circle is no transverse pair of entities: four entities meet
// there, two of them ends of the operand's own segments. The scene is a disc
// and a displaced triangle with one vertex on the rim (angle 0.5) whose two
// flanks leave it, one outward and one inward, so the triangle builds (it is
// counter-clockwise, which a record needs) and the arrangement reaches V1.
//
// This test is not parallel: it switches a guard off for a moment.
func TestReadVerticesOwnVertexOnCircle(t *testing.T) {
	triangle := rgDisplaced(momentinput.Profile{Outer: rgPoly(rgPolar(20, 0.5), rgPolar(24, 0.7), rgPolar(16, 0.3))}, 1e-13)
	sc := rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, triangle)
	_, ok := sc.read(t)
	require.False(t, ok, "V1 refuses the vertex where the triangle's own vertex meets the rim")

	restore := disableGuards("V1")
	defer restore()
	vr, ok := sc.read(t)
	require.True(t, ok, "with V1 off the same arrangement reads, so V1 is what refused it")
	require.NotEmpty(t, vr.Vertices)
}

// V2 (§6.3): a displaced block whose left side leans 1e-3 off the vertical
// crosses a square's right side 5e-5 from the square's own corner, inside the
// trim length 8F/s0 of 1e-4, so the audit refuses; with V2 off the same scene
// reads stable.
func TestStabilityGuardIsV2(t *testing.T) {
	const cross, delta = 10 - 5e-5, 1e-13
	x := func(y float64) float64 { return 10 + 1e-3*(y-cross) }
	block := momentinput.Profile{Outer: rgPoly(sectionrecord.Point2{U: x(cross - 2), V: cross - 2},
		sectionrecord.Point2{U: 14, V: cross - 2}, sectionrecord.Point2{U: 14, V: cross + 2},
		sectionrecord.Point2{U: x(cross + 2), V: cross + 2})}
	sc := rgArrange(t, prismrecord.Payload{Profile: rgSquare(0, 0, 10, 10)}, rgDisplaced(block, delta))
	stable, ok := sc.stable(t)
	require.True(t, ok)
	require.False(t, stable)

	restore := disableGuards("V2")
	defer restore()
	stable, ok = sc.stable(t)
	require.True(t, ok)
	require.True(t, stable, "with V2 off the audit passes, so V2 is what refused the scene")
}

// V3 (§5.3): two flanks that cross the rim within the charge of each other
// could swap; with V3 off the order check passes.
func TestOrderGuardIsV3(t *testing.T) {
	const delta = 2e-4
	tip := rgDisplaced(momentinput.Profile{Outer: rgPoly(rgPolar(18, -0.1), rgPolar(18, 0.1),
		sectionrecord.Point2{U: 20 + 1e-3})}, delta)
	sc := rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, tip)
	require.False(t, sc.ordered(t))

	restore := disableGuards("V3")
	defer restore()
	require.True(t, sc.ordered(t), "with V3 off the order check passes, so V3 is what refused the scene")
}

// V3 on a whole circle (§5.3) takes the arranged vertices cyclically: the last
// one before the seam pairs with the first one after it. A displaced tooth's
// flank crosses the rim 4e-3 before the seam point, closer than its charge
// allows, while its other flank is far away, so only the wrap-around pair
// (last vertex, seam) is too close.
func TestOrderedPairsWholeCircleCyclically(t *testing.T) {
	const delta = 1e-3
	tooth := rgDisplaced(momentinput.Profile{Outer: rgPoly(rgPolar(18, -2e-4), rgPolar(24, -2e-4),
		rgPolar(24, 0.3), rgPolar(18, 0.3))}, delta)
	sc := rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, tooth)
	require.False(t, sc.ordered(t))

	restore := disableGuards("V3cyclic")
	defer restore()
	require.True(t, sc.ordered(t), "without the wrap-around pair the last vertex is never compared with the seam")
}
