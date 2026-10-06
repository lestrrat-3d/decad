package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the PR 3 tests of docs/linkage-check-design.md: the layer
// exclusion (§5.7), the link-link swept-box exclusion and the held links of
// §6 steps 2 and 4. An exclusion admits a pair without evaluating it, so each
// rule is pinned by a fixture whose answer goes wrong when the rule lets
// through a pair it must not.

// requirePairUnevaluated asserts no pose of the report carries a row or a
// finding for the pair.
func requirePairUnevaluated(t *testing.T, report *decad.LinkageReport, a, b *decad.Body) {
	t.Helper()
	for _, p := range report.Poses {
		for _, row := range p.Clearances {
			require.False(t, row.A == a && row.B == b, `the pair is settled before any pose`)
		}
		for _, row := range p.Interferences {
			require.False(t, row.A == a && row.B == b, `the pair is settled before any pose`)
		}
		for _, d := range p.Diagnostics {
			require.False(t, d.Pair != nil && d.Pair.A == a && d.Pair.B == b, `the pair is settled before any pose`)
		}
	}
}

func requireCollisionOn(t *testing.T, report *decad.LinkageReport, a, b *decad.Body) {
	t.Helper()
	require.Equal(t, decad.Interfering, report.Status)
	for _, c := range report.Collisions {
		if c.A == a && c.B == b {
			return
		}
	}
	require.Fail(t, `no collision on the pair`)
}

// TestVerifyLinkageLayerExclusion pins §5.7. An arm x ∈ [0, 48],
// y ∈ [−14, 14], z ∈ [0, 10] turns 0° → 30° about Z; a table spans
// x, y ∈ [−100, 100].
//
//   - Over a table at z ∈ [−20, −10], the arm's z-extent is held by its
//     joint, 10 mm above the table's: the pair is settled with the exact
//     lower bound 10 and evaluated at no pose.
//   - A block x ∈ [30, 40], y ∈ [−5, 5], z ∈ [20, 30] under the arm turns
//     0° → 180° about the X axis through (0, 0, 10), off Z, and comes down to
//     z ∈ [−10, 0] into a table at z ∈ [−10, −2]: the off-axis joint keeps the
//     pair evaluated, and the collision is found.
//   - The same block slides 0 → 40 mm along (1, 0, −1), not perpendicular to
//     Z, down into the same table: found as well.
//   - The arm resting on a table's top face z = 0 touches it: touching
//     extents never exclude, so the pair is evaluated and the drive Suspect.
//   - A block sliding along (1, 0, 1) and then (0, 1, 1) has no coordinate
//     axis its slides are perpendicular to, but both are perpendicular to
//     their cross product (−1, −1, 1), along which a box far up and back is
//     140 mm away: settled.
//
// Legs seen to fail when deleted: the revolute-axis test (the off-axis block
// is settled and its collision missed); the slide-perpendicularity test (the
// sliding block's collision missed); the strict w > 0, with the positive-bound
// check behind it (the resting arm reads Sound); and the cross-product candidate (the slanted box is evaluated).
func TestVerifyLinkageLayerExclusion(t *testing.T) {
	t.Parallel()
	turn := func(link *decad.Link) decad.JointSweep {
		return decad.JointSweep{Link: link, From: units.Degrees(0), To: units.Degrees(30)}
	}
	t.Run("an arm over a table is settled", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		arm := motionArm(t, doc)
		table := boxBodyAtZ(t, doc, -100, -100, 100, 100, -20, 10)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{turn(swing)})
		require.Equal(t, decad.Sound, report.Status)
		require.Len(t, report.Poses, 2)
		requirePairUnevaluated(t, report, arm, table)
		require.Equal(t, 10.0, report.Intervals[0].Clearance.Value.Mag())
	})
	offAxisArm := func(t *testing.T, attach func(arm *decad.Link, block *decad.Body) (decad.JointSweep, error)) (*decad.LinkageReport, *decad.Body, *decad.Body, *decad.Body) {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		block := boxBodyAtZ(t, doc, 30, -5, 40, 5, 20, 10)
		table := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 8)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		sweep, err := attach(swing, block)
		require.NoError(t, err)
		return verifyLinkage(t, doc, l, decad.Drive{turn(swing), sweep}, decad.WithResolution(units.Scalar(1.0/64))), arm, block, table
	}
	t.Run("an off-axis joint keeps its pair evaluated", func(t *testing.T) {
		t.Parallel()
		report, arm, block, table := offAxisArm(t, func(swing *decad.Link, block *decad.Body) (decad.JointSweep, error) {
			tilt, err := swing.Revolute(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), []*decad.Body{block})
			return decad.JointSweep{Link: tilt, From: units.Degrees(0), To: units.Degrees(180)}, err
		})
		requireCollisionOn(t, report, block, table)
		requirePairUnevaluated(t, report, arm, table)
	})
	t.Run("a slide along Z keeps its pair evaluated", func(t *testing.T) {
		t.Parallel()
		report, arm, block, table := offAxisArm(t, func(swing *decad.Link, block *decad.Body) (decad.JointSweep, error) {
			slide, err := swing.Prismatic(r3.NewVec(1, 0, -1), []*decad.Body{block})
			return decad.JointSweep{Link: slide, From: units.Millimeters(0), To: units.Millimeters(40)}, err
		})
		requireCollisionOn(t, report, block, table)
		requirePairUnevaluated(t, report, arm, table)
	})
	t.Run("touching extents never exclude", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		arm := motionArm(t, doc)
		table := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{turn(swing)}, decad.WithResolution(units.Scalar(1.0/4)))
		require.Equal(t, decad.Suspect, report.Status)
		first := report.Poses[0]
		require.Len(t, first.Clearances, 1)
		require.Same(t, table, first.Clearances[0].B)
		require.Equal(t, decad.Exact, first.Clearances[0].Gap.Exactness)
		require.Zero(t, first.Clearances[0].Gap.Value.Mag())
	})
	t.Run("two slides settle along their cross product", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := boxBody(t, doc, 0, 0, 10, 10, 10)
		far := boxBodyAtZ(t, doc, -60, -60, -50, -50, 50, 10)
		carrier := boxBodyAtZ(t, doc, 200, 200, 210, 210, 0, 10)
		l2 := decad.NewLinkage()
		hub, err := l2.Ground().Prismatic(r3.NewVec(1, 0, 1), []*decad.Body{carrier})
		require.NoError(t, err)
		tip, err := hub.Prismatic(r3.NewVec(0, 1, 1), []*decad.Body{block})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l2, decad.Drive{
			{Link: hub, From: units.Millimeters(0), To: units.Millimeters(30)},
			{Link: tip, From: units.Millimeters(0), To: units.Millimeters(30)},
		})
		requirePairUnevaluated(t, report, block, far)
		require.Contains(t, report.Against, far)
	})
}

// TestVerifyLinkageLinkSweptBoxExclusion pins §6 step 4 for a link-link pair.
// Two links hang from the ground: the motion arm turning 0° → 90° about Z,
// and a block x ∈ [−5, 5], y ∈ [60, 70], z ∈ [0, 10] in the arm's layer.
//
//   - Sliding 0 → 10 mm along +X far away at x ∈ [500, 510], the block's swept
//     box never meets the arm's: the pair is settled and evaluated at no pose.
//   - Sliding 0 → 30 mm along −Y toward the arm, the block meets it: the rest
//     boxes are 46 mm apart, but each grows by its own link's travel from the
//     zero pose, and the collision is found.
//
// Leg seen to fail when deleted: growing each box by its link's travel (the
// approaching block's pair is settled on its rest boxes and the collision
// missed).
func TestVerifyLinkageLinkSweptBoxExclusion(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T, x0 float64, dir r3.Vec, to float64) (*decad.LinkageReport, *decad.Body, *decad.Body) {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		block := boxBody(t, doc, x0-5, 60, x0+5, 70, 10)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		slide, err := l.Ground().Prismatic(dir, []*decad.Body{block})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{
			{Link: swing, From: units.Degrees(0), To: units.Degrees(90)},
			{Link: slide, From: units.Millimeters(0), To: units.Millimeters(to)},
		}, decad.WithResolution(units.Scalar(1.0/64)))
		return report, arm, block
	}
	t.Run("a far block is settled", func(t *testing.T) {
		t.Parallel()
		report, arm, block := build(t, 505, r3.NewVec(1, 0, 0), 10)
		require.Equal(t, decad.Sound, report.Status)
		require.Len(t, report.Poses, 2)
		requirePairUnevaluated(t, report, arm, block)
	})
	t.Run("an approaching block is evaluated", func(t *testing.T) {
		t.Parallel()
		report, arm, block := build(t, 0, r3.NewVec(0, -1, 0), 30)
		requireCollisionOn(t, report, arm, block)
	})
}

// TestVerifyLinkageHeldLinks pins §6 step 2. A base x, y ∈ [−10, 10],
// z ∈ [0, 10] rests on a table's top face z = 0 on a joint the drive never
// moves, beside a wall x ∈ [15, 25] in its layer, with an arm above it.
//
//   - The base holds 0, so it stands where it is: its pairs with the table
//     and the wall are Verify's, not formed here, and the drive reads Sound.
//   - Held at 30° instead, the base is a constant placement: its pair with
//     the wall is formed, and every pose reports the same row.
//
// Leg seen to fail when deleted: leaving a held link's pair against a static
// body unformed (the resting base's touching pair reads Suspect).
func TestVerifyLinkageHeldLinks(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T, held units.Value) (*decad.LinkageReport, *decad.Body, *decad.Body, *decad.Body) {
		t.Helper()
		doc := decad.New()
		base := boxBody(t, doc, -10, -10, 10, 10, 10)
		arm := boxBodyAtZ(t, doc, 0, -14, 48, 14, 12, 10)
		table := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
		wall := boxBody(t, doc, 15, -30, 25, 30, 10)
		l := decad.NewLinkage()
		pedestal, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{base})
		require.NoError(t, err)
		swing, err := pedestal.Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		drive := decad.Drive{{Link: swing, From: units.Degrees(0), To: units.Degrees(90)}}
		if held.Mag() != 0 {
			drive = append(drive, decad.JointSweep{Link: pedestal, From: held, To: held})
		}
		return verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/4))), base, table, wall
	}
	t.Run("a link holding 0 stands where it is", func(t *testing.T) {
		t.Parallel()
		report, base, table, wall := build(t, units.Degrees(0))
		require.Equal(t, decad.Sound, report.Status)
		requirePairUnevaluated(t, report, base, table)
		requirePairUnevaluated(t, report, base, wall)
	})
	t.Run("a link held away from 0 is a constant placement", func(t *testing.T) {
		t.Parallel()
		report, base, _, wall := build(t, units.Degrees(30))
		var first *decad.Clearance
		for _, p := range report.Poses {
			for _, row := range p.Clearances {
				if row.A != base || row.B != wall {
					continue
				}
				if first == nil {
					first = &row
					continue
				}
				require.Equal(t, first.Gap, row.Gap, `a constant placement measures the same at every pose`)
			}
		}
		require.NotNil(t, first)
	})
}
