package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func internalBoxGroup(t *testing.T, doc *Document) (*Body, stackedPrismPayload) {
	t.Helper()
	group, err := Union(t.Context(), internalBoxBody(t, doc, 0, 0, 5, 10, 10), internalBoxBody(t, doc, 10, 0, 15, 10, 10))
	require.NoError(t, err)
	sp, ok := group.payload.(stackedPrismPayload)
	require.True(t, ok)
	require.True(t, sp.isGroup())
	return group, sp
}

func TestPrismGroupRegionsDisjointProof(t *testing.T) {
	doc := New()
	box := func(x0, y0, x1, y1 float64) ProfileRecord {
		return internalBoxBody(t, doc, x0, y0, x1, y1, 1).payload.(prismPayload).profile
	}
	cases := []struct {
		name    string
		regions []ProfileRecord
		want    bool
	}{
		{"apart", []ProfileRecord{box(0, 0, 5, 5), box(10, 0, 15, 5)}, true},
		{"crossing", []ProfileRecord{box(0, 0, 5, 5), box(3, 3, 8, 8)}, false},
		{"nested", []ProfileRecord{box(0, 0, 10, 10), box(3, 3, 6, 6)}, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			proven, walk, err := prismcells.ProveGroupDisjoint(t.Context(), proofbound.NewWorkBudget(t.Context()), tc.regions)
			require.NoError(t, err)
			require.Equal(t, tc.want, proven)
			require.Zero(t, walk)
		})
	}
}

func TestPrismGroupPayloadAudit(t *testing.T) {
	doc := New()
	_, base := internalBoxGroup(t, doc)
	require.NoError(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(base)))
	cases := []struct {
		name   string
		change func(*stackedPrismPayload)
		want   error
	}{
		{"an interface", func(sp *stackedPrismPayload) {
			sp.interfaces = []prismSlabInterface{{}}
		}, ErrUnsupported},
		{"empty interval", func(sp *stackedPrismPayload) { sp.slabs[0].z1 = sp.slabs[0].z0 }, ErrDegenerate},
		{"empty region", func(sp *stackedPrismPayload) { sp.slabs[0].regions[1] = ProfileRecord{} }, ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sp := cloneStackedForAudit(base)
			tc.change(&sp)
			require.ErrorIs(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(sp)), tc.want)
		})
	}
}

// The six-hole cut takes the clean-nesting match: every edge whole, so the
// result carries no section displacement.
func TestPrismGroupSixHoleCutIsUndisplaced(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, -20, 60, 20, 5)
	var group *Body
	for i := range 6 {
		disc := internalCircleBody(t, doc, 5+10*float64(i), 2, 0, Distance{D: units.Millimeters(5), Dir: Along})
		if group == nil {
			group = disc
			continue
		}
		var err error
		group, err = Union(t.Context(), group, disc)
		require.NoError(t, err)
	}
	sp := group.payload.(stackedPrismPayload)
	require.Len(t, sp.slabs[0].regions, 6)
	require.Zero(t, sp.sectionDelta)
	pp, ok, err := tryPrismGroupCut(t.Context(), plate, group)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, pp.profile.Holes, 6)
	require.Zero(t, pp.sectionDelta)
}

// A ring of four lumps around an empty cell: the void guard refuses the
// select-all merge, which would fill the cell.
func TestPrismGroupUnionVoidGuard(t *testing.T) {
	doc := New()
	bars, err := Union(t.Context(), internalBoxBody(t, doc, 0, 0, 10, 2, 3), internalBoxBody(t, doc, 0, 8, 10, 10, 3))
	require.NoError(t, err)
	sides, err := Union(t.Context(), internalBoxBody(t, doc, -1, 1, 2, 9, 3), internalBoxBody(t, doc, 8, 1, 11, 9, 3))
	require.NoError(t, err)
	_, ok, err := tryPrismGroupUnion(t.Context(), bars, sides)
	require.NoError(t, err)
	require.False(t, ok)
}

// The tolerance gate reads every lump: the diameter spans both boxes.
func TestPrismGroupGateDiameterSpansEveryLump(t *testing.T) {
	doc := New()
	group, _ := internalBoxGroup(t, doc)
	d, ok, err := bodyGateDiameter(t.Context(), group)
	require.NoError(t, err)
	require.True(t, ok)
	diagonal := math.Sqrt(15*15 + 10*10 + 10*10)
	require.LessOrEqual(t, d, diagonal)
	require.Greater(t, d, diagonal-1e-9)
}

func TestPrismGroupMirrorJoinRefuses(t *testing.T) {
	doc := New()
	group, _ := internalBoxGroup(t, doc)
	_, err := group.Mirrored(t.Context(), MirrorFace{Body: group, Face: Faces(Planar(), Facing(r3.NewVec(-1, 0, 0)))}, WithJoin())
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "prism group")
}

// TestPrismGroupDisplacedToolChargesTheCrossing is A6 on A5's group paths:
// a group of two 4 mm lumps whose lower edges sit just under a 30×10 plate's
// top edge, turned 0.01 rad about the origin by a hand-built basis (so G3
// still reads the normal exactly). The turn is a nonidentity
// re-expression, and each lump's lower edge now crosses the plate's top edge
// at sin θ < 0.01. Both the group Cut (its crossing sub-case) and the group
// Union build analytically, and each result's section displacement covers
// the re-expression rounding δ_B divided by that sine. Past the charge, an
// amplified group pair never refuses: fu141's overshooting quadrilateral,
// whose merge restates a corner it cannot close (RB9), cut by the same
// turned group crossing its right wall, falls back with no error.
//
// Shown to fail with tryPrismGroupCut's and tryPrismGroupUnion's former
// split-boundary reroute restored (neither built), with either path's
// chargeCrossings call skipped (the displacement fell short of δ_B/0.01),
// and with prismcells.AmplifiedFallback returning every error (the quadrilateral
// returned RB9).
func TestPrismGroupDisplacedToolChargesTheCrossing(t *testing.T) {
	t.Parallel()
	const theta = 0.01
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{X: math.Cos(theta), Y: math.Sin(theta)},
		EY: r3.Vec{X: -math.Sin(theta), Y: math.Cos(theta)},
		EZ: r3.Vec{Z: 1},
	}, r3.Vec{})
	require.NoError(t, err)

	build := func(t *testing.T) (*Body, *Body, float64) {
		t.Helper()
		doc := New()
		plate := internalBoxBody(t, doc, 0, 0, 30, 10, 4)
		group, err := Union(t.Context(), internalBoxBody(t, doc, 5, 9.95, 9, 12, 4), internalBoxBody(t, doc, 20, 9.8, 24, 12, 4))
		require.NoError(t, err)
		tool, err := group.Placed(t.Context(), turn)
		require.NoError(t, err)
		sp, ok := tool.payload.(stackedPrismPayload)
		require.True(t, ok)
		require.True(t, sp.isGroup())

		// δ_B: the re-expression's own rounding over the group's regions.
		target := plate.payload.(prismPayload)
		toolOp, ok := prismGroupOperandOf(tool)
		require.True(t, ok)
		re, err := prismcells.NewReexpression(prismPlacementOf(target), prismPlacementOf(toolOp.proxy))
		require.NoError(t, err)
		require.False(t, re.Identity)
		s, _, _, err := buildPrismSceneRegions(proofbound.NewWorkBudget(t.Context()), []ProfileRecord{target.profile}, toolOp.regions, re)
		require.NoError(t, err)
		split, err := prismcells.HasSplitBoundary(proofbound.NewWorkBudget(t.Context()), s.Profiles())
		require.NoError(t, err)
		require.True(t, split, "the turned lumps must cross the plate's edge, or the fixture tests nothing")
		require.Positive(t, re.Delta)
		return plate, tool, re.Delta
	}

	t.Run("cut", func(t *testing.T) {
		plate, tool, deltaB := build(t)
		got, ok, err := tryPrismGroupCut(t.Context(), plate, tool)
		require.NoError(t, err)
		require.True(t, ok, "A6 charges the displaced group's crossings instead of rerouting them")
		require.GreaterOrEqual(t, got.sectionDelta, deltaB/theta)
	})
	t.Run("an amplified merge failure falls back", func(t *testing.T) {
		doc := New()
		quad := prismOvershootQuadBody(t, doc, [][2]float64{{-9.317, -5.731}, {10.29, -6.113}, {8.877, 7.219}, {-7.331, 6.407}}, 0.13)
		group, err := Union(t.Context(), internalBoxBody(t, doc, 8, -1, 12, 1, prismFixtureHeight), internalBoxBody(t, doc, 7, 3, 11, 5, prismFixtureHeight))
		require.NoError(t, err)
		tool, err := group.Placed(t.Context(), turn)
		require.NoError(t, err)
		_, ok, err := tryPrismGroupCut(t.Context(), quad, tool)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("union", func(t *testing.T) {
		plate, tool, deltaB := build(t)
		got, ok, err := tryPrismGroupUnion(t.Context(), plate, tool)
		require.NoError(t, err)
		require.True(t, ok, "A6 charges the displaced group's crossings instead of rerouting them")
		pp, isPrism := got.(prismPayload)
		require.True(t, isPrism, "the two lumps merge into the plate as one region")
		require.GreaterOrEqual(t, pp.sectionDelta, deltaB/theta)
	})
}
