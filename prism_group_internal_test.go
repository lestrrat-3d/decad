package decad

import (
	"math"
	"testing"

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
			proven, walk, err := provePrismRegionsDisjoint(t.Context(), proofbound.NewWorkBudget(t.Context()), tc.regions)
			require.NoError(t, err)
			require.Equal(t, tc.want, proven)
			require.Zero(t, walk)
		})
	}
}

func TestPrismGroupPayloadAudit(t *testing.T) {
	doc := New()
	_, base := internalBoxGroup(t, doc)
	require.NoError(t, falsifyStackedPayload(t.Context(), base))
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
			require.ErrorIs(t, falsifyStackedPayload(t.Context(), sp), tc.want)
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
