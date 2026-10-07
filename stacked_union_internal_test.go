package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalCircleBody extrudes a circle centred at (x, 0) on the XY plane
// offset by z, with the given extent.
func internalCircleBody(t *testing.T, doc *Document, x, radius, z float64, extent Extent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(x, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], extent)
	require.NoError(t, err)
	return body
}

func internalBossOnPlate(t *testing.T, bossX, bossZ, bossHeight float64) (*Body, *Body) {
	t.Helper()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalCircleBody(t, doc, bossX, 5, bossZ, Distance{D: units.Millimeters(bossHeight), Dir: Along})
	return plate, boss
}

func TestStackedUnionRecordsSlabsAndFloor(t *testing.T) {
	t.Run("boss on plate", func(t *testing.T) {
		plate, boss := internalBossOnPlate(t, 0, 10, 15)
		sp, ok, err := tryStackedUnion(t.Context(), plate, boss)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, sp.slabs, 2)
		require.Equal(t, [4]float64{0, 10, 10, 25},
			[4]float64{sp.slabs[0].z0, sp.slabs[0].z1, sp.slabs[1].z0, sp.slabs[1].z1})
		require.Zero(t, sp.sectionDelta)
		require.Len(t, sp.interfaces, 1)
		require.Empty(t, sp.interfaces[0].upperExposed)
		require.Len(t, sp.interfaces[0].lowerExposed, 1)
		floor := sp.interfaces[0].lowerExposed[0]
		plateOuter := plate.payload.(prismPayload).profile.Outer
		same, err := loopRecordsEqual(nil, floor.Outer, plateOuter)
		require.NoError(t, err)
		require.True(t, same, "the floor's outer is the plate's own outer record")
		bossHole, err := reverseLoopRecordContext(t.Context(), boss.payload.(prismPayload).profile.Outer)
		require.NoError(t, err)
		require.Equal(t, []LoopRecord{bossHole}, floor.Holes, "the floor's one hole is the boss outline, reversed")
	})
	t.Run("rooted boss", func(t *testing.T) {
		plate, boss := internalBossOnPlate(t, 0, 5, 20)
		sp, ok, err := tryStackedUnion(t.Context(), plate, boss)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, sp.slabs, 3)
		plateProfile := plate.payload.(prismPayload).profile
		require.Equal(t, plateProfile, sp.slabs[1].regions[0],
			"the slab both operands reach takes the containing plate's record verbatim")
		require.Empty(t, sp.interfaces[0].lowerExposed)
		require.Empty(t, sp.interfaces[0].upperExposed)
		require.Len(t, sp.interfaces[1].lowerExposed, 1)
		require.Zero(t, sp.sectionDelta)
	})
}

func TestStackedUnionInterfaceReportsSplitBoundary(t *testing.T) {
	plate, crossing := internalBossOnPlate(t, 18, 10, 15)
	base := plate.payload.(prismPayload)
	m, err := stackedUnionInterfaceMatch(t.Context(), proofbound.NewWorkBudget(t.Context()), base, base.profile,
		crossing.payload.(prismPayload).profile)
	require.NoError(t, err)
	require.Equal(t, stackedNestNone, m.nest)
	require.True(t, m.split, "the interface scene reports a Partial edge where the boss crosses the outline")

	// prism-boolean §4.4: an unresolved topology is a silent miss.
	_, ok, err := tryStackedUnion(t.Context(), plate, crossing)
	require.NoError(t, err)
	require.False(t, ok)
}

// A B level that is no float rounds once, and the rounding is charged into
// that level's displacement (general-boolean §3 A1).
func TestStackedUnionLevelChargesExactOffsetSum(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 1)
	boss := internalCircleBody(t, doc, 0, 5, 0.1, Symmetric{D: units.Millimeters(0.3)})
	sp, ok, err := tryStackedUnion(t.Context(), plate, boss)
	require.NoError(t, err)
	require.True(t, ok)
	top := new(big.Rat).Add(proofarith.FloatRat(0.1), proofarith.FloatRat(0.3))
	held, _ := top.Float64()
	charge := proofarith.RationalFloatError(top, held)
	require.Positive(t, charge, "the fixture's top level must not be a float")
	found := false
	for k := 0; k+1 < len(sp.slabs); k++ {
		if sp.slabs[k].z1 != held {
			continue
		}
		found = true
		require.Equal(t, charge, sp.slabs[k].z1Delta)
		require.Equal(t, charge, sp.slabs[k+1].z0Delta)
	}
	require.True(t, found, "the boss's top level splits the plate")
	require.NoError(t, falsifyStackedPayload(t.Context(), sp))
}

// A boss moved in its plane re-expresses into the plate's frame, so the
// union carries that rounding as a section displacement.
func TestStackedUnionPlacedBossChargesSectionDisplacement(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalCircleBody(t, doc, -0.1, 5, 0, Distance{D: units.Millimeters(25), Dir: Along})
	move, err := r3.Translation(r3.NewVec(0.1, 0, 0))
	require.NoError(t, err)
	boss, err = boss.Placed(t.Context(), move)
	require.NoError(t, err)
	sp, ok, err := tryStackedUnion(t.Context(), plate, boss)
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, sp.sectionDelta)
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, Approximate, volume.Exactness)
	want := 16000 + 25*15*math.Pi
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
}

func TestStackedUnionPayloadAuditRejectsBrokenInterfaces(t *testing.T) {
	plate, boss := internalBossOnPlate(t, 0, 10, 15)
	base, ok, err := tryStackedUnion(t.Context(), plate, boss)
	require.NoError(t, err)
	require.True(t, ok)
	require.NoError(t, falsifyStackedPayload(t.Context(), base))
	bossOuter := boss.payload.(prismPayload).profile.Outer
	cases := []struct {
		name   string
		change func(*stackedPrismPayload)
		want   error
	}{
		{"exposure on the wrong side", func(sp *stackedPrismPayload) {
			sp.interfaces[0].upperExposed, sp.interfaces[0].lowerExposed = sp.interfaces[0].lowerExposed, nil
		}, ErrDegenerate},
		{"exposure on both sides", func(sp *stackedPrismPayload) {
			sp.interfaces[0].upperExposed = sp.interfaces[0].lowerExposed
		}, ErrDegenerate},
		{"no exposure", func(sp *stackedPrismPayload) {
			sp.interfaces[0].lowerExposed = nil
		}, ErrDegenerate},
		{"exposure without its hole", func(sp *stackedPrismPayload) {
			sp.interfaces[0].lowerExposed = []ProfileRecord{{Outer: sp.slabs[0].regions[0].Outer}}
		}, ErrDegenerate},
		{"holed region under a changed outer", func(sp *stackedPrismPayload) {
			sp.slabs[1].regions[0].Holes = []LoopRecord{bossOuter}
		}, ErrUnsupported},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sp := cloneStackedForAudit(base)
			tc.change(&sp)
			require.ErrorIs(t, falsifyStackedPayload(t.Context(), sp), tc.want)
		})
	}
}

// The tolerance gate's witnesses are body points: the plate's outer swept to
// the boss's top would hold corners at z = 25 the body never reaches.
func TestStackedUnionGateDiameterStaysOnTheBody(t *testing.T) {
	plate, boss := internalBossOnPlate(t, 0, 10, 15)
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	d, ok, err := bodyGateDiameter(t.Context(), got)
	require.NoError(t, err)
	require.True(t, ok)
	// The plate's body diagonal, √(40² + 40² + 10²), is the body's diameter.
	require.LessOrEqual(t, d, math.Sqrt(3300))
	require.Greater(t, d, math.Sqrt(3300)-1e-9)
}

func TestStackedUnionExtentReadsEveryRun(t *testing.T) {
	doc := New()
	shaft := internalDiscBody(t, doc, 5, 40)
	flange := internalCircleBody(t, doc, 0, 15, 40, Distance{D: units.Millimeters(5), Dir: Along})
	sp, ok, err := tryStackedUnion(t.Context(), shaft, flange)
	require.NoError(t, err)
	require.True(t, ok)
	lo, hi, bound, err := sp.extentAlong(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(hi-15), bound)
	require.LessOrEqual(t, math.Abs(lo+15), bound)
}

// A mirror join re-derives interfaces from exclusive holes, so it refuses a
// union-built stack (mirror J1) before reading any region.
func TestStackedUnionMirrorJoinRefuses(t *testing.T) {
	plate, boss := internalBossOnPlate(t, 0, 10, 15)
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	_, err = got.Mirrored(t.Context(), MirrorFace{Body: got, Face: Faces(Planar(), Facing(r3.NewVec(-1, 0, 0)))}, WithJoin())
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "union-built stack")
}
