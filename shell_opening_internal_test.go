package decad

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalSideSegments names the outer-loop segments of pp whose recorded
// line lies on the section line u = at (alongU false) or v = at (alongU
// true).
func internalSideSegments(t *testing.T, pp prismPayload, alongU bool, at float64) map[int]struct{} {
	t.Helper()
	out := map[int]struct{}{}
	for i, seg := range pp.profile.Outer.Segments {
		l, ok := seg.(LineSeg)
		require.True(t, ok)
		if alongU && l.Start.V == at && l.End.V == at || !alongU && l.Start.U == at && l.End.U == at {
			out[i] = struct{}{}
		}
	}
	require.NotEmpty(t, out)
	return out
}

// internalLoopPoints lists a line loop's start points in walk order.
func internalLoopPoints(t *testing.T, loop LoopRecord) []Point2 {
	t.Helper()
	out := make([]Point2, len(loop.Segments))
	for i, seg := range loop.Segments {
		l, ok := seg.(LineSeg)
		require.True(t, ok, "segment %d is %T", i, seg)
		out[i] = l.Start
		require.Equal(t, l.End, loop.Segments[(i+1)%len(loop.Segments)].(LineSeg).Start, "the loop closes at segment %d", i)
	}
	return out
}

// TestSideOpeningRegionsUChannel pins docs/shell-opening-design.md §3's three
// regions for the U-channel's x = 0 face removed at t = 2. Inward W walks K,
// the rim (0,20)→(0,18), K' backward and the rim (0,2)→(0,0); C is K' then
// R' = (0,18)→(0,2). Outward with the y = 20, x = 40 and x = 0 faces removed
// the one kept wall y = 0 cuts backward at both ends, to (0,−2) and (40,−2),
// and both end vertices are marked. Every cut is an exact level pair, so the
// displacement is zero.
func TestSideOpeningRegionsUChannel(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) Point2 { return Point2{U: u, V: v} }
	t.Run("inward", func(t *testing.T) {
		t.Parallel()
		box := internalBoxBody(t, New(), 0, 0, 40, 20, 10)
		pp := box.payload.(prismPayload)
		budget := proofbound.NewWorkBudget(t.Context())
		sec, err := sideOpeningRegions(budget, pp, internalSideSegments(t, pp, false, 0), 2, 1, units.Millimeters(2), 2, 0)
		require.NoError(t, err)
		require.Equal(t, []Point2{
			pt(0, 0), pt(40, 0), pt(40, 20), pt(0, 20), pt(0, 18),
			pt(38, 18), pt(38, 2), pt(0, 2),
		}, internalLoopPoints(t, sec.wall.Outer))
		require.Equal(t, []Point2{pt(0, 2), pt(38, 2), pt(38, 18), pt(0, 18)}, internalLoopPoints(t, sec.cavity.Outer))
		require.Equal(t, pp.profile, sec.caps)
		require.Empty(t, sec.corners, "both cuts run forward along the removed face")
		require.Zero(t, sec.delta)
	})
	t.Run("outward, one kept wall", func(t *testing.T) {
		t.Parallel()
		box := internalBoxBody(t, New(), 0, 0, 40, 20, 10)
		pp := box.payload.(prismPayload)
		sides := internalSideSegments(t, pp, false, 0)
		for k := range internalSideSegments(t, pp, false, 40) {
			sides[k] = struct{}{}
		}
		for k := range internalSideSegments(t, pp, true, 20) {
			sides[k] = struct{}{}
		}
		budget := proofbound.NewWorkBudget(t.Context())
		sec, err := sideOpeningRegions(budget, pp, sides, 2, -1, units.Millimeters(2), 2, 0)
		require.NoError(t, err)
		require.Equal(t, []Point2{pt(0, -2), pt(40, -2), pt(40, 0), pt(0, 0)}, internalLoopPoints(t, sec.wall.Outer))
		require.Equal(t, []Point2{pt(0, -2), pt(40, -2), pt(40, 20), pt(0, 20)}, internalLoopPoints(t, sec.caps.Outer))
		require.Equal(t, pp.profile, sec.cavity)
		require.ElementsMatch(t, []Point2{pt(0, 0), pt(40, 0)}, sec.corners)
		require.Zero(t, sec.delta)
	})
}

// TestSideOpeningRegionsLPrism pins §9's L prism with its x = 10 face
// removed, inward at t = 2: the reflex end (10,10) cuts backward at (10,8),
// the convex end (10,30) forward at (10,28), and C is (10,28) (2,28) (2,2)
// (28,2) (28,8) (10,8) with area 316. Only the reflex end is marked.
func TestSideOpeningRegionsLPrism(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) Point2 { return Point2{U: u, V: v} }
	l := internalPolyPrismBodyAtZ(t, New(), [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {10, 30}, {0, 30}}, 0, 10)
	pp := l.payload.(prismPayload)
	budget := proofbound.NewWorkBudget(t.Context())
	sec, err := sideOpeningRegions(budget, pp, internalSideSegments(t, pp, false, 10), 2, 1, units.Millimeters(2), 2, 0)
	require.NoError(t, err)
	require.Equal(t, []Point2{pt(10, 28), pt(2, 28), pt(2, 2), pt(28, 2), pt(28, 8), pt(10, 8)}, internalLoopPoints(t, sec.cavity.Outer))
	area, err := loopSignedAreaCB(sec.cavity.Outer)
	require.NoError(t, err)
	require.Equal(t, 316.0, area)
	require.Equal(t, []Point2{pt(10, 10)}, sec.corners)
	require.Zero(t, sec.delta)
}

// TestSideOpeningBrepChargesInexactThickness shells the U-channel with t =
// 0.1 in, a thickness whose millimetre float is not the value it denotes. The
// cavity's cut vertex (0, t) and its floor level z = t each sit off the
// denoted ones by the conversion gap, so every face's section displacement
// and the floor level's displacement must cover that gap, and the volume's
// bound the denoted volume 8000 − (40 − t*)(20 − 2t*)(10 − 2t*). Shown to fail
// with sideOpeningRegions' chainSectionDelta leg zeroed (every face delta 0)
// and with shellLevel's tDelta term deleted (the floor level's delta 0).
func TestSideOpeningBrepChargesInexactThickness(t *testing.T) {
	t.Parallel()
	thickness := units.Inches(0.1)
	tmm, _, err := extent.MagnitudeInBounded(thickness, units.Length, units.Millimeter, "t")
	require.NoError(t, err)
	exact := extent.ExactConversion(thickness, units.Millimeter)
	gap := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(tmm), exact))
	require.Positive(t, gap.Sign(), "0.1 in has no exact millimetre float")

	box := internalBoxBody(t, New(), 0, 0, 40, 20, 10)
	opening := Faces(Facing(r3.NewVec(-1, 0, 0)))
	body, err := box.Shell(t.Context(), opening, thickness)
	require.NoError(t, err)
	bp, ok := body.payload.(brepPayload)
	require.True(t, ok, "got %T", body.payload)
	require.Nil(t, bp.stack)
	covers := func(bound float64) bool { return proofarith.FloatRat(bound).Cmp(gap) >= 0 }
	floors := 0
	for _, f := range bp.faces {
		require.True(t, covers(f.delta), "face %s's delta %g covers the cut's gap", f.role, f.delta)
		if f.planar() && f.frame.N() == r3.NewVec(0, 0, 1) && f.z0 == tmm {
			floors++
			require.True(t, covers(f.z0Delta), "the floor level's delta %g covers the gap", f.z0Delta)
		}
	}
	require.Equal(t, 1, floors)

	// V* = 8000 − (40 − t*)(20 − 2t*)(10 − 2t*), read exactly.
	two := big.NewRat(2, 1)
	cavity := new(big.Rat).Sub(big.NewRat(40, 1), exact)
	cavity.Mul(cavity, new(big.Rat).Sub(big.NewRat(20, 1), new(big.Rat).Mul(two, exact)))
	cavity.Mul(cavity, new(big.Rat).Sub(big.NewRat(10, 1), new(big.Rat).Mul(two, exact)))
	want := new(big.Rat).Sub(big.NewRat(8000, 1), cavity)
	got := new(big.Rat).Sub(proofarith.FloatRat(body.volume.Value.Base()), want)
	require.LessOrEqual(t, got.Abs(got).Cmp(proofarith.FloatRat(body.volume.Bound.Base())), 0)
}

// TestRequireAreaIdentityRefuses feeds §4.7's area identity regions that do
// not tile: a wall section one unit short of P less C is SO5.
func TestRequireAreaIdentityRefuses(t *testing.T) {
	t.Parallel()
	rect := func(u0, v0, u1, v1 float64) ProfileRecord {
		p := []Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
		var loop LoopRecord
		for i := range p {
			loop.Segments = append(loop.Segments, LineSeg{Start: p[i], End: p[(i+1)%4], TStart: 0, TEnd: 1})
		}
		return ProfileRecord{Outer: loop}
	}
	require.NoError(t, requireAreaIdentity(rect(0, 0, 4, 2), rect(0, 0, 4, 1), rect(0, 1, 4, 2)))
	err := requireAreaIdentity(rect(0, 0, 4, 2), rect(0, 0, 3, 1), rect(0, 1, 4, 2))
	require.True(t, errors.Is(err, ErrUnsupported))
	require.ErrorContains(t, err, "SO5")
}
