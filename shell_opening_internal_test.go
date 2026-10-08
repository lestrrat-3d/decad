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

// internalSideSegmentsOn names the outer-loop segments of pp whose recorded
// line runs from a to b.
func internalSideSegmentsOn(t *testing.T, pp prismPayload, a, b Point2) map[int]struct{} {
	t.Helper()
	out := map[int]struct{}{}
	for i, seg := range pp.profile.Outer.Segments {
		l, ok := seg.(LineSeg)
		require.True(t, ok)
		if l.Start == a && l.End == b {
			out[i] = struct{}{}
		}
	}
	require.Len(t, out, 1)
	return out
}

// internalNear asserts the held point p lies within bound of the exact point
// (u, v), compared exactly.
func internalNear(t *testing.T, p Point2, u, v *big.Rat, bound float64) {
	t.Helper()
	du := new(big.Rat).Sub(proofarith.FloatRat(p.U), u)
	dv := new(big.Rat).Sub(proofarith.FloatRat(p.V), v)
	d2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	b := proofarith.FloatRat(bound)
	require.LessOrEqual(t, d2.Cmp(new(big.Rat).Mul(b, b)), 0, "held %v lies off (%s, %s) by more than %g", p, u.FloatString(20), v.FloatString(20), bound)
}

// TestSideOpeningRegionsOblique pins docs/shell-opening-design.md §3's regions
// where the removed face is oblique (§4.2). The triangle (0,0) (12,0) (0,9)
// without its hypotenuse at t = 3 cuts forward at both ends, so the cap
// region states the hypotenuse as three collinear pieces (12,0) → qB → qA →
// (0,9) with qB near (8, 3) and qA near (3, 6.75), and both end vertices are
// marked. The slanted reflex section (0,0) (30,0) (30,10) (14,10) (10,30)
// (0,30) without (14,10)→(10,30) at t = 2 cuts backward at (14.4, 8), so R'
// runs from that cut through the corner (14,10) as two pieces, and forward at
// (10.4, 28), which splits the cap region's walk there.
func TestSideOpeningRegionsOblique(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) Point2 { return Point2{U: u, V: v} }
	rat := func(n, d int64) *big.Rat { return big.NewRat(n, d) }
	t.Run("oblique removed face", func(t *testing.T) {
		t.Parallel()
		tri := internalPolyPrismBodyAtZ(t, New(), [][2]float64{{0, 0}, {12, 0}, {0, 9}}, 0, 10)
		pp := tri.payload.(prismPayload)
		budget := proofbound.NewWorkBudget(t.Context())
		sec, err := sideOpeningRegions(budget, pp, internalSideSegmentsOn(t, pp, pt(12, 0), pt(0, 9)), 2, 1, units.Millimeters(3), 3, 0)
		require.NoError(t, err)
		caps := internalLoopPoints(t, sec.caps.Outer)
		require.Len(t, caps, 5)
		require.Equal(t, []Point2{pt(0, 9), pt(0, 0), pt(12, 0)}, caps[:3])
		internalNear(t, caps[3], rat(8, 1), rat(3, 1), sec.delta)
		internalNear(t, caps[4], rat(3, 1), rat(27, 4), sec.delta)
		cavity := internalLoopPoints(t, sec.cavity.Outer)
		require.Equal(t, []Point2{caps[4], pt(3, 3), caps[3]}, cavity)
		require.ElementsMatch(t, []Point2{pt(12, 0), pt(0, 9)}, sec.corners)
	})
	t.Run("slanted reflex end", func(t *testing.T) {
		t.Parallel()
		l := internalPolyPrismBodyAtZ(t, New(), [][2]float64{{0, 0}, {30, 0}, {30, 10}, {14, 10}, {10, 30}, {0, 30}}, 0, 10)
		pp := l.payload.(prismPayload)
		budget := proofbound.NewWorkBudget(t.Context())
		sec, err := sideOpeningRegions(budget, pp, internalSideSegmentsOn(t, pp, pt(14, 10), pt(10, 30)), 2, 1, units.Millimeters(2), 2, 0)
		require.NoError(t, err)
		cavity := internalLoopPoints(t, sec.cavity.Outer)
		require.Len(t, cavity, 7)
		internalNear(t, cavity[0], rat(52, 5), rat(28, 1), sec.delta)
		require.Equal(t, []Point2{pt(2, 28), pt(2, 2), pt(28, 2), pt(28, 8)}, cavity[1:5])
		internalNear(t, cavity[5], rat(72, 5), rat(8, 1), sec.delta)
		require.Equal(t, pt(14, 10), cavity[6], "R' turns at the corner it runs back through")
		caps := internalLoopPoints(t, sec.caps.Outer)
		require.Equal(t, []Point2{pt(10, 30), pt(0, 30), pt(0, 0), pt(30, 0), pt(30, 10), pt(14, 10), cavity[0]}, caps)
		require.ElementsMatch(t, []Point2{pt(14, 10), pt(10, 30)}, sec.corners)
	})
}

// TestSideOpeningCutReachCoversExactCut is docs/shell-opening-design.md §9's
// record fixture: for the triangle and trapezoid families over a range of t,
// every vertex of the cavity C — the two rim cuts and K's miters — lies within
// the published section displacement of its exact rational closed form, and
// some cut is inexact, so the displacement is charged. Shown to fail with
// chainEndReach's opening ends returning zero (OpeningReach skipped): the
// triangle's cut at (31/3, 0) for t = 1 then lay off by more than a delta of
// zero.
func TestSideOpeningCutReachCoversExactCut(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) Point2 { return Point2{U: u, V: v} }
	type family struct {
		name   string
		pts    [][2]float64
		a, b   Point2
		cavity func(t *big.Rat) [][2]*big.Rat
	}
	// lin is a + b·t over the rationals.
	lin := func(a, b, d int64, t *big.Rat) *big.Rat {
		out := new(big.Rat).Mul(big.NewRat(b, d), t)
		return out.Add(out, big.NewRat(a, d))
	}
	families := []family{
		// The y = 0 leg removed: qA on 3x + 4y = 36 − 5t, the miter (t, 9 − 2t), qB (t, 0).
		{"triangle without its leg", [][2]float64{{0, 0}, {12, 0}, {0, 9}}, pt(0, 0), pt(12, 0), func(t *big.Rat) [][2]*big.Rat {
			return [][2]*big.Rat{{lin(36, -5, 3, t), new(big.Rat)}, {t, lin(9, -2, 1, t)}, {t, new(big.Rat)}}
		}},
		// The hypotenuse removed: qA (t, (36 − 3t)/4), the miter (t, t), qB ((36 − 4t)/3, t).
		{"triangle without its hypotenuse", [][2]float64{{0, 0}, {12, 0}, {0, 9}}, pt(12, 0), pt(0, 9), func(t *big.Rat) [][2]*big.Rat {
			return [][2]*big.Rat{{t, lin(36, -3, 4, t)}, {t, t}, {lin(36, -4, 3, t), t}}
		}},
		// The y = 4 side removed: qA ((12 + 5t)/4, 4), miters (2t, t) and (14 − 2t, t), qB ((44 − 5t)/4, 4).
		{"trapezoid without its top", [][2]float64{{0, 0}, {14, 0}, {11, 4}, {3, 4}}, pt(11, 4), pt(3, 4), func(t *big.Rat) [][2]*big.Rat {
			four := big.NewRat(4, 1)
			return [][2]*big.Rat{{lin(12, 5, 4, t), four}, {lin(0, 2, 1, t), t}, {lin(14, -2, 1, t), t}, {lin(44, -5, 4, t), four}}
		}},
	}
	charged := 0
	for _, f := range families {
		for _, tmm := range []float64{0.5, 1, 1.25, 1.5, 2, 2.5, 3} {
			body := internalPolyPrismBodyAtZ(t, New(), f.pts, 0, 10)
			pp := body.payload.(prismPayload)
			budget := proofbound.NewWorkBudget(t.Context())
			sec, err := sideOpeningRegions(budget, pp, internalSideSegmentsOn(t, pp, f.a, f.b), 2, 1, units.Millimeters(tmm), tmm, 0)
			require.NoError(t, err, "%s at t = %g", f.name, tmm)
			want := f.cavity(proofarith.FloatRat(tmm))
			held := internalLoopPoints(t, sec.cavity.Outer)
			require.Len(t, held, len(want), "%s at t = %g", f.name, tmm)
			for i, w := range want {
				internalNear(t, held[i], w[0], w[1], sec.delta)
			}
			if sec.delta > 0 {
				charged++
			}
		}
	}
	require.Positive(t, charged, "some cut is a float solve whose displacement is charged")
}
