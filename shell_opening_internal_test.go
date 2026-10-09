package decad

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// testSideOpeningRegions adapts prism fixtures to the record-only section input.
func testSideOpeningRegions(budget *proofbound.WorkBudget, pp prismPayload, sides map[int]struct{},
	keptCaps int, sense float64, thickness units.Value, held float64) (prismshell.Section, error) {
	return prismshell.SideOpeningRegions(budget, prismshell.SideOpeningInput{
		Profile: pp.profile, Height: pp.z1 - pp.z0, Sides: sides,
		KeptCaps: keptCaps, Sense: sense, Thickness: thickness, HeldThickness: held,
		ThicknessDelta: 0, Tolerance: shellTol,
	}, auditOffsetSectionBudget)
}

// internalSideSegments names the outer-loop segments of pp whose recorded
// line lies on the section line u = at (alongU false) or v = at (alongU
// true).
func internalSideSegments(t *testing.T, pp prismPayload, alongU bool, at float64) map[int]struct{} {
	t.Helper()
	out := map[int]struct{}{}
	for i, seg := range pp.profile.Outer.Segments {
		l, ok := seg.(lineSeg)
		require.True(t, ok)
		if alongU && l.Start.V == at && l.End.V == at || !alongU && l.Start.U == at && l.End.U == at {
			out[i] = struct{}{}
		}
	}
	require.NotEmpty(t, out)
	return out
}

// internalLoopPoints lists a line loop's start points in walk order.
func internalLoopPoints(t *testing.T, loop loopRecord) []Point2 {
	t.Helper()
	out := make([]Point2, len(loop.Segments))
	for i, seg := range loop.Segments {
		l, ok := seg.(lineSeg)
		require.True(t, ok, "segment %d is %T", i, seg)
		out[i] = l.Start
		require.Equal(t, l.End, loop.Segments[(i+1)%len(loop.Segments)].(lineSeg).Start, "the loop closes at segment %d", i)
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
		sec, err := testSideOpeningRegions(budget, pp, internalSideSegments(t, pp, false, 0), 2, 1, units.Millimeters(2), 2)
		require.NoError(t, err)
		require.Equal(t, []Point2{
			pt(0, 0), pt(40, 0), pt(40, 20), pt(0, 20), pt(0, 18),
			pt(38, 18), pt(38, 2), pt(0, 2),
		}, internalLoopPoints(t, sec.Wall.Outer))
		require.Equal(t, []Point2{pt(0, 2), pt(38, 2), pt(38, 18), pt(0, 18)}, internalLoopPoints(t, sec.Cavity.Outer))
		require.Equal(t, pp.profile, sec.Caps)
		require.Empty(t, sec.Corners, "both cuts run forward along the removed face")
		require.Zero(t, sec.Delta)
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
		sec, err := testSideOpeningRegions(budget, pp, sides, 2, -1, units.Millimeters(2), 2)
		require.NoError(t, err)
		require.Equal(t, []Point2{pt(0, -2), pt(40, -2), pt(40, 0), pt(0, 0)}, internalLoopPoints(t, sec.Wall.Outer))
		require.Equal(t, []Point2{pt(0, -2), pt(40, -2), pt(40, 20), pt(0, 20)}, internalLoopPoints(t, sec.Caps.Outer))
		require.Equal(t, pp.profile, sec.Cavity)
		require.ElementsMatch(t, []Point2{pt(0, 0), pt(40, 0)}, sec.Corners)
		require.Zero(t, sec.Delta)
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
	sec, err := testSideOpeningRegions(budget, pp, internalSideSegments(t, pp, false, 10), 2, 1, units.Millimeters(2), 2)
	require.NoError(t, err)
	require.Equal(t, []Point2{pt(10, 28), pt(2, 28), pt(2, 2), pt(28, 2), pt(28, 8), pt(10, 8)}, internalLoopPoints(t, sec.Cavity.Outer))
	area, err := loopSignedAreaCB(sec.Cavity.Outer)
	require.NoError(t, err)
	require.Equal(t, 316.0, area)
	require.Equal(t, []Point2{pt(10, 10)}, sec.Corners)
	require.Zero(t, sec.Delta)
}

// TestSideOpeningBrepChargesInexactThickness shells the U-channel with t =
// 0.1 in, a thickness whose millimetre float is not the value it denotes. The
// cavity's cut vertex (0, t) and its floor level z = t each sit off the
// denoted ones by the conversion gap, so every face's section displacement
// and the floor level's displacement must cover that gap, and the volume's
// bound the denoted volume 8000 − (40 − t*)(20 − 2t*)(10 − 2t*). Shown to fail
// with sideOpeningRegions' offset2d.ChainSectionDelta leg zeroed (every face delta 0)
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
	rect := func(u0, v0, u1, v1 float64) profileRecord {
		p := []Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
		var loop loopRecord
		for i := range p {
			loop.Segments = append(loop.Segments, lineSeg{Start: p[i], End: p[(i+1)%4], TStart: 0, TEnd: 1})
		}
		return profileRecord{Outer: loop}
	}
	require.NoError(t, offset2d.RequireAreaIdentity(rect(0, 0, 4, 2), rect(0, 0, 4, 1), rect(0, 1, 4, 2)))
	err := offset2d.RequireAreaIdentity(rect(0, 0, 4, 2), rect(0, 0, 3, 1), rect(0, 1, 4, 2))
	require.True(t, errors.Is(err, ErrUnsupported))
	require.ErrorContains(t, err, "SO5")
}

// internalSideSegmentsOn names the outer-loop segments of pp whose recorded
// line runs from a to b.
func internalSideSegmentsOn(t *testing.T, pp prismPayload, a, b Point2) map[int]struct{} {
	t.Helper()
	out := map[int]struct{}{}
	for i, seg := range pp.profile.Outer.Segments {
		l, ok := seg.(lineSeg)
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
		sec, err := testSideOpeningRegions(budget, pp, internalSideSegmentsOn(t, pp, pt(12, 0), pt(0, 9)), 2, 1, units.Millimeters(3), 3)
		require.NoError(t, err)
		caps := internalLoopPoints(t, sec.Caps.Outer)
		require.Len(t, caps, 5)
		require.Equal(t, []Point2{pt(0, 9), pt(0, 0), pt(12, 0)}, caps[:3])
		internalNear(t, caps[3], rat(8, 1), rat(3, 1), sec.Delta)
		internalNear(t, caps[4], rat(3, 1), rat(27, 4), sec.Delta)
		cavity := internalLoopPoints(t, sec.Cavity.Outer)
		require.Equal(t, []Point2{caps[4], pt(3, 3), caps[3]}, cavity)
		require.ElementsMatch(t, []Point2{pt(12, 0), pt(0, 9)}, sec.Corners)
	})
	t.Run("slanted reflex end", func(t *testing.T) {
		t.Parallel()
		l := internalPolyPrismBodyAtZ(t, New(), [][2]float64{{0, 0}, {30, 0}, {30, 10}, {14, 10}, {10, 30}, {0, 30}}, 0, 10)
		pp := l.payload.(prismPayload)
		budget := proofbound.NewWorkBudget(t.Context())
		sec, err := testSideOpeningRegions(budget, pp, internalSideSegmentsOn(t, pp, pt(14, 10), pt(10, 30)), 2, 1, units.Millimeters(2), 2)
		require.NoError(t, err)
		cavity := internalLoopPoints(t, sec.Cavity.Outer)
		require.Len(t, cavity, 7)
		internalNear(t, cavity[0], rat(52, 5), rat(28, 1), sec.Delta)
		require.Equal(t, []Point2{pt(2, 28), pt(2, 2), pt(28, 2), pt(28, 8)}, cavity[1:5])
		internalNear(t, cavity[5], rat(72, 5), rat(8, 1), sec.Delta)
		require.Equal(t, pt(14, 10), cavity[6], "R' turns at the corner it runs back through")
		caps := internalLoopPoints(t, sec.Caps.Outer)
		require.Equal(t, []Point2{pt(10, 30), pt(0, 30), pt(0, 0), pt(30, 0), pt(30, 10), pt(14, 10), cavity[0]}, caps)
		require.ElementsMatch(t, []Point2{pt(14, 10), pt(10, 30)}, sec.Corners)
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
			sec, err := testSideOpeningRegions(budget, pp, internalSideSegmentsOn(t, pp, f.a, f.b), 2, 1, units.Millimeters(tmm), tmm)
			require.NoError(t, err, "%s at t = %g", f.name, tmm)
			want := f.cavity(proofarith.FloatRat(tmm))
			held := internalLoopPoints(t, sec.Cavity.Outer)
			require.Len(t, held, len(want), "%s at t = %g", f.name, tmm)
			for i, w := range want {
				internalNear(t, held[i], w[0], w[1], sec.Delta)
			}
			if sec.Delta > 0 {
				charged++
			}
		}
	}
	require.Positive(t, charged, "some cut is a float solve whose displacement is charged")
}

// internalDSectionPrism is §9's D section — the semicircle of radius 5 about
// the origin from (0,−5) through (5,0) to (0,5), closed by the chord x = 0 —
// swept 10 along +z. It returns the prism and the indices of the arc's and
// the chord's recorded segments.
func internalDSectionPrism(t *testing.T) (prismPayload, int, int) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	a := s.CreatePoint(0, -5)
	b := s.CreatePoint(0, 5)
	s.CreateArc(o, a, b)
	s.CreateLine(b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	pp := body.payload.(prismPayload)
	arc, chord := -1, -1
	for i, seg := range pp.profile.Outer.Segments {
		switch seg.(type) {
		case arcSeg:
			arc = i
		case lineSeg:
			chord = i
		}
	}
	require.Len(t, pp.profile.Outer.Segments, 2)
	return pp, arc, chord
}

// internalWalkedPoints lists a loop's walked start points in walk order.
func internalWalkedPoints(t *testing.T, loop loopRecord) []Point2 {
	t.Helper()
	out := make([]Point2, len(loop.Segments))
	for i, seg := range loop.Segments {
		from, _, ok := offset2d.WalkedEnds(seg)
		require.True(t, ok, "segment %d is %T", i, seg)
		out[i] = from
	}
	return out
}

// internalArcPieces requires every circular segment of the three regions to
// be a parameter range of the receiver's own arc record — its Center, Start
// and End verbatim, or Start and End swapped (the arc's complement) — so the
// record build keys each on the arc's circle (§4.2).
func internalArcPieces(t *testing.T, own arcSeg, sec prismshell.Section) {
	t.Helper()
	for _, region := range []profileRecord{sec.Wall, sec.Cavity, sec.Caps} {
		for _, seg := range region.Outer.Segments {
			a, ok := seg.(arcSeg)
			if !ok {
				continue
			}
			require.Equal(t, own.Center, a.Center)
			swapped := a.Start == own.End && a.End == own.Start
			require.True(t, swapped || a.Start == own.Start && a.End == own.End, "%+v is a range of %+v or of its complement", a, own)
		}
	}
}

// internalCutWithin requires the point the arc segment seg denotes at the
// parameter t — enclosed over rational intervals — to lie within e of the
// exact cut, component by component: u exactly, and v = sign·√v2.
func internalCutWithin(t *testing.T, seg curveSegment, tCut float64, u *big.Rat, sign int, v2 int64, e float64) {
	t.Helper()
	uIv, vIv, ok := circularbounds.EndpointInterval(circularbounds.RecordSegment(seg), proofarith.FloatRat(tCut))
	require.True(t, ok)
	re := proofarith.FloatRat(e)
	require.GreaterOrEqual(t, uIv.Lo.Cmp(new(big.Rat).Sub(u, re)), 0, "the cut's u reaches down to its denoted end")
	require.LessOrEqual(t, uIv.Hi.Cmp(new(big.Rat).Add(u, re)), 0, "the cut's u reaches up to its denoted end")
	lo, hi := vIv.Lo, vIv.Hi
	if sign < 0 {
		lo, hi = new(big.Rat).Neg(vIv.Hi), new(big.Rat).Neg(vIv.Lo)
	}
	n := big.NewRat(v2, 1)
	// lo ≥ √n − e and hi ≤ √n + e, decided over squares.
	up := new(big.Rat).Add(lo, re)
	require.True(t, up.Sign() > 0 && new(big.Rat).Mul(up, up).Cmp(n) >= 0, "the cut's v reaches down to its denoted end")
	down := new(big.Rat).Sub(hi, re)
	require.True(t, down.Sign() <= 0 || new(big.Rat).Mul(down, down).Cmp(n) <= 0, "the cut's v reaches up to its denoted end")
}

// TestSideOpeningRegionsDSection pins §9's D section, inward. With the chord
// removed at t = 1 the cuts are the exact feet (0, ±4) and C is the radius-4
// arc then the chord's piece between them, with no displacement. With the arc
// removed at t = 3 the offset chord x = 3 meets the arc's circle at the exact
// points (3, ±4), and C is x = 3 then the arc's piece between them; at t = 2
// the cuts (2, ±√21) are float solves, and the section displacement is
// nonzero and covers the held cut's distance from √21. Every piece on the
// arc's circle is a parameter range of the arc's own record, ending at the
// cut's parameter, a float: the point it denotes there lies within the cut
// gap (plus the displacement) of the exact cut, kept caps or not. Shown to
// fail with offset2d.ChainSectionDelta's reach zeroed (the t = 2 displacement then 0)
// and with arcCutGap answering zero (the t = 3 rims' denoted ends then lie
// outside a zero bound).
func TestSideOpeningRegionsDSection(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) Point2 { return Point2{U: u, V: v} }
	pp, arc, chord := internalDSectionPrism(t)
	own := pp.profile.Outer.Segments[arc].(arcSeg)
	regions := func(t *testing.T, removed, keptCaps int, tmm float64) (prismshell.Section, error) {
		t.Helper()
		budget := proofbound.NewWorkBudget(t.Context())
		return testSideOpeningRegions(budget, pp, map[int]struct{}{removed: {}}, keptCaps, 1, units.Millimeters(tmm), tmm)
	}
	// rims are W's two rims: vB → qB at index 1, qA → vA at index 3.
	rims := func(t *testing.T, sec prismshell.Section) (arcSeg, arcSeg) {
		t.Helper()
		require.Len(t, sec.Wall.Outer.Segments, 4)
		b, okB := sec.Wall.Outer.Segments[1].(arcSeg)
		a, okA := sec.Wall.Outer.Segments[3].(arcSeg)
		require.True(t, okB && okA, "the rims are pieces of the removed arc")
		return b, a
	}
	t.Run("chord removed", func(t *testing.T) {
		t.Parallel()
		sec, err := regions(t, chord, 2, 1)
		require.NoError(t, err)
		require.Equal(t, []Point2{pt(0, -4), pt(0, 4)}, internalWalkedPoints(t, sec.Cavity.Outer))
		require.Equal(t, []Point2{pt(0, -5), pt(0, 5), pt(0, 4), pt(0, -4)}, internalWalkedPoints(t, sec.Wall.Outer))
		require.Empty(t, sec.Corners)
		require.Zero(t, sec.Delta)
		require.Zero(t, sec.CutGap, "the rims are lines")
	})
	t.Run("arc removed", func(t *testing.T) {
		t.Parallel()
		sec, err := regions(t, arc, 2, 3)
		require.NoError(t, err)
		require.Equal(t, lineSeg{Start: pt(3, 4), End: pt(3, -4), TStart: 0, TEnd: 1}, sec.Cavity.Outer.Segments[0], "C opens with x = 3")
		require.Equal(t, []Point2{pt(0, 5), pt(0, -5)}, internalWalkedPoints(t, sec.Wall.Outer)[:2])
		internalArcPieces(t, own, sec)
		b, a := rims(t, sec)
		require.Equal(t, 0.0, b.TStart, "the rim at (0, −5) starts at the arc's own start")
		require.Equal(t, 1.0, a.TEnd, "the rim at (0, 5) ends at the arc's own end")
		require.ElementsMatch(t, []Point2{pt(0, -5), pt(0, 5)}, sec.Corners, "both end vertices on the removed arc are marked")
		require.Zero(t, sec.Delta, "the exact cuts enclose to their held floats")
		require.Positive(t, sec.CutGap, "the cut parameter is a float")
		internalCutWithin(t, b, b.TEnd, big.NewRat(3, 1), -1, 16, sec.CutGap)
		internalCutWithin(t, a, a.TStart, big.NewRat(3, 1), 1, 16, sec.CutGap)
	})
	t.Run("arc removed at a float cut", func(t *testing.T) {
		t.Parallel()
		for _, keptCaps := range []int{0, 2} {
			sec, err := regions(t, arc, keptCaps, 2)
			require.NoError(t, err)
			q := internalWalkedPoints(t, sec.Cavity.Outer)[0]
			require.Equal(t, 2.0, q.U)
			require.Positive(t, sec.Delta)
			lo := new(big.Rat).Sub(proofarith.FloatRat(q.V), proofarith.FloatRat(sec.Delta))
			hi := new(big.Rat).Add(proofarith.FloatRat(q.V), proofarith.FloatRat(sec.Delta))
			require.Positive(t, lo.Sign())
			require.LessOrEqual(t, new(big.Rat).Mul(lo, lo).Cmp(big.NewRat(21, 1)), 0, "the displacement reaches down to √21")
			require.GreaterOrEqual(t, new(big.Rat).Mul(hi, hi).Cmp(big.NewRat(21, 1)), 0, "the displacement reaches up to √21")
			internalArcPieces(t, own, sec)
			b, a := rims(t, sec)
			e := proofbound.AbsSumUpper(sec.Delta, sec.CutGap)
			internalCutWithin(t, b, b.TEnd, big.NewRat(2, 1), -1, 21, e)
			internalCutWithin(t, a, a.TStart, big.NewRat(2, 1), 1, 21, e)
		}
	})
}

// TestSideOpeningRegionsArcArcCut pins an arc–arc end corner's float cut. The
// section is the concave arc of radius 13 about the origin from (0,13) to
// (13,0), the arc of radius 17 about (−2,8) from (13,0) to (13,16), then
// x = 13, y = 20 and x = 0 back to (0,13); every walk but the concave arc is
// removed. At t = 2 the kept arc's offset, radius 15, meets the removed arc's
// circle where x = 4y − 1 and 17y² − 8y − 224 = 0, at y = 4(1 + √239)/17: a
// float solve, so with both caps removed the section displacement is nonzero
// and covers the held cut's distance from it. At t = 4 the cut (15,8) is
// exact and both caps may stay. Shown to fail
// with chainEndReach's opening ends returning zero (the t = 2 displacement
// then 0).
func TestSideOpeningRegionsArcArcCut(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := map[string]*sketch.Point{}
	for name, p := range map[string][2]float64{"o": {0, 0}, "c": {-2, 8}, "a": {0, 13}, "b": {13, 0}, "e": {13, 16}, "f": {13, 20}, "g": {0, 20}} {
		pts[name] = s.CreatePoint(p[0], p[1])
		s.Fix(pts[name])
	}
	s.CreateArc(pts["o"], pts["b"], pts["a"])
	s.CreateArc(pts["c"], pts["b"], pts["e"])
	s.CreateLine(pts["e"], pts["f"])
	s.CreateLine(pts["f"], pts["g"])
	s.CreateLine(pts["g"], pts["a"])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	pp := body.payload.(prismPayload)
	removed := map[int]struct{}{}
	for i, seg := range pp.profile.Outer.Segments {
		if a, ok := seg.(arcSeg); ok && a.Center == (Point2{}) {
			continue
		}
		removed[i] = struct{}{}
	}
	require.Len(t, removed, 4)
	regions := func(t *testing.T, keptCaps int, tmm float64) prismshell.Section {
		t.Helper()
		sec, err := testSideOpeningRegions(proofbound.NewWorkBudget(t.Context()), pp, removed, keptCaps, 1, units.Millimeters(tmm), tmm)
		require.NoError(t, err)
		return sec
	}

	exact := regions(t, 2, 4)
	require.Equal(t, []Point2{{U: 0, V: 17}, {U: 15, V: 8}, {U: 13, V: 16}, {U: 13, V: 20}, {U: 0, V: 20}},
		internalWalkedPoints(t, exact.Cavity.Outer))

	sec := regions(t, 0, 2)
	q := internalWalkedPoints(t, sec.Cavity.Outer)[1]
	require.Positive(t, sec.Delta)
	// 17y/4 − 1 = √239 at the exact cut.
	root := func(y *big.Rat) *big.Rat {
		v := new(big.Rat).Mul(y, big.NewRat(17, 4))
		v.Sub(v, big.NewRat(1, 1))
		return v.Mul(v, v)
	}
	lo := new(big.Rat).Sub(proofarith.FloatRat(q.V), proofarith.FloatRat(sec.Delta))
	hi := new(big.Rat).Add(proofarith.FloatRat(q.V), proofarith.FloatRat(sec.Delta))
	require.Positive(t, new(big.Rat).Sub(new(big.Rat).Mul(lo, big.NewRat(17, 4)), big.NewRat(1, 1)).Sign())
	require.LessOrEqual(t, root(lo).Cmp(big.NewRat(239, 1)), 0, "the displacement reaches down to the cut")
	require.GreaterOrEqual(t, root(hi).Cmp(big.NewRat(239, 1)), 0, "the displacement reaches up to the cut")
}

// internalArcSectionPrism sweeps 10 along +z the section that build draws on
// a fixed XY sketch, and returns the prism and the indices of its outer-loop
// segments other than the concave arc of radius 13 about the origin.
func internalArcSectionPrism(t *testing.T, build func(s *sketch.Sketch)) (prismPayload, map[int]struct{}) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	build(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	pp := body.payload.(prismPayload)
	removed := map[int]struct{}{}
	for i, seg := range pp.profile.Outer.Segments {
		if a, ok := seg.(arcSeg); ok && a.Center == (Point2{}) {
			continue
		}
		removed[i] = struct{}{}
	}
	return pp, removed
}

// internalFixedArcSection draws, on fixed points, the arc about center from
// start counter-clockwise to end and the lines start → rest[0] → … → end.
func internalFixedArcSection(center, start, end [2]float64, rest ...[2]float64) func(s *sketch.Sketch) {
	return func(s *sketch.Sketch) {
		fix := func(p [2]float64) *sketch.Point {
			pt := s.CreatePoint(p[0], p[1])
			s.Fix(pt)
			return pt
		}
		c, a, b := fix(center), fix(start), fix(end)
		s.CreateArc(c, a, b)
		prev := a
		for _, p := range rest {
			next := fix(p)
			s.CreateLine(prev, next)
			prev = next
		}
		s.CreateLine(prev, b)
	}
}

// TestSideOpeningRegionsArcExtension pins a kept arc whose offset runs past
// its own end to the rim cut (docs/shell-opening-design.md §2.4), outward,
// every walk but the concave arc of radius 13 about the origin removed:
//
//   - the arc from (0,13) to (5,12), then x = 5, y = 20 and x = 0: at
//     t = 2.375 the offset arc of radius 10.625 runs from the exact cut
//     (0, 10.625) past the arc's end angle to the exact cut (5, 9.375) on
//     x = 5, so the outer region O walks K' from (0, 10.625) and then R'
//     from (5, 9.375), and the displacement is zero;
//   - the notch, whose arc runs to (13,0) against the arc of radius 17 about
//     (−2,8): at t = 4 the offset arc of radius 9 runs past (13,0) to the
//     float cut q with 17·q.v + 140 = 2√38, so with both caps removed the
//     displacement is nonzero and covers the held cut's distance from it.
//
// Shown to fail with chainSectionDelta's reach zeroed (the t = 4 displacement
// then 0), and with offsetOpenChain reading every walk through WalkConsumed
// (both fixtures then S11a).
func TestSideOpeningRegionsArcExtension(t *testing.T) {
	t.Parallel()
	regions := func(t *testing.T, pp prismPayload, removed map[int]struct{}, keptCaps int, tmm float64) prismshell.Section {
		t.Helper()
		sec, err := testSideOpeningRegions(proofbound.NewWorkBudget(t.Context()), pp, removed, keptCaps, -1, units.Millimeters(tmm), tmm)
		require.NoError(t, err)
		return sec
	}
	// offsetArc is where O's first segment, K', an arc about the origin,
	// starts and ends.
	offsetArc := func(t *testing.T, sec prismshell.Section) (Point2, Point2) {
		t.Helper()
		arc, ok := sec.Caps.Outer.Segments[0].(arcSeg)
		require.True(t, ok, "O opens with the offset arc")
		require.Equal(t, Point2{}, arc.Center)
		from, to, ok := offset2d.WalkedEnds(arc)
		require.True(t, ok)
		return from, to
	}

	t.Run("an exact cut on a vertical end face", func(t *testing.T) {
		t.Parallel()
		pp, removed := internalArcSectionPrism(t, internalFixedArcSection([2]float64{0, 0}, [2]float64{5, 12}, [2]float64{0, 13},
			[2]float64{5, 20}, [2]float64{0, 20}))
		require.Len(t, removed, 3)
		sec := regions(t, pp, removed, 2, 2.375)
		from, to := offsetArc(t, sec)
		require.Equal(t, Point2{U: 0, V: 10.625}, from)
		require.Equal(t, Point2{U: 5, V: 9.375}, to)
		// 9.375/5 < 12/5: the cut lies past the arc's own end angle.
		require.Equal(t, Point2{U: 5, V: 9.375}, internalWalkedPoints(t, sec.Caps.Outer)[1], "R' starts at the cut")
		require.Zero(t, sec.Delta)
		require.Zero(t, sec.CutGap, "the rims are lines")
	})
	t.Run("a float cut on the notch's removed arc", func(t *testing.T) {
		t.Parallel()
		pp, removed := internalArcSectionPrism(t, func(s *sketch.Sketch) {
			pts := map[string]*sketch.Point{}
			for name, p := range map[string][2]float64{"o": {0, 0}, "c": {-2, 8}, "a": {0, 13}, "b": {13, 0}, "e": {13, 16}, "f": {13, 20}, "g": {0, 20}} {
				pts[name] = s.CreatePoint(p[0], p[1])
				s.Fix(pts[name])
			}
			s.CreateArc(pts["o"], pts["b"], pts["a"])
			s.CreateArc(pts["c"], pts["b"], pts["e"])
			s.CreateLine(pts["e"], pts["f"])
			s.CreateLine(pts["f"], pts["g"])
			s.CreateLine(pts["g"], pts["a"])
		})
		require.Len(t, removed, 4)
		sec := regions(t, pp, removed, 0, 4)
		from, q := offsetArc(t, sec)
		require.Equal(t, Point2{U: 0, V: 9}, from)
		require.Negative(t, q.V, "the offset arc runs past (13, 0)")
		require.Positive(t, sec.Delta)
		require.Positive(t, sec.CutGap, "the rim is a range of the removed arc's complement")
		// 17v + 140 = 2√38 at the exact cut, so (17v + 140)² = 152.
		root := func(v *big.Rat) *big.Rat {
			x := new(big.Rat).Mul(v, big.NewRat(17, 1))
			x.Add(x, big.NewRat(140, 1))
			return x.Mul(x, x)
		}
		lo := new(big.Rat).Sub(proofarith.FloatRat(q.V), proofarith.FloatRat(sec.Delta))
		hi := new(big.Rat).Add(proofarith.FloatRat(q.V), proofarith.FloatRat(sec.Delta))
		require.Positive(t, new(big.Rat).Add(new(big.Rat).Mul(lo, big.NewRat(17, 1)), big.NewRat(140, 1)).Sign())
		require.LessOrEqual(t, root(lo).Cmp(big.NewRat(152, 1)), 0, "the displacement reaches down to the cut")
		require.GreaterOrEqual(t, root(hi).Cmp(big.NewRat(152, 1)), 0, "the displacement reaches up to the cut")
	})
}
