package prismcells

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func rgPolar(r, a float64) sectionrecord.Point2 {
	return sectionrecord.Point2{U: r * math.Cos(a), V: r * math.Sin(a)}
}

func rgLine(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{Start: a, End: b, TEnd: 1}
}

func rgPoly(pts ...sectionrecord.Point2) sectionrecord.LoopRecord {
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		segs[i] = rgLine(p, pts[(i+1)%len(pts)])
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func rgDisc() momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(20), CCW: true, TEnd: 1}}}}
}

// rgDisplaced is a payload whose every datum may sit delta from the section
// it denotes, as a pattern instance's record states.
func rgDisplaced(p momentinput.Profile, delta float64) prismrecord.Payload {
	cr := prismrecord.UniformCarrierRecord(p, delta)
	return prismrecord.Payload{Profile: p, SectionDelta: delta, Carriers: &cr}
}

type rgScene struct {
	budget *proofbound.WorkBudget
	tags   map[sketch.Entity]Origin
	cells  []*sketch.Profile
	recs   []prismrecord.CarrierRecord
	floor  float64
}

func rgArrange(t *testing.T, ps ...prismrecord.Payload) rgScene {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	profiles := make([]momentinput.Profile, len(ps))
	recs := make([]prismrecord.CarrierRecord, len(ps))
	var loops []sectionrecord.LoopRecord
	for i, p := range ps {
		profiles[i] = p.Profile
		cr, ok, err := p.CarrierRecord(budget)
		require.NoError(t, err)
		require.True(t, ok)
		recs[i] = cr
		loops = append(loops, p.Profile.Outer)
		loops = append(loops, p.Profile.Holes...)
	}
	s, tags, _, err := BuildSceneRegions(budget, SceneProfiles(profiles), nil, &Reexpression{Identity: true})
	require.NoError(t, err)
	cells, err := ProfilesContext(t.Context(), s.Profiles)
	require.NoError(t, err)
	cells, resolved, err := SplitRuns(budget, cells)
	require.NoError(t, err)
	require.True(t, resolved)
	entries, err := sectionaudit.EntriesOf(budget, loops)
	require.NoError(t, err)
	floor, err := sectionaudit.ContactFloor(budget, entries)
	require.NoError(t, err)
	return rgScene{budget: budget, tags: tags, cells: cells, recs: recs, floor: floor}
}

func (sc rgScene) read(t *testing.T) (VertexReading, bool) {
	t.Helper()
	vr, ok, err := ReadVertices(sc.budget, sc.tags, sc.cells, sc.recs)
	require.NoError(t, err)
	return vr, ok
}

func (sc rgScene) stable(t *testing.T) (bool, bool) {
	t.Helper()
	vr, ok := sc.read(t)
	if !ok {
		return false, false
	}
	arr, ok, err := NewArrangement(sc.budget, sc.tags, sc.cells, sc.recs, vr)
	require.NoError(t, err)
	if !ok {
		return false, true // an arrangement the audit cannot read is refused
	}
	stable, err := arr.Stable(sc.budget, sc.floor)
	require.NoError(t, err)
	return stable, true
}

func (sc rgScene) ordered(t *testing.T) bool {
	t.Helper()
	vr, ok := sc.read(t)
	require.True(t, ok)
	arr, ok, err := NewArrangement(sc.budget, sc.tags, sc.cells, sc.recs, vr)
	require.NoError(t, err)
	require.True(t, ok)
	ordered, err := arr.Ordered(sc.budget)
	require.NoError(t, err)
	return ordered
}

func rgSquare(x0, y0, x1, y1 float64) momentinput.Profile {
	return momentinput.Profile{Outer: rgPoly(
		sectionrecord.Point2{U: x0, V: y0}, sectionrecord.Point2{U: x1, V: y0},
		sectionrecord.Point2{U: x1, V: y1}, sectionrecord.Point2{U: x0, V: y1})}
}

// Lemma T (§6.1): the distance L bounds where the denoted carriers cross.
func TestLemmaTDistance(t *testing.T) {
	t.Parallel()
	crossing := [2]*big.Rat{big.NewRat(0, 1), big.NewRat(0, 1)}
	horizontal := rgLine(sectionrecord.Point2{U: -10}, sectionrecord.Point2{U: 10})
	vertical := rgLine(sectionrecord.Point2{V: -10}, sectionrecord.Point2{V: 10})

	t.Run("two lines", func(t *testing.T) {
		// q_i = err + h_i with s = 1: L = (q1 + q2)/s + min(q1, q2).
		l, ok := lemmaTDistance([2]sectionrecord.CurveSegment{horizontal, vertical}, [2]float64{},
			[2]float64{1e-9, 3e-9}, crossing, 0)
		require.True(t, ok)
		require.InDelta(t, 5e-9, l, 1e-15)
	})
	t.Run("a line and a circle", func(t *testing.T) {
		circle := sectionrecord.CircleSeg{Radius: units.Millimeters(10), CCW: true, TEnd: 1}
		at := [2]*big.Rat{big.NewRat(10, 1), big.NewRat(0, 1)}
		l, ok := lemmaTDistance([2]sectionrecord.CurveSegment{horizontal, circle}, [2]float64{},
			[2]float64{2e-9, 1e-9}, at, 1e-10)
		require.True(t, ok)
		require.InDelta(t, (2.1e-9+1.1e-9)+1.1e-9, l, 1e-14)
	})
	t.Run("a ball wider than half a circle refuses", func(t *testing.T) {
		// 2L > ρ can never be met: the first ball is wider than the circle's
		// half radius, where sinLower reads zero.
		small := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TEnd: 1}
		at := [2]*big.Rat{big.NewRat(1, 1), big.NewRat(0, 1)}
		_, ok := lemmaTDistance([2]sectionrecord.CurveSegment{vertical, small}, [2]float64{},
			[2]float64{0.4, 0.4}, at, 0)
		require.False(t, ok)
	})
	t.Run("a tangent crossing refuses", func(t *testing.T) {
		tangent := rgLine(sectionrecord.Point2{U: -10, V: 1e-12}, sectionrecord.Point2{U: 10, V: -1e-12})
		_, ok := lemmaTDistance([2]sectionrecord.CurveSegment{horizontal, tangent}, [2]float64{},
			[2]float64{1e-9, 1e-9}, crossing, 0)
		require.False(t, ok, "sin of the crossing angle is below the noise floor")
	})
	t.Run("a denoted line turns by 4ε/ℓ", func(t *testing.T) {
		l, ok := lemmaTDistance([2]sectionrecord.CurveSegment{horizontal, vertical}, [2]float64{1e-6, 0},
			[2]float64{1e-9, 1e-9}, crossing, 0)
		require.True(t, ok)
		s := 1 - 4e-6/20
		require.InDelta(t, 2e-9/s+1e-9, l, 1e-13)
	})
}

// §6.3: V1 and the vertex charge.
func TestReadVerticesCharges(t *testing.T) {
	t.Parallel()
	const delta = 5e-13
	// A disc and a displaced tooth crossing its rim twice.
	tooth := rgDisplaced(momentinput.Profile{Outer: rgPoly(rgPolar(18, -0.1), rgPolar(24, -0.05), rgPolar(24, 0.05), rgPolar(18, 0.1))}, delta)
	sc := rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, tooth)
	vr, ok := sc.read(t)
	require.True(t, ok)
	require.Len(t, vr.Vertices, 2)
	for _, v := range vr.Vertices {
		require.Greater(t, v.L, 0.0, "the displaced flank moves the crossing")
		require.LessOrEqual(t, v.L, 10*delta)
		require.GreaterOrEqual(t, v.Delta, v.CutDelta+v.Err+v.L)
		require.Less(t, v.Delta, 1e-10)
	}
	// With both operands undisplaced, Δ_v is the cut allowance alone.
	plain := rgArrange(t, prismrecord.Payload{Profile: rgDisc()},
		prismrecord.Payload{Profile: momentinput.Profile{Outer: tooth.Profile.Outer}})
	vr, ok = plain.read(t)
	require.True(t, ok)
	for _, v := range vr.Vertices {
		require.Zero(t, v.L)
		require.Equal(t, v.CutDelta, v.Delta)
	}
	// V1: a tooth vertex exactly on the disc circle is no transverse pair.
	on := prismrecord.Payload{Profile: momentinput.Profile{Outer: rgPoly(
		sectionrecord.Point2{U: 20}, rgPolar(24, 0.1), rgPolar(24, -0.1))}}
	_, ok = rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, on).read(t)
	require.False(t, ok)
}

// V2 (§6.3): the audit refuses what a displacement could reorder, and passes
// what it cannot.
func TestStabilityAudit(t *testing.T) {
	t.Parallel()
	const delta = 1e-13
	// F is ContactEps times the diagonal, 2.2e-5 for these 10 m squares, and
	// sketch snaps two vertices closer than about F.
	apart := func(gap float64) rgScene {
		return rgArrange(t, prismrecord.Payload{Profile: rgSquare(0, 0, 10000, 10000)},
			rgDisplaced(rgSquare(10000+gap, 2000, 20000, 8000), delta))
	}
	t.Run("an edge far from every other passes", func(t *testing.T) {
		stable, ok := apart(1e-1).stable(t)
		require.True(t, ok)
		require.True(t, stable)
	})
	t.Run("just above F passes", func(t *testing.T) {
		sc := apart(3e-5)
		require.Greater(t, 3e-5, sc.floor)
		stable, ok := sc.stable(t)
		require.True(t, ok)
		require.True(t, stable)
	})
	t.Run("a displaced edge within F of another refuses", func(t *testing.T) {
		sc := apart(2e-5)
		require.Less(t, 2e-5, sc.floor)
		stable, ok := sc.stable(t)
		require.True(t, ok)
		require.False(t, stable)
	})
	t.Run("the wheel-like teeth pass", func(t *testing.T) {
		var ps []prismrecord.Payload
		ps = append(ps, prismrecord.Payload{Profile: rgDisc()})
		for _, psi := range []float64{0, 2, 4.1} {
			ps = append(ps, rgDisplaced(momentinput.Profile{Outer: rgPoly(rgPolar(18, psi-0.1), rgPolar(24, psi-0.05),
				rgPolar(24, psi+0.05), rgPolar(18, psi+0.1))}, delta))
		}
		stable, ok := rgArrange(t, ps...).stable(t)
		require.True(t, ok)
		require.True(t, stable)
	})
	// A displaced block whose left edge leans 1e-3 off the vertical and
	// crosses a square's right side at height `cross`, a shallow angle.
	shallow := func(cross float64) rgScene {
		x := func(y float64) float64 { return 10 + 1e-3*(y-cross) }
		block := momentinput.Profile{Outer: rgPoly(sectionrecord.Point2{U: x(cross - 2), V: cross - 2},
			sectionrecord.Point2{U: 14, V: cross - 2}, sectionrecord.Point2{U: 14, V: cross + 2},
			sectionrecord.Point2{U: x(cross + 2), V: cross + 2})}
		return rgArrange(t, prismrecord.Payload{Profile: rgSquare(0, 0, 10, 10)}, rgDisplaced(block, delta))
	}
	t.Run("a shallow crossing clear of every own vertex passes", func(t *testing.T) {
		stable, ok := shallow(9).stable(t)
		require.True(t, ok)
		require.True(t, stable)
	})
	t.Run("a ball holding an own vertex refuses", func(t *testing.T) {
		// The crossing is 5e-5 from the square's corner, inside the trim
		// length 8F/s0 of 1e-4.
		stable, ok := shallow(10 - 5e-5).stable(t)
		require.True(t, ok)
		require.False(t, stable)
	})
}

// A walked end placed off P̂_v widens the ball B_v (§6.3, w_v): an own read end
// that clears the ball for a walked end at the vertex falls inside it for
// one placed off.
func TestAuditSharedBallGrowsWithWalkedEnd(t *testing.T) {
	t.Parallel()
	const delta = 1e-13
	sc := rgArrange(t, prismrecord.Payload{Profile: rgSquare(0, 0, 10, 10)},
		rgDisplaced(rgSquare(5, 5, 15, 15), delta))
	vr, ok := sc.read(t)
	require.True(t, ok)
	arr, ok, err := NewArrangement(sc.budget, sc.tags, sc.cells, sc.recs, vr)
	require.NoError(t, err)
	require.True(t, ok)
	var a, b *arrangedEdge
	for i := range arr.edges {
		for j := range arr.edges {
			if shared := sharedEndpoints(&arr.edges[i], &arr.edges[j]); len(shared) == 1 &&
				arr.edges[i].origin.Region == 0 && arr.edges[j].origin.Region == 1 {
				a, b = &arr.edges[i], &arr.edges[j]
			}
		}
	}
	require.NotNil(t, a)
	at := sharedEndpoints(a, b)
	pass, err := auditShared(sc.budget, a, b, at, vr, sc.floor)
	require.NoError(t, err)
	require.True(t, pass)
	// Place the walked end of a off the vertex by more than the distance from
	// the vertex to the source segment's own end.
	moved := *a
	if moved.start == at[0] {
		moved.walk.StartU += 20
	} else {
		moved.walk.EndU += 20
	}
	pass, err = auditShared(sc.budget, &moved, b, at, vr, sc.floor)
	require.NoError(t, err)
	require.False(t, pass)
}

// V3 (§5.3): consecutive vertices on one source segment stay in order.
func TestOrderedKeepsVerticesApart(t *testing.T) {
	t.Parallel()
	const delta = 2e-4
	// A triangle whose apex stands out of the rim by `out`: its two flanks
	// cross the rim a few times `out` apart.
	tip := func(out float64) prismrecord.Payload {
		return rgDisplaced(momentinput.Profile{Outer: rgPoly(rgPolar(18, -0.1), rgPolar(18, 0.1),
			sectionrecord.Point2{U: 20 + out})}, delta)
	}
	// An undisplaced rim cut by a displaced tooth: the rim's two vertices are
	// the tooth's width apart.
	require.True(t, rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, tip(0.5)).ordered(t))
	// Flanks whose crossings are within the charge of each other could swap.
	require.False(t, rgArrange(t, prismrecord.Payload{Profile: rgDisc()}, tip(1e-3)).ordered(t))
}

// §6.2: where an edge ends at an own vertex whose two read ends differ, the
// junction path between them is added to the edge's walk allowance c_e.
func TestJunctionClosingLength(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	t.Run("two lines closing through their supports' corner", func(t *testing.T) {
		// The supports cross at (10, 0): each leg is 1, the chord is √2.
		got := junctionClosing(rgLine(pt(0, 0), pt(9, 0)), rgLine(pt(10, -1), pt(10, 5)))
		require.InDelta(t, 2, got, 1e-12)
	})
	t.Run("two lines whose ends meet need no path", func(t *testing.T) {
		require.Zero(t, junctionClosing(rgLine(pt(0, 0), pt(10, 0)), rgLine(pt(10, 0), pt(10, 5))))
	})
	t.Run("parallel lines close through the chord", func(t *testing.T) {
		got := junctionClosing(rgLine(pt(0, 0), pt(10, 0)), rgLine(pt(10, 3), pt(0, 3)))
		require.InDelta(t, 3, got, 1e-12)
	})
}

// A displaced operand whose junction ends differ widens the adjacent edges'
// c_e by the exact closing path, and an own junction that meets adds nothing.
func TestArrangedEdgeAllowanceIncludesJunctionPath(t *testing.T) {
	t.Parallel()
	const delta = 1e-9
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	// The first corner's two read ends differ by gap on the y axis.
	const gap = 1e-8
	open := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		rgLine(pt(0, 0), pt(10, 0)),
		rgLine(pt(10, gap), pt(10, 10)),
		rgLine(pt(10, 10), pt(0, 10)),
		rgLine(pt(0, 10), pt(0, 0)),
	}}}
	closed := rgSquare(0, 0, 10, 10)
	allowances := func(p momentinput.Profile) map[int]float64 {
		sc := rgArrange(t, rgDisplaced(p, delta), prismrecord.Payload{Profile: rgSquare(30, 0, 40, 10)})
		arr, ok, err := NewArrangement(sc.budget, sc.tags, sc.cells, sc.recs, VertexReading{})
		require.NoError(t, err)
		require.True(t, ok)
		out := map[int]float64{}
		for _, e := range arr.edges {
			if e.origin.Region == 0 {
				out[e.origin.Segment] = e.c
			}
		}
		return out
	}
	base, withGap := allowances(closed), allowances(open)
	require.Len(t, base, 4)
	require.Len(t, withGap, 4)
	// The closing path (corner (10,0): legs 0 and gap) is gap long.
	require.InDelta(t, gap, withGap[0]-base[0], 1e-12, "the edge ending at the open junction")
	require.InDelta(t, gap, withGap[1]-base[1], 1e-12, "the edge starting at the open junction")
	require.InDelta(t, 0, withGap[2]-base[2], 1e-15)
	require.InDelta(t, 0, withGap[3]-base[3], 1e-15)
}

// ASA2 reads a trimmed edge's allowance with the closing paths of the ends it
// kept, which the walk's own allowance does not hold.
func TestTrimmedAllowanceKeepsClosing(t *testing.T) {
	t.Parallel()
	seg := rgLine(sectionrecord.Point2{U: 0, V: 0}, sectionrecord.Point2{U: 4, V: 0})
	w, err := boundarywalk.WalkOf(seg, nil)
	require.NoError(t, err)
	e := arrangedEdge{closing: 3e-7}
	require.InDelta(t, 3e-7, e.trimmedAllowance(seg, w)-WalkAllowance(seg, w), 1e-15)
	require.GreaterOrEqual(t, e.trimmedAllowance(seg, w), 3e-7)
}

// V3 reads an own end of an ArcSeg with the radial residual its read end can
// sit from the recorded End, not as an exact point.
func TestOwnEndErrChargesArcResidual(t *testing.T) {
	t.Parallel()
	arc := sectionrecord.ArcSeg{
		Start: sectionrecord.Point2{U: 5, V: 0}, End: sectionrecord.Point2{U: 0, V: 5 + 2e-9}, TEnd: 1}
	w, err := boundarywalk.WalkOf(arc, nil)
	require.NoError(t, err)
	e := arrangedEdge{src: arc, walk: w}
	rho := arcResidual(arc)
	require.InDelta(t, 2e-9, rho, 1e-12)
	require.GreaterOrEqual(t, e.ownEndErr(), rho)
	line := rgLine(sectionrecord.Point2{}, sectionrecord.Point2{U: 3, V: 4})
	lw, err := boundarywalk.WalkOf(line, nil)
	require.NoError(t, err)
	require.Zero(t, (&arrangedEdge{src: line, walk: lw}).ownEndErr(), `a line's ends are exact`)
}

// TangentAllowance divides by an exact lower bound of the line's length: for a
// 3-4-5 line the bound is the length itself, so θ = 4ε/5 rounded up and no
// ulp is lost to a float hypot.
func TestTangentAllowanceExactLength(t *testing.T) {
	t.Parallel()
	line := rgLine(sectionrecord.Point2{}, sectionrecord.Point2{U: 3, V: 4})
	const eps = 1e-9
	got := TangentAllowance(line, eps)
	require.GreaterOrEqual(t, got, 4*eps/5)
	require.InDelta(t, 4*eps/5, got, 4*eps/5*1e-14, `within rounding of 4ε/5`)
	require.Zero(t, TangentAllowance(line, 0))
	require.True(t, math.IsInf(TangentAllowance(rgLine(sectionrecord.Point2{}, sectionrecord.Point2{}), eps), 1))
}
