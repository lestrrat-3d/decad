package pointquery

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// boredSlabs is the 10 mm square over z ∈ [0, 5] under the same square with
// a radius-2 hole about (5, 5) over z ∈ [5, 10]: a blind round pocket.
func boredSlabs() []stackedrecord.Slab {
	square := momentinput.Profile{Outer: squareLoop()}
	bored := momentinput.Profile{Outer: squareLoop(), Holes: []sectionrecord.LoopRecord{
		{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 5, V: 5}, 2, false)}},
	}}
	return []stackedrecord.Slab{
		{Regions: []momentinput.Profile{square}, Z0: 0, Z1: 5},
		{Regions: []momentinput.Profile{bored}, Z0: 5, Z1: 10},
	}
}

func stackedOf(t *testing.T, slabs []stackedrecord.Slab) stackedrecord.Payload {
	t.Helper()
	pp := payloadOf(t, momentinput.Profile{})
	return stackedrecord.Payload{Slabs: slabs, Interfaces: make([]stackedrecord.Interface, len(slabs)-1), Frame: pp.Frame, Xform: pp.Xform}
}

func requireStackedLocations(t *testing.T, sp stackedrecord.Payload, cases []prismCase) {
	t.Helper()
	for _, tc := range cases {
		got, decided, err := LocateStacked(t.Context(), tc.at, sp)
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want != Undecided, decided, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
}

// pocketCases are the bored slabs' answers, the same for a stacked prism and
// a stack BRep.
var pocketCases = []prismCase{
	{name: "under the pocket", at: r3.NewVec(5, 5, 3), want: Inside},
	{name: "pocket floor", at: r3.NewVec(5, 5, 5), want: OnBoundary},
	{name: "in the pocket", at: r3.NewVec(5, 5, 7), want: Outside},
	{name: "pocket wall", at: r3.NewVec(5, 7, 7), want: OnBoundary},
	{name: "foot of the wall", at: r3.NewVec(5, 3, 5), want: OnBoundary},
	{name: "interface level in the material", at: r3.NewVec(1, 1, 5), want: Inside},
	{name: "interface level on the side", at: r3.NewVec(10, 1, 5), want: OnBoundary},
	{name: "beside the interface", at: r3.NewVec(11, 1, 5), want: Outside},
	{name: "pocket mouth", at: r3.NewVec(5, 5, 10), want: Outside},
	{name: "top face", at: r3.NewVec(1, 1, 10), want: OnBoundary},
	{name: "bottom face", at: r3.NewVec(5, 5, 0), want: OnBoundary},
	{name: "below", at: r3.NewVec(5, 5, -1e-9), want: Outside},
	{name: "above", at: r3.NewVec(1, 1, 10+1e-9), want: Outside},
}

func TestLocateStackedPocket(t *testing.T) {
	t.Parallel()
	requireStackedLocations(t, stackedOf(t, boredSlabs()), pocketCases)
}

// TestLocateStackedGroup reads one slab holding two squares that share the
// side u = 10: a point on that side is on both regions' boundaries, which the
// union table leaves undecided.
func TestLocateStackedGroup(t *testing.T) {
	t.Parallel()
	a, b, c, d := pt{U: 10, V: 0}, pt{U: 20, V: 0}, pt{U: 20, V: 10}, pt{U: 10, V: 10}
	right := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(a, b), line(b, c), line(c, d), line(d, a),
	}}}
	sp := stackedOf(t, []stackedrecord.Slab{{
		Regions: []momentinput.Profile{{Outer: squareLoop()}, right}, Z0: 0, Z1: 10,
	}})
	requireStackedLocations(t, sp, []prismCase{
		{name: "first square", at: r3.NewVec(5, 5, 5), want: Inside},
		{name: "second square", at: r3.NewVec(15, 5, 5), want: Inside},
		{name: "second square's side", at: r3.NewVec(20, 5, 5), want: OnBoundary},
		{name: "shared side", at: r3.NewVec(10, 5, 5), want: Undecided},
	})
}

// TestAtLevelTable checks the level table of
// agent-docs/point-containment-design.md §9.2, written out.
func TestAtLevelTable(t *testing.T) {
	t.Parallel()
	answers := []Location{Inside, OnBoundary, Outside, Undecided}
	want := [4][4]Location{
		{Inside, OnBoundary, OnBoundary, Undecided},
		{OnBoundary, OnBoundary, OnBoundary, OnBoundary},
		{OnBoundary, OnBoundary, Outside, Undecided},
		{Undecided, OnBoundary, Undecided, Undecided},
	}
	for i, below := range answers {
		for j, above := range answers {
			require.Equal(t, want[i][j], atLevel(below, above), "below %v above %v", below, above)
		}
	}
}

// TestLocateStackedLeavesOpenRecords covers every gate that hands a stacked
// record back to the mesh.
func TestLocateStackedLeavesOpenRecords(t *testing.T) {
	t.Parallel()
	at := r3.NewVec(5, 5, 3)
	gates := map[string]func(*stackedrecord.Payload){
		"section displacement":    func(sp *stackedrecord.Payload) { sp.SectionDelta = 1e-12 },
		"start level displaced":   func(sp *stackedrecord.Payload) { sp.Slabs[0].Z0Delta = 1e-12 },
		"end level displaced":     func(sp *stackedrecord.Payload) { sp.Slabs[1].Z1Delta = 1e-12 },
		"slab without a region":   func(sp *stackedrecord.Payload) { sp.Slabs[1].Regions = nil },
		"empty interval":          func(sp *stackedrecord.Payload) { sp.Slabs[0].Z1, sp.Slabs[1].Z0 = 0, 0 },
		"levels that do not meet": func(sp *stackedrecord.Payload) { sp.Slabs[1].Z0 = 5.5 },
		"no slab":                 func(sp *stackedrecord.Payload) { sp.Slabs = nil },
		"singular placement":      func(sp *stackedrecord.Payload) { sp.Xform = r3.Transform{} },
	}
	for name, gate := range gates {
		sp := stackedOf(t, boredSlabs())
		gate(&sp)
		_, decided, err := LocateStacked(t.Context(), at, sp)
		require.NoError(t, err, name)
		require.False(t, decided, name)
	}
}

// stackBrep is a BRep record keeping the bored slabs, its first face the
// bottom cap in the reference frame.
func stackBrep(t *testing.T) breprecord.Payload {
	t.Helper()
	pp := payloadOf(t, momentinput.Profile{})
	bottom := momentinput.Profile{Outer: squareLoop()}
	wall := breprecord.Face{Frame: pp.Frame, Wall: line(pt{U: 0, V: 0}, pt{U: 10, V: 0}), Z0: 0, Z1: 10}
	return breprecord.Payload{
		Faces: []breprecord.Face{{Frame: pp.Frame, Region: &bottom}, wall},
		Xform: pp.Xform,
		Stack: &breprecord.Stack{Slabs: boredSlabs()},
	}
}

func TestLocateStackBrep(t *testing.T) {
	t.Parallel()
	bp := stackBrep(t)
	for _, tc := range pocketCases {
		got, decided, err := LocateStackBrep(t.Context(), tc.at, bp)
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want != Undecided, decided, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
}

func TestLocatePointQueryCutBrep(t *testing.T) {
	t.Parallel()
	bp := stackBrep(t)
	outer := squareLoop()
	inner := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(pt{U: 4, V: 4}, pt{U: 4, V: 6}),
		line(pt{U: 4, V: 6}, pt{U: 6, V: 6}),
		line(pt{U: 6, V: 6}, pt{U: 6, V: 4}),
		line(pt{U: 6, V: 4}, pt{U: 4, V: 4}),
	}}
	target := payloadOf(t, momentinput.Profile{Outer: outer, Holes: []sectionrecord.LoopRecord{inner}})
	tool := payloadOf(t, momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(pt{U: 1, V: 1}, pt{U: 9, V: 1}),
		line(pt{U: 9, V: 1}, pt{U: 9, V: 9}),
		line(pt{U: 9, V: 9}, pt{U: 1, V: 9}),
		line(pt{U: 1, V: 9}, pt{U: 1, V: 1}),
	}}})
	tool.Z0, tool.Z1 = 6, 10
	bp.PointQueryCut = &breprecord.CutOperands{Target: target, Tool: tool}
	bp.Stack = nil
	for _, tc := range []prismCase{
		{name: "inside below tool", at: r3.NewVec(2, 5, 5), want: Inside},
		{name: "tool floor", at: r3.NewVec(2, 5, 6), want: OnBoundary},
		{name: "outside target", at: r3.NewVec(12, 5, 5), want: Outside},
	} {
		got, decided, err := LocateStackBrep(t.Context(), tc.at, bp)
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want != Undecided, decided, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
	bp.Stack = &breprecord.Stack{}
	_, decided, err := LocateStackBrep(t.Context(), r3.NewVec(5, 5, 3), bp)
	require.NoError(t, err)
	require.False(t, decided)
	bp.Stack = nil
	bp.PointQueryCut.Target.Profile.Holes = nil
	_, decided, err = LocateStackBrep(t.Context(), r3.NewVec(2, 5, 5), bp)
	require.NoError(t, err)
	require.False(t, decided)
}

// TestLocateStackBrepLeavesOpenRecords covers every gate that hands a BRep
// back to the mesh.
func TestLocateStackBrepLeavesOpenRecords(t *testing.T) {
	t.Parallel()
	at := r3.NewVec(5, 5, 3)
	gates := map[string]func(*breprecord.Payload){
		"no stack":              func(bp *breprecord.Payload) { bp.Stack = nil },
		"stack displacement":    func(bp *breprecord.Payload) { bp.Stack.Delta = 1e-12 },
		"stack level displaced": func(bp *breprecord.Payload) { bp.Stack.Slabs[0].Z1Delta, bp.Stack.Slabs[1].Z0Delta = 1e-12, 1e-12 },
		"no face":               func(bp *breprecord.Payload) { bp.Faces = nil },
		"loop band":             func(bp *breprecord.Payload) { bp.LoopBands = []breprecord.LoopBand{{}} },
		"boss shell band":       func(bp *breprecord.Payload) { bp.BossShell = &prismshell.BossShellBand{} },
		"pocket shell band":     func(bp *breprecord.Payload) { bp.PocketShell = &prismshell.PocketShellBand{} },
		"face displacement":     func(bp *breprecord.Payload) { bp.Faces[1].Delta = 1e-12 },
		"face level displaced":  func(bp *breprecord.Payload) { bp.Faces[1].Z0Delta = 1e-12 },
		"face end displaced":    func(bp *breprecord.Payload) { bp.Faces[1].Z1Delta = 1e-12 },
		"blend face":            func(bp *breprecord.Payload) { bp.Faces[1].Blend = "fillet" },
		"split displaced": func(bp *breprecord.Payload) {
			bp.Faces[1].Side1 = []brepgeom.Split{{Z: 5, ZDelta: 1e-12}}
		},
		"first face swept": func(bp *breprecord.Payload) { bp.Faces[0], bp.Faces[1] = bp.Faces[1], bp.Faces[0] },
		"open junction": func(bp *breprecord.Payload) {
			bp.Stack.Slabs[0].Regions[0].Outer = openSquare(0, 10)
		},
		"open junction in a hole": func(bp *breprecord.Payload) {
			bp.Stack.Slabs[1].Regions[0].Holes = []sectionrecord.LoopRecord{reverseLoop(openSquare(4, 6))}
		},
		"first face restated": func(bp *breprecord.Payload) { bp.Faces[0].Sweep = r3.NewVec(1, 0, 0) },
	}
	for name, gate := range gates {
		bp := stackBrep(t)
		gate(&bp)
		_, decided, err := LocateStackBrep(t.Context(), at, bp)
		require.NoError(t, err, name)
		require.False(t, decided, name)
	}
}

// openSquare is the square [lo, hi]² walked counter-clockwise, its second
// side ending one ulp past the third side's start.
func openSquare(lo, hi float64) sectionrecord.LoopRecord {
	a, b, c, d := pt{U: lo, V: lo}, pt{U: hi, V: lo}, pt{U: hi, V: hi}, pt{U: lo, V: hi}
	past := pt{U: hi, V: math.Nextafter(hi, math.Inf(1))}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{line(a, b), line(b, past), line(c, d), line(d, a)}}
}

// reverseLoop walks a loop of whole lines backwards.
func reverseLoop(loop sectionrecord.LoopRecord) sectionrecord.LoopRecord {
	out := sectionrecord.LoopRecord{}
	for _, segment := range slices.Backward(loop.Segments) {
		l := segment.(sectionrecord.LineSeg)
		out.Segments = append(out.Segments, reversedLine(l.End, l.Start))
	}
	return out
}
