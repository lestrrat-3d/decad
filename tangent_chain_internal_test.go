package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/tangentchain"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests cover docs/modify-reach-design.md §13's tangent-chain list at
// the expansion itself: the per-endpoint outcome table, the cycle and order
// rules, and a continuation whose tangent is exact but whose faces differ.
//
// Legs shown to fail before these fixtures were accepted: letting chainStep
// take the first proven candidate sends the Branch case red; treating an
// undecided candidate as a stop sends the Undecided case red; returning the
// expansion in set order rather than body order sends the body-order test
// red;
// and deciding a continuation on the tangent alone, without the face test,
// sends the face-sheet test red.

func TestChainStep(t *testing.T) {
	t.Parallel()
	yes, no, unknown := clearance.DegYes, clearance.DegNo, clearance.DegUnknown
	tests := []struct {
		name   string
		states []clearance.DegState
		next   int
		sx2    bool
	}{
		{name: "NoCandidate", states: nil, next: -1},
		{name: "AllNo", states: []clearance.DegState{no, no}, next: -1},
		{name: "OneProven", states: []clearance.DegState{no, yes, no}, next: 1},
		{name: "Branch", states: []clearance.DegState{yes, no, yes}, sx2: true},
		{name: "Undecided", states: []clearance.DegState{no, unknown}, sx2: true},
		{name: "ProvenBesideUndecided", states: []clearance.DegState{yes, unknown}, sx2: true},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			next, proven, undecided := tangentchain.Step(tc.states)
			if tc.sx2 {
				require.True(t, proven > 1 || undecided > 0)
				return
			}
			require.LessOrEqual(t, proven, 1)
			require.Zero(t, undecided)
			require.Equal(t, tc.next, next)
		})
	}
}

// chainSlot extrudes the exact stadium of apitest's slotBody: straight walls
// along y = 0 and y = 10 from x = 0 to 10, semicircles about (10, 5) and
// (0, 5).
func chainSlot(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	p0, p1, p2, p3 := fixed(0, 0), fixed(10, 0), fixed(10, 10), fixed(0, 10)
	s.CreateLine(p0, p1)
	s.CreateArc(fixed(10, 5), p1, p2)
	s.CreateLine(p2, p3)
	s.CreateArc(fixed(0, 5), p3, p0)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	return body
}

func TestTangentChainBodyOrderAndCycle(t *testing.T) {
	t.Parallel()
	body := chainSlot(t)
	capEnd := facesByRole(body)[roleCapEnd]
	require.NotNil(t, capEnd)
	inLoop := map[*Edge]struct{}{}
	for _, e := range capEnd.Edges() {
		inLoop[e] = struct{}{}
	}
	require.Len(t, inLoop, 4)

	// Seed the LAST loop edge in body order, and then two edges of the same
	// loop: either way the closed cycle visits each edge once and the
	// result keeps body edge order.
	var want []*Edge
	for _, e := range body.Edges() {
		if _, ok := inLoop[e]; ok {
			want = append(want, e)
		}
	}
	for _, seeds := range [][]*Edge{{want[3]}, {want[2], want[0]}} {
		got, err := expandTangentChain(t.Context(), body, Edges(), seeds)
		require.NoError(t, err)
		require.Equal(t, want, got)
	}

	// A lateral edge has no continuation: its endpoints' other edges are cap
	// rims, perpendicular to it.
	lateral, err := Edges(ParallelTo(r3.NewVec(0, 0, 1))).SelectEdges(body)
	require.NoError(t, err)
	got, err := expandTangentChain(t.Context(), body, Edges(), lateral[:1])
	require.NoError(t, err)
	require.Equal(t, lateral[:1], got)
}

func TestTangentContinuesFaceSheets(t *testing.T) {
	t.Parallel()
	// A box's top front edge e, from (0,0,4) to (10,0,4), between the top
	// (+z) and front (−y) walls. Candidate c continues e's line beyond
	// (10,0,4) exactly. With c between the top and a wall whose outward
	// normal is also −y, the faces map one to one with equal normals and c
	// continues e; with the −y wall replaced by the box's +x wall, the
	// tangent is unchanged and the face sheets are not, so c stops.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)

	var e *Edge
	for _, cand := range box.Edges() {
		a, b := cand.start.position, cand.end.position
		if a.Z == 4 && b.Z == 4 && a.Y == 0 && b.Y == 0 {
			e = cand
		}
	}
	require.NotNil(t, e)
	v := e.end
	if v.position.X != 10 {
		v = e.start
	}
	var top, front, right *Face
	for _, f := range box.Faces() {
		p, ok := f.surface.(Plane)
		if !ok {
			continue
		}
		n := p.Frame.N()
		if f.reversed {
			n = n.Scale(-1)
		}
		switch n {
		case r3.NewVec(0, 0, 1):
			top = f
		case r3.NewVec(0, -1, 0):
			front = f
		case r3.NewVec(1, 0, 0):
			right = f
		}
	}
	require.NotNil(t, top)
	require.NotNil(t, front)
	require.NotNil(t, right)

	far := &Vertex{position: r3.NewVec(20, 0, 4)}
	cont := &Edge{curve: Line3{}, start: v, end: far, faces: []*Face{front, top}}
	require.Equal(t, clearance.DegYes, tangentContinues(e, cont, v))
	stop := &Edge{curve: Line3{}, start: v, end: far, faces: []*Face{right, top}}
	require.Equal(t, clearance.DegNo, tangentContinues(e, stop, v))
	// The same line walked back over e is no continuation: its ray from v
	// points along e's own.
	back := &Edge{curve: Line3{}, start: v, end: &Vertex{position: r3.NewVec(5, 0, 4)}, faces: []*Face{front, top}}
	require.Equal(t, clearance.DegNo, tangentContinues(e, back, v))
}
