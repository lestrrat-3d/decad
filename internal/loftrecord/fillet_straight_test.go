package loftrecord_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func squareProfile() momentinput.Profile {
	corners := []sectionrecord.Point2{{}, {U: 4}, {U: 4, V: 4}, {V: 4}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

// prismLoft is an untwisted loft of one square section from z = 0 to z = 5.
func prismLoft(t *testing.T) loftrecord.Payload {
	t.Helper()
	frame := func(z float64) r3.Frame {
		f, err := r3.NewFrame(r3.Vec{Z: z}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		return f
	}
	plane := func(z float64) sectionrecord.PlaneRecord {
		return sectionrecord.PlaneRecord{Origin: r3.Vec{Z: z}, U: r3.Vec{X: 1}, V: r3.Vec{Y: 1}}
	}
	return loftrecord.Payload{
		Profile0: squareProfile(), Profile1: squareProfile(),
		Plane0: plane(0), Plane1: plane(5), Frame0: frame(0), Frame1: frame(5),
		Alignment: []int{0, 0}, Xform: r3.Identity(),
	}
}

func cornerEdge(u, v float64) *topo.Edge {
	return &topo.Edge{Curve: surfacegeom.Line3{},
		Start: &topo.Vertex{Position: r3.Vec{X: u, Y: v}}, End: &topo.Vertex{Position: r3.Vec{X: u, Y: v, Z: 5}}}
}

const (
	classMsg    = "a loft fillet needs matching, parallel, solid sections without holes"
	notEdgeMsg  = "the selected loft edge is not an axial corner"
	noCornerMsg = "a selected loft edge has no analytic section corner"
)

func TestFilletStraightGates(t *testing.T) {
	t.Parallel()

	offAxis := r3.Vec{X: 1, Z: 5}
	tests := map[string]struct {
		edit  func(p *loftrecord.Payload, edges *[]*topo.Edge)
		err   error
		msg   string
		works bool
	}{
		"fitted fillet again":   {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.ConstructionDelta = 1e-9 }, err: decaderr.ErrUnsupported, msg: "cannot be filleted again"},
		"arc upper recorded":    {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.ConstructionArcUpper = map[int]float64{0: 1} }, err: decaderr.ErrUnsupported, msg: "cannot be filleted again"},
		"length extra recorded": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.ConstructionLengthExtra = map[int]float64{0: 1} }, err: decaderr.ErrUnsupported, msg: "cannot be filleted again"},
		"sheet":                 {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.SurfaceResult = true }, err: decaderr.ErrUnsupported, msg: classMsg},
		"hole in the start": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) {
			p.Profile0.Holes = []sectionrecord.LoopRecord{p.Profile0.Outer}
		}, err: decaderr.ErrUnsupported, msg: classMsg},
		"hole in the end": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) {
			p.Profile1.Holes = []sectionrecord.LoopRecord{p.Profile1.Outer}
		}, err: decaderr.ErrUnsupported, msg: classMsg},
		"the same hole in both": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) {
			p.Profile0.Holes = []sectionrecord.LoopRecord{p.Profile0.Outer}
			p.Profile1.Holes = []sectionrecord.LoopRecord{p.Profile1.Outer}
		}, err: decaderr.ErrUnsupported, msg: classMsg},
		"different sections": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) {
			p.Profile1.Outer.Segments = slices.Clone(p.Profile1.Outer.Segments)
			p.Profile1.Outer.Segments[0] = sectionrecord.LineSeg{End: sectionrecord.Point2{U: 3}, TEnd: 1}
		}, err: decaderr.ErrUnsupported, msg: classMsg},
		"planes turned in u": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.Plane1.U = r3.Vec{Y: 1} }, err: decaderr.ErrUnsupported, msg: classMsg},
		"planes turned in v": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.Plane1.V = r3.Vec{X: 1} }, err: decaderr.ErrUnsupported, msg: classMsg},
		"sections out of step": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.Alignment = []int{0, 1} }, err: decaderr.ErrUnsupported,
			msg: "same segment order"},
		"slid along u": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.Plane1.Origin = offAxis }, err: decaderr.ErrUnsupported,
			msg: "axial section translation"},
		"slid along v": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) { p.Plane1.Origin = r3.Vec{Y: 1, Z: 5} }, err: decaderr.ErrUnsupported,
			msg: "axial section translation"},
		"one segment": {edit: func(p *loftrecord.Payload, _ *[]*topo.Edge) {
			one := sectionrecord.LoopRecord{Segments: p.Profile0.Outer.Segments[:1]}
			p.Profile0.Outer, p.Profile1.Outer = one, one
		}, err: decaderr.ErrUnsupported, msg: "needs an analytic section corner"},
		"curved edge":        {edit: func(_ *loftrecord.Payload, e *[]*topo.Edge) { (*e)[0].Curve = surfacegeom.Circle3{} }, err: decaderr.ErrUnsupported, msg: notEdgeMsg},
		"edge without start": {edit: func(_ *loftrecord.Payload, e *[]*topo.Edge) { (*e)[0].Start = nil }, err: decaderr.ErrUnsupported, msg: notEdgeMsg},
		"edge without end":   {edit: func(_ *loftrecord.Payload, e *[]*topo.Edge) { (*e)[0].End = nil }, err: decaderr.ErrUnsupported, msg: notEdgeMsg},
		"edge off every corner": {edit: func(_ *loftrecord.Payload, e *[]*topo.Edge) { *e = []*topo.Edge{cornerEdge(1, 1)} },
			err: decaderr.ErrUnsupported, msg: noCornerMsg},
		"edge on the cap only": {edit: func(_ *loftrecord.Payload, e *[]*topo.Edge) {
			(*e)[0].End.Position.Z = 2
		}, err: decaderr.ErrUnsupported, msg: noCornerMsg},
		"corner selected twice": {edit: func(_ *loftrecord.Payload, e *[]*topo.Edge) { *e = append(*e, cornerEdge(0, 0)) },
			err: decaderr.ErrDegenerate, msg: "selected twice"},
	}
	for name, tc := range tests {
		pl := prismLoft(t)
		edges := []*topo.Edge{cornerEdge(0, 0)}
		tc.edit(&pl, &edges)
		_, err := loftrecord.FilletStraight(t.Context(), topo.Env{}, "edges", edges, pl, 0.5)
		require.ErrorIs(t, err, tc.err, name)
		require.ErrorContains(t, err, tc.msg, name)
	}

	// An edge running from the top corner down to the bottom corner is the same edge.
	pl := prismLoft(t)
	reversed := cornerEdge(4, 4)
	reversed.Start, reversed.End = reversed.End, reversed.Start
	_, err := loftrecord.FilletStraight(t.Context(), topo.Env{}, "edges",
		[]*topo.Edge{reversed, cornerEdge(4, 4)}, pl, 0.5)
	require.ErrorIs(t, err, decaderr.ErrDegenerate, "either direction names the same corner")
	require.ErrorContains(t, err, "selected twice")
}
