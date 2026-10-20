package revolveplan_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const (
	rimLo    = 2.0
	rimHi    = 6.0
	rimOuter = 3.0
)

func rimPayload(t *testing.T) revolveplan.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	rect := []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(rimLo, 0), End: p(rimHi, 0), TEnd: 1},
		sectionrecord.LineSeg{Start: p(rimHi, 0), End: p(rimHi, rimOuter), TEnd: 1},
		sectionrecord.LineSeg{Start: p(rimHi, rimOuter), End: p(rimLo, rimOuter), TEnd: 1},
		sectionrecord.LineSeg{Start: p(rimLo, rimOuter), End: p(rimLo, 0), TEnd: 1},
	}
	return revolveplan.Payload{
		Frame: frame, Xform: r3.Identity(), Phi1: math.Pi / 2, RadialProof: true,
		Ax: revolveaxis.Frame{DU: 1, RadialProof: true},
		Den: revolveangle.Sweep{
			Phi0: revolveangle.Angle{Rad: new(big.Rat), Turn: new(big.Rat)},
			Phi1: revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 4)},
		},
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: rect}},
	}
}

func rimEdge() []*topo.Edge {
	return []*topo.Edge{{
		Curve: surfacegeom.Line3{},
		Start: &topo.Vertex{Position: r3.Vec{X: rimHi}},
		End:   &topo.Vertex{Position: r3.Vec{X: rimHi, Y: rimOuter}},
	}}
}

func TestPayloadRimSource(t *testing.T) {
	t.Parallel()

	good := rimPayload(t)
	source, ok := good.RimSource(rimEdge(), 0.5, 0)
	require.True(t, ok)
	require.Equal(t, revolvemesh.RimSource{XLo: rimLo, XHi: rimHi, Radius: rimOuter, Setback: 0.5}, source)

	reversed := rimEdge()
	reversed[0].Start, reversed[0].End = reversed[0].End, reversed[0].Start
	_, ok = good.RimSource(reversed, 0.5, 0)
	require.True(t, ok, "the edge may run either way")

	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	shifted, err := r3.NewFrame(r3.Vec{X: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	turned, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	segment := func(p *revolveplan.Payload, i int, edit func(*sectionrecord.LineSeg)) {
		segs := append([]sectionrecord.CurveSegment(nil), p.Profile.Outer.Segments...)
		l := segs[i].(sectionrecord.LineSeg)
		edit(&l)
		segs[i] = l
		p.Profile.Outer.Segments = segs
	}

	payloads := map[string]func(*revolveplan.Payload){
		"full turn":         func(p *revolveplan.Payload) { p.Full = true },
		"surface result":    func(p *revolveplan.Payload) { p.SurfaceResult = true },
		"section delta":     func(p *revolveplan.Payload) { p.SectionDelta = 1 },
		"whole section":     func(p *revolveplan.Payload) { p.SectionWhole = true },
		"no radial proof":   func(p *revolveplan.Payload) { p.RadialProof = false },
		"placed":            func(p *revolveplan.Payload) { p.Xform = move },
		"hole":              func(p *revolveplan.Payload) { p.Profile.Holes = []sectionrecord.LoopRecord{p.Profile.Outer} },
		"three segments":    func(p *revolveplan.Payload) { p.Profile.Outer.Segments = p.Profile.Outer.Segments[:3] },
		"blend segments":    func(p *revolveplan.Payload) { p.BlendSegs = []map[int]struct{}{{0: {}}} },
		"blend kind":        func(p *revolveplan.Payload) { p.BlendKind = "fillet" },
		"draft caps":        func(p *revolveplan.Payload) { p.DraftCircleCaps = true },
		"draft shell roles": func(p *revolveplan.Payload) { p.DraftCircleShellRoles = true },
		"frame origin":      func(p *revolveplan.Payload) { p.Frame = shifted },
		"frame U":           func(p *revolveplan.Payload) { p.Frame = turned },
		"axis origin U":     func(p *revolveplan.Payload) { p.Ax.AU = 1 },
		"axis origin V":     func(p *revolveplan.Payload) { p.Ax.AV = 1 },
		"axis direction U":  func(p *revolveplan.Payload) { p.Ax.DU = 0.5 },
		"axis direction V":  func(p *revolveplan.Payload) { p.Ax.DV = 1 },
		"axis bound AU":     func(p *revolveplan.Payload) { p.Ax.AUBound = 1 },
		"axis bound AV":     func(p *revolveplan.Payload) { p.Ax.AVBound = 1 },
		"axis bound DU":     func(p *revolveplan.Payload) { p.Ax.DUBound = 1 },
		"axis bound DV":     func(p *revolveplan.Payload) { p.Ax.DVBound = 1 },
		"axis no proof":     func(p *revolveplan.Payload) { p.Ax.RadialProof = false },
		"start angle":       func(p *revolveplan.Payload) { p.Phi0 = 0.5 },
		"start span":        func(p *revolveplan.Payload) { p.Den.Phi0.Span = &proofbound.RatInterval{} },
		"start no rad":      func(p *revolveplan.Payload) { p.Den.Phi0.Rad = nil },
		"start no turn":     func(p *revolveplan.Payload) { p.Den.Phi0.Turn = nil },
		"start rad":         func(p *revolveplan.Payload) { p.Den.Phi0.Rad = big.NewRat(1, 2) },
		"start turn":        func(p *revolveplan.Payload) { p.Den.Phi0.Turn = big.NewRat(1, 2) },
		"end span":          func(p *revolveplan.Payload) { p.Den.Phi1.Span = &proofbound.RatInterval{} },
		"end no rad":        func(p *revolveplan.Payload) { p.Den.Phi1.Rad = nil },
		"end no turn":       func(p *revolveplan.Payload) { p.Den.Phi1.Turn = nil },
		"end rad":           func(p *revolveplan.Payload) { p.Den.Phi1.Rad = big.NewRat(1, 2) },
		"end turn":          func(p *revolveplan.Payload) { p.Den.Phi1.Turn = big.NewRat(1, 3) },
		"arc segment":       func(p *revolveplan.Payload) { p.Profile.Outer.Segments[0] = sectionrecord.ArcSeg{} },
		"segment start t": func(p *revolveplan.Payload) {
			segment(p, 0, func(l *sectionrecord.LineSeg) { l.TStart = 0.25 })
		},
		"segment end t": func(p *revolveplan.Payload) {
			segment(p, 0, func(l *sectionrecord.LineSeg) { l.TEnd = 0.75 })
		},
		"bottom edge off the axis": func(p *revolveplan.Payload) {
			segment(p, 0, func(l *sectionrecord.LineSeg) { l.Start.V = 1 })
		},
		"side not vertical": func(p *revolveplan.Payload) {
			segment(p, 1, func(l *sectionrecord.LineSeg) { l.End.U = rimHi + 1 })
		},
		"top not level": func(p *revolveplan.Payload) {
			segment(p, 2, func(l *sectionrecord.LineSeg) { l.End.V = rimOuter + 1 })
		},
		"left not closed": func(p *revolveplan.Payload) {
			segment(p, 3, func(l *sectionrecord.LineSeg) { l.End.V = 1 })
		},
	}
	for name, edit := range payloads {
		p := good
		p.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), good.Profile.Outer.Segments...)
		edit(&p)
		_, ok := p.RimSource(rimEdge(), 0.5, 0)
		require.False(t, ok, name)
	}

	retarget := func(lo, hi, outer float64) func(*revolveplan.Payload) {
		return func(p *revolveplan.Payload) {
			q := sectionrecord.Point2{}
			pt := func(u, v float64) sectionrecord.Point2 { q.U, q.V = u, v; return q }
			p.Profile.Outer.Segments = []sectionrecord.CurveSegment{
				sectionrecord.LineSeg{Start: pt(lo, 0), End: pt(hi, 0), TEnd: 1},
				sectionrecord.LineSeg{Start: pt(hi, 0), End: pt(hi, outer), TEnd: 1},
				sectionrecord.LineSeg{Start: pt(hi, outer), End: pt(lo, outer), TEnd: 1},
				sectionrecord.LineSeg{Start: pt(lo, outer), End: pt(lo, 0), TEnd: 1},
			}
		}
	}
	// The retargeted rectangles keep the edge test honest by moving the edge too.
	for name, tc := range map[string]struct {
		lo, hi, outer, setback float64
	}{
		"setback below range":  {rimLo, rimHi, rimOuter, 0.0625},
		"setback above range":  {-16, 0, 16, 4.5},
		"setback not finite":   {rimLo, rimHi, rimOuter, math.NaN()},
		"setback eats length":  {2, 3.5, 3, 0.75},
		"setback eats radius":  {2, 6, 1.5, 0.75},
		"length below range":   {2, 2.5, 3, 0.125},
		"length above range":   {-8, 10, 3, 0.5},
		"radius below range":   {2, 6, 0.5, 0.125},
		"radius above range":   {2, 6, 17, 0.5},
		"start outside range":  {-17, -10, 3, 0.5},
		"end outside range":    {10, 17, 3, 0.5},
		"start not finite":     {math.Inf(-1), 6, 3, 0.5},
		"end not finite":       {2, math.Inf(1), 3, 0.5},
		"radius not finite":    {2, 6, math.Inf(1), 0.5},
		"start beyond the cap": {-17, 6, 3, 0.5},
	} {
		p := good
		retarget(tc.lo, tc.hi, tc.outer)(&p)
		edge := []*topo.Edge{{
			Curve: surfacegeom.Line3{},
			Start: &topo.Vertex{Position: r3.Vec{X: tc.hi}},
			End:   &topo.Vertex{Position: r3.Vec{X: tc.hi, Y: tc.outer}},
		}}
		_, ok := p.RimSource(edge, tc.setback, 0)
		require.False(t, ok, name)
	}
	// The rectangle itself is admitted at the edges of every range.
	for name, tc := range map[string]struct {
		lo, hi, outer, setback float64
	}{
		"smallest":  {0, 1, 1, 0.125},
		"largest":   {-16, 0, 16, 4},
		"far start": {-16, -15, 16, 0.125},
		"far end":   {15, 16, 16, 0.125},
	} {
		p := good
		retarget(tc.lo, tc.hi, tc.outer)(&p)
		edge := []*topo.Edge{{
			Curve: surfacegeom.Line3{},
			Start: &topo.Vertex{Position: r3.Vec{X: tc.hi}},
			End:   &topo.Vertex{Position: r3.Vec{X: tc.hi, Y: tc.outer}},
		}}
		got, ok := p.RimSource(edge, tc.setback, 0)
		require.True(t, ok, name)
		require.Equal(t, revolvemesh.RimSource{XLo: tc.lo, XHi: tc.hi, Radius: tc.outer, Setback: tc.setback}, got, name)
	}
	five := good
	five.Profile.Outer.Segments = append(append([]sectionrecord.CurveSegment(nil), good.Profile.Outer.Segments...),
		good.Profile.Outer.Segments[0])
	_, ok = five.RimSource(rimEdge(), 0.5, 0)
	require.False(t, ok, "five segments")

	for name, tc := range map[string]struct {
		edges    []*topo.Edge
		setback  float64
		delta    float64
		edgeEdit func([]*topo.Edge)
	}{
		"displaced setback": {edges: rimEdge(), setback: 0.5, delta: 1e-9},
		"no edge":           {edges: nil, setback: 0.5},
		"two edges":         {edges: append(rimEdge(), rimEdge()...), setback: 0.5},
		"curved edge":       {edges: rimEdge(), setback: 0.5, edgeEdit: func(e []*topo.Edge) { e[0].Curve = surfacegeom.Ellipse3{} }},
		"no start":          {edges: rimEdge(), setback: 0.5, edgeEdit: func(e []*topo.Edge) { e[0].Start = nil }},
		"no end":            {edges: rimEdge(), setback: 0.5, edgeEdit: func(e []*topo.Edge) { e[0].End = nil }},
		"wrong cap":         {edges: rimEdge(), setback: 0.5, edgeEdit: func(e []*topo.Edge) { e[0].Start.Position.X = rimLo; e[0].End.Position.X = rimLo }},
		"wrong start":       {edges: rimEdge(), setback: 0.5, edgeEdit: func(e []*topo.Edge) { e[0].Start.Position.Y = 1 }},
		"wrong end":         {edges: rimEdge(), setback: 0.5, edgeEdit: func(e []*topo.Edge) { e[0].End.Position.Y = 1 }},
	} {
		if tc.edgeEdit != nil {
			tc.edgeEdit(tc.edges)
		}
		_, ok := good.RimSource(tc.edges, tc.setback, tc.delta)
		require.False(t, ok, name)
	}
}
