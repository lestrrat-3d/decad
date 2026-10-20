package crossbore_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cylinder is a solid circular prism body on the cardinal frame (u, v), with
// its two circle edges bounded by bound.
func cylinder(t *testing.T, u, v r3.Vec, centerU, centerV, radius, z0, z1, bound float64) *topo.Body {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, u, v)
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: centerU, V: centerV},
		Radius: units.Millimeters(radius), CCW: true, TStart: 0, TEnd: 1}
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: z0, Z1: z1,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}}
	body := &topo.Body{Payload: pp, Solid: true, Kind: topo.BodySolid}
	mouth := func() *topo.Edge {
		return &topo.Edge{Curve: surfacegeom.Circle3{}, CurveBounded: true, CurveBound: bound}
	}
	loop := &topo.Loop{Outer: true}
	for _, e := range []*topo.Edge{mouth(), mouth()} {
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: e, Forward: true})
	}
	body.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{{Loops: []*topo.Loop{loop}}}}}}}
	return body
}

// pair is the canonical perpendicular cylinder Cut: a vertical cylinder of
// radius 5 and height 10 with a horizontal bore of radius 1 centred at height 5.
func pair(t *testing.T) (target, tool *topo.Body) {
	t.Helper()
	target = cylinder(t, r3.Vec{X: 1}, r3.Vec{Y: 1}, 0, 0, 5, 0, 10, 1e-9)
	tool = cylinder(t, r3.Vec{Y: 1}, r3.Vec{Z: 1}, 0, 5, 1, -10, 10, 1e-9)
	return target, tool
}

func TestRecognizeBore(t *testing.T) {
	t.Parallel()

	target, tool := pair(t)
	rec := crossbore.RecognizeBore(target, tool)
	require.NotNil(t, rec)
	require.Same(t, target, rec.Target)
	require.Same(t, tool, rec.Tool)
	require.Equal(t, 5.0, rec.Radius)
	require.Equal(t, 1.0, rec.Bore)
	require.Equal(t, 10.0, rec.Height)
	require.Equal(t, 5.0, rec.CenterZ)
	require.Greater(t, rec.SourceBound, 0.0)
	require.Less(t, rec.SourceBound, 1e-3)

	// Edges that are not circles do not count toward the two circle edges.
	target, tool = pair(t)
	loop := target.Lumps[0].Shells[0].Faces[0].Loops[0]
	loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: &topo.Edge{Curve: surfacegeom.Line3{}}, Forward: true})
	require.NotNil(t, crossbore.RecognizeBore(target, tool))

	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	tests := map[string]func(target, tool *topo.Body){
		"target not a prism":      func(t, _ *topo.Body) { t.Payload = nil },
		"target not solid":        func(t, _ *topo.Body) { t.Solid = false },
		"target not a solid kind": func(t, _ *topo.Body) { t.Kind = topo.BodySheet },
		"target a sheet result": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.SurfaceResult = true
			t.Payload = pp
		},
		"target section displaced": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.SectionDelta = 1e-6
			t.Payload = pp
		},
		"target start displaced": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Z0Delta = 1e-6
			t.Payload = pp
		},
		"target end displaced": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Z1Delta = 1e-6
			t.Payload = pp
		},
		"target placed": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Xform = move
			t.Payload = pp
		},
		"target has a hole": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Profile.Holes = []sectionrecord.LoopRecord{pp.Profile.Outer}
			t.Payload = pp
		},
		"target has two segments": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Profile.Outer.Segments = append(append([]sectionrecord.CurveSegment(nil), pp.Profile.Outer.Segments...),
				pp.Profile.Outer.Segments[0])
			t.Payload = pp
		},
		"target not a circle": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Profile.Outer.Segments = []sectionrecord.CurveSegment{sectionrecord.ArcSeg{}}
			t.Payload = pp
		},
		"target clockwise":  func(t, _ *topo.Body) { mapCircle(t, func(c *sectionrecord.CircleSeg) { c.CCW = false }) },
		"target part turn":  func(t, _ *topo.Body) { mapCircle(t, func(c *sectionrecord.CircleSeg) { c.TEnd = 0.5 }) },
		"target late start": func(t, _ *topo.Body) { mapCircle(t, func(c *sectionrecord.CircleSeg) { c.TStart = 0.25 }) },
		"target off axis": func(t, _ *topo.Body) {
			mapCircle(t, func(c *sectionrecord.CircleSeg) { c.Center.U = 0.5 })
		},
		"target not finite": func(t, _ *topo.Body) {
			mapCircle(t, func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(math.Inf(1)) })
		},
		"target zero radius": func(t, _ *topo.Body) {
			mapCircle(t, func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(0) })
		},
		"target starts above zero": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Z0 = 1
			t.Payload = pp
		},
		"target not tall": func(t, _ *topo.Body) {
			pp := t.Payload.(prismrecord.Payload)
			pp.Z1 = 0
			t.Payload = pp
		},
		"tool centred off the axis": func(_, tool *topo.Body) { mapCircle(tool, func(c *sectionrecord.CircleSeg) { c.Center.U = 0.5 }) },
		"tool too wide": func(_, tool *topo.Body) {
			mapCircle(tool, func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(2) })
		},
		"tool too thin": func(_, tool *topo.Body) {
			mapCircle(tool, func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(0.1) })
		},
		"tool leaves the top": func(_, tool *topo.Body) {
			mapCircle(tool, func(c *sectionrecord.CircleSeg) { c.Center.V = 9.5 })
		},
		"tool reaches the bottom": func(_, tool *topo.Body) {
			mapCircle(tool, func(c *sectionrecord.CircleSeg) { c.Center.V = 1 })
		},
		"tool too short": func(_, tool *topo.Body) {
			pp := tool.Payload.(prismrecord.Payload)
			pp.Z0 = -3
			tool.Payload = pp
		},
		"tool ends inside": func(_, tool *topo.Body) {
			pp := tool.Payload.(prismrecord.Payload)
			pp.Z1 = 3
			tool.Payload = pp
		},
		"tool too long": func(_, tool *topo.Body) {
			pp := tool.Payload.(prismrecord.Payload)
			pp.Z0 = -101
			tool.Payload = pp
		},
		"target edge bound unknown":  func(t, _ *topo.Body) { firstEdge(t).CurveBounded = false },
		"target edge bound infinite": func(t, _ *topo.Body) { firstEdge(t).CurveBound = math.Inf(1) },
		"target edge bound loose":    func(t, _ *topo.Body) { firstEdge(t).CurveBound = 1 },
		"target edge missing": func(t, _ *topo.Body) {
			shell := t.Lumps[0].Shells[0].Faces[0].Loops[0]
			shell.CoEdges = shell.CoEdges[:1]
		},
		"tool edge bound loose": func(_, tool *topo.Body) { firstEdge(tool).CurveBound = 1 },
		"bounds add past the smaller radius": func(t, tool *topo.Body) {
			firstEdge(t).CurveBound = 0.0049
			firstEdge(tool).CurveBound = 0.0009
		},
	}
	for name, edit := range tests {
		target, tool := pair(t)
		edit(target, tool)
		require.Nil(t, crossbore.RecognizeBore(target, tool), name)
	}
}

func mapCircle(body *topo.Body, edit func(*sectionrecord.CircleSeg)) {
	pp := body.Payload.(prismrecord.Payload)
	c := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	edit(&c)
	pp.Profile.Outer.Segments = []sectionrecord.CurveSegment{c}
	body.Payload = pp
}

func firstEdge(body *topo.Body) *topo.Edge {
	return body.Lumps[0].Shells[0].Faces[0].Loops[0].CoEdges[0].Edge
}

// mouthFixture is a cross-bore body whose one closed mouth edge sits between
// the target's and the tool's side faces, with the record that names them.
func mouthFixture(t *testing.T) (*topo.Body, facetedrecord.Payload, []*topo.Edge) {
	t.Helper()
	target, tool := pair(t)
	rec := crossbore.RecognizeBore(target, tool)
	require.NotNil(t, rec)
	side := func(body *topo.Body, producer provenance.Producer) *topo.Face {
		body.Origin = provenance.Ref{Producer: producer}
		face := &topo.Face{Surface: surfacegeom.Plane{},
			Origins: []provenance.Ref{{Producer: producer, Role: prismrecord.OuterFirstSideRole}}}
		body.Lumps[0].Shells[0].Faces = []*topo.Face{face}
		return face
	}
	outer, bore := side(target, 3), side(tool, 4)
	v := &topo.Vertex{Position: r3.Vec{X: 5}}
	mouth := &topo.Edge{Curve: surfacegeom.FacetedCurve{}, Start: v, End: v, Faces: []*topo.Face{outer, bore}}
	other := func() *topo.Edge { return &topo.Edge{Faces: []*topo.Face{outer}} }
	loop := &topo.Loop{Outer: true}
	for _, e := range []*topo.Edge{mouth, other(), other(), other()} {
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: e, Forward: true})
	}
	body := &topo.Body{Solid: true, Kind: topo.BodySolid}
	body.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{{Loops: []*topo.Loop{loop}}}}}}}
	return body, facetedrecord.Payload{CrossCylinderBore: rec, Xform: r3.Identity()}, []*topo.Edge{mouth}
}

const notMouth = "not the positive cross-bore mouth"

func TestMouthChamferGates(t *testing.T) {
	t.Parallel()

	const setback = 0.25
	run := func(body *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge, setback, delta float64, asym bool) error {
		_, recognized, err := crossbore.MouthChamfer(t.Context(), topo.Env{}, body, fp, edges, setback, delta, asym)
		if fp.CrossCylinderBore == nil {
			require.False(t, recognized)
			require.NoError(t, err)
			return nil
		}
		require.True(t, recognized, "a cross-bore receiver is always recognized")
		return err
	}
	require.NoError(t, run(&topo.Body{}, facetedrecord.Payload{}, nil, setback, 0, false), "no cross-bore record")

	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	tests := map[string]struct {
		edit func(body *topo.Body, fp *facetedrecord.Payload, edges *[]*topo.Edge)
		msg  string
	}{
		"placed": {func(_ *topo.Body, fp *facetedrecord.Payload, _ *[]*topo.Edge) { fp.Xform = move }, "one complete unplaced mouth loop"},
		"two edges": {func(_ *topo.Body, _ *facetedrecord.Payload, e *[]*topo.Edge) {
			*e = append(*e, (*e)[0])
		}, "one complete unplaced mouth loop"},
		"open edge": {func(_ *topo.Body, _ *facetedrecord.Payload, e *[]*topo.Edge) {
			(*e)[0].End = &topo.Vertex{}
		}, "one complete unplaced mouth loop"},
		"wrong edge count": {func(b *topo.Body, _ *facetedrecord.Payload, _ *[]*topo.Edge) {
			l := b.Lumps[0].Shells[0].Faces[0].Loops[0]
			l.CoEdges = l.CoEdges[:3]
		}, "one complete unplaced mouth loop"},
		"no faceted witness": {func(_ *topo.Body, _ *facetedrecord.Payload, e *[]*topo.Edge) {
			(*e)[0].Curve = surfacegeom.Circle3{}
		}, "no faceted edge witness"},
		"edge between other faces": {func(_ *topo.Body, _ *facetedrecord.Payload, e *[]*topo.Edge) {
			(*e)[0].Faces = []*topo.Face{{}, {}}
		}, notMouth},
		"edge with one face": {func(_ *topo.Body, _ *facetedrecord.Payload, e *[]*topo.Edge) {
			(*e)[0].Faces = (*e)[0].Faces[:1]
		}, notMouth},
		"edge on the negative side": {func(_ *topo.Body, _ *facetedrecord.Payload, e *[]*topo.Edge) {
			(*e)[0].Start.Position.X = 1
			(*e)[0].End = (*e)[0].Start
		}, notMouth},
		"target lost its side face": {func(_ *topo.Body, fp *facetedrecord.Payload, _ *[]*topo.Edge) {
			fp.CrossCylinderBore.Target.Lumps[0].Shells[0].Faces = nil
		}, notMouth},
		"tool lost its side face": {func(_ *topo.Body, fp *facetedrecord.Payload, _ *[]*topo.Edge) {
			fp.CrossCylinderBore.Tool.Lumps[0].Shells[0].Faces = nil
		}, notMouth},
	}
	for name, tc := range tests {
		body, fp, edges := mouthFixture(t)
		tc.edit(body, &fp, &edges)
		err := run(body, fp, edges, setback, 0, false)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
	}

	const clearance = "exceeds its proven clearance"
	for name, tc := range map[string]struct {
		setback, delta float64
		asym           bool
	}{
		"asymmetric":         {setback, 0, true},
		"displaced setback":  {setback, 1e-9, false},
		"zero setback":       {0, 0, false},
		"negative setback":   {-1, 0, false},
		"nan setback":        {math.NaN(), 0, false},
		"infinite setback":   {math.Inf(1), 0, false},
		"wider than bore/2":  {0.6, 0, false},
		"reaches the bottom": {4.2, 0, false},
		"reaches the top":    {4.2, 0, false},
	} {
		body, fp, edges := mouthFixture(t)
		switch name {
		case "wider than bore/2":
			fp.CrossCylinderBore.Bore = 1
		case "reaches the bottom":
			fp.CrossCylinderBore.Bore, fp.CrossCylinderBore.CenterZ = 4, 4.1
		case "reaches the top":
			fp.CrossCylinderBore.Bore, fp.CrossCylinderBore.CenterZ = 4, 5.9
			fp.CrossCylinderBore.Height = 10
		}
		if name == "reaches the bottom" || name == "reaches the top" {
			tc.setback = 0.5
		}
		err := run(body, fp, edges, tc.setback, tc.delta, tc.asym)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, clearance, name)
	}
}
