package decad

import (
	"math"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/contactround"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func proofarithVec(v r3.Vec) proofarith.DyV3 { return proofarith.DyVec(v) }

func internalBallBody(t *testing.T, doc *Document, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(-r, 0)
	s.Fix(o)
	end := s.CreatePoint(r, 0)
	c := s.CreatePoint(0, 0)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Revolve(t.Context(), doc, s, s.Profiles()[0], SketchLine{End: Point2{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestContactSphereAdmissionGates builds a real ball, then breaks one fact of
// its payload, kind or face at a time. Each break must make the sphere reader
// refuse; the unbroken ball must read as a ball of its radius.
func TestContactSphereAdmissionGates(t *testing.T) {
	doc := New()
	body := internalBallBody(t, doc, 4)
	good := body.n.Payload.(revolvePayload)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	tilt, err := r3.NewFrame(r3.Vec{}, r3.NewVec(math.Cos(0.5), math.Sin(0.5), 0), r3.NewVec(-math.Sin(0.5), math.Cos(0.5), 0))
	require.NoError(t, err)

	proof, ok := contactround.SphereRecord(&body.n)
	require.True(t, ok)
	require.NotNil(t, proof.Face)
	radius, exact := proof.Radius.Float64()
	require.True(t, exact)
	require.Equal(t, 4.0, radius)
	_, ok = contactround.SphereAtPose(&body.n, r3.Identity())
	require.True(t, ok)

	segments := func(p *revolvePayload) []sectionrecord.CurveSegment {
		return append([]sectionrecord.CurveSegment(nil), p.Profile.Outer.Segments...)
	}
	editArc := func(edit func(*sectionrecord.ArcSeg)) func(*revolvePayload) {
		return func(p *revolvePayload) {
			segs := segments(p)
			for i, s := range segs {
				if arc, ok := s.(sectionrecord.ArcSeg); ok {
					edit(&arc)
					segs[i] = arc
				}
			}
			p.Profile.Outer.Segments = segs
		}
	}
	editLine := func(edit func(*sectionrecord.LineSeg)) func(*revolvePayload) {
		return func(p *revolvePayload) {
			segs := segments(p)
			for i, s := range segs {
				if line, ok := s.(sectionrecord.LineSeg); ok {
					edit(&line)
					segs[i] = line
				}
			}
			p.Profile.Outer.Segments = segs
		}
	}
	// editSpan changes the arc and moves the line with it, so only the
	// geometry gate under test can refuse the profile.
	editSpan := func(edit func(*sectionrecord.ArcSeg)) func(*revolvePayload) {
		return func(p *revolvePayload) {
			segs := segments(p)
			var arc sectionrecord.ArcSeg
			arcAt, lineAt := -1, -1
			for i, s := range segs {
				switch v := s.(type) {
				case sectionrecord.ArcSeg:
					arc, arcAt = v, i
				case sectionrecord.LineSeg:
					lineAt = i
				}
			}
			line := segs[lineAt].(sectionrecord.LineSeg)
			forward := line.Start == arc.Start
			edit(&arc)
			if forward {
				line.Start, line.End = arc.Start, arc.End
			} else {
				line.Start, line.End = arc.End, arc.Start
			}
			segs[arcAt], segs[lineAt] = arc, line
			p.Profile.Outer.Segments = segs
		}
	}
	edits := []struct {
		name string
		edit func(*revolvePayload)
	}{
		{caseSurfaceResult, func(p *revolvePayload) { p.SurfaceResult = true }},
		{"partial revolution", func(p *revolvePayload) { p.Full = false }},
		{caseSectionDisplacement, func(p *revolvePayload) { p.SectionDelta = 1 }},
		{"profile with a hole", func(p *revolvePayload) { p.Profile.Holes = append(p.Profile.Holes, p.Profile.Outer) }},
		{"three segments", func(p *revolvePayload) {
			p.Profile.Outer.Segments = append(segments(p), p.Profile.Outer.Segments[0])
		}},
		{"frame off the cardinal axes", func(p *revolvePayload) { p.Frame = tilt }},
		{"axis direction U", func(p *revolvePayload) { p.Ax.DU = 0.5 }},
		{"axis direction V", func(p *revolvePayload) { p.Ax.DV = 1 }},
		{"axis origin U bound", func(p *revolvePayload) { p.Ax.AUBound = 1 }},
		{"axis origin V bound", func(p *revolvePayload) { p.Ax.AVBound = 1 }},
		{"axis direction U bound", func(p *revolvePayload) { p.Ax.DUBound = 1 }},
		{"axis direction V bound", func(p *revolvePayload) { p.Ax.DVBound = 1 }},
		{"non-arc segment", func(p *revolvePayload) {
			segs := segments(p)
			for i, s := range segs {
				if _, ok := s.(sectionrecord.ArcSeg); ok {
					segs[i] = sectionrecord.CircleSeg{}
				}
			}
			p.Profile.Outer.Segments = segs
		}},
		{"two lines", func(p *revolvePayload) {
			segs := segments(p)
			for i, s := range segs {
				if _, ok := s.(sectionrecord.ArcSeg); ok {
					segs[i] = sectionrecord.LineSeg{}
				}
			}
			p.Profile.Outer.Segments = segs
		}},
		{"arc start parameter", editArc(func(a *sectionrecord.ArcSeg) { a.TStart = 0.25 })},
		{"arc end parameter", editArc(func(a *sectionrecord.ArcSeg) { a.TEnd = 0.75 })},
		{"line start parameter", editLine(func(l *sectionrecord.LineSeg) { l.TStart = 0.25 })},
		{"line end parameter", editLine(func(l *sectionrecord.LineSeg) { l.TEnd = 0.75 })},
		{"arc centre off the axis", editSpan(func(a *sectionrecord.ArcSeg) { a.Center.V, a.Start.V, a.End.V = 1, 1, 1 })},
		{"arc start off the centre row", editSpan(func(a *sectionrecord.ArcSeg) { a.Start.V = 1 })},
		{"arc end off the centre row", editSpan(func(a *sectionrecord.ArcSeg) { a.End.V = 1 })},
		{"arc start left of the centre", editSpan(func(a *sectionrecord.ArcSeg) { a.Start.U = -4 })},
		{"arc end right of the centre", editSpan(func(a *sectionrecord.ArcSeg) { a.End.U = 4 })},
		{"line off the arc ends", editLine(func(l *sectionrecord.LineSeg) { l.End.U += 1 })},
		{"non-finite centre", editArc(func(a *sectionrecord.ArcSeg) { a.Center.U = math.NaN() })},
		{"unequal radii", editSpan(func(a *sectionrecord.ArcSeg) { a.End.U = -3 })},
	}
	for _, tc := range edits {
		t.Run(tc.name, func(t *testing.T) {
			saved := body.n.Payload
			defer func() { body.n.Payload = saved }()
			p := good
			tc.edit(&p)
			body.n.Payload = p
			_, ok := contactround.SphereRecord(&body.n)
			require.False(t, ok)
		})
	}
	t.Run("not a revolve", func(t *testing.T) {
		saved := body.n.Payload
		defer func() { body.n.Payload = saved }()
		body.n.Payload = nil
		_, ok := contactround.SphereRecord(&body.n)
		require.False(t, ok)
		_, ok = contactround.SphereAtPose(&body.n, r3.Identity())
		require.False(t, ok)
	})
	t.Run("not a solid", func(t *testing.T) {
		body.n.Solid = false
		defer func() { body.n.Solid = true }()
		_, ok := contactround.SphereRecord(&body.n)
		require.False(t, ok)
	})
	t.Run("sheet kind", func(t *testing.T) {
		kind := body.n.Kind
		body.n.Kind = topo.BodySheet
		defer func() { body.n.Kind = kind }()
		_, ok := contactround.SphereRecord(&body.n)
		require.False(t, ok)
	})
	t.Run("placed off the signed axes", func(t *testing.T) {
		saved := body.n.Payload
		defer func() { body.n.Payload = saved }()
		p := good
		p.Xform = turn
		body.n.Payload = p
		_, ok := contactround.SphereAtPose(&body.n, r3.Identity())
		require.False(t, ok)
	})
	t.Run("pose", func(t *testing.T) {
		// A ball centred at the query origin keeps its centre under any
		// rotation, and takes the pose's translation.
		away, err := r3.RotationAround(r3.Vec{X: 9}, r3.Vec{Z: 1}, units.Degrees(30))
		require.NoError(t, err)
		moved, ok := contactround.SphereAtPose(&body.n, away)
		require.True(t, ok)
		require.Equal(t, proofarithVec(away.Translation()), moved.Center)
		plane, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		flip, err := r3.Reflection(plane)
		require.NoError(t, err)
		_, ok = contactround.SphereAtPose(&body.n, flip)
		require.False(t, ok)
		_, ok = contactround.SphereAtPose(&body.n, r3.Transform{})
		require.False(t, ok)
	})

	faces := topobuild.Faces(&body.n)
	faceEdits := []struct {
		name string
		edit func(*topo.Face) func()
	}{
		{"flat face", func(f *topo.Face) func() {
			s := f.Surface
			f.Surface = surfacegeom.Plane{}
			return func() { f.Surface = s }
		}},
		{"held normal", func(f *topo.Face) func() {
			f.NormalBound = 1
			return func() { f.NormalBound = 0 }
		}},
		{"reversed face", func(f *topo.Face) func() {
			f.Reversed = true
			return func() { f.Reversed = false }
		}},
		{"face with a loop", func(f *topo.Face) func() {
			saved := f.Loops
			f.Loops = []*topo.Loop{{}}
			return func() { f.Loops = saved }
		}},
	}
	for _, tc := range faceEdits {
		t.Run(tc.name, func(t *testing.T) {
			restore := tc.edit(faces[0])
			defer restore()
			_, ok := contactround.SphereRecord(&body.n)
			require.False(t, ok)
		})
	}
	t.Run("two faces", func(t *testing.T) {
		shell := body.n.Lumps[0].Shells[0]
		saved := shell.Faces
		shell.Faces = append(append([]*topo.Face(nil), saved...), saved[0])
		defer func() { shell.Faces = saved }()
		_, ok := contactround.SphereRecord(&body.n)
		require.False(t, ok)
	})
}

func internalRevolvedCylinderBody(t *testing.T, doc *Document, r, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b := s.CreatePoint(0, 0), s.CreatePoint(h, 0)
	c, d := s.CreatePoint(h, r), s.CreatePoint(0, r)
	for _, p := range []*sketch.Point{a, b, c, d} {
		s.Fix(p)
	}
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Revolve(t.Context(), doc, s, s.Profiles()[0], SketchLine{End: Point2{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestContactCylinderAdmissionGates builds a real prism cylinder and a real
// revolved cylinder, then breaks one fact of the payload, kind or faces at a
// time. Each break must make the cylinder reader refuse.
func TestContactCylinderAdmissionGates(t *testing.T) {
	doc := New()
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	tilt, err := r3.NewFrame(r3.Vec{}, r3.NewVec(math.Cos(0.5), math.Sin(0.5), 0), r3.NewVec(-math.Sin(0.5), math.Cos(0.5), 0))
	require.NoError(t, err)

	for _, tc := range []struct {
		name string
		body *Body
	}{
		{name: "prism", body: internalDiscBody(t, doc, 3, 8)},
		{name: "revolve", body: internalRevolvedCylinderBody(t, doc, 3, 8)},
	} {
		body := tc.body
		t.Run(tc.name, func(t *testing.T) {
			proof, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
			require.True(t, ok)
			require.NotNil(t, proof.Faces[0])
			require.NotNil(t, proof.Faces[1])
			require.NotSame(t, proof.Faces[0], proof.Faces[1])
			_, ok = contactround.CylinderAtPose(&body.n, turn)
			require.False(t, ok, "pose off the signed axes")

			saved := body.n.Payload
			defer func() { body.n.Payload = saved }()
			refuse := func(t *testing.T, payload topo.Payload) {
				t.Helper()
				body.n.Payload = payload
				defer func() { body.n.Payload = saved }()
				_, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
				require.False(t, ok)
			}
			switch p := saved.(type) {
			case prismPayload:
				circle := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
				withCircle := func(edit func(*sectionrecord.CircleSeg)) prismPayload {
					q, c := p, circle
					edit(&c)
					q.Profile.Outer.Segments = []sectionrecord.CurveSegment{c}
					return q
				}
				for name, q := range map[string]prismPayload{
					caseSurfaceResult:       func() prismPayload { q := p; q.SurfaceResult = true; return q }(),
					caseSectionDisplacement: func() prismPayload { q := p; q.SectionDelta = 1; return q }(),
					"near level":            func() prismPayload { q := p; q.Z0Delta = 1; return q }(),
					"far level":             func() prismPayload { q := p; q.Z1Delta = 1; return q }(),
					"frame off the axes":    func() prismPayload { q := p; q.Frame = tilt; return q }(),
					"placed off the axes":   func() prismPayload { q := p; q.Xform = turn; return q }(),
					"non-finite level":      func() prismPayload { q := p; q.Z1 = math.Inf(1); return q }(),
					"empty sweep":           func() prismPayload { q := p; q.Z1 = q.Z0; return q }(),
					"hole":                  func() prismPayload { q := p; q.Profile.Holes = []sectionrecord.LoopRecord{{}}; return q }(),
					"two segments": func() prismPayload {
						q := p
						q.Profile.Outer.Segments = []sectionrecord.CurveSegment{circle, circle}
						return q
					}(),
					"clockwise":       withCircle(func(c *sectionrecord.CircleSeg) { c.CCW = false }),
					"circle start":    withCircle(func(c *sectionrecord.CircleSeg) { c.TStart = 0.25 }),
					"circle end":      withCircle(func(c *sectionrecord.CircleSeg) { c.TEnd = 0.75 }),
					"non-finite":      withCircle(func(c *sectionrecord.CircleSeg) { c.Center.U = math.NaN() }),
					"no radius":       withCircle(func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(0) }),
					"radius not size": withCircle(func(c *sectionrecord.CircleSeg) { c.Radius = units.Degrees(3) }),
				} {
					t.Run(name, func(t *testing.T) { refuse(t, q) })
				}
				t.Run("not a circle", func(t *testing.T) {
					q := p
					q.Profile.Outer.Segments = []sectionrecord.CurveSegment{sectionrecord.LineSeg{}}
					refuse(t, q)
				})
			case revolvePayload:
				for name, q := range map[string]revolvePayload{
					caseSurfaceResult:       func() revolvePayload { q := p; q.SurfaceResult = true; return q }(),
					"partial revolution":    func() revolvePayload { q := p; q.Full = false; return q }(),
					caseSectionDisplacement: func() revolvePayload { q := p; q.SectionDelta = 1; return q }(),
					"frame off the axes":    func() revolvePayload { q := p; q.Frame = tilt; return q }(),
					"placed off the axes":   func() revolvePayload { q := p; q.Xform = turn; return q }(),
					"axis origin U bound":   func() revolvePayload { q := p; q.Ax.AUBound = 1; return q }(),
					"axis origin V bound":   func() revolvePayload { q := p; q.Ax.AVBound = 1; return q }(),
					"axis dir U bound":      func() revolvePayload { q := p; q.Ax.DUBound = 1; return q }(),
					"axis dir V bound":      func() revolvePayload { q := p; q.Ax.DVBound = 1; return q }(),
					"axis off the axes":     func() revolvePayload { q := p; q.Ax.DU, q.Ax.DV = 0.6, 0.8; return q }(),
					"non-finite axis":       func() revolvePayload { q := p; q.Ax.AU = math.NaN(); return q }(),
				} {
					t.Run(name, func(t *testing.T) { refuse(t, q) })
				}
			default:
				t.Fatalf("unexpected payload %T", saved)
			}

			t.Run("not a solid", func(t *testing.T) {
				body.n.Solid = false
				defer func() { body.n.Solid = true }()
				_, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
				require.False(t, ok)
			})
			t.Run("sheet kind", func(t *testing.T) {
				kind := body.n.Kind
				body.n.Kind = topo.BodySheet
				defer func() { body.n.Kind = kind }()
				_, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
				require.False(t, ok)
			})

			faces := topobuild.Faces(&body.n)
			for _, fe := range []struct {
				name string
				edit func(*topo.Face) func()
			}{
				{"held normal", func(f *topo.Face) func() {
					f.NormalBound = 1
					return func() { f.NormalBound = 0 }
				}},
				{"other surface", func(f *topo.Face) func() {
					s := f.Surface
					f.Surface = surfacegeom.Faceted{}
					return func() { f.Surface = s }
				}},
				{"flipped face", func(f *topo.Face) func() {
					f.Reversed = !f.Reversed
					return func() { f.Reversed = !f.Reversed }
				}},
			} {
				for i, f := range faces {
					if _, flat := f.Surface.(surfacegeom.Plane); fe.name == "flipped face" && !flat {
						continue // a wall has no side to swap
					}
					t.Run(fe.name+"/"+strconv.Itoa(i), func(t *testing.T) {
						defer fe.edit(f)()
						_, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
						require.False(t, ok)
					})
				}
			}
			t.Run("two faces", func(t *testing.T) {
				shell := body.n.Lumps[0].Shells[0]
				saved := shell.Faces
				shell.Faces = saved[:2]
				defer func() { shell.Faces = saved }()
				_, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
				require.False(t, ok)
			})
			t.Run("two planes and no wall", func(t *testing.T) {
				var wall *topo.Face
				for _, f := range faces {
					if _, ok := f.Surface.(surfacegeom.Cylinder); ok {
						wall = f
					}
				}
				require.NotNil(t, wall)
				plane := faces[0]
				for _, f := range faces {
					if _, ok := f.Surface.(surfacegeom.Plane); ok {
						plane = f
					}
				}
				s := wall.Surface
				wall.Surface = plane.Surface
				defer func() { wall.Surface = s }()
				_, ok := contactround.CylinderAtPose(&body.n, r3.Identity())
				require.False(t, ok)
			})
		})
	}
}
