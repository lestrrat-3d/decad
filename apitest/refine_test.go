package apitest

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// refineSpan extrudes a disk of radius r along a→c: a cylinder placed by a
// frame whose third axis is the span direction.
func refineSpan(t *testing.T, doc *decad.Document, a, c r3.Vec, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), r), r))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	d := c.Sub(a)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(d.Len()), Dir: decad.Along})
	require.NoError(t, err)
	n, _ := d.Normalize()
	ref := r3.Vec{Z: 1}
	if math.Abs(n.Z) > 0.9 {
		ref = r3.Vec{X: 1}
	}
	u, _ := ref.Cross(n).Normalize()
	f, err := r3.NewFrame(a, u, n.Cross(u))
	require.NoError(t, err)
	xf, err := r3.FromFrame(f)
	require.NoError(t, err)
	body, err = decad.Transform(t.Context(), body, xf)
	require.NoError(t, err)
	return body
}

// refineChain unions cylinders of radius r along the polyline pts, left to
// right, and returns the result with the closed-form volume of the union:
// each span's πr²L less, at every joint turning by α, the (4/3)r³·tan(α/2)
// the two butt-ended cylinders share.
func refineChain(t *testing.T, doc *decad.Document, pts []r3.Vec, r float64) (*decad.Body, float64) {
	t.Helper()
	var u *decad.Body
	want := 0.0
	for i := 0; i+1 < len(pts); i++ {
		d := pts[i+1].Sub(pts[i])
		want += math.Pi * r * r * d.Len()
		if i > 0 {
			prev, _ := pts[i].Sub(pts[i-1]).Normalize()
			cur, _ := d.Normalize()
			alpha := math.Acos(math.Max(-1, math.Min(1, prev.Dot(cur))))
			want -= 4.0 / 3.0 * r * r * r * math.Tan(alpha/2)
		}
		span := refineSpan(t, doc, pts[i], pts[i+1], r)
		if u == nil {
			u = span
			continue
		}
		var err error
		u, err = decad.Union(t.Context(), u, span)
		require.NoError(t, err)
	}
	return u, want
}

// leaderPoints is a six-span route of a fine thread through a guide: a
// collinear pair, then bends of different angles, all turned off the world
// axes and placed far from the origin.
func leaderPoints(scale float64) []r3.Vec {
	p := r3.Vec{X: 58, Z: 22}
	radius := 31.0
	d2 := p.Dot(p)
	tangent := p.Scale(radius * radius / d2).Sub(r3.Vec{X: -p.Z, Z: p.X}.Scale(radius * math.Sqrt(d2-radius*radius) / d2))
	wp := []r3.Vec{{Y: -405}, {Y: -335}, {Y: -227}, {Y: -227, Z: 22}, {X: 58, Y: -224, Z: 22}, {X: 58, Y: -160, Z: 22}, {X: tangent.X, Y: -160, Z: tangent.Z}}
	c, s := math.Cos(math.Pi/3), math.Sin(math.Pi/3)
	out := make([]r3.Vec, len(wp))
	for i, v := range wp {
		v = v.Scale(scale)
		out[i] = r3.Vec{X: c*v.X - s*v.Z - 140, Y: v.Y + 270, Z: s*v.X + c*v.Z + 690}
	}
	return out
}

func requireVolumeContains(t *testing.T, body *decad.Body, want float64) {
	t.Helper()
	vol, err := body.Volume()
	require.NoError(t, err)
	v, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	b, err := vol.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(v-want), b, "held volume %v ± %v must contain the closed form %v", v, b, want)
}

func TestRefineBentCylinderChains(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name          string
		pts           []r3.Vec
		r, max, chain float64
	}{
		{"leader", leaderPoints(1), 0.35, 0.12, 0.1},
		{"four bends, thicker", []r3.Vec{{X: 1, Y: 2, Z: 3}, {X: 41, Y: 2, Z: 3}, {X: 41, Y: 32, Z: 13}, {X: 11, Y: 47, Z: 13}, {X: 11, Y: 47, Z: 43}}, 0.5, 0.12, 0.2},
		{"right-angle pair", []r3.Vec{{X: 3, Y: 1, Z: 2}, {X: 33, Y: 1, Z: 2}, {X: 33, Y: 21, Z: 2}}, 0.35, 0.05, 0.1},
		{"oblique zigzag", []r3.Vec{{X: -5, Y: 2, Z: 1}, {X: 15, Y: 9, Z: 4}, {X: 22, Y: 30, Z: -3}, {X: 41, Y: 33, Z: 10}}, 1.2, 0.2, 0.1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.WithChainTolerance(units.Millimeters(tc.chain)))
			chain, want := refineChain(t, doc, tc.pts, tc.r)
			requireVolumeContains(t, chain, want)
			held, err := chain.Tessellate(t.Context(), units.Millimeters(1e3))
			require.NoError(t, err)
			before, err := held.Bound().In(units.Millimeter)
			require.NoError(t, err)
			srcVol, err := chain.Volume()
			require.NoError(t, err)

			refined, err := decad.Refine(t.Context(), chain, units.Millimeters(tc.max))
			require.NoError(t, err)
			m, err := refined.Tessellate(t.Context(), units.Millimeters(tc.max))
			require.NoError(t, err)
			after, err := m.Bound().In(units.Millimeter)
			require.NoError(t, err)
			require.LessOrEqual(t, after, tc.max)
			require.LessOrEqual(t, after, before)
			require.Equal(t, len(held.Triangles()), len(m.Triangles()), "Refine moves no facet")
			require.Equal(t, held.Vertices(), m.Vertices(), "Refine moves no vertex")
			gotVol, err := refined.Volume()
			require.NoError(t, err)
			require.Equal(t, srcVol, gotVol)
			requireVolumeContains(t, refined, want)

			_, err = decad.Refine(t.Context(), chain, units.Millimeters(tc.max))
			require.ErrorIs(t, err, decad.ErrRetiredBody, "Refine retires its source")
			ops := doc.OperationHistory().Operations()
			require.Equal(t, decad.OperationRefine, ops[len(ops)-1].Kind())
		})
	}
}

func TestRefineLeaderLiftsTheTessellateRefusal(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	chain, _ := refineChain(t, doc, leaderPoints(1), 0.35)
	_, err := chain.Tessellate(t.Context(), units.Millimeters(0.12))
	require.ErrorIs(t, err, decad.ErrUnsupported, "the held crossing bound refuses 0.12 mm before Refine")
	refined, err := decad.Refine(t.Context(), chain, units.Millimeters(0.12))
	require.NoError(t, err)
	m, err := refined.Tessellate(t.Context(), units.Millimeters(0.12))
	require.NoError(t, err)
	before, err := m.Bound().In(units.Millimeter)
	require.NoError(t, err)

	// A rigid placement after Refine keeps the surface bound, grown by the
	// rounding the placement charges each vertex.
	turn, err := r3.RotationAround(r3.Vec{X: -140, Y: 300, Z: 700}, r3.Vec{Y: -1}, units.Radians(0.05))
	require.NoError(t, err)
	turned, err := decad.Transform(t.Context(), refined, turn)
	require.NoError(t, err)
	tm, err := turned.Tessellate(t.Context(), units.Millimeters(0.12))
	require.NoError(t, err)
	after, err := tm.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.GreaterOrEqual(t, after, before)
	require.Less(t, after, before+1e-9)
}

func TestRefineRefusals(t *testing.T) {
	t.Parallel()
	pair := []r3.Vec{{X: 3, Y: 1, Z: 2}, {X: 33, Y: 1, Z: 2}, {X: 33, Y: 21, Z: 2}}

	t.Run("arguments", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		chain, _ := refineChain(t, doc, pair, 0.35)
		for _, tc := range []struct {
			bound units.Value
			want  error
		}{
			{units.Millimeters(0), decad.ErrDegenerate},
			{units.Millimeters(-1), decad.ErrNegativeMagnitude},
			{units.Millimeters(math.Inf(1)), decad.ErrNotFinite},
			{units.Degrees(1), decad.ErrUnitKind},
		} {
			_, err := decad.Refine(t.Context(), chain, tc.bound)
			require.ErrorIs(t, err, tc.want)
		}
		_, err := decad.Refine(t.Context(), nil, units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("a bound below the true-to-held floor changes nothing", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		chain, _ := refineChain(t, doc, pair, 0.35)
		n := len(doc.OperationHistory().Operations())
		_, err := decad.Refine(t.Context(), chain, units.Millimeters(1e-6))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Len(t, doc.OperationHistory().Operations(), n)
		_, err = decad.Refine(t.Context(), chain, units.Millimeters(0.05))
		require.NoError(t, err, "the refused call left the source live")
	})

	t.Run("outside the input class", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		// A single cylinder holds no Union mesh to refine.
		single := refineSpan(t, doc, r3.Vec{}, r3.Vec{X: 10}, 1)
		_, err := decad.Refine(t.Context(), single, units.Millimeters(0.1))
		require.ErrorIs(t, err, decad.ErrUnsupported)

		// A placed Union result drops its record: the placement moved the held
		// vertices the record was proven for.
		chain, _ := refineChain(t, doc, pair, 0.35)
		turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Radians(0.3))
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), chain, turn)
		require.NoError(t, err)
		_, err = decad.Refine(t.Context(), moved, units.Millimeters(0.1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// refineTorus revolves a disc of radius rho, centred radius from the Z axis
// in the XZ plane, a full turn about Z: a ring like a wire eye.
func refineTorus(t *testing.T, doc *decad.Document, radius, rho float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(radius, 0)
	s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), c, radius), sketch.NewVerticalDistance(s.Origin(), c, 0))
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(c, rho), rho))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], decad.ConstructionAxis{Dir: r3.Vec{Z: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// refineCord sweeps a disc of radius rho from start along +X, then round a
// quarter bend of radius 5 towards +Y and on.
func refineCord(t *testing.T, doc *decad.Document, start r3.Vec, rho float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), rho), rho))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.Vec{Z: 200}},
		decad.ArcAround{Center: r3.Vec{Y: 5, Z: 200}, Axis: r3.Vec{X: -1}, End: r3.Vec{Y: 5, Z: 205}},
		decad.LineTo{End: r3.Vec{Y: 30, Z: 205}})
	require.NoError(t, err)
	cord, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path)
	require.NoError(t, err)
	// Turn the path's +Z start onto +X at start.
	f, err := r3.NewFrame(start, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	xf, err := r3.FromFrame(f)
	require.NoError(t, err)
	cord, err = decad.Transform(t.Context(), cord, xf)
	require.NoError(t, err)
	return cord
}

func TestRefineTubeEyes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		tool  func(t *testing.T, doc *decad.Document) *decad.Body
		bound float64
	}{
		{"straight wire leaving a ring", func(t *testing.T, doc *decad.Document) *decad.Body {
			return refineSpan(t, doc, r3.Vec{X: 2.4}, r3.Vec{X: 202.4}, 0.6)
		}, 0.05},
		{"swept cord leaving a ring", func(t *testing.T, doc *decad.Document) *decad.Body {
			return refineCord(t, doc, r3.Vec{X: 2.3, Y: 0.05, Z: 0.07}, 0.6)
		}, 0.05},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
			joined, err := decad.Union(t.Context(), tc.tool(t, doc), refineTorus(t, doc, 2.4, 0.6))
			require.NoError(t, err)
			held, err := joined.Tessellate(t.Context(), units.Millimeters(1e3))
			require.NoError(t, err)
			before, err := held.Bound().In(units.Millimeter)
			require.NoError(t, err)

			refined, err := decad.Refine(t.Context(), joined, units.Millimeters(tc.bound))
			require.NoError(t, err)
			m, err := refined.Tessellate(t.Context(), units.Millimeters(tc.bound))
			require.NoError(t, err)
			after, err := m.Bound().In(units.Millimeter)
			require.NoError(t, err)
			require.LessOrEqual(t, after, tc.bound)
			require.LessOrEqual(t, after, before)
			require.Equal(t, held.Vertices(), m.Vertices(), "Refine moves no vertex")
		})
	}
}

func TestRefineBoredBend(t *testing.T) {
	t.Parallel()
	// Two thick legs joined at a bend, each crossed by a bore, like a flexible
	// connector: a Union, then two Cuts by extruded circles.
	build := func(t *testing.T) *decad.Body {
		doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
		toe := r3.Vec{X: 10.25, Y: -3.5, Z: 40.125}
		up, _ := r3.Vec{X: 1, Y: 0.2, Z: 0.3}.Normalize()
		legs, err := decad.Union(t.Context(),
			refineSpan(t, doc, toe.Sub(r3.Vec{Z: 16}), toe, 3),
			refineSpan(t, doc, toe, toe.Add(up.Scale(32)), 3))
		require.NoError(t, err)
		across, _ := r3.Vec{X: 0.2, Y: 1}.Normalize()
		lower := toe.Sub(r3.Vec{Z: 12})
		body, err := decad.Cut(t.Context(), legs, refineSpan(t, doc, lower.Sub(across.Scale(14)), lower.Add(across.Scale(14)), 2.2))
		require.NoError(t, err)
		side := up.Cross(across)
		upper := toe.Add(up.Scale(26))
		body, err = decad.Cut(t.Context(), body, refineSpan(t, doc, upper.Sub(side.Scale(14)), upper.Add(side.Scale(14)), 2.2))
		require.NoError(t, err)
		return body
	}
	body := build(t)
	_, err := body.Tessellate(t.Context(), units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrUnsupported, "the held crossing bound refuses 0.05 mm before Refine")
	// Far below the held bound and below the bores' own chording.
	refined, err := decad.Refine(t.Context(), body, units.Millimeters(0.01))
	require.NoError(t, err)
	m, err := refined.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	after, err := m.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, after, 0.01)
}

func TestRefineDrilledBar(t *testing.T) {
	t.Parallel()
	// A rectangular bar with a hole through its section, then cross-drilled
	// by a cylinder: a convex polygon section with a circular hole, then a Cut.
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 10)
	s.Fix(rect.A)
	c := s.CreatePoint(6, 5)
	s.Fix(c)
	s.CreateCircle(c, 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	bar, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	turn, err := r3.RotationAround(r3.Vec{X: 20, Y: 5, Z: 5}, r3.Vec{X: 1, Y: 2, Z: -1}, units.Radians(0.4))
	require.NoError(t, err)
	bar, err = decad.Transform(t.Context(), bar, turn)
	require.NoError(t, err)
	drill := refineSpan(t, doc, turn.Apply(r3.Vec{X: 30, Y: -5, Z: 5}), turn.Apply(r3.Vec{X: 30, Y: 15, Z: 5}), 1.5)
	drilled, err := decad.Cut(t.Context(), bar, drill)
	require.NoError(t, err)
	held, err := drilled.Tessellate(t.Context(), units.Millimeters(1e3))
	require.NoError(t, err)
	before, err := held.Bound().In(units.Millimeter)
	require.NoError(t, err)

	refined, err := decad.Refine(t.Context(), drilled, units.Millimeters(0.01))
	require.NoError(t, err)
	m, err := refined.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	after, err := m.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, after, 0.01)
	require.LessOrEqual(t, after, before)
	require.Equal(t, held.Vertices(), m.Vertices(), "Refine moves no vertex")
}

func TestRefineLetsAChainContinue(t *testing.T) {
	t.Parallel()
	// Without refining, an oblique zigzag's fold is refused at its second
	// Union: the new contact touches a rim whose crossing bound is past the
	// document's 0.02 mm chain tolerance. Refining each held result first lets
	// the next Union read the surface bound instead, and the fold completes.
	const chain = 0.02
	pts := []r3.Vec{{X: -5, Y: 2, Z: 1}, {X: 15, Y: 9, Z: 4}, {X: 22, Y: 30, Z: -3}, {X: 41, Y: 33, Z: 10}}
	fold := func(t *testing.T, refine bool) error {
		doc := decad.New(decad.WithChainTolerance(units.Millimeters(chain)))
		u := refineSpan(t, doc, pts[0], pts[1], 0.5)
		for i := 1; i+1 < len(pts); i++ {
			next, err := decad.Union(t.Context(), u, refineSpan(t, doc, pts[i], pts[i+1], 0.5))
			if err != nil {
				return err
			}
			u = next
			if _, err := u.Tessellate(t.Context(), units.Millimeters(chain)); refine && err != nil {
				u, err = decad.Refine(t.Context(), u, units.Millimeters(chain))
				require.NoError(t, err)
			}
		}
		m, err := u.Tessellate(t.Context(), units.Millimeters(0.12))
		if err != nil {
			u, err = decad.Refine(t.Context(), u, units.Millimeters(0.12))
			require.NoError(t, err)
			m, err = u.Tessellate(t.Context(), units.Millimeters(0.12))
			require.NoError(t, err)
		}
		b, err := m.Bound().In(units.Millimeter)
		require.NoError(t, err)
		require.LessOrEqual(t, b, 0.12)
		return nil
	}
	require.ErrorIs(t, fold(t, false), decad.ErrUnsupported, "the unrefined fold is refused mid-chain")
	require.NoError(t, fold(t, true))
}
