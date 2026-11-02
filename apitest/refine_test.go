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
	_, err = refined.Tessellate(t.Context(), units.Millimeters(0.12))
	require.NoError(t, err)
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
