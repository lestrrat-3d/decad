package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The hand-built plans of agent-docs/prism-boolean-design.md Part R (Test plan): F2 and F4's
// sequential chains refuse, F1's builds because its second hole lands on the first hole's
// retained cylinder wall, and each regrouped plan builds through public calls.
// The test pins the premises the regroup depends on, namely which steps take the mesh path.

// regroupBoxVolume is the volume of retentionScene.roundedBox: 40x20x20 with four vertical
// edges filleted at r=3.
var regroupBoxVolume = (800 - 4*(1-math.Pi/4)*9) * 20

// regroupLens is the area of two intersecting discs' overlap.
func regroupLens(r1, r2, d float64) float64 {
	return r1*r1*math.Acos((d*d+r1*r1-r2*r2)/(2*d*r1)) +
		r2*r2*math.Acos((d*d+r2*r2-r1*r1)/(2*d*r2)) -
		0.5*math.Sqrt((-d+r1+r2)*(d+r1-r2)*(d-r1+r2)*(d+r1+r2))
}

// regroupCrossVolume is the volume shared by two Ø8 and Ø6 cylinders with crossing axes:
// 36 times the integral of cos²φ·√(16−9·sin²φ) over [−π/2, π/2], by the trapezoid rule on a
// smooth, π-periodic integrand.
func regroupCrossVolume(n int) float64 {
	h := math.Pi / float64(n)
	sum := 0.0
	for i := range n {
		phi := -math.Pi/2 + float64(i)*h
		c, s := math.Cos(phi), math.Sin(phi)
		sum += c * c * math.Sqrt(16-9*s*s)
	}
	return 36 * sum * h
}

// regroupCylinderZ is t3: a circle r=3 at (20, 10) on XY, extruded 20 along.
func regroupCylinderZ(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(20, 10)
	s.Fix(c)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// regroupSlabC is C: a rectangle x 18..26, y 2..18 on XY, extruded 20 along.
func regroupSlabC(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(18, 2, 26, 18)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

func requireRegroupResult(t *testing.T, got *decad.Body, want float64) {
	t.Helper()
	require.True(t, tookMeshPath(got), "the one mesh step is honest")
	requireBodyWatertight(t, got)
	v, err := got.Volume()
	require.NoError(t, err)
	mag := volumeMM(t, v)
	bound := v.Bound.Mag()
	require.InDelta(t, want, mag, bound+1e-6, "volume %v bound %v", mag, bound)
	require.Less(t, bound, 1e-3*want, "the volume bound stays usable")
}

func TestRegroupPlansBuildByHand(t *testing.T) {
	t.Parallel()

	t.Run("F1 two overlapping parallel holes", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(sc.roundedBox(t), sc.tool(t, 20)).Do(t.Context())
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		// The second hole's rim reads the first hole's cylinder slab, not the first rim's
		// bound (agent-docs/faceted-source-retention-design.md §11, Claim C).
		want := regroupBoxVolume - 20*(25*math.Pi-regroupLens(4, 3, 2))
		sequential, err := decad.Cut(held, slabYTool(t, sc, 22, 3)).Do(t.Context())
		require.NoError(t, err)
		requireRegroupResult(t, sequential, want)

		sc = newRetentionScene()
		box, t1, t2 := sc.roundedBox(t), sc.tool(t, 20), slabYTool(t, sc, 22, 3)
		u, err := decad.Union(t1, t2).Do(t.Context())
		require.NoError(t, err)
		require.False(t, tookMeshPath(u), "Union(t1, t2) is analytic")
		got, err := decad.Cut(box, u).Do(t.Context())
		require.NoError(t, err)
		requireRegroupResult(t, got, want)
	})

	t.Run("F2 a straight hole into an earlier hole", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(sc.roundedBox(t), sc.tool(t, 20)).Do(t.Context())
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(held, regroupCylinderZ(t, sc)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		box, t1, t3 := sc.roundedBox(t), sc.tool(t, 20), regroupCylinderZ(t, sc)
		step, err := decad.Cut(box, t3).Do(t.Context())
		require.NoError(t, err)
		require.False(t, tookMeshPath(step), "Cut(box, t3) is analytic")
		got, err := decad.Cut(step, t1).Do(t.Context())
		require.NoError(t, err)
		vx := regroupCrossVolume(4096)
		require.InDelta(t, vx, regroupCrossVolume(2048), 1e-12)
		requireRegroupResult(t, got, regroupBoxVolume-320*math.Pi-180*math.Pi+vx)
	})

	t.Run("F3 a box beside a plate carrying a boss", func(t *testing.T) {
		t.Parallel()
		build := func(t *testing.T) (plate, box2, boss *decad.Body) {
			doc := decad.New(decad.ExactOnly())
			plate = boxBody(t, doc, 0, 0, 40, 20, 10)
			box2 = boxBody(t, doc, 40, 0, 60, 20, 10)
			boss = translated(t, boxBodyAtZ(t, doc, 10, 8, 14, 12, 5, 20), 0.125, 0, 0)
			return plate, box2, boss
		}
		plate, box2, boss := build(t)
		first, err := decad.Union(plate, boss).Do(t.Context())
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is not a regroup. The copy of an axis-square result keeps
		// its exact vertices, so the grid route joins it with the box where the sequential gate once refused.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		joined, err := decad.Union(held, box2).Do(t.Context())
		require.NoError(t, err)
		requireRegroupResult(t, joined, 12240)

		plate, box2, boss = build(t)
		step, err := decad.Union(plate, box2).Do(t.Context())
		require.NoError(t, err)
		require.False(t, tookMeshPath(step), "Union(plate, box2) is analytic")
		got, err := decad.Union(step, boss).Do(t.Context())
		require.NoError(t, err)
		requireRegroupResult(t, got, 12240)
	})

	t.Run("F4 an analytic intersect after a mesh cut", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(sc.roundedBox(t), sc.tool(t, 20)).Do(t.Context())
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Intersect(held, regroupSlabC(t, sc)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		box, t1, c := sc.roundedBox(t), sc.tool(t, 20), regroupSlabC(t, sc)
		step, err := decad.Intersect(box, c).Do(t.Context())
		require.NoError(t, err)
		require.False(t, tookMeshPath(step), "Intersect(box, C) is analytic")
		got, err := decad.Cut(step, t1).Do(t.Context())
		require.NoError(t, err)
		requireRegroupResult(t, got, 2560-16*(32*math.Pi/3+4*math.Sqrt(3)))
	})

	// D1 and D4 cut a tapered first hole: its Cone wall states no slab thinner than the
	// first rim's bound, so the sequential chain refuses.
	t.Run("D1 a Copy keeps the sequential refusal", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(sc.roundedBox(t), sc.taperedTool(t)).Do(t.Context())
		require.NoError(t, err)
		cp, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(cp, slabYTool(t, sc, 22, 3)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("D4 a tilted second hole stays on the mesh path", func(t *testing.T) {
		t.Parallel()
		tilted := func(t *testing.T, sc *retentionScene) *decad.Body {
			tr, err := r3.RotationAround(r3.NewVec(22, 10, 10), r3.NewVec(1, 0, 0), units.Radians(0.01))
			require.NoError(t, err)
			b, err := decad.Transform(t.Context(), slabYTool(t, sc, 22, 3), tr)
			require.NoError(t, err)
			return b
		}
		sc := newRetentionScene()
		first, err := decad.Cut(sc.roundedBox(t), sc.taperedTool(t)).Do(t.Context())
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(held, tilted(t, sc)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		u, err := decad.Union(sc.tool(t, 20), tilted(t, sc)).Do(t.Context())
		require.NoError(t, err)
		require.True(t, tookMeshPath(u), "Union(t1, t2') takes the mesh path")
	})
}
