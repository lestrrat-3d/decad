package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestCylEndSense reads which way the plane's and the wall's chains leave an
// edge end at the origin: the plane's along the tag line x, from its last
// vertex, and the wall's along the end circle's tangent ±y, from its first.
// A vertex within its bound of the corner leaves the sense undecided.
func TestCylEndSense(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	const bound = 0.01

	t.Run("the plane's chain", func(t *testing.T) {
		t.Parallel()
		dir, err := lineSense(x, r3.NewVec(5, 0, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, x, dir)
		dir, err = lineSense(x, r3.NewVec(-5, 0, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, x.Scale(-1), dir)
		_, err = lineSense(x, r3.NewVec(0.5*bound, 0, 0), r3.Vec{}, bound)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "too short a run")
	})

	t.Run("the wall's chain", func(t *testing.T) {
		t.Parallel()
		kind, err := wallSense(y, r3.NewVec(0.1, 1, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, trims, kind)
		kind, err = wallSense(y, r3.NewVec(0.1, -1, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, extends, kind)
		_, err = wallSense(y, r3.NewVec(0, 0.5*bound, 0), r3.Vec{}, bound)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "too short a run")
		// Farther than the bound, but at under minSine of its distance.
		_, err = wallSense(y, r3.NewVec(100, 0.05, 0), r3.Vec{}, bound)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}

// TestCylRemovedArea charges the corner region A at each end of a strip of
// area 10 and a band of area 4: a convex edge takes A out of an end face
// where the wall ends with the edge, and gives A to it as the cap where the
// wall extends past it; a concave edge fills A at both ends.
func TestCylRemovedArea(t *testing.T) {
	t.Parallel()
	const strip, band, corner = 10.0, 4.0, 0.5
	for _, c := range []struct {
		name   string
		convex bool
		extB   [2]bool
		want   float64
	}{
		{"a cap at both ends", true, [2]bool{true, true}, strip + band - 2*corner},
		{"a cap at one end, a corner at the other", true, [2]bool{true, false}, strip + band},
		{"a corner at both ends", true, [2]bool{}, strip + band + 2*corner},
		{"a concave edge", false, [2]bool{}, strip + band - 2*corner},
	} {
		require.Equal(t, c.want, cylRemovedArea(strip, band, corner, c.convex, c.extB), c.name)
	}
}

// TestCapSimple reads end caps in the plane z = 0 that run from P_A = (20, 0)
// along rows to the band's corner at index 2, back along the wall's cut to
// the corner V = 0, and along the plane's line to P_A.
func TestCapSimple(t *testing.T) {
	t.Parallel()
	read := func(pts ...[2]float64) error {
		w := &work{}
		loop := make([]int, len(pts))
		for k, p := range pts {
			w.verts = append(w.verts, r3.NewVec(p[0], p[1], 0))
			loop[k] = k
		}
		return w.capSimple(r3.NewVec(0, 0, 1), loop, 2)
	}
	require.NoError(t, read([2]float64{20, 0}, [2]float64{6, 6}, [2]float64{0, 20}, [2]float64{-1, 10}, [2]float64{}))
	// The wall's cut crosses the first row segment.
	err := read([2]float64{20, 0}, [2]float64{6, 6}, [2]float64{0, 20}, [2]float64{1, 12}, [2]float64{8, 4},
		[2]float64{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the blend's end cap folds")
	// The wall's cut leaves the band's corner straight on from the last row:
	// the cap does not close in a cusp there.
	err = read([2]float64{20, 0}, [2]float64{6, 6}, [2]float64{0, 20}, [2]float64{-3, 27}, [2]float64{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the blend's end cap folds")
}

// TestClearCusp spaces a fillet's rows again so the last row before tb lies
// need from the wall's circle: the keyway's top edge K = (3, √91) in (y, z)
// between the wall y = 3 and a shaft of radius 10 (the ball inside the
// circle), and a hub keyway's K = (2, √21) between the wall y = 2 and a bore
// of radius 5 (the ball outside it). Every row stays on the ball's circle,
// the last row lies need from the wall's circle, the other gaps keep the
// profile's step, and the returned sagitta is the last gap's.
func TestClearCusp(t *testing.T) {
	t.Parallel()
	x := r3.NewVec(1, 0, 0)
	for _, c := range []struct {
		name       string
		rad, w, r  float64
		matOutside bool
		wA, wB     r3.Vec
		need       float64
	}{
		{"a shaft", 10, 3, 0.3, false, r3.NewVec(0, 0, -1), r3.NewVec(0, math.Sqrt(91), -3).Scale(0.1), 0.0025},
		{"a hub", 5, 2, 0.1, true, r3.NewVec(0, 0, 1), r3.NewVec(0, math.Sqrt(21), -2).Scale(0.2), 0.002},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			k0 := r3.NewVec(0, c.w, math.Sqrt(c.rad*c.rad-c.w*c.w))
			cl := cylLine{a: plane{n: r3.NewVec(0, -1, 0)}, axis: x, radius: c.rad, matOutside: c.matOutside}
			spec := blendSpec{fillet: true, r: c.r}
			tau := c.r / 1024
			sec, err := cl.solveSection(spec, k0, r3.Vec{}, c.wA, c.wB, true, tau, tau)
			require.NoError(t, err)
			kappa := 1.0
			if !c.matOutside {
				kappa = -1
			}
			gap := func(p r3.Vec) float64 { return kappa * (p.Len() - c.rad) }
			uniform := append([]r3.Vec(nil), sec.rows...)
			require.Less(t, gap(uniform[len(uniform)-2]), c.need, "the profile's own last row lies nearer the circle")

			// A need the last row already meets leaves the rows.
			same := sec
			sag, err := cl.clearCusp(spec, &same, r3.Vec{}, true, gap(uniform[len(uniform)-2]), tau)
			require.NoError(t, err)
			require.Zero(t, sag)
			require.Equal(t, uniform, same.rows)

			sag, err = cl.clearCusp(spec, &sec, r3.Vec{}, true, c.need, tau)
			require.NoError(t, err)
			rows := sec.rows
			require.Equal(t, sec.ta, rows[0])
			require.Equal(t, sec.tb, rows[len(rows)-1])
			for _, p := range rows {
				require.InDelta(t, c.r, p.Sub(sec.centre).Len(), 1e-12)
			}
			require.InDelta(t, c.need, gap(rows[len(rows)-2]), 1e-12)
			angle := func(p, q r3.Vec) float64 {
				return math.Acos(math.Min(1, p.Sub(sec.centre).Dot(q.Sub(sec.centre))/(c.r*c.r)))
			}
			step := 2 * math.Acos(1-tau/c.r)
			for k := 1; k+1 < len(rows); k++ {
				require.LessOrEqual(t, angle(rows[k-1], rows[k]), step+1e-12)
			}
			psi := angle(rows[len(rows)-2], rows[len(rows)-1])
			require.Greater(t, psi, step)
			require.InDelta(t, c.r*(1-math.Cos(psi/2)), sag, 1e-12)
			t.Logf("%d rows against the profile's %d, the last gap %.4f rad against the step %.4f, sagitta %.3g mm "+
				"against τ %.3g", len(rows), len(uniform), psi, step, sag, tau)

			// A need past the plane's tangent point refuses.
			again := sec
			_, err = cl.clearCusp(spec, &again, r3.Vec{}, true, gap(sec.ta)+tau, tau)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, "the fillet's arc comes no more than")
		})
	}
}

// TestCuspChargeCoversSamples samples the segment from P to C' and the arc
// from P to C on a circle of radius 0.3, C' moved eps from C in 16
// directions, at 2,001 points each: every point of either lies within
// cuspCharge of the other. Where eps is below four sagittas the farthest
// point lies past eps, which a cut vertex's own bound alone states.
func TestCuspChargeCoversSamples(t *testing.T) {
	t.Parallel()
	const r, n = 0.3, 2000
	for _, half := range []float64{0.02, 0.07, 0.3} {
		sag := r * (1 - math.Cos(half))
		p := [2]float64{r * math.Cos(half), r * math.Sin(half)}
		cc := [2]float64{r * math.Cos(half), -r * math.Sin(half)}
		for _, f := range []float64{0, 0.5, 2, 4, 8} {
			eps := f * sag
			charge := cuspCharge(sag, eps, r)
			worst := 0.0
			for d := range 16 {
				th := 2 * math.Pi * float64(d) / 16
				c2 := [2]float64{cc[0] + eps*math.Cos(th), cc[1] + eps*math.Sin(th)}
				// The segment's points against the arc.
				for i := 0; i <= n; i++ {
					s := float64(i) / n
					q := [2]float64{p[0] + s*(c2[0]-p[0]), p[1] + s*(c2[1]-p[1])}
					a := math.Atan2(q[1], q[0])
					dist := math.Abs(math.Hypot(q[0], q[1]) - r)
					if math.Abs(a) > half {
						dist = math.Min(math.Hypot(q[0]-p[0], q[1]-p[1]), math.Hypot(q[0]-cc[0], q[1]-cc[1]))
					}
					worst = math.Max(worst, dist)
				}
				// The arc's points against the segment.
				ux, uy := c2[0]-p[0], c2[1]-p[1]
				for i := 0; i <= n; i++ {
					a := half - 2*half*float64(i)/n
					q := [2]float64{r * math.Cos(a), r * math.Sin(a)}
					s := math.Min(1, math.Max(0, ((q[0]-p[0])*ux+(q[1]-p[1])*uy)/(ux*ux+uy*uy)))
					worst = math.Max(worst, math.Hypot(q[0]-p[0]-s*ux, q[1]-p[1]-s*uy))
				}
			}
			require.LessOrEqual(t, worst, charge, "half angle %g, eps %g sagittas", half, f)
			if f < 4 {
				require.Greater(t, worst, eps, "half angle %g, eps %g sagittas", half, f)
			}
			t.Logf("half angle %g, eps %g sagittas: the farthest point at %.4f of the charge", half, f, worst/charge)
		}
	}
}
