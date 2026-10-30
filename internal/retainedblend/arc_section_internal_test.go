package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// endPlaneFixture is a blend of an arc rim about the z axis, radius R in the
// plane z = 0, whose rows end on the plane x = h. It samples the end curve
// without the route's code: the row at profile parameter s is the circle of
// radius rho(s) at height z(s), written out from the torus (or cone) of
// agent-docs/retained-edge-blend-design.md §4.2, and its point on x = h is
// found by bisecting the circle's angle.
type endPlaneFixture struct {
	R, kappa, side, h float64
	spec              blendSpec
}

func (f endPlaneFixture) row(s float64) (rho, z float64) {
	if f.spec.fillet {
		a := s * math.Pi / 2
		return f.R + f.kappa*f.spec.r*(1-math.Sin(a)), f.side * f.spec.r * (1 - math.Cos(a))
	}
	return f.R + f.kappa*f.spec.dA*(1-s), f.side * f.spec.dB * s
}

// point is the row's point on x = h with y > 0.
func (f endPlaneFixture) point(s float64) r3.Vec {
	rho, z := f.row(s)
	lo, hi := 0.0, math.Pi/2 // rho·cos falls from rho to 0 over [0, π/2]
	for range 200 {
		mid := (lo + hi) / 2
		if rho*math.Cos(mid) > f.h {
			lo = mid
		} else {
			hi = mid
		}
	}
	return r3.NewVec(f.h, rho*math.Sin(lo), z)
}

// worstChord is the largest distance, over 400 samples of each gap, of the end
// curve from the chord between the points of the rows at ss.
func (f endPlaneFixture) worstChord(ss []float64) float64 {
	worst := 0.0
	for j := 0; j+1 < len(ss); j++ {
		a, b := f.point(ss[j]), f.point(ss[j+1])
		for k := 1; k < 400; k++ {
			p := f.point(ss[j] + (ss[j+1]-ss[j])*float64(k)/400)
			ab := b.Sub(a)
			t := math.Min(1, math.Max(0, p.Sub(a).Dot(ab)/ab.Dot(ab)))
			worst = math.Max(worst, p.Sub(a.Add(ab.Scale(t))).Len())
		}
	}
	return worst
}

func (f endPlaneFixture) rim() arcRim {
	return arcRim{n: r3.NewVec(0, 0, 1), u: r3.NewVec(1, 0, 0), w: r3.NewVec(0, 1, 0),
		radius: f.R, kappa: f.kappa, side: f.side}
}

func (f endPlaneFixture) section(t *testing.T) endSection {
	t.Helper()
	sec, err := f.rim().section(f.spec, plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(f.h, 0, 0)})
	require.NoError(t, err)
	return sec
}

// uniform is m + 1 evenly spaced profile parameters.
func uniform(m int) []float64 {
	out := make([]float64, m+1)
	for j := range out {
		out[j] = float64(j) / float64(m)
	}
	return out
}

// TestEndSectionTangent pins the end curve's slope against the sampler: in
// the end plane the curve's v = side·z grows against u = −κ·X at
// tan ψ = slope(s)·X/ρ, with slope tan α for the torus and dB/dA for the cone.
func TestEndSectionTangent(t *testing.T) {
	t.Parallel()
	for _, f := range []endPlaneFixture{
		{R: 4, kappa: 1, side: -1, h: 3.5, spec: blendSpec{fillet: true, r: 2}},
		{R: 3, kappa: -1, side: 1, h: 0.4, spec: blendSpec{fillet: true, r: 2.5}},
		{R: 4, kappa: 1, side: -1, h: 3.5, spec: blendSpec{dA: 2, dB: 1}},
	} {
		sec := f.section(t)
		for _, s := range []float64{0.1, 0.3, 0.5, 0.7, 0.9} {
			const ds = 1e-6
			p0, p1 := f.point(s-ds), f.point(s+ds)
			du, dv := -f.kappa*(p1.Y-p0.Y), f.side*(p1.Z-p0.Z)
			rho, _ := f.row(s)
			x := f.point(s).Y
			want := sec.slope(s) * x / rho
			require.InEpsilon(t, want, dv/du, 1e-6, "s = %v", s)
			require.Positive(t, du)
			require.Positive(t, dv)
		}
	}
}

// TestEndSectionCertifiesMeridian reads an end plane through the axis, where
// the end curve is the tube's own circle: a chord of tube angle Δα has the
// sagitta r·(1 − cos(Δα/2)). certify refuses any chord whose sagitta exceeds
// tau and accepts one whose sagitta is nine tenths of it.
func TestEndSectionCertifiesMeridian(t *testing.T) {
	t.Parallel()
	const r = 2.0
	tau := math.Ldexp(r, -10)
	f := endPlaneFixture{R: 5, kappa: 1, side: -1, h: 0, spec: blendSpec{fillet: true, r: r}}
	sec := f.section(t)
	// span is the profile parameter of the tube angle whose sagitta is k·tau.
	span := func(k float64) float64 { return 2 * math.Acos(1-k*tau/r) / (math.Pi / 2) }
	for _, s0 := range []float64{0, 0.3, 0.9} {
		require.False(t, sec.certify(s0, s0+span(1.01), tau), "s0 = %v", s0)
		require.True(t, sec.certify(s0, s0+span(0.9), tau), "s0 = %v", s0)
		require.InEpsilon(t, 0.9*tau, f.worstChord([]float64{s0, s0 + span(0.9)}), 1e-3)
	}
}

// TestArcRowsFollowEndCurve sizes the rows of a hole mouth's torus, of a
// boss's torus and of a hole mouth's cone ending on a plane nearly tangent to
// the rows' circles: the sine of the angle between the plane and the circle
// is 0.2 at the wall's row of the hole and 0.1 at the plane's row of the boss.
// Spaced by the profile alone, the chords in the plane stray past tau; the
// rows arcRows returns keep every chord within tau of the sampled curve, with
// at most 64 rows.
func TestArcRowsFollowEndCurve(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		f    endPlaneFixture
		size float64
		m0   int
	}{
		{"hole mouth fillet", endPlaneFixture{R: 4, kappa: 1, side: -1, h: 4 * math.Sqrt(1-0.2*0.2),
			spec: blendSpec{fillet: true, r: 4}}, 4, 18},
		{"boss fillet", endPlaneFixture{R: 3, kappa: -1, side: -1, h: 0.5 * math.Sqrt(1-0.1*0.1),
			spec: blendSpec{fillet: true, r: 2.5}}, 2.5, 18},
		{"hole mouth chamfer", endPlaneFixture{R: 4, kappa: 1, side: -1, h: 4 * math.Sqrt(1-0.2*0.2),
			spec: blendSpec{dA: 4, dB: 4}}, 4, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			tau := math.Ldexp(tc.size, -10)
			require.Greater(t, tc.f.worstChord(uniform(tc.m0)), tau, "the profile's own rows")
			rows, err := arcRows(tc.f.spec, tau, []endSection{tc.f.section(t)})
			require.NoError(t, err)
			require.LessOrEqual(t, tc.f.worstChord(rows), tau)
			require.LessOrEqual(t, len(rows)-1, 64)
			require.Equal(t, 0.0, rows[0])
			require.Equal(t, 1.0, rows[len(rows)-1])
		})
	}

	t.Run("an end plane through the axis adds no rows", func(t *testing.T) {
		t.Parallel()
		f := endPlaneFixture{R: 4, kappa: 1, side: -1, h: 0, spec: blendSpec{fillet: true, r: 4}}
		rows, err := arcRows(f.spec, math.Ldexp(4, -10), []endSection{f.section(t)})
		require.NoError(t, err)
		require.Equal(t, uniform(18), rows)
	})

	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		f := endPlaneFixture{R: 4, kappa: 1, side: -1, h: 3.5, spec: blendSpec{fillet: true, r: 4}}
		_, err := sectionRows(2, 1e-12, []endSection{f.section(t)})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "needs more than 1024 rows")
		// The plane's setback circle of radius 3 − 2 = 1 never reaches x = 1.5.
		boss := endPlaneFixture{R: 3, kappa: -1, side: -1, h: 1.5, spec: blendSpec{fillet: true, r: 2}}
		_, err = boss.rim().section(boss.spec, plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(1.5, 0, 0)})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "does not reach an end face")
	})
}

// TestShallowRefusesNaN reads the grazing-angle gates: a sine that is not a
// number refuses, as one under minSine does.
func TestShallowRefusesNaN(t *testing.T) {
	t.Parallel()
	require.True(t, shallow(math.NaN()))
	require.True(t, shallow(minSine/2))
	require.False(t, shallow(minSine))
	require.False(t, shallow(1))
	x, y := plane{n: r3.NewVec(1, 0, 0)}, plane{n: r3.NewVec(0, 1, 0)}
	_, ok := intersect3(x, y, plane{n: r3.NewVec(0, 0, math.NaN())})
	require.False(t, ok, "a plane with no normal meets no corner")
	_, ok = intersect3(x, y, plane{n: r3.NewVec(0, 0, 1)})
	require.True(t, ok)
	require.True(t, shallow(endSine(1, r3.NewVec(1, 0, 0), plane{n: r3.NewVec(math.NaN(), 0, 0)})),
		"a NaN normal reaches the corner route's sine")
}
