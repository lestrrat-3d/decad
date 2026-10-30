package retainedblend

import (
	"math"
	"math/big"
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
		// A fillet of 10⁻⁶ on a 1,000 mm bore whose end plane meets the wall's
		// row at the sine 10⁻³, the least readEnd admits: certify's rounding
		// margin alone passes tau = 2⁻¹⁰·r.
		const big, tiny = 1000.0, 1e-6
		grazing := endPlaneFixture{R: big, kappa: 1, side: -1, h: big * math.Sqrt(1-minSine*minSine),
			spec: blendSpec{fillet: true, r: tiny}}
		sec := grazing.section(t)
		require.GreaterOrEqual(t, sec.margin, math.Ldexp(tiny, -10))
		_, err = arcRows(grazing.spec, math.Ldexp(tiny, -10), []endSection{sec})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the blend's end curve on an end face needs more than 1024 rows")
		// The plane's setback circle of radius 3 − 2 = 1 never reaches x = 1.5.
		boss := endPlaneFixture{R: 3, kappa: -1, side: -1, h: 1.5, spec: blendSpec{fillet: true, r: 2}}
		_, err = boss.rim().section(boss.spec, plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(1.5, 0, 0)})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "does not reach an end face")
	})
}

// TestEndSectionMarginCoversRounding measures, in 200-bit arithmetic, the
// rounding of the two computations certify's margin stands for, at every row
// arcRows builds and at 64 points of each gap: certify's X = √(ρ² − h²), and
// the row vertex the route places with ringAzimuths and atTheta, both against
// the exact point of the plane x = h at the same radius and height. Each stays
// within a quarter of the margin, from a plane through the axis down to one
// meeting a row's circle at the sine 10⁻⁶.
func TestEndSectionMarginCoversRounding(t *testing.T) {
	t.Parallel()
	exact := func(rho, h float64) *big.Float {
		r2 := new(big.Float).SetPrec(200).Mul(big.NewFloat(rho), big.NewFloat(rho))
		r2.Sub(r2, new(big.Float).SetPrec(200).Mul(big.NewFloat(h), big.NewFloat(h)))
		return r2.Sqrt(r2)
	}
	diff := func(a float64, b *big.Float) float64 {
		d, _ := new(big.Float).SetPrec(200).Sub(big.NewFloat(a), b).Float64()
		return math.Abs(d)
	}
	var cases []endPlaneFixture
	for _, q := range []float64{1, 0.2, 1e-2, 1e-3} {
		cases = append(cases, endPlaneFixture{R: 4, kappa: 1, side: -1, h: 4 * math.Sqrt(1-q*q),
			spec: blendSpec{fillet: true, r: 4}})
	}
	for _, q := range []float64{0.1, 1e-4, 1e-6} {
		cases = append(cases, endPlaneFixture{R: 3, kappa: -1, side: -1, h: 0.5 * math.Sqrt(1-q*q),
			spec: blendSpec{fillet: true, r: 2.5}})
	}
	for _, f := range cases {
		sec := f.section(t)
		tau := math.Ldexp(f.spec.r, -10)
		rows, err := arcRows(f.spec, tau, []endSection{sec})
		require.NoError(t, err, "h = %v", f.h)
		require.LessOrEqual(t, len(rows), maxSectionRows)
		a := f.rim()
		a.sign = 1
		g := plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(f.h, 0, 0)}
		worstX, worstV := 0.0, 0.0
		for j := 0; j+1 < len(rows); j++ {
			for k := range 64 {
				s := rows[j] + (rows[j+1]-rows[j])*float64(k)/64
				rho, z := sec.p(s)
				want := exact(rho, f.h)
				u, _, _ := sec.at(s)
				worstX = math.Max(worstX, diff(math.Abs(u), want))
				roots, ok := a.ringAzimuths(rho, g, math.Pi/2)
				require.True(t, ok)
				th := roots[0]
				if math.Sin(roots[1]) > math.Sin(th) {
					th = roots[1]
				}
				v := a.atTheta(rho, th, z)
				dx, dy, dz := math.Abs(v.X-f.h), diff(v.Y, want), math.Abs(v.Z-z)
				worstV = math.Max(worstV, math.Sqrt(dx*dx+dy*dy+dz*dz))
			}
		}
		t.Logf("h = %v: %d rows, margin %.3g, X rounding %.3g, vertex rounding %.3g",
			f.h, len(rows)-1, sec.margin, worstX, worstV)
		require.LessOrEqual(t, worstX, sec.margin/4, "h = %v", f.h)
		require.LessOrEqual(t, worstV, sec.margin/4, "h = %v", f.h)
	}
}

// TestChamferRowsHoldEndShift sizes a hole mouth's cone at the sine 0.2: the
// azimuth at which each row meets x = h, acos(h/ρ) from the plane's normal,
// moves by at most chamferColumn between neighbouring rows, and two such steps,
// a column and a shift, keep the cone's sagitta rhoHi·(1 − cos(step)) within
// tau. Without the shift the profile's rows alone let it move further.
func TestChamferRowsHoldEndShift(t *testing.T) {
	t.Parallel()
	const d = 4.0
	tau := math.Ldexp(d, -10)
	f := endPlaneFixture{R: 4, kappa: 1, side: -1, h: 4 * math.Sqrt(1-0.2*0.2), spec: blendSpec{dA: d, dB: d}}
	sec := f.section(t)
	step := chamferColumn(tau, sec.rhoHi)
	require.LessOrEqual(t, sec.rhoHi*(1-math.Cos(step)), tau)
	worst := func(rows []float64) float64 {
		w := 0.0
		for j := 0; j+1 < len(rows); j++ {
			r0, _ := f.row(rows[j])
			r1, _ := f.row(rows[j+1])
			w = math.Max(w, math.Abs(math.Acos(f.h/r1)-math.Acos(f.h/r0)))
		}
		return w
	}
	rows, err := arcRows(f.spec, tau, []endSection{sec})
	require.NoError(t, err)
	require.LessOrEqual(t, worst(rows), step)
	plain, err := sectionRows(2, tau, []endSection{sec})
	require.NoError(t, err)
	require.Greater(t, worst(plain), step)
}

// TestEndSectionTiltCharge reads an end plane tilted off the axis by
// ε = 10⁻⁹, the most readEnd admits. The route places each row's end as if
// the plane were parallel to the axis; on the tilted plane the row's circle
// meets it at x = h − ε·z/√(1 − ε²) instead, and the offset between the two
// points, measured in 200-bit arithmetic, stays within the margin's tilt
// term 2·ε·Z·(1 + 1/q). Where q is small enough that the term passes tau, the
// rows refuse.
func TestEndSectionTiltCharge(t *testing.T) {
	t.Parallel()
	const eps = 1e-9
	tilted := plane{n: r3.NewVec(math.Sqrt(1-eps*eps), 0, eps)}
	for _, q := range []float64{0.1, 1e-3} {
		f := endPlaneFixture{R: 3, kappa: -1, side: -1, h: 0.5 * math.Sqrt(1-q*q), spec: blendSpec{fillet: true, r: 2.5}}
		tilted.o = r3.NewVec(f.h, 0, 0)
		flat := f.section(t)
		sec, err := f.rim().section(f.spec, tilted)
		require.NoError(t, err)
		zHi, qMin := 2.5, math.Min(q, 1)
		require.InEpsilon(t, 2*eps*zHi*(1+1/qMin), sec.margin-flat.margin, 0.05, "q = %v", q)
		worst := 0.0
		for k := range 201 {
			rho, z := f.row(float64(k) / 200)
			prec := func(x float64) *big.Float { return new(big.Float).SetPrec(200).SetFloat64(x) }
			y := func(x *big.Float) *big.Float {
				r2 := new(big.Float).SetPrec(200).Mul(prec(rho), prec(rho))
				return r2.Sqrt(r2.Sub(r2, new(big.Float).SetPrec(200).Mul(x, x)))
			}
			xt := prec(f.h)
			xt.Sub(xt, new(big.Float).SetPrec(200).Quo(prec(eps*z), prec(math.Sqrt(1-eps*eps))))
			dy, _ := new(big.Float).SetPrec(200).Sub(y(xt), y(prec(f.h))).Float64()
			worst = math.Max(worst, math.Hypot(dy, eps*z))
		}
		require.LessOrEqual(t, worst, sec.margin-flat.margin, "q = %v", q)
	}
	f := endPlaneFixture{R: 3, kappa: -1, side: -1, h: 0.5 * math.Sqrt(1-1e-14), spec: blendSpec{fillet: true, r: 2.5}}
	tilted.o = r3.NewVec(f.h, 0, 0)
	sec, err := f.rim().section(f.spec, tilted)
	require.NoError(t, err)
	tau := math.Ldexp(2.5, -10)
	require.GreaterOrEqual(t, sec.margin, tau)
	_, err = arcRows(f.spec, tau, []endSection{sec})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "needs more than 1024 rows")
	_, err = arcRows(f.spec, tau, []endSection{f.section(t)})
	require.NoError(t, err, "the same plane parallel to the axis builds")
}
