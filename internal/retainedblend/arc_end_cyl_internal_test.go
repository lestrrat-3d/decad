package retainedblend

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// 200-bit arithmetic for the measurements below. big.Float has no
// trigonometry, so sine and cosine run as Taylor series (the arguments stay
// within a few radians) and the arc cosine by Newton's method from the float
// answer.
const bigPrec = 200

const bigPiText = "3.14159265358979323846264338327950288419716939937510582097494459230781640628620899"

func bf(x float64) *big.Float { return new(big.Float).SetPrec(bigPrec).SetFloat64(x) }

func bigPi() *big.Float {
	p, _, err := big.ParseFloat(bigPiText, 10, bigPrec, big.ToNearestEven)
	if err != nil {
		panic(err)
	}
	return p
}

func bnew() *big.Float { return new(big.Float).SetPrec(bigPrec) }

func badd(a, b *big.Float) *big.Float { return bnew().Add(a, b) }
func bsub(a, b *big.Float) *big.Float { return bnew().Sub(a, b) }
func bmul(a, b *big.Float) *big.Float { return bnew().Mul(a, b) }
func bquo(a, b *big.Float) *big.Float { return bnew().Quo(a, b) }
func bsqrt(a *big.Float) *big.Float   { return bnew().Sqrt(a) }

// bigSinCos is sine and cosine of x by Taylor series.
func bigSinCos(x *big.Float) (sin, cos *big.Float) {
	sin, cos = bnew(), bnew().SetInt64(1)
	term := bnew().SetInt64(1) // x^n / n!
	for n := 1; n < 200; n++ {
		term = bquo(bmul(term, x), bnew().SetInt64(int64(n)))
		if n > 4 && term.Sign() != 0 && term.MantExp(nil) < -bigPrec-8 {
			break
		}
		switch n % 4 {
		case 1:
			sin = badd(sin, term)
		case 2:
			cos = bsub(cos, term)
		case 3:
			sin = bsub(sin, term)
		case 0:
			cos = badd(cos, term)
		}
	}
	return sin, cos
}

// bigAcos is acos(y) for |y| < 1, by Newton's method on cos x = y from the
// float answer.
func bigAcos(y *big.Float) *big.Float {
	yf, _ := y.Float64()
	x := bf(math.Acos(yf))
	for range 5 {
		s, c := bigSinCos(x)
		x = badd(x, bquo(bsub(c, y), s))
	}
	return x
}

func bigDiff(a float64, b *big.Float) float64 {
	d, _ := bsub(bf(a), b).Float64()
	return math.Abs(d)
}

// cylEndFixture is a blend of an arc rim of radius R about the z axis in the
// plane z = 0 whose rows end on a cylinder of radius rG about the axis through
// (cx, cy), at distance d from the rim's axis. It writes out the profile and
// the end curve without the route's code, in 200-bit arithmetic.
type cylEndFixture struct {
	R, kappa, side, rG, d float64
	spec                  blendSpec
	sigma                 float64 // +1 end 0, -1 end 1
	eps                   float64 // G's tilt off the axis
}

func (f cylEndFixture) rim() arcRim {
	rm := rim{n: r3.NewVec(0, 0, 1), u: r3.NewVec(1, 0, 0), w: r3.NewVec(0, 1, 0),
		radius: f.R, kappa: f.kappa, side: f.side, scale: f.d + f.rG}
	return arcRim{rim: rm, sign: 1}
}

func (f cylEndFixture) end() *cylEnd {
	c := r3.NewVec(f.d, 0, 0)
	return &cylEnd{c: c, axis: r3.NewVec(0, 0, 1), rG: f.rG, d: f.d, sigma: f.sigma, eps: f.eps,
		sFar: farScale(f.d+f.rG, r3.Vec{}, c, f.R, f.rG, f.d)}
}

func (f cylEndFixture) section() (arcRim, endSection, error) {
	a := f.rim()
	a.cyl[0] = f.end()
	e, err := a.endSectionAt(f.spec, 0)
	return a, e, err
}

func (f cylEndFixture) tau() float64 {
	size := f.spec.r
	if !f.spec.fillet {
		size = max(f.spec.dA, f.spec.dB)
	}
	return math.Ldexp(size, -10)
}

// exactRow is the profile's circle at the exact parameter s: (ρ, z) and
// g = |dz/dρ| (nil at a fillet's s = 1, where it is infinite).
func (f cylEndFixture) exactRow(s float64) (rho, z, g *big.Float) {
	R, kappa, side := bf(f.R), bf(f.kappa), bf(f.side)
	if f.spec.fillet {
		r := bf(f.spec.r)
		sin, cos := bigSinCos(bquo(bmul(bf(s), bigPi()), bf(2)))
		rho = badd(R, bmul(bmul(kappa, r), bsub(bf(1), sin)))
		z = bmul(bmul(side, r), bsub(bf(1), cos))
		if s < 1 {
			g = bquo(sin, cos)
		}
		return rho, z, g
	}
	rho1 := badd(R, bmul(kappa, bf(f.spec.dA)))
	rho = badd(rho1, bmul(bf(s), bsub(R, rho1)))
	z = bmul(bf(s), bmul(side, bf(f.spec.dB)))
	return rho, z, bquo(bf(f.spec.dB), bf(f.spec.dA))
}

// psiOf is the exact crossing angle at G's axis for a row of radius rho.
func (f cylEndFixture) psiOf(rho *big.Float) *big.Float {
	d, rG := bf(f.d), bf(f.rG)
	num := bsub(badd(bmul(d, d), bmul(rG, rG)), bmul(rho, rho))
	return bigAcos(bquo(num, bmul(bf(2), bmul(d, rG))))
}

// alphaOf is the exact azimuth distance of the row's crossing from G's centre.
func (f cylEndFixture) alphaOf(rho *big.Float) *big.Float {
	d, rG := bf(f.d), bf(f.rG)
	num := bsub(badd(bmul(rho, rho), bmul(d, d)), bmul(rG, rG))
	return bigAcos(bquo(num, bmul(bf(2), bmul(rho, d))))
}

// dForSine is, for G of radius 2, the centre distance at which a rim circle of radius rho meets G's
// circle at the sine q of ∠OPC, from the side of the circle that is tangent
// from outside (outer, cos∠OPC < 0) or from inside.
func dForSine(rho, q float64, outer bool) float64 {
	const rG = 2.0
	c := math.Sqrt(1 - q*q)
	if outer {
		c = -c
	}
	return math.Sqrt(rho*rho + rG*rG - 2*rho*rG*c)
}

func TestCylEndCrossingSine(t *testing.T) {
	t.Parallel()
	const R, rG = 7.5, 2.0
	sFar := func(d float64) float64 { return farScale(d+rG, r3.Vec{}, r3.NewVec(d, 0, 0), R, rG, d) }
	sine := func(d float64) float64 { return crossingSine(R, rG, d, sFar(d), 0, 0) }
	// The fixture: cos V = −20.75/30.
	require.InDelta(t, math.Sqrt(1-20.75/30*20.75/30), sine(9), 1e-12)
	require.InDelta(t, 0.72, sine(9), 1e-2)
	require.Greater(t, sine(9), minSine)
	// Circles tangent from outside at d = R + rG and from inside at d = R − rG:
	// 10⁻⁷ short of either, the charged sine is under minSine.
	require.LessOrEqual(t, sine(9.5-1e-7), minSine)
	require.LessOrEqual(t, sine(5.5+1e-7), minSine)
	require.Greater(t, sine(9.5-1e-3), minSine)
	require.Greater(t, sine(5.5+1e-3), minSine)
	// Tilt and normal bounds are charged: eps + eps³ and the faces' bounds.
	require.InDelta(t, sine(9)-1e-4-1e-12, crossingSine(R, rG, 9, sFar(9), 1e-4, 0), 1e-13)
	require.InDelta(t, sine(9)-3e-4, crossingSine(R, rG, 9, sFar(9), 0, 3e-4), 1e-13)
	// No crossing at all.
	require.LessOrEqual(t, sine(12), 0.0)
	require.True(t, math.IsNaN(crossingSine(R, rG, math.NaN(), 1, 0, 0)))
}

// TestCylEndSideRule reads both ends of the fixture's arc: end 0 is the root at
// θ_c + α and end 1 the root at θ_c − α, with the arc, in the direction of
// growing θ, outside G's disk beside each, whichever way θ runs about the axis.
func TestCylEndSideRule(t *testing.T) {
	t.Parallel()
	const R, rG, d = 7.5, 2.0, 9.0
	alpha := math.Acos((R*R + d*d - rG*rG) / (2 * R * d))
	for _, sign := range []float64{1, -1} {
		for _, phiC := range []float64{0, 0.7, 2.9, -2.0} {
			f := cylEndFixture{R: R, kappa: 1, side: 1, rG: rG, d: d}
			a := f.rim()
			a.sign = sign
			c := r3.NewVec(d*math.Cos(phiC), d*math.Sin(phiC), 0)
			for i, sigma := range []float64{1, -1} {
				ce := &cylEnd{c: c, rG: rG, d: d, thetaC: sign * phiC, sigma: sigma}
				v, other := a.cylRoots(ce, 0)
				require.InDelta(t, rG, v.Sub(c).Len(), 1e-12)
				require.InDelta(t, rG, other.Sub(c).Len(), 1e-12)
				th, ok := ce.azimuth(R, 0)
				require.True(t, ok)
				require.InDelta(t, 0, v.Sub(a.atTheta(R, th, 0)).Len(), 1e-12)
				require.Greater(t, other.Sub(v).Len(), 1.0)
				// Beside the end, inward along the arc, the circle lies outside G:
				// the arc of end 0 grows in θ from V, the arc of end 1 shrinks to V.
				toward := th + sigma*0.01
				require.Greater(t, a.atTheta(R, toward, 0).Sub(c).Len(), rG, "sign %v end %d", sign, i)
				require.Less(t, a.atTheta(R, th-sigma*0.01, 0).Sub(c).Len(), rG)
				// The angle is the one within π of ref.
				for _, ref := range []float64{0, 2 * math.Pi, -2*math.Pi + 0.3, 7} {
					got, ok := ce.azimuth(R, ref)
					require.True(t, ok)
					require.LessOrEqual(t, math.Abs(got-ref), math.Pi)
					require.InDelta(t, 0, math.Remainder(got-(sign*phiC+sigma*alpha), 2*math.Pi), 1e-12)
				}
			}
		}
	}
	// A row circle that does not reach G has no azimuth.
	ce := &cylEnd{rG: rG, d: d, sigma: 1}
	_, ok := ce.azimuth(6.9, 0)
	require.False(t, ok)
	_, ok = ce.azimuth(11.1, 0)
	require.False(t, ok)
}

// TestCylEndRootBoundCoversRounding measures the roots V and V* the end reads
// against the exact meetings of the two circles, in 200-bit arithmetic, for
// the sine of the crossing from 0.72 down to twice minSine, from outside and
// from inside, on both ends and both senses of θ: each stays under e_V/4.
func TestCylEndRootBoundCoversRounding(t *testing.T) {
	t.Parallel()
	const R, rG = 7.5, 2.0
	worstRatio := 0.0
	for _, q := range []float64{0.72, 0.3, 0.1, 0.03, 0.01, 4e-3, 2 * minSine} {
		for _, outer := range []bool{true, false} {
			d := dForSine(R, q, outer)
			for _, sign := range []float64{1, -1} {
				for _, phiC := range []float64{0, 0.7, 2.9, -2.0} {
					for _, sigma := range []float64{1, -1} {
						cx, cy := d*math.Cos(phiC), d*math.Sin(phiC)
						f := cylEndFixture{R: R, kappa: 1, side: 1, rG: rG, d: d}
						a := f.rim()
						a.sign = sign
						dFloat := math.Hypot(cx, cy)
						c := r3.NewVec(cx, cy, 0)
						sFar := farScale(dFloat+rG, r3.Vec{}, c, R, rG, dFloat)
						sV := crossingSine(R, rG, dFloat, sFar, 0, 0)
						require.InDelta(t, q, sV, 1e-3*q+1e-9, "q = %v", q)
						ce := &cylEnd{c: c, rG: rG, d: dFloat, thetaC: sign * math.Atan2(cy, cx), sigma: sigma,
							sV: sV, sFar: sFar}
						ce.eV = rootBound(sFar, rG, sV, 0)
						v, other := a.cylRoots(ce, 0)
						// The exact roots: φ = φ_c ± α about the axis, in terms of
						// cos φ_c = cx/d, sin φ_c = cy/d with d exact.
						dEx := bsqrt(badd(bmul(bf(cx), bf(cx)), bmul(bf(cy), bf(cy))))
						cosC, sinC := bquo(bf(cx), dEx), bquo(bf(cy), dEx)
						num := bsub(badd(bmul(bf(R), bf(R)), bmul(dEx, dEx)), bmul(bf(rG), bf(rG)))
						sinA, cosA := bigSinCos(bigAcos(bquo(num, bmul(bf(2), bmul(bf(R), dEx)))))
						exactRoot := func(s float64) (*big.Float, *big.Float) {
							// φ = φ_c + s·α; cos, sin by the angle sum.
							sa := bmul(bf(s), sinA)
							x := bsub(bmul(cosC, cosA), bmul(sinC, sa))
							y := badd(bmul(sinC, cosA), bmul(cosC, sa))
							return bmul(bf(R), x), bmul(bf(R), y)
						}
						for k, got := range []r3.Vec{v, other} {
							s := sign * sigma
							if k == 1 {
								s = -s
							}
							ex, ey := exactRoot(s)
							dx, dy := bigDiff(got.X, ex), bigDiff(got.Y, ey)
							worst := math.Hypot(dx, dy)
							worstRatio = math.Max(worstRatio, worst/ce.eV)
							require.LessOrEqual(t, worst, ce.eV/4, "q = %v outer %v sign %v phi %v", q, outer, sign, phiC)
						}
					}
				}
			}
		}
	}
	t.Logf("the roots' worst rounding is %.3g of e_V", worstRatio)
}

// cylMarginCases are rim circles of a blend that end on G at crossings from the
// fixture's down to q = 10⁻³ at one end of the profile, tangent from outside or
// inside, and one whose cos∠OPC changes sign over the profile.
func cylMarginCases() map[string]cylEndFixture {
	fillet := blendSpec{fillet: true, r: 0.5}
	out := map[string]cylEndFixture{
		"the fixture":        {R: 7.5, kappa: 1, side: 1, rG: 2, d: 9, spec: fillet, sigma: 1},
		"the fixture, end 1": {R: 7.5, kappa: 1, side: 1, rG: 2, d: 9, spec: fillet, sigma: -1},
		"cos changes sign":   {R: 7.5, kappa: 1, side: 1, rG: 2, d: math.Sqrt(7.75*7.75 + 4), spec: fillet, sigma: 1},
		"a boss, below":      {R: 7.5, kappa: -1, side: -1, rG: 2, d: 6, spec: blendSpec{fillet: true, r: 1}, sigma: 1},
		"a chamfer":          {R: 9, kappa: 1, side: 1, rG: 2, d: 9, spec: blendSpec{dA: 0.5, dB: 0.4}, sigma: -1},
	}
	// q at the row of least radius (7.5): tangent from outside.
	for _, q := range []float64{0.1, 1e-2, 1e-3} {
		out["outer, q = "+fmtG(q)] = cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: dForSine(7.5, q, true),
			spec: fillet, sigma: 1}
		// q at the row of greatest radius (8): tangent from inside.
		out["inner, q = "+fmtG(q)] = cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: dForSine(8, q, false),
			spec: fillet, sigma: -1}
		out["boss outer, q = "+fmtG(q)] = cylEndFixture{R: 7.5, kappa: -1, side: -1, rG: 2, d: dForSine(7, q, true),
			spec: fillet, sigma: -1}
	}
	return out
}

func fmtG(x float64) string { return big.NewFloat(x).Text('g', 3) }

// TestCylEndSectionMarginCoversRounding measures, in 200-bit arithmetic, the
// rounding of what certify's margin and angle allowance stand for, at every
// row arcRows builds and at 64 points of each gap: u = −κ·r_G·ψ and the row
// vertex the route places (endAzimuths and atTheta), each against the exact
// meeting of the row's circle and G, stay within a quarter of the margin; the
// computed tangent angle atan(g·q) and the computed ψ, against the exact ones
// at the exact profile parameter, stay within a quarter of the angle allowance.
func TestCylEndSectionMarginCoversRounding(t *testing.T) {
	t.Parallel()
	for name, f := range cylMarginCases() {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			a, sec, err := f.section()
			require.NoError(t, err)
			tau := f.tau()
			rows, err := arcRows(f.spec, tau, arcProfileShare(tau), []endSection{sec})
			require.NoError(t, err)
			require.LessOrEqual(t, len(rows), maxSectionRows)
			var worstU, worstV, worstPsiT, worstPsi float64
			sample := func(s float64) {
				rho, z := sec.p(s)
				rhoB := bf(rho)
				// u: the computed ψ-based coordinate against the exact one for the
				// same computed ρ.
				u, _, q := sec.at(s)
				wantU := bmul(bf(-f.kappa*f.rG), f.psiOf(rhoB))
				worstU = math.Max(worstU, bigDiff(u, wantU))
				// The placed row vertex against the exact meeting.
				th, ok := a.endAzimuths(0, rho, 0)
				require.True(t, ok)
				v := a.atTheta(rho, th, z)
				sinA, cosA := bigSinCos(bmul(bf(f.sigma), f.alphaOf(rhoB)))
				dx, dy, dz := bigDiff(v.X, bmul(rhoB, cosA)), bigDiff(v.Y, bmul(rhoB, sinA)), math.Abs(v.Z-z)
				worstV = math.Max(worstV, math.Sqrt(dx*dx+dy*dy+dz*dz))
				// Angles, at the exact parameter.
				rhoX, _, g := f.exactRow(s)
				psiX := f.psiOf(rhoX)
				psiGot := sec.cyl.psi(rho)
				if d := bigDiff(psiGot, psiX); d > worstPsi {
					worstPsi = d
				}
				got := math.Atan(sec.slope(s) * q)
				if g == nil {
					worstPsiT = math.Max(worstPsiT, bigDiff(got, bquo(bigPi(), bf(2))))
					return
				}
				c := bquo(bsub(badd(bmul(rhoX, rhoX), bf(f.rG*f.rG)), bf(f.d*f.d)), bmul(bf(2), bmul(rhoX, bf(f.rG))))
				qX := bsqrt(bsub(bf(1), bmul(c, c)))
				x := bmul(g, qX)
				sinT, cosT := bigSinCos(bf(got))
				tanT := bquo(sinT, cosT)
				y, _ := bquo(bsub(tanT, x), badd(bf(1), bmul(tanT, x))).Float64()
				worstPsiT = math.Max(worstPsiT, math.Abs(y))
			}
			for j := 0; j+1 < len(rows); j++ {
				for k := range 64 {
					sample(rows[j] + (rows[j+1]-rows[j])*float64(k)/64)
				}
			}
			sample(1)
			t.Logf("%s: %d rows: margin %.3g (u %.3g, vertex %.3g); angle allowance %.3g (tangent %.3g, psi %.3g)",
				name, len(rows)-1, sec.margin, worstU, worstV, sec.angle, worstPsiT, worstPsi)
			require.LessOrEqual(t, worstU, sec.margin/4)
			require.LessOrEqual(t, worstV, sec.margin/4)
			require.LessOrEqual(t, worstPsiT, sec.angle/4)
			// curv's widening, twice the allowance, covers the rounding of each of
			// the two ψ it differences.
			require.LessOrEqual(t, worstPsi, sec.angle/4)
		})
	}
}

// tiltedMeeting is the exact meeting, in 200-bit arithmetic, of the circle of
// radius rho at height z about the z axis with the cylinder of radius rG about
// the line through c0 = (d, 0, 0) with unit direction (ax, ay, √(1 − ax² − ay²)),
// near azimuth phi0 (Newton's method); it returns the point's x and y.
func tiltedMeeting(rho, z float64, d, rG, ax, ay float64, phi0 *big.Float) (x, y *big.Float) {
	az := bsqrt(bsub(bf(1), badd(bmul(bf(ax), bf(ax)), bmul(bf(ay), bf(ay)))))
	rhoB, zB := bf(rho), bf(z)
	eval := func(phi *big.Float) (f, df *big.Float) {
		sin, cos := bigSinCos(phi)
		px, py := bsub(bmul(rhoB, cos), bf(d)), bmul(rhoB, sin)
		dot := badd(badd(bmul(px, bf(ax)), bmul(py, bf(ay))), bmul(zB, az))
		f = bsub(bsub(badd(badd(bmul(px, px), bmul(py, py)), bmul(zB, zB)), bmul(dot, dot)), bmul(bf(rG), bf(rG)))
		dpx, dpy := bmul(rhoB, bnew().Neg(sin)), bmul(rhoB, cos)
		ddot := badd(bmul(dpx, bf(ax)), bmul(dpy, bf(ay)))
		df = bsub(bmul(bf(2), badd(bmul(px, dpx), bmul(py, dpy))), bmul(bf(2), bmul(dot, ddot)))
		return f, df
	}
	phi := phi0
	for range 10 {
		f, df := eval(phi)
		phi = bsub(phi, bquo(f, df))
	}
	sin, cos := bigSinCos(phi)
	return bmul(rhoB, cos), bmul(rhoB, sin)
}

// TestCylEndSectionTiltCharge reads an end cylinder tilted off the axis by
// ε = 10⁻⁹, the most readCylEnd admits, in each of two directions. The route
// places each row's end as if G were parallel to the axis; on the tilted G the
// row's circle meets it elsewhere, and the offset between the two points,
// measured in 200-bit arithmetic, stays within the margin's tilt term
// 2·ε·(Z + r_G)·(1 + 1/q). Where q is small enough that the term passes τ, the
// rows refuse.
func TestCylEndSectionTiltCharge(t *testing.T) {
	t.Parallel()
	const eps = 1e-9
	for _, q := range []float64{0.72, 0.1, 1e-3} {
		d := 9.0
		if q != 0.72 {
			d = dForSine(7.5, q, true)
		}
		f := cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: d, spec: blendSpec{fillet: true, r: 0.5}, sigma: 1}
		_, flat, err := f.section()
		require.NoError(t, err)
		tilted := f
		tilted.eps = eps
		_, sec, err := tilted.section()
		require.NoError(t, err)
		_, _, q0 := sec.at(0)
		_, _, q1 := sec.at(1)
		qMin := math.Min(q0, q1)
		zHi := 0.5
		require.InEpsilon(t, 2*eps*(zHi+f.rG)*(1+1/qMin), sec.margin-flat.margin, 0.05, "q = %v", q)
		worst := 0.0
		for k := range 201 {
			rho, z := flat.p(float64(k) / 200)
			rhoB := bf(rho)
			alpha := f.alphaOf(rhoB)
			x0, y0 := bmul(rhoB, mustCos(alpha)), bmul(rhoB, mustSin(alpha))
			for _, dir := range [][2]float64{{eps, 0}, {-eps, 0}, {0, eps}} {
				x1, y1 := tiltedMeeting(rho, z, d, f.rG, dir[0], dir[1], alpha)
				dx, _ := bsub(x1, x0).Float64()
				dy, _ := bsub(y1, y0).Float64()
				worst = math.Max(worst, math.Hypot(dx, dy))
			}
		}
		t.Logf("q = %v: tilt offset %.3g against the term %.3g", q, worst, sec.margin-flat.margin)
		require.LessOrEqual(t, worst, sec.margin-flat.margin, "q = %v", q)
	}

	// A crossing so shallow that the tilt term alone passes τ refuses; the same
	// G parallel to the axis builds.
	f := cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: dForSine(7.5, 1e-5, true),
		spec: blendSpec{fillet: true, r: 0.5}, sigma: 1}
	_, flat, err := f.section()
	require.NoError(t, err)
	tau := f.tau()
	f.eps = 1e-9
	_, sec, err := f.section()
	require.NoError(t, err)
	require.Less(t, flat.margin, tau)
	require.GreaterOrEqual(t, sec.margin, tau)
	_, err = arcRows(f.spec, tau, arcProfileShare(tau), []endSection{sec})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "needs more than 1024 rows")
	_, err = arcRows(f.spec, tau, arcProfileShare(tau), []endSection{flat})
	require.NoError(t, err, "the same cylinder parallel to the axis builds")
}

func mustSin(x *big.Float) *big.Float { s, _ := bigSinCos(x); return s }
func mustCos(x *big.Float) *big.Float { _, c := bigSinCos(x); return c }

// meetingPoint is the point of the circle of radius rho at height z that lies
// on G's circle on the end's root side, found by bisecting the azimuth: the
// distance from c grows with the angle from θ_c.
func (f cylEndFixture) meetingPoint(rho, z float64) r3.Vec {
	lo, hi := 0.0, math.Pi
	for range 200 {
		mid := (lo + hi) / 2
		if math.Sqrt(rho*rho+f.d*f.d-2*rho*f.d*math.Cos(mid)) < f.rG {
			lo = mid
		} else {
			hi = mid
		}
	}
	th := f.sigma * lo
	return r3.NewVec(rho*math.Cos(th), rho*math.Sin(th), z)
}

// TestCylEndCertifyChargesCurvature checks the per-gap curvature term. On the
// chamfer below the unrolled end curve is nearly straight over a chord whose
// 3D chord still strays from the cylinder by r_G·Δψ²/2, far past τ: certify
// refuses it, and it would pass without the term. A short chord certifies.
// arcRows on the fixture's end then returns rows whose every gap's chord, in
// 3D, lies within τ of densely sampled exact end-curve points.
func TestCylEndCertifyChargesCurvature(t *testing.T) {
	t.Parallel()
	f := cylEndFixture{R: 9, kappa: 1, side: 1, rG: 2, d: 9, spec: blendSpec{dA: 0.5, dB: 0.5}, sigma: 1}
	_, sec, err := f.section()
	require.NoError(t, err)
	tau := f.tau()
	// The premise: over [0.4, 0.6] the unrolled curve lies within τ/2 of its
	// chord, sampled without certify.
	au, av, _ := sec.at(0.4)
	bu, bv, _ := sec.at(0.6)
	worst := 0.0
	for k := 1; k < 1000; k++ {
		u, v, _ := sec.at(0.4 + 0.2*float64(k)/1000)
		worst = math.Max(worst, math.Abs((bu-au)*(v-av)-(bv-av)*(u-au))/math.Hypot(bu-au, bv-av))
	}
	require.Less(t, worst, tau/2)
	curv := sec.curv(0.4, 0.6)
	require.Greater(t, curv, 4*tau)
	require.False(t, sec.certify(0.4, 0.6, tau))
	// With τ raised by the term alone, the chord passes: the unrolled distance
	// and the margin were within τ already.
	require.True(t, sec.certify(0.4, 0.6, tau+curv*(1+1e-9)))
	require.True(t, sec.certify(0.49, 0.51, tau))
	require.Less(t, sec.curv(0.49, 0.51), tau/4)

	// The fixture's own rows.
	g := cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: 9, spec: blendSpec{fillet: true, r: 0.5}}
	for _, sigma := range []float64{1, -1} {
		g.sigma = sigma
		a, sec, err := g.section()
		require.NoError(t, err)
		tau := g.tau()
		rows, err := arcRows(g.spec, tau, arcProfileShare(tau), []endSection{sec})
		require.NoError(t, err)
		vertex := func(s float64) r3.Vec {
			rho, z := sec.p(s)
			th, ok := a.endAzimuths(0, rho, 0)
			require.True(t, ok)
			return a.atTheta(rho, th, z)
		}
		worst := 0.0
		for j := 0; j+1 < len(rows); j++ {
			p, q := vertex(rows[j]), vertex(rows[j+1])
			for k := 1; k < 400; k++ {
				s := rows[j] + (rows[j+1]-rows[j])*float64(k)/400
				rho, z := sec.p(s)
				x := g.meetingPoint(rho, z)
				pq := q.Sub(p)
				t := math.Min(1, math.Max(0, x.Sub(p).Dot(pq)/pq.Dot(pq)))
				worst = math.Max(worst, x.Sub(p.Add(pq.Scale(t))).Len())
			}
		}
		t.Logf("sigma %v: %d rows, the worst 3D chord distance is %.4f of tau", sigma, len(rows)-1, worst/tau)
		require.LessOrEqual(t, worst, tau)
		require.Greater(t, worst, tau/20, "the sampler sees the curve")
	}
}

// TestCylEndCuspParam places the fillet's last row before a wall cut at the
// parameter where the end curve on G lies need from the wall's generator, along
// G's circle: r_G·|ψ(s) − ψ(1)| = need.
func TestCylEndCuspParam(t *testing.T) {
	t.Parallel()
	for _, kappa := range []float64{1, -1} {
		f := cylEndFixture{R: 7.5, kappa: kappa, side: 1, rG: 2, d: 9, spec: blendSpec{fillet: true, r: 0.5}, sigma: 1}
		if kappa < 0 {
			f.d = 8
		}
		a, sec, err := f.section()
		require.NoError(t, err)
		psi1 := sec.cyl.psi(f.R)
		for _, need := range []float64{0.002, 0.01} {
			s, err := a.cuspParam(f.spec, sec, need)
			require.NoError(t, err)
			require.Greater(t, s, 0.0)
			require.Less(t, s, 1.0)
			rho, _ := sec.p(s)
			require.InDelta(t, need, f.rG*math.Abs(sec.cyl.psi(rho)-psi1), 1e-9)
			// Nearer the wall's row the curve is closer to the generator.
			rho, _ = sec.p(s + 0.01)
			require.Less(t, f.rG*math.Abs(sec.cyl.psi(rho)-psi1), need)
		}
		_, err = a.cuspParam(f.spec, sec, 10)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	}
}

// TestCylEndSectionRefuses covers the section's gates and the shift.
func TestCylEndSectionRefuses(t *testing.T) {
	t.Parallel()
	fillet := blendSpec{fillet: true, r: 0.5}
	// A row circle that does not reach G.
	f := cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: 9.6, spec: fillet, sigma: 1}
	_, _, err := f.section()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "does not reach an end face")
	// ρ² + d² ≤ r_G² somewhere on the profile.
	f = cylEndFixture{R: 3, kappa: 1, side: 1, rG: 6, d: 4, spec: fillet, sigma: 1}
	_, _, err = f.section()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	// shift is the change in the row's azimuth.
	f = cylEndFixture{R: 7.5, kappa: 1, side: 1, rG: 2, d: 9, spec: fillet, sigma: 1}
	a, sec, err := f.section()
	require.NoError(t, err)
	r0, _ := sec.p(0.2)
	r1, _ := sec.p(0.7)
	t0, _ := a.endAzimuths(0, r0, 0)
	t1, _ := a.endAzimuths(0, r1, 0)
	require.InDelta(t, math.Abs(t1-t0), sec.shift(0.2, 0.7), 1e-12)
}

// TestReadCylEnd reads an end from a synthetic tag and held vertex: the gates
// in order, and the root the side rule names.
func TestReadCylEnd(t *testing.T) {
	t.Parallel()
	const R, rG, d = 7.5, 2.0, 9.0
	rm := rim{n: r3.NewVec(0, 0, 1), u: r3.NewVec(1, 0, 0), w: r3.NewVec(0, 1, 0), radius: R,
		kappa: 1, side: 1, scale: 12, wallFace: &topo.Face{}}
	a := arcRim{rim: rm, sign: 1}
	alpha := math.Acos((R*R + d*d - rG*rG) / (2 * R * d))
	held := r3.NewVec(R*math.Cos(alpha), R*math.Sin(alpha), 0)
	fp := facetedrecord.Payload{Verts: []r3.Vec{held}, VertexBound: []float64{1e-9}}
	face := func(mod func(*topo.Face, *surfacegeom.Cylinder)) *topo.Face {
		cyl := surfacegeom.Cylinder{Origin: r3.NewVec(d, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(rG)}
		f := &topo.Face{RetainedTag: true, Reversed: true}
		if mod != nil {
			mod(f, &cyl)
		}
		f.Surface = cyl
		return f
	}
	read := func(i int, f *topo.Face) (lineEnd, *cylEnd, error) {
		return a.readCylEnd(i, lineEnd{face: f}, fp)
	}
	end, ce, err := read(0, face(nil))
	require.NoError(t, err)
	require.InDelta(t, 0, end.exact.Sub(held).Len(), 1e-12)
	require.Greater(t, ce.other.Y, -R)
	require.InDelta(t, -held.Y, ce.other.Y, 1e-12)
	require.InDelta(t, 1.0, ce.sigma, 0)
	require.InDelta(t, math.Sqrt(1-20.75/30*20.75/30), ce.sV, 1e-9)
	require.Greater(t, ce.eV, 0.0)
	// End 1 reads the other root, so a vertex held at V0 is not where its tags meet.
	_, _, err = read(1, face(nil))
	require.ErrorContains(t, err, "not where its three tags meet")
	for name, tc := range map[string]struct {
		mod  func(*topo.Face, *surfacegeom.Cylinder)
		want string
	}{
		"untagged":  {func(f *topo.Face, _ *surfacegeom.Cylinder) { f.RetainedTag = false }, "not a retained plane or cylinder"},
		"departure": {func(f *topo.Face, _ *surfacegeom.Cylinder) { f.TagDeparture = 1e-12 }, "not a retained plane or cylinder"},
		"axial delta": {func(f *topo.Face, _ *surfacegeom.Cylinder) { f.HasAxialDelta, f.AxialDelta = true, 1e-9 },
			"not a retained plane or cylinder"},
		"no radius": {func(_ *topo.Face, c *surfacegeom.Cylinder) { c.Radius = units.Millimeters(0) },
			"not a retained plane or cylinder"},
		"material inside": {func(f *topo.Face, _ *surfacegeom.Cylinder) { f.Reversed = false }, "material lies inside it"},
		"tilted": {func(_ *topo.Face, c *surfacegeom.Cylinder) { c.Axis = r3.NewVec(1e-6, 0, 1) },
			"not parallel to the axis"},
		"coaxial": {func(_ *topo.Face, c *surfacegeom.Cylinder) { c.Origin = r3.NewVec(0, 0, 0) }, "coaxial with the wall"},
		"grazing": {func(_ *topo.Face, c *surfacegeom.Cylinder) { c.Origin = r3.NewVec(9.5-1e-7, 0, 0) },
			"grazing angle"},
	} {
		_, _, err := read(0, face(tc.mod))
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, tc.want, name)
	}
}
