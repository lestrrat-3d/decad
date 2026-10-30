package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// This file sizes the rows of an arc blend (agent-docs/retained-edge-blend-design.md
// §4.4, §6.3) for the curve the blend cuts in each end plane G. G is parallel
// to the axis at distance h from it, so the row at profile parameter s, the
// circle of radius ρ(s) at height z(s), meets G at in-plane distance
// X(s) = sqrt(ρ² − h²) from the foot of the axis, and the end curve is
// (X(s), z(s)). The end face's boundary takes the chords between neighbouring
// rows' points, and each row vertex states the bound τ + slack, so every chord
// must lie within τ of the arc of the end curve it stands for.

// maxSectionRows is the most rows sectionRows builds before it refuses.
const maxSectionRows = 1024

// maxSectionLeaves is the most pieces one chord's certificate splits its arc
// into before it gives up and the chord is split instead.
const maxSectionLeaves = 4096

// endSection is the curve a blend cuts in one end plane, in coordinates
// u = −κ·X and v = side·z that both grow with s: ρ and z are monotone over the
// profile, so the curve runs up and to the right.
type endSection struct {
	p     arcProfile
	kappa float64
	side  float64
	h     float64
	// slope is |dz/ds| / |dρ/ds|, nondecreasing in s: tan(s·π/2) for the
	// torus, dB/dA for the cone.
	slope func(s float64) float64
	// margin is certify's rounding allowance, 32·ulp·S·(1 + 1/q) with S the
	// profile's largest ρ plus h plus its largest |z|, and q the smallest X/ρ
	// (§6.3): it covers the rounding of certify's own arithmetic and of the
	// row vertices' azimuths, both of which grow as 1/q.
	margin float64
}

// section is the end curve the blend of spec cuts in the end plane g. It
// refuses an end plane that some row's circle does not reach.
func (a arcRim) section(spec blendSpec, g plane) (endSection, error) {
	p, fillet := a.profile(spec)
	// ringAzimuths solves ρ·|(ca, cb)|·cos(φ − β) = h: G's in-plane distance
	// from the axis is h / |(ca, cb)|.
	ca, cb := g.n.Dot(a.u), g.n.Dot(a.w)
	h := math.Abs(g.n.Dot(g.o.Sub(a.center))) / math.Hypot(ca, cb)
	for _, s := range []float64{0, 1} {
		if rho, _ := p(s); !(h < rho) {
			return endSection{}, fmt.Errorf("%w: a blend circle does not reach an end face", decaderr.ErrUnsupported)
		}
	}
	e := endSection{p: p, kappa: a.kappa, side: a.side, h: h}
	// ρ and z are monotone over the profile, so their extremes and the
	// smallest X/ρ lie at its ends.
	rho0, z0 := p(0)
	rho1, z1 := p(1)
	_, _, q0 := e.at(0)
	_, _, q1 := e.at(1)
	size := max(rho0, rho1) + h + max(math.Abs(z0), math.Abs(z1))
	e.margin = 32 * ulp * size * (1 + 1/min(q0, q1))
	if fillet {
		e.slope = func(s float64) float64 {
			if s >= 1 {
				return math.Inf(1)
			}
			return math.Tan(s * math.Pi / 2)
		}
		return e, nil
	}
	ratio := spec.dB / spec.dA
	e.slope = func(float64) float64 { return ratio }
	return e, nil
}

// at is the end curve's point at s in (u, v), and q = X/ρ there.
func (e endSection) at(s float64) (u, v, q float64) {
	rho, z := e.p(s)
	x := math.Sqrt(rho*rho - e.h*e.h)
	return -e.kappa * x, e.side * z, x / rho
}

// certify reports whether every point of the end curve between s0 and s1 lies
// within tau of the chord between its points there.
//
// The curve's tangent makes the angle ψ with the u axis, tan ψ = slope(s)·q(s):
// dv/ds = |dz/ds| and du/ds = |dρ/ds|·ρ/X. Over a piece [t0, t1] slope is
// nondecreasing and q monotone, so ψ lies in [atan(slope(t0)·min q),
// atan(slope(t1)·max q)] ⊂ [0, π/2]. A curve from P to Q whose velocity lies
// in the cone of the two directions ψlo and ψhi lies in P + cone and in
// Q − cone: the parallelogram with diagonal PQ and sides along them. The
// piece's points are no farther from the chord's line than the
// parallelogram's corners are. Every parallelogram lies in the box with
// diagonal AB, whose points project onto the chord, so that distance is the
// distance from the chord itself. A piece whose corners reach past tau is
// split in two; certify gives up after maxSectionLeaves pieces. A piece
// certifies when its corners lie within tau less the rounding margin.
func (e endSection) certify(s0, s1, tau float64) bool {
	if !(e.margin < tau) {
		return false
	}
	au, av, _ := e.at(s0)
	bu, bv, _ := e.at(s1)
	du, dv := bu-au, bv-av
	ab := math.Hypot(du, dv)
	if !(ab > 0) {
		return false
	}
	dist := func(u, v float64) float64 { return math.Abs(du*(v-av)-dv*(u-au)) / ab }
	type piece struct{ t0, t1 float64 }
	stack := []piece{{s0, s1}}
	for n := 0; len(stack) > 0; n++ {
		if n >= maxSectionLeaves {
			return false
		}
		pc := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		pu, pv, q0 := e.at(pc.t0)
		qu, qv, q1 := e.at(pc.t1)
		lo := math.Atan(e.slope(pc.t0) * math.Min(q0, q1))
		hi := math.Atan(e.slope(pc.t1) * math.Max(q0, q1))
		d := math.Max(dist(pu, pv), dist(qu, qv))
		if hi > lo {
			// The corners P + |PQ|·sin(hi − ψ)/sin(hi − lo) along lo and
			// P + |PQ|·sin(ψ − lo)/sin(hi − lo) along hi, ψ the direction of PQ.
			psi := math.Min(hi, math.Max(lo, math.Atan2(qv-pv, qu-pu)))
			l := math.Hypot(qu-pu, qv-pv) / math.Sin(hi-lo)
			k1, k2 := l*math.Sin(hi-psi), l*math.Sin(psi-lo)
			d = math.Max(d, dist(pu+k1*math.Cos(lo), pv+k1*math.Sin(lo)))
			d = math.Max(d, dist(pu+k2*math.Cos(hi), pv+k2*math.Sin(hi)))
		}
		if d+e.margin <= tau {
			continue
		}
		mid := (pc.t0 + pc.t1) / 2
		if !(mid > pc.t0 && mid < pc.t1) {
			return false
		}
		stack = append(stack, piece{pc.t0, mid}, piece{mid, pc.t1})
	}
	return true
}

// sectionRows returns the profile parameters of the blend's rows, from 0 at
// the plane's setback to 1 at the wall's: m evenly spaced rows, with each gap
// halved until the chord between its rows' points on every end plane in ends
// certifies within tau. It refuses a blend that needs more than
// maxSectionRows rows.
func sectionRows(m int, tau float64, ends []endSection) ([]float64, error) {
	refuse := fmt.Errorf("%w: the blend's end curve on an end face needs more than %d rows",
		decaderr.ErrUnsupported, maxSectionRows)
	for _, e := range ends {
		// No number of rows certifies a chord when the rounding margin
		// alone reaches tau.
		if !(e.margin < tau) {
			return nil, refuse
		}
	}
	out := []float64{0}
	var fill func(s0, s1 float64) error
	fill = func(s0, s1 float64) error {
		ok := true
		for _, e := range ends {
			ok = ok && e.certify(s0, s1, tau)
		}
		if ok {
			out = append(out, s1)
			return nil
		}
		mid := (s0 + s1) / 2
		if len(out) >= maxSectionRows || !(mid > s0 && mid < s1) {
			return refuse
		}
		if err := fill(s0, mid); err != nil {
			return err
		}
		return fill(mid, s1)
	}
	for j := range m {
		if err := fill(float64(j)/float64(m), float64(j+1)/float64(m)); err != nil {
			return nil, err
		}
	}
	return out, nil
}

// arcRows is sectionRows for a blend whose rows end on the planes of ends,
// starting from the rows the profile itself needs: a fillet's arc sagitta
// r·(1 − cos(Δα/2)) within tau, and two for a chamfer, whose profile is
// straight.
func arcRows(spec blendSpec, tau float64, ends []endSection) ([]float64, error) {
	m := 2
	if spec.fillet {
		m = max(2, int(math.Ceil(math.Pi/2/rowStep(tau, spec.r, 1))))
	}
	return sectionRows(m, tau, ends)
}
