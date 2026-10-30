package saddlecanal

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Proof is the proven reading of a held strip. Chord bounds the distance
// between every point of a held strip triangle and the exact fillet at the
// same parameters (θ, t), the held vertices' own rounding included. AreaLo
// and AreaHi enclose the exact area of the fillet. ShaftLo and ShaftHi
// enclose the area of the larger cylinder between the seam and T1, which the
// fillet removes from that wall; BoreLo and BoreHi the area of the bore
// between the seam and T2.
type Proof struct {
	Chord, Sample    float64
	AreaLo, AreaHi   float64
	ShaftLo, ShaftHi float64
	BoreLo, BoreHi   float64
}

// cellCheck names a test a cell must pass. Each failure refuses the fillet.
type cellCheck int

const (
	// checkJacobian is det[F_θ, F_t, u] > 0: the pipe is regular, so its
	// envelope is the surface the ball sweeps, wound as for a small ball.
	checkJacobian cellCheck = 1 << iota
	// checkProgress is ∂Φ/∂θ > 0, Φ the angle of (F_y, F_z) about the bore's
	// axis: every row runs once around the bore.
	checkProgress
	// checkMonotone is ∂(Φ, G)/∂(θ, t) > 0, G the larger wall's depth less
	// the bore's: at a fixed Φ, G grows with t, so no two parameters share a
	// point and the strip is embedded.
	checkMonotone
)

// eval encloses F and the blend direction u over sin θ ∈ S, cos θ ∈ C and
// t ∈ T, with their partial derivatives to second order.
func (c Canal) eval(S, C, T iv) (jvec, jvec, bool) {
	zero := jconst(pt(0))
	s := jet{v: S, a: C, b: pt(0), aa: S.neg(), ab: pt(0), bb: pt(0)}
	co := jet{v: C, a: S.neg(), b: pt(0), aa: C.neg(), ab: pt(0), bb: pt(0)}
	t := jet{v: T, a: pt(0), b: pt(1), aa: pt(0), ab: pt(0), bb: pt(0)}
	Rp := jconst(pt(c.R).sub(pt(c.Rho)))
	rp := jconst(pt(c.Bore).add(pt(c.Rho)))
	rpc := rp.mul(co)
	x, ok := Rp.mul(Rp).sub(rpc.mul(rpc)).sqrt()
	if !ok {
		return jvec{}, jvec{}, false
	}
	a1x, ok1 := x.div(Rp)
	a1y, ok2 := rpc.div(Rp)
	if !ok1 || !ok2 {
		return jvec{}, jvec{}, false
	}
	a1 := jvec{a1x, a1y, zero}
	a2 := jvec{zero, co.neg(), s.neg()}
	// The arc runs from a1 through the bisector b̂ = (a1 + a2)/|a1 + a2| to
	// a2, each half the normalised blend of its two ends.
	mid := a1.add(a2)
	mn, ok := mid.dot(mid).sqrt()
	if !ok {
		return jvec{}, jvec{}, false
	}
	var from, to jvec
	var tau jet
	switch {
	case T.hi <= 0.5:
		from, to, tau = a1, jvec{}, t.scale(pt(2))
	case T.lo >= 0.5:
		from, to, tau = jvec{}, a2, t.scale(pt(2)).sub(jconst(pt(1)))
	default:
		return jvec{}, jvec{}, false
	}
	for i := range 3 {
		b, ok := mid[i].div(mn)
		if !ok {
			return jvec{}, jvec{}, false
		}
		if T.hi <= 0.5 {
			to[i] = b
		} else {
			from[i] = b
		}
	}
	w := from.mul(jconst(pt(1)).sub(tau)).add(to.mul(tau))
	n, ok := w.dot(w).sqrt()
	if !ok {
		return jvec{}, jvec{}, false
	}
	var u, f jvec
	centre := jvec{x, rpc, rp.mul(s)}
	for i := range 3 {
		if u[i], ok = w[i].div(n); !ok {
			return jvec{}, jvec{}, false
		}
		f[i] = centre[i].add(u[i].scale(pt(c.Rho)))
	}
	return f, u, true
}

func cross(a, b [3]dv) [3]dv {
	return [3]dv{
		a[1].mul(b[2]).sub(a[2].mul(b[1])),
		a[2].mul(b[0]).sub(a[0].mul(b[2])),
		a[0].mul(b[1]).sub(a[1].mul(b[0])),
	}
}

func dot(a, b [3]dv) dv { return a[0].mul(b[0]).add(a[1].mul(b[1])).add(a[2].mul(b[2])) }

// readings are the cell's tested quantities with their gradients: the
// Jacobian det[F_θ, F_t, u], the progress numerator F_y·∂θF_z − F_z·∂θF_y,
// the embedding Jacobian of (Φ, G) times |F_yz|², and the area density
// |F_θ × F_t|.
type readings struct{ jac, progress, monotone, density dv }

func (c Canal) read(f, u jvec) (readings, bool) {
	var ft, fu, fv, uv [3]dv
	for i := range 3 {
		ft[i], fu[i], fv[i], uv[i] = f[i].da(), f[i].db(), f[i].dv(), u[i].dv()
	}
	n := cross(ft, fu)
	var r readings
	r.jac = dot(n, uv)
	r.progress = fv[1].mul(ft[2]).sub(fv[2].mul(ft[1]))
	pb := fv[1].mul(fu[2]).sub(fv[2].mul(fu[1]))
	dxy, ok1 := f[0].mul(f[0]).add(f[1].mul(f[1])).sqrt()
	dyz, ok2 := f[1].mul(f[1]).add(f[2].mul(f[2])).sqrt()
	if !ok1 || !ok2 {
		return readings{}, false
	}
	// G = (R − |F_xy|) − (|F_yz| − r); its partials are those of −|F_xy| − |F_yz|.
	g := dxy.add(dyz).neg()
	r.monotone = r.progress.mul(g.db()).sub(pb.mul(g.da()))
	nn := dot(n, n)
	if nn.v.lo > 0 {
		var ok bool
		r.density, ok = nn.sqrt()
		return r, ok
	}
	// Over a cell whose enclosure of |n| reaches zero, each partial of |n| is
	// at most the length of that partial of n.
	var ga, gb iv
	for i := range 3 {
		ga, gb = ga.add(n[i].a.sq()), gb.add(n[i].b.sq())
	}
	ra, ok1 := ga.sqrt()
	rb, ok2 := gb.sqrt()
	rv, ok3 := nn.v.sqrt()
	r.density = dv{rv, iv{-ra.hi, ra.hi}, iv{-rb.hi, rb.hi}}
	return r, ok1 && ok2 && ok3
}

// meanValue encloses q over the cell from its value at the expansion point
// and its gradient over the cell: q(mid) + ∂θq·dθ + ∂t q·dt.
func meanValue(mid, cell dv, dTheta, dT iv) iv {
	return mid.v.add(cell.a.mul(dTheta)).add(cell.b.mul(dT))
}

// cellResult is one (θ, t) cell's reading.
type cellResult struct {
	chord          float64
	areaLo, areaHi float64
	failed         cellCheck
}

// subResult is one sub-cell's reading: the failed tests, the largest
// second partials of each coordinate, and the area over it.
type subResult struct {
	failed         cellCheck
	aa, ab, bb     [3]float64
	areaLo, areaHi float64
}

// proveSub reads the sub-cell sin θ ∈ S, cos θ ∈ C, t ∈ [t0, t1], with θ
// within h/2 (h ≥ hLo) of its midpoint, whose sin and cos are sMid and cMid.
// Each tested quantity is enclosed by its mean-value form about the midpoint
// (θ_mid, t_mid).
func (c Canal) proveSub(S, C, sMid, cMid iv, t0, t1, hLo, h float64) (subResult, bool) {
	tMid := t0 + (t1-t0)/2
	f, u, ok1 := c.eval(S, C, iv{t0, t1})
	fm, um, ok2 := c.eval(sMid, cMid, pt(tMid))
	if !ok1 || !ok2 {
		return subResult{}, false
	}
	cell, ok1 := c.read(f, u)
	mid, ok2 := c.read(fm, um)
	if !ok1 || !ok2 || !(mid.density.v.lo > 0) {
		return subResult{}, false
	}
	half := up(h / 2)
	dTheta := iv{-half, half}
	dT := iv{down(t0 - tMid), up(t1 - tMid)}
	var res subResult
	if meanValue(mid.jac, cell.jac, dTheta, dT).lo <= 0 {
		res.failed |= checkJacobian
	}
	if meanValue(mid.progress, cell.progress, dTheta, dT).lo <= 0 {
		res.failed |= checkProgress
	}
	if meanValue(mid.monotone, cell.monotone, dTheta, dT).lo <= 0 {
		res.failed |= checkMonotone
	}
	for i := range 3 {
		res.aa[i], res.ab[i], res.bb[i] = f[i].aa.abs(), f[i].ab.abs(), f[i].bb.abs()
	}
	// The area over the sub-cell: the density at the midpoint times h·k, plus
	// the integral of the density's change from the midpoint. Writing its
	// gradient as the midpoint of its enclosure plus a deviation, the
	// midpoint's term integrates to zero over the symmetric cell, and the
	// deviation's is at most half the enclosure's width times
	// ∫|θ − θ_mid| = (h²/4)·k, or ∫|t − t_mid| = h·(k²/4).
	k, kLo := up(t1-t0), down(t1-t0)
	rem := up(up(up(cell.density.a.width()/2)*up(up(up(h*h)/4)*k)) +
		up(up(cell.density.b.width()/2)*up(h*up(up(k*k)/4))))
	res.areaLo = down(down(down(mid.density.v.lo*hLo)*kLo) - rem)
	res.areaHi = up(up(up(mid.density.v.hi*h)*k) + rem)
	return res, true
}

// proveCell reads station cell k and row cell [t0, t1] split into q × q
// sub-cells. The chord of a held triangle on the whole cell is at most
// |F_θθ|·h²/8 + |F_θt|·h·k/4 + |F_tt|·k²/8 per coordinate, the bound of
// linear interpolation on a triangle of the cell with each second partial's
// largest value over the sub-cells.
func (c Canal) proveCell(tr trig, k int, t0, t1 float64, q int) (cellResult, bool) {
	var res cellResult
	var aa, ab, bb [3]float64
	hsLo, hs := down(tr.hLo/float64(q)), up(tr.hHi/float64(q))
	for i := range q {
		S, C, sMid, cMid := tr.sub(k, i, q)
		for j := range q {
			s0 := t0 + (t1-t0)*float64(j)/float64(q)
			s1 := t0 + (t1-t0)*float64(j+1)/float64(q)
			if j == q-1 {
				s1 = t1
			}
			sub, ok := c.proveSub(S, C, sMid, cMid, s0, s1, hsLo, hs)
			if !ok {
				return cellResult{}, false
			}
			res.failed |= sub.failed
			for d := range 3 {
				aa[d], ab[d], bb[d] = max(aa[d], sub.aa[d]), max(ab[d], sub.ab[d]), max(bb[d], sub.bb[d])
			}
			res.areaLo = down(res.areaLo + sub.areaLo)
			res.areaHi = up(res.areaHi + sub.areaHi)
		}
	}
	h, kk := tr.hHi, up(t1-t0)
	h2, k2, hk := up(h*h), up(kk*kk), up(h*kk)
	for d := range 3 {
		e := up(up(up(aa[d]*h2)/8) + up(up(ab[d]*hk)/4))
		res.chord = max(res.chord, up(e+up(up(bb[d]*k2)/8)))
	}
	return res, true
}

// trig holds the enclosures of sin and cos at the stations, at the cells'
// midpoints, and the cell width.
type trig struct {
	s, c       []iv // stations k/n, k = 0 … n
	sMid, cMid []iv // midpoints (2k + 1)/(2n)
	hLo, hHi   float64
}

func ratIV(q proofbound.RatInterval) iv {
	return iv{proofbound.RatFloatDown(q.Lo), proofbound.RatFloatUp(q.Hi)}
}

func stationTrig(n int) trig {
	t := trig{s: make([]iv, n+1), c: make([]iv, n+1), sMid: make([]iv, n), cMid: make([]iv, n)}
	memo := proofbound.NewTurnTrigMemo()
	for k := 0; k <= n; k++ {
		s, c := memo.SinCos(big.NewRat(int64(k), int64(n)))
		t.s[k], t.c[k] = ratIV(s), ratIV(c)
	}
	for k := range n {
		s, c := memo.SinCos(big.NewRat(int64(2*k+1), int64(2*n)))
		t.sMid[k], t.cMid[k] = ratIV(s), ratIV(c)
	}
	nn := big.NewRat(int64(n), 1)
	t.hLo = proofbound.RatFloatDown(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiLower), nn))
	t.hHi = proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), nn))
	return t
}

// cell returns the enclosures of sin and cos over station cell k: n is a
// multiple of four, so neither has an extremum inside a cell and the hull of
// the two ends holds the whole cell.
func (t trig) cell(k int) (iv, iv) { return t.s[k].hull(t.s[k+1]), t.c[k].hull(t.c[k+1]) }

// sub returns sub-cell i of q of station cell k: the enclosures of sin and
// cos over it and at its midpoint. Each comes from station k's by the angle
// sum with the offset δ = (i + f)·h/q, whose sine and cosine are enclosed by
// x − x³/6 ≤ sin x ≤ x and 1 − x²/2 ≤ cos x ≤ 1 − x²/2 + x⁴/24 on [0, h].
func (t trig) sub(k, i, q int) (iv, iv, iv, iv) {
	if q == 1 {
		S, C := t.cell(k)
		return S, C, t.sMid[k], t.cMid[k]
	}
	at := func(num float64) (iv, iv) {
		d := iv{down(down(t.hLo*num) / float64(q)), up(up(t.hHi*num) / float64(q))}
		d3 := d.mul(d).mul(d)
		sd := iv{down(d.lo - up(d3.hi/6)), d.hi}
		d2 := d.sq()
		cd := iv{down(1 - up(d2.hi/2)), up(up(1-down(d2.lo/2)) + up(d2.sq().hi/24))}
		return t.s[k].mul(cd).add(t.c[k].mul(sd)), t.c[k].mul(cd).sub(t.s[k].mul(sd))
	}
	s0, c0 := at(float64(i))
	s1, c1 := at(float64(i + 1))
	sm, cm := at(float64(i) + 0.5)
	return s0.hull(s1), c0.hull(c1), sm, cm
}

// maxSplit is the finest split per side of a cell.
const maxSplit = 8

// Prove proves the strip of the fillet c with the given rows over the
// Stations station cells, and the areas the fillet changes. held(k, j)
// returns the held world vertex of station k and row j, which pose maps from
// local coordinates. Every cell must pass checkJacobian, checkProgress and
// checkMonotone; a cell too wide to decide them is split, up to maxSplit
// sub-cells a side.
func Prove(ctx context.Context, c Canal, rows []float64, pose crossbore.Pose,
	held func(k, j int) r3.Vec) (Proof, error) {
	const n = Stations
	if !c.Exists() {
		return Proof{}, fmt.Errorf("%w: no ball of that radius touches both walls of the saddle", decaderr.ErrDegenerate)
	}
	m := len(rows) - 1
	if m < 1 || rows[0] != 0 || rows[m] != 1 {
		return Proof{}, fmt.Errorf("%w: the saddle fillet's rows do not span the arc", decaderr.ErrUnsupported)
	}
	tr := stationTrig(n)
	cells := make([]cellResult, n*m)
	fail := func(k int, what string) error {
		return fmt.Errorf("%w: the saddle fillet's cell at station %d %s", decaderr.ErrUnsupported, k, what)
	}
	workers := orderedwork.EachWorkers(n, 64)
	if _, err := orderedwork.Each(n, workers, 8, ctx.Err, func(k int) error {
		for j := range m {
			// A cell whose enclosures are too wide to decide a test is split
			// into finer sub-cells, up to maxSplit per side.
			var res cellResult
			ok := false
			for q := 1; q <= maxSplit; q *= 2 {
				if res, ok = c.proveCell(tr, k, rows[j], rows[j+1], q); ok && res.failed == 0 {
					break
				}
			}
			if !ok {
				return fail(k, "has no enclosure")
			}
			switch failed := res.failed; {
			case failed&checkJacobian != 0:
				return fail(k, "folds: the pipe is not regular there")
			case failed&checkProgress != 0:
				return fail(k, "does not run on around the bore")
			case failed&checkMonotone != 0:
				return fail(k, "is not embedded")
			}
			cells[k*m+j] = res
		}
		return nil
	}); err != nil {
		return Proof{}, err
	}
	if err := ctx.Err(); err != nil {
		return Proof{}, err
	}
	var p Proof
	for _, res := range cells {
		p.Chord = max(p.Chord, res.chord)
		p.AreaLo = down(p.AreaLo + res.areaLo)
		p.AreaHi = up(p.AreaHi + res.areaHi)
	}
	// Every held vertex against the exact point at its own station.
	for k := range n {
		for j := 0; j <= m; j++ {
			f, _, ok := c.eval(tr.s[k], tr.c[k], pt(rows[j]))
			if !ok {
				return Proof{}, fail(k, "has no point enclosure")
			}
			p.Sample = max(p.Sample, worldError(held(k, j), f, pose))
		}
	}
	p.Sample = proofbound.Radius3D(p.Sample)
	p.Chord = proofbound.AbsSumUpper(proofbound.Radius3D(p.Chord), p.Sample)
	var ok bool
	if p.ShaftLo, p.ShaftHi, p.BoreLo, p.BoreHi, ok = c.wallAreas(tr); !ok {
		return Proof{}, fmt.Errorf("%w: the saddle fillet's wall areas have no enclosure", decaderr.ErrUnsupported)
	}
	if slices.ContainsFunc([]float64{p.Chord, p.AreaHi, p.ShaftHi, p.BoreHi}, proofbound.IsNonFinite) {
		return Proof{}, fmt.Errorf("%w: the saddle fillet has no finite bound", decaderr.ErrUnsupported)
	}
	if !(p.AreaLo > 0) || !(p.ShaftLo > 0) || !(p.BoreLo > 0) {
		return Proof{}, fmt.Errorf("%w: the saddle fillet's areas have no positive enclosure", decaderr.ErrUnsupported)
	}
	return p, nil
}

// worldError bounds the distance between a held world coordinate triple and
// the local enclosure f mapped by pose, per coordinate.
func worldError(held r3.Vec, f jvec, pose crossbore.Pose) float64 {
	h := [3]float64{held.X, held.Y, held.Z}
	shift := [3]float64{pose.Shift.X, pose.Shift.Y, pose.Shift.Z}
	e := 0.0
	for i := range 3 {
		v := f[i].v
		if pose.Sign[i] < 0 {
			v = v.neg()
		}
		a := pose.Axis[i]
		v = v.add(pt(shift[a]))
		e = max(e, up(h[a]-v.lo), up(v.hi-h[a]))
	}
	return e
}

// wallAreas encloses the two areas the fillet removes from the walls, each a
// θ integral over the station cells. On cell k the integral of f lies within
// h·f(θ_mid) ± (w/2)·h²/4, w the width of the enclosure of f′ over the cell.
//
// The larger wall loses the region between the seam and T1. Its area over
// the (y, z) projection, where the wall is the graph x = √(R² − y²) with area
// element R/√(R² − y²), is A(r', R') − A(r, R) with
//
//	A(a, b) = ∫₀^{2π} R·a²·sin²θ / (b·√(1 − (a/b)²cos²θ)) dθ,
//
// the substitution y = (R·a/b)·cos θ, z = a·sin θ of the ellipse
// (b/R)²y² + z² = a². The bore loses the band between the seam
// x = √(R² − r²cos²θ) and T2 x = √(R'² − r'²cos²θ) along its generator at
// angle θ, an area ∫ r·(seam − T2) dθ.
func (c Canal) wallAreas(tr trig) (float64, float64, float64, float64, bool) {
	R, r := dconst(pt(c.R)), dconst(pt(c.Bore))
	Rp := dconst(pt(c.R).sub(pt(c.Rho)))
	rp := dconst(pt(c.Bore).add(pt(c.Rho)))
	one := dconst(pt(1))
	region := func(s, co, a, b dv) (dv, bool) {
		k, ok := a.div(b)
		if !ok {
			return dv{}, false
		}
		kc := k.mul(co)
		root, ok := one.sub(kc.mul(kc)).sqrt()
		if !ok {
			return dv{}, false
		}
		num := R.mul(a).mul(a).mul(s).mul(s)
		return num.div(b.mul(root))
	}
	shaft := func(s, co dv) (dv, bool) {
		outer, ok1 := region(s, co, rp, Rp)
		inner, ok2 := region(s, co, r, R)
		return outer.sub(inner), ok1 && ok2
	}
	bore := func(_, co dv) (dv, bool) {
		seam, ok1 := R.mul(R).sub(r.mul(r).mul(co).mul(co)).sqrt()
		t2, ok2 := Rp.mul(Rp).sub(rp.mul(rp).mul(co).mul(co)).sqrt()
		return r.mul(seam.sub(t2)), ok1 && ok2
	}
	integral := func(f func(s, co dv) (dv, bool)) (float64, float64, bool) {
		lo, hi := 0.0, 0.0
		for k := range len(tr.sMid) {
			mid, ok := f(dconst(tr.sMid[k]), dconst(tr.cMid[k]))
			if !ok {
				return 0, 0, false
			}
			S, C := tr.cell(k)
			slope, ok := f(dv{S, C, pt(0)}, dv{C, S.neg(), pt(0)})
			if !ok {
				return 0, 0, false
			}
			rem := up(up(slope.a.width()/2) * up(up(tr.hHi*tr.hHi)/4))
			part := mid.v.mul(iv{tr.hLo, tr.hHi})
			lo = down(lo + down(part.lo-rem))
			hi = up(hi + up(part.hi+rem))
		}
		return lo, hi, true
	}
	sLo, sHi, ok1 := integral(shaft)
	bLo, bHi, ok2 := integral(bore)
	return sLo, sHi, bLo, bHi, ok1 && ok2
}
