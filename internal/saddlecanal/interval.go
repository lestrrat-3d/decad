package saddlecanal

import (
	"math"
)

// iv is a float interval with outward rounding. Every operation rounds its
// float result one ulp outward, which covers the half-ulp error of a
// correctly rounded IEEE operation, so an iv computed from enclosures
// encloses the exact value. Each operation is its own function call, so no
// two roundings fuse.
type iv struct{ lo, hi float64 }

func down(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }
func up(x float64) float64   { return math.Nextafter(x, math.Inf(1)) }

func pt(x float64) iv { return iv{x, x} }

func (a iv) add(b iv) iv { return iv{down(a.lo + b.lo), up(a.hi + b.hi)} }
func (a iv) sub(b iv) iv { return iv{down(a.lo - b.hi), up(a.hi - b.lo)} }
func (a iv) neg() iv     { return iv{-a.hi, -a.lo} }

func (a iv) mul(b iv) iv {
	if !(a.lo >= -math.MaxFloat64 && a.lo <= a.hi && a.hi <= math.MaxFloat64) ||
		!(b.lo >= -math.MaxFloat64 && b.lo <= b.hi && b.hi <= math.MaxFloat64) {
		return a.mulFour(b)
	}
	var lo, hi float64
	if a.lo >= 0 {
		switch {
		case b.lo >= 0:
			lo, hi = a.lo*b.lo, a.hi*b.hi
		case b.hi <= 0:
			lo, hi = a.hi*b.lo, a.lo*b.hi
		default:
			lo, hi = a.hi*b.lo, a.hi*b.hi
		}
	} else if a.hi <= 0 {
		switch {
		case b.lo >= 0:
			lo, hi = a.lo*b.hi, a.hi*b.lo
		case b.hi <= 0:
			lo, hi = a.hi*b.hi, a.lo*b.lo
		default:
			lo, hi = a.lo*b.hi, a.lo*b.lo
		}
	} else if b.lo >= 0 {
		lo, hi = a.lo*b.hi, a.hi*b.hi
	} else if b.hi <= 0 {
		lo, hi = a.hi*b.lo, a.lo*b.lo
	} else {
		p0, p1 := a.lo*b.hi, a.hi*b.lo
		q0, q1 := a.lo*b.lo, a.hi*b.hi
		lo, hi = min(p0, p1), max(q0, q1)
	}
	return iv{down(lo), up(hi)}
}

func (a iv) mulFour(b iv) iv {
	p := [4]float64{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi}
	return iv{down(min(p[0], p[1], p[2], p[3])), up(max(p[0], p[1], p[2], p[3]))}
}

func (a iv) sq() iv {
	switch {
	case a.lo >= 0:
		return iv{down(a.lo * a.lo), up(a.hi * a.hi)}
	case a.hi <= 0:
		return iv{down(a.hi * a.hi), up(a.lo * a.lo)}
	}
	return iv{0, up(max(a.lo*a.lo, a.hi*a.hi))}
}

// div is a/b; ok is false when b may hold zero.
func (a iv) div(b iv) (iv, bool) {
	if !(b.lo > 0 || b.hi < 0) {
		return iv{}, false
	}
	q := [4]float64{a.lo / b.lo, a.lo / b.hi, a.hi / b.lo, a.hi / b.hi}
	return iv{down(min(q[0], q[1], q[2], q[3])), up(max(q[0], q[1], q[2], q[3]))}, true
}

// sqrt encloses the root of a value known to be non-negative: a lower end
// below zero is read as zero. ok is false when the whole interval is negative.
func (a iv) sqrt() (iv, bool) {
	if a.hi < 0 || a.lo != a.lo || a.hi != a.hi {
		return iv{}, false
	}
	return iv{max(0, down(math.Sqrt(max(a.lo, 0)))), up(math.Sqrt(a.hi))}, true
}

// width is an upper bound on hi − lo.
func (a iv) width() float64 { return up(a.hi - a.lo) }

// abs is an upper bound on the largest absolute value in a.
func (a iv) abs() float64 { return max(math.Abs(a.lo), math.Abs(a.hi)) }

// hull is the smallest interval holding a and b.
func (a iv) hull(b iv) iv { return iv{min(a.lo, b.lo), max(a.hi, b.hi)} }

// dv is a value with its two partial derivatives, ∂/∂θ and ∂/∂t, each an iv.
type dv struct{ v, a, b iv }

func dconst(x iv) dv { return dv{v: x, a: pt(0), b: pt(0)} }

func (x dv) add(y dv) dv { return dv{x.v.add(y.v), x.a.add(y.a), x.b.add(y.b)} }
func (x dv) sub(y dv) dv { return dv{x.v.sub(y.v), x.a.sub(y.a), x.b.sub(y.b)} }

func (x dv) mul(y dv) dv {
	return dv{x.v.mul(y.v), x.a.mul(y.v).add(x.v.mul(y.a)), x.b.mul(y.v).add(x.v.mul(y.b))}
}

func (x dv) div(y dv) (dv, bool) {
	v, ok := x.v.div(y.v)
	if !ok {
		return dv{}, false
	}
	y2 := y.v.sq()
	a, ok1 := x.a.mul(y.v).sub(x.v.mul(y.a)).div(y2)
	b, ok2 := x.b.mul(y.v).sub(x.v.mul(y.b)).div(y2)
	return dv{v, a, b}, ok1 && ok2
}

func (x dv) sqrt() (dv, bool) {
	v, ok := x.v.sqrt()
	if !ok || !(v.lo > 0) {
		return dv{}, false
	}
	two := v.mul(pt(2))
	a, ok1 := x.a.div(two)
	b, ok2 := x.b.div(two)
	return dv{v, a, b}, ok1 && ok2
}

// jet is a value with its partial derivatives to second order in (θ, t):
// v, ∂θ, ∂t, ∂θθ, ∂θt, ∂tt, each an iv.
type jet struct{ v, a, b, aa, ab, bb iv }

func jconst(x iv) jet { z := pt(0); return jet{x, z, z, z, z, z} }

func (x jet) add(y jet) jet {
	return jet{x.v.add(y.v), x.a.add(y.a), x.b.add(y.b), x.aa.add(y.aa), x.ab.add(y.ab), x.bb.add(y.bb)}
}

func (x jet) sub(y jet) jet {
	return jet{x.v.sub(y.v), x.a.sub(y.a), x.b.sub(y.b), x.aa.sub(y.aa), x.ab.sub(y.ab), x.bb.sub(y.bb)}
}

func (x jet) neg() jet {
	return jet{x.v.neg(), x.a.neg(), x.b.neg(), x.aa.neg(), x.ab.neg(), x.bb.neg()}
}

func (x jet) scale(k iv) jet {
	return jet{x.v.mul(k), x.a.mul(k), x.b.mul(k), x.aa.mul(k), x.ab.mul(k), x.bb.mul(k)}
}

func (x jet) mul(y jet) jet {
	return jet{
		v:  x.v.mul(y.v),
		a:  x.a.mul(y.v).add(x.v.mul(y.a)),
		b:  x.b.mul(y.v).add(x.v.mul(y.b)),
		aa: x.aa.mul(y.v).add(x.a.mul(y.a).mul(pt(2))).add(x.v.mul(y.aa)),
		ab: x.ab.mul(y.v).add(x.a.mul(y.b)).add(x.b.mul(y.a)).add(x.v.mul(y.ab)),
		bb: x.bb.mul(y.v).add(x.b.mul(y.b).mul(pt(2))).add(x.v.mul(y.bb)),
	}
}

// chain applies a scalar function with value f0 and derivatives f1, f2 at x.
func (x jet) chain(f0, f1, f2 iv) jet {
	return jet{
		v:  f0,
		a:  f1.mul(x.a),
		b:  f1.mul(x.b),
		aa: f2.mul(x.a.sq()).add(f1.mul(x.aa)),
		ab: f2.mul(x.a.mul(x.b)).add(f1.mul(x.ab)),
		bb: f2.mul(x.b.sq()).add(f1.mul(x.bb)),
	}
}

func (x jet) recip() (jet, bool) {
	r, ok := pt(1).div(x.v)
	if !ok {
		return jet{}, false
	}
	r2 := r.sq()
	return x.chain(r, r2.neg(), r2.mul(r).mul(pt(2))), true
}

func (x jet) div(y jet) (jet, bool) {
	r, ok := y.recip()
	return x.mul(r), ok
}

func (x jet) sqrt() (jet, bool) {
	s, ok := x.v.sqrt()
	if !ok || !(s.lo > 0) {
		return jet{}, false
	}
	d1, ok1 := pt(1).div(s.mul(pt(2)))
	d2, ok2 := pt(-1).div(s.mul(s.sq()).mul(pt(4)))
	return x.chain(s, d1, d2), ok1 && ok2
}

// dv returns the first-order view of x: its value and gradient.
func (x jet) dv() dv { return dv{x.v, x.a, x.b} }

// da and db return the first-order views of ∂θx and ∂t x.
func (x jet) da() dv { return dv{x.a, x.aa, x.ab} }
func (x jet) db() dv { return dv{x.b, x.ab, x.bb} }

// jvec is a point or vector of jet coordinates.
type jvec [3]jet

func (p jvec) add(q jvec) jvec { return jvec{p[0].add(q[0]), p[1].add(q[1]), p[2].add(q[2])} }
func (p jvec) mul(k jet) jvec  { return jvec{p[0].mul(k), p[1].mul(k), p[2].mul(k)} }
func (p jvec) dot(q jvec) jet  { return p[0].mul(q[0]).add(p[1].mul(q[1])).add(p[2].mul(q[2])) }
