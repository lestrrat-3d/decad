// Package flushface decides whether the held outlines of two coplanar faces keep their arrangement under every move
// their bounds allow (agent-docs/flush-contact-design.md §4.4): every part of either outline that may move either
// stays farther than its own radius plus the other outline's reach from every edge of the other outline, or is a
// straight edge that crosses such an edge robustly, so the crossing survives every move of the endpoints. It reads
// held data only: box separations bound distances from below, and orientations are exact rationals.
package flushface

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Edge is one held outline edge of a face lying exactly in a plane. RA and RB bound how far the true endpoints lie
// from A and B. Line marks an edge whose true counterpart lies on the held edge's own line (both endpoints exact, or a
// line two exact planes fix), so only its endpoints move, along that line. Straight marks an edge whose true
// counterpart is the segment between the true endpoints (both faces sharing it are planar).
type Edge struct {
	A, B     r3.Vec
	RA, RB   float64
	Line     bool
	Straight bool
}

// Outline is one face's held outline in its plane. Axis is the coordinate dropped to project the plane: the
// largest component of its normal.
type Outline struct {
	Edges []Edge
	Axis  int
}

// Reach returns the largest distance any point of the outline's true counterpart lies from the held outline.
func (o Outline) Reach() float64 {
	r := 0.0
	for _, e := range o.Edges {
		r = max(r, e.RA, e.RB)
	}
	return r
}

// Verdict is Clear's answer: whether the outlines are clear, and Cross, the largest bound of a robust crossing's
// true point from its held point (zero when no approximate part crosses).
type Verdict struct {
	Clear bool
	Cross float64
}

// Clear decides a and b. It steps budget once per box comparison and once per crossing test.
func Clear(budget *proofbound.WorkBudget, a, b Outline) (Verdict, error) {
	ra, rb := a.Reach(), b.Reach()
	if !finiteBound(ra) || !finiteBound(rb) {
		return Verdict{}, nil
	}
	v1, err := clearOf(budget, a, b, rb)
	if err != nil || !v1.Clear {
		return Verdict{}, err
	}
	v2, err := clearOf(budget, b, a, ra)
	if err != nil || !v2.Clear {
		return Verdict{}, err
	}
	return Verdict{Clear: true, Cross: max(v1.Cross, v2.Cross)}, nil
}

func finiteBound(r float64) bool { return r >= 0 && !math.IsInf(r, 0) && !math.IsNaN(r) }

// clearOf checks the parts of x that may move against every edge of y, whose true outline lies within reachY.
func clearOf(budget *proofbound.WorkBudget, x, y Outline, reachY float64) (Verdict, error) {
	out := Verdict{Clear: true}
	if len(y.Edges) == 0 {
		return out, nil
	}
	whole := segBox(y.Edges[0].A, y.Edges[0].B)
	for _, e := range y.Edges[1:] {
		whole = join(whole, segBox(e.A, e.B))
	}
	check := func(box [2]r3.Vec, r float64, seg *Edge) (bool, error) {
		pad := padOf(r, reachY)
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !within(box, whole, pad) {
			return true, nil
		}
		for k := range y.Edges {
			e := &y.Edges[k]
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !within(box, segBox(e.A, e.B), pad) {
				continue
			}
			if seg == nil || !seg.Straight || !e.Straight {
				return false, nil
			}
			ok, bound := robustCross(*seg, *e, x.Axis)
			if !ok {
				return false, nil
			}
			out.Cross = max(out.Cross, bound)
		}
		return true, nil
	}
	for k := range x.Edges {
		e := &x.Edges[k]
		for _, end := range [2]struct {
			p r3.Vec
			r float64
		}{{e.A, e.RA}, {e.B, e.RB}} {
			if end.r == 0 {
				continue
			}
			ok, err := check([2]r3.Vec{end.p, end.p}, end.r, nil)
			if err != nil || !ok {
				return Verdict{}, err
			}
		}
		if e.Line {
			continue
		}
		r := max(e.RA, e.RB)
		if r == 0 {
			continue
		}
		ok, err := check(segBox(e.A, e.B), r, e)
		if err != nil || !ok {
			return Verdict{}, err
		}
	}
	return out, nil
}

// padOf is r + reach, rounded up and widened by a relative 1e-9.
func padOf(r, reach float64) float64 {
	pad := proofbound.AbsSumUpper(r, reach)
	return proofbound.AbsSumUpper(pad, proofbound.ProductUpper(pad, 1e-9))
}

// robustCross reports whether segments s and e cross properly and keep crossing under every move of their endpoints
// within RA and RB, and bounds the distance between the held and the true crossing point. Projected to the plane that
// drops axis, the orientation of each segment's endpoints against the other must keep a strict, opposite sign: its
// exact value must exceed the largest change the moves can make (orientation is bilinear in the points).
func robustCross(s, e Edge, axis int) (bool, float64) {
	a, b, c, d := rat2(s.A, axis), rat2(s.B, axis), rat2(e.A, axis), rat2(e.B, axis)
	o1 := orient(c, d, a)
	o2 := orient(c, d, b)
	o3 := orient(a, b, c)
	o4 := orient(a, b, d)
	if o1.Sign()*o2.Sign() >= 0 || o3.Sign()*o4.Sign() >= 0 {
		return false, 0
	}
	lenCD, lenAB := lenUp(c, d), lenUp(a, b)
	lenCA, lenCB, lenAC, lenAD := lenUp(c, a), lenUp(c, b), lenUp(a, c), lenUp(a, d)
	m := func(len1, e1, len2, e2 float64) float64 {
		// |Δ cross(q−p, r−p)| ≤ |q−p|(εr+εp) + (εq+εp)|r−p| + (εq+εp)(εr+εp)
		t1 := proofbound.ProductUpper(len1, e1)
		t2 := proofbound.ProductUpper(e2, len2)
		return proofbound.AbsSumUpper(t1, t2, proofbound.ProductUpper(e2, e1))
	}
	// Against an edge whose true counterpart lies on its held line, only the tested point moves off that line.
	against := func(line bool, lenPQ, eR, eP, lenPR, eQ float64) float64 {
		if line {
			return proofbound.ProductUpper(lenPQ, eR)
		}
		return m(lenPQ, proofbound.AbsSumUpper(eR, eP), lenPR, proofbound.AbsSumUpper(eQ, eP))
	}
	need := []struct {
		o   *big.Rat
		lim float64
	}{
		{o1, against(e.Line, lenCD, s.RA, e.RA, lenCA, e.RB)},
		{o2, against(e.Line, lenCD, s.RB, e.RA, lenCB, e.RB)},
		{o3, against(s.Line, lenAB, e.RA, s.RA, lenAC, s.RB)},
		{o4, against(s.Line, lenAB, e.RB, s.RA, lenAD, s.RB)},
	}
	for _, n := range need {
		if !finiteBound(n.lim) || new(big.Rat).Abs(n.o).Cmp(new(big.Rat).SetFloat64(n.lim)) <= 0 {
			return false, 0
		}
	}
	// The true segments lie within w1 and w2 of the held ones' lines, so their crossing lies within (w1 + w2)/sin θ
	// of the held crossing. sin θ ≥ |cross(b−a, d−c)| / (|b−a|·|d−c|), the projected cross product bounding the full one
	// from below.
	width := func(x Edge) float64 {
		if x.Line {
			return 0
		}
		return max(x.RA, x.RB)
	}
	w := proofbound.AbsSumUpper(width(s), width(e))
	cr := new(big.Rat).Abs(cross(sub(b, a), sub(d, c)))
	crLo, _ := cr.Float64()
	crLo = math.Nextafter(crLo, 0)
	den := proofbound.ProductUpper(lenAB, lenCD)
	if !(crLo > 0) || !finiteBound(den) {
		return false, 0
	}
	sinLo := math.Nextafter(crLo/den, 0)
	if !(sinLo > 0) {
		return false, 0
	}
	// The projection that drops the normal's largest axis shortens a distance in the plane by at most √3.
	bound := proofbound.ProductUpper(proofbound.UpRound(w/sinLo), 1.7320508075688776)
	if !finiteBound(bound) {
		return false, 0
	}
	return true, bound
}

type rv [2]*big.Rat

func rat2(p r3.Vec, axis int) rv {
	c := [3]float64{p.X, p.Y, p.Z}
	var out rv
	k := 0
	for i := range 3 {
		if i == axis {
			continue
		}
		out[k] = new(big.Rat).SetFloat64(c[i])
		k++
	}
	return out
}

func sub(p, q rv) rv {
	return rv{new(big.Rat).Sub(p[0], q[0]), new(big.Rat).Sub(p[1], q[1])}
}

func cross(u, v rv) *big.Rat {
	return new(big.Rat).Sub(new(big.Rat).Mul(u[0], v[1]), new(big.Rat).Mul(u[1], v[0]))
}

func orient(p, q, r rv) *big.Rat { return cross(sub(q, p), sub(r, p)) }

// lenUp is an upper bound of the projected distance between p and q.
func lenUp(p, q rv) float64 {
	d := sub(q, p)
	s := new(big.Rat).Add(new(big.Rat).Mul(d[0], d[0]), new(big.Rat).Mul(d[1], d[1]))
	f, _ := s.Float64()
	return proofbound.UpRound(math.Sqrt(math.Nextafter(f, math.Inf(1))))
}

func segBox(a, b r3.Vec) [2]r3.Vec { return join([2]r3.Vec{a, a}, [2]r3.Vec{b, b}) }

func join(a, b [2]r3.Vec) [2]r3.Vec {
	return [2]r3.Vec{
		{X: math.Min(a[0].X, b[0].X), Y: math.Min(a[0].Y, b[0].Y), Z: math.Min(a[0].Z, b[0].Z)},
		{X: math.Max(a[1].X, b[1].X), Y: math.Max(a[1].Y, b[1].Y), Z: math.Max(a[1].Z, b[1].Z)},
	}
}

// within reports whether boxes a and b come within slack of each other on every axis. A rounded difference is within
// half an ulp of the exact one, so the next float toward minus infinity bounds the exact gap from below: a pair this
// calls apart is apart by more than slack, and rounding can only call more pairs within.
func within(a, b [2]r3.Vec, slack float64) bool {
	apart := func(lo, hi float64) bool { return math.Nextafter(lo-hi, math.Inf(-1)) > slack }
	return !(apart(b[0].X, a[1].X) || apart(a[0].X, b[1].X) ||
		apart(b[0].Y, a[1].Y) || apart(a[0].Y, b[1].Y) ||
		apart(b[0].Z, a[1].Z) || apart(a[0].Z, b[1].Z))
}
