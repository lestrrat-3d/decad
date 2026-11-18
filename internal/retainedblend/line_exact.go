package retainedblend

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/r3"
)

// lineFoot is one held setback point of an R1 chamfer and its proven distance from the exact point.
type lineFoot struct {
	p     r3.Vec
	bound float64
}

// chamferFeetInput is the plain data exactChamferFeet reads; every point is a held vertex with bound 0, chosen
// by agent-docs/exact-chamfer-design.md §4.1 (onA[i], onB[i] are vertices of both their side face and the end face
// G_i).
type chamferFeetInput struct {
	v        [2]r3.Vec // edge ends V₀, V₁
	onA, onB [2]r3.Vec // cA_i, cB_i
	intoA    r3.Vec    // wA: float sense of the way into A
	intoB    r3.Vec    // wB
	tA, tB   float64   // setbacks on A and B, exact millimetres
}

// primitive is the integer vector of smallest size along a: every component an integer, their gcd 1. It reports
// false for the zero vector.
func (a ratVec) primitive() (ratVec, bool) {
	if a.isZero() {
		return ratVec{}, false
	}
	lcm := big.NewInt(1)
	for _, x := range a {
		g := new(big.Int).GCD(nil, nil, lcm, x.Denom())
		lcm.Mul(lcm, new(big.Int).Quo(x.Denom(), g))
	}
	var ints [3]*big.Int
	gcd := new(big.Int)
	for i, x := range a {
		n := new(big.Int).Mul(x.Num(), new(big.Int).Quo(lcm, x.Denom()))
		ints[i] = n
		gcd.GCD(nil, nil, gcd, new(big.Int).Abs(n))
	}
	var out ratVec
	for i, n := range ints {
		out[i] = new(big.Rat).SetInt(n.Quo(n, gcd))
	}
	return out, true
}

// heldRatVec reads a held vector as exact rational coordinates, false when a coordinate is not finite.
func heldRatVec(v r3.Vec) (ratVec, bool) {
	r, ok := stitchflux.RatVecOf(v)
	return ratVec(r), ok
}

// exactChamferFeet returns feet[i][0] = P_A,i and feet[i][1] = P_B,i
// (agent-docs/exact-chamfer-design.md §4.2). ok is false when a carrier is degenerate or the sense check fails
// (§4.1); the caller then keeps today's feet.
func exactChamferFeet(in chamferFeetInput) (feet [2][2]lineFoot, ok bool) {
	var v, onA, onB [2]ratVec
	for i := range 2 {
		var okV, okA, okB bool
		v[i], okV = heldRatVec(in.v[i])
		onA[i], okA = heldRatVec(in.onA[i])
		onB[i], okB = heldRatVec(in.onB[i])
		if !okV || !okA || !okB {
			return feet, false
		}
	}
	wA, okA := heldRatVec(in.intoA)
	wB, okB := heldRatVec(in.intoB)
	tA, tB := proofarith.FloatRat(in.tA), proofarith.FloatRat(in.tB)
	if !okA || !okB || tA == nil || tB == nil || tA.Sign() <= 0 || tB.Sign() <= 0 {
		return feet, false
	}
	e := v[1].sub(v[0])
	if e.isZero() {
		return feet, false
	}
	uA, okA := squareDirection(e, onA[0].sub(v[0]), wA)
	uB, okB := squareDirection(e, onB[0].sub(v[0]), wB)
	if !okA || !okB {
		return feet, false
	}
	for i := range 2 {
		n := onA[i].sub(v[i]).cross(onB[i].sub(v[i]))
		ne := n.dot(e)
		if n.isZero() || ne.Sign() == 0 {
			return feet, false
		}
		for side, s := range []struct {
			u ratVec
			t *big.Rat
		}{{uA, tA}, {uB, tB}} {
			p, bound, ok := setbackFoot(v[i], e, n, ne, s.u, s.t)
			if !ok {
				return feet, false
			}
			feet[i][side] = lineFoot{p: p, bound: bound}
		}
	}
	return feet, true
}

// squareDirection is the integer direction in a face's plane square to the edge e, from the carrier offset off
// (a carrier minus the edge end), signed to point the way of the float sense w. It reports false when off lies
// on the edge's line or w is not within 30° of the direction.
func squareDirection(e, off, w ratVec) (ratVec, bool) {
	ee := e.dot(e)
	perp := off.scale(ee).sub(e.scale(off.dot(e)))
	u, ok := perp.primitive()
	if !ok {
		return ratVec{}, false
	}
	uw := u.dot(w)
	switch uw.Sign() {
	case 0:
		return ratVec{}, false
	case -1:
		u = u.scale(big.NewRat(-1, 1))
		uw.Neg(uw)
	}
	// (u·w)² ≥ ¾·(u·u)·(w·w): the angle between them is at most 30°.
	lhs := new(big.Rat).Mul(uw, uw)
	rhs := new(big.Rat).Mul(u.dot(u), w.dot(w))
	rhs.Mul(rhs, big.NewRat(3, 4))
	if lhs.Cmp(rhs) < 0 {
		return ratVec{}, false
	}
	return u, true
}

// setbackFoot is the meeting of the setback line V + k·u + λ·e, k = t/|u|, with the end plane through v of normal
// n (ne = n·e): v + k·(u − (n·u/ne)·e), the one square root enclosed and the held float and its proven distance
// read from the box (§4.2).
func setbackFoot(v, e, n ratVec, ne *big.Rat, u ratVec, t *big.Rat) (r3.Vec, float64, bool) {
	// w is u moved along e onto the end plane.
	w := u.sub(e.scale(new(big.Rat).Quo(n.dot(u), ne)))
	q := u.dot(u)
	sq, ok := ratSqrt(q)
	if !ok {
		return r3.Vec{}, 0, false
	}
	// coef is t/q, so a coordinate is v + w·coef·√q.
	coef := new(big.Rat).Quo(t, q)
	var box [3]proofbound.RatInterval
	for c := range box {
		box[c] = proofbound.IntervalAdd(proofbound.PointInterval(v[c]),
			proofbound.IntervalScale(sq, new(big.Rat).Mul(w[c], coef)))
	}
	return proofbound.BoxMidStation(box)
}

// ratSqrt encloses √q for the positive integer q: a point when q is a perfect square, else SqrtFixed's enclosure.
func ratSqrt(q *big.Rat) (proofbound.RatInterval, bool) {
	if q.IsInt() {
		s := new(big.Int).Sqrt(q.Num())
		if new(big.Int).Mul(s, s).Cmp(q.Num()) == 0 {
			return proofbound.PointInterval(new(big.Rat).SetInt(s)), true
		}
	}
	return proofbound.SqrtFixed(q)
}

// chamferCarriers picks cA_i and cB_i (§4.1) from the runs and end-face loops, or ok false. Each is the first
// vertex of its run's chain at end i with bound 0 that a loop of the end face G_i also holds; the edge's own end
// V_i must be a vertex of G_i too, and both ends must have bound 0. A face whose tag states a plane displacement
// keeps today's feet: those charge it into the vertex bounds, and the exact path does not read the tags.
func (w *work) chamferCarriers(l line, runA, runB edgeRun, fG [2]faceLoops) (onA, onB [2]int, ok bool) {
	if l.a.delta != 0 || l.b.delta != 0 {
		return onA, onB, false
	}
	for i := range 2 {
		if w.beta[l.ends[i].v] != 0 || l.ends[i].tilt != 0 || l.ends[i].g.delta != 0 ||
			!holds(fG[i], l.ends[i].v) {
			return onA, onB, false
		}
		var okA, okB bool
		onA[i], okA = w.zeroCarrier(runA.chain[i], fG[i])
		onB[i], okB = w.zeroCarrier(runB.chain[i], fG[i])
		if !okA || !okB {
			return onA, onB, false
		}
	}
	return onA, onB, true
}

// zeroCarrier is the first vertex of chain, walking out from the edge end, with bound 0 that a loop of g holds.
func (w *work) zeroCarrier(chain []int, g faceLoops) (int, bool) {
	for _, v := range chain {
		if w.beta[v] == 0 && holds(g, v) {
			return v, true
		}
	}
	return 0, false
}

// holds reports whether a loop of f holds the vertex v.
func holds(f faceLoops, v int) bool {
	for _, loop := range f.loops {
		if indexIn(loop, v) >= 0 {
			return true
		}
	}
	return false
}

// exactLineFeet is the exact setback points of an R1 chamfer (§4.3), or ok false where the edge keeps today's
// feet: the carriers of §4.1 are missing or the exact construction refuses them. wA and wB are the float ways into
// the faces, tA and tB the setbacks on them.
func (w *work) exactLineFeet(l line, runA, runB edgeRun, fG [2]faceLoops,
	wA, wB r3.Vec, tA, tB float64) ([2][2]lineFoot, bool) {
	onA, onB, ok := w.chamferCarriers(l, runA, runB, fG)
	if !ok {
		return [2][2]lineFoot{}, false
	}
	var in chamferFeetInput
	for i := range 2 {
		in.v[i] = w.verts[l.ends[i].v]
		in.onA[i], in.onB[i] = w.verts[onA[i]], w.verts[onB[i]]
	}
	in.intoA, in.intoB, in.tA, in.tB = wA, wB, tA, tB
	return exactChamferFeet(in)
}
