package pair

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file holds two shortcuts for the distance scan (planarKernel.distances).
// Neither changes an answer. Each one stands in for an exact computation only
// where it yields the same answer as that computation.
//
//   - A float pre-test of the box prune. Each box carries an outward-rounded
//     float copy, and the running minimum carries a float upper bound. Each
//     float step rounds toward a weaker claim. When the float lower bound on
//     the boxes' squared gap still exceeds the minimum's upper bound, the exact
//     gap exceeds the exact minimum, so the exact prune would also prune. In
//     every other case the exact test decides, as before.
//   - Per-triangle side planes for the vertex-facet test. The edge test
//     ((w−u)×(x−u))·n equals (x−u)·(n×(w−u)) exactly, by the scalar triple
//     product, so its sign is the same. The plane n×(w−u) is computed once per
//     triangle edge instead of once per vertex tested against it.
//   - A float pre-test of each side sign. The dot product (x−u)·m is enclosed
//     in a float interval from the outward float boxes of x, u and m, with
//     every operation rounded outward. An interval that excludes zero states
//     the exact sign. An interval that holds zero, or any NaN, defers to the
//     exact dot product.

// floatBox is a closed box rounded outward to floats: lo is at or below the
// exact lower corner and hi at or above the exact upper corner, on every axis.
// A coordinate whose float is not finite widens to an infinity on that side.
type floatBox struct {
	lo, hi [3]float64
}

// floatBounds returns floats lo ≤ d ≤ hi. A float conversion that is not exact
// rounds to the nearest float, so the exact value lies strictly between that
// float's neighbours. That also holds when the conversion overflows to an
// infinity: the infinity's neighbour toward zero is MaxFloat64.
func floatBounds(d proof.Dyadic) (float64, float64) {
	f, exact := d.Float64()
	if exact {
		return f, f
	}
	return math.Nextafter(f, math.Inf(-1)), math.Nextafter(f, math.Inf(1))
}

// floatBoxes fills the outward float boxes of every vertex, triangle and edge
// of p. ClassifyPlanar fills them only when it reaches the distance scan.
func (p *planarPrep) floatBoxes() {
	if p.vertBox != nil {
		return
	}
	p.vertBox = make([]floatBox, len(p.s.Verts))
	for v, point := range p.s.Verts {
		for axis := range 3 {
			p.vertBox[v].lo[axis], p.vertBox[v].hi[axis] = floatBounds(point[axis])
		}
	}
	p.triBox = make([]floatBox, len(p.s.Tris))
	for t, tri := range p.s.Tris {
		p.triBox[t] = p.unionBox(tri[:])
	}
	p.edgeBox = make([]floatBox, len(p.edges))
	for e, edge := range p.edges {
		p.edgeBox[e] = p.unionBox(edge[:])
	}
}

// unionBox is the smallest float box holding the float boxes of the given
// vertices. It holds every exact vertex, so it holds their exact box.
func (p *planarPrep) unionBox(indices []int) floatBox {
	out := p.vertBox[indices[0]]
	for _, v := range indices[1:] {
		for axis := range 3 {
			out.lo[axis] = math.Min(out.lo[axis], p.vertBox[v].lo[axis])
			out.hi[axis] = math.Max(out.hi[axis], p.vertBox[v].hi[axis])
		}
	}
	return out
}

// The directed operations below each round one float operation to nearest
// and then step the result one ulp outward. The exact result lies within half
// an ulp of the rounded one, so the step lands on the far side of it. An
// overflow to an infinity steps to MaxFloat64 toward zero, which the exact
// result also passes. A down result is at or below the exact result and an up
// result at or above it. The explicit float64 conversion of a product stops
// the compiler from fusing it into a later add, which would round differently.
func down(x float64) float64        { return math.Nextafter(x, math.Inf(-1)) }
func up(x float64) float64          { return math.Nextafter(x, math.Inf(1)) }
func addDown(a, b float64) float64  { return down(a + b) }
func addUp(a, b float64) float64    { return up(a + b) }
func subDown(a, b float64) float64  { return down(a - b) }
func subUp(a, b float64) float64    { return up(a - b) }
func mulDown(a, b float64) float64  { return down(float64(a * b)) }
func mulUp(a, b float64) float64    { return up(float64(a * b)) }
func divUp(a, b float64) float64    { return up(a / b) }
func nonnegative(x float64) float64 { return math.Max(x, 0) }

// gapSquaredBelow is a float at or below the exact squared gap of two boxes
// whose float boxes are a and b. Each term is nonnegative, so a bound below
// zero is raised to zero.
func gapSquaredBelow(a, b floatBox) float64 {
	sum := 0.0
	for axis := range 3 {
		var gap float64
		switch {
		case a.hi[axis] < b.lo[axis]:
			gap = nonnegative(subDown(b.lo[axis], a.hi[axis]))
		case b.hi[axis] < a.lo[axis]:
			gap = nonnegative(subDown(a.lo[axis], b.hi[axis]))
		default:
			continue
		}
		sum = nonnegative(addDown(sum, nonnegative(mulDown(gap, gap))))
	}
	return sum
}

// fracAbove is a float at or above x = num/den, or +Inf when no finite bound
// follows from the floats. num is nonnegative and den positive.
func fracAbove(x frac) float64 {
	_, num := floatBounds(x.num)
	den, _ := floatBounds(x.den)
	if !(den > 0) {
		return math.Inf(1)
	}
	return divUp(num, den)
}

// sidePlanes returns the three side planes n×(w−u) of triangle t, one per
// directed edge u->w in winding order, and their outward float boxes,
// computing both on first use.
func (p *planarPrep) sidePlanes(t int) (*[3]proof.DyV3, *[3]floatBox) {
	if p.sides == nil {
		p.sides = make([][3]proof.DyV3, len(p.s.Tris))
		p.sideBoxes = make([][3]floatBox, len(p.s.Tris))
		p.sidesSet = make([]bool, len(p.s.Tris))
	}
	if !p.sidesSet[t] {
		tri := p.s.Tris[t]
		for i := range 3 {
			u, w := p.s.Verts[tri[i]], p.s.Verts[tri[(i+1)%3]]
			plane := proof.DvCross(p.normal[t], proof.DvSub(w, u))
			p.sides[t][i] = plane
			for axis := range 3 {
				p.sideBoxes[t][i].lo[axis], p.sideBoxes[t][i].hi[axis] = floatBounds(plane[axis])
			}
		}
		p.sidesSet[t] = true
	}
	return &p.sides[t], &p.sideBoxes[t]
}

// mulEnclosure encloses every product of a value in [a, b] and one in [c, d]:
// the products lie between the least and greatest corner product. A NaN
// corner, from an infinity times zero, makes both ends NaN.
func mulEnclosure(a, b, c, d float64) (float64, float64) {
	lo := math.Min(math.Min(mulDown(a, c), mulDown(a, d)), math.Min(mulDown(b, c), mulDown(b, d)))
	hi := math.Max(math.Max(mulUp(a, c), mulUp(a, d)), math.Max(mulUp(b, c), mulUp(b, d)))
	return lo, hi
}

// dotEnclosure encloses (x−u)·m for every x, u and m in the float boxes given.
// A NaN, from an infinity times zero, makes the enclosure NaN, which
// floatDotSign treats as undecided. A box's lo is never +Inf and its hi never
// −Inf, so the differences and sums never meet opposed infinities.
func dotEnclosure(x, u, m floatBox) (float64, float64) {
	lo, hi := 0.0, 0.0
	for axis := range 3 {
		plo, phi := mulEnclosure(subDown(x.lo[axis], u.hi[axis]), subUp(x.hi[axis], u.lo[axis]), m.lo[axis], m.hi[axis])
		lo, hi = addDown(lo, plo), addUp(hi, phi)
	}
	return lo, hi
}

// floatDotSign returns the sign of (x−u)·m when its float enclosure excludes
// zero, and false otherwise.
func floatDotSign(x, u, m floatBox) (int, bool) {
	lo, hi := dotEnclosure(x, u, m)
	switch {
	case lo > 0:
		return 1, true
	case hi < 0:
		return -1, true
	}
	return 0, false
}
