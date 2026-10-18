package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The ceiling applies to the numerator and denominator of every source
// coordinate and every rational intermediate, including unreduced products.
const loftVolumeRationalBits = 8192

type loftVolumeVec [3]*big.Rat

type loftVolumeTriangle struct {
	ids     [3]int
	normal  loftVolumeVec
	minimum loftVolumeVec
	maximum loftVolumeVec
}

// loftVolumeArithmetic refuses before allocating an intermediate whose
// numerator or denominator could exceed the fixed rational-size ceiling.
type loftVolumeArithmetic struct{ ok bool }

func (c *loftVolumeArithmetic) valid(a *big.Rat) bool {
	if !c.ok || a == nil || a.Num().BitLen() > loftVolumeRationalBits ||
		a.Denom().BitLen() > loftVolumeRationalBits {
		c.ok = false
	}
	return c.ok
}

func (c *loftVolumeArithmetic) add(a, b *big.Rat) *big.Rat {
	if !c.valid(a) || !c.valid(b) ||
		max(a.Num().BitLen()+b.Denom().BitLen(), b.Num().BitLen()+a.Denom().BitLen())+1 > loftVolumeRationalBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > loftVolumeRationalBits {
		c.ok = false
		return new(big.Rat)
	}
	return new(big.Rat).Add(a, b)
}

func (c *loftVolumeArithmetic) sub(a, b *big.Rat) *big.Rat {
	if !c.valid(a) || !c.valid(b) ||
		max(a.Num().BitLen()+b.Denom().BitLen(), b.Num().BitLen()+a.Denom().BitLen())+1 > loftVolumeRationalBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > loftVolumeRationalBits {
		c.ok = false
		return new(big.Rat)
	}
	return new(big.Rat).Sub(a, b)
}

func (c *loftVolumeArithmetic) mul(a, b *big.Rat) *big.Rat {
	if !c.valid(a) || !c.valid(b) ||
		a.Num().BitLen()+b.Num().BitLen() > loftVolumeRationalBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > loftVolumeRationalBits {
		c.ok = false
		return new(big.Rat)
	}
	return new(big.Rat).Mul(a, b)
}

func (c *loftVolumeArithmetic) quo(a, b *big.Rat) *big.Rat {
	if !c.valid(a) || !c.valid(b) || b.Sign() == 0 ||
		a.Num().BitLen()+b.Denom().BitLen() > loftVolumeRationalBits ||
		a.Denom().BitLen()+b.Num().BitLen() > loftVolumeRationalBits {
		c.ok = false
		return new(big.Rat)
	}
	return new(big.Rat).Quo(a, b)
}

func (c *loftVolumeArithmetic) cmp(a, b *big.Rat) int {
	if !c.valid(a) || !c.valid(b) ||
		a.Num().BitLen()+b.Denom().BitLen() > loftVolumeRationalBits ||
		b.Num().BitLen()+a.Denom().BitLen() > loftVolumeRationalBits {
		c.ok = false
		return 0
	}
	return a.Cmp(b)
}

func (c *loftVolumeArithmetic) diff(a, b loftVolumeVec) loftVolumeVec {
	return loftVolumeVec{c.sub(a[0], b[0]), c.sub(a[1], b[1]), c.sub(a[2], b[2])}
}

func (c *loftVolumeArithmetic) cross(a, b loftVolumeVec) loftVolumeVec {
	return loftVolumeVec{
		c.sub(c.mul(a[1], b[2]), c.mul(a[2], b[1])),
		c.sub(c.mul(a[2], b[0]), c.mul(a[0], b[2])),
		c.sub(c.mul(a[0], b[1]), c.mul(a[1], b[0])),
	}
}

func (c *loftVolumeArithmetic) dot(a, b loftVolumeVec) *big.Rat {
	return c.add(c.add(c.mul(a[0], b[0]), c.mul(a[1], b[1])), c.mul(a[2], b[2]))
}

func (c *loftVolumeArithmetic) same(a, b loftVolumeVec) bool {
	return c.cmp(a[0], b[0]) == 0 && c.cmp(a[1], b[1]) == 0 && c.cmp(a[2], b[2]) == 0
}

func (c *loftVolumeArithmetic) onSegment(p, a, b loftVolumeVec) bool {
	delta := c.diff(b, a)
	axis := 0
	for axis < 3 && delta[axis].Sign() == 0 {
		axis++
	}
	if axis == 3 {
		return c.same(p, a)
	}
	t := c.quo(c.sub(p[axis], a[axis]), delta[axis])
	if c.cmp(t, loftOutputRat(0)) < 0 || c.cmp(t, loftOutputRat(1)) > 0 {
		return false
	}
	for k := range 3 {
		if c.cmp(p[k], c.add(a[k], c.mul(t, delta[k]))) != 0 {
			return false
		}
	}
	return c.ok
}

// loftVolumeExactShell returns the positive exact source-shell volume only
// when the bounded exact surface is one embedded, outward-oriented component.
// The caller must first authenticate source occupancy and patch coverage with
// certifyLoftOutputBoundary; this function proves the remaining global shell
// geometry. It uses the caller's shared output budget. Refusal has no witness.
func loftVolumeExactShell(budget *proofbound.WorkBudget, shell loftOutputShell) (*big.Rat, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	refuse := func() (*big.Rat, bool, error) { return nil, false, budget.Err() }
	if len(shell.vertices) < 4 || len(shell.vertices) > 512 ||
		len(shell.triangles) < 4 || len(shell.triangles) > 512 {
		return refuse()
	}
	c := &loftVolumeArithmetic{ok: true}
	points := make([]loftVolumeVec, len(shell.vertices))
	seenPoints := make(map[string]struct{}, len(points))
	for i, v := range shell.vertices {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		points[i] = loftVolumeVec(v.world)
		key := ""
		for _, coordinate := range points[i] {
			if !c.valid(coordinate) {
				return refuse()
			}
			key += coordinate.RatString() + ","
		}
		if _, duplicate := seenPoints[key]; duplicate {
			return refuse()
		}
		seenPoints[key] = struct{}{}
	}
	triangles := make([]loftVolumeTriangle, len(shell.triangles))
	directed := make(map[[2]int][2]int, 3*len(triangles))
	incident := make([][]int, len(points))
	for i, ids := range shell.triangles {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		for _, id := range ids {
			if id < 0 || id >= len(points) {
				return refuse()
			}
			incident[id] = append(incident[id], i)
		}
		if ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] {
			return refuse()
		}
		a, b, d := points[ids[0]], points[ids[1]], points[ids[2]]
		normal := c.cross(c.diff(b, a), c.diff(d, a))
		if !c.ok || normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
			return refuse()
		}
		tri := &triangles[i]
		tri.ids, tri.normal = ids, normal
		for axis := range 3 {
			tri.minimum[axis], tri.maximum[axis] = a[axis], a[axis]
			for _, p := range []loftVolumeVec{b, d} {
				if c.cmp(p[axis], tri.minimum[axis]) < 0 {
					tri.minimum[axis] = p[axis]
				}
				if c.cmp(p[axis], tri.maximum[axis]) > 0 {
					tri.maximum[axis] = p[axis]
				}
			}
		}
		if !c.ok {
			return refuse()
		}
		for edge := range 3 {
			key := [2]int{ids[edge], ids[(edge+1)%3]}
			if _, duplicate := directed[key]; duplicate {
				return refuse()
			}
			directed[key] = [2]int{i, edge}
		}
	}
	peers := make([][3]int, len(triangles))
	for edge, use := range directed {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		other, found := directed[[2]int{edge[1], edge[0]}]
		if !found || other[0] == use[0] {
			return refuse()
		}
		peers[use[0]][use[1]] = other[0]
	}
	connected := make([]bool, len(triangles))
	connected[0] = true
	queue := []int{0}
	for len(queue) > 0 {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		face := queue[0]
		queue = queue[1:]
		for _, peer := range peers[face] {
			if !connected[peer] {
				connected[peer] = true
				queue = append(queue, peer)
			}
		}
	}
	for _, joined := range connected {
		if !joined {
			return refuse()
		}
	}
	// A closed connected edge graph may still pinch at a vertex. Every vertex
	// link must be one cycle before the signed volume can name one interior.
	for vertex, faces := range incident {
		if len(faces) < 3 {
			return refuse()
		}
		link := map[int]struct{}{faces[0]: {}}
		queue = []int{faces[0]}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			face := queue[0]
			queue = queue[1:]
			for edge, id := range triangles[face].ids {
				if id != vertex {
					continue
				}
				for _, adjacent := range [2]int{edge, (edge + 2) % 3} {
					peer := peers[face][adjacent]
					if _, found := link[peer]; !found {
						link[peer] = struct{}{}
						queue = append(queue, peer)
					}
				}
			}
		}
		if len(link) != len(faces) {
			return refuse()
		}
	}
	for i := range triangles {
		for j := 0; j < i; j++ {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			allowed, err := loftVolumePairAllowed(budget, c, points, triangles[i], triangles[j])
			if err != nil {
				return nil, false, err
			}
			if !allowed || !c.ok {
				return refuse()
			}
		}
	}
	anchor := points[0]
	sixTimes := new(big.Rat)
	for _, tri := range triangles {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		a, b, d := points[tri.ids[0]], points[tri.ids[1]], points[tri.ids[2]]
		term := c.dot(c.diff(a, anchor), c.cross(c.diff(b, anchor), c.diff(d, anchor)))
		sixTimes = c.add(sixTimes, term)
		if !c.ok {
			return refuse()
		}
	}
	if sixTimes.Sign() <= 0 {
		return refuse()
	}
	volume := c.quo(sixTimes, loftOutputRat(6))
	if !c.ok || volume.Sign() <= 0 {
		return refuse()
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	return volume, true, nil
}

func loftVolumePairAllowed(budget *proofbound.WorkBudget, c *loftVolumeArithmetic,
	points []loftVolumeVec, a, b loftVolumeTriangle) (bool, error) {
	var shared [3]int
	count := 0
	for _, ai := range a.ids {
		for _, bi := range b.ids {
			if ai == bi {
				shared[count] = ai
				count++
			}
		}
	}
	if count == 3 {
		return false, nil
	}
	for axis := range 3 {
		if c.cmp(a.maximum[axis], b.minimum[axis]) < 0 ||
			c.cmp(b.maximum[axis], a.minimum[axis]) < 0 {
			return c.ok, nil
		}
	}
	if !c.ok {
		return false, nil
	}
	parallel := c.cross(a.normal, b.normal)
	if !c.ok {
		return false, nil
	}
	if parallel[0].Sign() == 0 && parallel[1].Sign() == 0 && parallel[2].Sign() == 0 {
		if c.dot(a.normal, c.diff(points[b.ids[0]], points[a.ids[0]])).Sign() != 0 {
			return c.ok, nil
		}
		return loftVolumeCoplanarAllowed(budget, c, points, a.ids, b.ids, a.normal, shared, count)
	}
	aCut, err := loftVolumePlaneCut(budget, c, points, a.ids, points[b.ids[0]], b.normal)
	if err != nil || len(aCut) == 0 {
		return err == nil && c.ok, err
	}
	bCut, err := loftVolumePlaneCut(budget, c, points, b.ids, points[a.ids[0]], a.normal)
	if err != nil || len(bCut) == 0 {
		return err == nil && c.ok, err
	}
	if !c.ok || len(aCut) > 2 || len(bCut) > 2 {
		return false, nil
	}
	axis := 0
	for parallel[axis].Sign() == 0 {
		axis++
	}
	if len(aCut) == 2 && c.cmp(aCut[0][axis], aCut[1][axis]) > 0 {
		aCut[0], aCut[1] = aCut[1], aCut[0]
	}
	if len(bCut) == 2 && c.cmp(bCut[0][axis], bCut[1][axis]) > 0 {
		bCut[0], bCut[1] = bCut[1], bCut[0]
	}
	lo, hi := aCut[0], aCut[len(aCut)-1]
	if c.cmp(bCut[0][axis], lo[axis]) > 0 {
		lo = bCut[0]
	}
	if c.cmp(bCut[len(bCut)-1][axis], hi[axis]) < 0 {
		hi = bCut[len(bCut)-1]
	}
	if c.cmp(lo[axis], hi[axis]) > 0 {
		return c.ok, nil
	}
	return c.ok && loftVolumeAllowedPoint(c, lo, points, shared, count) &&
		loftVolumeAllowedPoint(c, hi, points, shared, count) && c.ok, nil
}

func loftVolumeAllowedPoint(c *loftVolumeArithmetic, point loftVolumeVec,
	points []loftVolumeVec, shared [3]int, count int) bool {
	switch count {
	case 1:
		return c.same(point, points[shared[0]]) && c.ok
	case 2:
		return c.onSegment(point, points[shared[0]], points[shared[1]]) && c.ok
	default:
		return false
	}
}

func loftVolumePlaneCut(budget *proofbound.WorkBudget, c *loftVolumeArithmetic,
	points []loftVolumeVec, ids [3]int, plane loftVolumeVec, normal loftVolumeVec,
) ([]loftVolumeVec, error) {
	var signed [3]*big.Rat
	for k, id := range ids {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		signed[k] = c.dot(normal, c.diff(points[id], plane))
	}
	if !c.ok {
		return nil, nil
	}
	var cut []loftVolumeVec
	appendUnique := func(point loftVolumeVec) {
		for _, prior := range cut {
			if c.same(point, prior) {
				return
			}
		}
		cut = append(cut, point)
	}
	for k, id := range ids {
		if signed[k].Sign() == 0 {
			appendUnique(points[id])
		}
		j := (k + 1) % 3
		if signed[k].Sign()*signed[j].Sign() >= 0 {
			continue
		}
		if err := budget.Step(); err != nil {
			return nil, err
		}
		t := c.quo(signed[k], c.sub(signed[k], signed[j]))
		delta := c.diff(points[ids[j]], points[id])
		var point loftVolumeVec
		for axis := range 3 {
			point[axis] = c.add(points[id][axis], c.mul(t, delta[axis]))
		}
		if !c.ok {
			return nil, nil
		}
		appendUnique(point)
	}
	return cut, nil
}

func loftVolumeCross2(c *loftVolumeArithmetic, a, b [2]*big.Rat) *big.Rat {
	return c.sub(c.mul(a[0], b[1]), c.mul(a[1], b[0]))
}

func loftVolumeSub2(c *loftVolumeArithmetic, a, b [2]*big.Rat) [2]*big.Rat {
	return [2]*big.Rat{c.sub(a[0], b[0]), c.sub(a[1], b[1])}
}

func loftVolumePoint2(p loftVolumeVec, u, v int) [2]*big.Rat {
	return [2]*big.Rat{p[u], p[v]}
}

func loftVolumeInside2(c *loftVolumeArithmetic, p [2]*big.Rat, tri [3][2]*big.Rat) bool {
	positive, negative := false, false
	for k := range 3 {
		cross := loftVolumeCross2(c, loftVolumeSub2(c, tri[(k+1)%3], tri[k]),
			loftVolumeSub2(c, p, tri[k]))
		positive = positive || cross.Sign() > 0
		negative = negative || cross.Sign() < 0
	}
	return c.ok && !(positive && negative)
}

func loftVolumeOnSegment2(c *loftVolumeArithmetic, p, a, b [2]*big.Rat) bool {
	if loftVolumeCross2(c, loftVolumeSub2(c, p, a), loftVolumeSub2(c, b, a)).Sign() != 0 {
		return false
	}
	for axis := range 2 {
		if c.cmp(p[axis], a[axis]) < 0 && c.cmp(p[axis], b[axis]) < 0 ||
			c.cmp(p[axis], a[axis]) > 0 && c.cmp(p[axis], b[axis]) > 0 {
			return false
		}
	}
	return c.ok
}

func loftVolumeAllowed2(c *loftVolumeArithmetic, p [2]*big.Rat,
	points []loftVolumeVec, shared [3]int, count, u, v int) bool {
	switch count {
	case 1:
		q := loftVolumePoint2(points[shared[0]], u, v)
		return c.cmp(p[0], q[0]) == 0 && c.cmp(p[1], q[1]) == 0 && c.ok
	case 2:
		return loftVolumeOnSegment2(c, p, loftVolumePoint2(points[shared[0]], u, v),
			loftVolumePoint2(points[shared[1]], u, v)) && c.ok
	default:
		return false
	}
}

func loftVolumeCoplanarAllowed(budget *proofbound.WorkBudget, c *loftVolumeArithmetic,
	points []loftVolumeVec, ai, bi [3]int, normal loftVolumeVec, shared [3]int, count int,
) (bool, error) {
	drop := 0
	for normal[drop].Sign() == 0 {
		drop++
	}
	u, v := (drop+1)%3, (drop+2)%3
	var a, b [3][2]*big.Rat
	for k := range 3 {
		a[k] = loftVolumePoint2(points[ai[k]], u, v)
		b[k] = loftVolumePoint2(points[bi[k]], u, v)
	}
	for k := range 3 {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if loftVolumeInside2(c, a[k], b) && !loftVolumeAllowed2(c, a[k], points, shared, count, u, v) ||
			loftVolumeInside2(c, b[k], a) && !loftVolumeAllowed2(c, b[k], points, shared, count, u, v) {
			return false, nil
		}
		if !c.ok {
			return false, nil
		}
	}
	for i := range 3 {
		for j := range 3 {
			if err := budget.Step(); err != nil {
				return false, err
			}
			p, q := a[i], b[j]
			r := loftVolumeSub2(c, a[(i+1)%3], p)
			s := loftVolumeSub2(c, b[(j+1)%3], q)
			den := loftVolumeCross2(c, r, s)
			qp := loftVolumeSub2(c, q, p)
			if den.Sign() == 0 {
				if loftVolumeCross2(c, qp, r).Sign() != 0 {
					continue
				}
				for _, candidate := range [4][2]*big.Rat{a[i], a[(i+1)%3], b[j], b[(j+1)%3]} {
					if loftVolumeOnSegment2(c, candidate, p, a[(i+1)%3]) &&
						loftVolumeOnSegment2(c, candidate, q, b[(j+1)%3]) &&
						!loftVolumeAllowed2(c, candidate, points, shared, count, u, v) {
						return false, nil
					}
				}
				continue
			}
			t := c.quo(loftVolumeCross2(c, qp, s), den)
			w := c.quo(loftVolumeCross2(c, qp, r), den)
			if c.cmp(t, loftOutputRat(0)) < 0 || c.cmp(t, loftOutputRat(1)) > 0 ||
				c.cmp(w, loftOutputRat(0)) < 0 || c.cmp(w, loftOutputRat(1)) > 0 {
				continue
			}
			intersection := [2]*big.Rat{c.add(p[0], c.mul(t, r[0])), c.add(p[1], c.mul(t, r[1]))}
			if !loftVolumeAllowed2(c, intersection, points, shared, count, u, v) {
				return false, nil
			}
		}
	}
	return c.ok, nil
}
