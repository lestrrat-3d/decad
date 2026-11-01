package retainedblend

import (
	"cmp"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// pt2 is a point in a face plane's (u, w) coordinates.
type pt2 struct{ u, w float64 }

type planarEar struct {
	at      int
	quality float64
}

// sortPlanarEars orders ears best quality first, then by position. A NaN
// quality sorts after every number, so the order is total.
func sortPlanarEars(ears []planarEar) {
	slices.SortFunc(ears, comparePlanarEars)
}

func comparePlanarEars(a, b planarEar) int {
	return cmp.Or(cmp.Compare(b.quality, a.quality), cmp.Compare(a.at, b.at))
}

func bestPlanarEar(ears []planarEar) (planarEar, bool) {
	if len(ears) == 0 {
		return planarEar{}, false
	}
	best := ears[0]
	for _, ear := range ears[1:] {
		if comparePlanarEars(ear, best) < 0 {
			best = ear
		}
	}
	return best, true
}

// orient is the sign of (b − a) × (c − a): +1 for a left turn, −1 for a right
// turn, 0 for collinear points. A float evaluation decides it when its
// rounding cannot flip the sign, and exact rational arithmetic decides it
// otherwise.
func orient(a, b, c pt2) int {
	left := (b.u - a.u) * (c.w - a.w)
	right := (b.w - a.w) * (c.u - a.u)
	d := left - right
	if bound := 8 * 0x1p-53 * (math.Abs(left) + math.Abs(right)); d > bound {
		return 1
	} else if d < -bound {
		return -1
	}
	q := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	sub := func(x, y float64) *big.Rat { return new(big.Rat).Sub(q(x), q(y)) }
	l := new(big.Rat).Mul(sub(b.u, a.u), sub(c.w, a.w))
	r := new(big.Rat).Mul(sub(b.w, a.w), sub(c.u, a.u))
	return l.Cmp(r)
}

// signedArea2 is twice the signed area of the polygon through ps, positive when
// it runs counter-clockwise in (u, w).
func signedArea2(ps []pt2, loop []int) float64 {
	s := 0.0
	for i, vi := range loop {
		a, b := ps[vi], ps[loop[(i+1)%len(loop)]]
		s += a.u*b.w - b.u*a.w
	}
	return s
}

func samePt(a, b pt2) bool { return a.u == b.u && a.w == b.w }

// segmentsTouch reports whether the closed segments ab and cd share a point.
func segmentsTouch(a, b, c, d pt2) bool {
	o1, o2 := orient(a, b, c), orient(a, b, d)
	o3, o4 := orient(c, d, a), orient(c, d, b)
	if o1*o2 < 0 && o3*o4 < 0 {
		return true
	}
	on := func(p, q, r pt2) bool {
		return math.Min(p.u, q.u) <= r.u && r.u <= math.Max(p.u, q.u) &&
			math.Min(p.w, q.w) <= r.w && r.w <= math.Max(p.w, q.w)
	}
	return (o1 == 0 && on(a, b, c)) || (o2 == 0 && on(a, b, d)) ||
		(o3 == 0 && on(c, d, a)) || (o4 == 0 && on(c, d, b))
}

// insidePolygon reports whether p lies inside the polygon through ps by the
// crossing-number test.
func insidePolygon(ps []pt2, loop []int, p pt2) bool {
	in := false
	for i, vi := range loop {
		a, b := ps[vi], ps[loop[(i+1)%len(loop)]]
		if (a.w > p.w) != (b.w > p.w) && p.u < (b.u-a.u)*(p.w-a.w)/(b.w-a.w)+a.u {
			in = !in
		}
	}
	return in
}

// triangulateRegion triangulates the region inside the counter-clockwise loop
// outer and outside the clockwise loops holes. Every loop vertex appears in
// the output, including collinear ones, so the region's boundary keeps the
// vertices its neighbours share. ps indexes the vertices; the triangles are
// counter-clockwise in (u, w).
func triangulateRegion(ps []pt2, outer []int, holes [][]int) ([][3]int, error) {
	merged := append([]int(nil), outer...)
	remaining := append([][]int(nil), holes...)
	for len(remaining) > 0 {
		var err error
		merged, remaining, err = bridgeNearest(ps, merged, remaining)
		if err != nil {
			return nil, err
		}
	}
	tris, err := earClip(ps, merged)
	if err != nil {
		return nil, err
	}
	return flipSlivers(ps, tris), nil
}

// bridgeNearest splices one hole into the merged polygon along the shortest
// segment that crosses no edge and runs through the region.
func bridgeNearest(ps []pt2, merged []int, holes [][]int) ([]int, [][]int, error) {
	type candidate struct {
		d       float64
		m, h    int // position in merged, position in the hole
		holeIdx int
	}
	var cands []candidate
	for hi, hole := range holes {
		for hk, hv := range hole {
			for mi, mv := range merged {
				du, dw := ps[hv].u-ps[mv].u, ps[hv].w-ps[mv].w
				cands = append(cands, candidate{d: du*du + dw*dw, m: mi, h: hk, holeIdx: hi})
			}
		}
	}
	// cmp.Compare orders NaN before every number, so the order stays total.
	slices.SortFunc(cands, func(a, b candidate) int { return cmp.Compare(a.d, b.d) })
	for _, c := range cands {
		hole := holes[c.holeIdx]
		a, b := ps[merged[c.m]], ps[hole[c.h]]
		if !bridgeClear(ps, merged, holes, c.holeIdx, c.m, c.h) {
			continue
		}
		mid := pt2{(a.u + b.u) / 2, (a.w + b.w) / 2}
		if !insidePolygon(ps, merged, mid) {
			continue
		}
		spliced := make([]int, 0, len(merged)+len(hole)+2)
		spliced = append(spliced, merged[:c.m+1]...)
		for k := range len(hole) + 1 {
			spliced = append(spliced, hole[(c.h+k)%len(hole)])
		}
		spliced = append(spliced, merged[c.m])
		spliced = append(spliced, merged[c.m+1:]...)
		rest := append([][]int(nil), holes[:c.holeIdx]...)
		rest = append(rest, holes[c.holeIdx+1:]...)
		return spliced, rest, nil
	}
	return nil, nil, fmt.Errorf("%w: no segment joins a hole of the face to its outer boundary", decaderr.ErrUnsupported)
}

// bridgeClear reports whether the segment from merged[m] to holes[hole][h]
// meets no polygon edge except at its own two end points.
func bridgeClear(ps []pt2, merged []int, holes [][]int, hole, m, h int) bool {
	a, b := ps[merged[m]], ps[holes[hole][h]]
	if samePt(a, b) {
		return false
	}
	blocked := func(loop []int) bool {
		for i, vi := range loop {
			p, q := ps[vi], ps[loop[(i+1)%len(loop)]]
			switch {
			case samePt(p, a) || samePt(p, b):
				if collinearOverlap(a, b, p, q) {
					return true
				}
			case samePt(q, a) || samePt(q, b):
				if collinearOverlap(a, b, q, p) {
					return true
				}
			case segmentsTouch(a, b, p, q):
				return true
			}
		}
		return false
	}
	return !blocked(merged) && !slices.ContainsFunc(holes, blocked)
}

// collinearOverlap reports whether segment ab and the edge end→other, which
// meets ab at end, run along each other.
func collinearOverlap(a, b, end, other pt2) bool {
	if orient(a, b, other) != 0 {
		return false
	}
	far := a
	if samePt(end, a) {
		far = b
	}
	du, dw := far.u-end.u, far.w-end.w
	return (other.u-end.u)*du+(other.w-end.w)*dw > 0
}

// earClip triangulates the weakly simple counter-clockwise polygon poly by
// clipping strictly convex ears that no other vertex touches. A vertex at the
// same position as an ear corner (a bridge's second copy) does not block it.
// Of the ears that are free it clips the best shaped first, which keeps the
// nearly collinear vertices of a straight boundary run out of triangles of
// their own: such a sliver is exact in the plane but can fold when a later
// placement rounds its vertices.
func earClip(ps []pt2, poly []int) ([][3]int, error) {
	idx := append([]int(nil), poly...)
	tris := make([][3]int, 0, len(idx))
	var ears []planarEar
	for len(idx) > 3 {
		ears = ears[:0]
		n := len(idx)
		for i := range n {
			a, b, c := ps[idx[(i+n-1)%n]], ps[idx[i]], ps[idx[(i+1)%n]]
			if orient(a, b, c) <= 0 {
				continue
			}
			ears = append(ears, planarEar{i, earQuality(a, b, c)})
		}
		best, ok := bestPlanarEar(ears)
		if !ok {
			return nil, fmt.Errorf("%w: the face's boundary has no ear to clip", decaderr.ErrUnsupported)
		}
		ia, ib, ic := idx[(best.at+n-1)%n], idx[best.at], idx[(best.at+1)%n]
		if earBlocked(ps, idx, ia, ib, ic) {
			sortPlanarEars(ears)
			found := false
			for _, e := range ears[1:] {
				ia, ib, ic = idx[(e.at+n-1)%n], idx[e.at], idx[(e.at+1)%n]
				if earBlocked(ps, idx, ia, ib, ic) {
					continue
				}
				best = e
				found = true
				break
			}
			if !found {
				return nil, fmt.Errorf("%w: the face's boundary has no ear to clip", decaderr.ErrUnsupported)
			}
		}
		tris = append(tris, [3]int{ia, ib, ic})
		idx = append(idx[:best.at], idx[best.at+1:]...)
	}
	if orient(ps[idx[0]], ps[idx[1]], ps[idx[2]]) <= 0 {
		return nil, fmt.Errorf("%w: the face's boundary closes on a degenerate triangle", decaderr.ErrUnsupported)
	}
	return append(tris, [3]int{idx[0], idx[1], idx[2]}), nil
}

// earQuality is twice the triangle's area over the square of its longest side:
// zero for a sliver, about 0.87 for an equilateral triangle.
func earQuality(a, b, c pt2) float64 {
	area2 := math.Abs((b.u-a.u)*(c.w-a.w) - (b.w-a.w)*(c.u-a.u))
	long := 0.0
	du, dw := b.u-a.u, b.w-a.w
	long = math.Max(long, du*du+dw*dw)
	du, dw = c.u-b.u, c.w-b.w
	long = math.Max(long, du*du+dw*dw)
	du, dw = a.u-c.u, a.w-c.w
	long = math.Max(long, du*du+dw*dw)
	if long == 0 {
		return 0
	}
	return area2 / long
}

func earBlocked(ps []pt2, idx []int, ia, ib, ic int) bool {
	a, b, c := ps[ia], ps[ib], ps[ic]
	if finitePt(a) && finitePt(b) && finitePt(c) {
		// Keep non-finite points on the old exact-predicate path. The box proof
		// only holds for finite coordinates, and orient owns their behavior.
		minU, maxU := math.Min(a.u, math.Min(b.u, c.u)), math.Max(a.u, math.Max(b.u, c.u))
		minW, maxW := math.Min(a.w, math.Min(b.w, c.w)), math.Max(a.w, math.Max(b.w, c.w))
		for _, v := range idx {
			p := ps[v]
			if samePt(p, a) || samePt(p, b) || samePt(p, c) {
				continue
			}
			if finitePt(p) && (p.u < minU || p.u > maxU || p.w < minW || p.w > maxW) {
				continue
			}
			if orient(a, b, p) >= 0 && orient(b, c, p) >= 0 && orient(c, a, p) >= 0 {
				return true
			}
		}
		return false
	}
	for _, v := range idx {
		p := ps[v]
		if samePt(p, a) || samePt(p, b) || samePt(p, c) {
			continue
		}
		if orient(a, b, p) >= 0 && orient(b, c, p) >= 0 && orient(c, a, p) >= 0 {
			return true
		}
	}
	return false
}

func finitePt(p pt2) bool {
	return !math.IsNaN(p.u) && !math.IsInf(p.u, 0) && !math.IsNaN(p.w) && !math.IsInf(p.w, 0)
}

// sliverQuality is the earQuality below which a triangle counts as a sliver.
const sliverQuality = 1e-6

type undirected struct{ a, b int }

func edgeKey(a, b int) undirected { return undirected{min(a, b), max(a, b)} }

// flipSlivers replaces the long edge of each sliver by the other diagonal of
// the quadrilateral it closes, when both new triangles are counter-clockwise
// and each is better shaped than the sliver. A run of collinear boundary
// vertices ends as a fan of triangles from one apex, not as triangles that join
// three of its own vertices.
func flipSlivers(ps []pt2, tris [][3]int) [][3]int {
	quality := func(t [3]int) float64 { return earQuality(ps[t[0]], ps[t[1]], ps[t[2]]) }
	for range 4 * len(tris) {
		owners := map[undirected][]int{}
		for i, t := range tris {
			for k := range 3 {
				key := edgeKey(t[k], t[(k+1)%3])
				owners[key] = append(owners[key], i)
			}
		}
		flipped := false
		for i, t := range tris {
			q := quality(t)
			if q >= sliverQuality {
				continue
			}
			// The sliver's long edge is the one opposite its flat vertex.
			lenSq := func(a, b int) float64 {
				du, dw := ps[a].u-ps[b].u, ps[a].w-ps[b].w
				return du*du + dw*dw
			}
			long := 0
			for k := 1; k < 3; k++ {
				if lenSq(t[k], t[(k+1)%3]) > lenSq(t[long], t[(long+1)%3]) {
					long = k
				}
			}
			p, qv, x := t[long], t[(long+1)%3], t[(long+2)%3]
			shared := owners[edgeKey(p, qv)]
			if len(shared) != 2 {
				continue
			}
			j := shared[0]
			if j == i {
				j = shared[1]
			}
			var y int
			for _, v := range tris[j] {
				if v != p && v != qv {
					y = v
				}
			}
			// The sliver is (p, qv, x) and its neighbour holds (qv, p, y).
			first, second := [3]int{p, y, x}, [3]int{y, qv, x}
			if orient(ps[first[0]], ps[first[1]], ps[first[2]]) <= 0 ||
				orient(ps[second[0]], ps[second[1]], ps[second[2]]) <= 0 {
				continue
			}
			if min(quality(first), quality(second)) <= 2*min(q, quality(tris[j])) {
				continue
			}
			tris[i], tris[j] = first, second
			flipped = true
			break
		}
		if !flipped {
			break
		}
	}
	return tris
}
