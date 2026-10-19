package decad

import (
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type loftVolumeTriangle struct {
	ids     [3]int
	normal  loftcontact.Vec
	minimum loftcontact.Vec
	maximum loftcontact.Vec
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
	c := &loftcontact.Arithmetic{OK: true}
	points := make([]loftcontact.Vec, len(shell.vertices))
	seenPoints := make(map[string]struct{}, len(points))
	for i, v := range shell.vertices {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		points[i] = loftcontact.Vec(v.world)
		var key strings.Builder
		for _, coordinate := range points[i] {
			if !c.Valid(coordinate) {
				return refuse()
			}
			key.WriteString(coordinate.RatString())
			key.WriteByte(',')
		}
		if _, duplicate := seenPoints[key.String()]; duplicate {
			return refuse()
		}
		seenPoints[key.String()] = struct{}{}
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
		normal := c.Cross(c.Diff(b, a), c.Diff(d, a))
		if !c.OK || normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
			return refuse()
		}
		tri := &triangles[i]
		tri.ids, tri.normal = ids, normal
		for axis := range 3 {
			tri.minimum[axis], tri.maximum[axis] = a[axis], a[axis]
			for _, p := range []loftcontact.Vec{b, d} {
				if c.Cmp(p[axis], tri.minimum[axis]) < 0 {
					tri.minimum[axis] = p[axis]
				}
				if c.Cmp(p[axis], tri.maximum[axis]) > 0 {
					tri.maximum[axis] = p[axis]
				}
			}
		}
		if !c.OK {
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
		for j := range i {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			allowed, err := loftVolumePairAllowed(budget, c, points, triangles[i], triangles[j])
			if err != nil {
				return nil, false, err
			}
			if !allowed || !c.OK {
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
		term := c.Dot(c.Diff(a, anchor), c.Cross(c.Diff(b, anchor), c.Diff(d, anchor)))
		sixTimes = c.Add(sixTimes, term)
		if !c.OK {
			return refuse()
		}
	}
	if sixTimes.Sign() <= 0 {
		return refuse()
	}
	volume := c.Quo(sixTimes, loftOutputRat(6))
	if !c.OK || volume.Sign() <= 0 {
		return refuse()
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	return volume, true, nil
}

func loftVolumePairAllowed(budget *proofbound.WorkBudget, c *loftcontact.Arithmetic,
	points []loftcontact.Vec, a, b loftVolumeTriangle) (bool, error) {
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
		if c.Cmp(a.maximum[axis], b.minimum[axis]) < 0 ||
			c.Cmp(b.maximum[axis], a.minimum[axis]) < 0 {
			return c.OK, nil
		}
	}
	if !c.OK {
		return false, nil
	}
	parallel := c.Cross(a.normal, b.normal)
	if !c.OK {
		return false, nil
	}
	if parallel[0].Sign() == 0 && parallel[1].Sign() == 0 && parallel[2].Sign() == 0 {
		if c.Dot(a.normal, c.Diff(points[b.ids[0]], points[a.ids[0]])).Sign() != 0 {
			return c.OK, nil
		}
		return loftVolumeCoplanarAllowed(budget, c, points, a.ids, b.ids, a.normal, shared, count)
	}
	aCut, err := loftVolumePlaneCut(budget, c, points, a.ids, points[b.ids[0]], b.normal)
	if err != nil || len(aCut) == 0 {
		return err == nil && c.OK, err
	}
	bCut, err := loftVolumePlaneCut(budget, c, points, b.ids, points[a.ids[0]], a.normal)
	if err != nil || len(bCut) == 0 {
		return err == nil && c.OK, err
	}
	if !c.OK || len(aCut) > 2 || len(bCut) > 2 {
		return false, nil
	}
	axis := 0
	for parallel[axis].Sign() == 0 {
		axis++
	}
	if len(aCut) == 2 && c.Cmp(aCut[0][axis], aCut[1][axis]) > 0 {
		aCut[0], aCut[1] = aCut[1], aCut[0]
	}
	if len(bCut) == 2 && c.Cmp(bCut[0][axis], bCut[1][axis]) > 0 {
		bCut[0], bCut[1] = bCut[1], bCut[0]
	}
	lo, hi := aCut[0], aCut[len(aCut)-1]
	if c.Cmp(bCut[0][axis], lo[axis]) > 0 {
		lo = bCut[0]
	}
	if c.Cmp(bCut[len(bCut)-1][axis], hi[axis]) < 0 {
		hi = bCut[len(bCut)-1]
	}
	if c.Cmp(lo[axis], hi[axis]) > 0 {
		return c.OK, nil
	}
	return c.OK && loftVolumeAllowedPoint(c, lo, points, shared, count) &&
		loftVolumeAllowedPoint(c, hi, points, shared, count) && c.OK, nil
}

func loftVolumeAllowedPoint(c *loftcontact.Arithmetic, point loftcontact.Vec,
	points []loftcontact.Vec, shared [3]int, count int) bool {
	switch count {
	case 1:
		return c.Same(point, points[shared[0]]) && c.OK
	case 2:
		return c.OnSegment(point, points[shared[0]], points[shared[1]]) && c.OK
	default:
		return false
	}
}

func loftVolumePlaneCut(budget *proofbound.WorkBudget, c *loftcontact.Arithmetic,
	points []loftcontact.Vec, ids [3]int, plane loftcontact.Vec, normal loftcontact.Vec,
) ([]loftcontact.Vec, error) {
	var signed [3]*big.Rat
	for k, id := range ids {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		signed[k] = c.Dot(normal, c.Diff(points[id], plane))
	}
	if !c.OK {
		return nil, nil
	}
	var cut []loftcontact.Vec
	appendUnique := func(point loftcontact.Vec) {
		for _, prior := range cut {
			if c.Same(point, prior) {
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
		t := c.Quo(signed[k], c.Sub(signed[k], signed[j]))
		delta := c.Diff(points[ids[j]], points[id])
		var point loftcontact.Vec
		for axis := range 3 {
			point[axis] = c.Add(points[id][axis], c.Mul(t, delta[axis]))
		}
		if !c.OK {
			return nil, nil
		}
		appendUnique(point)
	}
	return cut, nil
}

func loftVolumeCross2(c *loftcontact.Arithmetic, a, b [2]*big.Rat) *big.Rat {
	return c.Sub(c.Mul(a[0], b[1]), c.Mul(a[1], b[0]))
}

func loftVolumeSub2(c *loftcontact.Arithmetic, a, b [2]*big.Rat) [2]*big.Rat {
	return [2]*big.Rat{c.Sub(a[0], b[0]), c.Sub(a[1], b[1])}
}

func loftVolumePoint2(p loftcontact.Vec, u, v int) [2]*big.Rat {
	return [2]*big.Rat{p[u], p[v]}
}

func loftVolumeInside2(c *loftcontact.Arithmetic, p [2]*big.Rat, tri [3][2]*big.Rat) bool {
	positive, negative := false, false
	for k := range 3 {
		cross := loftVolumeCross2(c, loftVolumeSub2(c, tri[(k+1)%3], tri[k]),
			loftVolumeSub2(c, p, tri[k]))
		positive = positive || cross.Sign() > 0
		negative = negative || cross.Sign() < 0
	}
	return c.OK && (!positive || !negative)
}

func loftVolumeOnSegment2(c *loftcontact.Arithmetic, p, a, b [2]*big.Rat) bool {
	if loftVolumeCross2(c, loftVolumeSub2(c, p, a), loftVolumeSub2(c, b, a)).Sign() != 0 {
		return false
	}
	for axis := range 2 {
		if c.Cmp(p[axis], a[axis]) < 0 && c.Cmp(p[axis], b[axis]) < 0 ||
			c.Cmp(p[axis], a[axis]) > 0 && c.Cmp(p[axis], b[axis]) > 0 {
			return false
		}
	}
	return c.OK
}

func loftVolumeAllowed2(c *loftcontact.Arithmetic, p [2]*big.Rat,
	points []loftcontact.Vec, shared [3]int, count, u, v int) bool {
	switch count {
	case 1:
		q := loftVolumePoint2(points[shared[0]], u, v)
		return c.Cmp(p[0], q[0]) == 0 && c.Cmp(p[1], q[1]) == 0 && c.OK
	case 2:
		return loftVolumeOnSegment2(c, p, loftVolumePoint2(points[shared[0]], u, v),
			loftVolumePoint2(points[shared[1]], u, v)) && c.OK
	default:
		return false
	}
}

func loftVolumeCoplanarAllowed(budget *proofbound.WorkBudget, c *loftcontact.Arithmetic,
	points []loftcontact.Vec, ai, bi [3]int, normal loftcontact.Vec, shared [3]int, count int,
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
		if !c.OK {
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
			t := c.Quo(loftVolumeCross2(c, qp, s), den)
			w := c.Quo(loftVolumeCross2(c, qp, r), den)
			if c.Cmp(t, loftOutputRat(0)) < 0 || c.Cmp(t, loftOutputRat(1)) > 0 ||
				c.Cmp(w, loftOutputRat(0)) < 0 || c.Cmp(w, loftOutputRat(1)) > 0 {
				continue
			}
			intersection := [2]*big.Rat{c.Add(p[0], c.Mul(t, r[0])), c.Add(p[1], c.Mul(t, r[1]))}
			if !loftVolumeAllowed2(c, intersection, points, shared, count, u, v) {
				return false, nil
			}
		}
	}
	return c.OK, nil
}
