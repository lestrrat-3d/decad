package pointquery

import (
	"context"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// distanceLeafSize is the most triangles a distance index leaf holds.
const distanceLeafSize = 8

// distanceIndexFits reports whether n triangles can be addressed by the
// int32 fields of distanceNode and distanceIndex.order.
func distanceIndexFits(n int) bool { return n <= math.MaxInt32 }

// distancePollInterval is how many heap pops a distance query makes between
// context checks.
const distancePollInterval = 256

// distanceNode is one node of a distanceIndex: the closed float box of its
// triangles, and either two children or a leaf run of order slots.
type distanceNode struct {
	lo, hi               [3]float64
	left, right          int32
	leafStart, leafCount int32
}

// distanceIndex answers MeshDistanceSquared for one immutable held mesh whose
// triangles were all checked once. A mesh that fails the check is not clean
// and keeps MeshDistanceSquared. After construction the index is read-only,
// so concurrent queries share it without a lock.
type distanceIndex struct {
	clean bool
	nodes []distanceNode
	// order lists triangle indices in leaf order; slot s of boxLo, boxHi is
	// the float box of triangle order[s].
	order        []int32
	boxLo, boxHi [][3]float64
	// exact holds every vertex as an exact dyadic.
	exact []proof.DyV3
}

func vecArray(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }

// buildDistanceIndex checks the mesh once, with the checks MeshDistanceSquared
// makes on every call: every vertex finite, every triangle index in range and
// every triangle's exact normal nonzero. It returns an index that is not clean
// when any check fails, the mesh has no triangle or it has more triangles
// than int32 addresses. The check does not poll a
// context.
func buildDistanceIndex(verts []r3.Vec, tris [][3]int) *distanceIndex {
	d := &distanceIndex{}
	if len(tris) == 0 || !distanceIndexFits(len(tris)) {
		return d
	}
	for _, v := range verts {
		if !proofbound.FiniteVec(v) {
			return d
		}
	}
	for _, tri := range tris {
		if checkTriangleIndices(tri, len(verts)) != nil {
			return d
		}
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		if !normalNonzeroFloat(a, b, c) && degenerateTriangle(a, b, c) {
			return d
		}
	}
	d.clean = true
	d.exact = make([]proof.DyV3, len(verts))
	for i, v := range verts {
		d.exact[i] = proof.DyVec(v)
	}
	n := len(tris)
	lo := make([][3]float64, n)
	hi := make([][3]float64, n)
	mid := make([][3]float64, n)
	for i, tri := range tris {
		a, b, c := vecArray(verts[tri[0]]), vecArray(verts[tri[1]]), vecArray(verts[tri[2]])
		for k := range 3 {
			lo[i][k] = min(a[k], b[k], c[k])
			hi[i][k] = max(a[k], b[k], c[k])
			mid[i][k] = lo[i][k]/2 + hi[i][k]/2
		}
	}
	d.order = make([]int32, n)
	for i := range d.order {
		d.order[i] = int32(i)
	}
	d.buildNode(lo, hi, mid, 0, n)
	d.boxLo = make([][3]float64, n)
	d.boxHi = make([][3]float64, n)
	for s, ti := range d.order {
		d.boxLo[s], d.boxHi[s] = lo[ti], hi[ti]
	}
	return d
}

// buildNode appends the node over order[start:end], splitting at the median
// box centre of the longest axis, and returns its index. A node's box is the
// union of its triangles' boxes.
func (d *distanceIndex) buildNode(lo, hi, mid [][3]float64, start, end int) int32 {
	node := distanceNode{left: -1, right: -1}
	node.lo, node.hi = lo[d.order[start]], hi[d.order[start]]
	for _, ti := range d.order[start+1 : end] {
		for k := range 3 {
			node.lo[k] = min(node.lo[k], lo[ti][k])
			node.hi[k] = max(node.hi[k], hi[ti][k])
		}
	}
	id := int32(len(d.nodes))
	d.nodes = append(d.nodes, node)
	if end-start <= distanceLeafSize {
		d.nodes[id].leafStart, d.nodes[id].leafCount = int32(start), int32(end-start)
		return id
	}
	axis, extent := 0, -1.0
	for k := range 3 {
		if e := node.hi[k] - node.lo[k]; e > extent {
			axis, extent = k, e
		}
	}
	slices.SortFunc(d.order[start:end], func(x, y int32) int {
		switch cx, cy := mid[x][axis], mid[y][axis]; {
		case cx < cy:
			return -1
		case cx > cy:
			return 1
		}
		return int(x - y)
	})
	half := (start + end) / 2
	left := d.buildNode(lo, hi, mid, start, half)
	right := d.buildNode(lo, hi, mid, half, end)
	d.nodes[id].left, d.nodes[id].right = left, right
	return id
}

// boxLower is boxDistanceSquaredLower over the closed box [lo, hi]: the same
// downward-rounded gaps, squares and partial sums. Rounding is monotone, so a
// node's bound never exceeds the bound of a triangle inside it.
func boxLower(p [3]float64, lo, hi *[3]float64) float64 {
	var result float64
	for k := range 3 {
		var gap float64
		switch {
		case p[k] < lo[k]:
			gap = proof.NextDown(lo[k] - p[k])
		case p[k] > hi[k]:
			gap = proof.NextDown(p[k] - hi[k])
		default:
			continue
		}
		gap = max(0, gap)
		squared := max(0, proof.NextDown(float64(gap*gap)))
		result = max(0, proof.NextDown(result+squared))
	}
	return result
}

// distanceEntry is a heap entry: a node (node >= 0) or a triangle slot
// (node < 0) with its downward-rounded lower bound.
type distanceEntry struct {
	lower float64
	node  int32
	slot  int32
}

// distanceHeap is a binary min-heap on lower.
type distanceHeap []distanceEntry

func (h *distanceHeap) push(e distanceEntry) {
	*h = append(*h, e)
	a := *h
	i := len(a) - 1
	for i > 0 {
		parent := (i - 1) / 2
		if a[parent].lower <= a[i].lower {
			break
		}
		a[parent], a[i] = a[i], a[parent]
		i = parent
	}
}

func (h *distanceHeap) pop() distanceEntry {
	a := *h
	top := a[0]
	last := len(a) - 1
	a[0] = a[last]
	a = a[:last]
	i := 0
	for {
		child := 2*i + 1
		if child >= len(a) {
			break
		}
		if r := child + 1; r < len(a) && a[r].lower < a[child].lower {
			child = r
		}
		if a[i].lower <= a[child].lower {
			break
		}
		a[i], a[child] = a[child], a[i]
		i = child
	}
	*h = a
	return top
}

// distanceSquared returns the exact squared distance from p, finite, to the
// triangles tris of a clean mesh, the value MeshDistanceSquared returns for
// that mesh. It visits triangles in ascending order of their downward-rounded
// box bound and stops at the first entry whose bound is at or above the
// upward-rounded exact minimum. A skipped triangle's exact distance is at
// least its bound, so it cannot lower the minimum. Candidates are compared as
// exact fractions; only a new minimum is normalised to a big.Rat. A cancelled
// context returns its error, checked before the search and every
// distancePollInterval heap pops.
func (d *distanceIndex) distanceSquared(ctx context.Context, p r3.Vec, tris [][3]int) (*big.Rat, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	q := vecArray(p)
	point := proof.DyVec(p)
	var minimum dyFraction
	var minimumRat *big.Rat
	minimumUpper := math.Inf(1)
	heap := make(distanceHeap, 0, 64)
	root := &d.nodes[0]
	heap.push(distanceEntry{lower: boxLower(q, &root.lo, &root.hi), node: 0, slot: -1})
	pops := 0
	for len(heap) > 0 {
		top := heap.pop()
		if minimumRat != nil && top.lower >= minimumUpper {
			break
		}
		pops++
		if pops%distancePollInterval == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if top.node < 0 {
			tri := tris[d.order[top.slot]]
			distance, err := triangleDistanceFraction(point, d.exact[tri[0]], d.exact[tri[1]], d.exact[tri[2]])
			if err != nil {
				return nil, err
			}
			if minimumRat == nil || distance.less(minimum) {
				minimum = distance
				minimumRat = distance.rat()
				minimumUpper = proofbound.RatFloatUp(minimumRat)
			}
			continue
		}
		node := &d.nodes[top.node]
		if node.leafCount > 0 {
			for s := node.leafStart; s < node.leafStart+node.leafCount; s++ {
				if lower := boxLower(q, &d.boxLo[s], &d.boxHi[s]); minimumRat == nil || lower < minimumUpper {
					heap.push(distanceEntry{lower: lower, node: -1, slot: s})
				}
			}
			continue
		}
		for _, child := range [2]int32{node.left, node.right} {
			c := &d.nodes[child]
			if lower := boxLower(q, &c.lo, &c.hi); minimumRat == nil || lower < minimumUpper {
				heap.push(distanceEntry{lower: lower, node: child, slot: -1})
			}
		}
	}
	return minimumRat, nil
}
