package meshreduce

import (
	"container/heap"
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/r3"
)

// ErrNotClosed reports a mesh that is not a closed, consistently wound surface in which every edge borders two
// triangles, the only kind Collapse keeps closed.
var ErrNotClosed = errors.New("the mesh is not a closed, consistently wound manifold")

// CollapseResult is a mesh thinned by edge collapse.
type CollapseResult struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	// Collapses counts the edges merged; each removes two triangles and one vertex.
	Collapses int
	// MaxMove is the greatest distance from an input vertex to the vertex it ended at, in millimetres.
	MaxMove float64
}

// collapseMinCosine is the least cosine between a triangle's normal before and after an edge collapse moves one
// of its corners; a collapse that turns a neighbouring triangle more than that is refused.
const collapseMinCosine = 0.2

// Collapse merges the shortest edges of a closed manifold mesh, one after another, each at its midpoint, until the
// mesh has at most target triangles or no edge can be merged. An edge is merged only when its two ends share exactly
// the two vertices opposite it (so the surface stays a closed manifold, with no pinched vertex or doubled triangle),
// the mesh keeps at least four triangles, and no triangle around the edge turns more than the cosine
// collapseMinCosine allows. The result is wound as the input is. It says nothing of self-intersection.
func Collapse(ctx context.Context, vertices []r3.Vec, triangles [][3]int, target int) (CollapseResult, error) {
	if err := requireClosed(triangles, len(vertices)); err != nil {
		return CollapseResult{}, err
	}
	c := newCollapser(vertices, triangles)
	if err := c.run(ctx, target); err != nil {
		return CollapseResult{}, err
	}
	return c.result(vertices), nil
}

// requireClosed reports ErrNotClosed unless every directed edge occurs once and its reverse occurs once.
func requireClosed(triangles [][3]int, n int) error {
	directed := map[[2]int]int{}
	for _, t := range triangles {
		for i := range t {
			a, b := t[i], t[(i+1)%3]
			if a < 0 || b < 0 || a >= n || b >= n || a == b {
				return ErrNotClosed
			}
			directed[[2]int{a, b}]++
		}
	}
	for e, count := range directed {
		if count != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return ErrNotClosed
		}
	}
	return nil
}

type collapser struct {
	pos    []r3.Vec
	tris   [][3]int
	alive  []bool
	vtris  []map[int]struct{}
	into   []int
	stamp  []int
	live   int
	merged int
	queue  edgeQueue
}

func newCollapser(vertices []r3.Vec, triangles [][3]int) *collapser {
	c := &collapser{
		pos:   append([]r3.Vec(nil), vertices...),
		tris:  append([][3]int(nil), triangles...),
		alive: make([]bool, len(triangles)),
		vtris: make([]map[int]struct{}, len(vertices)),
		into:  make([]int, len(vertices)),
		stamp: make([]int, len(vertices)),
		live:  len(triangles),
	}
	for i := range c.vtris {
		c.vtris[i] = map[int]struct{}{}
		c.into[i] = i
	}
	seen := map[[2]int]struct{}{}
	for ti, t := range triangles {
		c.alive[ti] = true
		for i := range t {
			c.vtris[t[i]][ti] = struct{}{}
			a, b := t[i], t[(i+1)%3]
			if a > b {
				a, b = b, a
			}
			if _, dup := seen[[2]int{a, b}]; !dup {
				seen[[2]int{a, b}] = struct{}{}
				c.queue.items = append(c.queue.items, c.edge(a, b))
			}
		}
	}
	heap.Init(&c.queue)
	return c
}

func (c *collapser) edge(a, b int) edgeItem {
	return edgeItem{a: a, b: b, length: c.pos[a].Sub(c.pos[b]).Len(), stampA: c.stamp[a], stampB: c.stamp[b]}
}

func (c *collapser) run(ctx context.Context, target int) error {
	for pops := 0; c.live > target && c.live-2 >= 4 && c.queue.Len() > 0; pops++ {
		if pops%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		e, ok := heap.Pop(&c.queue).(edgeItem)
		if !ok {
			break
		}
		if c.stamp[e.a] != e.stampA || c.stamp[e.b] != e.stampB || c.stamp[e.a] < 0 || c.stamp[e.b] < 0 {
			continue
		}
		if c.tryCollapse(e.a, e.b) {
			c.live -= 2
			c.merged++
			for t := range c.vtris[e.a] {
				for _, w := range c.tris[t] {
					if w != e.a {
						heap.Push(&c.queue, c.edge(e.a, w))
					}
				}
			}
		}
	}
	return nil
}

// tryCollapse merges b into a at their midpoint when the link condition holds and no neighbouring triangle turns
// too far, and reports whether it did.
func (c *collapser) tryCollapse(a, b int) bool {
	var shared []int
	for t := range c.vtris[a] {
		if _, ok := c.vtris[b][t]; ok {
			shared = append(shared, t)
		}
	}
	if len(shared) != 2 {
		return false
	}
	opposite := map[int]struct{}{}
	for _, t := range shared {
		for _, w := range c.tris[t] {
			if w != a && w != b {
				opposite[w] = struct{}{}
			}
		}
	}
	nearA := c.neighbours(a)
	common := 0
	for w := range c.neighbours(b) {
		if _, ok := nearA[w]; ok {
			if _, ok := opposite[w]; !ok {
				return false
			}
			common++
		}
	}
	if common != 2 {
		return false
	}
	mid := c.pos[a].Add(c.pos[b]).Scale(0.5)
	for _, v := range [2]int{a, b} {
		for t := range c.vtris[v] {
			if t == shared[0] || t == shared[1] {
				continue
			}
			if !c.keepsShape(t, v, mid) {
				return false
			}
		}
	}
	c.pos[a] = mid
	c.stamp[a]++
	c.stamp[b] = -1
	c.into[b] = a
	for _, t := range shared {
		c.alive[t] = false
		for _, w := range c.tris[t] {
			delete(c.vtris[w], t)
		}
	}
	for t := range c.vtris[b] {
		for i, w := range c.tris[t] {
			if w == b {
				c.tris[t][i] = a
			}
		}
		c.vtris[a][t] = struct{}{}
	}
	c.vtris[b] = nil
	return true
}

func (c *collapser) neighbours(v int) map[int]struct{} {
	out := map[int]struct{}{}
	for t := range c.vtris[v] {
		for _, w := range c.tris[t] {
			if w != v {
				out[w] = struct{}{}
			}
		}
	}
	return out
}

// keepsShape reports whether triangle t, with its corner v moved to p, still faces within collapseMinCosine of
// where it faced.
func (c *collapser) keepsShape(t, v int, p r3.Vec) bool {
	corners := c.tris[t]
	var before, after [3]r3.Vec
	for i, w := range corners {
		before[i], after[i] = c.pos[w], c.pos[w]
		if w == v {
			after[i] = p
		}
	}
	n0 := before[1].Sub(before[0]).Cross(before[2].Sub(before[0]))
	n1 := after[1].Sub(after[0]).Cross(after[2].Sub(after[0]))
	l0, l1 := n0.Len(), n1.Len()
	return l1 > 0 && l0 > 0 && n0.Dot(n1) > collapseMinCosine*l0*l1
}

// result compacts the surviving vertices and triangles and measures how far the input vertices moved.
func (c *collapser) result(input []r3.Vec) CollapseResult {
	index := make([]int, len(c.pos))
	for i := range index {
		index[i] = -1
	}
	var out CollapseResult
	for ti, t := range c.tris {
		if !c.alive[ti] {
			continue
		}
		var m [3]int
		for i, w := range t {
			if index[w] < 0 {
				index[w] = len(out.Vertices)
				out.Vertices = append(out.Vertices, c.pos[w])
			}
			m[i] = index[w]
		}
		out.Triangles = append(out.Triangles, m)
	}
	out.Collapses = c.merged
	for i, v := range input {
		root := i
		for c.into[root] != root {
			root = c.into[root]
		}
		out.MaxMove = math.Max(out.MaxMove, v.Sub(c.pos[root]).Len())
	}
	return out
}

type edgeItem struct {
	a, b           int
	length         float64
	stampA, stampB int
}

type edgeQueue struct{ items []edgeItem }

func (q *edgeQueue) Len() int { return len(q.items) }
func (q *edgeQueue) Less(i, j int) bool {
	x, y := q.items[i], q.items[j]
	switch {
	case x.length != y.length:
		return x.length < y.length
	case x.a != y.a:
		return x.a < y.a
	default:
		return x.b < y.b
	}
}
func (q *edgeQueue) Swap(i, j int) { q.items[i], q.items[j] = q.items[j], q.items[i] }
func (q *edgeQueue) Push(x any) {
	if item, ok := x.(edgeItem); ok {
		q.items = append(q.items, item)
	}
}
func (q *edgeQueue) Pop() any {
	last := q.items[len(q.items)-1]
	q.items = q.items[:len(q.items)-1]
	return last
}
