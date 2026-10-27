package meshreduce

import (
	"context"
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// remeshRounds is how many passes of split, collapse, flip and smooth Remesh makes.
const remeshRounds = 8

// remeshSmooth is the share of the way to its neighbours' centroid, along the surface, a vertex moves in a pass.
const remeshSmooth = 0.5

// Remesh makes the triangles of a closed manifold mesh near-equilateral with edges near edge, by the passes of
// Botsch and Kobbelt's incremental isotropic remeshing: split the edges over 4/3 of edge, collapse the edges under 4/5
// of it, flip edges to bring every vertex's valence toward six, and slide each vertex toward its neighbours' centroid
// along the surface. Every step keeps the surface closed and wound as it was. The vertices do not return to the
// input surface, so a convex surface shrinks a little in each pass, which the smoothing step's share keeps small.
func Remesh(ctx context.Context, vertices []r3.Vec, triangles [][3]int, edge float64) (CollapseResult, error) {
	if err := requireClosed(triangles, len(vertices)); err != nil {
		return CollapseResult{}, err
	}
	c := newCollapser(vertices, triangles)
	for range remeshRounds {
		if err := ctx.Err(); err != nil {
			return CollapseResult{}, err
		}
		c.splitLong(4 * edge / 3)
		c.collapseShort(4*edge/5, 4*edge/3)
		c.flipForValence()
		c.smooth()
	}
	return c.result(vertices), nil
}

// edges lists the live edges, each once as (low, high), in order.
func (c *collapser) edges() [][2]int {
	seen := map[[2]int]struct{}{}
	for ti, t := range c.tris {
		if !c.alive[ti] {
			continue
		}
		for i := range t {
			a, b := t[i], t[(i+1)%3]
			if a > b {
				a, b = b, a
			}
			seen[[2]int{a, b}] = struct{}{}
		}
	}
	out := make([][2]int, 0, len(seen))
	for e := range seen {
		out = append(out, e)
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i][0] != out[j][0] {
			return out[i][0] < out[j][0]
		}
		return out[i][1] < out[j][1]
	})
	return out
}

// shared returns the two triangles around the edge ab, and false when there are not exactly two.
func (c *collapser) shared(a, b int) ([2]int, bool) {
	var out [2]int
	n := 0
	for t := range c.vtris[a] {
		if _, ok := c.vtris[b][t]; ok {
			if n == 2 {
				return out, false
			}
			out[n] = t
			n++
		}
	}
	return out, n == 2
}

func (c *collapser) splitLong(limit float64) {
	for _, e := range c.edges() {
		a, b := e[0], e[1]
		if c.stamp[a] < 0 || c.stamp[b] < 0 || c.pos[a].Sub(c.pos[b]).Len() <= limit {
			continue
		}
		if tris, ok := c.shared(a, b); ok {
			c.split(a, b, tris)
		}
	}
}

// split cuts the edge ab at its midpoint m; each of the two triangles around it becomes two, the first keeping its
// id with b replaced by m and the second new with a replaced by m.
func (c *collapser) split(a, b int, tris [2]int) {
	m := len(c.pos)
	c.pos = append(c.pos, c.pos[a].Add(c.pos[b]).Scale(0.5))
	c.vtris = append(c.vtris, map[int]struct{}{})
	c.into = append(c.into, m)
	c.stamp = append(c.stamp, 0)
	for _, t := range tris {
		kept, added := c.tris[t], c.tris[t]
		for i := range kept {
			if kept[i] == b {
				kept[i] = m
			}
			if added[i] == a {
				added[i] = m
			}
		}
		c.tris[t] = kept
		delete(c.vtris[b], t)
		c.vtris[m][t] = struct{}{}
		id := len(c.tris)
		c.tris = append(c.tris, added)
		c.alive = append(c.alive, true)
		for _, w := range added {
			c.vtris[w][id] = struct{}{}
		}
		c.live++
	}
}

func (c *collapser) collapseShort(floor, ceil float64) {
	for _, e := range c.edges() {
		a, b := e[0], e[1]
		if c.stamp[a] < 0 || c.stamp[b] < 0 || c.pos[a].Sub(c.pos[b]).Len() >= floor {
			continue
		}
		mid := c.pos[a].Add(c.pos[b]).Scale(0.5)
		if c.reaches(a, b, mid, ceil) || c.reaches(b, a, mid, ceil) {
			continue
		}
		if c.live-2 >= 4 && c.tryCollapse(a, b) {
			c.live -= 2
			c.merged++
		}
	}
}

// reaches reports whether placing v's partner at p would leave an edge from p to a neighbour of v longer than limit.
func (c *collapser) reaches(v, other int, p r3.Vec, limit float64) bool {
	for w := range c.neighbours(v) {
		if w != other && c.pos[w].Sub(p).Len() > limit {
			return true
		}
	}
	return false
}

func (c *collapser) flipForValence() {
	for _, e := range c.edges() {
		a, b := e[0], e[1]
		if c.stamp[a] < 0 || c.stamp[b] < 0 {
			continue
		}
		tris, ok := c.shared(a, b)
		if !ok {
			continue
		}
		c.flip(a, b, tris)
	}
}

// flip turns the edge ab to the edge between the vertices opposite it when that lowers the valences' distance from
// six, the vertices opposite are not already joined, and neither new triangle turns away from the old surface.
func (c *collapser) flip(a, b int, tris [2]int) {
	t1, t2 := tris[0], tris[1]
	o1, ok1 := c.opposite(t1, a, b)
	o2, ok2 := c.opposite(t2, a, b)
	if !ok1 || !ok2 || o1 == o2 {
		return
	}
	// Orient so t1 runs a to b and t2 b to a.
	if !c.runs(t1, a, b) {
		t1, t2 = t2, t1
		o1, o2 = o2, o1
	}
	if !c.runs(t1, a, b) || !c.runs(t2, b, a) {
		return
	}
	if _, joined := c.neighbours(o1)[o2]; joined {
		return
	}
	dev := func(v, delta int) int {
		d := len(c.vtris[v]) + delta - 6
		return d * d
	}
	before := dev(a, 0) + dev(b, 0) + dev(o1, 0) + dev(o2, 0)
	after := dev(a, -1) + dev(b, -1) + dev(o1, 1) + dev(o2, 1)
	if after >= before {
		return
	}
	n1, n2 := [3]int{a, o2, o1}, [3]int{o2, b, o1}
	if !c.turnsLittle(t1, n1) || !c.turnsLittle(t2, n2) {
		return
	}
	c.tris[t1], c.tris[t2] = n1, n2
	delete(c.vtris[a], t2)
	delete(c.vtris[b], t1)
	c.vtris[o1][t2] = struct{}{}
	c.vtris[o2][t1] = struct{}{}
}

// opposite returns the corner of triangle t that is neither a nor b.
func (c *collapser) opposite(t, a, b int) (int, bool) {
	for _, w := range c.tris[t] {
		if w != a && w != b {
			return w, true
		}
	}
	return 0, false
}

// runs reports whether triangle t has the directed edge a to b.
func (c *collapser) runs(t, a, b int) bool {
	tr := c.tris[t]
	for i := range tr {
		if tr[i] == a && tr[(i+1)%3] == b {
			return true
		}
	}
	return false
}

// turnsLittle reports whether the corners next, in place of triangle t's, face within collapseMinCosine of where t
// faced.
func (c *collapser) turnsLittle(t int, next [3]int) bool {
	old := c.tris[t]
	n0 := c.pos[old[1]].Sub(c.pos[old[0]]).Cross(c.pos[old[2]].Sub(c.pos[old[0]]))
	n1 := c.pos[next[1]].Sub(c.pos[next[0]]).Cross(c.pos[next[2]].Sub(c.pos[next[0]]))
	l0, l1 := n0.Len(), n1.Len()
	return l0 > 0 && l1 > 0 && n0.Dot(n1) > collapseMinCosine*l0*l1
}

// smooth moves each vertex part of the way to its neighbours' centroid, along the plane of its area-weighted normal,
// unless a triangle around it would turn too far.
func (c *collapser) smooth() {
	for v := range c.pos {
		if c.stamp[v] < 0 || len(c.vtris[v]) == 0 {
			continue
		}
		var normal, centre r3.Vec
		near := c.neighbours(v)
		ids := make([]int, 0, len(near))
		for w := range near {
			ids = append(ids, w)
		}
		sort.Ints(ids)
		for _, w := range ids {
			centre = centre.Add(c.pos[w])
		}
		centre = centre.Scale(1 / float64(len(ids)))
		for t := range c.vtris[v] {
			tr := c.tris[t]
			normal = normal.Add(c.pos[tr[1]].Sub(c.pos[tr[0]]).Cross(c.pos[tr[2]].Sub(c.pos[tr[0]])))
		}
		if !(normal.Len() > 0) {
			continue
		}
		normal = normal.Scale(1 / normal.Len())
		move := centre.Sub(c.pos[v])
		move = move.Sub(normal.Scale(move.Dot(normal))).Scale(remeshSmooth)
		target := c.pos[v].Add(move)
		fine := true
		for t := range c.vtris[v] {
			fine = fine && c.keepsShape(t, v, target)
		}
		if fine && !math.IsNaN(target.X+target.Y+target.Z) {
			c.pos[v] = target
		}
	}
}
