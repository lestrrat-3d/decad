package rimextension

import (
	"cmp"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

func edgeOrder(a, b edge) int {
	if n := cmp.Compare(a[0], b[0]); n != 0 {
		return n
	}
	return cmp.Compare(a[1], b[1])
}
func (m *rewrite) boundary() (map[edge]int, error) {
	n := uint64(len(m.out.Triangles))
	if err := m.budget.ReserveBundle(m.ctx, 4*n, 3*n, 96*n); err != nil {
		return nil, err
	}
	edges := map[edge]int{}
	for i, t := range m.out.Triangles {
		if i%256 == 0 {
			if err := m.ctx.Err(); err != nil {
				return nil, err
			}
		}
		for j, a := range t {
			b := t[(j+1)%3]
			e, r := edge{a, b}, edge{b, a}
			if _, ok := edges[r]; ok {
				delete(edges, r)
			} else if _, ok := edges[e]; ok {
				return nil, errChart
			} else {
				edges[e] = i
			}
		}
	}
	return edges, nil
}
func sortedEdges(es map[edge]int) []edge {
	out := make([]edge, 0, len(es))
	for e := range es {
		out = append(out, e)
	}
	slices.SortFunc(out, edgeOrder)
	return out
}
func (m *rewrite) extendWall() error {
	es, err := m.boundary()
	if err != nil {
		return err
	}
	for _, e := range sortedEdges(es) {
		if m.out.Sources[es[e]] != m.groups.Boss {
			continue
		}
		var top [2]int
		for i, id := range e {
			p := m.exact(id)
			if p[1].Cmp(m.chart.start[1]) != 0 || m.distance(id)[2].Sign() < 0 {
				return errChart
			}
			p[1] = m.chart.height(p)
			top[i] = m.insert(p)
		}
		p := clean([]int{e[1], e[0], top[0], top[1]})
		if m.viable(p) {
			m.convex(p, m.groups.Extension)
		}
		if m.err != nil {
			return m.err
		}
	}
	return nil
}
func (m *rewrite) fill(poly []int, group, u, v int) error {
	if len(poly) < 3 || len(poly) > 512 {
		return errChart
	}
	meter := m.budget.WorkBudget(m.ctx)
	pts := make([]meshbool.Xp2, len(poly))
	ids := make([]int, len(poly))
	for i, id := range poly {
		p := m.exact(id)
		pts[i] = meshbool.NewXP2(p[u], p[v])
		ids[i] = i
	}
	sign, err := meshbool.PolyArea2Sign(meter, pts)
	if err != nil {
		return err
	}
	if sign == 0 {
		return errChart
	}
	if sign < 0 {
		slices.Reverse(ids)
	}
	tris, err := meshbool.EarClipX(meter, pts, ids)
	if err != nil {
		return err
	}
	for _, t := range tris {
		a, b, c := poly[t[0]], poly[t[1]], poly[t[2]]
		if sign < 0 {
			b, c = c, b
		}
		m.emit(a, b, c, group)
	}
	return m.err
}
func (m *rewrite) fillPlane(k, group, u, v int) error {
	es, err := m.boundary()
	if err != nil {
		return err
	}
	next := map[int]int{}
	pred := map[int]bool{}
	for _, e := range sortedEdges(es) {
		a, b := e[1], e[0]
		if m.distance(a)[k].Sign() != 0 || m.distance(b)[k].Sign() != 0 {
			continue
		}
		if _, ok := next[a]; ok || pred[b] {
			return errChart
		}
		next[a] = b
		pred[b] = true
	}
	if len(next) < 2 || len(next) > 512 {
		return errChart
	}
	if k == 2 {
		// The existing stop seam closes the start. At the tangent station the
		// two ends meet the end wall along one common line.
		start, end := -1, -1
		for a := range next {
			if !pred[a] {
				if start >= 0 || m.distance(a)[1].Sign() != 0 {
					return errChart
				}
				start = a
			}
		}
		for b := range pred {
			if _, ok := next[b]; !ok {
				if end >= 0 || m.distance(b)[1].Sign() != 0 {
					return errChart
				}
				end = b
			}
		}
		if start >= 0 && end >= 0 {
			next[end] = start
		} else if start != end {
			return errChart
		}
	}
	first := -1
	for a := range next {
		if first < 0 || a < first {
			first = a
		}
	}
	p := []int{first}
	a := first
	for {
		b, ok := next[a]
		if !ok {
			return errChart
		}
		delete(next, a)
		if b == first {
			break
		}
		p = append(p, b)
		a = b
	}
	if len(next) != 0 {
		return errChart
	}
	return m.fill(p, group, u, v)
}
func (m *rewrite) finish() error {
	if err := m.extendWall(); err != nil {
		return err
	}
	if err := m.fillPlane(2, m.groups.Bevel, 0, 1); err != nil {
		return err
	}
	if err := m.fillPlane(1, m.groups.Extension, 1, 2); err != nil {
		return err
	}
	es, err := m.boundary()
	if err != nil {
		return err
	}
	if len(es) != 0 {
		return errChart
	}
	return nil
}
