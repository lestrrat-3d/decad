package rimextension

import (
	"math/big"
	"slices"
)

type edge [2]int

func undirected(a, b int) edge {
	if a > b {
		return edge{b, a}
	}
	return edge{a, b}
}

func clean(ids []int) []int {
	ids = slices.Compact(ids)
	if len(ids) > 1 && ids[0] == ids[len(ids)-1] {
		ids = ids[:len(ids)-1]
	}
	return ids
}
func (m *rewrite) viable(p []int) bool {
	for j := 2; j < len(p); j++ {
		if !collinear(m.exact(p[0]), m.exact(p[j-1]), m.exact(p[j])) {
			return true
		}
	}
	return false
}
func (m *rewrite) split(p []int, plane int) ([]int, []int) {
	var in, out []int
	for j, a := range p {
		b := p[(j+1)%len(p)]
		u, v := m.distance(a)[plane], m.distance(b)[plane]
		if u.Sign() >= 0 {
			in = append(in, a)
		}
		if u.Sign() <= 0 {
			out = append(out, a)
		}
		if u.Sign()*v.Sign() < 0 {
			k := divide(u, minus(u, v))
			i := m.insert(interpolate(m.exact(a), m.exact(b), k))
			in = append(in, i)
			out = append(out, i)
		}
	}
	return clean(in), clean(out)
}
func (m *rewrite) clipTriangle(t [3]int) ([][]int, bool) {
	p := t[:]
	var pieces [][]int
	for k := range 3 {
		if m.distance(t[0])[k].Sign() <= 0 && m.distance(t[1])[k].Sign() <= 0 && m.distance(t[2])[k].Sign() <= 0 {
			return nil, false
		}
	}
	m.spend(4096, 131072)
	if m.err != nil {
		return nil, false
	}
	for k := range 3 {
		in, out := m.split(p, k)
		if !m.viable(in) {
			return nil, false
		}
		if m.viable(out) {
			pieces = append(pieces, out)
		}
		p = in
	}
	for _, i := range p {
		if !m.within(m.exact(i)) {
			m.err = errChart
			return nil, false
		}
	}
	return pieces, true
}
func sortAlong(p, q point, ids map[int]bool, at func(int) point) []int {
	axis := 0
	for i := 1; i < 3; i++ {
		if new(big.Rat).Abs(minus(q[i], p[i])).Cmp(new(big.Rat).Abs(minus(q[axis], p[axis]))) > 0 {
			axis = i
		}
	}
	dir := q[axis].Cmp(p[axis])
	var out []int
	for id := range ids {
		r := at(id)
		if dir*r[axis].Cmp(p[axis]) > 0 && dir*q[axis].Cmp(r[axis]) > 0 && collinear(p, q, r) {
			out = append(out, id)
		}
	}
	slices.SortFunc(out, func(a, b int) int { return dir * at(a)[axis].Cmp(at(b)[axis]) })
	return out
}
