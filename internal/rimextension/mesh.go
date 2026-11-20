package rimextension

import (
	"context"
	"errors"

	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Mesh contains a held shell and one source group per facet.
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	Sources   []int
}

var errChart = errors.New("the retained bevel has no simple supported chart")

type rewrite struct {
	ctx         context.Context //nolint:containedctx // The rewrite is local to one synchronous Apply call.
	budget      *proofbound.ReplayBudget
	chart       chart
	support     SourceWindow
	groups      Groups
	old         Mesh
	out         Mesh
	exactPoints map[int]point
	distances   map[int][3]*big.Rat
	lookup      map[[3]string]int
	original    map[r3.Vec]int
	pieces      map[int][][]int
	edgePoints  map[edge]map[int]bool
	err         error
}

func (m *rewrite) exact(i int) point {
	if p, ok := m.exactPoints[i]; ok {
		return p
	}
	p := lift(m.out.Vertices[i])
	m.exactPoints[i] = p
	return p
}
func (m *rewrite) distance(i int) [3]*big.Rat {
	if v, ok := m.distances[i]; ok {
		return v
	}
	m.spend(96, 8192)
	v := m.chart.distances(m.exact(i))
	m.distances[i] = v
	return v
}
func (m *rewrite) spend(work, bytes uint64) {
	if m.err == nil {
		m.err = m.budget.ReserveBundle(m.ctx, work, 1, bytes)
	}
}
func (m *rewrite) insert(p point) int {
	if m.err != nil {
		return 0
	}
	for _, coord := range p {
		if coord.Num().BitLen() > 4096 || coord.Denom().BitLen() > 4096 {
			m.err = errChart
			return 0
		}
	}
	key := p.key()
	if i, ok := m.lookup[key]; ok {
		return i
	}
	v := p.vec()
	if i, ok := m.original[v]; ok {
		orig := m.exact(i)
		if orig[0].Cmp(p[0]) == 0 && orig[1].Cmp(p[1]) == 0 && orig[2].Cmp(p[2]) == 0 {
			return i
		}
	}
	m.spend(24, 6144)
	id := len(m.out.Vertices)
	m.lookup[key] = id
	m.out.Vertices = append(m.out.Vertices, v)
	m.exactPoints[id] = p
	return id
}
func (m *rewrite) emit(a, b, c, group int) {
	if m.err != nil {
		return
	}
	if a == b || b == c || a == c {
		m.err = errChart
		return
	}
	if group == m.groups.Bevel || group == m.groups.Extension {
		for _, i := range [...]int{a, b, c} {
			if !m.within(m.exact(i)) {
				m.err = errChart
				return
			}
		}
	}
	m.out.Triangles = append(m.out.Triangles, [3]int{a, b, c})
	m.out.Sources = append(m.out.Sources, group)
}
func (m *rewrite) convex(p []int, group int) {
	if len(p) < 3 || len(p) > 32 {
		if m.err == nil {
			m.err = errChart
		}
		return
	}
	if len(p) == 3 {
		m.emit(p[0], p[1], p[2], group)
		return
	}
	sum := point{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, i := range p {
		v := m.exact(i)
		for k := range sum {
			sum[k].Add(sum[k], v[k])
		}
	}
	count := big.NewRat(int64(len(p)), 1)
	for _, n := range sum {
		n.Quo(n, count)
	}
	center := m.insert(sum)
	for i, a := range p {
		m.emit(a, p[(i+1)%len(p)], center, group)
	}
}
func (m *rewrite) recordCuts() error {
	for i, t := range m.old.Triangles {
		group := m.old.Sources[i]
		if group != m.groups.Cap && group != m.groups.Side && !m.groups.Stop[group] {
			continue
		}
		if err := m.budget.SpendWork(m.ctx, 12); err != nil {
			return err
		}
		if m.groups.Stop[group] {
			continue
		}
		parts, ok := m.clipTriangle(t)
		if m.err != nil {
			return m.err
		}
		if !ok {
			continue
		}
		m.pieces[i] = parts
		for _, p := range parts {
			for _, v := range p {
				for j, a := range t {
					b := t[(j+1)%3]
					if v == a || v == b || !collinear(m.exact(a), m.exact(b), m.exact(v)) {
						continue
					}
					e := undirected(a, b)
					if m.edgePoints[e] == nil {
						m.edgePoints[e] = map[int]bool{}
					}
					m.edgePoints[e][v] = true
				}
			}
		}
	}
	return nil
}
func (m *rewrite) conform(i int, t [3]int, p []int) []int {
	candidates := map[int]bool{}
	for _, piece := range m.pieces[i] {
		for _, id := range piece {
			candidates[id] = true
		}
	}
	for j, a := range t {
		for id := range m.edgePoints[undirected(a, t[(j+1)%3])] {
			candidates[id] = true
		}
	}
	if len(candidates) == 0 {
		return p
	}
	var out []int
	for j, a := range p {
		out = append(out, a)
		out = append(out, sortAlong(m.exact(a), m.exact(p[(j+1)%len(p)]), candidates, m.exact)...)
	}
	return out
}

// Apply constructs the held surface after its caller authenticates the
// source window, affected groups, and existing stop facets. It retains every
// old vertex index for the exact comparison with the previously audited mesh.
func Apply(ctx context.Context, old Mesh, section Section, groups Groups, stopFacets map[int]bool, support SourceWindow, budget *proofbound.ReplayBudget) (Mesh, bool, error) {
	var none Mesh
	chart, ok := newChart(section)
	if !ok || budget == nil || len(old.Vertices) > 16384 || len(old.Triangles) > 32768 || len(old.Sources) != len(old.Triangles) || len(stopFacets) == 0 {
		return none, false, nil
	}
	n := uint64(len(old.Triangles))
	v := uint64(len(old.Vertices))
	if err := budget.ReserveBundle(ctx, 12*n+2*v, 6*n+v, 800*n+400*v); err != nil {
		return none, false, err
	}
	m := rewrite{ctx: ctx, budget: budget, chart: chart, support: support, groups: groups, old: old,
		out:      Mesh{Vertices: slices.Clone(old.Vertices)},
		original: map[r3.Vec]int{}, lookup: map[[3]string]int{},
		exactPoints: map[int]point{}, distances: map[int][3]*big.Rat{},
		pieces: map[int][][]int{}, edgePoints: map[edge]map[int]bool{},
	}
	for i, p := range old.Vertices {
		if i%256 == 0 && ctx.Err() != nil {
			return none, false, ctx.Err()
		}
		if !chartCoordinate(p.X) || !chartCoordinate(p.Y) || !chartCoordinate(p.Z) {
			return none, false, nil
		}
		if _, duplicate := m.original[p]; duplicate {
			return none, false, nil
		}
		m.original[p] = i
	}
	for i, t := range old.Triangles {
		for _, j := range t {
			if j < 0 || j >= len(old.Vertices) {
				return none, false, nil
			}
		}
		if stopFacets[i] {
			if !groups.Stop[old.Sources[i]] {
				return none, false, nil
			}
			for _, j := range t {
				if !m.within(m.exact(j)) {
					return none, false, nil
				}
			}
			m.pieces[i] = nil
		}
	}
	if err := m.recordCuts(); err != nil {
		return none, false, errUnlessChart(err)
	}
	for i, t := range old.Triangles {
		parts, changed := m.pieces[i]
		if !changed {
			parts = [][]int{t[:]}
		}
		for _, p := range parts {
			m.convex(m.conform(i, t, p), old.Sources[i])
		}
		if m.err != nil {
			return none, false, errUnlessChart(m.err)
		}
	}
	if err := m.finish(); err != nil {
		return none, false, errUnlessChart(err)
	}
	return m.out, true, nil
}
func errUnlessChart(err error) error {
	if errors.Is(err, errChart) {
		return nil
	}
	return err
}

// within checks the exact pre-rounding coordinates against the full native
// source window. The chart map is affine, so testing polygon vertices bounds
// the entire polygon, including each newly exposed cylinder or bevel facet.
func (m *rewrite) within(p point) bool {
	side := plus(m.chart.start[2], times(m.chart.slope, minus(p[0], m.chart.start[0])))
	w := m.support
	for _, v := range []struct {
		q      *big.Rat
		lo, hi float64
	}{{p[0], w.XMin, w.XMax}, {p[1], w.YMin, w.YMax}, {minus(p[2], side), w.OffsetMin, w.OffsetMax}} {
		if v.q.Cmp(scalar(v.lo)) < 0 || v.q.Cmp(scalar(v.hi)) > 0 {
			return false
		}
	}
	return true
}
