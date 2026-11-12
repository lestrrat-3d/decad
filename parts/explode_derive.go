package parts

import (
	"context"
	"errors"
	"fmt"
	"math"
	"runtime"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/explodeblock"
	"github.com/lestrrat-3d/decad/internal/explodeorder"
	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Removal constrains how WithDerivedExplode removes one part.
type Removal struct {
	// Direction, when not zero, is the only direction the part may leave by. Its
	// length does not matter.
	Direction r3.Vec
	// After names parts that must leave in an earlier round than this part.
	After []string
	// Before names parts this part must leave before.
	Before []string
	// Group puts every part with the same non-empty Group into one rigid group
	// that leaves together along one direction.
	Group string
}

// WithDerivedExplode derives the removal order and direction of every part that
// no WithExplodeStages entry names from the parts' tessellations, instead of
// walking the joints. Explicit stages run first, unchanged. A part leaves along
// a joint axis, a stated pull or a world axis once no part still present stands
// in its way; parts that block each other leave together as one rigid group. A
// pair the tessellation cannot decide counts as free, so the result is a layout
// and not a proof (agent-docs/parts-design.md §5.8).
func WithDerivedExplode() ExplodeOption {
	return option{func(c *config) error { c.derived = true; return nil }}
}

// WithRemoval constrains the derived removal of one part and needs
// WithDerivedExplode. A part no Set has, a part without a body, the ground, or a
// part an explicit stage names is decad.ErrDegenerate at the query, as is a
// constraint the search cannot meet. A later WithRemoval for the same part
// replaces the earlier one.
func WithRemoval(part string, r Removal) ExplodeOption {
	if !finiteVec(r.Direction) {
		return failing(fmt.Errorf("parts: removal of %q direction: %w", part, decad.ErrNotFinite))
	}
	r.After, r.Before = slices.Clone(r.After), slices.Clone(r.Before)
	return option{func(c *config) error {
		if c.removals == nil {
			c.removals = map[string]Removal{}
		}
		c.removals[part] = r
		return nil
	}}
}

type derivedInput struct {
	ground    int
	idx       []int
	plan      map[int]int
	boxes     []explodeplan.Box
	stages    resolvedStages
	walk      []int
	nodes     []explodeplan.Node
	faces     *faceCache
	jointAxes []r3.Vec
}

type derivedStages struct {
	stages []explodeplan.Stage
	names  []string
	order  []int
	nodes  []explodeplan.Node
}

// deriveStages appends the removal rounds to the explicit stages.
func (s *Set) deriveStages(ctx context.Context, c config, in derivedInput) (derivedStages, error) {
	var out derivedStages
	n := len(in.idx)
	name := func(k int) string { return s.parts[in.idx[k]].Name }
	named := make([]bool, n)
	for _, st := range in.stages.stages {
		for _, m := range st.Members {
			named[m] = true
		}
	}
	gone := slices.Clone(named)
	gone[in.ground] = false
	cons, err := s.removalConstraints(c, in, named)
	if err != nil {
		return out, err
	}
	// The items of the table: the ground and every part that can leave.
	var items []int
	for k := range n {
		if !gone[k] {
			items = append(items, k)
		}
	}
	dirs := explodeblock.Directions(append(slices.Clone(cons.stated), in.jointAxes...), math.Sin(c.axisTol))
	for k, f := range cons.fixedVec {
		cons.fixed[k] = nearest(dirs, f)
	}
	sc := &layoutScorer{dirs: dirs, boxes: in.boxes, zero: make([]r3.Vec, n), joint: make([]bool, len(dirs)),
		projs: make([][]*explodeblock.Projection, len(dirs))}
	tol := math.Sin(c.axisTol)
	for i, d := range dirs {
		for _, a := range in.jointAxes {
			u, ok := a.Normalize()
			sc.joint[i] = sc.joint[i] || (ok && (near(u, d, tol) || near(u.Scale(-1), d, tol)))
		}
	}
	sc.groundCentre = in.boxes[in.ground].Centre()
	meshes := map[*decad.Body]explodeblock.Mesh{}
	for _, k := range items {
		b := s.parts[in.idx[k]].Body
		if _, ok := meshes[b]; ok {
			continue
		}
		bf, err := in.faces.get(ctx, b)
		if err != nil {
			return out, err
		}
		meshes[b] = meshOfFaces(bf)
	}
	workers := runtime.GOMAXPROCS(0)
	blockers := make([][][]int, len(dirs))
	for di, d := range dirs {
		if err := ctx.Err(); err != nil {
			return out, err
		}
		byBody := map[*decad.Body]*explodeblock.Projection{}
		sc.projs[di] = make([]*explodeblock.Projection, n)
		var insts []explodeblock.Instance
		for _, k := range items {
			b := s.parts[in.idx[k]].Body
			p, ok := byBody[b]
			if !ok {
				var err error
				if p, err = explodeblock.Project(meshes[b], d); err != nil {
					return out, fmt.Errorf("parts: part %q: %w", name(k), err)
				}
				byBody[b] = p
			}
			sc.projs[di][k] = p
			insts = append(insts, explodeblock.Instance{P: p})
		}
		tab := explodeblock.BuildTable(insts, workers)
		blockers[di] = make([][]int, n)
		for a, k := range items {
			if k == in.ground {
				continue
			}
			for _, b := range tab.Blockers(a) {
				blockers[di][k] = append(blockers[di][k], items[b])
			}
		}
	}
	rounds, err := explodeorder.Search(explodeorder.Input{N: n, Ground: in.ground, Gone: gone, Blockers: blockers,
		Group: cons.group, After: cons.after, Fixed: cons.fixed, Score: sc})
	if err != nil {
		return out, s.orderError(err, dirs, in)
	}
	out.stages, out.names = slices.Clone(in.stages.stages), slices.Clone(in.stages.names)
	for _, p := range in.walk {
		if named[p] {
			out.order = append(out.order, p)
		}
	}
	out.nodes = slices.Clone(in.nodes)
	for r, round := range rounds {
		var cur *explodeplan.Stage
		stageNo := 0
		emit := func(st explodeplan.Stage) {
			label := fmt.Sprintf("removal %d", r+1)
			if stageNo > 0 {
				label = fmt.Sprintf("removal %d.%d", r+1, stageNo+1)
			}
			stageNo++
			out.names = append(out.names, label)
			out.stages = append(out.stages, st)
		}
		flush := func() {
			if cur != nil {
				emit(*cur)
				cur = nil
			}
		}
		for _, u := range round {
			for _, m := range u.Members {
				out.order = append(out.order, m)
				out.nodes[m].Parent = -1
			}
			st := explodeplan.Stage{Members: u.Members, Gap: math.NaN(), Clear: explodeplan.ClearAll, Dir: dirs[u.Dir],
				Together: len(u.Members) > 1}
			if st.Together {
				flush()
				emit(st)
				continue
			}
			if cur != nil && cur.Dir == st.Dir {
				cur.Members = append(cur.Members, u.Members...)
				continue
			}
			flush()
			cur = &st
		}
		flush()
	}
	return out, nil
}

// meshOfFaces is the body's tessellation, or its box when it would not tessellate.
func meshOfFaces(bf *bodyFaces) explodeblock.Mesh {
	if !bf.unmeshed && len(bf.tris) > 0 {
		return explodeblock.Mesh{Vertices: bf.verts, Triangles: bf.tris, Bound: units.Millimeters(bf.meshBound)}
	}
	lo, hi := bf.box.Min, bf.box.Max
	var vs []r3.Vec
	for i := range 8 {
		vs = append(vs, r3.Vec{X: pick(i&1, lo.X, hi.X), Y: pick(i&2, lo.Y, hi.Y), Z: pick(i&4, lo.Z, hi.Z)})
	}
	tris := [][3]int{{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1}, {2, 3, 7}, {2, 7, 6},
		{0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}}
	return explodeblock.Mesh{Vertices: vs, Triangles: tris, Bound: bf.box.Bound}
}

func pick(bit int, lo, hi float64) float64 {
	if bit == 0 {
		return lo
	}
	return hi
}

func near(a, b r3.Vec, tol float64) bool {
	return math.Abs(a.X-b.X) <= tol && math.Abs(a.Y-b.Y) <= tol && math.Abs(a.Z-b.Z) <= tol
}

// nearest is the index of the direction closest to v.
func nearest(dirs []r3.Vec, v r3.Vec) int {
	u, _ := v.Normalize()
	best, bestDot := 0, math.Inf(-1)
	for i, d := range dirs {
		if dot := d.Dot(u); dot > bestDot {
			best, bestDot = i, dot
		}
	}
	return best
}

// constraints is WithRemoval resolved against the Set, in plan indices.
type constraints struct {
	stated   []r3.Vec // the stated directions
	fixedVec map[int]r3.Vec
	fixed    []int
	group    []int
	after    [][]int
}

func (s *Set) removalConstraints(c config, in derivedInput, named []bool) (constraints, error) {
	n := len(in.idx)
	cons := constraints{fixedVec: map[int]r3.Vec{}, fixed: make([]int, n), group: make([]int, n), after: make([][]int, n)}
	for k := range n {
		cons.fixed[k], cons.group[k] = -1, -1
	}
	lookup := func(owner, field, nm string) (int, error) {
		i, ok := s.index[nm]
		k, bodied := in.plan[i]
		if !ok || !bodied {
			return 0, fmt.Errorf("parts: %w: removal of %q: %s names %q, which is unknown or has no body", decad.ErrDegenerate,
				owner, field, nm)
		}
		if k == in.ground {
			return 0, fmt.Errorf("parts: %w: removal of %q: %s names the ground %q, which never leaves", decad.ErrDegenerate,
				owner, field, nm)
		}
		return k, nil
	}
	owners := make([]string, 0, len(c.removals))
	for nm := range c.removals {
		owners = append(owners, nm)
	}
	slices.Sort(owners)
	groups := map[string]int{}
	for _, nm := range owners {
		r := c.removals[nm]
		k, err := lookup(nm, "WithRemoval", nm)
		if err != nil {
			return cons, err
		}
		if named[k] {
			return cons, fmt.Errorf("parts: %w: removal of %q: an explicit stage already moves it", decad.ErrDegenerate, nm)
		}
		if u, ok := r.Direction.Normalize(); ok {
			cons.stated = append(cons.stated, u)
			cons.fixedVec[k] = u
		}
		if r.Group != "" {
			g, ok := groups[r.Group]
			if !ok {
				g = len(groups)
				groups[r.Group] = g
			}
			cons.group[k] = g
		}
		for _, a := range r.After {
			ak, err := lookup(nm, "After", a)
			if err != nil {
				return cons, err
			}
			if ak == k {
				return cons, fmt.Errorf("parts: %w: removal of %q: it cannot leave after itself", decad.ErrDegenerate, nm)
			}
			cons.after[k] = append(cons.after[k], ak)
		}
		for _, b := range r.Before {
			bk, err := lookup(nm, "Before", b)
			if err != nil {
				return cons, err
			}
			if bk == k {
				return cons, fmt.Errorf("parts: %w: removal of %q: it cannot leave before itself", decad.ErrDegenerate, nm)
			}
			if named[bk] {
				return cons, fmt.Errorf("parts: %w: removal of %q: Before names %q, which an explicit stage already moved",
					decad.ErrDegenerate, nm, b)
			}
			cons.after[bk] = append(cons.after[bk], k)
		}
	}
	return cons, nil
}

// orderError turns the search's errors into messages naming parts.
func (s *Set) orderError(err error, dirs []r3.Vec, in derivedInput) error {
	name := func(k int) string { return fmt.Sprintf("%q", s.parts[in.idx[k]].Name) }
	names := func(ks []int) string {
		var out []string
		for _, k := range ks {
			out = append(out, name(k))
		}
		return strings.Join(out, ", ")
	}
	var nd *explodeorder.NoDirectionError
	var cf *explodeorder.ConflictError
	switch {
	case errors.As(err, &nd):
		var along []string
		for _, b := range nd.Blocked {
			d := dirs[b.Dir]
			along = append(along, fmt.Sprintf("along (%.3g, %.3g, %.3g) by %s", d.X, d.Y, d.Z, names(b.By)))
		}
		what := "has no free direction"
		if nd.Forced {
			what = "has no free way out along the direction WithRemoval states"
		}
		more := ""
		if nd.Others > 0 {
			more = fmt.Sprintf("; %d more parts are stuck", nd.Others)
		}
		return fmt.Errorf("parts: %w: part %s %s: blocked %s%s", decad.ErrDegenerate, name(nd.Part), what,
			strings.Join(along, "; "), more)
	case errors.As(err, &cf):
		switch cf.Kind {
		case explodeorder.OrderCycle:
			return fmt.Errorf("parts: %w: WithRemoval order is a cycle through %s", decad.ErrDegenerate, names(cf.Parts))
		case explodeorder.Waits:
			return fmt.Errorf("parts: %w: part %s must leave after %s, which cannot leave first", decad.ErrDegenerate,
				name(cf.Parts[0]), names(cf.Parts[1:]))
		default:
			return fmt.Errorf("parts: %w: parts %s are in one removal group but fixed to different directions",
				decad.ErrDegenerate, names(cf.Parts))
		}
	}
	return err
}

// layoutScorer ranks directions by the layout rules of §5.8.
type layoutScorer struct {
	dirs         []r3.Vec
	joint        []bool
	boxes        []explodeplan.Box
	zero         []r3.Vec
	groundCentre r3.Vec
	projs        [][]*explodeblock.Projection // per direction, per plan index
}

func (l *layoutScorer) Joint(d int) bool    { return l.joint[d] }
func (l *layoutScorer) Positive(d int) bool { return explodeplan.DominantPositive(l.dirs[d]) }
func (l *layoutScorer) union(ms []int) explodeplan.Box {
	b := l.boxes[ms[0]]
	for _, m := range ms[1:] {
		o := l.boxes[m]
		b = explodeplan.Box{Min: r3.Vec{X: math.Min(b.Min.X, o.Min.X), Y: math.Min(b.Min.Y, o.Min.Y), Z: math.Min(b.Min.Z, o.Min.Z)},
			Max: r3.Vec{X: math.Max(b.Max.X, o.Max.X), Y: math.Max(b.Max.Y, o.Max.Y), Z: math.Max(b.Max.Z, o.Max.Z)}}
	}
	return b
}

func (l *layoutScorer) Outward(ms []int, d int) bool {
	return l.union(ms).Centre().Sub(l.groundCentre).Dot(l.dirs[d]) > 1e-9
}

func (l *layoutScorer) Travel(ms []int, d int, remaining []bool) float64 {
	var standing []int
	for p, ok := range remaining {
		if ok && !slices.Contains(ms, p) {
			standing = append(standing, p)
		}
	}
	return explodeplan.LeastClear(l.union(ms), l.dirs[d], 0, l.boxes, l.zero, standing)
}

func (l *layoutScorer) Trailing(ms []int, d int) float64 {
	lo := math.Inf(1)
	for _, m := range ms {
		a, _ := explodeblock.Instance{P: l.projs[d][m]}.Span()
		lo = math.Min(lo, a.Base())
	}
	return lo
}
