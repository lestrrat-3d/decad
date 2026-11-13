package parts

import (
	"context"
	"errors"
	"fmt"
	"math"
	"runtime"
	"slices"
	"strings"
	"sync"
	"sync/atomic"

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
// and not a proof (agent-docs/parts-design.md §5.8). For a thin assembly whose
// layers are not coaxial, the layers are also spread across their axis (§5.9);
// WithoutExplodeSpread keeps them stacked.
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
	jointAxes []jointAxis
}

type derivedStages struct {
	stages []explodeplan.Stage
	names  []string
	order  []int
	nodes  []explodeplan.Node
	report derivedReport
}

// jointAxis is an axis or pull a joint offers as a candidate direction, and where it was read from.
type jointAxis struct {
	dir r3.Vec
	src PullSource
}

// derivedReport is what the search says about the layout, in plan indices for the per-part slices.
type derivedReport struct {
	order, round []int    // 1-based; 0 for parts the search did not move
	grouped      []bool   // true when the part's WithRemoval names a Group
	dir          []r3.Vec // the unit direction each moved part leaves by
	src          []PullSource
	blocked      []BlockedPair
	undecided    []UndecidedPair
}

// report reads the pairs that forced the order and the pairs read as free off the tables, for the
// direction each unit left by.
func (s *Set) report(in derivedInput, items []int, tables []*explodeblock.Table, blockers [][][]int, rounds [][]explodeorder.Unit,
	cons constraints, axisSrc []PullSource, dirs []r3.Vec) derivedReport {
	n := len(in.idx)
	rep := derivedReport{order: make([]int, n), round: make([]int, n), src: make([]PullSource, n), dir: make([]r3.Vec, n),
		grouped: make([]bool, n)}
	for k := range n {
		rep.grouped[k] = cons.group[k] >= 0
	}
	pos := make([]int, n) // plan index -> table index
	for i, k := range items {
		pos[k] = i
	}
	dirOf, unitOf := make([]int, n), make([]int, n)
	seq, unitNo := 0, 0
	for r, round := range rounds {
		for _, u := range round {
			unitNo++
			for _, m := range u.Members {
				seq++
				rep.order[m], rep.round[m], dirOf[m], unitOf[m] = seq, r+1, u.Dir, unitNo
				rep.src[m], rep.dir[m] = axisSrc[u.Dir], dirs[u.Dir]
				if _, ok := cons.fixedVec[m]; ok {
					rep.src[m] = RemovalDirection
				}
			}
		}
	}
	name := func(k int) string { return s.parts[in.idx[k]].Name }
	var movers []int
	for k := range n {
		if rep.order[k] > 0 {
			movers = append(movers, k)
		}
	}
	slices.SortFunc(movers, func(a, b int) int { return rep.order[a] - rep.order[b] })
	for _, m := range movers {
		d := dirOf[m]
		tab := tables[d]
		for _, b := range blockers[d][m] {
			if unitOf[b] == unitOf[m] {
				continue
			}
			w := tab.At(pos[m], pos[b]).Witness
			rep.blocked = append(rep.blocked, BlockedPair{Mover: name(m), Blocker: name(b), Dir: dirs[d], Point: w.Point,
				Height: w.Height})
		}
		for _, bi := range tab.Uncertain(pos[m]) {
			b := items[bi]
			if unitOf[b] == unitOf[m] || (rep.round[b] > 0 && rep.round[b] < rep.round[m]) {
				continue
			}
			rep.undecided = append(rep.undecided, UndecidedPair{Mover: name(m), Other: name(b), Dir: dirs[d]})
		}
	}
	return rep
}

// deriveStages appends the removal rounds to the explicit stages, the round removed last first.
func (s *Set) deriveStages(ctx context.Context, c config, in derivedInput) (derivedStages, error) {
	var out derivedStages
	n := len(in.idx)
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
	dirs := explodeblock.Directions(append(slices.Clone(cons.stated), jointDirs(in.jointAxes)...), math.Sin(c.axisTol))
	for k, f := range cons.fixedVec {
		cons.fixed[k] = nearest(dirs, f)
	}
	sc := &layoutScorer{dirs: dirs, boxes: in.boxes, zero: make([]r3.Vec, n), joint: make([]bool, len(dirs)),
		trail: make([][]float64, len(dirs))}
	tol := math.Sin(c.axisTol)
	axisSrc := make([]PullSource, len(dirs)) // the source of each direction when a part leaves freely by it
	rank := map[PullSource]int{PullDeclared: 3, AxisFromFaces: 2, AxisDeclared: 1}
	for i, d := range dirs {
		axisSrc[i] = WorldAxis
		best := 0
		for _, a := range in.jointAxes {
			u, ok := a.dir.Normalize()
			if !ok || !(near(u, d, tol) || near(u.Scale(-1), d, tol)) {
				continue
			}
			sc.joint[i] = true
			if r := rank[a.src]; r > best {
				best, axisSrc[i] = r, a.src
			}
		}
	}
	sc.groundCentre = in.boxes[in.ground].Centre()
	// Identical bodies at different positions share one projection per direction: the meshes are
	// grouped once, and each part is an offset of its class.
	byBody := map[*decad.Body]explodeblock.Mesh{}
	meshes := make([]explodeblock.Mesh, len(items))
	for i, k := range items {
		b := s.parts[in.idx[k]].Body
		m, ok := byBody[b]
		if !ok {
			bf, err := in.faces.get(ctx, b)
			if err != nil {
				return out, err
			}
			m = meshOfFaces(bf)
			if c.boxOnly {
				m = meshOfBox(bf)
			}
			byBody[b] = m
		}
		meshes[i] = m
	}
	shares, classes := explodeblock.Group(meshes)
	workers := runtime.GOMAXPROCS(0)
	tables := make([]*explodeblock.Table, len(dirs))
	blockers := make([][][]int, len(dirs))
	for di, d := range dirs {
		if err := ctx.Err(); err != nil {
			return out, err
		}
		projs, err := projectClasses(ctx, classes, d, workers, c.projCount)
		if err != nil {
			return out, fmt.Errorf("parts: %w", err)
		}
		insts := make([]explodeblock.Instance, len(items))
		sc.trail[di] = make([]float64, n)
		for i, k := range items {
			insts[i] = explodeblock.Instance{P: projs[shares[i].Class], Offset: shares[i].Offset}
			lo, _ := insts[i].Span()
			sc.trail[di][k] = lo.Base()
		}
		tab := explodeblock.BuildTable(insts, workers)
		tables[di] = tab
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
	out.report = s.report(in, items, tables, blockers, rounds, cons, axisSrc, dirs)
	out.stages, out.names = slices.Clone(in.stages.stages), slices.Clone(in.stages.names)
	for _, p := range in.walk {
		if named[p] {
			out.order = append(out.order, p)
		}
	}
	out.nodes = slices.Clone(in.nodes)
	for _, round := range rounds {
		for _, u := range round {
			for _, m := range u.Members {
				out.order = append(out.order, m)
				out.nodes[m].Parent = -1
			}
		}
	}
	// The round removed last runs first and sits nearest the assembly; each earlier round runs after it
	// and ends beyond it (D7).
	for r, round := range slices.Backward(rounds) {
		var stages []explodeplan.Stage
		single := map[int]int{} // direction index -> its single-part stage in stages
		for _, u := range round {
			st := explodeplan.Stage{Members: slices.Clone(u.Members), Gap: math.NaN(), Clear: explodeplan.ClearAll,
				Dir: dirs[u.Dir], Together: len(u.Members) > 1, Round: r + 1}
			if st.Together {
				stages = append(stages, st)
				continue
			}
			if i, ok := single[u.Dir]; ok {
				stages[i].Members = append(stages[i].Members, u.Members...)
				continue
			}
			single[u.Dir] = len(stages)
			stages = append(stages, st)
		}
		for i, st := range stages {
			label := fmt.Sprintf("removal %d", r+1)
			if i > 0 {
				label = fmt.Sprintf("removal %d.%d", r+1, i+1)
			}
			out.names = append(out.names, label)
			out.stages = append(out.stages, st)
		}
	}
	return out, nil
}

// meshOfFaces is the body's tessellation, or its box when it would not tessellate.
func meshOfFaces(bf *bodyFaces) explodeblock.Mesh {
	if !bf.unmeshed && len(bf.tris) > 0 {
		return explodeblock.Mesh{Vertices: bf.verts, Triangles: bf.tris, Bound: units.Millimeters(bf.meshBound)}
	}
	return meshOfBox(bf)
}

// meshOfBox is the body's box as a mesh.
func meshOfBox(bf *bodyFaces) explodeblock.Mesh {
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
	trail        [][]float64 // per direction, per plan index: the lowest position along it
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
		lo = math.Min(lo, l.trail[d][m])
	}
	return lo
}

// projectClasses projects every class mesh along d with at most workers goroutines. Each projection
// built adds one to count when it is not nil.
func projectClasses(ctx context.Context, classes []explodeblock.Mesh, d r3.Vec, workers int,
	count *atomic.Int64) ([]*explodeblock.Projection, error) {
	out := make([]*explodeblock.Projection, len(classes))
	errs := make([]error, len(classes))
	next := make(chan int, len(classes))
	for i := range classes {
		next <- i
	}
	close(next)
	var wg sync.WaitGroup
	for range min(max(workers, 1), max(len(classes), 1)) {
		wg.Go(func() {
			for i := range next {
				if ctx.Err() != nil {
					return
				}
				out[i], errs[i] = explodeblock.Project(classes[i], d)
				if count != nil {
					count.Add(1)
				}
			}
		})
	}
	wg.Wait()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := errors.Join(errs...); err != nil {
		return nil, err
	}
	return out, nil
}

func jointDirs(as []jointAxis) []r3.Vec {
	out := make([]r3.Vec, len(as))
	for i, a := range as {
		out[i] = a.dir
	}
	return out
}
