// Package explodeorder searches for the order in which parts of an assembly
// leave it (agent-docs/parts-design.md §5.8). It works on plain part indices
// and per-direction blocker lists and touches no body.
//
// Every search round removes each part that has no blocker among the parts
// still present along some candidate direction. When no part can leave, parts
// that block each other along one direction form a strongly connected component
// and leave together as one rigid group. Each round removes at least one part or
// the search stops with an error, so it always terminates.
//
// The search finds each part's earliest round. A second pass then moves each
// part that waits for no other part to the latest round before the first part it
// blocks or must leave before, keeping its direction, so a part that is free from
// the start leaves next to the part it frees.
package explodeorder

import (
	"cmp"
	"slices"
)

// Scorer ranks the directions a part may leave by. Directions are indices into
// the candidate list the blocker lists were built for.
type Scorer interface {
	// Joint reports whether direction d is a joint axis or a stated pull.
	Joint(d int) bool
	// Outward reports whether d points from the ground toward the members.
	Outward(members []int, d int) bool
	// Travel is the distance the members must move along d to clear every part
	// still present, boxes only.
	Travel(members []int, d int, remaining []bool) float64
	// Trailing is the lowest position of the members along d.
	Trailing(members []int, d int) float64
	// Positive reports whether the largest component of d is positive.
	Positive(d int) bool
}

// Input is everything Search reads. Parts are 0..N-1.
type Input struct {
	N      int
	Ground int // never leaves
	// Gone marks parts earlier stages already moved. They never block.
	Gone []bool
	// Blockers[d][p] lists the parts that block p along direction d. Only the
	// rows of parts that can leave are read.
	Blockers [][][]int
	// Group is the rigid group id per part, -1 for none. Parts of one group leave
	// together.
	Group []int
	// After lists, per part, the parts that must have left in an earlier round.
	After [][]int
	// Fixed is the direction a part must leave by, -1 for any.
	Fixed []int
	Score Scorer
}

// Unit is parts that leave together along one direction.
type Unit struct {
	Members []int // ascending
	Dir     int
}

// DirBlock is one direction and the parts that block along it.
type DirBlock struct {
	Dir int
	By  []int
}

// NoDirectionError says a part has no direction along which it can leave.
type NoDirectionError struct {
	Part    int
	Forced  bool // the part's direction is fixed, so Blocked has one entry
	Blocked []DirBlock
	Others  int // other parts that were still present and stuck
}

func (e *NoDirectionError) Error() string { return "explodeorder: part has no free direction" }

// ConflictKind says which constraint could not hold.
type ConflictKind int

const (
	// OrderCycle: the parts wait for each other. Parts lists the cycle.
	OrderCycle ConflictKind = iota
	// Waits: Parts[0] could leave but waits for Parts[1:], which cannot.
	Waits
	// GroupDirection: the members of a group are fixed to different
	// directions. Parts lists two members.
	GroupDirection
)

// ConflictError says the constraints contradict each other or the blocking.
type ConflictError struct {
	Kind  ConflictKind
	Parts []int
}

func (e *ConflictError) Error() string { return "explodeorder: constraints conflict" }

type unit struct {
	members []int
	dir     int // -1: any
	after   []int
	ground  bool
	removed bool
}

type searcher struct {
	in      Input
	dirs    int
	units   []unit
	unitOf  []int
	present []bool // parts not yet removed, the ground included
	gone    []bool
}

// Search returns the rounds in which the parts leave. Parts earlier stages
// moved (Gone) and the ground are not in the result.
func Search(in Input) ([][]Unit, error) {
	s := &searcher{in: in, dirs: len(in.Blockers)}
	if err := s.init(); err != nil {
		return nil, err
	}
	var picks []pick
	for r := 0; s.left() > 0; r++ {
		cs, err := s.round()
		if err != nil {
			return nil, err
		}
		for _, c := range cs {
			picks = append(picks, pick{chosen: c, round: r})
		}
	}
	s.settle(picks)
	return s.rounds(picks), nil
}

// pick is a unit or group the search removed and the round it leaves in.
type pick struct {
	chosen
	round int
}

// settle moves each pick that waits for no other pick to the latest round before the first pick it must
// leave before: one it blocks along that pick's direction, or one whose After names it. A pick waits
// for another when that one blocks it along its direction or its After names it; such a pick keeps its
// search round, just after the last pick it waits for. Each part then sits next to a part it depends on:
// the one that freed it, or, when nothing held it, the one it frees.
//
// It walks the picks latest first, so every pick it could hold back already has its final round. A
// pick q that p must leave before was removed in a later search round than p (p was present when q was
// found free), so it is walked before p, and picks of one search round never hold each other. A moved
// pick has no blocker among the picks along its direction, so it stays free in any round. Each pick
// moves to a later round or stays, never past a pick it must leave before, so blockers keep their order
// and every After holds.
func (s *searcher) settle(picks []pick) {
	if len(picks) == 0 {
		return
	}
	pickOf := make([]int, len(s.units))
	for i := range pickOf {
		pickOf[i] = -1
	}
	for i, p := range picks {
		for _, u := range p.units {
			pickOf[u] = i
		}
	}
	held := make([][]int, len(picks)) // held[i]: the picks that pick i must leave before
	waits := make([]bool, len(picks)) // waits[j]: pick j must leave after some other pick
	for j, q := range picks {
		var need []int
		for _, m := range s.membersOf(q.units) {
			for _, b := range s.in.Blockers[q.dir][m] {
				if u := s.unitOf[b]; u >= 0 && pickOf[u] >= 0 && pickOf[u] != j {
					need = append(need, pickOf[u])
				}
			}
		}
		for _, u := range q.units {
			for _, a := range s.units[u].after {
				if pickOf[a] >= 0 {
					need = append(need, pickOf[a])
				}
			}
		}
		slices.Sort(need)
		waits[j] = len(need) > 0
		for _, i := range slices.Compact(need) {
			held[i] = append(held[i], j)
		}
	}
	last := picks[len(picks)-1].round
	for i := len(picks) - 1; i >= 0; i-- {
		if waits[i] {
			continue
		}
		limit := last
		for _, j := range held[i] {
			limit = min(limit, picks[j].round-1)
		}
		picks[i].round = max(picks[i].round, limit)
	}
}

// rounds groups the picks by their settled round, dropping rounds left empty. Inside a round the
// units keep their index order, then the part whose trailing face is lower goes closer in.
func (s *searcher) rounds(picks []pick) [][]Unit {
	byRound := map[int][]pick{}
	var keys []int
	for _, p := range picks {
		if _, ok := byRound[p.round]; !ok {
			keys = append(keys, p.round)
		}
		byRound[p.round] = append(byRound[p.round], p)
	}
	slices.Sort(keys)
	out := make([][]Unit, 0, len(keys))
	for _, k := range keys {
		ps := byRound[k]
		slices.SortStableFunc(ps, func(a, b pick) int { return cmp.Compare(a.units[0], b.units[0]) })
		round := make([]Unit, 0, len(ps))
		for _, p := range ps {
			round = append(round, Unit{Members: s.membersOf(p.units), Dir: p.dir})
		}
		slices.SortStableFunc(round, func(a, b Unit) int {
			return cmp.Compare(s.in.Score.Trailing(a.Members, a.Dir), s.in.Score.Trailing(b.Members, b.Dir))
		})
		out = append(out, round)
	}
	return out
}

func (s *searcher) init() error {
	n := s.in.N
	s.unitOf = make([]int, n)
	for i := range s.unitOf {
		s.unitOf[i] = -1
	}
	s.present = make([]bool, n)
	s.gone = s.in.Gone
	byGroup := map[int]int{}
	for p := range n {
		if s.gone[p] {
			continue
		}
		s.present[p] = true
		if g := s.in.Group[p]; g >= 0 && p != s.in.Ground {
			if u, ok := byGroup[g]; ok {
				s.unitOf[p] = u
				s.units[u].members = append(s.units[u].members, p)
				continue
			}
			byGroup[g] = len(s.units)
		}
		s.unitOf[p] = len(s.units)
		s.units = append(s.units, unit{members: []int{p}, dir: -1, ground: p == s.in.Ground})
	}
	for ui := range s.units {
		u := &s.units[ui]
		first := -1
		for _, m := range u.members {
			f := s.in.Fixed[m]
			if f < 0 {
				continue
			}
			if first >= 0 && u.dir != f {
				return &ConflictError{Kind: GroupDirection, Parts: []int{first, m}}
			}
			if first < 0 {
				first = m
			}
			u.dir = f
		}
	}
	for ui := range s.units {
		u := &s.units[ui]
		for _, m := range u.members {
			for _, a := range s.in.After[m] {
				if s.gone[a] || s.unitOf[a] == ui {
					continue
				}
				if !slices.Contains(u.after, s.unitOf[a]) {
					u.after = append(u.after, s.unitOf[a])
				}
			}
		}
	}
	return s.checkCycles()
}

// left is the number of units that have not left, the ground's excluded.
func (s *searcher) left() int {
	n := 0
	for _, u := range s.units {
		if !u.removed && !u.ground {
			n++
		}
	}
	return n
}

func (s *searcher) checkCycles() error {
	state := make([]int, len(s.units)) // 0 new, 1 on the path, 2 done
	var path []int
	var visit func(u int) []int
	visit = func(u int) []int {
		state[u] = 1
		path = append(path, u)
		for _, v := range s.units[u].after {
			switch state[v] {
			case 1:
				i := slices.Index(path, v)
				var cyc []int
				for _, w := range path[i:] {
					cyc = append(cyc, s.units[w].members[0])
				}
				return cyc
			case 0:
				if c := visit(v); c != nil {
					return c
				}
			}
		}
		path = path[:len(path)-1]
		state[u] = 2
		return nil
	}
	for u := range s.units {
		if state[u] != 0 {
			continue
		}
		if c := visit(u); c != nil {
			return &ConflictError{Kind: OrderCycle, Parts: c}
		}
	}
	return nil
}

func (s *searcher) eligible(ui int, ignoreAfter bool) bool {
	u := s.units[ui]
	if u.removed || u.ground {
		return false
	}
	if ignoreAfter {
		return true
	}
	for _, a := range u.after {
		if !s.units[a].removed {
			return false
		}
	}
	return true
}

// blockersOf lists, ascending, the present parts outside unit ui that block it
// along d.
func (s *searcher) blockersOf(ui, d int) []int {
	var out []int
	for _, m := range s.units[ui].members {
		for _, b := range s.in.Blockers[d][m] {
			if s.present[b] && s.unitOf[b] != ui && !slices.Contains(out, b) {
				out = append(out, b)
			}
		}
	}
	slices.Sort(out)
	return out
}

func (s *searcher) allows(ui, d int) bool { return s.units[ui].dir < 0 || s.units[ui].dir == d }

// better reports whether leaving along da beats db: a joint axis first, then
// the side away from the ground, then the shorter travel, then a positive
// largest component, then the lower direction index.
func (s *searcher) better(ma []int, da int, mb []int, db int) bool {
	sc := s.in.Score
	if ja, jb := sc.Joint(da), sc.Joint(db); ja != jb {
		return ja
	}
	if oa, ob := sc.Outward(ma, da), sc.Outward(mb, db); oa != ob {
		return oa
	}
	ta, tb := sc.Travel(ma, da, s.present), sc.Travel(mb, db, s.present)
	if d := ta - tb; d > 1e-9 || d < -1e-9 {
		return ta < tb
	}
	if pa, pb := sc.Positive(da), sc.Positive(db); pa != pb {
		return pa
	}
	return da < db
}

// freeDir returns the best direction along which unit ui has no blocker.
func (s *searcher) freeDir(ui int) (int, bool) {
	best := -1
	m := s.units[ui].members
	for d := range s.dirs {
		if !s.allows(ui, d) || len(s.blockersOf(ui, d)) > 0 {
			continue
		}
		if best < 0 || s.better(m, d, m, best) {
			best = d
		}
	}
	return best, best >= 0
}

// step finds the units that leave next: every unit with a free direction, else
// one group of mutually blocking units.
func (s *searcher) step(ignoreAfter bool) []chosen {
	var out []chosen
	for ui := range s.units {
		if !s.eligible(ui, ignoreAfter) {
			continue
		}
		if d, ok := s.freeDir(ui); ok {
			out = append(out, chosen{[]int{ui}, d})
		}
	}
	if len(out) > 0 {
		return out
	}
	if g, ok := s.groupStep(ignoreAfter); ok {
		return []chosen{g}
	}
	return nil
}

type chosen struct {
	units []int
	dir   int
}

func (s *searcher) membersOf(us []int) []int {
	var ms []int
	for _, u := range us {
		ms = append(ms, s.units[u].members...)
	}
	slices.Sort(ms)
	return ms
}

// round removes the units of the next search round.
func (s *searcher) round() ([]chosen, error) {
	picks := s.step(false)
	if picks == nil {
		return nil, s.stuck()
	}
	for _, c := range picks {
		for _, u := range c.units {
			s.units[u].removed = true
			for _, m := range s.units[u].members {
				s.present[m] = false
			}
		}
	}
	return picks, nil
}

// stuck explains why no unit can leave.
func (s *searcher) stuck() error {
	if free := s.step(true); free != nil {
		for _, c := range free {
			for _, ui := range c.units {
				var pending []int
				for _, a := range s.units[ui].after {
					if !s.units[a].removed {
						pending = append(pending, s.units[a].members[0])
					}
				}
				if len(pending) > 0 {
					return &ConflictError{Kind: Waits, Parts: append([]int{s.units[ui].members[0]}, pending...)}
				}
			}
		}
	}
	// A part fixed to one direction is the likelier cause, so it is named first.
	order := slices.Clone(s.unitOrder())
	slices.SortStableFunc(order, func(a, b int) int {
		return cmp.Compare(boolInt(s.units[b].dir >= 0), boolInt(s.units[a].dir >= 0))
	})
	for _, ui := range order {
		u := s.units[ui]
		if u.removed || u.ground {
			continue
		}
		e := &NoDirectionError{Part: u.members[0], Forced: u.dir >= 0, Others: s.left() - 1}
		for d := range s.dirs {
			if s.allows(ui, d) {
				e.Blocked = append(e.Blocked, DirBlock{Dir: d, By: s.blockersOf(ui, d)})
			}
		}
		return e
	}
	return nil
}

// groupStep finds units that block each other along one direction and have no
// other blocker along it: a sink of the blocking graph with more than one unit.
// The smallest such group wins, then the preferred direction.
func (s *searcher) groupStep(ignoreAfter bool) (chosen, bool) {
	var best *chosen
	for d := range s.dirs {
		for _, comp := range s.sinks(d) {
			ok := len(comp) > 1
			for _, ui := range comp {
				ok = ok && s.eligible(ui, ignoreAfter) && s.allows(ui, d)
			}
			if !ok {
				continue
			}
			c := chosen{comp, d}
			if best == nil || s.betterGroup(c, *best) {
				cc := c
				best = &cc
			}
		}
	}
	if best == nil {
		return chosen{}, false
	}
	return *best, true
}

func (s *searcher) betterGroup(a, b chosen) bool {
	if len(a.units) != len(b.units) {
		return len(a.units) < len(b.units)
	}
	ma, mb := s.membersOf(a.units), s.membersOf(b.units)
	if a.dir != b.dir && s.better(ma, a.dir, mb, b.dir) != s.better(mb, b.dir, ma, a.dir) {
		return s.better(ma, a.dir, mb, b.dir)
	}
	return slices.Compare(ma, mb) < 0
}

// sinks returns, for direction d, the strongly connected components of the
// blocking graph over the units still present that no edge leaves. Each
// component lists its units ascending.
func (s *searcher) sinks(d int) [][]int {
	n := len(s.units)
	succ := make([][]int, n)
	for ui := range s.units {
		if s.units[ui].removed || s.units[ui].ground {
			continue
		}
		for _, b := range s.blockersOf(ui, d) {
			if v := s.unitOf[b]; !slices.Contains(succ[ui], v) {
				succ[ui] = append(succ[ui], v)
			}
		}
	}
	idx := make([]int, n)
	low := make([]int, n)
	on := make([]bool, n)
	comp := make([]int, n)
	for i := range idx {
		idx[i], comp[i] = -1, -1
	}
	var stack []int
	var comps [][]int
	counter := 0
	var visit func(v int)
	visit = func(v int) {
		idx[v], low[v] = counter, counter
		counter++
		stack = append(stack, v)
		on[v] = true
		for _, w := range succ[v] {
			if idx[w] < 0 {
				visit(w)
				low[v] = min(low[v], low[w])
			} else if on[w] {
				low[v] = min(low[v], idx[w])
			}
		}
		if low[v] != idx[v] {
			return
		}
		var c []int
		for {
			w := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			on[w] = false
			comp[w] = len(comps)
			c = append(c, w)
			if w == v {
				break
			}
		}
		slices.Sort(c)
		comps = append(comps, c)
	}
	for ui := range s.units {
		if !s.units[ui].removed && idx[ui] < 0 {
			visit(ui)
		}
	}
	var out [][]int
	for ci, c := range comps {
		sink := true
		for _, v := range c {
			for _, w := range succ[v] {
				if comp[w] != ci {
					sink = false
				}
			}
		}
		if sink {
			out = append(out, c)
		}
	}
	slices.SortFunc(out, func(a, b []int) int { return slices.Compare(a, b) })
	return out
}

func (s *searcher) unitOrder() []int {
	out := make([]int, len(s.units))
	for i := range out {
		out[i] = i
	}
	return out
}

func boolInt(b bool) int {
	if b {
		return 1
	}
	return 0
}
