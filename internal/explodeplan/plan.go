// Package explodeplan walks the joint graph of a build from a ground part and
// chooses each part's offset for an exploded view (agent-docs/parts-design.md
// §5). It works on plain millimetre boxes and touches no body.
package explodeplan

import (
	"context"
	"math"

	"github.com/lestrrat-3d/r3"
)

// Box is an axis-aligned box.
type Box struct{ Min, Max r3.Vec }

// Centre returns the box centre.
func (b Box) Centre() r3.Vec { return b.Min.Add(b.Max).Scale(0.5) }

// Mode says how a joint gives the direction a part is pulled.
type Mode int

const (
	// ModeCentres has no direction of its own: the part moves along the line
	// from its parent's box centre to its own.
	ModeCentres Mode = iota
	// ModeAxis pulls along Dir in either sense.
	ModeAxis
	// ModeAway pulls A along Dir and B along -Dir: Dir is the direction that
	// takes A away from B.
	ModeAway
)

// Joint joins parts A and B, which index the boxes given to Plan. Source is
// carried to the result untouched.
type Joint struct {
	A, B   int
	Mode   Mode
	Dir    r3.Vec // unit; unused for ModeCentres
	Source int
}

// Blocker answers whether a part, swept from its assembled box moved by from to
// that box moved by to, stays strictly clear of every other part's box except
// those in ignore. offsets holds each part's offset so far; a part no joint has
// reached yet sits at its assembled place.
type Blocker interface {
	Clear(ctx context.Context, part int, from, to r3.Vec, offsets []r3.Vec, ignore []int) (bool, error)
}

// Result is one part's place in the plan.
type Result struct {
	Offset r3.Vec
	Depth  int // -1 when no joint reaches the part
	Joint  int // the parent joint's index, -1 for the ground and unreached parts
	Source int // the parent joint's Source, -1 when Joint is -1
}

const eps = 1e-9

// Plan walks the joints breadth-first from ground, joints in input order, and
// returns one Result per box. A part reached twice keeps its first parent. gap
// is added to every travel.
func Plan(ctx context.Context, boxes []Box, joints []Joint, ground int, gap float64, blocker Blocker) ([]Result, error) {
	res := make([]Result, len(boxes))
	for i := range res {
		res[i] = Result{Depth: -1, Joint: -1, Source: -1}
	}
	res[ground].Depth = 0
	offsets := make([]r3.Vec, len(boxes))
	queue := []int{ground}
	for len(queue) > 0 {
		p := queue[0]
		queue = queue[1:]
		for ji, j := range joints {
			var child int
			switch p {
			case j.A:
				child = j.B
			case j.B:
				child = j.A
			default:
				continue
			}
			if child == p || res[child].Depth >= 0 {
				continue
			}
			own, err := ownOffset(ctx, boxes, joints, res, offsets, j, p, child, gap, blocker)
			if err != nil {
				return nil, err
			}
			res[child] = Result{Offset: res[p].Offset.Add(own), Depth: res[p].Depth + 1, Joint: ji, Source: j.Source}
			offsets[child] = res[child].Offset
			queue = append(queue, child)
		}
	}
	return res, nil
}

// ownOffset is the child's move away from its parent. The parts the child is
// joined to never block it, because they are joined to it; those already
// placed are cleared by the travel itself.
func ownOffset(ctx context.Context, boxes []Box, joints []Joint, res []Result, offsets []r3.Vec, j Joint, parent, child int,
	gap float64, blocker Blocker) (r3.Vec, error) {
	dir, both := direction(boxes, j, parent, child)
	if dir == (r3.Vec{}) {
		return r3.Vec{}, nil
	}
	cleared := []int{parent}
	partners := []int{parent}
	for _, o := range joints {
		other := -1
		switch child {
		case o.A:
			other = o.B
		case o.B:
			other = o.A
		}
		if other < 0 || other == child {
			continue
		}
		partners = append(partners, other)
		if res[other].Depth >= 0 {
			cleared = append(cleared, other)
		}
	}
	start := res[parent].Offset
	senses := []r3.Vec{dir}
	if both {
		senses = append(senses, dir.Scale(-1))
	}
	type choice struct {
		move r3.Vec
		free bool
		t    float64
	}
	var best *choice
	for _, s := range senses {
		t := 0.0
		for _, c := range cleared {
			t = math.Max(t, travel(shift(boxes[child], start), shift(boxes[c], res[c].Offset), s))
		}
		if math.IsInf(t, 1) {
			continue
		}
		free, err := blocker.Clear(ctx, child, start, start.Add(s.Scale(t)), offsets, partners)
		if err != nil {
			return r3.Vec{}, err
		}
		c := choice{move: s, free: free, t: t}
		if best == nil || better(c.free, c.t, c.move, best.free, best.t, best.move) {
			cc := c
			best = &cc
		}
	}
	if best == nil {
		return r3.Vec{}, nil
	}
	return best.move.Scale(best.t + gap), nil
}

func shift(b Box, o r3.Vec) Box { return Box{Min: b.Min.Add(o), Max: b.Max.Add(o)} }

// better reports whether choice a beats b: a free sense beats a blocked one,
// then the shorter travel, then the sense whose largest component is positive.
func better(af bool, at float64, am r3.Vec, bf bool, bt float64, bm r3.Vec) bool {
	if af != bf {
		return af
	}
	if math.Abs(at-bt) > eps {
		return at < bt
	}
	return dominantPositive(am) && !dominantPositive(bm)
}

func dominantPositive(v r3.Vec) bool {
	a := [3]float64{v.X, v.Y, v.Z}
	k := 0
	for i := 1; i < 3; i++ {
		if math.Abs(a[i]) > math.Abs(a[k])+eps {
			k = i
		}
	}
	return a[k] > 0
}

// direction returns the unit direction the child moves along and whether the
// opposite sense is allowed too. A zero vector means none.
func direction(boxes []Box, j Joint, parent, child int) (r3.Vec, bool) {
	switch j.Mode {
	case ModeAxis:
		return j.Dir, true
	case ModeAway:
		if child == j.A {
			return j.Dir, false
		}
		return j.Dir.Scale(-1), false
	}
	d := boxes[child].Centre().Sub(boxes[parent].Centre())
	u, ok := d.Normalize()
	if !ok {
		return r3.Vec{}, false
	}
	return u, false
}

// travel is the least distance t >= 0 along unit dir that takes box b clear of
// box p, or +Inf when dir never separates them.
func travel(b, p Box, dir r3.Vec) float64 {
	bl := [3]float64{b.Min.X, b.Min.Y, b.Min.Z}
	bh := [3]float64{b.Max.X, b.Max.Y, b.Max.Z}
	pl := [3]float64{p.Min.X, p.Min.Y, p.Min.Z}
	ph := [3]float64{p.Max.X, p.Max.Y, p.Max.Z}
	d := [3]float64{dir.X, dir.Y, dir.Z}
	t := math.Inf(1)
	for k := range 3 {
		switch {
		case d[k] > eps:
			t = math.Min(t, (ph[k]-bl[k])/d[k])
		case d[k] < -eps:
			t = math.Min(t, (bh[k]-pl[k])/-d[k])
		}
	}
	if t < 0 {
		return 0
	}
	return t
}

// Port returns the point where a joint between boxes a and b attaches when the
// faces say nothing: the centre of their overlap, or the midpoint of the
// shortest segment between them when they do not overlap.
func Port(a, b Box) r3.Vec {
	lo := r3.Vec{X: math.Max(a.Min.X, b.Min.X), Y: math.Max(a.Min.Y, b.Min.Y), Z: math.Max(a.Min.Z, b.Min.Z)}
	hi := r3.Vec{X: math.Min(a.Max.X, b.Max.X), Y: math.Min(a.Max.Y, b.Max.Y), Z: math.Min(a.Max.Z, b.Max.Z)}
	return lo.Add(hi).Scale(0.5)
}

// Meet reports whether two boxes touch or overlap.
func Meet(a, b Box) bool {
	return a.Min.X <= b.Max.X && b.Min.X <= a.Max.X &&
		a.Min.Y <= b.Max.Y && b.Min.Y <= a.Max.Y &&
		a.Min.Z <= b.Max.Z && b.Min.Z <= a.Max.Z
}
