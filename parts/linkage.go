package parts

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
)

// Linkage builds a decad.Linkage from the Set (agent-docs/parts-design.md §7.4)
// rooted at the ground part, and returns the joints it could not place.
//
// Parts joined by Fixed pin, slot and face joints form one link; the ground's
// link is the linkage's ground, whose bodies belong to no link and so stay
// static. Walking the other joints breadth-first from the ground in Join order, a
// Turns joint becomes a revolute about its axis, and a Slides joint a prismatic
// along the joint's axis (a pin in a bore), along the slot (a pin in a slot) or
// along Direction (a face contact). The axis comes from r, a report of Check, and
// otherwise from the joint's declared Axis. Every Contact and Tie joint between
// bodies of different links is declared a joint contact, as is each placed
// pin, slot and face joint; a Keepout is not, since its parts must stay apart.
//
// A joint is returned unplaced, for the caller's Linkage.Close or for repair,
// when it would close a loop, when neither r nor the joint gives the axis or
// slot direction it needs, or when it is a TurnsAndSlides joint: a cylindrical
// joint is a revolute and a prismatic with an intermediate link that carries a
// body, which a part joint cannot supply. r may be nil. A body-less part takes
// no part. A part no placed joint reaches belongs to no link and is static.
func (s *Set) Linkage(ground string, r *JointReport) (*decad.Linkage, []Joint, error) {
	if s == nil || s.doc == nil {
		return nil, nil, fmt.Errorf("parts: %w: nil set", decad.ErrDegenerate)
	}
	gi, ok := s.index[ground]
	if !ok || s.parts[gi].Body == nil {
		return nil, nil, fmt.Errorf("parts: %w: ground %q is unknown or has no body", decad.ErrDegenerate, ground)
	}
	if err := s.live(); err != nil {
		return nil, nil, err
	}
	results := map[string]JointResult{}
	if r != nil {
		for _, jr := range r.Joints {
			results[jr.Joint.Name] = jr
		}
	}

	// K1: parts held by Fixed pin, slot and face joints are one link.
	group := newUnion(len(s.parts))
	for _, j := range s.joints {
		if rigid(j) {
			group.join(s.index[j.A], s.index[j.B])
		}
	}

	// K2: walk the joints that move something, breadth-first from the ground group.
	l := decad.NewLinkage()
	linkOf := map[int]*decad.Link{group.find(gi): l.Ground()}
	placed := map[string]struct{}{}
	var unplaced []Joint
	for progress := true; progress; {
		progress = false
		for _, j := range s.joints {
			if _, done := placed[j.Name]; done || !moves(j) {
				continue
			}
			ga, gb := group.find(s.index[j.A]), group.find(s.index[j.B])
			parentA, hasA := linkOf[ga]
			parentB, hasB := linkOf[gb]
			if hasA == hasB {
				continue // neither end is placed yet, or both are: a loop, settled below
			}
			child, parentLink := ga, parentB
			if hasA {
				child, parentLink = gb, parentA
			}
			link, ok, err := s.placeJoint(parentLink, child, group, j, results[j.Name])
			if err != nil {
				return nil, nil, err
			}
			placed[j.Name] = struct{}{}
			progress = true
			if !ok {
				unplaced = append(unplaced, j)
				continue
			}
			linkOf[child] = link
		}
	}
	// A moving joint left over joins two placed links, or reaches parts the walk never did: it is
	// returned unplaced for the caller.
	for _, j := range s.joints {
		if _, done := placed[j.Name]; !done && moves(j) {
			unplaced = append(unplaced, j)
		}
	}

	// K3: joints between bodies of different links are declared contacts.
	declared := map[[2]*decad.Body]struct{}{}
	for _, j := range s.joints {
		if j.Kind == Keepout {
			continue
		}
		pa, pb := s.parts[s.index[j.A]], s.parts[s.index[j.B]]
		if pa.Body == nil || pb.Body == nil {
			continue
		}
		la, lb := bodyLink(linkOf, l, group.find(s.index[j.A])), bodyLink(linkOf, l, group.find(s.index[j.B]))
		if la == lb {
			continue // one link, or both static
		}
		if _, dup := declared[[2]*decad.Body{pa.Body, pb.Body}]; dup {
			continue
		}
		if _, dup := declared[[2]*decad.Body{pb.Body, pa.Body}]; dup {
			continue
		}
		if err := l.DeclareJointContact(pa.Body, pb.Body); err != nil {
			return nil, nil, fmt.Errorf("parts: joint %q: %w", j.Name, err)
		}
		declared[[2]*decad.Body{pa.Body, pb.Body}] = struct{}{}
	}
	return l, unplaced, nil
}

// bodyLink is the link a group's bodies belong to, nil for the ground group and
// for a group no joint placed: both are static.
func bodyLink(linkOf map[int]*decad.Link, l *decad.Linkage, g int) *decad.Link {
	if link := linkOf[g]; link != l.Ground() {
		return link
	}
	return nil
}

// rigid reports whether a joint holds its two parts as one link.
func rigid(j Joint) bool {
	switch j.Kind {
	case PinInBore, PinInSlot, FaceContact:
		return j.Motion == Fixed
	}
	return false
}

// moves reports whether a joint lets one part move against the other.
func moves(j Joint) bool {
	switch j.Kind {
	case PinInBore, PinInSlot, FaceContact:
		return j.Motion != Fixed
	}
	return false
}

// placeJoint attaches the child group to parentLink by joint j. It reports false
// when the joint cannot be placed.
func (s *Set) placeJoint(parentLink *decad.Link, child int, group *union, j Joint, res JointResult) (*decad.Link, bool, error) {
	var bodies []*decad.Body
	for i, p := range s.parts {
		if p.Body != nil && group.find(i) == child {
			bodies = append(bodies, p.Body)
		}
	}
	axis := res.Axis
	if axis == nil && j.Axis != nil {
		axis = j.Axis
	}
	switch j.Motion {
	case Turns:
		if axis == nil {
			return nil, false, nil
		}
		link, err := parentLink.Revolute(axis.Point, axis.Dir, bodies)
		return link, err == nil, err
	case Slides:
		dir, ok := slideDirection(j, res, axis)
		if !ok {
			return nil, false, nil
		}
		link, err := parentLink.Prismatic(dir, bodies)
		return link, err == nil, err
	}
	return nil, false, nil
}

// slideDirection is a Slides joint's direction: Direction for a face contact,
// the slot's own direction for a pin in a slot, and the axis for a pin in a bore.
func slideDirection(j Joint, res JointResult, axis *Axis) (r3.Vec, bool) {
	switch j.Kind {
	case FaceContact:
		return j.Direction, j.Direction != (r3.Vec{})
	case PinInBore:
		if axis == nil {
			return r3.Vec{}, false
		}
		return axis.Dir, true
	case PinInSlot:
		if axis == nil {
			return r3.Vec{}, false
		}
		if len(res.FacesB) == 0 {
			return axis.Dir, false
		}
		n, err := res.FacesB[0].NormalAt(axis.Point)
		if err != nil {
			return r3.Vec{}, false
		}
		d, ok := axis.Dir.Cross(n.Value).Normalize()
		return d, ok
	}
	return r3.Vec{}, false
}

// union is a small union-find over part indices.
type union struct{ parent []int }

func newUnion(n int) *union {
	u := &union{parent: make([]int, n)}
	for i := range u.parent {
		u.parent[i] = i
	}
	return u
}

func (u *union) find(i int) int {
	for u.parent[i] != i {
		u.parent[i] = u.parent[u.parent[i]]
		i = u.parent[i]
	}
	return i
}

func (u *union) join(a, b int) { u.parent[u.find(b)] = u.find(a) }
