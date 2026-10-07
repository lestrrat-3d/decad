package brepgeom

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// Embed maps one face's local axes into the first face's reference frame.
// The signed permutation preserves recorded coordinates exactly.
type Embed struct {
	Axis [3]int
	Sign [3]float64
}

// Canon maps a face-local point into reference coordinates. Adding zero
// canonicalizes negative zero so equal points compare as map keys.
func (e Embed) Canon(u, v, z float64) [3]float64 {
	var out [3]float64
	for i, x := range [3]float64{u, v, z} {
		out[e.Axis[i]] = e.Sign[i]*x + 0
	}
	return out
}

// Local is Canon's inverse.
func (e Embed) Local(c [3]float64) [3]float64 {
	var out [3]float64
	for i := range out {
		out[i] = e.Sign[i]*c[e.Axis[i]] + 0
	}
	return out
}

// Embeds requires every frame to share the first origin and to carry a
// handed signed permutation of its axes. unsupported is the caller's sentinel.
func Embeds(frames []r3.Frame, unsupported error) ([]Embed, error) {
	ref := frames[0]
	refAxes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	out := make([]Embed, len(frames))
	for fi, frame := range frames {
		if frame.Origin() != ref.Origin() {
			return nil, fmt.Errorf(`%w: brep face %d's frame does not share the reference origin`, unsupported, fi)
		}
		var e Embed
		used := [3]bool{}
		for i, a := range [3]r3.Vec{frame.U(), frame.V(), frame.N()} {
			found := false
			for j, b := range refAxes {
				switch {
				case a == b:
					e.Axis[i], e.Sign[i], found = j, 1, true
				case a == b.Scale(-1):
					e.Axis[i], e.Sign[i], found = j, -1, true
				}
				if found {
					break
				}
			}
			if !found || used[e.Axis[i]] {
				return nil, fmt.Errorf(`%w: brep face %d's frame is not a signed permutation of the reference frame`, unsupported, fi)
			}
			used[e.Axis[i]] = true
		}
		if determinant(e) < 0 {
			return nil, fmt.Errorf(`%w: brep face %d's frame reverses the reference frame's handedness`, unsupported, fi)
		}
		out[fi] = e
	}
	return out, nil
}

func determinant(e Embed) float64 {
	det := e.Sign[0] * e.Sign[1] * e.Sign[2]
	inversions := 0
	for i := range 3 {
		for j := i + 1; j < 3; j++ {
			if e.Axis[i] > e.Axis[j] {
				inversions++
			}
		}
	}
	if inversions%2 == 1 {
		det = -det
	}
	return det
}

// Part names the boundary piece whose edge use is recorded.
type Part int

const (
	LoopSeg Part = iota
	Rim0
	Rim1
	Side0
	Side1
)

// EdgeKey identifies a line by sorted endpoints, an arc by its centre,
// axis and directed ends, and a whole circle by its centre, axis and radius.
type EdgeKey struct {
	Circular, Closed bool
	A, B, C          [3]float64
	Axis             int
	Radius           float64
}

// LineKey sorts the endpoints by their first distinct reference coordinate.
func LineKey(a, b [3]float64) EdgeKey {
	for i := range 3 {
		if a[i] != b[i] {
			if a[i] > b[i] {
				a, b = b, a
			}
			break
		}
	}
	return EdgeKey{A: a, B: b}
}

// CurveKey keys a walk at one level and reports its counterclockwise sense
// about the reference axis. The frame's signed map keeps handedness.
func CurveKey(e Embed, w survey2d.SegmentWalk, z float64) (EdgeKey, bool) {
	from, to := e.Canon(w.StartU, w.StartV, z), e.Canon(w.EndU, w.EndV, z)
	if !w.IsCircular() {
		return LineKey(from, to), false
	}
	ccw := (w.Th1 > w.Th0) == (e.Sign[2] > 0)
	key := EdgeKey{Circular: true, C: e.Canon(w.CU, w.CV, z), Axis: e.Axis[2]}
	if w.Closed {
		key.Closed, key.Radius = true, w.Radius
		return key, ccw
	}
	if !ccw {
		from, to = to, from
	}
	key.A, key.B = from, to
	return key, ccw
}

// Use is the edge identity and direction needed to pair face uses.
type Use struct {
	Face            int
	Loop            int
	Seg             int
	Part            Part
	Key             EdgeKey
	From, To        [3]float64
	DirFrom, DirTo  [3]float64
	Sense, DirSense bool
	Walk            survey2d.SegmentWalk
	Level           float64
	LevelDelta      float64
}

// FaceWalks is one validated face's recorded walks and sweep levels.
type FaceWalks struct {
	Embed            Embed
	Planar           [][]survey2d.SegmentWalk
	Wall             survey2d.SegmentWalk
	IsPlanar         bool
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
}

// Topology holds each face's uses in walk order and paired edge indices.
type Topology struct {
	Walls      map[int]survey2d.SegmentWalk
	Planar     map[int][][]survey2d.SegmentWalk
	Uses       []Use
	Edges      [][2]int
	EdgeOf     []int
	FaceUses   [][]int
	CoordUpper float64
}

// Build records every face's uses, then pairs them by exact edge identity.
// Every face's walks must already have passed the caller's record audit.
func Build(faces []FaceWalks, unsupported error) (*Topology, error) {
	topo := &Topology{
		Walls:    map[int]survey2d.SegmentWalk{},
		Planar:   map[int][][]survey2d.SegmentWalk{},
		FaceUses: make([][]int, len(faces)),
	}
	add := func(u Use) {
		topo.FaceUses[u.Face] = append(topo.FaceUses[u.Face], len(topo.Uses))
		topo.Uses = append(topo.Uses, u)
	}
	for fi, f := range faces {
		e := f.Embed
		topo.CoordUpper = math.Max(topo.CoordUpper, math.Max(math.Abs(f.Z0), math.Abs(f.Z1)))
		if f.IsPlanar {
			for li, loop := range f.Planar {
				for si, w := range loop {
					topo.CoordUpper = math.Max(topo.CoordUpper, w.CoordUpper)
					u := Use{Face: fi, Loop: li, Seg: si, Part: LoopSeg, Walk: w,
						Level: f.Z0, LevelDelta: f.Z0Delta}
					u.Key, u.Sense = CurveKey(e, w, f.Z0)
					u.DirSense = u.Sense
					u.From, u.To = e.Canon(w.StartU, w.StartV, f.Z0), e.Canon(w.EndU, w.EndV, f.Z0)
					u.DirFrom, u.DirTo = u.From, u.To
					add(u)
				}
			}
			topo.Planar[fi] = f.Planar
			continue
		}
		w := f.Wall
		topo.CoordUpper = math.Max(topo.CoordUpper, w.CoordUpper)
		topo.Walls[fi] = w
		s0, s1 := e.Canon(w.StartU, w.StartV, f.Z0), e.Canon(w.StartU, w.StartV, f.Z1)
		t0, t1 := e.Canon(w.EndU, w.EndV, f.Z0), e.Canon(w.EndU, w.EndV, f.Z1)
		rim := func(part Part, z, zDelta float64, from, to, dirFrom, dirTo [3]float64, reversed bool) Use {
			u := Use{Face: fi, Loop: -1, Seg: -1, Part: part, Walk: w, Level: z, LevelDelta: zDelta,
				From: from, To: to, DirFrom: dirFrom, DirTo: dirTo}
			u.Key, u.Sense = CurveKey(e, w, z)
			u.DirSense = u.Sense
			if reversed {
				u.Sense = !u.Sense
			}
			return u
		}
		add(rim(Rim0, f.Z0, f.Z0Delta, s0, t0, s0, t0, false))
		if w.Closed {
			add(rim(Rim1, f.Z1, f.Z1Delta, s1, s1, s1, s1, true))
			continue
		}
		add(Use{Face: fi, Loop: -1, Seg: -1, Part: Side1, Key: LineKey(t0, t1),
			From: t0, To: t1, DirFrom: t0, DirTo: t1})
		add(rim(Rim1, f.Z1, f.Z1Delta, t1, s1, s1, t1, true))
		add(Use{Face: fi, Loop: -1, Seg: -1, Part: Side0, Key: LineKey(s0, s1),
			From: s1, To: s0, DirFrom: s0, DirTo: s1})
	}
	var err error
	topo.Edges, topo.EdgeOf, err = Pair(topo.Uses, unsupported)
	if err != nil {
		return nil, err
	}
	return topo, nil
}

// Pair requires each edge to bound exactly two distinct faces. The first use
// is the owner whose natural direction and geometry define the edge.
func Pair(uses []Use, unsupported error) ([][2]int, []int, error) {
	byKey := map[EdgeKey][]int{}
	var order []EdgeKey
	for ui, u := range uses {
		if _, seen := byKey[u.Key]; !seen {
			order = append(order, u.Key)
		}
		byKey[u.Key] = append(byKey[u.Key], ui)
	}
	edgeOf := make([]int, len(uses))
	var edges [][2]int
	for _, key := range order {
		pair := byKey[key]
		if len(pair) != 2 || uses[pair[0]].Face == uses[pair[1]].Face {
			return nil, nil, fmt.Errorf(`%w: a brep edge must bound exactly two distinct faces; one bounds %d face uses`, unsupported, len(pair))
		}
		a, b := pair[0], pair[1]
		if ownerRank(uses[b].Part) < ownerRank(uses[a].Part) {
			a, b = b, a
		}
		if key.Circular && uses[a].Part == LoopSeg {
			return nil, nil, fmt.Errorf(`%w: a circular brep edge must bound a swept face`, unsupported)
		}
		if IsRim(uses[a].Part) && uses[b].Part != LoopSeg {
			return nil, nil, fmt.Errorf(`%w: a swept face's rim must meet a planar face`, unsupported)
		}
		edgeOf[a], edgeOf[b] = len(edges), len(edges)
		edges = append(edges, [2]int{a, b})
	}
	return edges, edgeOf, nil
}

// Forward reports whether use ui walks its owner's natural direction.
func Forward(uses []Use, edges [][2]int, edgeOf []int, ui int) bool {
	u := uses[ui]
	owner := uses[edges[edgeOf[ui]][0]]
	if u.Key.Closed {
		return u.Sense == owner.DirSense
	}
	return u.From == owner.DirFrom && u.To == owner.DirTo
}

// IsRim reports whether a part is either swept-face rim.
func IsRim(p Part) bool { return p == Rim0 || p == Rim1 }

func ownerRank(p Part) int {
	switch p {
	case Rim0, Rim1:
		return 0
	case Side0, Side1:
		return 1
	default:
		return 2
	}
}

// Convex reads an edge's walked-boundary convexity from its owner and mate.
// A swept-wall junction compares its two tangents in reference coordinates.
func Convex(pair [2]int, uses []Use, embeds []Embed, walls map[int]survey2d.SegmentWalk,
	unsupported error) (bool, error) {
	owner, other := uses[pair[0]], uses[pair[1]]
	switch owner.Part {
	case Rim0, Rim1:
		if owner.Walk.IsCircular() {
			return owner.Walk.Th1 >= owner.Walk.Th0, nil
		}
		hole := other.Loop != 0
		same := other.DirFrom == owner.DirFrom && other.DirTo == owner.DirTo
		return !hole == same, nil
	case Side0, Side1:
		if other.Part != Side0 && other.Part != Side1 {
			return other.Loop == 0, nil
		}
		prev, next := owner, other
		if prev.Part == Side0 {
			prev, next = next, prev
		}
		if prev.Part != Side1 || next.Part != Side0 {
			return false, fmt.Errorf(`%w: two swept faces meet at a junction both walk the same way`, unsupported)
		}
		pe, ne := embeds[prev.Face], embeds[next.Face]
		if pe.Axis[2] != ne.Axis[2] {
			return false, fmt.Errorf(`%w: two swept faces meeting at a junction sweep along different axes`, unsupported)
		}
		pw, nw := walls[prev.Face], walls[next.Face]
		tOut := (vec(pe.Canon(pw.TanOutU, pw.TanOutV, 0)))
		tIn := (vec(ne.Canon(nw.TanInU, nw.TanInV, 0)))
		axis := (vec(pe.Canon(0, 0, 1)))
		return tOut.Cross(tIn).Dot(axis) > 0, nil
	default:
		return owner.Loop == 0, nil
	}
}

func vec(c [3]float64) r3.Vec { return r3.NewVec(c[0], c[1], c[2]) }
