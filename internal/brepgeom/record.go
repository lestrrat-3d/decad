package brepgeom

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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

// NoSweep is a face's Sweep when it is no wall: a planar cap, floor or
// ceiling.
const NoSweep = -1

// Use is the edge identity and direction needed to pair face uses. A side
// piece's SideDelta holds the level displacements at its two ends, lower
// first. Sweep is the reference axis the use's face sweeps along as a wall —
// a swept face's own, a planar face's recorded one, NoSweep for a cap — and
// Outward is a planar face's outward flag. Record is the recorded segment a
// rim or loop-segment use walks, nil on a side line.
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
	Record          sectionrecord.CurveSegment
	Level           float64
	LevelDelta      float64
	SideDelta       [2]float64
	Sweep           int
	Outward         bool
}

// StartBound is the per-component bound on the distance from DirFrom, the
// held start of a rim's or loop segment's walk, to the point its Record
// denotes there (boundarywalk.DenotedStartBound). It is the walk's own start
// bound except at an arc's natural t = 1 end, which adds the arc's radial
// residual.
func (u Use) StartBound() proofbound.WalkEndBound {
	return boundarywalk.DenotedStartBound(u.Record, u.Walk)
}

// EndBound is StartBound for DirTo, the walk's held end.
func (u Use) EndBound() proofbound.WalkEndBound {
	return boundarywalk.DenotedEndBound(u.Record, u.Walk)
}

// Split is one level strictly inside a swept face's interval at which a side
// line is a vertex of the body, with that level's displacement.
type Split struct {
	Z, ZDelta float64
}

// FaceWalks is one validated face's recorded walks and sweep levels. Side0
// and Side1 are a swept face's side-line splits, ascending. A planar face
// states its outward flag and, when it restates a straight wall as a plane,
// the reference axis that wall sweeps along (NoSweep for a cap); a swept
// face's axis is its own frame normal's, and Build reads it from Embed.
// PlanarSegs and WallSeg are the recorded segments Planar and Wall walk, in
// the same order; Build copies each onto its uses' Record.
type FaceWalks struct {
	Embed            Embed
	Planar           [][]survey2d.SegmentWalk
	PlanarSegs       [][]sectionrecord.CurveSegment
	Wall             survey2d.SegmentWalk
	WallSeg          sectionrecord.CurveSegment
	IsPlanar         bool
	Outward          bool
	Sweep            int
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Side0, Side1     []Split
}

// SideLevels lists a side's levels from Z0 through its splits to Z1, each
// with its displacement.
func (f FaceWalks) SideLevels(side1 bool) []Split {
	splits := f.Side0
	if side1 {
		splits = f.Side1
	}
	out := make([]Split, 0, len(splits)+2)
	out = append(out, Split{Z: f.Z0, ZDelta: f.Z0Delta})
	out = append(out, splits...)
	return append(out, Split{Z: f.Z1, ZDelta: f.Z1Delta})
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
						Level: f.Z0, LevelDelta: f.Z0Delta, Sweep: f.Sweep, Outward: f.Outward}
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
		_ = t0
		rim := func(part Part, z, zDelta float64, from, to, dirFrom, dirTo [3]float64, reversed bool) Use {
			u := Use{Face: fi, Loop: -1, Seg: -1, Part: part, Walk: w, Record: f.WallSeg, Level: z, LevelDelta: zDelta,
				From: from, To: to, DirFrom: dirFrom, DirTo: dirTo, Sweep: e.Axis[2]}
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
		// Each side line is one use per piece between consecutive levels:
		// Side1 walks up the wall's end, Side0 down its start.
		levels1 := f.SideLevels(true)
		for i := 0; i+1 < len(levels1); i++ {
			lo, hi := e.Canon(w.EndU, w.EndV, levels1[i].Z), e.Canon(w.EndU, w.EndV, levels1[i+1].Z)
			add(Use{Face: fi, Loop: -1, Seg: -1, Part: Side1, Key: LineKey(lo, hi), Sweep: e.Axis[2],
				From: lo, To: hi, DirFrom: lo, DirTo: hi, SideDelta: [2]float64{levels1[i].ZDelta, levels1[i+1].ZDelta}})
		}
		add(rim(Rim1, f.Z1, f.Z1Delta, t1, s1, s1, t1, true))
		levels0 := f.SideLevels(false)
		for i := len(levels0) - 1; i > 0; i-- {
			lo, hi := e.Canon(w.StartU, w.StartV, levels0[i-1].Z), e.Canon(w.StartU, w.StartV, levels0[i].Z)
			add(Use{Face: fi, Loop: -1, Seg: -1, Part: Side0, Key: LineKey(lo, hi), Sweep: e.Axis[2],
				From: hi, To: lo, DirFrom: lo, DirTo: hi, SideDelta: [2]float64{levels0[i-1].ZDelta, levels0[i].ZDelta}})
		}
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

// Convex reads an edge's walked-boundary convexity from its owner and mate
// (docs/general-boolean-design.md §4.2).
//
// A rim reads the wall it runs along: a circular wall by its own turn, a
// straight wall by its loop's role, which the planar face sharing the rim
// states — its own role when it walks the rim the wall's way, the other when
// it walks it the opposite way.
//
// A JUNCTION is a line two walls of one sweep share: two swept faces, a swept
// face and a planar face restating a wall along the same axis (a side line),
// or two such planar faces. It is convex when the walk turns left there, the
// material wedge closing to under a half turn. Two swept walls compare their
// tangents. A planar wall reads the same turn from its own loop walk: the
// direction it runs from the line into its face — its frame normal crossed
// with the way its loop walks the line — is convex when it points to the
// inner side of the other wall, whose outward normal is a swept wall's
// tangent crossed with its axis or a planar face's own. Every vector but a
// swept wall's tangent is a signed reference axis, so the sign is exactly the
// tangent component's. A zero turn, a tangent-continuous junction, reads
// concave, as the tangent cross of two swept walls does.
//
// Any other line — a planar cap meeting a planar wall, or two planar faces of
// different sweeps — reads the owner's loop role: outer convex, hole concave.
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
			if other.Sweep != owner.Sweep {
				return other.Loop == 0, nil
			}
			return planarInward(other, embeds).Dot(sweptOutward(owner, embeds, walls)) < 0, nil
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
		if owner.Sweep == NoSweep || other.Sweep != owner.Sweep {
			return owner.Loop == 0, nil
		}
		return planarInward(other, embeds).Dot(planarOutward(owner, embeds)) < 0, nil
	}
}

// sweptOutward is a swept wall's outward normal at the side line a use
// names: the wall's walk tangent there (its end for Side1, its start for
// Side0) crossed with its sweep axis, the material lying on the walk's left.
func sweptOutward(side Use, embeds []Embed, walls map[int]survey2d.SegmentWalk) r3.Vec {
	e, w := embeds[side.Face], walls[side.Face]
	tu, tv := w.TanInU, w.TanInV
	if side.Part == Side1 {
		tu, tv = w.TanOutU, w.TanOutV
	}
	return vec(e.Canon(tu, tv, 0)).Cross(vec(e.Canon(0, 0, 1)))
}

// planarOutward is a planar face's outward normal, a signed reference axis.
func planarOutward(u Use, embeds []Embed) r3.Vec {
	n := vec(embeds[u.Face].Canon(0, 0, 1))
	if !u.Outward {
		return n.Scale(-1)
	}
	return n
}

// planarInward is the direction a planar face runs from one of its lines
// into the face: its frame normal crossed with the way its loop walks the
// line, the material lying on the walk's left in frame coordinates. The walk
// direction is taken by the sign of each coordinate difference, so every
// component is exactly −1, 0 or 1.
func planarInward(u Use, embeds []Embed) r3.Vec {
	var d [3]float64
	for i := range d {
		switch {
		case u.To[i] > u.From[i]:
			d[i] = 1
		case u.To[i] < u.From[i]:
			d[i] = -1
		}
	}
	return vec(embeds[u.Face].Canon(0, 0, 1)).Cross(vec(d))
}

func vec(c [3]float64) r3.Vec { return r3.NewVec(c[0], c[1], c[2]) }
