package brepgeom

import (
	"errors"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// ErrStackedWallMiss marks an uncovered stacked-union wall topology.
var ErrStackedWallMiss = errors.New("the stacked union brep build does not cover this pair")

// StackedWallKey names a planar wall and its material side.
type StackedWallKey struct {
	Axis  int
	Level float64
	Sign  float64
}

// StackedWallSegment is one directed boundary piece in reference coordinates.
type StackedWallSegment struct {
	From, To [3]float64
}

// StackedWallFrame constructs the wall's right-handed signed-permutation frame.
func StackedWallFrame(ref r3.Frame, key StackedWallKey) (r3.Frame, Embed, error) {
	axes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	var e Embed
	switch {
	case key.Axis == 0 && key.Sign > 0:
		e = Embed{Axis: [3]int{1, 2, 0}, Sign: [3]float64{1, 1, 1}}
	case key.Axis == 0:
		e = Embed{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, -1}}
	case key.Sign > 0:
		e = Embed{Axis: [3]int{2, 0, 1}, Sign: [3]float64{1, 1, 1}}
	default:
		e = Embed{Axis: [3]int{0, 2, 1}, Sign: [3]float64{1, 1, -1}}
	}
	var vecs [3]r3.Vec
	for i := range 3 {
		vecs[i] = axes[e.Axis[i]]
		if e.Sign[i] < 0 {
			vecs[i] = vecs[i].Scale(-1)
		}
	}
	frame, err := r3.NewFrame(ref.Origin(), vecs[0], vecs[1])
	if err != nil {
		return r3.Frame{}, Embed{}, ErrStackedWallMiss
	}
	if frame.U() != vecs[0] || frame.V() != vecs[1] || frame.N() != vecs[2] {
		return r3.Frame{}, Embed{}, ErrStackedWallMiss
	}
	return frame, e, nil
}

// ChainStackedWall chains surviving pieces and joins collinear edges where
// their shared point is no body vertex.
func ChainStackedWall(set map[StackedWallSegment]struct{}, levelAt map[float64]int,
	events map[sectionrecord.Point2]map[int]struct{},
) ([][]StackedWallSegment, error) {
	next := map[[3]float64]StackedWallSegment{}
	for s := range set {
		if _, dup := next[s.From]; dup {
			return nil, ErrStackedWallMiss
		}
		next[s.From] = s
	}
	for s := range set {
		if _, ok := next[s.To]; !ok {
			return nil, ErrStackedWallMiss
		}
	}
	used := map[StackedWallSegment]struct{}{}
	var loops [][]StackedWallSegment
	keys := make([]StackedWallSegment, 0, len(set))
	for s := range set {
		keys = append(keys, s)
	}
	slices.SortFunc(keys, func(x, y StackedWallSegment) int {
		for i := range 3 {
			if c := compareStackedWallFloat(x.From[i], y.From[i]); c != 0 {
				return c
			}
		}
		for i := range 3 {
			if c := compareStackedWallFloat(x.To[i], y.To[i]); c != 0 {
				return c
			}
		}
		return 0
	})
	for _, first := range keys {
		if _, done := used[first]; done {
			continue
		}
		var loop []StackedWallSegment
		for s := first; ; s = next[s.To] {
			if _, done := used[s]; done {
				return nil, ErrStackedWallMiss
			}
			used[s] = struct{}{}
			loop = append(loop, s)
			if s.To == first.From {
				break
			}
		}
		loops = append(loops, joinStackedWallCollinear(loop, levelAt, events))
	}
	return loops, nil
}

func compareStackedWallFloat(a, b float64) int {
	switch {
	case a < b:
		return -1
	case a > b:
		return 1
	}
	return 0
}

// joinStackedWallCollinear merges pieces along one axis in one sense.
func joinStackedWallCollinear(loop []StackedWallSegment, levelAt map[float64]int,
	events map[sectionrecord.Point2]map[int]struct{},
) []StackedWallSegment {
	axisOf := func(s StackedWallSegment) (int, float64) {
		for i := range 3 {
			if s.From[i] != s.To[i] {
				return i, s.To[i] - s.From[i]
			}
		}
		return -1, 0
	}
	for changed := true; changed && len(loop) > 1; {
		changed = false
		for i := range loop {
			j := (i + 1) % len(loop)
			a, c := loop[i], loop[j]
			ai, ad := axisOf(a)
			ci, cd := axisOf(c)
			if ai < 0 || ai != ci || (ad > 0) != (cd > 0) {
				continue
			}
			level, ok := levelAt[a.To[2]]
			_, event := events[sectionrecord.Point2{U: a.To[0], V: a.To[1]}][level]
			if !ok || event {
				continue
			}
			merged := StackedWallSegment{From: a.From, To: c.To}
			loop[i] = merged
			loop = slices.Delete(loop, j, j+1)
			changed = true
			break
		}
	}
	return loop
}

// StackedWallRegion writes one wall loop in its face frame and reads its level.
func StackedWallRegion(e Embed, loop []StackedWallSegment) (Profile, float64, error) {
	pts := make([]sectionrecord.Point2, len(loop))
	level := math.NaN()
	for i, s := range loop {
		l := e.Local(s.From)
		pts[i] = sectionrecord.Point2{U: l[0], V: l[1]}
		if i == 0 {
			level = l[2]
		} else if l[2] != level {
			return Profile{}, 0, ErrStackedWallMiss
		}
	}
	area := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		area += pts[i].U*pts[j].V - pts[j].U*pts[i].V
	}
	if !(area > 0) {
		return Profile{}, 0, ErrStackedWallMiss
	}
	var out sectionrecord.LoopRecord
	for i := range pts {
		out.Segments = append(out.Segments, sectionrecord.LineSeg{
			Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1,
		})
	}
	return Profile{Outer: out}, level, nil
}
