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

// StackedWallFrame constructs the wall's right-handed signed-permutation
// frame: PlanarFrame on the wall's axis and material-side sign.
func StackedWallFrame(ref r3.Frame, key StackedWallKey) (r3.Frame, Embed, error) {
	frame, e, err := PlanarFrame(ref, key.Axis, key.Sign)
	if err != nil {
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

// StackedWallRegions writes the chained loops of one wall key in its face
// frame and reads their level. Loops that all wind counter-clockwise are one
// face each. Exactly one counter-clockwise loop with every other loop
// clockwise is one face, the clockwise loops its holes in chain order. Any
// other mix, a loop with no signed area, or loops at different levels is
// ErrStackedWallMiss. The grouping proves nothing about nesting: the caller's
// section audit (modify §5's S9) refuses a hole outside its outer loop.
func StackedWallRegions(e Embed, loops [][]StackedWallSegment) ([]Profile, float64, error) {
	level := math.NaN()
	var outers, holes []sectionrecord.LoopRecord
	for li, loop := range loops {
		pts := make([]sectionrecord.Point2, len(loop))
		for i, s := range loop {
			l := e.Local(s.From)
			pts[i] = sectionrecord.Point2{U: l[0], V: l[1]}
			if li == 0 && i == 0 {
				level = l[2]
			} else if l[2] != level {
				return nil, 0, ErrStackedWallMiss
			}
		}
		area := 0.0
		for i := range pts {
			j := (i + 1) % len(pts)
			area += pts[i].U*pts[j].V - pts[j].U*pts[i].V
		}
		var out sectionrecord.LoopRecord
		for i := range pts {
			out.Segments = append(out.Segments, sectionrecord.LineSeg{
				Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1,
			})
		}
		switch {
		case area > 0:
			outers = append(outers, out)
		case area < 0:
			holes = append(holes, out)
		default:
			return nil, 0, ErrStackedWallMiss
		}
	}
	switch {
	case len(holes) == 0:
		out := make([]Profile, len(outers))
		for i, o := range outers {
			out[i] = Profile{Outer: o}
		}
		return out, level, nil
	case len(outers) == 1:
		return []Profile{{Outer: outers[0], Holes: holes}}, level, nil
	}
	return nil, 0, ErrStackedWallMiss
}
