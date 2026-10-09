package brepgeom

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// ThroughFace names the face classes in route S's through-cut reading.
type ThroughFace int

const (
	ThroughCap ThroughFace = iota
	ThroughWall
	ThroughHoleWall
	ThroughPierced
	ThroughTool
)

// ThroughToolFace contains the face record fields needed to pair a through
// tool's walls with the hole loops in its lower and upper pierced walls.
type ThroughToolFace struct {
	Region  *momentinput.Profile
	Z0, Z1  float64
	Outward bool
	Role    string
}

// ThroughToolRead is a through tool's paired hole loops and its section in
// an axis frame. The caller builds its prism payload from these records.
type ThroughToolRead struct {
	Axis, LowerFace, UpperFace, LowerLoop, UpperLoop int
	Lo, Hi                                           float64
	Walls                                            []int
	Frame                                            r3.Frame
	FrameEmbed                                       Embed
	Outer                                            sectionrecord.LoopRecord
}

// ReadThroughTools checks TC4 and TC5: each tool wall pairs with one segment
// in each of two pierced walls' hole loops, both loops map to one section,
// and every pierced-wall hole and tool wall belongs to exactly one tool.
// A failed reading returns its first face-specific reason and no error.
func ReadThroughTools(ctx context.Context, topo *Topology, faces []ThroughToolFace,
	embeds []Embed, kinds []ThroughFace, ref r3.Frame) ([]ThroughToolRead, string, error) {
	type holeRef struct{ face, loop int }
	type rimPair struct {
		lower, upper holeRef
		lowerSeg     int
	}
	reason := func(fi int, what string) string { return fmt.Sprintf("%s %s", faces[fi].Role, what) }
	partner := func(ui int) Use {
		pair := topo.Edges[topo.EdgeOf[ui]]
		if pair[0] == ui {
			return topo.Uses[pair[1]]
		}
		return topo.Uses[pair[0]]
	}
	rims := map[int]rimPair{}
	for fi, kind := range kinds {
		if kind != ThroughTool {
			continue
		}
		f, e := faces[fi], embeds[fi]
		var lower, upper []Use
		for _, ui := range topo.FaceUses[fi] {
			u := topo.Uses[ui]
			if !IsRim(u.Part) {
				continue
			}
			o := partner(ui)
			if o.Part != LoopSeg || o.Loop == 0 || kinds[o.Face] != ThroughPierced {
				return nil, reason(fi, "is a swept face across the axis whose rim is no hole segment of a pierced wall"), nil
			}
			atZ0 := u.Part == Rim0
			if (e.Sign[2] > 0) == atZ0 {
				lower = append(lower, o)
			} else {
				upper = append(upper, o)
			}
		}
		if len(lower) != 1 || len(upper) != 1 || f.Z0 == f.Z1 {
			return nil, reason(fi, "is a swept face across the axis that is not one tool wall between two pierced walls"), nil
		}
		rims[fi] = rimPair{
			lower:    holeRef{face: lower[0].Face, loop: lower[0].Loop},
			upper:    holeRef{face: upper[0].Face, loop: upper[0].Loop},
			lowerSeg: lower[0].Seg,
		}
	}
	byLower := map[holeRef][]int{}
	var order []holeRef
	for fi := range kinds {
		r, ok := rims[fi]
		if !ok {
			continue
		}
		if _, seen := byLower[r.lower]; !seen {
			order = append(order, r.lower)
		}
		byLower[r.lower] = append(byLower[r.lower], fi)
	}
	usedHoles := map[holeRef]struct{}{}
	var tools []ThroughToolRead
	for _, lower := range order {
		if err := ctx.Err(); err != nil {
			return nil, "", err
		}
		walls := byLower[lower]
		upper := rims[walls[0]].upper
		w0, w1 := faces[lower.face], faces[upper.face]
		e0, e1 := embeds[lower.face], embeds[upper.face]
		hole := w0.Region.Holes[lower.loop-1]
		hole1 := w1.Region.Holes[upper.loop-1]
		segs := map[int]struct{}{}
		for _, fi := range walls {
			if rims[fi].upper != upper {
				return nil, reason(fi, "is a tool wall that leaves one hole loop for another tool's"), nil
			}
			segs[rims[fi].lowerSeg] = struct{}{}
		}
		if len(segs) != len(hole.Segments) || len(walls) != len(hole.Segments) || len(hole1.Segments) != len(hole.Segments) {
			return nil, reason(lower.face, "holds a hole loop whose segments do not each lead to one tool wall"), nil
		}
		mapped, ok := NewPrismMap(e1, e0).Loop(hole1)
		if !ok || !LoopsEqual(hole, mapped) {
			return nil, reason(upper.face, "holds a hole loop that is not the loop the tool leaves from"), nil
		}
		if outwardSign(w0, e0) > 0 || outwardSign(w1, e1) < 0 {
			return nil, reason(lower.face, "is a pierced wall that the tool does not pass through into the material"), nil
		}
		for _, h := range []holeRef{lower, upper} {
			if _, dup := usedHoles[h]; dup {
				return nil, reason(h.face, "holds a hole loop two tools claim"), nil
			}
			usedHoles[h] = struct{}{}
		}
		j := e0.Axis[2]
		frame, eJ, err := AxisFrame(ref, j)
		if err != nil {
			return nil, reason(lower.face, "is a pierced wall along an axis with no exact frame"), nil //nolint:nilerr // no exact frame is no reading
		}
		section, ok := NewPrismMap(e0, eJ).Loop(hole)
		if !ok {
			return nil, reason(lower.face, "holds a hole loop that does not map into the tool's frame"), nil
		}
		outer, err := offset2d.ReverseLoopRecordContext(ctx, section)
		if err != nil {
			return nil, "", err
		}
		lo, hi := e0.Sign[2]*w0.Z0+0, e1.Sign[2]*w1.Z0+0
		tools = append(tools, ThroughToolRead{
			Axis: j, LowerFace: lower.face, UpperFace: upper.face,
			LowerLoop: lower.loop, UpperLoop: upper.loop,
			Lo: lo, Hi: hi, Walls: walls, Frame: frame, FrameEmbed: eJ, Outer: outer,
		})
	}
	for fi, kind := range kinds {
		if kind != ThroughPierced {
			continue
		}
		for li := range faces[fi].Region.Holes {
			if _, ok := usedHoles[holeRef{face: fi, loop: li + 1}]; !ok {
				return nil, reason(fi, "holds a hole loop no through tool passes"), nil
			}
		}
	}
	return tools, "", nil
}

func outwardSign(f ThroughToolFace, e Embed) float64 {
	if f.Outward {
		return e.Sign[2]
	}
	return -e.Sign[2]
}

// HoleWalkKeys names every section hole segment by the same walk key that
// the prism wall reader uses to identify its face.
func HoleWalkKeys(section momentinput.Profile, work *freeform.FreeformWork) (map[WalkKey]struct{}, error) {
	keys := map[WalkKey]struct{}{}
	for _, hole := range section.Holes {
		for _, seg := range hole.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			keys[WalkKeyOf(w)] = struct{}{}
		}
	}
	return keys, nil
}
