package classbgeom

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ThroughReach finds one or two target faces crossing the tool's section tube.
// Each face must lie across the tool sweep and strictly inside its interval.
// widen is a displacement every face and the tool may carry beyond their
// records: each face box and the tube grow by it on every side, and each
// slab's level band grows by it, so a recorded gap of at most 2·widen
// between a face and the tube reads as a meeting. A face that then meets the
// tube without lying across the sweep makes the reach miss. Class B's own
// boolean passes zero.
func ThroughReach(ctx context.Context, faces []FaceRecord, tool ToolRecord, widen float64) ([]Slab, bool, error) {
	slabs, ok, err := tubeSlabs(ctx, faces, tool, widen, 2)
	if err != nil || !ok {
		return nil, false, err
	}
	lowY := tool.Sign[2]*tool.Z0 + 0
	highY := tool.Sign[2]*tool.Z1 + 0
	if !QualifySlabs(slabs, lowY, highY, tool.Z0Delta, tool.Z1Delta, widen) {
		return nil, false, nil
	}
	return slabs, true, nil
}

// tubeSlabs lists the target faces whose boxes, grown by widen, are not
// strictly apart from the tool's section tube, each read across the tool's
// sweep. ok is false when no face, more than limit faces, or a face not
// across the sweep meets the tube; a limit of zero sets no count.
func tubeSlabs(ctx context.Context, faces []FaceRecord, tool ToolRecord, widen float64, limit int) ([]Slab, bool, error) {
	rat := proofarith.FloatRat
	grow := rat(widen)
	d := tool.Axis[2]
	var section Box2
	for i, seg := range tool.Profile.Outer.Segments {
		b, err := SegmentBox(seg)
		if err != nil {
			return nil, false, err
		}
		if i == 0 {
			section = b
			continue
		}
		section = Union(section, b)
	}
	tube := Place(section, rat(min(tool.Z0, tool.Z1)), rat(max(tool.Z0, tool.Z1)), tool.Axis, tool.Sign).Widened(grow)
	var slabs []Slab
	for fi, f := range faces {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		b, err := f.box()
		if err != nil {
			return nil, false, err
		}
		if b.Widened(grow).Apart(tube) {
			continue
		}
		slab, ok := Across(AcrossFace{
			Planar: f.Region != nil, Wall: f.Wall, Z0: f.Z0, Z0Delta: f.Z0Delta,
			Outward: f.Outward, Axis: f.Axis, Sign: f.Sign,
		}, d)
		if !ok {
			return nil, false, nil
		}
		slab.Face = fi
		slabs = append(slabs, slab)
	}
	if len(slabs) == 0 || (limit > 0 && len(slabs) > limit) {
		return nil, false, nil
	}
	return slabs, true, nil
}

// FlushSide says where the tool lies against the target faces its caps are
// flush with.
type FlushSide int

const (
	// FlushOutside: the tool lies on each face's outward side, a pin standing
	// on one face or bridging the gap between two.
	FlushOutside FlushSide = iota + 1
	// FlushInside: the tool lies inside the target, a blind hole opening in
	// one face or a through hole from one face to the other.
	FlushInside
)

// FlushReach is the through reach with every target face the tube meets lying
// in the plane of a tool cap (agent-docs/general-boolean-design.md §3 "Flush
// cap contact"). flush names the tool caps CoplanarFaces found in a target
// plane. The tube meets one or two faces, and each must sit at a distinct
// marked cap's level, compared as recorded floats, with the face's and that
// cap's level displacements zero, so the face and the cap denote one plane.
// The tube meets no other face's box, so the open tube holds no target
// boundary and the tool lies wholly on one side of the target: outside when
// a face's outward normal points from its cap into the tool, inside
// otherwise. Two faces must say the same side, or the pair misses. ok=false
// is a silent miss.
func FlushReach(ctx context.Context, faces []FaceRecord, tool ToolRecord, flush [2]bool) ([]Slab, FlushSide, bool, error) {
	slabs, ok, err := tubeSlabs(ctx, faces, tool, 0, 2)
	if err != nil || !ok {
		return nil, 0, false, err
	}
	ends := [2]struct{ at, other, delta float64 }{
		{tool.Z0, tool.Z1, tool.Z0Delta},
		{tool.Z1, tool.Z0, tool.Z1Delta},
	}
	var side FlushSide
	var met [2]bool
	for _, s := range slabs {
		k := -1
		for j, e := range ends {
			if flush[j] && !met[j] && s.Level == tool.Sign[2]*e.at+0 {
				k = j
				break
			}
		}
		if k < 0 || s.LevelDelta != 0 || ends[k].delta != 0 {
			return nil, 0, false, nil
		}
		met[k] = true
		level := tool.Sign[2]*ends[k].at + 0
		far := tool.Sign[2]*ends[k].other + 0
		into := 1
		if far < level {
			into = -1
		}
		got := FlushInside
		if s.Outward == into {
			got = FlushOutside
		}
		if side != 0 && got != side {
			return nil, 0, false, nil
		}
		side = got
	}
	return slabs, side, true, nil
}

// Slab names a face across the tool's sweep axis and its exact level test.
type Slab struct {
	Face              int
	Level, LevelDelta float64
	Outward           int
}

// AcrossFace carries the recorded fields needed to classify one face across
// the tool's sweep axis. Axis and Sign place the face in reference coordinates.
type AcrossFace struct {
	Planar      bool
	Wall        sectionrecord.CurveSegment
	Z0, Z0Delta float64
	Outward     bool
	Axis        [3]int
	Sign        [3]float64
}

// Across reads a planar face normal to d or a straight wall at constant d.
func Across(f AcrossFace, d int) (Slab, bool) {
	if f.Planar {
		if f.Axis[2] != d {
			return Slab{}, false
		}
		out := int(f.Sign[2])
		if !f.Outward {
			out = -out
		}
		return Slab{Level: f.Sign[2]*f.Z0 + 0, LevelDelta: f.Z0Delta, Outward: out}, true
	}
	line, ok := f.Wall.(sectionrecord.LineSeg)
	if !ok {
		return Slab{}, false
	}
	start, end := [2]float64{line.Start.U, line.Start.V}, [2]float64{line.End.U, line.End.V}
	for i := range 2 {
		if f.Axis[i] != d || start[i] != end[i] {
			continue
		}
		// The wall runs along local axis 1−i. The material lies on the left
		// of its walk, so the outward normal is its right: (t_v, −t_u).
		t := [2]float64{end[0] - start[0], end[1] - start[1]}
		right := [2]float64{t[1], -t[0]}
		sign := 1
		if right[i]*f.Sign[i] < 0 {
			sign = -1
		}
		return Slab{Level: f.Sign[i]*start[i] + 0, Outward: sign}, true
	}
	return Slab{}, false
}

// QualifySlabs checks one or two faces against the tool's displaced interval,
// each face's level band grown by widen beyond its own displacement. It sorts
// two slabs by level in place. The caller has checked the count.
func QualifySlabs(slabs []Slab, lowY, highY, lowDelta, highDelta, widen float64) bool {
	if len(slabs) == 2 && slabs[1].Level < slabs[0].Level {
		slabs[0], slabs[1] = slabs[1], slabs[0]
	}
	if len(slabs) == 2 && (!(slabs[0].Level < slabs[1].Level) || slabs[0].Outward != -1 || slabs[1].Outward != 1) {
		return false
	}
	if lowY > highY {
		lowY, highY = highY, lowY
		lowDelta, highDelta = highDelta, lowDelta
	}
	rat := proofarith.FloatRat
	b0 := new(big.Rat).Add(rat(lowY), rat(lowDelta))
	b1 := new(big.Rat).Sub(rat(highY), rat(highDelta))
	grow := rat(widen)
	for _, s := range slabs {
		band := new(big.Rat).Add(rat(s.LevelDelta), grow)
		lo := new(big.Rat).Sub(rat(s.Level), band)
		hi := new(big.Rat).Add(rat(s.Level), band)
		if b0.Cmp(lo) >= 0 || hi.Cmp(b1) >= 0 {
			return false
		}
	}
	return true
}
