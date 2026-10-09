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
	if len(slabs) == 0 || len(slabs) > 2 {
		return nil, false, nil
	}
	lowY := tool.Sign[2]*tool.Z0 + 0
	highY := tool.Sign[2]*tool.Z1 + 0
	if !QualifySlabs(slabs, lowY, highY, tool.Z0Delta, tool.Z1Delta, widen) {
		return nil, false, nil
	}
	return slabs, true, nil
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
