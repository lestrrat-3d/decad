package classbgeom

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// FaceRecord is a face's section and levels in its own signed reference axes.
type FaceRecord struct {
	Region           *momentinput.Profile
	Wall             sectionrecord.CurveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Outward          bool
	Axis             [3]int
	Sign             [3]float64
}

// ToolRecord is the admitted prism's section and sweep in reference axes.
type ToolRecord struct {
	Profile          momentinput.Profile
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Axis             [3]int
	Sign             [3]float64
}

func (f FaceRecord) box() (Box3, error) {
	segs := []sectionrecord.CurveSegment{f.Wall}
	if f.Region != nil {
		segs = f.Region.Outer.Segments
	}
	return FaceBox(segs, f.Z0, f.Z1, f.Z0Delta, f.Z1Delta, f.Axis, f.Sign)
}

func (f FaceRecord) loops() []sectionrecord.LoopRecord {
	if f.Region != nil {
		return append([]sectionrecord.LoopRecord{f.Region.Outer}, f.Region.Holes...)
	}
	return []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{f.Wall}}}
}

// NaturalPair checks that every segment uses its natural recorded range.
func NaturalPair(faces []FaceRecord, tool momentinput.Profile) bool {
	if !NaturalRecord(append([]sectionrecord.LoopRecord{tool.Outer}, tool.Holes...)) {
		return false
	}
	for _, f := range faces {
		if !NaturalRecord(f.loops()) {
			return false
		}
	}
	return true
}

// AxisAlignedPair checks that every straight wall follows a local coordinate axis.
func AxisAlignedPair(faces []FaceRecord, tool momentinput.Profile) bool {
	if !AxisAligned(append([]sectionrecord.LoopRecord{tool.Outer}, tool.Holes...)) {
		return false
	}
	for _, f := range faces {
		if !AxisAligned(f.loops()) {
			return false
		}
	}
	return true
}

// CurvedApart is B7: each curved wall pair has outward boxes separated on an
// axis. widen is a displacement every face and the tool may carry beyond
// their records: both boxes of a pair grow by it on every side before the
// comparison. Class B's own boolean passes zero.
func CurvedApart(ctx context.Context, faces []FaceRecord, tool ToolRecord, widen float64) (bool, error) {
	grow := proofarith.FloatRat(widen)
	var xs []Box3
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if f.Region != nil {
			continue
		}
		if _, line := f.Wall.(sectionrecord.LineSeg); line {
			continue
		}
		b, err := f.box()
		if err != nil {
			return false, err
		}
		xs = append(xs, b.Widened(grow))
	}
	var ys []Box3
	rat := proofarith.FloatRat
	lz, hz := rat(min(tool.Z0, tool.Z1)), rat(max(tool.Z0, tool.Z1))
	for _, seg := range tool.Profile.Outer.Segments {
		if _, line := seg.(sectionrecord.LineSeg); line {
			continue
		}
		b, err := SegmentBox(seg)
		if err != nil {
			return false, err
		}
		ys = append(ys, Place(b, lz, hz, tool.Axis, tool.Sign).Widened(grow))
	}
	for _, a := range xs {
		for _, b := range ys {
			if !a.Apart(b) {
				return false, nil
			}
		}
	}
	return true, nil
}

// CoplanarFaces is B8 with its flush lift (agent-docs/general-boolean-design.md
// §3 "Flush cap contact"). It compares every target cap and wall plane with
// every tool cap and wall plane by exact equality of their recorded axis and
// level. ok is false when a target plane is a tool wall's plane, and, with
// allowFlush false, when a target plane is a tool cap's plane: B8 itself.
// With allowFlush true a target plane may be a tool cap's plane, and flush
// names each tool cap (Z0, Z1) that is. A shared plane admits nothing by
// itself: FlushReach still has to find the one face the tool meets there.
func CoplanarFaces(ctx context.Context, faces []FaceRecord, tool ToolRecord, allowFlush bool) ([2]bool, bool, error) {
	var xs []Plane
	for _, f := range faces {
		if f.Region != nil {
			xs = append(xs, Plane{Axis: f.Axis[2], Level: proofarith.FloatRat(f.Sign[2]*f.Z0 + 0)})
			continue
		}
		xs = append(xs, RecordPlanes(f.loops(), f.Axis, f.Sign)...)
	}
	walls := RecordPlanes(append([]sectionrecord.LoopRecord{tool.Profile.Outer}, tool.Profile.Holes...),
		tool.Axis, tool.Sign)
	caps := [2]Plane{
		{Axis: tool.Axis[2], Level: proofarith.FloatRat(tool.Sign[2]*tool.Z0 + 0)},
		{Axis: tool.Axis[2], Level: proofarith.FloatRat(tool.Sign[2]*tool.Z1 + 0)},
	}
	same := func(a, b Plane) bool { return a.Axis == b.Axis && a.Level.Cmp(b.Level) == 0 }
	var flush [2]bool
	for _, a := range xs {
		if err := ctx.Err(); err != nil {
			return [2]bool{}, false, err
		}
		for _, b := range walls {
			if same(a, b) {
				return [2]bool{}, false, nil
			}
		}
		for k, b := range caps {
			if same(a, b) {
				flush[k] = true
			}
		}
	}
	if !allowFlush && (flush[0] || flush[1]) {
		return [2]bool{}, false, nil
	}
	return flush, true, nil
}
