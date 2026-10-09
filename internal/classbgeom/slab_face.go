package classbgeom

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// SlabFaceInput is the face record needed to restate a slab in the tool frame.
type SlabFaceInput struct {
	Region           *momentinput.Profile
	Wall             sectionrecord.CurveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Delta            float64
	Sweep, Normal    r3.Vec
}

// SlabFace is one slab face rebuilt in the tool frame, before its section is cut.
type SlabFace struct {
	Region     momentinput.Profile
	Level      float64
	LevelDelta float64
	Delta      float64
	Outward    bool
	Sweep      r3.Vec
}

// RestateSlabFace maps a slab face through the signed frame permutation.
// A straight wall becomes its swept rectangle; a planar face keeps its region
// and sweep. Reflected planar loops are rewound to preserve their orientation.
func RestateSlabFace(budget *proofbound.WorkBudget, f SlabFaceInput, e brepgeom.Embed,
	axis [3]int, sign [3]float64, s Slab) (SlabFace, error) {
	toG := func(u, v, z float64) [3]float64 {
		c := e.Canon(u, v, z)
		var out [3]float64
		for i := range out {
			out[i] = sign[i]*c[axis[i]] + 0
		}
		return out
	}
	out := SlabFace{
		Level: sign[2]*s.Level + 0, LevelDelta: s.LevelDelta,
		Outward: float64(s.Outward)*sign[2] > 0, Sweep: f.Sweep,
	}
	if f.Region == nil {
		out.Sweep = f.Normal
		line, ok := f.Wall.(sectionrecord.LineSeg)
		if !ok {
			return SlabFace{}, fmt.Errorf(`%w: a slab wall of the through-nesting reach is not a line`, decaderr.ErrDegenerate)
		}
		var corners []sectionrecord.Point2
		for _, c := range [][3]float64{
			{line.Start.U, line.Start.V, f.Z0}, {line.End.U, line.End.V, f.Z0},
			{line.End.U, line.End.V, f.Z1}, {line.Start.U, line.Start.V, f.Z1},
		} {
			p := toG(c[0], c[1], c[2])
			corners = append(corners, sectionrecord.Point2{U: p[0], V: p[1]})
		}
		out.Region = momentinput.Profile{Outer: rectLoop(corners)}
		out.Delta = max(f.Z0Delta, f.Z1Delta)
		return out, nil
	}
	origin, pu, pv := toG(0, 0, f.Z0), toG(1, 0, f.Z0), toG(0, 1, f.Z0)
	det := (pu[0]-origin[0])*(pv[1]-origin[1]) - (pu[1]-origin[1])*(pv[0]-origin[0])
	mapPoint := func(p sectionrecord.Point2) (sectionrecord.Point2, error) { //nolint:unparam // RewindLoop's map may fail.
		q := toG(p.U, p.V, f.Z0)
		return sectionrecord.Point2{U: q[0], V: q[1]}, nil
	}
	mapLoop := func(loop sectionrecord.LoopRecord) (sectionrecord.LoopRecord, error) {
		if det < 0 {
			rewound, _, err := prismcells.RewindLoop(budget, loop, mapPoint)
			return rewound, err
		}
		segs := make([]sectionrecord.CurveSegment, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return sectionrecord.LoopRecord{}, err
			}
			switch sg := seg.(type) {
			case sectionrecord.LineSeg:
				sg.Start, _ = mapPoint(sg.Start)
				sg.End, _ = mapPoint(sg.End)
				segs[i] = sg
			case sectionrecord.ArcSeg:
				sg.Center, _ = mapPoint(sg.Center)
				sg.Start, _ = mapPoint(sg.Start)
				sg.End, _ = mapPoint(sg.End)
				segs[i] = sg
			case sectionrecord.CircleSeg:
				sg.Center, _ = mapPoint(sg.Center)
				segs[i] = sg
			default:
				return sectionrecord.LoopRecord{}, fmt.Errorf(`%w: a %T segment has no class-B face map`,
					decaderr.ErrUnsupported, seg)
			}
		}
		return sectionrecord.LoopRecord{Segments: segs}, nil
	}
	outer, err := mapLoop(f.Region.Outer)
	if err != nil {
		return SlabFace{}, err
	}
	out.Region = momentinput.Profile{Outer: outer}
	for _, hole := range f.Region.Holes {
		mapped, err := mapLoop(hole)
		if err != nil {
			return SlabFace{}, err
		}
		out.Region.Holes = append(out.Region.Holes, mapped)
	}
	out.Delta = f.Delta
	return out, nil
}

// rectLoop is the counter-clockwise loop through four corners given in either winding.
func rectLoop(c []sectionrecord.Point2) sectionrecord.LoopRecord {
	area := 0.0
	for i := range c {
		j := (i + 1) % len(c)
		area += c[i].U*c[j].V - c[j].U*c[i].V
	}
	if area < 0 {
		c[1], c[3] = c[3], c[1]
	}
	var loop sectionrecord.LoopRecord
	for i := range c {
		loop.Segments = append(loop.Segments, sectionrecord.LineSeg{
			Start: c[i], End: c[(i+1)%len(c)], TStart: 0, TEnd: 1,
		})
	}
	return loop
}
