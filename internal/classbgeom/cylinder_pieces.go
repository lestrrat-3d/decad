package classbgeom

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// CrossingPieceFace carries a planar face's tagged section into the cylinder
// assembly, after canonical vertices and edge splits have been applied.
type CrossingPieceFace[C comparable] struct {
	Region   momentinput.Profile
	Carriers [][]C
	Frame    CrossingFrame
	Level    float64
	Delta    float64
}

// CylinderPiece is one surviving arc fragment swept between planar faces.
type CylinderPiece struct {
	Frame  r3.Frame
	Wall   sectionrecord.CurveSegment
	Z0, Z1 float64
	Delta  float64
}

// CylinderPieces pairs the planar faces that bound each surviving cylinder
// fragment. An odd level count or a zero-height piece refuses the build.
func CylinderPieces[C comparable](faces []CrossingPieceFace[C], frames [2]CrossingFrame,
	segments [2][]sectionrecord.CurveSegment,
	info func(C) (op, index int, cylinder bool, axis int), miss error) ([]CylinderPiece, error) {
	type fragKey struct {
		carrier  C
		from, to [2]float64
	}
	type frag struct {
		seg    sectionrecord.CurveSegment
		frame  CrossingFrame
		levels []float64
		delta  float64
	}
	frags := map[fragKey]*frag{}
	var order []fragKey
	for _, face := range faces {
		loops := append([]sectionrecord.LoopRecord{face.Region.Outer}, face.Region.Holes...)
		for li, loop := range loops {
			for si, seg := range loop.Segments {
				c := face.Carriers[li][si]
				_, _, cylinder, axis := info(c)
				if !cylinder || face.Frame.Axis[2] != axis {
					continue
				}
				w, err := boundarywalk.WalkOf(seg, nil)
				if err != nil {
					return nil, err
				}
				// Both faces name a fragment by its ends in the cylinder's
				// plane, in counter-clockwise order.
				from, to := [2]float64{w.StartU, w.StartV}, [2]float64{w.EndU, w.EndV}
				if w.Th1 < w.Th0 {
					from, to = to, from
				}
				key := fragKey{carrier: c, from: from, to: to}
				fr, ok := frags[key]
				if !ok {
					fr = &frag{seg: seg, frame: face.Frame}
					frags[key] = fr
					order = append(order, key)
				}
				fr.levels = append(fr.levels, ToX([3]float64{0, 0, face.Level}, face.Frame.Axis, face.Frame.Sign)[axis])
				fr.delta = max(fr.delta, face.Delta)
			}
		}
	}
	var out []CylinderPiece
	for _, key := range order {
		fr := frags[key]
		slices.Sort(fr.levels)
		if len(fr.levels)%2 != 0 {
			return nil, miss
		}
		op, index, _, _ := info(key.carrier)
		own := frames[op]
		wseg, ok := ReframeSegment(fr.seg, fr.frame.Axis, own.Axis, fr.frame.Sign, own.Sign)
		if !ok {
			return nil, miss
		}
		rw, err := boundarywalk.WalkOf(segments[op][index], nil)
		if err != nil {
			return nil, err
		}
		ww, err := boundarywalk.WalkOf(wseg, nil)
		if err != nil {
			return nil, err
		}
		same := (ww.Th1 > ww.Th0) == (rw.Th1 > rw.Th0)
		if (op == 0) != same {
			rev, err := offset2d.ReverseLoopRecord(sectionrecord.LoopRecord{
				Segments: []sectionrecord.CurveSegment{wseg},
			})
			if err != nil {
				return nil, err
			}
			wseg = rev.Segments[0]
		}
		for i := 0; i+1 < len(fr.levels); i += 2 {
			var x0, x1 [3]float64
			x0[own.Axis[2]] = fr.levels[i]
			x1[own.Axis[2]] = fr.levels[i+1]
			z0 := ToLocal(x0, own.Axis, own.Sign)[2]
			z1 := ToLocal(x1, own.Axis, own.Sign)[2]
			if z0 > z1 {
				z0, z1 = z1, z0
			}
			if !(z0 < z1) {
				return nil, miss
			}
			out = append(out, CylinderPiece{Frame: own.Frame, Wall: wseg, Z0: z0, Z1: z1, Delta: fr.delta})
		}
	}
	return out, nil
}
