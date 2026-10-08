package classbgeom

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CanonicalFaceHooks supplies the crossing table and vertex registry shared
// by every face of the same class-B build.
type CanonicalFaceHooks[C comparable] struct {
	Cylinder func(C) bool
	Point    func(f, c1, c2 C, at [3]float64, delta float64) ([3]float64, float64, error)
	Record   func(at [3]float64, carriers []C, delta float64)
}

// CanonicalizeFace merges circular seam fragments and pins every remaining
// junction to the build's canonical crossing vertex.
func CanonicalizeFace[C comparable](region momentinput.Profile, carriers [][]C,
	face C, frame CrossingFrame, level, delta float64,
	hooks CanonicalFaceHooks[C]) (momentinput.Profile, [][]C, float64, error) {
	loops := append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...)
	for li := range loops {
		segs, row := mergeCrossingSeams(loops[li].Segments, carriers[li], hooks.Cylinder)
		n := len(segs)
		if n == 1 {
			loops[li] = sectionrecord.LoopRecord{Segments: segs}
			carriers[li] = row
			continue
		}
		walks := make([]survey2d.SegmentWalk, n)
		for i, seg := range segs {
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return momentinput.Profile{}, nil, 0, err
			}
			walks[i] = w
		}
		ends := make([][3]float64, n)
		for i := range segs {
			j := (i + 1) % n
			// A line keeps its fixed coordinate exact, so its end takes
			// precedence over the adjacent arc's scene point.
			pick, wk := segs[i], walks[i]
			u, v := wk.EndU, wk.EndV
			bound := boundarywalk.DenotedEndBound(segs[i], wk)
			if _, line := segs[i].(sectionrecord.LineSeg); !line {
				if _, nextLine := segs[j].(sectionrecord.LineSeg); nextLine {
					pick, wk = segs[j], walks[j]
					u, v, bound = wk.StartU, wk.StartV, boundarywalk.DenotedStartBound(segs[j], wk)
				}
			}
			at := ToX([3]float64{u, v, level}, frame.Axis, frame.Sign)
			charge := proofbound.AbsSumUpper(proofbound.CutDisplacementAllow(CarrierSpeed(pick)),
				proofbound.WalkEndBoundAllow(bound))
			point, pointDelta, err := hooks.Point(face, row[i], row[j], at, charge)
			if err != nil {
				return momentinput.Profile{}, nil, 0, err
			}
			hooks.Record(point, []C{face, row[i], row[j]}, pointDelta)
			ends[i] = point
			delta = max(delta, pointDelta)
		}
		out := make([]sectionrecord.CurveSegment, n)
		for i := range segs {
			from, to := ends[(i+n-1)%n], ends[i]
			out[i] = BetweenVertices(walks[i], frame.Axis, frame.Sign, from, to)
		}
		loops[li] = sectionrecord.LoopRecord{Segments: out}
		carriers[li] = row
	}
	return momentinput.Profile{Outer: loops[0], Holes: loops[1:]}, carriers, delta, nil
}

func mergeCrossingSeams[C comparable](segs []sectionrecord.CurveSegment, carriers []C,
	cylinder func(C) bool) ([]sectionrecord.CurveSegment, []C) {
	n := len(segs)
	if n < 2 {
		return segs, carriers
	}
	outS, outC := []sectionrecord.CurveSegment{}, []C{}
	for i := range n {
		if i > 0 && carriers[i] == outC[len(outC)-1] && cylinder(carriers[i]) {
			outS[len(outS)-1] = JoinCircleFragments(outS[len(outS)-1], segs[i])
			continue
		}
		outS, outC = append(outS, segs[i]), append(outC, carriers[i])
	}
	if len(outS) > 1 && outC[0] == outC[len(outC)-1] && cylinder(outC[0]) {
		outS[0] = JoinCircleFragments(outS[len(outS)-1], outS[0])
		outS, outC = outS[:len(outS)-1], outC[:len(outC)-1]
	}
	return outS, outC
}

// SplitCrossingFace cuts each face edge at every canonical vertex on it.
func SplitCrossingFace[C comparable, V interface{ Carriers() []C }](region momentinput.Profile,
	carriers [][]C, face C, frame CrossingFrame, level float64, verts map[[3]float64]V,
	geometry func(C) (cylinder bool, axis int), miss error) (momentinput.Profile, [][]C, error) {
	loops := append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...)
	for li, loop := range loops {
		var segs []sectionrecord.CurveSegment
		var row []C
		for si, seg := range loop.Segments {
			c := carriers[li][si]
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return momentinput.Profile{}, nil, err
			}
			cylinder, axis := geometry(c)
			pieces, err := SplitSegment(w, seg, frame.Axis, frame.Sign, level,
				face, c, cylinder, axis, verts, miss)
			if err != nil {
				return momentinput.Profile{}, nil, err
			}
			for _, piece := range pieces {
				segs = append(segs, piece)
				row = append(row, c)
			}
		}
		loops[li] = sectionrecord.LoopRecord{Segments: segs}
		carriers[li] = row
	}
	return momentinput.Profile{Outer: loops[0], Holes: loops[1:]}, carriers, nil
}

// CarrierSpeed bounds a recorded segment's speed for a cut-point charge.
func CarrierSpeed(seg sectionrecord.CurveSegment) float64 {
	speed, err := prismcells.CarrierSpeedUpper(seg)
	if err != nil {
		return math.Inf(1)
	}
	return speed
}
