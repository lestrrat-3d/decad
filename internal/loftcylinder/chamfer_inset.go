package loftcylinder

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

type ChamferInsetSamples struct {
	Points              []r3.Vec
	LineStart, ArcStart []int
	Bound               float64
}

// ChamferInset chords the analytic offset of one authenticated held cap
// outline. Each source edge owns one offset line and its following corner arc.
func ChamferInset(ctx context.Context, verts []r3.Vec, ring []int, d float64) (ChamferInsetSamples, error) {
	if len(ring) < 3 {
		return ChamferInsetSamples{}, fmt.Errorf("%w: cap outline has fewer than three vertices", decaderr.ErrUnsupported)
	}
	segs := make([]sectionrecord.CurveSegment, len(ring))
	for i, index := range ring {
		next := ring[(i+1)%len(ring)]
		if index < 0 || next < 0 || index >= len(verts) || next >= len(verts) {
			return ChamferInsetSamples{}, fmt.Errorf("%w: cap outline has an invalid vertex", decaderr.ErrUnsupported)
		}
		a, b := verts[index], verts[next]
		segs[i] = sectionrecord.LineSeg{Start: sectionrecord.Point2{U: a.X, V: a.Y},
			End: sectionrecord.Point2{U: b.X, V: b.Y}, TStart: 0, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	budget := proofbound.NewWorkBudget(ctx)
	inset, err := offset2d.OffsetProfile(budget, profile, 1, d, offset2d.ShellTol)
	if err != nil {
		return ChamferInsetSamples{}, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, profile, inset); err != nil {
		return ChamferInsetSamples{}, err
	}
	delta, err := offset2d.OffsetSectionDelta(budget, profile, 1, d, 0, offset2d.ShellTol)
	if err != nil {
		return ChamferInsetSamples{}, err
	}
	const chord = 1e-4
	samples, err := tessellation.ChordLoop(ctx, inset.Outer, chord, d,
		freeform.NewFreeformWork(), nil, 0,
		func(w survey2d.SideWalk) (int, error) { return w.Segs[0], nil }, stationbound.ChordStationBound)
	if err != nil {
		return ChamferInsetSamples{}, err
	}
	out := ChamferInsetSamples{
		Points:    make([]r3.Vec, len(samples.Samples)),
		LineStart: make([]int, len(ring)), ArcStart: make([]int, len(ring)),
		Bound: proofbound.AbsSumUpper(delta, samples.MaxSag),
	}
	for i := range out.ArcStart {
		out.ArcStart[i] = -1
	}
	for i, p := range samples.Samples {
		out.Points[i] = r3.Vec{X: p.U, Y: p.V}
		if !proofbound.FiniteVec(out.Points[i]) {
			return ChamferInsetSamples{}, fmt.Errorf("%w: cap inset has a nonfinite station", decaderr.ErrUnsupported)
		}
	}
	segmentStart := make([]int, len(inset.Outer.Segments))
	for i := range segmentStart {
		segmentStart[i] = -1
	}
	for i, segment := range samples.FaceOf {
		if segment < 0 || segment >= len(segmentStart) {
			return ChamferInsetSamples{}, fmt.Errorf("%w: inset walk lost its source segment", decaderr.ErrUnsupported)
		}
		if segmentStart[segment] < 0 {
			segmentStart[segment] = i
		}
	}
	segment := 0
	for i := range ring {
		if segment >= len(segmentStart) || segmentStart[segment] < 0 {
			return ChamferInsetSamples{}, fmt.Errorf("%w: inset line lost its source edge", decaderr.ErrUnsupported)
		}
		if _, ok := inset.Outer.Segments[segment].(sectionrecord.LineSeg); !ok {
			return ChamferInsetSamples{}, fmt.Errorf("%w: inset source edge is not a line", decaderr.ErrUnsupported)
		}
		out.LineStart[i] = segmentStart[segment]
		segment++
		if segment < len(segmentStart) {
			if _, ok := inset.Outer.Segments[segment].(sectionrecord.ArcSeg); ok {
				if segmentStart[segment] < 0 {
					return ChamferInsetSamples{}, fmt.Errorf("%w: inset arc lost its source corner", decaderr.ErrUnsupported)
				}
				out.ArcStart[(i+1)%len(ring)] = segmentStart[segment]
				segment++
			}
		}
	}
	if segment != len(segmentStart) {
		return ChamferInsetSamples{}, fmt.Errorf("%w: inset has unmatched offset pieces", decaderr.ErrUnsupported)
	}
	return out, nil
}
