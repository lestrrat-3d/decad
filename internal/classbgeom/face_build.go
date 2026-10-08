package classbgeom

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// CrossingPrism is the section and sweep interval a crossing scene reads.
type CrossingPrism struct {
	Segments []sectionrecord.CurveSegment
	Z0, Z1   float64
}

// CrossingFrame maps frame-local coordinates to the reference axes.
type CrossingFrame struct {
	Frame r3.Frame
	Axis  [3]int
	Sign  [3]float64
}

// CrossingChord is one section trace's interval and its end carriers.
type CrossingChord[C comparable] struct {
	Lo, Hi   [3]float64
	CLo, CHi C
}

// CrossingFaceHooks asks the root evaluator for scene answers while the
// crossing builder constructs the face inputs from admitted records.
type CrossingFaceHooks[C comparable] struct {
	Carrier  func(op, index int) C
	Geometry func(C) (axis int, level float64)
	Chords   func(context.Context, int, C) ([]CrossingChord[C], error)
	Decide   func(context.Context, C, int, float64, bool, []CarrierSegment[C], [][]CarrierSegment[C], bool) error
	Miss     error
}

// SectionSegments tags an operand's section edges with their wall carriers.
func SectionSegments[C comparable](op int, segments []sectionrecord.CurveSegment,
	carrier func(int, int) C) []CarrierSegment[C] {
	out := make([]CarrierSegment[C], len(segments))
	for i, seg := range segments {
		out[i] = CarrierSegment[C]{Seg: seg, Carrier: carrier(op, i)}
	}
	return out
}

// BuildXCrossingFaces decides X's planar caps and straight walls against Y.
// Frame indices are X, Y, and the face frame normal to X's other section axis.
func BuildXCrossingFaces[C comparable](ctx context.Context, x, y CrossingPrism, e int,
	frames [3]CrossingFrame, hooks CrossingFaceHooks[C]) error {
	segs := x.Segments
	n := len(segs)
	capCorners := SectionSegments(0, segs, hooks.Carrier)
	for _, end := range []int{-1, -2} {
		f := hooks.Carrier(0, end)
		level := x.Z0
		if end == -2 {
			level = x.Z1
		}
		chords, err := hooks.Chords(ctx, 1, f)
		if err != nil {
			return err
		}
		other, err := ChordRectangles(frames[0], chords, 1, hooks.Carrier, hooks.Geometry, hooks.Miss)
		if err != nil {
			return err
		}
		if err := hooks.Decide(ctx, f, 0, level, end == -2, capCorners, other, true); err != nil {
			return err
		}
	}
	for i, seg := range segs {
		line, ok := seg.(sectionrecord.LineSeg)
		if !ok {
			continue
		}
		f := hooks.Carrier(0, i)
		axis, level := hooks.Geometry(f)
		prev, next := hooks.Carrier(0, (i+n-1)%n), hooks.Carrier(0, (i+1)%n)
		start := [3]float64{line.Start.U, line.Start.V, 0}
		end := [3]float64{line.End.U, line.End.V, 0}
		corners := [][3]float64{
			{start[0], start[1], x.Z0}, {end[0], end[1], x.Z0},
			{end[0], end[1], x.Z1}, {start[0], start[1], x.Z1},
		}
		carriers := []C{hooks.Carrier(0, -1), next, hooks.Carrier(0, -2), prev}
		outward := lineOutward(line)
		if axis == e {
			own := RectLoop(frames[2].Axis, frames[2].Sign, corners, carriers)
			chords, err := hooks.Chords(ctx, 1, f)
			if err != nil {
				return err
			}
			other, err := ChordRectangles(frames[2], chords, 1, hooks.Carrier, hooks.Geometry, hooks.Miss)
			if err != nil {
				return err
			}
			at := ToLocal(corners[0], frames[2].Axis, frames[2].Sign)[2]
			if err := hooks.Decide(ctx, f, 2, at, outwardIn(frames[2], outward), own, other, true); err != nil {
				return err
			}
			continue
		}
		own := RectLoop(frames[1].Axis, frames[1].Sign, corners, carriers)
		var other [][]CarrierSegment[C]
		if strictlyInside(level, y, frames[1]) {
			other = [][]CarrierSegment[C]{SectionSegments(1, y.Segments, hooks.Carrier)}
		}
		at := ToLocal(corners[0], frames[1].Axis, frames[1].Sign)[2]
		if err := hooks.Decide(ctx, f, 1, at, outwardIn(frames[1], outward), own, other, true); err != nil {
			return err
		}
	}
	return nil
}

// BuildYCrossingFaces decides Y's planar caps and straight walls against X.
// Y contributes the material-facing side of each surviving face.
func BuildYCrossingFaces[C comparable](ctx context.Context, x, y CrossingPrism, e int,
	frames [3]CrossingFrame, hooks CrossingFaceHooks[C]) error {
	segs := y.Segments
	n := len(segs)
	for _, end := range []int{-1, -2} {
		f := hooks.Carrier(1, end)
		level := y.Z0
		if end == -2 {
			level = y.Z1
		}
		chords, err := hooks.Chords(ctx, 0, f)
		if err != nil {
			return err
		}
		if len(chords) == 0 {
			continue
		}
		other, err := ChordRectangles(frames[1], chords, 0, hooks.Carrier, hooks.Geometry, hooks.Miss)
		if err != nil {
			return err
		}
		if err := hooks.Decide(ctx, f, 1, level, end == -1,
			SectionSegments(1, segs, hooks.Carrier), other, false); err != nil {
			return err
		}
	}
	for i, seg := range segs {
		line, ok := seg.(sectionrecord.LineSeg)
		if !ok {
			continue
		}
		f := hooks.Carrier(1, i)
		axis, level := hooks.Geometry(f)
		prev, next := hooks.Carrier(1, (i+n-1)%n), hooks.Carrier(1, (i+1)%n)
		at := func(p sectionrecord.Point2, z float64) [3]float64 {
			return ToX([3]float64{p.U, p.V, z}, frames[1].Axis, frames[1].Sign)
		}
		corners := [][3]float64{at(line.Start, y.Z0), at(line.End, y.Z0), at(line.End, y.Z1), at(line.Start, y.Z1)}
		carriers := []C{hooks.Carrier(1, -1), next, hooks.Carrier(1, -2), prev}
		inward := ToX(lineOutward(line), frames[1].Axis, frames[1].Sign)
		for k := range inward {
			inward[k] = -inward[k] + 0
		}
		if axis == e {
			own := RectLoop(frames[2].Axis, frames[2].Sign, corners, carriers)
			chords, err := hooks.Chords(ctx, 0, f)
			if err != nil {
				return err
			}
			if len(chords) == 0 {
				continue
			}
			other, err := ChordRectangles(frames[2], chords, 0, hooks.Carrier, hooks.Geometry, hooks.Miss)
			if err != nil {
				return err
			}
			at := ToLocal(corners[0], frames[2].Axis, frames[2].Sign)[2]
			if err := hooks.Decide(ctx, f, 2, at, outwardIn(frames[2], inward), own, other, false); err != nil {
				return err
			}
			continue
		}
		if !strictlyInside(level, x, frames[0]) {
			continue
		}
		own := RectLoop(frames[0].Axis, frames[0].Sign, corners, carriers)
		other := [][]CarrierSegment[C]{SectionSegments(0, x.Segments, hooks.Carrier)}
		if err := hooks.Decide(ctx, f, 0, level, outwardIn(frames[0], inward), own, other, false); err != nil {
			return err
		}
	}
	return nil
}

func lineOutward(line sectionrecord.LineSeg) [3]float64 {
	du, dv := line.End.U-line.Start.U, line.End.V-line.Start.V
	return [3]float64{dv, -du, 0}
}

func outwardIn(frame CrossingFrame, dir [3]float64) bool {
	return ToLocal(dir, frame.Axis, frame.Sign)[2] > 0
}

func strictlyInside(level float64, op CrossingPrism, frame CrossingFrame) bool {
	lo := frame.Sign[2]*op.Z0 + 0
	hi := frame.Sign[2]*op.Z1 + 0
	if lo > hi {
		lo, hi = hi, lo
	}
	return lo < level && level < hi
}

// ChordRectangles sweeps each section trace chord between the operand caps.
// A trace with more than one chord is outside this crossing reach.
func ChordRectangles[C comparable](frame CrossingFrame, chords []CrossingChord[C], op int,
	carrier func(int, int) C, geometry func(C) (int, float64), miss error) ([][]CarrierSegment[C], error) {
	if len(chords) > 1 {
		return nil, miss
	}
	withAxial := func(x [3]float64, capCarrier C) [3]float64 {
		axis, level := geometry(capCarrier)
		x[axis] = level
		return x
	}
	var out [][]CarrierSegment[C]
	for _, chord := range chords {
		lowCap, highCap := carrier(op, -1), carrier(op, -2)
		corners := [][3]float64{
			withAxial(chord.Lo, lowCap), withAxial(chord.Hi, lowCap),
			withAxial(chord.Hi, highCap), withAxial(chord.Lo, highCap),
		}
		segs := RectLoop(frame.Axis, frame.Sign, corners, []C{lowCap, chord.CHi, highCap, chord.CLo})
		out = append(out, segs)
	}
	return out, nil
}

// TraceCrossingChords asks sketch for the trace of one carrier through an
// operand section, then pins each chord end to the shared crossing table.
func TraceCrossingChords[C comparable](ctx context.Context, op int, face C,
	segments []sectionrecord.CurveSegment, frame CrossingFrame,
	carrier func(int, int) C, geometry func(C) (int, float64),
	canonical func(f, c1, c2 C, at [3]float64, delta float64) ([3]float64, error),
	miss error) ([]CrossingChord[C], error) {
	axis, level := geometry(face)
	var fixedLocal int
	for i := range 2 {
		if frame.Axis[i] == axis {
			fixedLocal = i
		}
	}
	fixedValue := frame.Sign[fixedLocal] * level
	traces, trace, err := TraceChords(ctx, SectionSegments(op, segments, carrier), fixedLocal, fixedValue, face, miss)
	if err != nil {
		return nil, err
	}
	caps := []C{carrier(op, -1), carrier(op, -2)}
	var out []CrossingChord[C]
	for _, traced := range traces {
		var chord CrossingChord[C]
		for k := range 2 {
			end := traced.Ends[k]
			x := ToX([3]float64{end.U, end.V, 0}, frame.Axis, frame.Sign)
			delta := proofbound.AbsSumUpper(proofbound.CutDisplacementAllow(CarrierSpeed(trace)),
				proofbound.WalkEndBoundAllow(traced.Bounds[k]))
			var point [3]float64
			for _, cap := range caps {
				axial, capLevel := geometry(cap)
				at := x
				at[axial] = capLevel
				point, err = canonical(face, traced.Crossed[k], cap, at, delta)
				if err != nil {
					return nil, err
				}
			}
			if k == 0 {
				chord.Lo, chord.CLo = point, traced.Crossed[k]
			} else {
				chord.Hi, chord.CHi = point, traced.Crossed[k]
			}
		}
		out = append(out, chord)
	}
	return out, nil
}
