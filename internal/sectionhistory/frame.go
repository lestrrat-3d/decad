package sectionhistory

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// permutation names one signed axis permutation of a plane frame: the
// candidate's U is su times (V when swap, else U) and its V is sv times the
// other axis. Every candidate spans the requested plane.
type permutation struct {
	swap   bool
	su, sv float64
}

// permutations lists the eight candidates, identity first, then the
// orientation-preserving ones, then the reflections.
var permutations = []permutation{
	{swap: false, su: 1, sv: 1},
	{swap: false, su: -1, sv: -1},
	{swap: true, su: 1, sv: -1},
	{swap: true, su: -1, sv: 1},
	{swap: false, su: 1, sv: -1},
	{swap: false, su: -1, sv: 1},
	{swap: true, su: 1, sv: 1},
	{swap: true, su: -1, sv: -1},
}

// frame builds the candidate frame at plane's origin. The identity candidate
// is plane itself. Any other is rebuilt by r3.NewFrame, which normalizes and
// may move the held axes of a tilted frame, so ok is false unless its held
// origin, U and V are bit for bit the requested origin and the signed
// permutation of the requested held axes.
func (p permutation) frame(plane r3.Frame) (r3.Frame, bool) {
	if !p.swap && p.su > 0 && p.sv > 0 {
		return plane, true
	}
	a, b := plane.U(), plane.V()
	if p.swap {
		a, b = b, a
	}
	u, v := a.Scale(p.su), b.Scale(p.sv)
	f, err := r3.NewFrame(plane.Origin(), u, v)
	if err != nil || f.Origin() != plane.Origin() || f.U() != u || f.V() != v {
		return r3.Frame{}, false
	}
	return f, true
}

// reflects reports whether the permutation reverses the plane's orientation,
// so a counter-clockwise loop in the candidate frame is clockwise in plane.
func (p permutation) reflects() bool {
	det := p.su * p.sv
	if p.swap {
		det = -det
	}
	return det < 0
}

// point maps a candidate-frame local point to plane's local coordinates. A
// point at u'U' + v'V' is the same world point in plane's held basis, because
// U' and V' are plane's held axes negated or swapped bit for bit. Negation and
// a swap of binary64 values are exact; 0 - x keeps a zero coordinate +0. This
// is a sign and order change of two plane coordinates, which r3's 3D types do
// not express.
func (p permutation) point(q sectionrecord.Point2) sectionrecord.Point2 {
	sign := func(s, x float64) float64 {
		if s < 0 {
			return 0 - x
		}
		return x
	}
	u, v := sign(p.su, q.U), sign(p.sv, q.V)
	if p.swap {
		return sectionrecord.Point2{U: v, V: u}
	}
	return sectionrecord.Point2{U: u, V: v}
}

// lineProfile is a profile whose every loop is forward whole lines.
type lineProfile struct {
	outer []sectionrecord.LineSeg
	holes [][]sectionrecord.LineSeg
}

// mapBack maps a whole-line profile from the candidate frame into plane's
// coordinates. A reflection re-walks every loop backwards, so the outer loop
// stays counter-clockwise and holes clockwise.
func (p permutation) mapBack(profile lineProfile) momentinput.Profile {
	loop := func(in []sectionrecord.LineSeg) sectionrecord.LoopRecord {
		out := make([]sectionrecord.CurveSegment, len(in))
		for i, line := range in {
			start, end := p.point(line.Start), p.point(line.End)
			if p.reflects() {
				start, end = end, start
			}
			out[i] = sectionrecord.LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}
		}
		if p.reflects() {
			slices.Reverse(out)
		}
		return sectionrecord.LoopRecord{Segments: out}
	}
	result := momentinput.Profile{Outer: loop(profile.outer)}
	for _, hole := range profile.holes {
		result.Holes = append(result.Holes, loop(hole))
	}
	return result
}

// wholeLines restates every segment as a forward whole LineSeg. A narrowed
// line enters at a computed walked end, and an arc or circle is not a line;
// either returns ok=false.
func wholeLines(profile momentinput.Profile) (lineProfile, bool) {
	loop := func(in sectionrecord.LoopRecord) ([]sectionrecord.LineSeg, bool) {
		out := make([]sectionrecord.LineSeg, len(in.Segments))
		for i, seg := range in.Segments {
			line, ok := seg.(sectionrecord.LineSeg)
			switch {
			case !ok:
				return nil, false
			case line.TStart == 0 && line.TEnd == 1:
				out[i] = line
			case line.TStart == 1 && line.TEnd == 0:
				out[i] = sectionrecord.LineSeg{Start: line.End, End: line.Start, TStart: 0, TEnd: 1}
			default:
				return nil, false
			}
		}
		return out, true
	}
	outer, ok := loop(profile.Outer)
	if !ok {
		return lineProfile{}, false
	}
	result := lineProfile{outer: outer}
	for _, hole := range profile.Holes {
		h, ok := loop(hole)
		if !ok {
			return lineProfile{}, false
		}
		result.holes = append(result.holes, h)
	}
	return result, true
}
