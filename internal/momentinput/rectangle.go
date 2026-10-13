package momentinput

import "github.com/lestrrat-3d/decad/internal/sectionrecord"

// RectangleBounds accepts only a complete four-line rectangle in its
// source coordinates. Sketch has already certified the source profile.
func RectangleBounds(profile Profile) (float64, float64, float64, float64, bool) {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return 0, 0, 0, 0, false
	}
	first, ok := profile.Outer.Segments[0].(LineSeg)
	if !ok {
		return 0, 0, 0, 0, false
	}
	u0, u1 := first.Start.U, first.Start.U
	v0, v1 := first.Start.V, first.Start.V
	var lines [4]LineSeg
	for i, segment := range profile.Outer.Segments {
		line, ok := segment.(LineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return 0, 0, 0, 0, false
		}
		lines[i] = line
		for _, point := range [...]sectionrecord.Point2{line.Start, line.End} {
			if point.U < u0 {
				u0 = point.U
			}
			if point.U > u1 {
				u1 = point.U
			}
			if point.V < v0 {
				v0 = point.V
			}
			if point.V > v1 {
				v1 = point.V
			}
		}
	}
	if u0 >= u1 || v0 >= v1 {
		return 0, 0, 0, 0, false
	}
	var sides [4]bool
	for _, line := range lines {
		index := -1
		switch {
		case line.Start.V == v0 && line.End.V == v0 &&
			((line.Start.U == u0 && line.End.U == u1) || (line.Start.U == u1 && line.End.U == u0)):
			index = 0
		case line.Start.U == u1 && line.End.U == u1 &&
			((line.Start.V == v0 && line.End.V == v1) || (line.Start.V == v1 && line.End.V == v0)):
			index = 1
		case line.Start.V == v1 && line.End.V == v1 &&
			((line.Start.U == u0 && line.End.U == u1) || (line.Start.U == u1 && line.End.U == u0)):
			index = 2
		case line.Start.U == u0 && line.End.U == u0 &&
			((line.Start.V == v0 && line.End.V == v1) || (line.Start.V == v1 && line.End.V == v0)):
			index = 3
		}
		if index < 0 || sides[index] {
			return 0, 0, 0, 0, false
		}
		sides[index] = true
	}
	return u0, u1, v0, v1, true
}
