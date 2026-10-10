package box

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// CardinalBasis reports whether the three frame directions are distinct signed axes.
func CardinalBasis(u, v, n r3.Vec) bool {
	axis := func(p r3.Vec) int {
		switch {
		case math.Abs(p.X) == 1 && p.Y == 0 && p.Z == 0:
			return 1
		case p.X == 0 && math.Abs(p.Y) == 1 && p.Z == 0:
			return 2
		case p.X == 0 && p.Y == 0 && math.Abs(p.Z) == 1:
			return 3
		default:
			return 0
		}
	}
	a, b, c := axis(u), axis(v), axis(n)
	return a != 0 && b != 0 && c != 0 && a != b && a != c && b != c
}

// RectangularProfile admits one complete axis-aligned recorded rectangle.
func RectangularProfile(profile momentinput.Profile) bool {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return false
	}
	var corners [4]sectionrecord.Point2
	var ends [4]sectionrecord.Point2
	for i, segment := range profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return false
		}
		switch {
		case line.TStart == 0 && line.TEnd == 1:
			corners[i], ends[i] = line.Start, line.End
		case line.TStart == 1 && line.TEnd == 0:
			corners[i], ends[i] = line.End, line.Start
		default:
			return false
		}
		if corners[i].U == ends[i].U && corners[i].V == ends[i].V {
			return false
		}
		if corners[i].U != ends[i].U && corners[i].V != ends[i].V {
			return false
		}
	}
	minU, maxU := corners[0].U, corners[0].U
	minV, maxV := corners[0].V, corners[0].V
	for _, p := range corners[1:] {
		minU, maxU = math.Min(minU, p.U), math.Max(maxU, p.U)
		minV, maxV = math.Min(minV, p.V), math.Max(maxV, p.V)
	}
	if minU == maxU || minV == maxV {
		return false
	}
	var seen [4]bool
	for i, p := range corners {
		if ends[i] != corners[(i+1)%4] {
			return false
		}
		if (p.U != minU && p.U != maxU) || (p.V != minV && p.V != maxV) {
			return false
		}
		corner := 0
		if p.U == maxU {
			corner += 1
		}
		if p.V == maxV {
			corner += 2
		}
		if seen[corner] {
			return false
		}
		seen[corner] = true
	}
	return true
}
