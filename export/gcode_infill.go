package export

import (
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// infillSegment is one extruding line of a layer's infill, from a to b.
type infillSegment struct{ a, b r3.Vec }

// rectilinearInfill returns the lines that fill the region the loops of a layer's outline enclose
// (agent-docs/gcode-design.md G7): parallel to the X axis when alongX, else to the Y axis, a line every spacing
// across the loops' extent starting half a spacing in, each cut where it crosses a loop, an outer loop and a hole
// alike, so that it runs through the material between crossings in pairs. Each piece is drawn shorter by inset at
// both ends, so it does not pile onto the perimeter, and one no longer than that is dropped. Alternate lines run
// the opposite way, so the nozzle zigzags. Points carry the loops' own X and Y and a zero Z.
func rectilinearInfill(loops [][]r3.Vec, alongX bool, spacing, inset float64) []infillSegment {
	// Read every loop with the scan axis as x, so one routine serves both directions.
	scan := func(p r3.Vec) (float64, float64) {
		if alongX {
			return p.X, p.Y
		}
		return p.Y, p.X
	}
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, loop := range loops {
		for _, p := range loop {
			_, across := scan(p)
			lo, hi = math.Min(lo, across), math.Max(hi, across)
		}
	}
	var out []infillSegment
	flip := false
	for k := 0; ; k++ {
		c := lo + (float64(k)+0.5)*spacing
		if !(c < hi) {
			break
		}
		var crossings []float64
		for _, loop := range loops {
			for i := range loop {
				ax, ay := scan(loop[i])
				bx, by := scan(loop[(i+1)%len(loop)])
				// A half-open interval counts a vertex on the line once on the side it continues to.
				if (ay <= c && c < by) || (by <= c && c < ay) {
					crossings = append(crossings, ax+(c-ay)/(by-ay)*(bx-ax))
				}
			}
		}
		if len(crossings)%2 != 0 {
			continue // an open or self-crossing outline has no material to read here
		}
		sort.Float64s(crossings)
		var row []infillSegment
		for i := 0; i+1 < len(crossings); i += 2 {
			x0, x1 := crossings[i]+inset, crossings[i+1]-inset
			if !(x1 > x0) {
				continue
			}
			seg := infillSegment{a: r3.Vec{X: x0, Y: c}, b: r3.Vec{X: x1, Y: c}}
			if !alongX {
				seg = infillSegment{a: r3.Vec{X: c, Y: x0}, b: r3.Vec{X: c, Y: x1}}
			}
			row = append(row, seg)
		}
		if flip {
			for i, j := 0, len(row)-1; i < j; i, j = i+1, j-1 {
				row[i], row[j] = row[j], row[i]
			}
			for i := range row {
				row[i].a, row[i].b = row[i].b, row[i].a
			}
		}
		out = append(out, row...)
		flip = !flip
	}
	return out
}

// insetLoops moves every loop of a layer's outline d towards the material (agent-docs/gcode-design.md G10): the outer
// loops inward and the loops of holes outward, each corner at the meeting of its two moved sides, so that a line
// traced along the result lays its edge, and not its middle, on the outline. A loop is wound with the material on its
// left, an outer loop counter-clockwise and a hole clockwise, decided by how many other loops enclose it. A corner
// sharper than about 170 degrees is cut off at d from the corner in place of reaching the meeting of its sides, and a
// repeated point or a loop of fewer than three points is left as it is.
func insetLoops(loops [][]r3.Vec, d float64) [][]r3.Vec {
	out := make([][]r3.Vec, len(loops))
	for i, loop := range loops {
		if len(loop) < 3 {
			out[i] = loop
			continue
		}
		depth := 0
		for j, other := range loops {
			if j != i && pointInLoop(loop[0], other) {
				depth++
			}
		}
		wound := loop
		if ccw := signedArea(loop) > 0; ccw != (depth%2 == 0) {
			wound = make([]r3.Vec, len(loop))
			for k, p := range loop {
				wound[len(loop)-1-k] = p
			}
		}
		out[i] = insetWound(wound, d)
	}
	return out
}

// outerLoops returns the loops of a layer's outline that no other loop encloses an odd number of times: the outlines
// of the material and not of its holes, or of an island inside a hole, which is material again and kept.
func outerLoops(loops [][]r3.Vec) [][]r3.Vec {
	var out [][]r3.Vec
	for i, loop := range loops {
		if len(loop) < 3 {
			continue
		}
		depth := 0
		for j, other := range loops {
			if j != i && pointInLoop(loop[0], other) {
				depth++
			}
		}
		if depth%2 == 0 {
			out = append(out, loop)
		}
	}
	return out
}

// signedArea is the shoelace area of a loop in the XY plane, positive when it runs counter-clockwise.
func signedArea(loop []r3.Vec) float64 {
	area := 0.0
	for i, p := range loop {
		q := loop[(i+1)%len(loop)]
		area += p.X*q.Y - q.X*p.Y
	}
	return area / 2
}

// pointInLoop reports whether p lies inside the loop, by the even-odd rule.
func pointInLoop(p r3.Vec, loop []r3.Vec) bool {
	inside := false
	for i, a := range loop {
		b := loop[(i+1)%len(loop)]
		if (a.Y <= p.Y) != (b.Y <= p.Y) && p.X < a.X+(p.Y-a.Y)/(b.Y-a.Y)*(b.X-a.X) {
			inside = !inside
		}
	}
	return inside
}

// insetWound moves a loop with its material on its left d to the left.
func insetWound(loop []r3.Vec, d float64) []r3.Vec {
	n := len(loop)
	// The unit left normal of the side from loop[i] to loop[i+1], zero for a repeated point.
	normal := make([]r3.Vec, n)
	for i := range loop {
		side := loop[(i+1)%n].Sub(loop[i])
		if length := math.Hypot(side.X, side.Y); length > 0 {
			normal[i] = r3.Vec{X: -side.Y / length, Y: side.X / length}
		}
	}
	out := make([]r3.Vec, n)
	for i, p := range loop {
		n1, n2 := normal[(i+n-1)%n], normal[i]
		switch {
		case n1 == (r3.Vec{}):
			n1 = n2
		case n2 == (r3.Vec{}):
			n2 = n1
		}
		// The corner of the two moved sides is p + (n1 + n2)·d/(1 + n1·n2); a sharp corner is cut off.
		denominator := 1 + n1.Dot(n2)
		if denominator < 0.0152 { // the sides turn back by more than 170 degrees: 1 + cos 170°
			out[i] = p.Add(n1.Add(n2).Scale(0.5 * d))
			continue
		}
		out[i] = p.Add(n1.Add(n2).Scale(d / denominator))
	}
	return out
}
