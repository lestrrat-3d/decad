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
