package drawing

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/raster"
)

// MaxHatchLines bounds how many hatch lines Hatch will cross a region with.
const MaxHatchLines = 100000

// Hatch fills the region the loops enclose with parallel lines. loops are closed
// polygons in the drawing plane, in millimetres, the last point joining the first;
// a point is inside the region when a ray from it crosses the loops an odd number
// of times, so a hole inside an outline is left empty whichever way the loops run.
// A line that lies along a boundary edge is drawn when the region lies on its side of
// increasing offset and not when it lies on the other. The lines run at angle radians from the X axis and lie a distance spacing apart,
// at offsets from the origin that are whole multiples of spacing, so two regions
// hatched with the same spacing and angle carry one family of lines. Each returned
// segment is the part of one line inside the region, ordered by line and then
// along it.
//
// spacing MUST be positive and finite, and the region MUST cross at most
// MaxHatchLines lines, otherwise raster.ErrDegenerate.
func Hatch(loops [][][2]float64, spacing, angle float64) ([]Segment, error) {
	if !(spacing > 0) || math.IsInf(spacing, 0) {
		return nil, fmt.Errorf("%w: the hatch spacing must be positive and finite", raster.ErrDegenerate)
	}
	if math.IsNaN(angle) || math.IsInf(angle, 0) {
		return nil, fmt.Errorf("%w: the hatch angle is not finite", raster.ErrDegenerate)
	}
	sin, cos := math.Sincos(angle)
	// along is a point's position along the lines and across its offset from the origin
	// at right angles to them.
	along := func(p [2]float64) float64 { return p[0]*cos + p[1]*sin }
	across := func(p [2]float64) float64 { return -p[0]*sin + p[1]*cos }
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, loop := range loops {
		for _, p := range loop {
			if math.IsNaN(p[0]) || math.IsNaN(p[1]) || math.IsInf(p[0], 0) || math.IsInf(p[1], 0) {
				return nil, fmt.Errorf("%w: a hatch boundary point is not finite", raster.ErrDegenerate)
			}
			t := across(p)
			lo, hi = math.Min(lo, t), math.Max(hi, t)
		}
	}
	if lo > hi {
		return nil, nil
	}
	first, last := math.Ceil(lo/spacing), math.Floor(hi/spacing)
	if last-first+1 > MaxHatchLines {
		return nil, fmt.Errorf("%w: the hatch would need more than %d lines; use a wider spacing", raster.ErrDegenerate, MaxHatchLines)
	}
	var out []Segment
	for k := first; k <= last; k++ {
		t := k * spacing
		var hits []float64
		for _, loop := range loops {
			for i, p := range loop {
				q := loop[(i+1)%len(loop)]
				tp, tq := across(p), across(q)
				// A crossing counts where the line passes from one side of the edge's
				// span to the other, with the lower end included, so a vertex on the line
				// is counted once or not at all.
				if (tp <= t) == (tq <= t) {
					continue
				}
				f := (t - tp) / (tq - tp)
				hits = append(hits, along(p)+f*(along(q)-along(p)))
			}
		}
		sort.Float64s(hits)
		for i := 0; i+1 < len(hits); i += 2 {
			if hits[i+1]-hits[i] <= 1e-12 {
				continue
			}
			out = append(out, Segment{
				A: [2]float64{hits[i]*cos - t*sin, hits[i]*sin + t*cos},
				B: [2]float64{hits[i+1]*cos - t*sin, hits[i+1]*sin + t*cos},
			})
		}
	}
	return out, nil
}
