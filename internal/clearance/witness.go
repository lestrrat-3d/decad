package clearance

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// Witness is a float point the coarse enclosure (CellSink.Coarse) reads for
// an upper bound, beside Gap, a proven upper bound on its distance from a
// point of the trimmed face or edge it samples.
//
// A witness is a float sample: math.Sincos values and frame lifts place it a
// few roundings off the carrier, and a lift whose terms cancel (a profile
// drawn far from its axis, a body built far from the origin and placed back)
// rounds at the scale of its terms, not of the point it returns. So the
// distance between two witnesses bounds the pair's gap only once each
// witness's own distance from its face is added. Gap is read exactly, never
// estimated from the coordinates: the same proven enclosure the vertex cells
// read for a point against the carrier (Height, SpineDist, ConeDist,
// PointLineDist, PointCircleDist), charged DirCharge for a direction that is
// not a signed coordinate axis, and taken only when the point's foot on the
// carrier is admitted by the trim at the kernel's own margin, as every cell's
// upper bound is. A witness whose foot the trim does not admit with margin —
// a sample on the trim's own boundary, where the foot can fall a rounding
// outside — has no gap and is never read.
type Witness struct {
	P   r3.Vec
	Gap float64
}

// Witnesses returns f's witnesses (f.Wit) that carry a proven gap, each with
// it, admitting their feet at margin.
func (f *CFace) Witnesses(margin float64) []Witness {
	out := make([]Witness, 0, len(f.Wit))
	for _, w := range f.Wit {
		if gap := f.WitnessGap(w, margin); !math.IsInf(gap, 1) {
			out = append(out, Witness{P: w, Gap: gap})
		}
	}
	return out
}

// WitnessGap returns a proven upper bound on the distance from w to a point of
// f's trimmed face, or +Inf when it cannot state one. The bound is the upper
// end of the exact distance from w to the carrier (Witness), and it stands
// only when f.AdmitPoint (or, for a plane, the region's classification)
// admits the carrier foot of w at margin: the foot is then a point of the
// trimmed face.
func (f *CFace) WitnessGap(w r3.Vec, margin float64) float64 {
	switch f.Kind {
	case CkPlane:
		h := w.Sub(f.O).Dot(f.N)
		x, y := f.PlaneCoords(w.Sub(f.N.Scale(h)))
		if f.Region.Classify(x, y, margin) != 1 {
			return math.Inf(1)
		}
		return Height(w, f.O, f.N).Abs().Widen(DirCharge([]r3.Vec{f.N}, []r3.Vec{w, f.O})).Hi
	case CkCone:
		rel := w.Sub(f.Anchor)
		z := rel.Dot(f.Axis)
		perp := rel.Sub(f.Axis.Scale(z))
		radial, ok := perp.Normalize()
		if !ok {
			return math.Inf(1)
		}
		_, t := f.ConeMeridian(z, perp.Len())
		if t <= margin {
			return math.Inf(1)
		}
		sinA, cosA := f.ConeSinCos()
		foot := f.Anchor.Add(f.Axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
		if f.AdmitPoint(foot, margin) != 1 {
			return math.Inf(1)
		}
		return f.ConeDist(w).Widen(DirCharge([]r3.Vec{f.Axis}, []r3.Vec{w, f.Anchor})).Hi
	default:
		d, spineFoot := SpineDistOf(f, w)
		if d <= margin {
			return math.Inf(1)
		}
		foot := spineFoot.Add(w.Sub(spineFoot).Scale(f.Radius / d))
		if f.AdmitPoint(foot, margin) != 1 {
			return math.Inf(1)
		}
		return SpineDist(f, w).Sub(f.Radius).Abs().Hi
	}
}

// Witnesses returns on-edge sample points with their proven gaps: a
// segment's two ends, which are the segment's own ends and so carry no gap,
// and its midpoint; an arc's start and mid-angle points, a whole circle's
// points at angles 0 and π. A point other than a segment end carries the
// exact distance from it to the edge's line or circle (PointLineDist,
// PointCircleDist with DirCharge), and is kept only when the edge's own
// parameter window admits it at margin.
func (e *CEdge) Witnesses(margin float64) []Witness {
	if e.Line {
		out := []Witness{{P: e.A}, {P: e.B}}
		mid := e.A.Add(e.B).Scale(0.5)
		seg, ok := SegDir(e.A, e.B)
		if ok && LineParamAdmit(e, mid, margin) == 1 {
			if gap := PointLineDist(mid, e.A, seg).Hi; !math.IsInf(gap, 1) {
				out = append(out, Witness{P: mid, Gap: gap})
			}
		}
		return out
	}
	lo, hi := 0.0, 2*math.Pi
	if !e.Ang.Full {
		lo, hi = e.Ang.Lo, e.Ang.Hi
	}
	var out []Witness
	for _, th := range []float64{lo, (lo + hi) / 2} {
		p := e.At(th)
		if CircleAngleAdmit(e, AngleOf(e, p.Sub(e.Center)), margin) != 1 {
			continue
		}
		gap := PointCircleDist(p, e.Center, e.Axis, e.Radius, 1).
			Widen(DirCharge([]r3.Vec{e.Axis}, []r3.Vec{p, e.Center}, e.Radius)).Hi
		if !math.IsInf(gap, 1) {
			out = append(out, Witness{P: p, Gap: gap})
		}
	}
	return out
}
