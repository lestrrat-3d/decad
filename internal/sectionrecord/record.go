package sectionrecord

import (
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CloneLoopRecord copies a loop and its segment records.
func CloneLoopRecord(loop LoopRecord) LoopRecord { return cloneLoopRecord(loop) }

// This file defines the evaluator's structural, plane-local profile records.
// They hold no live sketch profile or frame: decad converts the source geometry
// into values before evaluation.

// PlaneRecord is the sketch plane as three vectors. It is orthonormal and
// right-handed; the plane normal is
// U × V, and that normal is the sense Direction.Along means for a linear
// extent. Origin is a position in millimetres (docs/api-design.md §5.2); U
// and V are the in-plane axes the (u, v) of a Point2 is expressed in.
type PlaneRecord struct {
	Origin r3.Vec `json:"origin"`
	U      r3.Vec `json:"u"`
	V      r3.Vec `json:"v"`
}

// Point2 is a plane-local coordinate, a length in millimetres — the
// docs/api-design.md §5.2 carve-out in the plane's own (u, v).
type Point2 struct {
	U float64 `json:"u"`
	V float64 `json:"v"`
}

// LoopRecord is a closed, directed boundary loop: each segment's walk — from
// its point at TStart to its point at TEnd — ends where the next segment's
// walk starts, and the last closes onto the first. A single closed segment —
// a circle, an ellipse, a closed spline — is a loop on its own. Outer loops
// run counter-clockwise in (u, v), holes clockwise.
type LoopRecord struct {
	Segments []CurveSegment
}

// ChainRecord is momentinput.Profile's OPEN counterpart: one directed walk whose
// first segment's walk start and last segment's walk end are FREE — they
// meet nothing, and nothing closes onto them. It carries no Holes and no
// walk-level winding, because an open walk bounds no region and so has no
// inside. See docs/sketch-seam-design.md §2.2 and docs/surface-design.md §13.
type ChainRecord struct {
	Segments []CurveSegment
}

func cloneLoopRecord(l LoopRecord) LoopRecord {
	if l.Segments == nil {
		return LoopRecord{}
	}
	out := LoopRecord{Segments: make([]CurveSegment, len(l.Segments))}
	for i, segment := range l.Segments {
		out.Segments[i] = cloneSegment(segment)
	}
	return out
}

func cloneSegment(segment CurveSegment) CurveSegment {
	normalized, err := normalizeSegment(segment)
	if err != nil {
		return segment
	}
	switch segment := normalized.(type) {
	case SplineSeg:
		segment.Control = slices.Clone(segment.Control)
		return segment
	case NURBSSeg:
		segment.Control = slices.Clone(segment.Control)
		segment.Knots = slices.Clone(segment.Knots)
		segment.Weights = slices.Clone(segment.Weights)
		return segment
	case ClosedSplineSeg:
		segment.Control = slices.Clone(segment.Control)
		return segment
	case FitSplineSeg:
		segment.Fit = slices.Clone(segment.Fit)
		return segment
	default:
		return normalized
	}
}

// CurveSegment is one curve of a loop, recorded structurally — never as a
// sample. Sealed, like Surface. A variant records exactly the defining data
// of the curve the edge IS — the fields of the source entity's own geom
// value, verbatim, in plane-local Point2 — plus the recorded range:
// TStart/TEnd, sketch's normalized t on the entity, the full domain for a
// whole edge and the certified range for a Partial fragment. What geom
// DERIVES from those fields — an arc's radius and angles, an elliptical arc's
// eccentric parameters — is never recorded in their place. One variant serves
// each entity kind, whole and trimmed alike: the entity picks the variant,
// and only the range differs.
//
// A walk against the curve's natural sense is baked into the segment as the
// order of its range — TStart > TEnd says the segment runs backwards, and a
// closed kind's CCW flips with it. The entity's fields are never reordered.
type CurveSegment interface{ curveSegment() }

// The five analytic kinds. Every variant's TStart/TEnd — like a spline's
// knots and weights — is a curve parameter, not a quantity
// (docs/api-design.md §5.2).

// LineSeg mirrors geom.Line: the endpoints, verbatim.
type LineSeg struct {
	Start  Point2  `json:"start"`
	End    Point2  `json:"end"`
	TStart float64 `json:"t_start"` // the full domain for a whole edge
	TEnd   float64 `json:"t_end"`
}

// CircleSeg mirrors geom.Circle: the center and the radius. A closed analytic
// kind — a whole edge is a LoopRecord on its own — so, like ClosedSplineSeg,
// it carries the walk's winding in (u, v) as CCW alongside the range.
type CircleSeg struct {
	Center Point2      `json:"center"`
	Radius units.Value `json:"radius"`
	CCW    bool        `json:"ccw"`
	TStart float64     `json:"t_start"` // the full period for a whole edge
	TEnd   float64     `json:"t_end"`
}

// ArcSeg mirrors geom.Arc: three pinned points, the arc swept
// counter-clockwise from Start to End about Center. The sweep is the entity's
// own definition, so no field restates it; radius and angles are geom's
// derived readings, never fields.
type ArcSeg struct {
	Center Point2  `json:"center"`
	Start  Point2  `json:"start"`
	End    Point2  `json:"end"`
	TStart float64 `json:"t_start"` // the full domain for a whole edge
	TEnd   float64 `json:"t_end"`
}

// EllipseSeg is sketch's ellipse. Rx and Ry are the semi-axes along the
// ellipse's own local x and y, and they are UNORDERED — geom.Ellipse does not
// enforce Rx >= Ry; the axes are simply the local x and y, and Rotation is
// the angle of that local frame.
type EllipseSeg struct {
	Center   Point2      `json:"center"`
	Rx       units.Value `json:"rx"`
	Ry       units.Value `json:"ry"`
	Rotation units.Value `json:"rotation"`
	CCW      bool        `json:"ccw"`
	TStart   float64     `json:"t_start"` // the full period for a whole edge
	TEnd     float64     `json:"t_end"`
}

// EllipticalArcSeg mirrors geom.EllipticalArc: the ellipse (Center, Rx, Ry,
// Rotation — unordered, as EllipseSeg) restricted to the counter-clockwise
// eccentric-angle sweep from Start to End. Start and End are the entity's
// PINNED points, verbatim — they lie on the parametric ellipse only within
// solver tolerance — so no eccentric-angle pair can stand in for them.
type EllipticalArcSeg struct {
	Center   Point2      `json:"center"`
	Start    Point2      `json:"start"`
	End      Point2      `json:"end"`
	Rx       units.Value `json:"rx"`
	Ry       units.Value `json:"ry"`
	Rotation units.Value `json:"rotation"`
	TStart   float64     `json:"t_start"` // the full domain for a whole edge
	TEnd     float64     `json:"t_end"`
}

// The five free-form kinds. Degree, knots and weights are curve parameters on
// the same terms as every range; a conic's fullness Rho — from which a
// rational quadratic's apex weight derives as w = Rho/(1−Rho) — is of exactly
// the same class as a NURBS weight.

// SplineSeg mirrors geom.Spline: an open cubic B-spline over its control
// points. Degree 3, the clamped uniform knot vector and unit weights are the
// entity's DEFINITION, not stored data — geom.Spline's one field is Control —
// so the record carries none of them: a Degree, Knots or Weights field here
// would hold values the entity does not, synthesized, which the verbatim rule
// forbids.
type SplineSeg struct {
	Control []Point2 `json:"control"`
	TStart  float64  `json:"t_start"`
	TEnd    float64  `json:"t_end"`
}

// NURBSSeg mirrors geom.NURBS: a clamped B-spline of arbitrary degree with a
// non-decreasing knot vector and a per-control weight — every field the
// entity holds, verbatim, and nothing it derives.
type NURBSSeg struct {
	Degree  int       `json:"degree"`
	Control []Point2  `json:"control"`
	Knots   []float64 `json:"knots"`
	Weights []float64 `json:"weights"`
	TStart  float64   `json:"t_start"`
	TEnd    float64   `json:"t_end"`
}

// ClosedSplineSeg is sketch's periodic uniform cubic B-spline: a closed curve
// that bounds a region on its own, so it is a whole LoopRecord by itself.
type ClosedSplineSeg struct {
	Control []Point2 `json:"control"`
	CCW     bool     `json:"ccw"`
	TStart  float64  `json:"t_start"` // the full period for a whole edge
	TEnd    float64  `json:"t_end"`
}

// FitSplineSeg records the INTENT sketch was given: the points the curve
// interpolates. sketch's definition — a natural cubic with chord-length
// parameterisation through exactly these points — is the curve; decad records
// the points and NEVER runs the interpolation solve itself. Where a
// measurement needs the solve's RESULT, decad reads it back from sketch's own
// exported interpolant (spline_fit.go) rather than recomputing it.
type FitSplineSeg struct {
	Fit    []Point2 `json:"fit"`
	TStart float64  `json:"t_start"`
	TEnd   float64  `json:"t_end"`
}

// ConicSeg is a rational quadratic Bezier: endpoints, the apex where the end
// tangents meet, and the fullness Rho in (0, 1) — Rho < 0.5 an ellipse arc,
// 0.5 a parabola, > 0.5 a hyperbola arc.
type ConicSeg struct {
	Start  Point2  `json:"start"`
	Apex   Point2  `json:"apex"`
	End    Point2  `json:"end"`
	Rho    float64 `json:"rho"`
	TStart float64 `json:"t_start"`
	TEnd   float64 `json:"t_end"`
}

// The sealed set: one variant per sketch entity kind — that is sketch's
// entity vocabulary exactly and entirely. A new entity kind upstream needs a
// new variant before decad accepts a profile that uses it; there is no
// fallback to a sample.
func (LineSeg) curveSegment()          {}
func (CircleSeg) curveSegment()        {}
func (ArcSeg) curveSegment()           {}
func (EllipseSeg) curveSegment()       {}
func (EllipticalArcSeg) curveSegment() {}
func (SplineSeg) curveSegment()        {}
func (NURBSSeg) curveSegment()         {}
func (ClosedSplineSeg) curveSegment()  {}
func (FitSplineSeg) curveSegment()     {}
func (ConicSeg) curveSegment()         {}
