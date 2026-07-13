package decad

import (
	"encoding/json"
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the recording IR of docs/sketch-seam-design.md §2: the
// structural, plane-local types a Recipe Step records a profile in. A Step
// holds no *sketch.Profile and no r3.Frame — decad converts, it does not
// reference — so a Recipe stays a value after the sketch has moved on, and
// every type here is encodable and decodable (docs/api-design.md §6.2).

// PlaneRecord is the sketch plane, as three vectors: it survives encoding,
// which an r3.Frame does not. Orthonormal, right-handed; the plane normal is
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

// ProfileRecord is the region a Step extrudes or revolves: one outer loop and
// its holes, structural and plane-local. Not a sample, not a pointer, not a
// sketch.
type ProfileRecord struct {
	Outer LoopRecord   `json:"outer"`
	Holes []LoopRecord `json:"holes,omitempty"`
}

// LoopRecord is a closed, directed boundary loop: each segment's walk — from
// its point at TStart to its point at TEnd — ends where the next segment's
// walk starts, and the last closes onto the first. A single closed segment —
// a circle, an ellipse, a closed spline — is a loop on its own. Outer loops
// run counter-clockwise in (u, v), holes clockwise.
type LoopRecord struct {
	Segments []CurveSegment
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
// the points and NEVER runs the interpolation solve itself.
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

// CurveSegment is a closed variant set decad owns, so decad ships its codec
// (docs/api-design.md §6.2): each variant encodes as a tagged object —
// {"kind": "<tag>", ...the variant's fields} — and decoding dispatches on the
// tag. A units.Value field round-trips through its own text form ("5 mm"); a
// curve parameter is a plain dimensionless float.

const (
	segKindLine          = "line"
	segKindCircle        = "circle"
	segKindArc           = "arc"
	segKindEllipse       = "ellipse"
	segKindEllipticalArc = "elliptical_arc"
	segKindSpline        = "spline"
	segKindNURBS         = "nurbs"
	segKindClosedSpline  = "closed_spline"
	segKindFitSpline     = "fit_spline"
	segKindConic         = "conic"
)

// segmentKind returns the tag a variant encodes under. The switch is total
// over the sealed set.
func segmentKind(s CurveSegment) (string, error) {
	switch s.(type) {
	case LineSeg:
		return segKindLine, nil
	case CircleSeg:
		return segKindCircle, nil
	case ArcSeg:
		return segKindArc, nil
	case EllipseSeg:
		return segKindEllipse, nil
	case EllipticalArcSeg:
		return segKindEllipticalArc, nil
	case SplineSeg:
		return segKindSpline, nil
	case NURBSSeg:
		return segKindNURBS, nil
	case ClosedSplineSeg:
		return segKindClosedSpline, nil
	case FitSplineSeg:
		return segKindFitSpline, nil
	case ConicSeg:
		return segKindConic, nil
	default:
		return "", fmt.Errorf(`decad: unencodable curve segment type %T`, s)
	}
}

// marshalSegment encodes one variant as its tagged object: the variant's own
// fields, with the kind tag spliced in front.
func marshalSegment(s CurveSegment) ([]byte, error) {
	kind, err := segmentKind(s)
	if err != nil {
		return nil, err
	}
	body, err := json.Marshal(s)
	if err != nil {
		return nil, fmt.Errorf(`decad: failed to encode %s segment: %w`, kind, err)
	}
	tag, err := json.Marshal(struct {
		Kind string `json:"kind"`
	}{Kind: kind})
	if err != nil {
		return nil, fmt.Errorf(`decad: failed to encode %s segment tag: %w`, kind, err)
	}
	if string(body) == "{}" {
		return tag, nil
	}
	// Splice: {"kind":"..."} + {...fields} -> {"kind":"...",...fields}
	out := append(tag[:len(tag)-1], ',')
	out = append(out, body[1:]...)
	return out, nil
}

// unmarshalSegment dispatches on the kind tag and decodes the matching
// variant. A missing or unknown tag is an error: the set is closed, and there
// is no fallback.
func unmarshalSegment(data []byte) (CurveSegment, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, fmt.Errorf(`decad: failed to decode curve segment tag: %w`, err)
	}

	var seg CurveSegment
	switch probe.Kind {
	case segKindLine:
		seg = &LineSeg{}
	case segKindCircle:
		seg = &CircleSeg{}
	case segKindArc:
		seg = &ArcSeg{}
	case segKindEllipse:
		seg = &EllipseSeg{}
	case segKindEllipticalArc:
		seg = &EllipticalArcSeg{}
	case segKindSpline:
		seg = &SplineSeg{}
	case segKindNURBS:
		seg = &NURBSSeg{}
	case segKindClosedSpline:
		seg = &ClosedSplineSeg{}
	case segKindFitSpline:
		seg = &FitSplineSeg{}
	case segKindConic:
		seg = &ConicSeg{}
	case "":
		return nil, fmt.Errorf(`decad: curve segment is missing its kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown curve segment kind %q`, probe.Kind)
	}
	if err := json.Unmarshal(data, seg); err != nil {
		return nil, fmt.Errorf(`decad: failed to decode %s segment: %w`, probe.Kind, err)
	}
	return derefSegment(seg), nil
}

// derefSegment returns the value the decode buffer holds: the codec hands
// back the same value forms the sealed set is built from.
func derefSegment(s CurveSegment) CurveSegment {
	switch s := s.(type) {
	case *LineSeg:
		return *s
	case *CircleSeg:
		return *s
	case *ArcSeg:
		return *s
	case *EllipseSeg:
		return *s
	case *EllipticalArcSeg:
		return *s
	case *SplineSeg:
		return *s
	case *NURBSSeg:
		return *s
	case *ClosedSplineSeg:
		return *s
	case *FitSplineSeg:
		return *s
	case *ConicSeg:
		return *s
	default:
		return s
	}
}

// MarshalJSON encodes the loop as {"segments": [tagged objects...]}.
func (l LoopRecord) MarshalJSON() ([]byte, error) {
	segs := make([]json.RawMessage, 0, len(l.Segments))
	for _, s := range l.Segments {
		b, err := marshalSegment(s)
		if err != nil {
			return nil, err
		}
		segs = append(segs, b)
	}
	return json.Marshal(struct {
		Segments []json.RawMessage `json:"segments"`
	}{Segments: segs})
}

// UnmarshalJSON decodes {"segments": [...]}, dispatching each segment on its
// kind tag.
func (l *LoopRecord) UnmarshalJSON(data []byte) error {
	var raw struct {
		Segments []json.RawMessage `json:"segments"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return fmt.Errorf(`decad: failed to decode loop record: %w`, err)
	}
	segs := make([]CurveSegment, 0, len(raw.Segments))
	for _, b := range raw.Segments {
		s, err := unmarshalSegment(b)
		if err != nil {
			return err
		}
		segs = append(segs, s)
	}
	l.Segments = segs
	return nil
}
