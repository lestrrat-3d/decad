package decad

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math"

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

// TransformRecord is a rigid placement, as four vectors: it survives encoding,
// which an r3.Transform does not (its fields are unexported, so a Step that
// stored one would silently drop the motion — docs/api-design.md §6.2). EX,
// EY, EZ are the transformed world basis (r3.Transform.Basis), dimensionless
// directions; T is the translation, millimetres (§5.2). RecordTransform
// converts a live transform in, and TransformRecord.Transform rebuilds one —
// through r3.FromBasis, which snaps encoding drift straight and rejects
// anything that is not an isometry.
type TransformRecord struct {
	EX r3.Vec `json:"ex"`
	EY r3.Vec `json:"ey"`
	EZ r3.Vec `json:"ez"`
	T  r3.Vec `json:"t"`
}

// RecordTransform converts a rigid motion into its record form. The zero
// r3.Transform is invalid and is [ErrDegenerate], exactly as Body.Placed
// treats it (docs/api-design.md §8) — an invalid transform names no placement
// to record.
func RecordTransform(t r3.Transform) (TransformRecord, error) {
	if !t.IsValid() {
		return TransformRecord{}, fmt.Errorf(`%w: an invalid transform names no placement to record`, ErrDegenerate)
	}
	b := t.Basis()
	return TransformRecord{EX: b.EX, EY: b.EY, EZ: b.EZ, T: t.Translation()}, nil
}

// Transform rebuilds the recorded rigid motion through r3.FromBasis, which
// snaps encoding drift straight and rejects a record that is not an isometry —
// a decoded placement is a real rigid motion or an error, never a silent
// distortion. An invalid record is [ErrDegenerate].
func (r TransformRecord) Transform() (r3.Transform, error) {
	t, err := r3.FromBasis(r3.Basis{EX: r.EX, EY: r.EY, EZ: r.EZ}, r.T)
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: the recorded placement is not a rigid motion: %v`, ErrDegenerate, err)
	}
	return t, nil
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
	s, err := normalizeSegment(s)
	if err != nil {
		return nil, err
	}
	kind, err := segmentKind(s)
	if err != nil {
		return nil, err
	}
	return marshalTagged(kind, s)
}

// marshalTagged encodes v's own fields with the kind tag spliced in front —
// {"kind":"<kind>", ...v's fields} — the one tagged-object encoder every
// closed variant set's codec shares (core §6.2).
func marshalTagged(kind string, v any) ([]byte, error) {
	body, err := json.Marshal(v)
	if err != nil {
		return nil, fmt.Errorf(`decad: failed to encode %s: %w`, kind, err)
	}
	tag, err := json.Marshal(struct {
		Kind string `json:"kind"`
	}{Kind: kind})
	if err != nil {
		return nil, fmt.Errorf(`decad: failed to encode %s tag: %w`, kind, err)
	}
	if string(body) == "{}" {
		return tag, nil
	}
	// Splice: {"kind":"..."} + {...fields} -> {"kind":"...",...fields}
	out := append(tag[:len(tag)-1], ',')
	out = append(out, body[1:]...)
	return out, nil
}

// segmentWire is the presence-aware form of every CurveSegment variant. Raw
// fields distinguish an omitted or null required field from a valid zero
// coordinate, parameter or boolean.
type segmentWire struct {
	Kind     *string         `json:"kind"`
	Start    json.RawMessage `json:"start"`
	End      json.RawMessage `json:"end"`
	Center   json.RawMessage `json:"center"`
	Apex     json.RawMessage `json:"apex"`
	Radius   json.RawMessage `json:"radius"`
	Rx       json.RawMessage `json:"rx"`
	Ry       json.RawMessage `json:"ry"`
	Rotation json.RawMessage `json:"rotation"`
	CCW      json.RawMessage `json:"ccw"`
	Control  json.RawMessage `json:"control"`
	Degree   json.RawMessage `json:"degree"`
	Knots    json.RawMessage `json:"knots"`
	Weights  json.RawMessage `json:"weights"`
	Fit      json.RawMessage `json:"fit"`
	Rho      json.RawMessage `json:"rho"`
	TStart   json.RawMessage `json:"t_start"`
	TEnd     json.RawMessage `json:"t_end"`
}

type namedSegmentWireField struct {
	name string
	raw  json.RawMessage
}

func presentSegmentWireField(raw json.RawMessage) bool {
	return len(raw) != 0 && !bytes.Equal(bytes.TrimSpace(raw), []byte("null"))
}

func requireSegmentWireFields(kind string, fields ...namedSegmentWireField) error {
	for _, field := range fields {
		if !presentSegmentWireField(field.raw) {
			return prependCodecPath(fmt.Errorf(`%w: %s segment is missing required field %q`, ErrDegenerate, kind, field.name), field.name)
		}
	}
	return nil
}

func requirePointWire(kind, field string, raw json.RawMessage) error {
	var point struct {
		U *float64 `json:"u"`
		V *float64 `json:"v"`
	}
	if err := json.Unmarshal(raw, &point); err != nil {
		return prependCodecPath(
			codecJSONErrorAt(raw, &point, fmt.Errorf(`decad: failed to decode %s segment field %q: %w`, kind, field, err)),
			field,
		)
	}
	if point.U == nil {
		return prependCodecPath(fmt.Errorf(`%w: %s segment field %q is missing required coordinate "u"`, ErrDegenerate, kind, field), field+".u")
	}
	if point.V == nil {
		return prependCodecPath(fmt.Errorf(`%w: %s segment field %q is missing required coordinate "v"`, ErrDegenerate, kind, field), field+".v")
	}
	return nil
}

func requirePointArrayWire(kind, field string, raw json.RawMessage) error {
	var points []json.RawMessage
	if err := json.Unmarshal(raw, &points); err != nil {
		return prependCodecPath(fmt.Errorf(`decad: failed to decode %s segment field %q: %w`, kind, field, err), field)
	}
	for i, point := range points {
		if !presentSegmentWireField(point) {
			return prependCodecPath(fmt.Errorf(`%w: %s segment field %q has a null point at index %d`, ErrDegenerate, kind, field, i), fmt.Sprintf(`%s[%d]`, field, i))
		}
		if err := requirePointWire(kind, fmt.Sprintf(`%s[%d]`, field, i), point); err != nil {
			return err
		}
	}
	return nil
}

func requireNumberArrayWire(kind, field string, raw json.RawMessage) error {
	var numbers []json.RawMessage
	if err := json.Unmarshal(raw, &numbers); err != nil {
		return prependCodecPath(fmt.Errorf(`decad: failed to decode %s segment field %q: %w`, kind, field, err), field)
	}
	for i, number := range numbers {
		if !presentSegmentWireField(number) {
			return prependCodecPath(fmt.Errorf(`%w: %s segment field %q has a null value at index %d`, ErrDegenerate, kind, field, i), fmt.Sprintf(`%s[%d]`, field, i))
		}
	}
	return nil
}

// validateSegmentWire checks required top-level and nested fields before the
// concrete variant is built. Unknown fields remain the root recipe codec's
// responsibility; this layer owns only CurveSegment requiredness.
func validateSegmentWire(kind string, wire segmentWire) error {
	f := func(name string, raw json.RawMessage) namedSegmentWireField {
		return namedSegmentWireField{name: name, raw: raw}
	}

	var required []namedSegmentWireField
	var points, pointArrays, numberArrays []namedSegmentWireField
	switch kind {
	case segKindLine:
		required = []namedSegmentWireField{
			f("start", wire.Start), f("end", wire.End), f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		points = required[:2]
	case segKindCircle:
		required = []namedSegmentWireField{
			f("center", wire.Center), f("radius", wire.Radius), f("ccw", wire.CCW),
			f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		points = required[:1]
	case segKindArc:
		required = []namedSegmentWireField{
			f("center", wire.Center), f("start", wire.Start), f("end", wire.End),
			f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		points = required[:3]
	case segKindEllipse:
		required = []namedSegmentWireField{
			f("center", wire.Center), f("rx", wire.Rx), f("ry", wire.Ry), f("rotation", wire.Rotation),
			f("ccw", wire.CCW), f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		points = required[:1]
	case segKindEllipticalArc:
		required = []namedSegmentWireField{
			f("center", wire.Center), f("start", wire.Start), f("end", wire.End),
			f("rx", wire.Rx), f("ry", wire.Ry), f("rotation", wire.Rotation),
			f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		points = required[:3]
	case segKindSpline:
		required = []namedSegmentWireField{
			f("control", wire.Control), f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		pointArrays = required[:1]
	case segKindNURBS:
		required = []namedSegmentWireField{
			f("degree", wire.Degree), f("control", wire.Control), f("knots", wire.Knots), f("weights", wire.Weights),
			f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		pointArrays = required[1:2]
		numberArrays = required[2:4]
	case segKindClosedSpline:
		required = []namedSegmentWireField{
			f("control", wire.Control), f("ccw", wire.CCW), f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		pointArrays = required[:1]
	case segKindFitSpline:
		required = []namedSegmentWireField{
			f("fit", wire.Fit), f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		pointArrays = required[:1]
	case segKindConic:
		required = []namedSegmentWireField{
			f("start", wire.Start), f("apex", wire.Apex), f("end", wire.End), f("rho", wire.Rho),
			f("t_start", wire.TStart), f("t_end", wire.TEnd),
		}
		points = required[:3]
	default:
		return prependCodecPath(fmt.Errorf(`decad: unknown curve segment kind %q`, kind), "kind")
	}

	if err := requireSegmentWireFields(kind, required...); err != nil {
		return err
	}
	for _, field := range points {
		if err := requirePointWire(kind, field.name, field.raw); err != nil {
			return err
		}
	}
	for _, field := range pointArrays {
		if err := requirePointArrayWire(kind, field.name, field.raw); err != nil {
			return err
		}
	}
	for _, field := range numberArrays {
		if err := requireNumberArrayWire(kind, field.name, field.raw); err != nil {
			return err
		}
	}
	return nil
}

// unmarshalSegment dispatches on the kind tag and decodes the matching
// variant. A missing or unknown tag is an error: the set is closed, and there
// is no fallback. Every required field is present and every segment invariant
// holds before the decoded geometry is returned.
func unmarshalSegment(data []byte) (CurveSegment, error) {
	var wire segmentWire
	if err := json.Unmarshal(data, &wire); err != nil {
		return nil, codecJSONErrorAt(data, &wire, fmt.Errorf(`decad: failed to decode curve segment tag: %w`, err))
	}
	if wire.Kind == nil {
		return nil, prependCodecPath(fmt.Errorf(`decad: curve segment is missing its kind tag`), "kind")
	}
	kind := *wire.Kind
	if kind == "" {
		return nil, prependCodecPath(fmt.Errorf(`decad: curve segment is missing its kind tag`), "kind")
	}
	if err := validateSegmentWire(kind, wire); err != nil {
		return nil, err
	}

	var seg CurveSegment
	switch kind {
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
	}
	if err := json.Unmarshal(data, seg); err != nil {
		return nil, codecJSONErrorAt(data, seg, fmt.Errorf(`decad: failed to decode %s segment: %w`, kind, err))
	}
	seg, err := normalizeSegment(seg)
	if err != nil {
		return nil, err
	}
	if err := validateSegment(seg); err != nil {
		return nil, err
	}
	return seg, nil
}

func finiteSegmentValue(v float64) bool {
	return !math.IsNaN(v) && !math.IsInf(v, 0)
}

func validateSegmentPoint(point Point2, what string) error {
	if !finiteSegmentValue(point.U) || !finiteSegmentValue(point.V) {
		return fmt.Errorf(`%w: %s must have finite coordinates`, ErrNotFinite, what)
	}
	return nil
}

func validateSegmentPoints(points []Point2, minimum int, what string) error {
	if len(points) < minimum {
		return fmt.Errorf(`%w: %s requires at least %d points, got %d`, ErrDegenerate, what, minimum, len(points))
	}
	for i, point := range points {
		if err := validateSegmentPoint(point, fmt.Sprintf(`%s point %d`, what, i)); err != nil {
			return prependCodecPath(err, fmt.Sprintf(`[%d]`, i))
		}
	}
	return nil
}

func validateSegmentParameter(v float64, what string) error {
	if !finiteSegmentValue(v) {
		return fmt.Errorf(`%w: %s must be finite`, ErrNotFinite, what)
	}
	return nil
}

func validateSegmentRange(tStart, tEnd float64, kind string) error {
	if err := validateSegmentParameter(tStart, kind+" segment t_start"); err != nil {
		return prependCodecPath(err, "t_start")
	}
	if err := validateSegmentParameter(tEnd, kind+" segment t_end"); err != nil {
		return prependCodecPath(err, "t_end")
	}
	if tStart < 0 || tStart > 1 {
		return prependCodecPath(
			fmt.Errorf(`%w: %s segment range [%v, %v] is outside [0, 1]`, ErrDegenerate, kind, tStart, tEnd),
			"t_start",
		)
	}
	if tEnd < 0 || tEnd > 1 {
		return prependCodecPath(
			fmt.Errorf(`%w: %s segment range [%v, %v] is outside [0, 1]`, ErrDegenerate, kind, tStart, tEnd),
			"t_end",
		)
	}
	if tStart == tEnd {
		return prependCodecPath(
			fmt.Errorf(`%w: %s segment has an empty range at %v`, ErrDegenerate, kind, tStart),
			"t_end",
		)
	}
	return nil
}

func validateSegmentWinding(ccw bool, tStart, tEnd float64, kind string) error {
	if ccw != (tStart < tEnd) {
		return prependCodecPath(
			fmt.Errorf(`%w: %s segment's CCW flag contradicts its range order`, ErrDegenerate, kind),
			"ccw",
		)
	}
	return nil
}

func validateSegmentMagnitude(v units.Value, kind units.Kind, unit units.Unit, what string) error {
	_, err := magnitudeIn(v, kind, unit, what)
	return err
}

func validateSegmentQuantity(v units.Value, kind units.Kind, unit units.Unit, what string) error {
	if v.Kind() != kind {
		return fmt.Errorf(`%w: %s must be a %s, got %s`, ErrUnitKind, what, kind, v.Kind())
	}
	if _, err := v.In(unit); err != nil {
		return fmt.Errorf(`%w: %s is not representable: %s`, ErrNotFinite, what, err)
	}
	return nil
}

func validateNURBSSegment(seg NURBSSeg) error {
	if seg.Degree < 1 {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment degree must be at least 1, got %d`, ErrDegenerate, seg.Degree),
			"degree",
		)
	}
	n := len(seg.Control)
	if seg.Degree >= n {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment requires more control points than its degree %d, got %d`, ErrDegenerate, seg.Degree, n),
			"degree",
		)
	}
	if err := validateSegmentPoints(seg.Control, seg.Degree+1, "NURBS segment control"); err != nil {
		return prependCodecPath(err, "control")
	}
	if len(seg.Knots) != n+seg.Degree+1 {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment needs %d knots, got %d`, ErrDegenerate, n+seg.Degree+1, len(seg.Knots)),
			"knots",
		)
	}
	for i, knot := range seg.Knots {
		if err := validateSegmentParameter(knot, fmt.Sprintf(`NURBS segment knot %d`, i)); err != nil {
			return prependCodecPath(err, fmt.Sprintf(`knots[%d]`, i))
		}
		if i > 0 && knot < seg.Knots[i-1] {
			return prependCodecPath(fmt.Errorf(`%w: NURBS segment knots must be non-decreasing`, ErrDegenerate), fmt.Sprintf(`knots[%d]`, i))
		}
	}
	for i := 1; i <= seg.Degree; i++ {
		if seg.Knots[i] != seg.Knots[0] {
			return prependCodecPath(fmt.Errorf(`%w: NURBS segment knot vector is not clamped at the start`, ErrDegenerate), fmt.Sprintf(`knots[%d]`, i))
		}
		if seg.Knots[len(seg.Knots)-1-i] != seg.Knots[len(seg.Knots)-1] {
			return prependCodecPath(fmt.Errorf(`%w: NURBS segment knot vector is not clamped at the end`, ErrDegenerate), fmt.Sprintf(`knots[%d]`, len(seg.Knots)-1-i))
		}
	}
	if seg.Knots[seg.Degree] >= seg.Knots[n] {
		return prependCodecPath(fmt.Errorf(`%w: NURBS segment knot domain is empty`, ErrDegenerate), "knots")
	}
	if len(seg.Weights) != n {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment needs %d weights, got %d`, ErrDegenerate, n, len(seg.Weights)),
			"weights",
		)
	}
	for i, weight := range seg.Weights {
		if err := validateSegmentParameter(weight, fmt.Sprintf(`NURBS segment weight %d`, i)); err != nil {
			return prependCodecPath(err, fmt.Sprintf(`weights[%d]`, i))
		}
		if weight <= 0 {
			return prependCodecPath(
				fmt.Errorf(`%w: NURBS segment weight %d must be positive, got %v`, ErrDegenerate, i, weight),
				fmt.Sprintf(`weights[%d]`, i),
			)
		}
	}
	return nil
}

// validateSegment enforces the CurveSegment checks from recipe replay §3.1:
// finite values, physical unit kinds, non-negative magnitudes, normalized
// non-empty ranges, closed-curve winding, spline field counts, and the complete
// clamped NURBS shape.
func validateSegment(segment CurveSegment) error {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return err
	}
	validateRange := func(kind string, tStart, tEnd float64) error {
		return validateSegmentRange(tStart, tEnd, kind)
	}
	switch seg := segment.(type) {
	case LineSeg:
		if err := validateSegmentPoint(seg.Start, "line segment start"); err != nil {
			return prependCodecPath(err, "start")
		}
		if err := validateSegmentPoint(seg.End, "line segment end"); err != nil {
			return prependCodecPath(err, "end")
		}
		return validateRange(segKindLine, seg.TStart, seg.TEnd)
	case CircleSeg:
		if err := validateSegmentPoint(seg.Center, "circle segment center"); err != nil {
			return prependCodecPath(err, "center")
		}
		if err := validateSegmentMagnitude(seg.Radius, units.Length, units.Millimeter, "circle segment radius"); err != nil {
			return prependCodecPath(err, "radius")
		}
		if err := validateRange(segKindCircle, seg.TStart, seg.TEnd); err != nil {
			return err
		}
		return validateSegmentWinding(seg.CCW, seg.TStart, seg.TEnd, segKindCircle)
	case ArcSeg:
		for _, point := range []struct {
			value Point2
			name  string
		}{{seg.Center, "center"}, {seg.Start, "start"}, {seg.End, "end"}} {
			if err := validateSegmentPoint(point.value, "arc segment "+point.name); err != nil {
				return prependCodecPath(err, point.name)
			}
		}
		return validateRange(segKindArc, seg.TStart, seg.TEnd)
	case EllipseSeg:
		if err := validateSegmentPoint(seg.Center, "ellipse segment center"); err != nil {
			return prependCodecPath(err, "center")
		}
		if err := validateSegmentMagnitude(seg.Rx, units.Length, units.Millimeter, "ellipse segment rx"); err != nil {
			return prependCodecPath(err, "rx")
		}
		if err := validateSegmentMagnitude(seg.Ry, units.Length, units.Millimeter, "ellipse segment ry"); err != nil {
			return prependCodecPath(err, "ry")
		}
		if err := validateSegmentQuantity(seg.Rotation, units.Angle, units.Radian, "ellipse segment rotation"); err != nil {
			return prependCodecPath(err, "rotation")
		}
		if err := validateRange(segKindEllipse, seg.TStart, seg.TEnd); err != nil {
			return err
		}
		return validateSegmentWinding(seg.CCW, seg.TStart, seg.TEnd, segKindEllipse)
	case EllipticalArcSeg:
		for _, point := range []struct {
			value Point2
			name  string
		}{{seg.Center, "center"}, {seg.Start, "start"}, {seg.End, "end"}} {
			if err := validateSegmentPoint(point.value, "elliptical arc segment "+point.name); err != nil {
				return prependCodecPath(err, point.name)
			}
		}
		if err := validateSegmentMagnitude(seg.Rx, units.Length, units.Millimeter, "elliptical arc segment rx"); err != nil {
			return prependCodecPath(err, "rx")
		}
		if err := validateSegmentMagnitude(seg.Ry, units.Length, units.Millimeter, "elliptical arc segment ry"); err != nil {
			return prependCodecPath(err, "ry")
		}
		if err := validateSegmentQuantity(seg.Rotation, units.Angle, units.Radian, "elliptical arc segment rotation"); err != nil {
			return prependCodecPath(err, "rotation")
		}
		return validateRange(segKindEllipticalArc, seg.TStart, seg.TEnd)
	case SplineSeg:
		if err := validateSegmentPoints(seg.Control, 4, "spline segment control"); err != nil {
			return prependCodecPath(err, "control")
		}
		return validateRange(segKindSpline, seg.TStart, seg.TEnd)
	case NURBSSeg:
		if err := validateNURBSSegment(seg); err != nil {
			return err
		}
		return validateRange(segKindNURBS, seg.TStart, seg.TEnd)
	case ClosedSplineSeg:
		if err := validateSegmentPoints(seg.Control, 3, "closed spline segment control"); err != nil {
			return prependCodecPath(err, "control")
		}
		if err := validateRange(segKindClosedSpline, seg.TStart, seg.TEnd); err != nil {
			return err
		}
		return validateSegmentWinding(seg.CCW, seg.TStart, seg.TEnd, segKindClosedSpline)
	case FitSplineSeg:
		if err := validateSegmentPoints(seg.Fit, 2, "fit spline segment fit"); err != nil {
			return prependCodecPath(err, "fit")
		}
		return validateRange(segKindFitSpline, seg.TStart, seg.TEnd)
	case ConicSeg:
		for _, point := range []struct {
			value Point2
			name  string
		}{{seg.Start, "start"}, {seg.Apex, "apex"}, {seg.End, "end"}} {
			if err := validateSegmentPoint(point.value, "conic segment "+point.name); err != nil {
				return prependCodecPath(err, point.name)
			}
		}
		if err := validateSegmentParameter(seg.Rho, "conic segment rho"); err != nil {
			return prependCodecPath(err, "rho")
		}
		if seg.Rho <= 0 || seg.Rho >= 1 {
			return prependCodecPath(fmt.Errorf(`%w: conic segment rho must be in (0, 1), got %v`, ErrDegenerate, seg.Rho), "rho")
		}
		return validateRange(segKindConic, seg.TStart, seg.TEnd)
	default:
		return fmt.Errorf(`%w: unknown curve segment type %T`, ErrDegenerate, segment)
	}
}

// errNilSegment rejects a nil variant pointer: it names no curve to record.
// It wraps ErrDegenerate so the failure is branchable wherever it surfaces —
// the codec, the integrals, or a feature call.
var errNilSegment = fmt.Errorf(`%w: nil curve segment`, ErrDegenerate)

// normalizeSegment returns the value form of s. The variants implement
// CurveSegment with value receivers, so a *LineSeg satisfies the interface as
// readily as a LineSeg does — the codec accepts both and records the value
// the pointer names; the decode buffer normalizes through the same path, so
// the codec always hands back value forms.
func normalizeSegment(s CurveSegment) (CurveSegment, error) {
	switch s := s.(type) {
	case *LineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *CircleSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *ArcSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *EllipseSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *EllipticalArcSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *SplineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *NURBSSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *ClosedSplineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *FitSplineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *ConicSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	default:
		return s, nil
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
		return codecJSONError(fmt.Errorf(`decad: failed to decode loop record: %w`, err))
	}
	segs := make([]CurveSegment, 0, len(raw.Segments))
	for i, b := range raw.Segments {
		s, err := unmarshalSegment(b)
		if err != nil {
			return prependCodecPath(err, fmt.Sprintf(`segments[%d]`, i))
		}
		segs = append(segs, s)
	}
	l.Segments = segs
	return nil
}
