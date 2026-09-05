package decad

import (
	"encoding/json"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the revolve of docs/evaluator-design.md §6: the
// sealed Axis vocabulary of docs/api-design.md §6.2, the §6 axis-contact
// gates, angular-extent resolution to a sweep interval (the body-relative
// ToFaceAngular stop resolves through stops.go, its body recorded as a
// StepRef in the step's Inputs — core §6.2), and the analytic revolve
// evaluator — Cylinder, Cone, planar-annulus, Sphere and Torus side faces
// per boundary segment kind, caps only on partial sweeps, and Pappus
// measurements with proven float-evaluation bounds.
//
// That evaluator is spread over three sibling files, each with its own doc
// comment: revolve_axis.go resolves the axis and classifies what the profile
// sweeps around it, revolve_build.go builds the body, and revolve_extent.go
// answers the extent questions asked of the result.

// Axis is what a revolve may spin about (docs/api-design.md §6.2). Sealed:
// the variants are SketchLine, ConstructionAxis and EdgeAxis.
type Axis interface{ axis() }

// SketchLine is a line in the source sketch: its endpoints in the sketch
// plane's own (u, v), millimetres — the §5.2 coordinate carve-out — recorded
// verbatim in the step. The axis direction, which Along is right-handed
// about, runs from Start toward End. The evaluator lifts it into world space
// through the step's own PlaneRecord, so it is coplanar with the profile
// plane by construction.
type SketchLine struct {
	Start Point2 `json:"start"`
	End   Point2 `json:"end"`
}

// ConstructionAxis is an explicit axis in the document: a world-space point
// on the axis (millimetres) and its direction (dimensionless, non-zero —
// the sense Along is right-handed about). A revolve axis must be coplanar
// with the profile plane; one that is not is ErrDegenerate at the call.
type ConstructionAxis struct {
	Origin r3.Vec `json:"origin"`
	Dir    r3.Vec `json:"dir"`
}

// EdgeAxis is a linear edge, selected — never a pointer. Body is what Edge
// resolves against: a Revolve is handed no body, so the axis must name its
// own. Edge MUST resolve to exactly one linear edge of Body — any other
// count is ErrCardinality, zero included (the implicit exactly-one of
// core §12), and a non-linear edge named as an axis is ErrDegenerate. Body
// must be a live body of the same document at the call (a StepRef there is
// ErrUnresolvedBody), and its StepRef is recorded in the step's Inputs — the
// step depends on it; the body is not consumed and not retired. The axis
// runs from the resolved edge's start vertex toward its end vertex, the
// sense Along is right-handed about.
type EdgeAxis struct {
	Body BodyRef
	Edge EdgeSelector
}

// The sealed set.
func (SketchLine) axis()       {}
func (ConstructionAxis) axis() {}
func (EdgeAxis) axis()         {}

// Axis is a closed variant set decad owns, so decad ships its codec
// (core §6.2): tagged objects, dispatch on the tag, no fallback.

const (
	axisKindSketchLine   = "sketch_line"
	axisKindConstruction = "construction_axis"
	axisKindEdge         = "edge_axis"
)

// errNilAxis rejects a nil variant pointer: it names no axis to spin about.
// It wraps ErrDegenerate so a typed nil pointer is branchable exactly like
// an untyped nil axis.
var errNilAxis = fmt.Errorf(`%w: nil axis`, ErrDegenerate)

// normalizeAxis returns the value form of a: the variants seal with value
// receivers, so a *SketchLine satisfies Axis as readily as a SketchLine does
// — the codec accepts both and records the value the pointer names. A nil
// pointer is rejected.
func normalizeAxis(a Axis) (Axis, error) {
	switch a := a.(type) {
	case *SketchLine:
		if a == nil {
			return nil, errNilAxis
		}
		return *a, nil
	case *ConstructionAxis:
		if a == nil {
			return nil, errNilAxis
		}
		return *a, nil
	case *EdgeAxis:
		if a == nil {
			return nil, errNilAxis
		}
		return *a, nil
	case nil:
		return nil, errNilAxis
	default:
		return a, nil
	}
}

// cloneAxis deep-copies a recorded axis so a Recipe never aliases a
// caller-owned selector. A malformed nil pointer stays as-is — the codecs
// and the feature call reject it at their own gates.
func cloneAxis(a Axis) Axis {
	n, err := normalizeAxis(a)
	if err != nil {
		return a
	}
	ea, ok := n.(EdgeAxis)
	if !ok {
		return n
	}
	if ea.Edge != nil {
		if sel, ok := cloneSelector(ea.Edge).(EdgeSelector); ok {
			ea.Edge = sel
		}
	}
	return ea
}

// marshalAxis encodes one axis as its tagged object. An EdgeAxis records its
// body as the producing StepRef — a live *Body is a handle, not a record.
func marshalAxis(a Axis) ([]byte, error) {
	a, err := normalizeAxis(a)
	if err != nil {
		return nil, err
	}
	switch a := a.(type) {
	case SketchLine:
		return marshalTagged(axisKindSketchLine, a)
	case ConstructionAxis:
		return marshalTagged(axisKindConstruction, a)
	case EdgeAxis:
		ref, ok := a.Body.(StepRef)
		if !ok {
			return nil, fmt.Errorf(`decad: an edge axis records its body as a StepRef, got %T`, a.Body)
		}
		if a.Edge == nil {
			return nil, errNilSelector
		}
		edge, err := marshalSelector(a.Edge)
		if err != nil {
			return nil, err
		}
		return json.Marshal(struct {
			Kind string          `json:"kind"`
			Body StepRef         `json:"body"`
			Edge json.RawMessage `json:"edge"`
		}{Kind: axisKindEdge, Body: ref, Edge: edge})
	default:
		return nil, fmt.Errorf(`decad: unencodable axis type %T`, a)
	}
}

// unmarshalAxis dispatches on the kind tag; an unknown or missing tag is an
// error — the set is closed.
func unmarshalAxis(data []byte) (Axis, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode axis tag: %w`, err))
	}
	switch probe.Kind {
	case axisKindSketchLine:
		// Wire structs with pointer fields: an absent endpoint is malformed
		// input, never silently the plane origin.
		var raw struct {
			Start *Point2 `json:"start"`
			End   *Point2 `json:"end"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode sketch-line axis: %w`, err))
		}
		if raw.Start == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a sketch-line axis requires both start and end`), "start")
		}
		if raw.End == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a sketch-line axis requires both start and end`), "end")
		}
		return SketchLine{Start: *raw.Start, End: *raw.End}, nil
	case axisKindConstruction:
		var raw struct {
			Origin *r3.Vec `json:"origin"`
			Dir    *r3.Vec `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode construction axis: %w`, err))
		}
		if raw.Origin == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a construction axis requires both origin and dir`), "origin")
		}
		if raw.Dir == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a construction axis requires both origin and dir`), "dir")
		}
		return ConstructionAxis{Origin: *raw.Origin, Dir: *raw.Dir}, nil
	case axisKindEdge:
		var raw struct {
			Body *StepRef        `json:"body"`
			Edge json.RawMessage `json:"edge"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode edge axis: %w`, err))
		}
		if raw.Body == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: an edge axis requires both body and edge`), "body")
		}
		if raw.Edge == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: an edge axis requires both body and edge`), "edge")
		}
		sel, err := unmarshalSelector(raw.Edge)
		if err != nil {
			return nil, prependCodecPath(err, "edge")
		}
		edge, ok := sel.(EdgeSelector)
		if !ok {
			return nil, prependCodecPath(fmt.Errorf(`decad: an edge axis requires an edge selector, got %T`, sel), "edge")
		}
		return EdgeAxis{Body: *raw.Body, Edge: edge}, nil
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: axis is missing its kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown axis kind %q`, probe.Kind), "kind")
	}
}

// RevolveOption configures Revolve. No options are currently supported: the
// option group exists so options can be added without changing the signature —
// every such option MUST be representable in a recorded RevolveOpts (core §6.2).
type RevolveOption interface {
	option.Interface
	revolveOption()
}

// Revolve sweeps a profile of s about axis per the angular extent a, and
// registers the new body. p MUST be a profile of s (ErrForeignProfile) and a
// current, unaltered snapshot (ErrStaleProfile or ErrInvalidProfile); an invalid
// profile is also ErrInvalidProfile, and a boundary decad cannot record exactly
// is ErrUnrecordableProfile (core §7). The axis must be non-degenerate and
// coplanar with the profile plane, and it must not pass through the region's
// interior: the region lies in one closed half-plane of the axis, and boundary
// contact is allowed in exactly two forms — a segment endpoint on the axis, and
// a whole line segment lying along it; anything else is ErrDegenerate
// (docs/evaluator-design.md §6). The step records the profile, the plane, the
// angular extent and the axis; evaluation runs from that record, and a failed
// evaluation leaves the recipe and the document untouched.
func (d *Document) Revolve(s *sketch.Sketch, p *sketch.Profile, axis Axis, a AngularExtent, opts ...RevolveOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	profile, plane, profileArea, err := recordProfile(s, p)
	if err != nil {
		return nil, err
	}
	// ONE free-form work counter for this whole operation over this record: the
	// area falsifier's preflight opens it, the axis gates and the revolve build's
	// own preflight continue it, and every walkOf under them spends what is left
	// (docs/spline-design.md §5.2).
	work := newFreeformWork()
	if err := falsifyRecordedArea(profile, profileArea, work); err != nil {
		return nil, err
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}

	axis, err = normalizeAxis(axis)
	if err != nil {
		return nil, err
	}
	// The evaluator spins about a resolved line; for an EdgeAxis that line
	// comes from the one edge the selector names, the recorded axis keeps
	// the query (with the body as its producing StepRef — a live *Body is a
	// handle, not a record), and the step depends on the named body.
	evalAxis := axis
	var axisRefs []StepRef
	if ea, ok := axis.(EdgeAxis); ok {
		line, ref, err := d.resolveEdgeAxis(ea)
		if err != nil {
			return nil, err
		}
		evalAxis = line
		axisRefs = []StepRef{ref}
		// The recorded query is deep-copied so the step never aliases the
		// caller-owned selector (core §6.2).
		axis = cloneAxis(EdgeAxis{Body: ref, Edge: ea.Edge})
	}

	a, err = normalizeAngularExtent(a)
	if err != nil {
		return nil, err
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	line, err := axisInPlane(evalAxis, frame)
	if err != nil {
		return nil, err
	}
	ax, side, err := resolveAxisSide(profile, line, work)
	if err != nil {
		return nil, err
	}
	// The angular extent resolves in the caller's frame — a ToFaceAngular
	// stop needs the resolved axis, which is why the axis gates run first.
	phi0, phi1, full, extRefs, err := d.resolveAngularExtent(a, d.angularStopCtx(frame, line, ax))
	if err != nil {
		return nil, err
	}
	if side < 0 {
		// The axis frame was flipped to put the region on its non-negative
		// side; a rotation by φ about the given axis is a rotation by −φ
		// about the flipped one, so the interval flips with it.
		phi0, phi1 = -phi1, -phi0
	}

	step := Step{
		Op:      OpRevolve,
		Inputs:  dedupRefs(append(extRefs, axisRefs...)),
		Profile: profile,
		Plane:   plane,
		Angular: recordAngularExtent(a),
		Axis:    axis,
	}
	ref := d.nextStepRef()
	body, err := evalRevolveWork(d, ref, work, revolvePayload{
		profile: profile,
		frame:   frame,
		ax:      ax,
		phi0:    phi0,
		phi1:    phi1,
		full:    full,
		xform:   r3.Identity(),
	})
	if err != nil {
		return nil, err
	}
	d.commit(step, body)
	return body, nil
}

// resolveEdgeAxis runs the EdgeAxis gates and resolves the named edge into
// the construction axis the evaluator spins about: the named body must be a
// live body of this document, and the selector must resolve to exactly one
// LINEAR edge of it — any other count is ErrCardinality, zero included (the
// implicit exactly-one of core §12 takes precedence over ErrNoMatch), and a
// non-linear edge named as a revolve axis is ErrDegenerate. The axis runs
// from the edge's start vertex toward its end vertex. It returns the derived
// axis and the StepRef the step's Inputs record its dependency as.
func (d *Document) resolveEdgeAxis(ea EdgeAxis) (ConstructionAxis, StepRef, error) {
	var body *Body
	switch b := ea.Body.(type) {
	case nil:
		return ConstructionAxis{}, 0, fmt.Errorf(`%w: an edge axis names no body to resolve against`, ErrDegenerate)
	case StepRef:
		return ConstructionAxis{}, 0, ErrUnresolvedBody
	case *Body:
		if err := d.requireLive(b); err != nil {
			return ConstructionAxis{}, 0, err
		}
		body = b
	default:
		return ConstructionAxis{}, 0, fmt.Errorf(`%w: an edge axis cannot resolve against a %T`, ErrDegenerate, b)
	}
	// A typed nil query is as empty a selector as an untyped nil: malformed
	// input (errNilSelector, branchable ErrDegenerate). decad owns the selector
	// vocabulary, so the only valid EdgeSelector is the built-in *EdgeQuery the
	// constructors return; a foreign implementation — including one that embeds
	// *EdgeQuery to promote the sealed selector() marker — is malformed input,
	// rejected as ErrDegenerate before SelectEdges runs, so every count-not-one
	// that reaches impliedOneEdge is a concrete query and cannot miss its
	// SelectionError.
	q, ok := ea.Edge.(*EdgeQuery)
	switch {
	case ea.Edge == nil:
		return ConstructionAxis{}, 0, fmt.Errorf(`%w: an edge axis names no edge selector`, ErrDegenerate)
	case !ok:
		return ConstructionAxis{}, 0, fmt.Errorf(`%w: an edge axis's edge selector is not a decad edge query (%T)`, ErrDegenerate, ea.Edge)
	case q == nil:
		return ConstructionAxis{}, 0, errNilSelector
	}
	edges, err := q.SelectEdges(body)
	if err != nil {
		// The selector's own explicit assertion failed: SelectEdges already
		// returned its SelectionError, and the caller gets it unchanged.
		if errors.Is(err, ErrCardinality) {
			return ConstructionAxis{}, 0, err
		}
		// An unasserted resolution that matched nothing: the implicit
		// exactly-one rewrites it to ErrCardinality, Expected "exactly 1".
		if errors.Is(err, ErrNoMatch) {
			return ConstructionAxis{}, 0, impliedOneEdge(body, q, 0)
		}
		return ConstructionAxis{}, 0, err
	}
	if len(edges) != 1 {
		// A successful resolution the implicit exactly-one turns into
		// Expected "exactly 1" / Actual len(edges).
		return ConstructionAxis{}, 0, impliedOneEdge(body, q, len(edges))
	}
	e := edges[0]
	if _, ok := e.Curve().(Line3); !ok {
		return ConstructionAxis{}, 0, fmt.Errorf(`%w: a non-linear edge named as a revolve axis spins about no line`, ErrDegenerate)
	}
	dir := e.end.position.Sub(e.start.position)
	if zeroVec(dir) {
		return ConstructionAxis{}, 0, fmt.Errorf(`%w: a zero-length edge names no axis`, ErrDegenerate)
	}
	return ConstructionAxis{Origin: e.start.position, Dir: dir}, body.originStep(), nil
}

// angFullEps separates "a full turn" from "past a full turn": an angular
// magnitude that lands on 2π within floating-point conversion noise IS a
// full revolution — the two caps would coincide — and one beyond it sweeps
// a self-overlapping solid, which is rejected.
const angFullEps = 1e-12

// resolveAngularExtent turns an angular extent into the signed sweep
// interval [phi0, phi1] about the axis, radians, Along positive
// (docs/evaluator-design.md §6), and reports whether the sweep is a full
// revolution, plus the StepRefs of the bodies the extent's stops resolved
// against — in extent order, deduplicated with the axis ref by the caller
// (core §6.2). Magnitudes are validated per core §8.1/§12; a zero-angle
// sweep is ErrDegenerate, as is one past a full turn.
func (d *Document) resolveAngularExtent(a AngularExtent, st angularStops) (float64, float64, bool, []StepRef, error) {
	var phi0, phi1 float64
	var refs []StepRef
	full := false
	switch a := a.(type) {
	case AngleExtent:
		m, err := magnitudeIn(a.A, units.Angle, units.Radian, "the extent angle")
		if err != nil {
			return 0, 0, false, nil, err
		}
		if m == 0 {
			return 0, 0, false, nil, fmt.Errorf(`%w: a zero-angle extent sweeps no solid`, ErrDegenerate)
		}
		// An unknown Direction is malformed input, never silently Along.
		switch a.Dir {
		case Along:
			phi0, phi1 = 0, m
		case Against:
			phi0, phi1 = -m, 0
		default:
			return 0, 0, false, nil, fmt.Errorf(`%w: unknown direction %d`, ErrDegenerate, int(a.Dir))
		}
	case FullRevolution:
		return 0, 2 * math.Pi, true, nil, nil
	case SymmetricAngle:
		m, err := magnitudeIn(a.A, units.Angle, units.Radian, "the symmetric angle")
		if err != nil {
			return 0, 0, false, nil, err
		}
		if m == 0 {
			return 0, 0, false, nil, fmt.Errorf(`%w: a zero-angle extent sweeps no solid`, ErrDegenerate)
		}
		half := m
		if a.FullLength {
			half = m / 2
		}
		phi0, phi1 = -half, half
	case TwoSidedAngle:
		along, oneRefs, err := d.resolveAngleSide(a.One, st, 1, "the along side")
		if err != nil {
			return 0, 0, false, nil, err
		}
		against, twoRefs, err := d.resolveAngleSide(a.Two, st, -1, "the against side")
		if err != nil {
			return 0, 0, false, nil, err
		}
		if along == 0 && against == 0 {
			return 0, 0, false, nil, fmt.Errorf(`%w: a zero-angle extent sweeps no solid`, ErrDegenerate)
		}
		phi0, phi1 = against, along
		refs = append(oneRefs, twoRefs...)
	case ToFaceAngular:
		stop, ref, err := st.resolveToFaceAngular(a, 0, "a to-face extent")
		if err != nil {
			return 0, 0, false, nil, err
		}
		refs = []StepRef{ref}
		if stop > 0 {
			phi0, phi1 = 0, stop
		} else {
			phi0, phi1 = stop, 0
		}
	case nil:
		return 0, 0, false, nil, fmt.Errorf(`%w: a nil extent sweeps nothing`, ErrDegenerate)
	default:
		return 0, 0, false, nil, fmt.Errorf(`%w: angular extent %T is not supported by this evaluator`, ErrUnsupported, a)
	}
	total := phi1 - phi0
	if total > 2*math.Pi+angFullEps {
		return 0, 0, false, nil, fmt.Errorf(`%w: a sweep past a full turn overlaps itself`, ErrDegenerate)
	}
	if total >= 2*math.Pi-angFullEps {
		full = true
		phi1 = phi0 + 2*math.Pi
	}
	return phi0, phi1, full, refs, nil
}

// resolveAngleSide resolves one side of a TwoSidedAngle to its signed
// boundary angle; travel is +1 for the along side, −1 for the against side.
func (d *Document) resolveAngleSide(s SideAngular, st angularStops, travel float64, what string) (float64, []StepRef, error) {
	s, err := normalizeSideAngular(s)
	if err != nil {
		return 0, nil, err
	}
	switch s := s.(type) {
	case AngleSide:
		m, err := magnitudeIn(s.A, units.Angle, units.Radian, what)
		if err != nil {
			return 0, nil, err
		}
		return travel * m, nil, nil
	case ToFaceAngular:
		stop, ref, err := st.resolveToFaceAngular(s, travel, what)
		if err != nil {
			return 0, nil, err
		}
		return stop, []StepRef{ref}, nil
	case nil:
		return 0, nil, fmt.Errorf(`%w: a two-sided extent requires both sides`, ErrDegenerate)
	default:
		return 0, nil, fmt.Errorf(`%w: side angular %T is not supported by this evaluator`, ErrUnsupported, s)
	}
}
