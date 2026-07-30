package decad

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"math/big"

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

// axisLine2 is a revolve axis resolved into the profile plane: a point on
// the axis and its unit direction, plane-local (u, v).
type axisLine2 struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
}

// finiteAxisValues reports whether every derived axis value is representable.
// Axis inputs are checked before arithmetic, but subtracting finite endpoints
// or transforming a finite world point can still overflow.
func finiteAxisValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

// axisInPlane resolves an axis variant into plane-local coordinates,
// validating it non-degenerate and coplanar with the profile plane
// (docs/evaluator-design.md §6).
func axisInPlane(a Axis, frame r3.Frame) (axisLine2, error) {
	switch a := a.(type) {
	case SketchLine:
		for _, c := range []float64{a.Start.U, a.Start.V, a.End.U, a.End.V} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis endpoint is not finite`, ErrNotFinite)
			}
		}
		du, dv := a.End.U-a.Start.U, a.End.V-a.Start.V
		if !finiteAxisValues(du, dv) {
			return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis delta is not finite`, ErrNotFinite)
		}
		scale := math.Max(math.Abs(du), math.Abs(dv))
		if scale == 0 {
			return axisLine2{}, fmt.Errorf(`%w: a zero-length sketch line names no axis`, ErrDegenerate)
		}
		scaledU, scaledV := du/scale, dv/scale
		l := math.Hypot(scaledU, scaledV)
		if !finiteAxisValues(l) {
			return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis length is not finite`, ErrNotFinite)
		}
		dU, dV := scaledU/l, scaledV/l
		if !finiteAxisValues(dU, dV) {
			return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis direction is not finite`, ErrNotFinite)
		}
		// Recover the held length for exact direction-bound checks. An overflowing
		// magnitude falls back to the conservative direction bound.
		l = scale * l
		dUBound, dVBound := sketchAxisDirectionBounds(a, l, dU, dV)
		return axisLine2{
			aU: a.Start.U, aV: a.Start.V,
			dU: dU, dV: dV,
			dUBound: dUBound,
			dVBound: dVBound,
		}, nil
	case ConstructionAxis:
		for _, c := range []float64{a.Origin.X, a.Origin.Y, a.Origin.Z, a.Dir.X, a.Dir.Y, a.Dir.Z} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return axisLine2{}, fmt.Errorf(`%w: a construction axis component is not finite`, ErrNotFinite)
			}
		}
		dir, ok := a.Dir.Normalize()
		if !ok {
			return axisLine2{}, fmt.Errorf(`%w: a zero-direction construction axis names no axis`, ErrDegenerate)
		}
		if !finiteAxisValues(dir.X, dir.Y, dir.Z) {
			return axisLine2{}, fmt.Errorf(`%w: a normalized construction axis direction is not finite`, ErrNotFinite)
		}
		local := frame.ToLocal(a.Origin)
		if !finiteAxisValues(local.X, local.Y, local.Z) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis has non-finite plane-local coordinates`, ErrNotFinite)
		}
		localLen := local.Len()
		if !finiteAxisValues(localLen) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis plane-local length is not finite`, ErrNotFinite)
		}
		scale := math.Max(1, localLen)
		if math.Abs(local.Z) > 1e-9*scale {
			return axisLine2{}, fmt.Errorf(`%w: the revolve axis does not lie in the profile plane`, ErrDegenerate)
		}
		du, dv, dn := dir.Dot(frame.U()), dir.Dot(frame.V()), dir.Dot(frame.N())
		if !finiteAxisValues(du, dv, dn) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis has a non-finite plane-local direction`, ErrNotFinite)
		}
		if math.Abs(dn) > 1e-9 {
			return axisLine2{}, fmt.Errorf(`%w: the revolve axis does not lie in the profile plane`, ErrDegenerate)
		}
		l := math.Hypot(du, dv)
		if !finiteAxisValues(l) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis plane-local direction length is not finite`, ErrNotFinite)
		}
		if l == 0 {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis has no direction in the profile plane`, ErrDegenerate)
		}
		dU, dV := du/l, dv/l
		if !finiteAxisValues(dU, dV) {
			return axisLine2{}, fmt.Errorf(`%w: a normalized construction axis plane-local direction is not finite`, ErrNotFinite)
		}
		anchorUpper := absSumUpper(
			a.Origin.X, a.Origin.Y, a.Origin.Z,
			frame.Origin().X, frame.Origin().Y, frame.Origin().Z,
		)
		dUBound, dVBound := conservativeValueError(dU, 1), conservativeValueError(dV, 1)
		if (dU == 0 || math.Abs(dU) == 1) &&
			(dV == 0 || math.Abs(dV) == 1) &&
			dU*dU+dV*dV == 1 {
			dUBound, dVBound = 0, 0
		}
		return axisLine2{
			aU: local.X, aV: local.Y,
			aUBound: conservativeValueError(local.X, anchorUpper),
			aVBound: conservativeValueError(local.Y, anchorUpper),
			dU:      dU, dV: dV,
			dUBound: dUBound,
			dVBound: dVBound,
		}, nil
	default:
		// EdgeAxis is gated before extent resolution; any other variant is
		// staged, never guessed.
		return axisLine2{}, fmt.Errorf(`%w: axis %T is not supported by this evaluator`, ErrUnsupported, a)
	}
}

func sketchAxisDirectionBounds(a SketchLine, heldLength, heldU, heldV float64) (float64, float64) {
	u0, v0 := floatRat(a.Start.U), floatRat(a.Start.V)
	u1, v1 := floatRat(a.End.U), floatRat(a.End.V)
	length := floatRat(heldLength)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil || length == nil || length.Sign() == 0 {
		return conservativeValueError(heldU, 1), conservativeValueError(heldV, 1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	if new(big.Rat).Mul(length, length).Cmp(lengthSquared) != 0 {
		return conservativeValueError(heldU, 1), conservativeValueError(heldV, 1)
	}
	componentBound := func(delta *big.Rat, held float64) float64 {
		exact := new(big.Rat).Quo(delta, length)
		heldRat := floatRat(held)
		if heldRat != nil && exact.Cmp(heldRat) == 0 {
			return 0
		}
		return conservativeValueError(held, 1)
	}
	return componentBound(du, heldU), componentBound(dv, heldV)
}

// axisFrame is the revolve axis as a proper plane-local frame with the
// region on its non-negative side: z = (p−a)·d runs along the axis and
// ρ = cross(d, p−a) ≥ 0 is the radial coordinate. snapTol is the
// scale-relative tolerance that classified axis contact; a coordinate
// within it of the axis IS on the axis.
type axisFrame struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
	snapTol          float64
}

// toAxis maps a plane-local point into (z, ρ) axis coordinates.
func (ax axisFrame) toAxis(u, v float64) (float64, float64) {
	du, dv := u-ax.aU, v-ax.aV
	return du*ax.dU + dv*ax.dV, dv*ax.dU - du*ax.dV
}

// walk re-expresses one boundary walk in axis coordinates (the U fields
// carry z, the V fields ρ), snapping an endpoint within snapTol onto the
// axis so contact classification and vertex placement agree exactly.
func (ax axisFrame) walk(w segmentWalk) segmentWalk {
	out := w
	out.startU, out.startV = ax.toAxis(w.startU, w.startV)
	out.endU, out.endV = ax.toAxis(w.endU, w.endV)
	out.tanInU = w.tanInU*ax.dU + w.tanInV*ax.dV
	out.tanInV = w.tanInV*ax.dU - w.tanInU*ax.dV
	out.tanOutU = w.tanOutU*ax.dU + w.tanOutV*ax.dV
	out.tanOutV = w.tanOutV*ax.dU - w.tanOutU*ax.dV
	if math.Abs(out.startV) <= ax.snapTol {
		out.startV = 0
	}
	if math.Abs(out.endV) <= ax.snapTol {
		out.endV = 0
	}
	if w.isCircular() {
		beta := math.Atan2(ax.dV, ax.dU)
		out.cU, out.cV = ax.toAxis(w.cU, w.cV)
		out.th0 = w.th0 - beta
		out.th1 = w.th1 - beta
	}
	anchorUpper := absSumUpper(
		ax.aU, ax.aUBound,
		ax.aV, ax.aVBound,
	)
	rhoUpper := absSumUpper(w.coordUpper, anchorUpper)
	out.axisRadiusUpper = rhoUpper
	out.axisMomentUpper = productUpper(w.lengthUpper, rhoUpper)
	return out
}

// wallKind classifies what one boundary walk sweeps.
type wallKind int

const (
	// wallAxis is a line lying along the axis: it sweeps a zero-area set
	// and emits no face — the neighboring segments' faces close the solid.
	wallAxis wallKind = iota
	// wallCylinder is a line parallel to the axis.
	wallCylinder
	// wallPlane is a line perpendicular to the axis: a planar annulus, or a
	// disk when it reaches the axis.
	wallPlane
	// wallCone is an inclined line; an endpoint on the axis is its apex.
	wallCone
	// wallSphere is a circular walk whose center lies on the axis.
	wallSphere
	// wallTorus is a circular walk whose center lies off the axis.
	wallTorus
)

// classify names the surface of revolution one axis-coordinate walk sweeps.
func (ax axisFrame) classify(w segmentWalk) wallKind {
	if w.isCircular() {
		if math.Abs(w.cV) <= ax.snapTol {
			return wallSphere
		}
		return wallTorus
	}
	if w.startV == 0 && w.endV == 0 {
		return wallAxis
	}
	dz, dr := w.endU-w.startU, w.endV-w.startV
	l := math.Hypot(dz, dr)
	if math.Abs(dr) <= 1e-9*l {
		return wallCylinder
	}
	if math.Abs(dz) <= 1e-9*l {
		return wallPlane
	}
	return wallCone
}

// resolveAxisSide orients the axis so the recorded region lies on its
// non-negative-ρ side, enforcing the §6 half-plane and contact rules: a
// region with boundary on both sides of the axis is rejected, as is a curve
// tangent to the axis at an interior point (a circle kissing the axis would
// sweep a self-touching horn torus). It returns the oriented frame and the
// side sign the sweep interval must be remapped by.
func resolveAxisSide(profile ProfileRecord, line axisLine2, work *freeformWork) (axisFrame, float64, error) {
	nU, nV := -line.dV, line.dU
	rlo, rhi, err := boundaryExtremes(profile, nU, nV, work)
	if err != nil {
		return axisFrame{}, 0, err
	}
	roff := nU*line.aU + nV*line.aV
	rlo, rhi = rlo-roff, rhi-roff
	zlo, zhi, err := boundaryExtremes(profile, line.dU, line.dV, work)
	if err != nil {
		return axisFrame{}, 0, err
	}
	zoff := line.dU*line.aU + line.dV*line.aV
	zlo, zhi = zlo-zoff, zhi-zoff

	scale := math.Max(math.Max(math.Abs(rlo), math.Abs(rhi)), math.Max(math.Abs(zlo), math.Abs(zhi)))
	tol := 1e-9 * math.Max(1, scale)
	switch {
	case rhi > tol && rlo < -tol:
		return axisFrame{}, 0, fmt.Errorf(`%w: the revolve axis passes through the region`, ErrDegenerate)
	case rhi <= tol && rlo >= -tol:
		return axisFrame{}, 0, fmt.Errorf(`%w: the region collapses onto the revolve axis`, ErrDegenerate)
	}
	side := 1.0
	if rhi <= tol {
		side = -1
	}
	ax := axisFrame{
		aU: line.aU, aV: line.aV,
		aUBound: line.aUBound, aVBound: line.aVBound,
		dU: side * line.dU, dV: side * line.dV,
		dUBound: line.dUBound, dVBound: line.dVBound,
		snapTol: tol,
	}
	if err := ax.rejectInteriorContact(profile, work); err != nil {
		return axisFrame{}, 0, err
	}
	return ax, side, nil
}

// rejectInteriorContact rejects the circular boundary walks a revolve
// cannot sweep soundly: a walk tangent to the axis at a point interior to
// the walk — the horn-torus contact §6 forbids — and a walk whose circle
// center lies across the axis, whose swept surface is a spindle-branch
// torus the shipped Torus (non-negative Major) cannot represent; the solid
// is valid, so that one is the staged ErrUnsupported, never a wrong face.
// For the tangency, the circle's radial minimum sits at its lowest angle;
// when that angle is strictly inside the walked range and the minimum
// reaches the axis, the contact is neither of the two allowed forms.
func (ax axisFrame) rejectInteriorContact(profile ProfileRecord, work *freeformWork) error {
	const angEps = 1e-9
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return err
			}
			if err := requireAnalyticWalk(w, "the revolve axis-contact audit"); err != nil {
				return err
			}
			w = ax.walk(w)
			if !w.isCircular() {
				continue
			}
			if w.cV < -ax.snapTol {
				return fmt.Errorf(`%w: a boundary arc centered across the revolve axis sweeps a spindle torus this evaluator cannot represent`, ErrUnsupported)
			}
			if w.cV-w.radius > ax.snapTol {
				continue
			}
			lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			if w.closed {
				return fmt.Errorf(`%w: a closed curve touching the revolve axis sweeps a self-touching solid`, ErrDegenerate)
			}
			// The minimum-ρ angle is −π/2 modulo a full turn.
			for th := -math.Pi/2 + 2*math.Pi*math.Floor((lo+math.Pi/2)/(2*math.Pi)); th <= hi+angEps; th += 2 * math.Pi {
				if th > lo+angEps && th < hi-angEps {
					return fmt.Errorf(`%w: the boundary touches the revolve axis at an interior point`, ErrDegenerate)
				}
			}
		}
	}
	return nil
}

// revolvePayload is the evaluator's own record of a revolved body: the
// recorded region, the plane frame, the oriented plane-local axis, the sweep
// interval (right-handed about the axis, phi1 > phi0; exactly 2π apart when
// full), and the accumulated rigid placement. Every measurement and the
// whole topology derive from it, which is what makes Placed exact: it
// re-evaluates the same payload under the composed motion
// (docs/evaluator-design.md §8).
type revolvePayload struct {
	profile    ProfileRecord
	frame      r3.Frame
	ax         axisFrame
	phi0, phi1 float64
	full       bool
	xform      r3.Transform
}

// transform is the accumulated rigid placement.
func (rp revolvePayload) transform() r3.Transform { return rp.xform }

// placed re-evaluates the same record under the composed motion.
func (rp revolvePayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	rp.xform = composed
	return evalRevolveContext(ctx, d, ref, rp)
}

// revolveBasis is the unplaced world anchor of the sweep: a3 the axis
// origin, w the unit axis direction, e0 the in-plane radial direction at
// sweep angle zero, and e1 = w × e0 the sweep-velocity direction at zero —
// so a rotation by +φ about w carries e0 toward e1, the right-handed sense
// Along means.
type revolveBasis struct {
	a3, w, e0, e1 r3.Vec
}

// basis derives the sweep basis from the plane frame and the axis frame.
func (rp revolvePayload) basis() revolveBasis {
	a3 := rp.frame.ToWorldUV(rp.ax.aU, rp.ax.aV)
	w := rp.frame.U().Scale(rp.ax.dU).Add(rp.frame.V().Scale(rp.ax.dV))
	e0 := rp.frame.U().Scale(-rp.ax.dV).Add(rp.frame.V().Scale(rp.ax.dU))
	return revolveBasis{a3: a3, w: w, e0: e0, e1: w.Cross(e0)}
}

// revolveCentroidGeometryBound bounds the centroid independently of the
// Pappus quotient. Every material point starts in the recorded profile plane,
// rotates about the resolved axis, then passes through a rigid placement. The
// L1 envelopes use three times an input L1 norm for any orthogonal map.
func revolveCentroidGeometryBound(rp revolvePayload, held r3.Vec, work *freeformWork) (float64, error) {
	coordUpper, err := profileCoordinateUpper(rp.profile, work)
	if err != nil {
		return 0, err
	}
	originUpper := vecL1(rp.frame.Origin())
	profileUpper := absSumUpper(
		originUpper,
		productUpper(vecL1(rp.frame.U()), coordUpper),
		productUpper(vecL1(rp.frame.V()), coordUpper),
	)
	aUUpper := absSumUpper(rp.ax.aU, rp.ax.aUBound)
	aVUpper := absSumUpper(rp.ax.aV, rp.ax.aVBound)
	axisUpper := absSumUpper(
		originUpper,
		productUpper(vecL1(rp.frame.U()), aUUpper),
		productUpper(vecL1(rp.frame.V()), aVUpper),
	)
	rotatedUpper := absSumUpper(productUpper(3, profileUpper), productUpper(4, axisUpper))
	placedUpper := absSumUpper(productUpper(3, rotatedUpper), vecL1(rp.xform.Translation()))
	return absSumUpper(vecL1(held), placedUpper), nil
}

// point places the axis-frame point (z, ρ) at sweep angle φ into placed
// world space.
func (rp revolvePayload) point(b revolveBasis, z, rho, phi float64) r3.Vec {
	sin, cos := math.Sincos(phi)
	radial := b.e0.Scale(cos).Add(b.e1.Scale(sin))
	return rp.xform.Apply(b.a3.Add(b.w.Scale(z)).Add(radial.Scale(rho)))
}

// reflected reports whether the accumulated placement flips handedness — a
// reflected solid's rotational senses invert with it, and every swept edge's
// axis carries the corrected sign.
func (rp revolvePayload) reflected() bool { return rp.xform.IsReflection() }

// axisMoments re-references the plane-origin region integrals into the axis
// frame: q = ∫ρ dA (Pappus's second theorem reads volume off it), mzr =
// ∫zρ dA (the solid centroid's axial position), and mrr = ∫ρ² dA (the
// partial sweep's in-plane centroid term) — the §4 first, second and mixed
// moments with their source and arithmetic bounds (docs/evaluator-design.md
// §6).
func axisMoments(ig regionIntegrals, ax axisFrame) (boundedScalar, boundedScalar, boundedScalar) {
	aU, aV := measuredScalar(ax.aU, ax.aUBound), measuredScalar(ax.aV, ax.aVBound)
	dU, dV := measuredScalar(ax.dU, ax.dUBound), measuredScalar(ax.dV, ax.dVBound)
	nU, nV := measuredScalar(-ax.dV, ax.dVBound), measuredScalar(ax.dU, ax.dUBound)
	area := measuredScalar(ig.area, ig.areaBound)
	mu := measuredScalar(ig.mu, ig.muBound)
	mv := measuredScalar(ig.mv, ig.mvBound)
	muu := measuredScalar(ig.muu, ig.muuBound)
	muv := measuredScalar(ig.muv, ig.muvBound)
	mvv := measuredScalar(ig.mvv, ig.mvvBound)

	iuu := boundedAdd(
		boundedSub(muu, boundedMul(boundedMul(exactScalar(2), aU), mu)),
		boundedMul(boundedMul(aU, aU), area),
	)
	iuv := boundedAdd(
		boundedSub(boundedSub(muv, boundedMul(aU, mv)), boundedMul(aV, mu)),
		boundedMul(boundedMul(aU, aV), area),
	)
	ivv := boundedAdd(
		boundedSub(mvv, boundedMul(boundedMul(exactScalar(2), aV), mv)),
		boundedMul(boundedMul(aV, aV), area),
	)
	q := boundedAdd(
		boundedMul(nU, boundedSub(mu, boundedMul(aU, area))),
		boundedMul(nV, boundedSub(mv, boundedMul(aV, area))),
	)
	mzr := boundedAdd(
		boundedAdd(
			boundedMul(boundedMul(dU, nU), iuu),
			boundedMul(boundedAdd(boundedMul(dU, nV), boundedMul(dV, nU)), iuv),
		),
		boundedMul(boundedMul(dV, nV), ivv),
	)
	mrr := boundedAdd(
		boundedAdd(
			boundedMul(boundedMul(nU, nU), iuu),
			boundedMul(boundedMul(exactScalar(2), boundedMul(nU, nV)), iuv),
		),
		boundedMul(boundedMul(nV, nV), ivv),
	)
	return q, mzr, mrr
}

func boundedRevolveSweep(phi0, phi1 float64) boundedScalar {
	sweep := boundedSub(exactScalar(phi1), exactScalar(phi0))
	sweep.bound = math.Max(sweep.bound, conservativeValueError(sweep.value, twoPiUpper()))
	return sweep
}

// evalRevolve builds the analytic revolved body from the payload: side
// surfaces of revolution per boundary segment, caps only for a partial
// sweep, shared edges and vertices, and bounded mass measurements
// (docs/evaluator-design.md §6). The payload's segment kinds are line, circle
// and arc; anything else has already been rejected by the mass-property
// integrals it runs first.
func evalRevolve(d *Document, ref StepRef, rp revolvePayload) (*Body, error) {
	return evalRevolveWork(d, ref, newFreeformWork(), rp)
}

// evalRevolveWork is the build an operation that already holds this record's
// free-form work counter runs: the preflight below and every walkOf under it
// continue that counter rather than open a second ceiling on the same record.
func evalRevolveWork(d *Document, ref StepRef, work *freeformWork, rp revolvePayload) (*Body, error) {
	return evalRevolveContextWork(context.Background(), d, ref, rp, work)
}

func evalRevolveContext(ctx context.Context, d *Document, ref StepRef, rp revolvePayload) (*Body, error) {
	return evalRevolveContextWork(ctx, d, ref, rp, newFreeformWork())
}

func evalRevolveContextWork(ctx context.Context, d *Document, ref StepRef, rp revolvePayload, work *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := rp.profile.evaluatorIntegralsUncheckedContext(ctx, momentSecondOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}
	sweep := boundedRevolveSweep(rp.phi0, rp.phi1)
	dphi := sweep.value
	if dphi <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}
	q, mzr, mrr := axisMoments(ig, rp.ax)
	if q.value <= 0 {
		return nil, fmt.Errorf(`%w: the region has no material off the revolve axis`, ErrDegenerate)
	}

	b := rp.basis()
	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: "body"}, solid: true}

	// Partial sweeps get two planar cap faces; a full revolution has none.
	var capStart, capEnd *Face
	if !rp.full {
		startFrame, err := rp.capFrame(b, rp.phi0, true)
		if err != nil {
			return nil, err
		}
		endFrame, err := rp.capFrame(b, rp.phi1, false)
		if err != nil {
			return nil, err
		}
		capStart = &Face{
			surface:   Plane{Frame: startFrame},
			origins:   []FeatureRef{{Step: ref, Role: roleCapStart}},
			body:      body,
			area:      ig.area,
			areaBound: ig.areaBound,
		}
		capEnd = &Face{
			surface:   Plane{Frame: endFrame},
			origins:   []FeatureRef{{Step: ref, Role: roleCapEnd}},
			body:      body,
			area:      ig.area,
			areaBound: ig.areaBound,
		}
	}

	sideArea := boundedScalar{}
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	perLoop := make([][]*Face, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		parts, err := buildRevolveLoop(ctx, body, ref, rp, b, li, loop, work)
		if err != nil {
			return nil, err
		}
		perLoop[li] = parts.faces
		sideArea = boundedAdd(sideArea, parts.area)
		if !rp.full {
			capStart.loops = append(capStart.loops, &Loop{coedges: parts.startCo, outer: li == 0})
			capEnd.loops = append(capEnd.loops, &Loop{coedges: parts.endCo, outer: li == 0})
		}
	}

	// Shells: a partial sweep's caps connect every loop's walls into one
	// boundary, but a full revolution encloses each profile hole as its own
	// toroidal void — a separate shell, and a void one (evaluator §3).
	var shells []*Shell
	if rp.full {
		var err error
		shells, err = fullRevolveShellsContext(ctx, perLoop)
		if err != nil {
			return nil, err
		}
	} else {
		var faces []*Face
		for _, group := range perLoop {
			faces = append(faces, group...)
		}
		faces = append(faces, capStart, capEnd)
		if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
			return nil, err
		}
		shells = []*Shell{{faces: faces}}
	}
	body.lumps = []*Lump{{shells: shells}}

	// Measurements — Pappus with the profile and float-evaluation bounds
	// carried through (docs/evaluator-design.md §6).
	area := sideArea
	if !rp.full {
		area = boundedAdd(area, boundedMul(exactScalar(2), measuredScalar(ig.area, ig.areaBound)))
	}
	volume := boundedMul(q, sweep)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.value),
		Exactness: exactnessOf(volume.bound),
		Bound:     units.CubicMillimeters(volume.bound),
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.value),
		Exactness: exactnessOf(area.bound),
		Bound:     units.SquareMillimeters(area.bound),
	}

	axial := boundedDiv(mzr, q)
	cen := b.a3.Add(b.w.Scale(axial.value))
	centroidScale := absSumUpper(vecMaxAbs(b.a3), axial.value)
	centroidBound := absSumUpper(axial.bound, radius3D(analyticRoundBound(centroidScale)))
	if !rp.full {
		// The in-plane term is the swept radial direction integrated over
		// the interval — closed form in the sweep angle; a full turn's is
		// identically zero, which is what puts its centroid on the axis.
		sin1 := boundedSin(exactScalar(rp.phi1))
		cos1 := boundedCos(exactScalar(rp.phi1))
		sin0 := boundedSin(exactScalar(rp.phi0))
		cos0 := boundedCos(exactScalar(rp.phi0))
		rx := boundedSub(sin1, sin0)
		ry := boundedSub(cos0, cos1)
		radial := b.e0.Scale(rx.value).Add(b.e1.Scale(ry.value))
		radialBound := radius2D(rx.bound, ry.bound)
		radialScale := boundedDiv(mrr, boundedMul(sweep, q))
		cen = cen.Add(radial.Scale(radialScale.value))
		radialUpper := vecL1(radial)
		centroidBound = absSumUpper(
			centroidBound,
			productUpper(radialScale.value, radialBound),
			productUpper(radialUpper, radialScale.bound),
			radius3D(analyticRoundBound(productUpper(radialScale.value, radialUpper))),
		)
	}
	centroidBound = absSumUpper(
		centroidBound,
		rigidRoundAllow(vecMaxAbs(cen), vecMaxAbs(rp.xform.Translation())),
	)
	centroidValue := rp.xform.Apply(cen)
	geometryBound, err := revolveCentroidGeometryBound(rp, centroidValue, work)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, geometryBound)
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	bounds, err := revolveBoundsContext(ctx, rp, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = rp
	return body, nil
}

// capFrame is the plane frame of a cap at sweep angle φ, under the
// placement. The start cap's outward (material-leaving) normal points
// against the sweep, the end cap's along it; the axes are ordered so the
// frame's normal IS the outward normal, and a reflected placement swaps them
// once more (a reflection flips the cross product's handedness).
func (rp revolvePayload) capFrame(b revolveBasis, phi float64, start bool) (r3.Frame, error) {
	sin, cos := math.Sincos(phi)
	radial := b.e0.Scale(cos).Add(b.e1.Scale(sin))
	// (w, radial) has w × radial = the rotated plane normal — the sweep
	// velocity direction, the END cap's outward normal.
	u3, v3 := b.w, radial
	if start {
		u3, v3 = v3, u3
	}
	if rp.reflected() {
		u3, v3 = v3, u3
	}
	f, err := r3.NewFrame(rp.xform.Apply(b.a3), rp.xform.ApplyDir(u3), rp.xform.ApplyDir(v3))
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, ErrDegenerate, err)
	}
	return f, nil
}

// revLoopParts is what one recorded loop contributes to the revolved body.
type revLoopParts struct {
	faces   []*Face
	startCo []coedge      // the loop's start-cap coedges, walk order (partial only)
	endCo   []coedge      // the loop's end-cap coedges, walk order (partial only)
	area    boundedScalar // the loop's side-face area
}

// revJunction is one junction between consecutive walks: the shared point in
// axis coordinates and what it sweeps to — nothing on the axis, a full
// latitude circle for a full revolution, an arc between the two caps for a
// partial sweep.
type revJunction struct {
	z, rho float64
	onAxis bool
	v0, v1 *Vertex // partial: start-/end-cap vertices (one shared on the axis)
	arc    *Edge   // partial: the swept junction arc; nil on the axis
	lat    *Edge   // full: the latitude circle; nil on the axis
}

// buildRevolveLoop builds one loop's side faces with shared vertices and
// edges, returning the faces, the two caps' coedges in walk order, and the
// loop's side area.
func buildRevolveLoop(ctx context.Context, body *Body, ref StepRef, rp revolvePayload, b revolveBasis, li int, loop LoopRecord, work *freeformWork) (revLoopParts, error) {
	if err := ctx.Err(); err != nil {
		return revLoopParts{}, err
	}
	if len(loop.Segments) == 0 {
		return revLoopParts{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	raw := make([]sideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return revLoopParts{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return revLoopParts{}, err
		}
		if err := requireAnalyticWalk(w, "the revolve wall build"); err != nil {
			return revLoopParts{}, err
		}
		raw[i] = sideWalk{segmentWalk: rp.ax.walk(w), segs: []int{i}}
	}
	walks, err := coalesceWalksContext(ctx, raw)
	if err != nil {
		return revLoopParts{}, err
	}
	n := len(walks)
	singleClosed := n == 1 && walks[0].closed
	sweep := boundedRevolveSweep(rp.phi0, rp.phi1)
	dphi := sweep.value
	sweepSign := 1.0
	if rp.reflected() {
		sweepSign = -1
	}
	wDir := rp.xform.ApplyDir(b.w)

	kinds := make([]wallKind, n)
	for i, w := range walks {
		kinds[i] = rp.ax.classify(w.segmentWalk)
	}

	// Junction vertices and swept edges: junction i sits at walk i's start
	// (== walk i−1's end). A single whole closed curve has none; a junction
	// on the axis sweeps to a single point — no edge, and for a partial
	// sweep one vertex shared by both caps. Convexity comes from the 2D
	// turn, exactly as the prism's vertical edges: a positive cross of the
	// incoming and outgoing tangents is a convex edge on the outer loop and
	// works out identically for hole loops walked clockwise.
	var js []revJunction
	if !singleClosed {
		js = make([]revJunction, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			j := revJunction{z: w.startU, rho: w.startV, onAxis: w.startV == 0}
			prev := walks[(i+n-1)%n]
			turn := prev.tanOutU*w.tanInV - prev.tanOutV*w.tanInU
			center := rp.point(b, j.z, 0, 0)
			switch {
			case rp.full && !j.onAxis:
				seam := &Vertex{position: rp.point(b, j.z, j.rho, rp.phi0), bound: units.Millimeters(0)}
				latitudeLength := 2 * math.Pi * j.rho
				j.lat = &Edge{
					curve:       Circle3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
					start:       seam,
					end:         seam,
					convex:      turn > 0,
					length:      latitudeLength,
					lengthBound: conservativeValueError(latitudeLength, productUpper(w.axisRadiusUpper, twoPiUpper())),
				}
			case !rp.full:
				j.v0 = &Vertex{position: rp.point(b, j.z, j.rho, rp.phi0), bound: units.Millimeters(0)}
				j.v1 = j.v0
				if !j.onAxis {
					j.v1 = &Vertex{position: rp.point(b, j.z, j.rho, rp.phi1), bound: units.Millimeters(0)}
					arcLength := j.rho * dphi
					dphiUpper := absSumUpper(math.Abs(dphi), sweep.bound)
					j.arc = &Edge{
						curve:       Arc3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
						start:       j.v0,
						end:         j.v1,
						convex:      turn > 0,
						length:      arcLength,
						lengthBound: conservativeValueError(arcLength, productUpper(w.axisRadiusUpper, dphiUpper)),
					}
				}
			}
			js[i] = j
		}
	}

	// Cap edges (partial sweeps only): each walk's copy in each cap plane.
	// A line lying on the axis does not move when swept, so its two copies
	// coincide: ONE edge, shared by both caps — that is how the caps of a
	// solid wedge meet along the axis. Its dihedral is the sweep angle
	// itself.
	holeLoop := li != 0
	var cap0, cap1 []*Edge
	if !rp.full {
		cap0 = make([]*Edge, n)
		cap1 = make([]*Edge, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			if kinds[i] == wallAxis {
				shared := &Edge{
					curve:       Line3{},
					start:       js[i].v0,
					end:         js[(i+1)%n].v0,
					convex:      dphi < math.Pi,
					length:      w.length,
					lengthBound: w.lengthBound,
				}
				cap0[i], cap1[i] = shared, shared
				continue
			}
			var vs0, ve0, vs1, ve1 *Vertex
			if !singleClosed {
				vs0, ve0 = js[i].v0, js[(i+1)%n].v0
				vs1, ve1 = js[i].v1, js[(i+1)%n].v1
			}
			cap0[i] = rp.capEdge(b, w.segmentWalk, singleClosed, vs0, ve0, rp.phi0, holeLoop)
			cap1[i] = rp.capEdge(b, w.segmentWalk, singleClosed, vs1, ve1, rp.phi1, holeLoop)
		}
	}

	parts := revLoopParts{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return revLoopParts{}, err
		}
		if kinds[i] == wallAxis {
			if !rp.full {
				parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
				parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
			}
			continue
		}
		surf, reversed, err := rp.wallSurface(b, w.segmentWalk, kinds[i])
		if err != nil {
			return revLoopParts{}, err
		}
		origins, err := sideOriginsContext(ctx, ref, li, w.segs)
		if err != nil {
			return revLoopParts{}, err
		}
		faceArea := boundedMul(walkAxisMoment(w.segmentWalk, kinds[i]), sweep)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.value,
			areaBound: faceArea.bound,
			reversed:  reversed,
		}
		switch {
		case rp.full && singleClosed:
			// A whole closed generator swept a full turn bounds a closed
			// surface — a torus needs no boundary loop at all.
		case rp.full:
			face.loops = fullRevLoops(js[i], js[(i+1)%n], kinds[i])
		case singleClosed:
			// A closed generator's partial sweep is bounded by its two cap
			// copies — two loops, one whole closed edge each, mirroring the
			// prism's whole-circle two-loop discipline.
			face.loops = []*Loop{
				{coedges: []coedge{{edge: cap0[i], forward: true}}, outer: true},
				{coedges: []coedge{{edge: cap1[i], forward: false}}, outer: true},
			}
		default:
			co := []coedge{{edge: cap0[i], forward: true}}
			if a := js[(i+1)%n].arc; a != nil {
				co = append(co, coedge{edge: a, forward: true})
			}
			co = append(co, coedge{edge: cap1[i], forward: false})
			if a := js[i].arc; a != nil {
				co = append(co, coedge{edge: a, forward: false})
			}
			face.loops = []*Loop{{coedges: co, outer: true}}
		}
		if err := attachFaceLoopsContext(ctx, []*Face{face}); err != nil {
			return revLoopParts{}, err
		}
		parts.faces = append(parts.faces, face)
		parts.area = boundedAdd(parts.area, faceArea)
		if !rp.full {
			parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
			parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
		}
	}
	return parts, nil
}

func fullRevolveShellsContext(ctx context.Context, perLoop [][]*Face) ([]*Shell, error) {
	var shells []*Shell
	for li, group := range perLoop {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if len(group) == 0 {
			continue
		}
		shells = append(shells, &Shell{faces: group, void: li != 0})
	}
	return shells, nil
}

// fullRevLoops assembles a full-revolution side face's boundary loops from
// its junctions' latitude circles — none on the axis (a pole or an apex
// bounds nothing), and for a planar annulus the inner circle is genuinely a
// hole; every other face's circles are both outer, like the prism's whole
// cylinder.
func fullRevLoops(j0, j1 revJunction, kind wallKind) []*Loop {
	outer0, outer1 := true, true
	if kind == wallPlane && !j0.onAxis && !j1.onAxis {
		if j0.rho < j1.rho {
			outer0 = false
		} else {
			outer1 = false
		}
	}
	var loops []*Loop
	if j0.lat != nil {
		loops = append(loops, &Loop{coedges: []coedge{{edge: j0.lat, forward: true}}, outer: outer0})
	}
	if j1.lat != nil {
		loops = append(loops, &Loop{coedges: []coedge{{edge: j1.lat, forward: false}}, outer: outer1})
	}
	if len(loops) == 2 && !loops[0].outer {
		loops[0], loops[1] = loops[1], loops[0]
	}
	return loops
}

// capEdge is one boundary walk's copy in the cap plane at sweep angle φ: a
// straight walk stays a line, a circular one an arc — or a whole circle
// with a seam vertex, closing on itself. The cap edge sits between the cap
// and the side face it bounds; the material across it is a quarter-turn wedge
// everywhere, so the material angle decides nothing — a rim reports the
// WALKED-BOUNDARY convexity instead (topology.go, Edge.IsConvex), which is the
// side the wall's material lies on. A CIRCULAR walk decides that by its own
// sense (a clockwise arc keeps the
// material outside the sphere/torus it sweeps, exactly as wallSurface reads
// it, so its cap edge is concave — a hole's arc and a concave bite in the
// outer boundary alike), while a STRAIGHT walk has no sense of its own and
// takes the loop's: outer convex, hole concave.
func (rp revolvePayload) capEdge(b revolveBasis, w segmentWalk, closed bool, vs, ve *Vertex, phi float64, holeLoop bool) *Edge {
	convex := !holeLoop
	if w.isCircular() {
		convex = w.th0 < w.th1
	}
	e := &Edge{convex: convex, length: w.length, lengthBound: w.lengthBound}
	if !w.isCircular() {
		e.curve = Line3{}
		e.start, e.end = vs, ve
		return e
	}
	// The cap plane at φ is the profile plane rotated about the axis, so
	// its normal is the rotated sweep-velocity direction; the walk's own
	// range order is the arc's CCW sense about it, inverted once by a
	// reflected placement.
	sin, cos := math.Sincos(phi)
	normal := b.e0.Scale(-sin).Add(b.e1.Scale(cos))
	sign := 1.0
	if w.th1 < w.th0 {
		sign = -1
	}
	if rp.reflected() {
		sign = -sign
	}
	axis := rp.xform.ApplyDir(normal).Scale(sign)
	center := rp.point(b, w.cU, w.cV, phi)
	radius := units.Millimeters(w.radius)
	if closed {
		seam := &Vertex{position: rp.point(b, w.startU, w.startV, phi), bound: units.Millimeters(0)}
		e.curve = Circle3{Center: center, Axis: axis, Radius: radius}
		e.start, e.end = seam, seam
		return e
	}
	e.curve = Arc3{Center: center, Axis: axis, Radius: radius}
	e.start, e.end = vs, ve
	return e
}

// wallSurface is the placed surface of revolution one off-axis walk sweeps,
// and whether the face's outward (material-leaving) normal is the surface's
// geometric normal negated. The walk runs with the region's material on its
// left in (z, ρ), so its outward direction is the tangent rotated a quarter
// turn right: (t_ρ, −t_z). A cylinder's and a cone's geometric normal has a
// positive ρ component, so those reverse exactly when the tangent climbs
// (t_z > 0 — a hole wall, or the inner wall of an annular section); a
// sphere's and a torus's geometric normal points away from the generating
// circle's center, which a counter-clockwise walk's outward direction is —
// so those reverse exactly when the walk runs clockwise. Radial directions
// are reflection-equivariant, so a reflected placement changes none of
// this; only the plane frames (built from cross products) correct for it.
func (rp revolvePayload) wallSurface(b revolveBasis, w segmentWalk, kind wallKind) (Surface, bool, error) {
	place := func(z float64) r3.Vec { return rp.xform.Apply(b.a3.Add(b.w.Scale(z))) }
	axis := rp.xform.ApplyDir(b.w)
	switch kind {
	case wallCylinder:
		return Cylinder{
			Origin: place(w.startU),
			Axis:   axis,
			Radius: units.Millimeters((w.startV + w.endV) / 2),
		}, w.tanInU > 0, nil
	case wallPlane:
		// The outward normal is ±axis by the walk's radial heading; the
		// frame's axes are ordered so its normal is outward, swapped once
		// more under a reflected placement.
		u3, v3 := b.e0, b.e1
		if w.tanInV < 0 {
			u3, v3 = v3, u3
		}
		if rp.reflected() {
			u3, v3 = v3, u3
		}
		f, err := r3.NewFrame(place(w.startU), rp.xform.ApplyDir(u3), rp.xform.ApplyDir(v3))
		if err != nil {
			return nil, false, fmt.Errorf(`%w: the placed wall frame is degenerate: %s`, ErrDegenerate, err)
		}
		return Plane{Frame: f}, false, nil
	case wallCone:
		// The apex is where the wall meets the axis; the cone's radius
		// grows along its stored Axis direction.
		dz, dr := w.endU-w.startU, w.endV-w.startV
		apex := w.startU - w.startV*dz/dr
		growth := 1.0
		if dz*dr < 0 {
			growth = -1
		}
		return Cone{
			Origin:    place(apex),
			Axis:      axis.Scale(growth),
			Radius:    units.Millimeters(0),
			HalfAngle: units.Radians(math.Atan2(math.Abs(dr), math.Abs(dz))),
		}, w.tanInU > 0, nil
	case wallSphere:
		return Sphere{
			Center: place(w.cU),
			Radius: units.Millimeters(w.radius),
		}, w.th1 < w.th0, nil
	case wallTorus:
		return Torus{
			Center: place(w.cU),
			Axis:   axis,
			Major:  units.Millimeters(w.cV),
			Minor:  units.Millimeters(w.radius),
		}, w.th1 < w.th0, nil
	default:
		return nil, false, fmt.Errorf(`%w: a wall on the axis sweeps no surface`, ErrDegenerate)
	}
}

// walkAxisMoment is the first moment ∫ρ ds of one boundary walk about the
// axis — Pappus's first theorem reads the side face's area from it:
// a straight walk's is its length times its mean radius (ρ is linear along
// it), a circular walk's is the closed-form antiderivative over its angular
// range, and an on-axis walk sweeps nothing.
func walkAxisMoment(w segmentWalk, kind wallKind) boundedScalar {
	if kind == wallAxis {
		return boundedScalar{}
	}
	if !w.isCircular() {
		meanRadius := boundedDiv(
			boundedAdd(exactScalar(w.startV), exactScalar(w.endV)),
			exactScalar(2),
		)
		result := boundedMul(measuredScalar(w.length, w.lengthBound), meanRadius)
		result.bound = math.Max(result.bound, conservativeValueError(result.value, w.axisMomentUpper))
		return result
	}
	lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
	dtheta := boundedSub(exactScalar(hi), exactScalar(lo))
	cosDelta := boundedSub(boundedCos(exactScalar(lo)), boundedCos(exactScalar(hi)))
	result := boundedMul(
		exactScalar(w.radius),
		boundedAdd(
			boundedMul(exactScalar(w.cV), dtheta),
			boundedMul(exactScalar(w.radius), cosDelta),
		),
	)
	result.bound = math.Max(result.bound, conservativeValueError(result.value, w.axisMomentUpper))
	return result
}

// extentAlong is the revolved solid's exact extent interval along an
// arbitrary world direction g: the swept radial factor's range over the
// angular interval turns the extreme into a linear functional over the
// recorded boundary in (z, ρ) — ρ ≥ 0 over the region, so the solid's
// extreme along g is the extreme of wg·z + m·ρ with m at its own extreme,
// and a linear functional's extreme sits on the boundary. Shared by
// revolveBounds and the through-all stop resolution
// (docs/evaluator-design.md §5/§6).
func (rp revolvePayload) extentAlong(g r3.Vec) (float64, float64, error) {
	return rp.extentAlongContext(context.Background(), g)
}

func (rp revolvePayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return rp.extentAlongWork(ctx, g, newFreeformWork())
}

func (rp revolvePayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, error) {
	b := rp.basis()
	base := rp.xform.Apply(b.a3).Dot(g)
	wg := rp.xform.ApplyDir(b.w).Dot(g)
	mlo, mhi := sweepExtremes(rp.xform.ApplyDir(b.e0).Dot(g), rp.xform.ApplyDir(b.e1).Dot(g), rp.phi0, rp.phi1, rp.full)
	hi, err := axisExtremeContext(ctx, rp, wg, mhi, true, work)
	if err != nil {
		return 0, 0, err
	}
	lo, err := axisExtremeContext(ctx, rp, wg, mlo, false, work)
	if err != nil {
		return 0, 0, err
	}
	return base + lo, base + hi, nil
}

// revolveBoundsContext computes the exact axis-aligned bounds of the placed
// revolved solid — the same directional-extreme analysis the prism uses, in
// cylindrical coordinates (docs/evaluator-design.md §6).
func revolveBoundsContext(ctx context.Context, rp revolvePayload, work *freeformWork) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	for i, g := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, err := rp.extentAlongWork(ctx, g, work)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, nil
}

// sweepExtremes returns the range of m(φ) = c0·cos φ + c1·sin φ over the
// sweep interval: endpoints plus the interior critical angles, or the full
// ±amplitude for a whole turn.
func sweepExtremes(c0, c1, phi0, phi1 float64, full bool) (float64, float64) {
	amp := math.Hypot(c0, c1)
	if full {
		return -amp, amp
	}
	m := func(phi float64) float64 { return c0*math.Cos(phi) + c1*math.Sin(phi) }
	lo := math.Min(m(phi0), m(phi1))
	hi := math.Max(m(phi0), m(phi1))
	if amp == 0 {
		return lo, hi
	}
	star := math.Atan2(c1, c0)
	for _, cand := range []float64{star, star + math.Pi} {
		for k := math.Floor((phi0-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= phi1+1e-12; k += 2 * math.Pi {
			phi := cand + k
			if phi < phi0-1e-12 {
				continue
			}
			lo = math.Min(lo, m(phi))
			hi = math.Max(hi, m(phi))
		}
	}
	return lo, hi
}

// axisExtremeContext is one extreme of the linear functional wg·z + k·ρ over the
// recorded boundary, evaluated in axis coordinates through the plane-local
// boundary extremes.
func axisExtremeContext(ctx context.Context, rp revolvePayload, wg, k float64, wantMax bool, work *freeformWork) (float64, error) {
	gu := wg*rp.ax.dU - k*rp.ax.dV
	gv := wg*rp.ax.dV + k*rp.ax.dU
	lo, hi, err := boundaryExtremesContext(ctx, rp.profile, gu, gv, work)
	if err != nil {
		return 0, err
	}
	off := gu*rp.ax.aU + gv*rp.ax.aV
	if wantMax {
		return hi - off, nil
	}
	return lo - off, nil
}
