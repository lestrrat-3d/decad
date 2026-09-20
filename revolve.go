package decad

import (
	"context"
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
// ToFaceAngular stop resolves through stops.go and leaves its body live), and the analytic revolve
// evaluator — Cylinder, Cone, planar-annulus, Sphere and Torus side faces
// per boundary segment kind, caps on partial sweeps (both closing the solid
// and, under WithSurfaceResult, omitted from a sheet), and Pappus
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
// plane's own (u, v), millimetres — the §5.2 coordinate carve-out. The axis
// direction, which Along is right-handed
// about, runs from Start toward End. The evaluator lifts it into world space
// through the call's PlaneRecord, so it is coplanar with the profile
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
// must be a live body of the same document at the call and is not consumed or
// retired. The axis
// runs from the resolved edge's start vertex toward its end vertex, the
// sense Along is right-handed about.
type EdgeAxis struct {
	Body *Body
	Edge EdgeSelector
}

// The sealed set.
func (SketchLine) axis()       {}
func (ConstructionAxis) axis() {}
func (EdgeAxis) axis()         {}

// errNilAxis rejects a nil variant pointer: it names no axis to spin about.
// It wraps ErrDegenerate so a typed nil pointer is branchable exactly like
// an untyped nil axis.
var errNilAxis = fmt.Errorf(`%w: nil axis`, ErrDegenerate)

// normalizeAxis returns the value form of a: the variants seal with value
// receivers, so a *SketchLine satisfies Axis as readily as a SketchLine does
// and the call copies the value the pointer names. A nil
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

// RevolveOption configures Revolve. WithSurfaceResult (docs/surface-design.md
// §4) is the one option today: it omits the two caps a partial sweep would
// otherwise close with and publishes a sheet instead of a solid. A full
// revolution mints no closing face to omit, so there the option changes no
// face and returns a CLOSED sheet with no free edge — §2.1's closed-sheet
// rule, not a refusal (Table W). A repeated WithSurfaceResult() is
// idempotent, never an error.
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
// (docs/evaluator-design.md §6). The evaluator converts the profile and plane
// to structural records; a failed evaluation leaves the document untouched.
// WithSurfaceResult() omits the two caps a partial sweep closes with and
// publishes a sheet instead of a solid; a full revolution already closes with
// no cap to omit, so the option there yields a closed sheet rather than a
// refusal (docs/surface-design.md §4, Table W).
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
	surfaceResult := false
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		// RevolveOption carries exactly one variant today, so this is an if
		// rather than a type switch on o.Ident() — gocritic's singleCaseSwitch
		// flags a single-case switch, and a second RevolveOption is when this
		// grows one. A repeated WithSurfaceResult() is idempotent, the same
		// tolerance Extrude gives its own repeat (extrude.go).
		if _, ok := o.Ident().(identSurfaceResult); ok {
			surfaceResult = true
		}
	}

	axis, err = normalizeAxis(axis)
	if err != nil {
		return nil, err
	}
	// The evaluator spins about a resolved line; for an EdgeAxis that line
	// comes from the one edge the selector names. The named body stays live.
	evalAxis := axis
	if ea, ok := axis.(EdgeAxis); ok {
		line, err := d.resolveEdgeAxis(ea)
		if err != nil {
			return nil, err
		}
		evalAxis = line
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
	ax, side, err := resolveAxisSide(context.Background(), profile, line, work)
	if err != nil {
		return nil, err
	}
	// The angular extent resolves in the caller's frame — a ToFaceAngular
	// stop needs the resolved axis, which is why the axis gates run first.
	phi0, phi1, full, den, _, err := d.resolveAngularExtent(a, d.angularStopCtx(frame, line, ax))
	if err != nil {
		return nil, err
	}
	if side < 0 {
		// The axis frame was flipped to put the region on its non-negative
		// side; a rotation by φ about the given axis is a rotation by −φ
		// about the flipped one, so the interval flips with it — and so does
		// what it denotes, exactly: negate and swap both ends.
		phi0, phi1 = -phi1, -phi0
		den.phi0, den.phi1 = den.phi1.neg(), den.phi0.neg()
	}

	ref := d.nextProducerID()
	body, err := evalRevolveWork(d, ref, work, revolvePayload{
		profile:       profile,
		frame:         frame,
		ax:            ax,
		phi0:          phi0,
		phi1:          phi1,
		full:          full,
		den:           den,
		xform:         r3.Identity(),
		surfaceResult: surfaceResult,
	})
	if err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// resolveEdgeAxis runs the EdgeAxis gates and resolves the named edge into
// the construction axis the evaluator spins about: the named body must be a
// live body of this document, and the selector must resolve to exactly one
// LINEAR edge of it — any other count is ErrCardinality, zero included (the
// implicit exactly-one of core §12 takes precedence over ErrNoMatch), and a
// non-linear edge named as a revolve axis is ErrDegenerate. The axis runs
// from the edge's start vertex toward its end vertex. It returns the derived
// axis and the private producer identity used to track its dependency.
func (d *Document) resolveEdgeAxis(ea EdgeAxis) (ConstructionAxis, error) {
	body := ea.Body
	if body == nil {
		return ConstructionAxis{}, fmt.Errorf(`%w: an edge axis names no body to resolve against`, ErrDegenerate)
	}
	if err := d.requireLive(body); err != nil {
		return ConstructionAxis{}, err
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
		return ConstructionAxis{}, fmt.Errorf(`%w: an edge axis names no edge selector`, ErrDegenerate)
	case !ok:
		return ConstructionAxis{}, fmt.Errorf(`%w: an edge axis's edge selector is not a decad edge query (%T)`, ErrDegenerate, ea.Edge)
	case q == nil:
		return ConstructionAxis{}, errNilSelector
	}
	edges, err := q.SelectEdges(body)
	if err != nil {
		// The selector's own explicit assertion failed: SelectEdges already
		// returned its SelectionError, and the caller gets it unchanged.
		if errors.Is(err, ErrCardinality) {
			return ConstructionAxis{}, err
		}
		// An unasserted resolution that matched nothing: the implicit
		// exactly-one rewrites it to ErrCardinality, Expected "exactly 1".
		if errors.Is(err, ErrNoMatch) {
			return ConstructionAxis{}, impliedOneEdge(body, q, 0)
		}
		return ConstructionAxis{}, err
	}
	if len(edges) != 1 {
		// A successful resolution the implicit exactly-one turns into
		// Expected "exactly 1" / Actual len(edges).
		return ConstructionAxis{}, impliedOneEdge(body, q, len(edges))
	}
	e := edges[0]
	if _, ok := e.Curve().(Line3); !ok {
		return ConstructionAxis{}, fmt.Errorf(`%w: a non-linear edge named as a revolve axis spins about no line`, ErrDegenerate)
	}
	dir := e.end.position.Sub(e.start.position)
	if zeroVec(dir) {
		return ConstructionAxis{}, fmt.Errorf(`%w: a zero-length edge names no axis`, ErrDegenerate)
	}
	return ConstructionAxis{Origin: e.start.position, Dir: dir}, nil
}

// angFullEps separates "a full turn" from "past a full turn": an angular
// magnitude that lands on 2π within floating-point conversion noise IS a
// full revolution — the two caps would coincide — and one beyond it sweeps
// a self-overlapping solid, which is rejected.
const angFullEps = 1e-12

// resolveAngularExtent turns an angular extent into the signed sweep
// interval [phi0, phi1] about the axis, radians, Along positive
// (docs/evaluator-design.md §6), reports whether the sweep is a full
// revolution, and returns the exact angle each end DENOTES (§6, the angular
// twin of extrude's axial displacement) beside the private producer identities
// of the bodies the extent's stops resolved against — in extent order,
// deduplicated with the axis ref by the caller. The full-turn snap below never touches
// the denotation: the record still states what the caller said, and the
// displacement between the two is what every consumer charges, never a
// reinterpretation of the record. Magnitudes are validated per core §8.1/§12;
// a zero-angle sweep is ErrDegenerate, as is one past a full turn.
func (d *Document) resolveAngularExtent(a AngularExtent, st angularStops) (float64, float64, bool, sweepDenotation, []producerID, error) {
	var phi0, phi1 float64
	var den sweepDenotation
	var refs []producerID
	full := false
	switch a := a.(type) {
	case AngleExtent:
		m, err := magnitudeIn(a.A, units.Angle, units.Radian, "the extent angle")
		if err != nil {
			return 0, 0, false, sweepDenotation{}, nil, err
		}
		if m == 0 {
			return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: a zero-angle extent sweeps no solid`, ErrDegenerate)
		}
		stated := angleDenotationFromValue(a.A)
		// An unknown Direction is malformed input, never silently Along.
		switch a.Dir {
		case Along:
			phi0, phi1 = 0, m
			den = sweepDenotation{phi0: zeroAngleDenotation(), phi1: stated}
		case Against:
			phi0, phi1 = -m, 0
			den = sweepDenotation{phi0: stated.neg(), phi1: zeroAngleDenotation()}
		default:
			return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: unknown direction %d`, ErrDegenerate, int(a.Dir))
		}
	case FullRevolution:
		fullDen := sweepDenotation{phi0: zeroAngleDenotation(), phi1: angleDenotation{rad: new(big.Rat), turn: big.NewRat(1, 1)}}
		return 0, 2 * math.Pi, true, fullDen, nil, nil
	case SymmetricAngle:
		m, err := magnitudeIn(a.A, units.Angle, units.Radian, "the symmetric angle")
		if err != nil {
			return 0, 0, false, sweepDenotation{}, nil, err
		}
		if m == 0 {
			return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: a zero-angle extent sweeps no solid`, ErrDegenerate)
		}
		half := m
		stated := angleDenotationFromValue(a.A)
		if a.FullLength {
			half = m / 2
			stated = stated.scale(big.NewRat(1, 2))
		}
		phi0, phi1 = -half, half
		den = sweepDenotation{phi0: stated.neg(), phi1: stated}
	case TwoSidedAngle:
		along, alongDen, oneRefs, err := d.resolveAngleSide(a.One, st, 1, "the along side")
		if err != nil {
			return 0, 0, false, sweepDenotation{}, nil, err
		}
		against, againstDen, twoRefs, err := d.resolveAngleSide(a.Two, st, -1, "the against side")
		if err != nil {
			return 0, 0, false, sweepDenotation{}, nil, err
		}
		if along == 0 && against == 0 {
			return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: a zero-angle extent sweeps no solid`, ErrDegenerate)
		}
		phi0, phi1 = against, along
		// Each side denotes its own end independently; a side this evaluator
		// cannot denote exactly (a ToFaceAngular stop) leaves the WHOLE
		// sweep's denotation nil, never half of it — a nil end mixed with a
		// stated one would let one end's zero charge stand for a sweep whose
		// other end the resolver cannot prove at all.
		if againstDen.valid() && alongDen.valid() {
			den = sweepDenotation{phi0: againstDen, phi1: alongDen}
		}
		refs = append(oneRefs, twoRefs...)
	case ToFaceAngular:
		stop, ref, err := st.resolveToFaceAngular(a, 0, "a to-face extent")
		if err != nil {
			return 0, 0, false, sweepDenotation{}, nil, err
		}
		refs = []producerID{ref}
		if stop > 0 {
			phi0, phi1 = 0, stop
		} else {
			phi0, phi1 = stop, 0
		}
	case nil:
		return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: a nil extent sweeps nothing`, ErrDegenerate)
	default:
		return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: angular extent %T is not supported by this evaluator`, ErrUnsupported, a)
	}
	total := phi1 - phi0
	if total > 2*math.Pi+angFullEps {
		return 0, 0, false, sweepDenotation{}, nil, fmt.Errorf(`%w: a sweep past a full turn overlaps itself`, ErrDegenerate)
	}
	if total >= 2*math.Pi-angFullEps {
		full = true
		phi1 = phi0 + 2*math.Pi
	}
	return phi0, phi1, full, den, refs, nil
}

// resolveAngleSide resolves one side of a TwoSidedAngle to its signed
// boundary angle and the exact angle it denotes (§6); travel is +1 for the
// along side, −1 for the against side, and the denotation is scaled by the
// same sign — exact, since travel is always ±1.
func (d *Document) resolveAngleSide(s SideAngular, st angularStops, travel float64, what string) (float64, angleDenotation, []producerID, error) {
	s, err := normalizeSideAngular(s)
	if err != nil {
		return 0, angleDenotation{}, nil, err
	}
	switch s := s.(type) {
	case AngleSide:
		m, err := magnitudeIn(s.A, units.Angle, units.Radian, what)
		if err != nil {
			return 0, angleDenotation{}, nil, err
		}
		travelR := big.NewRat(1, 1)
		if travel < 0 {
			travelR = big.NewRat(-1, 1)
		}
		return travel * m, angleDenotationFromValue(s.A).scale(travelR), nil, nil
	case ToFaceAngular:
		stop, ref, err := st.resolveToFaceAngular(s, travel, what)
		if err != nil {
			return 0, angleDenotation{}, nil, err
		}
		return stop, angleDenotation{}, []producerID{ref}, nil
	case nil:
		return 0, angleDenotation{}, nil, fmt.Errorf(`%w: a two-sided extent requires both sides`, ErrDegenerate)
	default:
		return 0, angleDenotation{}, nil, fmt.Errorf(`%w: side angular %T is not supported by this evaluator`, ErrUnsupported, s)
	}
}
