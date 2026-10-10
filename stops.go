package decad

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the body-relative stop resolution
// (docs/evaluator-design.md §5/§6/§11, core §8.1): ToFace and
// ToFaceAngular stops at a selected face of a named body, and the
// ThroughAll/ThroughAllSide stops at the far side of every live body the
// sweep meets. Every stop is resolved at the feature call. The evaluator tracks
// named extents first, through-all bodies in stop order, and the axis last,
// deduplicated. A stop body is depended on, never consumed or retired.

// directionalExtent is what a through-all stop needs from a body's payload:
// the body's extent interval along an arbitrary world direction — the
// closed-form intersection of the sweep direction with the body's analytic
// surfaces (docs/evaluator-design.md §5) — BESIDE the proven displacement its
// two endpoints carry. An all-analytic body whose every extreme is exactly
// representable publishes a zero displacement and reads exactly, as it always
// did; a body whose own construction holds an extreme only to a bracket
// publishes that bracket's width and the stop charges it (docs/spline-design.md
// §6.2/§6.4). A payload that cannot state the interval at all, or that holds a
// displacement this reading does not carry, returns ErrUnsupported.
type directionalExtent interface {
	extentAlong(g r3.Vec) (float64, float64, float64, error)
}

// stopTol is the scale-relative tolerance the stop resolutions classify
// contact with: a stop within it of the sketch plane (or of the axis) IS on
// it.
const stopTol = 1e-9

// axialDisplacement is what a body-relative stop reads of a stop body's
// payload: a proven displacement covering its analytic planar stop faces. A
// level derived from such a face inherits it, so a stop cannot launder a held
// coordinate back into an exact one.
type axialDisplacement interface {
	axialDelta() float64
}

// payloadAxialDelta returns the payload's stop-face displacement. The figure a
// linear stop publishes composes this term with its own arithmetic.
func payloadAxialDelta(b *Body) float64 {
	if b == nil {
		return 0
	}
	ad, ok := b.payload.(axialDisplacement)
	if !ok {
		return 0
	}
	return ad.axialDelta()
}

// selectedFaceAxialDelta returns the selected planar stop face's own
// displacement. Payload-wide axialDelta remains for callers that need a bound
// over every face, such as ThroughAll; a ToFace stop names one face and must
// not charge an unrelated cap's bound to it.
func selectedFaceAxialDelta(b *Body, face *Face) float64 {
	if face != nil && face.hasAxialDelta {
		return face.axialDelta
	}
	return payloadAxialDelta(b)
}

// resolveStopBody runs the body gates every body-relative stop shares with
// EdgeAxis (core §8.1): the named body must be live in this document.
func (d *Document) resolveStopBody(b *Body, what string) (*Body, error) {
	if b == nil {
		return nil, fmt.Errorf(`%w: %s names no body to resolve against`, ErrDegenerate, what)
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	return b, nil
}

// selectImpliedOneFace resolves a ToFace, ToFaceAngular or MirrorFace face
// selector against the named body under the implicit exactly-one of core
// §12/§9, reported through the same SelectionError as a direct SelectFaces
// (selection_error.go); what names the caller in the messages. The rule turns
// on one distinction — did the selector's OWN explicit assertion fail? A
// failed explicit assertion (ErrCardinality from SelectFaces) is preserved
// unchanged, its Expected reflecting the caller's own assertion; the implicit
// exactly-one owns every other outcome that is not a single match — an
// unasserted zero (ErrNoMatch) or a successful resolution whose count is not
// one — rewriting it to a SelectionError wrapping ErrCardinality with Expected
// "exactly 1" and Actual the resolved count. A typed nil query is as empty a
// selector as an untyped nil: malformed input (errNilSelector, branchable
// ErrDegenerate), never a staged resolution.
//
// decad owns the selector vocabulary, so the only valid FaceSelector is the
// built-in *FaceQuery the constructors return. A FaceSelector whose dynamic
// type is anything else — a foreign implementation, including one that embeds
// *FaceQuery to promote the sealed selector() marker — is malformed input,
// not a resolvable query: it is rejected as ErrDegenerate before SelectFaces
// runs, so every count-not-one that reaches impliedOneFace is a concrete query
// and cannot miss its SelectionError.
func selectImpliedOneFace(body *Body, sel FaceSelector, what string) (*Face, error) {
	q, err := builtinFaceQuery(sel, what)
	if err != nil {
		return nil, err
	}
	faces, err := q.SelectFaces(body)
	if err != nil {
		// The selector's own explicit assertion failed: SelectFaces already
		// returned its SelectionError, and the caller gets it unchanged.
		if errors.Is(err, ErrCardinality) {
			return nil, err
		}
		// An unasserted resolution that matched nothing: the implicit
		// exactly-one rewrites it to ErrCardinality, Expected "exactly 1".
		if errors.Is(err, ErrNoMatch) {
			return nil, impliedOneFace(body, q, 0)
		}
		return nil, err
	}
	if len(faces) != 1 {
		// A successful resolution the implicit exactly-one turns into
		// Expected "exactly 1" / Actual len(faces).
		return nil, impliedOneFace(body, q, len(faces))
	}
	return faces[0], nil
}

// builtinFaceQuery gates a selector a body-relative reference names to the
// built-in *FaceQuery: a nil selector, a typed nil query and a foreign
// implementation are each malformed input (ErrDegenerate).
func builtinFaceQuery(sel FaceSelector, what string) (*FaceQuery, error) {
	q, ok := sel.(*FaceQuery)
	switch {
	case sel == nil:
		return nil, fmt.Errorf(`%w: %s names no face selector`, ErrDegenerate, what)
	case !ok:
		return nil, fmt.Errorf(`%w: %s's face selector is not a decad face query (%T)`, ErrDegenerate, what, sel)
	case q == nil:
		return nil, errNilSelector
	}
	return q, nil
}

// resolveToFace resolves a linear to-face stop into the signed stop
// coordinate along the sketch plane normal, beside that coordinate's own proven
// axial displacement. The level is COMPUTED — from another body's face, in
// float — so the displacement is what keeps it from publishing itself as the
// level it denotes: it composes this resolution's own rounding, the offset's
// conversion, and whatever the stop body proved about the level its face sits
// at. The resolved face must be usable
// as a prism cap — the §5 build table's stop is the closed-form intersection
// of the sweep direction with the target surface, and the cap it emits is
// planar and perpendicular to the sweep — so the face must be PLANAR with
// its normal parallel to the sweep direction; a non-planar stop face, or a
// planar one not perpendicular to the sweep, is a solid this evaluator
// cannot build (ErrUnsupported, staged). A face coplanar with the sketch
// plane stops the sweep before it starts (ErrDegenerate).
//
// travel is 0 for a standalone ToFace — the target face supplies the sense
// (core §8.1): the sweep runs toward whichever side the face is on — and ±1
// for a TwoSided side, whose face must lie on that side. The signed Offset
// displaces the stop along the travel: positive overshoots the face,
// negative stops short of it (core §8.1).
func (d *Document) resolveToFace(tf ToFace, frame r3.Frame, travel float64, what string) (float64, float64, producerID, error) {
	body, err := d.resolveStopBody(tf.Body, what)
	if err != nil {
		return 0, 0, 0, err
	}
	offset, offsetDelta, err := extent.DisplacementIn(tf.Offset, units.Length, units.Millimeter, "the to-face offset")
	if err != nil {
		return 0, 0, 0, err
	}
	face, err := selectImpliedOneFace(body, tf.Face, "the stop")
	if err != nil {
		return 0, 0, 0, err
	}
	pl, ok := face.Surface().(Plane)
	if !ok {
		// The gate is an ANALYTIC planar surface (a Plane — an extrude/revolve
		// cap), not merely a flat face: a boolean-built face can be flat
		// (isPlanar) yet carry a Faceted surface, and this evaluator cannot use
		// it as a stop. %T reports the actual surface so the caller sees why.
		return 0, 0, 0, fmt.Errorf(`%w: ToFace requires a stop face with an analytic Plane surface whose normal is parallel to the sweep direction, not merely a flat face; this face's surface is %T, which this evaluator cannot use as a stop even when flat — choose any analytic planar face with that orientation, or use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, ErrUnsupported, face.Surface())
	}
	n := frame.N()
	if !parallelDirs(pl.Frame.N(), n) {
		return 0, 0, 0, fmt.Errorf(`%w: ToFace requires the stop face's normal parallel to the sweep direction (the face perpendicular to the sweep); this face is tilted — choose a perpendicular face or use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, ErrUnsupported)
	}
	zFace := pl.Frame.Origin().Sub(frame.Origin()).Dot(n)
	tol := extent.RelativeStopTolerance(math.Max(pl.Frame.Origin().Len(), frame.Origin().Len()))
	if math.Abs(zFace) <= tol {
		return 0, 0, 0, fmt.Errorf(`%w: the stop face is coplanar with the sketch plane`, ErrDegenerate)
	}
	if travel == 0 {
		travel = 1
		if zFace < 0 {
			travel = -1
		}
	} else if zFace*travel < 0 {
		return 0, 0, 0, fmt.Errorf(`%w: the stop face lies on the other side of the sketch plane than %s sweeps`, ErrDegenerate, what)
	}
	stop := zFace + travel*offset
	if stop*travel <= tol {
		return 0, 0, 0, fmt.Errorf(`%w: the to-face offset pulls the stop behind the sketch plane`, ErrDegenerate)
	}
	// The three terms displace the level along the same normal, so they sum:
	// the whole expression's own rounding (which covers the difference, the dot
	// product and the offset step alike), the offset's conversion into
	// millimetres, and the displacement the stop body proved for its own levels.
	delta := proofbound.AbsSumUpper(
		extent.StopLevelRound(pl.Frame.Origin(), frame.Origin(), n, travel, offset, stop),
		offsetDelta,
		selectedFaceAxialDelta(body, face),
	)
	return stop, delta, body.originProducer(), nil
}

// resolveThroughAll resolves a through-all stop: the sweep runs from the
// sketch plane in the travel sense through the far side of every live body
// it meets, so the stop is the farthest far side among them
// (docs/evaluator-design.md §5). A body is met when it has material strictly
// beyond the sketch plane in the travel sense, judged on the body's own
// recorded payload — its directional extent along the sweep direction, beside
// the displacement that reading publishes; the sweep's lateral footprint is
// not consulted (an exact region-versus-projection overlap is boolean-grade
// machinery this evaluator does not have, and a conservative guess would
// fabricate or drop a recorded dependency). The in-path test is decided
// OUTSIDE that displacement — beyond the plane by more than it, or short of
// the plane by more than it — and an interval that straddles the plane in the
// travel sense refuses ErrUnsupported (docs/spline-design.md §6.4) rather than
// guess a dependency. It returns the signed stop coordinate along the plane
// normal, that coordinate's own proven axial displacement — which carries the
// winning body's extent displacement, so a level held to a bracket never
// publishes itself as the level it denotes — and the met bodies' private producer identities in
// stop order along the sweep, nearest far side first. No live body in the path
// is ErrDegenerate: the sweep has no stop at all.
func (d *Document) resolveThroughAll(frame r3.Frame, travel float64) (float64, float64, []producerID, error) {
	dir := frame.N().Scale(travel)
	stops := extent.NewThroughStops[producerID](frame.Origin(), dir, travel)
	for _, b := range d.liveBodies() {
		// A sheet encloses no material to stop a sweep.
		if b.Kind() == BodySheet {
			continue
		}
		ext, ok := b.payload.(directionalExtent)
		if !ok {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all stop needs every live body's directional extent, and this evaluator did not build one of them`, ErrUnsupported)
		}
		_, hi, bound, err := ext.extentAlong(dir)
		if err != nil {
			return 0, 0, nil, err
		}
		if err := stops.Add(hi, bound, payloadAxialDelta(b), b.originProducer()); err != nil {
			return 0, 0, nil, err
		}
	}
	return stops.Finish()
}

// dedupRefs deduplicates recorded stop refs preserving first occurrence
// returning nil when there are no dependencies
// no Inputs at all.
func dedupRefs(refs []producerID) []producerID {
	if len(refs) == 0 {
		return nil
	}
	seen := make(map[producerID]struct{}, len(refs))
	out := make([]producerID, 0, len(refs))
	for _, r := range refs {
		if _, ok := seen[r]; ok {
			continue
		}
		seen[r] = struct{}{}
		out = append(out, r)
	}
	return out
}

// angularStops is what a ToFaceAngular resolution needs of the revolve's
// resolved axis: a point on the axis, the CALLER's unit axis direction (the
// sense Along is right-handed about — the sweep interval is resolved in the
// caller's frame and remapped by the side sign afterward, like every other
// angular extent), and the region's own half-plane direction r0 (the sweep
// angle's zero), with e1 = w × r0 the right-handed angle velocity at zero.
type angularStops struct {
	d  *Document
	a3 r3.Vec
	w  r3.Vec
	r0 r3.Vec
	e1 r3.Vec
}

// angularStopCtx derives the stop context from the resolved plane frame, the
// plane-local axis line (caller sense) and the oriented axis frame (region
// on its non-negative side).
func (d *Document) angularStopCtx(frame r3.Frame, line revolveaxis.Line2, ax axisFrame) angularStops {
	a3 := frame.ToWorldUV(line.AU, line.AV)
	w := frame.U().Scale(line.DU).Add(frame.V().Scale(line.DV))
	r0 := frame.U().Scale(-ax.dV).Add(frame.V().Scale(ax.dU))
	return angularStops{d: d, a3: a3, w: w, r0: r0, e1: w.Cross(r0)}
}

// angTol decides angular agreement between a stop face's boundary points: a
// radial plane's points all sit at one angle about the axis, up to round-off.
const angTol = 1e-9

// resolveToFaceAngular resolves an angular to-face stop into the signed stop
// angle about the axis, caller frame, radians. The resolved face must be
// usable as a revolve cap — the §6 build table's caps are planar faces in
// the rotated profile plane, which CONTAINS the axis — so the face must be
// PLANAR, its plane must contain the revolve axis, and its material must lie
// in one half-plane of the axis; a non-planar face, or a plane that does not
// contain the axis, is a solid this evaluator cannot build
// (ErrUnsupported, staged), while a face spanning both half-planes names two
// stops at once and one in the profile's own half-plane a zero sweep — both
// ErrDegenerate.
//
// travel is 0 for a standalone ToFaceAngular — the target face supplies the
// sense (core §8.1), and both senses reach a radial plane, so the sweep
// takes the nearer way around (Along on the exact half-turn tie) — and ±1
// for a TwoSidedAngle side.
func (st angularStops) resolveToFaceAngular(tfa ToFaceAngular, travel float64, what string) (float64, producerID, error) {
	body, err := st.d.resolveStopBody(tfa.Body, what)
	if err != nil {
		return 0, 0, err
	}
	face, err := selectImpliedOneFace(body, tfa.Face, "the stop")
	if err != nil {
		return 0, 0, err
	}
	pl, ok := face.Surface().(Plane)
	if !ok {
		// The gate is an ANALYTIC planar surface (a Plane — a revolve radial
		// cap), not merely a flat face: a boolean-built face can be flat
		// (isPlanar) yet carry a Faceted surface, and this evaluator cannot use
		// it as a stop. %T reports the actual surface so the caller sees why.
		return 0, 0, fmt.Errorf(`%w: ToFaceAngular requires a stop face with an analytic Plane surface whose plane contains the revolve axis, not merely a flat face; this face's surface is %T, which this evaluator cannot use as a stop even when flat — choose any analytic planar face whose plane contains the axis, or use an angle extent (AngleExtent, or AngleSide inside a TwoSidedAngle extent)`, ErrUnsupported, face.Surface())
	}
	nf := pl.Frame.N()
	if math.Abs(nf.Dot(st.w)) > stopTol {
		return 0, 0, fmt.Errorf(`%w: ToFaceAngular requires the stop face's plane to contain the revolve axis; this plane is not parallel to the axis — choose a radial face or use an angle extent (AngleExtent, or AngleSide inside a TwoSidedAngle extent)`, ErrUnsupported)
	}
	off := pl.Frame.Origin().Sub(st.a3).Dot(nf)
	if math.Abs(off) > extent.RelativeStopTolerance(math.Max(pl.Frame.Origin().Len(), st.a3.Len())) {
		return 0, 0, fmt.Errorf(`%w: ToFaceAngular requires the stop face's plane to contain the revolve axis; this plane runs parallel to the axis but offset from it — choose a face through the axis or use an angle extent (AngleExtent, or AngleSide inside a TwoSidedAngle extent)`, ErrUnsupported)
	}
	phi, err := st.faceHalfPlane(face)
	if err != nil {
		return 0, 0, err
	}
	if phi <= angTol || phi >= 2*math.Pi-angTol {
		return 0, 0, fmt.Errorf(`%w: the stop face lies in the profile's own half-plane`, ErrDegenerate)
	}
	switch {
	case travel > 0:
		return phi, body.originProducer(), nil
	case travel < 0:
		return phi - 2*math.Pi, body.originProducer(), nil
	case phi <= math.Pi:
		return phi, body.originProducer(), nil
	default:
		return phi - 2*math.Pi, body.originProducer(), nil
	}
}

// faceHalfPlane maps source edges to the geometry the stop evaluator reads.
func (st angularStops) faceHalfPlane(face *Face) (float64, error) {
	edges := face.Edges()
	boundary := make([]extent.AngularEdge, len(edges))
	for i, edge := range edges {
		boundary[i] = angularStopEdge(edge)
	}
	return extent.FaceHalfPlane(st.stopGeometry(), boundary)
}

func (st angularStops) stopGeometry() extent.AngularStop {
	return extent.AngularStop{A3: st.a3, R0: st.r0, E1: st.e1}
}

func angularStopEdge(e *Edge) extent.AngularEdge {
	curve := e.Curve()
	record := extent.AngularEdge{CurveType: fmt.Sprintf("%T", curve)}
	if e.start != nil {
		record.Start = &e.start.position
	}
	if e.end != nil {
		record.End = &e.end.position
	}
	switch c := curve.(type) {
	case Line3:
		record.Kind = extent.AngularEdgeLine
	case Circle3:
		record.Kind, record.Center, record.Axis, record.Radius =
			extent.AngularEdgeCircle, c.Center, c.Axis, c.Radius
	case Arc3:
		record.Kind, record.Center, record.Axis, record.Radius =
			extent.AngularEdgeArc, c.Center, c.Axis, c.Radius
	}
	return record
}
