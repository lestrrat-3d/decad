package decad

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"

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
	ExtentAlong(g r3.Vec) (float64, float64, float64, error)
}

// axialDisplacement is what a body-relative stop reads of a stop body's
// payload: a proven displacement covering its analytic planar stop faces. A
// level derived from such a face inherits it, so a stop cannot launder a held
// coordinate back into an exact one.
type axialDisplacement interface {
	AxialDelta() float64
}

// payloadAxialDelta returns the payload's stop-face displacement. The figure a
// linear stop publishes composes this term with its own arithmetic.
func payloadAxialDelta(b *Body) float64 {
	if b == nil {
		return 0
	}
	ad, ok := b.n.Payload.(axialDisplacement)
	if !ok {
		return 0
	}
	return ad.AxialDelta()
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
// coordinate along the sketch plane normal, beside that coordinate's own
// proven axial displacement, and the stop body's producer. It resolves the
// live stop body, converts the Offset and selects exactly one face through
// root handles; extent.ToFaceStop then admits the face as a prism cap (§5
// build table) and computes the level and its displacement. travel is 0 for
// a standalone ToFace, whose target face supplies the sense, and ±1 for a
// TwoSided side (core §8.1).
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
	stop, delta, err := extent.ToFaceStop(faceNode(face), frame, travel, offset, offsetDelta, what)
	if err != nil {
		return 0, 0, 0, err
	}
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
		ext, ok := b.n.Payload.(directionalExtent)
		if !ok {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all stop needs every live body's directional extent, and this evaluator did not build one of them`, ErrUnsupported)
		}
		_, hi, bound, err := ext.ExtentAlong(dir)
		if err != nil {
			return 0, 0, nil, err
		}
		if err := stops.Add(hi, bound, payloadAxialDelta(b), b.originProducer()); err != nil {
			return 0, 0, nil, err
		}
	}
	return stops.Finish()
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
	r0 := frame.U().Scale(-ax.DV).Add(frame.V().Scale(ax.DU))
	return angularStops{d: d, a3: a3, w: w, r0: r0, e1: w.Cross(r0)}
}

// resolveToFaceAngular resolves an angular to-face stop into the signed stop
// angle about the axis, caller frame, radians. It selects the live stop body
// and its one face; extent.ToFaceAngularStop reads the face. travel is 0 for a
// standalone ToFaceAngular and ±1 for a TwoSidedAngle side.
func (st angularStops) resolveToFaceAngular(tfa ToFaceAngular, travel float64, what string) (float64, producerID, error) {
	body, err := st.d.resolveStopBody(tfa.Body, what)
	if err != nil {
		return 0, 0, err
	}
	face, err := selectImpliedOneFace(body, tfa.Face, "the stop")
	if err != nil {
		return 0, 0, err
	}
	stop, err := extent.ToFaceAngularStop(faceNode(face), st.stopGeometry(), travel)
	if err != nil {
		return 0, 0, err
	}
	return stop, body.originProducer(), nil
}

func (st angularStops) stopGeometry() extent.AngularStop {
	return extent.AngularStop{A3: st.a3, W: st.w, R0: st.r0, E1: st.e1}
}
