package decad

import (
	"cmp"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the body-relative stop resolution
// (docs/evaluator-design.md §5/§6/§11, core §8.1/§6.2): ToFace and
// ToFaceAngular stops at a selected face of a named body, and the
// ThroughAll/ThroughAllSide stops at the far side of every live body the
// sweep meets. Every stop is resolved at the feature call — the dependency
// is ambient at the CALL but never in the RECORD — and each stop body's
// StepRef is recorded in the step's Inputs: named-extent refs in extent
// order first, through-all stop bodies after them in stop order along the
// sweep, the axis ref last, deduplicated (core §6.2). A stop body is
// depended on, never consumed and never retired.

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

// stopLevelRound proves a to-face level's own float rounding: the level the
// resolution HELD, against the value its expression — (faceOrigin − planeOrigin)
// · n + travel·offset — takes exactly over the same inputs, every one of them a
// float64 and so an exact rational. It measures the arithmetic this resolution
// performs and nothing else: n is the normal the payload itself lifts along, so
// reading the level in that direction re-derives no coordinate.
func stopLevelRound(faceOrigin, planeOrigin, n r3.Vec, travel, offset, held float64) float64 {
	face := [3]float64{faceOrigin.X, faceOrigin.Y, faceOrigin.Z}
	plane := [3]float64{planeOrigin.X, planeOrigin.Y, planeOrigin.Z}
	normal := [3]float64{n.X, n.Y, n.Z}
	terms := make([]*big.Rat, 0, 4)
	for i := range face {
		f, p, nn := floatRat(face[i]), floatRat(plane[i]), floatRat(normal[i])
		if f == nil || p == nil || nn == nil {
			return math.Inf(1)
		}
		terms = append(terms, ratMul(new(big.Rat).Sub(f, p), nn))
	}
	t, o := floatRat(travel), floatRat(offset)
	if t == nil || o == nil {
		return math.Inf(1)
	}
	terms = append(terms, ratMul(t, o))
	return rationalFloatError(ratAdd(terms...), held)
}

// throughStopRound is stopLevelRound's through-all analogue: travel·(hi −
// origin·dir) taken exactly over the far side the winning stop body reported
// and the frame this sweep lifts through, against the float the resolution
// held. hi arrives from the body's own extentAlong, whose own displacement the
// resolution charges beside this term, so what this term adds is the
// subtraction and the sign.
func throughStopRound(origin, dir r3.Vec, hi, travel, held float64) float64 {
	o := [3]float64{origin.X, origin.Y, origin.Z}
	g := [3]float64{dir.X, dir.Y, dir.Z}
	base := new(big.Rat)
	for i := range o {
		oi, gi := floatRat(o[i]), floatRat(g[i])
		if oi == nil || gi == nil {
			return math.Inf(1)
		}
		base.Add(base, ratMul(oi, gi))
	}
	h, t := floatRat(hi), floatRat(travel)
	if h == nil || t == nil {
		return math.Inf(1)
	}
	return rationalFloatError(new(big.Rat).Mul(t, new(big.Rat).Sub(h, base)), held)
}

// relStopTol is stopTol scaled to the magnitudes in play.
func relStopTol(scale float64) float64 {
	return stopTol * math.Max(1, math.Abs(scale))
}

// displacementIn validates a signed-displacement parameter — magnitudeIn
// minus the sign gate: ToFace.Offset is the one signed number in the extent
// vocabulary, and its sign is legal intent (which side of the target face
// the sweep stops on), so ErrNegativeMagnitude does not apply (core
// §8.1/§12). The zero Value means no displacement.
// It returns the displacement beside the rounding its own conversion into unit
// committed, the same term magnitudeInBounded reports for a magnitude: an offset
// stated in a non-base unit reaches the level as a rescaled float, and the level
// carries that rounding.
func displacementIn(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, float64, error) {
	if v == (units.Value{}) {
		return 0, 0, nil
	}
	if v.Kind() != kind {
		return 0, 0, fmt.Errorf(`%w: %s must be a %s, got %s`, ErrUnitKind, what, kind, v.Kind())
	}
	m, err := v.In(unit)
	if err != nil {
		return 0, 0, fmt.Errorf(`%w: %s is not representable: %s`, ErrNotFinite, what, err)
	}
	return m, conversionRound(v, unit, m), nil
}

// resolveStopBody runs the body gates every body-relative stop shares with
// EdgeAxis (core §8.1): the named body must be a live *Body of this document
// — a StepRef is ErrUnresolvedBody, another document's body ErrForeignBody,
// a retired one ErrRetiredBody, and nothing at all ErrDegenerate.
func (d *Document) resolveStopBody(ref BodyRef, what string) (*Body, error) {
	switch b := ref.(type) {
	case nil:
		return nil, fmt.Errorf(`%w: %s names no body to resolve against`, ErrDegenerate, what)
	case StepRef:
		return nil, ErrUnresolvedBody
	case *Body:
		if err := d.requireLive(b); err != nil {
			return nil, err
		}
		return b, nil
	default:
		return nil, fmt.Errorf(`%w: %s cannot resolve against a %T`, ErrDegenerate, what, b)
	}
}

// selectStopFace resolves the stop's face selector against the named body
// under the implicit exactly-one of core §12/§9, reported through the same
// SelectionError as a direct SelectFaces (selection_error.go). The rule turns
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
func selectStopFace(body *Body, sel FaceSelector) (*Face, error) {
	q, ok := sel.(*FaceQuery)
	switch {
	case sel == nil:
		return nil, fmt.Errorf(`%w: the stop names no face selector`, ErrDegenerate)
	case !ok:
		return nil, fmt.Errorf(`%w: the stop's face selector is not a decad face query (%T)`, ErrDegenerate, sel)
	case q == nil:
		return nil, errNilSelector
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
func (d *Document) resolveToFace(tf ToFace, frame r3.Frame, travel float64, what string) (float64, float64, StepRef, error) {
	body, err := d.resolveStopBody(tf.Body, what)
	if err != nil {
		return 0, 0, 0, err
	}
	offset, offsetDelta, err := displacementIn(tf.Offset, units.Length, units.Millimeter, "the to-face offset")
	if err != nil {
		return 0, 0, 0, err
	}
	face, err := selectStopFace(body, tf.Face)
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
	tol := relStopTol(math.Max(pl.Frame.Origin().Len(), frame.Origin().Len()))
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
	delta := absSumUpper(
		stopLevelRound(pl.Frame.Origin(), frame.Origin(), n, travel, offset, stop),
		offsetDelta,
		selectedFaceAxialDelta(body, face),
	)
	return stop, delta, body.originStep(), nil
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
// publishes itself as the level it denotes — and the met bodies' StepRefs in
// stop order along the sweep, nearest far side first. No live body in the path
// is ErrDegenerate: the sweep has no stop at all.
func (d *Document) resolveThroughAll(frame r3.Frame, travel float64) (float64, float64, []StepRef, error) {
	dir := frame.N().Scale(travel)
	base := frame.Origin().Dot(dir)
	type stopAt struct {
		far   float64
		hi    float64
		delta float64
		ref   StepRef
	}
	var stops []stopAt
	for _, b := range d.bodies {
		ext, ok := b.payload.(directionalExtent)
		if !ok {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all stop needs every live body's directional extent, and this evaluator did not build one of them`, ErrUnsupported)
		}
		_, hi, bound, err := ext.extentAlong(dir)
		if err != nil {
			return 0, 0, nil, err
		}
		// Two mechanisms move this far side along this direction: the extent
		// reading's own bracket and the payload's stop-face displacement. They
		// displace the same coordinate, so they sum — and a zero term is left
		// out rather than rounded, so an all-analytic body keeps the exact
		// endpoint it has always published.
		delta := bound
		if axial := payloadAxialDelta(b); axial != 0 {
			delta = axial
			if bound != 0 {
				delta = absSumUpper(bound, axial)
			}
		}
		far := hi - base
		tol := relStopTol(math.Max(math.Abs(hi), math.Abs(base)))
		switch {
		case downRound(far-delta) > tol:
			// Material beyond the plane whatever the displacement hides.
		case upRound(far+delta) <= tol:
			// No material beyond the plane, whatever it hides.
			continue
		default:
			// The displacement straddles the plane, and a non-finite one
			// decides neither test above, so both land here.
			return 0, 0, nil, fmt.Errorf(`%w: a through-all sweep cannot decide whether a body is in its path: its far side sits %v mm beyond the sketch plane, known only to a displacement of %v mm`, ErrUnsupported, far, delta)
		}
		stops = append(stops, stopAt{far: far, hi: hi, delta: delta, ref: b.originStep()})
	}
	if len(stops) == 0 {
		return 0, 0, nil, fmt.Errorf(`%w: a through-all sweep found no live body in its path`, ErrDegenerate)
	}
	// Stop order along the sweep: the sweep passes the nearest far side
	// first. A tie keeps the live-body order, so the record is
	// deterministic.
	slices.SortStableFunc(stops, func(a, b stopAt) int { return cmp.Compare(a.far, b.far) })
	refs := make([]StepRef, len(stops))
	for i, s := range stops {
		refs[i] = s.ref
	}
	// The sweep still stops at the FARTHEST held far side, preserving the stop
	// order and endpoint the feature records. Its bound must nevertheless cover
	// every candidate's far-end interval: a lower held far side can be farther
	// in the denoted geometry. Refuse only when that competing interval cannot
	// be represented as a finite bound.
	last := stops[len(stops)-1]
	farDelta := last.delta
	for _, candidate := range stops {
		if isNonFinite(candidate.far) || isNonFinite(candidate.delta) {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all far-end uncertainty is not finite`, ErrUnsupported)
		}
		upper := candidate.far
		if candidate.delta != 0 {
			upper = upRound(candidate.far + candidate.delta)
		}
		if isNonFinite(upper) {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all far-end uncertainty cannot be represented`, ErrUnsupported)
		}
		if upper > last.far {
			farDelta = math.Max(farDelta, upRound(upper-last.far))
		}
	}
	stop := travel * last.far
	delta := absSumUpper(throughStopRound(frame.Origin(), dir, last.hi, travel, stop), farDelta)
	return stop, delta, refs, nil
}

// dedupRefs deduplicates recorded stop refs preserving first occurrence
// (core §6.2), returning nil for none so a step with no dependencies records
// no Inputs at all.
func dedupRefs(refs []StepRef) []StepRef {
	if len(refs) == 0 {
		return nil
	}
	seen := make(map[StepRef]struct{}, len(refs))
	out := make([]StepRef, 0, len(refs))
	for _, r := range refs {
		if _, ok := seen[r]; ok {
			continue
		}
		seen[r] = struct{}{}
		out = append(out, r)
	}
	return out
}

// recordExtent returns the recordable form of a resolved linear extent: a
// ToFace's body substituted by its producing StepRef and its selector
// deep-copied, so what the Recipe carries is only ever values (core §6.2).
// The extent was normalized and resolved already, so a ToFace's body is a
// live *Body here.
func recordExtent(e Extent) Extent {
	switch e := e.(type) {
	case ToFace:
		return recordToFace(e)
	case TwoSided:
		e.One = recordSideExtent(e.One)
		e.Two = recordSideExtent(e.Two)
		return e
	default:
		return e
	}
}

// recordSideExtent is recordExtent's side-tier analog.
func recordSideExtent(s SideExtent) SideExtent {
	if tf, ok := s.(ToFace); ok {
		return recordToFace(tf)
	}
	return s
}

// recordToFace substitutes the StepRef for the live *Body the caller passed
// and deep-copies the selector; the zero-value offset records as an explicit
// zero length.
func recordToFace(tf ToFace) ToFace {
	if b, ok := tf.Body.(*Body); ok {
		tf.Body = b.originStep()
	}
	tf.Offset = normalizeStopOffset(tf.Offset)
	return cloneToFace(tf)
}

// recordAngularExtent is recordExtent's angular analog.
func recordAngularExtent(a AngularExtent) AngularExtent {
	switch a := a.(type) {
	case ToFaceAngular:
		return recordToFaceAngular(a)
	case TwoSidedAngle:
		a.One = recordSideAngular(a.One)
		a.Two = recordSideAngular(a.Two)
		return a
	default:
		return a
	}
}

// recordSideAngular is recordAngularExtent's side-tier analog.
func recordSideAngular(s SideAngular) SideAngular {
	if tfa, ok := s.(ToFaceAngular); ok {
		return recordToFaceAngular(tfa)
	}
	return s
}

// recordToFaceAngular substitutes the StepRef for the live *Body the caller
// passed and deep-copies the selector.
func recordToFaceAngular(tfa ToFaceAngular) ToFaceAngular {
	if b, ok := tfa.Body.(*Body); ok {
		tfa.Body = b.originStep()
	}
	return cloneToFaceAngular(tfa)
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
func (d *Document) angularStopCtx(frame r3.Frame, line axisLine2, ax axisFrame) angularStops {
	a3 := frame.ToWorldUV(line.aU, line.aV)
	w := frame.U().Scale(line.dU).Add(frame.V().Scale(line.dV))
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
func (st angularStops) resolveToFaceAngular(tfa ToFaceAngular, travel float64, what string) (float64, StepRef, error) {
	body, err := st.d.resolveStopBody(tfa.Body, what)
	if err != nil {
		return 0, 0, err
	}
	face, err := selectStopFace(body, tfa.Face)
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
	if math.Abs(off) > relStopTol(math.Max(pl.Frame.Origin().Len(), st.a3.Len())) {
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
		return phi, body.originStep(), nil
	case travel < 0:
		return phi - 2*math.Pi, body.originStep(), nil
	case phi <= math.Pi:
		return phi, body.originStep(), nil
	default:
		return phi - 2*math.Pi, body.originStep(), nil
	}
}

// errStopFaceBothSides names two stops at once, which is no stop at all.
var errStopFaceBothSides = fmt.Errorf(`%w: the stop face spans both sides of the revolve axis`, ErrDegenerate)

// faceHalfPlane locates the one half-plane of the axis the stop face's
// material occupies, as its angle about the axis in [0, 2π), measured
// right-handed about the caller's axis direction from the region's own
// half-plane. The face plane contains the axis (checked by the caller), so
// every point sits at one of two opposite angles; the face is usable only
// when its boundary reaches exactly one of them — vertices on both sides,
// or a circular edge dipping across the axis between its vertices, put
// material in both half-planes (ErrDegenerate), and a face with no material
// off the axis names no half-plane at all. Vertices decide the angle first;
// the circular edges are then audited against it, exactly.
func (st angularStops) faceHalfPlane(face *Face) (float64, error) {
	edges := face.Edges()
	phi := math.NaN()
	for _, e := range edges {
		for _, p := range boundaryProbes(e) {
			x := p.Sub(st.a3)
			u, w := x.Dot(st.r0), x.Dot(st.e1)
			if math.Hypot(u, w) <= relStopTol(x.Len()) {
				continue
			}
			a := math.Atan2(w, u)
			if a < 0 {
				a += 2 * math.Pi
			}
			if math.IsNaN(phi) {
				phi = a
				continue
			}
			diff := math.Abs(a - phi)
			if diff > math.Pi {
				diff = 2*math.Pi - diff
			}
			if diff > angTol {
				return 0, errStopFaceBothSides
			}
		}
	}
	if math.IsNaN(phi) {
		return 0, fmt.Errorf(`%w: the stop face has no material off the revolve axis`, ErrDegenerate)
	}
	// The half-plane direction the probes agreed on; a circular edge may
	// still dip across the axis between its probes.
	m := st.r0.Scale(math.Cos(phi)).Add(st.e1.Scale(math.Sin(phi)))
	for _, e := range edges {
		if err := st.rejectAxisCrossing(e, m); err != nil {
			return 0, err
		}
	}
	return phi, nil
}

// boundaryProbes are the closed-form points that witness which half-plane an
// edge's material sits in: its vertices, plus — for a circular edge, whose
// off-axis material may sit entirely between on-axis vertices (a half-disk
// cap) — the arc's parametric midpoint, or the whole circle's seam antipode.
// Every probe is an exact point OF the edge; nothing is sampled.
func boundaryProbes(e *Edge) []r3.Vec {
	var probes []r3.Vec
	for _, v := range []*Vertex{e.start, e.end} {
		if v != nil {
			probes = append(probes, v.position)
		}
	}
	switch c := e.Curve().(type) {
	case Circle3:
		if e.start != nil {
			probes = append(probes, c.Center.Scale(2).Sub(e.start.position))
		}
	case Arc3:
		if e.start == nil || e.end == nil {
			break
		}
		n, ok := c.Axis.Normalize()
		if !ok {
			break
		}
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			break
		}
		u0, ok := e.start.position.Sub(c.Center).Normalize()
		if !ok {
			break
		}
		v0 := n.Cross(u0)
		x := e.end.position.Sub(c.Center)
		sweep := math.Atan2(x.Dot(v0), x.Dot(u0))
		if sweep < 0 {
			sweep += 2 * math.Pi
		}
		sin, cos := math.Sincos(sweep / 2)
		probes = append(probes, c.Center.Add(u0.Scale(cos*r)).Add(v0.Scale(sin*r)))
	}
	return probes
}

// rejectAxisCrossing rejects a circular boundary edge that reaches across
// the revolve axis into the opposite half-plane: the face would name two
// stops at once. m is the half-plane direction the face's vertices agreed
// on; the edge's circle lies in the face plane (which contains the axis), so
// the edge crosses exactly when its deepest point against m — the circle
// point at −m from the center, when the walk actually passes it — sits
// strictly across the axis. Closed-form: no sampling.
func (st angularStops) rejectAxisCrossing(e *Edge, m r3.Vec) error {
	var center r3.Vec
	var radius units.Value
	var axis r3.Vec
	closed := false
	switch c := e.Curve().(type) {
	case Circle3:
		center, radius, axis, closed = c.Center, c.Radius, c.Axis, true
	case Arc3:
		center, radius, axis = c.Center, c.Radius, c.Axis
	default:
		// A linear edge between agreeing vertices stays between them.
		return nil
	}
	r, err := radius.In(units.Millimeter)
	if err != nil {
		return fmt.Errorf(`%w: a stop face edge's radius is not representable: %s`, ErrNotFinite, err)
	}
	depth := center.Sub(st.a3).Dot(m) - r
	if depth >= -relStopTol(r) {
		// Even the circle's deepest point stays in the face's half-plane
		// (or on the axis).
		return nil
	}
	if closed {
		// A whole circle always reaches its deepest point.
		return errStopFaceBothSides
	}
	// An arc reaches its deepest point only when −m lies within its walked
	// range (CCW from start to end about the arc's own axis).
	n, ok := axis.Normalize()
	if !ok {
		return fmt.Errorf(`%w: a stop face arc edge has no axis`, ErrDegenerate)
	}
	if e.start == nil || e.end == nil {
		return errStopFaceBothSides
	}
	u0, ok := e.start.position.Sub(center).Normalize()
	if !ok {
		return errStopFaceBothSides
	}
	v0 := n.Cross(u0)
	ang := func(x r3.Vec) float64 {
		a := math.Atan2(x.Dot(v0), x.Dot(u0))
		if a < 0 {
			a += 2 * math.Pi
		}
		return a
	}
	sweep := ang(e.end.position.Sub(center))
	deepest := ang(m.Scale(-1))
	if deepest <= sweep+angTol {
		return errStopFaceBothSides
	}
	return nil
}
