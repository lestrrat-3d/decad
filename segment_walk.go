package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"reflect"

	"github.com/lestrrat-3d/units"
)

// This file is the package's profile-boundary walk: segmentWalk, the resolved
// per-segment form every feature reads a recorded CurveSegment through, and
// the per-kind builders that produce one.
//
// A walk is the recorded segment restated in the form a sweep needs — a
// centre, a radius and a turn for a circular kind, two endpoints for a line,
// a chorded chain for a Tier A free-form kind — beside the proven bound on
// each quantity the restatement rounded. A kind that cannot be restated with
// a stated bound refuses through requireAnalyticWalk rather than publishing
// an unbounded walk.
//
// Extrude, revolve and loft all read walks through profileWalks, which
// resolves a whole profile once and re-checks, on every later read, that the
// walks it holds still match the record they were resolved from.

// walkKind discriminates what a segmentWalk's geometry IS. It replaced a
// line-versus-circular boolean because a free-form walk is neither: a two-state
// flag left every "not circular" branch silently building a straight line out of
// a spline, which is exactly the confidently-wrong answer decad exists to
// prevent (docs/spline-design.md §6.2).
//
// A switch on walkKind MUST be total. A consumer that cannot yet handle
// walkFreeform refuses before building an analytic face: where it needs a walk
// it uses requireAnalyticWalk, and where no resolution can contribute it gates
// the recorded free-form kind before walkOf.
type walkKind uint8

const (
	// walkLine is a straight walk between its endpoints.
	walkLine walkKind = iota
	// walkCircular is a circular walk about (cU, cV) — a circle or an arc.
	walkCircular
	// walkFreeform is a free-form walk, whose geometry lives in its converted
	// Bézier spans rather than in the circular fields.
	walkFreeform
)

// segmentWalk is one boundary segment's walk geometry in plane coordinates.
type segmentWalk struct {
	// start/end are the walk's endpoints in (u, v); closed is true for a
	// whole closed curve (no junction vertices at all).
	startU, startV float64
	endU, endV     float64
	// startBound/endBound are the PROVEN error bounds on the endpoint beside
	// them, in the coordinates' own millimetres — radiusBound's twin two fields
	// up, and stated for the same reason. An endpoint is an exact leaf only
	// where the record STATES it: a line's own bounds and an arc's own bounds
	// are recorded coordinates the walk reads verbatim (lerp2, pinArcWalkEnds),
	// and those read zero. That zero is about THIS walk's own rounding, not
	// about the recorded coordinate agreeing with the curve the record denotes
	// at that parameter — arcWalkEnd's own doc comment states where an arc's
	// two readings part company, and names who owes the difference. Every other endpoint is computed — a trimmed line's
	// is a float lerp, a trimmed arc's and EVERY circle's is a
	// math.Cos/math.Sin at an angle this package itself computed — so each kind
	// STATES what its own endpoint is worth (lineWalkEndBound,
	// circularWalkEndBound, freeformEndpointBounds) or REFUSES with +Inf, never
	// leaves it silently zero. A reading that folds an endpoint into an answer
	// charges it through pointPerturbationAllow; one that cannot state the
	// charge refuses on the +Inf rather than publishing an exactness the
	// evaluator never proved.
	startBound, endBound walkEndBound
	closed               bool
	// tanIn/tanOut are the walk tangents at start and end (unit not
	// required), for junction convexity.
	tanInU, tanInV   float64
	tanOutU, tanOutV float64
	// tanInBound/tanOutBound are the PROVEN error bound on EITHER component
	// of the tangent beside them, in the coordinates' own millimetres. A
	// tangent is NOT an exact leaf the way a recorded coordinate is: a line
	// walk's is the float difference of two endpoints, a circular walk's runs
	// through math.Sincos at a computed angle, and a free-form walk's is an
	// exact rational leg rounded once into float64. Each kind therefore
	// STATES its bound or REFUSES with +Inf — never leaves it silently zero —
	// so a reading composed from a tangent can charge the error the evaluator
	// actually committed. The refusal is the circular kind's: its held
	// components come from a trig evaluation at an angle that is itself
	// computed, and this walk states no enclosure of either, so +Inf is the
	// underivable bound every consumer refuses on rather than publishes
	// (arcWalkRadiusBound's own convention).
	tanInBound, tanOutBound float64
	length                  float64
	lengthBound             float64
	lengthUpper             float64
	coordUpper              float64
	axisRadiusUpper         float64
	axisMomentUpper         float64
	// startVBound/endVBound/cVBound are the PROVEN error bounds on the radial
	// (V) axis-coordinate beside them — startV/endV/cV's own displacement from
	// the value the axis's TRUE (unrounded) direction and anchor would give,
	// through axisFrame.toAxisRhoBound, composed for startV/endV with whatever
	// magnitude that walk's own axis snap discarded to assign an endpoint
	// exactly zero. They are set ONLY by axisFrame.walk,
	// which re-expresses a plane-local walk into axis coordinates: a walk that
	// has not been through it (every use before revolve resolves an axis)
	// leaves them at their zero value, meaningless there. axisFrame.toAxis
	// itself states no such bound (its own doc comment), so a caller
	// composing a reading from startV/endV/cV — the revolve minimum-radius
	// meridian survey (survey.go's revolveMinRadius) — reads these instead of
	// the coordinate as an exact leaf; axisMoments (revolve.go) folds the
	// SAME axis-direction/anchor uncertainty into the region's moments through
	// bounded arithmetic instead, and does not read these fields.
	startVBound, endVBound, cVBound float64
	// kind says which geometry the walk carries; the fields below it are
	// meaningful only for walkCircular.
	kind   walkKind
	cU, cV float64
	radius float64
	// radiusBound is the PROVEN error bound on radius (millimetres). A
	// CircleSeg states its radius, so its walk holds that number and the bound
	// is zero; an ArcSeg states Start and Center only, so its walk's radius is
	// a math.Hypot evaluation and the bound is arcWalkRadiusBound's rational
	// bracket. It exists because radius is NOT an exact leaf the way a
	// recorded coordinate is, and a reading that treats it as one can publish
	// an interval its own truth sits outside of. The analytic surveys
	// (survey.go's minimum-radius arms, and survey2d.go through
	// surveyElem.rrBound) take it; a consumer that reads radius as a leaf
	// still owes its own account of the error, from its own envelope.
	radiusBound float64
	th0, th1    float64
	// spans is the converted Bézier chain of a walkFreeform walk, in the
	// curve's natural direction; reversed says the walk runs against it. Both
	// are zero for every other kind.
	spans    []bezierSpan
	reversed bool
	// fitInterpolated is set only for a walkFreeform walk whose chain came
	// from FitSplineSeg's §5.1.2 conversion (spline_fit.go's isFitSplineSeg,
	// read on the segment walkOf resolved — walkOf normalizes as its first
	// statement, so freeformWalk's own seg, and every isFitSplineSeg check
	// downstream of it, always sees the normalized value form). §6.5's
	// convexity certificate needs it to apply the FitSplineSeg carve-out: a
	// joint interior to that conversion's chain is verdict 0 BY CONSTRUCTION,
	// never by jointConvexitySign's cross product, because that cross carries
	// sketch's own rounded SecondDerivs solve rather than a turn of the
	// recorded curve (docs/spline-design.md §6.5, §5.1.2). It is false for
	// every other Tier A kind, whose joints are genuine C⁰ corners the cross
	// product must still fold.
	fitInterpolated bool
}

// walkEndBound is the proven error bound on a walk endpoint's two components,
// stated PER COMPONENT and never merged into one number. The two are
// independent readings and an endpoint routinely proves one exactly while the
// other carries error: a whole circle's own end is exactly that shape, since
// math.Cos returns 1 at the end angle while math.Sin does not return 0 there.
// Merging them would spend the exact axis's zero on the other axis's error, and
// a reading along the exact axis alone would then publish a width its own
// arithmetic never committed.
//
// A component the recorded data cannot enclose reads +Inf, the underivable
// bound every consumer refuses on, and never zero.
type walkEndBound struct {
	u, v float64
}

// derivable reports whether both components state a bound at all.
func (b walkEndBound) derivable() bool { return !isNonFinite(b.u) && !isNonFinite(b.v) }

// isCircular reports whether the walk is a circle or arc — the question the
// closed-form circular branches ask.
func (w segmentWalk) isCircular() bool { return w.kind == walkCircular }

// isLine reports whether the walk is straight. It is NOT "not circular": a
// free-form walk answers false to both.
func (w segmentWalk) isLine() bool { return w.kind == walkLine }

// requireAnalyticWalk refuses a free-form walk on behalf of a consumer that has
// no free-form construction yet. Reaching it is a staging limit, never a wrong
// answer — the reason each consumer stages is its own row in
// docs/spline-design.md Table R. The prism side-face build itself no longer
// calls this: buildLoopSidesAs switches on walkKind instead (§10 P4b), with its
// own free-form arm, and tessellate.go's chordLoop no longer calls it either:
// it switches on walkKind, with its own free-form chording arm
// (docs/tessellation-reach-design.md §5). Every remaining call site is a
// capability neither increment reaches — the modify ops (fillet.go,
// shell_offset.go, capblend_geom.go), revolve (revolve.go), and
// profileCoordinateUpper's own callers (capblend_centroid.go, revolve.go),
// which need a placed cap frame a free-form wall genuinely cannot represent.
//
// The one call site that reaches walkOf without this gate is
// moments_validate.go's validateMomentWalk: it runs only after every
// free-form segment kind has already been diverted to the exact integrator
// (spline_bezier.go/spline_moments.go), so a free-form segment never reaches
// it. This is deliberate, not a missed gate — adding one here would be dead
// code guarding an unreachable case.
func requireAnalyticWalk(w segmentWalk, what string) error {
	if w.kind != walkFreeform {
		return nil
	}
	return fmt.Errorf(`%w: %s does not support a free-form boundary segment`, ErrUnsupported, what)
}

// profileWalks is one profile's segment walks resolved ONCE, so that every
// consumer within a single prism evaluation reads the same resolution back
// instead of paying walkOf's own §5.2 charge again for it. Within one
// evalPrismContext call, buildLoopSidesAs, profileCoordinateEnvelope (called
// from prismCentroidGeometryBound and, four times over, from
// prismBoundsContext's per-axis extentBoundedAlong) and
// boundaryExtremesBoundedContext (three times, also from extentBoundedAlong)
// each used to call walkOf on the SAME recorded segment — eight resolutions of
// one segment, each recharging §5.2's exact-rational counter in full. On a
// 15-point involute fit spline that alone charged 230,168 units eight times
// over, tripping the R7 ceiling on a record whose deduplicated charge fits
// comfortably inside it. profileWalks is the fix: resolve every segment's walk
// once and let each consumer read it back.
//
// A nil *profileWalks means "resolve as before" everywhere below: revolve, the
// cap-loop chamfer, the shell cup, Verify, and every re-evaluation path
// (Placed/Duplicate/PlacedCopy) that has no preflight in hand pass nil and are
// unaffected, since they run over a DIFFERENT profile (a cap contour, an
// offset loop) or hold no single-evaluation resolution worth sharing.
type profileWalks struct {
	// profile is the record every walk below was resolved FROM, kept whole so
	// that a read against another profile is caught by comparing the recorded
	// segments themselves rather than their shape (matches).
	profile ProfileRecord
	// outer holds loop index 0's resolved walks, one per pp.profile.Outer
	// segment, in recorded order.
	outer []segmentWalk
	// holes holds loop index i>0's resolved walks as holes[i-1], one slice
	// per pp.profile.Holes entry, each in recorded order — the same
	// append([]LoopRecord{profile.Outer}, profile.Holes...) indexing every
	// consumer below already walks.
	holes [][]segmentWalk
}

// resolveProfileWalks resolves every segment of profile's outer loop and each
// hole loop through walkOf exactly once, charging work the same single time
// each segment's own conversion and length bracket cost (docs/spline-design.md
// §5.2), rather than once per consumer.
func resolveProfileWalks(profile ProfileRecord, work *freeformWork) (*profileWalks, error) {
	outer := make([]segmentWalk, len(profile.Outer.Segments))
	for i, seg := range profile.Outer.Segments {
		w, err := walkOf(seg, work)
		if err != nil {
			return nil, err
		}
		outer[i] = w
	}
	holes := make([][]segmentWalk, len(profile.Holes))
	for hi, hole := range profile.Holes {
		hw := make([]segmentWalk, len(hole.Segments))
		for i, seg := range hole.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			hw[i] = w
		}
		holes[hi] = hw
	}
	return &profileWalks{profile: profile, outer: outer, holes: holes}, nil
}

// at returns the resolved walk for loop index loopIndex (0 the outer loop,
// i>0 profile.Holes[i-1]) and segment index segIndex within that loop, in
// the same indexing every consumer's
// append([]LoopRecord{profile.Outer}, profile.Holes...) walk already uses.
// Callers check matches first; at itself trusts the index it is given.
func (pw *profileWalks) at(loopIndex, segIndex int) segmentWalk {
	if loopIndex == 0 {
		return pw.outer[segIndex]
	}
	return pw.holes[loopIndex-1][segIndex]
}

// loopWalks returns the resolved walk slice for loop index loopIndex (the
// same convention as at), or nil if loopIndex is out of range for pw. A
// single-loop consumer (buildLoopSidesAs) uses this instead of at plus its
// own per-segment loop, since it already owns the per-segment index into the
// slice it gets back.
func (pw *profileWalks) loopWalks(loopIndex int) []segmentWalk {
	if loopIndex == 0 {
		return pw.outer
	}
	hi := loopIndex - 1
	if hi < 0 || hi >= len(pw.holes) {
		return nil
	}
	return pw.holes[hi]
}

// matches reports whether pw was resolved from THIS profile: the same loops
// in the same order, each holding the same recorded segments — the same
// variant with the same field values, compared exactly (identicalRecord).
// Shape alone is not enough, and never was: two profiles can carry the same
// outer, hole and per-hole segment counts while every coordinate differs, and
// a set resolved from one read against the other would report the first
// section's geometry as the second's, silently.
//
// Every consumer that reads a non-nil *profileWalks checks this FIRST and
// refuses rather than reading it — docs/spline-design.md §5.2's own discipline
// extended to this cache. The refusal is one-directional, like every other
// decad-side check: only an exact agreement between the two records reads the
// cache, and anything else — a differing value, a differing variant, a shape
// the comparison does not know how to traverse — refuses. There is no
// tolerance and no "close enough" arm, so a near-miss profile is rejected on
// the same terms as an unrelated one, and a plumbing bug never hides behind a
// correct-looking answer.
func (pw *profileWalks) matches(profile ProfileRecord) bool {
	if pw == nil {
		return false
	}
	return identicalRecord(pw.profile, profile)
}

// loopMatches reports whether pw holds, at loop index loopIndex (the same
// convention as at), the walks resolved from exactly this loop's recorded
// segments. It is matches for the single-loop consumer buildLoopSidesAs,
// which is handed one LoopRecord and a role index rather than the whole
// profile, and it refuses on the same exact-comparison terms.
func (pw *profileWalks) loopMatches(loopIndex int, loop LoopRecord) bool {
	if pw == nil {
		return false
	}
	loops := append([]LoopRecord{pw.profile.Outer}, pw.profile.Holes...)
	if loopIndex < 0 || loopIndex >= len(loops) {
		return false
	}
	return identicalRecord(loops[loopIndex], loop)
}

// identicalRecord reports whether two recorded values are the same record:
// the same dynamic type throughout, and every field, element and float bit
// equal. It is the exact structural comparison profileWalks' own guard rests
// on, and it is deliberately stricter than a numeric comparison — floats are
// compared by their BITS (math.Float64bits), so a value that merely rounds to
// the same number, or a zero of the other sign, is a mismatch rather than a
// match.
//
// The traversal is reflective rather than a per-variant type switch on the
// sealed CurveSegment set, and that is the point: a hand-written comparator
// that forgets a field a variant gains later would go on reporting two
// different records as the same one, which is exactly the failure this guard
// exists to prevent. Reflection covers a new field the moment it is declared.
//
// A shape the traversal does not know — a map, a channel, a function — is
// reported as a mismatch, never as a match. Every refusal here is safe: it
// costs the caller a cached read, which it can always resolve itself, whereas
// a wrong match publishes another section's geometry as this one's.
func identicalRecord(a, b any) bool {
	return identicalRecordValue(reflect.ValueOf(a), reflect.ValueOf(b))
}

// identicalRecordValue is identicalRecord's traversal. The zero reflect.Value
// (a nil interface handed to reflect.ValueOf) matches only another zero one.
func identicalRecordValue(a, b reflect.Value) bool {
	if !a.IsValid() || !b.IsValid() {
		return a.IsValid() == b.IsValid()
	}
	if a.Type() != b.Type() {
		return false
	}
	switch a.Kind() { //nolint:exhaustive // an unhandled kind is a mismatch, by the doc comment above.
	case reflect.Bool:
		return a.Bool() == b.Bool()
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		return a.Int() == b.Int()
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr:
		return a.Uint() == b.Uint()
	case reflect.Float32, reflect.Float64:
		// Float() widens a float32 exactly, so one comparison serves both.
		return math.Float64bits(a.Float()) == math.Float64bits(b.Float())
	case reflect.String:
		return a.String() == b.String()
	case reflect.Struct:
		for i := range a.NumField() {
			// Field reads an unexported field read-only, which is all this
			// traversal ever does — units.Value's own magnitude and unit are
			// unexported and are compared here like any other field.
			if !identicalRecordValue(a.Field(i), b.Field(i)) {
				return false
			}
		}
		return true
	case reflect.Slice, reflect.Array:
		if a.Len() != b.Len() {
			return false
		}
		for i := range a.Len() {
			if !identicalRecordValue(a.Index(i), b.Index(i)) {
				return false
			}
		}
		return true
	case reflect.Interface, reflect.Pointer:
		if a.IsNil() || b.IsNil() {
			return a.IsNil() && b.IsNil()
		}
		return identicalRecordValue(a.Elem(), b.Elem())
	default:
		return false
	}
}

// errResolvedWalksMismatch reports a *profileWalks handed to a consumer that
// does not match the profile it is read against — an evaluator plumbing
// invariant break, never a caller-reachable refusal: every call site in this
// package resolves walks from the exact profile it later reads them against,
// so reaching this error means a future edit broke that pairing, not that the
// recorded geometry is at fault.
var errResolvedWalksMismatch = fmt.Errorf(`%w: resolved walks do not match the profile they are read against`, ErrUnsupported)

// walkOf resolves one recorded segment into its walk geometry.
//
// work is the RECORD's free-form work counter (docs/spline-design.md §5.2), and
// walkOf NEVER mints one: the R7 ceiling bounds one record's total free-form
// work, so a counter minted per call would hand every segment — and every later
// phase of the same operation — a fresh full ceiling. Callers that already hold
// the counter a moments preflight opened for this record pass THAT one, so the
// walk's arc-length bracket spends what the preflight left rather than a second
// ceiling; callers with no preflight in hand mint exactly one for the whole
// record walk. An analytic segment charges nothing, so a nil counter is harmless
// there and refused on the free-form arm rather than quietly replaced.
func walkOf(seg CurveSegment, work *freeformWork) (segmentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return segmentWalk{}, err
	}
	switch seg := seg.(type) {
	case LineSeg:
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		length := math.Hypot(du, dv)
		lengthBound, lengthUpper, coordUpper := lineWalkBounds(seg, length)
		tangentBound := lineWalkTangentBound(seg, du, dv)
		return segmentWalk{
			startU: u0, startV: v0, endU: u1, endV: v1,
			startBound: lineWalkEndBound(seg, seg.TStart, u0, v0),
			endBound:   lineWalkEndBound(seg, seg.TEnd, u1, v1),
			tanInU:     du, tanInV: dv, tanOutU: du, tanOutV: dv,
			tanInBound:  tangentBound,
			tanOutBound: tangentBound,
			length:      length,
			lengthBound: lengthBound,
			lengthUpper: lengthUpper,
			coordUpper:  coordUpper,
		}, nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return segmentWalk{}, fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return segmentWalk{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		th0, th1 := 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			r,
			th0,
			th1,
			math.Abs(r),
			circularSweepUpper(seg.TStart, seg.TEnd),
		)
		w.closed = math.Abs(math.Abs(th1-th0)-2*math.Pi) < 1e-12
		w.startBound = circularWalkEndBound(seg, seg.TStart, w.startU, w.startV)
		w.endBound = circularWalkEndBound(seg, seg.TEnd, w.endU, w.endV)
		if iv, ok := circularLengthInterval(seg); ok {
			w.lengthBound = math.Min(w.lengthBound, intervalFloatError(iv, w.length))
		}
		return w, nil
	case ArcSeg:
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			radius,
			a0+seg.TStart*sweep,
			a0+seg.TEnd*sweep,
			arcRadiusUpper(seg),
			circularSweepUpper(seg.TStart, seg.TEnd),
		)
		w.radiusBound = arcWalkRadiusBound(seg, radius)
		pinArcWalkEnds(&w, seg)
		if iv, ok := circularLengthInterval(seg); ok {
			w.lengthBound = math.Min(w.lengthBound, intervalFloatError(iv, w.length))
		}
		return w, nil
	default:
		if !isFreeformSegment(seg) {
			return segmentWalk{}, fmt.Errorf(`%w: this evaluator sweeps profiles of line, arc, circle and Tier A free-form segments only; the profile has a %T segment it cannot sweep into a side face yet`, ErrUnsupported, seg)
		}
		return freeformWalk(seg, work)
	}
}

// lineWalkTangentBound is the single owner of the proven bound on a line
// walk's tangent, and arcWalkRadiusBound's twin one field over: the record
// states the segment's endpoints and its parameter range, never the tangent,
// so the walk's held tangent is the float difference u1−u0, v1−v0 of two
// endpoints the float lerp already rounded. The tangent the record DENOTES is
// the difference of the exact rational lerps (ratLerp), which carries no
// rounding at either step, and the bound is the wider of the two components'
// gaps from it, rounded outward. A lerp that is not representable as a
// rational yields +Inf — the underivable bound consumers refuse on.
func lineWalkTangentBound(seg LineSeg, heldU, heldV float64) float64 {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1)
	}
	return math.Max(
		rationalFloatError(new(big.Rat).Sub(u1, u0), heldU),
		rationalFloatError(new(big.Rat).Sub(v1, v0), heldV),
	)
}

// lineWalkEndBound is the single owner of the proven bound on a LINE walk's
// endpoint, and lineWalkTangentBound's twin one field over: the record states
// the segment's endpoints and its parameter range, never the point at a trimmed
// parameter, so the walk's held endpoint is lerp2's float evaluation. The point
// the record DENOTES is the exact rational lerp (ratLerp), which carries no
// rounding at either step, and the bound is the wider of the two components'
// gaps from it, rounded outward. A natural bound needs no argument of its own:
// lerp2 and ratLerp both special-case t = 0 and t = 1 to the recorded Point2
// verbatim, so the two agree exactly and this answers zero. A lerp that is not
// representable as a rational yields +Inf — the underivable bound consumers
// refuse on.
func lineWalkEndBound(seg LineSeg, t, heldU, heldV float64) walkEndBound {
	return walkEndBound{
		u: rationalFloatError(ratLerp(seg.Start.U, seg.End.U, t), heldU),
		v: rationalFloatError(ratLerp(seg.Start.V, seg.End.V, t), heldV),
	}
}

// circularWalkEndBound is the single owner of the proven bound on a CIRCULAR
// walk's endpoint: circularWalk reaches every endpoint through math.Sincos at
// an angle this package computed — a CircleSeg's from a float multiply by 2π,
// an ArcSeg's from math.Atan2 of the recorded differences — and neither the
// trig nor its argument is a quantity that walk can enclose from the record
// alone (circularWalk's own comment). circularEndpointInterval encloses the
// point the record DENOTES at that parameter instead, from the recorded data
// and certified trigonometry, and each component's bound is its own gap from
// that enclosure.
//
// An enclosure the recorded data cannot state yields +Inf — an underivable
// bound, which every consumer refuses on rather than publishes.
func circularWalkEndBound(seg CurveSegment, t, heldU, heldV float64) walkEndBound {
	rt := floatRat(t)
	if rt == nil {
		return walkEndBound{u: math.Inf(1), v: math.Inf(1)}
	}
	return circularPointBound(seg, rt, heldU, heldV)
}

// circularPointBound is circularWalkEndBound read at an EXACT RATIONAL
// parameter rather than a held float, and owns the derivation both spellings
// share. It exists for a caller that generates a point at a parameter the
// record's own arithmetic states exactly — a uniform station division
// t_k = TStart + (k/m)·(TEnd − TStart) (loft_build.go's circularStationChain)
// is the one such caller today. Rounding that parameter to a float first would
// enclose the recorded curve at a NEIGHBOURING parameter, and the bound would
// then be a proof about a point the construction never named: the cells either
// side of it would no longer divide the sweep uniformly, the division
// docs/loft-design.md §5.2's per-cell sagitta row derives that term over.
//
// An enclosure the recorded data cannot state yields +Inf on both components,
// the underivable bound every consumer refuses on.
func circularPointBound(seg CurveSegment, t *big.Rat, heldU, heldV float64) walkEndBound {
	uIv, vIv, ok := circularEndpointInterval(seg, t)
	if !ok {
		return walkEndBound{u: math.Inf(1), v: math.Inf(1)}
	}
	return walkEndBound{
		u: intervalFloatError(uIv, heldU),
		v: intervalFloatError(vIv, heldV),
	}
}

// arcWalkRadiusBound is the single owner of the proven bound on an ArcSeg
// walk's radius, and the reason segmentWalk carries radiusBound at all: the
// record states Start and Center, never the radius, so the walk's held radius
// is the float math.Hypot of their difference. The exact radius is
// √((Su−Cu)² + (Sv−Cv)²) over the recorded coordinates, which ratSqrtDown and
// ratSqrtUp bracket without rounding, and the bound is the wider side of that
// bracket about the held float, rounded outward. A bracket that overflows
// yields +Inf — an underivable bound, which every consumer refuses on rather
// than publishes.
func arcWalkRadiusBound(seg ArcSeg, held float64) float64 {
	dx := exactCoordinateDelta(seg.Start.U, seg.Center.U)
	dy := exactCoordinateDelta(seg.Start.V, seg.Center.V)
	r2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	rLo, rHi := ratSqrtDown(r2), ratSqrtUp(r2)
	if isNonFinite(rLo) || isNonFinite(rHi) {
		return math.Inf(1)
	}
	return math.Max(upRound(held-rLo), upRound(rHi-held))
}

// freeformWalk resolves a Tier A free-form segment into its walk geometry
// (docs/spline-design.md Table F). Every field it fills is a proof:
//
//   - the endpoints are the converted chain's own first and last control
//     points, which a Bézier interpolates exactly, each under the bound of the
//     one rounding that conversion committed (freeformEndpointBounds);
//   - the tangents are the hodograph at those ends, exact directions;
//   - the length is §6.1's proven two-sided bracket, so lengthBound is
//     positive and the walk NEVER claims an exact length — a control net
//     collapsed to a single point has no positive bracket and refuses as
//     ErrDegenerate rather than resolve into a walk (Table R row R14), and a
//     curve whose enclosure runs past MaxFloat64 refuses as ErrUnsupported
//     (R15); freeformArcLength owns both;
//   - coordUpper and lengthUpper are convex-hull envelopes, so they bound the
//     curve and not merely its control net.
//
// axisRadiusUpper and axisMomentUpper stay zero: they are revolve's readings,
// and revolve refuses a free-form walk before reaching them.
//
// The conversion and the length bracket are charged against the caller's counter
// — the record's, never one minted here. A caller that reaches this arm with no
// counter has no ceiling at all, which is the one thing §5.2 forbids, so the
// resolution refuses rather than run unbounded work.
func freeformWalk(seg CurveSegment, work *freeformWork) (segmentWalk, error) {
	if work == nil {
		return segmentWalk{}, errFreeformWalkUncounted
	}
	spans, reversed, err := freeformBezierSpans(seg, work)
	if err != nil {
		return segmentWalk{}, err
	}
	start, end, err := freeformEndpoints(spans, reversed)
	if err != nil {
		return segmentWalk{}, err
	}
	length, bound, err := freeformArcLength(spans, work)
	if err != nil {
		return segmentWalk{}, err
	}
	tangents, err := freeformEndTangents(spans, reversed)
	if err != nil {
		return segmentWalk{}, err
	}
	startBound, endBound := freeformEndpointBounds(spans, reversed, start, end)
	return segmentWalk{
		startU: start.U, startV: start.V,
		endU: end.U, endV: end.V,
		startBound: startBound,
		endBound:   endBound,
		// A closed free-form curve returns to its start, so it carries no
		// junction vertex — the same fact CircleSeg's closed walk states.
		closed:          start == end,
		tanInU:          tangents.inU,
		tanInV:          tangents.inV,
		tanInBound:      tangents.inBound,
		tanOutU:         tangents.outU,
		tanOutV:         tangents.outV,
		tanOutBound:     tangents.outBound,
		length:          length,
		lengthBound:     bound,
		lengthUpper:     upRound(length + bound),
		coordUpper:      freeformControlExtent(spans),
		kind:            walkFreeform,
		spans:           spans,
		reversed:        reversed,
		fitInterpolated: isFitSplineSeg(seg),
	}, nil
}

// errFreeformWalkUncounted is the refusal of a free-form resolution handed no
// record counter. It is ErrUnsupported because the curve exists and this
// evaluator declines to resolve it without the ceiling §5.2 requires — never a
// silently minted counter, which is the second full ceiling the rule forbids.
var errFreeformWalkUncounted = fmt.Errorf(
	`%w: a free-form segment's walk needs its record's free-form work counter`, ErrUnsupported,
)

// pinArcWalkEnds states an arc walk's natural bounds as the record's own
// endpoints. A recorded arc runs Start → End over [0, 1] about Center
// (record.go), so its value at t = 0 is Start and at t = 1 is End, exactly,
// while circularWalk reaches those same two points through atan2 and cos/sin —
// a route that need not land back on them, because the angle it evaluates at
// the far bound is itself the rounded a0 + sweep. Only the two endpoints are
// restated; the walk's centre, radius, angles and tangents keep circularWalk's
// own values, and every reading derived from them keeps its own bound.
//
// This is the rule lerp2 (moments.go) applies at a line's own bounds, and the
// rule seam.go's edgeJoin applies when it reads an uncut bound off the record
// rather than off sketch's node. It matters for the same reason: buildPrismScene
// (prism_boolean.go) creates one sketch point per walked endpoint, so a walk
// that missed the vertex two segments share would offer sketch two points where
// the record states one, and RecordProfile would then refuse the region the
// arrangement admits on its own proximity threshold.
//
// A trimmed bound's POSITION is left alone: it has no recorded coordinate of
// its own, and inventing one is what this seam never does. What it does get is
// the bound circularWalk's route actually owes — see arcWalkEnd, which owns the
// natural-bound test for both readings so the pinned position and the zero
// bound can never drift apart.
func pinArcWalkEnds(w *segmentWalk, seg ArcSeg) {
	w.startU, w.startV, w.startBound = arcWalkEnd(seg, seg.TStart, w.startU, w.startV)
	w.endU, w.endV, w.endBound = arcWalkEnd(seg, seg.TEnd, w.endU, w.endV)
}

// arcWalkEnd states one arc walk end: its position and the proven bound on each
// of its components. At a natural bound the record states the point verbatim,
// so the walk reads Start or End and the bound is zero — the pin and the zero
// are one decision, taken here once. At any other parameter the walk keeps
// circularWalk's own held pair under the bound circularWalkEndBound proves for
// it.
//
// What the natural-bound zero states is that the held pair IS the recorded
// coordinate, with no rounding of this walk's own. It does NOT state that the
// recorded coordinate is the point the DENOTED curve passes through there. For
// an arc the two coincide at t == 0 and need not at t == 1: the denoted curve
// takes its radius from Start alone (circularEndpointInterval, moments.go), so
// its t == 1 point sits at Start's radius and End's angle, which is the
// recorded End only where the two recorded radii are equal — an equality
// nothing in this package certifies. A consumer that publishes a station's
// displacement from the DENOTED point owes that radial residual on top of this
// zero; docs/loft-design.md §5.2 names the term and loft_build.go's
// arcNaturalEndRadialUpper charges it for the loft.
func arcWalkEnd(seg ArcSeg, t, heldU, heldV float64) (float64, float64, walkEndBound) {
	switch t {
	case 0:
		return seg.Start.U, seg.Start.V, walkEndBound{}
	case 1:
		return seg.End.U, seg.End.V, walkEndBound{}
	}
	return heldU, heldV, circularWalkEndBound(seg, t, heldU, heldV)
}

// circularWalk builds the walk geometry of a circular path about (cu, cv).
//
// Its tangents REFUSE a bound (+Inf): the held components are math.Sincos
// evaluations at th0/th1, and those angles are themselves computed — a
// CircleSeg's from a float multiply by 2π, an ArcSeg's from math.Atan2 of the
// recorded differences — so neither the trig nor its argument is a quantity
// THIS function can enclose, holding floats alone. Stating zero there would
// hand a consumer an exactness the evaluator never proved; +Inf makes the
// absence visible, which is what every consumer refuses on.
//
// The endpoints are the same floats and carry the same absence, but they are
// not left at it: each caller holds the recorded segment those floats came
// from, and stamps the enclosure that record proves for its own endpoints
// (circularWalkEndBound) over the zero this function leaves behind. What has no
// enclosure is the tangent, not the point.
func circularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper float64) segmentWalk {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	sign := 1.0
	if th1 < th0 {
		sign = -1
	}
	length := r * math.Abs(th1-th0)
	lengthUpper := productUpper(radiusUpper, sweepUpper)
	coordUpper := absSumUpper(cu, cv, radiusUpper, radiusUpper)
	return segmentWalk{
		startU: cu + r*cos0, startV: cv + r*sin0,
		endU: cu + r*cos1, endV: cv + r*sin1,
		tanInU: -sign * sin0, tanInV: sign * cos0,
		tanOutU: -sign * sin1, tanOutV: sign * cos1,
		tanInBound:  math.Inf(1),
		tanOutBound: math.Inf(1),
		length:      length,
		lengthBound: conservativeValueError(length, lengthUpper),
		lengthUpper: lengthUpper,
		coordUpper:  coordUpper,
		kind:        walkCircular,
		cU:          cu, cV: cv, radius: r, th0: th0, th1: th1,
	}
}

// lineWalkBounds compares the held square root with the segment's exact
// rational squared length. A Pythagorean or axis-aligned length that lands
// exactly keeps a zero bound; every other square root uses the exact L1 length
// as a finite magnitude envelope, without assuming a Hypot ulp guarantee. It
// also returns an L1 coordinate envelope for later revolution bounds.
func lineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1), math.Inf(1), math.Inf(1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	heldRat := floatRat(held)
	coordUpper := math.Max(ratL1Upper(u0, v0), ratL1Upper(u1, v1))
	if heldRat != nil && new(big.Rat).Mul(heldRat, heldRat).Cmp(lengthSquared) == 0 {
		return 0, held, coordUpper
	}
	l1 := new(big.Rat).Add(new(big.Rat).Abs(du), new(big.Rat).Abs(dv))
	upper, exact := l1.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	bound := math.Min(conservativeValueError(held, upper), sqrtIntervalError(lengthSquared, held))
	return bound, upper, coordUpper
}

// sqrtIntervalError proves |held-sqrt(lengthSquared)| from the
// directed-rounding square root bracket (ratSqrtDown/ratSqrtUp,
// spline_length.go), assuming no ulp contract from Hypot or Sqrt. It returns
// +Inf when the bracket cannot be built (a non-finite endpoint), so a
// math.Min against it can only ever keep the caller's own bound.
func sqrtIntervalError(lengthSquared *big.Rat, held float64) float64 {
	lo, hi := floatRat(ratSqrtDown(lengthSquared)), floatRat(ratSqrtUp(lengthSquared))
	if lo == nil || hi == nil {
		return math.Inf(1)
	}
	return intervalFloatError(interval(lo, hi), held)
}

func ratL1Upper(values ...*big.Rat) float64 {
	total := new(big.Rat)
	for _, value := range values {
		total.Add(total, new(big.Rat).Abs(value))
	}
	upper, exact := total.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	return upper
}

// sideWalk is one side face's walk after canonicalization: consecutive
// collinear line walks coalesce into one (evaluator §3 — "adjacent coplanar
// side faces merge"), and the merged face carries every constituent
// segment's role.
type sideWalk struct {
	segmentWalk
	segs []int // the recorded segment indices this walk covers
}

// coalesceWalks merges consecutive collinear line walks, wrap-around
// included. Circular walks never merge; a loop that is entirely one straight
// line is degenerate and left to the area gate.
func coalesceWalks(walks []sideWalk) []sideWalk {
	out, _ := coalesceWalksBudget(walks, nil)
	return out
}

func coalesceWalksBudget(walks []sideWalk, budget *workBudget) ([]sideWalk, error) {
	return coalesceWalksWithPoll(func() error { return wallBudgetStep(budget) }, walks)
}

func coalesceWalksContext(ctx context.Context, walks []sideWalk) ([]sideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks)
}

func coalesceWalksWithPoll(poll func() error, walks []sideWalk) ([]sideWalk, error) {
	collinear := func(a, b sideWalk) bool {
		if !a.isLine() || !b.isLine() {
			return false
		}
		cross := a.tanOutU*b.tanInV - a.tanOutV*b.tanInU
		dot := a.tanOutU*b.tanInU + a.tanOutV*b.tanInV
		scale := math.Hypot(a.tanOutU, a.tanOutV) * math.Hypot(b.tanInU, b.tanInV)
		return dot > 0 && math.Abs(cross) <= 1e-12*scale
	}
	merge := func(a, b sideWalk) sideWalk {
		a.endU, a.endV = b.endU, b.endV
		// The merged walk leaves where b leaves, so it inherits b's leaving
		// tangent AND the bound b proved on it — never a's, and never zero.
		a.tanOutU, a.tanOutV = b.tanOutU, b.tanOutV
		a.tanOutBound = b.tanOutBound
		length := boundedAdd(measuredScalar(a.length, a.lengthBound), measuredScalar(b.length, b.lengthBound))
		a.length, a.lengthBound = length.value, length.bound
		a.lengthUpper = absSumUpper(a.lengthUpper, b.lengthUpper)
		a.coordUpper = math.Max(a.coordUpper, b.coordUpper)
		a.axisRadiusUpper = math.Max(a.axisRadiusUpper, b.axisRadiusUpper)
		a.axisMomentUpper = absSumUpper(a.axisMomentUpper, b.axisMomentUpper)
		a.segs = append(a.segs, b.segs...)
		return a
	}
	out := make([]sideWalk, 0, len(walks))
	for _, w := range walks {
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		if len(out) > 0 && collinear(out[len(out)-1], w) {
			out[len(out)-1] = merge(out[len(out)-1], w)
			continue
		}
		out = append(out, w)
	}
	// Wrap-around: the loop's last walk may continue into its first.
	for len(out) > 1 && collinear(out[len(out)-1], out[0]) {
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		out[0] = merge(out[len(out)-1], out[0])
		out = out[:len(out)-1]
	}
	return out, nil
}
