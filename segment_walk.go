package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// This file is the package's profile-boundary walk: survey2d.SegmentWalk, the resolved
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

// requireAnalyticWalk refuses a free-form walk on behalf of a consumer that has
// no free-form construction yet. Reaching it is a staging limit, never a wrong
// answer — the reason each consumer stages is its own row in
// docs/spline-design.md Table R. The prism side-face build itself no longer
// calls this: buildLoopSidesAs switches on survey2d.WalkKind instead (§10 P4b), with its
// own free-form arm, and tessellate.go's chordLoop no longer calls it either:
// it switches on survey2d.WalkKind, with its own free-form chording arm
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
func requireAnalyticWalk(w survey2d.SegmentWalk, what string) error {
	if w.Kind != survey2d.WalkFreeform {
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
// cap-loop chamfer, the shell cup, Verify, and every re-evaluation path that
// has no resolution in hand pass nil and are unaffected, since they run over a
// DIFFERENT profile (a cap contour, an offset loop) or hold no resolution worth
// sharing.
//
// A metered set outlives the evaluation that resolved it. evalPrismContext
// publishes the one it resolved onto the body's own prismPayload, so a rigid
// re-evaluation of that record — Placed, Duplicate, PlacedCopy — reads the
// plane-local walks back instead of bracketing every free-form arc again. What
// makes that sound is that a walk holds NOTHING placement-dependent: walkOf
// answers in the section's own (u, v), and the frame, the placement and the
// reflection correction are applied by consumers downstream of it. The reuse is
// still guarded by matches on every read, so a record that differs by one float
// bit resolves afresh (docs/evaluator-design.md §8, docs/spline-design.md §5.2).
type profileWalks struct {
	// profile is the record every walk below was resolved FROM, kept whole so
	// that a read against another profile is caught by comparing the recorded
	// segments themselves rather than their shape (matches).
	profile ProfileRecord
	// outer holds loop index 0's resolved walks, one per pp.profile.Outer
	// segment, in recorded order.
	outer []survey2d.SegmentWalk
	// holes holds loop index i>0's resolved walks as holes[i-1], one slice
	// per pp.profile.Holes entry, each in recorded order — the same
	// append([]LoopRecord{profile.Outer}, profile.Holes...) indexing every
	// consumer below already walks.
	holes [][]survey2d.SegmentWalk
	// spent and reconstructionSpent are what resolving this set CHARGED each of
	// its counters when it ran, measured across the resolution rather than
	// estimated. A later re-evaluation that reads these walks back replays the
	// two figures onto its own counter (charge) so the record's ceilings bind it
	// exactly as the work itself would have (docs/spline-design.md §5.2).
	spent, reconstructionSpent uint64
	// metered says the two figures above were MEASURED by resolveProfileWalks.
	// A set assembled from walks resolved elsewhere — loft_stations.go builds
	// one as a per-station view over walks its own pairing already charged —
	// leaves it false, and charge refuses such a set rather than replaying a
	// zero it never measured.
	metered bool
	// readCharges is set only by a chain build that captured each walk at
	// its existing build position. Bounds replays that segment's measured
	// charges at each read, preserving the old per-read work ceiling.
	readCharges [][]walkReadCharge
}

type walkReadCharge struct {
	spent, reconstructionSpent uint64
}

// resolveProfileWalks resolves every segment of profile's outer loop and each
// hole loop through walkOf exactly once, charging work the same single time
// each segment's own conversion and length bracket cost (docs/spline-design.md
// §5.2), rather than once per consumer. The set it returns records what that
// cost, so a later rigid re-evaluation can replay the charge instead of
// re-running the work (charge).
func resolveProfileWalks(profile ProfileRecord, work *freeform.FreeformWork) (*profileWalks, error) {
	before, beforeRecon := workSpent(work)
	outer := make([]survey2d.SegmentWalk, len(profile.Outer.Segments))
	for i, seg := range profile.Outer.Segments {
		w, err := walkOf(seg, work)
		if err != nil {
			return nil, err
		}
		outer[i] = w
	}
	holes := make([][]survey2d.SegmentWalk, len(profile.Holes))
	for hi, hole := range profile.Holes {
		hw := make([]survey2d.SegmentWalk, len(hole.Segments))
		for i, seg := range hole.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			hw[i] = w
		}
		holes[hi] = hw
	}
	after, afterRecon := workSpent(work)
	return &profileWalks{
		profile:             profile,
		outer:               outer,
		holes:               holes,
		spent:               after - before,
		reconstructionSpent: afterRecon - beforeRecon,
		metered:             true,
	}, nil
}

// reusable reports whether pw may stand in for resolving profile again: it was
// resolved from exactly this record, and it knows what that resolution cost. A
// set failing either test is not read, and the caller resolves as before.
func (pw *profileWalks) reusable(profile ProfileRecord) bool {
	return pw != nil && pw.metered && pw.matches(profile)
}

// charge replays onto work the exact free-form and reconstruction work this
// resolution's own walkOf calls charged when they ran, so a re-evaluation that
// reads the walks back is bound by the record's ceilings exactly as the work
// itself would have bound it (docs/spline-design.md §5.2).
//
// The replay is one step per counter rather than the original per-segment
// sequence, and that changes no verdict: every charge is non-negative, so the
// running total is monotone and a sequence of steps refuses precisely when its
// SUM exceeds what the counter has left — the same condition, returning the same
// step error, as the one aggregate step. It cannot refuse EARLIER than the work
// would have either, because every walk this set holds already resolved.
func (pw *profileWalks) charge(work *freeform.FreeformWork) error {
	if pw == nil || !pw.metered {
		return errUnmeteredWalksCharge
	}
	if err := work.Step(pw.spent); err != nil {
		return err
	}
	return work.ReconstructionStep(pw.reconstructionSpent)
}

// workSpent reads both of a counter's totals. A nil counter has spent nothing,
// which is what step and reconstructionStep already treat it as.
func workSpent(work *freeform.FreeformWork) (uint64, uint64) {
	if work == nil {
		return 0, 0
	}
	return work.Spent, work.ReconstructionSpent
}

// at returns the resolved walk for loop index loopIndex (0 the outer loop,
// i>0 profile.Holes[i-1]) and segment index segIndex within that loop, in
// the same indexing every consumer's
// append([]LoopRecord{profile.Outer}, profile.Holes...) walk already uses.
// Callers check matches first; at itself trusts the index it is given.
func (pw *profileWalks) at(loopIndex, segIndex int) survey2d.SegmentWalk {
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
func (pw *profileWalks) loopWalks(loopIndex int) []survey2d.SegmentWalk {
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

// errUnmeteredWalksCharge reports a charge replay asked of a walk set that
// never measured its own cost — the same class of evaluator plumbing invariant
// as errResolvedWalksMismatch, and unreachable for the same reason: reusable
// gates every replay on metered, so only a future edit that read a set past
// that gate can reach it. Refusing is the safe direction: a set that cannot
// state its charge must never replay a zero in its place.
var errUnmeteredWalksCharge = fmt.Errorf(`%w: resolved walks did not measure their own work charge`, ErrUnsupported)

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
func walkOf(seg CurveSegment, work *freeform.FreeformWork) (survey2d.SegmentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	switch seg := seg.(type) {
	case LineSeg:
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		length := math.Hypot(du, dv)
		lengthBound, lengthUpper, coordUpper := lineWalkBounds(seg, length)
		tangentBound := lineWalkTangentBound(seg, du, dv)
		return survey2d.SegmentWalk{
			StartU: u0, StartV: v0, EndU: u1, EndV: v1,
			StartBound: lineWalkEndBound(seg, seg.TStart, u0, v0),
			EndBound:   lineWalkEndBound(seg, seg.TEnd, u1, v1),
			TanInU:     du, TanInV: dv, TanOutU: du, TanOutV: dv,
			TanInBound:  tangentBound,
			TanOutBound: tangentBound,
			Length:      length,
			LengthBound: lengthBound,
			LengthUpper: lengthUpper,
			CoordUpper:  coordUpper,
		}, nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return survey2d.SegmentWalk{}, fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return survey2d.SegmentWalk{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		th0, th1 := 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			r,
			th0,
			th1,
			math.Abs(r),
			proofbound.CircularSweepUpper(seg.TStart, seg.TEnd),
		)
		w.Closed = math.Abs(math.Abs(th1-th0)-2*math.Pi) < 1e-12
		w.StartBound = circularWalkEndBound(seg, seg.TStart, w.StartU, w.StartV)
		w.EndBound = circularWalkEndBound(seg, seg.TEnd, w.EndU, w.EndV)
		if iv, ok := circularLengthInterval(seg); ok {
			w.LengthBound = math.Min(w.LengthBound, proofbound.IntervalFloatError(iv, w.Length))
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
			proofbound.CircularSweepUpper(seg.TStart, seg.TEnd),
		)
		pinArcWalkEnds(&w, seg)
		// circularWalkEnclosures brackets the radius from the same exact
		// squared Start-to-Center distance arcWalkRadiusBound does, so its
		// radius interval IS that function's bracket and is read here rather
		// than built twice. Both ends are floatRat of a float, so Float64
		// returns those floats exactly. The enclosures answer false exactly
		// where the bracket overflows, and arcWalkRadiusBound answers +Inf
		// there on its own.
		if rIv, sweepIv, ok := circularWalkEnclosures(seg); ok {
			rLo, _ := rIv.Lo.Float64()
			rHi, _ := rIv.Hi.Float64()
			w.RadiusBound = arcRadiusBoundFromBracket(radius, rLo, rHi)
			w.LengthBound = math.Min(w.LengthBound, proofbound.IntervalFloatError(proofbound.IntervalMul(rIv, sweepIv), w.Length))
		} else {
			w.RadiusBound = arcWalkRadiusBound(seg, radius)
		}
		return w, nil
	default:
		if !isFreeformSegment(seg) {
			return survey2d.SegmentWalk{}, fmt.Errorf(`%w: this evaluator sweeps profiles of line, arc, circle and Tier A free-form segments only; the profile has a %T segment it cannot sweep into a side face yet`, ErrUnsupported, seg)
		}
		return freeformWalk(seg, work)
	}
}

// lineWalkTangentBound is the single owner of the proven bound on a line
// walk's tangent, and arcWalkRadiusBound's twin one field over: the record
// states the segment's endpoints and its parameter range, never the tangent,
// so the walk's held tangent is the float difference u1−u0, v1−v0 of two
// endpoints the float lerp already rounded. The tangent the record DENOTES is
// the difference of the exact lerps (dyLerp), which carries no rounding at
// either step, and the bound is the wider of the two components' gaps from it,
// rounded outward. A lerp that is not representable as a rational yields +Inf
// — the underivable bound consumers refuse on.
func lineWalkTangentBound(seg LineSeg, heldU, heldV float64) float64 {
	u0, okU0 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0, okV0 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1, okU1 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1, okV1 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if !okU0 || !okV0 || !okU1 || !okV1 {
		return math.Inf(1)
	}
	return math.Max(
		proofarith.DyRoundedFloatError(proofarith.DySubScalar(u1, u0), heldU),
		proofarith.DyRoundedFloatError(proofarith.DySubScalar(v1, v0), heldV),
	)
}

// lineWalkEndBound is the single owner of the proven bound on a LINE walk's
// endpoint, and lineWalkTangentBound's twin one field over: the record states
// the segment's endpoints and its parameter range, never the point at a trimmed
// parameter, so the walk's held endpoint is lerp2's float evaluation. The point
// the record DENOTES is the exact lerp (dyLerp), which carries no rounding at
// either step, and the bound is the wider of the two components' gaps from it,
// rounded outward. A natural bound needs no argument of its own: lerp2 and
// dyLerp both special-case t = 0 and t = 1 to the recorded Point2 verbatim, so
// the two agree exactly and this answers zero. A lerp that is not
// representable as a rational yields +Inf on its component — the underivable
// bound consumers refuse on.
func lineWalkEndBound(seg LineSeg, t, heldU, heldV float64) proofbound.WalkEndBound {
	out := proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
	if u, ok := proofarith.DyLerp(seg.Start.U, seg.End.U, t); ok {
		out.U = proofarith.DyRoundedFloatError(u, heldU)
	}
	if v, ok := proofarith.DyLerp(seg.Start.V, seg.End.V, t); ok {
		out.V = proofarith.DyRoundedFloatError(v, heldV)
	}
	return out
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
func circularWalkEndBound(seg CurveSegment, t, heldU, heldV float64) proofbound.WalkEndBound {
	rt := proofarith.FloatRat(t)
	if rt == nil {
		return proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
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
func circularPointBound(seg CurveSegment, t *big.Rat, heldU, heldV float64) proofbound.WalkEndBound {
	uIv, vIv, ok := circularEndpointInterval(seg, t)
	if !ok {
		return proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
	}
	return proofbound.WalkEndBound{
		U: proofbound.IntervalFloatError(uIv, heldU),
		V: proofbound.IntervalFloatError(vIv, heldV),
	}
}

// arcWalkRadiusBound is the single owner of the proven bound on an ArcSeg
// walk's radius, and the reason survey2d.SegmentWalk carries radiusBound at all: the
// record states Start and Center, never the radius, so the walk's held radius
// is the float math.Hypot of their difference. The exact radius is
// √((Su−Cu)² + (Sv−Cv)²) over the recorded coordinates, which proofbound.RatSqrtDown and
// proofbound.RatSqrtUp bracket without rounding, and the bound is the wider side of that
// bracket about the held float, rounded outward. A bracket that overflows
// yields +Inf — an underivable bound, which every consumer refuses on rather
// than publishes.
func arcWalkRadiusBound(seg ArcSeg, held float64) float64 {
	dx := exactCoordinateDelta(seg.Start.U, seg.Center.U)
	dy := exactCoordinateDelta(seg.Start.V, seg.Center.V)
	r2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	rLo, rHi := proofbound.RatSqrtDown(r2), proofbound.RatSqrtUp(r2)
	if proofbound.IsNonFinite(rLo) || proofbound.IsNonFinite(rHi) {
		return math.Inf(1)
	}
	return arcRadiusBoundFromBracket(held, rLo, rHi)
}

// arcRadiusBoundFromBracket is arcWalkRadiusBound's formula over an already
// built radius bracket [rLo, rHi]: the wider side of the bracket about the
// held radius, rounded outward. It exists so walkOf, which reads the same
// bracket out of circularWalkEnclosures, states the formula through its one
// owner instead of copying it.
func arcRadiusBoundFromBracket(held, rLo, rHi float64) float64 {
	return math.Max(proofbound.UpRound(held-rLo), proofbound.UpRound(rHi-held))
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
//     (R15); freeform.FreeformArcLength owns both;
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
func freeformWalk(seg CurveSegment, work *freeform.FreeformWork) (survey2d.SegmentWalk, error) {
	if work == nil {
		return survey2d.SegmentWalk{}, errFreeformWalkUncounted
	}
	spans, reversed, err := freeformBezierSpans(seg, work)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	start, end, err := freeformEndpoints(spans, reversed)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	length, bound, err := freeform.FreeformArcLength(spans, work)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	tangents, err := freeform.FreeformEndTangents(spans, reversed)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	startBound, endBound := freeformEndpointBounds(spans, reversed, start, end)
	return survey2d.SegmentWalk{
		StartU: start.U, StartV: start.V,
		EndU: end.U, EndV: end.V,
		StartBound: startBound,
		EndBound:   endBound,
		// A closed free-form curve returns to its start, so it carries no
		// junction vertex — the same fact CircleSeg's closed walk states.
		Closed:          start == end,
		TanInU:          tangents.InU,
		TanInV:          tangents.InV,
		TanInBound:      tangents.InBound,
		TanOutU:         tangents.OutU,
		TanOutV:         tangents.OutV,
		TanOutBound:     tangents.OutBound,
		Length:          length,
		LengthBound:     bound,
		LengthUpper:     proofbound.UpRound(length + bound),
		CoordUpper:      freeform.FreeformControlExtent(spans),
		Kind:            survey2d.WalkFreeform,
		Spans:           spans,
		Reversed:        reversed,
		FitInterpolated: isFitSplineSeg(seg),
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
func pinArcWalkEnds(w *survey2d.SegmentWalk, seg ArcSeg) {
	w.StartU, w.StartV, w.StartBound = arcWalkEnd(seg, seg.TStart, w.StartU, w.StartV)
	w.EndU, w.EndV, w.EndBound = arcWalkEnd(seg, seg.TEnd, w.EndU, w.EndV)
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
func arcWalkEnd(seg ArcSeg, t, heldU, heldV float64) (float64, float64, proofbound.WalkEndBound) {
	switch t {
	case 0:
		return seg.Start.U, seg.Start.V, proofbound.WalkEndBound{}
	case 1:
		return seg.End.U, seg.End.V, proofbound.WalkEndBound{}
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
func circularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper float64) survey2d.SegmentWalk {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	sign := 1.0
	if th1 < th0 {
		sign = -1
	}
	length := r * math.Abs(th1-th0)
	lengthUpper := proofbound.ProductUpper(radiusUpper, sweepUpper)
	coordUpper := proofbound.AbsSumUpper(cu, cv, radiusUpper, radiusUpper)
	return survey2d.SegmentWalk{
		StartU: cu + r*cos0, StartV: cv + r*sin0,
		EndU: cu + r*cos1, EndV: cv + r*sin1,
		TanInU: -sign * sin0, TanInV: sign * cos0,
		TanOutU: -sign * sin1, TanOutV: sign * cos1,
		TanInBound:  math.Inf(1),
		TanOutBound: math.Inf(1),
		Length:      length,
		LengthBound: proofbound.ConservativeValueError(length, lengthUpper),
		LengthUpper: lengthUpper,
		CoordUpper:  coordUpper,
		Kind:        survey2d.WalkCircular,
		CU:          cu, CV: cv, Radius: r, Th0: th0, Th1: th1,
	}
}

// lineWalkBounds compares the held square root with the segment's exact
// squared length, a polynomial in the recorded floats and hence a dyadic
// (dyLerp). A Pythagorean or axis-aligned length that lands exactly keeps a
// zero bound; every other square root uses the exact L1 length as a finite
// magnitude envelope, without assuming a Hypot ulp guarantee. It also returns
// an L1 coordinate envelope for later revolution bounds.
func lineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	u0, okU0 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0, okV0 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1, okU1 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1, okV1 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if !okU0 || !okV0 || !okU1 || !okV1 {
		return math.Inf(1), math.Inf(1), math.Inf(1)
	}
	du := proofarith.DySubScalar(u1, u0)
	dv := proofarith.DySubScalar(v1, v0)
	lengthSquared := proofarith.DyAdd(proofarith.DyMul(du, du), proofarith.DyMul(dv, dv))
	coordUpper := math.Max(proofarith.DyL1Upper(u0, v0), proofarith.DyL1Upper(u1, v1))
	if proofarith.DySquareEquals(held, lengthSquared) {
		return 0, held, coordUpper
	}
	upper := proofarith.DyL1Upper(du, dv)
	bound := math.Min(proofbound.ConservativeValueError(held, upper), dySqrtIntervalError(lengthSquared, held))
	return bound, upper, coordUpper
}

// dySqrtIntervalError proves |held − sqrt(lengthSquared)| from the
// directed-rounding square root bracket (dyadic.go's dySqrtDown/dySqrtUp),
// assuming no ulp contract from Hypot or Sqrt. The answer is the farther of the
// held float's two gaps from the bracket's ends, each rounded outward through
// dyRoundedFloatError — proofbound.IntervalFloatError's rule over this arithmetic. It
// returns +Inf when the bracket cannot be built (an end past MaxFloat64), so a
// math.Min against it can only ever keep the caller's own bound.
func dySqrtIntervalError(lengthSquared proofarith.Dyadic, held float64) float64 {
	lo, okLo := proofarith.DyOf(proofarith.DySqrtDown(lengthSquared))
	hi, okHi := proofarith.DyOf(proofarith.DySqrtUp(lengthSquared))
	if !okLo || !okHi {
		return math.Inf(1)
	}
	return math.Max(proofarith.DyRoundedFloatError(lo, held), proofarith.DyRoundedFloatError(hi, held))
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

// coalesceWalks merges consecutive collinear line walks, wrap-around
// included. Circular walks never merge; a loop that is entirely one straight
// line is degenerate and left to the area gate.
func coalesceWalks(walks []survey2d.SideWalk) []survey2d.SideWalk {
	out, _ := coalesceWalksBudget(walks, nil)
	return out
}

func coalesceWalksBudget(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, walks, true)
}

func coalesceWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks, true)
}

// coalesceChainWalksContext is coalesceWalksContext's OPEN-walk counterpart:
// it merges adjacent collinear segments exactly as a loop's coalescing does,
// but never wraps the last walk into the first. An open chain's two ends are
// free — they meet no neighbour to merge into
// (docs/surface-design.md §13.4).
func coalesceChainWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks, false)
}

func coalesceWalksWithPoll(poll func() error, walks []survey2d.SideWalk, wrap bool) ([]survey2d.SideWalk, error) {
	collinear := func(a, b survey2d.SideWalk) bool {
		if !a.IsLine() || !b.IsLine() {
			return false
		}
		cross := a.TanOutU*b.TanInV - a.TanOutV*b.TanInU
		dot := a.TanOutU*b.TanInU + a.TanOutV*b.TanInV
		scale := math.Hypot(a.TanOutU, a.TanOutV) * math.Hypot(b.TanInU, b.TanInV)
		return dot > 0 && math.Abs(cross) <= 1e-12*scale
	}
	merge := func(a, b survey2d.SideWalk) survey2d.SideWalk {
		a.EndU, a.EndV = b.EndU, b.EndV
		// The merged walk leaves where b leaves, so it inherits b's leaving
		// tangent AND the bound b proved on it — never a's, and never zero.
		a.TanOutU, a.TanOutV = b.TanOutU, b.TanOutV
		a.TanOutBound = b.TanOutBound
		length := proofbound.BoundedAdd(proofbound.MeasuredScalar(a.Length, a.LengthBound), proofbound.MeasuredScalar(b.Length, b.LengthBound))
		a.Length, a.LengthBound = length.Value, length.Bound
		a.LengthUpper = proofbound.AbsSumUpper(a.LengthUpper, b.LengthUpper)
		a.CoordUpper = math.Max(a.CoordUpper, b.CoordUpper)
		a.AxisRadiusUpper = math.Max(a.AxisRadiusUpper, b.AxisRadiusUpper)
		a.AxisMomentUpper = proofbound.AbsSumUpper(a.AxisMomentUpper, b.AxisMomentUpper)
		a.Segs = append(a.Segs, b.Segs...)
		return a
	}
	out := make([]survey2d.SideWalk, 0, len(walks))
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
	// Wrap-around: a closed loop's last walk may continue into its first. An
	// open chain's never does (wrap is false), since its last segment meets
	// no neighbour at all.
	for wrap {
		if len(out) <= 1 || !collinear(out[len(out)-1], out[0]) {
			break
		}
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
