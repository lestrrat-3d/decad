package decad

import (
	"fmt"
	"math"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file caches the profile-boundary walks extrude, revolve and loft read.
// internal/walkconvert builds each survey2d.SegmentWalk from a recorded segment.
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
