package momentinput

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file caches the profile-boundary walks extrude, revolve and loft read.
// internal/boundarywalk builds each survey2d.SegmentWalk from a recorded segment.
//
// Extrude, revolve and loft all read walks through ProfileWalks, which
// resolves a whole profile once and re-checks, on every later read, that the
// walks it holds still match the record they were resolved from.

// ProfileWalks is one profile's segment walks resolved ONCE, so that every
// consumer within a single prism evaluation reads the same resolution back
// instead of paying walkOf's own §5.2 charge again for it. Within one
// evalPrismContext call, buildLoopSidesAs, CoordinateEnvelope (called
// from prismCentroidGeometryBound and, four times over, from
// prismBoundsContext's per-axis extentBoundedAlong) and
// boundaryExtremesBoundedContext (three times, also from extentBoundedAlong)
// each used to call walkOf on the SAME recorded segment — eight resolutions of
// one segment, each recharging §5.2's exact-rational counter in full. On a
// 15-point involute fit spline that alone charged 230,168 units eight times
// over, tripping the R7 ceiling on a record whose deduplicated charge fits
// comfortably inside it. ProfileWalks is the fix: resolve every segment's walk
// once and let each consumer read it back.
//
// A nil *ProfileWalks means "resolve as before" everywhere below: revolve, the
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
// still guarded by Matches on every read, so a record that differs by one float
// bit resolves afresh (docs/evaluator-design.md §8, docs/spline-design.md §5.2).
type ProfileWalks struct {
	// Profile is the record every walk below was resolved FROM, kept whole so
	// that a read against another profile is caught by comparing the recorded
	// segments themselves rather than their shape (Matches).
	Profile Profile
	// Outer holds loop index 0's resolved walks, one per Profile.Outer
	// segment, in recorded order.
	Outer []survey2d.SegmentWalk
	// Holes holds loop index i>0's resolved walks as Holes[i-1], one slice
	// per Profile.Holes entry, each in recorded order — the same
	// append([]LoopRecord{profile.Outer}, profile.Holes...) indexing every
	// consumer below already walks.
	Holes [][]survey2d.SegmentWalk
	// Spent and ReconstructionSpent are what resolving this set CHARGED each of
	// its counters when it ran, measured across the resolution rather than
	// estimated. A later re-evaluation that reads these walks back replays the
	// two figures onto its own counter (Charge) so the record's ceilings bind it
	// exactly as the work itself would have (docs/spline-design.md §5.2).
	Spent, ReconstructionSpent uint64
	// Metered says the two figures above were MEASURED by ResolveProfileWalks.
	// A set assembled from walks resolved elsewhere — loft_stations.go builds
	// one as a per-station view over walks its own pairing already charged —
	// leaves it false, and Charge refuses such a set rather than replaying a
	// zero it never measured.
	Metered bool
	// ReadCharges is set only by a chain build that captured each walk at
	// its existing build position. Bounds replays that segment's measured
	// charges at each read, preserving the old per-read work ceiling.
	ReadCharges [][]WalkReadCharge
}

type WalkReadCharge struct {
	Spent, ReconstructionSpent uint64
}

// ResolveProfileWalks resolves every segment of profile's outer loop and each
// hole loop through walkOf exactly once, charging work the same single time
// each segment's own conversion and length bracket cost (docs/spline-design.md
// §5.2), rather than once per consumer. The set it returns records what that
// cost, so a later rigid re-evaluation can replay the charge instead of
// re-running the work (Charge).
func ResolveProfileWalks(profile Profile, work *freeform.FreeformWork) (*ProfileWalks, error) {
	resolved, err := boundarywalk.ResolveProfile(boundarywalk.Profile{Outer: profile.Outer, Holes: profile.Holes}, work)
	if err != nil {
		return nil, err
	}
	return &ProfileWalks{
		Profile:             profile,
		Outer:               resolved.Outer,
		Holes:               resolved.Holes,
		Spent:               resolved.Spent,
		ReconstructionSpent: resolved.ReconstructionSpent,
		Metered:             true,
	}, nil
}

// Reusable reports whether pw may stand in for resolving profile again: it was
// resolved from exactly this record, and it knows what that resolution cost. A
// set failing either test is not read, and the caller resolves as before.
func (pw *ProfileWalks) Reusable(profile Profile) bool {
	return pw != nil && pw.Metered && pw.Matches(profile)
}

// Charge replays onto work the exact free-form and reconstruction work this
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
func (pw *ProfileWalks) Charge(work *freeform.FreeformWork) error {
	if pw == nil || !pw.Metered {
		return ErrUnmeteredWalksCharge
	}
	if err := work.Step(pw.Spent); err != nil {
		return err
	}
	return work.ReconstructionStep(pw.ReconstructionSpent)
}

// At returns the resolved walk for loop index loopIndex (0 the outer loop,
// i>0 profile.Holes[i-1]) and segment index segIndex within that loop, in
// the same indexing every consumer's
// append([]LoopRecord{profile.Outer}, profile.Holes...) walk already uses.
// Callers check Matches first; At itself trusts the index it is given.
func (pw *ProfileWalks) At(loopIndex, segIndex int) survey2d.SegmentWalk {
	if loopIndex == 0 {
		return pw.Outer[segIndex]
	}
	return pw.Holes[loopIndex-1][segIndex]
}

// LoopWalks returns the resolved walk slice for loop index loopIndex (the
// same convention as At), or nil if loopIndex is out of range for pw. A
// single-loop consumer (buildLoopSidesAs) uses this instead of At plus its
// own per-segment loop, since it already owns the per-segment index into the
// slice it gets back.
func (pw *ProfileWalks) LoopWalks(loopIndex int) []survey2d.SegmentWalk {
	if loopIndex == 0 {
		return pw.Outer
	}
	hi := loopIndex - 1
	if hi < 0 || hi >= len(pw.Holes) {
		return nil
	}
	return pw.Holes[hi]
}

// Matches reports whether pw was resolved from THIS profile: the same loops
// in the same order, each holding the same recorded segments — the same
// variant with the same field values, compared exactly by sectionrecord.IdenticalRecord.
// Shape alone is not enough, and never was: two profiles can carry the same
// outer, hole and per-hole segment counts while every coordinate differs, and
// a set resolved from one read against the other would report the first
// section's geometry as the second's, silently.
//
// Every consumer that reads a non-nil *ProfileWalks checks this FIRST and
// refuses rather than reading it — docs/spline-design.md §5.2's own discipline
// extended to this cache. The refusal is one-directional, like every other
// decad-side check: only an exact agreement between the two records reads the
// cache, and anything else — a differing value, a differing variant, a shape
// the comparison does not know how to traverse — refuses. There is no
// tolerance and no "close enough" arm, so a near-miss profile is rejected on
// the same terms as an unrelated one, and a plumbing bug never hides behind a
// correct-looking answer.
func (pw *ProfileWalks) Matches(profile Profile) bool {
	if pw == nil {
		return false
	}
	return sectionrecord.IdenticalRecord(pw.Profile, profile)
}

// LoopMatches reports whether pw holds, at loop index loopIndex (the same
// convention as At), the walks resolved from exactly this loop's recorded
// segments. It is Matches for the single-loop consumer buildLoopSidesAs,
// which is handed one LoopRecord and a role index rather than the whole
// profile, and it refuses on the same exact-comparison terms.
func (pw *ProfileWalks) LoopMatches(loopIndex int, loop LoopRecord) bool {
	if pw == nil {
		return false
	}
	loops := append([]LoopRecord{pw.Profile.Outer}, pw.Profile.Holes...)
	if loopIndex < 0 || loopIndex >= len(loops) {
		return false
	}
	return sectionrecord.IdenticalRecord(loops[loopIndex], loop)
}

// ErrResolvedWalksMismatch reports a *ProfileWalks handed to a consumer that
// does not match the profile it is read against — an evaluator plumbing
// invariant break, never a caller-reachable refusal: every call site in this
// package resolves walks from the exact profile it later reads them against,
// so reaching this error means a future edit broke that pairing, not that the
// recorded geometry is at fault.
var ErrResolvedWalksMismatch = fmt.Errorf(`%w: resolved walks do not match the profile they are read against`, ErrUnsupported)

// ErrUnmeteredWalksCharge reports a charge replay asked of a walk set that
// never measured its own cost — the same class of evaluator plumbing invariant
// as ErrResolvedWalksMismatch, and unreachable for the same reason: Reusable
// gates every replay on metered, so only a future edit that read a set past
// that gate can reach it. Refusing is the safe direction: a set that cannot
// state its charge must never replay a zero in its place.
var ErrUnmeteredWalksCharge = fmt.Errorf(`%w: resolved walks did not measure their own work charge`, ErrUnsupported)
