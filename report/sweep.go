package report

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/seal"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SweepStartPolicy selects what to prove when the bodies initially touch.
type SweepStartPolicy int

const (
	StopAtInitialContact SweepStartPolicy = iota
	ContinueSeparatingTouch
	ContinueCertifiedTouch
)

// SweepRequest bounds the time search and the contact geometry resolution.
//
// RestSpeed is a nonnegative Velocity; the zero Value rests no vertex. A band
// track over a support set (docs/multibody-dynamics-design.md §10.5) holds a
// lifted vertex that closes on the support plane no faster than RestSpeed, or
// rises, on both sides of the plane, as it holds a contact vertex (§10.8), so
// the track does not end where that vertex's lower height bound reaches the
// plane. A departure rests nothing.
type SweepRequest struct {
	ContactRequest
	TimeResolution     units.Value
	MaxPoseEvaluations uint64
	StartPolicy        SweepStartPolicy
	RestSpeed          units.Value
}

// SweepOutcome states the certified relation over the requested path.
type SweepOutcome int

const (
	SweepClear SweepOutcome = iota + 1
	SweepDepartedClear
	SweepPersistentTouch
	SweepContactTransitionBracket
	SweepImpactBracket
	SweepInitiallyTouching
	SweepInitiallyOverlapping
	SweepUndecided
	SweepGrazingTouch
	// SweepPersistentBand: the pair stays within a certified band of one
	// normal through its contact track. At every instant of the track the
	// signed separation along Normal() of every point the track publishes
	// lies in [−Depth, Depth], the interiors overlap by at most Depth along
	// that normal, every vertex of the touching body the track does not
	// publish stays strictly in front of the support plane, and the source
	// features are stable. Depth is the track's Band(). The track may end
	// before the duration.
	SweepPersistentBand
)

// SweepCause explains why a continuous claim was not proved.
type SweepCause int

const (
	SweepNoCause SweepCause = iota
	SweepPoseRelation
	SweepMissingBound
	SweepTimeFloor
	SweepFractionFloor
	SweepPoseBudget
	SweepContactUnsupported
	SweepDepartureUnproved
	SweepContactTrackUnproved
	SweepEventUnrepresentable
)

// SweepInstant identifies a dyadic fraction of the requested duration.
type SweepInstant struct {
	Fraction units.Value
	Elapsed  Measurement
}

// SweepInterval identifies an interval of the requested duration.
type SweepInterval struct{ From, To SweepInstant }

// SweepDeparture certifies positive separation after an initial touch.
type SweepDeparture struct {
	Until      SweepInstant
	GapAtUntil Measurement
}

// SweepEventOf reports the ideal path relation at one sampled instant.
type SweepEventOf[F, E, V any] struct {
	At       SweepInstant
	Relation ContactRelation
	Gap      *Measurement
	Overlap  *Measurement
	Manifold *ContactManifoldOf[F, E, V]
	Reason   ContactReason
}

// SweepSampleOf keeps the query pose and the transferred pair finding.
type SweepSampleOf[B, F, E, V any] struct {
	At            SweepInstant
	PoseA, PoseB  r3.Transform
	FloatContact  *ContactReportOf[B, F, E, V]
	Ideal         SweepEventOf[F, E, V]
	exactFraction *big.Rat
}

// ExactFraction returns the sample's exact fraction of the duration.
func (s *SweepSampleOf[B, F, E, V]) ExactFraction(seal.Key) *big.Rat { return s.exactFraction }

// SetExactFraction records the sample's exact fraction of the duration.
func (s *SweepSampleOf[B, F, E, V]) SetExactFraction(_ seal.Key, f *big.Rat) { s.exactFraction = f }

// SweepReplay is what a sweep report asks of its replay proof. The decad
// package supplies the proof; it is not part of the report's public surface.
type SweepReplay interface {
	// Present reports whether the proof exists; a nil proof answers false.
	Present() bool
	HasAffineProof() bool
	PosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error)
	PosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error)
}

// SweepReportOf states the first certified event or the earliest unresolved
// span. B, F, E and V are the body, face, edge and vertex reference types;
// T is the contact track type and R the replay proof type.
type SweepReportOf[B, F, E, V, T any, R SweepReplay] struct {
	A, B            B
	PathA, PathB    PairPath
	Request         SweepRequest
	Outcome         SweepOutcome
	Bracket         *SweepInterval
	Unresolved      *SweepInterval
	InitialEvent    *SweepEventOf[F, E, V]
	Event           *SweepEventOf[F, E, V]
	Departure       *SweepDeparture
	ContactTrack    *T
	Cause           SweepCause
	Samples         []SweepSampleOf[B, F, E, V]
	BoxExcluded     bool
	PoseEvaluations uint64
	replay          R
	bracketRight    *big.Rat
}

// Replay returns the report's replay proof.
func (r *SweepReportOf[B, F, E, V, T, R]) Replay(seal.Key) R { return r.replay }

// SetReplay records the report's replay proof.
func (r *SweepReportOf[B, F, E, V, T, R]) SetReplay(_ seal.Key, p R) { r.replay = p }

// BracketRight returns the exact right end of the bracket, or nil.
func (r *SweepReportOf[B, F, E, V, T, R]) BracketRight(seal.Key) *big.Rat { return r.bracketRight }

// SetBracketRight records the exact right end of the bracket.
func (r *SweepReportOf[B, F, E, V, T, R]) SetBracketRight(_ seal.Key, f *big.Rat) { r.bracketRight = f }

// BracketEndsAtDuration reports whether the sweep producer proved its bracket
// right endpoint is the exact final fraction of the requested duration.
func (r *SweepReportOf[B, F, E, V, T, R]) BracketEndsAtDuration() bool {
	return r != nil && r.Bracket != nil && r.bracketRight != nil &&
		r.bracketRight.Cmp(big.NewRat(1, 1)) == 0
}

// HasAffineReplayProof reports whether this sweep can certify rounded poses
// along an affine source-box, source-sphere, faceted clear, extruded or
// revolved source-cylinder, or oriented face-track path.
func (r *SweepReportOf[B, F, E, V, T, R]) HasAffineReplayProof() bool {
	return r != nil && r.replay.Present() && r.replay.HasAffineProof()
}

// CertifiedPosesAt evaluates the recorded paths at elapsed time and checks
// their rounded placements against the sweep's exact source proof.
// It reads no Document geometry and performs no new contact query. A pose whose
// rounding can change the reported relation beyond PointResolution is refused.
func (r *SweepReportOf[B, F, E, V, T, R]) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || !r.replay.Present() {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", decaderr.ErrUnsupported)
	}
	return r.replay.PosesAt(elapsed)
}

// CertifiedPosesAtInterval maps an exact held time from [start, end] onto the
// certified spatial path. It permits a caller to replay a slice whose global
// clock endpoints differ slightly from the rounded sweep duration.
func (r *SweepReportOf[B, F, E, V, T, R]) CertifiedPosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || !r.replay.Present() {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", decaderr.ErrUnsupported)
	}
	return r.replay.PosesAtInterval(time, start, end)
}
