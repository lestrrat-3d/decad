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
// span. B, F, E and V are the body, face, edge and vertex reference types and
// R is the replay proof type.
type SweepReportOf[B, F, E, V any, R SweepReplay] struct {
	A, B            B
	PathA, PathB    PairPath
	Request         SweepRequest
	Outcome         SweepOutcome
	Bracket         *SweepInterval
	Unresolved      *SweepInterval
	InitialEvent    *SweepEventOf[F, E, V]
	Event           *SweepEventOf[F, E, V]
	Departure       *SweepDeparture
	ContactTrack    *SweepContactTrackOf[F, E, V]
	Cause           SweepCause
	Samples         []SweepSampleOf[B, F, E, V]
	BoxExcluded     bool
	PoseEvaluations uint64
	replay          R
	bracketRight    *big.Rat
}

// Replay returns the report's replay proof.
func (r *SweepReportOf[B, F, E, V, R]) Replay(seal.Key) R { return r.replay }

// SetReplay records the report's replay proof.
func (r *SweepReportOf[B, F, E, V, R]) SetReplay(_ seal.Key, p R) { r.replay = p }

// BracketRight returns the exact right end of the bracket, or nil.
func (r *SweepReportOf[B, F, E, V, R]) BracketRight(seal.Key) *big.Rat { return r.bracketRight }

// SetBracketRight records the exact right end of the bracket.
func (r *SweepReportOf[B, F, E, V, R]) SetBracketRight(_ seal.Key, f *big.Rat) { r.bracketRight = f }

// BracketEndsAtDuration reports whether the sweep producer proved its bracket
// right endpoint is the exact final fraction of the requested duration.
func (r *SweepReportOf[B, F, E, V, R]) BracketEndsAtDuration() bool {
	return r != nil && r.Bracket != nil && r.bracketRight != nil &&
		r.bracketRight.Cmp(big.NewRat(1, 1)) == 0
}

// HasAffineReplayProof reports whether this sweep can certify rounded poses
// along an affine source-box, source-sphere, faceted clear, extruded or
// revolved source-cylinder, or oriented face-track path.
func (r *SweepReportOf[B, F, E, V, R]) HasAffineReplayProof() bool {
	return r != nil && r.replay.Present() && r.replay.HasAffineProof()
}

// CertifiedPosesAt evaluates the recorded paths at elapsed time and checks
// their rounded placements against the sweep's exact source proof.
// It reads no Document geometry and performs no new contact query. A pose whose
// rounding can change the reported relation beyond PointResolution is refused.
func (r *SweepReportOf[B, F, E, V, R]) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || !r.replay.Present() {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", decaderr.ErrUnsupported)
	}
	return r.replay.PosesAt(elapsed)
}

// CertifiedPosesAtInterval maps an exact held time from [start, end] onto the
// certified spatial path. It permits a caller to replay a slice whose global
// clock endpoints differ slightly from the rounded sweep duration.
func (r *SweepReportOf[B, F, E, V, R]) CertifiedPosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || !r.replay.Present() {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", decaderr.ErrUnsupported)
	}
	return r.replay.PosesAtInterval(time, start, end)
}

// SweepTrack is what a sweep contact track asks of its exact proof. The decad
// package supplies the proof; it is not part of the track's public surface.
type SweepTrack[F, E, V any] interface {
	Start() SweepInstant
	End() SweepInstant
	Features() (ContactFeatureOf[F, E, V], ContactFeatureOf[F, E, V])
	Normal() VecMeasurement
	Band() *Measurement
	BandAt(fraction units.Value) (*Measurement, error)
	ManifoldAt(fraction units.Value) (*ContactManifoldOf[F, E, V], error)
}

// SweepContactTrackOf owns the exact source geometry and affine motion of a
// certified touching prefix. Its source face references are the original
// faces.
type SweepContactTrackOf[F, E, V any] struct {
	proof  SweepTrack[F, E, V]
	source any
}

// NewSweepContactTrack wraps a producer's exact track proof. source is the
// producer's own record of it, which Source returns.
func NewSweepContactTrack[F, E, V any](_ seal.Key, proof SweepTrack[F, E, V], source any) *SweepContactTrackOf[F, E, V] {
	return &SweepContactTrackOf[F, E, V]{proof: proof, source: source}
}

// Source returns the producer record the track was made from.
func (t *SweepContactTrackOf[F, E, V]) Source(seal.Key) any { return t.source }

// Start is the first instant of the certified interval.
func (t *SweepContactTrackOf[F, E, V]) Start() SweepInstant { return t.proof.Start() }

// End is the last instant of the certified interval.
func (t *SweepContactTrackOf[F, E, V]) End() SweepInstant { return t.proof.End() }

// Features names the two source features that stay in contact over the track.
func (t *SweepContactTrackOf[F, E, V]) Features() (ContactFeatureOf[F, E, V], ContactFeatureOf[F, E, V]) {
	return t.proof.Features()
}

// Normal is the contact normal, pointing from the first body to the second.
func (t *SweepContactTrackOf[F, E, V]) Normal() VecMeasurement { return t.proof.Normal() }

// Band returns the depth of a band track (docs/multibody-dynamics-design.md
// §10.3), a Length Measurement whose Value plus Bound bounds every contact
// point's separation in both directions and the interiors' overlap along
// Normal(). It is nil for an exact touch track.
func (t *SweepContactTrackOf[F, E, V]) Band() *Measurement { return t.proof.Band() }

// BandAt returns the depth of a band track over its prefix through fraction
// (docs/multibody-dynamics-design.md §10.3): a Length Measurement whose Value
// plus Bound bounds every contact point's separation in both directions and
// the interiors' overlap along Normal() at every instant from the track's
// start through fraction. At the track's end it is at most Band(); it grows
// with fraction, so a caller can end a slice where the band reaches its own
// residual. It is nil for an exact touch track.
func (t *SweepContactTrackOf[F, E, V]) BandAt(fraction units.Value) (*Measurement, error) {
	return t.proof.BandAt(fraction)
}

// ManifoldAt returns a fresh reduction of the complete exact touching set at
// a fraction in this track's certified interval.
func (t *SweepContactTrackOf[F, E, V]) ManifoldAt(fraction units.Value) (*ContactManifoldOf[F, E, V], error) {
	return t.proof.ManifoldAt(fraction)
}

// liftedTrack answers a track's questions in another reference instantiation.
type liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2 any] struct {
	inner SweepTrack[F1, E1, V1]
	lift  *Lift[B1, F1, E1, V1, B2, F2, E2, V2]
}

func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) Start() SweepInstant { return t.inner.Start() }
func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) End() SweepInstant   { return t.inner.End() }
func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) Normal() VecMeasurement {
	return t.inner.Normal()
}
func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) Band() *Measurement { return t.inner.Band() }

func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) BandAt(fraction units.Value) (*Measurement, error) {
	return t.inner.BandAt(fraction)
}

func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) Features() (ContactFeatureOf[F2, E2, V2], ContactFeatureOf[F2, E2, V2]) {
	a, b := t.inner.Features()
	return t.lift.Feature(a), t.lift.Feature(b)
}

func (t liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]) ManifoldAt(fraction units.Value) (*ContactManifoldOf[F2, E2, V2], error) {
	manifold, err := t.inner.ManifoldAt(fraction)
	return t.lift.Manifold(manifold), err
}

// LiftSweepContactTrack returns t with its features and manifolds mapped by l.
// It keeps t's source record; a nil track stays nil. The track holds l by
// pointer, so tracks lifted by one Lift compare deeply equal.
func LiftSweepContactTrack[B1, F1, E1, V1, B2, F2, E2, V2 any](_ seal.Key, t *SweepContactTrackOf[F1, E1, V1],
	l *Lift[B1, F1, E1, V1, B2, F2, E2, V2]) *SweepContactTrackOf[F2, E2, V2] {
	if t == nil {
		return nil
	}
	return &SweepContactTrackOf[F2, E2, V2]{
		proof:  liftedTrack[B1, F1, E1, V1, B2, F2, E2, V2]{inner: t.proof, lift: l},
		source: t.source,
	}
}

// LiftSweepReport returns r with every body, face, edge and vertex reference
// mapped by l. The replay proof, the paths, the request and the exact bracket
// end carry over; each sample keeps its exact fraction, and the contact track
// keeps its producer record. A report whose Event is its InitialEvent keeps
// them one event. A nil report stays nil.
func LiftSweepReport[B1, F1, E1, V1 any, R SweepReplay, B2, F2, E2, V2 any](key seal.Key, r *SweepReportOf[B1, F1, E1, V1, R],
	l *Lift[B1, F1, E1, V1, B2, F2, E2, V2]) *SweepReportOf[B2, F2, E2, V2, R] {
	if r == nil {
		return nil
	}
	out := &SweepReportOf[B2, F2, E2, V2, R]{
		A: l.Body(r.A), B: l.Body(r.B), PathA: r.PathA, PathB: r.PathB, Request: r.Request,
		Outcome: r.Outcome, Bracket: r.Bracket, Unresolved: r.Unresolved,
		InitialEvent: l.SweepEvent(r.InitialEvent), Departure: r.Departure,
		ContactTrack: LiftSweepContactTrack(key, r.ContactTrack, l), Cause: r.Cause,
		BoxExcluded: r.BoxExcluded, PoseEvaluations: r.PoseEvaluations,
		replay: r.replay, bracketRight: r.bracketRight,
	}
	if r.Event != nil && r.Event == r.InitialEvent {
		out.Event = out.InitialEvent
	} else {
		out.Event = l.SweepEvent(r.Event)
	}
	if r.Samples != nil {
		out.Samples = make([]SweepSampleOf[B2, F2, E2, V2], len(r.Samples))
		for i := range r.Samples {
			s := &r.Samples[i]
			out.Samples[i] = SweepSampleOf[B2, F2, E2, V2]{At: s.At, PoseA: s.PoseA, PoseB: s.PoseB,
				Ideal: *l.SweepEvent(&s.Ideal), exactFraction: s.exactFraction}
			if s.FloatContact != nil {
				out.Samples[i].FloatContact = l.Report(s.FloatContact)
			}
		}
	}
	return out
}
