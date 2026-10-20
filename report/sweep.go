package report

import "github.com/lestrrat-3d/units"

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
