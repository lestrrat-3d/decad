package reportvocab

import "github.com/lestrrat-3d/units"

// ContactRelation is the proven relation of two complete occupied sets.
type ContactRelation int

const (
	ContactUndecided ContactRelation = iota
	ContactSeparated
	ContactTouching
	ContactOverlapping
	ContactBand
)

// SweepOutcome states the certified relation over a requested path.
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
	SweepPersistentBand
)

// SweepStartPolicy selects what a sweep proves when the bodies initially
// touch. Its constants match the root SweepStartPolicy's in value and order.
type SweepStartPolicy int

const (
	StopAtInitialContact SweepStartPolicy = iota
	ContinueSeparatingTouch
	ContinueCertifiedTouch
)

// SweepRequest is the internal form of the root package's SweepRequest
// (docs/contact-sweep-design.md §2, §7): the contact request every sampled
// pair query uses, the time resolution, the pose evaluation budget, the start
// policy and the rest speed of a band track. The sweep producers read it; the
// root package converts the caller's request to it once, at the public entry.
type SweepRequest struct {
	ContactRequest
	TimeResolution     units.Value
	MaxPoseEvaluations uint64
	StartPolicy        SweepStartPolicy
	RestSpeed          units.Value
}
