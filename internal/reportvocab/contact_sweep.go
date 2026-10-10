package reportvocab

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

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
	Elapsed  measurement.Measurement
}

// SweepInterval identifies an interval of the requested duration.
type SweepInterval struct{ From, To SweepInstant }

// SweepDeparture certifies positive separation after an initial touch.
type SweepDeparture struct {
	Until      SweepInstant
	GapAtUntil measurement.Measurement
}

// SweepEvent reports the ideal path relation at one sampled instant.
type SweepEvent[FaceT, EdgeT, VertexT comparable, RelationT, ReasonT any] struct {
	At       SweepInstant
	Relation RelationT
	Gap      *measurement.Measurement
	Overlap  *measurement.Measurement
	Manifold *ContactManifold[FaceT, EdgeT, VertexT]
	Reason   ReasonT
}
