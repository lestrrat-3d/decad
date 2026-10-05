package dynamics

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type BodyLoad struct {
	Body   *decad.Body
	Force  QuantityVec
	Torque QuantityVec
}

type KinematicDriver struct {
	Body *decad.Body
	Path decad.PairPath
}

type StepInput struct {
	Gravity QuantityVec
	Loads   []BodyLoad
	Drivers []KinematicDriver
}

type StepStatus int

const (
	Advanced StepStatus = iota + 1
	Undecided
)

// StepReason classifies why a step stopped (docs/multibody-dynamics-design.md
// §12).
type StepReason int

const (
	StepNoReason           StepReason = iota
	StepPairUndecided                 // a candidate sweep is Undecided before the next event
	StepManifoldMissing               // an event pair has no manifold within StepConfig.Contact
	StepIslandDegenerate              // a closing constraint with no dynamic body, or K <= 0, or a non-finite proposal
	StepIslandResidual                // a solver gate exceeds its limit at MaxIterations
	StepCorrectionFailed              // a position correction exceeds its allowance or loses a relation
	StepTrackUnproved                 // a contact-set pair has neither a persistent nor a band track
	StepKickUnbounded                 // the force kick cannot be bounded within the velocity residuals
	StepConservationFailed            // an island or step conservation gate fails
	StepEventBudget                   // MaxEvents reached with time remaining
	StepPairBudget                    // MaxPairSweeps reached
	StepTravelUnbounded               // SweptBox returned ErrUnsupported for a body
	StepFixedPairRelation             // a non-excluded Fixed/Fixed pair is Overlapping or Undecided
	StepUnsupported                   // the current phase has no solver for this event family
)

// StepDiagnostic says why a step is Undecided; its typed fields are
// docs/multibody-dynamics-design.md §12.
type StepDiagnostic struct {
	Code StepReason
	// Pair is the responsible pair, when one is.
	Pair BodyPair
	// Bodies lists the responsible island's bodies in world order, when an
	// island is responsible.
	Bodies []*decad.Body
	// From and To bound the time interval the diagnostic applies to, from the
	// start of the step.
	From, To units.Value
	// Limit is the configured limit a residual, allowance or budget exceeded,
	// when one did. A count limit such as MaxEvents is a dimensionless scalar.
	Limit units.Value
	// Reason is a human-readable message; callers branch on Code.
	Reason string
}

// ContactEventKind separates an impulse-bearing contact from a geometry-only transition.
type ContactEventKind int

const (
	ContactImpact ContactEventKind = iota + 1
	ContactTransition
	ContactGraze
)

type ContactEvent struct {
	Kind ContactEventKind
	Pair BodyPair
	// Bracket is local to the sweep starting at SliceStart for SliceDuration.
	Bracket                   decad.SweepInterval
	SliceStart, SliceDuration units.Value
	// Time is measured from the start of the complete step.
	Time                                       units.Value
	Manifold                                   decad.ContactManifold
	NormalImpulse                              units.Value
	TangentImpulse                             QuantityVec
	PointImpulses                              []ContactPointImpulse
	Solver                                     *ContactSolverReport
	PreVelocity                                QuantityVec
	PostVelocity                               QuantityVec
	PositionChange                             r3.Vec
	PreVelocityA, PreVelocityB                 QuantityVec
	PostVelocityA, PostVelocityB               QuantityVec
	PreAngularVelocityA, PreAngularVelocityB   QuantityVec
	PostAngularVelocityA, PostAngularVelocityB QuantityVec
	PoseA, PoseB                               r3.Transform
	PositionChangeA, PositionChangeB           r3.Vec
	// Island is the index into StepReport.Islands of the island that
	// published this event; a graze or a transition enters no solve and
	// carries -1.
	Island int
}

// ContactPointImpulse follows the matching point in ContactEvent.Manifold.
type ContactPointImpulse struct {
	Normal  units.Value
	Tangent QuantityVec
}

// ContactSolverReport records bounded residuals for a joint contact solve.
type ContactSolverReport struct {
	NormalResidual      units.Value
	TangentResidual     units.Value
	ConeResidual        units.Value
	PenetrationResidual units.Value
	AngularUpper        units.Value
	Iterations          int
	// The island solver also publishes the largest attained value of each certificate gate
	// (docs/multibody-dynamics-design.md §6.3): the linear and angular law
	// residuals, the kinetic-energy change's upper end, and the island's
	// linear and angular momentum residuals about the world origin. Its
	// AngularUpper bounds the largest published post-solve angular speed.
	LinearResidual          units.Value
	AngularResidual         units.Value
	EnergyResidual          units.Value
	MomentumResidual        units.Value
	AngularMomentumResidual units.Value
}

func zeroImpulseVec() QuantityVec {
	zero := units.KilogramMillimetersPerSecond(0)
	return QuantityVec{X: zero, Y: zero, Z: zero}
}

type StepReport struct {
	Status       StepStatus
	Next         *State
	Events       []ContactEvent
	Excluded     []BodyPair
	Trace        Trace
	Diagnostics  []StepDiagnostic
	Conservation *StepConservation
	// Islands lists the step's simultaneous solves, in solve order.
	Islands []IslandReport
}

func translatePose(pose r3.Transform, delta r3.Vec) (r3.Transform, error) {
	translation, err := r3.Translation(delta)
	if err != nil {
		return r3.Transform{}, err
	}
	return pose.Then(translation)
}

func velocityComponent(v QuantityVec, axis int) units.Value {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

func setVelocityComponent(v *QuantityVec, axis int, value units.Value) {
	switch axis {
	case 0:
		v.X = value
	case 1:
		v.Y = value
	default:
		v.Z = value
	}
}

// Step drifts every body from event to event on
// paths whose candidate pairs the broad phase selects and SweepPair certifies;
// at each event time it solves the touching and impacting pairs as certified
// Coulomb islands and continues (docs/multibody-dynamics-design.md §5,
// §6). Its Undecided report carries typed diagnostics and the certified prefix
// in its Trace.
func (w *World) Step(ctx context.Context, from State, input StepInput, dt units.Value) (*StepReport, error) {
	if w == nil || ctx == nil || from.world != w || !validQuantity(dt, units.Time, true) {
		return nil, fmt.Errorf("%w: invalid context, world, state, or duration", ErrInvalidInput)
	}
	return w.stepScheduled(ctx, from, input, dt)
}

func intervalDeviation(value, low, high *big.Rat) *big.Rat {
	a := absRat(new(big.Rat).Sub(value, low))
	b := absRat(new(big.Rat).Sub(value, high))
	if a.Cmp(b) < 0 {
		return b
	}
	return a
}

// ratFloat is the exact rational value of a float64.
func ratFloat(value float64) *big.Rat { return new(big.Rat).SetFloat64(value) }

func exactBase(value units.Value) *big.Rat {
	mag := new(big.Rat).SetFloat64(value.Mag())
	factor := new(big.Rat).SetFloat64(value.Unit().Factor())
	if mag == nil || factor == nil {
		return nil
	}
	return new(big.Rat).Mul(mag, factor)
}

func absRat(value *big.Rat) *big.Rat {
	if value.Sign() < 0 {
		value.Neg(value)
	}
	return value
}

// Row dominance proves a positive lower eigenvalue for every tensor inside
// the six published inertia intervals. It may reject a valid wider tensor.
func certifiedInertiaLower(m decad.MassProperties) *big.Rat {
	diagonal := []decad.Measurement{m.Inertia.XX, m.Inertia.YY, m.Inertia.ZZ}
	off := []decad.Measurement{m.Inertia.XY, m.Inertia.XZ, m.Inertia.YZ}
	var offUpper [3]*big.Rat
	for i, component := range off {
		value, bound := exactBase(component.Value), exactBase(component.Bound)
		if value == nil || bound == nil || bound.Sign() < 0 {
			return nil
		}
		offUpper[i] = new(big.Rat).Add(absRat(value), bound)
	}
	var lower *big.Rat
	for i, component := range diagonal {
		value, bound := exactBase(component.Value), exactBase(component.Bound)
		if value == nil || bound == nil || bound.Sign() < 0 {
			return nil
		}
		row := new(big.Rat).Sub(value, bound)
		switch i {
		case 0:
			row.Sub(row, offUpper[0]).Sub(row, offUpper[1])
		case 1:
			row.Sub(row, offUpper[0]).Sub(row, offUpper[2])
		case 2:
			row.Sub(row, offUpper[1]).Sub(row, offUpper[2])
		}
		if lower == nil || row.Cmp(lower) < 0 {
			lower = row
		}
	}
	return lower
}

// The reported elapsed readings enclose the exact dyadic sweep instants.
// Outward arithmetic makes their interval a conservative travel allowance.
func boundBracketTravel(bracket decad.SweepInterval, velocity float64) (float64, bool) {
	from, to := bracket.From.Elapsed, bracket.To.Elapsed
	if from.Value.Kind() != units.Time || from.Bound.Kind() != units.Time ||
		to.Value.Kind() != units.Time || to.Bound.Kind() != units.Time ||
		!finite(velocity, from.Value.Base(), from.Bound.Base(), to.Value.Base(), to.Bound.Base()) ||
		from.Bound.Base() < 0 || to.Bound.Base() < 0 {
		return 0, false
	}
	upper := math.Nextafter(to.Value.Base()+to.Bound.Base(), math.Inf(1))
	lower := math.Nextafter(from.Value.Base()-from.Bound.Base(), math.Inf(-1))
	width := math.Nextafter(upper-lower, math.Inf(1))
	travel := math.Nextafter(math.Abs(velocity)*width, math.Inf(1))
	return travel, finite(upper, lower, width, travel) && width >= 0
}

func outwardSum(values ...float64) float64 {
	sum := 0.0
	for _, value := range values {
		sum += value
		if sum != 0 {
			sum = math.Nextafter(sum, math.Inf(1))
		}
	}
	return sum
}

// sweepRequest is the request of every SweepPair the step runs. RestSpeed is
// VelocityResidual (docs/multibody-dynamics-design.md §10.8): §6.3 leaves a
// resting point's normal speed within that residual of zero, so a lifted
// vertex the solve just rested is held on both sides of the support plane on
// the next slice, while one that arrives faster still ends its band track a
// grid step before the plane.
func (w *World) sweepRequest(duration units.Value, policy decad.SweepStartPolicy) decad.SweepRequest {
	resolution := w.step.TimeResolution
	if duration.Base() < resolution.Base() {
		resolution = duration
	}
	return decad.SweepRequest{ContactRequest: w.step.Contact, TimeResolution: resolution,
		MaxPoseEvaluations: w.step.MaxPoseEvaluations, StartPolicy: policy, RestSpeed: w.step.VelocityResidual}
}

func cloneManifold(m decad.ContactManifold) decad.ContactManifold {
	m.Points = append([]decad.ContactPoint(nil), m.Points...)
	return m
}
