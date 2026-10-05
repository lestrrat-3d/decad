// Package dynamics advances rigid bodies using decad's certified geometry queries.
// The current implementation also admits one active pair in a three-body world.
package dynamics

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	ErrInvalidInput          = errors.New("dynamics: invalid input")
	ErrUnsupported           = errors.New("dynamics: unsupported by the current solver")
	ErrInvalidMassProperties = errors.New("dynamics: invalid mass properties")
)

type BodyRole int

const (
	Fixed BodyRole = iota + 1
	Kinematic
	Dynamic
)

type QuantityVec = decad.QuantityVec

type Material struct {
	Restitution units.Value
	Friction    units.Value
}

type RigidBody struct {
	Body     *decad.Body
	Role     BodyRole
	Material Material
	Density  *units.Value
	Supplied *decad.MassProperties
}

type BodyPair struct{ A, B *decad.Body }

type PairMaterial struct {
	Pair                  BodyPair
	Restitution, Friction units.Value
}

type StepConfig struct {
	Contact                 decad.ContactRequest
	TimeResolution          units.Value
	ContactSlop             units.Value
	VelocityResidual        units.Value
	AngularVelocityResidual units.Value
	ImpulseResidual         units.Value
	PenetrationResidual     units.Value
	ImpactSpeed             units.Value
	MaxPoseEvaluations      uint64
	MaxIterations           int
	MaxEvents               int
}

type WorldConfig struct {
	Bodies    []RigidBody
	Excluded  []BodyPair
	Overrides []PairMaterial
	Step      StepConfig
}

type worldBody struct {
	definition RigidBody
	mass       decad.MassProperties
}

// World holds immutable body definitions and the mass readings admitted at construction.
type World struct {
	doc         *decad.Document
	parts       [2]worldBody
	three       *threeBodyWorld
	excluded    []BodyPair
	step        StepConfig
	restitution units.Value
	friction    frictionCoefficient
}

// NewWorld admits a dynamic pair, a dynamic body with a fixed or kinematic
// body, or three bodies with one to three dynamic bodies and all others fixed.
func NewWorld(ctx context.Context, doc *decad.Document, cfg WorldConfig) (*World, error) {
	if doc == nil || ctx == nil {
		return nil, fmt.Errorf("%w: nil document or context", ErrInvalidInput)
	}
	if err := validateStepConfig(cfg.Step); err != nil {
		return nil, err
	}
	if len(cfg.Bodies) == 3 {
		return newThreeBodyWorld(ctx, doc, cfg)
	}
	if len(cfg.Bodies) != 2 {
		return nil, fmt.Errorf("%w: this stage admits one pair", ErrUnsupported)
	}
	live := doc.Bodies()
	seen := map[*decad.Body]struct{}{}
	w := &World{doc: doc, step: cfg.Step}
	fixed, kinematic, dynamic := 0, 0, 0
	for i, entry := range cfg.Bodies {
		if entry.Body == nil {
			return nil, fmt.Errorf("%w: nil body", ErrInvalidInput)
		}
		if entry.Body.Document() != doc || !containsBody(live, entry.Body) {
			return nil, fmt.Errorf("%w: body is foreign or retired", ErrInvalidInput)
		}
		if _, ok := seen[entry.Body]; ok {
			return nil, fmt.Errorf("%w: duplicate body", ErrInvalidInput)
		}
		seen[entry.Body] = struct{}{}
		if entry.Body.Kind() != decad.BodySolid || !entry.Body.IsSolid() {
			return nil, fmt.Errorf("%w: body is not a sound solid", ErrInvalidInput)
		}
		if err := validateMaterial(entry.Material); err != nil {
			return nil, err
		}
		if entry.Density != nil {
			density := *entry.Density
			entry.Density = &density
		}
		if entry.Supplied != nil {
			supplied := *entry.Supplied
			entry.Supplied = &supplied
		}
		w.parts[i].definition = entry
		switch entry.Role {
		case Fixed:
			fixed++
			if entry.Density != nil || entry.Supplied != nil {
				return nil, fmt.Errorf("%w: fixed body has mass input", ErrInvalidInput)
			}
		case Dynamic:
			dynamic++
			if (entry.Density == nil) == (entry.Supplied == nil) {
				return nil, fmt.Errorf("%w: dynamic body needs exactly one mass source", ErrInvalidInput)
			}
			var mass decad.MassProperties
			if entry.Supplied != nil {
				mass = *entry.Supplied
			} else {
				var err error
				mass, err = entry.Body.MassProperties(ctx, *entry.Density)
				if err != nil {
					return nil, err
				}
			}
			if err := validateMass(mass); err != nil {
				return nil, err
			}
			w.parts[i].mass = mass
		case Kinematic:
			kinematic++
			if entry.Density != nil || entry.Supplied != nil {
				return nil, fmt.Errorf("%w: kinematic body has mass input", ErrInvalidInput)
			}
		default:
			return nil, fmt.Errorf("%w: unknown body role", ErrInvalidInput)
		}
	}
	if dynamic == 0 || fixed+kinematic+dynamic != 2 {
		return nil, fmt.Errorf("%w: one or two dynamic bodies required", ErrUnsupported)
	}
	for _, pair := range cfg.Excluded {
		first, second := w.parts[0].definition.Body, w.parts[1].definition.Body
		if (pair.A != first || pair.B != second) && (pair.A != second || pair.B != first) {
			return nil, fmt.Errorf("%w: exclusion names an unknown pair", ErrInvalidInput)
		}
		if len(w.excluded) != 0 {
			return nil, fmt.Errorf("%w: duplicate pair exclusion", ErrInvalidInput)
		}
		w.excluded = []BodyPair{{A: first, B: second}}
	}
	if len(w.excluded) != 0 && len(cfg.Overrides) != 0 {
		return nil, fmt.Errorf("%w: excluded pair has a material override", ErrInvalidInput)
	}
	if len(w.excluded) != 0 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return w, nil
	}
	restitutionA := w.parts[0].definition.Material.Restitution
	restitutionB := w.parts[1].definition.Material.Restitution
	w.restitution = restitutionA
	if exactBase(restitutionB).Cmp(exactBase(restitutionA)) < 0 {
		w.restitution = restitutionB
	}
	frictionA := w.parts[0].definition.Material.Friction
	frictionB := w.parts[1].definition.Material.Friction
	w.friction = exactFrictionCoefficient(units.Scalar(0))
	if len(cfg.Overrides) > 0 {
		if err := w.setPairOverride(cfg.Overrides); err != nil {
			return nil, err
		}
	} else {
		var ok bool
		w.friction, ok = mixBodyFriction(frictionA, frictionB)
		if !ok {
			return nil, fmt.Errorf("%w: effective friction is outside the finite nonzero scalar range", ErrUnsupported)
		}
	}
	if w.friction.lower.Sign() > 0 &&
		(w.parts[0].definition.Role != Fixed || w.parts[1].definition.Role != Dynamic) &&
		(w.parts[0].definition.Role != Dynamic || w.parts[1].definition.Role != Fixed) &&
		(w.parts[0].definition.Role != Dynamic || w.parts[1].definition.Role != Dynamic) {
		return nil, fmt.Errorf("%w: positive friction requires a fixed/dynamic or dynamic/dynamic pair", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return w, nil
}

// Excluded returns the excluded pair in world order, if one was configured.
func (w *World) Excluded() []BodyPair {
	if w == nil {
		return nil
	}
	if w.three != nil {
		return append([]BodyPair(nil), w.three.excluded...)
	}
	return append([]BodyPair(nil), w.excluded...)
}

func (w *World) setPairOverride(overrides []PairMaterial) error {
	seen := false
	for _, override := range overrides {
		pair := override.Pair
		first, second := w.parts[0].definition.Body, w.parts[1].definition.Body
		if (pair.A != first || pair.B != second) && (pair.A != second || pair.B != first) {
			return fmt.Errorf("%w: override names an unknown pair", ErrInvalidInput)
		}
		if seen {
			return fmt.Errorf("%w: duplicate pair override", ErrInvalidInput)
		}
		if err := validateMaterial(Material{Restitution: override.Restitution, Friction: override.Friction}); err != nil {
			return err
		}
		w.restitution = override.Restitution
		w.friction = exactFrictionCoefficient(override.Friction)
		seen = true
	}
	return nil
}

func containsBody(bodies []*decad.Body, body *decad.Body) bool {
	return slices.Contains(bodies, body)
}

func validQuantity(v units.Value, kind units.Kind, strict bool) bool {
	x := v.Base()
	if v.Kind() != kind || math.IsNaN(x) || math.IsInf(x, 0) {
		return false
	}
	return x > 0 || (!strict && x == 0)
}

func validateMaterial(m Material) error {
	if m.Restitution.Kind() != units.Dimensionless || m.Friction.Kind() != units.Dimensionless ||
		!finite(m.Restitution.Mag(), m.Restitution.Unit().Factor(), m.Friction.Mag(),
			m.Friction.Unit().Factor(), m.Restitution.Base(), m.Friction.Base()) {
		return fmt.Errorf("%w: restitution or friction is not a finite scalar", ErrInvalidInput)
	}
	restitution, friction := exactBase(m.Restitution), exactBase(m.Friction)
	if restitution == nil || friction == nil || restitution.Sign() < 0 ||
		restitution.Cmp(big.NewRat(1, 1)) > 0 || friction.Sign() < 0 {
		return fmt.Errorf("%w: restitution must be in [0,1] and friction nonnegative", ErrInvalidInput)
	}
	return nil
}

func validateStepConfig(cfg StepConfig) error {
	checks := []struct {
		name   string
		value  units.Value
		kind   units.Kind
		strict bool
	}{
		{"point resolution", cfg.Contact.PointResolution, units.Length, true},
		{"normal resolution", cfg.Contact.NormalResolution, units.Angle, true},
		{"time resolution", cfg.TimeResolution, units.Time, true},
		{"contact slop", cfg.ContactSlop, units.Length, false},
		{"velocity residual", cfg.VelocityResidual, units.Velocity, true},
		{"angular velocity residual", cfg.AngularVelocityResidual, units.AngularVelocity, true},
		{"impulse residual", cfg.ImpulseResidual, units.Impulse, true},
		{"penetration residual", cfg.PenetrationResidual, units.Length, true},
		{"impact speed", cfg.ImpactSpeed, units.Velocity, false},
	}
	for _, check := range checks {
		if !validQuantity(check.value, check.kind, check.strict) {
			return fmt.Errorf("%w: invalid %s", ErrInvalidInput, check.name)
		}
	}
	if cfg.MaxPoseEvaluations < 2 || cfg.MaxIterations <= 0 || cfg.MaxEvents <= 0 {
		return fmt.Errorf("%w: work limits must be positive", ErrInvalidInput)
	}
	return nil
}

func validateMass(m decad.MassProperties) error {
	if m.Mass.Value.Kind() != units.Mass || m.Mass.Bound.Kind() != units.Mass ||
		m.Center.Bound.Kind() != units.Length {
		return fmt.Errorf("%w: wrong mass or center kind", ErrInvalidMassProperties)
	}
	if !finite(m.Mass.Value.Base(), m.Mass.Bound.Base(), m.Center.Value.X, m.Center.Value.Y,
		m.Center.Value.Z, m.Center.Bound.Base()) || m.Mass.Bound.Base() < 0 ||
		m.Mass.Value.Base()-m.Mass.Bound.Base() <= 0 || m.Center.Bound.Base() < 0 {
		return fmt.Errorf("%w: nonpositive or unbounded mass", ErrInvalidMassProperties)
	}
	if !validMassExactness(m.Mass.Exactness, m.Mass.Bound) ||
		!validMassExactness(m.Center.Exactness, m.Center.Bound) {
		return fmt.Errorf("%w: inconsistent mass or center exactness", ErrInvalidMassProperties)
	}
	values := []decad.Measurement{m.Inertia.XX, m.Inertia.YY, m.Inertia.ZZ,
		m.Inertia.XY, m.Inertia.XZ, m.Inertia.YZ}
	for _, value := range values {
		if value.Value.Kind() != units.MomentOfInertia || value.Bound.Kind() != units.MomentOfInertia ||
			!finite(value.Value.Base(), value.Bound.Base()) || value.Bound.Base() < 0 {
			return fmt.Errorf("%w: invalid inertia component", ErrInvalidMassProperties)
		}
		if !validMassExactness(value.Exactness, value.Bound) {
			return fmt.Errorf("%w: inconsistent inertia exactness", ErrInvalidMassProperties)
		}
	}
	// A strict row-dominance bound proves every tensor inside the component intervals positive.
	lower := certifiedInertiaLower(m)
	if lower == nil || lower.Sign() <= 0 {
		return fmt.Errorf("%w: tensor positivity is not proved", ErrInvalidMassProperties)
	}
	massLower := new(big.Rat).Sub(exactBase(m.Mass.Value), exactBase(m.Mass.Bound))
	if !finiteInverse(massLower) || !finiteInverse(lower) {
		return fmt.Errorf("%w: mass or inertia inverse is not finite", ErrInvalidMassProperties)
	}
	return nil
}

func validMassExactness(exactness decad.Exactness, bound units.Value) bool {
	switch exactness {
	case decad.Exact:
		return bound.Mag() == 0
	case decad.Approximate:
		return true
	default:
		return false
	}
}

func finiteInverse(lower *big.Rat) bool {
	if lower == nil || lower.Sign() <= 0 {
		return false
	}
	inverse, _ := new(big.Rat).Inv(lower).Float64()
	return finite(inverse)
}

func finite(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

type BodyState struct {
	Body            *decad.Body
	Pose            r3.Transform
	LinearVelocity  QuantityVec
	AngularVelocity QuantityVec
}

// State is a value snapshot bound to its source World.
type State struct {
	world    *World
	entries  [2]BodyState
	third    BodyState
	hasThird bool
}

func (s State) Entries() []BodyState {
	if s.hasThird {
		return []BodyState{s.entries[0], s.entries[1], s.third}
	}
	return []BodyState{s.entries[0], s.entries[1]}
}

func (s State) Body(body *decad.Body) (BodyState, bool) {
	for _, entry := range s.entries {
		if entry.Body == body {
			return entry, true
		}
	}
	if s.hasThird && s.third.Body == body {
		return s.third, true
	}
	return BodyState{}, false
}

// NewState records one posed entry for each body in world order.
func (w *World) NewState(entries []BodyState) (State, error) {
	if w != nil && w.three != nil {
		return w.newThreeBodyState(entries)
	}
	if w == nil || len(entries) != len(w.parts) {
		return State{}, fmt.Errorf("%w: state requires exactly two bodies", ErrInvalidInput)
	}
	out := State{world: w}
	seen := map[*decad.Body]struct{}{}
	for _, entry := range entries {
		if entry.Body == nil || !entry.Pose.IsValid() || entry.Pose.IsReflection() {
			return State{}, fmt.Errorf("%w: invalid body or pose", ErrInvalidInput)
		}
		if _, ok := seen[entry.Body]; ok {
			return State{}, fmt.Errorf("%w: duplicate body", ErrInvalidInput)
		}
		seen[entry.Body] = struct{}{}
		idx := -1
		for i := range w.parts {
			if w.parts[i].definition.Body == entry.Body {
				idx = i
				break
			}
		}
		if idx < 0 {
			return State{}, fmt.Errorf("%w: body not in world", ErrInvalidInput)
		}
		if err := validateQuantityVec(entry.LinearVelocity, units.Velocity); err != nil {
			return State{}, err
		}
		if err := validateQuantityVec(entry.AngularVelocity, units.AngularVelocity); err != nil {
			return State{}, err
		}
		if w.parts[idx].definition.Role == Kinematic && (entry.AngularVelocity.X.Mag() != 0 ||
			entry.AngularVelocity.Y.Mag() != 0 || entry.AngularVelocity.Z.Mag() != 0) {
			return State{}, fmt.Errorf("%w: kinematic body has stored angular velocity", ErrInvalidInput)
		}
		if w.parts[idx].definition.Role == Fixed && (entry.AngularVelocity.X.Mag() != 0 ||
			entry.AngularVelocity.Y.Mag() != 0 || entry.AngularVelocity.Z.Mag() != 0) {
			return State{}, fmt.Errorf("%w: fixed body has angular velocity", ErrInvalidInput)
		}
		if w.parts[idx].definition.Role == Fixed && (entry.LinearVelocity.X.Base() != 0 ||
			entry.LinearVelocity.Y.Base() != 0 || entry.LinearVelocity.Z.Base() != 0) {
			return State{}, fmt.Errorf("%w: fixed body has velocity", ErrInvalidInput)
		}
		if w.parts[idx].definition.Role == Kinematic && (entry.LinearVelocity.X.Mag() != 0 ||
			entry.LinearVelocity.Y.Mag() != 0 || entry.LinearVelocity.Z.Mag() != 0) {
			return State{}, fmt.Errorf("%w: kinematic body has stored velocity", ErrInvalidInput)
		}
		out.entries[idx] = entry
	}
	return out, nil
}

func validateQuantityVec(v QuantityVec, kind units.Kind) error {
	for _, c := range []units.Value{v.X, v.Y, v.Z} {
		if c.Kind() != kind || !finite(c.Base()) {
			return fmt.Errorf("%w: vector component has wrong kind or is non-finite", ErrInvalidInput)
		}
	}
	return nil
}
