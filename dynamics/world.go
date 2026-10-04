// Package dynamics advances rigid bodies using decad's certified geometry queries.
// The current implementation admits one frictionless translating body and one fixed body.
package dynamics

import (
	"context"
	"errors"
	"fmt"
	"math"
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
	doc   *decad.Document
	parts [2]worldBody
	step  StepConfig
}

// NewWorld admits the first supported pair: one fixed body and one density-derived dynamic body.
func NewWorld(ctx context.Context, doc *decad.Document, cfg WorldConfig) (*World, error) {
	if doc == nil || ctx == nil {
		return nil, fmt.Errorf("%w: nil document or context", ErrInvalidInput)
	}
	if err := validateStepConfig(cfg.Step); err != nil {
		return nil, err
	}
	if len(cfg.Bodies) != 2 || len(cfg.Excluded) != 0 || len(cfg.Overrides) != 0 {
		return nil, fmt.Errorf("%w: this stage admits exactly one fixed/dynamic pair without exclusions", ErrUnsupported)
	}
	live := doc.Bodies()
	seen := map[*decad.Body]struct{}{}
	w := &World{doc: doc, step: cfg.Step}
	fixed, dynamic := 0, 0
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
		w.parts[i].definition = entry
		switch entry.Role {
		case Fixed:
			fixed++
			if entry.Density != nil || entry.Supplied != nil {
				return nil, fmt.Errorf("%w: fixed body has mass input", ErrInvalidInput)
			}
		case Dynamic:
			dynamic++
			if entry.Density == nil || entry.Supplied != nil {
				return nil, fmt.Errorf("%w: this stage requires density-derived mass", ErrUnsupported)
			}
			mass, err := entry.Body.MassProperties(ctx, *entry.Density)
			if err != nil {
				return nil, err
			}
			if err := validateMass(mass); err != nil {
				return nil, err
			}
			w.parts[i].mass = mass
		case Kinematic:
			return nil, fmt.Errorf("%w: kinematic bodies are not implemented", ErrUnsupported)
		default:
			return nil, fmt.Errorf("%w: unknown body role", ErrInvalidInput)
		}
	}
	if fixed != 1 || dynamic != 1 {
		return nil, fmt.Errorf("%w: one fixed and one dynamic body required", ErrUnsupported)
	}
	return w, nil
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
	if !validQuantity(m.Restitution, units.Dimensionless, false) || m.Restitution.Base() > 1 ||
		!validQuantity(m.Friction, units.Dimensionless, false) {
		return fmt.Errorf("%w: restitution must be in [0,1] and friction nonnegative", ErrInvalidInput)
	}
	if m.Friction.Base() != 0 {
		return fmt.Errorf("%w: friction is not implemented", ErrUnsupported)
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
	values := []decad.Measurement{m.Inertia.XX, m.Inertia.YY, m.Inertia.ZZ,
		m.Inertia.XY, m.Inertia.XZ, m.Inertia.YZ}
	for _, value := range values {
		if value.Value.Kind() != units.MomentOfInertia || value.Bound.Kind() != units.MomentOfInertia ||
			!finite(value.Value.Base(), value.Bound.Base()) || value.Bound.Base() < 0 {
			return fmt.Errorf("%w: invalid inertia component", ErrInvalidMassProperties)
		}
	}
	// A strict row-dominance bound proves every tensor inside the component intervals positive.
	if lower := certifiedInertiaLower(m); lower == nil || lower.Sign() <= 0 {
		return fmt.Errorf("%w: tensor positivity is not proved", ErrInvalidMassProperties)
	}
	return nil
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
	world   *World
	entries [2]BodyState
}

func (s State) Entries() []BodyState {
	return []BodyState{s.entries[0], s.entries[1]}
}

func (s State) Body(body *decad.Body) (BodyState, bool) {
	for _, entry := range s.entries {
		if entry.Body == body {
			return entry, true
		}
	}
	return BodyState{}, false
}

// NewState records one posed entry for each body in world order.
func (w *World) NewState(entries []BodyState) (State, error) {
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
		if entry.AngularVelocity.X.Base() != 0 || entry.AngularVelocity.Y.Base() != 0 ||
			entry.AngularVelocity.Z.Base() != 0 {
			return State{}, fmt.Errorf("%w: angular motion is not implemented", ErrUnsupported)
		}
		if w.parts[idx].definition.Role == Fixed && (entry.LinearVelocity.X.Base() != 0 ||
			entry.LinearVelocity.Y.Base() != 0 || entry.LinearVelocity.Z.Base() != 0) {
			return State{}, fmt.Errorf("%w: fixed body has velocity", ErrInvalidInput)
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
