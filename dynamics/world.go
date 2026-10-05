// Package dynamics advances rigid bodies using decad's certified geometry queries.
// A world holds any number of bodies and their canonical pair table. Two- and
// three-body worlds step through their closed-form responses; a world of four
// or more bodies steps through the certified broad phase and publishes an
// event-free drift, stopping as Undecided at its first contact event.
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
	mass       decad.MassProperties // valid only for Dynamic
}

// World holds immutable body definitions, the mass readings admitted at
// construction, and the canonical pair table with each pair's material.
type World struct {
	doc    *decad.Document
	bodies []worldBody // insertion order
	pairs  []worldPair // canonical order: (0,1), (0,2), …, (1,2), …
	index  map[*decad.Body]int
	step   StepConfig
	three  *threeBodyWorld
}

// NewWorld admits two or more bodies and builds the canonical pair table.
// A two-body world needs a dynamic body; a three-body world needs a dynamic
// body and no kinematic body. A larger world carries no role limit.
func NewWorld(ctx context.Context, doc *decad.Document, cfg WorldConfig) (*World, error) {
	if doc == nil || ctx == nil {
		return nil, fmt.Errorf("%w: nil document or context", ErrInvalidInput)
	}
	if err := validateStepConfig(cfg.Step); err != nil {
		return nil, err
	}
	if len(cfg.Bodies) < 2 {
		return nil, fmt.Errorf("%w: a world needs at least two bodies", ErrUnsupported)
	}
	w := &World{doc: doc, step: cfg.Step}
	if err := w.admitBodies(ctx, cfg.Bodies); err != nil {
		return nil, err
	}
	if err := w.admitShape(); err != nil {
		return nil, err
	}
	pairs, err := buildPairTable(w.bodies, w.index, cfg.Excluded, cfg.Overrides)
	if err != nil {
		return nil, err
	}
	w.pairs = pairs
	if len(w.bodies) == 3 {
		w.three = newThreeBodyWorld(w)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return w, nil
}

// admitBodies validates and copies each body definition and reads the mass
// of every dynamic body once.
func (w *World) admitBodies(ctx context.Context, entries []RigidBody) error {
	live := w.doc.Bodies()
	w.bodies = make([]worldBody, len(entries))
	w.index = make(map[*decad.Body]int, len(entries))
	for i, entry := range entries {
		if entry.Body == nil {
			return fmt.Errorf("%w: nil body", ErrInvalidInput)
		}
		if entry.Body.Document() != w.doc || !containsBody(live, entry.Body) {
			return fmt.Errorf("%w: body is foreign or retired", ErrInvalidInput)
		}
		if _, ok := w.index[entry.Body]; ok {
			return fmt.Errorf("%w: duplicate body", ErrInvalidInput)
		}
		w.index[entry.Body] = i
		if entry.Body.Kind() != decad.BodySolid || !entry.Body.IsSolid() {
			return fmt.Errorf("%w: body is not a sound solid", ErrInvalidInput)
		}
		if err := validateMaterial(entry.Material); err != nil {
			return err
		}
		if entry.Density != nil {
			density := *entry.Density
			entry.Density = &density
		}
		if entry.Supplied != nil {
			supplied := *entry.Supplied
			entry.Supplied = &supplied
		}
		w.bodies[i].definition = entry
		switch entry.Role {
		case Fixed, Kinematic:
			if entry.Density != nil || entry.Supplied != nil {
				return fmt.Errorf("%w: fixed or kinematic body has mass input", ErrInvalidInput)
			}
		case Dynamic:
			if (entry.Density == nil) == (entry.Supplied == nil) {
				return fmt.Errorf("%w: dynamic body needs exactly one mass source", ErrInvalidInput)
			}
			var mass decad.MassProperties
			if entry.Supplied != nil {
				mass = *entry.Supplied
			} else {
				var err error
				mass, err = entry.Body.MassProperties(ctx, *entry.Density)
				if err != nil {
					return err
				}
			}
			if err := validateMass(mass); err != nil {
				return err
			}
			w.bodies[i].mass = mass
		default:
			return fmt.Errorf("%w: unknown body role", ErrInvalidInput)
		}
	}
	return nil
}

// admitShape refuses the two- and three-body role mixes the shipped steps
// cannot take. Larger worlds carry no role limit here.
func (w *World) admitShape() error {
	kinematic, dynamic := 0, 0
	for _, body := range w.bodies {
		switch body.definition.Role {
		case Kinematic:
			kinematic++
		case Dynamic:
			dynamic++
		}
	}
	switch len(w.bodies) {
	case 2:
		if dynamic == 0 {
			return fmt.Errorf("%w: one or two dynamic bodies required", ErrUnsupported)
		}
	case 3:
		if kinematic != 0 {
			return fmt.Errorf("%w: three-body kinematic response", ErrUnsupported)
		}
		if dynamic == 0 {
			return fmt.Errorf("%w: three-body world needs a dynamic body", ErrUnsupported)
		}
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

// State is a value snapshot bound to its source World. Its entries slice is
// never shared between two States that may diverge: code that derives a new
// State by changing an entry starts from clone.
type State struct {
	world   *World
	entries []BodyState // world order
}

func (s State) Entries() []BodyState {
	return slices.Clone(s.entries)
}

func (s State) Body(body *decad.Body) (BodyState, bool) {
	for _, entry := range s.entries {
		if entry.Body == body {
			return entry, true
		}
	}
	return BodyState{}, false
}

// clone returns a State with its own copy of the entries.
func (s State) clone() State {
	s.entries = slices.Clone(s.entries)
	return s
}

// NewState records one posed entry for each body in world order.
func (w *World) NewState(entries []BodyState) (State, error) {
	if w == nil || len(entries) != len(w.bodies) {
		return State{}, fmt.Errorf("%w: state requires exactly one entry per world body", ErrInvalidInput)
	}
	out := State{world: w, entries: make([]BodyState, len(w.bodies))}
	seen := make(map[*decad.Body]struct{}, len(entries))
	for _, entry := range entries {
		if entry.Body == nil || !entry.Pose.IsValid() || entry.Pose.IsReflection() {
			return State{}, fmt.Errorf("%w: invalid body or pose", ErrInvalidInput)
		}
		if _, ok := seen[entry.Body]; ok {
			return State{}, fmt.Errorf("%w: duplicate body", ErrInvalidInput)
		}
		seen[entry.Body] = struct{}{}
		idx, ok := w.index[entry.Body]
		if !ok {
			return State{}, fmt.Errorf("%w: body not in world", ErrInvalidInput)
		}
		if err := validateQuantityVec(entry.LinearVelocity, units.Velocity); err != nil {
			return State{}, err
		}
		if err := validateQuantityVec(entry.AngularVelocity, units.AngularVelocity); err != nil {
			return State{}, err
		}
		role := w.bodies[idx].definition.Role
		if role == Kinematic && (entry.AngularVelocity.X.Mag() != 0 ||
			entry.AngularVelocity.Y.Mag() != 0 || entry.AngularVelocity.Z.Mag() != 0) {
			return State{}, fmt.Errorf("%w: kinematic body has stored angular velocity", ErrInvalidInput)
		}
		if role == Fixed && (entry.AngularVelocity.X.Mag() != 0 ||
			entry.AngularVelocity.Y.Mag() != 0 || entry.AngularVelocity.Z.Mag() != 0) {
			return State{}, fmt.Errorf("%w: fixed body has angular velocity", ErrInvalidInput)
		}
		if role == Fixed && (entry.LinearVelocity.X.Base() != 0 ||
			entry.LinearVelocity.Y.Base() != 0 || entry.LinearVelocity.Z.Base() != 0) {
			return State{}, fmt.Errorf("%w: fixed body has velocity", ErrInvalidInput)
		}
		if role == Kinematic && (entry.LinearVelocity.X.Mag() != 0 ||
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
