package dynamics

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// worldPair is one unordered body pair in canonical world order (a < b).
//
// The table holds every pair of the world, excluded ones included. A pair
// whose bodies are both Fixed is never resolved by an impulse; its material
// is recorded when it mixes, and the representable-range refusal applies only
// to pairs with a moving body (docs/multibody-dynamics-design.md §3.1).
type worldPair struct {
	a, b        int
	excluded    bool
	moving      bool        // at least one body is not Fixed
	restitution units.Value // effective pair value, rigid-dynamics "World and State"
	friction    frictionCoefficient
}

// pairCount is the number of unordered pairs among n bodies.
func pairCount(n int) int {
	return n * (n - 1) / 2
}

// canonicalPairIndex returns the canonical table index of the unordered pair (a, b),
// with a != b. The table enumerates (0,1), (0,2), …, (0,n-1), (1,2), ….
func canonicalPairIndex(n, a, b int) int {
	if a > b {
		a, b = b, a
	}
	return a*n - a*(a+1)/2 + (b - a - 1)
}

// lookupPair resolves a caller-named pair against the world index. Either
// body order names the same pair; nil, foreign and repeated bodies do not.
func lookupPair(index map[*decad.Body]int, pair BodyPair) (int, bool) {
	a, okA := index[pair.A]
	b, okB := index[pair.B]
	if !okA || !okB || a == b {
		return 0, false
	}
	return canonicalPairIndex(len(index), a, b), true
}

// buildPairTable enumerates every body pair in canonical order, applies the
// exclusions and overrides, and mixes the effective material of each
// non-excluded pair.
func buildPairTable(bodies []worldBody, index map[*decad.Body]int, excluded []BodyPair,
	overrides []PairMaterial) ([]worldPair, error) {
	n := len(bodies)
	pairs := make([]worldPair, 0, pairCount(n))
	for a := range n {
		for b := a + 1; b < n; b++ {
			pairs = append(pairs, worldPair{a: a, b: b,
				moving: bodies[a].definition.Role != Fixed || bodies[b].definition.Role != Fixed})
		}
	}
	for _, pair := range excluded {
		key, ok := lookupPair(index, pair)
		if !ok {
			return nil, fmt.Errorf("%w: exclusion names an unknown pair", ErrInvalidInput)
		}
		if pairs[key].excluded {
			return nil, fmt.Errorf("%w: duplicate pair exclusion", ErrInvalidInput)
		}
		pairs[key].excluded = true
	}
	overridden := make(map[int]PairMaterial, len(overrides))
	for _, override := range overrides {
		key, ok := lookupPair(index, override.Pair)
		if !ok {
			return nil, fmt.Errorf("%w: override names an unknown pair", ErrInvalidInput)
		}
		if _, duplicate := overridden[key]; duplicate {
			return nil, fmt.Errorf("%w: duplicate pair override", ErrInvalidInput)
		}
		if pairs[key].excluded {
			return nil, fmt.Errorf("%w: excluded pair has a material override", ErrInvalidInput)
		}
		if err := validateMaterial(Material{Restitution: override.Restitution, Friction: override.Friction}); err != nil {
			return nil, err
		}
		overridden[key] = override
	}
	for key := range pairs {
		pair := &pairs[key]
		if pair.excluded {
			// An excluded pair skips effective material mixing.
			continue
		}
		if override, ok := overridden[key]; ok {
			pair.restitution = override.Restitution
			pair.friction = exactFrictionCoefficient(override.Friction)
		} else if err := mixPairMaterial(pair, bodies[pair.a].definition.Material,
			bodies[pair.b].definition.Material); err != nil {
			return nil, err
		}
		if !pair.moving || pair.friction.lower == nil || pair.friction.lower.Sign() <= 0 {
			continue
		}
		roleA, roleB := bodies[pair.a].definition.Role, bodies[pair.b].definition.Role
		if (roleA != Fixed || roleB != Dynamic) && (roleA != Dynamic || roleB != Fixed) &&
			(roleA != Dynamic || roleB != Dynamic) {
			return nil, fmt.Errorf("%w: positive friction requires a fixed/dynamic or dynamic/dynamic pair", ErrUnsupported)
		}
	}
	return pairs, nil
}

// mixPairMaterial sets the smaller restitution and the bounded geometric mean
// of the two body friction coefficients.
func mixPairMaterial(pair *worldPair, a, b Material) error {
	pair.restitution = a.Restitution
	if exactBase(b.Restitution).Cmp(exactBase(a.Restitution)) < 0 {
		pair.restitution = b.Restitution
	}
	friction, ok := mixBodyFriction(a.Friction, b.Friction)
	if ok {
		pair.friction = friction
		return nil
	}
	if pair.moving {
		return fmt.Errorf("%w: effective friction is outside the finite nonzero scalar range", ErrUnsupported)
	}
	// A Fixed/Fixed pair enters no response, so its unrepresentable mean is
	// left unrecorded instead of refusing the world.
	return nil
}

// Bodies returns copies of the world's rigid-body definitions in world order.
func (w *World) Bodies() []RigidBody {
	if w == nil {
		return nil
	}
	out := make([]RigidBody, len(w.bodies))
	for i, body := range w.bodies {
		definition := body.definition
		if definition.Density != nil {
			density := *definition.Density
			definition.Density = &density
		}
		if definition.Supplied != nil {
			supplied := *definition.Supplied
			definition.Supplied = &supplied
		}
		out[i] = definition
	}
	return out
}

// Pairs returns every body pair in canonical world order: (0,1), (0,2), …,
// (1,2), …. Excluded pairs are included; Excluded lists them separately.
func (w *World) Pairs() []BodyPair {
	if w == nil {
		return nil
	}
	out := make([]BodyPair, len(w.pairs))
	for i, pair := range w.pairs {
		out[i] = w.bodyPair(pair)
	}
	return out
}

// Excluded returns the excluded pairs in canonical world order.
func (w *World) Excluded() []BodyPair {
	if w == nil {
		return nil
	}
	var out []BodyPair
	for _, pair := range w.pairs {
		if pair.excluded {
			out = append(out, w.bodyPair(pair))
		}
	}
	return out
}

func (w *World) bodyPair(pair worldPair) BodyPair {
	return BodyPair{A: w.bodies[pair.a].definition.Body, B: w.bodies[pair.b].definition.Body}
}
