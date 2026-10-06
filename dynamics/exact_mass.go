package dynamics

import (
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// exactMass is one dynamic body's admitted mass reading converted to exact
// rationals once, by NewWorld. The certificate, the conservation readings and
// the kick read the same values on every step; converting them per read
// dominated the island certificate. A World never changes a body's mass after
// admission, so the conversion is the one each reader made before, and every
// reader shares it: no field here is ever mutated or handed to a proof
// constructor that takes ownership. A nil field marks a reading that does not
// convert, and every reader applies its own check to it, as it did to the
// converted reading.
type exactMass struct {
	mass, bound *big.Rat          // exactBase of the mass value and bound
	low, high   *big.Rat          // mass − bound and mass + bound; nil when either is nil
	interval    proof.RatInterval // [low, high]
	components  [6]exactComponent // inertiaComponents order
	// tensor is the local inertia tensor as value ± bound intervals, and
	// largest the largest magnitude any of them admits; both are valid only
	// when tensorOK, which holds when every component converts with a
	// nonnegative bound.
	tensor     [3][3]proof.RatInterval
	largest    *big.Rat
	tensorOK   bool
	floor      *big.Rat    // certifiedInertiaFloor
	rowCeiling *big.Rat    // inertiaRowCeiling
	center     r3.Vec      // the mass center's source value
	local      [3]*big.Rat // the exact rationals of center; nil when not finite
	radius     *big.Rat    // exactBase of the mass center's bound
	// shared holds the float-sourced readings above over D^0, which is the
	// same rational under every SharedDenom, for the island certificate's
	// direct reading (island_certify.go).
	shared sharedMass
}

// sharedMass is the part of an exactMass the island certificate reads, over
// D^0. Each field holds the rational its big.Rat twin holds, and is valid
// where that twin is: interval when mass and bound convert, tensor, largest
// and components when tensorOK, local when every local coordinate converts,
// radius when the center's bound converts. The inertia floor is not here: it
// divides, so its denominator may have an odd factor, and the certificate
// lifts it over its own D.
type sharedMass struct {
	interval   proof.SInterval
	tensor     [3][3]proof.SInterval
	largest    proof.SRat
	components [6]sharedComponent
	local      [3]proof.SRat
	radius     proof.SRat
}

// sharedComponent is an exactComponent over D^0.
type sharedComponent struct {
	value, bound proof.SRat
	i, j         int
}

// exactComponent is one source inertia component: its exact value and bound
// (nil when not finite) and its tensor indices.
type exactComponent struct {
	value, bound *big.Rat
	i, j         int
}

// exactInertia converts the six source inertia components in
// inertiaComponents order.
func exactInertia(inertia decad.InertiaReading) [6]exactComponent {
	var out [6]exactComponent
	for k, component := range inertiaComponents(inertia) {
		out[k] = exactComponent{value: exactBase(component.reading.Value), bound: exactBase(component.reading.Bound),
			i: component.i, j: component.j}
	}
	return out
}

// newExactMass converts one admitted mass reading.
func newExactMass(m decad.MassProperties) *exactMass {
	out := &exactMass{mass: exactBase(m.Mass.Value), bound: exactBase(m.Mass.Bound),
		components: exactInertia(m.Inertia), floor: certifiedInertiaFloor(m),
		rowCeiling: inertiaRowCeiling(m.Inertia), center: m.Center.Value, radius: exactBase(m.Center.Bound)}
	if out.mass != nil && out.bound != nil {
		out.low = new(big.Rat).Sub(out.mass, out.bound)
		out.high = new(big.Rat).Add(out.mass, out.bound)
		out.interval = proof.OwnedInterval(new(big.Rat).Set(out.low), new(big.Rat).Set(out.high))
	}
	for axis, coordinate := range [3]float64{m.Center.Value.X, m.Center.Value.Y, m.Center.Value.Z} {
		out.local[axis] = ratFloat(coordinate)
	}
	out.readShared(m)
	out.tensorOK = true
	out.largest = new(big.Rat)
	for _, component := range out.components {
		if component.value == nil || component.bound == nil || component.bound.Sign() < 0 {
			out.tensorOK = false
			return out
		}
	}
	for _, component := range out.components {
		interval := proof.OwnedInterval(new(big.Rat).Sub(component.value, component.bound),
			new(big.Rat).Add(component.value, component.bound))
		out.tensor[component.i][component.j] = interval
		out.tensor[component.j][component.i] = interval
		if magnitude := magnitude(interval); magnitude.Cmp(out.largest) > 0 {
			out.largest = magnitude
		}
	}
	return out
}

// readShared fills shared from the source reading, with the conversions and
// the sums the big.Rat fields were built with.
func (out *exactMass) readShared(m decad.MassProperties) {
	s := proof.NewSharedDenom(nil)
	shared := &out.shared
	mass, okMass := sharedBase(s, m.Mass.Value)
	bound, okBound := sharedBase(s, m.Mass.Bound)
	if okMass && okBound {
		shared.interval = proof.SInterval{Lo: s.Sub(mass, bound), Hi: s.Add(mass, bound)}
	}
	center, _ := sharedVec(m.Center.Value)
	shared.local = center
	shared.radius, _ = sharedBase(s, m.Center.Bound)
	for k, component := range inertiaComponents(m.Inertia) {
		value, okValue := sharedBase(s, component.reading.Value)
		bound, okBound := sharedBase(s, component.reading.Bound)
		if !okValue || !okBound || bound.Sign() < 0 {
			return
		}
		shared.components[k] = sharedComponent{value: value, bound: bound, i: component.i, j: component.j}
	}
	for k := range shared.components {
		component := &shared.components[k]
		interval := proof.SInterval{Lo: s.Sub(component.value, component.bound),
			Hi: s.Add(component.value, component.bound)}
		shared.tensor[component.i][component.j] = interval
		shared.tensor[component.j][component.i] = interval
		if magnitude := s.Magnitude(interval); s.Cmp(magnitude, shared.largest) > 0 {
			shared.largest = magnitude
		}
	}
}
