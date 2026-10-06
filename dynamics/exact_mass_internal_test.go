package dynamics

import (
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests hold the per-World exact mass reading (exact_mass.go) and the
// zero shortcuts of the certificate and the conservation readings to the
// forms in exact_mass_oracle_internal_test.go: on the same random inputs,
// including readings that fail to convert, both must reach the same decision
// and exactly the same rationals.

// equivalenceInputs draws the random inputs. Every draw mixes zeros of both
// signs, small integers, dyadics and arbitrary reals, so the zero shortcuts,
// the exact unit factors and the general paths all run.
type equivalenceInputs struct {
	t *testing.T
	r *rand.Rand
}

func newEquivalenceInputs(t *testing.T, seed uint64) *equivalenceInputs {
	return &equivalenceInputs{t: t, r: rand.New(rand.NewPCG(seed, seed^0x9e3779b97f4a7c15))}
}

func (g *equivalenceInputs) float() float64 {
	switch g.r.IntN(9) {
	case 0:
		return 0
	case 1:
		return math.Copysign(0, -1)
	case 2:
		return float64(g.r.IntN(41) - 20)
	case 3:
		return float64(g.r.IntN(2001)-1000) / 64
	case 4:
		return g.r.NormFloat64() * 1e3
	case 5:
		return g.r.NormFloat64() * 1e-6
	case 6:
		return g.r.NormFloat64()
	default:
		return g.r.Float64()*200 - 100
	}
}

// nonnegative is a bound: zero most often, else a small positive value.
func (g *equivalenceInputs) nonnegative() float64 {
	if g.r.IntN(2) == 0 {
		return 0
	}
	return math.Abs(g.float()) * 1e-6
}

func (g *equivalenceInputs) vec() r3.Vec { return r3.Vec{X: g.float(), Y: g.float(), Z: g.float()} }

// linear and angular draw velocities, in base units most often, sometimes in
// a unit whose factor is not one, and rarely with an infinite component.
func (g *equivalenceInputs) linear() QuantityVec {
	unit := units.MillimeterPerSecond
	switch g.r.IntN(6) {
	case 0:
		unit = units.MeterPerSecond
	case 1:
		if g.r.IntN(8) == 0 {
			return QuantityVec{X: units.New(math.Inf(1), unit), Y: units.New(0, unit), Z: units.New(0, unit)}
		}
	}
	if g.r.IntN(3) == 0 {
		return QuantityVec{X: units.New(0, unit), Y: units.New(0, unit), Z: units.New(0, unit)}
	}
	return QuantityVec{X: units.New(g.float(), unit), Y: units.New(g.float(), unit), Z: units.New(g.float(), unit)}
}

func (g *equivalenceInputs) angular() QuantityVec {
	unit := units.RadianPerSecond
	switch g.r.IntN(6) {
	case 0:
		unit = units.DegreePerSecond
	case 1:
		if g.r.IntN(8) == 0 {
			return QuantityVec{X: units.New(0, unit), Y: units.New(math.Inf(-1), unit), Z: units.New(0, unit)}
		}
	}
	if g.r.IntN(2) == 0 {
		return QuantityVec{X: units.New(0, unit), Y: units.New(0, unit), Z: units.New(0, unit)}
	}
	return QuantityVec{X: units.New(g.float(), unit), Y: units.New(g.float(), unit), Z: units.New(g.float(), unit)}
}

func (g *equivalenceInputs) pose() r3.Transform {
	translation, err := r3.Translation(g.vec())
	require.NoError(g.t, err)
	if g.r.IntN(3) == 0 {
		return translation
	}
	axis := r3.Vec{X: g.r.NormFloat64(), Y: g.r.NormFloat64(), Z: g.r.NormFloat64() + 0.1}
	turn, err := r3.RotationAround(g.vec(), axis, units.Radians(g.r.Float64()*2*math.Pi))
	require.NoError(g.t, err)
	pose, err := turn.Then(translation)
	require.NoError(g.t, err)
	return pose
}

// mass draws a mass reading. Most are valid; some carry a negative bound, a
// nonpositive mass interval, an indefinite tensor, an infinite reading or a
// non-base mass unit, so every conversion and check fails somewhere.
func (g *equivalenceInputs) mass() decad.MassProperties {
	measure := func(value, bound float64, unit units.Unit) decad.Measurement {
		return decad.Measurement{Value: units.New(value, unit), Bound: units.New(bound, unit)}
	}
	massUnit, massValue := units.Kilogram, 0.5+g.r.Float64()*4
	if g.r.IntN(4) == 0 {
		massUnit, massValue = units.Gram, 500+g.r.Float64()*4000
	}
	massBound := g.nonnegative()
	switch g.r.IntN(16) {
	case 0:
		massBound = -1e-3
	case 1:
		massBound = massValue
	case 2:
		massValue = math.Inf(1)
	}
	diagonal := func() float64 { return 100 + g.r.Float64()*900 }
	off := func() float64 {
		switch g.r.IntN(4) {
		case 0:
			return 0
		case 1:
			return g.r.NormFloat64() * 2000
		default:
			return g.r.NormFloat64() * 10
		}
	}
	component := func(value float64) decad.Measurement {
		bound := g.nonnegative() * 1e3
		if g.r.IntN(40) == 0 {
			bound = -1
		}
		return measure(value, bound, units.KilogramSquareMillimeter)
	}
	centerBound := g.nonnegative()
	if g.r.IntN(20) == 0 {
		centerBound = -1
	}
	return decad.MassProperties{
		Mass:   measure(massValue, massBound, massUnit),
		Center: decad.VecMeasurement{Value: g.vec(), Bound: units.Millimeters(centerBound)},
		Inertia: decad.InertiaReading{XX: component(diagonal()), YY: component(diagonal()), ZZ: component(diagonal()),
			XY: component(off()), XZ: component(off()), YZ: component(off())},
	}
}

// world draws n bodies: Dynamic most often, with Fixed and Kinematic ones,
// under the residuals of a typical scene.
func (g *equivalenceInputs) world(n int) *World {
	w := &World{bodies: make([]worldBody, n), step: StepConfig{
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-3),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(float64(g.r.IntN(3)) * 32),
	}}
	for i := range w.bodies {
		switch g.r.IntN(5) {
		case 0:
			w.bodies[i].definition.Role = Fixed
		case 1:
			w.bodies[i].definition.Role = Kinematic
		default:
			mass := g.mass()
			w.bodies[i] = worldBody{definition: RigidBody{Role: Dynamic}, mass: mass, exact: newExactMass(mass)}
		}
	}
	return w
}

func (g *equivalenceInputs) state(w *World) State {
	state := State{world: w, entries: make([]BodyState, len(w.bodies))}
	for i := range state.entries {
		state.entries[i] = BodyState{Pose: g.pose(), LinearVelocity: g.linear(), AngularVelocity: g.angular()}
	}
	return state
}

func (g *equivalenceInputs) drive(w *World) map[int]driverMotion {
	drive := map[int]driverMotion{}
	for i, body := range w.bodies {
		if body.definition.Role != Kinematic || g.r.IntN(4) == 0 {
			continue
		}
		// An exact driver velocity divides by a duration, so some components
		// carry an odd factor in their denominators, and the certificate's
		// shared denominator is then not 1.
		value := func() *big.Rat {
			r := ratFloat(g.float())
			if g.r.IntN(3) == 0 {
				r.Quo(r, big.NewRat(int64(3+2*g.r.IntN(4)), 1))
			}
			return r
		}
		var motion driverMotion
		for axis := range 3 {
			motion.linear[axis], motion.angular[axis] = value(), new(big.Rat)
			if g.r.IntN(2) == 0 {
				motion.angular[axis] = value()
			}
		}
		drive[i] = motion
	}
	return drive
}

func (g *equivalenceInputs) contactPoint() decad.ContactPoint {
	normal := r3.Vec{X: g.r.NormFloat64(), Y: g.r.NormFloat64(), Z: g.r.NormFloat64()}
	length := math.Sqrt(normal.Dot(normal))
	normal = r3.Vec{X: normal.X / length, Y: normal.Y / length, Z: normal.Z / length}
	if g.r.IntN(3) == 0 {
		normal = r3.Vec{Z: 1}
	}
	at := g.vec()
	return decad.ContactPoint{
		OnA:         decad.VecMeasurement{Value: at, Bound: units.Millimeters(g.nonnegative())},
		OnB:         decad.VecMeasurement{Value: at, Bound: units.Millimeters(g.nonnegative())},
		Normal:      decad.VecMeasurement{Value: normal, Bound: units.Scalar(g.nonnegative())},
		NormalAngle: units.Radians(g.nonnegative()),
	}
}

func requireSameRat(t *testing.T, want, got *big.Rat, msgAndArgs ...any) {
	t.Helper()
	if want == nil || got == nil {
		require.True(t, want == nil && got == nil, msgAndArgs...)
		return
	}
	require.Zero(t, want.Cmp(got), append([]any{"want %s, got %s", want.RatString(), got.RatString()},
		msgAndArgs...)...)
	// The same lowest terms, numerator and denominator, not only the same
	// value: a form that skips big.Rat's reduction must still reach them.
	require.Zero(t, want.Num().Cmp(got.Num()), msgAndArgs...)
	require.Zero(t, want.Denom().Cmp(got.Denom()), msgAndArgs...)
}

func requireSameRats(t *testing.T, want, got [3]*big.Rat, msgAndArgs ...any) {
	t.Helper()
	for axis := range want {
		requireSameRat(t, want[axis], got[axis], msgAndArgs...)
	}
}

func requireSameInterval(t *testing.T, want, got proof.RatInterval, msgAndArgs ...any) {
	t.Helper()
	requireSameRat(t, want.Lo, got.Lo, msgAndArgs...)
	requireSameRat(t, want.Hi, got.Hi, msgAndArgs...)
}

func requireSameIVec(t *testing.T, want, got ivec, msgAndArgs ...any) {
	t.Helper()
	for axis := range want {
		requireSameInterval(t, want[axis], got[axis], msgAndArgs...)
	}
}

func TestRatFloatMatchesSetFloat64(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 1)
	values := []float64{0, math.Copysign(0, -1), 1, -1, math.MaxFloat64, -math.MaxFloat64,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 0x1p-1022, math.Inf(1), math.Inf(-1), math.NaN()}
	for range 2000 {
		values = append(values, g.float())
	}
	for _, value := range values {
		requireSameRat(t, oldRatFloat(value), ratFloat(value), "ratFloat(%v)", value)
	}
}

func TestExactBaseMatchesUnitProduct(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 2)
	unitsUnderTest := []units.Unit{units.Millimeter, units.Centimeter, units.Meter, units.Kilogram, units.Gram,
		units.MillimeterPerSecond, units.MeterPerSecond, units.RadianPerSecond, units.DegreePerSecond,
		units.Degree, units.Radian, units.KilogramSquareMillimeter, units.One}
	magnitudes := []float64{0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(), math.MaxFloat64}
	for range 500 {
		magnitudes = append(magnitudes, g.float())
	}
	for _, unit := range unitsUnderTest {
		for _, magnitude := range magnitudes {
			value := units.New(magnitude, unit)
			requireSameRat(t, oldExactBase(value), exactBase(value), "exactBase(%v)", value)
		}
	}
}

// TestExactMassMatchesReadings holds every cached conversion to the
// conversion its readers made from the source reading.
func TestExactMassMatchesReadings(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 3)
	for k := range 500 {
		m := g.mass()
		exact := newExactMass(m)
		requireSameRat(t, oldExactBase(m.Mass.Value), exact.mass, "draw %d mass", k)
		requireSameRat(t, oldExactBase(m.Mass.Bound), exact.bound, "draw %d bound", k)
		if exact.mass != nil && exact.bound != nil {
			requireSameRat(t, new(big.Rat).Sub(exact.mass, exact.bound), exact.low, "draw %d low", k)
			requireSameRat(t, new(big.Rat).Add(exact.mass, exact.bound), exact.high, "draw %d high", k)
			requireSameInterval(t, proof.OwnedInterval(exact.low, exact.high), exact.interval, "draw %d interval", k)
		} else {
			require.Nil(t, exact.low, "draw %d", k)
		}
		valid := true
		for i, component := range inertiaComponents(m.Inertia) {
			value, bound := oldExactBase(component.reading.Value), oldExactBase(component.reading.Bound)
			requireSameRat(t, value, exact.components[i].value, "draw %d component %d", k, i)
			requireSameRat(t, bound, exact.components[i].bound, "draw %d component %d bound", k, i)
			require.Equal(t, [2]int{component.i, component.j}, [2]int{exact.components[i].i, exact.components[i].j})
			valid = valid && value != nil && bound != nil && bound.Sign() >= 0
		}
		largest := new(big.Rat)
		for _, component := range inertiaComponents(m.Inertia) {
			if !valid {
				break
			}
			value, bound := oldExactBase(component.reading.Value), oldExactBase(component.reading.Bound)
			interval := proof.OwnedInterval(new(big.Rat).Sub(value, bound), new(big.Rat).Add(value, bound))
			requireSameInterval(t, interval, exact.tensor[component.i][component.j], "draw %d tensor", k)
			requireSameInterval(t, interval, exact.tensor[component.j][component.i], "draw %d tensor", k)
			if m := magnitude(interval); m.Cmp(largest) > 0 {
				largest = m
			}
		}
		require.Equal(t, valid, exact.tensorOK, "draw %d", k)
		if valid {
			requireSameRat(t, largest, exact.largest, "draw %d largest", k)
		}
		requireSameRat(t, certifiedInertiaFloor(m), exact.floor, "draw %d floor", k)
		requireSameRat(t, inertiaRowCeiling(m.Inertia), exact.rowCeiling, "draw %d ceiling", k)
		center, _ := oldRatVec(m.Center.Value)
		requireSameRats(t, center, exact.local, "draw %d center", k)
		requireSameRat(t, oldExactBase(m.Center.Bound), exact.radius, "draw %d radius", k)
	}
}

func requireSameCertBody(t *testing.T, want, got certBody, msgAndArgs ...any) {
	t.Helper()
	require.Equal(t, [3]any{want.index, want.dynamic, want.kinematic}, [3]any{got.index, got.dynamic, got.kinematic},
		msgAndArgs...)
	requireSameRats(t, want.v, got.v, msgAndArgs...)
	requireSameRats(t, want.w, got.w, msgAndArgs...)
	requireSameRats(t, want.vPost, got.vPost, msgAndArgs...)
	requireSameRats(t, want.wPost, got.wPost, msgAndArgs...)
	if !want.dynamic && !want.kinematic {
		return
	}
	requireSameIVec(t, want.center, got.center, msgAndArgs...)
	if !want.dynamic {
		return
	}
	requireSameInterval(t, want.mass, got.mass, msgAndArgs...)
	for i := range 3 {
		requireSameIVec(t, want.inertia[i], got.inertia[i], msgAndArgs...)
		requireSameRats(t, want.rotation[i], got.rotation[i], msgAndArgs...)
	}
	requireSameRat(t, want.defect, got.defect, msgAndArgs...)
	requireSameRat(t, want.inertiaLower, got.inertiaLower, msgAndArgs...)
	requireSameRat(t, want.rowCeiling, got.rowCeiling, msgAndArgs...)
	requireSameRat(t, want.centerL1, got.centerL1, msgAndArgs...)
}

// TestCertBodyMatchesOldForm compares newCertBody, and certMotion against
// newCertBody with an unchanged post velocity, as pairActive called it.
func TestCertBodyMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 4)
	seen := map[bool]int{}
	for k := range 100 {
		w := g.world(4)
		pre, post, drive := g.state(w), g.state(w), g.drive(w)
		for i := range w.bodies {
			want, okWant := oldNewCertBody(w, i, pre.entries[i], post.entries[i], drive)
			got, okGot := w.newCertBody(i, pre.entries[i], post.entries[i], drive)
			require.Equal(t, okWant, okGot, "draw %d body %d", k, i)
			seen[okWant]++
			if okWant {
				requireSameCertBody(t, want, got, "draw %d body %d", k, i)
			}
			want, okWant = oldNewCertBody(w, i, pre.entries[i], pre.entries[i], drive)
			got, okGot = w.certMotion(i, pre.entries[i], drive)
			require.Equal(t, okWant, okGot, "draw %d body %d motion", k, i)
			if !okWant {
				continue
			}
			require.Equal(t, [3]any{want.index, want.dynamic, want.kinematic},
				[3]any{got.index, got.dynamic, got.kinematic})
			requireSameRats(t, want.v, got.v, "draw %d body %d motion", k, i)
			requireSameRats(t, want.w, got.w, "draw %d body %d motion", k, i)
			requireSameRats(t, want.vPost, got.vPost, "draw %d body %d motion", k, i)
			requireSameRats(t, want.wPost, got.wPost, "draw %d body %d motion", k, i)
			if want.dynamic || want.kinematic {
				requireSameIVec(t, want.center, got.center, "draw %d body %d motion", k, i)
			}
		}
	}
	require.Positive(t, seen[true], "premise: some bodies read")
	require.Positive(t, seen[false], "premise: some bodies fail to read")
}

// TestCertPointMatchesOldForm compares newCertPoint's balls and levers with
// the old form's over bodies with and without a mass center.
func TestCertPointMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 14)
	read := 0
	for k := 0; read < 200; k++ {
		w := g.world(3)
		state, drive := g.state(w), g.drive(w)
		bodies := make([]certBody, len(w.bodies))
		ok := true
		for i := range bodies {
			bodies[i], ok = w.newCertBody(i, state.entries[i], state.entries[i], drive)
			if !ok {
				break
			}
		}
		if !ok {
			continue
		}
		read++
		a := g.r.IntN(len(bodies) - 1)
		b := a + 1 + g.r.IntN(len(bodies)-1-a)
		point := g.contactPoint()
		want, okWant := oldNewCertPoint(0, a, b, point, bodies, new(big.Rat))
		got, okGot := newCertPoint(0, a, b, point, bodies, new(big.Rat))
		require.Equal(t, okWant, okGot, "draw %d", k)
		if !okWant {
			continue
		}
		for _, pair := range [][2]ivec{{want.normal, got.normal}, {want.onA, got.onA}, {want.onB, got.onB},
			{want.rA, got.rA}, {want.rB, got.rB}} {
			requireSameIVec(t, pair[0], pair[1], "draw %d", k)
		}
		requireSameRat(t, want.ballA, got.ballA, "draw %d", k)
		requireSameRat(t, want.ballB, got.ballB, "draw %d", k)
	}
}

func TestPointVelocityMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 5)
	for k := range 500 {
		body := certBody{dynamic: g.r.IntN(2) == 0, kinematic: g.r.IntN(2) == 0}
		var v, omega [3]*big.Rat
		for axis := range 3 {
			v[axis], omega[axis] = ratFloat(g.float()), ratFloat(g.float())
		}
		if g.r.IntN(2) == 0 {
			omega = [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
		}
		lever, ok := ballIVec(g.vec(), ratFloat(g.nonnegative()))
		require.True(t, ok)
		requireSameIVec(t, oldPointVelocity(body, v, omega, lever), pointVelocity(body, v, omega, lever), "draw %d", k)
	}
}

// pairInputs draws a world, a state, a driver field and a gathered pair of
// two of its bodies with one to four manifold points.
func (g *equivalenceInputs) pairInputs() (*World, State, map[int]driverMotion, islandPair) {
	w := g.world(3)
	pair := islandPair{a: g.r.IntN(2)}
	pair.b = pair.a + 1 + g.r.IntN(2-pair.a)
	for range 1 + g.r.IntN(4) {
		pair.manifold.Points = append(pair.manifold.Points, g.contactPoint())
	}
	state := g.state(w)
	// A slow pre-solve speed lets some points close within VelocityResidual.
	for i := range state.entries {
		if g.r.IntN(2) == 0 {
			state.entries[i].LinearVelocity = QuantityVec{X: units.MillimetersPerSecond(g.r.NormFloat64() * 1e-7),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(g.r.NormFloat64() * 1e-7)}
		}
	}
	return w, state, g.drive(w), pair
}

func TestPairActiveMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 6)
	seen := map[[2]bool]int{}
	grazes := map[bool]int{}
	for k := range 400 {
		w, state, drive, pair := g.pairInputs()
		activeWant, validWant := oldPairActive(w, pair, state, drive)
		activeGot, validGot := w.pairActive(pair, state, drive)
		require.Equal(t, [2]bool{activeWant, validWant}, [2]bool{activeGot, validGot}, "draw %d", k)
		seen[[2]bool{activeWant, validWant}]++
		graze := oldGrazeSpeedWithin(w, pair, state, drive)
		require.Equal(t, graze, w.grazeSpeedWithin(pair, state, drive), "draw %d graze", k)
		grazes[graze]++
	}
	require.Len(t, seen, 3, "premise: active, separating and unreadable pairs all occur: %v", seen)
	require.Len(t, grazes, 2, "premise: grazes within and beyond the residual both occur")
}

func TestSpinReadingMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 7)
	seen := map[bool]int{}
	for k := range 400 {
		m, pose, omega := g.mass(), g.pose(), g.angular()
		components := exactInertia(m.Inertia)
		want, okWant := oldSpinReadings(m.Inertia, pose, omega)
		got, okGot := spinReadings(&components, pose, omega)
		require.Equal(t, okWant, okGot, "draw %d", k)
		seen[okWant]++
		if okWant {
			requireSameRats(t, want.value, got.value, "draw %d value", k)
			requireSameRats(t, want.low, got.low, "draw %d low", k)
			requireSameRats(t, want.high, got.high, "draw %d high", k)
			requireSameRats(t, [3]*big.Rat{want.energy, want.energyLow, want.energyHigh},
				[3]*big.Rat{got.energy, got.energyLow, got.energyHigh}, "draw %d energy", k)
		}
		rotationWant, localWant, okWant := oldSpinBasis(pose, omega)
		rotationGot, localGot, okGot := spinBasis(pose, omega)
		require.Equal(t, okWant, okGot, "draw %d basis", k)
		if okWant {
			for row := range 3 {
				requireSameRats(t, rotationWant[row], rotationGot[row], "draw %d basis", k)
			}
			requireSameRats(t, localWant, localGot, "draw %d basis", k)
		}
	}
	require.Len(t, seen, 2, "premise: readings that convert and that fail both occur")
}

// TestSpinEnergyChangeMatchesOldForm feeds the certificate's exact basis and
// velocities to the current form, and the old one their rad/s round trip.
func TestSpinEnergyChangeMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 8)
	for k := range 400 {
		m, pose := g.mass(), g.pose()
		before, okBefore := oldQuantityRats(g.angular())
		after, okAfter := oldQuantityRats(g.angular())
		if !okBefore || !okAfter {
			continue
		}
		var rotation [3][3]*big.Rat
		basis := pose.Basis()
		for column, axis := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
			values, ok := oldRatVec(axis)
			require.True(t, ok)
			for row := range 3 {
				rotation[row][column] = values[row]
			}
		}
		components := exactInertia(m.Inertia)
		want, okWant := oldSpinEnergyChange(m.Inertia, pose, oldRatQuantity(before), oldRatQuantity(after))
		got, okGot := spinEnergyChange(&components, rotation, before, after)
		require.Equal(t, okWant, okGot, "draw %d", k)
		if okWant {
			requireSameRat(t, want, got, "draw %d", k)
		}
	}
}

func TestAddIntervalProductMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 9)
	for k := range 1000 {
		start := [3]float64{g.float(), g.float(), g.float()}
		nominal, uncertainty, coefficient := ratFloat(g.float()), ratFloat(g.nonnegative()), ratFloat(g.float())
		if g.r.IntN(8) == 0 {
			uncertainty = ratFloat(-g.nonnegative())
		}
		var want, got [3]*big.Rat
		for i := range 3 {
			want[i], got[i] = ratFloat(start[i]), ratFloat(start[i])
		}
		oldAddIntervalProduct(want[0], want[1], want[2], nominal, uncertainty, coefficient)
		addIntervalProduct(got[0], got[1], got[2], nominal, uncertainty, coefficient)
		requireSameRats(t, want, got, "draw %d", k)
	}
}

func TestAddMassProductMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 10)
	for k := range 1000 {
		values := [6]*big.Rat{}
		for i := range values {
			values[i] = ratFloat(g.float())
		}
		mass, coefficient := values[0], values[3]
		massLow, massHigh := values[1], values[2]
		coefficientLow, coefficientHigh := values[4], values[5]
		if g.r.IntN(4) != 0 {
			// The callers' shape: an ordered positive mass interval about
			// the mass, and an ordered coefficient interval about the
			// coefficient.
			m, b := ratFloat(1+g.r.Float64()*4), ratFloat(g.nonnegative())
			mass, massLow, massHigh = m, new(big.Rat).Sub(m, b), new(big.Rat).Add(m, b)
			u := ratFloat(g.nonnegative())
			coefficientLow, coefficientHigh = new(big.Rat).Sub(coefficient, u), new(big.Rat).Add(coefficient, u)
		}
		start := [3]float64{g.float(), g.float(), g.float()}
		var want, got [3]*big.Rat
		for i := range 3 {
			want[i], got[i] = ratFloat(start[i]), ratFloat(start[i])
		}
		oldAddMassProduct(want[0], want[1], want[2], mass, massLow, massHigh, coefficient, coefficientLow,
			coefficientHigh)
		addMassProduct(got[0], got[1], got[2], mass, massLow, massHigh, coefficient, coefficientLow, coefficientHigh)
		requireSameRats(t, want, got, "draw %d", k)
	}
}

func TestWorldCenterReadingMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 11)
	seen := map[bool]int{}
	for k := range 500 {
		m, pose := g.mass(), g.pose()
		exact := newExactMass(m)
		centerWant, errorWant, okWant := oldWorldCenterReading(pose, m.Center)
		centerGot, errorGot, okGot := worldCenterReading(pose, exact)
		require.Equal(t, okWant, okGot, "draw %d", k)
		require.Equal(t, okWant, centerReadable(pose, exact), "draw %d readable", k)
		seen[okWant]++
		if okWant {
			requireSameRats(t, centerWant, centerGot, "draw %d center", k)
			requireSameRats(t, errorWant, errorGot, "draw %d error", k)
		}
	}
	require.Len(t, seen, 2, "premise: readable and unreadable centers both occur")
}

func TestConservationStateMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 12)
	seen := map[bool]int{}
	for k := range 200 {
		w := g.world(1 + g.r.IntN(4))
		state := g.state(w)
		want, okWant := oldConservationState(w, state)
		got, okGot := w.conservationState(state)
		require.Equal(t, okWant, okGot, "draw %d", k)
		require.Equal(t, want, got, "draw %d", k)
		seen[okWant]++
	}
	require.Len(t, seen, 2, "premise: readable and unreadable states both occur")
}

// TestStepWorkConservationReuse checks the readings a step reuses: the
// input state's carried Completion only for entries equal to the cache's,
// and a reading the step already took only for equal entries. A planted
// carried reading shows which path answered.
func TestStepWorkConservationReuse(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 15)
	readable := 0
	for k := 0; readable < 50; k++ {
		w := g.world(1 + g.r.IntN(4))
		from, other := g.state(w), g.state(w)
		if _, ok := w.conservationState(from); !ok {
			continue
		}
		readable++
		planted := ConservationState{KineticEnergy: decad.Measurement{Value: units.New(float64(k+1),
			units.KilogramSquareMillimeterPerSecondSquared)}}
		from.cache = &contactCache{entries: slices.Clone(from.entries), completion: &planted}
		work := newStepWork(w, from)
		got, ok := work.conservation(from)
		require.True(t, ok)
		require.Equal(t, planted, got, "draw %d: the carried reading answers its own entries", k)
		want, okWant := w.conservationState(other)
		got, okGot := work.conservation(other)
		require.Equal(t, okWant, okGot, "draw %d", k)
		require.Equal(t, want, got, "draw %d: other entries read afresh", k)
		again, okAgain := work.conservation(State{world: w, entries: slices.Clone(other.entries)})
		require.Equal(t, [2]any{want, okWant}, [2]any{again, okAgain}, "draw %d: equal entries reuse", k)
		moved := from.clone()
		speed := 7.0
		if moved.entries[0].LinearVelocity.X.Base() == speed {
			speed = 8
		}
		moved.entries[0].LinearVelocity.X = units.MillimetersPerSecond(speed)
		want, okWant = w.conservationState(moved)
		got, okGot = work.conservation(moved)
		require.Equal(t, [2]any{want, okWant}, [2]any{got, okGot}, "draw %d: changed entries read afresh", k)
		require.NotEqual(t, planted, got, "draw %d", k)
	}
}

// island draws a certificate's input: two to four bodies, read before and
// after with random drivers, and one to six points between them, some of
// whose normal or tangent impulses are exactly zero. ok is false when a body
// does not read as exact intervals.
func (g *equivalenceInputs) island() (*World, []certBody, []certPoint, bool) {
	w := g.world(2 + g.r.IntN(3))
	pre, post, drive := g.state(w), g.state(w), g.drive(w)
	bodies := make([]certBody, len(w.bodies))
	for i := range bodies {
		var ok bool
		if bodies[i], ok = w.newCertBody(i, pre.entries[i], post.entries[i], drive); !ok {
			return nil, nil, nil, false
		}
	}
	var points []certPoint
	for range 1 + g.r.IntN(6) {
		a := g.r.IntN(len(bodies) - 1)
		b := a + 1 + g.r.IntN(len(bodies)-1-a)
		p, ok := newCertPoint(0, a, b, g.contactPoint(), bodies, ratFloat(float64(g.r.IntN(3))*0.3))
		require.True(g.t, ok)
		p.mu, p.lambda = ratFloat(float64(g.r.IntN(3))*0.2), ratFloat(math.Abs(g.float()))
		if g.r.IntN(8) == 0 {
			p.lambda = ratFloat(-1)
		}
		p.tangent = [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
		switch g.r.IntN(3) {
		case 0:
			p.lambda = new(big.Rat)
		case 1:
			for axis := range 3 {
				p.tangent[axis] = ratFloat(g.float() * 1e-3)
			}
		}
		points = append(points, p)
	}
	return w, bodies, points, true
}

func requireSameCertificate(t *testing.T, want, got islandCertificate, msgAndArgs ...any) {
	t.Helper()
	for i, pair := range [][2]*big.Rat{{want.linear, got.linear}, {want.angular, got.angular},
		{want.normal, got.normal}, {want.energy, got.energy}, {want.momentum, got.momentum},
		{want.angularMomentum, got.angularMomentum}, {want.cone, got.cone}, {want.tangent, got.tangent},
		{want.spin, got.spin}, {want.witnessTorque, got.witnessTorque}, {want.witnessSpin, got.witnessSpin},
		{want.value, got.value}, {want.limit, got.limit}} {
		requireSameRat(t, pair[0], pair[1], append([]any{"field %d"}, append([]any{i}, msgAndArgs...)...)...)
	}
	require.Equal(t, want.failed, got.failed, msgAndArgs...)
	require.Equal(t, want.refused, got.refused, msgAndArgs...)
}

// TestCertifyIslandMatchesOldForm certifies random islands, some of whose
// points carry an exactly zero normal or tangent impulse, some of whose
// bodies do not spin and some of whose drivers move at velocities with odd
// denominators, with the old and the current certificate.
func TestCertifyIslandMatchesOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 13)
	passed, refused, shared := 0, 0, 0
	gates := map[islandGate]int{}
	for k := 0; passed+refused < 600; k++ {
		w, bodies, points, ok := g.island()
		if !ok {
			continue
		}
		want := oldCertifyIsland(w, bodies, points)
		got := w.certifyIsland(bodies, points)
		requireSameCertificate(t, want, got, "draw %d", k)
		if want.failed == 0 {
			passed++
		} else {
			refused++
			gates[want.failed]++
		}
		if oddDenominator(bodies, points) {
			shared++
		}
	}
	require.Positive(t, refused, "premise: some islands are refused")
	require.Positive(t, shared, "premise: some islands read a driver velocity whose denominator is not a power of two")
	t.Logf("%d islands passed, %d refused (first refused gate: %v), %d over an odd shared denominator",
		passed, refused, gates, shared)
}

// oddDenominator reports whether an island certificate reads a kinematic
// participant's velocity whose denominator is not a power of two, so that
// its shared denominator is not 1.
func oddDenominator(bodies []certBody, points []certPoint) bool {
	for _, p := range points {
		for _, slot := range [2]int{p.a, p.b} {
			if !bodies[slot].kinematic {
				continue
			}
			for _, value := range append(bodies[slot].v[:], bodies[slot].w[:]...) {
				if _, ok := proof.DyOfRat(value); !ok {
					return true
				}
			}
		}
	}
	return false
}

// sharedIVec converts a shared-denominator interval vector back to
// RatIntervals for comparison.
func sharedIVec(s *proof.SharedDenom, v proof.SIVec3) ivec {
	var out ivec
	for axis := range out {
		component := v[axis]
		out[axis] = proof.OwnedInterval(s.Rat(component.Lo), s.Rat(component.Hi))
	}
	return out
}

func zeroCertificate() islandCertificate {
	return islandCertificate{linear: new(big.Rat), angular: new(big.Rat), normal: new(big.Rat),
		energy: new(big.Rat), momentum: new(big.Rat), angularMomentum: new(big.Rat),
		cone: new(big.Rat), tangent: new(big.Rat), spin: new(big.Rat),
		witnessTorque: new(big.Rat), witnessSpin: new(big.Rat)}
}

// TestCertificateRowsMatchOldForm holds the parts of the shared-denominator
// certificate to their old forms one at a time, where a whole-island draw
// rarely reaches them first: the friction rows of each point over a fresh
// certificate, so a stick or slip refusal publishes its own value and limit;
// the Euclidean bounds of each point's interval vectors; each dynamic body's
// inertia product and witness torque.
func TestCertificateRowsMatchOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 17)
	rows := map[islandGate]int{}
	for k := range 400 {
		w, bodies, points, ok := g.island()
		if !ok {
			continue
		}
		// A slipping point lies within ImpulseResidual of its cone's surface,
		// which a random impulse rarely does: place some there.
		for i := range points {
			p := &points[i]
			mu, _ := p.mu.Float64()
			if mu == 0 || g.r.IntN(2) == 0 {
				continue
			}
			var square float64
			for _, component := range p.tangent {
				f, _ := component.Float64()
				square += f * f
			}
			p.lambda = ratFloat(math.Sqrt(square) / mu)
		}
		impulseLimit, velocityLimit := exactBase(w.step.ImpulseResidual), exactBase(w.step.VelocityResidual)
		oldImpulses := make([]ivec, len(points))
		for i, p := range points {
			oldImpulses[i] = oldAddIVec(oldScaleIVec(p.normal, p.lambda), pointIVec(p.tangent))
			want := zeroCertificate()
			oldCertifyFriction(&want, p, bodies, impulseLimit, velocityLimit)
			run := w.readIsland(bodies, points)
			run.certifyFriction(&run.points[i], run.bodies)
			requireSameCertificate(t, want, run.publish(), "draw %d point %d", k, i)
			rows[want.failed]++
		}
		run := w.readIsland(bodies, points)
		s := run.s
		impulses := make([]proof.SIVec3, len(points))
		for i, p := range points {
			read := &run.points[i]
			impulses[i] = s.AddI3(s.ScaleI3(read.normal, read.lambda), proof.SPoint3(read.tangent))
			requireSameIVec(t, oldImpulses[i], sharedIVec(s, impulses[i]), "draw %d point %d", k, i)
			for _, v := range [][2]any{{p.rA, read.rA}, {p.rB, read.rB}, {p.normal, read.normal},
				{oldImpulses[i], impulses[i]}} {
				old, shared := v[0].(ivec), v[1].(proof.SIVec3)
				requireSameRat(t, oldEuclideanUpper(old), s.Rat(run.euclideanUpper(shared)), "draw %d point %d", k, i)
				requireSameRat(t, oldEuclideanLower(old), s.Rat(run.euclideanLower(shared)), "draw %d point %d", k, i)
			}
		}
		for slot, body := range bodies {
			if !body.dynamic {
				continue
			}
			x := [3]*big.Rat{ratFloat(g.float()), ratFloat(g.float()), ratFloat(g.float())}
			requireSameIVec(t, oldInertiaApply(body, x), sharedIVec(s, run.inertiaApply(&run.bodies[slot], liftRats(s, x))),
				"draw %d body %d", k, slot)
			requireSameRat(t, oldBodyWitnessTorque(slot, points, oldImpulses),
				s.Rat(run.bodyWitnessTorque(slot, run.points, impulses)), "draw %d body %d", k, slot)
		}
	}
	for _, gate := range []islandGate{0, gateCone, gateStick, gateSlip} {
		require.Positive(t, rows[gate], "premise: some friction rows end at %v", gate)
	}
	t.Logf("friction rows: %v", rows)
}

// TestCertificateSquareRootsMatchOldForm holds the certificate's square root
// bounds to their old forms on squares of every scale, the two ends whose
// float leaves the normal range among them, over a shared denominator that
// is not 1.
func TestCertificateSquareRootsMatchOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 19)
	odd := []int64{1, 3, 5, 7, 15, 21}
	squares := []*big.Rat{new(big.Rat), big.NewRat(-1, 3),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(3), 1100)),
		new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(5), 1100), big.NewInt(7))}
	for range 2000 {
		r := new(big.Rat).Abs(ratFloat(g.float()))
		r.Mul(r, ratFloat(math.Ldexp(1, g.r.IntN(200)-100)))
		r.Quo(r, big.NewRat(odd[g.r.IntN(len(odd))], 1))
		squares = append(squares, r)
	}
	run := &islandRun{s: proof.NewSharedDenom(nil)}
	for _, square := range squares {
		run.s.Lift(square)
	}
	run.s, _ = run.s.Widen()
	for i, square := range squares {
		x := run.s.Lift(square)
		requireSameRat(t, oldRatSqrtUpper(square), run.s.Rat(run.sqrtUpper(x)), "square %d %s", i, square.RatString())
		// The old lower bound fails on a square whose float rounds to
		// infinity; the certificate's squares never do.
		if approx, _ := square.Float64(); !math.IsInf(approx, 0) {
			requireSameRat(t, oldRatSqrtLower(square), run.s.Rat(run.sqrtLower(x)), "square %d %s", i, square.RatString())
		}
	}
}

// TestMomentumRowsMatchOldForm runs the momentum rows alone over a fresh
// certificate, with residual vectors whose linear and angular rows fail on
// different axes, so the first refusal depends on the rows' order.
func TestMomentumRowsMatchOldForm(t *testing.T) {
	t.Parallel()
	g := newEquivalenceInputs(t, 23)
	firsts := map[islandGate]int{}
	vector := func() ivec {
		var out ivec
		for axis := range out {
			lo, hi := g.float(), g.float()
			if lo > hi {
				lo, hi = hi, lo
			}
			out[axis] = proof.OwnedInterval(ratFloat(lo), ratFloat(hi))
		}
		return out
	}
	for k := range 2000 {
		linear, angular := vector(), vector()
		linearLimit, angularLimit := ratFloat(math.Abs(g.float())), ratFloat(math.Abs(g.float()))
		want := zeroCertificate()
		oldCheckMomentum(&want, linear, angular, linearLimit, angularLimit)
		run := &islandRun{s: proof.NewSharedDenom(nil)}
		s := run.s
		run.checkMomentum(s.LiftIVec3(linear), s.LiftIVec3(angular), s.Lift(linearLimit), s.Lift(angularLimit))
		requireSameCertificate(t, want, run.publish(), "draw %d", k)
		firsts[want.failed]++
	}
	require.Positive(t, firsts[gateLinearMomentum], "premise: the linear row refuses first")
	require.Positive(t, firsts[gateAngularMomentum], "premise: the angular row refuses first")
}
