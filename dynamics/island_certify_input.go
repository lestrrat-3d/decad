package dynamics

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file reads the island certificate's inputs (island_certify.go), and
// the pre-solve relative normal speed of a gathered pair, straight from the
// held float readings into the certificate's shared-denominator arithmetic
// (internal/proof/shared_interval.go). Every float-sourced reading is a
// Dyadic over D^0, the same rational under every shared denominator D, and
// is read once. Only the inputs held as big.Rat are lifted over D: a driver's
// exact velocity, a body's inertia floor and row ceiling, and a pair's lower
// friction coefficient. One whose odd denominator D lacks widens D, and those
// inputs alone are lifted again.

// certInput is an island certificate's source: each participant's event
// state and published post-solve velocities, the drivers' exact fields, and
// each manifold point with its pair's coefficients and published impulse.
type certInput struct {
	bodies []certBodyInput
	points []certPointInput
	drive  map[int]driverMotion
}

// certBodyInput is one island participant: its world index, its state at the
// event and its published post-solve velocities.
type certBodyInput struct {
	index        int
	entry        BodyState
	vPost, wPost QuantityVec
}

// certPointInput is one manifold point between island slots a and b: its
// pair's restitution and lower friction coefficient μ_lo, and its published
// normal impulse and world tangent impulse on B.
type certPointInput struct {
	a, b        int
	point       *decad.ContactPoint
	restitution units.Value
	mu          *big.Rat
	lambda      float64
	tangent     r3.Vec
}

// sharedBody is one island participant read over the certificate's
// SharedDenom. Only a dynamic body carries its mass, inertia, pose and energy
// readings, and only a dynamic or kinematic one its center: a dynamic body's
// mass center ball, a kinematic one's world origin, about which its driver's
// field is written. A Fixed participant has zero velocity.
type sharedBody struct {
	index                                      int
	dynamic, kinematic                         bool
	mass                                       proof.SInterval
	inertia                                    [3][3]proof.SInterval
	rotation                                   [3][3]proof.SRat // pose basis, rotation[row][column]
	defect, inertiaLower, rowCeiling, centerL1 proof.SRat
	center                                     proof.SIVec3
	v, w                                       [3]proof.SRat // pre-solve
	vPost, wPost                               [3]proof.SRat // published
	// spinUpper is spinEnergyChange's reading, valid when spinOK.
	spinUpper proof.SRat
	spinOK    bool
}

// sharedPoint is one manifold point read over the certificate's
// SharedDenom. Its lever intervals run from each dynamic or kinematic
// body's center to that body's witness.
type sharedPoint struct {
	a, b                     int
	normal, onA, onB, rA, rB proof.SIVec3
	ballA, ballB             proof.SRat // the witness ball radii, OnA.Bound and OnB.Bound
	lambda, mu, restitution  proof.SRat
	tangent                  [3]proof.SRat
}

// readIsland reads every input of an island's certificate before any row
// runs: the participants in slot order, then the points in order. The first
// participant or point that does not read names the island's failure.
func (w *World) readIsland(in certInput) (*islandRun, *islandFailure) {
	s := proof.NewSharedDenom(nil)
	run := &islandRun{s: s}
	// NewWorld admits only finite residuals.
	run.impulseLimit, _ = sharedBase(s, w.step.ImpulseResidual)
	run.velocityLimit, _ = sharedBase(s, w.step.VelocityResidual)
	run.angularLimit, _ = sharedBase(s, w.step.AngularVelocityResidual)
	run.impactSpeed, _ = sharedBase(s, w.step.ImpactSpeed)
	run.bodies = make([]sharedBody, len(in.bodies))
	for slot := range in.bodies {
		body := &in.bodies[slot]
		if !w.readBody(s, &run.bodies[slot], body.index, &body.entry, &body.vPost, &body.wPost, in.drive) {
			return nil, &islandFailure{code: StepIslandDegenerate,
				reason: fmt.Sprintf("body %d cannot be read as exact intervals", body.index)}
		}
	}
	run.points = make([]sharedPoint, len(in.points))
	for k := range in.points {
		p, read := &in.points[k], &run.points[k]
		restitution, ok := sharedBase(s, p.restitution)
		if !readPoint(s, read, p.a, p.b, p.point, run.bodies) || !ok || p.mu == nil {
			return nil, &islandFailure{code: StepIslandDegenerate, reason: "manifold point cannot be read as exact intervals"}
		}
		lambda, okLambda := proof.SFloat(p.lambda)
		tangent, okTangent := sharedVec(p.tangent)
		if !okLambda || !okTangent {
			return nil, &islandFailure{code: StepIslandDegenerate, reason: "island impulse is not finite"}
		}
		read.restitution, read.lambda, read.tangent = restitution, lambda, tangent
	}
	for {
		w.liftIsland(run, in)
		next, widen := run.s.Widen()
		if !widen {
			return run, nil
		}
		run.s = next
	}
}

// liftIsland writes the island's rational inputs over the run's SharedDenom.
func (w *World) liftIsland(run *islandRun, in certInput) {
	s := run.s
	for slot := range run.bodies {
		read := &run.bodies[slot]
		liftDriver(s, read, in.drive)
		if !read.dynamic {
			continue
		}
		exact := w.bodies[read.index].exact
		read.inertiaLower, read.rowCeiling = s.Lift(exact.floor), s.Lift(exact.rowCeiling)
	}
	for k := range run.points {
		run.points[k].mu = s.Lift(in.points[k].mu)
	}
}

// liftDriver writes a kinematic participant's driver velocities over s,
// unchanged by the event.
func liftDriver(s *proof.SharedDenom, read *sharedBody, drive map[int]driverMotion) {
	if !read.kinematic {
		return
	}
	motion := drive[read.index]
	read.v, read.w = liftRats(s, motion.linear), liftRats(s, motion.angular)
	read.vPost, read.wPost = read.v, read.w
}

func liftRats(s *proof.SharedDenom, x [3]*big.Rat) [3]proof.SRat {
	x0, x1, x2 := x[0], x[1], x[2]
	return [3]proof.SRat{s.Lift(x0), s.Lift(x1), s.Lift(x2)}
}

// readBody reads one island participant's float-sourced readings at its
// event pose; liftDriver and liftIsland write the rest. A nil vPost reads
// only what the pre-solve relative velocity needs: the role, the pre-solve
// velocities (the post-solve ones set equal to them) and the center. That
// reading fails exactly when the full one fails on the same entry with
// unchanged post-solve velocities, since everything the full reading adds
// derives from what it checks: a dynamic body needs an exact mass reading
// with a positive mass interval, inertia intervals with a positive certified
// floor and a row ceiling, a finite pose basis, a mass center reading and
// exact velocities.
func (w *World) readBody(s *proof.SharedDenom, read *sharedBody, index int, entry *BodyState,
	vPost, wPost *QuantityVec, drive map[int]driverMotion) bool {
	role := w.bodies[index].definition.Role
	*read = sharedBody{index: index, dynamic: role == Dynamic}
	if _, ok := drive[index]; ok && role == Kinematic {
		read.kinematic = true
	}
	if !read.dynamic {
		return true
	}
	exact := w.bodies[index].exact
	if exact.mass == nil || exact.bound == nil || exact.bound.Sign() < 0 || exact.interval.Lo.Sign() <= 0 ||
		!exact.tensorOK || exact.floor == nil || exact.floor.Sign() <= 0 || exact.rowCeiling == nil {
		return false
	}
	basis := entry.Pose.Basis()
	ex, okX := sharedVec(basis.EX)
	ey, okY := sharedVec(basis.EY)
	ez, okZ := sharedVec(basis.EZ)
	if !okX || !okY || !okZ || !readCenter(s, &read.center, entry.Pose, exact) {
		return false
	}
	var ok [2]bool
	read.v, ok[0] = sharedQuantity(s, &entry.LinearVelocity)
	read.w, ok[1] = sharedQuantity(s, &entry.AngularVelocity)
	if !ok[0] || !ok[1] {
		return false
	}
	if vPost == nil {
		read.vPost, read.wPost = read.v, read.w
		return true
	}
	read.vPost, ok[0] = sharedQuantity(s, vPost)
	read.wPost, ok[1] = sharedQuantity(s, wPost)
	if !ok[0] || !ok[1] {
		return false
	}
	read.mass, read.inertia = exact.shared.interval, exact.shared.tensor
	read.rotation = [3][3]proof.SRat{
		{ex[0], ey[0], ez[0]},
		{ex[1], ey[1], ez[1]},
		{ex[2], ey[2], ez[2]},
	}
	read.defect = basisDefect(s, &read.rotation, exact.shared.largest)
	c0, c1, c2 := read.center[0], read.center[1], read.center[2]
	read.centerL1 = s.Add(s.Add(s.Magnitude(c0), s.Magnitude(c1)), s.Magnitude(c2))
	read.spinUpper, read.spinOK = spinEnergyChange(s, exact, &read.rotation, &read.w, &read.wPost)
	return true
}

// basisDefect is §8.1's orthonormality-defect widening of every world inertia
// component, 3·d·(2+d)·m, with d the entrywise absolute sum of RᵀR − I and m
// the local tensor's largest magnitude.
func basisDefect(s *proof.SharedDenom, rotation *[3][3]proof.SRat, largest proof.SRat) proof.SRat {
	var d proof.SRat
	for i := range 3 {
		for j := range 3 {
			var entry proof.SRat
			for k := range 3 {
				ki, kj := rotation[k][i], rotation[k][j]
				entry = s.Add(entry, s.Mul(ki, kj))
			}
			if i == j {
				entry = s.Sub(entry, proof.SInt(1))
			}
			d = s.Add(d, proof.SAbs(entry))
		}
	}
	defect := s.Mul(s.Mul(proof.SInt(3), d), s.Add(proof.SInt(2), d))
	return s.Mul(defect, largest)
}

// readCenter is worldCenterReading over D^0: it uses r3 for the point
// transform, then encloses both its floating-point evaluation and the
// source mass center's ball uncertainty, and writes the enclosure to center.
func readCenter(s *proof.SharedDenom, center *proof.SIVec3, pose r3.Transform, exact *exactMass) bool {
	if exact.radius == nil || exact.radius.Sign() < 0 {
		return false
	}
	for _, term := range exact.local {
		if term == nil {
			return false
		}
	}
	world, translation, basis := pose.Apply(exact.center), pose.Translation(), pose.Basis()
	shared := &exact.shared
	l0, l1, l2 := shared.local[0], shared.local[1], shared.local[2]
	axis := func(output, coordinate, b0, b1, b2 float64) (proof.SInterval, bool) {
		nominal, okN := proof.SFloat(output)
		value, okV := proof.SFloat(coordinate)
		f0, ok0 := proof.SFloat(b0)
		f1, ok1 := proof.SFloat(b1)
		f2, ok2 := proof.SFloat(b2)
		if !okN || !okV || !ok0 || !ok1 || !ok2 {
			return proof.SInterval{}, false
		}
		value = s.Add(s.Add(s.Add(value, s.Mul(f0, l0)), s.Mul(f1, l1)), s.Mul(f2, l2))
		bound := proof.SAbs(s.Sub(value, nominal))
		if shared.radius.Sign() != 0 {
			rowSum := s.Add(s.Add(proof.SAbs(f0), proof.SAbs(f1)), proof.SAbs(f2))
			bound = s.Add(bound, s.Mul(shared.radius, rowSum))
		}
		return proof.SInterval{Lo: s.Sub(nominal, bound), Hi: s.Add(nominal, bound)}, true
	}
	x, okX := axis(world.X, translation.X, basis.EX.X, basis.EY.X, basis.EZ.X)
	y, okY := axis(world.Y, translation.Y, basis.EX.Y, basis.EY.Y, basis.EZ.Y)
	z, okZ := axis(world.Z, translation.Z, basis.EX.Z, basis.EY.Z, basis.EZ.Z)
	*center = proof.SIVec3{x, y, z}
	return okX && okY && okZ
}

// readPoint reads one manifold point between body slots a and b. The normal
// ball is the published normal bound plus its angle; each witness ball is
// its own bound.
func readPoint(s *proof.SharedDenom, read *sharedPoint, a, b int, point *decad.ContactPoint,
	bodies []sharedBody) bool {
	normalBound, okBound := sharedBase(s, point.Normal.Bound)
	angle, okAngle := sharedBase(s, point.NormalAngle)
	ballA, okBallA := sharedBase(s, point.OnA.Bound)
	ballB, okBallB := sharedBase(s, point.OnB.Bound)
	if !okBound || !okAngle || !okBallA || !okBallB {
		return false
	}
	normal, okN := sharedBall(s, point.Normal.Value, s.Add(normalBound, angle))
	onA, okA := sharedBall(s, point.OnA.Value, ballA)
	onB, okB := sharedBall(s, point.OnB.Value, ballB)
	if !okN || !okA || !okB {
		return false
	}
	*read = sharedPoint{a: a, b: b, normal: normal, onA: onA, onB: onB, ballA: ballA, ballB: ballB}
	if body := &bodies[a]; body.dynamic || body.kinematic {
		read.rA = s.SubI3(onA, body.center)
	}
	if body := &bodies[b]; body.dynamic || body.kinematic {
		read.rB = s.SubI3(onB, body.center)
	}
	return true
}

// sharedBall encloses every point within radius of v in each coordinate.
func sharedBall(s *proof.SharedDenom, v r3.Vec, radius proof.SRat) (proof.SIVec3, bool) {
	center, ok := sharedVec(v)
	if !ok || radius.Sign() < 0 {
		return proof.SIVec3{}, false
	}
	c0, c1, c2 := center[0], center[1], center[2]
	return proof.SIVec3{
		{Lo: s.Sub(c0, radius), Hi: s.Add(c0, radius)},
		{Lo: s.Sub(c1, radius), Hi: s.Add(c1, radius)},
		{Lo: s.Sub(c2, radius), Hi: s.Add(c2, radius)},
	}, true
}

// sharedBase is exactBase over D^0: the exact rational of a value's
// magnitude times its unit's factor, false exactly where exactBase is nil.
func sharedBase(s *proof.SharedDenom, value units.Value) (proof.SRat, bool) {
	mag, ok := proof.SFloat(value.Mag())
	if !ok {
		return proof.SRat{}, false
	}
	factor := value.Unit().Factor()
	if factor == 1 {
		return mag, true
	}
	exact, ok := proof.SFloat(factor)
	if !ok {
		return proof.SRat{}, false
	}
	return s.Mul(mag, exact), true
}

// sharedVec is ratVec over D^0.
func sharedVec(v r3.Vec) ([3]proof.SRat, bool) {
	x, okX := proof.SFloat(v.X)
	y, okY := proof.SFloat(v.Y)
	z, okZ := proof.SFloat(v.Z)
	return [3]proof.SRat{x, y, z}, okX && okY && okZ
}

// sharedQuantity is quantityRats over D^0.
func sharedQuantity(s *proof.SharedDenom, q *QuantityVec) ([3]proof.SRat, bool) {
	x, okX := sharedBase(s, q.X)
	y, okY := sharedBase(s, q.Y)
	z, okZ := sharedBase(s, q.Z)
	return [3]proof.SRat{x, y, z}, okX && okY && okZ
}

// spinEnergyChange keeps each source inertia interval shared across the
// before/after squared-speed difference of one impulse. It returns the upper
// end of ωᵀIω's change, twice the rotational energy change. rotation is the
// pose basis, rotation[row][column]; before and after are exact angular
// velocities over D^0, each component read at its nearest float64 as a rad/s
// quantity would hold it. It fails when a component does not convert with a
// nonnegative bound, or a velocity component's float is not finite.
func spinEnergyChange(s *proof.SharedDenom, exact *exactMass, rotation *[3][3]proof.SRat,
	before, after *[3]proof.SRat) (proof.SRat, bool) {
	if !exact.tensorOK {
		return proof.SRat{}, false
	}
	mass := &exact.shared
	i0, i1, i2, ok := localSpin(s, rotation, before)
	if !ok {
		return proof.SRat{}, false
	}
	f0, f1, f2, ok := localSpin(s, rotation, after)
	if !ok {
		return proof.SRat{}, false
	}
	pick := func(i int, x0, x1, x2 proof.SRat) proof.SRat {
		switch i {
		case 0:
			return x0
		case 1:
			return x1
		default:
			return x2
		}
	}
	var upper proof.SRat
	for k := range mass.components {
		component := &mass.components[k]
		coefficient := s.Sub(s.Mul(pick(component.i, f0, f1, f2), pick(component.j, f0, f1, f2)),
			s.Mul(pick(component.i, i0, i1, i2), pick(component.j, i0, i1, i2)))
		if component.i != component.j {
			coefficient = proof.SShift(coefficient, 1)
		}
		contribution := s.Mul(component.value, coefficient)
		upper = s.Add(upper, s.Add(contribution, s.Mul(component.bound, proof.SAbs(coefficient))))
	}
	return upper, true
}

// localSpin is Rᵀω for an exact basis and an angular velocity read through
// float64, as a rad/s QuantityVec of its components would hold it: each
// component is rounded to its nearest float64 and fails when that is not
// finite.
func localSpin(s *proof.SharedDenom, rotation *[3][3]proof.SRat,
	omega *[3]proof.SRat) (proof.SRat, proof.SRat, proof.SRat, bool) {
	round := func(x proof.SRat) (proof.SRat, bool) {
		f, _ := s.Float64(x)
		return proof.SFloat(f)
	}
	v0, ok0 := round(omega[0])
	v1, ok1 := round(omega[1])
	v2, ok2 := round(omega[2])
	if !ok0 || !ok1 || !ok2 {
		return proof.SRat{}, proof.SRat{}, proof.SRat{}, false
	}
	column := func(i int) proof.SRat {
		var sum proof.SRat
		if v0.Sign() != 0 {
			sum = s.Add(sum, s.Mul(rotation[0][i], v0))
		}
		if v1.Sign() != 0 {
			sum = s.Add(sum, s.Mul(rotation[1][i], v1))
		}
		if v2.Sign() != 0 {
			sum = s.Add(sum, s.Mul(rotation[2][i], v2))
		}
		return sum
	}
	return column(0), column(1), column(2), true
}

// readPair reads a gathered pair's two participants into bodies, a in slot
// 0 and b in slot 1, as the certificate reads them before the solve: only
// their pre-solve velocities and centers, over the returned SharedDenom. ok
// is false when either does not read.
func (w *World) readPair(bodies *[2]sharedBody, pair islandPair, state State,
	drive map[int]driverMotion) (*proof.SharedDenom, bool) {
	s := proof.NewSharedDenom(nil)
	okA := w.readBody(s, &bodies[0], pair.a, &state.entries[pair.a], nil, nil, drive)
	okB := w.readBody(s, &bodies[1], pair.b, &state.entries[pair.b], nil, nil, drive)
	if !okA || !okB {
		return nil, false
	}
	for {
		liftDriver(s, &bodies[0], drive)
		liftDriver(s, &bodies[1], drive)
		next, widen := s.Widen()
		if !widen {
			return s, true
		}
		s = next
	}
}
