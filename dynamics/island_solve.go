package dynamics

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the frictionless proposal of docs/multibody-dynamics-design.md
// §6.2: projected Gauss–Seidel in float64, in the fixed order pairs
// (canonical) then manifold points (manifold order), for at most
// MaxIterations sweeps. The proposal proves nothing; island_certify.go
// certifies what it publishes.

// nominalBody is one island participant's float64 response data. A Fixed
// participant has zero inverse mass and inertia and zero velocity.
type nominalBody struct {
	dynamic    bool
	invMass    float64
	invInertia r3.SymmetricTensor // world axes at the event pose
	center     r3.Vec
	v, w       r3.Vec
}

// nominalPoint is one constraint's float64 data: levers from each world
// mass center, the A-to-B normal, the effective mass K_nn and the target
// post-solve normal speed.
type nominalPoint struct {
	a, b   int
	rA, rB r3.Vec
	n      r3.Vec
	k      float64
	target float64
}

// islandSolution is a certified proposal: each island body's published post
// velocities, each point's normal impulse, and the solver report.
type islandSolution struct {
	linear, angular []QuantityVec // island body slot order
	lambda          []float64     // island point order
	separating      []bool        // island pair order: every point leaves faster than VelocityResidual
	report          ContactSolverReport
}

// islandFailure is a refusal raised while solving an island.
type islandFailure struct {
	code   StepReason
	reason string
}

func vecOf(q QuantityVec) r3.Vec {
	return r3.Vec{X: q.X.Base(), Y: q.Y.Base(), Z: q.Z.Base()}
}

// nominalBodies reads each island participant's float64 inverse mass, world
// inverse inertia through r3, world mass center and pre-solve velocities.
func (w *World) nominalBodies(isl island, pre State) ([]nominalBody, *islandFailure) {
	out := make([]nominalBody, len(isl.bodies))
	for slot, index := range isl.bodies {
		if w.bodies[index].definition.Role != Dynamic {
			continue
		}
		entry, mass := pre.entries[index], w.bodies[index].mass
		inertia := mass.Inertia
		local, err := r3.NewSymmetricTensor(inertia.XX.Value.Base(), inertia.YY.Value.Base(),
			inertia.ZZ.Value.Base(), inertia.XY.Value.Base(), inertia.XZ.Value.Base(), inertia.YZ.Value.Base())
		if err == nil {
			local, err = local.Rotate(entry.Pose)
		}
		if err == nil {
			local, err = local.Inverse()
		}
		invMass := 1 / mass.Mass.Value.Base()
		if err != nil || !finite(invMass) || invMass <= 0 {
			return nil, &islandFailure{StepIslandDegenerate, fmt.Sprintf("body %d has no finite inverse mass or inertia", index)}
		}
		out[slot] = nominalBody{dynamic: true, invMass: invMass, invInertia: local,
			center: entry.Pose.Apply(mass.Center.Value),
			v:      vecOf(entry.LinearVelocity), w: vecOf(entry.AngularVelocity)}
	}
	return out, nil
}

func (b nominalBody) pointVelocity(r r3.Vec) r3.Vec {
	if !b.dynamic {
		return r3.Vec{}
	}
	return b.v.Add(b.w.Cross(r))
}

// apply adds the impulse j at lever r to a dynamic body.
func (b *nominalBody) apply(j, r r3.Vec) {
	if !b.dynamic {
		return
	}
	b.v = b.v.Add(j.Scale(b.invMass))
	b.w = b.w.Add(b.invInertia.Apply(r.Cross(j)))
}

func (b nominalBody) angularMass(r, n r3.Vec) float64 {
	if !b.dynamic {
		return 0
	}
	arm := r.Cross(n)
	return arm.Dot(b.invInertia.Apply(arm))
}

// nominalPoints builds each constraint's levers, K_nn and target.
func (w *World) nominalPoints(isl island, slots map[int]int, bodies []nominalBody) ([]nominalPoint, *islandFailure) {
	var out []nominalPoint
	impact := w.step.ImpactSpeed.Base()
	for _, pair := range isl.pairs {
		restitution := w.pairs[pair.key].restitution.Base()
		a, b := slots[pair.a], slots[pair.b]
		for _, point := range pair.manifold.Points {
			p := nominalPoint{a: a, b: b, n: point.Normal.Value}
			if bodies[a].dynamic {
				p.rA = point.OnA.Value.Sub(bodies[a].center)
			}
			if bodies[b].dynamic {
				p.rB = point.OnB.Value.Sub(bodies[b].center)
			}
			p.k = bodies[a].invMass + bodies[b].invMass + bodies[a].angularMass(p.rA, p.n) +
				bodies[b].angularMass(p.rB, p.n)
			speed := bodies[b].pointVelocity(p.rB).Sub(bodies[a].pointVelocity(p.rA)).Dot(p.n)
			if speed < -impact {
				p.target = -restitution * speed
			}
			if !finite(p.k, p.target, speed) || p.k <= 0 {
				return nil, &islandFailure{StepIslandDegenerate, "constraint effective mass is not finite and positive"}
			}
			out = append(out, p)
		}
	}
	return out, nil
}

// solveIsland proposes impulses by projected Gauss–Seidel until a sweep
// changes no impulse in float64, or MaxIterations sweeps have run, and then
// certifies the published proposal once. Running to that fixed point lets a
// resting island publish exactly zero velocities, which its continuation
// sweeps need to prove persistent touch. A certificate that fails refuses
// the island with the failing gate and its limit.
func (w *World) solveIsland(isl island, pre State) (islandSolution, *islandFailure) {
	slots := make(map[int]int, len(isl.bodies))
	for slot, index := range isl.bodies {
		slots[index] = slot
	}
	bodies, failure := w.nominalBodies(isl, pre)
	if failure != nil {
		return islandSolution{}, failure
	}
	points, failure := w.nominalPoints(isl, slots, bodies)
	if failure != nil {
		return islandSolution{}, failure
	}
	current := append([]nominalBody(nil), bodies...)
	lambda := make([]float64, len(points))
	for sweep := 1; sweep <= w.step.MaxIterations; sweep++ {
		largest := 0.0
		for k, p := range points {
			speed := current[p.b].pointVelocity(p.rB).Sub(current[p.a].pointVelocity(p.rA)).Dot(p.n)
			next := math.Max(0, lambda[k]+(p.target-speed)/p.k)
			delta := next - lambda[k]
			lambda[k] = next
			j := p.n.Scale(delta)
			current[p.a].apply(j.Scale(-1), p.rA)
			current[p.b].apply(j, p.rB)
			largest = math.Max(largest, math.Abs(delta))
		}
		if !finite(largest) {
			return islandSolution{}, &islandFailure{StepIslandDegenerate, "island proposal is not finite"}
		}
		if largest != 0 && sweep < w.step.MaxIterations {
			continue
		}
		solution, cert, failure := w.publishIsland(isl, pre, bodies, points, lambda)
		if failure != nil {
			return islandSolution{}, failure
		}
		if cert.failed == 0 {
			report, err := solverReport(cert, islandPenetration(isl), sweep)
			if err != nil {
				return islandSolution{}, &islandFailure{StepIslandResidual, err.Error()}
			}
			solution.report = report
			return solution, nil
		}
		if sweep == w.step.MaxIterations {
			value, _ := cert.value.Float64()
			limit, _ := cert.limit.Float64()
			return islandSolution{}, &islandFailure{StepIslandResidual,
				fmt.Sprintf("%v gate reaches %g, beyond its limit %g, after %d sweeps",
					cert.failed, value, limit, sweep)}
		}
	}
	return islandSolution{}, &islandFailure{StepIslandResidual, "island solver ran no sweep"}
}

// publishIsland rounds a proposal to its published values and certifies it.
// Post velocities are recomputed from the pre-solve velocities and the
// final impulses in the fixed point order. A post component within 1/16 of
// its residual of zero is published as exactly zero, so a body the solve
// brings to rest drifts as a rest or a pure translation; the certificate
// then judges the published values, never the unrounded ones.
func (w *World) publishIsland(isl island, pre State, bodies []nominalBody, points []nominalPoint,
	lambda []float64) (islandSolution, islandCertificate, *islandFailure) {
	post := append([]nominalBody(nil), bodies...)
	for k, p := range points {
		j := p.n.Scale(lambda[k])
		post[p.a].apply(j.Scale(-1), p.rA)
		post[p.b].apply(j, p.rB)
	}
	linearSnap := w.step.VelocityResidual.Base() / 16
	angularSnap := w.step.AngularVelocityResidual.Base() / 16
	snap := func(v r3.Vec, limit float64) r3.Vec {
		for _, c := range []*float64{&v.X, &v.Y, &v.Z} {
			if math.Abs(*c) <= limit {
				*c = 0
			}
		}
		return v
	}
	solution := islandSolution{linear: make([]QuantityVec, len(bodies)), angular: make([]QuantityVec, len(bodies)),
		lambda: append([]float64(nil), lambda...)}
	for slot, index := range isl.bodies {
		entry := pre.entries[index]
		solution.linear[slot], solution.angular[slot] = entry.LinearVelocity, entry.AngularVelocity
		if !post[slot].dynamic {
			continue
		}
		v, omega := snap(post[slot].v, linearSnap), snap(post[slot].w, angularSnap)
		if !finite(v.X, v.Y, v.Z, omega.X, omega.Y, omega.Z) {
			return islandSolution{}, islandCertificate{}, &islandFailure{StepIslandDegenerate,
				"island proposal velocity is not finite"}
		}
		solution.linear[slot] = QuantityVec{X: units.MillimetersPerSecond(v.X),
			Y: units.MillimetersPerSecond(v.Y), Z: units.MillimetersPerSecond(v.Z)}
		solution.angular[slot] = QuantityVec{X: units.RadiansPerSecond(omega.X),
			Y: units.RadiansPerSecond(omega.Y), Z: units.RadiansPerSecond(omega.Z)}
	}
	solution.separating = make([]bool, len(isl.pairs))
	k := 0
	for pairIndex, pair := range isl.pairs {
		solution.separating[pairIndex] = true
		for range pair.manifold.Points {
			published := func(slot int, r r3.Vec) r3.Vec {
				if !post[slot].dynamic {
					return r3.Vec{}
				}
				return vecOf(solution.linear[slot]).Add(vecOf(solution.angular[slot]).Cross(r))
			}
			speed := published(points[k].b, points[k].rB).Sub(published(points[k].a, points[k].rA)).Dot(points[k].n)
			if !(speed > w.step.VelocityResidual.Base()) {
				solution.separating[pairIndex] = false
			}
			k++
		}
	}
	cert, failure := w.certifyProposal(isl, pre, points, solution)
	return solution, cert, failure
}

// certifyProposal reads a published proposal and its inputs as exact
// intervals and runs the certificate over them.
func (w *World) certifyProposal(isl island, pre State, points []nominalPoint,
	solution islandSolution) (islandCertificate, *islandFailure) {
	certBodies := make([]certBody, len(isl.bodies))
	for slot, index := range isl.bodies {
		after := pre.entries[index]
		after.LinearVelocity, after.AngularVelocity = solution.linear[slot], solution.angular[slot]
		body, ok := w.newCertBody(index, pre.entries[index], after)
		if !ok {
			return islandCertificate{}, &islandFailure{StepIslandDegenerate,
				fmt.Sprintf("body %d cannot be read as exact intervals", index)}
		}
		certBodies[slot] = body
	}
	certPoints := make([]certPoint, 0, len(points))
	k := 0
	for pairIndex, pair := range isl.pairs {
		restitution := exactBase(w.pairs[pair.key].restitution)
		for _, point := range pair.manifold.Points {
			p, ok := newCertPoint(pairIndex, points[k].a, points[k].b, point, certBodies, restitution)
			if !ok || restitution == nil {
				return islandCertificate{}, &islandFailure{StepIslandDegenerate,
					"manifold point cannot be read as exact intervals"}
			}
			p.lambda = ratFloat(solution.lambda[k])
			if p.lambda == nil {
				return islandCertificate{}, &islandFailure{StepIslandDegenerate, "island impulse is not finite"}
			}
			certPoints = append(certPoints, p)
			k++
		}
	}
	return w.certifyIsland(certBodies, certPoints), nil
}

// islandPenetration is the deepest penetration the island's manifolds
// report, before correction.
func islandPenetration(isl island) *big.Rat {
	deepest := new(big.Rat)
	for _, pair := range isl.pairs {
		for _, point := range pair.manifold.Points {
			depth := ratFloat(-point.Separation.Value.Base())
			if depth != nil && depth.Cmp(deepest) > 0 {
				deepest = depth
			}
		}
	}
	return deepest
}
