package dynamics

import (
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the proposal of docs/multibody-dynamics-design.md §6.2:
// projected Gauss–Seidel in float64, in the fixed order pairs (canonical) then
// manifold points (manifold order), for at most MaxIterations sweeps. Each
// point takes its normal row and then, for a positive-friction pair, its
// tangent row projected onto the Coulomb disk. The proposal proves nothing;
// island_certify.go certifies what it publishes.

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
// post-solve normal speed. A positive-friction point also carries the
// deterministic tangent basis, the pair's nominal coefficient and the
// tangent row's step bound: max(K_t1t1, K_t2t2) + |K_t1t2|, which is at
// least the largest eigenvalue of the 2×2 K_tt.
type nominalPoint struct {
	a, b   int
	rA, rB r3.Vec
	n      r3.Vec
	k      float64
	target float64
	mu     float64
	t1, t2 r3.Vec
	kt     float64
}

// islandSolution is a certified proposal: each island body's published post
// velocities, each point's normal impulse and world tangent impulse, and the
// solver report.
type islandSolution struct {
	linear, angular []QuantityVec // island body slot order
	lambda          []float64     // island point order
	tangent         []r3.Vec      // island point order, on B; A receives its negation
	separating      []bool        // island pair order: every point leaves faster than VelocityResidual
	report          ContactSolverReport
}

// islandFailure is a refusal raised while solving an island, with the
// limit the failing gate exceeded when one did.
type islandFailure struct {
	code   StepReason
	reason string
	limit  units.Value
}

func vecOf(q QuantityVec) r3.Vec {
	return r3.Vec{X: q.X.Base(), Y: q.Y.Base(), Z: q.Z.Base()}
}

// nominalBodies reads each island participant's float64 inverse mass, world
// inverse inertia through r3, world mass center and pre-solve velocities. A
// translating kinematic participant moves at its driver velocity.
func (w *World) nominalBodies(isl island, pre State, drive map[int][3]*big.Rat) ([]nominalBody, *islandFailure) {
	out := make([]nominalBody, len(isl.bodies))
	for slot, index := range isl.bodies {
		if v, ok := drive[index]; ok {
			out[slot].v = vecOf(ratVelocity(v))
			continue
		}
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
			return nil, &islandFailure{code: StepIslandDegenerate, reason: fmt.Sprintf("body %d has no finite inverse mass or inertia", index)}
		}
		out[slot] = nominalBody{dynamic: true, invMass: invMass, invInertia: local,
			center: entry.Pose.Apply(mass.Center.Value),
			v:      vecOf(entry.LinearVelocity), w: vecOf(entry.AngularVelocity)}
	}
	return out, nil
}

// pointVelocity is v + ω×r for a dynamic body, the driver velocity of a
// kinematic one and zero for a fixed one.
func (b nominalBody) pointVelocity(r r3.Vec) r3.Vec {
	if !b.dynamic {
		return b.v
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

// tangentBasis is rigid-dynamics "Response"'s deterministic tangent basis:
// the world axis least aligned with n (the lowest index on a tie) crossed
// with n gives t1, and n×t1 gives t2.
func tangentBasis(n r3.Vec) (r3.Vec, r3.Vec, bool) {
	axes := [3]r3.Vec{{X: 1}, {Y: 1}, {Z: 1}}
	components := [3]float64{math.Abs(n.X), math.Abs(n.Y), math.Abs(n.Z)}
	least := 0
	for axis := 1; axis < 3; axis++ {
		if components[axis] < components[least] {
			least = axis
		}
	}
	t1, ok := n.Cross(axes[least]).Normalize()
	if !ok {
		return r3.Vec{}, r3.Vec{}, false
	}
	t2, ok := n.Cross(t1).Normalize()
	return t1, t2, ok
}

// tangentMass is (r×s)·I⁻¹(r×t) for a dynamic body and zero otherwise.
func (b nominalBody) tangentMass(r, s, t r3.Vec) float64 {
	if !b.dynamic {
		return 0
	}
	return r.Cross(s).Dot(b.invInertia.Apply(r.Cross(t)))
}

// nominalPoints builds each constraint's levers, K_nn and target, and for a
// positive-friction pair its tangent basis and tangent step bound.
func (w *World) nominalPoints(isl island, slots map[int]int, bodies []nominalBody) ([]nominalPoint, *islandFailure) {
	var out []nominalPoint
	impact := w.step.ImpactSpeed.Base()
	for _, pair := range isl.pairs {
		restitution := w.pairs[pair.key].restitution.Base()
		mu := w.pairs[pair.key].friction.nominal.Base()
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
				return nil, &islandFailure{code: StepIslandDegenerate, reason: "constraint effective mass is not finite and positive"}
			}
			if mu > 0 {
				t1, t2, ok := tangentBasis(p.n)
				if !ok {
					return nil, &islandFailure{code: StepIslandDegenerate, reason: "constraint normal has no tangent basis"}
				}
				p.mu, p.t1, p.t2 = mu, t1, t2
				inverse := bodies[a].invMass + bodies[b].invMass
				mass := func(s, t r3.Vec) float64 {
					return bodies[a].tangentMass(p.rA, s, t) + bodies[b].tangentMass(p.rB, s, t)
				}
				k11, k22, k12 := inverse+mass(t1, t1), inverse+mass(t2, t2), mass(t1, t2)
				p.kt = math.Max(k11, k22) + math.Abs(k12)
				if !finite(mu, p.kt) || p.kt <= 0 {
					return nil, &islandFailure{code: StepIslandDegenerate, reason: "constraint tangent mass is not finite and positive"}
				}
			}
			out = append(out, p)
		}
	}
	return out, nil
}

// solveIsland proposes impulses by projected Gauss–Seidel, started from the
// direct frictionless solution when the island is small enough, until a
// sweep changes no impulse in float64, or MaxIterations sweeps have run, and then
// certifies the published proposal once. Running to that fixed point lets a
// resting island publish exactly zero velocities, which its continuation
// sweeps need to prove persistent touch. A certificate that fails refuses
// the island with the failing gate and its limit.
//
// An island whose problem equals one in the input state's cache restarts
// from that island's final impulses and velocities (§6.2's warm start). When
// the cold solve ended at a fixed point, the restart's first sweep is that
// solve's last one, so it changes no impulse and the solve publishes the
// cold solve's proposal after one sweep. When the cold solve ran to
// MaxIterations instead, its proposal is published again as it stands, with
// its sweep count, since further sweeps would move it. Either way the
// certificate judges the proposal afresh. Every certified island is kept for
// the next step's cache.
func (w *World) solveIsland(isl island, pre State, drive map[int][3]*big.Rat,
	work *stepWork) (islandSolution, *islandFailure) {
	slots := make(map[int]int, len(isl.bodies))
	for slot, index := range isl.bodies {
		slots[index] = slot
	}
	bodies, failure := w.nominalBodies(isl, pre, drive)
	if failure != nil {
		return islandSolution{}, failure
	}
	points, failure := w.nominalPoints(isl, slots, bodies)
	if failure != nil {
		return islandSolution{}, failure
	}
	current := append([]nominalBody(nil), bodies...)
	lambda := make([]float64, len(points))
	tangent := make([][2]float64, len(points))
	// The tangent rows join once a sweep of the normal rows alone changes no
	// impulse: friction then starts from the frictionless contact state, whose
	// patch neither spins nor slides where the slide was stopped, so a
	// sticking patch never turns the transient spin of the first normal
	// sweeps into self-cancelling corner friction. An island with no
	// positive-friction point has no tangent rows to hold back.
	tangentRows := !slices.ContainsFunc(points, func(p nominalPoint) bool { return p.mu > 0 })
	warm := work.warmStart(isl, pre, drive)
	if warm != nil && !warm.fixed {
		// A cold solve that ran to MaxIterations is republished as it stands.
		solution, cert, failure := w.publishIsland(isl, pre, bodies, points, warm.lambda, warm.tangent, drive)
		if failure == nil && cert.failed == 0 {
			if report, err := solverReport(cert, islandPenetration(isl), warm.sweeps); err == nil {
				solution.report = report
				work.keep(warm)
				return solution, nil
			}
		}
		// The cached proposal no longer certifies: solve cold.
		warm = nil
	}
	if warm != nil {
		copy(current, warm.current)
		copy(lambda, warm.lambda)
		copy(tangent, warm.tangent)
		tangentRows = true
	} else if start, held, ok := w.directStart(points, bodies); ok {
		// A small island starts from its direct solution (island_direct.go):
		// the sticking solution when one exists, which the tangent rows
		// refine, or else the frictionless state the delay would reach.
		for k, p := range points {
			lambda[k], tangent[k] = start[k], held[k]
			j := p.n.Scale(start[k]).Add(p.t1.Scale(held[k][0])).Add(p.t2.Scale(held[k][1]))
			current[p.a].apply(j.Scale(-1), p.rA)
			current[p.b].apply(j, p.rB)
		}
		tangentRows = true
	}
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
			if p.mu == 0 || !tangentRows {
				continue
			}
			change := tangentRow(p, current, lambda[k], &tangent[k])
			largest = math.Max(largest, math.Max(math.Abs(change[0]), math.Abs(change[1])))
		}
		if !finite(largest) {
			return islandSolution{}, &islandFailure{code: StepIslandDegenerate, reason: "island proposal is not finite"}
		}
		if largest == 0 && !tangentRows && sweep < w.step.MaxIterations {
			tangentRows = true
			continue
		}
		if largest != 0 && sweep < w.step.MaxIterations {
			continue
		}
		solution, cert, failure := w.publishIsland(isl, pre, bodies, points, lambda, tangent, drive)
		if failure != nil {
			return islandSolution{}, failure
		}
		if cert.failed == 0 {
			report, err := solverReport(cert, islandPenetration(isl), sweep)
			if err != nil {
				return islandSolution{}, &islandFailure{code: StepIslandResidual, reason: err.Error()}
			}
			solution.report = report
			held := newWarmIsland(isl, pre, drive)
			held.lambda, held.tangent = slices.Clone(lambda), slices.Clone(tangent)
			held.current = slices.Clone(current)
			held.fixed, held.sweeps = largest == 0 && tangentRows, sweep
			work.keep(held)
			return solution, nil
		}
		if sweep == w.step.MaxIterations {
			value, _ := cert.value.Float64()
			limit, _ := cert.limit.Float64()
			return islandSolution{}, &islandFailure{code: StepIslandResidual,
				reason: fmt.Sprintf("%v gate reaches %g, beyond its limit %g, after %d sweeps",
					cert.failed, value, limit, sweep), limit: cert.failed.limitValue(cert.limit)}
		}
	}
	return islandSolution{}, &islandFailure{code: StepIslandResidual, reason: "island solver ran no sweep"}
}

// tangentRow is one point's friction row: the tangent impulse steps against
// the relative tangent velocity by the step bound, λt −= w_t / kt, and is
// projected onto the disk of radius μ·λn. The scalar step keeps a slipping
// point's impulse opposite its slip at the fixed point, as the slip gate
// requires; a step by K_tt⁻¹ would leave it opposite K_tt⁻¹·w_t instead.
// It applies the change to both bodies and returns it.
func tangentRow(p nominalPoint, current []nominalBody, normal float64, held *[2]float64) [2]float64 {
	w := current[p.b].pointVelocity(p.rB).Sub(current[p.a].pointVelocity(p.rA))
	next := [2]float64{held[0] - w.Dot(p.t1)/p.kt, held[1] - w.Dot(p.t2)/p.kt}
	radius := p.mu * normal
	if length := math.Hypot(next[0], next[1]); length > radius {
		if radius <= 0 {
			next = [2]float64{}
		} else {
			next = [2]float64{next[0] * radius / length, next[1] * radius / length}
		}
	}
	change := [2]float64{next[0] - held[0], next[1] - held[1]}
	*held = next
	j := p.t1.Scale(change[0]).Add(p.t2.Scale(change[1]))
	current[p.a].apply(j.Scale(-1), p.rA)
	current[p.b].apply(j, p.rB)
	return change
}

// publishIsland rounds a proposal to its published values and certifies it.
// Each point's world tangent impulse is t1·λt1 + t2·λt2, rounded once, with a
// component within 1/16 of ImpulseResidual of zero published as exactly zero,
// so a sticking patch whose corners exchange float-rounding friction
// publishes none. Post velocities are recomputed from the pre-solve
// velocities and the final normal and published tangent impulses in the
// fixed point order. A post component within 1/16 of its residual of zero is
// published as exactly zero (a spin component also only within 1/16 of
// VelocityResidual over the body's longest lever), so a body the solve brings to rest drifts as a
// rest or a pure translation, and co-moving dynamic bodies publish one common
// velocity (commonVelocities); the certificate then judges the published
// values, never the unrounded ones.
func (w *World) publishIsland(isl island, pre State, bodies []nominalBody, points []nominalPoint,
	lambda []float64, tangent [][2]float64, drive map[int][3]*big.Rat) (islandSolution, islandCertificate, *islandFailure) {
	snap := func(v r3.Vec, limit float64) r3.Vec {
		for _, c := range []*float64{&v.X, &v.Y, &v.Z} {
			if math.Abs(*c) <= limit {
				*c = 0
			}
		}
		return v
	}
	post := append([]nominalBody(nil), bodies...)
	tangents := make([]r3.Vec, len(points))
	for k, p := range points {
		j := p.n.Scale(lambda[k])
		if p.mu > 0 {
			tangents[k] = snap(p.t1.Scale(tangent[k][0]).Add(p.t2.Scale(tangent[k][1])), w.step.ImpulseResidual.Base()/16)
			j = j.Add(tangents[k])
		}
		post[p.a].apply(j.Scale(-1), p.rA)
		post[p.b].apply(j, p.rB)
	}
	linearSnap := w.step.VelocityResidual.Base() / 16
	// A spin component snaps only while the change it makes at the body's
	// longest island lever stays within the linear snap, so a rolling body
	// under a coarse AngularVelocityResidual keeps the spin its contacts need.
	angularSnap := make([]float64, len(bodies))
	for slot := range angularSnap {
		angularSnap[slot] = w.step.AngularVelocityResidual.Base() / 16
	}
	for _, p := range points {
		for _, end := range [2]struct {
			slot  int
			lever r3.Vec
		}{{p.a, p.rA}, {p.b, p.rB}} {
			if length := end.lever.Len(); length > 0 {
				angularSnap[end.slot] = math.Min(angularSnap[end.slot], linearSnap/length)
			}
		}
	}
	solution := islandSolution{linear: make([]QuantityVec, len(bodies)), angular: make([]QuantityVec, len(bodies)),
		lambda: append([]float64(nil), lambda...), tangent: tangents}
	for slot, index := range isl.bodies {
		entry := pre.entries[index]
		solution.linear[slot], solution.angular[slot] = entry.LinearVelocity, entry.AngularVelocity
		if !post[slot].dynamic {
			continue
		}
		v, omega := snap(post[slot].v, linearSnap), snap(post[slot].w, angularSnap[slot])
		if !finite(v.X, v.Y, v.Z, omega.X, omega.Y, omega.Z) {
			return islandSolution{}, islandCertificate{}, &islandFailure{code: StepIslandDegenerate, reason: "island proposal velocity is not finite"}
		}
		solution.linear[slot] = QuantityVec{X: units.MillimetersPerSecond(v.X),
			Y: units.MillimetersPerSecond(v.Y), Z: units.MillimetersPerSecond(v.Z)}
		solution.angular[slot] = QuantityVec{X: units.RadiansPerSecond(omega.X),
			Y: units.RadiansPerSecond(omega.Y), Z: units.RadiansPerSecond(omega.Z)}
	}
	w.commonVelocities(isl, solution)
	solution.separating = make([]bool, len(isl.pairs))
	k := 0
	for pairIndex, pair := range isl.pairs {
		solution.separating[pairIndex] = true
		for range pair.manifold.Points {
			published := func(slot int, r r3.Vec) r3.Vec {
				if !post[slot].dynamic {
					return post[slot].v
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
	cert, failure := w.certifyProposal(isl, pre, points, solution, drive)
	return solution, cert, failure
}

// commonVelocities publishes one velocity for each group of co-moving dynamic
// bodies of an island. Two dynamic bodies of an island pair are co-moving
// when their published spins are equal and every component of their
// published linear velocities differs by at most VelocityResidual/8; the
// groups are the connected components of that relation. A group whose every
// member lies within VelocityResidual/16 of the group's mass-weighted mean
// velocity, per component, publishes that mean for each member. Float
// rounding of the solve leaves a resting or bouncing stack's bodies about
// 1e-7 mm/s apart, and only exactly equal velocities let SweepPair prove the
// persistent touch that continues the stack. The rule is a published
// proposal, not a claim: the certificate judges the common velocities like
// any other, so each member's change, at most VelocityResidual/16 per
// component, must still pass the linear law within ImpulseResidual +
// m_hi·VelocityResidual.
func (w *World) commonVelocities(isl island, solution islandSolution) {
	limit := w.step.VelocityResidual.Base()
	slots := make(map[int]int, len(isl.bodies))
	parent := make([]int, len(isl.bodies))
	for slot, index := range isl.bodies {
		slots[index], parent[slot] = slot, slot
	}
	find := func(i int) int {
		for parent[i] != i {
			parent[i] = parent[parent[i]]
			i = parent[i]
		}
		return i
	}
	for _, pair := range isl.pairs {
		if w.bodies[pair.a].definition.Role != Dynamic || w.bodies[pair.b].definition.Role != Dynamic {
			continue
		}
		sa, sb := slots[pair.a], slots[pair.b]
		if solution.angular[sa] != solution.angular[sb] {
			continue
		}
		va, vb := vecOf(solution.linear[sa]), vecOf(solution.linear[sb])
		if math.Abs(va.X-vb.X) > limit/8 || math.Abs(va.Y-vb.Y) > limit/8 || math.Abs(va.Z-vb.Z) > limit/8 {
			continue
		}
		parent[find(sa)] = find(sb)
	}
	groups := map[int][]int{}
	for slot := range isl.bodies {
		if w.bodies[isl.bodies[slot]].definition.Role == Dynamic {
			root := find(slot)
			groups[root] = append(groups[root], slot)
		}
	}
	for _, members := range groups {
		if len(members) < 2 {
			continue
		}
		var momentum r3.Vec
		total := 0.0
		for _, slot := range members {
			m := w.bodies[isl.bodies[slot]].mass.Mass.Value.Base()
			momentum = momentum.Add(vecOf(solution.linear[slot]).Scale(m))
			total += m
		}
		mean := momentum.Scale(1 / total)
		if !finite(mean.X, mean.Y, mean.Z) {
			continue
		}
		within := true
		for _, slot := range members {
			v := vecOf(solution.linear[slot])
			if math.Abs(v.X-mean.X) > limit/16 || math.Abs(v.Y-mean.Y) > limit/16 || math.Abs(v.Z-mean.Z) > limit/16 {
				within = false
			}
		}
		if !within {
			continue
		}
		for _, slot := range members {
			solution.linear[slot] = QuantityVec{X: units.MillimetersPerSecond(mean.X),
				Y: units.MillimetersPerSecond(mean.Y), Z: units.MillimetersPerSecond(mean.Z)}
		}
	}
}

// certifyProposal reads a published proposal and its inputs as exact
// intervals and runs the certificate over them.
func (w *World) certifyProposal(isl island, pre State, points []nominalPoint,
	solution islandSolution, drive map[int][3]*big.Rat) (islandCertificate, *islandFailure) {
	certBodies := make([]certBody, len(isl.bodies))
	for slot, index := range isl.bodies {
		after := pre.entries[index]
		after.LinearVelocity, after.AngularVelocity = solution.linear[slot], solution.angular[slot]
		body, ok := w.newCertBody(index, pre.entries[index], after, drive)
		if !ok {
			return islandCertificate{}, &islandFailure{code: StepIslandDegenerate, reason: fmt.Sprintf("body %d cannot be read as exact intervals", index)}
		}
		certBodies[slot] = body
	}
	certPoints := make([]certPoint, 0, len(points))
	k := 0
	for pairIndex, pair := range isl.pairs {
		restitution := exactBase(w.pairs[pair.key].restitution)
		mu := w.pairs[pair.key].friction.lower
		for _, point := range pair.manifold.Points {
			p, ok := newCertPoint(pairIndex, points[k].a, points[k].b, point, certBodies, restitution)
			if !ok || restitution == nil || mu == nil {
				return islandCertificate{}, &islandFailure{code: StepIslandDegenerate, reason: "manifold point cannot be read as exact intervals"}
			}
			p.lambda, p.mu = ratFloat(solution.lambda[k]), mu
			tangent, okTangent := ratVec(solution.tangent[k])
			if p.lambda == nil || !okTangent {
				return islandCertificate{}, &islandFailure{code: StepIslandDegenerate, reason: "island impulse is not finite"}
			}
			p.tangent = tangent
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
