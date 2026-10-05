package dynamics

import (
	"context"
	"fmt"
	"maps"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file forms islands (docs/multibody-dynamics-design.md §6.1), corrects
// their positions (§6.6) and publishes their events for the step of a world
// of four or more bodies.

// IslandReport is one simultaneous solve: the bodies and pairs that shared
// constraints at one event time, and the certified residuals of that solve.
type IslandReport struct {
	Time   units.Value   // from the start of the step
	Bodies []*decad.Body // world order; Fixed and Kinematic participants included
	Pairs  []BodyPair    // world order
	Events []int         // indices into StepReport.Events
	Solver ContactSolverReport
}

// islandPair is one gathered pair at an event: its canonical key, its
// world-order bodies, the manifold its producer certified, and the sweep
// instant of that manifold. An interior impact keeps its bracket, whose
// travel widens the correction allowance, and the manifold its rounded event
// poses show, whose penetration the correction removes; a pair the contact
// set continued on a persistent track is marked track.
type islandPair struct {
	key      int
	a, b     int
	manifold decad.ContactManifold
	at       decad.SweepInstant
	bracket  *decad.SweepInterval
	depth    *decad.ContactManifold
	track    bool
}

// island is one connected component of the dynamic-body contact graph.
// Fixed and Kinematic participants attach to it without joining two islands.
type island struct {
	dynamic []int        // world order
	bodies  []int        // world order, Fixed and Kinematic participants included
	pairs   []islandPair // canonical order
}

// eventIslands is the input of one event time's solve: the state at the
// event, the gathered pairs, the exact velocity of every translating
// kinematic participant, and where the event sits in the step.
type eventIslands struct {
	pre        State
	at         units.Value // held time from the step start
	sliceStart units.Value
	sliceSpan  units.Value
	gathered   []islandPair
	drive      map[int][3]*big.Rat
	eventBase  int // events published earlier in the step
	islandBase int // islands solved earlier in the step
	work       *stepWork
}

// solvedEvent is the outcome of one event time's islands: the post-event
// state, the published events and island reports, and the continuation
// policy of every gathered pair.
type solvedEvent struct {
	post     State
	events   []ContactEvent
	islands  []IslandReport
	policies map[int]decad.SweepStartPolicy
}

// manifoldWithin reports whether every point of a gathered manifold is
// finite and within the step's contact request.
func (w *World) manifoldWithin(manifold *decad.ContactManifold) bool {
	if manifold == nil || len(manifold.Points) == 0 {
		return false
	}
	points, normals := w.step.Contact.PointResolution.Base(), w.step.Contact.NormalResolution.Base()
	for _, p := range manifold.Points {
		if !finite(p.OnA.Value.X, p.OnA.Value.Y, p.OnA.Value.Z, p.OnB.Value.X, p.OnB.Value.Y, p.OnB.Value.Z,
			p.Normal.Value.X, p.Normal.Value.Y, p.Normal.Value.Z, p.OnA.Bound.Base(), p.OnB.Bound.Base(),
			p.Normal.Bound.Base(), p.NormalAngle.Base(), p.Separation.Value.Base(), p.Separation.Bound.Base()) ||
			p.OnA.Bound.Base() < 0 || p.OnB.Bound.Base() < 0 || p.Normal.Bound.Base() < 0 ||
			p.NormalAngle.Base() < 0 || p.Separation.Bound.Base() < 0 ||
			p.OnA.Bound.Base() > points || p.OnB.Bound.Base() > points ||
			p.Normal.Bound.Base()+p.NormalAngle.Base() > normals {
			return false
		}
	}
	return true
}

// pairActive reports whether any point of a gathered pair may be closing:
// its enclosed relative normal speed reaches down to VelocityResidual or
// below. Only a pair whose every point certainly separates by more than that
// stays out of the solve.
func (w *World) pairActive(pair islandPair, state State, drive map[int][3]*big.Rat) (bool, bool) {
	a, okA := w.newCertBody(pair.a, state.entries[pair.a], state.entries[pair.a], drive)
	b, okB := w.newCertBody(pair.b, state.entries[pair.b], state.entries[pair.b], drive)
	if !okA || !okB {
		return false, false
	}
	bodies := []certBody{a, b}
	limit := exactBase(w.step.VelocityResidual)
	for _, point := range pair.manifold.Points {
		p, ok := newCertPoint(0, 0, 1, point, bodies, new(big.Rat))
		if !ok {
			return false, false
		}
		if preNormalSpeed(p, bodies).Lo.Cmp(limit) <= 0 {
			return true, true
		}
	}
	return false, true
}

// formIslands groups gathered pairs into the connected components of the
// graph whose vertices are dynamic bodies, numbered by their smallest world
// index. A pair with no dynamic participant cannot be solved.
func (w *World) formIslands(pairs []islandPair) ([]island, *StepDiagnostic) {
	parent := make([]int, len(w.bodies))
	for i := range parent {
		parent[i] = i
	}
	find := func(i int) int {
		for parent[i] != i {
			parent[i] = parent[parent[i]]
			i = parent[i]
		}
		return i
	}
	dynamic := func(i int) bool { return w.bodies[i].definition.Role == Dynamic }
	for _, pair := range pairs {
		switch {
		case !dynamic(pair.a) && !dynamic(pair.b):
			d := scheduleDiagnostic(StepIslandDegenerate, w.bodyPair(w.pairs[pair.key]),
				"a closing constraint has no dynamic participant")
			return nil, &d
		case dynamic(pair.a) && dynamic(pair.b):
			ra, rb := find(pair.a), find(pair.b)
			if ra < rb {
				parent[rb] = ra
			} else {
				parent[ra] = rb
			}
		}
	}
	byRoot := map[int]*island{}
	var roots []int
	for _, pair := range pairs {
		anchor := pair.a
		if !dynamic(anchor) {
			anchor = pair.b
		}
		root := find(anchor)
		isl, ok := byRoot[root]
		if !ok {
			isl = &island{}
			byRoot[root] = isl
			roots = append(roots, root)
		}
		isl.pairs = append(isl.pairs, pair)
		for _, body := range [2]int{pair.a, pair.b} {
			if !slices.Contains(isl.bodies, body) {
				isl.bodies = append(isl.bodies, body)
			}
			if dynamic(body) && !slices.Contains(isl.dynamic, body) {
				isl.dynamic = append(isl.dynamic, body)
			}
		}
	}
	slices.Sort(roots)
	out := make([]island, 0, len(roots))
	for _, root := range roots {
		isl := byRoot[root]
		slices.Sort(isl.bodies)
		slices.Sort(isl.dynamic)
		slices.SortFunc(isl.pairs, func(a, b islandPair) int { return a.key - b.key })
		out = append(out, *isl)
	}
	return out, nil
}

// islandBodies names an island's bodies for a diagnostic.
func (w *World) islandBodies(isl island) []*decad.Body {
	out := make([]*decad.Body, 0, len(isl.bodies))
	for _, index := range isl.bodies {
		out = append(out, w.bodies[index].definition.Body)
	}
	return out
}

// correctIsland is §6.6: each dynamic body receives one translation, the sum
// over the island's pairs of its inverse-mass share of the pair's deepest
// penetration along the pair normal; a Fixed or Kinematic body takes no
// share, and neither does a group resting on one through a persistent track
// (it is anchored). Bodies joined by pairs the contact set continued on
// persistent tracks move as one: their touch is exact and their velocities equal, so
// they take one translation with their summed mass, which keeps that touch.
// The penetration is the one the pair's rounded event poses show. It may not
// exceed its allowance: ContactSlop, plus the point's certified geometry
// displacement (separation and witness bounds), plus, for an interior
// impact, the travel its bracket allows at the pair's closing speed. Each
// translation's length is bounded by the summed allowances of the pairs that
// moved the body. The translations, and each moved body's summed allowance,
// which also bounds a later push (correctedRelation), are returned by world
// index.
func (w *World) correctIsland(isl island, pre State, drive map[int][3]*big.Rat) (map[int]r3.Vec, map[int]float64,
	*StepDiagnostic) {
	group := make(map[int]int, len(isl.dynamic))
	for _, index := range isl.dynamic {
		group[index] = index
	}
	find := func(i int) int {
		for group[i] != i {
			i = group[i]
		}
		return i
	}
	dynamic := func(i int) bool { return w.bodies[i].definition.Role == Dynamic }
	for _, pair := range isl.pairs {
		if pair.track && dynamic(pair.a) && dynamic(pair.b) {
			ra, rb := find(pair.a), find(pair.b)
			group[max(ra, rb)] = min(ra, rb)
		}
	}
	groupMass := map[int]float64{}
	for _, index := range isl.dynamic {
		groupMass[find(index)] += w.bodies[index].mass.Mass.Value.Base()
	}
	// A group resting on a Fixed or Kinematic body through a persistent
	// track is anchored: moving it would break that exact touch, so another
	// pair's penetration is removed by moving its other side alone.
	anchored := map[int]struct{}{}
	for _, pair := range isl.pairs {
		if pair.track && dynamic(pair.a) != dynamic(pair.b) {
			index := pair.a
			if !dynamic(index) {
				index = pair.b
			}
			anchored[find(index)] = struct{}{}
		}
	}
	moves := map[int]r3.Vec{}
	allowances := map[int]float64{}
	for _, pair := range isl.pairs {
		if pair.depth == nil || dynamic(pair.a) && dynamic(pair.b) && find(pair.a) == find(pair.b) {
			continue
		}
		depth, geometry := 0.0, 0.0
		normal := pair.depth.Points[0].Normal.Value
		for _, p := range pair.depth.Points {
			depth = math.Max(depth, -p.Separation.Value.Base())
			geometry = math.Max(geometry, outwardSum(p.Separation.Bound.Base(), p.OnA.Bound.Base(), p.OnB.Bound.Base()))
			if depth > 0 && p.Normal.Value != normal {
				d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
					"a penetrating pair has no single correction normal")
				return nil, nil, &d
			}
		}
		if depth <= 0 {
			continue
		}
		allowance := outwardSum(w.step.ContactSlop.Base(), geometry)
		if pair.bracket != nil {
			travel, ok := boundBracketTravel(*pair.bracket, w.closingSpeedUpper(pair, pre, drive))
			if !ok {
				d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
					"impact bracket travel is not bounded")
				return nil, nil, &d
			}
			allowance = outwardSum(allowance, travel)
		}
		if !finite(depth, allowance) || depth > allowance {
			d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
				fmt.Sprintf("penetration %g exceeds its correction allowance %g", depth, allowance))
			d.Limit = units.Millimeters(allowance)
			return nil, nil, &d
		}
		var inverse [2]float64
		for side, index := range [2]int{pair.a, pair.b} {
			if _, ok := anchored[find(index)]; dynamic(index) && !ok {
				inverse[side] = 1 / groupMass[find(index)]
			}
		}
		total := inverse[0] + inverse[1]
		if total <= 0 {
			d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
				"a penetrating pair has no body free to move")
			return nil, nil, &d
		}
		for side, index := range [2]int{pair.a, pair.b} {
			if inverse[side] == 0 {
				continue
			}
			share := depth * inverse[side] / total
			if side == 0 {
				share = -share
			}
			root := find(index)
			moves[root] = moves[root].Add(normal.Scale(share))
			allowances[root] = outwardSum(allowances[root], allowance)
		}
	}
	out, limits := map[int]r3.Vec{}, map[int]float64{}
	for _, index := range isl.dynamic {
		root := find(index)
		move, ok := moves[root]
		if !ok {
			continue
		}
		length := math.Nextafter(math.Abs(move.X)+math.Abs(move.Y)+math.Abs(move.Z), math.Inf(1))
		if !finite(length) || length > allowances[root] {
			d := scheduleDiagnostic(StepCorrectionFailed, BodyPair{},
				fmt.Sprintf("correction of body %d exceeds its allowance", index))
			d.Bodies, d.Limit = w.islandBodies(isl), units.Millimeters(allowances[root])
			return nil, nil, &d
		}
		out[index], limits[index] = move, allowances[root]
	}
	return out, limits, nil
}

// closingSpeedUpper bounds from above the speed of every contact point of a
// pair relative to the other body, from the pre-event velocities: the L1
// norm of the linear difference plus each body's spin times its lever, both
// L1 norms of an L2 quantity. A kinematic participant contributes its exact
// translation velocity.
func (w *World) closingSpeedUpper(pair islandPair, pre State, drive map[int][3]*big.Rat) float64 {
	velocity := func(index int) ([3]float64, [3]float64) {
		if v, ok := drive[index]; ok {
			var out [3]float64
			for axis := range out {
				out[axis], _ = v[axis].Float64()
			}
			return out, [3]float64{}
		}
		if w.bodies[index].definition.Role != Dynamic {
			return [3]float64{}, [3]float64{}
		}
		entry := pre.entries[index]
		v, omega := vecOf(entry.LinearVelocity), vecOf(entry.AngularVelocity)
		return [3]float64{v.X, v.Y, v.Z}, [3]float64{omega.X, omega.Y, omega.Z}
	}
	vA, wA := velocity(pair.a)
	vB, wB := velocity(pair.b)
	speed := 0.0
	for axis := range 3 {
		speed = outwardSum(speed, math.Abs(vB[axis]-vA[axis]))
	}
	spin := func(omega [3]float64, index int, witness decad.VecMeasurement) float64 {
		if omega == ([3]float64{}) {
			return 0
		}
		center := pre.entries[index].Pose.Apply(w.bodies[index].mass.Center.Value)
		lever := outwardSum(math.Abs(witness.Value.X-center.X), math.Abs(witness.Value.Y-center.Y),
			math.Abs(witness.Value.Z-center.Z), 3*witness.Bound.Base(), 3*w.bodies[index].mass.Center.Bound.Base())
		rate := outwardSum(math.Abs(omega[0]), math.Abs(omega[1]), math.Abs(omega[2]))
		return math.Nextafter(math.Nextafter(lever*rate, math.Inf(1)), math.Inf(1))
	}
	largest := 0.0
	for _, point := range pair.manifold.Points {
		largest = math.Max(largest, outwardSum(spin(wA, pair.a, point.OnA), spin(wB, pair.b, point.OnB)))
	}
	return outwardSum(speed, largest)
}

// solveIslands is §5 step 7 at one event time: §6.1's islands over the
// active gathered pairs, solved and certified in island order (§6.2,
// §6.3), corrected (§6.6), and published as one event per solved pair and one
// IslandReport per island. An island made only of pairs the contact set
// continued on persistent tracks holds no event and is left to drift. Every
// gathered pair receives its continuation policy (§5.2).
func (w *World) solveIslands(ctx context.Context, in eventIslands,
	scheduled map[int]struct{}) (*solvedEvent, []StepDiagnostic, error) {
	var active []islandPair
	out := &solvedEvent{post: in.pre.clone(), policies: map[int]decad.SweepStartPolicy{}}
	for _, pair := range in.gathered {
		if !w.drivenWithin(pair, in.drive) {
			// A rotating driver's pair cannot be classified; it stays in the
			// solve, which refuses it if an event reaches its island.
			active = append(active, pair)
			continue
		}
		ok, valid := w.pairActive(pair, in.pre, in.drive)
		if !valid {
			return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(w.pairs[pair.key]),
				"event manifold cannot be read as exact intervals")}, nil
		}
		if !ok {
			out.policies[pair.key] = decad.ContinueSeparatingTouch
			continue
		}
		active = append(active, pair)
	}
	islands, diagnostic := w.formIslands(active)
	if diagnostic != nil {
		return nil, []StepDiagnostic{*diagnostic}, nil
	}
	var solved []island
	moves := map[int]r3.Vec{}
	push := correctionPush{allowance: map[int]float64{}, separating: map[int]struct{}{}, policies: out.policies}
	for _, isl := range islands {
		if !slices.ContainsFunc(isl.pairs, func(p islandPair) bool { return !p.track }) {
			for _, pair := range isl.pairs {
				out.policies[pair.key] = decad.ContinueCertifiedTouch
			}
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		for _, pair := range isl.pairs {
			if !w.drivenWithin(pair, in.drive) {
				return nil, []StepDiagnostic{scheduleDiagnostic(StepUnsupported, w.bodyPair(w.pairs[pair.key]),
					"an island with a rotating kinematic participant has no solver yet")}, nil
			}
		}
		number := in.islandBase + len(solved)
		solution, failure := w.solveIsland(isl, in.pre, in.drive, in.work)
		if failure != nil {
			d := scheduleDiagnostic(failure.code, w.bodyPair(w.pairs[isl.pairs[0].key]),
				fmt.Sprintf("island %d: %s", number, failure.reason))
			d.Bodies, d.Limit = w.islandBodies(isl), failure.limit
			return nil, []StepDiagnostic{d}, nil
		}
		corrections, allowances, diagnostic := w.correctIsland(isl, in.pre, in.drive)
		if diagnostic != nil {
			return nil, []StepDiagnostic{*diagnostic}, nil
		}
		maps.Copy(push.allowance, allowances)
		if len(corrections) == 0 && w.silentIsland(isl, solution, in) {
			for _, pair := range isl.pairs {
				out.policies[pair.key] = decad.ContinueCertifiedTouch
			}
			continue
		}
		for slot, index := range isl.bodies {
			if w.bodies[index].definition.Role != Dynamic {
				continue
			}
			entry := &out.post.entries[index]
			entry.LinearVelocity, entry.AngularVelocity = solution.linear[slot], solution.angular[slot]
			if move, ok := corrections[index]; ok {
				pose, err := translatePose(entry.Pose, move)
				if err != nil {
					d := scheduleDiagnostic(StepCorrectionFailed, BodyPair{},
						fmt.Sprintf("corrected pose of body %d is not finite: %v", index, err))
					d.Bodies = w.islandBodies(isl)
					return nil, []StepDiagnostic{d}, nil
				}
				entry.Pose = pose
				moves[index] = pose.Translation().Sub(in.pre.entries[index].Pose.Translation())
			}
		}
		out.islands = append(out.islands, w.islandEvents(number, isl, solution, in, &out.events))
		solved = append(solved, isl)
		// §5.2: a solved pair whose every point leaves faster than
		// VelocityResidual continues under ContinueSeparatingTouch, which
		// proves the departure itself; every other one under
		// ContinueCertifiedTouch.
		for i, pair := range isl.pairs {
			out.policies[pair.key] = decad.ContinueCertifiedTouch
			if solution.separating[i] {
				out.policies[pair.key] = decad.ContinueSeparatingTouch
				push.separating[pair.key] = struct{}{}
			}
		}
	}
	if count := in.eventBase + len(out.events); count > w.step.MaxEvents {
		d := scheduleDiagnostic(StepEventBudget, BodyPair{},
			fmt.Sprintf("%d events exceed MaxEvents %d", count, w.step.MaxEvents))
		d.Limit = units.Scalar(float64(w.step.MaxEvents))
		return nil, []StepDiagnostic{d}, nil
	}
	diagnostics, err := w.checkCorrections(ctx, in.work, in.pre, out.post, solved, moves, scheduled, push)
	if err != nil || len(diagnostics) != 0 {
		return nil, diagnostics, err
	}
	for i := range out.events {
		e := &out.events[i]
		e.PositionChangeA, e.PositionChangeB = moves[w.index[e.Pair.A]], moves[w.index[e.Pair.B]]
	}
	return out, nil, nil
}

// silentIsland reports whether a solved island changes nothing and so
// publishes nothing (docs/multibody-dynamics-design.md §6.1): every point's
// enclosed pre-solve normal speed lies within VelocityResidual of zero, every
// normal and tangent impulse is exactly zero, and every dynamic body keeps
// its exact pre-solve velocities. Its pairs join the contact set under
// ContinueCertifiedTouch without an event, as a stationary or sliding touch
// does in the two-body step.
func (w *World) silentIsland(isl island, solution islandSolution, in eventIslands) bool {
	for _, lambda := range solution.lambda {
		if lambda != 0 {
			return false
		}
	}
	for _, tangent := range solution.tangent {
		if tangent != (r3.Vec{}) {
			return false
		}
	}
	for slot, index := range isl.bodies {
		entry := in.pre.entries[index]
		if w.bodies[index].definition.Role == Dynamic &&
			(solution.linear[slot] != entry.LinearVelocity || solution.angular[slot] != entry.AngularVelocity) {
			return false
		}
	}
	for _, pair := range isl.pairs {
		if !w.grazeSpeedWithin(pair, in.pre, in.drive) {
			return false
		}
	}
	return true
}

// drivenWithin reports whether every kinematic body of a pair has the exact
// translation velocity of its driver in drive.
func (w *World) drivenWithin(pair islandPair, drive map[int][3]*big.Rat) bool {
	for _, index := range [2]int{pair.a, pair.b} {
		if _, ok := drive[index]; !ok && w.bodies[index].definition.Role == Kinematic {
			return false
		}
	}
	return true
}

// islandEvents publishes one ContactEvent per island pair and the island's
// report. Point impulses follow each manifold's point order; the aggregate
// normal impulse is their sum. A kinematic participant reports its driver
// velocity before and after the event.
func (w *World) islandEvents(number int, isl island, solution islandSolution, in eventIslands,
	events *[]ContactEvent) IslandReport {
	slots := make(map[int]int, len(isl.bodies))
	for slot, index := range isl.bodies {
		slots[index] = slot
	}
	pre := in.pre
	report := IslandReport{Time: in.at, Solver: solution.report}
	for _, index := range isl.bodies {
		report.Bodies = append(report.Bodies, w.bodies[index].definition.Body)
	}
	velocity := func(index int, published QuantityVec) QuantityVec {
		if v, ok := in.drive[index]; ok {
			return ratVelocity(v)
		}
		return published
	}
	k := 0
	for _, pair := range isl.pairs {
		solver := solution.report
		bracket := decad.SweepInterval{From: pair.at, To: pair.at}
		if pair.bracket != nil {
			bracket = *pair.bracket
		}
		event := ContactEvent{Kind: ContactImpact, Pair: w.bodyPair(w.pairs[pair.key]),
			Bracket: bracket, SliceStart: in.sliceStart, SliceDuration: in.sliceSpan,
			Time: in.at, Manifold: cloneManifold(pair.manifold),
			TangentImpulse: zeroImpulseVec(), Solver: &solver, Island: number,
			PoseA: pre.entries[pair.a].Pose, PoseB: pre.entries[pair.b].Pose}
		total := 0.0
		var tangent r3.Vec
		for range pair.manifold.Points {
			total += solution.lambda[k]
			tangent = tangent.Add(solution.tangent[k])
			event.PointImpulses = append(event.PointImpulses, ContactPointImpulse{
				Normal: units.KilogramMillimetersPerSecond(solution.lambda[k]), Tangent: impulseVec(solution.tangent[k])})
			k++
		}
		event.NormalImpulse = units.KilogramMillimetersPerSecond(total)
		event.TangentImpulse = impulseVec(tangent)
		sa, sb := slots[pair.a], slots[pair.b]
		event.PreVelocityA = velocity(pair.a, pre.entries[pair.a].LinearVelocity)
		event.PreVelocityB = velocity(pair.b, pre.entries[pair.b].LinearVelocity)
		event.PreAngularVelocityA = pre.entries[pair.a].AngularVelocity
		event.PreAngularVelocityB = pre.entries[pair.b].AngularVelocity
		event.PostVelocityA = velocity(pair.a, solution.linear[sa])
		event.PostVelocityB = velocity(pair.b, solution.linear[sb])
		event.PostAngularVelocityA, event.PostAngularVelocityB = solution.angular[sa], solution.angular[sb]
		event.PreVelocity, event.PostVelocity = event.PreVelocityB, event.PostVelocityB
		if w.bodies[pair.b].definition.Role != Dynamic {
			event.PreVelocity, event.PostVelocity = event.PreVelocityA, event.PostVelocityA
		}
		report.Pairs = append(report.Pairs, event.Pair)
		report.Events = append(report.Events, in.eventBase+len(*events))
		*events = append(*events, event)
	}
	return report
}

// impulseVec publishes a world impulse vector.
func impulseVec(v r3.Vec) QuantityVec {
	return QuantityVec{X: units.KilogramMillimetersPerSecond(v.X), Y: units.KilogramMillimetersPerSecond(v.Y),
		Z: units.KilogramMillimetersPerSecond(v.Z)}
}

// ratVelocity publishes an exact velocity at the nearest float components.
func ratVelocity(v [3]*big.Rat) QuantityVec {
	var out QuantityVec
	for axis, value := range v {
		f, _ := value.Float64()
		setVelocityComponent(&out, axis, units.MillimetersPerSecond(f))
	}
	return out
}

// checkCorrections proves §6.6's corrections lose no relation and create no
// contact: every island pair with a moved body must still touch at the
// corrected poses, or, when its solve separates it, be pushed just apart
// (correctedRelation), and every other scheduled pair with a moved body must
// sweep clear over the correction, after the swept-box exclusion.
func (w *World) checkCorrections(ctx context.Context, work *stepWork, pre, post State, islands []island,
	moves map[int]r3.Vec, scheduled map[int]struct{}, push correctionPush) ([]StepDiagnostic, error) {
	if len(moves) == 0 {
		return nil, nil
	}
	moved := func(key int) bool {
		_, a := moves[w.pairs[key].a]
		_, b := moves[w.pairs[key].b]
		return a || b
	}
	inIsland := map[int]struct{}{}
	for _, isl := range islands {
		for _, pair := range isl.pairs {
			inIsland[pair.key] = struct{}{}
		}
	}
	// A push moves a body another island pair shares, so the pairs are
	// checked again until a pass pushes nothing; each pass that pushes
	// moves some pair apart, and pushLimit passes bound the work.
	for pass := 0; ; pass++ {
		pushed := false
		for _, isl := range islands {
			for _, pair := range isl.pairs {
				if !moved(pair.key) {
					continue
				}
				if err := ctx.Err(); err != nil {
					return nil, err
				}
				done, diagnostic, err := w.correctedRelation(ctx, pre, post, moves, pair, push, pass < pushLimit)
				if err != nil {
					return nil, err
				}
				if diagnostic != nil {
					diagnostic.Bodies = w.islandBodies(isl)
					return []StepDiagnostic{*diagnostic}, nil
				}
				pushed = pushed || !done
			}
		}
		if !pushed {
			break
		}
	}
	for key := range w.pairs {
		if _, ok := scheduled[key]; !ok {
			continue
		}
		if _, ok := inIsland[key]; ok || !moved(key) {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		pair := w.pairs[key]
		paths := [2]decad.PairPath{}
		var boxes [2]decad.SweptBox
		duration := units.Seconds(1)
		for side, index := range [2]int{pair.a, pair.b} {
			paths[side] = decad.PoseSegment{From: pre.entries[index].Pose, To: post.entries[index].Pose,
				Duration: duration}
			box, err := work.sweptBox(ctx, index, paths[side])
			if err != nil {
				return nil, err
			}
			boxes[side] = box
		}
		if boxes[0].StrictlyDisjoint(boxes[1]) {
			continue
		}
		sweep, err := work.sweepPair(ctx, key, paths[0], paths[1], w.sweepRequest(duration, decad.StopAtInitialContact))
		if err != nil {
			return nil, err
		}
		if sweep.Outcome != decad.SweepClear {
			return []StepDiagnostic{scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(pair),
				fmt.Sprintf("correction sweep returned %v", sweep.Outcome))}, nil
		}
	}
	return nil, nil
}

// pushLimit bounds the passes over an event's island pairs that may push a
// separating pair apart.
const pushLimit = 4

// correctionPush is what correctedRelation needs to push a pair apart: each
// corrected body's correction allowance, the island pairs whose every point the
// solve separates, and the contact-set policies, which a separated pair
// leaves.
type correctionPush struct {
	allowance  map[int]float64
	separating map[int]struct{}
	policies   map[int]decad.SweepStartPolicy
}

// correctedRelation checks one island pair at the corrected poses. A
// touching pair passes. A translation along a float normal rarely lands two
// curved bodies in exact touch, so a pair the solve separates is pushed just
// apart (rigid-dynamics "Response": a separated corrected pose within the
// correction allowance) when it reads either of two ways:
//   - it still overlaps: its dynamic bodies move along the deepest point's
//     normal by that point's depth plus its separation bound, split by
//     inverse mass and doubled until it moves the rounded pose, and the
//     caller checks the island again;
//   - it is apart by less than ContactPair can prove (Undecided with
//     ContactNoGapProof): its dynamic bodies move apart along the event
//     manifold's normal, split by inverse mass, by one ulp of their largest
//     coordinate and then twice as far each time, until ContactPair proves
//     the pair separated or touching.
//
// A pushed pair that ends touching continues under ContinueSeparatingTouch;
// one that ends separated leaves the contact set, so its next slice starts
// under StopAtInitialContact. A pair the solve does not separate is never pushed and may not end
// separated: its continuation needs exact touch. Every pushed body's whole
// translation from its pre-event pose, measured as the correction's is, must
// stay within its correction allowance. It reports false when it pushed, so the
// caller checks the island again; allowed is false on the last pass, where a
// pair that still needs a push is refused.
func (w *World) correctedRelation(ctx context.Context, pre, post State, moves map[int]r3.Vec, pair islandPair,
	push correctionPush, allowed bool) (bool, *StepDiagnostic, error) {
	fail := func(reason string) (bool, *StepDiagnostic, error) {
		d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]), reason)
		return true, &d, nil
	}
	contact, err := w.contactAt(ctx, pair, post)
	if err != nil {
		return true, nil, err
	}
	_, separating := push.separating[pair.key]
	switch {
	case contact.Relation == decad.ContactTouching:
		return true, nil, nil
	case contact.Relation == decad.ContactSeparated && separating:
		// A separated pair leaves the contact set: its next slice starts
		// clear, under StopAtInitialContact, where ContinueSeparatingTouch
		// needs a touching start.
		delete(push.policies, pair.key)
		return true, nil, nil
	case contact.Relation == decad.ContactUndecided && contact.Reason == decad.ContactNoGapProof &&
		separating && allowed && len(pair.manifold.Points) != 0:
		return w.pushProvablyApart(ctx, pre, post, moves, pair, push)
	case contact.Relation != decad.ContactOverlapping || !separating || !allowed ||
		!w.manifoldWithin(contact.Manifold):
		return fail(fmt.Sprintf("corrected pair relation is %v, not touching", contact.Relation))
	}
	depth, normal := 0.0, r3.Vec{}
	for _, p := range contact.Manifold.Points {
		if d := outwardSum(-p.Separation.Value.Base(), p.Separation.Bound.Base()); d > depth {
			depth, normal = d, p.Normal.Value
		}
	}
	if !finite(depth) || depth <= 0 {
		return fail("an overlapping separating pair has no push")
	}
	poses, diagnostic := w.pushPoses(pre, post, pair, normal, depth, push, true)
	if diagnostic != nil {
		return true, diagnostic, nil
	}
	for index, pose := range poses {
		post.entries[index].Pose = pose
		moves[index] = pose.Translation().Sub(pre.entries[index].Pose.Translation())
	}
	return false, nil, nil
}

// pushProvablyApart moves a pair that is apart by less than ContactPair can
// prove along its event manifold's normal, by one ulp of the larger body
// coordinate and then twice as far each time, until ContactPair proves it
// separated or touching. It reports false when it pushed.
func (w *World) pushProvablyApart(ctx context.Context, pre, post State, moves map[int]r3.Vec, pair islandPair,
	push correctionPush) (bool, *StepDiagnostic, error) {
	normal := pair.manifold.Points[0].Normal.Value
	largest := 0.0
	for _, index := range [2]int{pair.a, pair.b} {
		t := post.entries[index].Pose.Translation()
		largest = math.Max(largest, math.Max(math.Abs(t.X), math.Max(math.Abs(t.Y), math.Abs(t.Z))))
	}
	amount := math.Nextafter(largest, math.Inf(1)) - largest
	trial := post.clone()
	for range 64 {
		poses, diagnostic := w.pushPoses(pre, post, pair, normal, amount, push, false)
		if diagnostic != nil {
			return true, diagnostic, nil
		}
		for index, pose := range poses {
			trial.entries[index].Pose = pose
		}
		if err := ctx.Err(); err != nil {
			return true, nil, err
		}
		contact, err := w.contactAt(ctx, pair, trial)
		if err != nil {
			return true, nil, err
		}
		switch contact.Relation {
		case decad.ContactSeparated, decad.ContactTouching:
			for index, pose := range poses {
				post.entries[index].Pose = pose
				moves[index] = pose.Translation().Sub(pre.entries[index].Pose.Translation())
			}
			return false, nil, nil
		case decad.ContactUndecided:
			amount *= 2
		default:
			d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
				fmt.Sprintf("pushed pair relation is %v", contact.Relation))
			return true, &d, nil
		}
	}
	d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
		"push finds no provable separation")
	return true, &d, nil
}

// contactAt queries ContactPair for an island pair at a state's poses.
func (w *World) contactAt(ctx context.Context, pair islandPair, state State) (*decad.ContactReport, error) {
	return w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body, w.bodies[pair.b].definition.Body,
		state.entries[pair.a].Pose, state.entries[pair.b].Pose, w.step.Contact)
}

// pushPoses moves a pair's dynamic bodies apart along normal by amount,
// split by inverse mass (A against the normal, B along it). With grow set a
// share too small to move a rounded pose doubles until it does. Every moved
// body's whole translation from its pre-event pose, measured as the
// correction's is, must stay within its correction allowance. It returns the new
// pose of each moved body by world index.
func (w *World) pushPoses(pre, post State, pair islandPair, normal r3.Vec, amount float64, push correctionPush,
	grow bool) (map[int]r3.Transform, *StepDiagnostic) {
	fail := func(reason string, limit units.Value) (map[int]r3.Transform, *StepDiagnostic) {
		d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]), reason)
		d.Limit = limit
		return nil, &d
	}
	var inverse [2]float64
	for side, index := range [2]int{pair.a, pair.b} {
		if w.bodies[index].definition.Role == Dynamic {
			inverse[side] = 1 / w.bodies[index].mass.Mass.Value.Base()
		}
	}
	total := inverse[0] + inverse[1]
	if !finite(amount, total) || amount <= 0 || total <= 0 {
		return fail("a separating pair has no push", units.Value{})
	}
	out := map[int]r3.Transform{}
	for side, index := range [2]int{pair.a, pair.b} {
		if inverse[side] == 0 {
			continue
		}
		share := amount * inverse[side] / total
		if side == 0 {
			share = -share
		}
		start := post.entries[index].Pose
		pose, err := translatePose(start, normal.Scale(share))
		// A share below the ulp of the body's coordinates rounds to no move;
		// it doubles until the translation changes, and the allowance below
		// bounds it.
		for doubling := 0; grow && err == nil && pose.Translation() == start.Translation() && doubling < 64; doubling++ {
			share *= 2
			pose, err = translatePose(start, normal.Scale(share))
		}
		if err != nil {
			return fail(fmt.Sprintf("pushed pose of body %d is not finite: %v", index, err), units.Value{})
		}
		move := pose.Translation().Sub(pre.entries[index].Pose.Translation())
		length := math.Nextafter(math.Abs(move.X)+math.Abs(move.Y)+math.Abs(move.Z), math.Inf(1))
		allowance, ok := push.allowance[index]
		if !ok || !finite(length) || length > allowance {
			return fail(fmt.Sprintf("push of body %d exceeds its correction allowance", index),
				units.Millimeters(allowance))
		}
		out[index] = pose
	}
	return out, nil
}

// islandContactImpulse sums the impulses island events deliver from Fixed
// and Kinematic bodies to dynamic ones: each point's normal impulse widened
// by its normal ball, plus its exact published tangent impulse. A dynamic
// pair's two impulses cancel in the world total, and a graze or transition
// delivers none.
func (w *World) islandContactImpulse(events []ContactEvent) (MomentumReading, bool) {
	var value, low, high [3]*big.Rat
	for axis := range value {
		value[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for _, event := range events {
		if event.Kind != ContactImpact {
			continue
		}
		a, b := w.index[event.Pair.A], w.index[event.Pair.B]
		dynamicA, dynamicB := w.bodies[a].definition.Role == Dynamic, w.bodies[b].definition.Role == Dynamic
		if dynamicA == dynamicB || len(event.PointImpulses) != len(event.Manifold.Points) {
			if dynamicA && dynamicB {
				continue
			}
			return MomentumReading{}, false
		}
		sign := big.NewRat(1, 1)
		if dynamicA {
			sign = big.NewRat(-1, 1)
		}
		for i, point := range event.Manifold.Points {
			lambda := exactBase(event.PointImpulses[i].Normal)
			bound, angle := exactBase(point.Normal.Bound), exactBase(point.NormalAngle)
			normal, ok := ratVec(point.Normal.Value)
			tangent, okTangent := quantityRats(event.PointImpulses[i].Tangent)
			if lambda == nil || bound == nil || angle == nil || !ok || !okTangent {
				return MomentumReading{}, false
			}
			width := new(big.Rat).Mul(lambda, new(big.Rat).Add(bound, angle))
			for axis := range value {
				applied := new(big.Rat).Add(new(big.Rat).Mul(lambda, normal[axis]), tangent[axis])
				applied.Mul(applied, sign)
				value[axis].Add(value[axis], applied)
				low[axis].Add(low[axis], new(big.Rat).Sub(applied, width))
				high[axis].Add(high[axis], new(big.Rat).Add(applied, width))
			}
		}
	}
	return boundedMomentum(value, low, high)
}

// islandKinematicWork sums the work kinematic drivers deliver at island
// events: at each point, J·V for a driver on side A of the pair and −J·V on
// side B, with J = λ·n + λt the impulse on B and V the driver's published
// velocity. The normal ball widens the reading by λ·|V|₁·(bound + angle).
func (w *World) islandKinematicWork(events []ContactEvent) (decad.Measurement, bool) {
	value, low, high := new(big.Rat), new(big.Rat), new(big.Rat)
	for _, event := range events {
		if event.Kind != ContactImpact {
			continue
		}
		for side, body := range [2]*decad.Body{event.Pair.A, event.Pair.B} {
			if w.bodies[w.index[body]].definition.Role != Kinematic {
				continue
			}
			velocity := event.PreVelocityA
			sign := big.NewRat(1, 1)
			if side == 1 {
				velocity, sign = event.PreVelocityB, big.NewRat(-1, 1)
			}
			v, ok := quantityRats(velocity)
			if !ok || len(event.PointImpulses) != len(event.Manifold.Points) {
				return decad.Measurement{}, false
			}
			speed := new(big.Rat)
			for _, component := range v {
				speed.Add(speed, absRat(new(big.Rat).Set(component)))
			}
			for i, point := range event.Manifold.Points {
				lambda := exactBase(event.PointImpulses[i].Normal)
				bound, angle := exactBase(point.Normal.Bound), exactBase(point.NormalAngle)
				normal, okNormal := ratVec(point.Normal.Value)
				tangent, okTangent := quantityRats(event.PointImpulses[i].Tangent)
				if lambda == nil || bound == nil || angle == nil || !okNormal || !okTangent {
					return decad.Measurement{}, false
				}
				work := new(big.Rat).Set(proof.DotInterval3(pointIVec(normal), pointIVec(v)).Lo)
				work.Mul(work, lambda)
				work.Add(work, proof.DotInterval3(pointIVec(tangent), pointIVec(v)).Lo)
				work.Mul(work, sign)
				width := new(big.Rat).Mul(absRat(new(big.Rat).Set(lambda)), speed)
				width.Mul(width, new(big.Rat).Add(bound, angle))
				value.Add(value, work)
				low.Add(low, new(big.Rat).Sub(work, width))
				high.Add(high, new(big.Rat).Add(work, width))
			}
		}
	}
	return boundedReading(value, low, high, units.KilogramSquareMillimeterPerSecondSquared)
}
