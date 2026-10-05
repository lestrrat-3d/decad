package dynamics

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file forms islands (docs/multibody-dynamics-design.md §6.1) and
// corrects their positions (§6.6) for the step of a world of four or more
// bodies.

// IslandReport is one simultaneous solve: the bodies and pairs that shared
// constraints at one event time, and the certified residuals of that solve.
type IslandReport struct {
	Time   units.Value   // from the start of the step
	Bodies []*decad.Body // world order; Fixed participants included
	Pairs  []BodyPair    // world order
	Events []int         // indices into StepReport.Events
	Solver ContactSolverReport
}

// islandPair is one gathered pair at an event: its canonical key, its
// world-order bodies, the manifold its producer certified, and the sweep
// instant of that manifold.
type islandPair struct {
	key      int
	a, b     int
	manifold decad.ContactManifold
	at       decad.SweepInstant
}

// island is one connected component of the dynamic-body contact graph.
// Fixed participants attach to it without joining two islands.
type island struct {
	dynamic []int        // world order
	bodies  []int        // world order, Fixed participants included
	pairs   []islandPair // canonical order
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

// pairActive reports whether any point of an initially touching pair may be
// closing: its enclosed relative normal speed reaches down to
// VelocityResidual or below. Only a pair whose every point certainly
// separates by more than that stays out of the solve.
func (w *World) pairActive(pair islandPair, state State) (bool, bool) {
	a, okA := w.newCertBody(pair.a, state.entries[pair.a], state.entries[pair.a])
	b, okB := w.newCertBody(pair.b, state.entries[pair.b], state.entries[pair.b])
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
		out = append(out, *isl)
	}
	return out, nil
}

// correctIsland is §6.6 at an initial contact: each dynamic body receives one
// translation, the sum over the island's pairs of its inverse-mass share of
// the pair's deepest penetration along the pair normal; a Fixed body takes
// no share. A pair's penetration may not exceed its allowance,
// ContactSlop plus the point's certified geometry displacement (separation
// and witness bounds); at an initial contact there is no bracket travel.
// Each translation's length is bounded by the summed allowances of the
// pairs that moved the body. The translations are returned by world index.
func (w *World) correctIsland(isl island) (map[int]r3.Vec, *StepDiagnostic) {
	out := map[int]r3.Vec{}
	allowances := map[int]float64{}
	for _, pair := range isl.pairs {
		depth, geometry := 0.0, 0.0
		normal := pair.manifold.Points[0].Normal.Value
		for _, p := range pair.manifold.Points {
			depth = math.Max(depth, -p.Separation.Value.Base())
			geometry = math.Max(geometry, outwardSum(p.Separation.Bound.Base(), p.OnA.Bound.Base(), p.OnB.Bound.Base()))
			if depth > 0 && p.Normal.Value != normal {
				d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
					"a penetrating pair has no single correction normal")
				return nil, &d
			}
		}
		if depth <= 0 {
			continue
		}
		allowance := outwardSum(w.step.ContactSlop.Base(), geometry)
		if !finite(depth, allowance) || depth > allowance {
			d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
				fmt.Sprintf("initial penetration %g exceeds its correction allowance %g", depth, allowance))
			return nil, &d
		}
		var inverse [2]float64
		for side, index := range [2]int{pair.a, pair.b} {
			if w.bodies[index].definition.Role == Dynamic {
				inverse[side] = 1 / w.bodies[index].mass.Mass.Value.Base()
			}
		}
		total := inverse[0] + inverse[1]
		for side, index := range [2]int{pair.a, pair.b} {
			if inverse[side] == 0 {
				continue
			}
			share := depth * inverse[side] / total
			if side == 0 {
				share = -share
			}
			out[index] = out[index].Add(normal.Scale(share))
			allowances[index] = outwardSum(allowances[index], allowance)
		}
	}
	for _, index := range isl.dynamic {
		move, ok := out[index]
		if !ok {
			continue
		}
		length := math.Nextafter(math.Abs(move.X)+math.Abs(move.Y)+math.Abs(move.Z), math.Inf(1))
		if !finite(length) || length > allowances[index] {
			d := scheduleDiagnostic(StepCorrectionFailed, BodyPair{},
				fmt.Sprintf("correction of body %d exceeds its allowance", index))
			return nil, &d
		}
	}
	return out, nil
}

// initialIslands is the event at the step start: §6.1's islands over the
// gathered initial contacts, solved and certified in island order (§6.2,
// §6.3), corrected (§6.6), and published as one event per gathered pair
// and one IslandReport per island. It returns that event record, holding the
// post-event state and the contact-set policy of every gathered pair, or an
// Undecided report.
func (w *World) initialIslands(ctx context.Context, kicked State, dt units.Value,
	gathered []islandPair, scheduled map[int]struct{}) (*initialEvent, *StepReport, error) {
	var active []islandPair
	policies := map[int]decad.SweepStartPolicy{}
	for _, pair := range gathered {
		roles := [2]BodyRole{w.bodies[pair.a].definition.Role, w.bodies[pair.b].definition.Role}
		if roles[0] == Kinematic || roles[1] == Kinematic {
			return nil, w.scheduleUndecided(scheduleDiagnostic(StepUnsupported, w.bodyPair(w.pairs[pair.key]),
				"an island with a kinematic participant has no solver yet")), nil
		}
		if upper := w.pairs[pair.key].friction.upper; upper != nil && upper.Sign() > 0 {
			return nil, w.scheduleUndecided(scheduleDiagnostic(StepUnsupported, w.bodyPair(w.pairs[pair.key]),
				"a positive-friction island pair has no solver yet")), nil
		}
		ok, valid := w.pairActive(pair, kicked)
		if !valid {
			return nil, w.scheduleUndecided(scheduleDiagnostic(StepManifoldMissing, w.bodyPair(w.pairs[pair.key]),
				"initial manifold cannot be read as exact intervals")), nil
		}
		if !ok {
			policies[pair.key] = decad.ContinueSeparatingTouch
			continue
		}
		active = append(active, pair)
	}
	islands, diagnostic := w.formIslands(active)
	if diagnostic != nil {
		return nil, w.scheduleUndecided(*diagnostic), nil
	}
	event := &initialEvent{pre: kicked, post: kicked.clone(), policies: policies}
	moves := map[int]r3.Vec{}
	for number, isl := range islands {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		solution, failure := w.solveIsland(isl, kicked)
		if failure != nil {
			return nil, w.scheduleUndecided(scheduleDiagnostic(failure.code, w.bodyPair(w.pairs[isl.pairs[0].key]),
				fmt.Sprintf("island %d: %s", number, failure.reason))), nil
		}
		corrections, diagnostic := w.correctIsland(isl)
		if diagnostic != nil {
			return nil, w.scheduleUndecided(*diagnostic), nil
		}
		for slot, index := range isl.bodies {
			if w.bodies[index].definition.Role != Dynamic {
				continue
			}
			entry := &event.post.entries[index]
			entry.LinearVelocity, entry.AngularVelocity = solution.linear[slot], solution.angular[slot]
			if move, ok := corrections[index]; ok {
				pose, err := translatePose(entry.Pose, move)
				if err != nil {
					return nil, w.scheduleUndecided(scheduleDiagnostic(StepCorrectionFailed, BodyPair{},
						fmt.Sprintf("corrected pose of body %d is not finite: %v", index, err))), nil
				}
				entry.Pose = pose
				moves[index] = pose.Translation().Sub(kicked.entries[index].Pose.Translation())
			}
		}
		event.islands = append(event.islands, w.islandEvents(number, isl, solution, kicked, &event.events, dt))
		// §5.2: a solved pair whose every point leaves faster than
		// VelocityResidual continues under ContinueSeparatingTouch, which
		// proves the departure itself; every other one under
		// ContinueCertifiedTouch.
		for i, pair := range isl.pairs {
			policies[pair.key] = decad.ContinueCertifiedTouch
			if solution.separating[i] {
				policies[pair.key] = decad.ContinueSeparatingTouch
			}
		}
	}
	for i := range event.events {
		e := &event.events[i]
		e.PositionChangeA, e.PositionChangeB = moves[w.index[e.Pair.A]], moves[w.index[e.Pair.B]]
	}
	if len(event.events) > w.step.MaxEvents {
		return nil, w.scheduleUndecided(scheduleDiagnostic(StepEventBudget, BodyPair{},
			fmt.Sprintf("%d events exceed MaxEvents %d", len(event.events), w.step.MaxEvents))), nil
	}
	diagnostics, err := w.checkCorrections(ctx, event, islands, moves, scheduled, dt)
	if err != nil {
		return nil, nil, err
	}
	if len(diagnostics) != 0 {
		return nil, w.scheduleUndecided(diagnostics...), nil
	}
	return event, nil, nil
}

// initialEvent is the published event at the step start.
type initialEvent struct {
	pre, post State
	events    []ContactEvent
	islands   []IslandReport
	policies  map[int]decad.SweepStartPolicy
}

// islandEvents publishes one ContactEvent per island pair and the island's
// report. Point impulses follow each manifold's point order; the aggregate
// normal impulse is their sum.
func (w *World) islandEvents(number int, isl island, solution islandSolution, pre State,
	events *[]ContactEvent, dt units.Value) IslandReport {
	slots := make(map[int]int, len(isl.bodies))
	for slot, index := range isl.bodies {
		slots[index] = slot
	}
	report := IslandReport{Time: units.Seconds(0), Solver: solution.report}
	for _, index := range isl.bodies {
		report.Bodies = append(report.Bodies, w.bodies[index].definition.Body)
	}
	k := 0
	for _, pair := range isl.pairs {
		solver := solution.report
		event := ContactEvent{Kind: ContactImpact, Pair: w.bodyPair(w.pairs[pair.key]),
			Bracket: decad.SweepInterval{From: pair.at, To: pair.at}, SliceStart: units.Seconds(0),
			SliceDuration: dt, Time: units.Seconds(0), Manifold: cloneManifold(pair.manifold),
			TangentImpulse: zeroImpulseVec(), Solver: &solver, Island: number,
			PoseA: pre.entries[pair.a].Pose, PoseB: pre.entries[pair.b].Pose}
		total := 0.0
		for range pair.manifold.Points {
			total += solution.lambda[k]
			event.PointImpulses = append(event.PointImpulses, ContactPointImpulse{
				Normal: units.KilogramMillimetersPerSecond(solution.lambda[k]), Tangent: zeroImpulseVec()})
			k++
		}
		event.NormalImpulse = units.KilogramMillimetersPerSecond(total)
		sa, sb := slots[pair.a], slots[pair.b]
		event.PreVelocityA, event.PreVelocityB = pre.entries[pair.a].LinearVelocity, pre.entries[pair.b].LinearVelocity
		event.PreAngularVelocityA = pre.entries[pair.a].AngularVelocity
		event.PreAngularVelocityB = pre.entries[pair.b].AngularVelocity
		event.PostVelocityA, event.PostVelocityB = solution.linear[sa], solution.linear[sb]
		event.PostAngularVelocityA, event.PostAngularVelocityB = solution.angular[sa], solution.angular[sb]
		event.PreVelocity, event.PostVelocity = event.PreVelocityB, event.PostVelocityB
		if w.bodies[pair.b].definition.Role != Dynamic {
			event.PreVelocity, event.PostVelocity = event.PreVelocityA, event.PostVelocityA
		}
		report.Pairs = append(report.Pairs, event.Pair)
		report.Events = append(report.Events, len(*events))
		*events = append(*events, event)
	}
	return report
}

// checkCorrections proves §6.6's corrections lose no relation and create no
// contact: every island pair with a moved body must still touch at the
// corrected poses, and every other scheduled pair with a moved body must
// sweep clear over the correction, after the swept-box exclusion.
func (w *World) checkCorrections(ctx context.Context, event *initialEvent, islands []island,
	moves map[int]r3.Vec, scheduled map[int]struct{}, dt units.Value) ([]StepDiagnostic, error) {
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
			if !moved(pair.key) {
				continue
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			contact, err := w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body, w.bodies[pair.b].definition.Body,
				event.post.entries[pair.a].Pose, event.post.entries[pair.b].Pose, w.step.Contact)
			if err != nil {
				return nil, err
			}
			if contact.Relation != decad.ContactTouching {
				return []StepDiagnostic{scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]),
					fmt.Sprintf("corrected pair relation is %v, not touching", contact.Relation))}, nil
			}
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
		for side, index := range [2]int{pair.a, pair.b} {
			paths[side] = decad.PoseSegment{From: event.pre.entries[index].Pose,
				To: event.post.entries[index].Pose, Duration: dt}
			box, err := w.doc.SweptBox(ctx, w.bodies[index].definition.Body, paths[side])
			if err != nil {
				return nil, err
			}
			boxes[side] = box
		}
		if boxes[0].StrictlyDisjoint(boxes[1]) {
			continue
		}
		sweep, err := w.doc.SweepPair(ctx, w.bodies[pair.a].definition.Body, w.bodies[pair.b].definition.Body,
			paths[0], paths[1], w.sweepRequest(dt, decad.StopAtInitialContact))
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

// islandContactImpulse sums the impulses island events deliver from Fixed
// bodies to dynamic ones, each point's normal widened by its normal ball.
// A dynamic pair's two impulses cancel in the world total.
func (w *World) islandContactImpulse(events []ContactEvent) (MomentumReading, bool) {
	var value, low, high [3]*big.Rat
	for axis := range value {
		value[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for _, event := range events {
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
			if lambda == nil || bound == nil || angle == nil || !ok {
				return MomentumReading{}, false
			}
			width := new(big.Rat).Mul(lambda, new(big.Rat).Add(bound, angle))
			for axis := range value {
				applied := new(big.Rat).Mul(new(big.Rat).Mul(lambda, normal[axis]), sign)
				value[axis].Add(value[axis], applied)
				low[axis].Add(low[axis], new(big.Rat).Sub(applied, width))
				high[axis].Add(high[axis], new(big.Rat).Add(applied, width))
			}
		}
	}
	return boundedMomentum(value, low, high)
}
