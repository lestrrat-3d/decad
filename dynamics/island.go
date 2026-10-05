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
// set continued on a persistent track is marked track. rounded is the
// relation ContactPair reads at the rounded event poses, ContactUndecided
// when the gather did not read it; §5.2 assigns the pair's continuation
// policy from it (§10.7).
type islandPair struct {
	key      int
	a, b     int
	manifold decad.ContactManifold
	at       decad.SweepInstant
	bracket  *decad.SweepInterval
	depth    *decad.ContactManifold
	track    bool
	bandEnd  bool    // a band track ended the slice at the event (§10.3)
	band     float64 // the band depth through the event when a band track ended the slice there (§10.3)
	rounded  decad.ContactRelation
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
	drive      map[int]driverMotion
	eventBase  int // events published earlier in the step
	islandBase int // islands solved earlier in the step
	work       *stepWork
}

// solvedEvent is the outcome of one event time's islands: the post-event
// state, the published events and island reports, the continuation policy of
// every gathered pair, and the pairs that leave the contact set because their
// post-event poses read Separated (§10.7).
type solvedEvent struct {
	post     State
	events   []ContactEvent
	islands  []IslandReport
	policies map[int]decad.SweepStartPolicy
	leave    map[int]struct{}
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
func (w *World) pairActive(pair islandPair, state State, drive map[int]driverMotion) (bool, bool) {
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
func (w *World) correctIsland(isl island, pre State, drive map[int]driverMotion) (map[int]r3.Vec, map[int]float64,
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
		allowance := outwardSum(w.step.ContactSlop.Base(), geometry, pair.band)
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
// L1 norms of an L2 quantity. A kinematic participant contributes its
// driver's field: the linear part at the world origin, and the angular part
// times the lever from the world origin.
func (w *World) closingSpeedUpper(pair islandPair, pre State, drive map[int]driverMotion) float64 {
	velocity := func(index int) ([3]float64, [3]float64) {
		if motion, ok := drive[index]; ok {
			v, omega := floatVec(motion.linear), floatVec(motion.angular)
			return [3]float64{v.X, v.Y, v.Z}, [3]float64{omega.X, omega.Y, omega.Z}
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
		// A driver's field turns about the world origin; a dynamic body
		// turns about its mass center.
		center, centerBound := r3.Vec{}, 0.0
		if w.bodies[index].definition.Role == Dynamic {
			center = pre.entries[index].Pose.Apply(w.bodies[index].mass.Center.Value)
			centerBound = 3 * w.bodies[index].mass.Center.Bound.Base()
		}
		lever := outwardSum(math.Abs(witness.Value.X-center.X), math.Abs(witness.Value.Y-center.Y),
			math.Abs(witness.Value.Z-center.Z), 3*witness.Bound.Base(), centerBound)
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
// gathered pair receives its continuation policy (§5.2, continuations).
func (w *World) solveIslands(ctx context.Context, in eventIslands,
	scheduled map[int]struct{}) (*solvedEvent, []StepDiagnostic, error) {
	var active, quiet, separated []islandPair
	out := &solvedEvent{post: in.pre.clone(), policies: map[int]decad.SweepStartPolicy{}, leave: map[int]struct{}{}}
	for _, pair := range in.gathered {
		if !w.drivenWithin(pair, in.drive) {
			// A pair whose driver has no exact velocity field cannot be
			// classified; it stays in the solve, which refuses it if an event
			// reaches its island.
			active = append(active, pair)
			continue
		}
		ok, valid := w.pairActive(pair, in.pre, in.drive)
		if !valid {
			return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(w.pairs[pair.key]),
				"event manifold cannot be read as exact intervals")}, nil
		}
		// §5 step 7: a gathered pair whose rounded poses read Overlapping
		// enters the solve whether or not a point closes, so its island's
		// correction removes the penetration (§10.8); when nothing closes the
		// impulses are zero and the island publishes only the correction.
		if !ok && pair.rounded != decad.ContactOverlapping {
			out.policies[pair.key] = decad.ContinueSeparatingTouch
			quiet = append(quiet, pair)
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
	push := correctionPush{allowance: map[int]float64{}, separating: map[int]struct{}{}, policies: out.policies,
		untouched: map[int]struct{}{}, relations: map[int]decad.ContactRelation{}}
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
					"an island with a kinematic participant whose driver has no exact velocity field has no solver yet")}, nil
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
		// VelocityResidual may continue under ContinueSeparatingTouch, which
		// proves the departure itself, once its corrected poses read exactly
		// Touching (continuations); every other one continues under
		// ContinueCertifiedTouch.
		for i, pair := range isl.pairs {
			out.policies[pair.key] = decad.ContinueCertifiedTouch
			if solution.separating[i] {
				out.policies[pair.key] = decad.ContinueSeparatingTouch
				push.separating[pair.key] = struct{}{}
				separated = append(separated, pair)
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
	if err := w.continuations(ctx, out, quiet, separated, moves, push.relations); err != nil {
		return nil, nil, err
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

// continuations completes §5.2's policies for the pairs whose continuation
// the relation at their post-event poses decides (docs/multibody-dynamics-design.md
// §10.7). A pair standing inside a ContactBand within PenetrationResidual is a
// contact for the step, and a band track over its support set is the only
// certificate that carries it, so:
//   - a solved pair the solve separates keeps ContinueSeparatingTouch only
//     when its corrected poses read exactly Touching; a ContactBand continues
//     it under ContinueCertifiedTouch, and Separated takes it out of the
//     contact set;
//   - a pair gathered at a band end or from a track that enters no solve
//     keeps ContinueCertifiedTouch while its rounded event poses read
//     Touching or such a band, and leaves the contact set when they read
//     Separated;
//   - any other pair that enters no solve continues under
//     ContinueCertifiedTouch when its rounded event poses read such a band.
//
// Every other reading keeps the policy solveIslands assigned. A body the
// correction moved reads the relation checkCorrections recorded at its final
// poses; an unmoved pair reads the relation the gather recorded at the rounded
// event poses, or ContactPair at them when the gather recorded none.
func (w *World) continuations(ctx context.Context, out *solvedEvent, quiet, separated []islandPair,
	moves map[int]r3.Vec, relations map[int]decad.ContactRelation) error {
	read := func(pair islandPair) (decad.ContactRelation, error) {
		_, movedA := moves[pair.a]
		_, movedB := moves[pair.b]
		if relation, ok := relations[pair.key]; ok && (movedA || movedB) {
			return relation, nil
		}
		if pair.rounded != decad.ContactUndecided && !movedA && !movedB {
			return pair.rounded, nil
		}
		if err := ctx.Err(); err != nil {
			return decad.ContactUndecided, err
		}
		contact, err := w.contactAt(ctx, pair, out.post)
		if err != nil {
			return decad.ContactUndecided, err
		}
		return w.continuationRelation(contact), nil
	}
	for _, pair := range separated {
		if out.policies[pair.key] != decad.ContinueSeparatingTouch {
			continue
		}
		relation, err := read(pair)
		if err != nil {
			return err
		}
		switch relation {
		case decad.ContactBand:
			out.policies[pair.key] = decad.ContinueCertifiedTouch
		case decad.ContactSeparated:
			delete(out.policies, pair.key)
			out.leave[pair.key] = struct{}{}
		}
	}
	for _, pair := range quiet {
		relation, err := read(pair)
		if err != nil {
			return err
		}
		continued := pair.track || pair.bandEnd
		switch {
		case relation == decad.ContactBand, continued && relation == decad.ContactTouching:
			out.policies[pair.key] = decad.ContinueCertifiedTouch
		case continued && relation == decad.ContactSeparated:
			delete(out.policies, pair.key)
			out.leave[pair.key] = struct{}{}
		}
	}
	return nil
}

// continuationRelation is the relation §5.2 reads off a contact report: a
// ContactBand counts only within PenetrationResidual (§10.4), and any band
// beyond it reads ContactUndecided, which keeps the assigned policy.
func (w *World) continuationRelation(contact *decad.ContactReport) decad.ContactRelation {
	if contact.Relation == decad.ContactBand && !w.contactBandWithin(contact.Gap) {
		return decad.ContactUndecided
	}
	return contact.Relation
}

// drivenWithin reports whether every kinematic body of a pair has its
// driver's exact velocity field in drive.
func (w *World) drivenWithin(pair islandPair, drive map[int]driverMotion) bool {
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
	// A kinematic participant reports its driver's field at its pose's
	// origin, and the field's angular velocity.
	velocity := func(index int, published QuantityVec) QuantityVec {
		if motion, ok := in.drive[index]; ok {
			at, ok := ratVec(pre.entries[index].Pose.Translation())
			if ok {
				return ratVelocity(motion.at(at))
			}
		}
		return published
	}
	angular := func(index int, published QuantityVec) QuantityVec {
		if motion, ok := in.drive[index]; ok && motion.rotates() {
			var out QuantityVec
			for axis, value := range motion.angular {
				f, _ := value.Float64()
				setVelocityComponent(&out, axis, units.RadiansPerSecond(f))
			}
			return out
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
		event.PreAngularVelocityA = angular(pair.a, pre.entries[pair.a].AngularVelocity)
		event.PreAngularVelocityB = angular(pair.b, pre.entries[pair.b].AngularVelocity)
		event.PostVelocityA = velocity(pair.a, solution.linear[sa])
		event.PostVelocityB = velocity(pair.b, solution.linear[sb])
		event.PostAngularVelocityA = angular(pair.a, solution.angular[sa])
		event.PostAngularVelocityB = angular(pair.b, solution.angular[sb])
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
// corrected poses, or, unless it rests on a persistent track, be pushed apart
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
// corrected body's correction allowance (a body without one is not pushed),
// the island pairs whose every point the solve separates, and the
// contact-set policies, which a separated pair leaves.
type correctionPush struct {
	allowance  map[int]float64
	separating map[int]struct{}
	policies   map[int]decad.SweepStartPolicy
	untouched  map[int]struct{}              // resting pairs restInTouch found no touch for
	relations  map[int]decad.ContactRelation // each checked pair's last corrected relation (continuations)
}

// correctedRelation checks one island pair at the corrected poses. A
// touching pair passes. A translation along a float normal rarely lands two
// curved bodies in exact touch (two spheres whose center line is not along an
// axis never touch exactly at float centers), so a pair the solve leaves
// resting is first placed back in touch where a touching pose exists
// (restInTouch). A pair the solve separates, and a resting pair with no
// touching pose that does not continue on a persistent track, is pushed apart
// (rigid-dynamics "Response": a separated corrected pose within the
// correction allowance) when it reads any of three ways:
//   - it still overlaps: its pushable bodies move along the deepest point's
//     normal by that point's depth plus its separation bound plus the pair's
//     margin, split by inverse mass and doubled until it moves the rounded
//     pose, and the caller checks the island again;
//   - it is apart by less than ContactPair can prove (Undecided with
//     ContactNoGapProof): its pushable bodies move apart along the event
//     manifold's normal, split by inverse mass, by the pair's margin or one
//     ulp of their largest coordinate, whichever is larger, and then twice as
//     far each time, until ContactPair proves the pair separated or touching;
//   - it is resting and apart by less than its margin: its pushable bodies
//     move along the event manifold's normal by the shortfall of the proved
//     gap.
//
// A pair the solve separates has no margin and ends just apart. A resting
// pair's margin is half of ContactSlop, which every pair's allowance carries:
// at an ulp apart, the smallest correction of a neighbor later in the step
// would reach it. A pushable body is a dynamic one with a correction
// allowance; a body the correction held in place (an anchored group, or a
// body no penetration moved) carries none and is not pushed either.
//
// A pushed pair that ends touching continues under ContinueSeparatingTouch;
// one that ends separated leaves the contact set, so its next slice starts
// under StopAtInitialContact. A resting pair on a persistent track is never
// pushed apart and may not end separated: its track needs exact touch. Every
// moved body's whole translation from its pre-event pose, measured as the
// correction's is, must stay within its correction allowance. It reports
// false when it moved a body, so the caller checks the island again; allowed
// is false on the last pass, where a pair that still needs a move is refused
// (a resting pair apart by less than its margin is then left as it is).
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
	// A pass that moves nothing reads every checked pair at its final poses,
	// so the last reading is the post-event relation §5.2 continues from.
	push.relations[pair.key] = w.continuationRelation(contact)
	if w.touchWithin(contact) {
		return true, nil, nil
	}
	_, separating := push.separating[pair.key]
	_, untouched := push.untouched[pair.key]
	if !separating && !untouched && allowed && len(pair.manifold.Points) != 0 {
		touched, err := w.restInTouch(ctx, pre, post, moves, pair, push, contact)
		if err != nil || touched {
			return !touched, nil, err
		}
		push.untouched[pair.key] = struct{}{}
	}
	pushable := separating || !pair.track
	margin := 0.0
	if !separating {
		margin = w.step.ContactSlop.Base() / 2
	}
	switch {
	case contact.Relation == decad.ContactSeparated && pushable && margin > 0 && allowed &&
		len(pair.manifold.Points) != 0 && contact.Gap != nil &&
		contact.Gap.Value.Base()-contact.Gap.Bound.Base() < margin:
		// A resting pair the correction left apart by less than its margin
		// is pushed out to it along the event manifold's normal.
		poses, diagnostic := w.pushPoses(pre, post, pair, pair.manifold.Points[0].Normal.Value,
			margin-(contact.Gap.Value.Base()-contact.Gap.Bound.Base()), push, true)
		if diagnostic != nil {
			return true, diagnostic, nil
		}
		applyPoses(pre, post, moves, poses)
		return false, nil, nil
	case contact.Relation == decad.ContactSeparated && pushable:
		// A separated pair leaves the contact set: its next slice starts
		// clear, under StopAtInitialContact, where ContinueSeparatingTouch
		// needs a touching start.
		delete(push.policies, pair.key)
		return true, nil, nil
	case contact.Relation == decad.ContactUndecided && contact.Reason == decad.ContactNoGapProof &&
		pushable && allowed && len(pair.manifold.Points) != 0:
		return w.pushProvablyApart(ctx, pre, post, moves, pair, push, margin)
	case contact.Relation != decad.ContactOverlapping || !pushable || !allowed ||
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
		return fail("an overlapping pushable pair has no push")
	}
	poses, diagnostic := w.pushPoses(pre, post, pair, normal, outwardSum(depth, margin), push, true)
	if diagnostic != nil {
		return true, diagnostic, nil
	}
	applyPoses(pre, post, moves, poses)
	return false, nil, nil
}

// pushProvablyApart moves a pair that is apart by less than ContactPair can
// prove along its event manifold's normal, by margin or one ulp of the larger
// body coordinate, whichever is larger, and then twice as far each time, until
// ContactPair proves it separated or touching. It reports false when it
// pushed.
func (w *World) pushProvablyApart(ctx context.Context, pre, post State, moves map[int]r3.Vec, pair islandPair,
	push correctionPush, margin float64) (bool, *StepDiagnostic, error) {
	normal := pair.manifold.Points[0].Normal.Value
	largest := 0.0
	for _, index := range [2]int{pair.a, pair.b} {
		t := post.entries[index].Pose.Translation()
		largest = math.Max(largest, math.Max(math.Abs(t.X), math.Max(math.Abs(t.Y), math.Abs(t.Z))))
	}
	amount := math.Max(math.Nextafter(largest, math.Inf(1))-largest, margin)
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
			applyPoses(pre, post, moves, poses)
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

// applyPoses sets pushed poses in post and records each moved body's whole
// translation from its pre-event pose.
func applyPoses(pre, post State, moves map[int]r3.Vec, poses map[int]r3.Transform) {
	for index, pose := range poses {
		post.entries[index].Pose = pose
		moves[index] = pose.Translation().Sub(pre.entries[index].Pose.Translation())
	}
}

// touchWithin reports whether a contact report is a touch: Touching, or a
// band within PenetrationResidual (§10.4).
func (w *World) touchWithin(contact *decad.ContactReport) bool {
	return contact.Relation == decad.ContactTouching ||
		contact.Relation == decad.ContactBand && w.contactBandWithin(contact.Gap)
}

// restSearchLimit bounds the probes of each half of restInTouch's search.
const restSearchLimit = 128

// restInTouch places a resting pair whose corrected poses overlap or stand
// apart back in touch: it moves the pair's pushable bodies along the event
// manifold's normal, split by inverse mass as pushPoses does (apart for a
// positive amount, together for a negative one), and asks ContactPair at
// each trial. From the corrected poses it steps away from their relation —
// apart by the overlap's depth plus its bound when they overlap, together by
// the gap plus its bound (or one ulp of the larger body coordinate when the
// gap cannot be proved) when they do not — doubling the step until the
// relation changes, then halves the interval between the last overlapping
// and the last non-overlapping amount until the two are adjacent floats. The
// first trial ContactPair proves touching ends the search: it is applied and
// reported true. A trial outside the correction allowance, or of any other
// relation, ends it unapplied. Two curved bodies may have no touching pose
// at float coordinates at all (two spheres whose center line is off every
// axis never do); the search then reports false and moves nothing.
func (w *World) restInTouch(ctx context.Context, pre, post State, moves map[int]r3.Vec, pair islandPair,
	push correctionPush, contact *decad.ContactReport) (bool, error) {
	normal := pair.manifold.Points[0].Normal.Value
	const (
		over = iota
		apart
		touch
		other
	)
	classify := func(report *decad.ContactReport) int {
		switch {
		case w.touchWithin(report):
			return touch
		case report.Relation == decad.ContactOverlapping:
			return over
		case report.Relation == decad.ContactSeparated,
			report.Relation == decad.ContactUndecided && report.Reason == decad.ContactNoGapProof:
			return apart
		}
		return other
	}
	trial := post.clone()
	var found map[int]r3.Transform
	probe := func(amount float64) (int, error) {
		poses, diagnostic := w.pushPoses(pre, post, pair, normal, amount, push, false)
		if diagnostic != nil {
			return other, nil
		}
		for index := range poses {
			trial.entries[index].Pose = poses[index]
		}
		if err := ctx.Err(); err != nil {
			return other, err
		}
		report, err := w.contactAt(ctx, pair, trial)
		for index := range poses {
			trial.entries[index].Pose = post.entries[index].Pose
		}
		if err != nil {
			return other, err
		}
		class := classify(report)
		if class == touch {
			found = poses
		}
		return class, nil
	}
	start := classify(contact)
	var step float64
	switch start {
	case over:
		if !w.manifoldWithin(contact.Manifold) {
			return false, nil
		}
		for _, p := range contact.Manifold.Points {
			step = math.Max(step, outwardSum(-p.Separation.Value.Base(), p.Separation.Bound.Base()))
		}
	case apart:
		if contact.Gap != nil {
			step = -outwardSum(contact.Gap.Value.Base(), contact.Gap.Bound.Base())
		}
		if !(step < 0) {
			largest := 0.0
			for _, index := range [2]int{pair.a, pair.b} {
				t := post.entries[index].Pose.Translation()
				largest = math.Max(largest, math.Max(math.Abs(t.X), math.Max(math.Abs(t.Y), math.Abs(t.Z))))
			}
			step = largest - math.Nextafter(largest, math.Inf(1))
		}
	default:
		return false, nil
	}
	if !finite(step) || step == 0 {
		return false, nil
	}
	// overSide and apartSide hold the last amounts of each relation.
	overSide, apartSide := 0.0, 0.0
	for range restSearchLimit {
		class, err := probe(step)
		if err != nil {
			return true, err
		}
		if class == other {
			return false, nil
		}
		if class == touch {
			applyPoses(pre, post, moves, found)
			return true, nil
		}
		if class != start {
			if class == over {
				overSide = step
			} else {
				apartSide = step
			}
			break
		}
		step *= 2
	}
	if overSide == apartSide {
		return false, nil
	}
	for range restSearchLimit {
		mid := overSide + (apartSide-overSide)/2
		if mid == overSide || mid == apartSide {
			return false, nil
		}
		class, err := probe(mid)
		if err != nil {
			return true, err
		}
		switch class {
		case touch:
			applyPoses(pre, post, moves, found)
			return true, nil
		case over:
			overSide = mid
		case apart:
			apartSide = mid
		default:
			return false, nil
		}
	}
	return false, nil
}

// contactAt queries ContactPair for an island pair at a state's poses.
func (w *World) contactAt(ctx context.Context, pair islandPair, state State) (*decad.ContactReport, error) {
	return w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body, w.bodies[pair.b].definition.Body,
		state.entries[pair.a].Pose, state.entries[pair.b].Pose, w.step.Contact)
}

// pushPoses moves a pair's pushable bodies apart along normal by amount,
// split by inverse mass (A against the normal, B along it). A pushable body
// is a dynamic one with a correction allowance: a body the correction held
// in place, such as an anchored group whose exact touch with its support a
// move would break, carries none and stays put. With grow set a share too
// small to move a rounded pose doubles until it does. Every moved body's
// whole translation from its pre-event pose, measured as the correction's
// is, must stay within its correction allowance. It returns the new pose of
// each moved body by world index.
func (w *World) pushPoses(pre, post State, pair islandPair, normal r3.Vec, amount float64, push correctionPush,
	grow bool) (map[int]r3.Transform, *StepDiagnostic) {
	fail := func(reason string, limit units.Value) (map[int]r3.Transform, *StepDiagnostic) {
		d := scheduleDiagnostic(StepCorrectionFailed, w.bodyPair(w.pairs[pair.key]), reason)
		d.Limit = limit
		return nil, &d
	}
	var inverse [2]float64
	for side, index := range [2]int{pair.a, pair.b} {
		if _, ok := push.allowance[index]; ok && w.bodies[index].definition.Role == Dynamic {
			inverse[side] = 1 / w.bodies[index].mass.Mass.Value.Base()
		}
	}
	total := inverse[0] + inverse[1]
	if !finite(amount, total) || amount == 0 || total <= 0 {
		return fail("a pushed pair has no body free to move", units.Value{})
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
// velocity at the point's witness: v + ω × (witness − pose origin). The
// normal ball widens the reading by λ·|V|₁·(bound + angle), and the witness
// ball by |J|₁·|ω|₁·3·(witness bound).
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
			velocity, spin, pose := event.PreVelocityA, event.PreAngularVelocityA, event.PoseA
			sign := big.NewRat(1, 1)
			if side == 1 {
				velocity, spin, pose, sign = event.PreVelocityB, event.PreAngularVelocityB, event.PoseB, big.NewRat(-1, 1)
			}
			origin, okOrigin := quantityRats(velocity)
			omega, okOmega := quantityRats(spin)
			at, okAt := ratVec(pose.Translation())
			if !okOrigin || !okOmega || !okAt || len(event.PointImpulses) != len(event.Manifold.Points) {
				return decad.Measurement{}, false
			}
			rate := new(big.Rat)
			for _, component := range omega {
				rate.Add(rate, absRat(new(big.Rat).Set(component)))
			}
			for i, point := range event.Manifold.Points {
				lambda := exactBase(event.PointImpulses[i].Normal)
				bound, angle := exactBase(point.Normal.Bound), exactBase(point.NormalAngle)
				normal, okNormal := ratVec(point.Normal.Value)
				tangent, okTangent := quantityRats(event.PointImpulses[i].Tangent)
				witness := point.OnA
				if side == 1 {
					witness = point.OnB
				}
				x, okX := ratVec(witness.Value)
				witnessBound := exactBase(witness.Bound)
				if lambda == nil || bound == nil || angle == nil || !okNormal || !okTangent || !okX || witnessBound == nil {
					return decad.Measurement{}, false
				}
				for axis := range x {
					x[axis].Sub(x[axis], at[axis])
				}
				v := driverMotion{linear: origin, angular: omega}.at(x)
				speed := new(big.Rat)
				for _, component := range v {
					speed.Add(speed, absRat(new(big.Rat).Set(component)))
				}
				work := new(big.Rat).Set(proof.DotInterval3(pointIVec(normal), pointIVec(v)).Lo)
				work.Mul(work, lambda)
				work.Add(work, proof.DotInterval3(pointIVec(tangent), pointIVec(v)).Lo)
				work.Mul(work, sign)
				width := new(big.Rat).Mul(absRat(new(big.Rat).Set(lambda)), speed)
				width.Mul(width, new(big.Rat).Add(bound, angle))
				impulse := absRat(new(big.Rat).Set(lambda))
				for _, component := range tangent {
					impulse.Add(impulse, absRat(new(big.Rat).Set(component)))
				}
				spread := new(big.Rat).Mul(impulse, rate)
				spread.Mul(spread, new(big.Rat).Mul(big.NewRat(3, 1), witnessBound))
				width.Add(width, spread)
				value.Add(value, work)
				low.Add(low, new(big.Rat).Sub(work, width))
				high.Add(high, new(big.Rat).Add(work, width))
			}
		}
	}
	return boundedReading(value, low, high, units.KilogramSquareMillimeterPerSecondSquared)
}
