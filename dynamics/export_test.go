package dynamics

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// TracePairProof is one scheduled pair's certificate in a trace slice, read
// by the external schedule tests: the pair's sweep, or the swept-box
// exclusion that made no sweep necessary.
type TracePairProof struct {
	Pair     BodyPair
	BoxClear bool
	Sweep    *decad.SweepReport
}

// TraceSliceProofs returns the scheduled-pair certificates of each slice of
// a trace of a world of four or more bodies, in canonical pair order.
func TraceSliceProofs(tr Trace) [][]TracePairProof {
	out := make([][]TracePairProof, len(tr.slices))
	for i, slice := range tr.slices {
		for _, proof := range slice.proofs {
			out[i] = append(out[i], TracePairProof{Pair: slice.from.world.bodyPair(slice.from.world.pairs[proof.pair]),
				BoxClear: proof.boxClear, Sweep: proof.sweep})
		}
	}
	return out
}

// IslandProposal is the published proposal of one island, in the island's
// body order (world order, Fixed participants included) and point order
// (canonical pair order, then manifold order); Tangent holds each point's
// world tangent impulse on B. A tamper function may change any value before
// the certificate reads it.
type IslandProposal struct {
	Bodies  []*decad.Body
	Linear  []QuantityVec
	Angular []QuantityVec
	Lambda  []float64
	Tangent []r3.Vec
}

// IslandProposalGates runs a step's kick and first slice through the real
// producers, forms the islands of its initial contacts, and solves the
// first island to its certified proposal. It then lets tamper change that
// proposal and returns the names of every certificate gate that refuses the
// tampered values, in table order. An empty list means the certificate
// passes.
func IslandProposalGates(ctx context.Context, w *World, from State, gravity QuantityVec, dt units.Value,
	tamper func(*IslandProposal)) ([]string, error) {
	kicked, ok := w.kickByLoads(from, gravity, make([]*BodyLoad, len(w.bodies)), dt)
	if !ok {
		return nil, ErrUnsupported
	}
	paths := w.slicePaths(kicked, make([]decad.PoseSegment, len(w.bodies)), dt)
	sweeps, diagnostics, err := w.sweepSlice(ctx, directWork(w), paths, dt, w.pairSchedule(), nil)
	if err != nil || len(diagnostics) != 0 {
		return nil, ErrUnsupported
	}
	var active []islandPair
	for _, key := range sweeps.candidates {
		sweep, pair := sweeps.swept[key], w.pairs[key]
		if sweep.Outcome != decad.SweepInitiallyTouching && sweep.Outcome != decad.SweepInitiallyOverlapping {
			continue
		}
		item := islandPair{key: key, a: pair.a, b: pair.b, manifold: cloneManifold(*sweep.InitialEvent.Manifold),
			at: sweep.InitialEvent.At}
		if ok, valid := w.pairActive(item, kicked, nil); ok && valid {
			active = append(active, item)
		}
	}
	islands, diagnostic := w.formIslands(active)
	if diagnostic != nil || len(islands) == 0 {
		return nil, ErrUnsupported
	}
	isl := islands[0]
	solution, failure := w.solveIsland(isl, kicked, nil, directWork(w))
	if failure != nil {
		return nil, ErrUnsupported
	}
	slots := make(map[int]int, len(isl.bodies))
	proposal := IslandProposal{Linear: slices.Clone(solution.linear), Angular: slices.Clone(solution.angular),
		Lambda: slices.Clone(solution.lambda), Tangent: slices.Clone(solution.tangent)}
	for slot, index := range isl.bodies {
		slots[index] = slot
		proposal.Bodies = append(proposal.Bodies, w.bodies[index].definition.Body)
	}
	bodies, failure := w.nominalBodies(isl, kicked, nil)
	if failure != nil {
		return nil, ErrUnsupported
	}
	points, failure := w.nominalPoints(isl, slots, bodies)
	if failure != nil {
		return nil, ErrUnsupported
	}
	tamper(&proposal)
	solution.linear, solution.angular, solution.lambda = proposal.Linear, proposal.Angular, proposal.Lambda
	solution.tangent = proposal.Tangent
	cert, failure := w.certifyProposal(isl, kicked, points, solution, nil)
	if failure != nil {
		return nil, ErrUnsupported
	}
	var names []string
	for gate := gateLinearLaw; gate <= gateAngularMomentum; gate++ {
		if _, ok := cert.refused[gate]; ok {
			names = append(names, gate.String())
		}
	}
	return names, nil
}

// CertifiedInertiaFloor is the certified lower inertia eigenvalue λ_lo the
// island certificate reads for body, exact; nil when body is not in w.
func CertifiedInertiaFloor(w *World, body *decad.Body) *big.Rat {
	index, ok := w.index[body]
	if !ok {
		return nil
	}
	return certifiedInertiaFloor(w.bodies[index].mass)
}

// TracePairCalls is the number of SweptBox and SweepPair calls the step that
// recorded a trace made, reused certificates excluded.
func TracePairCalls(tr Trace) uint64 {
	return tr.pairCalls
}

// WithoutCache returns s without its reuse cache; its contact set stays. A
// step from it recomputes every certificate and solves every island cold.
func WithoutCache(s State) State {
	s.cache = nil
	return s
}

// PushApart runs correctedRelation's passes on the pair (a, b) of state,
// which the solve is taken to separate, with each body of allowances given
// that correction allowance. normal, when nonzero, is the event manifold's
// normal (A to B). pre is the state before the correction. It returns the
// refusal reason, or "" and the pushed state when the push fits.
func PushApart(ctx context.Context, w *World, pre, state State, a, b *decad.Body,
	allowances map[*decad.Body]units.Value, normal r3.Vec) (string, State, error) {
	reason, post, _, err := settlePair(ctx, w, pre, state, a, b, allowances, normal, true, false)
	return reason, post, err
}

// SettleResting runs correctedRelation's passes on the pair (a, b) of state
// as PushApart does, for a pair the solve leaves resting (it does not
// separate it), continued on a persistent track when track is set. It also
// reports whether the pair stays in the contact set (false when it ends
// apart and leaves it).
func SettleResting(ctx context.Context, w *World, pre, state State, a, b *decad.Body,
	allowances map[*decad.Body]units.Value, normal r3.Vec, track bool) (string, State, bool, error) {
	return settlePair(ctx, w, pre, state, a, b, allowances, normal, false, track)
}

func settlePair(ctx context.Context, w *World, pre, state State, a, b *decad.Body,
	allowances map[*decad.Body]units.Value, normal r3.Vec, separating, track bool) (string, State, bool, error) {
	key, ok := lookupPair(w.index, BodyPair{A: a, B: b})
	if !ok {
		return "", State{}, false, ErrInvalidInput
	}
	post := state.clone()
	moves := map[int]r3.Vec{}
	push := correctionPush{allowance: map[int]float64{}, separating: map[int]struct{}{},
		policies: map[int]decad.SweepStartPolicy{key: decad.ContinueCertifiedTouch}, untouched: map[int]struct{}{},
		relations: map[int]decad.ContactRelation{}}
	if separating {
		push.separating[key] = struct{}{}
	}
	for body, allowance := range allowances {
		push.allowance[w.index[body]] = allowance.Base()
	}
	pair := islandPair{key: key, a: w.pairs[key].a, b: w.pairs[key].b, track: track}
	if normal != (r3.Vec{}) {
		pair.manifold.Points = []decad.ContactPoint{{Normal: decad.VecMeasurement{Value: normal}}}
	}
	for pass := 0; ; pass++ {
		done, diagnostic, err := w.correctedRelation(ctx, pre, post, moves, pair, push, pass < pushLimit)
		if err != nil {
			return "", State{}, false, err
		}
		if diagnostic != nil {
			return diagnostic.Reason, State{}, false, nil
		}
		if done {
			_, kept := push.policies[key]
			return "", post, kept, nil
		}
	}
}

// TraceSliceTimes returns each slice's held start, end and span (the end of
// its paths and certificates) of a trace of a world of four or more bodies,
// in slice order.
func TraceSliceTimes(tr Trace) [][3]units.Value {
	out := make([][3]units.Value, len(tr.slices))
	for i, slice := range tr.slices {
		out[i] = [3]units.Value{slice.start, slice.end, slice.span}
	}
	return out
}

// KinematicWork is the driver work reading islandKinematicWork makes of
// published events, the reading an advanced step reports as
// StepConservation.KinematicWork.
func KinematicWork(w *World, events []ContactEvent) (decad.Measurement, bool) {
	return w.islandKinematicWork(events)
}
