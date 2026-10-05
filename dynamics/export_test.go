package dynamics

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad"
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
// (canonical pair order, then manifold order). A tamper function may change
// any value before the certificate reads it.
type IslandProposal struct {
	Bodies  []*decad.Body
	Linear  []QuantityVec
	Angular []QuantityVec
	Lambda  []float64
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
	sweeps, report, err := w.sweepSlice(ctx, kicked, make([]decad.PoseSegment, len(w.bodies)), dt,
		w.pairSchedule(), nil)
	if err != nil || report != nil {
		return nil, ErrUnsupported
	}
	gathered, diagnostics := w.gatherInitialContacts(sweeps)
	if len(diagnostics) != 0 {
		return nil, ErrUnsupported
	}
	var active []islandPair
	for _, pair := range gathered {
		if ok, valid := w.pairActive(pair, kicked); ok && valid {
			active = append(active, pair)
		}
	}
	islands, diagnostic := w.formIslands(active)
	if diagnostic != nil || len(islands) == 0 {
		return nil, ErrUnsupported
	}
	isl := islands[0]
	solution, failure := w.solveIsland(isl, kicked)
	if failure != nil {
		return nil, ErrUnsupported
	}
	slots := make(map[int]int, len(isl.bodies))
	proposal := IslandProposal{Linear: slices.Clone(solution.linear), Angular: slices.Clone(solution.angular),
		Lambda: slices.Clone(solution.lambda)}
	for slot, index := range isl.bodies {
		slots[index] = slot
		proposal.Bodies = append(proposal.Bodies, w.bodies[index].definition.Body)
	}
	bodies, failure := w.nominalBodies(isl, kicked)
	if failure != nil {
		return nil, ErrUnsupported
	}
	points, failure := w.nominalPoints(isl, slots, bodies)
	if failure != nil {
		return nil, ErrUnsupported
	}
	tamper(&proposal)
	solution.linear, solution.angular, solution.lambda = proposal.Linear, proposal.Angular, proposal.Lambda
	cert, failure := w.certifyProposal(isl, kicked, points, solution)
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
