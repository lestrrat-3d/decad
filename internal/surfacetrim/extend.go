package surfacetrim

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ExtendPrism widens, for each of b's edges, the recorded end that edge names
// until it meets tl (agent-docs/surface-intersection-design.md §3.2), and returns
// rcv with a fresh walk set and its SectionDelta set to the largest δ_cut any
// widening read. rcv is b's own record as AdmitExtendPair returned it; its
// walk set is copied, never written.
func ExtendPrism(ctx context.Context, budget *proofbound.WorkBudget, b *topo.Body,
	rcv chainrecord.Payload, tl prismrecord.Payload, edges []*topo.Edge,
) (chainrecord.Payload, error) {
	chains, cutDelta, err := widenEnds(ctx, budget, rcv.Chains, rcv.Prism(), tl, edges,
		func(edge *topo.Edge) (int, int, bool, error) { return endSegment(b, rcv.Chains, edge) })
	if err != nil {
		return chainrecord.Payload{}, err
	}
	rcv.Chains = chains
	rcv.SectionDelta = cutDelta
	return rcv, nil
}

// ExtendRevolve is ExtendPrism over a pair S1 routed to the revolve family.
// Everything past the gate is the prism arm's own: ResolveSurfaceExtend reads
// sketch's cut parameter off the recreated carrier in the two operands'
// MERIDIAN views rcvView and tlView (§3.1), as revolveplan.AdmitExtendPair
// returned them, and the widened range goes back into the receiver's own
// record. Only the free-edge reading differs. §7.1's fold reads SectionDelta as
// its gate, and ResolveSurfaceExtend's δ_cut is the whole of it: S4 and S7 zero
// every other term prism §7 derives.
func ExtendRevolve(ctx context.Context, budget *proofbound.WorkBudget, b *topo.Body,
	rcv revolveplan.ChainPayload, rcvView, tlView prismrecord.Payload, edges []*topo.Edge,
) (revolveplan.ChainPayload, error) {
	chains, cutDelta, err := widenEnds(ctx, budget, rcv.Chains, rcvView, tlView, edges,
		func(edge *topo.Edge) (int, int, bool, error) { return revolveEndSegment(b, rcv.Chains, edge) })
	if err != nil {
		return revolveplan.ChainPayload{}, err
	}
	rcv.Chains = chains
	rcv.SectionDelta = cutDelta
	return rcv, nil
}

// widenEnds copies chains, then widens the segment endOf names for each edge in
// turn through prismcells.ResolveSurfaceExtend over the two views. It returns
// the copy and the largest δ_cut any widening read. The first refusal, of
// either endOf or the resolution, ends the walk.
func widenEnds(ctx context.Context, budget *proofbound.WorkBudget, chains []sectionrecord.ChainRecord,
	rcvView, tlView prismrecord.Payload, edges []*topo.Edge,
	endOf func(*topo.Edge) (int, int, bool, error),
) ([]sectionrecord.ChainRecord, float64, error) {
	out := make([]sectionrecord.ChainRecord, len(chains))
	for i, chain := range chains {
		out[i] = sectionrecord.ChainRecord{Segments: append([]sectionrecord.CurveSegment(nil), chain.Segments...)}
	}
	cutDelta := 0.0
	for _, edge := range edges {
		ci, si, atStart, err := endOf(edge)
		if err != nil {
			return nil, 0, err
		}
		widened, delta, err := prismcells.ResolveSurfaceExtend(ctx, budget,
			prismcells.SurfaceOperandOf(rcvView), prismcells.SurfaceOperandOf(tlView), out[ci].Segments[si], atStart)
		if err != nil {
			return nil, 0, err
		}
		out[ci].Segments[si] = widened
		cutDelta = math.Max(cutDelta, delta)
	}
	return out, cutDelta, nil
}

// endSegment maps a selected topology edge to the recorded end it names: the
// chain index, the segment index, and whether the end is that segment's start.
// The ribbon builder places the free sweep edges at the first and last wall's
// vertical sides; it preserves walk order through the one-lump-per-chain build.
//
// The loop indexes b.Lumps and reads chains at the SAME index, which assumes
// the body's i-th lump is the record's i-th chain. That holds because every
// chain record a body can hold is built by chainrecord.Build — for chain
// Extrude (root extrude.go), root Trim and root Extend, and for a placed
// copy's re-evaluation (chainrecord.Payload.Placed) — which builds exactly one
// lump per chain in chain order. A trimmed body is therefore read on the same
// terms whatever SectionDelta it recorded; S7 refuses one whose SectionDelta
// is nonzero before this reading runs, and admits one whose SectionDelta is
// zero, as it is when every surviving fragment was whole. What would
// break it: a producer that emits a lump per connected PIECE rather than per
// chain (one self-touching walk splitting into two lumps, or two walks meeting
// at a vertex merging into one), or one that reorders lumps. Either would
// silently widen a bound on the wrong chain, so a producer adding that shape
// owes this function a recorded lump-to-chain map rather than the shared index.
func endSegment(b *topo.Body, chains []sectionrecord.ChainRecord, edge *topo.Edge) (int, int, bool, error) {
	miss := func() (int, int, bool, error) {
		return 0, 0, false, fmt.Errorf(`%w: Extend needs a free sweep edge at an open section end`, decaderr.ErrUnsupported)
	}
	if len(edge.Faces) != 1 {
		return miss()
	}
	for ci, lump := range b.Lumps {
		if ci >= len(chains) || len(lump.Shells) != 1 || len(lump.Shells[0].Faces) == 0 {
			continue
		}
		faces := lump.Shells[0].Faces
		first := faces[0].Loops[0].CoEdges
		last := faces[len(faces)-1].Loops[0].CoEdges
		if edge == first[3].Edge {
			return ci, 0, true, nil
		}
		if edge == last[1].Edge {
			return ci, len(chains[ci].Segments) - 1, false, nil
		}
	}
	return miss()
}

// revolveEndSegment maps a selected topology edge to the recorded meridian end
// it names — endSegment's own reading, re-derived for the chain-revolve build,
// whose free ends are SWEPT latitude circles rather than straight sweep edges.
//
// Under a full revolution buildChainRevolveWalls gives every wall face its
// junctions' latitude circles as SEPARATE single-coedge loops through
// fullRevLoops (both in internal/revolveplan/build.go), which sets forward true
// for the wall's own START junction and false for its END. That flag is the
// map: the receiver's two free ends are junction 0 of the FIRST wall and
// junction n of the LAST, and reading the flag rather than the loop position is
// what makes the reading survive fullRevLoops' own outer-loop swap. The first
// and last wall are never the skipped revolveaxis.WallAxis kind — a segment lying on the axis puts
// both its ends at ρ == 0, which revolveaxis.RequireChainAxisIncidence already
// refuses at a chain's free end — so faces[0] and faces[len-1] are walk 0 and
// walk n-1.
//
// The loop indexes b.Lumps and reads chains at the SAME index, on endSegment's
// own terms: revolveplan.BuildChain appends each chain's faces in chain order
// and topobuild.SheetLumps preserves first-appearance order, so the body's i-th
// lump is the record's i-th chain. A producer that emits a lump per connected
// PIECE rather than per chain owes this function a recorded map rather than
// the shared index.
func revolveEndSegment(b *topo.Body, chains []sectionrecord.ChainRecord, edge *topo.Edge) (int, int, bool, error) {
	miss := func() (int, int, bool, error) {
		return 0, 0, false, fmt.Errorf(`%w: Extend needs a free swept edge at an open meridian end`, decaderr.ErrUnsupported)
	}
	if len(edge.Faces) != 1 {
		return miss()
	}
	for ci, lump := range b.Lumps {
		if ci >= len(chains) || len(lump.Shells) != 1 || len(lump.Shells[0].Faces) == 0 {
			continue
		}
		faces := lump.Shells[0].Faces
		if named, ok := latitudeLoopEdge(faces[0], true); ok && named == edge {
			return ci, 0, true, nil
		}
		if named, ok := latitudeLoopEdge(faces[len(faces)-1], false); ok && named == edge {
			return ci, len(chains[ci].Segments) - 1, false, nil
		}
	}
	return miss()
}

// latitudeLoopEdge reads one wall face's own latitude loop for the junction
// fullRevLoops' forward flag names: true for the wall's start junction, false
// for its end. A junction ON the axis mints no latitude circle at all, so that
// end answers absent and the caller refuses.
func latitudeLoopEdge(f *topo.Face, start bool) (*topo.Edge, bool) {
	for _, l := range f.Loops {
		if len(l.CoEdges) != 1 || l.CoEdges[0].Forward != start {
			continue
		}
		return l.CoEdges[0].Edge, true
	}
	return nil, false
}
