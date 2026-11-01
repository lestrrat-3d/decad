package meshbool

import (
	"context"
	"math/big"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type brakeTriCutter struct {
	Verts          []CutVert
	Edges          []CutEdge
	Swap           bool // projection coordinate swap that keeps the facet CCW
	U, V           int  // projection axes
	Work           *proofbound.WorkBudget
	Reserve        func(uint64) error
	raw            cutRawIndex
	seen           map[[2]int]struct{}
	anchors        map[[2]int]ChainAnchor
	replayCtx      context.Context //nolint:containedctx // One bounded cutter call.
	replaySeen     *replayTable[[2]int, struct{}]
	replayAnchors  *replayTable[[2]int, ChainAnchor]
	replayBudget   *PairCutsBudget
	replayVert     *replayTable[string, int]
	replayZero     big.Int
	regionPoints   []Xp2
	contactRefusal BooleanExpectedError
}

var replayHalf = big.NewRat(1, 2)

const brakeReplayCutterLimit = 65536

func cutTriangleSharedWork(ctx context.Context, xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg, budget *PairCutsBudget) ([]CutRegion, error) {
	tc, err := newBrakeTriCutter(ctx, len(segs), budget)
	if err != nil {
		return nil, err
	}
	return tc.cut(ctx, xtri, normal, segs)
}

func newBrakeTriCutter(ctx context.Context, segmentCount int, budget *PairCutsBudget) (*brakeTriCutter, error) {
	if budget == nil || budget.replay == nil {
		return nil, errReplayCutterNeedsTheSharedReplayBudget
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 8); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, 1, uint64(unsafe.Sizeof(brakeTriCutter{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, 1, uint64(unsafe.Sizeof(proofbound.WorkBudget{}))+256); err != nil {
		return nil, err
	}
	tc := &brakeTriCutter{replayCtx: ctx, replayBudget: budget}
	var err error
	tc.replayVert, err = newReplayGrowingTable[string, int](ctx, brakeReplayCutterLimit, budget.replay, replayStringHash)
	if err != nil {
		return nil, err
	}
	tc.replayVert.comparisonWork = func(s string) uint64 { return uint64(len(s)) }
	tc.replaySeen, err = newReplayGrowingTable[[2]int, struct{}](ctx, segmentCount, budget.replay, replayEdgeHash)
	if err != nil {
		return nil, err
	}
	tc.Reserve = func(n uint64) error { return budget.ReserveRecords(ctx, n) }
	tc.Work = &proofbound.WorkBudget{Replay: true, ReplayID: budget.replay, StepFn: func() error { return budget.SpendWork(ctx, 1) }, ReserveFn: func(n uint64) error { return budget.ReserveRecords(ctx, n) }, ReserveBytesFn: func(n uint64) error { return budget.ReserveBytes(ctx, n, 1) }, ErrFn: ctx.Err}
	return tc, nil
}

func (tc *brakeTriCutter) projectChecked(p proof.Xpt, u, v int) (Xp2, error) {
	if tc.replayBudget == nil || tc.replayBudget.replay == nil {
		return Xp2{}, errReplayMissingReplayCutterBudget
	}
	return replayProjection(tc.replayCtx, p, u, v, tc.replayBudget.replay)
}

func (tc *brakeTriCutter) ratMidpoint(a, b *big.Rat) (*big.Rat, error) {
	if tc.replayBudget == nil {
		return nil, errReplayMissingReplayCutterBudget
	}
	sum, err := proof.ReplayRatArithmetic(tc.replayCtx, a, b, '+', tc.replayBudget.replay)
	if err != nil {
		return nil, err
	}
	return proof.ReplayRatArithmetic(tc.replayCtx, sum, replayHalf, '*', tc.replayBudget.replay)
}

func (tc *brakeTriCutter) ratCompare(a, b *big.Rat) (int, error) {
	if tc.replayBudget == nil || tc.replayBudget.replay == nil {
		return 0, errReplayMissingReplayCutterBudget
	}
	return proof.ReplayRatCompare(tc.replayCtx, a, b, tc.replayBudget.replay)
}

func (tc *brakeTriCutter) splitAtU(a, b CutVert, c *big.Rat) (Xp2, proof.Xpt, error) {
	if tc.replayBudget == nil || tc.replayBudget.replay == nil {
		return Xp2{}, proof.Xpt{}, errReplayMissingReplayCutterBudget
	}
	budget := tc.replayBudget.replay
	num, err := proof.ReplayRatArithmetic(tc.replayCtx, c, a.P2.U, '-', budget)
	if err != nil {
		return Xp2{}, proof.Xpt{}, err
	}
	den, err := proof.ReplayRatArithmetic(tc.replayCtx, b.P2.U, a.P2.U, '-', budget)
	if err != nil {
		return Xp2{}, proof.Xpt{}, err
	}
	t, err := proof.ReplayRatArithmetic(tc.replayCtx, num, den, '/', budget)
	if err != nil {
		return Xp2{}, proof.Xpt{}, err
	}
	p3, err := proof.ReplayXlerp(tc.replayCtx, a.P3, b.P3, t.Num(), t.Denom(), budget)
	if err != nil {
		return Xp2{}, proof.Xpt{}, err
	}
	u, v := tc.U, tc.V
	if tc.Swap {
		u, v = v, u
	}
	p2, err := tc.projectChecked(p3, u, v)
	if err != nil {
		return Xp2{}, proof.Xpt{}, err
	}
	return p2, p3, nil
}

func (tc *brakeTriCutter) addVertChecked(p2 Xp2, p3 proof.Xpt) (int, error) {
	if tc.Reserve == nil {
		return 0, errReplayMissingReplayCutterReservation
	}
	if err := tc.Reserve(12); err != nil {
		return 0, err
	}
	return tc.addVertShared(p2, p3, true)
}

func (tc *brakeTriCutter) addVertShared(p2 Xp2, p3 proof.Xpt, boundary bool) (int, error) {
	if p2.deferredRats {
		return 0, errBrakeDeferredVertex
	}
	if tc.replayVert != nil {
		u, v := tc.U, tc.V
		if tc.Swap {
			u, v = v, u
		}
		projected := proof.Xpt{X: XIntCoordOf(p3, u), Y: XIntCoordOf(p3, v), Z: &tc.replayZero, W: p3.W}
		key, err := tc.replayBudget.keys.Key(tc.replayCtx, projected)
		if err != nil {
			return 0, err
		}
		i, found, err := tc.replayVert.Get(tc.replayCtx, key)
		if err != nil {
			return 0, err
		}
		if found {
			if boundary {
				tc.Verts[i].Boundary = true
			}
			return i, nil
		}
		i = len(tc.Verts)
		verts, err := appendReplay(tc.replayCtx, tc.Verts, CutVert{P2: p2, P3: p3, Boundary: boundary}, tc.replayBudget)
		if err != nil {
			return 0, err
		}
		if err := tc.replayVert.Set(tc.replayCtx, key, i); err != nil {
			return 0, err
		}
		tc.Verts = verts
		return i, nil
	}
	return 0, errReplayMissingReplayVertexTable
}

func (tc *brakeTriCutter) addPointChecked(p proof.Xpt, boundary bool) (int, error) {
	if tc.Reserve == nil {
		return 0, errReplayMissingReplayCutterReservation
	}
	if err := tc.Reserve(14); err != nil {
		return 0, err
	}
	u, v := tc.U, tc.V
	if tc.Swap {
		u, v = v, u
	}
	if tc.replayVert != nil {
		projected, err := tc.projectChecked(p, u, v)
		if err != nil {
			return 0, err
		}
		return tc.addVertShared(projected, p, boundary)
	}
	return 0, errReplayMissingReplayVertexTable
}

func (tc *brakeTriCutter) reserveArray(n int, size uintptr) error {
	if tc.replayBudget == nil || tc.replayBudget.replay == nil {
		return errReplayMissingReplayCutterBudget
	}
	if n < 0 {
		return errReplayNegativeCutterArraySize
	}
	if _, err := tc.Work.StepN(uint64(n)); err != nil {
		return err
	}
	return tc.replayBudget.ReserveBytes(tc.replayCtx, uint64(n), uint64(size))
}

func (tc *brakeTriCutter) reserveProduct(n int, scale, extra uint64) error {
	if tc.Reserve == nil {
		return errReplayMissingReplayCutterReservation
	}
	if n < 0 {
		return errReplayNegativeCutterRecordCount
	}
	count := uint64(n)
	if scale != 0 && count > (^uint64(0)-extra)/scale {
		return errReplayCutterRecordCountOverflow
	}
	return tc.Reserve(count*scale + extra)
}

func (tc *brakeTriCutter) cut(ctx context.Context, xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg) ([]CutRegion, error) {
	if tc.replayBudget == nil || tc.replayBudget.replay == nil || tc.Work == nil || !tc.Work.Replay ||
		tc.Work.ReplayID != tc.replayBudget.replay || tc.Reserve == nil || tc.replayVert == nil || tc.replaySeen == nil ||
		tc.replayVert.budget != tc.replayBudget.replay || tc.replaySeen.budget != tc.replayBudget.replay {
		return nil, errReplayIncompleteReplayCutter
	}
	tc.U, tc.V = ProjAxes(normal)
	cornerIdx, err := tc.internFacet(xtri, segs)
	if err != nil {
		return nil, err
	}
	return tc.cutInterned(ctx, cornerIdx)
}

func (tc *brakeTriCutter) cutInterned(ctx context.Context, cornerIdx [3]int) ([]CutRegion, error) {
	if len(tc.Edges) == 0 {
		return nil, errBrakeNoContactEdge
	}

	// Open every closed chain with split lines, then split the facet into
	// convex pieces along those same lines.
	lines, chains, err := tc.OpenLoops()
	if err != nil {
		return nil, err
	}
	if err := tc.reserveProduct(0, 0, 4); err != nil {
		return nil, err
	}
	if tc.replayBudget != nil {
		if _, err := tc.Work.StepN(4); err != nil {
			return nil, err
		}
		if err := tc.replayBudget.ReserveBytes(ctx, 1, uint64(unsafe.Sizeof([]int{}))); err != nil {
			return nil, err
		}
		if err := tc.replayBudget.ReserveBytes(ctx, 3, uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
	}
	pieces := [][]int{{cornerIdx[0], cornerIdx[1], cornerIdx[2]}}
	for _, c := range lines {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		if err := tc.reserveProduct(len(pieces), 2, 0); err != nil {
			return nil, err
		}
		var next [][]int
		for _, piece := range pieces {
			if tc.Reserve != nil {
				if err := tc.Work.Step(); err != nil {
					return nil, err
				}
			}
			left, right, err := tc.SplitConvexByU(piece, c)
			if err != nil {
				return nil, err
			}
			if left != nil {
				next, err = appendReplay(ctx, next, left, tc.replayBudget)
				if err != nil {
					return nil, err
				}
			}
			if right != nil {
				next, err = appendReplay(ctx, next, right, tc.replayBudget)
				if err != nil {
					return nil, err
				}
			}
		}
		pieces = next
	}

	// Assign each chain to the piece containing it, then split the pieces
	// along their chains into the final region polygons. The vertex table is
	// final from here on (nothing below adds a vertex or changes a boundary
	// flag), so each chain carries the probe found here into SplitByChains.
	if err := tc.reserveProduct(len(pieces), 1, uint64(len(chains))); err != nil {
		return nil, err
	}
	if tc.replayBudget != nil {
		if _, err := tc.Work.StepN(uint64(len(pieces))); err != nil {
			return nil, err
		}
		if err := tc.replayBudget.ReserveBytes(ctx, uint64(len(pieces)), uint64(unsafe.Sizeof([]ChainPath{}))); err != nil {
			return nil, err
		}
	}
	byPiece := make([][]ChainPath, len(pieces))
	for _, ch := range chains {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		probe, steps, err := tc.chainProbe(ch)
		if err != nil {
			return nil, err
		}
		ch.probe, ch.probeSteps, ch.hasProbe = probe, steps, true
		placed := false
		for pi, piece := range pieces {
			if tc.Reserve != nil {
				if err := tc.Work.Step(); err != nil {
					return nil, err
				}
			}
			pts, err := tc.PolyPoints(piece)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := replayPointInPoly2(tc.replayCtx, tc.Work, pts, probe) //nolint:contextcheck // The cutter retains its authenticated replay context.
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, errReplayAContactChainLiesOnASubdivision
			}
			if inside {
				byPiece[pi], err = appendReplay(ctx, byPiece[pi], ch, tc.replayBudget)
				if err != nil {
					return nil, err
				}
				placed = true
				break
			}
		}
		if !placed {
			return nil, errReplayAContactChainEscapedEverySubdivisionPiece
		}
	}

	if err := tc.reserveProduct(len(tc.Edges), 1, 0); err != nil {
		return nil, err
	}
	chainEdges := tc.anchors
	if tc.replayBudget != nil {
		var err error
		tc.replayAnchors, err = newReplayTable[[2]int, ChainAnchor](ctx, len(tc.Edges), tc.replayBudget.replay, replayEdgeHash)
		if err != nil {
			return nil, err
		}
	}
	for _, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		key := [2]int{min(e.A, e.B), max(e.A, e.B)}
		anchor := ChainAnchor{Partner: e.Partner, ViaParity: e.ViaParity}

		if tc.replayAnchors != nil {
			if err := tc.replayAnchors.Set(ctx, key, anchor); err != nil {
				return nil, err
			}
		} else {
			chainEdges[key] = anchor
		}
	}

	var regions []CutRegion
	for pi, piece := range pieces {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		polys, err := tc.SplitByChains(piece, byPiece[pi])
		if err != nil {
			return nil, err
		}
		for _, poly := range polys {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			reg, err := tc.RegionOf(poly, chainEdges)
			if err != nil {
				return nil, err
			}
			if err := tc.reserveProduct(0, 0, 1); err != nil {
				return nil, err
			}
			regions, err = appendReplay(ctx, regions, reg, tc.replayBudget)
			if err != nil {
				return nil, err
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return regions, nil
}

func (tc *brakeTriCutter) internFacet(xtri [3]proof.Xpt, segs []Xseg) ([3]int, error) {
	var corners [3]Xp2
	for i, p := range xtri {
		var err error
		corners[i], err = tc.projectChecked(p, tc.U, tc.V)
		if err != nil {
			return [3]int{}, err
		}
	}
	orientation, err := polygonSide(tc.replayCtx, tc.Work, corners[0], corners[1], corners[2])
	if err != nil {
		return [3]int{}, err
	}
	if orientation < 0 {
		tc.Swap = true
		for i := range corners {
			corners[i] = xp2Swapped(corners[i])
		}
	}
	var cornerIdx [3]int
	for i, p := range xtri {
		if err := tc.Work.Step(); err != nil {
			return cornerIdx, err
		}
		c := corners[i]
		if tc.Reserve != nil {
			if err := tc.Reserve(2); err != nil {
				return cornerIdx, err
			}
		}
		idx, err := tc.addVertChecked(c, p)
		if err != nil {
			return cornerIdx, err
		}
		cornerIdx[i] = idx
		if tc.replayVert == nil {
			tc.raw.add(rawHash(c.Hu, c.Hv, c.Hw), c.Hu, c.Hv, c.Hw, cornerIdx[i])
		}
	}

	seen := tc.seen
	if seen == nil && tc.replaySeen == nil {
		seen = map[[2]int]struct{}{}
		tc.seen = seen
	}
	for _, s := range segs {
		if err := tc.Work.Step(); err != nil {
			return cornerIdx, err
		}

		ia, err := tc.addPointChecked(s.A, s.AOnEdge)
		if err != nil {
			return cornerIdx, err
		}
		ib, err := tc.addPointChecked(s.B, s.BOnEdge)
		if err != nil {
			return cornerIdx, err
		}
		if ia == ib {
			continue
		}
		k := [2]int{min(ia, ib), max(ia, ib)}
		_, exists := seen[k]
		if tc.replaySeen != nil {
			var err error
			_, exists, err = tc.replaySeen.Get(tc.replayCtx, k)
			if err != nil {
				return cornerIdx, err
			}
		}
		if exists {
			continue
		}
		if tc.Reserve != nil {
			if err := tc.Reserve(2); err != nil {
				return cornerIdx, err
			}
		}
		if tc.replaySeen != nil {
			if err := tc.replaySeen.Set(tc.replayCtx, k, struct{}{}); err != nil {
				return cornerIdx, err
			}
		} else {
			seen[k] = struct{}{}
		}
		tc.Edges, err = appendReplay(tc.replayCtx, tc.Edges, CutEdge{A: ia, B: ib, Partner: s.Partner, ViaParity: s.ViaParity}, tc.replayBudget)
		if err != nil {
			return cornerIdx, err
		}
	}
	return cornerIdx, nil
}

func (tc *brakeTriCutter) ChainProbe(ch ChainPath) (Xp2, error) {
	p, _, err := tc.chainProbe(ch)
	return p, err
}

func (tc *brakeTriCutter) chainProbe(ch ChainPath) (Xp2, int, error) {
	if len(ch.Verts) < 2 {
		return Xp2{}, 0, errBrakeInvalidChain
	}
	steps := 0
	for _, vi := range ch.Verts[1 : len(ch.Verts)-1] {
		if err := tc.Work.Step(); err != nil {
			return Xp2{}, steps, err
		}
		steps++
		if !tc.Verts[vi].Boundary {
			p := tc.Verts[vi].P2
			if p.deferredRats {
				return Xp2{}, steps, errBrakeDeferredProbe
			}
			return p, steps, nil
		}
	}
	a, b := tc.Verts[ch.Verts[0]].P2, tc.Verts[ch.Verts[1]].P2
	if a.deferredRats || b.deferredRats {
		return Xp2{}, steps, errBrakeDeferredProbe
	}
	if tc.replayBudget == nil || tc.replayBudget.replay == nil {
		return Xp2{}, steps, errReplayMissingReplayCutterBudget
	}
	u, err := tc.ratMidpoint(a.U, b.U)
	if err != nil {
		return Xp2{}, steps, err
	}
	v, err := tc.ratMidpoint(a.V, b.V)
	if err != nil {
		return Xp2{}, steps, err
	}
	p, err := replayXP2FromRats(tc.replayCtx, u, v, tc.replayBudget.replay)
	return p, steps, err
}

func (tc *brakeTriCutter) recordedChainProbe(ch ChainPath) (Xp2, error) {
	if !ch.hasProbe {
		return tc.ChainProbe(ch)
	}
	for range ch.probeSteps {
		if err := tc.Work.Step(); err != nil {
			return Xp2{}, err
		}
	}
	return ch.probe, nil
}

func (tc *brakeTriCutter) PolyPoints(poly []int) ([]Xp2, error) {
	if err := tc.reserveProduct(len(poly), 1, 0); err != nil {
		return nil, err
	}
	if err := tc.reserveArray(len(poly), unsafe.Sizeof(Xp2{})); err != nil {
		return nil, err
	}
	out := make([]Xp2, len(poly))
	for i, vi := range poly {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		out[i] = tc.Verts[vi].P2
	}
	return out, nil
}

func (tc *brakeTriCutter) OpenLoops() ([]*big.Rat, []ChainPath, error) {
	var lines []*big.Rat
	guard := len(tc.Edges) + 8
	for iter := 0; ; iter++ {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		if iter > guard {
			return nil, nil, errReplayLoopOpeningDidNotConverge
		}
		chains, loops, err := tc.BuildChains()
		if err != nil {
			return nil, nil, err
		}
		if len(loops) == 0 {
			return lines, chains, nil
		}
		c, err := tc.ChooseSplitU(loops[0])
		if err != nil {
			return nil, nil, err
		}
		if err := tc.SplitEdgesAtU(c); err != nil {
			return nil, nil, err
		}
		if err := tc.reserveProduct(0, 0, 1); err != nil {
			return nil, nil, err
		}
		lines, err = appendReplay(tc.replayCtx, lines, c, tc.replayBudget)
		if err != nil {
			return nil, nil, err
		}
	}
}

func (tc *brakeTriCutter) BuildChains() ([]ChainPath, []ChainPath, error) {
	// At most two adjacency entries and keys, one used entry, two
	// path vertices, and one path record per edge.
	if err := tc.reserveProduct(len(tc.Edges), 8, 0); err != nil {
		return nil, nil, err
	}
	var adj map[int][]int
	var replayAdj [][]int
	if tc.replayBudget != nil {
		if _, err := tc.Work.StepN(uint64(len(tc.Verts))); err != nil {
			return nil, nil, err
		}
		if err := tc.replayBudget.ReserveRecords(tc.replayCtx, uint64(len(tc.Verts))); err != nil {
			return nil, nil, err
		}
		if err := tc.replayBudget.ReserveBytes(tc.replayCtx, uint64(len(tc.Verts)), uint64(unsafe.Sizeof([]int{}))); err != nil {
			return nil, nil, err
		}
		replayAdj = make([][]int, len(tc.Verts))
	} else {
		adj = map[int][]int{}
	}
	adjOf := func(vi int) []int {
		if replayAdj != nil {
			return replayAdj[vi]
		}
		return adj[vi]
	}
	for ei, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		if replayAdj != nil {
			for _, vi := range [2]int{e.A, e.B} {
				if vi < 0 || vi >= len(replayAdj) {
					return nil, nil, errReplayInvalidCutterEdge
				}
				list, err := appendReplay(tc.replayCtx, replayAdj[vi], ei, tc.replayBudget)
				if err != nil {
					return nil, nil, err
				}
				replayAdj[vi] = list
			}
		} else {
			adj[e.A] = append(adj[e.A], ei)
			adj[e.B] = append(adj[e.B], ei)
		}
	}
	check := func(vi int, list []int) error {
		if err := tc.Work.Step(); err != nil {
			return err
		}
		if !tc.Verts[vi].Boundary && len(list) > 2 {
			return tc.branchContactRefusal()
		}
		if !tc.Verts[vi].Boundary && len(list) == 1 {
			return errReplayAContactChainDanglesMidFacet
		}
		return nil
	}
	if replayAdj != nil {
		for vi, list := range replayAdj {
			if len(list) > 0 {
				if err := check(vi, list); err != nil {
					return nil, nil, err
				}
			}
		}
	} else {
		for vi, list := range adj {
			if err := check(vi, list); err != nil {
				return nil, nil, err
			}
		}
	}
	otherEnd := func(ei, vi int) int {
		if tc.Edges[ei].A == vi {
			return tc.Edges[ei].B
		}
		return tc.Edges[ei].A
	}
	if tc.replayBudget != nil {
		if _, err := tc.Work.StepN(uint64(len(tc.Edges))); err != nil {
			return nil, nil, err
		}
		if err := tc.replayBudget.ReserveBytes(tc.replayCtx, uint64(len(tc.Edges)), uint64(unsafe.Sizeof(false))); err != nil {
			return nil, nil, err
		}
	}
	used := make([]bool, len(tc.Edges))
	var chains, loops []ChainPath
	// A chain interior vertex continues the walk exactly when it is
	// interior with degree two.
	continues := func(vi int) bool { return !tc.Verts[vi].Boundary && len(adjOf(vi)) == 2 }
	nextEdge := func(vi, from int) (int, error) {
		for _, ei := range adjOf(vi) {
			if err := tc.Work.Step(); err != nil {
				return -1, err
			}
			if ei != from && !used[ei] {
				return ei, nil
			}
		}
		return -1, nil
	}
	for start := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		if used[start] {
			continue
		}
		used[start] = true
		var path []int
		var appendErr error
		path, appendErr = appendReplay(tc.replayCtx, path, tc.Edges[start].A, tc.replayBudget)
		if appendErr != nil {
			return nil, nil, appendErr
		}
		path, appendErr = appendReplay(tc.replayCtx, path, tc.Edges[start].B, tc.replayBudget)
		if appendErr != nil {
			return nil, nil, appendErr
		}
		isLoop := false
		// Extend forward from the tail, then backward from the head.
		for continues(path[len(path)-1]) {
			if err := tc.Work.Step(); err != nil {
				return nil, nil, err
			}
			ei, err := nextEdge(path[len(path)-1], -1)
			if err != nil {
				return nil, nil, err
			}
			if ei < 0 {
				isLoop = true
				break
			}
			used[ei] = true
			path, appendErr = appendReplay(tc.replayCtx, path, otherEnd(ei, path[len(path)-1]), tc.replayBudget)
			if appendErr != nil {
				return nil, nil, appendErr
			}
		}
		if !isLoop {
			for continues(path[0]) {
				if err := tc.Work.Step(); err != nil {
					return nil, nil, err
				}
				ei, err := nextEdge(path[0], -1)
				if err != nil {
					return nil, nil, err
				}
				if ei < 0 {
					break
				}
				used[ei] = true
				next := otherEnd(ei, path[0])
				path, appendErr = appendReplay(tc.replayCtx, path, 0, tc.replayBudget)
				if appendErr != nil {
					return nil, nil, appendErr
				}
				for i := len(path) - 1; i > 0; i-- {
					if err := tc.Work.Step(); err != nil {
						return nil, nil, err
					}
					path[i] = path[i-1]
				}
				path[0] = next
			}
		}
		if isLoop || path[0] == path[len(path)-1] {
			loops, appendErr = appendReplay(tc.replayCtx, loops, ChainPath{Verts: path}, tc.replayBudget)
			if appendErr != nil {
				return nil, nil, appendErr
			}
			continue
		}
		chains, appendErr = appendReplay(tc.replayCtx, chains, ChainPath{Verts: path}, tc.replayBudget)
		if appendErr != nil {
			return nil, nil, appendErr
		}
	}
	return chains, loops, nil
}

func (tc *brakeTriCutter) ChooseSplitU(loop ChainPath) (*big.Rat, error) {
	materializeXP2(tc.Verts[loop.Verts[0]].P2)
	lo, hi := tc.Verts[loop.Verts[0]].P2.U, tc.Verts[loop.Verts[0]].P2.U
	for _, vi := range loop.Verts {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		materializeXP2(tc.Verts[vi].P2)
		u := tc.Verts[vi].P2.U
		cmp, err := tc.ratCompare(u, lo)
		if err != nil {
			return nil, err
		}
		if cmp < 0 {
			lo = u
		}
		cmp, err = tc.ratCompare(u, hi)
		if err != nil {
			return nil, err
		}
		if cmp > 0 {
			hi = u
		}
	}
	cmp, err := tc.ratCompare(lo, hi)
	if err != nil {
		return nil, err
	}
	if cmp == 0 {
		return nil, errReplayAClosedContactChainHasNoWidth
	}
	c, err := tc.ratMidpoint(lo, hi)
	if err != nil {
		return nil, err
	}
	for range 64 {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		hit := false
		for _, v := range tc.Verts {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			materializeXP2(v.P2)
			cmp, err := tc.ratCompare(v.P2.U, c)
			if err != nil {
				return nil, err
			}
			if cmp == 0 {
				hit = true
				break
			}
		}
		if !hit {
			return c, nil
		}
		c, err = tc.ratMidpoint(lo, c)
		if err != nil {
			return nil, err
		}
	}
	return nil, errReplayNoVertexFreeSplitLineFound
}

func (tc *brakeTriCutter) SplitEdgesAtU(c *big.Rat) error {
	if err := tc.reserveProduct(len(tc.Edges), 2, 0); err != nil {
		return err
	}
	var out []CutEdge
	for _, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return err
		}
		materializeXP2(tc.Verts[e.A].P2)
		materializeXP2(tc.Verts[e.B].P2)
		ua := tc.Verts[e.A].P2.U
		ub := tc.Verts[e.B].P2.U
		sa, err := tc.ratCompare(ua, c)
		if err != nil {
			return err
		}
		sb, err := tc.ratCompare(ub, c)
		if err != nil {
			return err
		}
		if sa*sb >= 0 {
			var err error
			out, err = appendReplay(tc.replayCtx, out, e, tc.replayBudget)
			if err != nil {
				return err
			}
			continue
		}
		p2, p3, err := tc.splitAtU(tc.Verts[e.A], tc.Verts[e.B], c)
		if err != nil {
			return err
		}
		mid, err := tc.addVertChecked(p2, p3)
		if err != nil {
			return err
		}
		out, err = appendReplay(tc.replayCtx, out, CutEdge{A: e.A, B: mid, Partner: e.Partner, ViaParity: e.ViaParity}, tc.replayBudget)
		if err != nil {
			return err
		}
		out, err = appendReplay(tc.replayCtx, out, CutEdge{A: mid, B: e.B, Partner: e.Partner, ViaParity: e.ViaParity}, tc.replayBudget)
		if err != nil {
			return err
		}
	}
	tc.Edges = out
	return nil
}

func (tc *brakeTriCutter) SplitConvexByU(piece []int, c *big.Rat) ([]int, []int, error) {
	if err := tc.reserveProduct(len(piece), 4, 0); err != nil {
		return nil, nil, err
	}
	n := len(piece)
	var left, right []int
	var appendErr error
	for i := range n {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		vi, vj := piece[i], piece[(i+1)%n]
		materializeXP2(tc.Verts[vi].P2)
		materializeXP2(tc.Verts[vj].P2)
		si, err := tc.ratCompare(tc.Verts[vi].P2.U, c)
		if err != nil {
			return nil, nil, err
		}
		sj, err := tc.ratCompare(tc.Verts[vj].P2.U, c)
		if err != nil {
			return nil, nil, err
		}
		if si <= 0 {
			left, appendErr = appendReplay(tc.replayCtx, left, vi, tc.replayBudget)
			if appendErr != nil {
				return nil, nil, appendErr
			}
		}
		if si >= 0 {
			right, appendErr = appendReplay(tc.replayCtx, right, vi, tc.replayBudget)
			if appendErr != nil {
				return nil, nil, appendErr
			}
		}
		if si*sj >= 0 {
			continue
		}
		p2, p3, err := tc.splitAtU(tc.Verts[vi], tc.Verts[vj], c)
		if err != nil {
			return nil, nil, err
		}
		mid, err := tc.addVertChecked(p2, p3)
		if err != nil {
			return nil, nil, err
		}
		left, appendErr = appendReplay(tc.replayCtx, left, mid, tc.replayBudget)
		if appendErr != nil {
			return nil, nil, appendErr
		}
		right, appendErr = appendReplay(tc.replayCtx, right, mid, tc.replayBudget)
		if appendErr != nil {
			return nil, nil, appendErr
		}
	}
	leftPts, err := tc.PolyPoints(left)
	if err != nil {
		return nil, nil, err
	}
	leftArea, err := replayPolyArea2Sign(tc.replayCtx, tc.Work, leftPts)
	if err != nil {
		return nil, nil, err
	}
	if len(left) < 3 || leftArea <= 0 {
		left = nil
	}
	rightPts, err := tc.PolyPoints(right)
	if err != nil {
		return nil, nil, err
	}
	rightArea, err := replayPolyArea2Sign(tc.replayCtx, tc.Work, rightPts)
	if err != nil {
		return nil, nil, err
	}
	if len(right) < 3 || rightArea <= 0 {
		right = nil
	}
	return left, right, nil
}

func (tc *brakeTriCutter) SplitByChains(piece []int, chains []ChainPath) ([][]int, error) {
	type work struct {
		poly   []int
		chains []ChainPath
	}
	if err := tc.reserveProduct(0, 0, 1); err != nil {
		return nil, err
	}
	if err := tc.reserveArray(1, unsafe.Sizeof(work{})); err != nil {
		return nil, err
	}
	stack := []work{{poly: piece, chains: chains}}
	var out [][]int
	for len(stack) > 0 {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		w := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if len(w.chains) == 0 {
			if err := tc.reserveProduct(0, 0, 1); err != nil {
				return nil, err
			}
			var err error
			out, err = appendReplay(tc.replayCtx, out, w.poly, tc.replayBudget)
			if err != nil {
				return nil, err
			}
			continue
		}
		ch := w.chains[0]
		rest := w.chains[1:]
		poly, err := tc.InsertOnBoundary(w.poly, ch.Verts[0])
		if err != nil {
			return nil, err
		}
		poly, err = tc.InsertOnBoundary(poly, ch.Verts[len(ch.Verts)-1])
		if err != nil {
			return nil, err
		}
		i, err := tc.IndexOf(poly, ch.Verts[0])
		if err != nil {
			return nil, err
		}
		j, err := tc.IndexOf(poly, ch.Verts[len(ch.Verts)-1])
		if err != nil {
			return nil, err
		}
		if i < 0 || j < 0 || i == j {
			return nil, errReplayAChainEndpointIsMissingFromIts
		}
		// Walk the boundary i→j one way and j→i the other, closing each side
		// with the chain path so both stay counter-clockwise.
		interior := ch.Verts[1 : len(ch.Verts)-1]
		if err := tc.reserveProduct(len(poly), 2, 0); err != nil {
			return nil, err
		}
		if err := tc.reserveProduct(len(interior), 2, 0); err != nil {
			return nil, err
		}
		var polyA []int
		for k := i; ; k = (k + 1) % len(poly) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyA, err = appendReplay(tc.replayCtx, polyA, poly[k], tc.replayBudget)
			if err != nil {
				return nil, err
			}
			if k == j {
				break
			}
		}
		for _, vi := range slices.Backward(interior) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyA, err = appendReplay(tc.replayCtx, polyA, vi, tc.replayBudget)
			if err != nil {
				return nil, err
			}
		}
		var polyB []int
		for k := j; ; k = (k + 1) % len(poly) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyB, err = appendReplay(tc.replayCtx, polyB, poly[k], tc.replayBudget)
			if err != nil {
				return nil, err
			}
			if k == i {
				break
			}
		}
		for _, vi := range interior {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyB, err = appendReplay(tc.replayCtx, polyB, vi, tc.replayBudget)
			if err != nil {
				return nil, err
			}
		}
		ptsA, err := tc.PolyPoints(polyA)
		if err != nil {
			return nil, err
		}
		areaA, err := replayPolyArea2Sign(tc.replayCtx, tc.Work, ptsA)
		if err != nil {
			return nil, err
		}
		ptsB, err := tc.PolyPoints(polyB)
		if err != nil {
			return nil, err
		}
		areaB, err := replayPolyArea2Sign(tc.replayCtx, tc.Work, ptsB)
		if err != nil {
			return nil, err
		}
		if areaA <= 0 || areaB <= 0 {
			return nil, errReplayAChainSplitProducedANonPositive
		}
		if err := tc.reserveProduct(len(rest), 1, 2); err != nil {
			return nil, err
		}
		if err := tc.reserveArray(len(rest), unsafe.Sizeof(ChainPath{})); err != nil {
			return nil, err
		}
		chainParts := make([]ChainPath, len(rest))
		aEnd := 0
		bStart := len(chainParts)
		for _, rc := range rest {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			probe, err := tc.recordedChainProbe(rc)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := replayPointInPoly2(tc.replayCtx, tc.Work, ptsA, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, errReplayAChainLiesOnAFreshlySplit
			}
			if inside {
				chainParts[aEnd] = rc
				aEnd++
				continue
			}
			inside, onBoundary, err = replayPointInPoly2(tc.replayCtx, tc.Work, ptsB, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary || !inside {
				return nil, errReplayAChainEscapedBothSplitRegions
			}
			bStart--
			chainParts[bStart] = rc
		}
		aChains := chainParts[:aEnd:aEnd]
		bChains := chainParts[bStart:len(chainParts):len(chainParts)]
		if tc.Reserve != nil {
			if _, err := tc.Work.StepN(uint64(len(bChains) / 2)); err != nil {
				return nil, err
			}
		}
		slices.Reverse(bChains)
		wa := work{poly: polyA, chains: aChains}
		wb := work{poly: polyB, chains: bChains}
		stack, err = appendReplay(tc.replayCtx, stack, wa, tc.replayBudget)
		if err != nil {
			return nil, err
		}
		stack, err = appendReplay(tc.replayCtx, stack, wb, tc.replayBudget)
		if err != nil {
			return nil, err
		}
	}
	return out, nil
}

func (tc *brakeTriCutter) InsertOnBoundary(poly []int, vi int) ([]int, error) {
	found, err := tc.IndexOf(poly, vi)
	if err != nil {
		return nil, err
	}
	if found >= 0 {
		return poly, nil
	}
	p := tc.Verts[vi].P2
	n := len(poly)
	for i := range n {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		a := tc.Verts[poly[i]].P2
		b := tc.Verts[poly[(i+1)%n]].P2
		interior := false
		if tc.Work.Replay {
			on, e := replayOnSegment2(tc.replayCtx, tc.Work, a, b, p)
			if e != nil {
				return nil, e
			}
			if on {
				first, e := replayCompare2(tc.replayCtx, tc.Work, a, p, 0)
				if e != nil {
					return nil, e
				}
				second, e := replayCompare2(tc.replayCtx, tc.Work, b, p, 0)
				if e != nil {
					return nil, e
				}
				if first == 0 {
					first, e = replayCompare2(tc.replayCtx, tc.Work, a, p, 1)
					if e != nil {
						return nil, e
					}
				}
				if second == 0 {
					second, e = replayCompare2(tc.replayCtx, tc.Work, b, p, 1)
					if e != nil {
						return nil, e
					}
				}
				interior = first != 0 && second != 0
			}
		} else {
			_, interior = OnSegment2(a, b, p)
		}
		if !interior {
			continue
		}
		if err := tc.reserveProduct(n, 1, 1); err != nil {
			return nil, err
		}
		if tc.Reserve != nil {
			if _, err := tc.Work.StepN(uint64(n)); err != nil {
				return nil, err
			}
		}
		if err := tc.reserveArray(n+1, unsafe.Sizeof(int(0))); err != nil {
			return nil, err
		}
		out := make([]int, 0, n+1)
		out = append(out, poly[:i+1]...)
		out = append(out, vi)
		out = append(out, poly[i+1:]...)
		return out, nil
	}
	return nil, errReplayAChainEndpointLiesOnNoRegion
}

func (tc *brakeTriCutter) IndexOf(poly []int, vi int) (int, error) {
	for i, v := range poly {
		if err := tc.Work.Step(); err != nil {
			return -1, err
		}
		if v == vi {
			return i, nil
		}
	}
	return -1, nil
}

func (tc *brakeTriCutter) RegionOf(poly []int, chainEdges map[[2]int]ChainAnchor) (CutRegion, error) {
	points, err := tc.collectP2Into(tc.regionPoints)
	if err != nil {
		return CutRegion{}, err
	}
	// EarClipX only reads the projections; the result carries copied 3D points.
	tc.regionPoints = points
	tris2, err := replayEarClip(tc.replayCtx, tc.Work, points, poly)
	if err != nil {
		return CutRegion{}, err
	}
	if len(tris2) == 0 {
		return CutRegion{}, errReplayARegionTriangulatedToNothing
	}
	if err := tc.reserveProduct(len(tris2), 1, 0); err != nil {
		return CutRegion{}, err
	}
	reg := CutRegion{}

	for _, t := range tris2 {
		if err := tc.Work.Step(); err != nil {
			return CutRegion{}, err
		}
		corners := [3]proof.Xpt{tc.Verts[t[0]].P3, tc.Verts[t[1]].P3, tc.Verts[t[2]].P3}
		reg.Tris, err = appendReplay(tc.replayCtx, reg.Tris, corners, tc.replayBudget)
		if err != nil {
			return CutRegion{}, err
		}
		if reg.HasAnchor {
			continue
		}
		for k := range 3 {
			if err := tc.Work.Step(); err != nil {
				return CutRegion{}, err
			}
			key := [2]int{min(t[k], t[(k+1)%3]), max(t[k], t[(k+1)%3])}
			anchor, ok := chainEdges[key]
			if tc.replayAnchors != nil {
				var err error
				anchor, ok, err = tc.replayAnchors.Get(tc.replayCtx, key)
				if err != nil {
					return CutRegion{}, err
				}
			}
			if !ok {
				continue
			}

			if reg.HasAnchor {
				continue
			}
			if anchor.ViaParity {
				continue
			}
			reg.HasAnchor = true
			reg.Partner = anchor.Partner
			reg.Probe, err = tc.centroid(corners[0], corners[1], corners[2])
			if err != nil {
				return CutRegion{}, err
			}
			break
		}
	}
	if !reg.HasAnchor {
		first := reg.Tris[0]
		reg.Probe, err = tc.centroid(first[0], first[1], first[2])
		if err != nil {
			return CutRegion{}, err
		}
	}
	return reg, nil
}

func (tc *brakeTriCutter) collectP2Into(out []Xp2) ([]Xp2, error) {
	if cap(out) < len(tc.Verts) {
		if err := tc.reserveProduct(len(tc.Verts), 1, 0); err != nil {
			return nil, err
		}
		if err := tc.reserveArray(len(tc.Verts), unsafe.Sizeof(Xp2{})); err != nil {
			return nil, err
		}
		out = make([]Xp2, len(tc.Verts))
	} else {
		out = out[:len(tc.Verts)]
	}
	for i, v := range tc.Verts {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		out[i] = v.P2
	}
	return out, nil
}

func (tc *brakeTriCutter) centroid(a, b, c proof.Xpt) (proof.Xpt, error) {
	if tc.Work.Replay {
		return proof.ReplayCentroid(tc.replayCtx, a, b, c, replayExactWork{tc.Work})
	}
	return XCentroid(a, b, c), nil
}
