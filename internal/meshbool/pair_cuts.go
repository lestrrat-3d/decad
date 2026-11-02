package meshbool

import (
	"context"
	"fmt"
	"sort"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PairCutsResult holds the exact contact cuts before classification and stitching.
type PairCutsResult struct {
	CutsA, CutsB                   map[int][]Xseg
	replayCutsA, replayCutsB       [][]Xseg
	replayBlockedA, replayBlockedB *replayTable[[2]int, struct{}]
	replayRims                     *replayTable[string, float64]
	BlockedA, BlockedB             map[[2]int]struct{}
	Rims                           map[string]float64
	MaxRim                         float64
}

// PairCutsBudget tracks work and record reservations across collector runs.
// Zero limits mean unlimited. Ordinary callers do not receive a byte bound;
// replay budgets are accepted only by the private bounded collector.
type PairCutsBudget struct {
	Work, Allocations, MaxWork, MaxAllocations uint64
	replay                                     *proofbound.ReplayBudget
	keys                                       *ReplayKeys
	contactRefusal                             BooleanExpectedError
}

// NewReplayPairCutsBudget shares the private operation's cumulative limits.
func NewReplayPairCutsBudget(replay *proofbound.ReplayBudget) (*PairCutsBudget, error) {
	if replay == nil {
		return nil, errReplayReplayPairCutsNeedASharedBudget
	}
	return &PairCutsBudget{replay: replay, keys: NewReplayKeys(replay)}, nil
}

// SpendWork reserves work before it is performed. A failed charge changes no counters.
func (b *PairCutsBudget) SpendWork(ctx context.Context, n uint64) error {
	if b == nil {
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > ^uint64(0)-b.Work || (b.MaxWork != 0 && (b.Work > b.MaxWork || n > b.MaxWork-b.Work)) {
		if b.replay != nil {
			return errReplayPairCutWorkBudgetExceeded
		}
		return fmt.Errorf("%w: pair cut work budget exceeded", decaderr.ErrUnsupported)
	}
	if b.replay != nil {
		if err := b.replay.SpendWork(ctx, n); err != nil {
			return err
		}
	}
	b.Work += n
	return nil
}

// ReserveRecords reserves records before allocating or appending them.
func (b *PairCutsBudget) ReserveRecords(ctx context.Context, n uint64) error {
	if b == nil {
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > ^uint64(0)-b.Allocations || (b.MaxAllocations != 0 &&
		(b.Allocations > b.MaxAllocations || n > b.MaxAllocations-b.Allocations)) {
		if b.replay != nil {
			return errReplayPairCutRecordBudgetExceeded
		}
		return fmt.Errorf("%w: pair cut record budget exceeded", decaderr.ErrUnsupported)
	}
	if b.replay != nil {
		if err := b.replay.ReserveRecords(ctx, n); err != nil {
			return err
		}
	}
	b.Allocations += n
	return nil
}

func (b *PairCutsBudget) pointKey(ctx context.Context, p proof.Xpt, fallback func(proof.Xpt) string) (string, error) {
	if b != nil && b.keys != nil {
		return b.keys.Key(ctx, p)
	}
	return fallback(p), nil
}

// ReserveBytes charges replay-owned storage without changing legacy callers.
func (b *PairCutsBudget) ReserveBytes(ctx context.Context, count, size uint64) error {
	if b == nil || b.replay == nil {
		return nil
	}
	if size != 0 && count > ^uint64(0)/size {
		return errReplayPairCutByteReservationOverflow
	}
	return b.replay.ReserveArray(ctx, count, size)
}

// The envelope covers the result, captured cells, and escaping closures before
// the compiler can allocate them at collector entry.
const replayCollectorFixedRecords = 16
const replayCollectorFixedBytes = 1024

func reserveReplayCollectorFixed(ctx context.Context, budget *PairCutsBudget) error {
	if budget == nil || budget.replay == nil {
		return errReplayMissingReplayCollectorBudget
	}
	if err := budget.ReserveRecords(ctx, replayCollectorFixedRecords); err != nil {
		return err
	}
	return budget.ReserveBytes(ctx, 1, uint64(unsafe.Sizeof(PairCutsResult{}))+replayCollectorFixedBytes)
}

// PairCuts rejects unprovable contacts and collects the exact cuts without classifying either side.
func PairCuts(ctx context.Context, ma, mb *BoolMesh, memo *ContactMemo, budget *PairCutsBudget) (*PairCutsResult, error) {
	if budget != nil && budget.replay != nil {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return nil, errReplayReplayPairCutsRequireThePrivateCollector
	}
	return boundedPairCuts(ctx, ma, mb, memo, newKeyMemo(), budget)
}

func boundedPairCuts(ctx context.Context, ma, mb *BoolMesh, memo *ContactMemo, keyOf func(proof.Xpt) string, budget *PairCutsBudget) (*PairCutsResult, error) {
	if budget != nil && budget.replay != nil {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if ma == nil || mb == nil || len(ma.SlabBound) != len(ma.Tris) || len(mb.SlabBound) != len(mb.Tris) {
			return nil, errReplayReplayPairCutsNeedPreparedFacetSlab
		}
		if err := reserveReplayCollectorFixed(ctx, budget); err != nil {
			return nil, err
		}
	}
	return pairCutsImpl(ctx, ma, mb, memo, keyOf, budget)
}

// pairCutsImpl is called only after the replay wrapper reserves its fixed
// escaped locals and closures. The compiler may allocate those at entry.
func pairCutsImpl(ctx context.Context, ma, mb *BoolMesh, memo *ContactMemo, keyOf func(proof.Xpt) string, budget *PairCutsBudget) (*PairCutsResult, error) {
	var cutsA, cutsB map[int][]Xseg
	var replayCutsA, replayCutsB [][]Xseg
	if budget != nil && budget.replay != nil {
		for _, n := range []int{len(ma.Tris), len(mb.Tris)} {
			if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
				return nil, err
			}
			if err := budget.SpendWork(ctx, uint64(n)); err != nil {
				return nil, err
			}
			if err := budget.ReserveBytes(ctx, uint64(n), uint64(unsafe.Sizeof([]Xseg{}))); err != nil {
				return nil, err
			}
		}
		replayCutsA = make([][]Xseg, len(ma.Tris))
		replayCutsB = make([][]Xseg, len(mb.Tris))
	} else {
		cutsA = map[int][]Xseg{}
		cutsB = map[int][]Xseg{}
	}
	appendCut := func(side, facet int, seg Xseg) error {
		if replayCutsA == nil {
			if side == 0 {
				cutsA[facet] = append(cutsA[facet], seg)
			} else {
				cutsB[facet] = append(cutsB[facet], seg)
			}
			return nil
		}
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		lists := replayCutsA
		if side == 1 {
			lists = replayCutsB
		}
		if facet < 0 || facet >= len(lists) {
			return errReplayInvalidContactFacet
		}
		if len(lists[facet]) == cap(lists[facet]) {
			n := max(1, 2*cap(lists[facet]))
			if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
				return err
			}
			if err := budget.ReserveBytes(ctx, uint64(n), uint64(unsafe.Sizeof(Xseg{}))); err != nil {
				return err
			}
			if err := budget.SpendWork(ctx, uint64(len(lists[facet]))); err != nil {
				return err
			}
			next := make([]Xseg, len(lists[facet]), n)
			copy(next, lists[facet])
			lists[facet] = next
		}
		lists[facet] = append(lists[facet], seg)
		return nil
	}
	var pointTouches []proof.Xpt
	var pointFacets [][2]int
	var segEnds []proof.Xpt
	var inPlane []PairContact
	// rims maps each rim vertex (a contact segment's endpoint, by exact key)
	// to the largest RimBound over the facet pairs whose segment ends there.
	var rims map[string]float64
	var replayRims *replayTable[string, float64]
	if budget != nil && budget.replay != nil {
		var err error
		replayRims, err = newReplayGrowingTable[string, float64](ctx, 2*1024*1024, budget.replay, replayStringHash)
		if err != nil {
			return nil, err
		}
		replayRims.comparisonWork = func(s string) uint64 { return uint64(len(s)) }
	} else {
		rims = map[string]float64{}
	}
	setRim := func(key string, rim float64) error {
		if replayRims != nil {
			prev, ok, err := replayRims.Get(ctx, key)
			if err != nil {
				return err
			}
			if !ok || rim > prev {
				return replayRims.Set(ctx, key, rim)
			}
			return nil
		}
		if prev, ok := rims[key]; !ok || rim > prev {
			rims[key] = rim
		}
		return nil
	}
	maxRim := 0.0
	// touchedA and touchedB are the facets of each operand that meet the
	// other, which the held-mesh gate reads once classification is done.
	var touchedA, touchedB []int
	var replayTouched [2]float64
	// coplanar collects the coplanar facet pairs the scan meets, up to
	// maxTouchContacts, so the refusal can say which planes are involved.
	var coplanar []TouchContact
	work := 0

	var boxTree heldBoxTree
	if err := budget.ReserveRecords(ctx, uint64(len(mb.Boxes))*5); err != nil {
		return nil, err
	}
	if budget == nil {
		boxTree = newBoxTree(mb.Boxes)
	} else {
		workBudget := &proofbound.WorkBudget{Replay: budget.replay != nil, ReplayID: budget.replay, StepFn: func() error { return budget.SpendWork(ctx, 1) }, ErrFn: ctx.Err, ReserveBytesFn: func(n uint64) error { return budget.ReserveBytes(ctx, n, 1) }}
		var err error
		boxTree, err = newBoxTreeWork(mb.Boxes, workBudget)
		if err != nil {
			return nil, err
		}
	}
	var treeStack []int
	if budget != nil {
		if err := budget.ReserveRecords(ctx, uint64(len(boxTree.nodes))); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytes(ctx, uint64(len(boxTree.nodes)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
		if budget.replay != nil {
			if err := budget.SpendWork(ctx, uint64(len(boxTree.nodes))); err != nil {
				return nil, err
			}
		}
		treeStack = make([]int, 0, len(boxTree.nodes))
	}
	// Precharge the reusable candidate and four fixed contact-batch buffers.
	if err := budget.ReserveRecords(ctx, 64+4*ContactBatchSize); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, 64, uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	if budget != nil && budget.replay != nil {
		if err := budget.SpendWork(ctx, 64+4*ContactBatchSize); err != nil {
			return nil, err
		}
	}
	candidates := make([]int, 0, 64)
	if budget != nil && budget.replay != nil {
		if err := budget.ReserveRecords(ctx, 1); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytes(ctx, 1, uint64(unsafe.Sizeof(ContactBatchExecutor{}))+256); err != nil {
			return nil, err
		}
	}
	contacts := NewContactBatchExecutor(ctx, ma, mb, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		ta := TriCorners(ma, i)
		tb := TriCorners(mb, j)
		if c.Kind == ContactRegion {
			var err error
			coplanar, err = appendReplay(ctx, coplanar, coplanarContactOf(ma, mb, i, j), budget)
			if err != nil {
				return err
			}
			if len(coplanar) >= maxTouchContacts {
				return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
			}
			return nil
		}
		if c.Kind == ContactNone {
			return nil
		}
		var appendErr error
		if budget != nil && budget.replay != nil {
			if err := budget.SpendWork(ctx, 2); err != nil {
				return err
			}
			if ma.Gate > 0 {
				replayTouched[0] = max(replayTouched[0], ma.SlabBound[i])
			}
			if mb.Gate > 0 {
				replayTouched[1] = max(replayTouched[1], mb.SlabBound[j])
			}
		} else {
			touchedA, appendErr = appendReplay(ctx, touchedA, i, budget)
			if appendErr != nil {
				return appendErr
			}
			touchedB, appendErr = appendReplay(ctx, touchedB, j, budget)
			if appendErr != nil {
				return appendErr
			}
		}
		if c.Kind == ContactPoint {
			pointTouches, appendErr = appendReplay(ctx, pointTouches, c.P0, budget)
			if appendErr != nil {
				return appendErr
			}
			pointFacets, appendErr = appendReplay(ctx, pointFacets, [2]int{i, j}, budget)
			if appendErr != nil {
				return appendErr
			}
			return nil
		}
		segEnds, appendErr = appendReplay(ctx, segEnds, c.P0, budget)
		if appendErr != nil {
			return appendErr
		}
		segEnds, appendErr = appendReplay(ctx, segEnds, c.P1, budget)
		if appendErr != nil {
			return appendErr
		}
		var rim float64
		if budget != nil && budget.replay != nil {
			rim, appendErr = replayRimBound(ctx, ma.SlabBound[i], mb.SlabBound[j], c.Sin2, budget.replay)
			if appendErr != nil {
				return appendErr
			}
		} else {
			rim = RimBound(ma.SlabBound[i], mb.SlabBound[j], c.Sin2)
		}
		maxRim = max(maxRim, rim)
		for _, p := range []proof.Xpt{c.P0, c.P1} {
			k, err := budget.pointKey(ctx, p, keyOf)
			if err != nil {
				return err
			}
			if err := setRim(k, rim); err != nil {
				return err
			}
		}
		if c.EdgeA >= 0 || c.EdgeB >= 0 {
			// The segment runs ALONG a facet edge. Graze or crossing is not
			// decidable here — hold it for the mesh-level call below.
			inPlane, appendErr = appendReplay(ctx, inPlane, PairContact{I: i, J: j, Ta: ta, Tb: tb, C: c}, budget)
			if appendErr != nil {
				return appendErr
			}
			return nil
		}
		if err := appendCut(0, i, Xseg{A: c.P0, B: c.P1, AOnEdge: c.P0OnA, BOnEdge: c.P1OnA, Partner: tb}); err != nil {
			return err
		}
		if err := appendCut(1, j, Xseg{A: c.P0, B: c.P1, AOnEdge: c.P0OnB, BOnEdge: c.P1OnB, Partner: ta}); err != nil {
			return err
		}
		return nil
	})
	if budget != nil && budget.replay != nil {
		for _, size := range []uintptr{unsafe.Sizeof(ContactBatchEntry{}), unsafe.Sizeof(ContactPair{}), unsafe.Sizeof(ContactBatchResult{}), unsafe.Sizeof(TriContact{})} {
			if err := budget.ReserveBytes(ctx, ContactBatchSize, uint64(size)); err != nil {
				return nil, err
			}
		}
		contacts.Workers = 1
		contacts.NoMemoLookup = true
		contacts.Run = replayContactRunner(budget.replay)
		contacts.Batch = make([]ContactBatchEntry, 0, ContactBatchSize)
		contacts.Misses = make([]ContactPair, 0, ContactBatchSize)
		contacts.Results = make([]ContactBatchResult, 0, ContactBatchSize)
		contacts.memoHits = make([]TriContact, 0, ContactBatchSize)
	}
	// A bounded pass does not promote the memo into an uncharged dense table.
	// The pair loop submits each facet pair once, so a fresh memo is unnecessary.
	contacts.NoMemoStore = budget != nil
	for i := range ma.Tris {
		if budget != nil && budget.replay != nil {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
		}
		candidates = candidates[:0]
		collect := func(j int) error {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if err := budget.ReserveRecords(ctx, 1); err != nil {
				return err
			}
			if budget != nil && budget.replay != nil && len(candidates) == cap(candidates) {
				capacity := max(1, 2*cap(candidates))
				if err := budget.ReserveBytes(ctx, uint64(capacity), uint64(unsafe.Sizeof(int(0)))); err != nil {
					return err
				}
				if err := budget.SpendWork(ctx, uint64(len(candidates))); err != nil {
					return err
				}
				next := make([]int, len(candidates), capacity)
				copy(next, candidates)
				candidates = next
			}
			candidates = append(candidates, j)
			return nil
		}
		var err error
		if budget == nil {
			err = boxTree.visitOverlaps(ma.Boxes[i], ctx.Err, collect)
		} else {
			err = boxTree.visitOverlapsWork(ma.Boxes[i], func() error { return budget.SpendWork(ctx, 1) }, treeStack, collect)
		}
		if err != nil {
			return nil, err
		}
		if budget == nil {
			sort.Ints(candidates)
		} else {
			for i := 1; i < len(candidates); i++ {
				for j := i; j > 0; j-- {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return nil, err
					}
					if candidates[j] >= candidates[j-1] {
						break
					}
					candidates[j], candidates[j-1] = candidates[j-1], candidates[j]
				}
			}
		}
		for _, j := range candidates {
			if err := budget.SpendWork(ctx, 4); err != nil {
				return nil, err
			}
			// Covers this pair's queued batch entries and its maximum retained
			// contact, endpoint, key, blocked-edge and cut records.
			if err := budget.ReserveRecords(ctx, 24); err != nil {
				return nil, err
			}
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if err := contacts.Add(i, j); err != nil {
				return nil, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		return nil, err
	}
	if len(coplanar) > 0 {
		return nil, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
	}
	// The held-mesh gate runs on the classification's own answer and before
	// any facet is cut (agent-docs/faceted-vertex-bounds-design.md §5).
	if budget != nil && budget.replay != nil {
		if err := refuseCoarseHeldMaximum(ma, 0, replayTouched[0]); err != nil {
			return nil, err
		}
		if err := refuseCoarseHeldMaximum(mb, 1, replayTouched[1]); err != nil {
			return nil, err
		}
	} else {
		if err := budget.SpendWork(ctx, uint64(len(touchedA))+uint64(len(touchedB))); err != nil {
			return nil, err
		}
		if err := RefuseCoarseHeldContact(ma, 0, touchedA); err != nil {
			return nil, err
		}
		if err := RefuseCoarseHeldContact(mb, 1, touchedB); err != nil {
			return nil, err
		}
	}

	// The graze-or-crossing call, made once, OUTSIDE the pair loop: an in-plane
	// facet edge grazes the other operand exactly when the edge's two adjacent
	// facets lie strictly on ONE side of the other facet's plane — the operand's
	// boundary touches the plane and comes back, a tangency no side
	// classification can be proven for. Apexes that STRADDLE the plane are an
	// ordinary crossing: the boundary genuinely passes through, and the segment
	// is a real rim of the result.
	var blockedA, blockedB map[[2]int]struct{}
	var replayBlockedA, replayBlockedB *replayTable[[2]int, struct{}]
	if budget != nil && budget.replay != nil {
		var err error
		replayBlockedA, err = newReplayTable[[2]int, struct{}](ctx, 3*len(ma.Tris), budget.replay, replayEdgeHash)
		if err != nil {
			return nil, err
		}
		replayBlockedB, err = newReplayTable[[2]int, struct{}](ctx, 3*len(mb.Tris), budget.replay, replayEdgeHash)
		if err != nil {
			return nil, err
		}
	} else {
		blockedA = map[[2]int]struct{}{}
		blockedB = map[[2]int]struct{}{}
	}
	setBlocked := func(side int, key [2]int) error {
		if replayBlockedA != nil {
			table := replayBlockedA
			if side == 1 {
				table = replayBlockedB
			}
			return table.Set(ctx, key, struct{}{})
		}
		if side == 0 {
			blockedA[key] = struct{}{}
		} else {
			blockedB[key] = struct{}{}
		}
		return nil
	}
	// grazes collects the facets of the first operand at every graze, so the
	// refusal can say where the operands touch.
	var grazes []TouchContact
	for i, p := range inPlane {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if p.C.EdgeA >= 0 {
			var key [2]int
			var crossing bool
			var err error
			if budget != nil && budget.replay != nil {
				key, crossing, err = edgeCrossesReplay(ctx, ma, p.I, p.C.EdgeA, mb, p.J, budget.replay)
			} else {
				key, crossing, err = EdgeCrosses(ma, p.I, p.C.EdgeA, p.Tb)
			}
			if err != nil {
				return nil, err
			}
			if !crossing {
				if len(grazes) < maxTouchContacts {
					var err error
					grazes, err = appendReplay(ctx, grazes, touchOf(ma, p.I, p.J), budget)
					if err != nil {
						return nil, err
					}
				}
				continue
			}
			if err := setBlocked(0, key); err != nil {
				return nil, err
			}
		}
		if p.C.EdgeB >= 0 {
			var key [2]int
			var crossing bool
			var err error
			if budget != nil && budget.replay != nil {
				key, crossing, err = edgeCrossesReplay(ctx, mb, p.J, p.C.EdgeB, ma, p.I, budget.replay)
			} else {
				key, crossing, err = EdgeCrosses(mb, p.J, p.C.EdgeB, p.Ta)
			}
			if err != nil {
				return nil, err
			}
			if !crossing {
				if len(grazes) < maxTouchContacts {
					var err error
					grazes, err = appendReplay(ctx, grazes, touchOf(ma, p.I, p.J), budget)
					if err != nil {
						return nil, err
					}
				}
				continue
			}
			if err := setBlocked(1, key); err != nil {
				return nil, err
			}
		}
		// A crossing segment subdivides whichever facet does NOT already own it
		// as an edge — a segment lying along a facet's own boundary cuts nothing
		// off it. Its regions classify by exact parity, never by the partner
		// facet's plane: the other operand's boundary there is the DIHEDRAL
		// between two facets, and one plane of it decides nothing.
		if p.C.EdgeA < 0 {
			if err := appendCut(0, p.I, Xseg{A: p.C.P0, B: p.C.P1, AOnEdge: p.C.P0OnA, BOnEdge: p.C.P1OnA, Partner: p.Tb, ViaParity: true}); err != nil {
				return nil, err
			}
		}
		if p.C.EdgeB < 0 {
			if err := appendCut(1, p.J, Xseg{A: p.C.P0, B: p.C.P1, AOnEdge: p.C.P0OnB, BOnEdge: p.C.P1OnB, Partner: p.Ta, ViaParity: true}); err != nil {
				return nil, err
			}
		}
	}

	if len(grazes) > 0 {
		return nil, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchGraze, Contacts: grazes})
	}

	// A point contact is legitimate only as the endpoint of some crossing
	// segment (a chain passing exactly through a vertex). An isolated one is a
	// tangency the boolean cannot classify: the operands pinch at a point, and
	// stitching it would emit a non-manifold result — refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		var ends map[string]struct{}
		var fixedEnds *replayTable[string, struct{}]
		if budget != nil && budget.replay != nil {
			var err error
			fixedEnds, err = newReplayGrowingTable[string, struct{}](ctx, len(segEnds), budget.replay, replayStringHash)
			if err != nil {
				return nil, err
			}
			fixedEnds.comparisonWork = func(s string) uint64 { return uint64(len(s)) }
		} else {
			ends = map[string]struct{}{}
		}
		for _, p := range segEnds {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			key, err := budget.pointKey(ctx, p, keyOf)
			if err != nil {
				return nil, err
			}
			if fixedEnds != nil {
				if err := fixedEnds.Set(ctx, key, struct{}{}); err != nil {
					return nil, err
				}
			} else {
				ends[key] = struct{}{}
			}
		}
		var isolated []TouchContact
		for k, p := range pointTouches {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			key, err := budget.pointKey(ctx, p, keyOf)
			if err != nil {
				return nil, err
			}
			_, ok := ends[key]
			if fixedEnds != nil {
				var err error
				_, ok, err = fixedEnds.Get(ctx, key)
				if err != nil {
					return nil, err
				}
			}
			if !ok && len(isolated) < maxTouchContacts {
				isolated, err = appendReplay(ctx, isolated, touchOf(ma, pointFacets[k][0], pointFacets[k][1]), budget)
				if err != nil {
					return nil, err
				}
			}
		}
		if len(isolated) > 0 {
			return nil, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchPoint, Contacts: isolated})
		}
	}

	return &PairCutsResult{CutsA: cutsA, CutsB: cutsB, replayCutsA: replayCutsA, replayCutsB: replayCutsB, replayBlockedA: replayBlockedA, replayBlockedB: replayBlockedB, replayRims: replayRims, BlockedA: blockedA, BlockedB: blockedB, Rims: rims, MaxRim: maxRim}, nil
}
