package meshbool

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const brakeReplayFacetLimit = 65536

// MeshBooleanReplayWithKeys shares exact keys with the later stitch.
func MeshBooleanReplayWithKeys(ctx context.Context, op OperationKind, ma, mb *BoolMesh, memo *ContactMemo, replay *proofbound.ReplayBudget, keys *ReplayKeys) ([]KeptFacet, float64, *ContactKeys, error) {
	if replay == nil || keys == nil || keys.budget != replay {
		return nil, 0, nil, errReplayMismatchedReplayKeys
	}
	if err := replay.ReserveRecords(ctx, 1); err != nil {
		return nil, 0, nil, err
	}
	if err := replay.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(PairCutsBudget{}))); err != nil {
		return nil, 0, nil, err
	}
	budget := &PairCutsBudget{replay: replay, keys: keys}
	wantA, wantB, flipB, err := BooleanKeep(op)
	if err != nil {
		return nil, 0, nil, err
	}
	keyOf := func(p proof.Xpt) string { return p.Key() }
	cuts, err := boundedPairCuts(ctx, ma, mb, memo, keyOf, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	a, err := brakeKeepSideReplay(ctx, ma, mb, cuts.CutsA, cuts.replayCutsA, cuts.BlockedA, cuts.replayBlockedA, cuts.Rims, cuts.replayRims, wantA, false, keyOf, budget, keepOptions{Coplanar: cuts.CoplanarA, Op: op})
	if err != nil {
		return nil, 0, nil, err
	}
	b, err := brakeKeepSideReplay(ctx, mb, ma, cuts.CutsB, cuts.replayCutsB, cuts.BlockedB, cuts.replayBlockedB, cuts.Rims, cuts.replayRims, wantB, flipB, keyOf, budget, keepOptions{Coplanar: cuts.CoplanarB, Op: op, Second: true})
	if err != nil {
		return nil, 0, nil, err
	}
	if len(a)+len(b) > brakeReplayFacetLimit {
		return nil, 0, nil, errReplayBoundedOutputExceedsFacets
	}
	if err := budget.ReserveRecords(ctx, uint64(len(a)+len(b))); err != nil {
		return nil, 0, nil, err
	}
	if err := budget.SpendWork(ctx, uint64(len(a)+len(b))); err != nil {
		return nil, 0, nil, err
	}
	if err := budget.ReserveBytes(ctx, uint64(len(a)+len(b)), uint64(unsafe.Sizeof(KeptFacet{}))); err != nil {
		return nil, 0, nil, err
	}
	for i := range b {
		b[i].Second = true
	}
	out := make([]KeptFacet, 0, len(a)+len(b))
	out = append(out, a...)
	out = append(out, b...)
	var coplanarKeys *ContactKeys
	if len(cuts.Owned) > 0 {
		coplanarKeys = &ContactKeys{Owned: cuts.Owned, Legacy: cuts.Legacy}
	}
	return out, cuts.MaxRim, coplanarKeys, nil
}

func brakeKeepSideReplay(ctx context.Context, m, other *BoolMesh, cuts map[int][]Xseg, replayCuts [][]Xseg, blocked map[[2]int]struct{}, replayBlocked *replayTable[[2]int, struct{}], rims map[string]float64, replayRims *replayTable[string, float64], wantInside, flip bool, keyOf func(proof.Xpt) string, shared *PairCutsBudget, opts keepOptions) ([]KeptFacet, error) {
	var kept []KeptFacet
	// A facet with a coplanar partner takes the cut path even with no segment
	// (agent-docs/coplanar-contact-design.md §3.4).
	getCuts := func(i int) ([]Xseg, bool) {
		var segs []Xseg
		var ok bool
		switch {
		case replayCuts != nil:
			if i < 0 || i >= len(replayCuts) {
				return nil, false
			}
			segs, ok = replayCuts[i], len(replayCuts[i]) > 0
		default:
			segs, ok = cuts[i]
		}
		return segs, ok || len(opts.Coplanar[i]) > 0
	}
	emit := func(tri [3]proof.Xpt, beta [3]float64, keys [3]string, src int, facet int) error {
		if shared != nil {
			if err := shared.SpendWork(ctx, 1); err != nil {
				return err
			}
			if len(kept) >= brakeReplayFacetLimit {
				return errReplayBoundedOutputExceedsFacets
			}
			if len(kept) == cap(kept) {
				n := max(1, 2*cap(kept))
				if err := shared.ReserveRecords(ctx, uint64(n)); err != nil {
					return err
				}
				if err := shared.SpendWork(ctx, uint64(len(kept))); err != nil {
					return err
				}
				if err := shared.ReserveBytes(ctx, uint64(n), uint64(unsafe.Sizeof(KeptFacet{}))); err != nil {
					return err
				}
				next := make([]KeptFacet, len(kept), n)
				copy(next, kept)
				kept = next
			}
		}
		if flip {
			tri[1], tri[2] = tri[2], tri[1]
			beta[1], beta[2] = beta[2], beta[1]
			keys[1], keys[2] = keys[2], keys[1]
		}
		kept = append(kept, KeptFacet{V: tri, Src: src, Beta: beta, Keys: keys, Delta: m.FacetBound[facet], Facet: facet})
		return nil
	}

	// The cut facets: exact subdivision along the contact chains, then the
	// exact local side-of-contact classification per region.
	var cutFacets []int
	if shared != nil {
		if err := shared.ReserveRecords(ctx, 2*uint64(len(m.Tris))); err != nil {
			return nil, err
		}
	}
	if shared != nil && shared.replay != nil {
		if err := shared.ReserveBytes(ctx, uint64(len(m.Tris)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
		if err := shared.SpendWork(ctx, uint64(len(m.Tris))); err != nil {
			return nil, err
		}
		cutFacets = make([]int, 0, len(m.Tris))
	}
	for i := range m.Tris {
		if shared != nil {
			if err := shared.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if _, ok := getCuts(i); ok {
			cutFacets = append(cutFacets, i)
		}
	}
	// Limit live subdivision output while independent facets use separate
	// workers. Consume each batch in the original facet and region order.
	const cutBatchSize = 128
	for start := 0; start < len(cutFacets); start += cutBatchSize {
		facets := cutFacets[start:min(start+cutBatchSize, len(cutFacets))]
		var results []cutFacetResult
		var err error
		if shared != nil {
			if err := shared.ReserveRecords(ctx, uint64(len(facets))); err != nil {
				return nil, err
			}
			if shared.replay != nil {
				if err := shared.ReserveBytes(ctx, uint64(len(facets)), uint64(unsafe.Sizeof(cutFacetResult{}))); err != nil {
					return nil, err
				}
			}
			if err := shared.SpendWork(ctx, uint64(len(facets))); err != nil {
				return nil, err
			}
			results = make([]cutFacetResult, len(facets))
			for slot, i := range facets {
				segments, _ := getCuts(i)
				if len(segments) == 0 {
					// Only coplanar partners: one region, the facet itself, and
					// CutTriangle is not called.
					x := XtriCorners(m, i)
					probe, perr := proof.ReplayCentroid(ctx, x[0], x[1], x[2], shared.replay)
					if perr != nil {
						return nil, perr
					}
					results[slot].regions = []CutRegion{{Tris: [][3]proof.Xpt{x}, Probe: probe}}
					continue
				}
				results[slot].regions, results[slot].err = cutTriangleSharedWork(ctx, XtriCorners(m, i), m.Norms[i], segments, shared)

				if results[slot].err != nil {
					break
				}
			}
		} else {
			results, err = subdivideCutFacets(ctx, m, cuts, facets)
		}
		if err != nil {
			return nil, err
		}
		for slot, i := range facets {
			result := results[slot]
			if result.err != nil {
				return nil, result.err
			}
			corners := m.Tris[i]
			var cornerKeys [3]string
			for k, index := range corners {
				cornerKeys[k], err = shared.pointKey(ctx, m.Xverts[index], keyOf)
				if err != nil {
					return nil, err
				}
			}
			for _, reg := range result.regions {
				var inside bool
				var err error
				if partners := opts.Coplanar[i]; len(partners) > 0 {
					// The cover test is exact work on every partner.
					if err := shared.SpendWork(ctx, coplanarCoverWork*uint64(len(partners))); err != nil {
						return nil, err
					}
					var twin func(m *BoolMesh, f, k int) (int, bool, error)
					if shared != nil && shared.replay != nil {
						twin = func(mm *BoolMesh, f, k int) (int, bool, error) { return mm.twinFacetReplay(ctx, f, k) }
					}
					cover, err := coverOf(other, partners, reg.Probe, twin)
					if err != nil {
						return nil, err
					}
					if len(cover) > 0 {
						same, err := coverFacing(m, i, other, cover)
						if err != nil {
							return nil, err
						}
						if coplanarKeep(opts.Op, opts.Second, same) {
							if err := emitReplayRegion(ctx, reg, i, cornerKeys, keyOf, m, rims, replayRims, shared, emit); err != nil {
								return nil, err
							}
						}
						continue
					}
				}
				if shared != nil {
					inside, err = classifyRegionReplay(ctx, reg, other, shared)
				} else {
					inside, err = ClassifyRegion(ctx, reg, other)
				}

				if err != nil {
					return nil, err
				}
				if inside != wantInside {
					continue
				}
				if err := emitReplayRegion(ctx, reg, i, cornerKeys, keyOf, m, rims, replayRims, shared, emit); err != nil {
					return nil, err
				}
			}
		}
	}

	// The uncut facets: constant classification per connected uncut
	// component, one exact parity seed each.
	if shared != nil {
		// Component array, adjacency map/slices and all queue/member growth.
		if err := shared.ReserveRecords(ctx, 12*uint64(len(m.Tris))+1); err != nil {
			return nil, err
		}
		if err := shared.SpendWork(ctx, 8*uint64(len(m.Tris))); err != nil {
			return nil, err
		}
	}
	if shared != nil && shared.replay != nil {
		if err := shared.ReserveBytes(ctx, 3*uint64(len(m.Tris)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
	}
	comp := make([]int, len(m.Tris))
	for i := range comp {
		comp[i] = -1
	}
	var adj [][]int
	var err error
	if shared != nil && shared.replay != nil {
		adj, err = facetAdjacencyReplay(ctx, m, shared.replay)
	} else {
		adj, err = FacetAdjacencyContext(ctx, m.Tris)
	}
	if err != nil {
		return nil, err
	}
	var replayQueue, replayMembers []int
	if shared != nil && shared.replay != nil {
		replayQueue = make([]int, 0, len(m.Tris))
		replayMembers = make([]int, 0, len(m.Tris))
	}
	next := 0
	componentWork := 0
	for i := range m.Tris {
		if shared != nil {
			if err := shared.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if _, cut := getCuts(i); cut || comp[i] != -1 {
			continue
		}
		id := next
		next++
		var queue, members []int
		if shared != nil && shared.replay != nil {
			queue, members = replayQueue[:0], replayMembers[:0]
		}
		queue = append(queue, i)
		comp[i] = id
		for queuePos := 0; queuePos < len(queue); queuePos++ {
			if shared != nil {
				if err := shared.SpendWork(ctx, 1); err != nil {
					return nil, err
				}
			}
			componentWork++
			if componentWork%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			f := queue[queuePos]
			members = append(members, f)
			for _, nb := range adj[f] {
				if shared != nil {
					if err := shared.SpendWork(ctx, 1); err != nil {
						return nil, err
					}
				}
				if _, cut := getCuts(nb); cut || comp[nb] != -1 {
					continue
				}
				isBlocked := false
				if replayBlocked != nil {
					var err error
					isBlocked, err = blockedBetweenReplay(ctx, m, f, nb, replayBlocked)
					if err != nil {
						return nil, err
					}
				} else {
					isBlocked = BlockedBetween(m, f, nb, blocked)
				}
				if isBlocked {
					continue
				}
				comp[nb] = id
				queue = append(queue, nb)
			}
		}
		// Every facet has a real interior to probe: prepBoolMesh refused the
		// operand outright if any facet collapsed.
		seed := XtriCorners(m, members[0])
		var probe proof.Xpt
		if shared != nil && shared.replay != nil {
			probe, err = proof.ReplayCentroid(ctx, seed[0], seed[1], seed[2], shared.replay)
			if err != nil {
				return nil, err
			}
		} else {
			probe = XCentroid(seed[0], seed[1], seed[2])
		}
		var inside, onBoundary bool
		if shared != nil {
			work, allocErr := newBrakeParityWork(ctx, shared)
			if allocErr != nil {
				return nil, allocErr
			}
			inside, onBoundary, err = meshParityIndexedShared(ctx, probe, other.Parity, work)
		} else {
			inside, onBoundary, err = MeshParityAllContext(ctx, probe, other.Parity)
		}
		if err != nil {
			return nil, err
		}
		if onBoundary {
			if shared != nil && shared.replay != nil {
				return nil, shared.boundaryContactRefusal()
			}
			return nil, ErrUnclassifiableContact(`an operand facet touches the other operand's boundary`)
		}
		if inside != wantInside {
			continue
		}
		for _, f := range members {
			t := m.Tris[f]
			x := XtriCorners(m, f)
			var keys [3]string
			for j := range x {
				keys[j], err = shared.pointKey(ctx, x[j], keyOf)
				if err != nil {
					return nil, err
				}
			}
			if err := emit(x, [3]float64{m.VertexBound[t[0]], m.VertexBound[t[1]], m.VertexBound[t[2]]}, keys, m.Src[f], f); err != nil {
				return nil, err
			}
		}
	}
	return kept, nil
}

func cutCornerBoundReplay(ctx context.Context, m *BoolMesh, i int, cornerKeys [3]string, key string, rims *replayTable[string, float64]) (float64, error) {
	beta, known := 0.0, false
	for k, ck := range cornerKeys {
		if err := rims.budget.SpendWork(ctx, uint64(len(key))+1); err != nil {
			return 0, err
		}
		if ck == key {
			beta, known = max(beta, m.VertexBound[m.Tris[i][k]]), true
		}
	}
	rim, ok, err := rims.Get(ctx, key)
	if err != nil {
		return 0, err
	}
	if ok {
		beta, known = max(beta, rim), true
	}
	if !known {
		return m.FacetBound[i], nil
	}
	return beta, nil
}

func classifyRegionReplay(ctx context.Context, reg CutRegion, other *BoolMesh, budget *PairCutsBudget) (bool, error) {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if reg.HasAnchor {
		if budget.replay == nil {
			return ClassifyRegion(ctx, reg, other)
		}
		sign, err := proof.ReplayOrientSignMixed(ctx, reg.Partner[0], reg.Partner[1], reg.Partner[2], reg.Probe, budget.replay)
		if err != nil {
			return false, err
		}
		if sign < 0 {
			return true, nil
		}
		if sign > 0 {
			return false, nil
		}
		return false, errReplayARegionProbeLandedOnItsContact
	}
	if budget.replay == nil {
		return ClassifyRegion(ctx, reg, other)
	}
	// An unanchored region, which a segment that runs along an edge or a
	// coplanar patch boundary leaves, is classified by exact parity of its probe
	// on the replay meter, as an uncut component's seed is.
	work, err := newBrakeParityWork(ctx, budget)
	if err != nil {
		return false, err
	}
	inside, onBoundary, err := meshParityIndexedShared(ctx, reg.Probe, other.Parity, work)
	if err != nil {
		return false, err
	}
	if onBoundary {
		return false, budget.boundaryContactRefusal()
	}
	return inside, nil
}

func blockedBetweenReplay(ctx context.Context, m *BoolMesh, a, b int, blocked *replayTable[[2]int, struct{}]) (bool, error) {
	ta, tb := m.Tris[a], m.Tris[b]
	for k := range 3 {
		u, v := ta[k], ta[(k+1)%3]
		_, ok, err := blocked.Get(ctx, [2]int{min(u, v), max(u, v)})
		if err != nil {
			return false, err
		}
		if ok && SharesVertices(tb, u, v) {
			return true, nil
		}
	}
	return false, nil
}

func meshParityIndexedShared(ctx context.Context, p proof.Xpt, pm *ParityMesh, budget *proofbound.WorkBudget) (bool, bool, error) {
	if budget == nil || !budget.Replay {
		return false, false, errReplayReplayParityNeedsAReplayBudget
	}
	return replayParityScan(ctx, p, pm, budget)
}

func newBrakeParityWork(ctx context.Context, budget *PairCutsBudget) (*proofbound.WorkBudget, error) {
	if budget == nil || budget.replay == nil {
		return nil, errReplayMissingParityReplayBudget
	}
	if err := budget.ReserveRecords(ctx, 6); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, 1, uint64(unsafe.Sizeof(proofbound.WorkBudget{}))+256); err != nil {
		return nil, err
	}
	work := &proofbound.WorkBudget{Replay: true, ReplayID: budget.replay, StepFn: func() error { return budget.SpendWork(ctx, 1) }, ReserveFn: func(n uint64) error { return budget.ReserveRecords(ctx, n) }, ReserveBytesFn: func(n uint64) error { return budget.ReserveBytes(ctx, n, 1) }, ErrFn: ctx.Err}
	work.SpendWorkFn = func(n uint64) error { return budget.SpendWork(ctx, n) }
	return work, nil
}

// coplanarCoverWork is the work charged for one exact cover test of a probe
// against one coplanar partner.
const coplanarCoverWork = 256

// emitReplayRegion emits the triangles of one kept region of facet i, each
// corner with the bound the cutter gives it.
func emitReplayRegion(ctx context.Context, reg CutRegion, i int, cornerKeys [3]string, keyOf func(proof.Xpt) string, m *BoolMesh,
	rims map[string]float64, replayRims *replayTable[string, float64], shared *PairCutsBudget,
	emit func(tri [3]proof.Xpt, beta [3]float64, keys [3]string, src int, facet int) error) error {
	for _, tri := range reg.Tris {
		var beta [3]float64
		var keys [3]string
		var err error
		for k, p := range tri {
			keys[k], err = shared.pointKey(ctx, p, keyOf)
			if err != nil {
				return err
			}
			if replayRims != nil {
				beta[k], err = cutCornerBoundReplay(ctx, m, i, cornerKeys, keys[k], replayRims)
				if err != nil {
					return err
				}
			} else {
				beta[k] = CutCornerBound(m, i, cornerKeys, keys[k], rims)
			}
		}
		if err := emit(tri, beta, keys, m.Src[i], i); err != nil {
			return err
		}
	}
	return nil
}
