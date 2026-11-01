package meshbool

import (
	"context"
	"math/big"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// StitchedMesh is the boolean output after welding, conforming and rounding:
// the held float mesh, its per-facet source-face ids, and the proven terms the
// rounding's own error composes from.
// stitchFacetsContextBudgeted is a private resource-limited stitch attempt.
func stitchFacetsContextBudgeted(ctx context.Context, kept []KeptFacet, shared *PairCutsBudget) (*StitchedMesh, error) {
	if shared == nil {
		return nil, errReplayBoundedStitchNeedsASharedBudget
	}
	return stitchFacetsContext(ctx, kept, shared, brakeReplayCutterLimit)
}

// StitchFacetsReplay applies the existing bounded stitch to the shared replay counter.
func StitchFacetsReplay(ctx context.Context, kept []KeptFacet, replay *proofbound.ReplayBudget) (*StitchedMesh, error) {
	keys, err := NewReplayKeysBounded(ctx, replay)
	if err != nil {
		return nil, err
	}
	return StitchFacetsReplayWithKeys(ctx, kept, replay, keys)
}

// StitchFacetsReplayWithKeys reuses the Boolean's authenticated key memo.
func StitchFacetsReplayWithKeys(ctx context.Context, kept []KeptFacet, replay *proofbound.ReplayBudget, keys *ReplayKeys) (*StitchedMesh, error) {
	if replay == nil {
		return nil, errReplayMissingStitchReplayBudget
	}
	if err := reserveBrakeStitchFixed(ctx, replay); err != nil {
		return nil, err
	}
	if keys == nil || keys.budget != replay {
		return nil, errReplayMismatchedReplayKeys
	}
	budget := &PairCutsBudget{replay: replay, keys: keys}
	return stitchFacetsContextBudgeted(ctx, kept, budget)
}

// reserveBrakeStitchFixed precedes construction of the adapter and result.
func reserveBrakeStitchFixed(ctx context.Context, replay *proofbound.ReplayBudget) error {
	if err := replay.SpendWork(ctx, 8); err != nil {
		return err
	}
	if err := replay.ReserveRecords(ctx, 8); err != nil {
		return err
	}
	return replay.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(PairCutsBudget{}))+uint64(unsafe.Sizeof(proofbound.WorkBudget{}))+uint64(unsafe.Sizeof(StitchedMesh{}))+512)
}

func stitchFacetsContext(ctx context.Context, kept []KeptFacet, shared *PairCutsBudget, maxFacets int) (*StitchedMesh, error) {
	var work *proofbound.WorkBudget
	if shared == nil {
		work = proofbound.NewWorkBudget(ctx)
	}
	if maxFacets > 0 && len(kept) > maxFacets {
		return nil, errReplayBoundedStitchFacetCapExceeded
	}
	if shared != nil {
		work = &proofbound.WorkBudget{Replay: shared.replay != nil, ReplayID: shared.replay, StepFn: func() error { return shared.SpendWork(ctx, 1) }, ErrFn: ctx.Err,
			ReserveFn: func(n uint64) error { return shared.ReserveRecords(ctx, n) }, ReserveBytesFn: func(n uint64) error { return shared.ReserveBytes(ctx, n, 1) }}
		if len(kept) > 65536 {
			return nil, errReplayBoundedStitchFacetCapExceeded
		}
	}
	if len(kept) == 0 {
		return nil, errReplayTheOperationLeavesNoBoundaryAtAll
	}
	// Each kept facet contributes three corners before welding, so the facet
	// count is a useful estimate for the unique stitched vertices. The output
	// triangles, source ids and facet bounds retain one entry per kept facet
	// unless the conforming pass splits one.
	// Three corners per facet bound the two vertex arrays and two maps;
	// the three facet arrays each hold one entry per input facet.
	if err := work.ReserveMul(uint64(len(kept)), 15); err != nil {
		return nil, err
	}
	if shared != nil && shared.replay != nil {
		if _, err := work.StepN(9 * uint64(len(kept))); err != nil {
			return nil, err
		}
	}
	vertexCap := len(kept)
	var fixedIndex *replayTable[string, int]
	var index map[string]int
	var indexByRaw map[[4]*big.Int]int
	if shared != nil && shared.replay != nil {
		vertexCap = 3 * len(kept)
		var err error
		fixedIndex, err = newReplayTable[string, int](ctx, vertexCap, shared.replay, replayStringHash)
		if err != nil {
			return nil, err
		}
		fixedIndex.comparisonWork = func(s string) uint64 { return uint64(len(s)) }
		if err := work.ReserveBytesMul(uint64(vertexCap), uint64(unsafe.Sizeof(proof.Xpt{}))); err != nil {
			return nil, err
		}
		if err := work.ReserveBytesMul(uint64(vertexCap), uint64(unsafe.Sizeof(float64(0)))); err != nil {
			return nil, err
		}
	} else {
		index = make(map[string]int, vertexCap)
		indexByRaw = make(map[[4]*big.Int]int)
	}
	xverts := make([]proof.Xpt, 0, vertexCap)
	xbeta := make([]float64, 0, vertexCap)
	addKey := func(p proof.Xpt, key string) (int, error) {
		if fixedIndex != nil {
			i, ok, err := fixedIndex.Get(ctx, key)
			if err != nil {
				return 0, err
			}
			if !ok {
				i = len(xverts)
				if err := fixedIndex.Set(ctx, key, i); err != nil {
					return 0, err
				}
				xverts = append(xverts, p)
				xbeta = append(xbeta, 0)
			}
			return i, nil
		}
		i, ok := index[key]
		if !ok {
			i = len(xverts)
			index[key] = i
			xverts = append(xverts, p)
			xbeta = append(xbeta, 0)
		}
		return i, nil
	}
	addVert := func(p proof.Xpt, key string) (int, error) {
		if key != "" {
			return addKey(p, key)
		}
		if shared != nil && shared.keys != nil {
			resolved, err := shared.keys.Key(ctx, p)
			if err != nil {
				return 0, err
			}
			return addKey(p, resolved)
		}
		raw := [4]*big.Int{p.X, p.Y, p.Z, p.W}
		if i, ok := indexByRaw[raw]; ok {
			return i, nil
		}
		i, err := addKey(p, p.Key())
		if err != nil {
			return 0, err
		}
		indexByRaw[raw] = i
		return i, nil
	}
	if fixedIndex != nil {
		for _, size := range []uintptr{unsafe.Sizeof([3]int{}), unsafe.Sizeof(int(0)), unsafe.Sizeof(float64(0))} {
			if err := work.ReserveBytesMul(uint64(len(kept)), uint64(size)); err != nil {
				return nil, err
			}
		}
	}
	tris := make([][3]int, 0, len(kept))
	src := make([]int, 0, len(kept))
	// facetBound is each stitched triangle's δ: the largest corner claim of
	// the kept facet it descends from. A conforming split keeps its parent's
	// figure, since every sub-triangle lies on that held facet.
	facetBound := make([]float64, 0, len(kept))
	for i, f := range kept {
		if shared != nil {
			if err := work.Step(); err != nil {
				return nil, err
			}
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, err := addVert(f.V[0], f.Keys[0])
		if err != nil {
			return nil, err
		}
		b, err := addVert(f.V[1], f.Keys[1])
		if err != nil {
			return nil, err
		}
		c, err := addVert(f.V[2], f.Keys[2])
		if err != nil {
			return nil, err
		}
		if a == b || b == c || c == a {
			return nil, errReplayAKeptFacetCollapsed
		}
		tris = append(tris, [3]int{a, b, c})
		src = append(src, f.Src)
		for k, vi := range [3]int{a, b, c} {
			xbeta[vi] = max(xbeta[vi], f.Beta[k])
		}
		facetBound = append(facetBound, max(f.Beta[0], f.Beta[1], f.Beta[2]))
	}

	for round := 0; ; round++ {
		if round > 6 {
			return nil, errReplayTheConformingPassDidNotConverge
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var split bool
		var err error
		if shared == nil && maxFacets == 0 {
			split, err = ConformOnce(ctx, xverts, &tris, &src, &facetBound, xbeta)
		} else {
			split, err = brakeconformOnce(ctx, xverts, &tris, &src, &facetBound, xbeta, work, maxFacets)
		}
		if err != nil {
			return nil, err
		}
		if shared != nil && len(tris) > 65536 {
			return nil, errReplayBoundedStitchFacetCapExceeded
		}
		if !split {
			break
		}
	}

	if shared != nil {
		// Closure and weld component checks visit bounded multiples of each
		// facet; the subsequent embedding search charges its own candidates.
		// Reserve closure maps, welded output, component queues, exact/held
		// remaps and gap arrays before any of those allocations.
		if err := shared.SpendWork(ctx, 30*uint64(len(tris))+12*uint64(len(xverts))); err != nil {
			return nil, err
		}
		if err := shared.ReserveRecords(ctx, 20*uint64(len(tris))+16*uint64(len(xverts))+16); err != nil {
			return nil, err
		}
	}

	// Closure audit: each directed edge exactly once, each with its reverse.
	// A closed boundary has three edge uses per triangle and two per key.
	// Open boundaries may grow the table; this hint admits no geometry.
	var edges map[[2]int]uint8
	if shared != nil && shared.replay != nil {
		ok, err := stitchEdgesPairedReplay(ctx, tris, shared.replay)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, errReplayTheStitchedBoundaryDoesNotClose
		}
	} else {
		edges = make(map[[2]int]uint8, len(tris)+len(tris)/2)
		if !stitchEdgesPaired(tris, edges) {
			return nil, errReplayTheStitchedBoundaryDoesNotClose
		}
	}
	// Retain only a small table for the rounded audit. Release a large one
	// before the rounding and embedding work allocates its own geometry.
	if len(tris) > 4096 {
		edges = nil
	}

	// Round to float64, welding vertices whose roundings coincide — two exact
	// points closer than an ulp become one held vertex — and drop the facets
	// that collapse under the weld: a collapsed facet's two real directed
	// edges cancel each other, so closure survives, and the final audit
	// re-proves it. A collapsed facet is a zero-area triangle, so it moves
	// neither the volume integral nor the area sum of the HELD mesh — but the
	// facet it stands for was not zero-area before the weld, and what it did
	// carry has to be answered for. Two answers, below: the swept volume and
	// the missing area are charged against the PRE-ROUND surface (preArea,
	// dropArea), and a whole component welded out of existence is REFUSED —
	// there no bound would help, because a lump would be gone from the body
	// (its volume, its place in the lump count, its share of the bounds) while
	// the closure audit, which the surviving components still pass, reports
	// nothing wrong.
	var floatIdx map[r3.Vec]int
	var fixedFloat *replayTable[r3.Vec, int]
	if shared != nil && shared.replay != nil {
		outputRecords := 2*uint64(len(xverts)) + 4*uint64(len(tris))
		if err := shared.ReserveRecords(ctx, outputRecords); err != nil {
			return nil, err
		}
		if err := shared.SpendWork(ctx, outputRecords); err != nil {
			return nil, err
		}
		var err error
		fixedFloat, err = newReplayTable[r3.Vec, int](ctx, len(xverts), shared.replay, replayVecHash)
		if err != nil {
			return nil, err
		}
		for _, allocation := range [][2]uint64{{uint64(len(xverts)), uint64(unsafe.Sizeof(r3.Vec{}))}, {uint64(len(tris)), uint64(unsafe.Sizeof([3]int{}))}, {uint64(len(tris)), uint64(unsafe.Sizeof(int(0)))}, {uint64(len(xverts)), uint64(unsafe.Sizeof(int(0)))}, {uint64(len(tris)), uint64(unsafe.Sizeof(false))}, {uint64(len(tris)), uint64(unsafe.Sizeof([3]int{}))}} {
			if err := shared.ReserveBytes(ctx, allocation[0], allocation[1]); err != nil {
				return nil, err
			}
		}
	} else {
		floatIdx = make(map[r3.Vec]int, len(xverts))
	}
	out := &StitchedMesh{
		Verts: make([]r3.Vec, 0, len(xverts)),
		Tris:  make([][3]int, 0, len(tris)),
		Src:   make([]int, 0, len(tris)),
	}
	remap := make([]int, len(xverts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		var v r3.Vec
		if shared != nil && shared.replay != nil {
			var err error
			v, err = proof.ReplayXptVec(ctx, p, replayExactWork{work: work})
			if err != nil {
				return nil, err
			}
		} else {
			v = p.Vec()
		}
		fi, ok := floatIdx[v]
		if fixedFloat != nil {
			if !proofbound.FiniteVec(v) {
				return nil, errReplayNonfiniteRoundedReplayVertex
			}
			var err error
			fi, ok, err = fixedFloat.Get(ctx, v)
			if err != nil {
				return nil, err
			}
		}
		if !ok {
			fi = len(out.Verts)
			if fixedFloat != nil {
				if err := fixedFloat.Set(ctx, v, fi); err != nil {
					return nil, err
				}
			} else {
				floatIdx[v] = fi
			}
			out.Verts = append(out.Verts, v)
		}
		remap[i] = fi
	}

	dropped := make([]bool, len(tris))
	welded := make([][3]int, len(tris))
	for ti, tri := range tris {
		a, b, c := remap[tri[0]], remap[tri[1]], remap[tri[2]]
		welded[ti] = [3]int{a, b, c}
		if a == b || b == c || c == a {
			dropped[ti] = true
			continue
		}
		out.Tris = append(out.Tris, [3]int{a, b, c})
		out.Src = append(out.Src, src[ti])
	}
	if len(out.Tris) == 0 {
		return nil, errReplayTheWholeResultCollapsedUnderRounding
	}
	if err := RefuseWeldedAwayComponent(ctx, tris, dropped); err != nil {
		return nil, err
	}
	var gaps []*big.Rat
	var err error
	if shared != nil {
		gaps, err = keepRoundedEmbeddedReplay(ctx, xverts, remap, out, work)
	} else {
		gaps, err = keepRoundedEmbedded(ctx, xverts, remap, out)
	}
	if err != nil {
		return nil, err
	}
	// worst is the max PER-COORDINATE distance from an exact vertex to the
	// held float that stands for it — measured AFTER the embedding check, so a
	// vertex it placed at another corner of its float box is charged where it
	// actually sits. The consumers read a 3D distance bound, and all three
	// coordinates can round at once (internal/proofbound/bounds.go,
	// proofbound.Radius3D). A positive rational error can round to zero as
	// float64, so preserve that proof before widening it to a 3D radius.
	var worst *big.Rat
	var vertexBound []float64
	if work.Replay {
		worst, vertexBound, err = replayGapBounds(ctx, work, gaps, xbeta, remap, len(out.Verts))
	} else {
		worst, err = maxGap(ctx, gaps)
		if err == nil {
			vertexBound, err = heldVertexBoundsFromGaps(ctx, gaps, xbeta, remap, len(out.Verts))
		}
	}
	if err != nil {
		return nil, err
	}
	if worst.Sign() > 0 {
		var w float64
		if work.Replay {
			w, err = proof.ReplayRatFloat64(ctx, worst, replayExactWork{work})
			if err != nil {
				return nil, err
			}
		} else {
			w, _ = worst.Float64()
		}
		out.Round = proofbound.Radius3D(proofbound.ProvenUpRound(w))
	}
	if err != nil {
		return nil, err
	}
	out.VertexBound = vertexBound
	// The pre-round surface: every facet the exact stitch produced, dropped
	// ones included, measured on the held vertices and inflated by the
	// rounding they may each have travelled (internal/proofbound/bounds.go, proofbound.PerturbedAreaUpper).
	if out.EmbeddingReach == nil {
		if shared != nil {
			if err := shared.ReserveRecords(ctx, uint64(len(welded))+2*uint64(len(out.Verts))+4); err != nil {
				return nil, err
			}
			if err := shared.SpendWork(ctx, uint64(len(welded))); err != nil {
				return nil, err
			}
			out.PreArea, err = proofbound.PerturbedAreaUpperReplay(ctx, work, out.Verts, welded, out.Round)
			if err != nil {
				return nil, err
			}
		} else {
			out.PreArea = proofbound.PerturbedAreaUpper(out.Verts, welded, out.Round)
		}
	} else {
		if shared != nil {
			return nil, errReplayBoundedStitchReachRepairIsUnavailable
		}
		// Zero pre-weld bounds isolate rounding from inherited geometry error.
		rounding, err := heldVertexBoundsFromGaps(ctx, gaps, make([]float64, len(gaps)), remap, len(out.Verts))
		if err != nil {
			return nil, err
		}
		area, volume, err := proofbound.PerturbedAreaAndRoundingVolumeContext(ctx, out.Verts, welded, out.Round, rounding)
		if err != nil {
			return nil, err
		}
		out.PreArea, out.localRoundVolume = area, &volume
	}
	var droppedTris [][3]int
	if shared != nil && shared.replay != nil {
		droppedTris, err = collectDroppedTrianglesReplay(ctx, dropped, welded, shared)
		if err != nil {
			return nil, err
		}
	} else {
		for ti := range tris {
			if dropped[ti] {
				droppedTris = append(droppedTris, welded[ti])
			}
		}
	}
	if shared != nil {
		records := 2*uint64(len(out.Verts)) + 4
		if shared.replay == nil {
			records += uint64(len(droppedTris))
		}
		if err := shared.ReserveRecords(ctx, records); err != nil {
			return nil, err
		}
		if err := shared.SpendWork(ctx, uint64(len(droppedTris))); err != nil {
			return nil, err
		}
		out.DropArea, err = proofbound.PerturbedAreaUpperReplay(ctx, work, out.Verts, droppedTris, out.Round)
		if err != nil {
			return nil, err
		}
	} else {
		out.DropArea = proofbound.PerturbedAreaUpper(out.Verts, droppedTris, out.Round)
	}

	if shared != nil && shared.replay != nil {
		ok, err := stitchEdgesPairedReplay(ctx, out.Tris, shared.replay)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, errReplayTheRoundedBoundaryDoesNotClose
		}
	} else {
		if edges == nil {
			edges = make(map[[2]int]uint8, len(out.Tris)+len(out.Tris)/2)
		} else {
			clear(edges)
		}
		if !stitchEdgesPaired(out.Tris, edges) {
			return nil, errReplayTheRoundedBoundaryDoesNotClose
		}
	}
	return out, nil
}

// collectDroppedTrianglesReplay reserves the exact dropped-facet buffer before allocation.
func collectDroppedTrianglesReplay(ctx context.Context, dropped []bool, welded [][3]int, shared *PairCutsBudget) ([][3]int, error) {
	if shared == nil || shared.replay == nil || len(dropped) != len(welded) {
		return nil, errReplayInvalidReplayDroppedFacets
	}
	if err := shared.SpendWork(ctx, uint64(len(dropped))); err != nil {
		return nil, err
	}
	count := 0
	for _, drop := range dropped {
		if drop {
			count++
		}
	}
	if count == 0 {
		return nil, nil
	}
	if err := shared.ReserveRecords(ctx, uint64(count)); err != nil {
		return nil, err
	}
	if err := shared.ReserveBytes(ctx, uint64(count), uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return nil, err
	}
	if err := shared.SpendWork(ctx, uint64(len(dropped))); err != nil {
		return nil, err
	}
	out := make([][3]int, 0, count)
	for i, drop := range dropped {
		if drop {
			out = append(out, welded[i])
		}
	}
	return out, nil
}

func stitchEdgesPairedReplay(ctx context.Context, tris [][3]int, budget *proofbound.ReplayBudget) (bool, error) {
	if len(tris) > int(^uint(0)>>1)/3 {
		return false, errReplayStitchEdgeCapacityOverflow
	}
	edges, err := newReplayTable[[2]int, uint8](ctx, 3*len(tris), budget, replayEdgeHash)
	if err != nil {
		return false, err
	}
	for _, tri := range tris {
		for k, a := range tri {
			b := tri[(k+1)%3]
			direction := uint8(1)
			if a > b {
				a, b = b, a
				direction = 2
			} else if a == b {
				direction = 3
			}
			key := [2]int{a, b}
			seen, _, err := edges.Get(ctx, key)
			if err != nil {
				return false, err
			}
			if seen&direction != 0 {
				return false, nil
			}
			if err := edges.Set(ctx, key, seen|direction); err != nil {
				return false, err
			}
		}
	}
	for _, entry := range edges.slots {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if entry.used && entry.value != 3 {
			return false, nil
		}
	}
	return true, nil
}

// stitchEdgesPaired proves that every directed edge appears once and has its
// reverse. edges must be empty. Each undirected key records the two directions
// as bits; a repeated bit fails immediately, so a third use cannot cancel a
// duplicate in the opposite direction. No reverse-key lookup is needed.
func keepRoundedEmbeddedReplay(ctx context.Context, xverts []proof.Xpt, remap []int, out *StitchedMesh, work *proofbound.WorkBudget) ([]*big.Rat, error) {
	if work != nil && work.Replay {
		count := uint64(len(xverts)) + 4*uint64(len(out.Verts))
		if err := work.Reserve(count); err != nil {
			return nil, err
		}
		if _, err := work.StepN(2 * count); err != nil {
			return nil, err
		}
		for _, a := range []struct{ n, size uint64 }{{uint64(len(xverts)), uint64(unsafe.Sizeof((*big.Rat)(nil)))}, {uint64(len(out.Verts)), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(len(out.Verts)), uint64(unsafe.Sizeof(false))}, {uint64(len(out.Verts)), uint64(unsafe.Sizeof(false))}, {uint64(len(out.Verts)), uint64(unsafe.Sizeof(int(0)))}} {
			if err := work.ReserveBytesMul(a.n, a.size); err != nil {
				return nil, err
			}
		}
	}
	gaps := make([]*big.Rat, len(xverts))
	h := HeldRounding{
		Verts:   out.Verts,
		Tris:    out.Tris,
		Exact:   make([]proof.Xpt, len(out.Verts)),
		Moved:   make([]bool, len(out.Verts)),
		Movable: make([]bool, len(out.Verts)),
		Reach:   gaptol.Reach(ctx),
	}
	count := make([]int, len(out.Verts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		fi := remap[i]
		count[fi]++
		h.Exact[fi] = p
		if work != nil && work.Replay {
			var err error
			gaps[i], err = replayCoordDistance(ctx, work, p, out.Verts[fi])
			if err != nil {
				return nil, err
			}
		} else {
			gaps[i] = CoordDistance(p, out.Verts[fi])
		}
		if gaps[i].Sign() != 0 {
			h.Moved[fi] = true
		}
	}
	couldMove := false
	for fi, n := range count {
		if n > 1 {
			h.Moved[fi] = true
		}
		h.Movable[fi] = n == 1 && h.Moved[fi]
		couldMove = couldMove || h.Movable[fi]
	}
	var before []r3.Vec
	if couldMove {
		if work != nil && work.Replay {
			if err := work.Reserve(uint64(len(out.Verts))); err != nil {
				return nil, err
			}
			if _, err := work.StepN(uint64(len(out.Verts))); err != nil {
				return nil, err
			}
			if err := work.ReserveBytesMul(uint64(len(out.Verts)), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
				return nil, err
			}
		}
		before = slices.Clone(out.Verts)
	}
	var moved int
	var repair *EmbeddingReach
	var err error
	if work != nil {
		moved, repair, err = EnforceHeldEmbeddingReplay(ctx, h, work)
	} else {
		moved, repair, err = EnforceHeldEmbeddingWithSummary(ctx, h)
	}
	if err != nil {
		return nil, err
	}
	out.EmbeddingReach = repair
	if moved > 0 {
		for i, p := range xverts {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			fi := remap[i]
			if out.Verts[fi] != before[fi] {
				if work != nil && work.Replay {
					gaps[i], err = replayCoordDistance(ctx, work, p, out.Verts[fi])
					if err != nil {
						return nil, err
					}
				} else {
					gaps[i] = CoordDistance(p, out.Verts[fi])
				}
			}
		}
	}
	return gaps, nil
}

// RefuseWeldedAwayComponent refuses the one class of weld collapse no bound can
// answer for: a connected component of the stitched surface EVERY facet of
// which the weld collapses. That component is a shell — a lump of the result,
// or a cavity inside one — and dropping all of it removes the lump from the
// body entirely: its volume, its place in Lumps(), its reach in the bounds box.
// The closure audit does not catch it, because the components that remain still
// close; nothing else in the pipeline would report it either. Every other
// collapse is an edge contraction WITHIN a component that survives, and the
// rounding bounds account for it: the swept volume is charged against the
// pre-round surface (preArea) and the missing facet area against dropArea, both
// of which count the dropped facets.
