package loftmesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const tiledBlockSize = 2048
const tiledScanLimit = 64_000_000
const tiledTableFacetLimit = 8_000_000

// LoftCrossingAuditTiled audits a large held shell without lifting the entire
// shell into exact coordinates. The caller supplies its facet ceiling.
func LoftCrossingAuditTiled(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, maxFacets int) error {
	return loftCrossingAuditTiled(budget, verts, tris, maxFacets, tiledBlockSize)
}

func loftCrossingAuditTiled(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, maxFacets, blockSize int) error {
	return loftCrossingAuditTiledLimits(budget, verts, tris, maxFacets, blockSize, tiledScanLimit, proofbound.MaxFacetPairTestsPerCall, tiledTableFacetLimit)
}

func loftCrossingAuditTiledLimits(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, maxFacets, blockSize int, scanLimit, pairLimit, tableLimit uint64) error {
	if err := budget.Err(); err != nil {
		return err
	}
	if len(tris) > maxFacets {
		return proofbound.WithWorkLimit(fmt.Errorf("%w: tiled audit facet ceiling", decaderr.ErrUnsupported), "mesh_facets", uint64(len(tris)), uint64(maxFacets))
	}
	for _, v := range verts {
		if err := budget.Step(); err != nil {
			return err
		}
		if math.IsNaN(v.X) || math.IsNaN(v.Y) || math.IsNaN(v.Z) || math.IsInf(v.X, 0) || math.IsInf(v.Y, 0) || math.IsInf(v.Z, 0) {
			return fmt.Errorf("%w: tiled audit nonfinite vertex", decaderr.ErrUnsupported)
		}
	}
	boxes := make([][2]r3.Vec, len(tris))
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		for _, v := range tri {
			if v < 0 || v >= len(verts) {
				return fmt.Errorf("%w: tiled audit invalid vertex index", decaderr.ErrUnsupported)
			}
		}
		if TriangleCollapsed(verts, tri) {
			return fmt.Errorf("%w: loft triangle %d has collapsed to zero area", decaderr.ErrDegenerate, i)
		}
		lo, hi := verts[tri[0]], verts[tri[0]]
		for _, v := range tri[1:] {
			p := verts[v]
			lo = r3.Vec{X: min(lo.X, p.X), Y: min(lo.Y, p.Y), Z: min(lo.Z, p.Z)}
			hi = r3.Vec{X: max(hi.X, p.X), Y: max(hi.Y, p.Y), Z: max(hi.Z, p.Z)}
		}
		boxes[i] = [2]r3.Vec{lo, hi}
	}
	blocks := make([]treeScan, 0, (len(tris)+blockSize-1)/blockSize)
	for start := 0; start < len(tris); start += blockSize {
		if err := budget.Err(); err != nil {
			return err
		}
		end := min(start+blockSize, len(tris))
		members := make([]int, end-start)
		for i := range members {
			members[i] = start + i
		}
		blocks = append(blocks, newTreeScan(boxes, members))
	}
	var scans, pairs, tableFacets uint64
	var live [][2]int
	var first *LoftContactError
	for a := range blocks {
		for b := a; b < len(blocks); b++ {
			if err := budget.Step(); err != nil {
				return err
			}
			scans++
			if scans > scanLimit {
				return proofbound.WithWorkLimit(errLoftAuditCeiling(), "box_scans", scans, scanLimit)
			}
			if !meshbool.BoxesOverlap(blocks[a].nodes[0].box, blocks[b].nodes[0].box) {
				continue
			}
			before := pairs
			count := func(_, _ int) { pairs++ }
			var n uint64
			var exceeded bool
			var err error
			if a == b {
				n, exceeded, err = blocks[a].visit(budget, scanLimit-scans, count)
			} else {
				n, exceeded, err = tiledCrossVisit(budget, &blocks[a], &blocks[b], scanLimit-scans, count)
			}
			scans += n
			if err != nil {
				return err
			}
			if exceeded {
				return proofbound.WithWorkLimit(errLoftAuditCeiling(), "box_scans", scans, scanLimit)
			}
			if pairs > pairLimit {
				return proofbound.WithWorkLimit(errLoftAuditCeiling(), "facet_pairs", pairs, pairLimit)
			}
			if pairs != before {
				tableFacets += uint64(len(blocks[a].order))
				if a != b {
					tableFacets += uint64(len(blocks[b].order))
				}
				if tableFacets > tableLimit {
					return proofbound.WithWorkLimit(errLoftAuditCeiling(), "table_facets", tableFacets, tableLimit)
				}
				live = append(live, [2]int{a, b})
			}
		}
	}
	for _, pair := range live {
		a, b := pair[0], pair[1]
		if err := budget.Err(); err != nil {
			return err
		}
		if !meshbool.BoxesOverlap(blocks[a].nodes[0].box, blocks[b].nodes[0].box) {
			continue
		}
		startA, endA := a*blockSize, min((a+1)*blockSize, len(tris))
		startB, endB := b*blockSize, min((b+1)*blockSize, len(tris))
		ids := make(map[int]int)
		var localVerts []r3.Vec
		var localTris [][3]int
		var global []int
		add := func(start, end int) {
			for i := start; i < end; i++ {
				var t [3]int
				for k, v := range tris[i] {
					x, ok := ids[v]
					if !ok {
						x = len(localVerts)
						ids[v] = x
						localVerts = append(localVerts, verts[v])
					}
					t[k] = x
				}
				localTris = append(localTris, t)
				global = append(global, i)
			}
		}
		add(startA, endA)
		if a != b {
			add(startB, endB)
		}
		data := newCompactLoftAuditData(localVerts, localTris)
		var contact error
		classify := func(i, j int) {
			if contact != nil || (first != nil && (i > first.I || (i == first.I && j >= first.J))) {
				return
			}
			if err := budget.Step(); err != nil {
				contact = err
				return
			}
			li := i - startA
			lj := j - startA
			if a != b {
				lj = endA - startA + j - startB
			}
			_, err := AuditLoftPairData(data, localTris, li, lj, LoftAuditShortcuts{})
			if e, ok := err.(*LoftContactError); ok {
				first = &LoftContactError{I: global[e.I], J: global[e.J], Reason: e.Reason}
			} else if err != nil {
				contact = err
			}
		}
		if a == b {
			_, _, err := blocks[a].visit(budget, scanLimit, classify)
			if err != nil {
				return err
			}
		} else {
			_, _, err := tiledCrossVisit(budget, &blocks[a], &blocks[b], scanLimit, classify)
			if err != nil {
				return err
			}
		}
		if contact != nil {
			return contact
		}
	}
	if first != nil {
		return first
	}
	return budget.Err()
}

func tiledCrossVisit(budget *proofbound.WorkBudget, a, b *treeScan, limit uint64, fn func(int, int)) (uint64, bool, error) {
	var scanned uint64
	var walk func(int, int) (bool, error)
	step := func() (bool, error) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		scanned++
		return scanned > limit, nil
	}
	walk = func(x, y int) (bool, error) {
		if exceeded, err := step(); exceeded || err != nil {
			return exceeded, err
		}
		nx, ny := a.nodes[x], b.nodes[y]
		if !meshbool.BoxesOverlap(nx.box, ny.box) {
			return false, nil
		}
		if nx.left < 0 && ny.left < 0 {
			for _, i := range a.order[nx.start:nx.end] {
				for _, j := range b.order[ny.start:ny.end] {
					if exceeded, err := step(); exceeded || err != nil {
						return exceeded, err
					}
					if meshbool.BoxesOverlap(a.boxes[i], b.boxes[j]) {
						fn(int(i), int(j))
					}
				}
			}
			return false, nil
		}
		if ny.left < 0 || (nx.left >= 0 && nx.end-nx.start >= ny.end-ny.start) {
			if exceeded, err := walk(nx.left, y); exceeded || err != nil {
				return exceeded, err
			}
			return walk(nx.right, y)
		}
		if exceeded, err := walk(x, ny.left); exceeded || err != nil {
			return exceeded, err
		}
		return walk(x, ny.right)
	}
	exceeded, err := walk(0, 0)
	return scanned, exceeded, err
}
