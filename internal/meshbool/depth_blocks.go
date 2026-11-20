package meshbool

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// depthFanout is how many consecutive entries of one level a depth block of
// the next level covers: level 0 covers facets, and each later level covers
// blocks of the level before it.
const depthFanout = 4

// everywhereBox contains every point, so PointBoxDist reads 0 against it.
var everywhereBox = [2]r3.Vec{
	{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)},
	{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)},
}

// setDepthBlocks summarizes Boxes for depthBeyond's scan. Level 0 holds the
// union box of every run of depthFanout consecutive facet boxes, and each
// later level the union of every run of depthFanout consecutive blocks of the
// level before, up to a single block. A run holding a box that is not
// ordered (a NaN coordinate, or a low corner past the high one on an axis)
// gets everywhereBox. Every block contains the boxes it covers, so its
// PointBoxDist is no larger than theirs (agent-docs/evaluator-design.md §9).
// It then fills depthKappa (setDepthKappa) for the depth threshold prune.
// PrepContext calls it once Boxes and the depth normals are filled, and a
// caller that changes Verts, Tris, Boxes or Norms afterwards must call it
// again.
func (bm *BoolMesh) setDepthBlocks() {
	bm.depthLevels = nil
	level := bm.Boxes
	for len(level) > 1 {
		level = unionRuns(level, depthFanout)
		bm.depthLevels = append(bm.depthLevels, level)
	}
	bm.setDepthKappa()
}

// unionRuns returns the union box of every run of size consecutive boxes, or
// everywhereBox for a run that holds a box boxOrdered rejects.
func unionRuns(boxes [][2]r3.Vec, size int) [][2]r3.Vec {
	out := make([][2]r3.Vec, 0, (len(boxes)+size-1)/size)
	for start := 0; start < len(boxes); start += size {
		u := boxes[start]
		for _, box := range boxes[start:min(start+size, len(boxes))] {
			if !boxOrdered(box) {
				u = everywhereBox
				break
			}
			u = [2]r3.Vec{
				{X: math.Min(u[0].X, box[0].X), Y: math.Min(u[0].Y, box[0].Y), Z: math.Min(u[0].Z, box[0].Z)},
				{X: math.Max(u[1].X, box[1].X), Y: math.Max(u[1].Y, box[1].Y), Z: math.Max(u[1].Z, box[1].Z)},
			}
		}
		out = append(out, u)
	}
	return out
}

// boxOrdered reports whether box's low corner is at or below its high corner
// on every axis, which is false for a NaN coordinate.
func boxOrdered(box [2]r3.Vec) bool {
	return box[0].X <= box[1].X && box[0].Y <= box[1].Y && box[0].Z <= box[1].Z
}

// depthBlocksFit reports whether depthLevels has the shape setDepthBlocks
// gives a mesh of n facets.
func (bm *BoolMesh) depthBlocksFit(n int) bool {
	for _, level := range bm.depthLevels {
		n = (n + depthFanout - 1) / depthFanout
		if len(level) != n {
			return false
		}
	}
	return n <= 1
}

// depthScan is the running state of depthBeyond's facet scan: the rounded
// query point, the mesh, the bound and the running minimum. The remaining
// fields serve the depth threshold prune (depth_prune.go): whether it runs,
// the threshold t with its slack factors once computed, and whether the scan
// has passed over a box.
type depthScan struct {
	pf    r3.Vec
	other *BoolMesh
	b     float64
	best  float64

	prune  bool
	tdSet  bool
	td     float64
	a1, a2 float64
	passed bool
}

// scan runs CertifiedInteriorDepth's facet scan, in facet order with its box
// prune, and stops on the first facet whose distance lowers the running
// minimum to a value that settles p as no deeper than b. It returns that
// facet, or -1 when the scan ends unsettled with the definition's minimum in
// best. It descends the depth blocks when they fit the mesh, and scans the
// facets directly otherwise. With prune set and depthKappa fitting the
// blocks, it runs prunedBlock instead; a scan that ends with passed set has
// proven p deeper than b and leaves best above the threshold, and a scan the
// prune gives up on starts over without it.
func (s *depthScan) scan() int {
	n := len(s.other.Tris)
	top := len(s.other.depthLevels) - 1
	if top < 0 || !s.other.depthBlocksFit(n) {
		return s.facets(0, n)
	}
	if s.prune && s.other.depthKappaFit() {
		if i := s.prunedBlock(top, 0); i != depthScanRestart {
			return i
		}
		s.best, s.passed = math.Inf(1), false
	}
	return s.block(top, 0)
}

// block scans the facets block j of level covers, in facet order. A block
// whose PointBoxDist is already at or past the running minimum is passed
// over whole: every facet box it covers reads at least that distance, so the
// prune skips each of those facets and the minimum does not move.
func (s *depthScan) block(level, j int) int {
	if PointBoxDist(s.pf, s.other.depthLevels[level][j]) >= s.best {
		return -1
	}
	lo := j * depthFanout
	if level == 0 {
		return s.facets(lo, min(lo+depthFanout, len(s.other.Tris)))
	}
	for k := lo; k < min(lo+depthFanout, len(s.other.depthLevels[level-1])); k++ {
		if i := s.block(level-1, k); i >= 0 {
			return i
		}
	}
	return -1
}

// facets scans facets lo to hi-1 as CertifiedInteriorDepth's loop does.
func (s *depthScan) facets(lo, hi int) int {
	for i := lo; i < hi; i++ {
		if PointBoxDist(s.pf, s.other.Boxes[i]) >= s.best {
			continue
		}
		s.best = math.Min(s.best, depthFacetDistance(s.pf, TriCorners(s.other, i), s.other.depthNormal(i)))
		if s.best*(1-1e-12) <= s.b {
			return i
		}
	}
	return -1
}
