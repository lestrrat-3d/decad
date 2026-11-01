package meshbool

import (
	"context"
	"math"
	"math/big"
	"sort"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// brakeConformOnce inserts, into every facet edge, the mesh vertices that lie
// exactly in that edge's interior, re-triangulating the facet so the
// subdivision conforms. Returns whether anything split.
//
// bounds runs parallel to tris and each sub-triangle inherits its parent's
// entry. xbeta is per exact vertex: a vertex inserted into a facet's edge
// lies on that held facet, so it is raised to the facet's bound
// (agent-docs/faceted-vertex-bounds-design.md §3.3).
func brakeconformOnce(ctx context.Context, verts []proof.Xpt, tris *[][3]int, src *[]int, bounds *[]float64,
	xbeta []float64, budget *proofbound.WorkBudget, maxFacets int) (bool, error) {
	// One counter spans the facet walk, the grid-cell candidate scan nested
	// under it, and the along-edge ordering: the candidate vertices a single
	// facet edge sweeps are the candidate operations §7.2 counts, not the
	// facets.
	if err := budget.Err(); err != nil {
		return false, err
	}
	scan, err := brakeNewConformScan(ctx, budget, verts)
	if err != nil {
		return false, err
	}

	var outTris [][3]int
	var outSrc []int
	var outBounds []float64
	splitAny := false
	for ti, tri := range *tris {
		if err := budget.Step(); err != nil {
			return false, err
		}
		inserted := [3][]int{}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			hits, err := scan.EdgeInteriorHits(ctx, budget, a, b, tri)
			if err != nil {
				return false, err
			}
			if len(hits) > 0 {
				if err := brakeSortAlongEdge(ctx, budget, verts, a, b, hits); err != nil {
					return false, err
				}
				inserted[k] = hits
			}
		}
		if inserted[0] == nil && inserted[1] == nil && inserted[2] == nil {
			if splitAny {
				if maxFacets > 0 && len(outTris) >= maxFacets {
					return false, errReplayBoundedStitchFacetCapExceeded
				}
				if err := budget.Reserve(3); err != nil {
					return false, err
				}
				outTris, err = appendWork(outTris, [][3]int{tri}, budget)
				if err != nil {
					return false, err
				}
				outSrc, err = appendWork(outSrc, []int{(*src)[ti]}, budget)
				if err != nil {
					return false, err
				}
				outBounds, err = appendWork(outBounds, []float64{(*bounds)[ti]}, budget)
				if err != nil {
					return false, err
				}
			}
			continue
		}
		if !splitAny {
			if err := budget.Reserve(3 * uint64(ti)); err != nil {
				return false, err
			}
			outTris, err = appendWork(outTris, (*tris)[:ti], budget)
			if err != nil {
				return false, err
			}
			outSrc, err = appendWork(outSrc, (*src)[:ti], budget)
			if err != nil {
				return false, err
			}
			outBounds, err = appendWork(outBounds, (*bounds)[:ti], budget)
			if err != nil {
				return false, err
			}
		}
		count := 3 + len(inserted[0]) + len(inserted[1]) + len(inserted[2])
		if budget.Replay {
			if _, err := budget.StepN(2 * uint64(count)); err != nil {
				return false, err
			}
		}
		splitAny = true
		for _, hits := range inserted {
			for _, h := range hits {
				xbeta[h] = max(xbeta[h], (*bounds)[ti])
			}
		}
		if maxFacets > 0 && count-2 > maxFacets-len(outTris) {
			return false, errReplayBoundedStitchFacetCapExceeded
		}
		if err := budget.Reserve(uint64(count)); err != nil {
			return false, err
		}
		if err := budget.ReserveBytesMul(uint64(count), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return false, err
		}
		poly := make([]int, 0, count)
		poly = append(poly, tri[0])
		poly = append(poly, inserted[0]...)
		poly = append(poly, tri[1])
		poly = append(poly, inserted[1]...)
		poly = append(poly, tri[2])
		poly = append(poly, inserted[2]...)
		newTris, err := braketriangulatePlanarPolygon(ctx, verts, poly, budget)
		if err != nil {
			return false, err
		}
		if maxFacets > 0 && (len(outTris) > maxFacets || len(newTris) > maxFacets-len(outTris)) {
			return false, errReplayBoundedStitchFacetCapExceeded
		}
		if err := budget.Reserve(3 * uint64(len(newTris))); err != nil {
			return false, err
		}
		for _, nt := range newTris {
			outTris, err = appendWork(outTris, [][3]int{nt}, budget)
			if err != nil {
				return false, err
			}
			outSrc, err = appendWork(outSrc, []int{(*src)[ti]}, budget)
			if err != nil {
				return false, err
			}
			outBounds, err = appendWork(outBounds, []float64{(*bounds)[ti]}, budget)
			if err != nil {
				return false, err
			}
		}
	}
	if splitAny {
		*tris = outTris
		*src = outSrc
		*bounds = outBounds
	}
	return splitAny, nil
}

// brakeOnSegmentInterior3 reports, exactly, whether p lies strictly inside the 3D
// segment (a, b).
//
// The parameter t = ap[axis]/d[axis] is compared only against 0 and 1, which
// is decided with no division and no big.Rat: ap[axis] and d[axis] are raw
// homogeneous numerators over their own (possibly different) positive
// denominators ap.w and d.w, so "t < 1" cross-multiplies them
// (ap[axis]·d.w vs d[axis]·ap.w — never a sign flip, since both denominators
// are positive) and "t > 0" reads ap[axis]'s sign directly (ap.w is
// positive). Only the FINAL combination branches on d[axis]'s sign, the same
// rule stage A's version of this function established.
func brakeOnSegmentInterior3(a, b, p proof.Xpt) bool {
	d := proof.Xsub(b, a)
	ap := proof.Xsub(p, a)
	cr := proof.Xcross(d, ap)
	if cr.X.Sign() != 0 || cr.Y.Sign() != 0 || cr.Z.Sign() != 0 {
		return false
	}
	axis := brakeDominantAxis(d)
	dAxis := XIntCoordOf(d, axis)
	dSign := dAxis.Sign()
	if dSign == 0 {
		return false
	}
	apAxis := XIntCoordOf(ap, axis)
	cmp := new(big.Int).Mul(apAxis, d.W).Cmp(new(big.Int).Mul(dAxis, ap.W))
	if dSign > 0 {
		return apAxis.Sign() > 0 && cmp < 0
	}
	return apAxis.Sign() < 0 && cmp > 0
}

// brakeDominantAxis picks the coordinate of d with the largest magnitude. d's
// three coordinates share one positive denominator, so their magnitudes
// compare directly as integers — no cross-multiplication needed.
func brakeDominantAxis(d proof.Xpt) int {
	ax := new(big.Int).Abs(d.X)
	ay := new(big.Int).Abs(d.Y)
	az := new(big.Int).Abs(d.Z)
	if ax.Cmp(ay) >= 0 && ax.Cmp(az) >= 0 {
		return 0
	}
	if ay.Cmp(az) >= 0 {
		return 1
	}
	return 2
}

// brakeConformScan indexes the stitched vertices by their float approximations and
// retains the exact vertices for the on-edge predicate. Its reject-only filter
// threshold is derived from the approximation error, so pruning cannot miss an
// exact incidence.
type brakeConformScan struct {
	Verts      []proof.Xpt
	Approx     []r3.Vec
	points     heldBoxTree
	pointCells [][3]int
	treeStack  []int
	Lo         r3.Vec
	Cell       float64
	Slack      float64
	Tau2       float64
}

func brakeNewConformScan(ctx context.Context, budget *proofbound.WorkBudget, verts []proof.Xpt) (*brakeConformScan, error) {
	if err := budget.Reserve(10 * uint64(len(verts))); err != nil {
		return nil, err
	}
	if budget.Replay {
		if _, err := budget.StepN(5 * uint64(len(verts))); err != nil {
			return nil, err
		}
	}
	lo, hi := r3.Vec{}, r3.Vec{}
	if err := budget.ReserveBytesMul(uint64(len(verts)), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
		return nil, err
	}
	approx := make([]r3.Vec, len(verts))
	for i, p := range verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		var v r3.Vec
		if budget.Replay {
			var err error
			v, err = proof.ReplayXptVec(ctx, p, replayExactWork{work: budget})
			if err != nil {
				return nil, err
			}
		} else {
			v = p.Vec()
		}
		approx[i] = v
		if i == 0 {
			lo, hi = v, v
			continue
		}
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	diag := hi.Sub(lo).Len()
	if diag == 0 {
		return nil, errReplayTheStitchedBoundaryHasNoExtent
	}
	cell := diag / 64
	// maxAbs bounds every coordinate of every vertex, so the one threshold it
	// yields serves every segment and every candidate (SegAdmissionRadius2).
	maxAbs := math.Max(
		math.Max(math.Abs(lo.X), math.Abs(hi.X)),
		math.Max(math.Max(math.Abs(lo.Y), math.Abs(hi.Y)), math.Max(math.Abs(lo.Z), math.Abs(hi.Z))),
	)
	if err := budget.Reserve(1); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(1, uint64(unsafe.Sizeof(brakeConformScan{}))); err != nil {
		return nil, err
	}
	s := &brakeConformScan{
		Verts:  verts,
		Approx: approx,
		Lo:     lo,
		Cell:   cell,
		Slack:  diag*1e-9 + cell*1e-9,
	}
	s.Tau2 = SegAdmissionRadius2(s.Slack, maxAbs)
	if err := budget.ReserveBytesMul(uint64(len(verts)), uint64(unsafe.Sizeof([2]r3.Vec{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(uint64(len(verts)), uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return nil, err
	}
	boxes := make([][2]r3.Vec, len(verts))
	s.pointCells = make([][3]int, len(verts))
	for i := range verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		c := s.CellOf(approx[i].X, approx[i].Y, approx[i].Z)
		s.pointCells[i] = c
		boxes[i] = [2]r3.Vec{approx[i], approx[i]}
	}
	if budget.ReserveFn == nil {
		s.points = newBoxTree(boxes)
	} else {
		var err error
		s.points, err = newBoxTreeWork(boxes, budget)
		if err != nil {
			return nil, err
		}
	}
	if budget.ReserveFn != nil {
		if err := budget.ReserveBytesMul(uint64(len(s.points.nodes)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
		s.treeStack = make([]int, 0, len(s.points.nodes))
	}
	return s, nil
}

func (s *brakeConformScan) CellOf(x, y, z float64) [3]int {
	return [3]int{
		int(math.Floor((x - s.Lo.X) / s.Cell)),
		int(math.Floor((y - s.Lo.Y) / s.Cell)),
		int(math.Floor((z - s.Lo.Z) / s.Cell)),
	}
}

// EdgeInteriorHits returns the mesh vertices lying exactly in the interior of
// facet edge (a, b), searched through the point boxes in a conservative
// expanded segment box.
// Both tree traversal and candidate checks step the budget.
//
// Every candidate meets the reject-only SegFilter before the exact predicate,
// and the exact predicate still decides every candidate the filter does not
// reject. The traversal itself is untouched by that filter: it still visits the
// whole slack box, which is a SUPERSET of the cells the edge can touch. Walking
// only the cells the segment really passes through would cut the candidate set
// at its source, but it would also owe a completeness proof a superset does not
// — and it measured no faster here, because the filter has already reduced what
// a visited cell costs to a handful of float operations.
func (s *brakeConformScan) EdgeInteriorHits(ctx context.Context, budget *proofbound.WorkBudget, a, b int, tri [3]int) ([]int, error) {
	pa, pb := s.Approx[a], s.Approx[b]
	filter := NewSegFilter(pa, pb, s.Tau2)
	var hits []int
	// An exact on-edge point's float approximation is within sqrt(Tau2) of
	// this float segment, by SegAdmissionRadius2. Querying that expanded segment
	// box is therefore a complete superset, even when subtracting Lo to find a
	// grid cell loses precision at large coordinate offsets.
	tau := math.Nextafter(math.Sqrt(s.Tau2), math.Inf(1))
	box := [2]r3.Vec{
		{X: math.Nextafter(math.Min(pa.X, pb.X)-tau, math.Inf(-1)), Y: math.Nextafter(math.Min(pa.Y, pb.Y)-tau, math.Inf(-1)), Z: math.Nextafter(math.Min(pa.Z, pb.Z)-tau, math.Inf(-1))},
		{X: math.Nextafter(math.Max(pa.X, pb.X)+tau, math.Inf(1)), Y: math.Nextafter(math.Max(pa.Y, pb.Y)+tau, math.Inf(1)), Z: math.Nextafter(math.Max(pa.Z, pb.Z)+tau, math.Inf(1))},
	}
	visit := func(i int) error {
		if err := budget.Step(); err != nil {
			return err
		}
		if i == tri[0] || i == tri[1] || i == tri[2] || filter.TooFar(s.Approx[i]) {
			return nil
		}
		inside := false
		if budget.Replay {
			var err error
			inside, err = replaySegmentInterior3(ctx, budget, s.Verts[a], s.Verts[b], s.Verts[i])
			if err != nil {
				return err
			}
		} else {
			inside = brakeOnSegmentInterior3(s.Verts[a], s.Verts[b], s.Verts[i])
		}
		if inside {
			if err := budget.Reserve(1); err != nil {
				return err
			}
			var err error
			hits, err = appendWork(hits, []int{i}, budget)
			if err != nil {
				return err
			}
		}
		return nil
	}
	var err error
	if budget.ReserveFn == nil {
		err = s.points.visitOverlaps(box, budget.Step, visit)
	} else {
		err = s.points.visitOverlapsWork(box, budget.Step, s.treeStack, visit)
	}
	if err != nil {
		return nil, err
	}
	if len(hits) < 2 {
		return hits, nil
	}
	// Every tree candidate steps the budget before rejection. Order the hits
	// by grid cell and vertex index, independently of tree traversal order.
	less := func(a, b int) bool {
		for axis := range 3 {
			if s.pointCells[a][axis] != s.pointCells[b][axis] {
				return s.pointCells[a][axis] < s.pointCells[b][axis]
			}
		}
		return a < b
	}
	if budget.ReserveFn == nil {
		sort.Slice(hits, func(i, j int) bool { return less(hits[i], hits[j]) })
	} else {
		for i := 1; i < len(hits); i++ {
			for j := i; j > 0; j-- {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				if !less(hits[j], hits[j-1]) {
					break
				}
				hits[j], hits[j-1] = hits[j-1], hits[j]
			}
		}
	}
	return hits, nil
}

// brakeSortAlongEdge orders the inserted vertices by their exact parameter along
// (a, b). Each parameter is computed once and carried through the sort: the
// comparison is a leaf exact predicate, so recomputing an exact division inside
// it would make the pass quadratic in rational arithmetic rather than in
// comparisons, with the same ordering.
// brakeSortAlongEdge-scoped param is one hit's own parameter numerator over its
// own positive denominator (ap[axis]/ap.w) — never divided by the shared
// dominant-axis component d[axis]/d.w, which is constant across every hit and
// so only decides whether the comparison below runs forward or reversed.
type brakeEdgeParam struct{ Num, Den *big.Int }

// brakeSortAlongEdge orders the inserted vertices by their exact parameter along
// (a, b). Each comparison cross-multiplies two hits' own (numerator,
// positive-denominator) pairs rather than dividing — recomputing a division
// inside the comparator would make the pass quadratic in rational arithmetic
// rather than in comparisons; cross-multiplying keeps every intermediate an
// integer product with no normalisation at all.
func brakeSortAlongEdge(ctx context.Context, budget *proofbound.WorkBudget, verts []proof.Xpt, a, b int, hits []int) error {
	if budget.Replay {
		return brakesortAlongEdgeReplay(ctx, budget, verts, a, b, hits)
	}
	d := proof.Xsub(verts[b], verts[a])
	axis := brakeDominantAxis(d)
	daSign := XIntCoordOf(d, axis).Sign()
	if err := budget.Reserve(uint64(len(hits))); err != nil {
		return err
	}
	if err := budget.ReserveBytesMul(uint64(len(hits)), uint64(unsafe.Sizeof(brakeEdgeParam{}))); err != nil {
		return err
	}
	params := make([]brakeEdgeParam, len(hits))
	for i, vi := range hits {
		if err := budget.Step(); err != nil {
			return err
		}
		ap := proof.Xsub(verts[vi], verts[a])
		params[i] = brakeEdgeParam{Num: XIntCoordOf(ap, axis), Den: ap.W}
	}
	less := func(i, j int) bool {
		// Both denominators are positive, so this cross-multiplication never
		// needs a sign flip on its own; the shared divisor d[axis]/d.w this
		// parameter is implicitly measured against is what can be negative,
		// and it is constant across every hit, so it flips the WHOLE
		// ordering once rather than each comparison individually.
		lhs := new(big.Int).Mul(params[i].Num, params[j].Den)
		rhs := new(big.Int).Mul(params[j].Num, params[i].Den)
		if daSign < 0 {
			return lhs.Cmp(rhs) > 0
		}
		return lhs.Cmp(rhs) < 0
	}
	for i := 1; i < len(hits); i++ {
		for j := i; j > 0; j-- {
			if budget.ReserveFn != nil {
				if err := budget.Step(); err != nil {
					return err
				}
			}
			if !less(j, j-1) {
				break
			}
			if budget.ReserveFn == nil {
				if err := budget.Step(); err != nil {
					return err
				}
			}
			hits[j], hits[j-1] = hits[j-1], hits[j]
			params[j], params[j-1] = params[j-1], params[j]
		}
	}
	return nil
}

// brakesortAlongEdgeReplay compares exact coordinate products without math/big scratch.
func brakesortAlongEdgeReplay(ctx context.Context, budget *proofbound.WorkBudget, verts []proof.Xpt, a, b int, hits []int) error {
	if a < 0 || b < 0 || a >= len(verts) || b >= len(verts) {
		return errReplayInvalidConformEdge
	}
	exact := replayExactWork{work: budget}
	axis := 0
	var best *big.Int
	for k := range 3 {
		d, err := proof.ReplayProductDifference(ctx, XIntCoordOf(verts[b], k), verts[a].W, XIntCoordOf(verts[a], k), verts[b].W, exact)
		if err != nil {
			return err
		}
		if best == nil {
			best = d
			continue
		}
		if _, err := budget.StepN(uint64(max(len(best.Bits()), len(d.Bits()))) + 1); err != nil {
			return err
		}
		if d.CmpAbs(best) > 0 {
			axis = k
			best = d
		}
	}
	direction := best.Sign()
	for i := 1; i < len(hits); i++ {
		for j := i; j > 0; j-- {
			left, right := hits[j], hits[j-1]
			if left < 0 || right < 0 || left >= len(verts) || right >= len(verts) {
				return errReplayInvalidConformVertex
			}
			p, q := verts[left], verts[right]
			d, err := proof.ReplayProductDifference(ctx, XIntCoordOf(p, axis), q.W, XIntCoordOf(q, axis), p.W, exact)
			if err != nil {
				return err
			}
			less := d.Sign() < 0
			if direction < 0 {
				less = d.Sign() > 0
			}
			if !less {
				break
			}
			hits[j], hits[j-1] = hits[j-1], hits[j]
		}
	}
	return nil
}

func braketriangulatePlanarPolygon(ctx context.Context, verts []proof.Xpt, poly []int,
	budget *proofbound.WorkBudget) ([][3]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(poly) < 3 {
		return nil, errReplayAConformingPolygonLostItsCorners
	}
	// The polygon is a triangle with edge insertions: its normal is the
	// original facet's, recoverable from any strict corner.
	var n proof.Xpt
	if !budget.Replay {
		n = proof.Xpt{X: new(big.Int), Y: new(big.Int), Z: new(big.Int), W: big.NewInt(1)}
	}
	found := false
	for i := 1; i+1 < len(poly); i++ {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		var cand proof.Xpt
		if budget.Replay {
			v, err := replayCrossFromPoints(ctx, budget, verts[poly[0]], verts[poly[i]], verts[poly[i+1]])
			if err != nil {
				return nil, err
			}
			cand = proof.Xpt{X: v[0], Y: v[1], Z: v[2], W: verts[poly[0]].W}
		} else {
			cand = proof.Xcross(proof.Xsub(verts[poly[i]], verts[poly[0]]), proof.Xsub(verts[poly[i+1]], verts[poly[0]]))
		}
		if cand.X.Sign() != 0 || cand.Y.Sign() != 0 || cand.Z.Sign() != 0 {
			n = cand
			found = true
			break
		}
	}
	if !found {
		return nil, errReplayAConformingPolygonIsDegenerate
	}
	u, v := 0, 0
	if budget.Replay {
		if _, err := budget.StepN(uint64(max(len(n.X.Bits()), max(len(n.Y.Bits()), len(n.Z.Bits())))) + 1); err != nil {
			return nil, err
		}
		if n.Z.CmpAbs(n.X) >= 0 && n.Z.CmpAbs(n.Y) >= 0 {
			u, v = 0, 1
		} else if n.Y.CmpAbs(n.X) >= 0 {
			u, v = 2, 0
		} else {
			u, v = 1, 2
		}
	} else {
		u, v = ProjAxes(n)
	}
	if err := budget.Reserve(2 * uint64(len(poly))); err != nil {
		return nil, err
	}
	if budget.Replay {
		if _, err := budget.StepN(2 * uint64(len(poly))); err != nil {
			return nil, err
		}
	}
	if err := budget.ReserveBytesMul(uint64(len(poly)), uint64(unsafe.Sizeof(Xp2{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(uint64(len(poly)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	pts := make([]Xp2, len(poly))
	idx := make([]int, len(poly))
	for i, vi := range poly {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if budget.Replay {
			var err error
			pts[i], err = replayProjection(ctx, verts[vi], u, v, replayExactWork{work: budget})
			if err != nil {
				return nil, err
			}
		} else {
			pts[i] = NewXP2(RatCoordOf(verts[vi], u), RatCoordOf(verts[vi], v))
		}
		idx[i] = i
	}
	// Keep the projected orientation counter-clockwise so ear clipping and
	// the emitted winding agree with the facet's own.
	var area int
	var err error
	if budget.Replay {
		area, err = replayPolyArea2Sign(ctx, budget, pts)
	} else {
		area, err = PolyArea2Sign(budget, pts)
	}
	if err != nil {
		return nil, err
	}
	flip := area < 0
	if flip {
		for i, j := 0, len(idx)-1; i < j; i, j = i+1, j-1 {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			idx[i], idx[j] = idx[j], idx[i]
		}
	}
	var tris2 [][3]int
	if budget.Replay {
		tris2, err = replayEarClip(ctx, budget, pts, idx)
	} else {
		tris2, err = EarClipX(budget, pts, idx)
	}
	if err != nil {
		return nil, err
	}
	if err := budget.Reserve(uint64(len(tris2))); err != nil {
		return nil, err
	}
	if budget.Replay {
		if _, err := budget.StepN(uint64(len(tris2))); err != nil {
			return nil, err
		}
	}
	if err := budget.ReserveBytesMul(uint64(len(tris2)), uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return nil, err
	}
	out := make([][3]int, 0, len(tris2))
	for _, t := range tris2 {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		a, b, c := poly[t[0]], poly[t[1]], poly[t[2]]
		if flip {
			b, c = c, b
		}
		out = append(out, [3]int{a, b, c})
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return out, nil
}
