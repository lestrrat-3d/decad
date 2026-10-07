package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ConformOnce inserts, into every facet edge, the mesh vertices that lie
// exactly in that edge's interior, re-triangulating the facet so the
// subdivision conforms. Returns whether anything split.
//
// bounds runs parallel to tris and each sub-triangle inherits its parent's
// entry. xbeta is per exact vertex: a vertex inserted into a facet's edge
// lies on that held facet, so it is raised to the facet's bound
// (docs/faceted-vertex-bounds-design.md §3.3).
func ConformOnce(ctx context.Context, verts []proof.Xpt, tris *[][3]int, src *[]int, bounds *[]float64, xbeta []float64) (bool, error) {
	// One counter spans the facet walk, the grid-cell candidate scan nested
	// under it, and the along-edge ordering: the candidate vertices a single
	// facet edge sweeps are the candidate operations §7.2 counts, not the
	// facets.
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return false, err
	}
	scan, err := NewConformScan(budget, verts)
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
			hits, err := scan.EdgeInteriorHits(budget, a, b, tri)
			if err != nil {
				return false, err
			}
			if len(hits) > 0 {
				if err := SortAlongEdge(budget, verts, a, b, hits); err != nil {
					return false, err
				}
				inserted[k] = hits
			}
		}
		if inserted[0] == nil && inserted[1] == nil && inserted[2] == nil {
			outTris = append(outTris, tri)
			outSrc = append(outSrc, (*src)[ti])
			outBounds = append(outBounds, (*bounds)[ti])
			continue
		}
		splitAny = true
		for _, hits := range inserted {
			for _, h := range hits {
				xbeta[h] = max(xbeta[h], (*bounds)[ti])
			}
		}
		poly := []int{tri[0]}
		poly = append(poly, inserted[0]...)
		poly = append(poly, tri[1])
		poly = append(poly, inserted[1]...)
		poly = append(poly, tri[2])
		poly = append(poly, inserted[2]...)
		newTris, err := TriangulatePlanarPolygon(ctx, verts, poly)
		if err != nil {
			return false, err
		}
		for _, nt := range newTris {
			outTris = append(outTris, nt)
			outSrc = append(outSrc, (*src)[ti])
			outBounds = append(outBounds, (*bounds)[ti])
		}
	}
	*tris = outTris
	*src = outSrc
	*bounds = outBounds
	return splitAny, nil
}

// OnSegmentInterior3 reports, exactly, whether p lies strictly inside the 3D
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
func OnSegmentInterior3(a, b, p proof.Xpt) bool {
	d := proof.Xsub(b, a)
	ap := proof.Xsub(p, a)
	cr := proof.Xcross(d, ap)
	if cr.X.Sign() != 0 || cr.Y.Sign() != 0 || cr.Z.Sign() != 0 {
		return false
	}
	axis := DominantAxis(d)
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

// DominantAxis picks the coordinate of d with the largest magnitude. d's
// three coordinates share one positive denominator, so their magnitudes
// compare directly as integers — no cross-multiplication needed.
func DominantAxis(d proof.Xpt) int {
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

// ConformScan is the searchable form of the stitched vertex set each facet edge
// is swept through: the exact vertices, their float approximations, a coarse
// uniform grid over those approximations, and the reject-only filter threshold
// every candidate is measured against. The grid prunes the exact on-edge tests;
// the slack absorbs the approximation, so no incidence is missed.
type ConformScan struct {
	Verts  []proof.Xpt
	Approx []r3.Vec
	Grid   map[[3]int][]int
	Lo     r3.Vec
	Cell   float64
	Slack  float64
	Tau2   float64
}

func NewConformScan(budget *proofbound.WorkBudget, verts []proof.Xpt) (*ConformScan, error) {
	lo, hi := r3.Vec{}, r3.Vec{}
	approx := make([]r3.Vec, len(verts))
	for i, p := range verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		v := p.Vec()
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
		return nil, fmt.Errorf(`%w: the stitched boundary has no extent`, decaderr.ErrBooleanFailed)
	}
	cell := diag / 64
	// maxAbs bounds every coordinate of every vertex, so the one threshold it
	// yields serves every segment and every candidate (SegAdmissionRadius2).
	maxAbs := math.Max(
		math.Max(math.Abs(lo.X), math.Abs(hi.X)),
		math.Max(math.Max(math.Abs(lo.Y), math.Abs(hi.Y)), math.Max(math.Abs(lo.Z), math.Abs(hi.Z))),
	)
	s := &ConformScan{
		Verts:  verts,
		Approx: approx,
		Grid:   map[[3]int][]int{},
		Lo:     lo,
		Cell:   cell,
		Slack:  diag*1e-9 + cell*1e-9,
	}
	s.Tau2 = SegAdmissionRadius2(s.Slack, maxAbs)
	for i := range verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		c := s.CellOf(approx[i].X, approx[i].Y, approx[i].Z)
		s.Grid[c] = append(s.Grid[c], i)
	}
	return s, nil
}

func (s *ConformScan) CellOf(x, y, z float64) [3]int {
	return [3]int{
		int(math.Floor((x - s.Lo.X) / s.Cell)),
		int(math.Floor((y - s.Lo.Y) / s.Cell)),
		int(math.Floor((z - s.Lo.Z) / s.Cell)),
	}
}

// edgeInteriorHits returns the mesh vertices lying exactly in the interior of
// facet edge (a, b), searched through the grid cells the edge's slack box
// covers. An edge spanning the mesh sweeps the whole grid, so the cells and the
// candidate vertices in them — not the facets — are the candidate operations
// §7.2 counts, and both step the budget.
//
// Every candidate meets the reject-only SegFilter before the exact predicate,
// and the exact predicate still decides every candidate the filter does not
// reject. The traversal itself is untouched by that filter: it still visits the
// whole slack box, which is a SUPERSET of the cells the edge can touch. Walking
// only the cells the segment really passes through would cut the candidate set
// at its source, but it would also owe a completeness proof a superset does not
// — and it measured no faster here, because the filter has already reduced what
// a visited cell costs to a handful of float operations.
func (s *ConformScan) EdgeInteriorHits(budget *proofbound.WorkBudget, a, b int, tri [3]int) ([]int, error) {
	pa, pb := s.Approx[a], s.Approx[b]
	filter := NewSegFilter(pa, pb, s.Tau2)
	var hits []int
	cLo := s.CellOf(math.Min(pa.X, pb.X)-s.Slack, math.Min(pa.Y, pb.Y)-s.Slack, math.Min(pa.Z, pb.Z)-s.Slack)
	cHi := s.CellOf(math.Max(pa.X, pb.X)+s.Slack, math.Max(pa.Y, pb.Y)+s.Slack, math.Max(pa.Z, pb.Z)+s.Slack)
	for cx := cLo[0]; cx <= cHi[0]; cx++ {
		for cy := cLo[1]; cy <= cHi[1]; cy++ {
			for cz := cLo[2]; cz <= cHi[2]; cz++ {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				for _, vi := range s.Grid[[3]int{cx, cy, cz}] {
					if err := budget.Step(); err != nil {
						return nil, err
					}
					if vi == tri[0] || vi == tri[1] || vi == tri[2] {
						continue
					}
					if filter.TooFar(s.Approx[vi]) {
						continue
					}
					if OnSegmentInterior3(s.Verts[a], s.Verts[b], s.Verts[vi]) {
						hits = append(hits, vi)
					}
				}
			}
		}
	}
	return hits, nil
}

// SortAlongEdge orders the inserted vertices by their exact parameter along
// (a, b). Each parameter is computed once and carried through the sort: the
// comparison is a leaf exact predicate, so recomputing an exact division inside
// it would make the pass quadratic in rational arithmetic rather than in
// comparisons, with the same ordering.
// SortAlongEdge-scoped param is one hit's own parameter numerator over its
// own positive denominator (ap[axis]/ap.w) — never divided by the shared
// dominant-axis component d[axis]/d.w, which is constant across every hit and
// so only decides whether the comparison below runs forward or reversed.
type EdgeParam struct{ Num, Den *big.Int }

// SortAlongEdge orders the inserted vertices by their exact parameter along
// (a, b). Each comparison cross-multiplies two hits' own (numerator,
// positive-denominator) pairs rather than dividing — recomputing a division
// inside the comparator would make the pass quadratic in rational arithmetic
// rather than in comparisons; cross-multiplying keeps every intermediate an
// integer product with no normalisation at all.
func SortAlongEdge(budget *proofbound.WorkBudget, verts []proof.Xpt, a, b int, hits []int) error {
	d := proof.Xsub(verts[b], verts[a])
	axis := DominantAxis(d)
	daSign := XIntCoordOf(d, axis).Sign()
	params := make([]EdgeParam, len(hits))
	for i, vi := range hits {
		if err := budget.Step(); err != nil {
			return err
		}
		ap := proof.Xsub(verts[vi], verts[a])
		params[i] = EdgeParam{Num: XIntCoordOf(ap, axis), Den: ap.W}
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
		for j := i; j > 0 && less(j, j-1); j-- {
			if err := budget.Step(); err != nil {
				return err
			}
			hits[j], hits[j-1] = hits[j-1], hits[j]
			params[j], params[j-1] = params[j-1], params[j]
		}
	}
	return nil
}

// TriangulatePlanarPolygon triangulates a planar polygon of mesh vertices
// (a facet boundary with collinear insertions) by exact ear clipping in the
// polygon's own plane, preserving orientation and using every vertex.
func TriangulatePlanarPolygon(ctx context.Context, verts []proof.Xpt, poly []int) ([][3]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if len(poly) < 3 {
		return nil, fmt.Errorf(`%w: a conforming polygon lost its corners`, decaderr.ErrBooleanFailed)
	}
	// The polygon is a triangle with edge insertions: its normal is the
	// original facet's, recoverable from any strict corner.
	n := proof.Xpt{X: new(big.Int), Y: new(big.Int), Z: new(big.Int), W: big.NewInt(1)}
	found := false
	for i := 1; i+1 < len(poly); i++ {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		cand := proof.Xcross(proof.Xsub(verts[poly[i]], verts[poly[0]]), proof.Xsub(verts[poly[i+1]], verts[poly[0]]))
		if cand.X.Sign() != 0 || cand.Y.Sign() != 0 || cand.Z.Sign() != 0 {
			n = cand
			found = true
			break
		}
	}
	if !found {
		return nil, fmt.Errorf(`%w: a conforming polygon is degenerate`, decaderr.ErrBooleanFailed)
	}
	u, v := ProjAxes(n)
	pts := make([]Xp2, len(poly))
	idx := make([]int, len(poly))
	for i, vi := range poly {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		pts[i] = NewXP2(RatCoordOf(verts[vi], u), RatCoordOf(verts[vi], v))
		idx[i] = i
	}
	// Keep the projected orientation counter-clockwise so ear clipping and
	// the emitted winding agree with the facet's own.
	area, err := PolyArea2Sign(budget, pts)
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
	tris2, err := EarClipX(budget, pts, idx)
	if err != nil {
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
