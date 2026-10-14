package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ParityIndexMinFacets is the smallest mesh the parity index is built for.
const ParityIndexMinFacets = 64

// ParityLineErrCoef and ParityLineFloor define the margin m that widens every
// float band the index places (lineAt):
//
//	m = ParityLineErrCoef·(|b| + (|x|+|a|)·(1+|s|)) + ParityLineFloor
//	  = 1e-12·(|b| + (|x|+|a|)·(1+|s|)) + 2⁻¹⁰⁰⁰.
//
// The slope (two subtractions and a division) and the band's height y (a
// subtraction, a product and a sum) are separately rounded float64 operations,
// each forced by an explicit conversion so no fused multiply-add enters and
// amd64 and arm64 build identical lists. With ε = 2⁻⁵³ the
// total error of y and of the rounded band end y±m is below
// 10ε·(|b| + (|x|+|a|)·(1+|s|)) + 2⁻¹⁰⁷¹, so the first term of m is about 900
// times the proven rounding error (1e-12 / (10·2⁻⁵³) ≈ 900) and the floor is
// far above the gradual-underflow crumb. A band widened by m therefore always
// contains the exact height. The second widening is one float step: every
// cell keeps lo[k] = down(b[k]) and hi[k] = up(b[k+1]), one step outside its
// edges, so a query whose exact coordinate rounds onto an edge is still inside
// the strip the lists were built for.
//
// Invariant: a facet is omitted from a cell only if its closed projection,
// widened by the margin, is disjoint from the cell, and the margin exceeds the
// maximum float error of the placement arithmetic. Float placement can thus
// only ADD facets to a cell's list, never drop one that can reach a query in
// it. TestParityLineMarginCoversTheError demands a 256-fold headroom of the
// margin over the measured error.
const (
	ParityLineErrCoef = 1e-12
	ParityLineFloor   = 0x1p-1000
)

// parityGridAxis is one axis of the index grid: c cells split by the
// non-decreasing boundaries b[0] = -Inf < ... < b[c] = +Inf, the two end
// cells unbounded. A query coordinate f lands in cell cellOf(f), where
// b[k] <= f <= b[k+1]. Because f is the nearest float to the query's exact
// coordinate and rounding is monotone, that exact coordinate lies in
// [lo[k], hi[k]] = [down(b[k]), up(b[k+1])].
type parityGridAxis struct {
	b, lo, hi, abs []float64
}

func newParityGridAxis(first, last float64, inner int) parityGridAxis {
	c := inner + 2
	ax := parityGridAxis{
		b:   make([]float64, c+1),
		lo:  make([]float64, c),
		hi:  make([]float64, c),
		abs: make([]float64, 2*c),
	}
	ax.b[0], ax.b[1], ax.b[c-1], ax.b[c] = math.Inf(-1), first, last, math.Inf(1)
	w := last - first
	for k := 1; k < inner; k++ {
		// The conversions keep each step separately rounded on every
		// architecture, so the grid does not depend on fused multiply-add.
		x := float64(first + float64(w*float64(float64(k)/float64(inner))))
		ax.b[k+1] = math.Min(math.Max(x, ax.b[k]), last)
	}
	for k := range c {
		ax.lo[k] = math.Nextafter(ax.b[k], math.Inf(-1))
		ax.hi[k] = math.Nextafter(ax.b[k+1], math.Inf(1))
		ax.abs[2*k], ax.abs[2*k+1] = absf(ax.lo[k]), absf(ax.hi[k])
	}
	return ax
}

func absf(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// cellOf counts the inner boundaries b[1..c-1] at or below f: a monotone
// function of f, in [0, c-1].
func (ax *parityGridAxis) cellOf(f float64) int {
	lo, hi := 1, len(ax.b)-1
	for lo < hi {
		mid := int(uint(lo+hi) >> 1)
		if ax.b[mid] <= f {
			lo = mid + 1
		} else {
			hi = mid
		}
	}
	return lo - 1
}

// ParityIndex is a grid over one swept axis's projection plane that lists, per
// cell, every facet that can do anything but `continue` in the parity scan of
// a query whose float projection lands in that cell. The cell contract: a
// facet is left out of a cell only when the query's exact projection, which
// lies in [lo[k], hi[k]] on each axis (parityGridAxis), is provably outside
// the facet's closed projected triangle, or off the facet's whole line when it
// projects to a segment, widened by the margin of ParityLineErrCoef; of the
// zero-area facets on one axis-parallel line only the first in index order is
// listed, because for any query on that line the first one is ambiguous and
// ends the scan, and off the line every one continues. The grid decides no
// sign: it only chooses which facets the unchanged exact scan visits, in
// ascending facet order. The index is immutable once built.
type ParityIndex struct {
	u, v  parityGridAxis
	start []int32 // cell c lists ids[start[c]:start[c+1]]
	ids   []int32
}

// buildParityIndex builds the index for one swept axis, or returns nil when
// the mesh is too small or has a facet the grid cannot place.
func (pm *ParityMesh) buildParityIndex(axis, u, v int) *ParityIndex {
	n := len(pm.Tris)
	if n < ParityIndexMinFacets {
		return nil
	}
	boxes := pm.FacetBoxes(axis)
	umin, umax := math.Inf(1), math.Inf(-1)
	vmin, vmax := math.Inf(1), math.Inf(-1)
	for ti := range pm.Tris {
		if !boxes[ti].Built {
			boxes[ti] = pm.BuildFacetBox(axis, u, v, ti)
		}
		b := &boxes[ti]
		if proofbound.IsNonFinite(b.MinU) || proofbound.IsNonFinite(b.MaxU) ||
			proofbound.IsNonFinite(b.MinV) || proofbound.IsNonFinite(b.MaxV) {
			return nil
		}
		umin, umax = math.Min(umin, b.MinU), math.Max(umax, b.MaxU)
		vmin, vmax = math.Min(vmin, b.MinV), math.Max(vmax, b.MaxV)
	}
	if !(umin < umax) || !(vmin < vmax) {
		return nil
	}
	inner := min(max(int(math.Ceil(2*math.Sqrt(float64(n)))), 4), 256)
	idx := &ParityIndex{u: newParityGridAxis(umin, umax, inner), v: newParityGridAxis(vmin, vmax, inner)}
	c := inner + 2
	var cells, facets []int32
	flat, upright := map[float64]struct{}{}, map[float64]struct{}{}
	for ti, tri := range pm.Tris {
		b := &boxes[ti]
		if b.Nondegenerate {
			cells, facets = idx.addTriangle(pm, tri, u, v, ti, b, cells, facets)
			continue
		}
		// Facets on one axis-parallel line answer alike: off the line every
		// one continues, and on it the first in index order is ambiguous and
		// ends the scan. Only that first facet is listed.
		a, b2, c2 := pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]
		sameU := CoordOf(a, u) == CoordOf(b2, u) && CoordOf(a, u) == CoordOf(c2, u)
		sameV := CoordOf(a, v) == CoordOf(b2, v) && CoordOf(a, v) == CoordOf(c2, v)
		if sameU && sameV {
			return nil // one point: every query on this ray is ambiguous there
		}
		if fv := CoordOf(a, v); sameV {
			if _, seen := flat[fv]; seen {
				continue
			}
			flat[fv] = struct{}{}
		} else if fu := CoordOf(a, u); sameU {
			if _, seen := upright[fu]; seen {
				continue
			}
			upright[fu] = struct{}{}
		}
		var ok bool
		cells, facets, ok = idx.addLine(pm, tri, u, v, ti, cells, facets)
		if !ok {
			return nil
		}
	}
	// Counting sort by cell; entries arrive in ascending facet order, so each
	// cell's list stays ascending.
	idx.start = make([]int32, c*c+1)
	for _, cell := range cells {
		idx.start[cell+1]++
	}
	for k := 1; k <= c*c; k++ {
		idx.start[k] += idx.start[k-1]
	}
	idx.ids = make([]int32, len(cells))
	fill := make([]int32, c*c)
	copy(fill, idx.start[:c*c])
	for e, cell := range cells {
		idx.ids[fill[cell]] = facets[e]
		fill[cell]++
	}
	return idx
}

// addLine lists a facet whose projection has exactly zero area in every cell
// its line may pass through. It reports false for a facet the grid cannot
// place: a projection that is one point (every query is then ambiguous on
// this ray) or a band that is not finite.
func (idx *ParityIndex) addLine(pm *ParityMesh, tri [3]int, u, v, ti int,
	cells, facets []int32) ([]int32, []int32, bool) {
	var pu, pv [3]float64
	for k := range 3 {
		vert := pm.Verts[tri[k]]
		pu[k], pv[k] = CoordOf(vert, u), CoordOf(vert, v)
	}
	// The farthest-apart distinct pair spans the line; all three corners lie
	// on it.
	best, bk := -1.0, -1
	for k := range 3 {
		l := (k + 1) % 3
		if pu[l] == pu[k] && pv[l] == pv[k] {
			continue
		}
		if d := absf(pu[l]-pu[k]) + absf(pv[l]-pv[k]); d > best {
			best, bk = d, k
		}
	}
	if bk < 0 {
		return cells, facets, false
	}
	l := (bk + 1) % 3
	a, b, ra, rb := pu[bk], pv[bk], pu[l], pv[l]
	run, cross := &idx.u, &idx.v
	shallow := absf(ra-a) >= absf(rb-b)
	if !shallow {
		a, b, ra, rb = b, a, rb, ra
		run, cross = &idx.v, &idx.u
	}
	du, dv := float64(ra-a), float64(rb-b)
	s := float64(dv / du)
	if proofbound.IsNonFinite(du) || proofbound.IsNonFinite(dv) || proofbound.IsNonFinite(s) {
		return cells, facets, false
	}
	absB, onePlusS := absf(b), 1+absf(s)
	c := len(run.lo)
	for i := range c {
		x0, x1 := run.lo[i], run.hi[i]
		var lo, hi float64
		switch {
		case math.IsInf(x0, -1) || math.IsInf(x1, 1):
			// An unbounded end cell: the height is the constant b when the
			// slope is exactly zero, and otherwise a half-line from the finite
			// edge, open toward +Inf when the slope's sign and the unbounded
			// side agree.
			xf, xa := x1, run.abs[2*i+1]
			if math.IsInf(x1, 1) {
				xf, xa = x0, run.abs[2*i]
			}
			y, m := lineAt(a, b, s, absB, onePlusS, xf, xa)
			lo, hi = y-m, y+m
			if s != 0 {
				if (s > 0) == math.IsInf(x1, 1) {
					hi = math.Inf(1)
				} else {
					lo = math.Inf(-1)
				}
			}
		default:
			y0, m0 := lineAt(a, b, s, absB, onePlusS, x0, run.abs[2*i])
			y1, m1 := lineAt(a, b, s, absB, onePlusS, x1, run.abs[2*i+1])
			m := math.Max(m0, m1)
			lo, hi = math.Min(y0, y1)-m, math.Max(y0, y1)+m
		}
		if lo != lo || hi != hi || math.IsInf(lo, 1) || math.IsInf(hi, -1) {
			return cells, facets, false
		}
		j0, j1 := cross.cellOf(lo), cross.cellOf(hi)
		for j := j0; j <= j1; j++ {
			cell := i*c + j
			if !shallow {
				cell = j*c + i
			}
			cells = append(cells, int32(cell))
			facets = append(facets, int32(ti))
		}
	}
	return cells, facets, true
}

// addTriangle lists a facet whose projection has nonzero area in every cell
// of its box column range that the closed projected triangle may reach: per
// column, the union of its three edges' height bands over the column's exact
// strip, clipped to the box. A query outside the closed triangle classifies
// strictly outside. When an edge band is not finite, the facet's box cells are
// listed instead, which the box filter already justifies.
func (idx *ParityIndex) addTriangle(pm *ParityMesh, tri [3]int, u, v, ti int, box *ParityFacetBox,
	cells, facets []int32) ([]int32, []int32) {
	c := len(idx.v.lo)
	var pu, pv [3]float64
	for k := range 3 {
		vert := pm.Verts[tri[k]]
		pu[k], pv[k] = CoordOf(vert, u), CoordOf(vert, v)
	}
	i0, i1 := idx.u.cellOf(box.MinU), idx.u.cellOf(box.MaxU)
	j0, j1 := idx.v.cellOf(box.MinV), idx.v.cellOf(box.MaxV)
	mark := len(cells)
	for i := i0; i <= i1; i++ {
		x0, x1 := math.Max(idx.u.lo[i], box.MinU), math.Min(idx.u.hi[i], box.MaxU)
		lo, hi := math.Inf(1), math.Inf(-1)
		ok := true
		for k := range 3 {
			l := (k + 1) % 3
			au, av, bu, bv := pu[k], pv[k], pu[l], pv[l]
			if au > bu {
				au, av, bu, bv = bu, bv, au, av
			}
			if bu < x0 || au > x1 {
				continue
			}
			if au == bu {
				lo, hi = math.Min(lo, math.Min(av, bv)), math.Max(hi, math.Max(av, bv))
				continue
			}
			s := float64(float64(bv-av) / float64(bu-au))
			absB, onePlusS := absf(av), 1+absf(s)
			for _, x := range [2]float64{math.Max(x0, au), math.Min(x1, bu)} {
				switch x {
				case au:
					lo, hi = math.Min(lo, av), math.Max(hi, av)
				case bu:
					lo, hi = math.Min(lo, bv), math.Max(hi, bv)
				default:
					y, m := lineAt(au, av, s, absB, onePlusS, x, absf(x))
					if proofbound.IsNonFinite(y) || proofbound.IsNonFinite(m) {
						ok = false
					}
					lo, hi = math.Min(lo, y-m), math.Max(hi, y+m)
				}
			}
		}
		if !ok || lo != lo || hi != hi {
			cells, facets = cells[:mark], facets[:mark]
			for i := i0; i <= i1; i++ {
				for j := j0; j <= j1; j++ {
					cells = append(cells, int32(i*c+j))
					facets = append(facets, int32(ti))
				}
			}
			return cells, facets
		}
		lo, hi = math.Max(lo, box.MinV), math.Min(hi, box.MaxV)
		if lo > hi {
			continue
		}
		for j := idx.v.cellOf(lo); j <= idx.v.cellOf(hi); j++ {
			cells = append(cells, int32(i*c+j))
			facets = append(facets, int32(ti))
		}
	}
	return cells, facets
}

// lineAt evaluates the line through (a, b) with slope s at the finite x, with
// the margin of ParityLineErrCoef bounding the evaluation's error. A
// non-finite result makes the band NaN or infinite, which addLine refuses and
// addTriangle answers with the box fallback.
//
// With S the exact slope, ε = 2⁻⁵³ and s within 4ε of S relative, the computed
// y is within 8ε·(|b| + (|x|+|a|)·|s|) of the exact height, and rounding the
// band ends adds at most ε of their magnitude; the margin is about 900 times
// that. absB is |b| and onePlusS is 1+|s|, fixed per line; xa is |x|.
func lineAt(a, b, s, absB, onePlusS, x, xa float64) (float64, float64) {
	t := float64(x - a)
	p := float64(t * s)
	y := float64(b + p)
	return y, float64(ParityLineErrCoef*float64(absB+float64(float64(xa+absf(a))*onePlusS))) + ParityLineFloor
}

// candidates returns the cell list for a finite float projection.
func (idx *ParityIndex) candidates(fu, fv float64) []int32 {
	c := len(idx.v.lo)
	cell := idx.u.cellOf(fu)*c + idx.v.cellOf(fv)
	return idx.ids[idx.start[cell]:idx.start[cell+1]]
}

// parityIndex returns the axis's index, building it on first use.
func (pm *ParityMesh) parityIndex(axis, u, v int) *ParityIndex {
	if !pm.indexBuilt[axis] {
		pm.indexBuilt[axis] = true
		pm.index[axis] = pm.buildParityIndex(axis, u, v)
	}
	return pm.index[axis]
}
