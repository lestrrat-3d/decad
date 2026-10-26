package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ParityIndexMinFacets is the smallest mesh the parity index is built for.
const ParityIndexMinFacets = 64

// ParityIndexMaxMemberships caps the (cell, facet) memberships of one index:
// an axis whose placement lists more gets no index and runs the plain scan.
// Each membership costs 8 bytes while the lists are collected (a cell and a
// facet id, up to 16 with slice growth) and 4 bytes once built, so the cap
// bounds the membership lists at about 130 MB while collected (one facet may
// append up to c² = 66 564 more before the check) and a built index at 32 MB.
// This is a bound on the memberships only: the build also holds the facet boxes
// (40 bytes per facet) and, for facets that project to a segment, the vertex
// projection cache, all allocated before the check, and they are not part of
// the bound. The real shells list about 43 thousand memberships per index (3 996 facets, 129 by 129
// cells), a factor of 190 below it. The count is taken on the int slices'
// lengths while collecting and checked after every facet, before the int32
// offsets are summed or any list is filled, so the offsets cannot overflow:
// the cap is far below MaxInt32. The cap is a count compared with >, so an
// index of exactly the cap is built.
const ParityIndexMaxMemberships = 1 << 23

// The range guard. Every float quantity the placement rounds is zero or lies
// in a fixed range of magnitudes:
//
//   - each projected vertex coordinate: zero or in [2⁻³⁰⁰, parityCoordHi = 2²⁴⁰];
//   - each coordinate difference du, dv and run offset t = x-a: zero or in
//     [parityGuardLo, parityGuardHi] = [2⁻³⁰⁰, 2²⁵⁰];
//   - each slope s = dv/du and product p = t·s: in [parityResultLo,
//     parityResultHi] = [2⁻⁶⁰⁰, 2⁵⁰⁰], except that s is zero exactly when dv
//     is zero and p is zero exactly when t or s is zero.
//
// A mesh with any quantity outside the guard, on one swept axis, gets no
// index for that axis (buildParityIndex returns nil) and the scan on that axis
// is the plain all-facets scan, so the answer is the old one. Inside the guard
// no multiplication or division underflows, overflows or turns a nonzero
// operand pair into zero, and a subtraction whose result is subnormal is
// exact; so every rounded result carries a relative error of at most
// ε = 2⁻⁵³ per operation and the exact slope's sign is the computed slope's
// sign. The guard never changes an answer: it only chooses "use the exact
// scan".
const (
	parityGuardLo  = 0x1p-300
	parityGuardHi  = 0x1p250
	parityCoordHi  = 0x1p240
	parityResultLo = 0x1p-600
	parityResultHi = 0x1p500
)

// parityGuarded reports whether f is zero or has magnitude in [2⁻³⁰⁰, 2²⁵⁰]. NaN
// and the infinities are outside.
func parityGuarded(f float64) bool {
	x := absf(f)
	return f == 0 || (x >= parityGuardLo && x <= parityGuardHi)
}

// parityResultInRange reports whether the nonzero quotient or product f has
// magnitude in [2⁻⁶⁰⁰, 2⁵⁰⁰]. Two operands of magnitude at least 2⁻³⁰⁰ give a
// result of at least 2⁻⁶⁰⁰, far above the subnormal range, so a result below
// it is an underflow of operands outside the guard.
func parityResultInRange(f float64) bool {
	x := absf(f)
	return x >= parityResultLo && x <= parityResultHi
}

// parityCoordGuarded reports whether the projected vertex coordinate f is zero
// or in [2⁻³⁰⁰, parityCoordHi].
func parityCoordGuarded(f float64) bool {
	x := absf(f)
	return f == 0 || (x >= parityGuardLo && x <= parityCoordHi)
}

// parityLineSlope returns the slope dv/du, with ok false when du, dv or the
// quotient is outside the range guard. du must be nonzero. The slope is zero
// exactly when dv is zero.
func parityLineSlope(du, dv float64) (float64, bool) {
	if du == 0 || !parityGuarded(du) || !parityGuarded(dv) {
		return 0, false
	}
	if dv == 0 {
		return 0, true
	}
	s := float64(dv / du)
	return s, parityResultInRange(s)
}

// ParityLineErrCoef and ParityLineFloor define the margin m that widens every
// float band the index places (lineAt):
//
//	m = ParityLineErrCoef·K + ParityLineFloor,  K = |b| + (|x|+|a|)·(1+|s|),
//	  = 1e-12·K + 2⁻¹⁰⁰⁰.
//
// Inside the range guard above, with ε = 2⁻⁵³: the slope has relative error at
// most 3ε, t = x-a at most ε, p = t·s at most ε and y = b+p at most ε, so
// |y - exact| <= ε|b| + 6.1ε(|x|+|a|)|s| + ε|p| <= 8ε·K. Rounding the band
// end y±m adds at most ε(|y|+m) (nothing when the result is subnormal), so
// the computed band end lies within 9.5ε·K + εm of the exact height's side.
// The margin covers this: when K = 0 every quantity is exact; otherwise
// K >= 2⁻³⁰⁰ (b, a and x are each zero or at least 2⁻³⁰⁰ in magnitude, and
// they are not all zero: a widened cell edge x is never zero, and x = a = 0
// is the vertex case that never reaches lineAt). The computed margin m̂ is not
// the real 1e-12·K + 2⁻¹⁰⁰⁰: K̂ = |b| + (|x|+|a|)·(1+|s|) carries four
// roundings (|x|+|a|, 1+|s|, their product, the sum with |b|), and the product
// with 1e-12 and the sum with the floor add two more, so m̂ is within a relative
// 7ε of 1e-12·K + 2⁻¹⁰⁰⁰ (each factor (1±ε), at most seven of them) and
// m̂ >= (1-7ε)·1e-12·K. That exceeds the 9.5ε·K + εm̂ above with a factor of
// about 900 to spare, since 1e-12 / (10·2⁻⁵³) ≈ 900. The floor is for K = 0
// only; it does not enter the bound. The slope, y and the band ends are separately
// rounded float64 operations, each forced by an explicit conversion, so no
// fused multiply-add enters and amd64 and arm64 build identical lists.
//
// The second widening is one float step: every cell keeps lo[k] = down(b[k])
// and hi[k] = up(b[k+1]), one step outside its edges (for |b| <= 2⁻³⁰⁰ the
// step is 2⁻²⁹⁹, which keeps x inside the guard), so a query whose exact
// coordinate rounds onto an edge is still inside the strip the lists were built
// for.
//
// Invariant: a facet is omitted from a cell only if its closed projection,
// widened by the margin, is disjoint from the cell, and the margin exceeds the
// maximum float error of the placement arithmetic. Float placement can thus
// only ADD facets to a cell's list, never drop one that can reach a query in
// it. TestParityLineMarginCoversTheError demands a 256-fold headroom of the
// margin over the measured error inside the guard.
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
		ax.lo[k] = parityStepOut(ax.b[k], math.Inf(-1))
		ax.hi[k] = parityStepOut(ax.b[k+1], math.Inf(1))
		ax.abs[2*k], ax.abs[2*k+1] = absf(ax.lo[k]), absf(ax.hi[k])
	}
	return ax
}

// parityStepOut moves the cell edge b one float step toward the infinity inf,
// or, for |b| <= 2⁻³⁰⁰ (including zero, whose neighbours are subnormal), by
// 2⁻²⁹⁹: either is outward of b, and the result is zero-free and inside the
// range guard. An infinite b stays infinite.
func parityStepOut(b, inf float64) float64 {
	if math.IsInf(b, 0) {
		return b
	}
	if absf(b) <= parityGuardLo {
		return b + math.Copysign(0x1p-299, inf)
	}
	return math.Nextafter(b, inf)
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

// membershipLimit is the membership cap in force: the production cap, or the
// test override when that is positive and lower. An override can never raise it.
func (pm *ParityMesh) membershipLimit() int {
	if pm.maxMemberships > 0 {
		return min(pm.maxMemberships, ParityIndexMaxMemberships)
	}
	return ParityIndexMaxMemberships
}

// buildParityIndex builds the index for one swept axis, or returns nil when
// the mesh is too small or has a facet the grid cannot place.
func (pm *ParityMesh) buildParityIndex(axis, u, v int) *ParityIndex {
	n := len(pm.Tris)
	if n < ParityIndexMinFacets || n > math.MaxInt32 {
		return nil
	}
	limit := pm.membershipLimit()
	boxes := pm.FacetBoxes(axis)
	umin, umax := math.Inf(1), math.Inf(-1)
	vmin, vmax := math.Inf(1), math.Inf(-1)
	for ti := range pm.Tris {
		if !boxes[ti].Built {
			boxes[ti] = pm.BuildFacetBox(axis, u, v, ti)
		}
		b := &boxes[ti]
		for _, vi := range pm.Tris[ti] {
			vert := pm.Verts[vi]
			if !parityCoordGuarded(CoordOf(vert, u)) || !parityCoordGuarded(CoordOf(vert, v)) {
				return nil
			}
		}
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
		if len(cells) > limit {
			return nil // too many memberships: the scan stays plain
		}
		b := &boxes[ti]
		if b.Nondegenerate {
			var ok bool
			cells, facets, ok = idx.addTriangle(pm, tri, u, v, ti, b, cells, facets)
			if !ok {
				return nil
			}
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
	if len(cells) > limit {
		return nil
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
	s, ok := parityLineSlope(float64(ra-a), float64(rb-b))
	if !ok {
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
			y, m, ok := lineAt(a, b, s, absB, onePlusS, xf, xa)
			if !ok {
				return cells, facets, false
			}
			lo, hi = y-m, y+m
			if s != 0 {
				if (s > 0) == math.IsInf(x1, 1) {
					hi = math.Inf(1)
				} else {
					lo = math.Inf(-1)
				}
			}
		default:
			y0, m0, ok0 := lineAt(a, b, s, absB, onePlusS, x0, run.abs[2*i])
			y1, m1, ok1 := lineAt(a, b, s, absB, onePlusS, x1, run.abs[2*i+1])
			if !ok0 || !ok1 {
				return cells, facets, false
			}
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
// strictly outside. It reports false when a slope or band is outside the
// range guard, which refuses the index.
func (idx *ParityIndex) addTriangle(pm *ParityMesh, tri [3]int, u, v, ti int, box *ParityFacetBox,
	cells, facets []int32) ([]int32, []int32, bool) {
	c := len(idx.v.lo)
	var pu, pv [3]float64
	for k := range 3 {
		vert := pm.Verts[tri[k]]
		pu[k], pv[k] = CoordOf(vert, u), CoordOf(vert, v)
	}
	i0, i1 := idx.u.cellOf(box.MinU), idx.u.cellOf(box.MaxU)
	for i := i0; i <= i1; i++ {
		x0, x1 := math.Max(idx.u.lo[i], box.MinU), math.Min(idx.u.hi[i], box.MaxU)
		lo, hi := math.Inf(1), math.Inf(-1)
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
			s, ok := parityLineSlope(float64(bu-au), float64(bv-av))
			if !ok {
				return cells, facets, false
			}
			absB, onePlusS := absf(av), 1+absf(s)
			for _, x := range [2]float64{math.Max(x0, au), math.Min(x1, bu)} {
				switch x {
				case au:
					lo, hi = math.Min(lo, av), math.Max(hi, av)
				case bu:
					lo, hi = math.Min(lo, bv), math.Max(hi, bv)
				default:
					y, m, ok := lineAt(au, av, s, absB, onePlusS, x, absf(x))
					if !ok {
						return cells, facets, false
					}
					lo, hi = math.Min(lo, y-m), math.Max(hi, y+m)
				}
			}
		}
		if lo != lo || hi != hi {
			return cells, facets, false
		}
		lo, hi = math.Max(lo, box.MinV), math.Min(hi, box.MaxV)
		if lo > hi {
			continue
		}
		jHi := idx.v.cellOf(hi)
		for j := idx.v.cellOf(lo); j <= jHi; j++ {
			cells = append(cells, int32(i*c+j))
			facets = append(facets, int32(ti))
		}
	}
	return cells, facets, true
}

// lineAt evaluates the line through (a, b) with slope s at the finite x, with
// the margin of ParityLineErrCoef bounding the evaluation's error. It reports
// false when t = x-a or the product t·s is outside the range guard (zero is
// allowed only as an exact zero: t·s is zero exactly when t or s is zero).
// Inside the guard the error is at most 8ε·(|b| + (|x|+|a|)·|s|) before
// rounding the band ends; see ParityLineErrCoef for the margin's proof.
// absB is |b| and onePlusS is 1+|s|, fixed per line; xa is |x|.
func lineAt(a, b, s, absB, onePlusS, x, xa float64) (float64, float64, bool) {
	t := float64(x - a)
	if !parityGuarded(t) {
		return 0, 0, false
	}
	p := float64(t * s)
	if t != 0 && s != 0 && !parityResultInRange(p) {
		return 0, 0, false
	}
	y := float64(b + p)
	m := float64(ParityLineErrCoef*float64(absB+float64(float64(xa+absf(a))*onePlusS))) + ParityLineFloor
	return y, m, true
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
