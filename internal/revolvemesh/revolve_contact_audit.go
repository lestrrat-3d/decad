package revolvemesh

import (
	"cmp"
	"fmt"
	"math"
	"runtime"
	"slices"
	"sync"
	"sync/atomic"
	"time"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// RevolveAuditTri is one triangle's exact lift, held for the whole audit: its
// three corners, the three edge vectors (u = p1−p0, v = p2−p0, w = p2−p1),
// their cross product, and proven upper bounds on the three edge lengths.
// Every predicate below reads these rather than rebuilding them per pair,
// exactly as internal/loftmesh/loft_audit.go's own audit data does. fp holds the stored float
// corners, and fu, fv, fw and fn enclose u, v, w and n in float intervals for
// the pre-test (tessellate_revolve_filter.go). FnProj is RevIvProject(Fp,
// Fn), the pre-test's projection of the triangle onto its own normal, which no
// pair changes.
type RevolveAuditTri struct {
	P              [3]proofarith.DyV3
	U, V, W, N     proofarith.DyV3
	Lu, Lv, Lw     float64
	Box            [2]r3.Vec
	Fp             [3]r3.Vec
	Fu, Fv, Fw, Fn RevIvVec
	FnProj         [2]float64
	// off[k] holds the two corner offsets measured from corner k, in
	// increasing corner order, each with its proven length bound.
	Off [3][2]RevolveOffset
}

func NewRevolveAuditTri(verts []r3.Vec, tri [3]int) (RevolveAuditTri, bool) {
	var out RevolveAuditTri
	for k, vi := range tri {
		if !proofbound.FiniteVec(verts[vi]) {
			return out, false
		}
		out.P[k] = proofarith.DyVec(verts[vi])
		out.Fp[k] = verts[vi]
	}
	out.U = proofarith.DvSub(out.P[1], out.P[0])
	out.V = proofarith.DvSub(out.P[2], out.P[0])
	out.W = proofarith.DvSub(out.P[2], out.P[1])
	out.N = proofarith.DvCross(out.U, out.V)
	out.Fu = RevIvPointDiff(out.Fp[1], out.Fp[0])
	out.Fv = RevIvPointDiff(out.Fp[2], out.Fp[0])
	out.Fw = RevIvPointDiff(out.Fp[2], out.Fp[1])
	out.Fn = RevIvCross(out.Fu, out.Fv)
	out.FnProj[0], out.FnProj[1] = RevIvProject(out.Fp, out.Fn)
	out.Lu = proofbound.DvLenUpper(out.U)
	out.Lv = proofbound.DvLenUpper(out.V)
	out.Lw = proofbound.DvLenUpper(out.W)
	out.Box = meshbool.TriBox(verts, tri)
	out.Off[0] = [2]RevolveOffset{{V: out.U, F: out.Fu, Length: out.Lu}, {V: out.V, F: out.Fv, Length: out.Lv}}
	out.Off[1] = [2]RevolveOffset{
		RevolveOffsetOf(proofarith.DvSub(out.P[0], out.P[1]), RevIvPointDiff(out.Fp[0], out.Fp[1])),
		{V: out.W, F: out.Fw, Length: out.Lw},
	}
	out.Off[2] = [2]RevolveOffset{
		RevolveOffsetOf(proofarith.DvSub(out.P[0], out.P[2]), RevIvPointDiff(out.Fp[0], out.Fp[2])),
		RevolveOffsetOf(proofarith.DvSub(out.P[1], out.P[2]), RevIvPointDiff(out.Fp[1], out.Fp[2])),
	}
	return out, true
}

// RevolveSeparated proves two triangles stay farther apart than 2·delta, so
// no member of the displaced family they stand for can touch. It is the
// separating-axis theorem over the exact rationals: a single axis on which the
// two projections leave a gap wider than the two triangles' own displacement
// budget proves the disjointness for the whole family, since a point sliding by
// at most delta moves its projection onto a unit axis by at most delta.
//
// The seventeen candidates are the two face normals, the nine edge-pair cross
// products, and each face normal crossed with its own three edges — the set
// that decides every disjoint pair of triangles, coplanar ones included. A
// candidate that vanishes carries no information and is skipped, and failing on
// all of them is a refusal, never an admission. The axes are built in the same
// order as before, one at a time as the loop reaches them, and each one's
// length bound is tried in its cheap form (|x × y| ≤ |x||y|) before its exact
// form, so a pair that a low-index axis decides never pays for the rest. The
// float pre-test runs the same walk first (RevolveSeparatedFloat); an axis it
// accepts is one the exact walk accepts too.
func RevolveSeparated(a, b RevolveAuditTri, delta float64) bool {
	return RevolveSeparatedFloat(a, b, delta) || RevolveSeparatedExact(a, b, delta)
}

// RevolveSeparatedExact is RevolveSeparated's walk over the exact Dyadic axes.
func RevolveSeparatedExact(a, b RevolveAuditTri, delta float64) bool {
	margin := proofbound.ProductUpper(2, delta)
	if proofbound.IsNonFinite(margin) {
		return false
	}
	ea := [3]proofarith.DyV3{a.U, a.V, a.W}
	eb := [3]proofarith.DyV3{b.U, b.V, b.W}
	la := [3]float64{a.Lu, a.Lv, a.Lw}
	lb := [3]float64{b.Lu, b.Lv, b.Lw}
	lnA := proofbound.ProductUpper(a.Lu, a.Lv)
	lnB := proofbound.ProductUpper(b.Lu, b.Lv)
	// axis is one candidate, built only when the candidates before it have
	// failed, beside a cheap proven upper bound on its length: |x × y| ≤ |x||y|.
	type axis struct {
		g     proofarith.DyV3
		bound float64
	}
	next := func(gi int) axis {
		switch {
		case gi == 0:
			return axis{a.N, lnA}
		case gi == 1:
			return axis{b.N, lnB}
		case gi < 11:
			x, y := (gi-2)/3, (gi-2)%3
			return axis{proofarith.DvCross(ea[x], eb[y]), proofbound.ProductUpper(la[x], lb[y])}
		case gi < 14:
			x := gi - 11
			return axis{proofarith.DvCross(a.N, ea[x]), proofbound.ProductUpper(lnA, la[x])}
		default:
			y := gi - 14
			return axis{proofarith.DvCross(b.N, eb[y]), proofbound.ProductUpper(lnB, lb[y])}
		}
	}
	for gi := range 17 {
		ax := next(gi)
		g := ax.g
		if proofarith.DvIsZero(g) {
			continue
		}
		aLo, aHi := DvProject(a.P, g)
		bLo, bHi := DvProject(b.P, g)
		gap := proofarith.DySubScalar(bLo, aHi)
		if other := proofarith.DySubScalar(aLo, bHi); proofarith.DyCmp(other, gap) > 0 {
			gap = other
		}
		if gap.Sign() <= 0 {
			continue
		}
		// The cheap bound is at least the exact one, so a gap that clears it
		// clears the exact one too; a gap that does not is retried exactly.
		if cheap, ok := proofarith.DyOf(proofbound.ProductUpper(margin, ax.bound)); ok && proofarith.DyCmp(gap, cheap) > 0 {
			return true
		}
		if need, ok := proofarith.DyOf(proofbound.ProductUpper(margin, proofbound.DvLenUpper(g))); ok && proofarith.DyCmp(gap, need) > 0 {
			return true
		}
	}
	return false
}

// DvProject is the exact projection range of a triangle's three corners onto
// one axis, before normalisation.
func DvProject(p [3]proofarith.DyV3, g proofarith.DyV3) (proofarith.Dyadic, proofarith.Dyadic) {
	lo := proofarith.DvDot(p[0], g)
	hi := lo
	for _, q := range p[1:] {
		d := proofarith.DvDot(q, g)
		if proofarith.DyCmp(d, lo) < 0 {
			lo = d
		}
		if proofarith.DyCmp(d, hi) > 0 {
			hi = d
		}
	}
	return lo, hi
}

// BoxGapExceeds is the float pre-filter in front of RevolveSeparated: two
// axis-aligned boxes separated on one coordinate by more than margin prove the
// same thing the exact test would, for the cost of six comparisons. The
// difference is rounded DOWNWARD, so a gap this reports is one the exact
// arithmetic also has.
func BoxGapExceeds(a, b [2]r3.Vec, margin float64) bool {
	gap := func(x, y float64) bool { return freeform.DownRound(y-x) > margin }
	return gap(a[1].X, b[0].X) || gap(b[1].X, a[0].X) ||
		gap(a[1].Y, b[0].Y) || gap(b[1].Y, a[0].Y) ||
		gap(a[1].Z, b[0].Z) || gap(b[1].Z, a[0].Z)
}

// RequireRevolveFacetAreas proves agent-docs/tessellation-design.md §1's positive-area
// row over the whole mesh and returns the audit triangles it built on the way.
//
// It stands OUTSIDE RevolveContactAudit, and runs at every verification level,
// because the two answer different questions at different costs. Positive area
// is per FACET and linear; a zero-area facet has no normal, so a renderer, the
// exporter's own facet normals and the boolean all need it. Contact is per
// facet PAIR and quadratic, and it is what a caller who only wants to draw the
// mesh declines. Folding the first into the second would silently drop §1's
// positive-area row from every mesh built below VerifyBoundary.
//
// The returned slice is parallel to tris, so RevolveContactAudit consumes it
// rather than walking the facets a second time. delta is the combined
// coordinate displacement deltaC + deltaR, as the audit's own doc comment
// derives it.
func RequireRevolveFacetAreas(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, delta float64) ([]RevolveAuditTri, error) {
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if proofbound.IsNonFinite(delta) || delta < 0 {
		return nil, fmt.Errorf(`%w: this revolve mesh states no finite coordinate displacement, so its facets cannot be audited`, decaderr.ErrUnsupported)
	}
	data := make([]RevolveAuditTri, len(tris))
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		t, ok := NewRevolveAuditTri(verts, tri)
		if !ok {
			return nil, fmt.Errorf(`%w: revolve facet %d holds a coordinate that is not finite`, decaderr.ErrUnsupported, i)
		}
		if err := RequireRevolveFacetArea(t, i, delta); err != nil {
			return nil, err
		}
		data[i] = t
	}
	return data, budget.Err()
}

// RevolveContactAudit is agent-docs/tessellation-design.md §9's facet-contact audit:
// adjacent facets meet ONLY along the vertex or edge their indices share, and
// no non-adjacent pair touches at all. It runs at VerifyBoundary and above, and
// data is RequireRevolveFacetAreas' own output for the same triangle set, which
// has already proven every facet positive-area.
//
// §9 asks for that verdict four times over — at the ideal-coordinate endpoint,
// at the stored unplaced endpoint, and across the two affine homotopies that
// join them. This runs it ONCE, at the final stored coordinates, against the
// COMBINED displacement delta = deltaC + deltaR, and that single pass carries
// all four. The argument is the one §9's own homotopies are built on:
//
//   - Every mesh on the path is a vertex-wise displacement of the final stored
//     mesh by at most delta. The construction stage joins the ideal unplaced
//     mesh to the stored unplaced one within deltaC; the exact rigid placement
//     is an ISOMETRY, so it carries that whole family into placed space
//     unchanged in shape and within deltaR of the final vertices; the placement
//     stage joins the rigid image to the final mesh within deltaR. Composing
//     the two, every vertex of every intermediate boundary lies within
//     deltaC + deltaR of the vertex this mesh stored for it.
//   - Every reading the verdict rests on is a POLYNOMIAL in those vertices, so
//     a bound on the vertices' motion bounds the reading's
//     (PerturbBilinearAllow). A reading whose stored value exceeds its own
//     allowance cannot change sign anywhere on the family, and a reading that
//     is structurally zero — a corner lying in a plane that was BUILT through
//     it — stays zero at every point of the family.
//   - Those signs decide the contact outright. A pair sharing a vertex or an
//     edge is isolated by a half-space whose boundary plane contains the shared
//     feature identically, with one triangle inside it and the other's
//     non-shared corners strictly outside (AuditRevolvePair's own doc comment
//     carries the candidates). A pair sharing nothing is proven apart by a
//     fixed separating axis with the same margin.
//
// So every intermediate boundary is embedded and its contact relations are the
// ones the stored mesh has, which is exactly what §9 charges to deltaC and
// deltaR. A pair the audit cannot decide is ErrUnsupported (§12), never an
// admission.
func RevolveContactAudit(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, delta float64) error {
	if err := budget.Err(); err != nil {
		return err
	}
	if len(data) != len(tris) {
		return fmt.Errorf(`%w: the revolve facet audit holds %d triangles for a mesh of %d facets`, decaderr.ErrUnsupported, len(data), len(tris))
	}

	f := len(tris)
	pairs, ok := proofbound.WallChoose2(uint64(f))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		err := fmt.Errorf(`%w: this revolve mesh's facet-pair audit needs %d exact tests, past the fixed ceiling of %d; retry with a coarser chord tolerance`, decaderr.ErrUnsupported, pairs, proofbound.MaxFacetPairTestsPerCall)
		if ok {
			return proofbound.WithWorkLimit(err, "facet_pairs", pairs, proofbound.MaxFacetPairTestsPerCall)
		}
		return err
	}
	margin := proofbound.ProductUpper(2, delta)
	if !(margin >= 0) {
		return revolveContactAuditAllPairs(budget, data, tris, delta, margin)
	}
	rows, err := revolveContactCandidates(budget, data, tris, margin)
	if err != nil {
		return err
	}
	return revolveAuditRows(budget, data, tris, rows, delta, margin,
		runtime.GOMAXPROCS(0) > 1 && revolveRowsLen(rows) >= revolveParallelMinPairs)
}

// revolveParallelMinPairs is the candidate count from which the audit runs its
// pairs on several goroutines.
const revolveParallelMinPairs = 20000

func revolveRowsLen(rows [][]int32) int {
	n := 0
	for _, row := range rows {
		n += len(row)
	}
	return n
}

// revolveAuditRows audits every listed pair in (i, j) order and returns the
// first error that order meets. With parallel set, the pairs run on several
// goroutines and the result is still the serial one.
func revolveAuditRows(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, rows [][]int32,
	delta, margin float64, parallel bool) error {
	if parallel {
		return revolveAuditRowsParallel(budget, data, tris, rows, delta, margin)
	}
	axes := newRevolveAxisCache(len(data))
	for i, row := range rows {
		for _, jj := range row {
			j := int(jj)
			if err := budget.Step(); err != nil {
				return err
			}
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if count == 0 && BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			if err := auditRevolvePairCached(data, tris, shared, count, i, j, delta, axes); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}

// revolveAuditRowsParallel splits the flat candidate list into contiguous
// chunks, one per worker, each with its own axis cache. A worker records the
// smallest failing flat index it meets and stops once it is past the smallest
// failure any worker has recorded, so the lowest failing index is always
// found. The caller then replays the budget steps in list order up to that
// index, so the returned error is the one the serial loop returns first. Only
// the caller touches the budget; workers never do. If cancellation interrupts
// the workers, the caller runs the serial walk because stopped workers may not
// have found an earlier facet error.
func revolveAuditRowsParallel(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, rows [][]int32,
	delta, margin float64) error {
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	return revolveAuditRowsParallelWithPoll(budget, data, tris, rows, delta, margin, ticker.C, nil)
}

func revolveAuditRowsParallelWithPoll(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int,
	rows [][]int32, delta, margin float64, poll <-chan time.Time, beforeWorker func()) error {
	total := revolveRowsLen(rows)
	is := make([]int32, 0, total)
	js := make([]int32, 0, total)
	for i, row := range rows {
		for _, j := range row {
			is = append(is, int32(i))
			js = append(js, j)
		}
	}
	workers := min(runtime.GOMAXPROCS(0), 4)
	errs := make([]error, workers)
	errAt := make([]int, workers)
	var best atomic.Int64
	best.Store(int64(total))
	var stopped atomic.Bool
	var group sync.WaitGroup
	group.Add(workers)
	for w := range workers {
		start := total * w / workers
		end := total * (w + 1) / workers
		go func() {
			defer group.Done()
			if beforeWorker != nil {
				beforeWorker()
			}
			axes := newRevolveAxisCache(len(data))
			for k := start; k < end; k++ {
				if k%proofbound.WorkPollInterval == 0 && stopped.Load() {
					return
				}
				if int64(k) > best.Load() {
					return
				}
				i, j := int(is[k]), int(js[k])
				shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
				if count == 0 && BoxGapExceeds(data[i].Box, data[j].Box, margin) {
					continue
				}
				if err := auditRevolvePairCached(data, tris, shared, count, i, j, delta, axes); err != nil {
					errs[w], errAt[w] = err, k
					for {
						cur := best.Load()
						if int64(k) >= cur || best.CompareAndSwap(cur, int64(k)) {
							break
						}
					}
					return
				}
			}
		}()
	}
	done := make(chan struct{})
	go func() {
		group.Wait()
		close(done)
	}()
	for running := true; running; {
		select {
		case <-done:
			running = false
		case <-poll:
			if err := budget.Err(); err != nil {
				stopped.Store(true)
				<-done
				return revolveAuditRows(budget, data, tris, rows, delta, margin, false)
			}
		}
	}
	first, firstErr := total, error(nil)
	for w, err := range errs {
		if err != nil && errAt[w] < first {
			first, firstErr = errAt[w], err
		}
	}
	for k := 0; k < total && k <= first; k++ {
		if err := budget.Step(); err != nil {
			return err
		}
	}
	if firstErr != nil {
		return firstErr
	}
	return budget.Err()
}

// revolveContactAuditAllPairs is the plain all-pairs walk, kept for a margin
// the sweep cannot order by (negative or NaN).
func revolveContactAuditAllPairs(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, delta, margin float64) error {
	f := len(tris)
	for i := range f {
		for j := i + 1; j < f; j++ {
			if err := budget.Step(); err != nil {
				return err
			}
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if count == 0 && BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			if err := AuditRevolvePair(data, tris, shared, count, i, j, delta); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}

// ContactCandidates is revolveContactCandidates for an audit that walks its
// own subset of the pairs, such as compositesweep's cross-span audit. margin
// must be a non-negative number (not NaN): the sweep orders by it.
func ContactCandidates(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, margin float64) ([][]int32, error) {
	return revolveContactCandidates(budget, data, tris, margin)
}

// revolveContactCandidates lists, per facet i, every j > i in ascending order
// that the audit may have to test: every pair the box gap test does not
// separate, and every pair that shares a vertex. Facets are swept in order of
// their box's low coordinate on the sweep axis; for a facet i, the facets after
// it in that order are read until the first whose low coordinate clears i's
// high coordinate by more than margin. The gap test is monotone in that low
// coordinate, so every later facet is cleared too, and a pair the sweep never
// lists is one BoxGapExceeds separates on that axis alone. The caller applies
// the same skip test the all-pairs walk applies, so it audits the same pairs
// in the same order.
func revolveContactCandidates(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, margin float64) ([][]int32, error) {
	axis := revolveSweepAxis(data)
	order := make([]int32, len(data))
	for i := range order {
		order[i] = int32(i)
	}
	slices.SortStableFunc(order, func(a, b int32) int {
		return cmp.Compare(boxCoord(data[a].Box[0], axis), boxCoord(data[b].Box[0], axis))
	})
	rows := make([][]int32, len(data))
	for p, i := range order {
		hi := boxCoord(data[i].Box[1], axis)
		for _, j := range order[p+1:] {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if freeform.DownRound(boxCoord(data[j].Box[0], axis)-hi) > margin {
				break
			}
			if BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			lo, up := min(i, j), max(i, j)
			rows[lo] = append(rows[lo], up)
		}
	}
	// Every pair that shares a vertex is listed regardless of its boxes.
	incident := make(map[int][]int32)
	for i, t := range tris {
		for _, v := range t {
			incident[v] = append(incident[v], int32(i))
		}
	}
	for i, t := range tris {
		for _, v := range t {
			for _, j := range incident[v] {
				if int(j) > i {
					rows[i] = append(rows[i], j)
				}
			}
		}
	}
	for i, row := range rows {
		slices.Sort(row)
		rows[i] = slices.Compact(row)
	}
	return rows, nil
}

func boxCoord(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// revolveSweepAxis picks the coordinate whose boxes overlap least: the one
// with the smallest summed box width per unit of total extent. Any axis lists
// every pair the gap test keeps; the choice only changes how many it lists.
func revolveSweepAxis(data []RevolveAuditTri) int {
	best, bestScore := 0, math.Inf(1)
	for axis := range 3 {
		lo, hi, width := math.Inf(1), math.Inf(-1), 0.0
		for _, t := range data {
			a, b := boxCoord(t.Box[0], axis), boxCoord(t.Box[1], axis)
			lo, hi, width = min(lo, a), max(hi, b), width+(b-a)
		}
		if extent := hi - lo; extent > 0 {
			if score := width / extent; score < bestScore {
				best, bestScore = axis, score
			}
		}
	}
	return best
}

// RequireRevolveFacetArea proves one facet keeps a positive area everywhere on
// the displaced family: its exact held area, bracketed from BELOW, exceeds the
// most a displacement of delta at each corner can take from it
// (agent-docs/tessellation-design.md §5's own per-triangle area allowance, read here
// as a gate rather than as a slack term).
func RequireRevolveFacetArea(t RevolveAuditTri, i int, delta float64) error {
	held := proofarith.DySqrtDown(proofarith.DvDot(t.N, t.N))
	allow := proofbound.ProductUpper(2, proofbound.AbsSumUpper(
		proofbound.ProductUpper(delta, proofbound.AbsSumUpper(t.Lu, t.Lv)),
		proofbound.ProductUpper(proofbound.ProductUpper(2, delta), delta),
	))
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(allow) || held <= allow {
		return fmt.Errorf(`%w: revolve facet %d does not keep a positive area under the coordinate displacement this mesh carries`, decaderr.ErrUnsupported, i)
	}
	return nil
}

// AuditRevolvePair decides one facet pair against the contact its shared vertex
// indices require, under RevolveContactAudit's own displacement margin.
//
// A pair sharing nothing is proven apart by a fixed separating axis. A pair
// sharing a vertex or an edge is REQUIRED to touch there, so what has to be
// proven instead is that it touches NOWHERE ELSE, and every proof of that below
// has the same shape: a half-space H whose boundary plane contains the shared
// feature, with one triangle inside H and the other's non-shared corners
// strictly outside. Then the intersection is contained in the boundary plane,
// and the strictly-outside triangle meets that plane in exactly the shared
// feature, so the two triangles meet in exactly it too.
//
// The boundary plane cannot be a FIXED one: the shared feature moves along the
// homotopy, and the plane has to keep containing it. So every candidate is
// built as a POLYNOMIAL in the pair's own corners — a triangle normal, a normal
// crossed with one of its own edges, or the shared edge's rejection of an
// apex — whose defining incidences hold identically rather than numerically.
// Only the strict-side readings are then charged a perturbation allowance.
func AuditRevolvePair(data []RevolveAuditTri, tris [][3]int, shared [3]int, count, i, j int, delta float64) error {
	switch count {
	case 3:
		return fmt.Errorf(`%w: revolve facets %d and %d are the same triangle`, decaderr.ErrUnsupported, i, j)
	case 0:
		if RevolveSeparated(data[i], data[j], delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d share no vertex, and this mesh cannot prove they stay apart under its own coordinate displacement`, decaderr.ErrUnsupported, i, j)
	case 1:
		if RevolveVertexIsolated(data[i], tris[i], data[j], tris[j], shared[0], delta) ||
			RevolveVertexIsolated(data[j], tris[j], data[i], tris[i], shared[0], delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only at the vertex they share`, decaderr.ErrUnsupported, i, j)
	default:
		if RevolveEdgeIsolated(data[i], tris[i], data[j], tris[j], shared, delta) ||
			RevolveEdgeIsolated(data[j], tris[j], data[i], tris[i], shared, delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only along the edge they share`, decaderr.ErrUnsupported, i, j)
	}
}

// RevolveSepAxis is one candidate boundary plane through the shared feature,
// carried as the plane's own (unnormalised) normal: the exact vector, a proven
// upper bound on its length, and a proven bound on how far that vector itself
// moves when every corner it was built from slides by up to the audit's margin.
// f encloses g in float intervals for the pre-test.
type RevolveSepAxis struct {
	G      proofarith.DyV3
	F      RevIvVec
	Length float64
	Drift  float64
}

// sideOf reads which side of the candidate plane an offset lies on, and whether
// that reading survives the whole displaced family. The plane passes through the
// shared feature, so the offset is measured from a shared corner. The float
// pre-test reads the same dot product first (RevIvSide) and answers for the
// exact reading wherever its interval settles it.
func (ax RevolveSepAxis) SideOf(o RevolveOffset, offsetDrift float64) (int, bool) {
	allow := PerturbBilinearAllow(ax.Length, o.Length, ax.Drift, offsetDrift)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	if side, ok := RevIvSide(RevIvDot(ax.F, o.F), allow); ok {
		return side, side != 0
	}
	return ax.SideOfExact(o.V, o.Length, offsetDrift)
}

// SideOfExact is sideOf over the exact Dyadic dot product alone.
func (ax RevolveSepAxis) SideOfExact(offset proofarith.DyV3, offsetLen, offsetDrift float64) (int, bool) {
	h := proofarith.DvDot(ax.G, offset)
	if h.IsZero() {
		return 0, false
	}
	allow := PerturbBilinearAllow(ax.Length, offsetLen, ax.Drift, offsetDrift)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	bound, ok := proofarith.DyOf(allow)
	if !ok || proofarith.DyCmp(proofarith.DyAbs(h), bound) <= 0 {
		return 0, false
	}
	return h.Sign(), true
}

// PerturbBilinearAllow bounds |a'∘b' − a∘b| for a dot or cross product when a
// and b slide by at most da and db: the two first-order terms plus the second.
func PerturbBilinearAllow(la, lb, da, db float64) float64 {
	return proofbound.AbsSumUpper(proofbound.ProductUpper(la, db), proofbound.ProductUpper(lb, da), proofbound.ProductUpper(da, db))
}

// RevolveNormalAxis is the candidate whose plane IS a triangle's own plane:
// every corner of that triangle reads exactly zero on it, identically, for the
// whole family.
func RevolveNormalAxis(t RevolveAuditTri, delta float64) RevolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	return RevolveSepAxis{
		G:      t.N,
		F:      t.Fn,
		Length: proofbound.ProductUpper(t.Lu, t.Lv),
		Drift:  PerturbBilinearAllow(t.Lu, t.Lv, e, e),
	}
}

// RevolveEdgeFanAxis is the candidate whose plane contains a triangle's own
// plane normal and one of its edges through the shared corner. The edge itself
// reads exactly zero (a determinant with a repeated vector), so that triangle's
// only reading to charge is its remaining corner.
func RevolveEdgeFanAxis(t RevolveAuditTri, edge RevolveOffset, delta float64) RevolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	normal := RevolveNormalAxis(t, delta)
	return RevolveSepAxis{
		G:      proofarith.DvCross(normal.G, edge.V),
		F:      RevIvCross(normal.F, edge.F),
		Length: proofbound.ProductUpper(normal.Length, edge.Length),
		Drift:  PerturbBilinearAllow(normal.Length, edge.Length, normal.Drift, e),
	}
}

// RevolveRejectionAxis is the candidate for a SHARED EDGE: the rejection of one
// triangle's apex off that edge, (d×u)×d = |d|²u − (d·u)d. Both shared corners
// read exactly zero on it — the first by construction, the second because the
// scalar triple product repeats d — and the apex reads the Gram determinant
// |d|²|u|² − (d·u)², which Cauchy-Schwarz makes non-negative identically. So
// that whole triangle sits in the closed half-space for the entire family with
// nothing to charge, and only the other triangle's apex is read.
func RevolveRejectionAxis(d, u RevolveOffset, delta float64) RevolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	dLen, uLen := d.Length, u.Length
	cross := proofarith.DvCross(d.V, u.V)
	crossLen := proofbound.ProductUpper(dLen, uLen)
	crossDrift := PerturbBilinearAllow(dLen, uLen, e, e)
	return RevolveSepAxis{
		G:      proofarith.DvCross(cross, d.V),
		F:      RevIvCross(RevIvCross(d.F, u.F), d.F),
		Length: proofbound.ProductUpper(crossLen, dLen),
		Drift:  PerturbBilinearAllow(crossLen, dLen, crossDrift, e),
	}
}

// RevolveVertexIsolated proves the pair meets only at the vertex they share,
// with the boundary plane built from triangle a. Four candidates are tried, and
// every one of them holds a identically inside its own closed half-space:
//
//   - a's own plane, on which all three of its corners read an identical zero;
//   - a's plane rotated onto either of its two edges at the shared corner, on
//     which that edge reads zero identically (a determinant repeating a vector)
//     and only a's remaining corner has to be signed;
//   - a's plane rotated onto the CHORD between its two other corners, on which
//     those two corners read the SAME value identically — their difference is a
//     determinant repeating the chord — so a still sits on one side. This is the
//     candidate that answers a pair of exactly opposite sectors of one pole fan,
//     where each triangle's own edge rays run straight into the other's.
//
// A fifth family is not built from a's plane at all: the EDGE-PAIR planes,
// whose normal g = eA × eB takes one edge of EACH triangle at the shared
// corner. Both of those edges read an identical zero on g — a determinant
// repeating a vector, twice over — so the plane contains one whole edge of a
// and one whole edge of b for every member of the displaced family, and only
// the two remaining corners have to be signed. It is the family that answers a
// partial cap's fan triangle against the wall triangle of the NEXT meridian
// chord, a pair no plane through a's own normal decides: a's normal reads the
// wall's in-plane corner at a numerical zero it cannot sign, and every rotation
// of it reads the wall's two corners with opposite signs, because the off-plane
// corner's in-plane component is fixed in sign and only shrinks as dφ² under
// refinement. Without this family that pair is undecidable at every angular
// count, so a partial-sweep revolve carrying a chorded arc refuses however fine
// the chording.
//
// The mirror, with the roles swapped, is the caller's second call.
func RevolveVertexIsolated(a RevolveAuditTri, triA [3]int, b RevolveAuditTri, triB [3]int, shared int, delta float64) bool {
	return revolveVertexIsolated(a, triA, b, triB, shared, delta, nil, 0)
}

// revolveVertexIsolated is RevolveVertexIsolated reading a's candidate axes
// from cache entry ia when cache is non-nil. Every cached axis is the value
// the uncached path builds from a, the shared corner and delta alone.
func revolveVertexIsolated(a RevolveAuditTri, triA [3]int, b RevolveAuditTri, triB [3]int, shared int, delta float64,
	cache *revolveAxisCache, ia int) bool {
	e := proofbound.ProductUpper(2, delta)
	ai := TriangleVertexSlot(triA, shared)
	bi := TriangleVertexSlot(triB, shared)
	if ai < 0 || bi < 0 {
		return false
	}
	aOff := RevolveCornerOffsets(a, ai)
	bOff := RevolveCornerOffsets(b, bi)

	// signed reads the common sign of the listed offsets, or reports that this
	// candidate cannot sign them all.
	signed := func(ax RevolveSepAxis, offs []RevolveOffset) (int, bool) {
		side := 0
		for _, o := range offs {
			s, ok := ax.SideOf(o, e)
			if !ok {
				return 0, false
			}
			if side == 0 {
				side = s
			} else if side != s {
				return 0, false
			}
		}
		return side, true
	}
	try := func(ax RevolveSepAxis, check []RevolveOffset) bool {
		side, ok := signed(ax, check)
		if !ok {
			return false
		}
		want, ok := signed(ax, bOff[:])
		if !ok || want == 0 {
			return false
		}
		return side == 0 || side != want
	}
	var corner *revolveCornerAxes
	if cache != nil {
		corner = cache.corner(ia, a, ai, delta)
	}
	normal := func() RevolveSepAxis {
		if cache != nil {
			return cache.normalAxis(ia, a, delta)
		}
		return RevolveNormalAxis(a, delta)
	}
	if try(normal(), nil) {
		return true
	}
	for k := range 2 {
		var ax RevolveSepAxis
		if corner != nil {
			ax = corner.fan[k]
		} else {
			ax = RevolveEdgeFanAxis(a, aOff[k], delta)
		}
		if try(ax, []RevolveOffset{aOff[1-k]}) {
			return true
		}
	}
	var chordAxis RevolveSepAxis
	if corner != nil {
		chordAxis = corner.chord
	} else {
		chord := RevolveOffsetOf(proofarith.DvSub(aOff[0].V, aOff[1].V), RevIvSubVec(aOff[0].F, aOff[1].F))
		chordAxis = RevolveEdgeFanAxis(a, chord, delta)
	}
	if try(chordAxis, aOff[:]) {
		return true
	}
	// The edge-pair family. g = eA × eB zeroes both eA and eB identically, so
	// the plane holds one edge of each triangle for the WHOLE family rather
	// than at the stored coordinates alone. That leaves a with two corners on
	// the plane and one strictly off it, and b likewise, so a sits in one
	// closed half-space and b in the other and their intersection lies in the
	// plane. A triangle with two corners on a plane and its third strictly off
	// meets that plane in exactly the closed segment between the two, so the
	// intersection is contained in eA ∩ eB — two segments from the shared
	// corner that a non-zero g makes non-parallel, and which therefore meet
	// only at that corner.
	//
	// g stays non-zero over the whole family without a separate test: a member
	// whose g vanished would read zero on both remaining corners, and sideOf
	// has already proven each of them strictly outside its own perturbation
	// allowance, which bounds exactly how far that reading can move. length
	// bounds |eA × eB| by the product of the two factor lengths and drift is
	// PerturbBilinearAllow over the two factors sliding by e, so the charge is
	// the one every other candidate here makes. A reading inside its allowance
	// stays UNDECIDED and the candidate is skipped, never admitted.
	for k := range 2 {
		for m := range 2 {
			g := proofarith.DvCross(aOff[k].V, bOff[m].V)
			if proofarith.DvIsZero(g) {
				continue
			}
			ax := RevolveSepAxis{
				G:      g,
				F:      RevIvCross(aOff[k].F, bOff[m].F),
				Length: proofbound.ProductUpper(aOff[k].Length, bOff[m].Length),
				Drift:  PerturbBilinearAllow(aOff[k].Length, bOff[m].Length, e, e),
			}
			sa, okA := ax.SideOf(aOff[1-k], e)
			sb, okB := ax.SideOf(bOff[1-m], e)
			if okA && okB && sa != sb {
				return true
			}
		}
	}
	return false
}

// RevolveOffset is one corner offset from the pair's shared corner, beside the
// proven upper bound on its length every perturbation allowance reads. f
// encloses v in float intervals for the pre-test.
type RevolveOffset struct {
	V      proofarith.DyV3
	F      RevIvVec
	Length float64
}

func RevolveOffsetOf(v proofarith.DyV3, f RevIvVec) RevolveOffset {
	return RevolveOffset{V: v, F: f, Length: proofbound.DvLenUpper(v)}
}

// RevolveCornerOffsets is a triangle's two corners other than the one at slot
// at, measured from that one.
func RevolveCornerOffsets(t RevolveAuditTri, at int) [2]RevolveOffset {
	return t.Off[at]
}

// RevolveEdgeIsolated proves the pair meets only along the edge they share,
// with the boundary plane built from triangle a: either a's own plane, or the
// rejection of a's apex off the shared edge — the candidate that answers the
// COPLANAR case every planar cell and every cap triangulation produces, where
// a's own plane says nothing at all.
func RevolveEdgeIsolated(a RevolveAuditTri, triA [3]int, b RevolveAuditTri, triB [3]int, shared [3]int, delta float64) bool {
	e := proofbound.ProductUpper(2, delta)
	p0 := TriangleVertexSlot(triA, shared[0])
	p1 := TriangleVertexSlot(triA, shared[1])
	apexA := tessellation.TriangleApexIndex(triA, shared[0], shared[1])
	apexB := tessellation.TriangleApexIndex(triB, shared[0], shared[1])
	if p0 < 0 || p1 < 0 || apexA < 0 || apexB < 0 {
		return false
	}
	bApex := TriangleVertexSlot(triB, apexB)
	aApex := TriangleVertexSlot(triA, apexA)
	if bApex < 0 || aApex < 0 {
		return false
	}
	offB := RevolveOffsetOf(proofarith.DvSub(b.P[bApex], a.P[p0]), RevIvPointDiff(b.Fp[bApex], a.Fp[p0]))
	if _, ok := RevolveNormalAxis(a, delta).SideOf(offB, e); ok {
		return true
	}
	d := RevolveOffsetOf(proofarith.DvSub(a.P[p1], a.P[p0]), RevIvPointDiff(a.Fp[p1], a.Fp[p0]))
	u := RevolveOffsetOf(proofarith.DvSub(a.P[aApex], a.P[p0]), RevIvPointDiff(a.Fp[aApex], a.Fp[p0]))
	ax := RevolveRejectionAxis(d, u, delta)
	side, ok := ax.SideOf(offB, e)
	// a itself lies in the g ≥ 0 half-space identically (the Gram determinant),
	// so the pair is isolated exactly when b's apex reads strictly negative.
	return ok && side < 0
}

// TriangleVertexSlot is the corner index a triangle carries a given vertex at,
// or −1 when it carries none.
func TriangleVertexSlot(tri [3]int, v int) int {
	for k, x := range tri {
		if x == v {
			return k
		}
	}
	return -1
}
