package meshbool

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// ErrUnclassifiableContact signals a VALID-but-unclassifiable contact the exact
// predicates refuse to classify: coplanar face-on-face overlap, an edge grazing
// along the other operand's facet plane, and intersection curves branching at a
// point. A tangent contact admits no side, so no facet classification is proven
// — the operation is rejected, never a wrong mesh. The input names a real solid
// and the refusal is the evaluator's reach, so it carries the BooleanExpectedContact
// signal and wraps ErrUnsupported (never ErrDegenerate); the public boundary remaps
// it to BooleanUnsupportedContact (boolean.go, asBooleanError; docs/api-design.md
// §8 / H2, docs/evaluator-design.md §9).
func ErrUnclassifiableContact(what string) error {
	return ExpectedBoolean(BooleanExpectedContact,
		fmt.Errorf(`%w: %s — the exact predicates cannot classify a tangent contact`, decaderr.ErrUnsupported, what))
}

// ContactKind is the DIMENSION of the exact intersection of two closed
// triangles. Two closed triangles are convex sets, so their intersection is
// convex and can only be one of these four — and naming it is what makes the
// classification direction-free: nothing in it depends on whose geometry the
// answer was looked for on.
type ContactKind int

const (
	// ContactNone: the closed triangles are disjoint.
	ContactNone ContactKind = iota
	// ContactPoint: they meet at exactly one point.
	ContactPoint
	// ContactSegment: they meet along a positive-length segment.
	ContactSegment
	// ContactRegion: they meet in a 2-D region — only possible for a coplanar
	// pair, and a face-on-face tangency the predicates refuse to classify.
	ContactRegion
)

// TriContact is one exact triangle/triangle intersection, as computed by
// TriTriClassify.
type TriContact struct {
	Kind   ContactKind
	P0, P1 proof.Xpt
	// p0OnA … p1OnB record whether each endpoint lies on each triangle's own
	// closed BOUNDARY — decided by asking what the point is (OnTriBoundary),
	// never by which triangle's crossing list it happened to be drawn from.
	P0OnA, P1OnA, P0OnB, P1OnB bool
	// edgeA and edgeB name the facet edge a SEGMENT contact runs ALONG (−1
	// when it runs along none): an in-plane edge. Whether such an edge grazes
	// the other operand or genuinely crosses it is NOT a property of this pair
	// — it is a property of the edge's two adjacent facets — so the pair only
	// reports the fact and the mesh pass decides (docs/evaluator-design.md §9).
	EdgeA, EdgeB int
	// sin2 is the exact squared sine of the two facet planes' crossing angle,
	// nil for a coplanar or empty pair. A rim vertex this pair's segment ends
	// at is displaced by (δA + δB)/sin θ of THIS pair (RimBound).
	Sin2 *big.Rat
}

// TriTriClassify computes the exact intersection of two CLOSED triangles: the
// single symmetric entry point every contact question goes through. ta/tb are
// float corners (the adaptive orient filter reads them), xta/xtb their exact
// lifts, na/nb the exact normals.
//
// The pair's intersection is the overlap of two convex sets, so it is empty, a
// point, a segment or a 2-D region — and the routine decides WHICH without
// ever branching on "how many of A's vertices sit on B's plane, and whose
// geometry do I look on". For a non-coplanar pair the two planes meet in one
// line; each triangle's intersection with the OTHER's plane is an interval on
// that line, and the answer is the intervals' overlap. That is the whole rule.
func TriTriClassify(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt) (TriContact, error) {
	return TriTriClassifyCore(ta, tb, xta, xtb, na, nb, nil, nil, nil, nil, true, r3.Vec{}, r3.Vec{}, false)
}

// TriTriClassifyPrepared is TriTriClassify for prepared boolean operands.
// fna and fnb are the correctly rounded float values of na and nb, computed
// once per facet rather than once per candidate pair.
func TriTriClassifyPrepared(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, fna, fnb r3.Vec) (TriContact, error) {
	return TriTriClassifyCore(ta, tb, xta, xtb, na, nb, nil, nil, nil, nil, true, fna, fnb, true)
}

// useFilter says whether this call may take TriTriMissesFilter's early exit in
// the non-coplanar arm below. Every production caller passes true. Only the
// tests proving the filter changes no verdict pass false, and what they
// establish is that the two agree across their whole corpus.
//
// It is an argument rather than a package-level switch because the choice
// belongs to one call. A switch a test flipped would decide the classification
// of every other caller running at that moment, which is both a data race and a
// wrong answer, and it is why those tests could not run in parallel.
func TriTriClassifyWithProjections(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int, useFilter bool) (TriContact, error) {
	return TriTriClassifyCore(ta, tb, xta, xtb, na, nb, pa, pb, sa, sb, useFilter, r3.Vec{}, r3.Vec{}, false)
}

func TriTriClassifyCore(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int,
	useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (TriContact, error) {
	out := TriContact{EdgeA: -1, EdgeB: -1}
	var signsB, signsA [3]int
	if sb != nil {
		signsB = *sb
	} else {
		for i := range 3 {
			signsB[i] = proof.OrientSignPrepared(ta[0], ta[1], ta[2], tb[i], xta[0], xtb[i], na)
		}
	}
	if AllOneSide(signsB) {
		return out, nil
	}
	if sa != nil {
		signsA = *sa
	} else {
		for i := range 3 {
			signsA[i] = proof.OrientSignPrepared(tb[0], tb[1], tb[2], ta[i], xtb[0], xta[i], nb)
		}
	}
	if AllOneSide(signsA) {
		return out, nil
	}

	if CountZero(signsA) == 3 || CountZero(signsB) == 3 {
		// Coplanar. The intersection of two coplanar closed triangles is a
		// convex polygon: positive area — or a positive-length shared boundary
		// — is a face-on-face tangency (§9 refuses it); anything else is at
		// most a point, carried for the isolated-point rule.
		if CoplanarFloatSeparated(ta, tb, na) {
			return out, nil
		}
		overlap := false
		if pa != nil && pb != nil {
			overlap = CoplanarOverlapProjected(*pa, *pb)
		} else {
			overlap = CoplanarOverlap(xta, xtb, na)
		}
		if overlap {
			out.Kind = ContactRegion
			return out, nil
		}
		if p, ok := CoplanarTouch(xta, xtb, na, nb); ok {
			out.Kind, out.P0, out.P1 = ContactPoint, p, p
		}
		return out, nil
	}

	// Non-coplanar: the planes are distinct and non-parallel (a parallel pair
	// would leave every vertex strictly on one side, already returned), so they
	// meet in exactly one line, and every point of the intersection lies on it.
	if useFilter {
		if !normalsPrepared {
			fna, fnb = na.Vec(), nb.Vec()
		}
		if TriTriMissesFilter(ta, tb, fna, fnb, signsA, signsB) {
			return out, nil
		}
	}
	ptsA := PlaneCrossings(xta, xtb, signsA)
	ptsB := PlaneCrossings(xtb, xta, signsB)
	if len(ptsA) == 0 || len(ptsB) == 0 {
		return out, nil
	}
	if len(ptsA) > 2 || len(ptsB) > 2 {
		return out, fmt.Errorf(`%w: a facet crosses a plane more than twice`, decaderr.ErrBooleanFailed)
	}
	dir := proof.Xcross(na, nb)
	loA, hiA := OrderOnLine(ptsA, dir)
	loB, hiB := OrderOnLine(ptsB, dir)
	lo, hi := loA, hiA
	if CmpOnLine(loB, lo, dir) > 0 {
		lo = loB
	}
	if CmpOnLine(hiB, hi, dir) < 0 {
		hi = hiB
	}
	switch CmpOnLine(lo, hi, dir) {
	case 1:
		return out, nil // the intervals miss: no contact at all
	case 0:
		out.Kind, out.P0, out.P1 = ContactPoint, lo, lo
	default:
		out.Kind, out.P0, out.P1 = ContactSegment, lo, hi
	}
	out.Sin2 = SinSquared(na, nb)
	out.P0OnA = OnTriBoundary(out.P0, xta, na)
	out.P0OnB = OnTriBoundary(out.P0, xtb, nb)
	if out.Kind == ContactSegment {
		out.P1OnA = OnTriBoundary(out.P1, xta, na)
		out.P1OnB = OnTriBoundary(out.P1, xtb, nb)
		out.EdgeA = SegAlongEdge(out.P0, out.P1, xta, na)
		out.EdgeB = SegAlongEdge(out.P0, out.P1, xtb, nb)
	} else {
		out.P1OnA, out.P1OnB = out.P0OnA, out.P0OnB
	}
	return out, nil
}

// AllOneSide reports whether all three plane-side signs are strictly the same:
// the triangle misses the plane entirely.
func AllOneSide(s [3]int) bool {
	return s[0] > 0 && s[1] > 0 && s[2] > 0 || s[0] < 0 && s[1] < 0 && s[2] < 0
}

// CoplanarFloatSeparated is a conservative separating-axis filter for
// coplanar triangles. It only rejects when every orientation sign is clear of
// the floating-point error margin; ambiguous pairs continue to exact clipping.
func CoplanarFloatSeparated(a, b [3]r3.Vec, n proof.Xpt) bool {
	u, v := ProjAxes(n)
	pa := [3][2]float64{}
	pb := [3][2]float64{}
	for i := range 3 {
		pa[i] = [2]float64{AxisCoord(a[i], u), AxisCoord(a[i], v)}
		pb[i] = [2]float64{AxisCoord(b[i], u), AxisCoord(b[i], v)}
	}
	for i := range 3 {
		if FloatEdgeSeparates(pa[i], pa[(i+1)%3], pa[(i+2)%3], pb) ||
			FloatEdgeSeparates(pb[i], pb[(i+1)%3], pb[(i+2)%3], pa) {
			return true
		}
	}
	return false
}

func FloatEdgeSeparates(a, b, inside [2]float64, tri [3][2]float64) bool {
	insideSign := Orient2Float(a, b, inside)
	if insideSign == 0 {
		return false
	}
	for _, p := range tri {
		if Orient2Float(a, b, p) != -insideSign {
			return false
		}
	}
	return true
}

func Orient2Float(a, b, c [2]float64) int {
	bx, by := b[0]-a[0], b[1]-a[1]
	cx, cy := c[0]-a[0], c[1]-a[1]
	det := bx*cy - by*cx
	perm := math.Abs(bx)*math.Abs(cy) + math.Abs(by)*math.Abs(cx)
	err := 1e-12 * perm
	if det > err {
		return 1
	}
	if det < -err {
		return -1
	}
	return 0
}

func AxisCoord(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// CoplanarTouch returns the single point two coplanar, non-overlapping closed
// triangles share, looking BOTH ways — the symmetric shape the whole classifier
// is built on.
func CoplanarTouch(xta, xtb [3]proof.Xpt, na, nb proof.Xpt) (proof.Xpt, bool) {
	for i := range 3 {
		if PointOnTri(xta[i], xtb, nb) {
			return xta[i], true
		}
	}
	for i := range 3 {
		if PointOnTri(xtb[i], xta, na) {
			return xtb[i], true
		}
	}
	return proof.Xpt{}, false
}

// OrderOnLine sorts a triangle's one or two plane crossings along the planes'
// common line, giving the interval the triangle occupies on it.
func OrderOnLine(pts []proof.Xpt, dir proof.Xpt) (proof.Xpt, proof.Xpt) {
	if len(pts) == 1 {
		return pts[0], pts[0]
	}
	if CmpOnLine(pts[0], pts[1], dir) > 0 {
		return pts[1], pts[0]
	}
	return pts[0], pts[1]
}

// CmpOnLine orders two points of the planes' common line along it: a·dir and
// b·dir share dir's own denominator, which cancels, so the two remaining
// positive denominators (a.w, b.w) are cross-multiplied rather than divided.
func CmpOnLine(a, b, dir proof.Xpt) int {
	left := new(big.Int).Mul(proof.XdotNum(a, dir), b.W)
	right := new(big.Int).Mul(proof.XdotNum(b, dir), a.W)
	return left.Cmp(right)
}

// OnTriBoundary reports whether the exact point p — already on the triangle's
// plane — lies on one of its three CLOSED edges. The projection is invertible
// on the plane, so the 2-D answer IS the 3-D one.
func OnTriBoundary(p proof.Xpt, xt [3]proof.Xpt, n proof.Xpt) bool {
	for i := range 3 {
		a, b := xt[i], xt[(i+1)%3]
		if PlaneSide(a, b, p, n) == 0 && PointOnSegment3D(a, b, p) {
			return true
		}
	}
	return false
}

// SegAlongEdge names the triangle edge a positive-length contact segment runs
// along, or −1. Such a segment lies in the other facet's plane, so the edge it
// runs along has both its endpoints on that plane — the in-plane edge whose
// graze-or-crossing verdict only the edge's two adjacent facets can give.
func SegAlongEdge(p0, p1 proof.Xpt, xt [3]proof.Xpt, n proof.Xpt) int {
	for i := range 3 {
		a, b := xt[i], xt[(i+1)%3]
		if PlaneSide(a, b, p0, n) != 0 || !PointOnSegment3D(a, b, p0) {
			continue
		}
		if PlaneSide(a, b, p1, n) == 0 && PointOnSegment3D(a, b, p1) {
			return i
		}
	}
	return -1
}

// SinSquared is the exact sin²θ of the angle between two facet planes:
// |na × nb|² / (|na|²·|nb|²).
func SinSquared(na, nb proof.Xpt) *big.Rat {
	c := proof.Xcross(na, nb)
	num := proof.XdotRat(c, c)
	den := new(big.Rat).Mul(proof.XdotRat(na, na), proof.XdotRat(nb, nb))
	if den.Sign() == 0 {
		return new(big.Rat)
	}
	return num.Quo(num, den)
}

// SinLowerBound is a PROVEN lower bound on sin θ from its exact square: the
// float square root, nudged down twice so neither the rational's rounding nor
// the root's can push it above the truth. A lower bound on the sine is what
// makes (δA + δB)/sin θ an UPPER bound on the rim displacement.
func SinLowerBound(sin2 *big.Rat) float64 {
	f, _ := sin2.Float64()
	if f <= 0 {
		return 0
	}
	s := math.Sqrt(f)
	s = math.Nextafter(s, 0)
	return math.Nextafter(s, 0)
}

// PlaneCrossings collects the exact points where triangle t crosses the
// other triangle's plane, given the per-vertex signs and the exact
// plane-side values (computed lazily, only for crossing edges) — as integer
// numerator/denominator pairs, never as a materialised big.Rat: the crossing
// parameter t = vi/(vi − vj) is formed directly as tn/td from the two orient
// values' own numerators and positive denominators (vi = ni/di, vj = nj/dj
// gives t = ni·dj / (ni·dj − nj·di)) and handed straight to xlerp, which
// renormalises td's sign itself. A nondegenerate triangle contributes at most
// two distinct points: its on-plane vertices, or crossings on edges whose
// endpoints have strictly opposite signs. No canonical point-key pass is
// needed to deduplicate this list.
func PlaneCrossings(xt [3]proof.Xpt, xo [3]proof.Xpt, signs [3]int) []proof.Xpt {
	var out []proof.Xpt
	nums := [3]*big.Int{}
	dens := [3]*big.Int{}
	val := func(i int) (*big.Int, *big.Int) {
		if nums[i] == nil {
			nums[i], dens[i] = proof.OrientNum(xo[0], xo[1], xo[2], xt[i])
		}
		return nums[i], dens[i]
	}
	for i := range 3 {
		if signs[i] == 0 {
			out = append(out, xt[i])
		}
		j := (i + 1) % 3
		if signs[i]*signs[j] < 0 {
			// t = vi/(vi − vj) is in (0, 1): the edge crosses the plane.
			ni, di := val(i)
			nj, dj := val(j)
			tn := new(big.Int).Mul(ni, dj)
			td := new(big.Int).Sub(tn, new(big.Int).Mul(nj, di))
			out = append(out, proof.Xlerp(xt[i], xt[j], tn, td))
		}
	}
	return out
}

func CountZero(s [3]int) int {
	n := 0
	for _, v := range s {
		if v == 0 {
			n++
		}
	}
	return n
}

// ProjAxes picks the two projection coordinates for a plane with exact
// normal n: the dominant-magnitude axis is dropped, so the projection is
// invertible on the plane. n's three coordinates share one positive
// denominator, so their magnitudes compare directly as integers — no
// cross-multiplication needed.
func ProjAxes(n proof.Xpt) (int, int) {
	ax := new(big.Int).Abs(n.X)
	ay := new(big.Int).Abs(n.Y)
	az := new(big.Int).Abs(n.Z)
	if az.Cmp(ax) >= 0 && az.Cmp(ay) >= 0 {
		return 0, 1
	}
	if ay.Cmp(ax) >= 0 {
		return 2, 0
	}
	return 1, 2
}

// PointOnTri reports whether the exact point p — already on the triangle's
// plane — lies inside or on the closed triangle, via exact side-of-edge signs.
func PointOnTri(p proof.Xpt, xt [3]proof.Xpt, n proof.Xpt) bool {
	s0 := PlaneSide(xt[0], xt[1], p, n)
	s1 := PlaneSide(xt[1], xt[2], p, n)
	s2 := PlaneSide(xt[2], xt[0], p, n)
	return (s0 >= 0 && s1 >= 0 && s2 >= 0) || (s0 <= 0 && s1 <= 0 && s2 <= 0)
}

func PlaneSide(a, b, p, n proof.Xpt) int {
	return proof.XdotSign(proof.Xcross(proof.Xsub(b, a), proof.Xsub(p, a)), n)
}

func PointOnSegment3D(a, b, p proof.Xpt) bool {
	delta := proof.Xsub(b, a)
	axis := 0
	if AbsInt(delta.Y).Cmp(AbsInt(delta.X)) > 0 {
		axis = 1
	}
	if AbsInt(delta.Z).Cmp(AbsInt(Coord(delta, axis))) > 0 {
		axis = 2
	}
	lo, hi := a, b
	if CompareCoord(lo, hi, axis) > 0 {
		lo, hi = hi, lo
	}
	return CompareCoord(lo, p, axis) <= 0 && CompareCoord(p, hi, axis) <= 0
}

func Coord(p proof.Xpt, axis int) *big.Int {
	switch axis {
	case 0:
		return p.X
	case 1:
		return p.Y
	default:
		return p.Z
	}
}

func AbsInt(v *big.Int) *big.Int {
	return new(big.Int).Abs(v)
}

func CompareCoord(a, b proof.Xpt, axis int) int {
	left := new(big.Int).Mul(Coord(a, axis), b.W)
	right := new(big.Int).Mul(Coord(b, axis), a.W)
	return left.Cmp(right)
}

// SegTriOverlap2 reports whether segment (a, b) meets the closed triangle
// with positive length — exactly.
func SegTriOverlap2(a, b, ta, tb, tc Xp2) bool {
	// Clip the segment parameter interval [0, 1] against each closed edge
	// half-plane of the triangle, in exact arithmetic; a positive-length
	// remainder is an overlap.
	//
	// A clip parameter is only ever compared here — against the running bounds
	// and, at the end, against each other — and never printed or combined into
	// a further value. So the bounds stay UNNORMALISED fractions and compare by
	// cross-multiplication (ClipFrac): every comparison is exact, and none of
	// them pays the GCD that reducing a big.Rat costs.
	ccw := Cross2xSign(ta, tb, tc)
	if ccw == 0 {
		return false
	}
	edges := [3][2]Xp2{{ta, tb}, {tb, tc}, {tc, ta}}
	lo := ClipFrac{Num: big.NewInt(0), Den: big.NewInt(1)}
	hi := ClipFrac{Num: big.NewInt(1), Den: big.NewInt(1)}
	for _, e := range edges {
		// Signs decide the two no-crossing cases without constructing rational
		// values. Only an edge crossing the segment needs its exact clip point.
		sa := Cross2xSign(e[0], e[1], a)
		sb := Cross2xSign(e[0], e[1], b)
		if ccw < 0 {
			sa = -sa
			sb = -sb
		}
		// f(t) = fa + t·(fb − fa) must stay ≥ 0.
		switch {
		case sa >= 0 && sb >= 0:
			continue
		case sa < 0 && sb < 0:
			return false
		}
		// Exactly one of the two signs is negative here, so fa ≠ fb and the
		// clip point t = −fa/(fb − fa) is defined. Writing fa = na/wa and
		// fb = nb/wb turns it into
		//
		//	t = −na·wb / (nb·wa − na·wb)
		//
		// whose denominator is nonzero for the same reason, and which needs no
		// division at all. The ccw < 0 negation the sign test above applies is
		// deliberately NOT applied to fa and fb: t does not move when both are
		// multiplied by one nonzero constant, and −1 is such a constant.
		fa, fb := EdgeCross2Fracs(e[0], e[1], a, b)
		naWb := new(big.Int).Mul(fa.Num, fb.Den)
		t := ClipFrac{
			Num: new(big.Int).Neg(naWb),
			Den: new(big.Int).Sub(new(big.Int).Mul(fb.Num, fa.Den), naWb),
		}
		if t.Den.Sign() < 0 {
			// CmpClipFrac's cross-multiplication keeps its direction only
			// while both denominators are positive, so canonicalise the sign
			// onto the numerator before this t is ever compared.
			t.Num.Neg(t.Num)
			t.Den.Neg(t.Den)
		}
		if sa < 0 {
			if CmpClipFrac(t, lo) > 0 {
				lo = t
			}
			continue
		}
		if CmpClipFrac(t, hi) < 0 {
			hi = t
		}
	}
	return CmpClipFrac(lo, hi) < 0
}

// CoplanarOverlap reports whether two coplanar triangles share positive
// area or a positive-length boundary segment — exactly.
func CoplanarOverlap(xta, xtb [3]proof.Xpt, n proof.Xpt) bool {
	u, v := ProjAxes(n)
	var a2, b2 [3]Xp2
	for i := range 3 {
		a2[i] = NewXP2(RatCoordOf(xta[i], u), RatCoordOf(xta[i], v))
		b2[i] = NewXP2(RatCoordOf(xtb[i], u), RatCoordOf(xtb[i], v))
	}
	return CoplanarOverlapProjected(a2, b2)
}

func CoplanarOverlapProjected(a2, b2 [3]Xp2) bool {
	// Any edge of one meeting the closed other with positive length is an
	// overlap; this covers containment (all edges inside), proper crossings,
	// and collinear boundary contact alike.
	for i := range 3 {
		if SegTriOverlap2(a2[i], a2[(i+1)%3], b2[0], b2[1], b2[2]) {
			return true
		}
		if SegTriOverlap2(b2[i], b2[(i+1)%3], a2[0], a2[1], a2[2]) {
			return true
		}
	}
	return false
}
