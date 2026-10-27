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
// it to BooleanUnsupportedContact (boolean.go, asBooleanError; agent-docs/api-design.md
// §8 / H2, agent-docs/evaluator-design.md §9).
func ErrUnclassifiableContact(what string) error {
	return ExpectedBoolean(BooleanExpectedContact,
		fmt.Errorf(`%w: %s — the exact predicates cannot classify a tangent contact`, decaderr.ErrUnsupported, what))
}

// TouchContact is one contact the exact predicates cannot classify: the first
// operand's outward unit normal at the contact, and whether the way the second
// operand must move to resolve it matters to the result. A coplanar overlap is
// Decisive when the second operand's facet faces the opposite way, which puts
// the operands on opposite sides of the shared plane (a face-on-face touch)
// rather than flush on one side. An edge grazing along a facet and an isolated
// point touch are always Decisive.
type TouchContact struct {
	NormalA  r3.Vec
	Decisive bool
	// FacetsB names facets of the second operand's mesh that take part in the
	// contact, so a caller that moves each connected piece of the second operand
	// on its own knows which piece to move. Empty when the contact does not say.
	FacetsB []int
	// Slack is how deep the second operand must reach into the first along
	// NormalA for the pair to be decided: the sum of the two faces' chord
	// displacements for a contact the proximity gate refused, and zero for a
	// contact the exact predicates refused.
	Slack float64
}

// TouchContactError is ErrUnclassifiableContact's refusal for a coplanar
// overlap, an edge grazing along a facet or an isolated point touch, carrying
// the contacts it found so that a caller allowed to move an operand can choose
// the direction that opens them. It reads as the same refusal, and unwraps to
// ErrUnsupported.
type TouchContactError struct {
	// What names the contact as ErrUnclassifiableContact does.
	What string
	// Detail, when set, is the whole text of the refusal after its sentinel, for
	// a refusal that is not one of the exact predicates'.
	Detail   string
	Contacts []TouchContact
}

func (e *TouchContactError) Error() string {
	if e.Detail != "" {
		return fmt.Sprintf(`%v: %s`, decaderr.ErrUnsupported, e.Detail)
	}
	return fmt.Sprintf(`%v: %s — the exact predicates cannot classify a tangent contact`, decaderr.ErrUnsupported, e.What)
}

func (e *TouchContactError) Unwrap() error { return decaderr.ErrUnsupported }

// The contacts a TouchContactError names.
const (
	touchCoplanar = `two operand facets overlap in one plane`
	touchGraze    = `an operand edge grazes along the other operand's facet`
	touchPoint    = `the operand boundaries touch at an isolated point`
	touchNear     = `the operands' held facets come within the chord tolerance`
)

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
	// P0OnA … P1OnB record whether each endpoint lies on each triangle's own
	// closed boundary. Exact interval comparisons and plane-side signs decide
	// the same membership as OnTriBoundary (contactSpanBoundary).
	P0OnA, P1OnA, P0OnB, P1OnB bool
	// edgeA and edgeB name the facet edge a SEGMENT contact runs ALONG (−1
	// when it runs along none): an in-plane edge. Whether such an edge grazes
	// the other operand or genuinely crosses it is NOT a property of this pair
	// — it is a property of the edge's two adjacent facets — so the pair only
	// reports the fact and the mesh pass decides (agent-docs/evaluator-design.md §9).
	EdgeA, EdgeB int
	// sin2 is the exact squared sine of the two facet planes' crossing angle,
	// nil for a coplanar or empty pair. A rim vertex this pair's segment ends
	// at is displaced by (δA + δB)/sin θ of THIS pair (RimBound).
	Sin2 *SinSquare
}

// SinSquare is an exact sin²θ held as the fraction Num/Den, Den > 0, in any
// terms: the classifier never reduces it, because its one reader needs only
// the nearest float64, which depends on the value alone.
type SinSquare struct {
	Num, Den *big.Int
}

// Float64 is the nearest float64 to Num/Den, ties to even: the bits
// big.Rat.Float64 returns for the same value (proof.FracFloat64).
func (s *SinSquare) Float64() float64 { return proof.FracFloat64(s.Num, s.Den) }

// Rat is Num/Den in lowest terms.
func (s *SinSquare) Rat() *big.Rat { return new(big.Rat).SetFrac(s.Num, s.Den) }

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

// TriContactSpan is the kind and the end points of an exact
// triangle/triangle intersection, without the boundary, edge and angle
// records TriContact adds.
type TriContactSpan struct {
	Kind   ContactKind
	P0, P1 proof.Xpt
}

func TriTriClassifyCore(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int,
	useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (TriContact, error) {
	span, crossing, err := triTriSpan(ta, tb, xta, xtb, na, nb, pa, pb, sa, sb, useFilter, fna, fnb, normalsPrepared)
	out := TriContact{EdgeA: -1, EdgeB: -1}
	if err != nil {
		return out, err
	}
	out.Kind, out.P0, out.P1 = span.Kind, span.P0, span.P1
	if !crossing {
		return out, nil
	}
	out.Sin2 = SinSquared(na, nb)
	boundary := span.boundary
	out.P0OnA, out.P1OnA = boundary.p0A, boundary.p1A
	out.P0OnB, out.P1OnB = boundary.p0B, boundary.p1B
	out.EdgeA, out.EdgeB = boundary.edgeA, boundary.edgeB
	return out, nil
}

// TriTriContactSpanCore takes exactly the arguments of TriTriClassifyCore. It
// returns the Kind, P0, P1 and error TriTriClassifyCore returns for the same
// arguments, and skips the boundary, edge and angle records. Use it only where
// those records are never read.
func TriTriContactSpanCore(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int,
	useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (TriContactSpan, error) {
	span, _, err := triTriSpan(ta, tb, xta, xtb, na, nb, pa, pb, sa, sb, useFilter, fna, fnb, normalsPrepared)
	return span.TriContactSpan, err
}

// contactSpanBoundary records OnTriBoundary and SegAlongEdge's answers from
// the exact intersection intervals. For two distinct nonparallel facet planes,
// their common line meets each nondegenerate triangle in a closed interval.
// Unless the line contains a triangle edge, only that interval's endpoints
// lie on the triangle boundary: a line crossing the relative interior of an
// edge enters or leaves the convex triangle there. If it contains an edge,
// the whole interval is that edge, identified by its two zero plane-side signs.
//
// For a positive-length overlap, its lower end is on A's boundary exactly
// when loB <= loA, and on B's exactly when loB >= loA; the upper-end tests
// reverse those inequalities. Equality counts for both triangles, regardless
// of which point representation the overlap keeps. The in-plane-edge case
// marks both ends. For a point overlap, the two end records are combined,
// because the common point can be either end of either triangle's interval.
// Every comparison and zero sign is exact, so these are the same memberships
// the closed edge predicates decide, with no additional integer products.
type contactSpanBoundary struct {
	p0A, p1A, p0B, p1B bool
	edgeA, edgeB       int
}

type classifiedContactSpan struct {
	TriContactSpan
	boundary contactSpanBoundary
}

// triTriSpan is the classification shared by TriTriClassifyCore and
// TriTriContactSpanCore. crossing is true only when the non-coplanar branch
// produced a point or a segment.
func triTriSpan(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int,
	useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (classifiedContactSpan, bool, error) {
	var span classifiedContactSpan
	var signsB, signsA [3]int
	if sb != nil {
		signsB = *sb
	} else {
		for i := range 3 {
			signsB[i] = proof.OrientSignPrepared(ta[0], ta[1], ta[2], tb[i], xta[0], xtb[i], na)
		}
	}
	if AllOneSide(signsB) {
		return span, false, nil
	}
	if sa != nil {
		signsA = *sa
	} else {
		for i := range 3 {
			signsA[i] = proof.OrientSignPrepared(tb[0], tb[1], tb[2], ta[i], xtb[0], xta[i], nb)
		}
	}
	if AllOneSide(signsA) {
		return span, false, nil
	}

	if CountZero(signsA) == 3 || CountZero(signsB) == 3 {
		// Coplanar. The intersection of two coplanar closed triangles is a
		// convex polygon: positive area — or a positive-length shared boundary
		// — is a face-on-face tangency (§9 refuses it); anything else is at
		// most a point, carried for the isolated-point rule.
		if useFilter && CoplanarFloatSeparated(ta, tb, na) {
			return span, false, nil
		}
		overlap := false
		if pa != nil && pb != nil {
			overlap = CoplanarOverlapProjected(*pa, *pb)
		} else {
			overlap = CoplanarOverlap(xta, xtb, na)
		}
		if overlap {
			span.Kind = ContactRegion
			return span, false, nil
		}
		if p, ok := CoplanarTouch(xta, xtb, na, nb); ok {
			span.Kind, span.P0, span.P1 = ContactPoint, p, p
		}
		return span, false, nil
	}

	// Non-coplanar: the planes are distinct and non-parallel (a parallel pair
	// would leave every vertex strictly on one side, already returned), so they
	// meet in exactly one line, and every point of the intersection lies on it.
	if useFilter {
		if !normalsPrepared {
			fna, fnb = na.Vec(), nb.Vec()
		}
		if TriTriMissesFilter(ta, tb, fna, fnb, signsA, signsB) {
			return span, false, nil
		}
	}
	var ptsABuf, ptsBBuf [3]proof.Xpt
	ptsA := ptsABuf[:planeCrossingsPreparedInto(&ptsABuf, xta, xtb[0], nb, signsA)]
	ptsB := ptsBBuf[:planeCrossingsPreparedInto(&ptsBBuf, xtb, xta[0], na, signsB)]
	if len(ptsA) == 0 || len(ptsB) == 0 {
		return span, false, nil
	}
	if len(ptsA) > 2 || len(ptsB) > 2 {
		return classifiedContactSpan{}, false, fmt.Errorf(`%w: a facet crosses a plane more than twice`, decaderr.ErrBooleanFailed)
	}
	axis, reverse, hasLine := commonLineAxis(na, nb)
	var dir proof.Xpt
	if !hasLine {
		// The non-coplanar branch above guarantees a common line. Keep the
		// former exact dot ordering as a defensive fallback if that invariant
		// ever changes.
		dir = proof.Xcross(na, nb)
		axis = -1
	}
	order := lineOrder{axis: axis, reverse: reverse}
	loA, hiA := order.sorted(ptsA, dir)
	loB, hiB := order.sorted(ptsB, dir)
	lo, hi := loA, hiA
	loCmp, hiCmp := order.cmp(loB, loA), order.cmp(hiB, hiA)
	if loCmp > 0 {
		lo = loB
	}
	if hiCmp < 0 {
		hi = hiB
	}
	switch order.cmp(lo, hi) {
	case 1:
		return span, false, nil // the intervals miss: no contact at all
	case 0:
		span.Kind, span.P0, span.P1 = ContactPoint, lo.P, lo.P
	default:
		span.Kind, span.P0, span.P1 = ContactSegment, lo.P, hi.P
	}
	{
		boundary := &span.boundary
		edgeA, edgeB := inPlaneEdge(signsA), inPlaneEdge(signsB)
		boundary.p0A, boundary.p1A = edgeA >= 0 || loCmp <= 0, edgeA >= 0 || hiCmp >= 0
		boundary.p0B, boundary.p1B = edgeB >= 0 || loCmp >= 0, edgeB >= 0 || hiCmp <= 0
		boundary.edgeA, boundary.edgeB = -1, -1
		if span.Kind == ContactPoint {
			boundary.p0A = boundary.p0A || boundary.p1A
			boundary.p0B = boundary.p0B || boundary.p1B
			boundary.p1A, boundary.p1B = boundary.p0A, boundary.p0B
		} else {
			boundary.edgeA, boundary.edgeB = edgeA, edgeB
		}
	}
	return span, true, nil
}

// inPlaneEdge names the edge in the other facet's plane. signs must be exact
// plane-side signs of a non-coplanar triangle, so at most two can be zero.
func inPlaneEdge(signs [3]int) int {
	for i := range 3 {
		if signs[i] == 0 && signs[(i+1)%3] == 0 {
			return i
		}
	}
	return -1
}

// linePoint is a point of the planes' common line together with D =
// XdotNum(P, dir), the numerator CmpOnLine reads for it.
type linePoint struct {
	P proof.Xpt
	D *big.Int
}

// lineOrder orders linePoints exactly as OrderOnLine and CmpOnLine order
// their points: it compares the same two cross-multiplied integers, reading
// each point's dot numerator from the linePoint instead of recomputing it for
// every comparison. Its two integers are scratch for one goroutine.
type lineOrder struct {
	left, right big.Int
	axis        int
	reverse     bool
}

func (o *lineOrder) cmp(a, b linePoint) int {
	if o.axis >= 0 {
		o.left.Mul(Coord(a.P, o.axis), b.P.W)
		o.right.Mul(Coord(b.P, o.axis), a.P.W)
		cmp := o.left.Cmp(&o.right)
		if o.reverse {
			return -cmp
		}
		return cmp
	}
	o.left.Mul(a.D, b.P.W)
	o.right.Mul(b.D, a.P.W)
	return o.left.Cmp(&o.right)
}

// sorted is OrderOnLine over linePoints.
func (o *lineOrder) sorted(pts []proof.Xpt, dir proof.Xpt) (linePoint, linePoint) {
	first := linePoint{P: pts[0]}
	if o.axis < 0 {
		first.D = proof.XdotNum(pts[0], dir)
	}
	if len(pts) == 1 {
		return first, first
	}
	second := linePoint{P: pts[1]}
	if o.axis < 0 {
		second.D = proof.XdotNum(pts[1], dir)
	}
	if o.cmp(first, second) > 0 {
		return second, first
	}
	return first, second
}

// commonLineAxis selects a coordinate that changes along the intersection of
// the planes with normals a and b. On that line, the coordinate orders points
// exactly as their dot product with a×b, accounting for the component's sign.
func commonLineAxis(a, b proof.Xpt) (axis int, reverse, ok bool) {
	var left, right big.Int
	left.Mul(a.Y, b.Z)
	right.Mul(a.Z, b.Y)
	left.Sub(&left, &right)
	if sign := left.Sign(); sign != 0 {
		return 0, sign < 0, true
	}
	left.Mul(a.Z, b.X)
	right.Mul(a.X, b.Z)
	left.Sub(&left, &right)
	if sign := left.Sign(); sign != 0 {
		return 1, sign < 0, true
	}
	left.Mul(a.X, b.Y)
	right.Mul(a.Y, b.X)
	left.Sub(&left, &right)
	if sign := left.Sign(); sign != 0 {
		return 2, sign < 0, true
	}
	return -1, false, false
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
	// Each admitted nonzero rounded difference lies in [2^-300, 2^250]
	// and on the 2^-352 binary64 grid. Products and their difference stay
	// on the 2^-704 grid, so even cancellation cannot round a nonzero
	// result through underflow. The maximum permanent is below 2^502.
	// Each determinant product charges its two input subtraction roundings
	// and its multiplication rounding; the final subtraction adds one more.
	// With u=2^-53, gamma4=4u/(1-4u) bounds the determinant error by
	// gamma4 times the exact permanent. The rounded permanent bounds that
	// permanent from below by (1-u)^4. If nonzero, it is at least 2^-600,
	// so 1e-12 times it stays normal and exceeds the error bound. A zero
	// rounded difference means equal finite operands, hence exact zero.
	for _, v := range [...]float64{bx, by, cx, cy} {
		m := proof.Abs(v)
		if !(v == 0 || m >= 0x1p-300 && m <= 0x1p250) {
			return 0
		}
	}
	det := bx*cy - by*cx
	perm := proof.Abs(bx)*proof.Abs(cy) + proof.Abs(by)*proof.Abs(cx)
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
func SinSquared(na, nb proof.Xpt) *SinSquare {
	// Both homogeneous weights cancel from the ratio, so build one fraction
	// from integer dot products instead of normalizing each intermediate.
	c := proof.XhpCross(proof.Xhp(na), proof.Xhp(nb))
	num := proof.XhpDotNum(c, c)
	den := new(big.Int).Mul(proof.XdotNum(na, na), proof.XdotNum(nb, nb))
	if den.Sign() == 0 {
		return &SinSquare{Num: new(big.Int), Den: big.NewInt(1)}
	}
	return &SinSquare{Num: num, Den: den}
}

// SinLowerBound is a PROVEN lower bound on sin θ from its exact square: the
// float square root, nudged down twice so neither the rational's rounding nor
// the root's can push it above the truth. A lower bound on the sine is what
// makes (δA + δB)/sin θ an UPPER bound on the rim displacement.
func SinLowerBound(sin2 *SinSquare) float64 {
	f := sin2.Float64()
	if f <= 0 {
		return 0
	}
	s := math.Sqrt(f)
	s = math.Nextafter(s, 0)
	return math.Nextafter(s, 0)
}

// PlaneCrossings collects the exact points where triangle t crosses the
// other triangle's plane, given the per-vertex signs and the exact
// plane-side values (computed lazily, only for crossing edges). A
// nondegenerate triangle contributes at most two distinct points: its on-plane
// vertices, or crossings on edges whose endpoints have strictly opposite
// signs. No canonical point-key pass is needed to deduplicate this list.
func PlaneCrossings(xt [3]proof.Xpt, xo [3]proof.Xpt, signs [3]int) []proof.Xpt {
	n := proof.Xcross(proof.Xsub(xo[1], xo[0]), proof.Xsub(xo[2], xo[0]))
	return planeCrossingsPrepared(xt, xo[0], n, signs)
}

// planeCrossingsPrepared reuses the other facet's normal. The positive normal
// and origin weights are common to all plane-side values and cancel from the
// edge crossing ratio.
func planeCrossingsPrepared(xt [3]proof.Xpt, origin, normal proof.Xpt, signs [3]int) []proof.Xpt {
	var out [3]proof.Xpt
	n := planeCrossingsPreparedInto(&out, xt, origin, normal, signs)
	return out[:n]
}

func planeCrossingsPreparedInto(out *[3]proof.Xpt, xt [3]proof.Xpt, origin, normal proof.Xpt, signs [3]int) int {
	n := 0
	var nums [3]big.Int
	var numsReady [3]bool
	var base, scaledBase big.Int
	baseReady := false
	val := func(i int) *big.Int {
		if !numsReady[i] {
			if !baseReady {
				proof.XdotNumScratch(&base, normal, origin)
				baseReady = true
			}
			point := &nums[i]
			proof.XdotNumScratch(point, normal, xt[i])
			if xt[i].W.Cmp(origin.W) == 0 {
				point.Sub(point, &base)
				point.Mul(point, xt[i].W)
			} else {
				point.Mul(point, origin.W)
				scaledBase.Mul(&base, xt[i].W)
				point.Sub(point, &scaledBase)
			}
			numsReady[i] = true
		}
		return &nums[i]
	}
	for i := range 3 {
		if signs[i] == 0 {
			out[n] = xt[i]
			n++
		}
		j := (i + 1) % 3
		if signs[i]*signs[j] < 0 {
			out[n] = proof.XplaneCrossing(xt[i], xt[j], val(i), val(j))
			n++
		}
	}
	return n
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

// PlaneSide is the sign of ((b-a) x (p-a)) . n. It returns 0 without exact
// work when p holds the same four coordinate pointers as a or as b: p-a is then
// the zero vector, or p-a equals b-a and their cross product is zero. This
// holds for any weights. The shortcut applies only when a, b and n have every
// coordinate set (p then has them too, sharing a or b), so a nil field still
// panics in the exact path.
func PlaneSide(a, b, p, n proof.Xpt) int {
	if xptComplete(a) && xptComplete(b) && xptComplete(n) &&
		(xptShared(p, a) || xptShared(p, b)) {
		return 0
	}
	return proof.XdotSign(proof.Xcross(proof.Xsub(b, a), proof.Xsub(p, a)), n)
}

// xptShared reports whether p and q hold the same four coordinate pointers.
func xptShared(p, q proof.Xpt) bool {
	return p.X == q.X && p.Y == q.Y && p.Z == q.Z && p.W == q.W
}

// xptComplete reports whether all four coordinates are set.
func xptComplete(p proof.Xpt) bool { return p.X != nil && p.Y != nil && p.Z != nil && p.W != nil }

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
