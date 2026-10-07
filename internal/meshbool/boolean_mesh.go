package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// BoolMesh is one operand's tessellation prepared for exact work: float
// vertices lifted to rationals, exact facet normals and their lazy rounded-
// float cache, facet boxes for pair pruning, and the global source-face id
// each facet remembers.
type BoolMesh struct {
	Verts       []r3.Vec
	Xverts      []proof.Xpt
	Tris        [][3]int
	Norms       []proof.Xpt
	Fnorms      []r3.Vec
	FnormsReady []bool
	Boxes       [][2]r3.Vec
	Src         []int
	// VertexBound is β(v) per vertex and FacetBound δ(t) per facet, the
	// largest of its three corners' β (docs/faceted-vertex-bounds-design.md
	// §2): a true boundary point lies within VertexBound[v] of vertex v, and
	// facet t's true piece within FacetBound[t] of the facet. The rim, the
	// cutter and the stitch compose every result vertex's own bound from them
	// (§3).
	VertexBound []float64
	FacetBound  []float64
	// Gate is the pair's chord tolerance when this operand restates a held
	// mesh (a boolean result or a mitred sweep), and zero for every other
	// operand. A gated operand's facets that meet the other operand, or come
	// within the pre-pass slack of it, must carry a FacetBound no larger than
	// Gate (docs/faceted-vertex-bounds-design.md §5, RefuseCoarseHeldContact).
	Gate float64
	// owner maps each directed mesh edge to the facet that walks it, so the
	// twin across a facet edge is one lookup. The graze-or-crossing call
	// needs it: whether an in-plane edge grazes the other operand or crosses
	// it is a property of the edge's TWO adjacent facets, which no facet pair
	// can see (docs/evaluator-design.md §9).
	Owner map[[2]int]int
	// parity caches this operand's vertex projections for the exact ray-parity
	// kernel, which every uncut-component seed, unanchored region probe and
	// near-miss depth witness of one operation asks about THIS operand. The
	// cache is empty until a query sweeps an axis, and it lives exactly as long
	// as the prepared operand does.
	Parity *ParityMesh
}

// twinFacet returns the facet across edge k of facet f — the neighbour a
// watertight mesh's reversed directed edge names.
func (bm *BoolMesh) TwinFacet(f, k int) (int, bool) {
	tri := bm.Tris[f]
	t, ok := bm.Owner[[2]int{tri[(k+1)%3], tri[k]}]
	return t, ok && t != f
}

// prepareFloatNormal computes the correctly rounded float value of one exact
// facet normal. RunContactBatch calls it before starting workers, so workers
// only read the cache; ContactMemo's serial path does the same before use.
func (bm *BoolMesh) PrepareFloatNormal(i int) {
	if bm.FnormsReady[i] {
		return
	}
	bm.Fnorms[i] = bm.Norms[i].Vec()
	bm.FnormsReady[i] = true
}

// TriBox is the facet's float bounding box — float min/max are exact, so the
// box is a true bound.
func TriBox(verts []r3.Vec, tri [3]int) [2]r3.Vec {
	lo, hi := verts[tri[0]], verts[tri[0]]
	for _, vi := range tri[1:] {
		v := verts[vi]
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	return [2]r3.Vec{lo, hi}
}

func BoxesOverlap(a, b [2]r3.Vec) bool {
	return a[0].X <= b[1].X && b[0].X <= a[1].X &&
		a[0].Y <= b[1].Y && b[0].Y <= a[1].Y &&
		a[0].Z <= b[1].Z && b[0].Z <= a[1].Z
}

// KeptFacet is one facet of the boolean result, still exact: its corners,
// the global source-face id it approximates, and which operand it came from.
type KeptFacet struct {
	V   [3]proof.Xpt
	Src int
	// Beta is each corner's pre-weld bound (docs/faceted-vertex-bounds-
	// design.md §3.1–§3.3): an operand vertex's own β, a rim vertex's
	// RimBound, or, for a point the cutter placed on the operand facet, that
	// facet's δ. Each is a two-sided claim the corner carries into the stitch.
	Beta [3]float64
}

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

// ContactMemo caches TriTriClassify's answer per facet-index pair for ONE
// operand pair, so the second ask a call reaches — the tangency gate
// (facesNearMiss) and the mesh pass (MeshBoolean) walk the same two prepared
// tessellations — is served from the store instead of recomputed.
// TriTriClassify is a pure function of the two facets' float corners, exact
// lifts and exact normals, all read-only fields of ma/mb, so a stored answer
// for (i, j) stays correct for as long as ma and mb do — which is exactly one
// evaluateBoolean call, the memo's whole lifetime. Binding ma/mb into the
// memo itself, rather than taking them per call, means no ask names an
// operand, so a caller cannot key an answer for one operand pair and read it
// back for another.
//
// A returned TriContact is shared: its p0/p1 endpoints and its sin2 are
// *big.Rat values a second caller reads, never a fresh copy. Every consumer
// today only reads them, so this is safe, but no consumer may mutate one in
// place. ContactMemo is not safe for concurrent use; nothing today shares one
// across goroutines.
//
// The sparse store records a compact status instead of copying TriContact into
// every map bucket. Status 1 means ContactNone; status n+2 indexes values[n].
// Zero remains the dense table's absent marker. Once at least one of every 64
// possible pairs has been classified, a table of at most two million pairs is
// promoted to direct indexing. The table therefore occupies at most 8 MB,
// while larger or sparser operand products keep the compact map.
type ContactMemo struct {
	Ma, Mb *BoolMesh
	Cols   int
	Dense  []uint32
	Values []TriContact
	Sparse map[[2]int32]uint64
}

const (
	ContactMemoDensePairLimit    = 2_000_000
	ContactMemoDensePromoteRatio = 64
)

// NewContactMemo builds a memo scoped to one evaluateBoolean call over ma/mb.
func NewContactMemo(ma, mb *BoolMesh) *ContactMemo {
	return &ContactMemo{Ma: ma, Mb: mb, Cols: len(mb.Tris), Sparse: map[[2]int32]uint64{}}
}

func (c *ContactMemo) Lookup(i, j int) (TriContact, bool) {
	var status uint64
	var ok bool
	if c.Dense != nil {
		status = uint64(c.Dense[i*c.Cols+j])
		ok = status != 0
	} else {
		status, ok = c.Sparse[[2]int32{int32(i), int32(j)}]
	}
	if !ok {
		return TriContact{}, false
	}
	if status == 1 {
		return TriContact{EdgeA: -1, EdgeB: -1}, true
	}
	return c.Values[status-2], true
}

func (c *ContactMemo) Store(i, j int, v TriContact) {
	status := uint64(1)
	if v.Kind != ContactNone {
		c.Values = append(c.Values, v)
		status = uint64(len(c.Values) + 1)
	}
	if c.Dense != nil {
		index := i*c.Cols + j
		c.Dense[index] = uint32(status)
		return
	}
	key := [2]int32{int32(i), int32(j)}
	c.Sparse[key] = status
	c.PromoteDense()
}

func (c *ContactMemo) PromoteDense() {
	rows := len(c.Ma.Tris)
	if rows == 0 || c.Cols == 0 || rows > ContactMemoDensePairLimit/c.Cols {
		return
	}
	pairs := rows * c.Cols
	if len(c.Sparse)*ContactMemoDensePromoteRatio < pairs {
		return
	}
	c.Dense = make([]uint32, pairs)
	for key, status := range c.Sparse {
		c.Dense[int(key[0])*c.Cols+int(key[1])] = uint32(status)
	}
	c.Sparse = nil
}

// classify returns the exact classification of facet i of ma against facet j
// of mb, computing it on the first ask and replaying the stored answer on
// every later one. An error is never stored, so a later ask still retries.
func (c *ContactMemo) Classify(i, j int) (TriContact, error) {
	if v, ok := c.Lookup(i, j); ok {
		return v, nil
	}
	c.Ma.PrepareFloatNormal(i)
	c.Mb.PrepareFloatNormal(j)
	v, err := TriTriClassifyPrepared(TriCorners(c.Ma, i), TriCorners(c.Mb, j), XtriCorners(c.Ma, i), XtriCorners(c.Mb, j),
		c.Ma.Norms[i], c.Mb.Norms[j], c.Ma.Fnorms[i], c.Mb.Fnorms[j])
	if err != nil {
		return TriContact{}, err
	}
	c.Store(i, j, v)
	return v, nil
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
			signsB[i] = OrientSignPrepared(ta[0], ta[1], ta[2], tb[i], xta[0], xtb[i], na)
		}
	}
	if AllOneSide(signsB) {
		return out, nil
	}
	if sa != nil {
		signsA = *sa
	} else {
		for i := range 3 {
			signsA[i] = OrientSignPrepared(tb[0], tb[1], tb[2], ta[i], xtb[0], xta[i], nb)
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
	dir := Xcross(na, nb)
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
	c := Xcross(na, nb)
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
			nums[i], dens[i] = OrientNum(xo[0], xo[1], xo[2], xt[i])
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
			out = append(out, Xlerp(xt[i], xt[j], tn, td))
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
	return XdotSign(Xcross(proof.Xsub(b, a), proof.Xsub(p, a)), n)
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

// BooleanKeep is the classification table of §9: which side of the other
// solid each operand's boundary keeps, and whether the kept B side flips
// orientation (a cut turns the tool's skin inside out).
func BooleanKeep(op OperationKind) (bool, bool, bool, error) {
	switch op {
	case OpUnion:
		return false, false, false, nil
	case OpIntersect:
		return true, true, false, nil
	case OpCut:
		return false, true, true, nil
	default:
		return false, false, false, fmt.Errorf(`%w: %q is not a boolean op`, decaderr.ErrBooleanFailed, op)
	}
}

// PairContact is one classified facet pair, kept whole so the decisions the
// PAIR cannot make are made later with the mesh in hand.
type PairContact struct {
	I, J   int // the facet in ma, the facet in mb
	Ta, Tb [3]r3.Vec
	C      TriContact
}

// RimBound is docs/faceted-vertex-bounds-design.md §3.2's pre-weld bound of
// a rim vertex one facet pair creates: the true pieces of the two facets lie
// within deltaA and deltaB of their planes, the two slabs meet in a tube of
// half-width (deltaA + deltaB)/sin θ about the exact crossing line, and the
// true rim point lies in that tube. sin2 is THIS pair's exact squared sine.
// Two exactly held facets amplify nothing and answer 0 at any angle; a
// crossing with no proven positive sine answers +Inf, which the caller
// refuses.
func RimBound(deltaA, deltaB float64, sin2 *big.Rat) float64 {
	d := proofbound.AbsSumUpper(deltaA, deltaB)
	if d <= 0 {
		return 0
	}
	if sin2 == nil {
		return math.Inf(1)
	}
	return proofbound.DivUpper(d, SinLowerBound(sin2))
}

// MeshBoolean runs the exact-predicate boolean over two prepared operand
// tessellations. It returns the kept, still-exact facets, each corner carrying
// its pre-weld bound, and the largest RimBound any contact segment it used
// gives a rim vertex — the figure the caller refuses at the pair diameter
// (docs/faceted-vertex-bounds-design.md §3.2).
func MeshBoolean(ctx context.Context, op OperationKind, ma, mb *BoolMesh, memo *ContactMemo) ([]KeptFacet, float64, error) {
	wantA, wantB, flipB, err := BooleanKeep(op)
	if err != nil {
		return nil, 0, err
	}

	// Exact contacts, per facet of each operand. Facet boxes prune the pairs.
	cutsA := map[int][]Xseg{}
	cutsB := map[int][]Xseg{}
	var pointTouches []proof.Xpt
	var segEnds []proof.Xpt
	var inPlane []PairContact
	// rims maps each rim vertex (a contact segment's endpoint, by exact key)
	// to the largest RimBound over the facet pairs whose segment ends there.
	rims := map[string]float64{}
	maxRim := 0.0
	// touchedA and touchedB are the facets of each operand that meet the
	// other, which the held-mesh gate reads once classification is done.
	var touchedA, touchedB []int
	work := 0
	contacts := NewContactBatchExecutor(ctx, ma, mb, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		ta := TriCorners(ma, i)
		tb := TriCorners(mb, j)
		if c.Kind == ContactRegion {
			return ErrUnclassifiableContact(`two operand facets overlap in one plane`)
		}
		if c.Kind == ContactNone {
			return nil
		}
		touchedA = append(touchedA, i)
		touchedB = append(touchedB, j)
		if c.Kind == ContactPoint {
			pointTouches = append(pointTouches, c.P0)
			return nil
		}
		segEnds = append(segEnds, c.P0, c.P1)
		rim := RimBound(ma.FacetBound[i], mb.FacetBound[j], c.Sin2)
		maxRim = max(maxRim, rim)
		for _, p := range []proof.Xpt{c.P0, c.P1} {
			k := p.Key()
			if prev, ok := rims[k]; !ok || rim > prev {
				rims[k] = rim
			}
		}
		if c.EdgeA >= 0 || c.EdgeB >= 0 {
			// The segment runs ALONG a facet edge. Graze or crossing is not
			// decidable here — hold it for the mesh-level call below.
			inPlane = append(inPlane, PairContact{I: i, J: j, Ta: ta, Tb: tb, C: c})
			return nil
		}
		cutsA[i] = append(cutsA[i], Xseg{
			A: c.P0, B: c.P1,
			AOnEdge: c.P0OnA, BOnEdge: c.P1OnA,
			Partner: tb,
		})
		cutsB[j] = append(cutsB[j], Xseg{
			A: c.P0, B: c.P1,
			AOnEdge: c.P0OnB, BOnEdge: c.P1OnB,
			Partner: ta,
		})
		return nil
	})
	for i := range ma.Tris {
		for j := range mb.Tris {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, 0, err
				}
			}
			if !BoxesOverlap(ma.Boxes[i], mb.Boxes[j]) {
				continue
			}
			if err := contacts.Add(i, j); err != nil {
				return nil, 0, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		return nil, 0, err
	}
	// The held-mesh gate runs on the classification's own answer and before
	// any facet is cut (docs/faceted-vertex-bounds-design.md §5).
	if err := RefuseCoarseHeldContact(ma, 0, touchedA); err != nil {
		return nil, 0, err
	}
	if err := RefuseCoarseHeldContact(mb, 1, touchedB); err != nil {
		return nil, 0, err
	}

	// The graze-or-crossing call, made once, OUTSIDE the pair loop: an in-plane
	// facet edge grazes the other operand exactly when the edge's two adjacent
	// facets lie strictly on ONE side of the other facet's plane — the operand's
	// boundary touches the plane and comes back, a tangency no side
	// classification can be proven for. Apexes that STRADDLE the plane are an
	// ordinary crossing: the boundary genuinely passes through, and the segment
	// is a real rim of the result.
	blockedA := map[[2]int]struct{}{}
	blockedB := map[[2]int]struct{}{}
	for i, p := range inPlane {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, 0, err
			}
		}
		if p.C.EdgeA >= 0 {
			key, crossing, err := EdgeCrosses(ma, p.I, p.C.EdgeA, p.Tb)
			if err != nil {
				return nil, 0, err
			}
			if !crossing {
				return nil, 0, ErrUnclassifiableContact(`an operand edge grazes along the other operand's facet`)
			}
			blockedA[key] = struct{}{}
		}
		if p.C.EdgeB >= 0 {
			key, crossing, err := EdgeCrosses(mb, p.J, p.C.EdgeB, p.Ta)
			if err != nil {
				return nil, 0, err
			}
			if !crossing {
				return nil, 0, ErrUnclassifiableContact(`an operand edge grazes along the other operand's facet`)
			}
			blockedB[key] = struct{}{}
		}
		// A crossing segment subdivides whichever facet does NOT already own it
		// as an edge — a segment lying along a facet's own boundary cuts nothing
		// off it. Its regions classify by exact parity, never by the partner
		// facet's plane: the other operand's boundary there is the DIHEDRAL
		// between two facets, and one plane of it decides nothing.
		if p.C.EdgeA < 0 {
			cutsA[p.I] = append(cutsA[p.I], Xseg{
				A: p.C.P0, B: p.C.P1,
				AOnEdge: p.C.P0OnA, BOnEdge: p.C.P1OnA,
				Partner: p.Tb, ViaParity: true,
			})
		}
		if p.C.EdgeB < 0 {
			cutsB[p.J] = append(cutsB[p.J], Xseg{
				A: p.C.P0, B: p.C.P1,
				AOnEdge: p.C.P0OnB, BOnEdge: p.C.P1OnB,
				Partner: p.Ta, ViaParity: true,
			})
		}
	}

	// A point contact is legitimate only as the endpoint of some crossing
	// segment (a chain passing exactly through a vertex). An isolated one is a
	// tangency the boolean cannot classify: the operands pinch at a point, and
	// stitching it would emit a non-manifold result — refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		ends := map[string]struct{}{}
		for _, p := range segEnds {
			ends[p.Key()] = struct{}{}
		}
		for _, p := range pointTouches {
			if _, ok := ends[p.Key()]; !ok {
				return nil, 0, ErrUnclassifiableContact(`the operand boundaries touch at an isolated point`)
			}
		}
	}

	var kept []KeptFacet
	if err := ctx.Err(); err != nil {
		return nil, 0, err
	}
	keep, err := KeepSide(ctx, ma, mb, cutsA, blockedA, rims, wantA, false)
	if err != nil {
		return nil, 0, err
	}
	kept = append(kept, keep...)
	keep, err = KeepSide(ctx, mb, ma, cutsB, blockedB, rims, wantB, flipB)
	if err != nil {
		return nil, 0, err
	}
	return append(kept, keep...), maxRim, nil
}

// EdgeCrosses decides whether edge k of facet f — which lies exactly in the
// partner facet's plane — GRAZES that plane or genuinely CROSSES it, and
// returns the undirected mesh-edge key either way. The verdict is read off the
// edge's two adjacent facets: their apex vertices strictly on one side is a
// graze; straddling is a crossing. An apex exactly ON the plane proves nothing
// (the facet lies in it), so it reads as a graze — reject-only.
func EdgeCrosses(m *BoolMesh, f, k int, partner [3]r3.Vec) ([2]int, bool, error) {
	tri := m.Tris[f]
	u, v := tri[k], tri[(k+1)%3]
	key := [2]int{min(u, v), max(u, v)}
	twin, ok := m.TwinFacet(f, k)
	if !ok {
		return key, false, fmt.Errorf(`%w: an in-plane facet edge has no twin facet`, decaderr.ErrBooleanFailed)
	}
	tt := m.Tris[twin]
	apex := m.Verts[tri[0]+tri[1]+tri[2]-u-v]
	apexTwin := m.Verts[tt[0]+tt[1]+tt[2]-u-v]
	s0 := OrientSign(partner[0], partner[1], partner[2], apex)
	s1 := OrientSign(partner[0], partner[1], partner[2], apexTwin)
	return key, s0*s1 < 0, nil
}

// BlockedBetween reports whether the mesh edge two adjacent facets share
// carries an in-plane CROSSING contact. The other operand's boundary passes
// exactly along that edge, so the two facets sit on opposite sides of it: they
// must not flood-fill into one classification, or one of them inherits the
// other's answer.
func BlockedBetween(m *BoolMesh, a, b int, blocked map[[2]int]struct{}) bool {
	if len(blocked) == 0 {
		return false
	}
	ta, tb := m.Tris[a], m.Tris[b]
	for k := range 3 {
		u, v := ta[k], ta[(k+1)%3]
		if _, ok := blocked[[2]int{min(u, v), max(u, v)}]; !ok {
			continue
		}
		if SharesVertices(tb, u, v) {
			return true
		}
	}
	return false
}

func SharesVertices(t [3]int, u, v int) bool {
	hasU, hasV := false, false
	for _, x := range t {
		switch x {
		case u:
			hasU = true
		case v:
			hasV = true
		}
	}
	return hasU && hasV
}

func TriCorners(m *BoolMesh, i int) [3]r3.Vec {
	t := m.Tris[i]
	return [3]r3.Vec{m.Verts[t[0]], m.Verts[t[1]], m.Verts[t[2]]}
}

func XtriCorners(m *BoolMesh, i int) [3]proof.Xpt {
	t := m.Tris[i]
	return [3]proof.Xpt{m.Xverts[t[0]], m.Xverts[t[1]], m.Xverts[t[2]]}
}

// KeepSide classifies one operand's boundary against the other solid and
// returns the kept facets: subdivided pieces for the cut facets, whole
// facets for the uncut regions (classified per connected component by exact
// ray parity — the classification is constant on a component that crosses
// nothing). Every kept corner carries its pre-weld bound (KeptFacet.Beta):
// an operand vertex keeps its own β, and a subdivision vertex takes rims'
// bound when it is a rim vertex and its facet's δ otherwise (CutCornerBound).
func KeepSide(ctx context.Context, m, other *BoolMesh, cuts map[int][]Xseg, blocked map[[2]int]struct{}, rims map[string]float64, wantInside, flip bool) ([]KeptFacet, error) {
	var kept []KeptFacet
	emit := func(tri [3]proof.Xpt, beta [3]float64, src int) {
		if flip {
			tri[1], tri[2] = tri[2], tri[1]
			beta[1], beta[2] = beta[2], beta[1]
		}
		kept = append(kept, KeptFacet{V: tri, Src: src, Beta: beta})
	}

	// The cut facets: exact subdivision along the contact chains, then the
	// exact local side-of-contact classification per region.
	for i := range m.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		segs, ok := cuts[i]
		if !ok {
			continue
		}
		regions, err := CutTriangle(ctx, XtriCorners(m, i), m.Norms[i], segs)
		if err != nil {
			return nil, err
		}
		corners := m.Tris[i]
		cornerKeys := [3]string{m.Xverts[corners[0]].Key(), m.Xverts[corners[1]].Key(), m.Xverts[corners[2]].Key()}
		for _, reg := range regions {
			inside, err := ClassifyRegion(ctx, reg, other)
			if err != nil {
				return nil, err
			}
			if inside != wantInside {
				continue
			}
			for _, tri := range reg.Tris {
				var beta [3]float64
				for k, p := range tri {
					beta[k] = CutCornerBound(m, i, cornerKeys, p.Key(), rims)
				}
				emit(tri, beta, m.Src[i])
			}
		}
	}

	// The uncut facets: constant classification per connected uncut
	// component, one exact parity seed each.
	comp := make([]int, len(m.Tris))
	for i := range comp {
		comp[i] = -1
	}
	adj, err := FacetAdjacencyContext(ctx, m.Tris)
	if err != nil {
		return nil, err
	}
	next := 0
	componentWork := 0
	all := AllFacets(other)
	for i := range m.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if _, cut := cuts[i]; cut || comp[i] != -1 {
			continue
		}
		id := next
		next++
		queue := []int{i}
		comp[i] = id
		var members []int
		for len(queue) > 0 {
			componentWork++
			if componentWork%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			f := queue[0]
			queue = queue[1:]
			members = append(members, f)
			for _, nb := range adj[f] {
				if _, cut := cuts[nb]; cut || comp[nb] != -1 {
					continue
				}
				if BlockedBetween(m, f, nb, blocked) {
					continue
				}
				comp[nb] = id
				queue = append(queue, nb)
			}
		}
		// Every facet has a real interior to probe: prepBoolMesh refused the
		// operand outright if any facet collapsed.
		seed := XtriCorners(m, members[0])
		probe := XCentroid(seed[0], seed[1], seed[2])
		inside, onBoundary, err := MeshParityPreparedContext(ctx, probe, other.Parity, all)
		if err != nil {
			return nil, err
		}
		if onBoundary {
			return nil, ErrUnclassifiableContact(`an operand facet touches the other operand's boundary`)
		}
		if inside != wantInside {
			continue
		}
		for _, f := range members {
			t := m.Tris[f]
			emit(XtriCorners(m, f), [3]float64{m.VertexBound[t[0]], m.VertexBound[t[1]], m.VertexBound[t[2]]}, m.Src[f])
		}
	}
	return kept, nil
}

// CutCornerBound is the pre-weld bound of one corner of a piece CutTriangle
// cut from facet i (docs/faceted-vertex-bounds-design.md §3.1–§3.3), the
// corner named by its exact key: one of the facet's own corners keeps that
// vertex's β, a rim vertex takes the largest RimBound recorded for it, a
// point that is both takes the larger, and any other point — a split-line or
// edge point the cutter placed on the held facet — lies on the facet and
// takes its δ.
func CutCornerBound(m *BoolMesh, i int, cornerKeys [3]string, key string, rims map[string]float64) float64 {
	beta, known := 0.0, false
	for k, ck := range cornerKeys {
		if ck == key {
			beta, known = max(beta, m.VertexBound[m.Tris[i][k]]), true
		}
	}
	if rim, ok := rims[key]; ok {
		beta, known = max(beta, rim), true
	}
	if !known {
		return m.FacetBound[i]
	}
	return beta
}

// ClassifyRegion decides whether a subdivision region lies inside the other
// solid. A region anchored to a contact chain is decided by the exact side
// of the partner facet's plane — the other solid's boundary IS that facet
// along the shared chain edge, so the side is the answer; an unanchored
// region (an artifact of the loop-opening split lines) falls back to exact
// parity.
func ClassifyRegion(ctx context.Context, reg CutRegion, other *BoolMesh) (bool, error) {
	if reg.HasAnchor {
		switch s := OrientSignMixed(reg.Partner[0], reg.Partner[1], reg.Partner[2], reg.Probe); {
		case s < 0:
			return true, nil
		case s > 0:
			return false, nil
		default:
			return false, fmt.Errorf(`%w: a region probe landed on its contact plane`, decaderr.ErrBooleanFailed)
		}
	}
	inside, onBoundary, err := MeshParityPreparedContext(ctx, reg.Probe, other.Parity, AllFacets(other))
	if err != nil {
		return false, err
	}
	if onBoundary {
		return false, ErrUnclassifiableContact(`a subdivision region touches the other operand's boundary`)
	}
	return inside, nil
}

func AllFacets(m *BoolMesh) []int {
	out := make([]int, len(m.Tris))
	for i := range out {
		out[i] = i
	}
	return out
}

// FacetAdjacencyContext maps each facet to its edge neighbors through the
// watertight mesh's paired directed edges, with bounded cancellation checks.
func FacetAdjacencyContext(ctx context.Context, tris [][3]int) ([][]int, error) {
	owner := map[[2]int]int{}
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj, nil
}

// XCentroid is (a+b+c)/3, exact. Dividing by 3 multiplies the shared
// denominator by 3 rather than dividing the numerators by it, since a
// numerator need not be a multiple of 3 — the same "multiply w, never divide
// the numerator" rule every homogeneous construction here follows.
func XCentroid(a, b, c proof.Xpt) proof.Xpt {
	bwcw := new(big.Int).Mul(b.W, c.W)
	awcw := new(big.Int).Mul(a.W, c.W)
	awbw := new(big.Int).Mul(a.W, b.W)
	axis := func(ax, bx, cx *big.Int) *big.Int {
		s := new(big.Int).Mul(ax, bwcw)
		s.Add(s, new(big.Int).Mul(bx, awcw))
		s.Add(s, new(big.Int).Mul(cx, awbw))
		return s
	}
	w := new(big.Int).Mul(awbw, c.W)
	w.Mul(w, big.NewInt(3))
	return proof.Xpt(proof.XhpStripTwosOwned(proof.Xhp{
		X: axis(a.X, b.X, c.X),
		Y: axis(a.Y, b.Y, c.Y),
		Z: axis(a.Z, b.Z, c.Z),
		W: w,
	}))
}

// StitchedMesh is the boolean output after welding, conforming and rounding:
// the held float mesh, its per-facet source-face ids, and the proven terms the
// rounding's own error composes from.
type StitchedMesh struct {
	Verts []r3.Vec
	Tris  [][3]int
	Src   []int
	// round is the proven displacement (mm) of one vertex under the final
	// float rounding: no held vertex is farther than this from the exact
	// point it stands for.
	Round float64
	// preArea upper-bounds the area of the surface the rounding acted ON —
	// the stitched surface BEFORE any facet was dropped, and along the whole
	// motion from it to the held mesh. It is what the volume error is charged
	// against, NOT the held mesh's area: a facet the weld collapses is gone
	// from the held mesh, and charging the rounding against what survived
	// would leave that facet's own swept volume out of the bound.
	PreArea float64
	// dropArea upper-bounds the area of the facets the weld collapsed, which
	// the held mesh no longer carries. The reported surface area is short by
	// exactly this much, so the area bound must cover it.
	DropArea float64
	// VertexBound is β(v) per held vertex (docs/faceted-vertex-bounds-design.md
	// §3.4): the largest, over the exact vertices the held one stands for, of
	// that exact vertex's pre-weld bound plus its own rounding distance to the
	// held float, rounded up. An exact vertex held at its own float adds
	// nothing, so an operand vertex keeps its operand β to the bit.
	VertexBound []float64
}

// stitchFacets welds the kept facets by shared exact vertices, makes the
// subdivision conforming (a vertex lying exactly on another facet's edge
// splits that facet — exact incidence, so nothing moves), audits closure,
// and rounds to float64. The audit is the §9 guarantee: every directed edge
// pairs with its reverse, or the boolean fails — never a cracked mesh.
func StitchFacetsContext(ctx context.Context, kept []KeptFacet) (*StitchedMesh, error) {
	if len(kept) == 0 {
		return nil, fmt.Errorf(`%w: the operation leaves no boundary at all`, decaderr.ErrBooleanFailed)
	}
	var xverts []proof.Xpt
	// xbeta is each exact vertex's pre-weld bound: the largest claim any kept
	// facet corner makes for it, raised below by every conforming split that
	// inserts it into another facet's edge.
	var xbeta []float64
	index := map[string]int{}
	// Exact vertices are immutable after construction. Reuse the canonical key
	// when another incident facet carries the same four integer pointers.
	keyByRaw := map[[4]*big.Int]string{}
	addVert := func(p proof.Xpt) int {
		raw := [4]*big.Int{p.X, p.Y, p.Z, p.W}
		k, ok := keyByRaw[raw]
		if !ok {
			k = p.Key()
			keyByRaw[raw] = k
		}
		if i, ok := index[k]; ok {
			return i
		}
		index[k] = len(xverts)
		xverts = append(xverts, p)
		xbeta = append(xbeta, 0)
		return len(xverts) - 1
	}
	var tris [][3]int
	var src []int
	// facetBound is each stitched triangle's δ: the largest corner claim of
	// the kept facet it descends from. A conforming split keeps its parent's
	// figure, since every sub-triangle lies on that held facet.
	var facetBound []float64
	for i, f := range kept {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c := addVert(f.V[0]), addVert(f.V[1]), addVert(f.V[2])
		if a == b || b == c || c == a {
			return nil, fmt.Errorf(`%w: a kept facet collapsed`, decaderr.ErrBooleanFailed)
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
			return nil, fmt.Errorf(`%w: the conforming pass did not converge`, decaderr.ErrBooleanFailed)
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		split, err := ConformOnce(ctx, xverts, &tris, &src, &facetBound, xbeta)
		if err != nil {
			return nil, err
		}
		if !split {
			break
		}
	}

	// Closure audit: each directed edge exactly once, each with its reverse.
	directed := map[[2]int]int{}
	for _, tri := range tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the stitched boundary does not close`, decaderr.ErrBooleanFailed)
		}
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
	out := &StitchedMesh{}
	floatIdx := map[r3.Vec]int{}
	remap := make([]int, len(xverts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		v := p.Vec()
		fi, ok := floatIdx[v]
		if !ok {
			fi = len(out.Verts)
			floatIdx[v] = fi
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
		return nil, fmt.Errorf(`%w: the whole result collapsed under rounding`, decaderr.ErrBooleanFailed)
	}
	if err := RefuseWeldedAwayComponent(ctx, tris, dropped); err != nil {
		return nil, err
	}
	if err := keepRoundedEmbedded(ctx, xverts, remap, out); err != nil {
		return nil, err
	}
	// worst is the max PER-COORDINATE distance from an exact vertex to the
	// held float that stands for it — measured AFTER the embedding check, so a
	// vertex it placed at another corner of its float box is charged where it
	// actually sits. The consumers read a 3D distance bound, and all three
	// coordinates can round at once (internal/proofbound/bounds.go,
	// proofbound.Radius3D). A positive rational error can round to zero as
	// float64, so preserve that proof before widening it to a 3D radius.
	worst := new(big.Rat)
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if d := CoordDistance(p, out.Verts[remap[i]]); d.Cmp(worst) > 0 {
			worst = d
		}
	}
	if worst.Sign() > 0 {
		w, _ := worst.Float64()
		out.Round = proofbound.Radius3D(proofbound.ProvenUpRound(w))
	}
	vertexBound, err := HeldVertexBounds(ctx, xverts, xbeta, remap, out.Verts)
	if err != nil {
		return nil, err
	}
	out.VertexBound = vertexBound
	// The pre-round surface: every facet the exact stitch produced, dropped
	// ones included, measured on the held vertices and inflated by the
	// rounding they may each have travelled (internal/proofbound/bounds.go, proofbound.PerturbedAreaUpper).
	out.PreArea = proofbound.PerturbedAreaUpper(out.Verts, welded, out.Round)
	var droppedTris [][3]int
	for ti := range tris {
		if dropped[ti] {
			droppedTris = append(droppedTris, welded[ti])
		}
	}
	out.DropArea = proofbound.PerturbedAreaUpper(out.Verts, droppedTris, out.Round)

	directed = map[[2]int]int{}
	for _, tri := range out.Tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the rounded boundary does not close`, decaderr.ErrBooleanFailed)
		}
	}
	return out, nil
}

// HeldVertexBounds is docs/faceted-vertex-bounds-design.md §3.4's weld, per
// vertex: each exact vertex's own rounding distance to the held float that
// stands for it, read exactly and widened to a 3D radius, is added to that
// exact vertex's pre-weld bound, and a held vertex takes the largest sum over
// the exact vertices welded into it. An exact vertex its float holds exactly
// adds nothing and is not rounded up, so its bound passes through unchanged.
func HeldVertexBounds(ctx context.Context, xverts []proof.Xpt, xbeta []float64, remap []int, held []r3.Vec) ([]float64, error) {
	out := make([]float64, len(held))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		fi := remap[i]
		beta := xbeta[i]
		if gap := CoordDistance(p, held[fi]); gap.Sign() > 0 {
			g, _ := gap.Float64()
			beta = proofbound.AbsSumUpper(beta, proofbound.Radius3D(proofbound.ProvenUpRound(g)))
		}
		if proofbound.IsNonFinite(beta) {
			return nil, fmt.Errorf(`%w: a result vertex's displacement bound is not finite`, decaderr.ErrUnsupported)
		}
		out[fi] = max(out[fi], beta)
	}
	return out, nil
}

// CoordDistance is the exact largest per-coordinate distance from p to v.
func CoordDistance(p proof.Xpt, v r3.Vec) *big.Rat {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	d := new(big.Rat)
	for _, pair := range [][2]*big.Rat{{px, polynomial.MustRatOf(v.X)}, {py, polynomial.MustRatOf(v.Y)}, {pz, polynomial.MustRatOf(v.Z)}} {
		dd := new(big.Rat).Sub(pair[0], pair[1])
		dd.Abs(dd)
		if dd.Cmp(d) > 0 {
			d = dd
		}
	}
	return d
}

// keepRoundedEmbedded runs EnforceHeldEmbedding over the rounded result. A
// held vertex is moved when it differs from an exact vertex it stands for,
// and the search may place it only when it stands for exactly one exact
// vertex: a welded vertex is checked where it sits, never moved, because no
// single float box belongs to it.
func keepRoundedEmbedded(ctx context.Context, xverts []proof.Xpt, remap []int, out *StitchedMesh) error {
	h := HeldRounding{
		Verts:   out.Verts,
		Tris:    out.Tris,
		Exact:   make([]proof.Xpt, len(out.Verts)),
		Moved:   make([]bool, len(out.Verts)),
		Movable: make([]bool, len(out.Verts)),
	}
	count := make([]int, len(out.Verts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		fi := remap[i]
		count[fi]++
		h.Exact[fi] = p
		if CoordDistance(p, out.Verts[fi]).Sign() != 0 {
			h.Moved[fi] = true
		}
	}
	for fi, n := range count {
		if n > 1 {
			h.Moved[fi] = true
		}
		h.Movable[fi] = n == 1 && h.Moved[fi]
	}
	_, err := EnforceHeldEmbedding(ctx, h)
	return err
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
func RefuseWeldedAwayComponent(ctx context.Context, tris [][3]int, dropped []bool) error {
	comp := make([]int, len(tris))
	for i := range comp {
		comp[i] = -1
	}
	adj, err := FacetAdjacencyContext(ctx, tris)
	if err != nil {
		return err
	}
	work := 0
	for i := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		if comp[i] != -1 {
			continue
		}
		queue := []int{i}
		comp[i] = i
		alive := false
		for len(queue) > 0 {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			f := queue[0]
			queue = queue[1:]
			alive = alive || !dropped[f]
			for _, nb := range adj[f] {
				if comp[nb] != -1 {
					continue
				}
				comp[nb] = i
				queue = append(queue, nb)
			}
		}
		if !alive {
			return fmt.Errorf(`%w: a whole component of the result welds away under float rounding — the lump would vanish from the body, and no volume, lump count or bound could then be trusted; this evaluator refuses rather than drop it`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

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
	cr := Xcross(d, ap)
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
		cand := Xcross(proof.Xsub(verts[poly[i]], verts[poly[0]]), proof.Xsub(verts[poly[i+1]], verts[poly[0]]))
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
