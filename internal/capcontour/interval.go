package capcontour

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Point is a rational-interval enclosure of one plane-local (u, v) point.
type Point struct{ U, V proofbound.RatInterval }

// InsideSignOf reads a circular wall's walked sense: positive when its
// material lies inside the circle and negative when it lies outside.
func InsideSignOf(w survey2d.SideWalk) *big.Rat {
	if w.Th1 < w.Th0 {
		return big.NewRat(-1, 1)
	}
	return big.NewRat(1, 1)
}

// CapWallRadiusOffset is the exact cap contour radius change, -insideSign*d.
// A non-finite setback has no rational offset and returns nil.
func CapWallRadiusOffset(w survey2d.SideWalk, d float64) *big.Rat {
	rd := proofarith.FloatRat(d)
	if rd == nil {
		return nil
	}
	return new(big.Rat).Neg(new(big.Rat).Mul(InsideSignOf(w), rd))
}

// ExactPoint lifts a pair of float64 coordinates, which are exact rationals.
func ExactPoint(u, v float64) (Point, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	if ru == nil || rv == nil {
		return Point{}, false
	}
	return Point{U: proofbound.PointInterval(ru), V: proofbound.PointInterval(rv)}, true
}

// Reach is an upper bound on |p − q| over every q the enclosure holds, so a
// point known to lie in the enclosure sits at most this far from p.
func (e Point) Reach(u, v float64) float64 {
	du, okU := AxisSpread(e.U, u)
	dv, okV := AxisSpread(e.V, v)
	if !okU || !okV {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	))
}

// AxisSpread is max(|lo − c|, |hi − c|), the furthest the interval reaches
// from c along one axis.
func AxisSpread(iv proofbound.RatInterval, c float64) (*big.Rat, bool) {
	rc := proofarith.FloatRat(c)
	if rc == nil || iv.Lo == nil || iv.Hi == nil {
		return nil, false
	}
	lo := new(big.Rat).Abs(new(big.Rat).Sub(iv.Lo, rc))
	hi := new(big.Rat).Abs(new(big.Rat).Sub(iv.Hi, rc))
	if lo.Cmp(hi) >= 0 {
		return lo, true
	}
	return hi, true
}

// Union is the smallest box holding both enclosures — what an ambiguous root
// selection reports, so an unresolved choice widens the displacement rather
// than picking a branch the exact arithmetic has not decided.
func Union(a, b Point) Point {
	return Point{U: IntervalHull(a.U, b.U), V: IntervalHull(a.V, b.V)}
}

// IntervalHull returns the smallest interval containing both inputs.
func IntervalHull(a, b proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := a.Lo, a.Hi
	if b.Lo.Cmp(lo) < 0 {
		lo = b.Lo
	}
	if b.Hi.Cmp(hi) > 0 {
		hi = b.Hi
	}
	return proofbound.Interval(lo, hi)
}

// UnitVec encloses the EXACT unit vector of a float pair — the value
// normalize2 rounds. The pair itself is exact, so the only widening is the
// length's own outward-rounded square root.
func UnitVec(x, y float64) (Point, bool) {
	rx, ry := proofarith.FloatRat(x), proofarith.FloatRat(y)
	if rx == nil || ry == nil {
		return Point{}, false
	}
	n2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx), new(big.Rat).Mul(ry, ry))
	if n2.Sign() == 0 {
		return Point{}, false
	}
	l, ok := proofbound.IntervalSqrt(proofbound.PointInterval(n2))
	if !ok || l.Lo.Sign() <= 0 {
		return Point{}, false
	}
	u, okU := proofbound.IntervalQuo(proofbound.PointInterval(rx), l)
	v, okV := proofbound.IntervalQuo(proofbound.PointInterval(ry), l)
	if !okU || !okV {
		return Point{}, false
	}
	return Point{U: u, V: v}, true
}

// OffsetFoot encloses the exact material-side foot v + d·rot90(unit(t)) —
// the point shell_offset.go and capblend_geom.go both spell
// v + d·(−ty, tx) after normalize2.
func OffsetFoot(vU, vV, tu, tv, d float64) (Point, bool) {
	n, ok := UnitVec(tu, tv)
	if !ok {
		return Point{}, false
	}
	v, okV := ExactPoint(vU, vV)
	rd := proofarith.FloatRat(d)
	if !okV || rd == nil {
		return Point{}, false
	}
	return Point{
		U: proofbound.IntervalAdd(v.U, proofbound.IntervalScale(proofbound.IntervalNeg(n.V), rd)),
		V: proofbound.IntervalAdd(v.V, proofbound.IntervalScale(n.U, rd)),
	}, true
}

// Carrier is one wall's offset carrier, enclosed: an offset LINE (a point on
// it plus its unit direction) or a concentric CIRCLE (the wall's own exact
// centre and the exact offset radius). It mirrors shell_offset.go's
// offsetCarrier field for field, so the enclosure is of the same object the
// float miter solve intersects.
type Carrier struct {
	IsLine bool
	P, Dir Point
	C      Point
	R      proofbound.RatInterval
}

func CarrierOf(w survey2d.SideWalk, d float64) (Carrier, bool) {
	if !w.IsCircular() {
		p, okP := OffsetFoot(w.StartU, w.StartV, w.TanInU, w.TanInV, d)
		dir, okD := UnitVec(w.TanInU, w.TanInV)
		if !okP || !okD {
			return Carrier{}, false
		}
		return Carrier{IsLine: true, P: p, Dir: dir}, true
	}
	r, ok := ExactOffsetRadius(w, d)
	if !ok {
		return Carrier{}, false
	}
	c, okC := ExactPoint(w.CU, w.CV)
	if !okC {
		return Carrier{}, false
	}
	return Carrier{C: c, R: proofbound.PointInterval(r)}, true
}

// ExactOffsetRadius is offsetRadius's own R − insideSign·d taken EXACTLY:
// both operands are float64s, so their difference is a rational with no
// rounding at all, and the float the build holds is the rounding of THIS value.
func ExactOffsetRadius(w survey2d.SideWalk, d float64) (*big.Rat, bool) {
	inside := 1.0
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	rr, rd := proofarith.FloatRat(w.Radius), proofarith.FloatRat(inside*d)
	if rr == nil || rd == nil {
		return nil, false
	}
	out := new(big.Rat).Sub(rr, rd)
	if out.Sign() <= 0 {
		return nil, false
	}
	return out, true
}

// Intersect encloses every root of the two offset carriers, dispatching
// exactly as fillet.go's intersectOffsets does over the same three cases.
func Intersect(a, b Carrier) ([]Point, bool) {
	switch {
	case a.IsLine && b.IsLine:
		return lineLine(a, b)
	case a.IsLine:
		return lineCircle(a, b)
	case b.IsLine:
		return lineCircle(b, a)
	default:
		return circleCircle(a, b)
	}
}

func lineLine(a, b Carrier) ([]Point, bool) {
	den := proofbound.IntervalSub(proofbound.IntervalMul(a.Dir.U, b.Dir.V), proofbound.IntervalMul(a.Dir.V, b.Dir.U))
	num := proofbound.IntervalSub(
		proofbound.IntervalMul(proofbound.IntervalSub(b.P.U, a.P.U), b.Dir.V),
		proofbound.IntervalMul(proofbound.IntervalSub(b.P.V, a.P.V), b.Dir.U),
	)
	s, ok := proofbound.IntervalQuo(num, den)
	if !ok {
		return nil, false
	}
	return []Point{{
		U: proofbound.IntervalAdd(a.P.U, proofbound.IntervalMul(s, a.Dir.U)),
		V: proofbound.IntervalAdd(a.P.V, proofbound.IntervalMul(s, a.Dir.V)),
	}}, true
}

func lineCircle(l, c Carrier) ([]Point, bool) {
	fx := proofbound.IntervalSub(l.P.U, c.C.U)
	fy := proofbound.IntervalSub(l.P.V, c.C.V)
	bb := proofbound.IntervalAdd(proofbound.IntervalMul(fx, l.Dir.U), proofbound.IntervalMul(fy, l.Dir.V))
	cc := proofbound.IntervalSub(proofbound.IntervalAdd(proofbound.IntervalSquare(fx), proofbound.IntervalSquare(fy)), proofbound.IntervalSquare(c.R))
	disc := proofbound.IntervalSub(proofbound.IntervalSquare(bb), cc)
	if disc.Hi.Sign() < 0 {
		// The exact carriers miss each other entirely: the float solve reached
		// a root of a system that has none, so there is no denoted point to
		// enclose.
		return nil, false
	}
	sq, ok := proofbound.IntervalSqrt(disc)
	if !ok {
		return nil, false
	}
	nb := proofbound.IntervalNeg(bb)
	out := make([]Point, 0, 2)
	for _, s := range []proofbound.RatInterval{proofbound.IntervalAdd(nb, sq), proofbound.IntervalSub(nb, sq)} {
		out = append(out, Point{
			U: proofbound.IntervalAdd(l.P.U, proofbound.IntervalMul(s, l.Dir.U)),
			V: proofbound.IntervalAdd(l.P.V, proofbound.IntervalMul(s, l.Dir.V)),
		})
	}
	return out, true
}

func circleCircle(a, b Carrier) ([]Point, bool) {
	dx := proofbound.IntervalSub(b.C.U, a.C.U)
	dy := proofbound.IntervalSub(b.C.V, a.C.V)
	dsq := proofbound.IntervalAdd(proofbound.IntervalSquare(dx), proofbound.IntervalSquare(dy))
	dist, ok := proofbound.IntervalSqrt(dsq)
	if !ok || dist.Lo.Sign() <= 0 {
		return nil, false
	}
	mid, okMid := proofbound.IntervalQuo(
		proofbound.IntervalSub(proofbound.IntervalAdd(dsq, proofbound.IntervalSquare(a.R)), proofbound.IntervalSquare(b.R)),
		proofbound.IntervalScale(dist, big.NewRat(2, 1)),
	)
	if !okMid {
		return nil, false
	}
	h2 := proofbound.IntervalSub(proofbound.IntervalSquare(a.R), proofbound.IntervalSquare(mid))
	if h2.Hi.Sign() < 0 {
		return nil, false
	}
	h, okH := proofbound.IntervalSqrt(h2)
	if !okH {
		return nil, false
	}
	along, okA := proofbound.IntervalQuo(mid, dist)
	across, okC := proofbound.IntervalQuo(h, dist)
	if !okA || !okC {
		return nil, false
	}
	baseU := proofbound.IntervalAdd(a.C.U, proofbound.IntervalMul(along, dx))
	baseV := proofbound.IntervalAdd(a.C.V, proofbound.IntervalMul(along, dy))
	offU := proofbound.IntervalMul(across, dy)
	offV := proofbound.IntervalMul(across, dx)
	return []Point{
		{U: proofbound.IntervalSub(baseU, offU), V: proofbound.IntervalAdd(baseV, offV)},
		{U: proofbound.IntervalAdd(baseU, offU), V: proofbound.IntervalSub(baseV, offV)},
	}, true
}

// Nearest encloses intersectOffsets' own "root nearest the corner". A
// candidate whose squared-distance interval starts beyond another's end is
// PROVEN not to be the nearest and is dropped; every candidate the exact
// arithmetic leaves undecided joins the hull, so a near-tangency reports one
// wide honest displacement rather than a branch nothing decided.
func Nearest(cands []Point, vU, vV float64) (Point, bool) {
	corner, ok := ExactPoint(vU, vV)
	if !ok {
		return Point{}, false
	}
	return NearestTo(cands, corner)
}

// NearestTo is Nearest about an enclosed corner: a candidate is dropped
// only when its squared distance to EVERY corner the box holds is proven
// beyond another candidate's farthest.
func NearestTo(cands []Point, corner Point) (Point, bool) {
	if len(cands) == 0 {
		return Point{}, false
	}
	d2 := make([]proofbound.RatInterval, len(cands))
	for i, c := range cands {
		d2[i] = proofbound.IntervalAdd(
			proofbound.IntervalSquare(proofbound.IntervalSub(c.U, corner.U)),
			proofbound.IntervalSquare(proofbound.IntervalSub(c.V, corner.V)),
		)
	}
	best := d2[0].Hi
	for _, iv := range d2[1:] {
		if iv.Hi.Cmp(best) < 0 {
			best = iv.Hi
		}
	}
	var out Point
	found := false
	for i, iv := range d2 {
		if iv.Lo.Cmp(best) > 0 {
			continue
		}
		if !found {
			out, found = cands[i], true
			continue
		}
		out = Union(out, cands[i])
	}
	return out, found
}

// offsetFootRange generalises OffsetFoot to an OFFSET INTERVAL rather
// than one float: the enclosure of v + t·rot90(unit(t)) for every offset
// amount t in tRange, the point family a line carrier's own anchor sweeps as
// the offset amount varies. OffsetFoot is this at one degenerate point.
func offsetFootRange(vU, vV, tu, tv float64, tRange proofbound.RatInterval) (Point, bool) {
	n, ok := UnitVec(tu, tv)
	if !ok {
		return Point{}, false
	}
	v, okV := ExactPoint(vU, vV)
	if !okV {
		return Point{}, false
	}
	return Point{
		U: proofbound.IntervalAdd(v.U, proofbound.IntervalMul(proofbound.IntervalNeg(n.V), tRange)),
		V: proofbound.IntervalAdd(v.V, proofbound.IntervalMul(n.U, tRange)),
	}, true
}

// carrierOverRange generalises CarrierOf to an OFFSET INTERVAL [t0, t1]
// rather than one float, enclosing every carrier the wall's own offset
// construction occupies as the offset amount ranges over it — the same
// closed forms CarrierOf evaluates at one point, evaluated over the whole
// range instead. A line's carrier stays a single line: only its anchor point
// moves, along the line's own FIXED unit normal, so the direction needs no
// widening at all. A circle's carrier stays a single concentric circle whose
// radius now encloses the offset radius's own range rather than one value.
// At t0 == t1 == d it reduces to CarrierOf(w, d)'s own enclosure, since
// both build the offset amount from the identical closed form.
func carrierOverRange(w survey2d.SideWalk, t0, t1 float64) (Carrier, bool) {
	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return Carrier{}, false
	}
	tRange := proofbound.Interval(rt0, rt1)
	if !w.IsCircular() {
		p, okP := offsetFootRange(w.StartU, w.StartV, w.TanInU, w.TanInV, tRange)
		dir, okD := UnitVec(w.TanInU, w.TanInV)
		if !okP || !okD {
			return Carrier{}, false
		}
		return Carrier{IsLine: true, P: p, Dir: dir}, true
	}
	rr := proofarith.FloatRat(w.Radius)
	if rr == nil {
		return Carrier{}, false
	}
	inside := big.NewRat(1, 1)
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = big.NewRat(-1, 1)
	}
	r := proofbound.IntervalSub(proofbound.PointInterval(rr), proofbound.IntervalMul(tRange, proofbound.PointInterval(inside)))
	if r.Lo.Sign() <= 0 {
		return Carrier{}, false
	}
	c, okC := ExactPoint(w.CU, w.CV)
	if !okC {
		return Carrier{}, false
	}
	return Carrier{C: c, R: r}, true
}

// miterConstraintRow is one carrier's own linear constraint on the miter
// locus's velocity P'(t), read at a foot enclosed within the box the
// intersection of the corner's two carriers over the offset range proves
// (carrierOverRange, Intersect, Nearest).
//
// A line carrier's offset satisfies n·X = n·v0 + t for its own fixed unit
// normal n (the construction's own p0(t) = v0 + t·n, and every point of the
// offset LINE shares n's component since the line runs perpendicular to it),
// so differentiating in t gives n·P'(t) = 1 — a row that needs no widening at
// all, since n does not depend on t.
//
// A circle carrier's offset satisfies |X-c| = R - inside·t (offsetRadius's
// own closed form), so differentiating gives û·P'(t) = -inside, with
// û = (X-c)/|X-c| the unit radial direction AT THE FOOT — which does depend
// on t, so it is enclosed from the box foot proves rather than read at one
// point.
func miterConstraintRow(w survey2d.SideWalk, c Carrier, foot Point) (Point, *big.Rat, bool) {
	if c.IsLine {
		n := Point{U: proofbound.IntervalNeg(c.Dir.V), V: c.Dir.U}
		return n, big.NewRat(1, 1), true
	}
	dx := proofbound.IntervalSub(foot.U, c.C.U)
	dy := proofbound.IntervalSub(foot.V, c.C.V)
	distSq := proofbound.IntervalAdd(proofbound.IntervalSquare(dx), proofbound.IntervalSquare(dy))
	dist, ok := proofbound.IntervalSqrt(distSq)
	if !ok || dist.Lo.Sign() <= 0 {
		return Point{}, nil, false
	}
	ux, okU := proofbound.IntervalQuo(dx, dist)
	uy, okV := proofbound.IntervalQuo(dy, dist)
	if !okU || !okV {
		return Point{}, nil, false
	}
	rhs := big.NewRat(-1, 1)
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		rhs = big.NewRat(1, 1)
	}
	return Point{U: ux, V: uy}, rhs, true
}

// CircleCircleLocusSpeedUpper bounds |dP/dt| for a corner where TWO circular
// walls' own offset carriers meet, by enclosing the corner foot over the
// whole offset range at once (carrierOverRange, Intersect, Nearest)
// and solving the 2x2 linear system miterConstraintRow builds from each
// carrier — Cramer's rule, carried out in interval arithmetic. ok is false
// whenever the determinant's own enclosure straddles zero (the two carriers'
// rows are nearly parallel there) or any enclosure along the way fails.
//
// This decorrelates the two carriers' own offset amount — each is enclosed
// over [t0, t1] independently, rather than tracked as the SAME shared
// parameter through both — which is sound (the true trajectory is always
// inside the resulting box) but loose exactly where a corner is TANGENT: two
// circles tangent at the corner stay tangent under a consistent offset (the
// same identity LineCircleLocusSpeedUpper's own doc comment derives for a
// line and a circle), so the true velocity is finite there, but this
// decorrelated enclosure cannot tell that persistent tangency from a
// momentary one and refuses on both. LineCircleLocusSpeedUpper below carries
// the exact closed form that tells them apart for a line meeting a circle —
// the common case, reached at every tangent-filleted corner in this
// codebase's own test fixtures — and this generic solve is kept for the
// circle-circle miter it does not cover, on the same reject-only footing
// every other refusal here stands on: never a wrong bound, only a
// conservative one at a tangent circle-circle corner.
func CircleCircleLocusSpeedUpper(prev, cur survey2d.SideWalk, t0, t1, vU, vV float64) (float64, bool) {
	ca, okA := carrierOverRange(prev, t0, t1)
	cb, okB := carrierOverRange(cur, t0, t1)
	if !okA || !okB {
		return 0, false
	}
	cands, okI := Intersect(ca, cb)
	if !okI {
		return 0, false
	}
	foot, okN := Nearest(cands, vU, vV)
	if !okN {
		return 0, false
	}
	rowA, rhsA, okRA := miterConstraintRow(prev, ca, foot)
	rowB, rhsB, okRB := miterConstraintRow(cur, cb, foot)
	if !okRA || !okRB {
		return 0, false
	}
	det := proofbound.IntervalSub(proofbound.IntervalMul(rowA.U, rowB.V), proofbound.IntervalMul(rowB.U, rowA.V))
	if det.Lo.Sign() <= 0 && det.Hi.Sign() >= 0 {
		return 0, false
	}
	pu, okU := proofbound.IntervalQuo(proofbound.IntervalSub(proofbound.IntervalScale(rowB.V, rhsA), proofbound.IntervalScale(rowA.V, rhsB)), det)
	pv, okV := proofbound.IntervalQuo(proofbound.IntervalSub(proofbound.IntervalScale(rowA.U, rhsB), proofbound.IntervalScale(rowB.U, rhsA)), det)
	if !okU || !okV {
		return 0, false
	}
	mag, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(proofbound.IntervalSquare(pu), proofbound.IntervalSquare(pv)))
	if !ok {
		return 0, false
	}
	upper, exact := mag.Hi.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	if proofbound.IsNonFinite(upper) || upper < 0 {
		return 0, false
	}
	return upper, true
}

// lineWallFrame is a straight wall's own EXACT local frame: n is the
// enclosed unit MATERIAL-SIDE normal (rot90 of the enclosed unit tangent)
// and e is the enclosed unit tangent itself, both read once, since a line's
// own direction never moves as its offset amount varies — only its anchor
// point does, along n.
type lineWallFrame struct {
	anchorU, anchorV float64
	n, e             Point
}

// The interval e below deliberately EXCLUDES normalize2(w.tanInU, w.tanInV).
// That is not a hole in the enclosure, and widening e to cover normalize2's
// output would be wrong. UnitVec encloses the EXACT unit vector of the float
// pair — the value normalize2 rounds — and §8.4 requires this frame to hold the
// direction the construction DENOTES "whatever the platform's sqrt and hypot
// did", so anchoring it on a rounded float would replace the denoted direction
// with one particular platform's approximation of it. A tangent of (3, 4) is
// the clearest case: e is the exact [3/5, 3/5] × [4/5, 4/5], 3/5 is not a
// float, and normalize2 lands 2.22e-17 below the interval it is not supposed
// to be in. To falsify this claim, exhibit an admitted corner whose published
// Edge.Length interval fails to enclose the true locus length — not merely one
// where normalize2's output falls outside e, which is every rotated wall.
func lineWallFrameOf(w survey2d.SideWalk) (lineWallFrame, bool) {
	e, ok := UnitVec(w.TanInU, w.TanInV)
	if !ok {
		return lineWallFrame{}, false
	}
	n := Point{U: proofbound.IntervalNeg(e.V), V: e.U}
	return lineWallFrame{anchorU: w.StartU, anchorV: w.StartV, n: n, e: e}, true
}

// LineCircleLocusSpeedUpper bounds |dP/dt| for the corner foot where a
// STRAIGHT wall's own offset carrier meets a CIRCULAR wall's, over the
// offset range [t0, t1], by an EXACT closed form rather than
// CircleCircleLocusSpeedUpper's decorrelated enclosure — which matters
// because this is the common case, reached at every straight-to-arc
// tangent-filleted corner this codebase builds (a rounded rectangle's own
// corners among them), and that decorrelated method refuses on every one of
// them (see CircleCircleLocusSpeedUpper's own doc comment).
//
// Parametrise a point on the offset line as anchor + t·n + s·e (n, e the
// line's own fixed unit normal and tangent — offsetCarrier's own
// construction), and substitute into the offset circle's own
// |X−c| = R − inside·t. Writing w(t) = (anchor + t·n) − c, the quadratic in
// s is s² + 2(w(t)·e)s + (w(t)·w(t) − (R−inside·t)²) = 0, whose
// discriminant is Δ(t) = (w(t)·e)² − w(t)·w(t) + (R−inside·t)². Because
// n·e = 0 and |n| = 1, w(t)·e is CONSTANT (call it β) and w(t)·w(t) is
// R² − α² + ... — expanding fully, Δ(t)'s own t² coefficient is exactly
// 1 − inside² = 0 (inside is ±1), so Δ is EXACTLY AFFINE in t:
// Δ(t) = (R² − α²) − 2(α + inside·R)·t, α = w0·n, w0 = anchor − c.
//
// That affine form is what tells a TANGENT join's own PERSISTENT tangency
// (Δ(t) ≡ 0, both coefficients zero) from a momentary one (Δ0 = 0 but
// Δ1 ≠ 0, a true fold where the speed is genuinely unbounded right at the
// corner): both of s's two roots collapse to the SAME value −β for every t
// when Δ1 = 0, so X(t) = anchor + t·n − β·e is itself affine in t and
// X'(t) = n exactly, a closed-form, finite velocity that never needed a
// square root at all. Where Δ1 ≠ 0, |s'(t)| = |Δ1| / (2·sqrt(Δ(t))), whose
// magnitude does not depend on which of the two roots s(t) actually is — so
// this never needs to pick the branch nearest the corner the way
// intersectOffsets' own POSITION solve does; both roots move at the same
// speed. Δ affine means its minimum over [t0, t1] is at one of the two
// ends, so bounding it needs no search.
//
// ok is false whenever Δ's own enclosure reaches or crosses zero somewhere
// in [t0, t1], WITHOUT Δ1's own enclosure being exactly zero (a momentary
// fold within this range), or any enclosure fails.
//
// A TANGENT join this evaluator itself built — Fillet's own corner rewrite
// (fillet.go), which is exactly what a rounded-rectangle wall's corner is —
// lands Δ1 at EXACTLY zero, bit for bit: the tangent condition
// α = −inside·R the construction holds by is stated in the SAME floats this
// derivation reads, with no residual from a numerical solve to round away.
// A corner recorded through sketch's own solver need not be so exact, and
// Δ1's enclosure straddling (rather than sitting AT) zero there still falls
// through to the general bound below, which is sound but can refuse on a
// near-tangent corner no public fixture reaches today — the PR body for
// this change names that as a known limitation.
func LineCircleLocusSpeedUpper(line, circle survey2d.SideWalk, t0, t1 float64) (float64, bool) {
	frame, ok := lineWallFrameOf(line)
	if !ok {
		return 0, false
	}
	cx, cy := proofarith.FloatRat(circle.CU), proofarith.FloatRat(circle.CV)
	radius := proofarith.FloatRat(circle.Radius)
	anchorU, anchorV := proofarith.FloatRat(frame.anchorU), proofarith.FloatRat(frame.anchorV)
	if cx == nil || cy == nil || radius == nil || anchorU == nil || anchorV == nil {
		return 0, false
	}
	w0u := new(big.Rat).Sub(anchorU, cx)
	w0v := new(big.Rat).Sub(anchorV, cy)
	alpha := proofbound.IntervalAdd(proofbound.IntervalMul(proofbound.PointInterval(w0u), frame.n.U), proofbound.IntervalMul(proofbound.PointInterval(w0v), frame.n.V))
	inside := InsideSignOf(circle)

	// Delta(t) = (R^2 - alpha^2) - 2*(alpha + inside*R)*t = delta0 + delta1*t.
	delta0 := proofbound.IntervalSub(proofbound.IntervalSquare(proofbound.PointInterval(radius)), proofbound.IntervalSquare(alpha))
	delta1 := proofbound.IntervalScale(proofbound.IntervalAdd(alpha, proofbound.IntervalScale(proofbound.PointInterval(radius), inside)), big.NewRat(-2, 1))

	// Delta1 == 0 EXACTLY (both ends of its own enclosure) is the persistent-
	// tangency closed form this function's own doc comment derives: s(t) is
	// then constant, so X'(t) = n exactly and no square root — nor Delta0's
	// own sign, which this branch never even reads — enters the answer. The
	// bound is |n|'s own enclosed magnitude (n is unit by construction, so
	// this is 1 up to UnitVec's own tiny sqrt rounding) rather than
	// proofbound.Radius2D's √2-scaled one, since this specific case is common enough —
	// every tangent-filleted corner in this codebase's own test fixtures —
	// to be worth the tighter bound.
	if delta1.Lo.Sign() == 0 && delta1.Hi.Sign() == 0 {
		nMagUpper := proofbound.RatSqrtUp(proofbound.IntervalAbsUpper(proofbound.IntervalAdd(proofbound.IntervalSquare(frame.n.U), proofbound.IntervalSquare(frame.n.V))))
		if proofbound.IsNonFinite(nMagUpper) {
			return 0, false
		}
		return nMagUpper, true
	}

	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return 0, false
	}
	deltaAt := func(t *big.Rat) proofbound.RatInterval {
		return proofbound.IntervalAdd(delta0, proofbound.IntervalMul(delta1, proofbound.PointInterval(t)))
	}
	d0, d1 := deltaAt(rt0), deltaAt(rt1)
	// Delta is affine, so its minimum over [t0, t1] is at one of the two
	// ends — no interior point needs checking.
	deltaMinLo := d0.Lo
	if d1.Lo.Cmp(deltaMinLo) < 0 {
		deltaMinLo = d1.Lo
	}
	if deltaMinLo.Sign() <= 0 {
		// The discriminant reaches or crosses zero somewhere this enclosure
		// cannot rule out: a genuine fold inside this range, where the
		// position solve's own branch can turn without bound.
		return 0, false
	}
	sqrtLower := proofbound.RatSqrtDown(deltaMinLo)
	if sqrtLower <= 0 || proofbound.IsNonFinite(sqrtLower) {
		return 0, false
	}
	delta1Upper := proofbound.IntervalAbsUpper(delta1)
	delta1UpperF, exact := delta1Upper.Float64()
	if !exact {
		delta1UpperF = math.Nextafter(delta1UpperF, math.Inf(1))
	}
	sPrimeUpper := proofbound.UpRound(delta1UpperF / (2 * sqrtLower))
	if proofbound.IsNonFinite(sPrimeUpper) {
		return 0, false
	}
	// |dP/dt| = sqrt(1 + s'(t)^2), since n and e are orthonormal.
	upper := proofbound.Radius2D(1, sPrimeUpper)
	if proofbound.IsNonFinite(upper) {
		return 0, false
	}
	return upper, true
}

// MiterLocusSpeedUpper bounds |dP/dt| — the in-plane speed of the corner
// foot's own denoted locus (docs/modify-reach-design.md §8.3's exact offset
// family) — over the offset range [t0, t1], for the corner where prev's own
// offset carrier meets cur's, dispatching to the exact closed form
// (LineCircleLocusSpeedUpper) where one carrier is straight and the other
// circular, or the generic interval solve (CircleCircleLocusSpeedUpper)
// where both are circular. At least one of prev, cur must be circular — the
// caller (capSlantEdge) never reaches here for a line-line miter, whose
// locus is already exact.
func MiterLocusSpeedUpper(prev, cur survey2d.SideWalk, t0, t1, vU, vV float64) (float64, bool) {
	switch {
	case !prev.IsCircular() && cur.IsCircular():
		return LineCircleLocusSpeedUpper(prev, cur, t0, t1)
	case prev.IsCircular() && !cur.IsCircular():
		return LineCircleLocusSpeedUpper(cur, prev, t0, t1)
	default:
		return CircleCircleLocusSpeedUpper(prev, cur, t0, t1, vU, vV)
	}
}
