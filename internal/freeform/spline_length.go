package freeform

import (
	"fmt"
	"math"
	"math/big"
	"math/bits"
	"slices"
	"strconv"
	"strings"
	"sync"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is agent-docs/spline-design.md §6.1: the arc length of a free-form curve
// is never exact in any tier, because it integrates a square root that has no
// polynomial antiderivative. What IS available is a PROVEN two-sided bracket:
//
//   - the CHORD is below the arc — the straight line between two points on a
//     curve is no longer than the curve between them;
//   - the CONTROL POLYGON is above it — a Bézier is variation diminishing, so
//     its arc length never exceeds its control polygon's length;
//   - de Casteljau subdivision narrows the gap, at NO guaranteed per-level rate:
//     the familiar 4x is asymptotic, and an early level can do far less — the
//     degree-31 span the tests pin narrows by 1.37x at one of its levels.
//
// Both bounds are proofs, not samples, and the subdivision runs over exact
// rationals so no level introduces error of its own. Only the final square
// roots are floats, and each is rounded OUTWARD against an exact rational
// comparison — so the reported interval encloses the true length whatever the
// platform's sqrt does.

// FreeformLengthDepth is how far the bracket subdivides, and the depth is
// FIXED. This bracket has no target of its own — nothing downstream compares an
// arc length against a caller tolerance — so there is no threshold for a
// measured-target loop to stop on, and the depth is sized to bound both the work
// and the rational denominators the subdivision introduces (they double per
// level) instead. Where a target does exist, agent-docs/spline-design.md §6.1 puts the
// loop there: §6.4's build-time gate and §6.1.1's product enclosure.
//
// A fixed depth promises NO relative width, and nothing here claims one:
// FreeformArcLength sums the ACTUAL leaf brackets at this depth and reports the
// half width of the enclosure it just measured. What that width comes to varies
// with the span, and the spread is wide — an ordinary cubic measures a relative
// half width of 1.9e-7, while the widest span this bracket's own preflight
// admits (32 controls, FreeformBracketCost) measures above 2e-5.
const FreeformLengthDepth = 10

// Bound cached spans across documents so distinct records cannot grow memory
// without limit. Entries leave in insertion order when the cache fills.
const lengthBracketCacheSize = 4096

var lengthBracketCache = struct {
	sync.RWMutex
	values map[string][2]float64
	order  []string
	next   int
}{}

// FreeformArcLength brackets the converted chain's arc length. It returns the
// interval midpoint and its half width, so the caller reports a value with a
// proven bound and NEVER an Exact zero — §6.1 forbids one here.
//
// A zero half width is that forbidden Exact, so the bracket is the gate: a
// curve whose control net has collapsed to a single point is the one shape
// whose control-polygon upper bound is zero, and it refuses as ErrDegenerate
// (Table R row R14) rather than report a length at all. That is the same answer
// the moments path already gives the identical record (freeformDegenerate in
// internal/momentinput/validate.go). Every curve that is not a point brackets strictly wide:
// each subdivision level rounds the lower sum down and the upper sum up, so a
// positive length can never close its own interval.
//
// The other refusal is the interval float64 cannot hold: a curve long enough
// that its proven upper bound runs past MaxFloat64 has no representable
// interval to report, and that is Table R row R15, ErrUnsupported. The test is
// on the ENCLOSURE, so a length just under the top of the range whose upper
// bound is not representable refuses too — refusing an answer decad cannot
// state is right, and the enclosure is the only length in hand. Scale alone
// never refuses otherwise: the square-root seeds work at every scale a finite
// coordinate can reach (proofbound.RatSqrtSeed), so a valid curve is never turned away for
// being merely small or large.
func FreeformArcLength(spans []BezierSpan, work *FreeformWork) (float64, float64, error) {
	lo, hi := 0.0, 0.0
	for _, span := range spans {
		if err := work.Step(FreeformBracketCost(len(span))); err != nil {
			return 0, 0, err
		}
		spanLo, spanHi := SpanLengthBracket(span, FreeformLengthDepth)
		lo = DownRound(lo + spanLo)
		hi = proofbound.UpRound(hi + spanHi)
	}
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || hi < lo {
		return 0, 0, ErrFreeformLengthUnrepresentable
	}
	mid := lo + (hi-lo)/2
	bound := proofbound.UpRound(math.Max(mid-lo, hi-mid))
	if bound <= 0 {
		return 0, 0, ErrFreeformLengthDegenerate
	}
	return mid, bound, nil
}

// ErrFreeformLengthUnrepresentable is Table R row R15: the enclosure is still
// proven, but no float64 interval holds it, so there is no Measurement to
// publish. The sentinel is ErrUnsupported — the curve EXISTS and this evaluator
// cannot report its length — never ErrNotFinite, whose subject is a non-finite
// INPUT while every coordinate reaching here is finite. The guard also catches
// an inverted interval, which the outward rounding makes unreachable and which
// no reading could use either.
var ErrFreeformLengthUnrepresentable = fmt.Errorf(
	`%w: a free-form segment's arc length runs past the representable float64 range`, decaderr.ErrUnsupported,
)

var ErrFreeformLengthDegenerate = fmt.Errorf(
	`%w: a free-form segment whose control points all coincide bounds no arc length`, decaderr.ErrDegenerate,
)

// FreeformBracketCost is the conservative preflight of ONE span's bracket,
// charged before the first subdivision allocates anything.
//
// The cost is driven by the subdivision depth and the span DEGREE together.
// Depth d makes 2ᵈ−1 exact de Casteljau splits and 2ᵈ leaves; one split blends
// all n(n−1)/2 de Casteljau pairs over two coordinates, and one leaf takes its
// chord and its control polygon — n exact square-root brackets between them.
// A charge read off the depth alone counts the leaves and nothing else, which
// lets an arbitrarily wide span through: a single degree-1000 span is 1024
// leaves and over five hundred million rational midpoints.
//
// This preflight is why the fixed-depth bracket's LengthSplitScratch.Split
// carries no counter: the whole subtree is paid for here, before the first
// level allocates anything, and charging each split again would charge the
// same work twice. That split works on the span's n−1 legs, so it blends
// (n−1)(n−2)/2 pairs where this charge counts n(n−1)/2, and the charge stays
// an upper bound on the work it pays for.
//
// ITS UNIT IS NOT THE METERED UNIT. perSplit counts one unit per COORDINATE
// blend — a split's n(n−1)/2 de Casteljau pairs over two coordinates — while
// DyadicSplitOps (spline_sagitta.go) counts the big.Int OPERATIONS the same
// bisection performs, which is 6 per blend plus the split's own allocations and
// copies. So this bracket charges about a third of what the sagitta walk
// charges for the same bisection. That gap is deliberate and is NOT closed
// here: this ceiling is a whole-record one shared with conversion, integration
// and reconstruction (§5.2), the involute fit-spline record already spends 91%
// of it, and raising the per-split term to the metered count would refuse a
// capability that ships today. Reconciling the two units is a §6.1 decision
// about FreeformWorkLimit itself, not an accounting repair.
//
// Saturating arithmetic keeps the estimate an UPPER bound at every size, so an
// oversized span refuses at FreeformWorkLimit instead of wrapping to a small
// charge (work_budget.go).
func FreeformBracketCost(controls int) uint64 {
	if controls < 2 {
		return 0
	}
	n := uint64(controls)
	leaves := uint64(1) << FreeformLengthDepth
	perSplit := CostMul(n, n-1)
	perLeaf := n
	return CostAdd(CostMul(leaves-1, perSplit), CostMul(leaves, perLeaf))
}

// SpanLengthBracket brackets one Bézier span's arc length, subdividing to the
// given depth and summing each piece's chord (below) and control polygon
// (above).
//
// A cache hit at the production depth returns the same bounds for the exact
// same rational controls. On a miss, the span is re-expressed once into the
// split form below; every level under it works in that form.
func SpanLengthBracket(span BezierSpan, depth int) (float64, float64) {
	if depth != FreeformLengthDepth || len(span) < 4 {
		return spanLengthBracket(span, depth)
	}
	// The record work charge precedes this call. Reusing a result skips only
	// arithmetic, so a later walk still consumes its original work budget.
	key := lengthBracketKey(span, depth)
	lengthBracketCache.RLock()
	bound, ok := lengthBracketCache.values[key]
	lengthBracketCache.RUnlock()
	if ok {
		return bound[0], bound[1]
	}
	lo, hi := spanLengthBracket(span, depth)
	lengthBracketCache.Lock()
	if lengthBracketCache.values == nil {
		lengthBracketCache.values = make(map[string][2]float64, lengthBracketCacheSize)
	}
	if _, present := lengthBracketCache.values[key]; !present {
		if len(lengthBracketCache.order) == lengthBracketCacheSize {
			delete(lengthBracketCache.values, lengthBracketCache.order[lengthBracketCache.next])
			lengthBracketCache.order[lengthBracketCache.next] = key
			lengthBracketCache.next = (lengthBracketCache.next + 1) % lengthBracketCacheSize
		} else {
			lengthBracketCache.order = append(lengthBracketCache.order, key)
		}
		lengthBracketCache.values[key] = [2]float64{lo, hi}
	}
	lengthBracketCache.Unlock()
	return lo, hi
}

// lengthBracketKey names every rational control coordinate, so two keys are
// equal exactly when their spans and subdivision depths are equal.
func lengthBracketKey(span BezierSpan, depth int) string {
	var key strings.Builder
	key.Grow(16 + 64*len(span))
	key.WriteString(strconv.Itoa(depth))
	for _, point := range span {
		key.WriteByte('|')
		key.WriteString(point.U.RatString())
		key.WriteByte(',')
		key.WriteString(point.V.RatString())
	}
	return key.String()
}

func spanLengthBracket(span BezierSpan, depth int) (float64, float64) {
	// Unmetered on purpose: FreeformArcLength has already charged this whole
	// span's subtree through FreeformBracketCost, and a nil counter is exactly
	// how FreeformWork.step spells "already accounted for". The error a metered
	// conversion could return is unreachable here for that reason.
	s, _ := DyadicSpanOf(nil, span)
	s.liftDenominatorTwos()
	// This square is part of the bracket's already-paid subtree, not the
	// conversion shared with other dyadic-span consumers.
	s.DenSq = new(big.Int).Mul(s.Den, s.Den)
	if depth <= 0 || len(s.Points) < 2 {
		return s.ChordLower(), s.PolygonUpper()
	}
	net := lengthEdgeNetOf(s)
	frames := make([]LengthSplitScratch, depth)
	for i := range frames {
		frames[i].Init(len(net.Edges))
	}
	return net.LengthBracketScratch(frames, &LengthDistanceScratch{})
}

// liftDenominatorTwos moves the power of two in the span's shared denominator
// into every point's exponent, leaving Den odd. Each value u/(den·2^exp) is
// unchanged, so every exact comparison below decides the same way, and the
// square-root seed reads only the ratio's leading bits and binary exponent,
// which a power-of-two rescaling of numerator and denominator leaves
// identical (agent-docs/spline-design.md §6.1). What changes is the cost: a
// recorded float carries its precision as a power of two, so a converted
// fit-spline span's den can be a small odd factor times 2^160, and every leaf
// comparison multiplies by den². The receiver's points are its own fresh
// conversion, so their exponents are updated in place.
func (s *DyadicSpan) liftDenominatorTwos() {
	twos := s.Den.TrailingZeroBits()
	if twos == 0 {
		return
	}
	s.Den = new(big.Int).Rsh(s.Den, twos)
	for i := range s.Points {
		s.Points[i].Exp += twos
	}
}

// DyadicPoint is one split value: the plane-local coordinate
// (u, v) / (den · 2ᵉˣᵖ), over its span's own shared den.
//
// SUBDIVIDING INTRODUCES NOTHING BUT POWERS OF TWO. Every blend below is a
// MIDPOINT, so a child value is the sum of two values of the same denominator
// halved — the denominator never gains a new odd factor at any depth, which is
// what makes an integer numerator beside a binary exponent the exact form of the
// arithmetic rather than an approximation of it. A midpoint is then one integer
// addition and one exponent increment, with no common factor to strip, where a
// normalising rational pays two GCDs to re-derive a denominator it already knows.
//
// The SPAN's own denominator is factored out because it is NOT a power of two.
// A recorded control coordinate is a float64, hence dyadic, but a converted one
// need not be: Boehm knot insertion divides by a knot difference and the
// closed-spline identity divides by 3 and by 6 (spline_bezier.go), so a
// converted control point carries whatever odd denominator that left it. That
// denominator is a CONSTANT of the whole subdivision, so it is lifted out once
// per span and the exponent carries everything the splitting adds.
type DyadicPoint struct {
	U, V *big.Int
	Exp  uint
}

// DyadicSpan is one Bézier span in that form: its split values beside the
// denominator every one of them shares.
type DyadicSpan struct {
	Points []DyadicPoint
	Den    *big.Int
	// denSq is populated by the arc-length path and shared by its child spans.
	// Other dyadic-span consumers do not need to pay for this cache.
	DenSq *big.Int
}

// ValueWidth is one split value's own operand width in bits: the widest of its
// two integer numerators and of the den·2^exp denominator they are read
// against. It is the shape every charge over a single dyadic value scales by
// (spline_sagitta.go's WidthUnits).
func (s DyadicSpan) ValueWidth(p DyadicPoint) int {
	return max(s.Den.BitLen()+int(p.Exp), p.U.BitLen(), p.V.BitLen())
}

// SpanWidth is the widest operand width any of the span's own values carries —
// the shape a whole-span charge scales by.
func (s DyadicSpan) SpanWidth() int {
	widest := s.Den.BitLen()
	for _, p := range s.Points {
		widest = max(widest, s.ValueWidth(p))
	}
	return widest
}

// DyadicSpanOf re-expresses a converted span over one shared denominator. The
// denominator is the least common multiple of the span's own, so every
// coordinate lifts to an EXACT integer numerator and the span it describes is
// the span it was given, to the last bit.
//
// It is the ONE entry point for that conversion, and it meters itself: it
// charges DyadicSpanOfCharge (spline_sagitta.go) as its first statement and
// returns FreeformWork.step's own refusal, having converted nothing, when the
// counter cannot cover it. A nil counter is unmetered, which is what the
// fixed-depth arc-length bracket passes under its own FreeformBracketCost
// preflight.
func DyadicSpanOf(w *FreeformWork, span BezierSpan) (DyadicSpan, error) {
	if err := w.Step(DyadicSpanOfCharge(span)); err != nil {
		return DyadicSpan{}, err
	}
	den := big.NewInt(1)
	for _, point := range span {
		den = RatLCM(den, new(big.Int).Set(point.U.Denom()))
		den = RatLCM(den, new(big.Int).Set(point.V.Denom()))
	}
	points := make([]DyadicPoint, len(span))
	for i, point := range span {
		points[i] = DyadicPoint{U: ScaledNumerator(point.U, den), V: ScaledNumerator(point.V, den)}
	}
	return DyadicSpan{Points: points, Den: den}, nil
}

// RatLCM returns the least common multiple of two positive integers. A big.Rat
// denominator is always positive, so no sign case arises.
func RatLCM(a, b *big.Int) *big.Int {
	gcd := new(big.Int).GCD(nil, nil, a, b)
	out := new(big.Int).Quo(a, gcd)
	return out.Mul(out, b)
}

// ScaledNumerator returns r·den as an exact integer. den is a common multiple
// of every denominator in the span, so the division leaves no remainder.
func ScaledNumerator(r *big.Rat, den *big.Int) *big.Int {
	out := new(big.Int).Quo(den, r.Denom())
	return out.Mul(out, r.Num())
}

// LengthEdge is one control-polygon leg b−a of a span in the bracket's edge
// form: its two integer numerators over the net's shared den · 2^Exp.
type LengthEdge struct {
	U, V big.Int
}

// LengthEdgeNet is a span's control polygon held as its legs instead of its
// points. Every bound the bracket takes reads a leg or the chord, and the chord
// is the sum of the legs, so the points themselves are never needed below the
// conversion (agent-docs/spline-design.md §6.1). All legs share Exp, and each
// one's numerator is the exact integer the point form's own aligned difference
// gives at that exponent, because den and the exponent fix the numerator of a
// value uniquely.
type LengthEdgeNet struct {
	Edges []LengthEdge
	Exp   uint
	DenSq *big.Int
}

// lengthEdgeNetOf takes the legs of the bracket's own lifted conversion.
// DyadicSpanOf gives every point exponent zero and liftDenominatorTwos raises
// them all by the same count, so the points share one exponent and each leg is
// a plain difference of numerators.
func lengthEdgeNetOf(s DyadicSpan) LengthEdgeNet {
	edges := make([]LengthEdge, len(s.Points)-1)
	for i := range edges {
		a, b := s.Points[i], s.Points[i+1]
		edges[i].U.Sub(b.U, a.U)
		edges[i].V.Sub(b.V, a.V)
	}
	return LengthEdgeNet{Edges: edges, Exp: s.Points[0].Exp, DenSq: s.DenSq}
}

// LengthSplitScratch owns the two child leg nets one subdivision level
// produces. Its integer storage remains live for the whole depth-first walk,
// so sibling subtrees reuse it without changing the order of leaf summation.
type LengthSplitScratch struct {
	Left, Right []LengthEdge
}

// Init sizes the scratch for nets of m legs.
func (f *LengthSplitScratch) Init(m int) {
	f.Left = make([]LengthEdge, m)
	f.Right = make([]LengthEdge, m)
}

// LengthBracketScratch brackets the net to len(frames) further levels. It
// consumes the net: each level's split works in its input's storage, which the
// walk never reads again once the children are taken.
func (n LengthEdgeNet) LengthBracketScratch(
	frames []LengthSplitScratch, distance *LengthDistanceScratch,
) (float64, float64) {
	if len(frames) == 0 {
		return n.ChordLowerScratch(distance), n.PolygonUpperScratch(distance)
	}
	f := &frames[len(frames)-1]
	f.Split(n.Edges)
	exp := n.Exp + uint(len(n.Edges))
	left := LengthEdgeNet{Edges: f.Left, Exp: exp, DenSq: n.DenSq}
	right := LengthEdgeNet{Edges: f.Right, Exp: exp, DenSq: n.DenSq}
	leftLo, leftHi := left.LengthBracketScratch(frames[:len(frames)-1], distance)
	rightLo, rightHi := right.LengthBracketScratch(frames[:len(frames)-1], distance)
	return DownRound(leftLo + rightLo), proofbound.UpRound(leftHi + rightHi)
}

// Split is the de Casteljau bisection at t = 1/2 of a leg net, read at the
// point form's child exponent: the input's exponent plus m, the leg count.
//
// The legs of a Bézier's halves are the halves of the de Casteljau blends of
// its legs. With W_k the k-th round of neighbour sums taken WITHOUT halving
// (so W_k is 2^k times the true blend), the left child's leg k is
// W_k[0] / 2^(k+1) and the right child's leg m−1−k is W_k[m−1−k] / 2^(k+1).
// Read at the input exponent plus m, each is that work entry shifted up by
// m−1−k, which is never negative. The halves' points therefore have exactly
// these legs, at exactly the exponent the point form gives them.
//
// The input is the work storage, so Split consumes it.
func (f *LengthSplitScratch) Split(in []LengthEdge) {
	m := len(in)
	for k := range m {
		if k > 0 {
			for i := range m - k {
				in[i].U.Add(&in[i].U, &in[i+1].U)
				in[i].V.Add(&in[i].V, &in[i+1].V)
			}
		}
		shift := uint(m - 1 - k)
		last := m - 1 - k
		f.Left[k].U.Lsh(&in[0].U, shift)
		f.Left[k].V.Lsh(&in[0].V, shift)
		f.Right[last].U.Lsh(&in[last].U, shift)
		f.Right[last].V.Lsh(&in[last].V, shift)
	}
}

// split halves a span by de Casteljau at t = 1/2, exactly: every blend is a
// midpoint, so the arithmetic is a binary bisection over integer numerators.
//
// It is the ONE entry point for that bisection, and it meters itself: it
// charges DyadicSplitOps (spline_sagitta.go) at its own operand width as its
// first statement — the span's control count and its widest value are the
// receiver's own shape, never a caller's loop bound — and returns
// FreeformWork.step's own refusal, having split nothing, when the counter
// cannot cover it. A nil counter is unmetered, which is what the fixed-depth
// arc-length bracket passes under its own FreeformBracketCost preflight.
func (s DyadicSpan) Split(w *FreeformWork) (DyadicSpan, DyadicSpan, error) {
	n := len(s.Points)
	if err := w.Step(CostMul(DyadicSplitOps(uint64(n)), WidthUnits(s.SpanWidth()))); err != nil {
		return DyadicSpan{}, DyadicSpan{}, err
	}
	work := make([]DyadicPoint, n)
	copy(work, s.Points)
	left := make([]DyadicPoint, 0, n)
	right := make([]DyadicPoint, n)
	left = append(left, work[0])
	right[n-1] = work[n-1]
	for round := n - 1; round > 0; round-- {
		for i := range round {
			work[i] = DyadicMidpoint(work[i], work[i+1])
		}
		left = append(left, work[0])
		right[round-1] = work[round-1]
	}
	return DyadicSpan{Points: left, Den: s.Den, DenSq: s.DenSq},
		DyadicSpan{Points: right, Den: s.Den, DenSq: s.DenSq}, nil
}

// DyadicMidpoint is (a+b)/2 exactly: the two numerators are raised to their
// common exponent, added, and the exponent goes up by one for the halving.
func DyadicMidpoint(a, b DyadicPoint) DyadicPoint {
	exp := max(a.Exp, b.Exp)
	return DyadicPoint{
		U:   AlignedSum(a.U, exp-a.Exp, b.U, exp-b.Exp),
		V:   AlignedSum(a.V, exp-a.Exp, b.V, exp-b.Exp),
		Exp: exp + 1,
	}
}

// AlignedSum is a+b with each numerator first shifted up to the common
// exponent. Shifting is exact, so the sum is the sum of the two values.
func AlignedSum(a *big.Int, aShift uint, b *big.Int, bShift uint) *big.Int {
	out := new(big.Int).Lsh(a, aShift)
	if bShift == 0 {
		return out.Add(out, b)
	}
	return out.Add(out, new(big.Int).Lsh(b, bShift))
}

// AlignedDifference is a−b under the same alignment.
func AlignedDifference(a *big.Int, aShift uint, b *big.Int, bShift uint) *big.Int {
	out := new(big.Int).Lsh(a, aShift)
	if bShift == 0 {
		return out.Sub(out, b)
	}
	return out.Sub(out, new(big.Int).Lsh(b, bShift))
}

// chordLower is a proven lower bound on the distance between the span's two
// ends: the largest float whose square does not exceed the exact squared
// distance.
func (s DyadicSpan) ChordLower() float64 {
	return SpanSqrtDown(s.DistanceSquared(s.Points[0], s.Points[len(s.Points)-1]))
}

// PolygonUpper is a proven upper bound on a control polygon's length.
func (s DyadicSpan) PolygonUpper() float64 {
	total := 0.0
	for i := 0; i+1 < len(s.Points); i++ {
		total = proofbound.UpRound(total + SpanSqrtUp(s.DistanceSquared(s.Points[i], s.Points[i+1])))
	}
	return total
}

// ChordLowerScratch is ChordLower on a leg net: the chord is the sum of the
// legs, an exact integer sum at the net's shared exponent.
func (n LengthEdgeNet) ChordLowerScratch(scratch *LengthDistanceScratch) float64 {
	scratch.Du.Set(&n.Edges[0].U)
	scratch.Dv.Set(&n.Edges[0].V)
	for i := 1; i < len(n.Edges); i++ {
		scratch.Du.Add(&scratch.Du, &n.Edges[i].U)
		scratch.Dv.Add(&scratch.Dv, &n.Edges[i].V)
	}
	return SpanSqrtDownScratch(n.squaredLength(&scratch.Du, &scratch.Dv, scratch), scratch)
}

// PolygonUpperScratch is PolygonUpper on a leg net.
func (n LengthEdgeNet) PolygonUpperScratch(scratch *LengthDistanceScratch) float64 {
	total := 0.0
	for i := range n.Edges {
		// Uniform halves of one Bézier have the same derivative at their
		// shared endpoint. Their adjoining control edges therefore have
		// equal exact lengths, so the previous leaf's last upper bound is
		// this leaf's first upper bound. Keep every outward sum in place.
		edge := scratch.LastUpper
		if i != 0 || !scratch.HasLastUpper {
			e := &n.Edges[i]
			edge = SpanSqrtUpScratch(n.squaredLength(&e.U, &e.V, scratch), scratch)
		}
		total = proofbound.UpRound(total + edge)
		if i+1 == len(n.Edges) {
			scratch.LastUpper = edge
			scratch.HasLastUpper = true
		}
	}
	return total
}

// squaredLength is |(du, dv)|² over the net's denominator, written into the
// scratch's Num.
func (n LengthEdgeNet) squaredLength(du, dv *big.Int, scratch *LengthDistanceScratch) SpanSquaredDistance {
	scratch.Num.Mul(du, du)
	scratch.Tmp.Mul(dv, dv)
	scratch.Num.Add(&scratch.Num, &scratch.Tmp)
	return SpanSquaredDistance{Num: &scratch.Num, DenSq: n.DenSq, Exp: n.Exp}
}

// LengthDistanceScratch reuses exact integer operands across successive leaf
// distances. A leaf completes each square-root comparison before the next
// distance overwrites num, so no returned interval holds one of these values.
type LengthDistanceScratch struct {
	Du, Dv, Tmp, Num, Lhs, SquareMant     big.Int
	LhsWords                              [3]big.Word
	SeedMant, SeedRatio, SeedNum, SeedDen big.Float
	// SpanLengthBracket creates one scratch per original span, so this
	// bound is never reused across unrelated control nets.
	LastUpper    float64
	HasLastUpper bool
}

// SpanSquaredDistance is |b−a|² = num / (denSq · 2^(2 exp)). Keeping the
// denominator in this form lets the leaf square roots compare integers without
// reducing a new rational for every control-polygon leg.
type SpanSquaredDistance struct {
	Num, DenSq *big.Int
	Exp        uint
}

func (s DyadicSpan) DistanceSquared(a, b DyadicPoint) SpanSquaredDistance {
	exp := max(a.Exp, b.Exp)
	du := AlignedDifference(b.U, exp-b.Exp, a.U, exp-a.Exp)
	dv := AlignedDifference(b.V, exp-b.Exp, a.V, exp-a.Exp)
	num := du.Mul(du, du)
	num.Add(num, dv.Mul(dv, dv))
	denSq := s.DenSq
	if denSq == nil {
		denSq = new(big.Int).Mul(s.Den, s.Den)
	}
	return SpanSquaredDistance{Num: num, DenSq: denSq, Exp: exp}
}

// SpanSquareCmp compares f² to the exact squared distance without a rational
// reduction. A finite float is an integer mantissa times a power of two, so
// multiplying by the shared denSq and shifting preserves the exact ordering.
func SpanSquareCmp(f float64, d SpanSquaredDistance) int {
	square, ok := proofarith.DyOf(f)
	if !ok {
		return 1
	}
	if square.IsZero() {
		return -d.Num.Sign()
	}
	mant := square.Mant()
	lhs := new(big.Int).Mul(mant, mant)
	lhs.Mul(lhs, d.DenSq)
	shift := 2 * (square.Exp() + int(d.Exp))
	if shift >= 0 {
		return lhs.Lsh(lhs, uint(shift)).Cmp(d.Num)
	}
	return lhs.Cmp(new(big.Int).Lsh(d.Num, uint(-shift)))
}

// SpanSquareCmpScratch is SpanSquareCmp deciding the same exact integer
// comparison, mant²·denSq·2^shift against num, without materialising either
// shifted operand: shiftedCmp reads the shift off the two bit lengths and then
// the words themselves. Where denSq is one 64-bit word — the odd den the
// bracket lifts its powers of two out of is usually a small one — the
// left-hand product is formed on machine words. f's mantissa need not be
// reduced to odd for this: any integer m with |f| = m·2^e gives the same
// product.
func SpanSquareCmpScratch(f float64, d SpanSquaredDistance, scratch *LengthDistanceScratch) int {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return 1
	}
	if f == 0 {
		return -d.Num.Sign()
	}
	if d.Num.Sign() < 0 {
		// The left-hand side is positive. A sum of squares never takes this
		// branch; it keeps the comparison total.
		return 1
	}
	den := d.DenSq.Bits()
	if bits.UintSize == 64 && len(den) == 1 {
		mag, exp := floatMagnitude(f)
		hi, lo := bits.Mul64(mag, mag)
		w := uint64(den[0])
		c0, r0 := bits.Mul64(lo, w)
		c1, r1 := bits.Mul64(hi, w)
		r1, carry := bits.Add64(r1, c0, 0)
		// mag < 2^53 and w < 2^64, so the product is below 2^170 and r2
		// cannot wrap.
		r2 := c1 + carry
		lhs := scratch.LhsWords[:3]
		lhs[0], lhs[1], lhs[2] = big.Word(r0), big.Word(r1), big.Word(r2)
		for len(lhs) > 0 && lhs[len(lhs)-1] == 0 {
			lhs = lhs[:len(lhs)-1]
		}
		return orderedShiftCmp(lhs, 2*(exp+int(d.Exp)), d.Num.Bits())
	}
	square := proofarith.DyOfFinite(f)
	mant := square.MantInto(&scratch.SquareMant)
	scratch.Lhs.Mul(mant, mant)
	scratch.Lhs.Mul(&scratch.Lhs, d.DenSq)
	return orderedShiftCmp(scratch.Lhs.Bits(), 2*(square.Exp()+int(d.Exp)), d.Num.Bits())
}

// floatMagnitude is |f| = mag·2^exp for a finite non-zero f, read off its
// bits: the significand with its hidden bit, and the unbiased exponent of its
// last place.
func floatMagnitude(f float64) (uint64, int) {
	b := math.Float64bits(f)
	mag, exp := b&(1<<52-1), int(b>>52&0x7ff)
	if exp == 0 {
		return mag, -1074
	}
	return mag | 1<<52, exp - 1075
}

// orderedShiftCmp compares a·2^shift against b for non-negative magnitudes in
// big.Int's normalised little-endian word form; a negative shift scales b
// instead.
func orderedShiftCmp(a []big.Word, shift int, b []big.Word) int {
	if shift >= 0 {
		return shiftedCmp(a, uint(shift), b)
	}
	return -shiftedCmp(b, uint(-shift), a)
}

// shiftedCmp compares a·2^s against b, both normalised magnitudes, exactly.
// Unequal bit lengths decide it at once; equal ones compare word by word from
// the top, each word of a·2^s assembled from the two words of a it straddles.
func shiftedCmp(a []big.Word, s uint, b []big.Word) int {
	la, lb := wordsBitLen(a), wordsBitLen(b)
	if la == 0 {
		// A zero a·2^s ties a zero b and is below any other. A zero b
		// against a positive a is left to the width test below.
		if lb == 0 {
			return 0
		}
		return -1
	}
	if width := uint64(la) + uint64(s); width != uint64(lb) {
		if width < uint64(lb) {
			return -1
		}
		return 1
	}
	ws, bs := int(s/bits.UintSize), s%bits.UintSize
	for i, bw := range slices.Backward(b) {
		var w big.Word
		if j := i - ws; j >= 0 && j < len(a) {
			w = a[j] << bs
		}
		// At bs = 0 the shift below is by the full word width, which Go
		// defines as zero.
		if j := i - ws - 1; j >= 0 && j < len(a) {
			w |= a[j] >> (bits.UintSize - bs)
		}
		if w != bw {
			if w < bw {
				return -1
			}
			return 1
		}
	}
	return 0
}

// wordsBitLen is the bit length of a normalised magnitude.
func wordsBitLen(x []big.Word) int {
	if len(x) == 0 {
		return 0
	}
	return (len(x)-1)*bits.UintSize + bits.Len(uint(x[len(x)-1]))
}

// SpanSqrtSeed matches proofbound.RatSqrtSeed's 64-bit quotient, keeping the
// power-of-two denominator as an exponent. Integer prefixes determine the
// rounded mantissa when RatioSqrtSeed succeeds; otherwise full-precision
// big.Float division supplies it.
func SpanSqrtSeed(d SpanSquaredDistance) float64 {
	if seed, ok := proofarith.RatioSqrtSeed(d.Num, d.DenSq, 2*int(d.Exp)); ok {
		return seed
	}
	mant := new(big.Float).SetPrec(64)
	ratio := new(big.Float).SetPrec(64).Quo(
		new(big.Float).SetInt(d.Num), new(big.Float).SetInt(d.DenSq),
	)
	exp := ratio.MantExp(mant) - 2*int(d.Exp)
	if exp%2 != 0 {
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

// SpanSqrtSeedScratch uses the same prefix shortcut as SpanSqrtSeed. In the
// full-division fallback, each integer input starts with zero precision so
// SetInt gives it its full bit length, as a new big.Float would. The quotient
// and mantissa use 64 bits, reusing storage across this bracket's leaf legs.
func SpanSqrtSeedScratch(d SpanSquaredDistance, scratch *LengthDistanceScratch) float64 {
	if seed, ok := proofarith.RatioSqrtSeed(d.Num, d.DenSq, 2*int(d.Exp)); ok {
		return seed
	}
	scratch.SeedMant.SetPrec(64)
	scratch.SeedRatio.SetPrec(64)
	ratio := scratch.SeedRatio.Quo(
		scratch.SeedNum.SetPrec(0).SetInt(d.Num),
		scratch.SeedDen.SetPrec(0).SetInt(d.DenSq),
	)
	exp := ratio.MantExp(&scratch.SeedMant) - 2*int(d.Exp)
	if exp%2 != 0 {
		exp--
		scratch.SeedMant.SetMantExp(&scratch.SeedMant, 1)
	}
	m, _ := scratch.SeedMant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

func SpanSqrtDown(d SpanSquaredDistance) float64 {
	if d.Num.Sign() <= 0 {
		return 0
	}
	f := SpanSqrtSeed(d)
	if proofbound.IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if SpanSquareCmp(f, d) <= 0 {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

func SpanSqrtDownScratch(d SpanSquaredDistance, scratch *LengthDistanceScratch) float64 {
	if d.Num.Sign() <= 0 {
		return 0
	}
	f := SpanSqrtSeedScratch(d, scratch)
	if proofbound.IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if SpanSquareCmpScratch(f, d, scratch) <= 0 {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

func SpanSqrtUp(d SpanSquaredDistance) float64 {
	if d.Num.Sign() <= 0 {
		return 0
	}
	f := SpanSqrtSeed(d)
	if proofbound.IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if SpanSquareCmp(f, d) >= 0 {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

func SpanSqrtUpScratch(d SpanSquaredDistance, scratch *LengthDistanceScratch) float64 {
	if d.Num.Sign() <= 0 {
		return 0
	}
	f := SpanSqrtSeedScratch(d, scratch)
	if proofbound.IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if SpanSquareCmpScratch(f, d, scratch) >= 0 {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

// DownRound is proofbound.UpRound's mirror: the next float toward zero, so a sum of lower
// bounds stays a lower bound.
func DownRound(x float64) float64 {
	if x <= 0 || proofbound.IsNonFinite(x) {
		return x
	}
	return proofarith.NextDown(x)
}
