package freeform

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is docs/spline-design.md §6.1: the arc length of a free-form curve
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
// level) instead. Where a target does exist, docs/spline-design.md §6.1 puts the
// loop there: §6.4's build-time gate and §6.1.1's product enclosure.
//
// A fixed depth promises NO relative width, and nothing here claims one:
// FreeformArcLength sums the ACTUAL leaf brackets at this depth and reports the
// half width of the enclosure it just measured. What that width comes to varies
// with the span, and the spread is wide — an ordinary cubic measures a relative
// half width of 1.9e-7, while the widest span this bracket's own preflight
// admits (32 controls, FreeformBracketCost) measures above 2e-5.
const FreeformLengthDepth = 10

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
func FreeformArcLength(spans []survey2d.BezierSpan, work *FreeformWork) (float64, float64, error) {
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
// This preflight is why the fixed-depth bracket calls DyadicSpan.split
// UNMETERED: the whole subtree is paid for here, before the first level
// allocates anything, and charging each split again would charge the same work
// twice.
//
// ITS UNIT IS NOT THE METERED UNIT. perSplit counts one unit per COORDINATE
// blend — a split's n(n−1)/2 de Casteljau pairs over two coordinates — while
// DyadicSplitOps (spline_sagitta.go) counts the big.Int OPERATIONS the same
// bisection performs, which is 6 per blend plus the split's own allocations and
// copies. So this bracket charges about a third of what the sagitta walk
// charges for the identical code. That gap is deliberate and is NOT closed
// here: this ceiling is a whole-record one shared with conversion, integration
// and reconstruction (§5.2), the involute fit-spline record already spends 91%
// of it, and raising the per-split term to the metered count would refuse a
// capability that ships today. Reconciling the two units is a §6.1 decision
// about FreeformWorkLimit itself, not an accounting repair.
//
// Saturating arithmetic keeps the estimate an UPPER bound at every size, so an
// oversized span refuses at FreeformWorkLimit instead of wrapping to a small
// charge (spline_bezier.go).
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
// The span is re-expressed once, here, into the split form below; every level
// under it works in that form and only the leaves come back out as rationals.
func SpanLengthBracket(span survey2d.BezierSpan, depth int) (float64, float64) {
	// Unmetered on purpose: FreeformArcLength has already charged this whole
	// span's subtree through FreeformBracketCost, and a nil counter is exactly
	// how FreeformWork.step spells "already accounted for". The error a metered
	// conversion could return is unreachable here for that reason.
	s, _ := DyadicSpanOf(nil, span)
	// This square is part of the bracket's already-paid subtree, not the
	// conversion shared with other dyadic-span consumers.
	s.DenSq = new(big.Int).Mul(s.Den, s.Den)
	if depth <= 0 || len(s.Points) < 2 {
		return s.ChordLower(), s.PolygonUpper()
	}
	frames := make([]LengthSplitScratch, depth)
	for i := range frames {
		frames[i].Init(len(s.Points))
	}
	return s.LengthBracketScratch(frames, &LengthDistanceScratch{})
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

// valueWidth is one split value's own operand width in bits: the widest of its
// two integer numerators and of the den·2^exp denominator they are read
// against. It is the shape every charge over a single dyadic value scales by
// (spline_sagitta.go's WidthUnits).
func (s DyadicSpan) ValueWidth(p DyadicPoint) int {
	return max(s.Den.BitLen()+int(p.Exp), p.U.BitLen(), p.V.BitLen())
}

// spanWidth is the widest operand width any of the span's own values carries —
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
func DyadicSpanOf(w *FreeformWork, span survey2d.BezierSpan) (DyadicSpan, error) {
	if err := w.Step(DyadicSpanOfCharge(span)); err != nil {
		return DyadicSpan{}, err
	}
	den := big.NewInt(1)
	for _, point := range span {
		den = RatLCM(den, point.U.Denom())
		den = RatLCM(den, point.V.Denom())
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

func (s DyadicSpan) LengthBracket(depth int) (float64, float64) {
	if depth == 0 || len(s.Points) < 2 {
		return s.ChordLower(), s.PolygonUpper()
	}
	// Unmetered: FreeformBracketCost already charged every split in this
	// subtree up front (its own doc comment), so a nil counter cannot refuse.
	left, right, _ := s.Split(nil)
	leftLo, leftHi := left.LengthBracket(depth - 1)
	rightLo, rightHi := right.LengthBracket(depth - 1)
	return DownRound(leftLo + rightLo), proofbound.UpRound(leftHi + rightHi)
}

// LengthSplitScratch owns the three control nets used at one subdivision
// level. Its integer storage remains live for the whole depth-first walk, so
// sibling subtrees reuse it without changing the order of leaf summation.
type LengthSplitScratch struct {
	Work, Left, Right []DyadicPoint
	Tmp               big.Int
}

func (f *LengthSplitScratch) Init(n int) {
	f.Work = make([]DyadicPoint, n)
	f.Left = make([]DyadicPoint, n)
	f.Right = make([]DyadicPoint, n)
	for _, points := range [][]DyadicPoint{f.Work, f.Left, f.Right} {
		for i := range points {
			points[i].U = new(big.Int)
			points[i].V = new(big.Int)
		}
	}
}

func (s DyadicSpan) LengthBracketScratch(
	frames []LengthSplitScratch, distance *LengthDistanceScratch,
) (float64, float64) {
	if len(frames) == 0 {
		return s.ChordLowerScratch(distance), s.PolygonUpperScratch(distance)
	}
	f := &frames[len(frames)-1]
	f.Split(s.Points)
	left := DyadicSpan{Points: f.Left, Den: s.Den, DenSq: s.DenSq}
	right := DyadicSpan{Points: f.Right, Den: s.Den, DenSq: s.DenSq}
	leftLo, leftHi := left.LengthBracketScratch(frames[:len(frames)-1], distance)
	rightLo, rightHi := right.LengthBracketScratch(frames[:len(frames)-1], distance)
	return DownRound(leftLo + rightLo), proofbound.UpRound(leftHi + rightHi)
}

func (f *LengthSplitScratch) Split(points []DyadicPoint) {
	n := len(points)
	for i, p := range points {
		f.Work[i].U.Set(p.U)
		f.Work[i].V.Set(p.V)
		f.Work[i].Exp = p.Exp
	}
	f.Left[0].U.Set(f.Work[0].U)
	f.Left[0].V.Set(f.Work[0].V)
	f.Left[0].Exp = f.Work[0].Exp
	f.Right[n-1].U.Set(f.Work[n-1].U)
	f.Right[n-1].V.Set(f.Work[n-1].V)
	f.Right[n-1].Exp = f.Work[n-1].Exp
	for round := n - 1; round > 0; round-- {
		for i := range round {
			MidpointInto(&f.Work[i], f.Work[i], f.Work[i+1], &f.Tmp)
		}
		f.Left[n-round].U.Set(f.Work[0].U)
		f.Left[n-round].V.Set(f.Work[0].V)
		f.Left[n-round].Exp = f.Work[0].Exp
		f.Right[round-1].U.Set(f.Work[round-1].U)
		f.Right[round-1].V.Set(f.Work[round-1].V)
		f.Right[round-1].Exp = f.Work[round-1].Exp
	}
}

func MidpointInto(dst *DyadicPoint, a, b DyadicPoint, tmp *big.Int) {
	exp := max(a.Exp, b.Exp)
	dst.U.Lsh(a.U, exp-a.Exp)
	tmp.Lsh(b.U, exp-b.Exp)
	dst.U.Add(dst.U, tmp)
	dst.V.Lsh(a.V, exp-a.Exp)
	tmp.Lsh(b.V, exp-b.Exp)
	dst.V.Add(dst.V, tmp)
	dst.Exp = exp + 1
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

func (s DyadicSpan) ChordLowerScratch(scratch *LengthDistanceScratch) float64 {
	return SpanSqrtDownScratch(s.DistanceSquaredScratch(s.Points[0], s.Points[len(s.Points)-1], scratch), scratch)
}

// polygonUpper is a proven upper bound on a control polygon's length.
func (s DyadicSpan) PolygonUpper() float64 {
	total := 0.0
	for i := 0; i+1 < len(s.Points); i++ {
		total = proofbound.UpRound(total + SpanSqrtUp(s.DistanceSquared(s.Points[i], s.Points[i+1])))
	}
	return total
}

func (s DyadicSpan) PolygonUpperScratch(scratch *LengthDistanceScratch) float64 {
	total := 0.0
	for i := 0; i+1 < len(s.Points); i++ {
		// Uniform halves of one Bézier have the same derivative at their
		// shared endpoint. Their adjoining control edges therefore have
		// equal exact lengths, so the previous leaf's last upper bound is
		// this leaf's first upper bound. Keep every outward sum in place.
		edge := scratch.LastUpper
		if i != 0 || !scratch.HasLastUpper {
			edge = SpanSqrtUpScratch(s.DistanceSquaredScratch(s.Points[i], s.Points[i+1], scratch), scratch)
		}
		total = proofbound.UpRound(total + edge)
		if i+2 == len(s.Points) {
			scratch.LastUpper = edge
			scratch.HasLastUpper = true
		}
	}
	return total
}

// LengthDistanceScratch reuses exact integer operands across successive leaf
// distances. A leaf completes each square-root comparison before the next
// distance overwrites num, so no returned interval holds one of these values.
type LengthDistanceScratch struct {
	Du, Dv, Tmp, Num, Lhs, Rhs, SquareMant big.Int
	SeedMant, SeedRatio, SeedNum, SeedDen  big.Float
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

func (s DyadicSpan) DistanceSquaredScratch(
	a, b DyadicPoint, scratch *LengthDistanceScratch,
) SpanSquaredDistance {
	exp := max(a.Exp, b.Exp)
	scratch.Du.Lsh(b.U, exp-b.Exp)
	scratch.Tmp.Lsh(a.U, exp-a.Exp)
	scratch.Du.Sub(&scratch.Du, &scratch.Tmp)
	scratch.Dv.Lsh(b.V, exp-b.Exp)
	scratch.Tmp.Lsh(a.V, exp-a.Exp)
	scratch.Dv.Sub(&scratch.Dv, &scratch.Tmp)
	scratch.Num.Mul(&scratch.Du, &scratch.Du)
	scratch.Tmp.Mul(&scratch.Dv, &scratch.Dv)
	scratch.Num.Add(&scratch.Num, &scratch.Tmp)
	return SpanSquaredDistance{Num: &scratch.Num, DenSq: s.DenSq, Exp: exp}
}

// squaredDistance is the rational reference for callers that need the exact
// squared distance rather than a directed float square root.
func (s DyadicSpan) SquaredDistance(a, b DyadicPoint) *big.Rat {
	d := s.DistanceSquared(a, b)
	return new(big.Rat).SetFrac(d.Num, new(big.Int).Lsh(d.DenSq, 2*d.Exp))
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

func SpanSquareCmpScratch(f float64, d SpanSquaredDistance, scratch *LengthDistanceScratch) int {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return 1
	}
	if f == 0 {
		return -d.Num.Sign()
	}
	square := proofarith.DyOfFinite(f)
	mant := square.MantInto(&scratch.SquareMant)
	scratch.Lhs.Mul(mant, mant)
	scratch.Lhs.Mul(&scratch.Lhs, d.DenSq)
	shift := 2 * (square.Exp() + int(d.Exp))
	if shift >= 0 {
		return scratch.Lhs.Lsh(&scratch.Lhs, uint(shift)).Cmp(d.Num)
	}
	return scratch.Lhs.Cmp(scratch.Rhs.Lsh(d.Num, uint(-shift)))
}

// SpanSqrtSeed follows proofbound.RatSqrtSeed's 64-bit big.Float quotient, while keeping
// the power-of-two part of the denominator as an exponent. SetRat uses the same
// full-precision integer operands and Quo for a noninteger rational.
func SpanSqrtSeed(d SpanSquaredDistance) float64 {
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

// SpanSqrtSeedScratch uses the same precision and rounding as SpanSqrtSeed.
// Each integer input starts with zero precision so SetInt gives it its full
// bit length, as a newly allocated big.Float would. The quotient and mantissa
// use 64 bits, while their storage is reused across this bracket's leaf legs.
func SpanSqrtSeedScratch(d SpanSquaredDistance, scratch *LengthDistanceScratch) float64 {
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
	return math.Nextafter(x, 0)
}
