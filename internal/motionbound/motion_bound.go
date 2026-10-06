package motionbound

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MotionParam is one motion parameter's exact denotation. An angle denotes
// θ = 2π·turn + base radians: a degree-stated angle is an exact rational turn
// (units.Degree's own factor is a rounded π/180, so the degree count, not the
// factor, is what the caller stated), and any other angle unit is base =
// magnitude × factor radians, read exactly. A length denotes base millimetres
// and leaves turn zero, and a Between's dimensionless fraction denotes base
// and leaves turn zero.
type MotionParam struct {
	Turn *big.Rat
	Base *big.Rat
}

// ExactMotionParam reads v's exact denotation. It fails only on a non-finite
// magnitude or factor.
func ExactMotionParam(v units.Value) (MotionParam, bool) {
	mag := proofarith.FloatRat(v.Mag())
	if mag == nil {
		return MotionParam{}, false
	}
	if v.Unit() == units.Degree {
		return MotionParam{Turn: new(big.Rat).Quo(mag, big.NewRat(360, 1)), Base: new(big.Rat)}, true
	}
	factor := proofarith.FloatRat(v.Unit().Factor())
	if factor == nil {
		return MotionParam{}, false
	}
	return MotionParam{Turn: new(big.Rat), Base: new(big.Rat).Mul(mag, factor)}, true
}

// lerp is p + f·(q − p), exactly.
func (p MotionParam) Lerp(q MotionParam, f *big.Rat) MotionParam {
	at := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		return d.Add(a, d.Mul(d, f))
	}
	return MotionParam{Turn: at(p.Turn, q.Turn), Base: at(p.Base, q.Base)}
}

// spanUpper is a proven upper bound on |θ(q) − θ(p)| in the base unit:
// 2π·|Δturn| + |Δbase|, with π taken at its upper enclosure. It is exact for
// a length, whose turn is always zero.
func (p MotionParam) SpanUpper(q MotionParam) *big.Rat {
	dTurn := new(big.Rat).Sub(q.Turn, p.Turn)
	dTurn.Abs(dTurn)
	dBase := new(big.Rat).Sub(q.Base, p.Base)
	dBase.Abs(dBase)
	out := new(big.Rat).Mul(dTurn, proofbound.TwoPiInterval().Hi)
	return out.Add(out, dBase)
}

// ExactTurnSinCos encloses sin(2πt) and cos(2πt) for an exact rational turn.
// A whole number of quarter turns answers exactly — that is what keeps an
// identity pose (the parameter 0) and a right-angle pose free of any
// enclosure width — and every other turn takes proofbound.TurnSinCosInterval.
func ExactTurnSinCos(t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	q := new(big.Rat).Mul(t, big.NewRat(4, 1))
	if q.IsInt() {
		k := new(big.Int).Mod(q.Num(), big.NewInt(4)).Int64()
		sinCos := [4][2]int64{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}[k]
		return proofbound.PointInterval(big.NewRat(sinCos[0], 1)), proofbound.PointInterval(big.NewRat(sinCos[1], 1))
	}
	return proofbound.TurnSinCosInterval(t)
}

// RadianSinCos encloses sin(r) and cos(r) for an exact rational angle r in
// radians. r is the turn r/2π, which lies in [r/(2·πhi), r/(2·πlo)] for a
// non-negative r (reversed for a negative one); the pair is evaluated at the
// lower turn and widened by the turn interval's own angular width, since sine
// and cosine are 1-Lipschitz in the angle: |sin(2πt) − sin(2πt_lo)| ≤
// 2π·(t_hi − t_lo) ≤ 2·πhi·(t_hi − t_lo).
//
// A nonzero angle's pair is read through the process-wide memo
// (sin_cos_memo.go); the caller owns the endpoints it gets either way.
func RadianSinCos(r *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	if r.Sign() == 0 || !MemosOn() {
		return radianSinCos(r)
	}
	key := string(AppendRatKey(nil, r))
	if entry, ok := sinCosMemo.load(key); ok {
		return entry.sin, entry.cos
	}
	sin, cos := radianSinCos(r)
	sinCosMemo.store(key, sinCosEntry{sin: sin, cos: cos})
	return sin, cos
}

// radianSinCos is RadianSinCos computed afresh.
func radianSinCos(r *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	if r.Sign() == 0 {
		return proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1))
	}
	twoPi := proofbound.TwoPiInterval()
	tA := new(big.Rat).Quo(r, twoPi.Hi)
	tB := new(big.Rat).Quo(r, twoPi.Lo)
	tLo, tHi := tA, tB
	if tLo.Cmp(tHi) > 0 {
		tLo, tHi = tHi, tLo
	}
	sin, cos := proofbound.TurnSinCosInterval(tLo)
	width := new(big.Rat).Sub(tHi, tLo)
	width.Mul(width, twoPi.Hi)
	widen := func(iv proofbound.RatInterval) proofbound.RatInterval {
		return proofbound.IntervalOwned(new(big.Rat).Sub(iv.Lo, width), new(big.Rat).Add(iv.Hi, width))
	}
	return widen(sin), widen(cos)
}

// ParamSinCos encloses sin θ and cos θ for θ = 2π·turn + base, by the angle
// sum formulas over the two parts' own enclosures. A part that is exactly zero
// contributes the point pair (0, 1), so a pure-degree or pure-radian angle
// passes its own enclosure through unwidened.
func ParamSinCos(p MotionParam) (proofbound.RatInterval, proofbound.RatInterval) {
	sT, cT := ExactTurnSinCos(p.Turn)
	if p.Base.Sign() == 0 {
		return sT, cT
	}
	sR, cR := RadianSinCos(p.Base)
	if p.Turn.Sign() == 0 {
		return sR, cR
	}
	sin := proofbound.IntervalAdd(proofbound.IntervalMul(sT, cR), proofbound.IntervalMul(cT, sR))
	cos := proofbound.IntervalSub(proofbound.IntervalMul(cT, cR), proofbound.IntervalMul(sT, sR))
	return sin, cos
}

// RatVec is an exact rational vector.
type RatVec [3]*big.Rat

func RatVecOf(v r3.Vec) (RatVec, bool) {
	x, y, z := proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)
	if x == nil || y == nil || z == nil {
		return RatVec{}, false
	}
	return RatVec{x, y, z}, true
}

// IvVec and IvMat are interval vectors and 3×3 interval matrices, the
// matrix stored by rows.
type IvVec [3]proofbound.RatInterval

type IvMat [3][3]proofbound.RatInterval

func PointVec(v RatVec) IvVec {
	return IvVec{proofbound.PointInterval(v[0]), proofbound.PointInterval(v[1]), proofbound.PointInterval(v[2])}
}

func (m IvMat) Apply(v IvVec) IvVec {
	var out IvVec
	for i := range 3 {
		sum := proofbound.IntervalMul(m[i][0], v[0])
		sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][1], v[1]))
		out[i] = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][2], v[2]))
	}
	return out
}

// ScaledIvMat is an interval matrix with its 18 endpoints written as integer
// numerators over one shared positive denominator: entry (i, j) is
// [lo[i][j]/den, hi[i][j]/den] exactly.
type ScaledIvMat struct {
	Den    *big.Int
	Lo, Hi [3][3]*big.Int
}

// entry is entry (i, j) as its exact rational interval.
func (s ScaledIvMat) Entry(i, j int) proofbound.RatInterval {
	return proofbound.IntervalOwned(new(big.Rat).SetFrac(s.Lo[i][j], s.Den), new(big.Rat).SetFrac(s.Hi[i][j], s.Den))
}

func NewScaledIvMat(m IvMat) ScaledIvMat {
	den := big.NewInt(1)
	for i := range 3 {
		for j := range 3 {
			den = proofarith.LcmInt(proofarith.LcmInt(den, m[i][j].Lo.Denom()), m[i][j].Hi.Denom())
		}
	}
	s := ScaledIvMat{Den: den}
	for i := range 3 {
		for j := range 3 {
			s.Lo[i][j], s.Hi[i][j] = proofarith.ScaledNum(m[i][j].Lo, den), proofarith.ScaledNum(m[i][j].Hi, den)
		}
	}
	return s
}

// applyScaled is IvMat.apply on the exact point whose coordinates are n/q,
// for integer numerators n over a positive q: the endpoint numerators of each
// row over den·q. The interval product of an entry with a point coordinate v
// is [lo·v, hi·v] for v ≥ 0 and [hi·v, lo·v] for v < 0 (MulInterval), and a
// row sums them, so each endpoint is one integer dot product.
func (s ScaledIvMat) ApplyScaled(n [3]*big.Int) ([3]*big.Int, [3]*big.Int) {
	var lo, hi [3]*big.Int
	term := new(big.Int)
	for i := range 3 {
		lo[i], hi[i] = new(big.Int), new(big.Int)
		for j := range 3 {
			low, high := s.Lo[i][j], s.Hi[i][j]
			if n[j].Sign() < 0 {
				low, high = high, low
			}
			lo[i].Add(lo[i], term.Mul(low, n[j]))
			hi[i].Add(hi[i], term.Mul(high, n[j]))
		}
	}
	return lo, hi
}

// mulPoints is IvMat.mul with a right factor o whose entries are all points,
// as ExactTransform's are: the interval product of an entry with a point v is
// [lo·v, hi·v] for v ≥ 0 and [hi·v, lo·v] for v < 0 (MulInterval), so each
// result endpoint is one integer dot product over den times o's own shared
// denominator. ok is false when an entry of o is not a point.
func (s ScaledIvMat) MulPoints(o IvMat) (ScaledIvMat, bool) {
	values := make([]*big.Rat, 0, 9)
	for k := range 3 {
		for j := range 3 {
			if o[k][j].Lo.Cmp(o[k][j].Hi) != 0 {
				return ScaledIvMat{}, false
			}
			values = append(values, o[k][j].Lo)
		}
	}
	pointDen := proofarith.CommonDenom(values...)
	var point [3][3]*big.Int
	for k := range 3 {
		for j := range 3 {
			point[k][j] = proofarith.ScaledNum(values[3*k+j], pointDen)
		}
	}
	out := ScaledIvMat{Den: new(big.Int).Mul(s.Den, pointDen)}
	term := new(big.Int)
	for i := range 3 {
		for j := range 3 {
			lo, hi := new(big.Int), new(big.Int)
			for k := range 3 {
				low, high := s.Lo[i][k], s.Hi[i][k]
				if point[k][j].Sign() < 0 {
					low, high = high, low
				}
				lo.Add(lo, term.Mul(low, point[k][j]))
				hi.Add(hi, term.Mul(high, point[k][j]))
			}
			out.Lo[i][j], out.Hi[i][j] = lo, hi
		}
	}
	return out, true
}

func IvVecAdd(a, b IvVec) IvVec {
	return IvVec{proofbound.IntervalAdd(a[0], b[0]), proofbound.IntervalAdd(a[1], b[1]), proofbound.IntervalAdd(a[2], b[2])}
}

func IvVecSub(a, b IvVec) IvVec {
	return IvVec{proofbound.IntervalSub(a[0], b[0]), proofbound.IntervalSub(a[1], b[1]), proofbound.IntervalSub(a[2], b[2])}
}

// MagnitudeSquaredUpper is Σ max(|lo|, |hi|)² over the entries: an exact
// upper bound on the squared Euclidean (or, over a matrix's entries,
// Frobenius) norm of every member of the enclosure.
func MagnitudeSquaredUpper(entries ...proofbound.RatInterval) *big.Rat {
	sum := new(big.Rat)
	for _, e := range entries {
		m := new(big.Rat).Abs(e.Lo)
		if hi := new(big.Rat).Abs(e.Hi); hi.Cmp(m) > 0 {
			m = hi
		}
		sum.Add(sum, m.Mul(m, m))
	}
	return sum
}

// UnitScaleInterval encloses 1/|a| for a nonzero exact vector a: the inverse
// of proofbound.RatSqrtUp/proofbound.RatSqrtDown's directed roots of |a|², each proven by exact
// comparison. When |a|² is the exact square of a float both roots agree and
// the enclosure is a point, which is what keeps an axis-aligned direction
// exact.
func UnitScaleInterval(a RatVec) (proofbound.RatInterval, bool) {
	sq := proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
	up, down := proofbound.RatSqrtUp(sq), proofbound.RatSqrtDown(sq)
	if !(down > 0) || proofbound.IsNonFinite(up) {
		return proofbound.RatInterval{}, false
	}
	lo, hi := proofarith.FloatRat(up), proofarith.FloatRat(down)
	return proofbound.IntervalOwned(new(big.Rat).Inv(lo), new(big.Rat).Inv(hi)), true
}

func (m IvMat) Mul(o IvMat) IvMat {
	var out IvMat
	for i := range 3 {
		for j := range 3 {
			sum := proofbound.IntervalMul(m[i][0], o[0][j])
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][1], o[1][j]))
			out[i][j] = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][2], o[2][j]))
		}
	}
	return out
}

// ExactTransform reads a float transform's linear part (by rows) and its
// translation as the exact rationals its entries denote.
func ExactTransform(t r3.Transform) (IvMat, RatVec, bool) {
	b := t.Basis()
	var rot IvMat
	for j, col := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := RatVecOf(col)
		if !ok {
			return IvMat{}, RatVec{}, false
		}
		for i := range 3 {
			rot[i][j] = proofbound.PointInterval(c[i])
		}
	}
	shift, ok := RatVecOf(t.Translation())
	if !ok {
		return IvMat{}, RatVec{}, false
	}
	return rot, shift, true
}

// IdealPose is the exact rigid motion T*(θ) a motion parameter denotes,
// enclosed over rationals: x ↦ rot·(x − pivot) + pivot + shift. A revolute
// carries the Rodrigues rotation about its exact unit axis Axis/|Axis| and
// its exact Center as pivot; a prismatic carries the identity rotation and
// the shift d·Dir/|Dir|; a between carries the exact screw composed onto its
// exact From with a zero pivot (MotionFrame.at).
type IdealPose struct {
	Rot   IvMat
	Pivot IvVec
	Shift IvVec
}

// Then is the ideal pose that applies p first and q after it, x ↦ q(p(x)),
// in x ↦ rot·x + shift form with a zero pivot
// (docs/linkage-check-design.md §5.1): p carries x to B_p·x + t_p, with
// t_p = B_p·(−c_p) + c_p + s_p its image of the origin, so q(p(x)) is
// B_q·B_p·x + B_q·(t_p − c_q) + c_q + s_q. Every term is an interval product
// or sum, so the result encloses q'∘p' for every member p' of p's enclosure
// and q' of q's.
func (p IdealPose) Then(q IdealPose) IdealPose {
	zero := proofbound.PointInterval(new(big.Rat))
	origin := IvVec{zero, zero, zero}
	tp := IvVecAdd(IvVecAdd(p.Rot.Apply(IvVecSub(origin, p.Pivot)), p.Pivot), p.Shift)
	return IdealPose{
		Rot:   q.Rot.Mul(p.Rot),
		Pivot: origin,
		Shift: IvVecAdd(IvVecAdd(q.Rot.Apply(IvVecSub(tp, q.Pivot)), q.Pivot), q.Shift),
	}
}

// MotionFrame is the exact reading of a Motion's own fields every ideal pose
// is built from. For a Between, axis and center are the read screw's Axis and
// Point, theta and slide its Angle and Slide, each the exact rational of the
// float r3 returned, and from and to its two stated poses read exactly.
type MotionFrame struct {
	Kind   MotionKind
	Axis   RatVec                 // Axis (revolute, between) or Dir (prismatic), exact
	Unit   proofbound.RatInterval // 1/|axis|
	Center RatVec                 // the revolute's pivot or the screw's Point; zero for a prismatic

	Theta          MotionParam // the screw's angle θ, radians
	Slide          *big.Rat    // the screw's slide d, millimetres
	FromRot, ToRot IvMat       // B(From), B(To)
	FromT, ToT     RatVec      // t(From), t(To)
}

// rotation is the Rodrigues matrix R = cos·I + sin·[k]× + (1 − cos)·k kᵀ
// about k = axis/|axis| for an enclosed sine and cosine: k kᵀ = a aᵀ/|a|² is
// exact, and [k]× = [a]×·(1/|a|) carries UnitScaleInterval's enclosure.
func (mf MotionFrame) Rotation(sin, cos proofbound.RatInterval) IvMat {
	one := proofbound.PointInterval(big.NewRat(1, 1))
	a := mf.Axis
	sq := proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
	oneMinusCos := proofbound.IntervalSub(one, cos)
	cross := [3][3]*big.Rat{
		{new(big.Rat), new(big.Rat).Neg(a[2]), a[1]},
		{a[2], new(big.Rat), new(big.Rat).Neg(a[0])},
		{new(big.Rat).Neg(a[1]), a[0], new(big.Rat)},
	}
	sinUnit := proofbound.IntervalMul(sin, mf.Unit)
	var rot IvMat
	for i := range 3 {
		for j := range 3 {
			outer := new(big.Rat).Quo(proofbound.RatMul(a[i], a[j]), sq)
			entry := proofbound.IntervalAdd(proofbound.IntervalScale(sinUnit, cross[i][j]), proofbound.IntervalScale(oneMinusCos, outer))
			if i == j {
				entry = proofbound.IntervalAdd(entry, cos)
			}
			rot[i][j] = entry
		}
	}
	return rot
}

// scaledRotation is rotation's matrix in the common-denominator form, each
// entry the same exact interval rotation builds. With a = A/d for integers A
// over the axis's shared denominator d, k kᵀ = A Aᵀ/|A|² and [k]× =
// [A]×·(1/(d·|a|)), so every entry is sinUnit·C + (1 − cos)·O (+ cos on the
// diagonal) for integer C = [A]× and O = A Aᵀ, and proofbound.IntervalScale's endpoint
// order follows the sign of C or O as it follows the sign of the scale.
func (mf MotionFrame) ScaledRotation(sin, cos proofbound.RatInterval) ScaledIvMat {
	a := mf.Axis
	axisDen := proofarith.CommonDenom(a[0], a[1], a[2])
	var axis [3]*big.Int
	for k := range 3 {
		axis[k] = proofarith.ScaledNum(a[k], axisDen)
	}
	squared := new(big.Int)
	for k := range 3 {
		squared.Add(squared, new(big.Int).Mul(axis[k], axis[k]))
	}
	sinUnit := proofbound.IntervalMul(sin, mf.Unit)
	sinDen := proofarith.LcmInt(new(big.Int).Set(sinUnit.Lo.Denom()), sinUnit.Hi.Denom())
	cosDen := proofarith.LcmInt(new(big.Int).Set(cos.Lo.Denom()), cos.Hi.Denom())
	sinScale := new(big.Int).Mul(sinDen, axisDen)
	cosScale := new(big.Int).Mul(cosDen, squared)
	den := proofarith.LcmInt(sinScale, cosScale)
	// The three intervals' endpoints over den once C, O and 1 multiply them.
	sinMultiplier := new(big.Int).Quo(den, sinScale)
	cosMultiplier := new(big.Int).Quo(den, cosScale)
	diagonalMultiplier := new(big.Int).Quo(den, cosDen)
	sinLo := proofarith.ScaledNum(sinUnit.Lo, sinDen)
	sinLo.Mul(sinLo, sinMultiplier)
	sinHi := proofarith.ScaledNum(sinUnit.Hi, sinDen)
	sinHi.Mul(sinHi, sinMultiplier)
	cosLo, cosHi := proofarith.ScaledNum(cos.Lo, cosDen), proofarith.ScaledNum(cos.Hi, cosDen)
	// 1 − cos is [1 − hi, 1 − lo].
	oneMinusLo := new(big.Int).Sub(cosDen, cosHi)
	oneMinusLo.Mul(oneMinusLo, cosMultiplier)
	oneMinusHi := new(big.Int).Sub(cosDen, cosLo)
	oneMinusHi.Mul(oneMinusHi, cosMultiplier)
	cosLo.Mul(cosLo, diagonalMultiplier)
	cosHi.Mul(cosHi, diagonalMultiplier)
	zero := new(big.Int)
	cross := [3][3]*big.Int{
		{zero, new(big.Int).Neg(axis[2]), axis[1]},
		{axis[2], zero, new(big.Int).Neg(axis[0])},
		{new(big.Int).Neg(axis[1]), axis[0], zero},
	}
	s := ScaledIvMat{Den: den}
	for i := range 3 {
		for j := range 3 {
			c, o := cross[i][j], new(big.Int).Mul(axis[i], axis[j])
			sinLow, sinHigh := sinLo, sinHi
			if c.Sign() < 0 {
				sinLow, sinHigh = sinHi, sinLo
			}
			cosLow, cosHigh := oneMinusLo, oneMinusHi
			if o.Sign() < 0 {
				cosLow, cosHigh = oneMinusHi, oneMinusLo
			}
			lo := new(big.Int).Mul(sinLow, c)
			lo.Add(lo, new(big.Int).Mul(cosLow, o))
			hi := new(big.Int).Mul(sinHigh, c)
			hi.Add(hi, o.Mul(cosHigh, o))
			if i == j {
				lo.Add(lo, cosLo)
				hi.Add(hi, cosHi)
			}
			s.Lo[i][j], s.Hi[i][j] = lo, hi
		}
	}
	return s
}

// at builds the ideal pose for the exact parameter p.
//
// A between's parameter is the fraction s = p.base, and its ideal path is
// docs/motion-check-design.md §5.1's T*(s) = S*(s) ∘ From: the exact screw of
// the read parameters, rotating by s·θ about the line through c = Point along
// n = Axis/|Axis| and sliding s·d along n, applied after the exact From. In
// x ↦ rot·x + shift form that is rot = R(s·θ, n)·B(From) and
// shift = R(s·θ, n)·(t(From) − c) + c + s·d·n. s·θ is a radian value, so its
// sine and cosine come from ParamSinCos's π enclosures, and at s = 0 they are
// the point pair (0, 1): T*(0) is From exactly.
func (mf MotionFrame) At(p MotionParam) IdealPose {
	zero := proofbound.PointInterval(new(big.Rat))
	one := proofbound.PointInterval(big.NewRat(1, 1))
	pose := IdealPose{Pivot: PointVec(mf.Center), Shift: IvVec{zero, zero, zero}}
	switch mf.Kind {
	case MotionPrismatic:
		for i := range 3 {
			for j := range 3 {
				pose.Rot[i][j] = zero
			}
			pose.Rot[i][i] = one
			pose.Shift[i] = proofbound.IntervalMul(proofbound.IntervalScale(mf.Unit, mf.Axis[i]), proofbound.PointInterval(p.Base))
		}
		return pose
	case MotionRevolute:
		pose.Rot = mf.Rotation(ParamSinCos(p))
		return pose
	}
	s := p.Base
	phi := MotionParam{Turn: new(big.Rat).Mul(mf.Theta.Turn, s), Base: new(big.Rat).Mul(mf.Theta.Base, s)}
	r := mf.Rotation(ParamSinCos(phi))
	c := PointVec(mf.Center)
	slide := new(big.Rat).Mul(s, mf.Slide)
	var along IvVec
	for i := range 3 {
		along[i] = proofbound.IntervalScale(proofbound.IntervalScale(mf.Unit, mf.Axis[i]), slide)
	}
	return IdealPose{
		Rot:   r.Mul(mf.FromRot),
		Pivot: IvVec{zero, zero, zero},
		Shift: IvVecAdd(IvVecAdd(r.Apply(IvVecSub(PointVec(mf.FromT), c)), c), along),
	}
}

// statedEnd is the ideal pose of a between's stated To, x ↦ B(To)·x + t(To)
// read exactly: what η_To of docs/motion-check-design.md §5.1 charges the
// s = 1 pose against, beside the ideal end T*(1).
func (mf MotionFrame) StatedEnd() IdealPose {
	zero := proofbound.PointInterval(new(big.Rat))
	return IdealPose{Rot: mf.ToRot, Pivot: IvVec{zero, zero, zero}, Shift: PointVec(mf.ToT)}
}

// placeFrom maps an exact point through the between's From exactly,
// x' = B(From)·x + t(From) — the ideal T*(0). Every other kind starts its path
// at rest, so the point is returned unchanged.
func (mf MotionFrame) PlaceFrom(x RatVec) RatVec {
	if mf.Kind != MotionBetween {
		return x
	}
	var out RatVec
	for i := range 3 {
		sum := new(big.Rat).Set(mf.FromT[i])
		for j := range 3 {
			sum.Add(sum, proofbound.RatMul(mf.FromRot[i][j].Lo, x[j]))
		}
		out[i] = sum
	}
	return out
}

// PoseDeviation is docs/motion-check-design.md §5.1's η: a proven upper bound
// on how far any point of a mover sits between where the kernel measured it —
// under the float composed placement C = P0 then PoseAt — and where the ideal
// motion T* puts it, T* composed onto the mover's own placement P0. For a
// record point p with |p| ≤ r0 the two images are B(C)·p + t(C) and
// R·(B(P0)·p + t(P0) − c) + c + s, so they differ by at most
// ‖B(C) − R·B(P0)‖_F·r0 + |t(C) − (R·(t(P0) − c) + c + s)|. Every float is
// read exactly off Basis()/Translation(), and both norms are rational
// enclosures rooted by proofbound.RatSqrtUp, so η covers every rounding the pose
// committed: math.Sincos, Rodrigues' formula, the pivot offset, Then, and
// the rounded π/180 of a degree-stated angle.
//
// A linear part matching the ideal one exactly contributes nothing whatever
// r0 is — the true radius is finite even where no reader states it — so a
// pure translation stays chargeable on a payload with no record radius.
// Every other unreadable term answers +Inf, a refusal rather than a bound.
//
// The second result is the linear term's own factor, an upper bound on
// ‖B(C) − R·B(P0)‖_F, which PathAreaUpper reads to bound how far the straight
// path between the two images stretches the mover's surface.
func PoseDeviation(composed, placement r3.Transform, ideal IdealPose, r0 float64) (float64, float64) {
	bc, bp := composed.Basis(), placement.Basis()
	colsC := [3]r3.Vec{bc.EX, bc.EY, bc.EZ}
	colsP := [3]r3.Vec{bp.EX, bp.EY, bp.EZ}
	var linear []proofbound.RatInterval
	for j := range 3 {
		c, okC := RatVecOf(colsC[j])
		p, okP := RatVecOf(colsP[j])
		if !okC || !okP {
			return math.Inf(1), math.Inf(1)
		}
		image := ideal.Rot.Apply(PointVec(p))
		diff := IvVecSub(PointVec(c), image)
		linear = append(linear, diff[:]...)
	}
	tc, okC := RatVecOf(composed.Translation())
	tp, okP := RatVecOf(placement.Translation())
	if !okC || !okP {
		return math.Inf(1), math.Inf(1)
	}
	idealT := IvVecAdd(IvVecAdd(ideal.Rot.Apply(IvVecSub(PointVec(tp), ideal.Pivot)), ideal.Pivot), ideal.Shift)
	dt := IvVecSub(PointVec(tc), idealT)
	transUp := proofbound.RatSqrtUp(MagnitudeSquaredUpper(dt[:]...))
	linSq := MagnitudeSquaredUpper(linear...)
	if linSq.Sign() == 0 {
		return transUp, 0
	}
	linUp := proofbound.RatSqrtUp(linSq)
	return proofbound.AbsSumUpper(proofbound.ProductUpper(linUp, r0), transUp), linUp
}

// BasisSigmaLower is a proven lower bound on the smallest singular value of a
// placement's linear part B. r3 holds B orthonormal only to rounding, so
// BᵀB = I + E with E read exactly off the float columns; every eigenvalue of
// BᵀB is then at least 1 − ‖E‖_F, and the bound is that value's root,
// rounded down. An exactly orthonormal basis answers exactly 1; a defect too
// large to bound answers 0, which PathAreaUpper reads as a refusal.
func BasisSigmaLower(t r3.Transform) float64 {
	e, ok := BasisDefectUpper(t)
	switch {
	case !ok:
		return 0
	case e.Sign() == 0:
		return 1
	case e.Cmp(big.NewRat(1, 1)) >= 0:
		return 0
	}
	return proofbound.RatSqrtDown(new(big.Rat).Sub(big.NewRat(1, 1), e))
}

// BasisSigmaUpper is BasisSigmaLower's mirror: a proven upper bound on the
// largest singular value of a transform's linear part B. Every eigenvalue of
// BᵀB = I + E is at most 1 + ‖E‖_F, and the bound is that value's root,
// rounded up. An exactly orthonormal basis answers exactly 1; a defect that
// cannot be read answers +Inf, a refusal rather than a bound.
func BasisSigmaUpper(t r3.Transform) float64 {
	e, ok := BasisDefectUpper(t)
	switch {
	case !ok:
		return math.Inf(1)
	case e.Sign() == 0:
		return 1
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(big.NewRat(1, 1), e))
}

// BasisDefectUpper is a proven upper bound on ‖BᵀB − I‖_F for a transform's
// linear part B, read exactly off its float columns and rooted upward; it is
// exactly zero for an exactly orthonormal basis. ok is false when a column is
// not finite or the root overflows.
func BasisDefectUpper(t r3.Transform) (*big.Rat, bool) {
	b := t.Basis()
	var cols [3]RatVec
	for j, v := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := RatVecOf(v)
		if !ok {
			return nil, false
		}
		cols[j] = c
	}
	var defect []proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			e := proofbound.RatAdd(proofbound.RatMul(cols[i][0], cols[j][0]), proofbound.RatMul(cols[i][1], cols[j][1]), proofbound.RatMul(cols[i][2], cols[j][2]))
			if i == j {
				e.Sub(e, big.NewRat(1, 1))
			}
			defect = append(defect, proofbound.PointInterval(e))
		}
	}
	sq := MagnitudeSquaredUpper(defect...)
	if sq.Sign() == 0 {
		return new(big.Rat), true
	}
	e := proofarith.FloatRat(proofbound.RatSqrtUp(sq))
	return e, e != nil
}

// PathAreaUpper bounds the mover's surface area at every point of the straight
// path between the float pose's image and the ideal pose's — the area
// proofbound.SweptVolumeAllow's own contract asks for along the WHOLE path. Relative to
// the mover at rest, a point of that path is M_t·x + c with
// M_t = R + (1 − t)·(B(C) − R·B(P0))·B(P0)⁻¹, so
// ‖M_t‖₂ ≤ ‖R‖₂ + linear/sigma, and an area scales by at most ‖M_t‖₂². area
// is the rest area's upper bound (Area().Value + Bound), linear
// PoseDeviation's second result, sigma BasisSigmaLower of the rest placement.
//
// base is a proven upper bound on ‖R‖₂, the stretch base of
// docs/motion-check-design.md §5.1: exactly 1 for a Revolute and a Prismatic,
// whose ideal rotation is exactly orthogonal, and BasisSigmaUpper of the
// between's From — at s = 1 the larger of From's and To's — for a Between,
// whose ideal linear part R(s·θ, n)·B(From) is orthogonal only as far as
// B(From) is. An exact linear part scales area by base² alone, and base 1
// leaves it unscaled.
func PathAreaUpper(area, linear, sigma, base float64) float64 {
	if linear == 0 {
		if base == 1 {
			return area
		}
		return proofbound.ProductUpper(area, proofbound.ProductUpper(base, base))
	}
	stretch := proofbound.AbsSumUpper(base, proofbound.DivUpper(linear, sigma))
	return proofbound.ProductUpper(area, proofbound.ProductUpper(stretch, stretch))
}

// ExceedsResolution reports whether the interval between two exact parameters
// is wider than the resolution. Parts stated in the same terms — both whole
// turns, or both base units — compare exactly, which is what lets a dyadic
// step equal to the resolution stop on it. Mixed parts compare the interval's
// smallest possible width, π at its lower enclosure, against the resolution's
// largest, π at its upper, so the floor never stops refinement early by
// rounding.
func ExceedsResolution(p, q, res MotionParam) bool {
	dTurn := new(big.Rat).Sub(q.Turn, p.Turn)
	dBase := new(big.Rat).Sub(q.Base, p.Base)
	switch {
	case dBase.Sign() == 0 && res.Base.Sign() == 0:
		return new(big.Rat).Abs(dTurn).Cmp(res.Turn) > 0
	case dTurn.Sign() == 0 && res.Turn.Sign() == 0:
		return new(big.Rat).Abs(dBase).Cmp(res.Base) > 0
	}
	twoPi := proofbound.TwoPiInterval()
	width := proofbound.IntervalAdd(proofbound.IntervalScale(twoPi, dTurn), proofbound.PointInterval(dBase))
	lower := new(big.Rat)
	switch {
	case width.Lo.Sign() > 0:
		lower = width.Lo
	case width.Hi.Sign() < 0:
		lower = new(big.Rat).Neg(width.Hi)
	}
	upper := new(big.Rat).Mul(twoPi.Hi, res.Turn)
	upper.Add(upper, res.Base)
	return lower.Cmp(upper) > 0
}

// MoverTravel is τ of docs/motion-check-design.md §5.2 as an exact rational:
// a proven upper bound on how far any point of the mover travels while the
// parameter runs from p to q. A prismatic moves every point by exactly the
// displacement change along a unit direction; a revolute moves a point at
// distance ρ from the axis along an arc of length ρ·|Δθ|, which bounds its
// chord, and ρ ≤ rho. A between rotates a point at distance ρ from the screw
// axis through an arc of length ρ·|Δs|·θ and slides it |Δs|·|d| along the
// axis; the sum bounds the resultant, so τ = |Δs|·(rho·θ + |d|), with θ and
// d the exact rationals of the floats r3 read and no π entering. No rounding
// is committed: rho is already an upper bound, and a span over an angle is
// taken at π's upper enclosure.
func MoverTravel(mf MotionFrame, rho float64, p, q MotionParam) *big.Rat {
	span := p.SpanUpper(q)
	if mf.Kind == MotionPrismatic {
		return span
	}
	r := proofarith.FloatRat(rho)
	if r == nil {
		return nil
	}
	if mf.Kind == MotionRevolute {
		return span.Mul(span, r)
	}
	zero := MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	rate := r.Mul(r, zero.SpanUpper(mf.Theta))
	rate.Add(rate, new(big.Rat).Abs(mf.Slide))
	return span.Mul(span, rate)
}
