package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

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

// AtRange is the ideal pose for every parameter between lo and hi at once
// (docs/linkage-check-design.md §15.4): a revolute's Rodrigues matrix built
// from ParamSinCos(lo) widened on both sides by lo.SpanUpper(hi) — sine and
// cosine are 1-Lipschitz in the angle — and a prismatic's translation by the
// interval [lo, hi] of base millimetres along its unit direction. Every member
// of the enclosure is the joint's motion at some value in [lo, hi] or a
// superset's, so a pose deviation charged against it covers the motion at
// every value of the range. AtRange(p, p) is At(p). A between has no range
// form and answers At(lo).
func (mf MotionFrame) AtRange(lo, hi MotionParam) IdealPose {
	if lo.Turn.Cmp(hi.Turn) == 0 && lo.Base.Cmp(hi.Base) == 0 {
		return mf.At(lo)
	}
	zero := proofbound.PointInterval(new(big.Rat))
	switch mf.Kind {
	case MotionPrismatic:
		pose := mf.At(lo)
		lower, upper := lo.Base, hi.Base
		if lower.Cmp(upper) > 0 {
			lower, upper = upper, lower
		}
		along := proofbound.Interval(lower, upper)
		for i := range 3 {
			pose.Shift[i] = proofbound.IntervalMul(proofbound.IntervalScale(mf.Unit, mf.Axis[i]), along)
		}
		return pose
	case MotionRevolute:
		width := lo.SpanUpper(hi)
		widen := func(iv proofbound.RatInterval) proofbound.RatInterval {
			return proofbound.IntervalOwned(new(big.Rat).Sub(iv.Lo, width), new(big.Rat).Add(iv.Hi, width))
		}
		sin, cos := ParamSinCos(lo)
		return IdealPose{Rot: mf.Rotation(widen(sin), widen(cos)), Pivot: PointVec(mf.Center), Shift: IvVec{zero, zero, zero}}
	}
	return mf.At(lo)
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
