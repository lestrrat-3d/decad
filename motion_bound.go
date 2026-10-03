package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file proves the bounds docs/motion-check-design.md §5 builds its
// interval certificate from, every one of them over exact rationals so that no
// float rounding sits between a bound and the claim it supports:
//
//   - the exact denotation of a motion parameter (motionParam) and of the
//     ideal pose it names (idealPose), with the ideal rotation's sine and
//     cosine enclosed by moments_trig.go's turnSinCosInterval; for a Between,
//     the exact screw of the parameters r3 read, composed onto the exact
//     From, and the stated To read exactly (motionFrame);
//   - η, the proven distance between the float pose the kernel measured and
//     the ideal pose the claim is about (poseDeviation, §5.1);
//   - R0, the record-coordinate radius η is charged at (moverRecordRadius);
//   - ρ_max, the largest distance from the rotation or screw axis of any
//     point of a mover (moverAxisRadius, §5.2);
//   - the travel bound τ and the swept-box exclusion (§5.2, §6);
//   - the area the collision transfer charges its swept-volume allowance at
//     (pathAreaUpper, basisSigmaLower, basisSigmaUpper, §5.1);
//   - the resolution floor's width comparison (exceedsResolution, §6).

// motionParam is one motion parameter's exact denotation. An angle denotes
// θ = 2π·turn + base radians: a degree-stated angle is an exact rational turn
// (units.Degree's own factor is a rounded π/180, so the degree count, not the
// factor, is what the caller stated), and any other angle unit is base =
// magnitude × factor radians, read exactly. A length denotes base millimetres
// and leaves turn zero, and a Between's dimensionless fraction denotes base
// and leaves turn zero.
type motionParam struct {
	turn *big.Rat
	base *big.Rat
}

// exactMotionParam reads v's exact denotation. It fails only on a non-finite
// magnitude or factor.
func exactMotionParam(v units.Value) (motionParam, bool) {
	mag := floatRat(v.Mag())
	if mag == nil {
		return motionParam{}, false
	}
	if v.Unit() == units.Degree {
		return motionParam{turn: new(big.Rat).Quo(mag, big.NewRat(360, 1)), base: new(big.Rat)}, true
	}
	factor := floatRat(v.Unit().Factor())
	if factor == nil {
		return motionParam{}, false
	}
	return motionParam{turn: new(big.Rat), base: new(big.Rat).Mul(mag, factor)}, true
}

// lerp is p + f·(q − p), exactly.
func (p motionParam) lerp(q motionParam, f *big.Rat) motionParam {
	at := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		return d.Add(a, d.Mul(d, f))
	}
	return motionParam{turn: at(p.turn, q.turn), base: at(p.base, q.base)}
}

// spanUpper is a proven upper bound on |θ(q) − θ(p)| in the base unit:
// 2π·|Δturn| + |Δbase|, with π taken at its upper enclosure. It is exact for
// a length, whose turn is always zero.
func (p motionParam) spanUpper(q motionParam) *big.Rat {
	dTurn := new(big.Rat).Sub(q.turn, p.turn)
	dTurn.Abs(dTurn)
	dBase := new(big.Rat).Sub(q.base, p.base)
	dBase.Abs(dBase)
	out := new(big.Rat).Mul(dTurn, twoPiInterval().hi)
	return out.Add(out, dBase)
}

// exactTurnSinCos encloses sin(2πt) and cos(2πt) for an exact rational turn.
// A whole number of quarter turns answers exactly — that is what keeps an
// identity pose (the parameter 0) and a right-angle pose free of any
// enclosure width — and every other turn takes turnSinCosInterval.
func exactTurnSinCos(t *big.Rat) (ratInterval, ratInterval) {
	q := new(big.Rat).Mul(t, big.NewRat(4, 1))
	if q.IsInt() {
		k := new(big.Int).Mod(q.Num(), big.NewInt(4)).Int64()
		sinCos := [4][2]int64{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}[k]
		return pointInterval(big.NewRat(sinCos[0], 1)), pointInterval(big.NewRat(sinCos[1], 1))
	}
	return turnSinCosInterval(t)
}

// radianSinCos encloses sin(r) and cos(r) for an exact rational angle r in
// radians. r is the turn r/2π, which lies in [r/(2·πhi), r/(2·πlo)] for a
// non-negative r (reversed for a negative one); the pair is evaluated at the
// lower turn and widened by the turn interval's own angular width, since sine
// and cosine are 1-Lipschitz in the angle: |sin(2πt) − sin(2πt_lo)| ≤
// 2π·(t_hi − t_lo) ≤ 2·πhi·(t_hi − t_lo).
func radianSinCos(r *big.Rat) (ratInterval, ratInterval) {
	if r.Sign() == 0 {
		return pointInterval(new(big.Rat)), pointInterval(big.NewRat(1, 1))
	}
	twoPi := twoPiInterval()
	tA := new(big.Rat).Quo(r, twoPi.hi)
	tB := new(big.Rat).Quo(r, twoPi.lo)
	tLo, tHi := tA, tB
	if tLo.Cmp(tHi) > 0 {
		tLo, tHi = tHi, tLo
	}
	sin, cos := turnSinCosInterval(tLo)
	width := new(big.Rat).Sub(tHi, tLo)
	width.Mul(width, twoPi.hi)
	widen := func(iv ratInterval) ratInterval {
		return intervalOwned(new(big.Rat).Sub(iv.lo, width), new(big.Rat).Add(iv.hi, width))
	}
	return widen(sin), widen(cos)
}

// paramSinCos encloses sin θ and cos θ for θ = 2π·turn + base, by the angle
// sum formulas over the two parts' own enclosures. A part that is exactly zero
// contributes the point pair (0, 1), so a pure-degree or pure-radian angle
// passes its own enclosure through unwidened.
func paramSinCos(p motionParam) (ratInterval, ratInterval) {
	sT, cT := exactTurnSinCos(p.turn)
	if p.base.Sign() == 0 {
		return sT, cT
	}
	sR, cR := radianSinCos(p.base)
	if p.turn.Sign() == 0 {
		return sR, cR
	}
	sin := intervalAdd(intervalMul(sT, cR), intervalMul(cT, sR))
	cos := intervalSub(intervalMul(cT, cR), intervalMul(sT, sR))
	return sin, cos
}

// ratVec is an exact rational vector.
type ratVec [3]*big.Rat

func ratVecOf(v r3.Vec) (ratVec, bool) {
	x, y, z := floatRat(v.X), floatRat(v.Y), floatRat(v.Z)
	if x == nil || y == nil || z == nil {
		return ratVec{}, false
	}
	return ratVec{x, y, z}, true
}

// ivVec and ivMat are interval vectors and 3×3 interval matrices, the
// matrix stored by rows.
type ivVec [3]ratInterval
type ivMat [3][3]ratInterval

func pointVec(v ratVec) ivVec {
	return ivVec{pointInterval(v[0]), pointInterval(v[1]), pointInterval(v[2])}
}

func (m ivMat) apply(v ivVec) ivVec {
	var out ivVec
	for i := range 3 {
		sum := intervalMul(m[i][0], v[0])
		sum = intervalAdd(sum, intervalMul(m[i][1], v[1]))
		out[i] = intervalAdd(sum, intervalMul(m[i][2], v[2]))
	}
	return out
}

func ivVecAdd(a, b ivVec) ivVec {
	return ivVec{intervalAdd(a[0], b[0]), intervalAdd(a[1], b[1]), intervalAdd(a[2], b[2])}
}

func ivVecSub(a, b ivVec) ivVec {
	return ivVec{intervalSub(a[0], b[0]), intervalSub(a[1], b[1]), intervalSub(a[2], b[2])}
}

// magnitudeSquaredUpper is Σ max(|lo|, |hi|)² over the entries: an exact
// upper bound on the squared Euclidean (or, over a matrix's entries,
// Frobenius) norm of every member of the enclosure.
func magnitudeSquaredUpper(entries ...ratInterval) *big.Rat {
	sum := new(big.Rat)
	for _, e := range entries {
		m := new(big.Rat).Abs(e.lo)
		if hi := new(big.Rat).Abs(e.hi); hi.Cmp(m) > 0 {
			m = hi
		}
		sum.Add(sum, m.Mul(m, m))
	}
	return sum
}

// unitScaleInterval encloses 1/|a| for a nonzero exact vector a: the inverse
// of ratSqrtUp/ratSqrtDown's directed roots of |a|², each proven by exact
// comparison. When |a|² is the exact square of a float both roots agree and
// the enclosure is a point, which is what keeps an axis-aligned direction
// exact.
func unitScaleInterval(a ratVec) (ratInterval, bool) {
	sq := ratAdd(ratMul(a[0], a[0]), ratMul(a[1], a[1]), ratMul(a[2], a[2]))
	up, down := ratSqrtUp(sq), ratSqrtDown(sq)
	if !(down > 0) || isNonFinite(up) {
		return ratInterval{}, false
	}
	lo, hi := floatRat(up), floatRat(down)
	return intervalOwned(new(big.Rat).Inv(lo), new(big.Rat).Inv(hi)), true
}

func (m ivMat) mul(o ivMat) ivMat {
	var out ivMat
	for i := range 3 {
		for j := range 3 {
			sum := intervalMul(m[i][0], o[0][j])
			sum = intervalAdd(sum, intervalMul(m[i][1], o[1][j]))
			out[i][j] = intervalAdd(sum, intervalMul(m[i][2], o[2][j]))
		}
	}
	return out
}

// exactTransform reads a float transform's linear part (by rows) and its
// translation as the exact rationals its entries denote.
func exactTransform(t r3.Transform) (ivMat, ratVec, bool) {
	b := t.Basis()
	var rot ivMat
	for j, col := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := ratVecOf(col)
		if !ok {
			return ivMat{}, ratVec{}, false
		}
		for i := range 3 {
			rot[i][j] = pointInterval(c[i])
		}
	}
	shift, ok := ratVecOf(t.Translation())
	if !ok {
		return ivMat{}, ratVec{}, false
	}
	return rot, shift, true
}

// idealPose is the exact rigid motion T*(θ) a motion parameter denotes,
// enclosed over rationals: x ↦ rot·(x − pivot) + pivot + shift. A revolute
// carries the Rodrigues rotation about its exact unit axis Axis/|Axis| and
// its exact Center as pivot; a prismatic carries the identity rotation and
// the shift d·Dir/|Dir|; a between carries the exact screw composed onto its
// exact From with a zero pivot (motionFrame.at).
type idealPose struct {
	rot   ivMat
	pivot ivVec
	shift ivVec
}

// motionFrame is the exact reading of a Motion's own fields every ideal pose
// is built from. For a Between, axis and center are the read screw's Axis and
// Point, theta and slide its Angle and Slide, each the exact rational of the
// float r3 returned, and from and to its two stated poses read exactly.
type motionFrame struct {
	kind   motionKind
	axis   ratVec      // Axis (revolute, between) or Dir (prismatic), exact
	unit   ratInterval // 1/|axis|
	center ratVec      // the revolute's pivot or the screw's Point; zero for a prismatic

	theta          motionParam // the screw's angle θ, radians
	slide          *big.Rat    // the screw's slide d, millimetres
	fromRot, toRot ivMat       // B(From), B(To)
	fromT, toT     ratVec      // t(From), t(To)
}

func newMotionFrame(spec motionSpec) (motionFrame, bool) {
	dirVec := spec.dir
	center := r3.Vec{}
	switch spec.kind {
	case motionRevolute:
		dirVec, center = spec.axis, spec.center
	case motionBetween:
		dirVec, center = spec.screw.Axis, spec.screw.Point
	}
	axis, okA := ratVecOf(dirVec)
	pivot, okC := ratVecOf(center)
	if !okA || !okC {
		return motionFrame{}, false
	}
	unit, ok := unitScaleInterval(axis)
	if !ok {
		return motionFrame{}, false
	}
	mf := motionFrame{kind: spec.kind, axis: axis, unit: unit, center: pivot}
	if spec.kind != motionBetween {
		return mf, true
	}
	theta, okT := exactMotionParam(spec.screw.Angle)
	slide := floatRat(spec.screw.Slide)
	fromRot, fromT, okF := exactTransform(spec.between.From)
	toRot, toT, okTo := exactTransform(spec.between.To)
	if !okT || slide == nil || !okF || !okTo {
		return motionFrame{}, false
	}
	mf.theta, mf.slide = theta, slide
	mf.fromRot, mf.fromT, mf.toRot, mf.toT = fromRot, fromT, toRot, toT
	return mf, true
}

// rotation is the Rodrigues matrix R = cos·I + sin·[k]× + (1 − cos)·k kᵀ
// about k = axis/|axis| for an enclosed sine and cosine: k kᵀ = a aᵀ/|a|² is
// exact, and [k]× = [a]×·(1/|a|) carries unitScaleInterval's enclosure.
func (mf motionFrame) rotation(sin, cos ratInterval) ivMat {
	one := pointInterval(big.NewRat(1, 1))
	a := mf.axis
	sq := ratAdd(ratMul(a[0], a[0]), ratMul(a[1], a[1]), ratMul(a[2], a[2]))
	oneMinusCos := intervalSub(one, cos)
	cross := [3][3]*big.Rat{
		{new(big.Rat), new(big.Rat).Neg(a[2]), a[1]},
		{a[2], new(big.Rat), new(big.Rat).Neg(a[0])},
		{new(big.Rat).Neg(a[1]), a[0], new(big.Rat)},
	}
	sinUnit := intervalMul(sin, mf.unit)
	var rot ivMat
	for i := range 3 {
		for j := range 3 {
			outer := new(big.Rat).Quo(ratMul(a[i], a[j]), sq)
			entry := intervalAdd(intervalScale(sinUnit, cross[i][j]), intervalScale(oneMinusCos, outer))
			if i == j {
				entry = intervalAdd(entry, cos)
			}
			rot[i][j] = entry
		}
	}
	return rot
}

// at builds the ideal pose for the exact parameter p.
//
// A between's parameter is the fraction s = p.base, and its ideal path is
// docs/motion-check-design.md §5.1's T*(s) = S*(s) ∘ From: the exact screw of
// the read parameters, rotating by s·θ about the line through c = Point along
// n = Axis/|Axis| and sliding s·d along n, applied after the exact From. In
// x ↦ rot·x + shift form that is rot = R(s·θ, n)·B(From) and
// shift = R(s·θ, n)·(t(From) − c) + c + s·d·n. s·θ is a radian value, so its
// sine and cosine come from paramSinCos's π enclosures, and at s = 0 they are
// the point pair (0, 1): T*(0) is From exactly.
func (mf motionFrame) at(p motionParam) idealPose {
	zero := pointInterval(new(big.Rat))
	one := pointInterval(big.NewRat(1, 1))
	pose := idealPose{pivot: pointVec(mf.center), shift: ivVec{zero, zero, zero}}
	switch mf.kind {
	case motionPrismatic:
		for i := range 3 {
			for j := range 3 {
				pose.rot[i][j] = zero
			}
			pose.rot[i][i] = one
			pose.shift[i] = intervalMul(intervalScale(mf.unit, mf.axis[i]), pointInterval(p.base))
		}
		return pose
	case motionRevolute:
		pose.rot = mf.rotation(paramSinCos(p))
		return pose
	}
	s := p.base
	phi := motionParam{turn: new(big.Rat).Mul(mf.theta.turn, s), base: new(big.Rat).Mul(mf.theta.base, s)}
	r := mf.rotation(paramSinCos(phi))
	c := pointVec(mf.center)
	slide := new(big.Rat).Mul(s, mf.slide)
	var along ivVec
	for i := range 3 {
		along[i] = intervalScale(intervalScale(mf.unit, mf.axis[i]), slide)
	}
	return idealPose{
		rot:   r.mul(mf.fromRot),
		pivot: ivVec{zero, zero, zero},
		shift: ivVecAdd(ivVecAdd(r.apply(ivVecSub(pointVec(mf.fromT), c)), c), along),
	}
}

// statedEnd is the ideal pose of a between's stated To, x ↦ B(To)·x + t(To)
// read exactly: what η_To of docs/motion-check-design.md §5.1 charges the
// s = 1 pose against, beside the ideal end T*(1).
func (mf motionFrame) statedEnd() idealPose {
	zero := pointInterval(new(big.Rat))
	return idealPose{rot: mf.toRot, pivot: ivVec{zero, zero, zero}, shift: pointVec(mf.toT)}
}

// placeFrom maps an exact point through the between's From exactly,
// x' = B(From)·x + t(From) — the ideal T*(0). Every other kind starts its path
// at rest, so the point is returned unchanged.
func (mf motionFrame) placeFrom(x ratVec) ratVec {
	if mf.kind != motionBetween {
		return x
	}
	var out ratVec
	for i := range 3 {
		sum := new(big.Rat).Set(mf.fromT[i])
		for j := range 3 {
			sum.Add(sum, ratMul(mf.fromRot[i][j].lo, x[j]))
		}
		out[i] = sum
	}
	return out
}

// poseDeviation is docs/motion-check-design.md §5.1's η: a proven upper bound
// on how far any point of a mover sits between where the kernel measured it —
// under the float composed placement C = P0 then PoseAt — and where the ideal
// motion T* puts it, T* composed onto the mover's own placement P0. For a
// record point p with |p| ≤ r0 the two images are B(C)·p + t(C) and
// R·(B(P0)·p + t(P0) − c) + c + s, so they differ by at most
// ‖B(C) − R·B(P0)‖_F·r0 + |t(C) − (R·(t(P0) − c) + c + s)|. Every float is
// read exactly off Basis()/Translation(), and both norms are rational
// enclosures rooted by ratSqrtUp, so η covers every rounding the pose
// committed: math.Sincos, Rodrigues' formula, the pivot offset, Then, and
// the rounded π/180 of a degree-stated angle.
//
// A linear part matching the ideal one exactly contributes nothing whatever
// r0 is — the true radius is finite even where no reader states it — so a
// pure translation stays chargeable on a payload with no record radius.
// Every other unreadable term answers +Inf, a refusal rather than a bound.
//
// The second result is the linear term's own factor, an upper bound on
// ‖B(C) − R·B(P0)‖_F, which pathAreaUpper reads to bound how far the straight
// path between the two images stretches the mover's surface.
func poseDeviation(composed, placement r3.Transform, ideal idealPose, r0 float64) (float64, float64) {
	bc, bp := composed.Basis(), placement.Basis()
	colsC := [3]r3.Vec{bc.EX, bc.EY, bc.EZ}
	colsP := [3]r3.Vec{bp.EX, bp.EY, bp.EZ}
	var linear []ratInterval
	for j := range 3 {
		c, okC := ratVecOf(colsC[j])
		p, okP := ratVecOf(colsP[j])
		if !okC || !okP {
			return math.Inf(1), math.Inf(1)
		}
		image := ideal.rot.apply(pointVec(p))
		diff := ivVecSub(pointVec(c), image)
		linear = append(linear, diff[:]...)
	}
	tc, okC := ratVecOf(composed.Translation())
	tp, okP := ratVecOf(placement.Translation())
	if !okC || !okP {
		return math.Inf(1), math.Inf(1)
	}
	idealT := ivVecAdd(ivVecAdd(ideal.rot.apply(ivVecSub(pointVec(tp), ideal.pivot)), ideal.pivot), ideal.shift)
	dt := ivVecSub(pointVec(tc), idealT)
	transUp := ratSqrtUp(magnitudeSquaredUpper(dt[:]...))
	linSq := magnitudeSquaredUpper(linear...)
	if linSq.Sign() == 0 {
		return transUp, 0
	}
	linUp := ratSqrtUp(linSq)
	return absSumUpper(productUpper(linUp, r0), transUp), linUp
}

// basisSigmaLower is a proven lower bound on the smallest singular value of a
// placement's linear part B. r3 holds B orthonormal only to rounding, so
// BᵀB = I + E with E read exactly off the float columns; every eigenvalue of
// BᵀB is then at least 1 − ‖E‖_F, and the bound is that value's root,
// rounded down. An exactly orthonormal basis answers exactly 1; a defect too
// large to bound answers 0, which pathAreaUpper reads as a refusal.
func basisSigmaLower(t r3.Transform) float64 {
	e, ok := basisDefectUpper(t)
	switch {
	case !ok:
		return 0
	case e.Sign() == 0:
		return 1
	case e.Cmp(big.NewRat(1, 1)) >= 0:
		return 0
	}
	return ratSqrtDown(new(big.Rat).Sub(big.NewRat(1, 1), e))
}

// basisSigmaUpper is basisSigmaLower's mirror: a proven upper bound on the
// largest singular value of a transform's linear part B. Every eigenvalue of
// BᵀB = I + E is at most 1 + ‖E‖_F, and the bound is that value's root,
// rounded up. An exactly orthonormal basis answers exactly 1; a defect that
// cannot be read answers +Inf, a refusal rather than a bound.
func basisSigmaUpper(t r3.Transform) float64 {
	e, ok := basisDefectUpper(t)
	switch {
	case !ok:
		return math.Inf(1)
	case e.Sign() == 0:
		return 1
	}
	return ratSqrtUp(new(big.Rat).Add(big.NewRat(1, 1), e))
}

// basisDefectUpper is a proven upper bound on ‖BᵀB − I‖_F for a transform's
// linear part B, read exactly off its float columns and rooted upward; it is
// exactly zero for an exactly orthonormal basis. ok is false when a column is
// not finite or the root overflows.
func basisDefectUpper(t r3.Transform) (*big.Rat, bool) {
	b := t.Basis()
	var cols [3]ratVec
	for j, v := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := ratVecOf(v)
		if !ok {
			return nil, false
		}
		cols[j] = c
	}
	var defect []ratInterval
	for i := range 3 {
		for j := range 3 {
			e := ratAdd(ratMul(cols[i][0], cols[j][0]), ratMul(cols[i][1], cols[j][1]), ratMul(cols[i][2], cols[j][2]))
			if i == j {
				e.Sub(e, big.NewRat(1, 1))
			}
			defect = append(defect, pointInterval(e))
		}
	}
	sq := magnitudeSquaredUpper(defect...)
	if sq.Sign() == 0 {
		return new(big.Rat), true
	}
	e := floatRat(ratSqrtUp(sq))
	return e, e != nil
}

// pathAreaUpper bounds the mover's surface area at every point of the straight
// path between the float pose's image and the ideal pose's — the area
// sweptVolumeAllow's own contract asks for along the WHOLE path. Relative to
// the mover at rest, a point of that path is M_t·x + c with
// M_t = R + (1 − t)·(B(C) − R·B(P0))·B(P0)⁻¹, so
// ‖M_t‖₂ ≤ ‖R‖₂ + linear/sigma, and an area scales by at most ‖M_t‖₂². area
// is the rest area's upper bound (Area().Value + Bound), linear
// poseDeviation's second result, sigma basisSigmaLower of the rest placement.
//
// base is a proven upper bound on ‖R‖₂, the stretch base of
// docs/motion-check-design.md §5.1: exactly 1 for a Revolute and a Prismatic,
// whose ideal rotation is exactly orthogonal, and basisSigmaUpper of the
// between's From — at s = 1 the larger of From's and To's — for a Between,
// whose ideal linear part R(s·θ, n)·B(From) is orthogonal only as far as
// B(From) is. An exact linear part scales area by base² alone, and base 1
// leaves it unscaled.
func pathAreaUpper(area, linear, sigma, base float64) float64 {
	if linear == 0 {
		if base == 1 {
			return area
		}
		return productUpper(area, productUpper(base, base))
	}
	stretch := absSumUpper(base, divUpper(linear, sigma))
	return productUpper(area, productUpper(stretch, stretch))
}

// exceedsResolution reports whether the interval between two exact parameters
// is wider than the resolution. Parts stated in the same terms — both whole
// turns, or both base units — compare exactly, which is what lets a dyadic
// step equal to the resolution stop on it. Mixed parts compare the interval's
// smallest possible width, π at its lower enclosure, against the resolution's
// largest, π at its upper, so the floor never stops refinement early by
// rounding.
func exceedsResolution(p, q, res motionParam) bool {
	dTurn := new(big.Rat).Sub(q.turn, p.turn)
	dBase := new(big.Rat).Sub(q.base, p.base)
	switch {
	case dBase.Sign() == 0 && res.base.Sign() == 0:
		return new(big.Rat).Abs(dTurn).Cmp(res.turn) > 0
	case dTurn.Sign() == 0 && res.turn.Sign() == 0:
		return new(big.Rat).Abs(dBase).Cmp(res.base) > 0
	}
	twoPi := twoPiInterval()
	width := intervalAdd(intervalScale(twoPi, dTurn), pointInterval(dBase))
	lower := new(big.Rat)
	switch {
	case width.lo.Sign() > 0:
		lower = width.lo
	case width.hi.Sign() < 0:
		lower = new(big.Rat).Neg(width.hi)
	}
	upper := new(big.Rat).Mul(twoPi.hi, res.turn)
	upper.Add(upper, res.base)
	return lower.Cmp(upper) > 0
}

// moverRecordRadius is R0 of docs/motion-check-design.md §5.1: a proven upper
// bound on |p| for every point p of the mover's record before its own
// placement applies, read off the payload's own envelopes. It is an L1 bound,
// which dominates the Euclidean one. A prism reads its profile coordinate
// envelope (prism_payload.go) widened by its section displacement, and its
// sweep levels widened by their axial displacement; a revolve reads the
// generator envelope and axis anchor revolveCentroidGeometryBound already
// bounds a rotated material point with. Any other payload states no record
// radius and answers +Inf, which poseDeviation charges only where the pose's
// linear part departs from the ideal one.
func moverRecordRadius(ctx context.Context, b *Body) float64 {
	if ctx.Err() != nil {
		return math.Inf(1)
	}
	switch pl := b.payload.(type) {
	case prismPayload:
		coordUpper, err := profileCoordinateEnvelope(pl.profile, newFreeformWork(), pl.walks)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = absSumUpper(coordUpper, pl.sectionDelta)
		zUpper := absSumUpper(math.Max(math.Abs(pl.z0), math.Abs(pl.z1)), pl.axialDelta())
		return absSumUpper(
			vecL1(pl.frame.Origin()),
			productUpper(vecL1(pl.frame.U()), coordUpper),
			productUpper(vecL1(pl.frame.V()), coordUpper),
			productUpper(vecL1(pl.frame.N()), zUpper),
		)
	case revolvePayload:
		coordUpper, err := profileCoordinateUpper(pl.profile, newFreeformWork(), nil)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = absSumUpper(coordUpper, pl.sectionDelta)
		originUpper := vecL1(pl.frame.Origin())
		profileUpper := absSumUpper(
			originUpper,
			productUpper(vecL1(pl.frame.U()), coordUpper),
			productUpper(vecL1(pl.frame.V()), coordUpper),
		)
		axisUpper := absSumUpper(
			originUpper,
			productUpper(vecL1(pl.frame.U()), absSumUpper(pl.ax.aU, pl.ax.aUBound)),
			productUpper(vecL1(pl.frame.V()), absSumUpper(pl.ax.aV, pl.ax.aVBound)),
		)
		return absSumUpper(productUpper(3, profileUpper), productUpper(4, axisUpper))
	default:
		return math.Inf(1)
	}
}

// boxCornersExact is the mover's Bounds box inflated outward by its own Bound
// and by an extra margin, as exact rational extremes per axis. The true body
// lies inside the box inflated by its Bound (core §5.3), so the inflated box
// encloses every point the claim speaks for.
func boxCornersExact(box Box, extra *big.Rat) (lo, hi ratVec, ok bool) {
	minV, okMin := ratVecOf(box.Min)
	maxV, okMax := ratVecOf(box.Max)
	bound := floatRat(box.Bound.Base())
	if !okMin || !okMax || bound == nil {
		return ratVec{}, ratVec{}, false
	}
	pad := new(big.Rat).Add(bound, extra)
	for i := range 3 {
		lo[i] = new(big.Rat).Sub(minV[i], pad)
		hi[i] = new(big.Rat).Add(maxV[i], pad)
	}
	return lo, hi, true
}

// startCorners is the eight corners of the mover's Bounds box inflated by its
// own Bound, as exact rationals, each mapped to where the path STARTS:
// unchanged for a Revolute and a Prismatic, whose parameter 0 is the mover at
// rest, and through the between's From exactly for a Between (placeFrom). The
// true body at the start lies inside the convex hull of these eight points.
func startCorners(box Box, mf motionFrame) ([8]ratVec, bool) {
	lo, hi, ok := boxCornersExact(box, new(big.Rat))
	if !ok {
		return [8]ratVec{}, false
	}
	var out [8]ratVec
	for corner := range 8 {
		var x ratVec
		for i := range 3 {
			x[i] = lo[i]
			if corner&(1<<i) != 0 {
				x[i] = hi[i]
			}
		}
		out[corner] = mf.placeFrom(x)
	}
	return out, true
}

// moverAxisRadius is ρ_max of docs/motion-check-design.md §5.2: a proven
// upper bound on the distance from the rotation axis of every point of the
// mover, read ONCE off its Bounds box at its current placement. The box is
// inflated by its own Bound, its eight corners are mapped to the path's start
// (startCorners: through From exactly for a Between, whose screw axis passes
// nowhere near the rest box in general), and each image's squared distance
// from the axis line, |(x − c) × a|²/|a|², is taken exactly over rationals;
// distance from a line is convex, so its maximum over the hull of the images
// sits at one of them. ratSqrtUp roots the largest. A rotation about the axis
// and a slide along it both preserve every point's distance from it, so this
// one reading covers every pose.
func moverAxisRadius(b *Body, mf motionFrame) float64 {
	corners, ok := startCorners(b.bounds, mf)
	if !ok {
		return math.Inf(1)
	}
	a := mf.axis
	sq := ratAdd(ratMul(a[0], a[0]), ratMul(a[1], a[1]), ratMul(a[2], a[2]))
	best := new(big.Rat)
	for _, x := range corners {
		var w ratVec
		for i := range 3 {
			w[i] = new(big.Rat).Sub(x[i], mf.center[i])
		}
		cx := new(big.Rat).Sub(ratMul(w[1], a[2]), ratMul(w[2], a[1]))
		cy := new(big.Rat).Sub(ratMul(w[2], a[0]), ratMul(w[0], a[2]))
		cz := new(big.Rat).Sub(ratMul(w[0], a[1]), ratMul(w[1], a[0]))
		d := ratAdd(ratMul(cx, cx), ratMul(cy, cy), ratMul(cz, cz))
		d.Quo(d, sq)
		if d.Cmp(best) > 0 {
			best = d
		}
	}
	return ratSqrtUp(best)
}

// moverTravel is τ of docs/motion-check-design.md §5.2 as an exact rational:
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
func moverTravel(mf motionFrame, rho float64, p, q motionParam) *big.Rat {
	span := p.spanUpper(q)
	if mf.kind == motionPrismatic {
		return span
	}
	r := floatRat(rho)
	if r == nil {
		return nil
	}
	if mf.kind == motionRevolute {
		return span.Mul(span, r)
	}
	zero := motionParam{turn: new(big.Rat), base: new(big.Rat)}
	rate := r.Mul(r, zero.spanUpper(mf.theta))
	rate.Add(rate, new(big.Rat).Abs(mf.slide))
	return span.Mul(span, rate)
}

// moverSweptBox is §6 step 3's swept box: every point the mover occupies over
// the whole path, as exact rational extremes per axis. A Revolute or a
// Prismatic reads its Bounds box at rest, inflated by its own Bound plus
// travel — the farthest any of its points moves from where it sits now. A
// Between's path starts at From, which the rest box has not undergone, so it
// reads the From-placed box: the axis-aligned hull of startCorners, which
// already carry the box's Bound, inflated by travel and by nothing else.
func moverSweptBox(box Box, mf motionFrame, travel *big.Rat) (ratVec, ratVec, bool) {
	if travel == nil {
		return ratVec{}, ratVec{}, false
	}
	if mf.kind != motionBetween {
		return boxCornersExact(box, travel)
	}
	corners, ok := startCorners(box, mf)
	if !ok {
		return ratVec{}, ratVec{}, false
	}
	var lo, hi ratVec
	for i := range 3 {
		lo[i], hi[i] = corners[0][i], corners[0][i]
		for _, c := range corners[1:] {
			if c[i].Cmp(lo[i]) < 0 {
				lo[i] = c[i]
			}
			if c[i].Cmp(hi[i]) > 0 {
				hi[i] = c[i]
			}
		}
		lo[i] = new(big.Rat).Sub(lo[i], travel)
		hi[i] = new(big.Rat).Add(hi[i], travel)
	}
	return lo, hi, true
}

// sweptBoxLower is §6 step 3's swept-box exclusion for one (mover, static)
// pair, decided over exact rationals: the mover's swept box (moverSweptBox)
// against the static body's box inflated by its own Bound. Boxes separated by
// a strictly positive gap along some axis prove the pair apart at every
// parameter, and the largest such axis gap is a proven lower bound on the
// pair's distance over the whole path. ok is false when the boxes do not
// separate.
func sweptBoxLower(mLo, mHi ratVec, static Box) (float64, bool) {
	sLo, sHi, okS := boxCornersExact(static, new(big.Rat))
	if !okS {
		return 0, false
	}
	var best *big.Rat
	for i := range 3 {
		for _, gap := range []*big.Rat{new(big.Rat).Sub(sLo[i], mHi[i]), new(big.Rat).Sub(mLo[i], sHi[i])} {
			if gap.Sign() > 0 && (best == nil || gap.Cmp(best) > 0) {
				best = gap
			}
		}
	}
	if best == nil {
		return 0, false
	}
	lower := ratFloatDown(best)
	return lower, lower > 0
}
