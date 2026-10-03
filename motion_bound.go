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
//     cosine enclosed by moments_trig.go's turnSinCosInterval;
//   - η, the proven distance between the float pose the kernel measured and
//     the ideal pose the claim is about (poseDeviation, §5.1);
//   - R0, the record-coordinate radius η is charged at (moverRecordRadius);
//   - ρ_max, the largest distance from the rotation axis of any point of a
//     mover (moverAxisRadius, §5.2);
//   - the travel bound τ and the swept-box exclusion (§5.2, §6);
//   - the area the collision transfer charges its swept-volume allowance at
//     (pathAreaUpper, basisSigmaLower, §5.1);
//   - the resolution floor's width comparison (exceedsResolution, §6).

// motionParam is one motion parameter's exact denotation. An angle denotes
// θ = 2π·turn + base radians: a degree-stated angle is an exact rational turn
// (units.Degree's own factor is a rounded π/180, so the degree count, not the
// factor, is what the caller stated), and any other angle unit is base =
// magnitude × factor radians, read exactly. A length denotes base millimetres
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

// idealPose is the exact rigid motion T*(θ) a motion parameter denotes,
// enclosed over rationals: x ↦ rot·(x − pivot) + pivot + shift. A revolute
// carries the Rodrigues rotation about its exact unit axis Axis/|Axis| and
// its exact Center as pivot; a prismatic carries the identity rotation and
// the shift d·Dir/|Dir|.
type idealPose struct {
	rot   ivMat
	pivot ivVec
	shift ivVec
}

// motionFrame is the exact reading of a Motion's own fields every ideal pose
// is built from.
type motionFrame struct {
	revolute bool
	axis     ratVec      // Axis (revolute) or Dir (prismatic), exact
	unit     ratInterval // 1/|axis|
	center   ratVec      // the revolute's pivot; zero for a prismatic
}

func newMotionFrame(spec motionSpec) (motionFrame, bool) {
	dirVec := spec.dir
	center := r3.Vec{}
	if spec.revolute {
		dirVec, center = spec.axis, spec.center
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
	return motionFrame{revolute: spec.revolute, axis: axis, unit: unit, center: pivot}, true
}

// at builds the ideal pose for the exact parameter p. The revolute's matrix
// is R = cos·I + sin·[k]× + (1 − cos)·k kᵀ with k = a/|a|: k kᵀ = a aᵀ/|a|²
// is exact, and [k]× = [a]×·(1/|a|) carries unitScaleInterval's enclosure.
func (mf motionFrame) at(p motionParam) idealPose {
	zero := pointInterval(new(big.Rat))
	one := pointInterval(big.NewRat(1, 1))
	pose := idealPose{pivot: pointVec(mf.center), shift: ivVec{zero, zero, zero}}
	if !mf.revolute {
		for i := range 3 {
			for j := range 3 {
				pose.rot[i][j] = zero
			}
			pose.rot[i][i] = one
			pose.shift[i] = intervalMul(intervalScale(mf.unit, mf.axis[i]), pointInterval(p.base))
		}
		return pose
	}
	sin, cos := paramSinCos(p)
	sq := ratAdd(ratMul(mf.axis[0], mf.axis[0]), ratMul(mf.axis[1], mf.axis[1]), ratMul(mf.axis[2], mf.axis[2]))
	oneMinusCos := intervalSub(one, cos)
	a := mf.axis
	cross := [3][3]*big.Rat{
		{new(big.Rat), new(big.Rat).Neg(a[2]), a[1]},
		{a[2], new(big.Rat), new(big.Rat).Neg(a[0])},
		{new(big.Rat).Neg(a[1]), a[0], new(big.Rat)},
	}
	sinUnit := intervalMul(sin, mf.unit)
	for i := range 3 {
		for j := range 3 {
			outer := new(big.Rat).Quo(ratMul(a[i], a[j]), sq)
			entry := intervalAdd(intervalScale(sinUnit, cross[i][j]), intervalScale(oneMinusCos, outer))
			if i == j {
				entry = intervalAdd(entry, cos)
			}
			pose.rot[i][j] = entry
		}
	}
	return pose
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
	b := t.Basis()
	var cols [3]ratVec
	for j, v := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := ratVecOf(v)
		if !ok {
			return 0
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
		return 1
	}
	e := floatRat(ratSqrtUp(sq))
	if e == nil || e.Cmp(big.NewRat(1, 1)) >= 0 {
		return 0
	}
	return ratSqrtDown(new(big.Rat).Sub(big.NewRat(1, 1), e))
}

// pathAreaUpper bounds the mover's surface area at every point of the straight
// path between the float pose's image and the ideal pose's — the area
// sweptVolumeAllow's own contract asks for along the WHOLE path. Relative to
// the mover at rest, a point of that path is M_t·x + c with
// M_t = R + (1 − t)·(B(C) − R·B(P0))·B(P0)⁻¹ and R exactly orthogonal, so
// ‖M_t‖₂ ≤ 1 + linear/sigma, and an area scales by at most ‖M_t‖₂². area is
// the rest area's upper bound (Area().Value + Bound), linear poseDeviation's
// second result, sigma basisSigmaLower of the rest placement. An exact linear
// part leaves area unscaled.
func pathAreaUpper(area, linear, sigma float64) float64 {
	if linear == 0 {
		return area
	}
	stretch := absSumUpper(1, divUpper(linear, sigma))
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

// moverAxisRadius is ρ_max of docs/motion-check-design.md §5.2: a proven
// upper bound on the distance from the rotation axis of every point of the
// mover, read ONCE off its Bounds box at its current placement. The box is
// inflated by its own Bound, and each of its eight corners' squared distance
// from the axis line, |(x − c) × a|²/|a|², is taken exactly over rationals;
// distance from a line is convex, so its maximum over the box sits at a
// corner. ratSqrtUp roots the largest. A rotation about the axis preserves
// every point's distance from it, so this one reading covers every pose.
func moverAxisRadius(b *Body, mf motionFrame) float64 {
	lo, hi, ok := boxCornersExact(b.bounds, new(big.Rat))
	if !ok {
		return math.Inf(1)
	}
	a := mf.axis
	sq := ratAdd(ratMul(a[0], a[0]), ratMul(a[1], a[1]), ratMul(a[2], a[2]))
	best := new(big.Rat)
	for corner := range 8 {
		var w ratVec
		for i := range 3 {
			x := lo[i]
			if corner&(1<<i) != 0 {
				x = hi[i]
			}
			w[i] = new(big.Rat).Sub(x, mf.center[i])
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
// chord, and ρ ≤ rho. No rounding is committed: rho is already an upper
// bound, and the span is taken at π's upper enclosure.
func moverTravel(revolute bool, rho float64, p, q motionParam) *big.Rat {
	span := p.spanUpper(q)
	if !revolute {
		return span
	}
	r := floatRat(rho)
	if r == nil {
		return nil
	}
	return span.Mul(span, r)
}

// sweptBoxLower is §6 step 3's swept-box exclusion for one (mover, static)
// pair, decided over exact rationals: the mover's Bounds box inflated by its
// own Bound plus travel — the farthest any of its points moves from where it
// sits now over the whole path — against the static body's box inflated by
// its own Bound. Inflated boxes separated by a strictly positive gap along
// some axis prove the pair apart at every parameter, and the largest such
// axis gap is a proven lower bound on the pair's distance over the whole
// path. ok is false when the boxes do not separate.
func sweptBoxLower(mover, static Box, travel *big.Rat) (float64, bool) {
	if travel == nil {
		return 0, false
	}
	mLo, mHi, okM := boxCornersExact(mover, travel)
	sLo, sHi, okS := boxCornersExact(static, new(big.Rat))
	if !okM || !okS {
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
