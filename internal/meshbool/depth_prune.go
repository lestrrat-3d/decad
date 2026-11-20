package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The depth threshold prune (agent-docs/evaluator-design.md §9, "The witness
// depth scan passes over blocks that cannot settle a candidate"). depthBeyond's
// block scan skips a block or facet box whose PointBoxDist P proves that every
// facet it covers reads a depthFacetDistance D above a threshold t. The
// threshold is t = fl(fl(fl(b + E) + R̄)·(1 + 2^-38)), a positive float with
// fl(fl(fl(t·c) − R̄) − E) > b, where R̄ >= R = PointRoundBound is
// depthRoundBoundUpper, which needs no exact arithmetic. The certificate
// fl(fl(fl(x·c) − r) − e) never decreases as x grows and never increases as r
// grows, because each step rounds a monotone function of the step before it.
// So the certificate depthCertificate gives any minimum at or above t, with
// the true R, is above b: any such minimum certifies p deeper than b.
//
// The lower bound. Write u = 2^-53, M the coordinate magnitude of
// depthCertificate (at most 2^500 whenever t is set), d* the exact distance
// from the rounded point pf to a facet with float corners, and κ an upper
// bound on L²/|N|, with L the facet's longest edge and N the exact normal of
// its corners. Then
//
//	D >= (1 − 20u)·d* − S,  S = 2σκ + 20u·M + 2^-1000,  σ = 83u·M + 2^-568,
//
// for a facet whose edges are all at least 2^-499.5 long, and S = 9u·M +
// 2^-1000 for a facet with N = 0. The error terms below follow the
// depthCertificate proof and add the directions it did not need.
//
//   - An edge reading. pointSegDist3 clamps its parameter into [0, 1], so the
//     exact point q0 + t·(q1 − q0) lies on the closed edge whatever the
//     rounding of t. The computed point lies within 2.02u·|q1 − q0| + √3·u·M
//     <= 8.74u·M of it, and the subtraction and Len (fast path within 2.6u,
//     nested math.Hypot within 8.3u, both relative) lose at most 10u of the
//     result. Every point of the edge is at least d* from pf, so the reading
//     is at least (1 − 10u)·d* − 8.8u·M.
//   - The foot reading. With N' = 2^shift·N the depth normal before rounding,
//     w = pf − t0 (|w| <= 2√3·M), h* = |N'·w|/|N'| the plane distance and f
//     the exact foot, the computed n·w errs by at most 5.01u·|N'||w| and n·n
//     by 5.03u·|N'|², so the computed step h satisfies |h|·|N'| >= (1 −
//     7.05u)·h* − 5.03u·|w|. The foot coordinates add 2.01u·|n||h| + u·|pf|,
//     and the final subtraction and Len lose at most 10u. So the reading is at
//     least (1 − 20u)·h* − 20u·M. The computed foot g lies within 14.1u·|w| +
//     u·|pf| <= 50.6u·M of f.
//   - A foot read inside. PointInTriangle's edge value for edge e = t_{j+1} −
//     t_j at g errs from (e × (f − t_j))·N' by at most |e||N'| times 8.9u·|f −
//     t_j| + 1.01·|g − f|, plus 2^-1069 for underflow. That exact value is
//     |e||N'| times the signed in-plane distance from f to the edge's line,
//     |N'| >= 1 and |e| >= 2^-499.5, and |f − t_j| <= |pf − t_j| <= 2√3·M, so
//     an accepted foot puts f within σ of every edge's inner half-plane. That
//     region is the facet grown by σ about its incentre. Its corners are the
//     farthest points from the facet, σ/sin(α/2) from the corner of angle α,
//     and 1/sin(α/2) <= 2/sin α = 2|e_a||e_b|/|N| <= 2κ. So f lies within 2σκ
//     of the facet, d* <= h* + 2σκ, and the reading is at least (1 − 20u)·d*
//     − 2σκ − 20u·M. A facet with N = 0 has a zero depth normal and always
//     reads its edges.
//   - Underflow in the readings costs at most 2^-1000 beyond the terms above,
//     because n·n >= 1 and every quotient and product it loses is within a
//     few multiples of 2^-1075.
//
// The box. A facet box holds its facet's corners, checked when depthKappa is
// built, and a block holds the boxes it covers. PointBoxDist reads at most
// (1+3.6u) times the exact box distance plus 2^-536 (its squares may
// underflow), and the exact box distance is at most d* for every facet the box
// holds. So every such facet has D >= (1 − 24u)·P − 2^-536 − S. depthPrunable
// tests fl(fl(P·(1 − 2^-46)) − s) > t with s = fl(fl(κ·a1) + a2), a1 =
// fl(2^-44·M + 2^-490) and a2 = fl(2^-47·M + 2^-490). Rounding to nearest is
// monotone and t is a float, so the test passing means the exact difference
// fl(P·(1 − 2^-46)) − s exceeds t, and fl(P·(1 − 2^-46)) <= (1 − 2^-46)(1 +
// u)·P <= (1 − 24u)·P. With κ the box's largest facet κ, s is at least
// 0.98κ·(512u·M + 2^-490) + 0.98·(64u·M + 2^-490), which covers S + 2^-536
// for every facet the box holds. So every one of them reads D > t.
//
// The emulation. The prune must leave depthBeyond's answer and its hint
// exactly as the block scan without it leaves them; both depend on the order
// in which the scan lowers its running minimum. The pruned scan keeps that
// scan's minimum B until it passes over a box, and from then on a flag says
// that the minimum B' of the scan without the prune lies in [min(B, t'), B]
// for some t' > t: B' is B lowered only by facets inside passed-over boxes,
// which read above t. The scan passes over a box only while B > t and the box
// reads above t (the band). For a box outside the band, P >= B means the
// unpruned scan skips it too (B' <= B), and otherwise P <= t < B' or the flag
// is clear (B' = B), so the unpruned scan enters it too. A box in the band is
// passed over when depthPrunable proves it above t. A facet box in the band
// that depthPrunable does not prove is read, and with the flag set a reading
// above t passes it over too. Any other box in the band the scan enters as
// the unpruned scan does while the flag is clear. With the flag set it cannot
// know whether the unpruned scan enters that box, so it starts over without
// the prune. So outside the passed-over boxes both scans take the same facets
// in the same order. A facet that settles has fl(D·c) <= b, while the
// certificate of t above b gives fl(t·c) > b, so it reads below t and is
// never passed over: both scans settle on the same facet. Once B drops to t
// or below, B' = B and the flag clears. A scan that ends with the flag set
// has B' >= min(B, t') > t, so the certificate of B' is above b and the
// answer is "deeper". Otherwise B' = B and the certificate decides as before.
//
// The cost. Let U be the number of readings (box distances and facet
// distances) the unpruned scan makes. Until it starts over, the pruned scan
// enters a box only when the unpruned scan does, so it reads no box distance
// the unpruned scan does not. Its only other reading is a facet in the band
// with the flag set, at most one for each facet box the unpruned scan reads.
// A scan that starts over has done at most that much and then runs the
// unpruned scan, so its total is at most 2·U plus one facet reading for each
// facet box the unpruned scan reads. That bound is on all readings together:
// depthFacetDistance calls alone can exceed twice the unpruned scan's on a
// small scan. The prune itself never computes PointRoundBound.
const (
	// depthPruneShrink is 1 − 128u, which covers the 24u the box argument
	// needs and the rounding of the product.
	depthPruneShrink = 1 - 0x1p-46
	// depthSlackCoef is 512u, about three times the 166u of 2σ.
	depthSlackCoef = 0x1p-44
	// depthSlackBase is 64u, about three times the 20u of the readings.
	depthSlackBase = 0x1p-47
	// depthSlackFloor covers every underflow term of the lower bound.
	depthSlackFloor = 0x1p-490
	// depthKappaEdgeFloor is the least computed squared edge length a facet
	// may have and still carry a finite κ: its exact edges are then at least
	// 2^-499.5 long.
	depthKappaEdgeFloor = 0x1p-998
	// depthThresholdPad lifts b + E + R past the certificate's factor c, which
	// shrinks by 1e-12 < 2^-39.
	depthThresholdPad = 1 + 0x1p-38
	// depthScanRestart is what the pruned scan returns when it must start over
	// without the prune.
	depthScanRestart = -2
)

// depthKappaOf is the facet's κ for the depth threshold prune: an upper bound
// on L²/|N|, L the longest edge of the float corners t and N their exact
// normal, read from the depth normal n = depthNormalOf(N) and its scaling
// power shift. It is +Inf, which no block prune can pass, when box does not
// hold every corner (a NaN coordinate fails), when an edge's computed squared
// length is below depthKappaEdgeFloor or not finite, or when the bound
// overflows. It is 0 for a zero normal, whose facet always reads its edges.
func depthKappaOf(t [3]r3.Vec, box [2]r3.Vec, n r3.Vec, shift int) float64 {
	for _, c := range t {
		if !(box[0].X <= c.X && c.X <= box[1].X && box[0].Y <= c.Y && c.Y <= box[1].Y && box[0].Z <= c.Z && c.Z <= box[1].Z) {
			return math.Inf(1)
		}
	}
	if n == (r3.Vec{}) {
		return 0
	}
	lo, hi := math.Inf(1), 0.0
	for k := range 3 {
		e := t[(k+1)%3].Sub(t[k])
		l := e.Dot(e)
		lo, hi = min(lo, l), max(hi, l)
	}
	if !(lo >= depthKappaEdgeFloor) || !(hi <= math.MaxFloat64) {
		return math.Inf(1)
	}
	// The computed squared length is at least (1 − 5u) of the exact one, and
	// sqrt(n·n) at most (1 + 4.1u) of |N'| = 2^shift·|N|. The 2^-40 pad covers
	// both and the two roundings here; Ldexp is exact or overflows to +Inf.
	r := float64(float64(hi/math.Sqrt(n.Dot(n))) * (1 + 0x1p-40))
	return math.Ldexp(r, shift)
}

// setDepthKappa fills depthKappa from Verts, Tris, Boxes, Norms and
// depthNorms: level 0 holds each facet's depthKappaOf, and level k+1 the
// largest value of each run of depthFanout entries of level k, so it lines up
// with depthLevels[k]. It leaves depthKappa nil when the mesh has no depth
// blocks or no prepared depth normals, which turns the prune off.
func (bm *BoolMesh) setDepthKappa() {
	bm.depthKappa = nil
	n := len(bm.Tris)
	if len(bm.depthLevels) == 0 || len(bm.Norms) != n || len(bm.depthNorms) != n || len(bm.Boxes) != n {
		return
	}
	level := make([]float64, n)
	for i := range bm.Tris {
		level[i] = depthKappaOf(TriCorners(bm, i), bm.Boxes[i], bm.depthNorms[i], depthNormalShift(bm.Norms[i]))
	}
	bm.depthKappa = append(bm.depthKappa, level)
	for range bm.depthLevels {
		up := make([]float64, 0, (len(level)+depthFanout-1)/depthFanout)
		for start := 0; start < len(level); start += depthFanout {
			k := 0.0
			for _, v := range level[start:min(start+depthFanout, len(level))] {
				k = max(k, v)
			}
			up = append(up, k)
		}
		level = up
		bm.depthKappa = append(bm.depthKappa, level)
	}
}

// depthKappaFit reports whether depthKappa lines up with the facets and with
// depthLevels.
func (bm *BoolMesh) depthKappaFit() bool {
	if len(bm.depthKappa) != len(bm.depthLevels)+1 || len(bm.depthKappa[0]) != len(bm.Tris) {
		return false
	}
	for k, level := range bm.depthLevels {
		if len(bm.depthKappa[k+1]) != len(level) {
			return false
		}
	}
	return true
}

// setThreshold computes the prune threshold t and the slack factors a1, a2
// on first use. It leaves t at +Inf, which turns the prune off, when b is
// negative, infinite or NaN, when M exceeds depthReachLimit or is not finite,
// or when t is not finite and positive or its certificate, taken with R̄ in
// place of R, is not above b.
func (s *depthScan) setThreshold() {
	s.tdSet = true
	s.td = math.Inf(1)
	if !(s.b >= 0) || math.IsInf(s.b, 1) {
		return
	}
	m := max(math.Abs(s.pf.X), math.Abs(s.pf.Y), math.Abs(s.pf.Z), s.other.depthCoordReach())
	if !(m <= depthReachLimit) {
		return
	}
	e := depthErrBound(m)
	r := depthRoundBoundUpper(s.pf)
	t := float64(float64(float64(s.b+e)+r) * depthThresholdPad)
	if !(t > 0) || math.IsInf(t, 1) || !(depthCertFrom(t, r, e) > s.b) {
		return
	}
	s.td = t
	s.a1, s.a2 = depthSlackFactors(m)
}

// depthRoundBoundUpper is R̄, an upper bound on PointRoundBound(p, pf) for
// every exact p whose nearest float is pf, computed without exact
// arithmetic: Radius3D(ProvenUpRound(g)), g the largest gap between a
// coordinate magnitude of pf and the next float above it. Each coordinate of
// p lies within half the gap on its side of pf, which is at most g/2, so the
// rounded per-coordinate gap PointRoundBound reads is at most g, and
// ProvenUpRound and Radius3D never decrease. It is +Inf when a coordinate is
// the largest finite float.
func depthRoundBoundUpper(pf r3.Vec) float64 {
	gap := func(c float64) float64 {
		a := math.Abs(c)
		return math.Nextafter(a, math.Inf(1)) - a
	}
	return proofbound.Radius3D(proofbound.ProvenUpRound(max(gap(pf.X), gap(pf.Y), gap(pf.Z))))
}

// depthSlackFactors are the slack factors a1 = fl(2^-44·m + 2^-490) and a2 =
// fl(2^-47·m + 2^-490) for the coordinate magnitude m.
func depthSlackFactors(m float64) (float64, float64) {
	return float64(float64(depthSlackCoef*m) + depthSlackFloor), float64(float64(depthSlackBase*m) + depthSlackFloor)
}

// inBand reports whether a box at distance reading d may be passed over:
// both d and the running minimum lie above the threshold. The threshold is
// above b, so it is computed only once both lie above b.
func (s *depthScan) inBand(d float64) bool {
	if !(d > s.b && s.best > s.b) {
		return false
	}
	if !s.tdSet {
		s.setThreshold()
	}
	return d > s.td && s.best > s.td
}

// depthPrunable reports whether a box at distance reading d whose facets'
// largest κ is kappa proves that each of those facets reads above t.
func (s *depthScan) depthPrunable(d, kappa float64) bool {
	slack := float64(float64(kappa*s.a1) + s.a2)
	return float64(float64(d*depthPruneShrink)-slack) > s.td
}

// prunedBlock is block with the threshold prune. It returns the settling
// facet, -1, or depthScanRestart when the scan must start over unpruned. It
// passes over a box in the band that depthPrunable proves. Any other box in
// the band it enters as block does while it has passed over nothing, and
// otherwise starts over.
func (s *depthScan) prunedBlock(level, j int) int {
	d := PointBoxDist(s.pf, s.other.depthLevels[level][j])
	if d >= s.best {
		return -1
	}
	if s.inBand(d) {
		if s.depthPrunable(d, s.other.depthKappa[level+1][j]) {
			s.passed = true
			return -1
		}
		if s.passed {
			return depthScanRestart
		}
	}
	lo := j * depthFanout
	if level == 0 {
		for i := lo; i < min(lo+depthFanout, len(s.other.Tris)); i++ {
			if r := s.prunedFacet(i); r != -1 {
				return r
			}
		}
		return -1
	}
	for k := lo; k < min(lo+depthFanout, len(s.other.depthLevels[level-1])); k++ {
		if r := s.prunedBlock(level-1, k); r != -1 {
			return r
		}
	}
	return -1
}

// prunedFacet is one step of facets with the threshold prune. A facet read
// while the pass-over flag is clear joins the minimum, as the unpruned scan
// takes it too. With the flag set, a facet in the band that depthPrunable
// does not prove is read: a reading above t passes it over, and any other
// starts the scan over.
func (s *depthScan) prunedFacet(i int) int {
	d := PointBoxDist(s.pf, s.other.Boxes[i])
	if d >= s.best {
		return -1
	}
	band := s.inBand(d)
	if band && s.depthPrunable(d, s.other.depthKappa[0][i]) {
		s.passed = true
		return -1
	}
	dist := depthFacetDistance(s.pf, TriCorners(s.other, i), s.other.depthNormal(i))
	if band && s.passed {
		if dist > s.td {
			return -1
		}
		return depthScanRestart
	}
	s.best = math.Min(s.best, dist)
	if s.best*(1-1e-12) <= s.b {
		return i
	}
	if s.passed && s.best <= s.td {
		s.passed = false
	}
	return -1
}
