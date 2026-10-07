package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarsweep"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// This file continues the general rotating sweep from an initial touch
// (docs/multibody-dynamics-design.md §10.2 and §10.3). Both proofs read one
// SUPPORT PLANE: a face plane of one body S that every vertex of the other
// body M lies on or in front of, touching it at the contact set. When every
// vertex of S lies on or behind it, S lies in its vertices' hull and moves
// rigidly with its plane, and M lies in its vertices' hull, so the lowest
// M vertex height above the moving plane bounds the pair's separation below.
// A FACE-LOCAL plane (§10.6), whose owner has material in front of it, the
// floor of a tray, adds the column test: every S triangle with a vertex in
// front of the plane projects strictly apart from M's ideal path box, so that
// material lies at least the lateral clearance away from M, and the vertex
// heights bound the distance to the rest of S.
//
// Each M vertex height h(u) has an exact start value and start rate, and a
// second derivative bounded by the §10.2 curvature with the vertex's own
// p'' term (§10.8); Taylor's theorem then bounds h(u) between
// h(0) + h'(0)·u ∓ K_p·u² with K_p half that bound. A departure needs every
// contact rate positive and every other vertex positive; a band track takes
// any contact rate, holds a lifted vertex that closes no faster than the
// request's RestSpeed on both sides of the plane (§10.8), and publishes the
// depth those bounds allow.

// planarMotion is the sweep's exact first-order rigid motion.
type planarMotion = planarsweep.Motion

func planarMotionOf(p *rotationalSweepPath) (planarMotion, bool) {
	return planarsweep.MotionOf(planarsweep.MotionInput{
		Points: p.startPoints, Delta: p.path.delta, Duration: p.path.duration,
		Velocity: p.velocity, Frame: p.frame,
		Drift: p.path.drift != nil, Screw: p.path.screw != nil,
	})
}

// planarSupport is one support plane of the touching pair. Paths index the
// sweep's two bodies: m touches the plane, s owns it.
type planarSupport struct {
	m, s     int
	normal   proofarith.DyV3  // exact outward normal of S there, not unit length
	origin   proofarith.DyV3  // a start vertex of S on the plane
	tri      int              // the S triangle the plane was read from
	heights  []*big.Rat       // n·(p − origin) for every M start vertex, all >= 0
	rates    []*big.Rat       // exact n·(dp/du − dq/du) at u = 0 for every M vertex
	contact  []int            // the M vertices with zero height, ascending
	lifted   []int            // the M vertices with a positive height within the support band (§10.5), ascending
	rested   map[int]struct{} // the lifted vertices closing no faster than the request's RestSpeed (§10.8)
	spin     []*big.Rat       // upper bound on |ω_M|·|ω_M×(p − c_M)| for every M vertex (§10.8)
	local    bool             // S has a vertex strictly in front of the plane: a face-local plane (§10.6)
	nLow     *big.Rat         // lower bound on |n|
	nHigh    *big.Rat         // upper bound on |n|
	motionM  planarMotion
	motionS  planarMotion
	pathS    *rotationalSweepPath
	pathM    *rotationalSweepPath
	duration *big.Rat
}

// planarSupports lists every support plane of the initial touch, in a fixed
// order: S is the sweep's B body first, then its A body, and planes follow
// S's triangle order, each plane once.
func (r *rotationalPairSweep) planarSupports(poll func() error) ([]planarSupport, error) {
	paths := [2]*rotationalSweepPath{&r.a, &r.b}
	var motions [2]planarMotion
	var spins [2][]*big.Rat
	for i, path := range paths {
		motion, ok := planarMotionOf(path)
		if !ok {
			return nil, nil
		}
		spin, ok := vertexSpins(path, motion)
		if !ok {
			return nil, nil
		}
		motions[i], spins[i] = motion, spin
	}
	rest := new(big.Rat)
	if r.req.RestSpeed != (units.Value{}) {
		speed, ok := exactBaseValue(r.req.RestSpeed)
		if !ok {
			return nil, nil
		}
		rest = speed
	}
	var out []planarSupport
	for _, s := range []int{1, 0} {
		m := 1 - s
		S, M := paths[s], paths[m]
		// A named local, read once: no array element is re-read across the
		// calls below.
		spinM := spins[m]
		boxesM := make([]proofarith.FloatBox3, len(M.startPoints))
		for i, v := range M.startPoints {
			boxesM[i] = proofarith.DvFloatBox(v)
		}
		tried := make(map[string]struct{})
		var key []byte
		for t, tri := range S.solid.Tris {
			if err := poll(); err != nil {
				return nil, err
			}
			a := S.startPoints[tri[0]]
			n := proofarith.DvCross(proofarith.DvSub(S.startPoints[tri[1]], a),
				proofarith.DvSub(S.startPoints[tri[2]], a))
			if proofarith.DvIsZero(n) {
				continue
			}
			key = planarPlaneKey(key[:0], n, a)
			if _, ok := tried[string(key)]; ok {
				continue
			}
			tried[string(key)] = struct{}{}
			if planarSupportRuledOut(boxesM, n, a, r.req.ContactRequest) {
				continue
			}
			support, ok, err := planarSupportOf(S, M, n, a, r.req.ContactRequest, poll)
			if err != nil {
				return nil, err
			}
			if !ok {
				continue
			}
			support.m, support.s, support.tri = m, s, t
			support.motionM, support.motionS = motions[m], motions[s]
			support.pathM, support.pathS = M, S
			support.duration = r.a.path.duration
			support.rates = planarsweep.Rates(M.startPoints, n, support.motionM, support.motionS)
			support.spin = spinM
			support.rested = restedVertices(&support, rest)
			out = append(out, support)
		}
	}
	return out, nil
}

// planarPlaneTried reports whether n through a names a plane already tried:
// the same outward direction and the same offset.
func planarPlaneTried(tried []planarSupport, n, a proofarith.DyV3) bool {
	for _, p := range tried {
		if proofarith.DvIsZero(proofarith.DvCross(n, p.normal)) &&
			proofarith.DvDot(n, p.normal).Sign() > 0 &&
			proofarith.DvDot(p.normal, proofarith.DvSub(a, p.origin)).Sign() == 0 {
			return true
		}
	}
	return false
}

// planarPlaneKey appends to b a key naming the plane through a with nonzero
// normal n: n's primitive integer direction d (proofarith.DvPrimitive), sign
// kept, and the exact offset d·a. Two keys are equal exactly when
// planarPlaneTried would match the two planes. The directions agree exactly
// when one normal is a positive multiple of the other, which is the cross
// product's zero with a positive dot product. Each normal is then a positive
// multiple of d, so p.normal·(a − p.origin) is zero exactly when d·a equals
// d·p.origin.
func planarPlaneKey(b []byte, n, a proofarith.DyV3) []byte {
	d := proofarith.DvPrimitive(n)
	for _, c := range d {
		b = c.AppendKey(b)
	}
	return proofarith.DvDot(d, a).AppendKey(b)
}

// planarSupportRuledOut reports whether planarSupportOf must reject the plane
// through a with normal n, decided from outward float enclosures of the M
// vertex heights h = n·v − n·a. A true answer holds only where the exact test
// reaches the same rejection; false decides nothing. Two cases are proven:
//
//   - an enclosure entirely below zero: that vertex's exact height is
//     negative, and planarSupportOf rejects a plane with any M vertex behind
//     it;
//   - every enclosure entirely above t, with t zero, or, under a positive
//     band, a float at or above sqrt(band²·n·n) (proofarith.DySqrtUp): every
//     exact height h then exceeds t ≥ 0, so h is positive and h² exceeds
//     band²·n·n. No vertex is in contact and none is lifted, and
//     planarSupportOf rejects an empty support set.
//
// planarSupportOf accepts only when both of these fail. Its other early
// answers, a face-local plane's, are rejections too, so a proven rejection
// stands for whichever one the exact test reaches first.
func planarSupportRuledOut(boxesM []proofarith.FloatBox3, n, a proofarith.DyV3, req ContactRequest) bool {
	nBox := proofarith.DvFloatBox(n)
	cLo, cHi := proofarith.FloatBounds(proofarith.DvDot(n, a))
	// Every comparison below is false on a NaN end, so a NaN decides nothing.
	allAbove, minLo := true, math.Inf(1)
	for _, box := range boxesM {
		lo, hi := proofarith.DotSubEnclosure(nBox, box, cLo, cHi)
		if hi < 0 {
			return true
		}
		if !(lo > 0) {
			allAbove = false
			continue
		}
		minLo = math.Min(minLo, lo)
	}
	if !allAbove {
		return false
	}
	band := supportBandOf(req)
	if band.Sign() <= 0 {
		return true
	}
	limit := proofarith.DyMul(proofarith.DyMul(band, band), proofarith.DvDot(n, n))
	return minLo > proofarith.DySqrtUp(limit)
}

// planarSupportOf checks one plane: every M vertex on or in front of it, and
// a nonempty support set (§10.5): at least one M vertex on it, or, under a
// positive SupportBand, within the band above it, h² <= band²·n·n compared
// exactly. A plane with an S vertex strictly in front of it is face-local
// (§10.6): S must only translate, so the column test can read S's start
// triangles less its translation, and the plane must be one flat face of S
// (planarSupportFace). The column test itself depends on the horizon and runs
// in the departure's and the band's grid searches.
func planarSupportOf(S, M *rotationalSweepPath, n, a proofarith.DyV3, req ContactRequest,
	poll func() error) (planarSupport, bool, error) {
	band := supportBandOf(req)
	limit := proofarith.DyMul(proofarith.DyMul(band, band), proofarith.DvDot(n, n))
	local := false
	for _, v := range S.startPoints {
		if err := poll(); err != nil {
			return planarSupport{}, false, err
		}
		if proofarith.DvDot(n, proofarith.DvSub(v, a)).Sign() > 0 {
			local = true
			break
		}
	}
	if local && (S.path.drift != nil || S.path.screw != nil) {
		return planarSupport{}, false, nil
	}
	support := planarSupport{normal: n, origin: a, local: local, pathS: S,
		heights: make([]*big.Rat, len(M.startPoints))}
	for i, v := range M.startPoints {
		if err := poll(); err != nil {
			return planarSupport{}, false, err
		}
		height := proofarith.DvDot(n, proofarith.DvSub(v, a))
		switch height.Sign() {
		case -1:
			return planarSupport{}, false, nil
		case 0:
			support.contact = append(support.contact, i)
		default:
			if band.Sign() > 0 && proofarith.DyCmp(proofarith.DyMul(height, height), limit) <= 0 {
				support.lifted = append(support.lifted, i)
			}
		}
		support.heights[i] = height.Rat()
	}
	if len(support.contact) == 0 && len(support.lifted) == 0 {
		return planarSupport{}, false, nil
	}
	if local {
		if _, ok := planarSupportFace(&support); !ok {
			return planarSupport{}, false, nil
		}
	}
	squared := proofarith.DvDot(n, n).Rat()
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finiteMeasurementValues(low, high) {
		return planarSupport{}, false, nil
	}
	support.nLow, support.nHigh = proofarith.FloatRat(low), proofarith.FloatRat(high)
	return support, true, nil
}

func vertexSpins(p *rotationalSweepPath, motion planarMotion) ([]*big.Rat, bool) {
	return planarsweep.VertexSpins(p.startPoints, motion)
}

func restedVertices(s *planarSupport, rest *big.Rat) map[int]struct{} {
	return planarsweep.RestedVertices(s.lifted, s.rates, s.nLow, rest)
}

func (s *planarSupport) curvature(t *big.Rat) ([]*big.Rat, bool) {
	return planarsweep.Curvature(s.motionM, s.motionS, s.spin, s.nHigh, t)
}

func (s *planarSupport) clearAt(t *big.Rat, k []*big.Rat, rest bool) bool {
	return planarsweep.ClearAt(s.heights, s.rates, s.contact, s.rested, t, k, rest)
}

// column is §10.6's column test through fraction f. The box spans every M
// vertex's ideal path over [0, f], less S's own translation over that span,
// so it holds M in S's start frame at every instant; S only translates on a
// face-local plane. clear is false when an S triangle in front of the plane
// meets the column; clearance is then the lateral clearance m(f), nil for a
// plane with nothing of S in front of it (m = +∞). The box grows with f, so
// the test and m(f) are monotone.
func (s *planarSupport) column(f *big.Rat, poll func() error) (*big.Rat, bool, error) {
	if !s.local {
		return nil, true, nil
	}
	spans := s.pathM.cornerSpan(new(big.Rat), f)
	var lo, hi [3]*big.Rat
	for axis := range 3 {
		lo[axis], hi[axis] = spans.Hull(axis)
		shift := new(big.Rat).Mul(s.pathS.path.delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(lo[axis], proofbound.RatMax(shift, new(big.Rat)))
		hi[axis] = new(big.Rat).Sub(hi[axis], proofbound.RatMin(shift, new(big.Rat)))
	}
	solid := planar.PlanarSolid{Verts: s.pathS.startPoints, Tris: s.pathS.solid.Tris}
	return planar.PlanarColumnClear(&solid, s.normal, s.origin, lo, hi, poll)
}

func (s *planarSupport) depthAt(t *big.Rat, k []*big.Rat, rate *big.Rat) *big.Rat {
	return planarsweep.DepthAt(s.heights, s.rates, s.contact, s.lifted, s.rested, t, k, rate)
}

// gridHorizon returns the largest fraction m/2^depth in (0, 1] at which holds
// is true, the grid being the sweep's own dyadic search grid capped at 52
// levels so every fraction is a float. holds must be monotone: true at a
// fraction implies true at every smaller one.
func (r *rotationalPairSweep) gridHorizon(holds func(f *big.Rat) (bool, error)) (*big.Rat, bool, error) {
	return sweepGridHorizon(r.resolution, r.a.path.duration, holds)
}

// sweepGridHorizon is gridHorizon for a sweep of the given time resolution
// and duration.
func sweepGridHorizon(resolution, duration *big.Rat,
	holds func(f *big.Rat) (bool, error)) (*big.Rat, bool, error) {
	ok, err := holds(big.NewRat(1, 1))
	if err != nil || ok {
		return big.NewRat(1, 1), ok, err
	}
	depth := uint(0)
	for depth < 52 && new(big.Rat).Mul(resolution, new(big.Rat).SetInt64(int64(1)<<depth)).Cmp(duration) < 0 {
		depth++
	}
	low, high := int64(0), int64(1)<<depth
	for high-low > 1 {
		mid := low + (high-low)/2
		holdsMid, err := holds(big.NewRat(mid, int64(1)<<depth))
		if err != nil {
			return nil, false, err
		}
		if holdsMid {
			low = mid
		} else {
			high = mid
		}
	}
	if low == 0 {
		return nil, false, nil
	}
	return big.NewRat(low, int64(1)<<depth), true, nil
}

// planarDepartureProof is §10.2's certificate: on (0, until] every M vertex
// height stays positive, so the pair is strictly separated there.
type planarDepartureProof struct {
	support   planarSupport
	curvature []*big.Rat // K_p for every M vertex at until
	until     *big.Rat   // fraction
}

// lowerGap is a lower bound, in millimetres, on the pair's separation at a
// fraction in (0, until]: the least vertex height bound divided by |n|, and
// on a face-local plane (§10.6) at most the lateral clearance m(f). Material
// of S behind the plane lies at least that height bound away, and material
// in front of it at least m(f). It returns nil when the column test fails at
// f, which no fraction of a proven departure does.
func (p *planarDepartureProof) lowerGap(f *big.Rat) *big.Rat {
	t := new(big.Rat).Mul(f, p.support.duration)
	var least *big.Rat
	for i, height := range p.support.heights {
		value := proofbound.RatAdd(height, proofbound.RatMul(p.support.rates[i], t))
		value.Sub(value, proofbound.RatMul(p.curvature[i], t, t))
		if least == nil || value.Cmp(least) < 0 {
			least = value
		}
	}
	if least.Sign() <= 0 {
		return least
	}
	least.Quo(least, p.support.nHigh)
	clearance, open, err := p.support.column(f, noSweepPoll)
	if err != nil || !open {
		return nil
	}
	if clearance != nil && clearance.Cmp(least) < 0 {
		return clearance
	}
	return least
}

// planarDepartureFraction proves §10.2's departure on the first support plane
// whose every contact rate is positive and returns the largest grid fraction
// it covers.
func (r *rotationalPairSweep) planarDepartureFraction(ctx context.Context) (*big.Rat, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	supports, err := r.planarSupports(budget.Step)
	if err != nil {
		return nil, false, err
	}
	for i := range supports {
		support := &supports[i]
		positive := true
		for _, index := range support.contact {
			if support.rates[index].Sign() <= 0 {
				positive = false
				break
			}
		}
		if !positive {
			continue
		}
		holds := func(f *big.Rat) (bool, error) {
			if err := budget.Step(); err != nil {
				return false, err
			}
			t := new(big.Rat).Mul(f, support.duration)
			k, ok := support.curvature(t)
			if !ok {
				return false, nil
			}
			for _, index := range support.contact {
				if new(big.Rat).Sub(support.rates[index], proofbound.RatMul(k[index], t)).Sign() <= 0 {
					return false, nil
				}
			}
			if !support.clearAt(t, k, false) {
				return false, nil
			}
			_, open, err := support.column(f, budget.Step)
			return open, err
		}
		until, ok, err := r.gridHorizon(holds)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		k, _ := support.curvature(new(big.Rat).Mul(until, support.duration))
		r.departure = &planarDepartureProof{support: *support, curvature: k, until: until}
		return until, true, nil
	}
	return nil, false, nil
}

// planarTrackProof is the private certificate of a planar persistent touch or
// band track. It owns copies of both prepared paths, so ManifoldAt and replay
// read no document state.
type planarTrackProof struct {
	paths     [2]rotationalSweepPath
	m, s      int
	contact   []int // M vertices in published order
	featureM  []ContactFeature
	featureS  ContactFeature
	normal    proofarith.DyV3 // S's exact outward normal
	origin    int             // an S vertex on the plane
	direction VecMeasurement  // the published A-to-B normal
	angle     units.Value
	heldDepth *big.Rat // millimetres: the held vertices' band, which replay checks
	depth     *big.Rat // millimetres, outward: heldDepth widened by 2δ (§10.4)
	depthUp   float64
	deltaM    *big.Rat // M's held displacement δ (§10.4); S's is zero
	band      *Measurement
	nHigh     *big.Rat
	support   planarSupport // the support plane the band was proved on
	rate      *big.Rat      // the largest |h'(0)| over the contact set
	widening  *big.Rat      // 2δ (§10.4)
}

// depthThrough is the band's published depth over [0, f]: the §10.3 bound
// (r·t + K(t)·t²)/|n|_lo at t = f·duration, widened by 2δ. K(t) bounds the
// height curvature over [0, t], so the bound holds at every earlier instant,
// and f lies inside the track, whose clearance and face containment hold
// through its end and so through f.
func (p *planarTrackProof) depthThrough(f *big.Rat) (*big.Rat, bool) {
	t := new(big.Rat).Mul(f, p.support.duration)
	k, ok := p.support.curvature(t)
	if !ok {
		return nil, false
	}
	depth := p.support.depthAt(t, k, p.rate)
	depth.Quo(depth, p.support.nLow)
	return depth.Add(depth, p.widening), true
}

// planarBand proves §10.3's band track on the first support plane that
// admits one. S must not rotate, so its face normal is constant, and every
// contact vertex's foot must stay inside S's face for the whole track. A
// track with zero depth that reaches the duration is an exact persistent
// touch.
//
// §10.4 runs the same proof over the held vertices of a positive-displacement
// body M and charges δ afterwards: the published depth is the held depth
// widened by 2δ, δ the summed displacement, and each foot's box grows by M's
// δ, since a true contact point lies within it of a held vertex. S must carry
// no displacement, because the track publishes S's held face normal as the
// true one.
func (r *rotationalPairSweep) planarBand(ctx context.Context) (*SweepContactTrack, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	supports, err := r.planarSupports(budget.Step)
	if err != nil {
		return nil, false, err
	}
	for i := range supports {
		support := &supports[i]
		if support.motionS.Rotating || support.pathS.path.drift != nil || support.pathS.delta.Sign() != 0 {
			continue
		}
		face, ok := planarSupportFace(support)
		if !ok {
			continue
		}
		rate := new(big.Rat)
		for _, index := range support.contact {
			if magnitude := new(big.Rat).Abs(support.rates[index]); magnitude.Cmp(rate) > 0 {
				rate = magnitude
			}
		}
		depthAt := func(t *big.Rat, k []*big.Rat) *big.Rat {
			depth := support.depthAt(t, k, rate)
			return depth.Quo(depth, support.nLow)
		}
		holds := func(f *big.Rat) (bool, error) {
			if err := budget.Step(); err != nil {
				return false, err
			}
			t := new(big.Rat).Mul(f, support.duration)
			k, ok := support.curvature(t)
			if !ok || !support.clearAt(t, k, true) {
				return false, nil
			}
			// §10.6: material of S in front of the plane meets no part of M,
			// so M ∩ S lies behind the plane and within the band's depth.
			if _, open, err := support.column(f, budget.Step); err != nil || !open {
				return false, err
			}
			return face.contains(support, f, new(big.Rat).Add(depthAt(t, k), support.pathM.delta.Rat()), budget.Step)
		}
		end, ok, err := r.gridHorizon(holds)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		t := new(big.Rat).Mul(end, support.duration)
		k, _ := support.curvature(t)
		track, ok, err := r.planarTrack(support, face, end, depthAt(t, k), rate)
		if err != nil {
			return nil, false, err
		}
		if ok {
			return track, true, nil
		}
	}
	return nil, false, nil
}

// planarTrack builds the public track of a proven band and checks that its
// manifold publishes at the start. depth is the held vertices' band; the
// published one adds 2δ.
func (r *rotationalPairSweep) planarTrack(support *planarSupport, face planarFace,
	end, heldDepth, rate *big.Rat) (*SweepContactTrack, bool, error) {
	solids := [2]*planar.PlanarSolid{r.a.solid, r.b.solid}
	features, err := newPlanarFeatureMap(r.a.body, r.b.body, solids[0], solids[1])
	if err != nil {
		return nil, false, err
	}
	if solids[support.s].Faces == nil || solids[support.m].Faces == nil {
		return nil, false, nil
	}
	featureS, ok := features.feature(support.s, planar.PatchFeature{Kind: planar.FeatureFacet,
		Faces: []int{face.ID}})
	if !ok {
		return nil, false, nil
	}
	type entry struct {
		index   int
		feature ContactFeature
		key     [2]int
	}
	// The contact set comes first and the lifted set after it (§10.5), each
	// in the body's feature order.
	var entries []entry
	for _, set := range [2][]int{support.contact, support.lifted} {
		from := len(entries)
		for _, index := range set {
			feature, ok := features.feature(support.m, planar.PatchFeature{Kind: planar.FeatureVertex,
				Faces: planar.VertexFaceIDs(solids[support.m], index)})
			if !ok {
				return nil, false, nil
			}
			entries = append(entries, entry{index: index, feature: feature, key: features.order(support.m, feature)})
		}
		part := entries[from:]
		sort.SliceStable(part, func(i, j int) bool { return compareKey(part[i].key, part[j].key) < 0 })
	}
	direction := support.normal
	if support.m == 0 {
		direction = proofarith.DyV3{proofarith.DyNeg(direction[0]), proofarith.DyNeg(direction[1]),
			proofarith.DyNeg(direction[2])}
	}
	normal, angle, ok := planarNormal(direction)
	if !ok || angle.Base() > r.req.NormalResolution.Base() {
		return nil, false, nil
	}
	widening := proofbound.RatMul(big.NewRat(2, 1), proofarith.DyAdd(r.a.delta, r.b.delta).Rat())
	depth := new(big.Rat).Add(heldDepth, widening)
	proof := &planarTrackProof{paths: [2]rotationalSweepPath{r.a, r.b}, m: support.m, s: support.s,
		featureS: featureS, normal: support.normal, origin: r.solidTriVertex(support),
		direction: normal, angle: angle, heldDepth: heldDepth, depth: depth, depthUp: proofbound.RatFloatUp(depth),
		deltaM: support.pathM.delta.Rat(), nHigh: support.nHigh, support: *support, rate: rate, widening: widening}
	if !finiteMeasurementValues(proof.depthUp) {
		return nil, false, nil
	}
	for _, e := range entries {
		proof.contact = append(proof.contact, e.index)
		proof.featureM = append(proof.featureM, e.feature)
	}
	zero := new(big.Rat)
	if depth.Sign() > 0 || end.Cmp(big.NewRat(1, 1)) < 0 {
		value := ratFloatNearest(depth)
		bound := proofarith.RationalFloatError(depth, value)
		proof.band = &Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessFromBound(bound)}
	}
	track := &SweepContactTrack{start: zero, end: new(big.Rat).Set(end), duration: r.a.path.duration,
		request: r.req.ContactRequest, normal: normal, planar: proof}
	// A track whose start manifold is refused (a point ball over
	// PointResolution) is not published; the refusal is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	track.features[support.m], track.features[support.s] = proof.featureM[0], featureS
	track.pointCount = len(proof.contact)
	return track, true, nil
}

// solidTriVertex names the S vertex the support plane was read through.
func (r *rotationalPairSweep) solidTriVertex(support *planarSupport) int {
	solid := r.a.solid
	if support.s == 1 {
		solid = r.b.solid
	}
	return solid.Tris[support.tri][0]
}

// planarFace carries the exact planar face selected by the sweep.
type planarFace struct{ planar.SupportFace }

func planarSupportFace(s *planarSupport) (planarFace, bool) {
	face, ok := planar.BuildSupportFace(s.pathS.solid, s.pathS.startPoints, s.normal, s.origin)
	return planarFace{face}, ok
}

// contains reports whether, through fraction f, the foot of every contact
// vertex on the moving plane stays inside the face. The foot lies within
// depth of the vertex, so the vertex's ideal path box over [0, f], less S's
// own translation and grown by depth on every axis, encloses it in S's start
// frame. That box, projected along the dropped axis, holds the projected foot,
// and the projection is a bijection on the plane: a projected box that meets
// no bounding edge and has a corner in a face triangle lies inside the face.
func (face *planarFace) contains(s *planarSupport, f, depth *big.Rat, poll func() error) (bool, error) {
	spans := s.pathM.cornerSpan(new(big.Rat), f)
	i, j := (face.Drop+1)%3, (face.Drop+2)%3
	for _, index := range slices.Concat(s.contact, s.lifted) {
		if err := poll(); err != nil {
			return false, err
		}
		var lo, hi [2]*big.Rat
		for slot, axis := range [2]int{i, j} {
			shift := new(big.Rat).Mul(s.pathS.path.delta[axis].Rat(), f)
			shiftLo, shiftHi := proofbound.RatMin(shift, new(big.Rat)), proofbound.RatMax(shift, new(big.Rat))
			span := spans.Span(index, axis)
			lo[slot] = proofbound.RatAdd(span.Lo, new(big.Rat).Neg(shiftHi), new(big.Rat).Neg(depth))
			hi[slot] = proofbound.RatAdd(span.Hi, new(big.Rat).Neg(shiftLo), depth)
		}
		if !face.HoldsBox(lo, hi) {
			return false, nil
		}
	}
	return true, nil
}

// planarManifoldAt publishes the track's manifold at an exact fraction. Each
// contact vertex is staged through the rounded pose, exactly as ContactPair
// stages it, and published with its foot on S's rounded plane. The rounded
// and ideal vertex differ by at most M's pose deviation, and the two feet by
// at most both deviations, because S only translates and the plane keeps its
// normal. A true contact point of a positive-displacement M lies within its
// δ of the held vertex, and its foot within δ of the held foot, so both balls
// also carry M's δ. The separation is the band's two-sided bound.
func (p *planarTrackProof) planarManifoldAt(f *big.Rat, req ContactRequest) (*ContactManifold, error) {
	verts, eta, ok := p.roundedVertices(f)
	if !ok {
		return nil, fmt.Errorf("%w: planar contact track pose has no finite bound", ErrUnsupported)
	}
	n := ratOfDyV3(p.normal)
	nn := ratDot3(n, n)
	q := ratOfDyV3(verts[p.s][p.origin])
	resolution, okResolution := exactBaseValue(req.PointResolution)
	if !okResolution {
		return nil, fmt.Errorf("%w: planar contact track resolution is invalid", ErrUnsupported)
	}
	separation := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(p.depthUp),
		Exactness: exactnessFromBound(p.depthUp)}
	points := make([]ContactPoint, 0, len(p.contact))
	for slot, index := range p.contact {
		vertex := ratOfDyV3(verts[p.m][index])
		height := new(big.Rat).Quo(ratDot3(n, ratSub3(vertex, q)), nn)
		var foot [3]*big.Rat
		for k := range 3 {
			foot[k] = new(big.Rat).Sub(vertex[k], new(big.Rat).Mul(height, n[k]))
		}
		onM, okM := planarTrackPoint([3]*big.Rat(vertex), new(big.Rat).Add(eta[p.m], p.deltaM), resolution)
		onS, okS := planarTrackPoint(foot, proofbound.RatAdd(eta[p.m], eta[p.s], p.deltaM), resolution)
		if !okM || !okS {
			return nil, fmt.Errorf("%w: planar contact track point exceeds point resolution", ErrUnsupported)
		}
		point := ContactPoint{Normal: p.direction, NormalAngle: p.angle, Separation: separation}
		if p.m == 0 {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = p.featureM[slot], p.featureS
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = p.featureS, p.featureM[slot]
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	return &ContactManifold{Points: points}, nil
}

// planarTrackPoint converts an exact point once and widens its ball by the
// pose deviation; the result must fit the resolution.
func planarTrackPoint(point [3]*big.Rat, deviation, resolution *big.Rat) (VecMeasurement, bool) {
	value, ok := orientedBoxPoint(point)
	if !ok {
		return VecMeasurement{}, false
	}
	bound := new(big.Rat).Add(proofarith.FloatRat(value.Bound.Base()), deviation)
	published := proofbound.RatFloatUp(bound)
	if !finiteMeasurementValues(published) || bound.Cmp(resolution) > 0 ||
		proofarith.FloatRat(published).Cmp(resolution) > 0 {
		return VecMeasurement{}, false
	}
	value.Bound, value.Exactness = units.Millimeters(published), exactnessFromBound(published)
	return value, true
}

// roundedVertices stages both snapshots through the float poses at f and
// returns each body's pose deviation. S must keep its start basis: the track
// admits only a translating S, whose rounded plane then keeps its normal.
func (p *planarTrackProof) roundedVertices(f *big.Rat) ([2][]proofarith.DyV3, [2]*big.Rat, bool) {
	var verts [2][]proofarith.DyV3
	var eta [2]*big.Rat
	for i := range p.paths {
		pose, err := p.paths[i].poseAt(f)
		if err != nil {
			return verts, eta, false
		}
		if i == p.s && pose.Basis() != p.paths[i].path.from.Basis() {
			return verts, eta, false
		}
		points, bound, ok, _ := p.paths[i].pointDeviation(pose, f, noSweepPoll)
		if !ok {
			return verts, eta, false
		}
		verts[i], eta[i] = points, proofarith.FloatRat(bound)
	}
	return verts, eta, true
}

// replayHeights is the band's replay check: every held M vertex of the
// rounded pose stays above −(held depth + both deviations) on S's rounded
// plane. It reads the held depth, not the published one: the held vertices
// are what it measures, and the true pair then lies within the published
// band widened by the deviations. It can only refuse; the producer's
// certificate covers the ideal path.
func (p *planarTrackProof) replayHeights(f *big.Rat) (*big.Rat, bool) {
	verts, eta, ok := p.roundedVertices(f)
	if !ok {
		return nil, false
	}
	deviation := new(big.Rat).Add(eta[0], eta[1])
	floor := new(big.Rat).Neg(proofbound.RatMul(new(big.Rat).Add(p.heldDepth, deviation), p.nHigh))
	q := verts[p.s][p.origin]
	for _, v := range verts[p.m] {
		if proofarith.DvDot(p.normal, proofarith.DvSub(v, q)).Rat().Cmp(floor) < 0 {
			return nil, false
		}
	}
	return deviation, true
}

func ratOfDyV3(v proofarith.DyV3) motionbound.RatVec {
	return motionbound.RatVec{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func ratAdd3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{new(big.Rat).Add(a[0], b[0]), new(big.Rat).Add(a[1], b[1]), new(big.Rat).Add(a[2], b[2])}
}

func ratSub3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func ratDot3(a, b motionbound.RatVec) *big.Rat {
	return proofbound.RatAdd(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1]), new(big.Rat).Mul(a[2], b[2]))
}

func ratCross3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

// ratSqrtUpRat is an exact upper bound on the square root of q.
func ratSqrtUpRat(q *big.Rat) (*big.Rat, bool) {
	up := proofbound.RatSqrtUp(q)
	if !finiteMeasurementValues(up) {
		return nil, false
	}
	return proofarith.FloatRat(up), true
}
