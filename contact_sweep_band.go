package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/pair"
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

// planarMotion is one body's exact first-order rigid motion in world
// coordinates. A path that does not rotate has a zero omega, and its center
// is its first start vertex, so every distance below stays defined.
type planarMotion struct {
	velocity ratVec   // mm/s
	omega    ratVec   // rad/s
	center   ratVec   // world pivot at the start
	rotating bool     // omega is nonzero
	omegaSq  *big.Rat // exact |ω|²
	omegaUp  *big.Rat // upper bound on |ω|
	rho      *big.Rat // upper bound on the largest start-vertex distance from center
}

func planarMotionOf(p *rotationalSweepPath) (planarMotion, bool) {
	if p.path.screw != nil || len(p.startPoints) == 0 {
		return planarMotion{}, false
	}
	m := planarMotion{omega: ratVec{new(big.Rat), new(big.Rat), new(big.Rat)},
		omegaSq: new(big.Rat), omegaUp: new(big.Rat)}
	if p.path.drift == nil {
		for k := range 3 {
			m.velocity[k] = new(big.Rat).Quo(p.path.delta[k].Rat(), p.path.duration)
		}
		m.center = ratOfDyV3(p.startPoints[0])
	} else {
		for k := range 3 {
			if p.velocity[k] == nil || p.frame.axis[k] == nil || p.frame.center[k] == nil {
				return planarMotion{}, false
			}
			m.velocity[k] = new(big.Rat).Set(p.velocity[k])
			m.omega[k] = new(big.Rat).Set(p.frame.axis[k])
			m.center[k] = new(big.Rat).Set(p.frame.center[k])
		}
		m.rotating = true
		m.omegaSq = ratDot3(m.omega, m.omega)
		up, ok := ratSqrtUpRat(m.omegaSq)
		if !ok {
			return planarMotion{}, false
		}
		m.omegaUp = up
	}
	rhoSq := new(big.Rat)
	for _, v := range p.startPoints {
		d := ratSub3(ratOfDyV3(v), m.center)
		if sq := ratDot3(d, d); sq.Cmp(rhoSq) > 0 {
			rhoSq = sq
		}
	}
	rho, ok := ratSqrtUpRat(rhoSq)
	if !ok {
		return planarMotion{}, false
	}
	m.rho = rho
	return m, true
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
		var tried []planarSupport
		for t, tri := range S.solid.Tris {
			if err := poll(); err != nil {
				return nil, err
			}
			a := S.startPoints[tri[0]]
			n := proofarith.DvCross(proofarith.DvSub(S.startPoints[tri[1]], a),
				proofarith.DvSub(S.startPoints[tri[2]], a))
			if proofarith.DvIsZero(n) || planarPlaneTried(tried, n, a) {
				continue
			}
			tried = append(tried, planarSupport{normal: n, origin: a})
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
			support.rates = make([]*big.Rat, len(M.startPoints))
			relative := ratSub3(support.motionM.velocity, support.motionS.velocity)
			normal := ratOfDyV3(n)
			for i, v := range M.startPoints {
				p := ratOfDyV3(v)
				rate := ratAdd3(relative, ratCross3(support.motionM.omega, ratSub3(p, support.motionM.center)))
				rate = ratSub3(rate, ratCross3(support.motionS.omega, ratSub3(p, support.motionS.center)))
				support.rates[i] = ratDot3(normal, rate)
			}
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
	low, high := ratSqrtDown(squared), ratSqrtUp(squared)
	if low <= 0 || !finiteMeasurementValues(low, high) {
		return planarSupport{}, false, nil
	}
	support.nLow, support.nHigh = proofarith.FloatRat(low), proofarith.FloatRat(high)
	return support, true, nil
}

// vertexSpins returns, for every start vertex p of a path, an upper bound on
// |ω|·|ω×(p − c)|, the magnitude of p's second derivative under the path's
// drift: p(u) = c + v·u + R(ωu)·(p − c), so p”(u) = R(ωu)·(ω×(ω×(p − c))),
// whose length |ω|·|ω×(p − c)| holds for every u (§10.8). Each factor is
// rounded up. A path that does not rotate has a zero spin at every vertex.
func vertexSpins(p *rotationalSweepPath, motion planarMotion) ([]*big.Rat, bool) {
	out := make([]*big.Rat, len(p.startPoints))
	for i, v := range p.startPoints {
		if !motion.rotating {
			out[i] = new(big.Rat)
			continue
		}
		arm := ratCross3(motion.omega, ratSub3(ratOfDyV3(v), motion.center))
		armUp, ok := ratSqrtUpRat(ratDot3(arm, arm))
		if !ok {
			return nil, false
		}
		out[i] = ratMul(motion.omegaUp, armUp)
	}
	return out, true
}

// restedVertices returns the support's lifted vertices that §10.8 rests under
// a rest speed: those whose exact start rate satisfies h'(0) >= −rest·|n|_lo,
// so they close on the plane no faster than rest, or rise. A zero rest speed
// rests none.
func restedVertices(s *planarSupport, rest *big.Rat) map[int]struct{} {
	if rest.Sign() <= 0 || len(s.lifted) == 0 {
		return nil
	}
	floor := new(big.Rat).Neg(ratMul(rest, s.nLow))
	out := make(map[int]struct{})
	for _, index := range s.lifted {
		if s.rates[index].Cmp(floor) >= 0 {
			out[index] = struct{}{}
		}
	}
	return out
}

// curvature returns K_p for every M vertex p, for the unnormalized heights
// over [0, t] seconds: half of |n| times the §10.2 bound on the second
// derivative of p's height,
//
//	|ω_M|·|ω_M×(p − c_M)| + |ω_S|²·ρ_S                     from p'' and q''
//	+ 2·|ω_S|·(|v_M − v_S| + |ω_M|·ρ_M + |ω_S|·ρ_S)       from 2·n'·(p' − q')
//	+ |ω_S|²·D,  D = ρ_M + ρ_S + |c_M − c_S| + |v_M − v_S|·t   from n''·(p − q)
//
// with q the foot of S's pivot on the plane: it moves rigidly with S, lies
// within ρ_S of the pivot (the plane holds an S vertex), and stays within D of
// every M vertex. The p” term is p's own (vertexSpins, §10.8), at most §10.2's
// |ω_M|²·ρ_M; the S terms stay global. The terms are nondecreasing in t, so
// K_p(t) covers [0, t].
func (s *planarSupport) curvature(t *big.Rat) ([]*big.Rat, bool) {
	m, o := s.motionM, s.motionS
	relative := ratSub3(m.velocity, o.velocity)
	speed, okSpeed := ratSqrtUpRat(ratDot3(relative, relative))
	offset := ratSub3(m.center, o.center)
	distance, okDistance := ratSqrtUpRat(ratDot3(offset, offset))
	if !okSpeed || !okDistance {
		return nil, false
	}
	shared := ratMul(o.omegaSq, o.rho)
	lever := ratAdd(speed, ratMul(m.omegaUp, m.rho), ratMul(o.omegaUp, o.rho))
	shared.Add(shared, ratMul(big.NewRat(2, 1), o.omegaUp, lever))
	reach := ratAdd(m.rho, o.rho, distance, ratMul(speed, t))
	shared.Add(shared, ratMul(o.omegaSq, reach))
	half := ratMul(s.nHigh, big.NewRat(1, 2))
	out := make([]*big.Rat, len(s.spin))
	for i, spin := range s.spin {
		out[i] = ratMul(ratAdd(shared, spin), half)
	}
	return out, true
}

// clearAt reports whether every vertex outside the contact set, the lifted
// set's included (§10.5), keeps a positive height through [0, t]:
// h(0) + h'(0)·u − K_p·u² is concave and positive at u = 0, so its value at t
// decides the whole span. A lifted vertex that would reach the plane inside
// the slice therefore ends a band track before it does. Under rest a rested
// vertex (§10.8) is skipped: a band track holds it on both sides of the plane
// (depthAt). A departure passes rest false, so every vertex outside its
// contact set stays positive, rested or not: it claims strict separation.
func (s *planarSupport) clearAt(t *big.Rat, k []*big.Rat, rest bool) bool {
	contact := 0
	for i, height := range s.heights {
		if contact < len(s.contact) && s.contact[contact] == i {
			contact++
			continue
		}
		if _, rested := s.rested[i]; rest && rested {
			continue
		}
		value := ratAdd(height, ratMul(s.rates[i], t))
		value.Sub(value, ratMul(k[i], t, t))
		if value.Sign() <= 0 {
			return false
		}
	}
	return true
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
		lo[axis], hi[axis] = spans[0][axis].lo, spans[0][axis].hi
		for _, span := range spans[1:] {
			lo[axis], hi[axis] = ratMin(lo[axis], span[axis].lo), ratMax(hi[axis], span[axis].hi)
		}
		shift := new(big.Rat).Mul(s.pathS.path.delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(lo[axis], ratMax(shift, new(big.Rat)))
		hi[axis] = new(big.Rat).Sub(hi[axis], ratMin(shift, new(big.Rat)))
	}
	solid := pair.PlanarSolid{Verts: s.pathS.startPoints, Tris: s.pathS.solid.Tris}
	return pair.PlanarColumnClear(&solid, s.normal, s.origin, lo, hi, poll)
}

// depthAt is the band's unnormalized depth at elapsed time t under the
// per-vertex curvature k, read over [0, t], the largest of three bounds:
// r·t + K_p·t² over the contact set, r the largest contact rate magnitude;
// h0 + |h'(0)|·t + K_p·t² over the rested set (§10.8); and
// h0 + max(0, h'(0))·t + K_p·t² over the rest of the lifted set (§10.5). A
// contact vertex's height lies in [−(r·t + K_p·t²), r·t + K_p·t²] and a rested
// vertex's in [−(h0 + |h'(0)|·t + K_p·t²), h0 + |h'(0)|·t + K_p·t²], both by
// Taylor's theorem; any other lifted vertex, which clearAt keeps above the
// plane, lies in (0, h0 + max(0, h'(0))·t + K_p·t²].
func (s *planarSupport) depthAt(t *big.Rat, k []*big.Rat, rate *big.Rat) *big.Rat {
	depth := new(big.Rat)
	for _, index := range s.contact {
		contact := ratAdd(ratMul(rate, t), ratMul(k[index], t, t))
		if contact.Cmp(depth) > 0 {
			depth = contact
		}
	}
	for _, index := range s.lifted {
		lifted := ratAdd(s.heights[index], ratMul(k[index], t, t))
		_, rested := s.rested[index]
		if rested || s.rates[index].Sign() > 0 {
			lifted.Add(lifted, ratMul(new(big.Rat).Abs(s.rates[index]), t))
		}
		if lifted.Cmp(depth) > 0 {
			depth = lifted
		}
	}
	return depth
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
		value := ratAdd(height, ratMul(p.support.rates[i], t))
		value.Sub(value, ratMul(p.curvature[i], t, t))
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
	budget := newWorkBudget(ctx)
	supports, err := r.planarSupports(budget.step)
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
			if err := budget.step(); err != nil {
				return false, err
			}
			t := new(big.Rat).Mul(f, support.duration)
			k, ok := support.curvature(t)
			if !ok {
				return false, nil
			}
			for _, index := range support.contact {
				if new(big.Rat).Sub(support.rates[index], ratMul(k[index], t)).Sign() <= 0 {
					return false, nil
				}
			}
			if !support.clearAt(t, k, false) {
				return false, nil
			}
			_, open, err := support.column(f, budget.step)
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
	budget := newWorkBudget(ctx)
	supports, err := r.planarSupports(budget.step)
	if err != nil {
		return nil, false, err
	}
	for i := range supports {
		support := &supports[i]
		if support.motionS.rotating || support.pathS.path.drift != nil || support.pathS.delta.Sign() != 0 {
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
			if err := budget.step(); err != nil {
				return false, err
			}
			t := new(big.Rat).Mul(f, support.duration)
			k, ok := support.curvature(t)
			if !ok || !support.clearAt(t, k, true) {
				return false, nil
			}
			// §10.6: material of S in front of the plane meets no part of M,
			// so M ∩ S lies behind the plane and within the band's depth.
			if _, open, err := support.column(f, budget.step); err != nil || !open {
				return false, err
			}
			return face.contains(support, f, new(big.Rat).Add(depthAt(t, k), support.pathM.delta.Rat()), budget.step)
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
	solids := [2]*pair.PlanarSolid{r.a.solid, r.b.solid}
	features, err := newPlanarFeatureMap(r.a.body, r.b.body, solids[0], solids[1])
	if err != nil {
		return nil, false, err
	}
	if solids[support.s].Faces == nil || solids[support.m].Faces == nil {
		return nil, false, nil
	}
	featureS, ok := features.feature(support.s, pair.PatchFeature{Kind: pair.FeatureFacet,
		Faces: []int{face.id}})
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
			feature, ok := features.feature(support.m, pair.PatchFeature{Kind: pair.FeatureVertex,
				Faces: vertexFaceIDs(solids[support.m], index)})
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
	widening := ratMul(big.NewRat(2, 1), proofarith.DyAdd(r.a.delta, r.b.delta).Rat())
	depth := new(big.Rat).Add(heldDepth, widening)
	proof := &planarTrackProof{paths: [2]rotationalSweepPath{r.a, r.b}, m: support.m, s: support.s,
		featureS: featureS, normal: support.normal, origin: r.solidTriVertex(support),
		direction: normal, angle: angle, heldDepth: heldDepth, depth: depth, depthUp: ratFloatUp(depth),
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

// vertexFaceIDs lists the distinct face ids of the triangles holding vertex v.
func vertexFaceIDs(solid *pair.PlanarSolid, v int) []int {
	var ids []int
	for t, tri := range solid.Tris {
		if tri[0] != v && tri[1] != v && tri[2] != v {
			continue
		}
		if id := solid.Faces[t]; !slices.Contains(ids, id) {
			ids = append(ids, id)
		}
	}
	sort.Ints(ids)
	return ids
}

// planarFace is the part of S on its support plane: the S triangles lying in
// it, projected to the plane's frame by dropping the axis of n's largest
// component, with the edges that bound their union.
type planarFace struct {
	id    int
	drop  int
	tris  [][3][2]*big.Rat
	edges [][2][2]*big.Rat
}

// planarSupportFace collects S's triangles on the support plane. They must
// all belong to one face of S, which the manifold names.
func planarSupportFace(s *planarSupport) (planarFace, bool) {
	solid := s.pathS.solid
	if solid.Faces == nil {
		return planarFace{}, false
	}
	drop := 0
	for k := 1; k < 3; k++ {
		if proofarith.DyCmp(proofarith.DyAbs(s.normal[k]), proofarith.DyAbs(s.normal[drop])) > 0 {
			drop = k
		}
	}
	project := func(v proofarith.DyV3) [2]*big.Rat {
		i, j := (drop+1)%3, (drop+2)%3
		return [2]*big.Rat{v[i].Rat(), v[j].Rat()}
	}
	face := planarFace{id: -1, drop: drop}
	directed := make(map[[2]int]struct{})
	var held [][3]int
	for t, tri := range solid.Tris {
		onPlane := true
		for _, v := range tri {
			if proofarith.DvDot(s.normal, proofarith.DvSub(s.pathS.startPoints[v], s.origin)).Sign() != 0 {
				onPlane = false
				break
			}
		}
		if !onPlane {
			continue
		}
		a := s.pathS.startPoints[tri[0]]
		n := proofarith.DvCross(proofarith.DvSub(s.pathS.startPoints[tri[1]], a),
			proofarith.DvSub(s.pathS.startPoints[tri[2]], a))
		if proofarith.DvDot(n, s.normal).Sign() <= 0 {
			return planarFace{}, false
		}
		if face.id >= 0 && solid.Faces[t] != face.id {
			return planarFace{}, false
		}
		face.id = solid.Faces[t]
		held = append(held, tri)
		face.tris = append(face.tris, [3][2]*big.Rat{project(s.pathS.startPoints[tri[0]]),
			project(s.pathS.startPoints[tri[1]]), project(s.pathS.startPoints[tri[2]])})
		for i := range 3 {
			directed[[2]int{tri[i], tri[(i+1)%3]}] = struct{}{}
		}
	}
	if face.id < 0 {
		return planarFace{}, false
	}
	for _, tri := range held {
		for i := range 3 {
			from, to := tri[i], tri[(i+1)%3]
			if _, shared := directed[[2]int{to, from}]; shared {
				continue
			}
			face.edges = append(face.edges, [2][2]*big.Rat{project(s.pathS.startPoints[from]),
				project(s.pathS.startPoints[to])})
		}
	}
	return face, true
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
	i, j := (face.drop+1)%3, (face.drop+2)%3
	for _, index := range slices.Concat(s.contact, s.lifted) {
		if err := poll(); err != nil {
			return false, err
		}
		var lo, hi [2]*big.Rat
		for slot, axis := range [2]int{i, j} {
			shift := new(big.Rat).Mul(s.pathS.path.delta[axis].Rat(), f)
			shiftLo, shiftHi := ratMin(shift, new(big.Rat)), ratMax(shift, new(big.Rat))
			lo[slot] = ratAdd(spans[index][axis].lo, new(big.Rat).Neg(shiftHi), new(big.Rat).Neg(depth))
			hi[slot] = ratAdd(spans[index][axis].hi, new(big.Rat).Neg(shiftLo), depth)
		}
		inside := false
		for _, tri := range face.tris {
			if planarPointInTriangle(lo, tri) {
				inside = true
				break
			}
		}
		if !inside {
			return false, nil
		}
		for _, edge := range face.edges {
			if planarSegmentMeetsBox(edge[0], edge[1], lo, hi) {
				return false, nil
			}
		}
	}
	return true, nil
}

func planarOrient(a, b, c [2]*big.Rat) int {
	left := new(big.Rat).Mul(new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(c[1], a[1]))
	right := new(big.Rat).Mul(new(big.Rat).Sub(b[1], a[1]), new(big.Rat).Sub(c[0], a[0]))
	return left.Cmp(right)
}

// planarPointInTriangle tests the closed triangle in either winding.
func planarPointInTriangle(p [2]*big.Rat, tri [3][2]*big.Rat) bool {
	sign := planarOrient(tri[0], tri[1], tri[2])
	if sign == 0 {
		return false
	}
	for k := range 3 {
		if planarOrient(tri[k], tri[(k+1)%3], p)*sign < 0 {
			return false
		}
	}
	return true
}

// planarSegmentMeetsBox is the separating-axis test of a segment and a closed
// axis-aligned box: the two box axes and the segment's normal.
func planarSegmentMeetsBox(a, b, lo, hi [2]*big.Rat) bool {
	for axis := range 2 {
		if ratMax(a[axis], b[axis]).Cmp(lo[axis]) < 0 || ratMin(a[axis], b[axis]).Cmp(hi[axis]) > 0 {
			return false
		}
	}
	positive, negative := false, false
	for _, corner := range [4][2]*big.Rat{{lo[0], lo[1]}, {hi[0], lo[1]}, {lo[0], hi[1]}, {hi[0], hi[1]}} {
		switch planarOrient(a, b, corner) {
		case 1:
			positive = true
		case -1:
			negative = true
		default:
			return true
		}
	}
	return positive && negative
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
		onS, okS := planarTrackPoint(foot, ratAdd(eta[p.m], eta[p.s], p.deltaM), resolution)
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
	published := ratFloatUp(bound)
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
	floor := new(big.Rat).Neg(ratMul(new(big.Rat).Add(p.heldDepth, deviation), p.nHigh))
	q := verts[p.s][p.origin]
	for _, v := range verts[p.m] {
		if proofarith.DvDot(p.normal, proofarith.DvSub(v, q)).Rat().Cmp(floor) < 0 {
			return nil, false
		}
	}
	return deviation, true
}

func ratOfDyV3(v proofarith.DyV3) ratVec {
	return ratVec{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func ratAdd3(a, b ratVec) ratVec {
	return ratVec{new(big.Rat).Add(a[0], b[0]), new(big.Rat).Add(a[1], b[1]), new(big.Rat).Add(a[2], b[2])}
}

func ratSub3(a, b ratVec) ratVec {
	return ratVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func ratDot3(a, b ratVec) *big.Rat {
	return ratAdd(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1]), new(big.Rat).Mul(a[2], b[2]))
}

func ratCross3(a, b ratVec) ratVec {
	return ratVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

// ratSqrtUpRat is an exact upper bound on the square root of q.
func ratSqrtUpRat(q *big.Rat) (*big.Rat, bool) {
	up := ratSqrtUp(q)
	if !finiteMeasurementValues(up) {
		return nil, false
	}
	return proofarith.FloatRat(up), true
}
