package decad

import (
	"context"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/sweeppath"

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
	return planarsweep.MotionOf(planarSupportPathOf(p).Motion)
}

func planarSupportPathOf(p *rotationalSweepPath) planarsweep.SupportPath {
	return planarsweep.SupportPath{
		Points: p.startPoints, Solid: p.solid,
		MovingOwner: p.path.Drift != nil || p.path.Screw != nil,
		Motion: planarsweep.MotionInput{
			Points: p.startPoints, Delta: p.path.Delta, Duration: p.path.Duration,
			Velocity: p.velocity, Frame: p.frame,
			Drift: p.path.Drift != nil, Screw: p.path.Screw != nil,
		},
	}
}

// planarSupport adds the root sweep paths to an exact internal support plane.
type planarSupport struct {
	planarsweep.SupportCandidate
	pathS, pathM *rotationalSweepPath
}

// planarSupports lists every support plane of the initial touch, in a fixed
// order: S is the sweep's B body first, then its A body, and planes follow
// S's triangle order, each plane once.
func (r *rotationalPairSweep) planarSupports(poll func() error) ([]planarSupport, error) {
	paths := [2]*rotationalSweepPath{&r.a, &r.b}
	rest := new(big.Rat)
	if r.req.RestSpeed != (units.Value{}) {
		speed, ok := sweeppath.ExactBaseValue(r.req.RestSpeed)
		if !ok {
			return nil, nil
		}
		rest = speed
	}
	inputs := [2]planarsweep.SupportPath{planarSupportPathOf(paths[0]), planarSupportPathOf(paths[1])}
	candidates, err := planarsweep.FindSupports(inputs, supportBandOf(r.req.ContactRequest), rest, poll)
	if err != nil {
		return nil, err
	}
	var out []planarSupport
	for _, c := range candidates {
		out = append(out, planarSupport{
			SupportCandidate: c,
			pathM:            paths[c.Guest], pathS: paths[c.Owner],
		})
	}
	return out, nil
}

// column is §10.6's column test through fraction f. The box spans every M
// vertex's ideal path over [0, f], less S's own translation over that span,
// so it holds M in S's start frame at every instant; S only translates on a
// face-local plane. clear is false when an S triangle in front of the plane
// meets the column; clearance is then the lateral clearance m(f), nil for a
// plane with nothing of S in front of it (m = +∞). The box grows with f, so
// the test and m(f) are monotone.
func (s *planarSupport) column(f *big.Rat, poll func() error) (*big.Rat, bool, error) {
	if !s.Local {
		return nil, true, nil
	}
	spans := s.pathM.cornerSpan(new(big.Rat), f)
	var lo, hi [3]*big.Rat
	for axis := range 3 {
		lo[axis], hi[axis] = spans.Hull(axis)
		shift := new(big.Rat).Mul(s.pathS.path.Delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(lo[axis], proofbound.RatMax(shift, new(big.Rat)))
		hi[axis] = new(big.Rat).Sub(hi[axis], proofbound.RatMin(shift, new(big.Rat)))
	}
	solid := planar.PlanarSolid{Verts: s.pathS.startPoints, Tris: s.pathS.solid.Tris}
	return planar.PlanarColumnClear(&solid, s.Normal, s.Origin, lo, hi, poll)
}

// gridHorizon returns the largest fraction m/2^depth in (0, 1] at which holds
// is true, the grid being the sweep's own dyadic search grid capped at 52
// levels so every fraction is a float. holds must be monotone: true at a
// fraction implies true at every smaller one.
func (r *rotationalPairSweep) gridHorizon(holds func(f *big.Rat) (bool, error)) (*big.Rat, bool, error) {
	return planarsweep.GridHorizon(r.resolution, r.a.path.Duration, holds)
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
	t := new(big.Rat).Mul(f, p.support.Duration)
	least := planarsweep.DepartureHeight(p.support.Heights, p.support.Rates, p.curvature, t, p.support.NormalHigh)
	if least.Sign() <= 0 {
		return least
	}
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
		for _, index := range support.Contact {
			if support.Rates[index].Sign() <= 0 {
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
			t := new(big.Rat).Mul(f, support.Duration)
			k, ok := support.Curvature(t)
			if !ok {
				return false, nil
			}
			for _, index := range support.Contact {
				if new(big.Rat).Sub(support.Rates[index], proofbound.RatMul(k[index], t)).Sign() <= 0 {
					return false, nil
				}
			}
			if !support.ClearAt(t, k, false) {
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
		k, _ := support.Curvature(new(big.Rat).Mul(until, support.Duration))
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
	t := new(big.Rat).Mul(f, p.support.Duration)
	k, ok := p.support.Curvature(t)
	if !ok {
		return nil, false
	}
	depth := p.support.DepthAt(t, k, p.rate)
	depth.Quo(depth, p.support.NormalLow)
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
		if support.OwnerMotion.Rotating || support.pathS.path.Drift != nil || support.pathS.delta.Sign() != 0 {
			continue
		}
		face, ok := planarSupportFace(support)
		if !ok {
			continue
		}
		rate := new(big.Rat)
		for _, index := range support.Contact {
			if magnitude := new(big.Rat).Abs(support.Rates[index]); magnitude.Cmp(rate) > 0 {
				rate = magnitude
			}
		}
		depthAt := func(t *big.Rat, k []*big.Rat) *big.Rat {
			depth := support.DepthAt(t, k, rate)
			return depth.Quo(depth, support.NormalLow)
		}
		holds := func(f *big.Rat) (bool, error) {
			if err := budget.Step(); err != nil {
				return false, err
			}
			t := new(big.Rat).Mul(f, support.Duration)
			k, ok := support.Curvature(t)
			if !ok || !support.ClearAt(t, k, true) {
				return false, nil
			}
			// §10.6: material of S in front of the plane meets no part of M,
			// so M ∩ S lies behind the plane and within the band's depth.
			if _, open, err := support.column(f, budget.Step); err != nil || !open {
				return false, err
			}
			spans := support.pathM.cornerSpan(new(big.Rat), f)
			depth := new(big.Rat).Add(depthAt(t, k), support.pathM.delta.Rat())
			return planarsweep.FaceContains(&face.SupportFace, spans, support.pathS.path.Delta,
				support.Contact, support.Lifted, f, depth, budget.Step)
		}
		end, ok, err := r.gridHorizon(holds)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		t := new(big.Rat).Mul(end, support.Duration)
		k, _ := support.Curvature(t)
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
	if solids[support.Owner].Faces == nil || solids[support.Guest].Faces == nil {
		return nil, false, nil
	}
	featureS, ok := features.feature(support.Owner, planar.PatchFeature{Kind: planar.FeatureFacet,
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
	for _, set := range [2][]int{support.Contact, support.Lifted} {
		from := len(entries)
		for _, index := range set {
			feature, ok := features.feature(support.Guest, planar.PatchFeature{Kind: planar.FeatureVertex,
				Faces: planar.VertexFaceIDs(solids[support.Guest], index)})
			if !ok {
				return nil, false, nil
			}
			entries = append(entries, entry{index: index, feature: feature, key: features.order(support.Guest, feature)})
		}
		part := entries[from:]
		sort.SliceStable(part, func(i, j int) bool { return compareKey(part[i].key, part[j].key) < 0 })
	}
	direction := support.Normal
	if support.Guest == 0 {
		direction = proofarith.DyV3{proofarith.DyNeg(direction[0]), proofarith.DyNeg(direction[1]),
			proofarith.DyNeg(direction[2])}
	}
	normal, angle, ok := planarNormal(direction)
	if !ok || angle.Base() > r.req.NormalResolution.Base() {
		return nil, false, nil
	}
	widening := proofbound.RatMul(big.NewRat(2, 1), proofarith.DyAdd(r.a.delta, r.b.delta).Rat())
	depth := new(big.Rat).Add(heldDepth, widening)
	proof := &planarTrackProof{paths: [2]rotationalSweepPath{r.a, r.b}, m: support.Guest, s: support.Owner,
		featureS: featureS, normal: support.Normal, origin: r.solidTriVertex(support),
		direction: normal, angle: angle, heldDepth: heldDepth, depth: depth, depthUp: proofbound.RatFloatUp(depth),
		deltaM: support.pathM.delta.Rat(), nHigh: support.NormalHigh, support: *support, rate: rate, widening: widening}
	if !finiteMeasurementValues(proof.depthUp) {
		return nil, false, nil
	}
	for _, e := range entries {
		proof.contact = append(proof.contact, e.index)
		proof.featureM = append(proof.featureM, e.feature)
	}
	zero := new(big.Rat)
	if depth.Sign() > 0 || end.Cmp(big.NewRat(1, 1)) < 0 {
		value := sweeppath.RatFloatNearest(depth)
		bound := proofarith.RationalFloatError(depth, value)
		proof.band = &Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessFromBound(bound)}
	}
	track := &SweepContactTrack{start: zero, end: new(big.Rat).Set(end), duration: r.a.path.Duration,
		request: r.req.ContactRequest, normal: normal, planar: proof}
	// A track whose start manifold is refused (a point ball over
	// PointResolution) is not published; the refusal is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	track.features[support.Guest], track.features[support.Owner] = proof.featureM[0], featureS
	track.pointCount = len(proof.contact)
	return track, true, nil
}

// solidTriVertex names the S vertex the support plane was read through.
func (r *rotationalPairSweep) solidTriVertex(support *planarSupport) int {
	solid := r.a.solid
	if support.Owner == 1 {
		solid = r.b.solid
	}
	return solid.Tris[support.Triangle][0]
}

// planarFace carries the exact planar face selected by the sweep.
type planarFace struct{ planar.SupportFace }

func planarSupportFace(s *planarSupport) (planarFace, bool) {
	face, ok := planar.BuildSupportFace(s.pathS.solid, s.pathS.startPoints, s.Normal, s.Origin)
	return planarFace{face}, ok
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
	resolution, okResolution := sweeppath.ExactBaseValue(req.PointResolution)
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
		pose, err := p.paths[i].path.RoundedPoseAt(f)
		if err != nil {
			return verts, eta, false
		}
		if i == p.s && pose.Basis() != p.paths[i].path.From.Basis() {
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
	if !planarsweep.ReplayHeights(verts[p.m], verts[p.s][p.origin], p.normal,
		p.heldDepth, deviation, p.nHigh) {
		return nil, false
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
