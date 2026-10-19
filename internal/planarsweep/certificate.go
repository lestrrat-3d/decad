package planarsweep

import (
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/units"
)

// This file holds the certificates that continue the general rotating sweep
// from an initial touch (docs/multibody-dynamics-design.md §10.2 and §10.3).
// Both proofs read one SUPPORT PLANE: a face plane of one body S that every
// vertex of the other body M lies on or in front of, touching it at the
// contact set. When every vertex of S lies on or behind it, S lies in its
// vertices' hull and moves rigidly with its plane, and M lies in its
// vertices' hull, so the lowest M vertex height above the moving plane
// bounds the pair's separation below. A FACE-LOCAL plane (§10.6), whose
// owner has material in front of it, the floor of a tray, adds the column
// test: every S triangle with a vertex in front of the plane projects
// strictly apart from M's ideal path box, so that material lies at least the
// lateral clearance away from M, and the vertex heights bound the distance
// to the rest of S.
//
// Each M vertex height h(u) has an exact start value and start rate, and a
// second derivative bounded by the §10.2 curvature with the vertex's own
// p'' term (§10.8); Taylor's theorem then bounds h(u) between
// h(0) + h'(0)·u ∓ K_p·u² with K_p half that bound. A departure needs every
// contact rate positive and every other vertex positive; a band track takes
// any contact rate, holds a lifted vertex that closes no faster than the
// request's RestSpeed on both sides of the plane (§10.8), and publishes the
// depth those bounds allow.

// Support is one support plane of an initial touch with the prepared paths
// of its owner S and its guest M.
type Support struct {
	SupportCandidate
	PathS, PathM *BodyPath
}

// FindPathSupports lists every support plane of the initial touch, in a
// fixed order: S is the sweep's second body first, then its first, and
// planes follow S's triangle order, each plane once. paths are in sweep
// order, and each Support points at its two entries.
func FindPathSupports(paths [2]*BodyPath, band proofarith.Dyadic, rest *big.Rat,
	poll func() error) ([]Support, error) {
	inputs := [2]SupportPath{paths[0].SupportPath(), paths[1].SupportPath()}
	candidates, err := FindSupports(inputs, band, rest, poll)
	if err != nil {
		return nil, err
	}
	var out []Support
	for _, c := range candidates {
		out = append(out, Support{SupportCandidate: c, PathM: paths[c.Guest], PathS: paths[c.Owner]})
	}
	return out, nil
}

// Column is §10.6's column test through fraction f. The box spans every M
// vertex's ideal path over [0, f], less S's own translation over that span,
// so it holds M in S's start frame at every instant; S only translates on a
// face-local plane. clear is false when an S triangle in front of the plane
// meets the column; clearance is then the lateral clearance m(f), nil for a
// plane with nothing of S in front of it (m = +∞). The box grows with f, so
// the test and m(f) are monotone.
func (s *Support) Column(f *big.Rat, poll func() error) (*big.Rat, bool, error) {
	if !s.Local {
		return nil, true, nil
	}
	spans := s.PathM.CornerSpan(new(big.Rat), f)
	var lo, hi [3]*big.Rat
	for axis := range 3 {
		lo[axis], hi[axis] = spans.Hull(axis)
		shift := new(big.Rat).Mul(s.PathS.Path.Delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(lo[axis], proofbound.RatMax(shift, new(big.Rat)))
		hi[axis] = new(big.Rat).Sub(hi[axis], proofbound.RatMin(shift, new(big.Rat)))
	}
	solid := planar.PlanarSolid{Verts: s.PathS.StartPoints, Tris: s.PathS.Solid.Tris}
	return planar.PlanarColumnClear(&solid, s.Normal, s.Origin, lo, hi, poll)
}

// DepartureProof is §10.2's certificate: on (0, Until] every M vertex height
// stays positive, so the pair is strictly separated there.
type DepartureProof struct {
	support   Support
	curvature []*big.Rat // K_p for every M vertex at Until
	Until     *big.Rat   // fraction
}

// ProveDeparture proves §10.2's departure on the first support plane, in
// order, whose contact rates are all positive and whose departure covers a
// grid fraction, and returns the proof of the largest grid fraction it
// covers. ok is false when no plane departs.
func ProveDeparture(supports []Support, resolution, duration *big.Rat,
	poll func() error) (*DepartureProof, bool, error) {
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
			if err := poll(); err != nil {
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
			_, open, err := support.Column(f, poll)
			return open, err
		}
		until, ok, err := GridHorizon(resolution, duration, holds)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		k, _ := support.Curvature(new(big.Rat).Mul(until, support.Duration))
		return &DepartureProof{support: *support, curvature: k, Until: until}, true, nil
	}
	return nil, false, nil
}

// LowerGap is a lower bound, in millimetres, on the pair's separation at a
// fraction in (0, Until]: the least vertex height bound divided by |n|, and
// on a face-local plane (§10.6) at most the lateral clearance m(f). Material
// of S behind the plane lies at least that height bound away, and material
// in front of it at least m(f). It returns nil when the column test fails at
// f, which no fraction of a proven departure does.
func (p *DepartureProof) LowerGap(f *big.Rat) *big.Rat {
	t := new(big.Rat).Mul(f, p.support.Duration)
	least := departureHeight(p.support.Heights, p.support.Rates, p.curvature, t, p.support.NormalHigh)
	if least.Sign() <= 0 {
		return least
	}
	clearance, open, err := p.support.Column(f, NoPoll)
	if err != nil || !open {
		return nil
	}
	if clearance != nil && clearance.Cmp(least) < 0 {
		return clearance
	}
	return least
}

// Replay is the replay certificate of a planar sweep. It never reruns the
// pair relation: a rounded pose is accepted when its vertex deviation from
// the ideal path stays below a lower bound on the ideal separation that the
// sweep already proved, so each replay costs one pass over both vertex sets.
type Replay struct {
	Departure *DepartureProof // (0, Until], when the sweep departed
	Travel    *big.Rat        // both bodies' travel bound per unit fraction
	spans     []ClearSpan     // the certified clear intervals, in order
}

// ClearSpan is one interval the search certified clear: by its two ends'
// lower gaps and the §4.3 travel bound, or by separated vertex hulls.
type ClearSpan struct {
	From, To    *big.Rat
	Left, Right *big.Rat // lower gaps at the two ends, nil when unproved
	Axis        *big.Rat // hull gap over the span, nil when the hulls meet
}

// AddSpan records span after the spans already recorded. It reports false,
// recording nothing, when the span has no hull gap and its two end gaps do
// not exceed the travel over it.
func (p *Replay) AddSpan(span ClearSpan) bool {
	travel := new(big.Rat).Mul(new(big.Rat).Sub(span.To, span.From), p.Travel)
	if span.Axis == nil && (span.Left == nil || span.Right == nil ||
		new(big.Rat).Add(span.Left, span.Right).Cmp(travel) <= 0) {
		return false
	}
	p.spans = append(p.spans, span)
	return true
}

// LowerGap bounds the ideal separation at f from below, or returns nil when
// no recorded certificate covers f.
func (p *Replay) LowerGap(f *big.Rat) *big.Rat {
	if p.Departure != nil && f.Sign() > 0 && f.Cmp(p.Departure.Until) <= 0 {
		return p.Departure.LowerGap(f)
	}
	for _, span := range p.spans {
		if f.Cmp(span.From) < 0 || f.Cmp(span.To) > 0 {
			continue
		}
		var best *big.Rat
		candidates := []*big.Rat{span.Axis}
		if span.Left != nil {
			candidates = append(candidates, new(big.Rat).Sub(span.Left,
				new(big.Rat).Mul(new(big.Rat).Sub(f, span.From), p.Travel)))
		}
		if span.Right != nil {
			candidates = append(candidates, new(big.Rat).Sub(span.Right,
				new(big.Rat).Mul(new(big.Rat).Sub(span.To, f), p.Travel)))
		}
		for _, candidate := range candidates {
			if candidate != nil && (best == nil || candidate.Cmp(best) > 0) {
				best = candidate
			}
		}
		return best
	}
	return nil
}

// departureHeight bounds the distance from the guest's vertices to the
// owner's support plane at elapsed time t. A nonpositive height is returned
// before division, matching the departure certificate's refusal path.
func departureHeight(heights, rates, curvature []*big.Rat, t, nHigh *big.Rat) *big.Rat {
	var least *big.Rat
	for i, height := range heights {
		value := proofbound.RatAdd(height, proofbound.RatMul(rates[i], t))
		value.Sub(value, proofbound.RatMul(curvature[i], t, t))
		if least == nil || value.Cmp(least) < 0 {
			least = value
		}
	}
	if least.Sign() <= 0 {
		return least
	}
	return least.Quo(least, nHigh)
}

// BandHorizon is §10.3's band proof on one support plane: the track holds
// through End with the held vertices' depth HeldDepth, and Rate is the
// largest |h'(0)| over the contact set.
type BandHorizon struct {
	Support   *Support
	Face      planar.SupportFace
	End       *big.Rat
	HeldDepth *big.Rat
	Rate      *big.Rat
}

// ProveBand proves §10.3's band track on support. S must not rotate, so its
// face normal is constant, and every contact vertex's foot must stay inside
// S's face for the whole track. A track with zero depth that reaches the
// duration is an exact persistent touch. ok is false when the plane admits
// no track.
//
// §10.4 runs the same proof over the held vertices of a positive-displacement
// body M and charges δ afterwards: the published depth is the held depth
// widened by 2δ, δ the summed displacement, and each foot's box grows by M's
// δ, since a true contact point lies within it of a held vertex. S must carry
// no displacement, because the track publishes S's held face normal as the
// true one.
func ProveBand(support *Support, resolution, duration *big.Rat,
	poll func() error) (BandHorizon, bool, error) {
	if support.OwnerMotion.Rotating || support.PathS.Path.Drift != nil || support.PathS.HeldDelta.Sign() != 0 {
		return BandHorizon{}, false, nil
	}
	face, ok := planar.BuildSupportFace(support.PathS.Solid, support.PathS.StartPoints, support.Normal, support.Origin)
	if !ok {
		return BandHorizon{}, false, nil
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
		if err := poll(); err != nil {
			return false, err
		}
		t := new(big.Rat).Mul(f, support.Duration)
		k, ok := support.Curvature(t)
		if !ok || !support.ClearAt(t, k, true) {
			return false, nil
		}
		// §10.6: material of S in front of the plane meets no part of M,
		// so M ∩ S lies behind the plane and within the band's depth.
		if _, open, err := support.Column(f, poll); err != nil || !open {
			return false, err
		}
		spans := support.PathM.CornerSpan(new(big.Rat), f)
		depth := new(big.Rat).Add(depthAt(t, k), support.PathM.HeldDelta.Rat())
		return faceContains(&face, spans, support.PathS.Path.Delta,
			support.Contact, support.Lifted, f, depth, poll)
	}
	end, ok, err := GridHorizon(resolution, duration, holds)
	if err != nil || !ok {
		return BandHorizon{}, false, err
	}
	t := new(big.Rat).Mul(end, support.Duration)
	k, _ := support.Curvature(t)
	return BandHorizon{Support: support, Face: face, End: end, HeldDepth: depthAt(t, k), Rate: rate}, true, nil
}

// faceContains proves that each held contact vertex's projected path box,
// widened by depth, remains inside the support face through fraction f.
func faceContains(face *planar.SupportFace, spans sweepmemo.CornerSpans, delta [3]proofarith.Dyadic,
	contact, lifted []int, f, depth *big.Rat, poll func() error) (bool, error) {
	i, j := (face.Drop+1)%3, (face.Drop+2)%3
	for _, index := range slices.Concat(contact, lifted) {
		if err := poll(); err != nil {
			return false, err
		}
		var lo, hi [2]*big.Rat
		for slot, axis := range [2]int{i, j} {
			shift := new(big.Rat).Mul(delta[axis].Rat(), f)
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

// TrackProof is the private certificate of a planar persistent touch or band
// track. It owns copies of both prepared paths, so the manifold and replay
// read no document state.
type TrackProof struct {
	paths     [2]BodyPath
	M, S      int                          // the guest and owner sides, in sweep order
	Contact   []int                        // M vertices in published order
	FeatureM  []reportvocab.ContactFeature // one per Contact entry
	FeatureS  reportvocab.ContactFeature
	Direction measurement.VecMeasurement // the published A-to-B normal
	Angle     units.Value
	DepthUp   float64                  // the published depth, rounded up
	Band      *measurement.Measurement // the published depth; nil for an exact touch
	normal    proofarith.DyV3          // S's exact outward normal
	origin    int                      // an S vertex on the plane
	heldDepth *big.Rat                 // millimetres: the held vertices' band, which replay checks
	deltaM    *big.Rat                 // M's held displacement δ (§10.4); S's is zero
	nHigh     *big.Rat
	support   Support  // the support plane the band was proved on
	rate      *big.Rat // the largest |h'(0)| over the contact set
	widening  *big.Rat // 2δ (§10.4)
}

// NewTrackProof builds the certificate of a proven band from paths, in sweep
// order, and the pair's feature map. The contact set comes first and the
// lifted set after it (§10.5), each in the body's feature order. ok is false
// when a feature does not resolve, the published normal is wider than
// normalResolution, or the depth is not finite.
func NewTrackProof(paths [2]BodyPath, band *BandHorizon, features *reportvocab.PlanarFeatureMap,
	normalResolution units.Value) (*TrackProof, bool) {
	support := band.Support
	solids := [2]*planar.PlanarSolid{paths[0].Solid, paths[1].Solid}
	if solids[support.Owner].Faces == nil || solids[support.Guest].Faces == nil {
		return nil, false
	}
	featureS, ok := features.Feature(support.Owner, planar.PatchFeature{Kind: planar.FeatureFacet,
		Faces: []int{band.Face.ID}})
	if !ok {
		return nil, false
	}
	type entry struct {
		index   int
		feature reportvocab.ContactFeature
		key     [2]int
	}
	var entries []entry
	for _, set := range [2][]int{support.Contact, support.Lifted} {
		from := len(entries)
		for _, index := range set {
			feature, ok := features.Feature(support.Guest, planar.PatchFeature{Kind: planar.FeatureVertex,
				Faces: planar.VertexFaceIDs(solids[support.Guest], index)})
			if !ok {
				return nil, false
			}
			entries = append(entries, entry{index: index, feature: feature, key: features.Order(support.Guest, feature)})
		}
		part := entries[from:]
		sort.SliceStable(part, func(i, j int) bool { return slices.Compare(part[i].key[:], part[j].key[:]) < 0 })
	}
	direction := support.Normal
	if support.Guest == 0 {
		direction = proofarith.DyV3{proofarith.DyNeg(direction[0]), proofarith.DyNeg(direction[1]),
			proofarith.DyNeg(direction[2])}
	}
	reading, angle, ok := pairbox.PlanarNormalMeasurement(direction)
	if !ok || angle.Base() > normalResolution.Base() {
		return nil, false
	}
	widening := proofbound.RatMul(big.NewRat(2, 1), proofarith.DyAdd(paths[0].HeldDelta, paths[1].HeldDelta).Rat())
	depth := new(big.Rat).Add(band.HeldDepth, widening)
	proof := &TrackProof{paths: paths, M: support.Guest, S: support.Owner,
		FeatureS: featureS, normal: support.Normal, origin: solids[support.Owner].Tris[support.Triangle][0],
		Direction: reading, Angle: angle, heldDepth: band.HeldDepth, DepthUp: proofbound.RatFloatUp(depth),
		deltaM: support.PathM.HeldDelta.Rat(), nHigh: support.NormalHigh, support: *support, rate: band.Rate,
		widening: widening}
	if !measurement.FiniteValues(proof.DepthUp) {
		return nil, false
	}
	for _, e := range entries {
		proof.Contact = append(proof.Contact, e.index)
		proof.FeatureM = append(proof.FeatureM, e.feature)
	}
	if depth.Sign() > 0 || band.End.Cmp(big.NewRat(1, 1)) < 0 {
		value := sweeppath.RatFloatNearest(depth)
		bound := proofarith.RationalFloatError(depth, value)
		proof.Band = &measurement.Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: measurement.ExactnessOf(bound)}
	}
	return proof, true
}

// DepthThrough is the band's published depth over [0, f]: the §10.3 bound
// (r·t + K(t)·t²)/|n|_lo at t = f·duration, widened by 2δ. K(t) bounds the
// height curvature over [0, t], so the bound holds at every earlier instant,
// and f lies inside the track, whose clearance and face containment hold
// through its end and so through f.
func (p *TrackProof) DepthThrough(f *big.Rat) (*big.Rat, bool) {
	t := new(big.Rat).Mul(f, p.support.Duration)
	k, ok := p.support.Curvature(t)
	if !ok {
		return nil, false
	}
	depth := p.support.DepthAt(t, k, p.rate)
	depth.Quo(depth, p.support.NormalLow)
	return depth.Add(depth, p.widening), true
}

// RoundedVertices stages both snapshots through the float poses at f and
// returns each body's pose deviation. S must keep its start basis: the track
// admits only a translating S, whose rounded plane then keeps its normal.
func (p *TrackProof) RoundedVertices(f *big.Rat) ([2][]proofarith.DyV3, [2]*big.Rat, bool) {
	var verts [2][]proofarith.DyV3
	var eta [2]*big.Rat
	for i := range p.paths {
		pose, err := p.paths[i].Path.RoundedPoseAt(f)
		if err != nil {
			return verts, eta, false
		}
		if i == p.S && pose.Basis() != p.paths[i].Path.From.Basis() {
			return verts, eta, false
		}
		points, bound, ok, _ := p.paths[i].PointDeviation(pose, f, NoPoll)
		if !ok {
			return verts, eta, false
		}
		verts[i], eta[i] = points, proofarith.FloatRat(bound)
	}
	return verts, eta, true
}

// TrackPoint is one published contact point of a planar track: M's vertex
// and its foot on S's rounded plane.
type TrackPoint struct {
	OnM, OnS measurement.VecMeasurement
}

// Points publishes the contact set from RoundedVertices' vertices and
// deviations, one TrackPoint per Contact entry. Each contact vertex is
// staged through the rounded pose, exactly as ContactPair stages it, and
// published with its foot on S's rounded plane. The rounded and ideal vertex
// differ by at most M's pose deviation, and the two feet by at most both
// deviations, because S only translates and the plane keeps its normal. A
// true contact point of a positive-displacement M lies within its δ of the
// held vertex, and its foot within δ of the held foot, so both balls also
// carry M's δ. ok is false when a point does not fit resolution.
func (p *TrackProof) Points(verts [2][]proofarith.DyV3, eta [2]*big.Rat,
	resolution *big.Rat) ([]TrackPoint, bool) {
	n := RatOfDyV3(p.normal)
	nn := RatDot3(n, n)
	q := RatOfDyV3(verts[p.S][p.origin])
	points := make([]TrackPoint, 0, len(p.Contact))
	for _, index := range p.Contact {
		vertex := RatOfDyV3(verts[p.M][index])
		height := new(big.Rat).Quo(RatDot3(n, RatSub3(vertex, q)), nn)
		var foot [3]*big.Rat
		for k := range 3 {
			foot[k] = new(big.Rat).Sub(vertex[k], new(big.Rat).Mul(height, n[k]))
		}
		onM, okM := PointReading([3]*big.Rat(vertex), new(big.Rat).Add(eta[p.M], p.deltaM), resolution)
		onS, okS := PointReading(foot, proofbound.RatAdd(eta[p.M], eta[p.S], p.deltaM), resolution)
		if !okM || !okS {
			return nil, false
		}
		points = append(points, TrackPoint{OnM: onM, OnS: onS})
	}
	return points, true
}

// PointReading converts an exact point once and widens its ball by the pose
// deviation; the result must fit the resolution.
func PointReading(point [3]*big.Rat, deviation, resolution *big.Rat) (measurement.VecMeasurement, bool) {
	value, ok := pairbox.PointMeasurement(point)
	if !ok {
		return measurement.VecMeasurement{}, false
	}
	bound := new(big.Rat).Add(proofarith.FloatRat(value.Bound.Base()), deviation)
	published := proofbound.RatFloatUp(bound)
	if !measurement.FiniteValues(published) || bound.Cmp(resolution) > 0 ||
		proofarith.FloatRat(published).Cmp(resolution) > 0 {
		return measurement.VecMeasurement{}, false
	}
	value.Bound, value.Exactness = units.Millimeters(published), measurement.ExactnessOf(published)
	return value, true
}

// ReplayHeights is the band's replay check: every held M vertex of the
// rounded pose stays above −(held depth + both deviations) on S's rounded
// plane. It reads the held depth, not the published one: the held vertices
// are what it measures, and the true pair then lies within the published
// band widened by the deviations. It can only refuse; the producer's
// certificate covers the ideal path.
func (p *TrackProof) ReplayHeights(f *big.Rat) (*big.Rat, bool) {
	verts, eta, ok := p.RoundedVertices(f)
	if !ok {
		return nil, false
	}
	deviation := new(big.Rat).Add(eta[0], eta[1])
	origin := verts[p.S][p.origin]
	floor := new(big.Rat).Neg(proofbound.RatMul(new(big.Rat).Add(p.heldDepth, deviation), p.nHigh))
	for _, v := range verts[p.M] {
		if proofarith.DvDot(p.normal, proofarith.DvSub(v, origin)).Rat().Cmp(floor) < 0 {
			return nil, false
		}
	}
	return deviation, true
}
