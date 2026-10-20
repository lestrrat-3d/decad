package planarsweep

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/placedruling"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the rolling band track of docs/multibody-dynamics-design.md
// §10.4 ("Rolling"): a full source cylinder M lying on its side on an exact
// planar body S, M drifting at constant ω and S only translating, keeps a
// §10.3 track whose contact set is the two ends of the contact ruling.
//
// A cylinder is no vertex hull, so the proof reads the centers c± of its two
// end disks, material points on its axis, staged exactly through the start
// pose's float basis B (placedruling.Cylinder). B is orthonormal only to
// rounding, so the start body's section is the disk's image under B; gram
// bounds that, and α = n̂·Bâ is the staged axis column's normal component.
// Over an end disk the least height along the unit normal n̂ is
// n̂·c − r·|P·Bᵀ(u)n̂|, B(u) = R(u)·B and P removing the identity axis; over
// the solid it is the lesser of its two disks', since height is affine along
// the axis. With S's plane at offset d and H(u) = n̂·c(u) − d − r, each rim's
// least height is g(u) = H(u) + r·(1 − |P·Bᵀ(u)n̂|). With ã = Bâ and
// β = |ω×ã|:
//
//	H'(0) = n̂·(v_M − v_S + ω×(c − c_M))           exact
//	|H''| <= |ω|·|ω×(c − c_M)|                    c'' = R(u)·(ω×(ω×(c − c_M))); S only translates
//	|r − r·|P·Bᵀ(u)n̂|| <= r·(gram + α(u)²)        placedruling.Cylinder.SectionDrift
//	|α(u)| <= |α| + β·u                           |R(u)ã − ã| <= β·u
//
// so each end satisfies |g(u)| <= c₀ + (|H'(0)| + 2·r·β·|α|)·u + (K + r·β²)·u²
// on [0, h], c₀ = |H(0)| + r·(gram + α²) and K half the curvature bound, and
// the band depth is that bound at h for the larger coefficients of the two
// ends. The cylinder's lowest point is one of the two rims', so the whole body
// stays above −depth. Each rim's lowest point lies within
// r·(3·gram + (3/2)·|α(u)|) of c − r·n̂ while |α(u)| <= 1/4
// (placedruling.Cylinder.RimDrift); the foot check and ManifoldAt charge that
// drift. A signed-axis start with α = 0 has gram = c₀ = 0, and its drift
// r·√(2·(1 − s)) <= (3/2)·r·β·u, s the cosine of the axis tilt, holds with
// no gate.
//
// S's plane may be face-local (§10.6), a tray's floor with walls in front of
// it: the band search then runs the column test at every grid fraction, so no
// material of S in front of the plane comes near the cylinder inside the
// track.

// rollingCoefficients are the depth bound's terms, constant + rate·t +
// quadratic·t², and the rim drift's, lateralBase + lateral·t. With ã the
// staged axis column, α = n̂·ã, β = |ω×ã| and both ends' H±:
//
//	constant  = max|H±(0)| + r·gram + r·α²   (placedruling.Cylinder.SectionDrift)
//	rate      = max|H±'(0)| + 2·r·β·|α|
//	quadratic = max K + r·β²
//
// since |α(u)| <= |α| + β·u. The drift is r·(3·gram + (3/2)·|α(u)|)
// (placedruling.Cylinder.RimDrift), which needs gram <= 1/16 and
// |α(u)| <= 1/4. An exact start, a signed-axis pose with α = 0, has a zero
// constant and lateralBase, and its drift r·√(2·(1 − s)) <= (3/2)·r·β·u holds
// with no gate.
type rollingCoefficients struct {
	constant, rate, quadratic *big.Rat
	lateralBase, lateral      *big.Rat
	alpha, beta               *big.Rat
	gated                     bool
}

// rollingInput contains the exact source geometry and path readings for a
// cylinder rolling on a stationary-or-translating planar support.
type rollingInput struct {
	cylinder        placedruling.Cylinder
	moving, support Motion
	points          []proofarith.DyV3
	normal          proofarith.DyV3
	heights         [2]proofarith.Dyadic
	alpha           proofarith.Dyadic
}

// rollingCoefficientsOf bounds both end-disk heights and the rim's lateral
// drift from the same exact motion and staged cylinder readings.
func rollingCoefficientsOf(in rollingInput) (rollingCoefficients, bool) {
	radius := in.cylinder.Radius.Rat()
	normal := RatOfDyV3(in.normal)
	relative := RatSub3(in.moving.Velocity, in.support.Velocity)
	out := rollingCoefficients{rate: new(big.Rat), quadratic: new(big.Rat), constant: new(big.Rat),
		alpha: new(big.Rat).Abs(in.alpha.Rat())}
	for i, c := range in.points {
		lever := ratCross3(in.moving.Omega, RatSub3(RatOfDyV3(c), in.moving.Center))
		rate := RatDot3(normal, ratAdd3(relative, lever))
		if rate.Abs(rate).Cmp(out.rate) > 0 {
			out.rate = rate
		}
		curvature, ok := ratSqrtUpRat(proofbound.RatMul(in.moving.OmegaSq, RatDot3(lever, lever)))
		if !ok {
			return rollingCoefficients{}, false
		}
		if k := proofbound.RatMul(curvature, big.NewRat(1, 2)); k.Cmp(out.quadratic) > 0 {
			out.quadratic = k
		}
		if h := new(big.Rat).Abs(in.heights[i].Rat()); h.Cmp(out.constant) > 0 {
			out.constant = h
		}
	}
	// β² = |ω|²·|ã|² − (ω·ã)², exact.
	axis := RatOfDyV3(in.cylinder.Columns[in.cylinder.Axis])
	along := RatDot3(in.moving.Omega, axis)
	tiltSq := new(big.Rat).Sub(proofbound.RatMul(in.moving.OmegaSq, RatDot3(axis, axis)), proofbound.RatMul(along, along))
	tilt, ok := ratSqrtUpRat(tiltSq)
	if !ok {
		return rollingCoefficients{}, false
	}
	out.beta = tilt
	out.constant = proofbound.RatAdd(out.constant, in.cylinder.SectionDrift(in.alpha).Rat())
	out.rate = proofbound.RatAdd(out.rate, proofbound.RatMul(big.NewRat(2, 1), radius, tilt, out.alpha))
	out.quadratic = proofbound.RatAdd(out.quadratic, proofbound.RatMul(radius, tiltSq))
	out.lateral = proofbound.RatMul(big.NewRat(3, 2), radius, tilt)
	out.lateralBase = new(big.Rat)
	if in.cylinder.Gram.Sign() != 0 || in.alpha.Sign() != 0 {
		out.gated = true
		out.lateralBase = in.cylinder.RimDrift(in.alpha.Rat())
	}
	return out, true
}

// at returns the band depth and lateral rim drift at t seconds.
func (c rollingCoefficients) at(t *big.Rat) (*big.Rat, *big.Rat) {
	return proofbound.RatAdd(c.constant, proofbound.RatMul(c.rate, t), proofbound.RatMul(c.quadratic, t, t)),
		proofbound.RatAdd(c.lateralBase, proofbound.RatMul(c.lateral, t))
}

// driftAdmitted reports whether the rim drift's gate holds through t.
func (c rollingCoefficients) driftAdmitted(t *big.Rat) bool {
	return !c.gated || proofbound.RatAdd(c.alpha, proofbound.RatMul(c.beta, t)).Cmp(big.NewRat(1, 4)) <= 0
}

// rollingColumnBox holds what the column test reads of the cylinder: its
// eight staged corners at the start, whose hull holds it, and reach, an upper
// bound on r·√(1 + gram), the farthest any cylinder point lies from the point
// of its axis segment in the same section: that offset is B·w with |w| <= r,
// and |B·w|² <= (1 + gram)·|w|².
type rollingColumnBox struct {
	corners []proofarith.DyV3
	reach   *big.Rat
}

func rollingColumnBoxOf(c placedruling.Cylinder) (rollingColumnBox, bool) {
	corners := c.StagedCorners()
	stretch, ok := ratSqrtUpRat(proofbound.RatAdd(big.NewRat(1, 1), c.Gram.Rat()))
	if !ok {
		return rollingColumnBox{}, false
	}
	return rollingColumnBox{corners: corners[:], reach: proofbound.RatMul(c.Radius.Rat(), stretch)}, true
}

// RollingBand is §10.4's rolling band proof: the track of the cylinder on
// paths[M] over the exact planar body on paths[1−M] holds on the support
// plane Support through fraction End, and Depth bounds its band there. It
// owns copies of both prepared paths.
type RollingBand struct {
	Support      placedruling.Plane
	End, Depth   *big.Rat
	paths        [2]BodyPath
	m, s         int
	cylinder     placedruling.Cylinder
	coefficients rollingCoefficients
}

// ProveRollingBand proves the rolling track of the cylinder staged through
// its start pose on paths[m] (PrepareRollingPath) over the exact planar body
// on paths[1−m] (PreparePlanarPath, zero δ). Each end's foot must stay inside
// S's face: the end centers' ideal path boxes over [0, f], less S's
// translation and grown by the depth and the rim drift, hold every rim point
// near the plane, and their union holds the ruling between the ends.
// Projected along the normal's axis, the union must meet no bounding edge of
// the face and have a corner inside one of its triangles. On a face-local
// plane the column test must hold through f as well. ok is false when the
// pair has no support plane, S rotates, gram exceeds placedruling.GramLimit,
// a bound has no finite reading, or no grid fraction holds.
func ProveRollingBand(paths [2]BodyPath, m int, cylinder placedruling.Cylinder, resolution *big.Rat,
	poll func() error) (*RollingBand, bool, error) {
	band := &RollingBand{paths: paths, m: m, s: 1 - m, cylinder: cylinder}
	S := &band.paths[band.s]
	support, ok, err := placedruling.Support(cylinder, S.Solid, S.StartPoints, poll)
	if err != nil || !ok {
		return nil, false, err
	}
	band.Support = support
	if band.coefficients, ok = band.coefficientsOf(); !ok {
		return nil, false, nil
	}
	duration := band.paths[m].Path.Duration
	box, ok := rollingColumnBoxOf(cylinder)
	if !ok {
		return nil, false, nil
	}
	holds := func(f *big.Rat) (bool, error) {
		if err := poll(); err != nil {
			return false, err
		}
		t := new(big.Rat).Mul(f, duration)
		if !band.coefficients.driftAdmitted(t) {
			return false, nil
		}
		depth, lateral := band.coefficients.at(t)
		if !band.footInside(f, proofbound.RatAdd(depth, lateral)) {
			return false, nil
		}
		return band.column(box, f, poll)
	}
	end, ok, err := GridHorizon(resolution, duration, holds)
	if err != nil || !ok {
		return nil, false, err
	}
	band.End = end
	band.Depth, _ = band.coefficients.at(new(big.Rat).Mul(end, duration))
	return band, true, nil
}

func (b *RollingBand) coefficientsOf() (rollingCoefficients, bool) {
	motionM, okM := MotionOf(b.paths[b.m].SupportPath().Motion)
	motionS, okS := MotionOf(b.paths[b.s].SupportPath().Motion)
	if !okM || !okS || motionS.Rotating || proofarith.DyCmp(b.cylinder.Gram, placedruling.GramLimit) > 0 {
		return rollingCoefficients{}, false
	}
	return rollingCoefficientsOf(rollingInput{
		cylinder: b.cylinder, moving: motionM, support: motionS,
		points: b.paths[b.m].StartPoints, normal: b.Support.Normal,
		heights: b.Support.Heights, alpha: b.Support.Alpha,
	})
}

// column is §10.6's column test through fraction f on a face-local plane.
// Two boxes hold the cylinder at every instant of [0, f]: the coordinate box
// of its staged corners' ideal paths, and the hull of its end centers' ideal
// path boxes grown by box.reach, since every cylinder point lies within reach
// of a point between the end centers. Their intersection, less S's own
// translation over the span, holds the cylinder in S's start frame, and every
// S triangle with a vertex strictly in front of the plane must project apart
// from it. The corners' interval enclosure loosens past a quarter turn while
// the end centers of a cylinder turning about its own axis barely move, so
// the second box carries a whole turn. Both grow with f, so the test is
// monotone. A plane with all of S behind it needs no test.
func (b *RollingBand) column(box rollingColumnBox, f *big.Rat, poll func() error) (bool, error) {
	if !b.Support.Local {
		return true, nil
	}
	zero := new(big.Rat)
	path := b.paths[b.m]
	centers := path.CornerSpan(zero, f)
	path.StartPoints = box.corners
	corners := path.CornerSpan(zero, f)
	S := &b.paths[b.s]
	var lo, hi [3]*big.Rat
	for axis := range 3 {
		low, high := corners.Hull(axis)
		axisLow, axisHigh := centers.Hull(axis)
		low = proofbound.RatMax(low, new(big.Rat).Sub(axisLow, box.reach))
		high = proofbound.RatMin(high, new(big.Rat).Add(axisHigh, box.reach))
		shift := new(big.Rat).Mul(S.Path.Delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(low, proofbound.RatMax(shift, zero))
		hi[axis] = new(big.Rat).Sub(high, proofbound.RatMin(shift, zero))
	}
	solid := planar.PlanarSolid{Verts: S.StartPoints, Tris: S.Solid.Tris}
	return planar.PlanarColumnApart(&solid, b.Support.Normal, S.StartPoints[b.Support.Origin], lo, hi, poll)
}

// footInside encloses the ruling's lateral footprint through f in S's start
// frame, widened by growth, and checks it inside S's face.
func (b *RollingBand) footInside(f, growth *big.Rat) bool {
	spans := b.paths[b.m].CornerSpan(new(big.Rat), f)
	delta := b.paths[b.s].Path.Delta
	var lo, hi [2]*big.Rat
	for slot, direction := range [2]int{(b.Support.Axis + 1) % 3, (b.Support.Axis + 2) % 3} {
		low, high := spans.Hull(direction)
		shift := new(big.Rat).Mul(delta[direction].Rat(), f)
		shiftLo, shiftHi := proofbound.RatMin(shift, new(big.Rat)), proofbound.RatMax(shift, new(big.Rat))
		lo[slot] = proofbound.RatAdd(low, new(big.Rat).Neg(shiftHi), new(big.Rat).Neg(growth))
		hi[slot] = proofbound.RatAdd(high, new(big.Rat).Neg(shiftLo), growth)
	}
	return b.Support.Face.HoldsBox(lo, hi)
}

// RollingTrack is the private certificate of a rolling band or exact rolling
// touch track. It owns copies of both prepared paths. M's source points are
// the eight corners of its identity disk-by-interval box, whose hull holds
// the whole cylinder, so their deviation bounds every cylinder point's.
type RollingTrack struct {
	paths              [2]BodyPath
	M, S               int                // the cylinder and planar sides, in sweep order
	ends               [2]proofarith.DyV3 // M's end-disk centers at the identity pose, in published order
	radius             *big.Rat
	normal             proofarith.DyV3 // S's exact unit outward normal, a signed axis
	origin             int             // an S vertex on the plane
	FeatureM, FeatureS contactFeature
	Direction          measurement.VecMeasurement // the published A-to-B normal
	Angle              units.Value
	coefficients       rollingCoefficients
	depth              *big.Rat // mm
	depthUp            float64
	Band               *measurement.Measurement // the published depth; nil for an exact touch
}

// Track builds the certificate of b. source are M's end-disk centers at the
// identity pose; the ends publish in the exact coordinate order of their
// start rims, as ContactPair publishes the ruling, and a rigid motion keeps
// that pairing. ok is false when the published normal has no finite reading
// or is wider than normalResolution, or the depth is not finite.
func (b *RollingBand) Track(source [2]proofarith.DyV3, featureM, featureS contactFeature,
	normalResolution units.Value) (*RollingTrack, bool) {
	direction := b.Support.Normal
	if b.m == 0 {
		direction = proofarith.DvSub(proofarith.DyV3{}, direction)
	}
	reading, angle, ok := pairbox.PlanarNormalMeasurement(direction)
	if !ok || angle.Base() > normalResolution.Base() {
		return nil, false
	}
	ends := source
	M := &b.paths[b.m]
	starts := [2]proofarith.DyV3{M.StartPoints[0], M.StartPoints[1]}
	if ordered := clearance.OrderedRulingEnds(starts); !proofarith.DvEqual(ordered[0], starts[0]) {
		ends = [2]proofarith.DyV3{ends[1], ends[0]}
	}
	p := &RollingTrack{paths: b.paths, M: b.m, S: b.s, ends: ends, radius: b.cylinder.Radius.Rat(),
		normal: b.Support.Normal, origin: b.Support.Origin, FeatureM: featureM, FeatureS: featureS,
		Direction: reading, Angle: angle, coefficients: b.coefficients, depth: b.Depth,
		depthUp: proofbound.RatFloatUp(b.Depth)}
	if !measurement.FiniteValues(p.depthUp) {
		return nil, false
	}
	if b.Depth.Sign() > 0 || b.End.Cmp(big.NewRat(1, 1)) < 0 {
		value := sweeppath.RatFloatNearest(b.Depth)
		bound := proofarith.RationalFloatError(b.Depth, value)
		p.Band = &measurement.Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: measurement.ExactnessOf(bound)}
	}
	return p, true
}

// DepthThrough is the band depth over [0, f]: every coefficient's term is
// nondecreasing in time, so their value at f bounds every earlier instant.
func (p *RollingTrack) DepthThrough(f *big.Rat) *big.Rat {
	depth, _ := p.coefficients.at(new(big.Rat).Mul(f, p.paths[p.M].Path.Duration))
	return depth
}

// rounded stages both bodies through their float poses at f and returns each
// pose with its deviation and S's staged vertices. S must keep its start
// basis, so its rounded plane keeps its normal.
func (p *RollingTrack) rounded(f *big.Rat) ([2]r3.Transform, [2]*big.Rat, []proofarith.DyV3, bool) {
	var poses [2]r3.Transform
	var eta [2]*big.Rat
	var vertsS []proofarith.DyV3
	for i := range p.paths {
		pose, err := p.paths[i].Path.RoundedPoseAt(f)
		if err != nil {
			return poses, eta, nil, false
		}
		if i == p.S && pose.Basis() != p.paths[i].Path.From.Basis() {
			return poses, eta, nil, false
		}
		points, bound, ok, _ := p.paths[i].PointDeviation(pose, f, NoPoll)
		if !ok {
			return poses, eta, nil, false
		}
		if i == p.S {
			vertsS = points
		}
		poses[i], eta[i] = pose, proofarith.FloatRat(bound)
	}
	return poses, eta, vertsS, true
}

// ManifoldAt publishes the two ruling ends at an exact fraction. Each end
// center is staged through M's rounded pose and lowered by r along the
// normal; the true rim point lies within M's pose deviation plus the lateral
// drift of it, and its foot on S's rounded plane within S's deviation more.
// The separation is the band's two-sided bound.
func (p *RollingTrack) ManifoldAt(f *big.Rat, req report.ContactRequest) (*contactManifold, error) {
	poses, eta, vertsS, ok := p.rounded(f)
	if !ok {
		return nil, fmt.Errorf("%w: rolling contact track pose has no finite bound", decaderr.ErrUnsupported)
	}
	resolution, okResolution := sweeppath.ExactBaseValue(req.PointResolution)
	if !okResolution {
		return nil, fmt.Errorf("%w: rolling contact track resolution is invalid", decaderr.ErrUnsupported)
	}
	n := RatOfDyV3(p.normal)
	q := RatOfDyV3(vertsS[p.origin])
	_, lateral := p.coefficients.at(new(big.Rat).Mul(f, p.paths[p.M].Path.Duration))
	separation := measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(p.depthUp),
		Exactness: measurement.ExactnessOf(p.depthUp)}
	points := make([]contactPoint, 0, len(p.ends))
	for _, end := range p.ends {
		center := RatOfDyV3(proofarith.DvTransform(poses[p.M], end))
		var rim, foot [3]*big.Rat
		for k := range 3 {
			rim[k] = new(big.Rat).Sub(center[k], new(big.Rat).Mul(p.radius, n[k]))
		}
		height := RatDot3(n, RatSub3(motionbound.RatVec(rim), q))
		for k := range 3 {
			foot[k] = new(big.Rat).Sub(rim[k], new(big.Rat).Mul(height, n[k]))
		}
		onM, okM := PointReading(rim, proofbound.RatAdd(eta[p.M], lateral), resolution)
		onS, okS := PointReading(foot, proofbound.RatAdd(eta[p.M], eta[p.S], lateral), resolution)
		if !okM || !okS {
			return nil, fmt.Errorf("%w: rolling contact track point exceeds point resolution", decaderr.ErrUnsupported)
		}
		point := contactPoint{Normal: p.Direction, NormalAngle: p.Angle, Separation: separation}
		if p.M == 0 {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = p.FeatureM, p.FeatureS
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = p.FeatureS, p.FeatureM
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	return &contactManifold{Points: points}, nil
}

// ReplayPoses replays the track at f without rerunning the pair relation.
// Both rounded poses must fit resolution of their ideal poses, and each
// rounded end center's height less r above S's rounded plane must stay within
// the depth widened by both deviations: the ideal one lies within the depth,
// and the deviations bound the move to the rounded one. It can only refuse;
// the producer's certificate covers the ideal path.
func (p *RollingTrack) ReplayPoses(f *big.Rat, resolution units.Value) (r3.Transform, r3.Transform, error) {
	poses, eta, vertsS, ok := p.rounded(f)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rolling replay pose has no finite error bound", decaderr.ErrUnsupported)
	}
	deviation := new(big.Rat).Add(eta[0], eta[1])
	limit, ok := sweeppath.ExactBaseValue(resolution)
	if !ok || deviation.Cmp(limit) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rolling replay pose exceeds point resolution", decaderr.ErrUnsupported)
	}
	band := new(big.Rat).Add(p.depth, deviation)
	q := vertsS[p.origin]
	for _, end := range p.ends {
		height := proofarith.DvDot(p.normal, proofarith.DvSub(proofarith.DvTransform(poses[p.M], end), q)).Rat()
		height.Sub(height, p.radius)
		if height.Abs(height).Cmp(band) > 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rolling replay pose leaves the certified band", decaderr.ErrUnsupported)
		}
	}
	return poses[0], poses[1], nil
}
