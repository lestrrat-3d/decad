package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyMotion (docs/motion-check-design.md §3, §5-§8)
// and the engine it shares with Document.VerifyLinkage
// (docs/linkage-check-design.md §6): validation, the swept-box exclusion, the
// per-pose pair proof over transient placements, the two-sided interval
// certificate, and the report assembly. motion.go owns the vocabulary and
// motion_bound.go the bounds; linkage_verify.go drives the same engine over a
// chain of joints.
//
// The engine moves one or more GROUPS of bodies, each group rigid under one
// pose per parameter, against every other live body of the document. A
// VerifyMotion call is one group under one Motion; a VerifyLinkage call is one
// group per link. A pair is a moving body and a static body, or two moving
// bodies of different groups; two bodies of one group form no pair, since a
// shared rigid motion preserves their relation. The driver (motionDriver)
// supplies each group's float and ideal poses at a parameter and each pair's
// travel bound across an interval; everything else is the engine's.
//
// The check evaluates the two endpoints, then bisects on a dyadic grid of the
// path (§6): first for the verdict, until every interval is certified clear,
// colliding at both ends, or no wider than the resolution — a colliding
// interval with a collision-free end is halved toward the contact's onset;
// then for the
// readings, refining the interval holding the smallest certified clearance
// while the whole-path reading fails the tolerance gate or a requested margin
// is neither proven nor disproven, on the same floor. An interval the floor
// leaves uncertified reads IntervalUndecided and makes the report Suspect;
// nothing between evaluated poses is ever assumed clear.

// transientProducer is the reserved producer identity every transient pose
// placement is built under. It never enters the document: a transient body is
// never committed, never published, and its provenance is never read back.
const transientProducer producerID = -1

// motionDomain is the parameter a check bisects: its Kind, its two endpoints
// as stated, and their exact denotations. A Motion's domain is its own From
// and To; a linkage drive's is the Dimensionless fraction [0, 1], exactly a
// Between's (fractionDomain).
type motionDomain struct {
	quantity   units.Kind
	from, to   units.Value
	fromP, toP motionbound.MotionParam
}

// paramKind is the Kind the parameter, and so its resolution, is stated in.
func (s motionDomain) paramKind() units.Kind {
	return s.quantity
}

// fractionDomain is the Dimensionless fraction s ∈ [0, 1] a Between and a
// linkage drive both run over.
func fractionDomain() motionDomain {
	return motionDomain{
		quantity: units.Dimensionless,
		from:     units.Scalar(0),
		to:       units.Scalar(1),
		fromP:    motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)},
		toP:      motionbound.MotionParam{Turn: new(big.Rat), Base: big.NewRat(1, 1)},
	}
}

// motionSpec is a validated Motion read into the fields every pose and bound
// is built from. A Between's parameter runs over the dimensionless fraction
// [0, 1], so its from and to are units.Scalar(0) and units.Scalar(1), and
// between and screw carry its poses and the screw r3 reads off them.
type motionSpec struct {
	motionDomain
	motion  Motion
	kind    motionbound.MotionKind
	center  r3.Vec
	axis    r3.Vec
	dir     r3.Vec
	between Between
	screw   r3.Screw
	frame   motionbound.MotionFrame
}

// resolveMotion validates m (docs/motion-check-design.md §2, §8): the
// variant's own field refusals, From == To, and both endpoint poses building
// under r3. A nil motion is ErrDegenerate.
func resolveMotion(m Motion) (motionSpec, error) {
	var spec motionSpec
	switch mv := m.(type) {
	case Revolute:
		if err := mv.validate(); err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{kind: motionbound.MotionRevolute, center: mv.Center, axis: mv.Axis,
			motionDomain: motionDomain{quantity: units.Angle, from: mv.From, to: mv.To}}
	case *Revolute:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Prismatic:
		if err := mv.validate(); err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{kind: motionbound.MotionPrismatic, dir: mv.Dir,
			motionDomain: motionDomain{quantity: units.Length, from: mv.From, to: mv.To}}
	case *Prismatic:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Between:
		sc, err := mv.resolve()
		if err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{kind: motionbound.MotionBetween, between: mv, screw: sc,
			motionDomain: motionDomain{quantity: units.Dimensionless, from: units.Scalar(0), to: units.Scalar(1)}}
	case *Between:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case nil:
		return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
	default:
		return motionSpec{}, fmt.Errorf(`%w: a motion of type %T is not one this evaluator checks`, ErrUnsupported, m)
	}
	spec.motion = m
	var okF, okT bool
	spec.fromP, okF = motionbound.ExactMotionParam(spec.from)
	spec.toP, okT = motionbound.ExactMotionParam(spec.to)
	if !okF || !okT {
		return motionSpec{}, fmt.Errorf(`%w: a motion endpoint is not representable`, ErrNotFinite)
	}
	if sameMotionValue(spec.from, spec.to) {
		return motionSpec{}, fmt.Errorf(`%w: From and To are both %s, which names no path`, ErrDegenerate, spec.from)
	}
	for _, end := range []units.Value{spec.from, spec.to} {
		if _, err := m.PoseAt(end); err != nil {
			return motionSpec{}, err
		}
	}
	frame, ok := newMotionFrame(spec)
	if !ok {
		return motionSpec{}, fmt.Errorf(`%w: the motion's axis is not representable`, ErrNotFinite)
	}
	spec.frame = frame
	return spec, nil
}

// defaultResolution is the published default resolution, |To − From|/1024
// carried in From's unit. When that value underflows in From's unit or its
// base unit, it returns the smallest positive resolution in From's unit
// accepted by WithResolution and reports clamped so the check uses that floor.
func (s motionDomain) defaultResolution() (units.Value, bool) {
	var d *big.Rat
	if s.quantity == units.Length {
		// The exact base-unit difference survives conversion that could round
		// two distinct endpoints to the same float in From's unit.
		d = new(big.Rat).Sub(s.toP.Base, s.fromP.Base)
		d.Abs(d)
		d.Quo(d, proofarith.FloatRat(s.from.Unit().Factor()))
	} else {
		from := proofarith.FloatRat(s.from.Mag())
		toMag, err := s.to.In(s.from.Unit())
		to := proofarith.FloatRat(toMag)
		if err != nil || from == nil || to == nil {
			return units.New(0, s.from.Unit()), false
		}
		d = new(big.Rat).Sub(to, from)
		d.Abs(d)
	}
	mag, _ := d.Quo(d, big.NewRat(1024, 1)).Float64()
	base, _ := units.BaseUnit(s.paramKind())
	reported := units.New(mag, s.from.Unit())
	converted, err := reported.In(base)
	if mag > 0 && err == nil && converted > 0 {
		return reported, false
	}
	// A positive exact floor can underflow in From's unit or its base unit.
	// Binary search positive finite float bits for the first value whose base
	// conversion is nonzero, which is also the first WithResolution accepts.
	low, high := uint64(0), math.Float64bits(1)
	for high-low > 1 {
		mid := low + (high-low)/2
		candidate := units.New(math.Float64frombits(mid), s.from.Unit())
		converted, err := candidate.In(base)
		if err != nil || converted == 0 {
			low = mid
			continue
		}
		high = mid
	}
	return units.New(math.Float64frombits(high), s.from.Unit()), true
}

// defaultResolutionParam is |θ(To) − θ(From)|/1024 taken part by part over
// exact rationals: never zero for a validated motion, and exactly one
// dyadic step of depth ten whatever units From and To were stated in.
func (s motionDomain) defaultResolutionParam() motionbound.MotionParam {
	part := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		d.Abs(d)
		return d.Quo(d, big.NewRat(1024, 1))
	}
	return motionbound.MotionParam{Turn: part(s.fromP.Turn, s.toP.Turn), Base: part(s.fromP.Base, s.toP.Base)}
}

// label is the published parameter of the pose at fraction f of the path:
// From and To exactly at the ends, and otherwise the float nearest the exact
// interpolation carried in From's unit — exact itself whenever From and To
// share a unit and the dyadic step is representable. It is a label: every
// bound is built from the exact parameter fromP.lerp(toP, f), and
// motionbound.PoseDeviation charges whatever separates the pose PoseAt builds from this
// label and the ideal pose at f.
func (s motionDomain) label(f *big.Rat) units.Value {
	switch {
	case f.Sign() == 0:
		return s.from
	case f.Cmp(big.NewRat(1, 1)) == 0:
		return s.to
	}
	from := proofarith.FloatRat(s.from.Mag())
	toMag, err := s.to.In(s.from.Unit())
	to := proofarith.FloatRat(toMag)
	if err != nil || from == nil || to == nil {
		// Unreachable for a validated motion, whose endpoints both convert;
		// the exact parameter still governs every bound if it were reached.
		return s.from
	}
	d := new(big.Rat).Sub(to, from)
	d.Mul(d, f)
	mag, _ := d.Add(d, from).Float64()
	return units.New(mag, s.from.Unit())
}

// resolve validates a Between for VerifyMotion (docs/motion-check-design.md
// §2, §8): its own field refusals, then From == To, then the screw r3 reads
// off the relative motion, which must be representable and must not be the
// zero screw — a relative motion with neither angle nor slide names no path,
// since PoseAt is then From at every s.
func (m Between) resolve() (r3.Screw, error) {
	if err := m.validate(); err != nil {
		return r3.Screw{}, err
	}
	if m.From == m.To {
		return r3.Screw{}, fmt.Errorf(`%w: a between whose From equals its To names no path`, ErrDegenerate)
	}
	sc, err := m.screw()
	if err != nil {
		return r3.Screw{}, err
	}
	if sc.Angle.Mag() == 0 && sc.Slide == 0 {
		return r3.Screw{}, fmt.Errorf(`%w: a between whose relative motion is the zero screw names no path`, ErrDegenerate)
	}
	return sc, nil
}

// resolveMotionAs resolves a dereferenced pointer motion while keeping the
// caller's own value as the stated motion the report echoes.
func resolveMotionAs(stated Motion, value Motion) (motionSpec, error) {
	spec, err := resolveMotion(value)
	if err != nil {
		return motionSpec{}, err
	}
	spec.motion = stated
	return spec, nil
}

// resolveMovers validates the moving set (docs/motion-check-design.md §3, §8).
func (d *Document) resolveMovers(moving []*Body) error {
	if len(moving) == 0 {
		return fmt.Errorf(`%w: an empty moving set moves nothing`, ErrDegenerate)
	}
	seen := make(map[*Body]struct{}, len(moving))
	for _, b := range moving {
		if err := d.requireLive(b); err != nil {
			return err
		}
		if _, dup := seen[b]; dup {
			return fmt.Errorf(`%w: a body is listed twice in the moving set`, ErrDegenerate)
		}
		seen[b] = struct{}{}
		if b.payload == nil {
			return fmt.Errorf(`%w: this evaluator cannot move a body it did not build`, ErrUnsupported)
		}
	}
	return nil
}

// VerifyMotion checks whether the rigid set moving, carried along m, meets
// any other live body of d anywhere on the path (docs/motion-check-design.md).
// It never changes the document: each pose re-evaluates a mover's payload
// under the composed motion as a transient body that is never committed, so
// Document.Bodies(), every body's live state and the next producer identity
// are what they were before the call.
//
// Every pair result at a pose is Verify's own row or diagnostic. Between two
// adjacent poses the interval certificate (§5.2) proves the path clear only
// when every (mover, static) pair's two proven lower bounds together exceed
// the farthest any mover point can travel across the interval; an interval it
// cannot certify down to the resolution is IntervalUndecided and the report
// reads Suspect, never Sound. A pose publishes a Collision only for an overlap
// that survives the pose's own deviation from the ideal motion (§5.1).
//
// moving MUST be non-empty, hold live bodies of d, and list no body twice.
// A nil context returns ErrDegenerate before validation. Other validation
// errors precede cancellation; after validation a canceled context returns
// ctx.Err() and no report.
func (d *Document) VerifyMotion(ctx context.Context, moving []*Body, m Motion, opts ...MotionOption) (*MotionReport, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control motion verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := d.resolveMovers(moving); err != nil {
		return nil, err
	}
	spec, err := resolveMotion(m)
	if err != nil {
		return nil, err
	}
	cfg, err := resolveMotionOptions(opts, spec)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	run := &motionRun{ctx: ctx, d: d, spec: spec, cfg: cfg, cache: &bodyGeomCache{}}
	run.setup(moving)
	poses, spans, err := run.refine()
	if err != nil {
		return nil, err
	}
	return run.publish(poses, spans), nil
}

// motionRun is one call's working state, for VerifyMotion and VerifyLinkage
// alike. It lives in the call alone, never on the Document.
type motionRun struct {
	ctx       context.Context //nolint:containedctx // motionRun is per-call state and never outlives its call.
	d         *Document
	transient *Document // per-call identity counters for uncommitted pose bodies
	dom       motionDomain
	cfg       motionConfig
	cache     *bodyGeomCache
	drive     motionDriver
	movers    []motionMover  // every moving body, group by group
	statics   []motionStatic // every other live body, in Document.Bodies() order
	// pairs is, per mover, its row of pairs in pair order: every static body
	// in order, then every body of every later group in order.
	pairs [][]motionPair
	// declared names the pairs a linkage declares as joint contacts, each in
	// both orders; nil for VerifyMotion.
	declared map[[2]*Body]struct{}

	// VerifyMotion's own motion. spec is the validated Motion; stretch and
	// stretchEnd are motionbound.PathAreaUpper's stretch base (§5.1) for a
	// pose before the end and for the end itself: exactly 1 for a Revolute
	// and a Prismatic; motionbound.BasisSigmaUpper of From, and at s = 1 the
	// larger of From's and To's, for a Between.
	spec                motionSpec
	stretch, stretchEnd float64
}

// motionDriver is what a check's moving groups answer for: their poses at a
// parameter and a pair's travel bound across an interval.
type motionDriver interface {
	// posesAt builds every group's pose at fraction f of the path, published
	// as the parameter at, whose exact denotation is param.
	posesAt(f *big.Rat, at units.Value, param motionbound.MotionParam) ([]motionGroupPose, error)
	// travel is a proven upper bound, as an exact rational, on how much the
	// distance between the two bodies of pair k of mover i can change while
	// the parameter runs from a to b; nil when no bound exists.
	travel(i, k int, a, b motionbound.MotionParam) *big.Rat
}

// motionGroupPose is one moving group's pose at one parameter: the float
// transform composed onto each of its bodies' own placements, the ideal poses
// η is charged against, PathAreaUpper's stretch base, and, for a linkage, the
// joint value of the group's link.
type motionGroupPose struct {
	pose    r3.Transform
	ideals  []motionbound.IdealPose
	stretch float64
	value   units.Value
}

type motionMover struct {
	body     *Body
	group    int
	validity ValidityResult
	rho      float64 // ρ_max under VerifyMotion's revolute or between motion
	r0       float64 // the record radius motionbound.PoseDeviation charges at
	area     float64 // a proven upper bound on the mover's surface area at rest
	sigma    float64 // a proven lower bound on its placement's smallest singular value
}

type motionStatic struct {
	body     *Body
	validity ValidityResult
}

// motionPair is one pair's standing for the whole call.
type motionPair struct {
	// other is the partner mover's index, or −1 when the partner is the
	// static body at the pair's own position in its row.
	other int
	// excluded: the swept-box exclusion proved the pair apart over the whole
	// path, with lower a proven lower bound on its distance throughout.
	excluded bool
	lower    float64
	// invalid: an operand is not a proven-valid body; the pair is never
	// evaluated and leaves every interval undecided.
	invalid bool
	// sheet: an operand is a sheet; the pair reads DiagUnsupportedPairSheet
	// at every pose and leaves every interval undecided.
	sheet bool
	// declared: a linkage's declared joint contact
	// (docs/linkage-check-design.md §5.4). It is evaluated at every pose and
	// publishes a transferred collision and a measured gap row, but no
	// undecided, touching or tolerance finding, and it enters no interval
	// certificate and no whole-path reading.
	declared bool
}

func (p motionPair) evaluated() bool { return !p.excluded && !p.invalid && !p.sheet }

// motionSweptBox is one mover's swept box over the whole path, as exact
// rational extremes per axis; ok is false when none could be formed.
type motionSweptBox struct {
	lo, hi motionbound.RatVec
	ok     bool
}

// motionPairPose is one pair's answer at one pose. A gap is a proven interval
// [lo, hi] on the distance between the IDEAL placed bodies, η already
// charged.
type motionPairPose struct {
	hasGap    bool
	lo, hi    float64
	diam      float64
	collision bool
}

// motionPose is one evaluated pose. result carries the pose's rows in
// PoseResult's shape, with Pose the first group's pose — the whole pose for
// VerifyMotion's one group. collisions name the moving body as Moving and its
// partner, static or moving, as Static.
type motionPose struct {
	f          *big.Rat
	param      motionbound.MotionParam
	groups     []motionGroupPose
	violated   bool // some pair's gap here is proven below the requested minimum
	result     PoseResult
	pairs      [][]motionPairPose
	findings   []Diagnostic // this pose's findings, in report order
	collisions []Collision
}

// motionPlaced is one mover's transient placement at one pose, with the pose
// deviation η it carries and the swept-volume allowance a collision on it
// must clear.
type motionPlaced struct {
	body      *Body
	eta       float64
	allowance float64
}

// setup reads VerifyMotion's one moving group: ρ_max for a revolute or a
// between, the stretch bases, and the swept box each mover's exclusion
// compares the static bodies against.
func (r *motionRun) setup(moving []*Body) {
	r.dom = r.spec.motionDomain
	r.drive = singleMotion{r: r}
	r.readBodies([][]*Body{moving})
	r.stretch, r.stretchEnd = 1, 1
	if r.spec.kind == motionbound.MotionBetween {
		r.stretch = motionbound.BasisSigmaUpper(r.spec.between.From)
		r.stretchEnd = math.Max(r.stretch, motionbound.BasisSigmaUpper(r.spec.between.To))
	}
	zero := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	swept := make([]motionSweptBox, len(r.movers))
	for i := range r.movers {
		mv := &r.movers[i]
		if r.spec.kind != motionbound.MotionPrismatic {
			mv.rho = moverAxisRadius(mv.body, r.spec.frame)
		}
		// The swept box covers every pose from where the path's box was read:
		// for a Revolute or a Prismatic that is the mover at rest — the
		// parameter 0 — so its travel runs from 0 to the farther endpoint,
		// never merely across [From, To]; for a Between it is the From-placed
		// box at s = 0, From itself, so the travel is the whole path's.
		travel := maxRat(
			motionbound.MoverTravel(r.spec.frame, mv.rho, zero, r.spec.fromP),
			motionbound.MoverTravel(r.spec.frame, mv.rho, zero, r.spec.toP),
		)
		lo, hi, ok := moverSweptBox(mv.body.bounds, r.spec.frame, travel)
		swept[i] = motionSweptBox{lo: lo, hi: hi, ok: ok}
	}
	r.formPairs(swept)
}

// readBodies reads every moving group's bodies and every static body: the
// transient identity counters, each body's validity, and each mover's record
// radius, area and singular-value bound.
func (r *motionRun) readBodies(groups [][]*Body) {
	// Payload rebuilding mints level and curve denotations. Start above every
	// live identity, then mint only on this copy: successive pose bodies stay
	// distinct from static geometry without changing the caller's document.
	live := r.d.liveBodies()
	r.transient = &Document{
		bodies:       live,
		nextProducer: r.d.nextProducer,
		nextLevel:    r.d.nextLevel,
		nextCurve:    r.d.nextCurve,
	}
	isMover := make(map[*Body]struct{})
	for g, bodies := range groups {
		for _, b := range bodies {
			isMover[b] = struct{}{}
			r.movers = append(r.movers, motionMover{body: b, group: g, validity: publishValidityResult(b, bodyValidityEvidence(r.ctx, b))})
		}
	}
	for _, b := range live {
		if _, ok := isMover[b]; ok {
			continue
		}
		r.statics = append(r.statics, motionStatic{body: b, validity: publishValidityResult(b, bodyValidityEvidence(r.ctx, b))})
	}
	for i := range r.movers {
		mv := &r.movers[i]
		mv.r0 = moverRecordRadius(r.ctx, mv.body)
		mv.area = proofbound.AbsSumUpper(mv.body.area.Value.Base(), mv.body.area.Bound.Base())
		mv.sigma = motionbound.BasisSigmaLower(mv.body.payload.transform())
	}
}

// formPairs forms every mover's row of pairs in pair order and settles each
// pair's standing for the whole call. A (mover, static) pair is excluded when
// the mover's swept box separates from the static body's box; a pair of two
// movers is always evaluated.
func (r *motionRun) formPairs(swept []motionSweptBox) {
	r.pairs = make([][]motionPair, len(r.movers))
	for i, mv := range r.movers {
		row := make([]motionPair, 0, len(r.statics))
		for _, st := range r.statics {
			pair := staticPair(mv, st, swept[i])
			pair.declared = r.isDeclared(mv.body, st.body)
			row = append(row, pair)
		}
		for o := i + 1; o < len(r.movers); o++ {
			other := r.movers[o]
			if other.group == mv.group {
				continue
			}
			pair := motionPair{other: o, declared: r.isDeclared(mv.body, other.body)}
			switch {
			case mv.validity.Outcome != ValidityValid || other.validity.Outcome != ValidityValid:
				pair.invalid = true
			case mv.body.Kind() == BodySheet || other.body.Kind() == BodySheet:
				pair.sheet = true
			}
			row = append(row, pair)
		}
		r.pairs[i] = row
	}
}

// isDeclared reports whether a and b form a declared joint contact.
func (r *motionRun) isDeclared(a, b *Body) bool {
	_, ok := r.declared[[2]*Body{a, b}]
	return ok
}

// staticPair settles a (mover, static) pair: invalid when an operand is not
// proven valid, excluded when the swept-box exclusion separates it, a sheet
// pair when an operand is a sheet, and evaluated otherwise.
func staticPair(mv motionMover, st motionStatic, swept motionSweptBox) motionPair {
	pair := motionPair{other: -1}
	if mv.validity.Outcome != ValidityValid || st.validity.Outcome != ValidityValid {
		pair.invalid = true
		return pair
	}
	if swept.ok {
		if lower, ok := sweptBoxLower(swept.lo, swept.hi, st.body.bounds); ok {
			pair.excluded, pair.lower = true, lower
			return pair
		}
	}
	pair.sheet = mv.body.Kind() == BodySheet || st.body.Kind() == BodySheet
	return pair
}

// partner is the caller's own body pair k of mover i names beside the mover.
func (r *motionRun) partner(i, k int) *Body {
	if o := r.pairs[i][k].other; o >= 0 {
		return r.movers[o].body
	}
	return r.statics[k].body
}

// singleMotion drives VerifyMotion's one group along its Motion.
type singleMotion struct{ r *motionRun }

func (m singleMotion) posesAt(f *big.Rat, at units.Value, param motionbound.MotionParam) ([]motionGroupPose, error) {
	r := m.r
	pose, err := r.spec.motion.PoseAt(at)
	if err != nil {
		return nil, err
	}
	ideals := []motionbound.IdealPose{r.spec.frame.At(param)}
	stretch := r.stretch
	if r.spec.kind == motionbound.MotionBetween && f.Cmp(big.NewRat(1, 1)) == 0 {
		// PoseAt(1) returns the stated To, which the exact screw of the read
		// parameters rebuilds only to rounding: the pose is charged against
		// both the ideal end and To itself (§5.1, η_1 = max(η_ideal, η_To)).
		ideals = append(ideals, r.spec.frame.StatedEnd())
		stretch = r.stretchEnd
	}
	return []motionGroupPose{{pose: pose, ideals: ideals, stretch: stretch}}, nil
}

// travel is τ of docs/motion-check-design.md §5.2 for mover i: every pair of
// one group's mover has a static partner, which does not move.
func (m singleMotion) travel(i, _ int, a, b motionbound.MotionParam) *big.Rat {
	return motionbound.MoverTravel(m.r.spec.frame, m.r.movers[i].rho, a, b)
}

// maxRat is the larger of two optional rationals; a nil (unbounded) operand
// wins.
func maxRat(a, b *big.Rat) *big.Rat {
	if a == nil || b == nil {
		return nil
	}
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// motionSpan is one interval's standing between two adjacent poses.
type motionSpan struct {
	outcome   IntervalOutcome
	clearance *Measurement
}

// refine evaluates the two endpoints and bisects (§6), returning the poses in
// parameter order and the intervals between them.
func (r *motionRun) refine() ([]*motionPose, []motionSpan, error) {
	var poses []*motionPose
	for _, end := range []struct {
		f  *big.Rat
		at units.Value
	}{{new(big.Rat), r.dom.from}, {big.NewRat(1, 1), r.dom.to}} {
		pose, err := r.evaluatePose(end.f, end.at)
		if err != nil {
			return nil, nil, err
		}
		poses = append(poses, pose)
	}
	spans := []motionSpan{r.intervalVerdict(poses[0], poses[1])}
	for {
		k := r.nextRefinement(poses, spans)
		if k < 0 {
			break
		}
		f := new(big.Rat).Add(poses[k].f, poses[k+1].f)
		f.Quo(f, big.NewRat(2, 1))
		mid, err := r.evaluatePose(f, r.dom.label(f))
		if err != nil {
			return nil, nil, err
		}
		poses = slices.Insert(poses, k+1, mid)
		spans[k] = r.intervalVerdict(poses[k], mid)
		spans = slices.Insert(spans, k+1, r.intervalVerdict(mid, poses[k+2]))
	}
	if err := r.ctx.Err(); err != nil {
		return nil, nil, err
	}
	return poses, spans, nil
}

// nextRefinement picks the interval §6 bisects next, or −1 when refinement is
// done. Step 5 comes first: the first interval in traversal order, still wider
// than the resolution, that is either undecided or colliding at one end only —
// halving the latter walks the first collision toward the contact's onset,
// while an interval colliding at both ends says nothing more for being split.
// Step 6
// follows: the interval holding the smallest certified clearance (ties in
// traversal order), while the whole-path reading would fail the tolerance gate
// or a requested margin is neither proven nor disproven by that interval, and
// only while it is wider than the resolution. Every interval narrower than the
// floor stops, so the loop ends.
func (r *motionRun) nextRefinement(poses []*motionPose, spans []motionSpan) int {
	allClear := true
	smallest := -1
	for k, span := range spans {
		startHits, endHits := len(poses[k].collisions) > 0, len(poses[k+1].collisions) > 0
		onset := span.outcome == IntervalColliding && startHits != endHits
		if (span.outcome == IntervalUndecided || onset) && r.wide(poses[k], poses[k+1]) {
			return k
		}
		if span.outcome != IntervalClear {
			allClear = false
		}
		if span.clearance != nil && (smallest < 0 || span.clearance.Value.Base() < spans[smallest].clearance.Value.Base()) {
			smallest = k
		}
	}
	if smallest < 0 || !r.wide(poses[smallest], poses[smallest+1]) {
		return -1
	}
	if allClear {
		if reading, _ := r.pathClearance(poses, spans[smallest].clearance); reading != nil && reading.Tolerance.State != ToleranceSatisfied {
			return smallest
		}
	}
	if r.cfg.minimumMM != nil && !anyViolated(poses) && !r.meetsMinimum(spans[smallest].clearance) {
		return smallest
	}
	return -1
}

// wide reports whether the interval between two poses is wider than the
// resolution.
func (r *motionRun) wide(a, b *motionPose) bool {
	return motionbound.ExceedsResolution(a.param, b.param, r.cfg.resolutionP)
}

// meetsMinimum reports whether a certified interval lower bound proves the
// requested minimum, compared exactly against the minimum as stated.
func (r *motionRun) meetsMinimum(clearance *Measurement) bool {
	if clearance == nil {
		return true
	}
	return proofarith.FloatRat(clearance.Value.Base()).Cmp(r.cfg.minimumMM) >= 0
}

func anyViolated(poses []*motionPose) bool {
	return slices.ContainsFunc(poses, func(p *motionPose) bool { return p.violated })
}

// evaluatePose builds every group's pose at fraction f of the path, published
// as the parameter at, and runs every evaluated pair at it. Every bound is
// built from the exact parameter at f; motionbound.PoseDeviation charges
// whatever separates the pose the driver builds from at and the ideal pose at
// f. Each mover's transient placement is built when its first pair needs it
// and dropped from the carrier cache when the pose is done.
func (r *motionRun) evaluatePose(f *big.Rat, at units.Value) (*motionPose, error) {
	if err := r.ctx.Err(); err != nil {
		return nil, err
	}
	param := r.dom.fromP.Lerp(r.dom.toP, f)
	groups, err := r.drive.posesAt(f, at, param)
	if err != nil {
		return nil, err
	}
	mp := &motionPose{
		f:      f,
		param:  param,
		groups: groups,
		result: PoseResult{
			At:            at,
			Pose:          groups[0].pose,
			Interferences: []Interference{},
			Clearances:    []Clearance{},
			Diagnostics:   []Diagnostic{},
		},
		pairs: make([][]motionPairPose, len(r.movers)),
	}
	for _, mv := range r.movers {
		r.validityFindings(mp, mv.validity)
	}
	for _, st := range r.statics {
		r.validityFindings(mp, st.validity)
	}
	placed := make([]*motionPlaced, len(r.movers))
	defer func() {
		for _, p := range placed {
			if p != nil {
				delete(r.cache.entries, p.body)
			}
		}
	}()
	for i := range r.movers {
		mp.pairs[i] = make([]motionPairPose, len(r.pairs[i]))
		if err := r.evaluateMover(mp, i, placed); err != nil {
			return nil, err
		}
	}
	return mp, nil
}

// validityFindings carries a not-proven-valid body's own validity diagnostic
// into the pose (docs/motion-check-design.md §7), with At set.
func (r *motionRun) validityFindings(mp *motionPose, validity ValidityResult) {
	if validity.Outcome == ValidityValid {
		return
	}
	for _, diag := range validity.Diagnostics {
		diag = withAt(diag, mp.result.At)
		mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
		mp.findings = append(mp.findings, diag)
	}
}

func withAt(diag Diagnostic, at units.Value) Diagnostic {
	diag.At = &at
	return diag
}

// evaluateMover runs mover i's row of pairs at the pose: a sheet pair's
// finding first, then every evaluated pair over the transient placements.
func (r *motionRun) evaluateMover(mp *motionPose, i int, placed []*motionPlaced) error {
	mv := r.movers[i]
	need := false
	for k, pair := range r.pairs[i] {
		if pair.sheet && !pair.declared {
			diag := withAt(pairDiagNone(mv.body, r.partner(i, k), DiagUnsupportedPairSheet,
				"a sheet operand has no clearance the motion check can certify, so this pair is undecided at every pose"), mp.result.At)
			mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
			mp.findings = append(mp.findings, diag)
		}
		need = need || pair.evaluated()
	}
	if !need {
		return nil
	}
	a, err := r.place(mp, i, placed)
	if err != nil {
		return err
	}
	for k, pair := range r.pairs[i] {
		if !pair.evaluated() {
			continue
		}
		var b *motionPlaced
		if pair.other >= 0 {
			if b, err = r.place(mp, pair.other, placed); err != nil {
				return err
			}
		}
		if err := r.evaluatePair(mp, i, k, a, b); err != nil {
			return err
		}
	}
	return nil
}

// place builds mover i's transient placement at the pose once. η is the
// largest motionbound.PoseDeviation against every ideal pose the claim at
// this parameter speaks for, and the swept-volume allowance is charged at the
// largest linear factor among them and the group's stretch base.
func (r *motionRun) place(mp *motionPose, i int, placed []*motionPlaced) (*motionPlaced, error) {
	if placed[i] != nil {
		return placed[i], nil
	}
	mv := r.movers[i]
	group := mp.groups[mv.group]
	placement := mv.body.payload.transform()
	composed, err := placement.Then(group.pose)
	if err != nil {
		return nil, fmt.Errorf(`%w: composing the pose onto a moving body's placement failed: %w`, ErrNotFinite, err)
	}
	transient, err := mv.body.payload.placed(r.ctx, r.transient, transientProducer, composed)
	if err != nil {
		return nil, err
	}
	var eta, linear float64
	for _, ideal := range group.ideals {
		e, l := motionbound.PoseDeviation(composed, placement, ideal, mv.r0)
		eta, linear = math.Max(eta, e), math.Max(linear, l)
	}
	// The volume an overlap can lose between the float pose and the ideal one
	// (§5.1): every boundary point moves at most η along the straight path
	// between the two images, and the area that path carries is the mover's
	// own, scaled by the path's largest linear stretch.
	allowance := proofbound.SweptVolumeAllow(eta, motionbound.PathAreaUpper(mv.area, linear, mv.sigma, group.stretch))
	placed[i] = &motionPlaced{body: transient, eta: eta, allowance: allowance}
	return placed[i], nil
}

// evaluatePair runs Verify's own pair procedure (verify.go) on pair k of
// mover i at one pose, with the gap always asked: box separation, the
// closed-form axis-box gap or the clearance kernel, then the read-only
// overlap proof. a is the mover's transient placement; b is its partner's,
// or nil when the partner is a static body measured as it stands. A moving
// partner adds its own η to the gap's widening and its own allowance to the
// collision transfer (docs/linkage-check-design.md §5.1).
func (r *motionRun) evaluatePair(mp *motionPose, i, k int, a, b *motionPlaced) error {
	mover, partner := r.movers[i].body, r.partner(i, k)
	target, etaB, allowance := partner, 0.0, a.allowance
	if b != nil {
		target, etaB, allowance = b.body, b.eta, proofbound.AbsSumUpper(a.allowance, b.allowance)
	}
	at := mp.result.At
	boxProven := boxesDisjoint(a.body.bounds, target.bounds)
	res, fast := clearanceAxisBoxes(a.body, target)
	if !fast {
		var err error
		res, err = clearancePairCached(r.ctx, a.body, target, boxProven, r.cache)
		if err != nil {
			return err
		}
	}
	if err := r.ctx.Err(); err != nil {
		return err
	}
	if res.verdict == pairDisjoint || res.verdict == pairTouching {
		r.recordGap(mp, i, k, res, a.eta, etaB)
		return nil
	}
	declared := r.pairs[i][k].declared
	if boxProven {
		if !declared {
			r.poseDiag(mp, withAt(pairDiagNone(mover, partner, DiagUndecidedClearance,
				"the pair is proven disjoint at this pose but its gap is unmeasured"), at))
		}
		return nil
	}
	volume, outcome, err := measuredInterference(r.ctx, a.body, target, res, pairMeshes{})
	if err != nil {
		return err
	}
	if outcome != interferenceMeasured && res.verdict != pairOverlapping {
		if declared {
			return nil
		}
		diag := undecidedPairDiag(a.body, target, res.verdict, outcome)
		diag.Pair = &DiagnosticPair{A: mover, B: partner}
		r.poseDiag(mp, withAt(diag, at))
		return nil
	}
	published, ok := transferredOverlap(volume, outcome == interferenceMeasured, allowance)
	if !ok && declared {
		return nil
	}
	if !ok {
		msg := "the pair is proven to overlap at the evaluated float pose, but the overlap volume is unmeasured, so it cannot be carried to the ideal pose and no collision is proven at this parameter"
		if outcome == interferenceMeasured {
			msg = fmt.Sprintf("the pair is proven to overlap at the evaluated float pose, but the measured volume %s does not clear the %v mm^3 the pose's deviation from the ideal motion can sweep, so no collision is proven at this parameter", volume.Value, allowance)
		}
		r.poseDiag(mp, withAt(pairDiagNone(mover, partner, DiagUndecidedInterference, msg), at))
		return nil
	}
	mp.pairs[i][k].collision = true
	obs := published
	mp.collisions = append(mp.collisions, Collision{At: at, Pose: mp.result.Pose, Moving: mover, Static: partner, Volume: published})
	mp.result.Interferences = append(mp.result.Interferences, Interference{A: mover, B: partner, Volume: published})
	msg := fmt.Sprintf("the moving body overlaps a static body at %s", at)
	if b != nil {
		msg = fmt.Sprintf("the moving body overlaps another moving body at %s", at)
	}
	mp.findings = append(mp.findings, withAt(Diagnostic{
		Code:     DiagMotionCollision,
		Status:   Interfering,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  ReadingOverlapVolume,
		Observed: &obs,
		Message:  msg,
	}, at))
	pairD, err := interferencePairDiameter(r.ctx, a.body, target)
	if err != nil {
		return err
	}
	pass, ref, haveRef := interferenceToleranceRef(published, a.body, target, pairD, r.cfg.rel)
	if !pass {
		beyond := Diagnostic{
			Code:     DiagMeasurementBeyondTolerance,
			Status:   Suspect,
			Pair:     &DiagnosticPair{A: mover, B: partner},
			Reading:  ReadingOverlapVolume,
			Observed: &obs,
			Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", published.Bound),
		}
		if haveRef {
			beyond.Required = requiredThreshold(r.cfg.rel*ref, published.Value)
		}
		mp.findings = append(mp.findings, withAt(beyond, at))
	}
	return nil
}

// transferredOverlap is §5.1's collision transfer. An overlap measured at the
// float pose is a collision at the ideal pose only when its proven lower end,
// Value − Bound rounded down, strictly exceeds the volume the mover's boundary
// can sweep between the two poses; the published volume then carries that
// allowance in its Bound, so Value − Bound stays a proven lower bound on the
// ideal overlap. An unmeasured overlap never transfers.
func transferredOverlap(volume Measurement, measured bool, allowance float64) (Measurement, bool) {
	if !measured || proofbound.IsNonFinite(allowance) {
		return Measurement{}, false
	}
	value, bound := proofarith.FloatRat(volume.Value.Base()), proofarith.FloatRat(volume.Bound.Base())
	if value == nil || bound == nil {
		return Measurement{}, false
	}
	lower := proofbound.RatFloatDown(new(big.Rat).Sub(value, bound))
	if !(lower > allowance) {
		return Measurement{}, false
	}
	if allowance == 0 {
		return volume, true
	}
	return Measurement{
		Value:     volume.Value,
		Exactness: Approximate,
		Bound:     units.CubicMillimeters(proofbound.AbsSumUpper(volume.Bound.Base(), allowance)),
	}, true
}

// poseDiag records an undecided or unsupported pair finding both on the pose
// and in the report's flat list.
func (r *motionRun) poseDiag(mp *motionPose, diag Diagnostic) {
	mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
	mp.findings = append(mp.findings, diag)
}

// recordGap charges η into the kernel's proven gap interval and publishes the
// pose's Clearance row. The kernel's interval is about the float placements it
// measured; widening it by both bodies' η on both sides — distance is
// 1-Lipschitz in a displacement of either set — makes it a proven interval on
// the IDEAL poses, which is what the interval certificate consumes. A static
// partner's η is zero. An η that cannot be bounded leaves the gap unmeasured.
func (r *motionRun) recordGap(mp *motionPose, i, k int, res pairResult, etaA, etaB float64) {
	mover, partner := r.movers[i].body, r.partner(i, k)
	at := mp.result.At
	if r.pairs[i][k].declared {
		r.recordDeclaredGap(mp, mover, partner, res, etaA, etaB)
		return
	}
	if proofbound.IsNonFinite(etaA) || proofbound.IsNonFinite(etaB) {
		r.poseDiag(mp, withAt(pairDiagNone(mover, partner, DiagUndecidedClearance,
			"the pair is proven disjoint at this pose, but the pose's departure from the ideal motion is unbounded for this payload, so its gap is unmeasured"), at))
		return
	}
	lo, hi, exact := clearanceDeltaWiden(res.lo, res.hi, res.exact, etaA, etaB)
	mp.pairs[i][k] = motionPairPose{hasGap: true, lo: lo, hi: hi, diam: res.diam}
	gap := pairGapMeasurement(pairResult{lo: lo, hi: hi, exact: exact})
	mp.result.Clearances = append(mp.result.Clearances, Clearance{A: mover, B: partner, Gap: gap})
	if r.cfg.minimumMM != nil && proofarith.FloatRat(hi).Cmp(r.cfg.minimumMM) < 0 {
		// The proven upper end of the ideal pose's gap lies below the spec:
		// the margin is disproven here, whatever the reading's precision.
		mp.violated = true
		violation := gap
		mp.findings = append(mp.findings, withAt(Diagnostic{
			Code:     DiagMotionClearanceViolated,
			Status:   Violating,
			Pair:     &DiagnosticPair{A: mover, B: partner},
			Reading:  ReadingGap,
			Observed: &violation,
			Required: r.cfg.minimum,
			Message:  fmt.Sprintf("the gap at %s is proven below the required minimum %s", at, *r.cfg.minimum),
		}, at))
	}
	pass, ref, haveRef := scalarToleranceRef(gap, r.cfg.rel, pairToleranceInputs{diameter: res.diam}.lengthReference)
	if pass {
		return
	}
	obs := gap
	beyond := Diagnostic{
		Code:     DiagMeasurementBeyondTolerance,
		Status:   Suspect,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  ReadingGap,
		Observed: &obs,
		Message:  fmt.Sprintf("the gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	if haveRef {
		beyond.Required = requiredThreshold(r.cfg.rel*ref, gap.Value)
	}
	mp.findings = append(mp.findings, withAt(beyond, at))
}

// recordDeclaredGap publishes a declared joint contact's measured gap as a
// Clearance row and nothing else (docs/linkage-check-design.md §5.4): a
// touching pair or an unbounded η publishes nothing, no finding is raised,
// and the gap feeds neither an interval certificate nor the whole-path
// reading.
func (r *motionRun) recordDeclaredGap(mp *motionPose, mover, partner *Body, res pairResult, etaA, etaB float64) {
	if res.verdict != pairDisjoint || proofbound.IsNonFinite(etaA) || proofbound.IsNonFinite(etaB) {
		return
	}
	lo, hi, exact := clearanceDeltaWiden(res.lo, res.hi, res.exact, etaA, etaB)
	gap := pairGapMeasurement(pairResult{lo: lo, hi: hi, exact: exact})
	mp.result.Clearances = append(mp.result.Clearances, Clearance{A: mover, B: partner, Gap: gap})
}

// intervalVerdict decides one interval between adjacent poses a and b
// (docs/motion-check-design.md §5.2). A proven collision at either end makes
// it IntervalColliding. Otherwise it is IntervalClear only when EVERY pair
// certifies: a swept-box-excluded pair by its whole-path lower bound, an
// evaluated pair by lo_a + lo_b > τ — taken over exact rationals, so no
// rounding sits between the proven terms and the strict comparison. The lower
// envelope's minimum over the interval, (lo_a + lo_b − τ)/2, is rounded down
// to the float the Clearance publishes.
func (r *motionRun) intervalVerdict(a, b *motionPose) motionSpan {
	outcome, clearance := r.intervalOutcome(a, b)
	return motionSpan{outcome: outcome, clearance: clearance}
}

func (r *motionRun) intervalOutcome(a, b *motionPose) (IntervalOutcome, *Measurement) {
	for i := range r.movers {
		for k := range r.pairs[i] {
			if a.pairs[i][k].collision || b.pairs[i][k].collision {
				return IntervalColliding, nil
			}
		}
	}
	var lowest *big.Rat
	for i := range r.movers {
		for k, pair := range r.pairs[i] {
			if pair.declared {
				continue
			}
			if pair.excluded {
				lowest = minRat(lowest, proofarith.FloatRat(pair.lower))
				continue
			}
			pa, pb := a.pairs[i][k], b.pairs[i][k]
			if !pair.evaluated() || !pa.hasGap || !pb.hasGap {
				return IntervalUndecided, nil
			}
			tau := r.drive.travel(i, k, a.param, b.param)
			if tau == nil {
				return IntervalUndecided, nil
			}
			sum := new(big.Rat).Add(proofarith.FloatRat(pa.lo), proofarith.FloatRat(pb.lo))
			if sum.Cmp(tau) <= 0 {
				return IntervalUndecided, nil
			}
			envelope := sum.Sub(sum, tau)
			lowest = minRat(lowest, envelope.Quo(envelope, big.NewRat(2, 1)))
		}
	}
	if lowest == nil {
		return IntervalClear, nil
	}
	return IntervalClear, &Measurement{
		Value:     units.Millimeters(proofbound.RatFloatDown(lowest)),
		Exactness: Approximate,
		Bound:     units.Millimeters(0),
	}
}

// minRat is the smaller of an optional running minimum and a candidate.
func minRat(running, candidate *big.Rat) *big.Rat {
	if running == nil || candidate.Cmp(running) < 0 {
		return candidate
	}
	return running
}

// motionConclusion is the part of a report every check publishes the same
// way (docs/motion-check-design.md §4, docs/linkage-check-design.md §4).
type motionConclusion struct {
	request     MotionRequest
	against     []*Body
	intervals   []MotionInterval
	clearance   *ScalarReading
	assessment  Assessment
	diagnostics []Diagnostic // interval findings, then pose findings, then the path reading's
	status      Status
}

// conclude assembles the intervals, the whole-path reading, the assessment,
// the diagnostics and the status from the evaluated poses and intervals.
func (r *motionRun) conclude(poses []*motionPose, spans []motionSpan) motionConclusion {
	c := motionConclusion{
		request: MotionRequest{
			RelativeTolerance: units.Scalar(r.cfg.rel),
			Resolution:        r.cfg.resolution,
			MinClearance:      r.cfg.minimum,
		},
		against:     []*Body{},
		diagnostics: []Diagnostic{},
	}
	for _, st := range r.statics {
		c.against = append(c.against, st.body)
	}

	violated := anyViolated(poses)
	allClear, met := true, true
	var lowest *Measurement
	for k, span := range spans {
		a, b := poses[k], poses[k+1]
		interval := MotionInterval{From: a.result.At, To: b.result.At, Outcome: span.outcome, Clearance: span.clearance}
		c.intervals = append(c.intervals, interval)
		if span.outcome != IntervalClear {
			allClear, met = false, false
		}
		if span.clearance != nil && (lowest == nil || span.clearance.Value.Base() < lowest.Value.Base()) {
			lowest = span.clearance
		}
		switch {
		case span.outcome == IntervalUndecided:
			c.diagnostics = append(c.diagnostics, withAt(Diagnostic{
				Code:    DiagMotionUndecidedInterval,
				Status:  Suspect,
				Reading: ReadingNone,
				Message: fmt.Sprintf("the motion from %s to %s is neither certified clear nor bounded by a proven collision", a.result.At, b.result.At),
			}, a.result.At))
		case span.outcome == IntervalClear && r.cfg.minimumMM != nil && !r.meetsMinimum(span.clearance):
			met = false
			if violated {
				break
			}
			obs := *span.clearance
			c.diagnostics = append(c.diagnostics, withAt(Diagnostic{
				Code:     DiagMotionUndecidedClearance,
				Status:   Suspect,
				Reading:  ReadingGap,
				Observed: &obs,
				Required: r.cfg.minimum,
				Message:  fmt.Sprintf("the motion from %s to %s is certified clear, but its proven lower bound does not reach the required minimum", a.result.At, b.result.At),
			}, a.result.At))
		}
	}
	switch {
	case r.cfg.minimumMM == nil:
		c.assessment = AssessmentNotEvaluated
	case violated:
		c.assessment = AssessmentViolated
	case met:
		c.assessment = AssessmentMet
	default:
		c.assessment = AssessmentUndecided
	}
	for _, pose := range poses {
		c.diagnostics = append(c.diagnostics, pose.findings...)
	}
	if allClear && lowest != nil {
		if reading, diag := r.pathClearance(poses, lowest); reading != nil {
			c.clearance = reading
			if diag != nil {
				c.diagnostics = append(c.diagnostics, *diag)
			}
		}
	}
	c.status = Sound
	for _, diag := range c.diagnostics {
		c.status = max(c.status, diag.Status)
	}
	return c
}

// publish assembles VerifyMotion's report (docs/motion-check-design.md §4).
func (r *motionRun) publish(poses []*motionPose, spans []motionSpan) *MotionReport {
	c := r.conclude(poses, spans)
	report := &MotionReport{
		Request:     c.request,
		Motion:      r.spec.motion,
		Against:     c.against,
		Intervals:   c.intervals,
		Collisions:  []Collision{},
		Clearance:   c.clearance,
		Assessment:  c.assessment,
		Diagnostics: c.diagnostics,
		Status:      c.status,
	}
	for _, mv := range r.movers {
		report.Moving = append(report.Moving, mv.body)
	}
	for _, pose := range poses {
		report.Poses = append(report.Poses, pose.result)
		report.Collisions = append(report.Collisions, pose.collisions...)
	}
	return report
}

// pathClearance is docs/motion-check-design.md §5.3's whole-path reading: the
// minimum gap over the path lies between the smallest interval lower bound
// and the smallest evaluated upper bound, since any pose's upper bound bounds
// the path's minimum from above. It reports that interval's midpoint and
// half-width, never Exact, judged against the diameter of the pair that
// attained the upper end. A path whose pairs were all settled by swept-box
// exclusion has no upper bound and carries no reading.
func (r *motionRun) pathClearance(poses []*motionPose, lowest *Measurement) (*ScalarReading, *Diagnostic) {
	upper := math.Inf(1)
	diam := 0.0
	for _, pose := range poses {
		for _, row := range pose.pairs {
			for _, pp := range row {
				if pp.hasGap && pp.hi < upper {
					upper, diam = pp.hi, pp.diam
				}
			}
		}
	}
	if math.IsInf(upper, 1) {
		return nil, nil
	}
	// Lowering a proven lower bound keeps it one, so a lower end above the
	// upper end — which sound bounds never produce — is clamped rather than
	// published inverted.
	lower := math.Min(lowest.Value.Base(), upper)
	gap := pairGapMeasurement(pairResult{lo: lower, hi: upper})
	gap.Exactness = Approximate
	pass, ref, haveRef := scalarToleranceRef(gap, r.cfg.rel, pairToleranceInputs{diameter: diam}.lengthReference)
	reading := &ScalarReading{Measurement: gap, Tolerance: judgeTolerance(pass, haveRef, r.cfg.rel, ref, gap.Value)}
	if pass {
		return reading, nil
	}
	obs := gap
	diag := &Diagnostic{
		Code:     toleranceDiagnostic(reading.Tolerance),
		Status:   Suspect,
		Reading:  ReadingGap,
		Observed: &obs,
		Required: reading.Tolerance.Limit,
		Message:  fmt.Sprintf("the whole-path gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	return reading, diag
}
