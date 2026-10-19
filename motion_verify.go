package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/motionrun"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyMotion (docs/motion-check-design.md §3, §5-§8)
// and the engine it shares with Document.VerifyLinkage
// (docs/linkage-check-design.md §6): validation, the swept-box exclusion, the
// per-pose pair proof over transient placements, the two-sided interval
// certificate, and the report assembly. motion.go owns the vocabulary and
// internal/motionbound and internal/linkagebound the bounds; linkage_verify.go
// drives the same engine over a chain of joints.
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
		if b.n.Payload == nil {
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
	encodedOpts, err := decodeMotionOptions(opts)
	if err != nil {
		return nil, err
	}
	cfg, err := motionoption.Resolve(encodedOpts, spec.Domain)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	run := &motionRun{ctx: ctx, d: d, spec: spec, cfg: cfg, cache: &pairkernel.Cache{}}
	run.setup(moving)
	poses, spans, err := run.refine()
	if err != nil {
		return nil, err
	}
	report := run.publish(poses, spans)
	report.Motion = m
	return report, nil
}

// motionRun is one call's working state, for VerifyMotion and VerifyLinkage
// alike. It lives in the call alone, never on the Document.
type motionRun struct {
	ctx       context.Context //nolint:containedctx // motionRun is per-call state and never outlives its call.
	d         *Document
	transient *Document // per-call identity counters for uncommitted pose bodies
	dom       motionbound.Domain
	cfg       motionConfig
	cache     *pairkernel.Cache
	drive     motionDriver
	movers    []motionMover  // every moving body, group by group
	statics   []motionStatic // every other live body, in Document.Bodies() order
	// pairs is, per mover, its row of pairs in pair order: every static body
	// in order, then every body of every later group in order.
	pairs [][]motionPair
	// declared names the pairs a linkage declares as joint contacts, each in
	// both orders; nil for VerifyMotion.
	declared map[[2]*Body]struct{}
	// constPlaced holds, per mover of a constant group, the transient
	// placement built at its first pose and reused at every later one.
	constPlaced []*motionPlaced
	// poseBodies is enabled only by VerifyJointBox. Its cells can visit the
	// same float placement repeatedly while another joint's range is split.
	poseBodies []poseBodyCache

	// VerifyMotion's own motion. spec is the validated Motion; stretch and
	// stretchEnd are motionbound.PathAreaUpper's stretch base (§5.1) for a
	// pose before the end and for the end itself: exactly 1 for a Revolute
	// and a Prismatic; motionbound.BasisSigmaUpper of From, and at s = 1 the
	// larger of From's and To's, for a Between.
	spec                motionbound.Spec
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
	// projection is a second proven lower bound, as an exact rational, on
	// the distance between the two bodies of pair k of mover i at every
	// parameter between the poses a and b, which certifies the pair alone
	// when it is positive (docs/linkage-check-design.md §5.8); nil when the
	// driver supplies none.
	projection(i, k int, a, b *motionPose) *big.Rat
}

// motionGroupPose is one moving group's pose at one parameter: the float
// transform composed onto each of its bodies' own placements, the ideal poses
// η is charged against, PathAreaUpper's stretch base, and, for a linkage, the
// joint value of the group's link.
//
// fixed marks a group whose ideal pose is the identity at every parameter:
// its bodies are measured as they stand, with no transient placement and no
// η. constant marks a group whose pose is the same at every parameter: its
// transient placements are built once and reused
// (docs/linkage-check-design.md §6 step 2).
type motionGroupPose struct {
	pose     r3.Transform
	ideals   []motionbound.IdealPose
	stretch  float64
	value    units.Value
	bound    units.Value // value's proven half-width; zero for a stated joint
	fixed    bool
	constant bool
}

// motionIntervalGate is a driver that can refuse an interval before any pair
// is read: a linkage drive whose loop cannot be enclosed over it
// (docs/linkage-check-design.md §15.6). intervalGate returns the refusal's
// message, empty when the interval may be certified; a refused interval is
// IntervalUndecided whatever its pairs prove.
type motionIntervalGate interface {
	intervalGate(a, b *motionPose) string
}

// errPoseUnbuildable is what a driver's posesAt returns, wrapped, for a pose
// it cannot build because a loop cannot be enclosed there
// (docs/linkage-check-design.md §15.6): the pose is not evaluated, and every
// interval ending at it is IntervalUndecided.
type errPoseUnbuildable interface {
	error
	unbuildable()
}

type motionMover struct {
	body       *Body
	group      int
	validity   ValidityResult
	rho        float64 // ρ_max under VerifyMotion's revolute or between motion
	r0         float64 // the record radius motionbound.PoseDeviation charges at
	heldRadius float64 // the faceted mover's current true-solid radius
	area       float64 // a proven upper bound on the mover's surface area at rest
	sigma      float64 // a proven lower bound on its placement's smallest singular value
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
	// unformed: the pair is not a question this check asks — a held link's
	// body against a static body, or two held links' bodies, whose relation
	// is Verify's (docs/linkage-check-design.md §6 step 2). It is never
	// evaluated, raises nothing, and enters no interval.
	unformed bool
	// declared: a linkage's declared joint contact
	// (docs/linkage-check-design.md §5.4). It is evaluated at every pose and
	// publishes a transferred collision and a measured gap row, but no
	// undecided, touching or tolerance finding, and it enters no interval
	// certificate and no whole-path reading.
	declared bool
	// constant: the pair's relation is the same at every configuration
	// (docs/linkage-check-design.md §5.9) — its relative path is one revolute
	// joint and one of its bodies is symmetric about that joint's axis. Its
	// first evaluation reads the two bodies as they stand; once holds the
	// outcome, which every pose replays with no placement, and the pair
	// travels nothing over an interval or a cell. A constant pair whose one
	// evaluation is undecided, touching or unmeasured is evaluated at every
	// pose like any other, with constant cleared.
	constant bool
	once     *constantOutcome
}

// constantOutcome is a constant pair's one evaluation on the bodies as they
// stand, where δ and η are zero: a measured gap, or a proven overlap with its
// volume measured once and its tolerance finding, nil when it passes.
type constantOutcome struct {
	res       pairkernel.Result
	collision bool
	volume    Measurement
	beyond    *Diagnostic
}

func (p motionPair) evaluated() bool { return !p.excluded && !p.invalid && !p.sheet && !p.unformed }

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
//
// A path's pose sits at the fraction f, whose exact parameter is param, and
// its findings carry At. A joint box's centre has neither: cell is the cell
// whose centre it is, and its findings carry Cell instead
// (docs/linkage-check-design.md §14.2). where is what a finding's message
// names the pose by: the parameter on a path, the configuration in a box.
type motionPose struct {
	f          *big.Rat
	param      motionbound.MotionParam
	cell       *JointCell
	where      any
	groups     []motionGroupPose
	violated   bool // some pair's gap here is proven below the requested minimum
	result     PoseResult
	pairs      [][]motionPairPose
	findings   []Diagnostic // this pose's findings, in report order
	collisions []Collision
	// unbuildable is why the driver could not build the pose; nil for a pose
	// it built. An unbuildable pose evaluates no pair.
	unbuildable error
}

// motionPlaced is one mover's transient placement at one pose, with the pose
// deviation η it carries and the swept-volume allowance a collision on it
// must clear.
type motionPlaced struct {
	body      *Body
	eta       float64
	allowance float64
	// kept: the placement outlives the pose — a fixed group's own body, or a
	// constant group's reused transient — so its carriers stay cached.
	kept bool
}

// setup reads VerifyMotion's one moving group: ρ_max for a revolute or a
// between, the stretch bases, and the swept box each mover's exclusion
// compares the static bodies against.
func (r *motionRun) setup(moving []*Body) {
	r.dom = r.spec.Domain
	r.drive = &singleMotion{r: r}
	r.readBodies([][]*Body{moving})
	r.stretch, r.stretchEnd = 1, 1
	if r.spec.Kind == motionbound.MotionBetween {
		r.stretch = motionbound.BasisSigmaUpper(r.spec.Between.From)
		r.stretchEnd = math.Max(r.stretch, motionbound.BasisSigmaUpper(r.spec.Between.To))
	}
	zero := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	swept := make([]motionSweptBox, len(r.movers))
	for i := range r.movers {
		mv := &r.movers[i]
		if r.spec.Kind != motionbound.MotionPrismatic {
			mv.rho = motionbound.MoverAxisRadius(mv.body.n.Bounds, r.spec.Frame)
		}
		// The swept box covers every pose from where the path's box was read:
		// for a Revolute or a Prismatic that is the mover at rest — the
		// parameter 0 — so its travel runs from 0 to the farther endpoint,
		// never merely across [From, To]; for a Between it is the From-placed
		// box at s = 0, From itself, so the travel is the whole path's.
		travel := maxRat(
			motionbound.MoverTravel(r.spec.Frame, mv.rho, zero, r.spec.FromP),
			motionbound.MoverTravel(r.spec.Frame, mv.rho, zero, r.spec.ToP),
		)
		lo, hi, ok := motionbound.MoverSweptBox(mv.body.n.Bounds, r.spec.Frame, travel)
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
	r.transient = bornDocument(&Document{
		n: topo.Document{
			NextLevel:    r.d.n.NextLevel,
			NextCurve:    r.d.n.NextCurve,
			NextProducer: r.d.n.NextProducer,
			LiveSet:      bodyNodes(live),
		},
	})
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
		mv.heldRadius = math.Inf(1)
		if fp, ok := mv.body.n.Payload.(facetedPayload); ok {
			mv.heldRadius = motionbound.HeldRadiusUpper(fp.Verts, fp.MeshBound)
		}
		mv.area = proofbound.AbsSumUpper(mv.body.n.Area.Value.Base(), mv.body.n.Area.Bound.Base())
		mv.sigma = motionbound.BasisSigmaLower(mv.body.n.Payload.Transform())
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
			state, lower := motionrun.ClassifyStaticPair(&mv.body.n, &st.body.n,
				mv.validity.Outcome == ValidityValid, st.validity.Outcome == ValidityValid,
				swept[i].lo, swept[i].hi, swept[i].ok)
			pair := motionPair{other: -1, declared: r.isDeclared(mv.body, st.body), lower: lower}
			switch state {
			case motionrun.StaticPairInvalid:
				pair.invalid = true
			case motionrun.StaticPairExcluded:
				pair.excluded = true
			case motionrun.StaticPairSheet:
				pair.sheet = true
			}
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

// partner is the caller's own body pair k of mover i names beside the mover.
func (r *motionRun) partner(i, k int) *Body {
	if o := r.pairs[i][k].other; o >= 0 {
		return r.movers[o].body
	}
	return r.statics[k].body
}

// singleMotion drives VerifyMotion's one group along its Motion. points
// keeps each mover's projection reading at each pose, read once.
type singleMotion struct {
	r      *motionRun
	points map[motionPointsKey]cornerBounds
}

// motionPointsKey names one mover's projection reading at one pose: its box
// corners, or its hull points.
type motionPointsKey struct {
	pose  *motionPose
	mover int
	hull  bool
}

func (m *singleMotion) posesAt(f *big.Rat, at units.Value, param motionbound.MotionParam) ([]motionGroupPose, error) {
	r := m.r
	pose, err := r.spec.Motion.PoseAt(at)
	if err != nil {
		return nil, err
	}
	ideals := []motionbound.IdealPose{r.spec.Frame.At(param)}
	stretch := r.stretch
	if r.spec.Kind == motionbound.MotionBetween && f.Cmp(big.NewRat(1, 1)) == 0 {
		// PoseAt(1) returns the stated To, which the exact screw of the read
		// parameters rebuilds only to rounding: the pose is charged against
		// both the ideal end and To itself (§5.1, η_1 = max(η_ideal, η_To)).
		ideals = append(ideals, r.spec.Frame.StatedEnd())
		stretch = r.stretchEnd
	}
	return []motionGroupPose{{pose: pose, ideals: ideals, stretch: stretch}}, nil
}

// travel is τ of docs/motion-check-design.md §5.2 for mover i: every pair of
// one group's mover has a static partner, which does not move.
func (m *singleMotion) travel(i, _ int, a, b motionbound.MotionParam) *big.Rat {
	return motionbound.MoverTravel(m.r.spec.Frame, m.r.movers[i].rho, a, b)
}

// projection is the projection bound of docs/linkage-check-design.md §5.8
// for pair k of mover i over the interval between poses a and b, the motion
// read as one joint whose value is the motion's parameter
// (docs/motion-check-design.md §5.2): the mover's points — its box corners,
// then its hull points — posed at each end and expanded to second order in
// the parameter along the interval's own step, the static partner's points
// as they stand. The parameter is affine in the fraction over the whole path,
// so the segment term always serves. It is the largest bound any end and
// reading gives, nil when none can be read.
func (m *singleMotion) projection(i, k int, a, b *motionPose) *big.Rat {
	r := m.r
	if r.pairs[i][k].other >= 0 {
		// Unreachable: VerifyMotion moves one group, whose movers pair only
		// with static bodies.
		return nil
	}
	return linkagebound.MotionProjection(r.spec.Frame, r.movers[i].rho, a.param, b.param,
		func(end int, hull bool) (cornerBounds, bool) {
			if end == 0 {
				return m.moverPoints(a, i, hull)
			}
			return m.moverPoints(b, i, hull)
		},
		func(hull bool) (cornerBounds, bool) { return m.staticPoints(k, hull) })
}

// moverPoints is mover i's projection reading at a pose: its points posed by
// the motion's ideal pose there, each with its velocity per unit of the
// parameter — k × (x − c) per radian for a Revolute, the unit direction per
// millimetre for a Prismatic, θ·k × (x − c) + d·k per unit fraction for a
// Between — rounded outward, read once and kept.
func (m *singleMotion) moverPoints(pose *motionPose, i int, hull bool) (cornerBounds, bool) {
	key := motionPointsKey{pose: pose, mover: i, hull: hull}
	if got, ok := m.points[key]; ok {
		return got, true
	}
	points, ok := bodyPoints(m.r.movers[i].body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := linkagebound.MotionMoverPoints(m.r.spec.Frame, pose.param, points)
	if !ok {
		return cornerBounds{}, false
	}
	if m.points == nil {
		m.points = make(map[motionPointsKey]cornerBounds)
	}
	m.points[key] = reading
	return reading, true
}

// staticPoints is static body k's projection reading: its box corners, or
// its hull points.
func (m *singleMotion) staticPoints(k int, hull bool) (cornerBounds, bool) {
	key := motionPointsKey{mover: -1 - k, hull: hull}
	if got, ok := m.points[key]; ok {
		return got, true
	}
	points, ok := bodyPoints(m.r.statics[k].body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := linkagebound.RoundCorners(points)
	if !ok {
		return cornerBounds{}, false
	}
	if m.points == nil {
		m.points = make(map[motionPointsKey]cornerBounds)
	}
	m.points[key] = reading
	return reading, true
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
	note      string // why a driver refused the interval, appended to its finding
}

// refine evaluates the two endpoints and bisects (§6), returning the poses in
// parameter order and the intervals between them.
func (r *motionRun) refine() ([]*motionPose, []motionSpan, error) {
	var poses []*motionPose
	for _, end := range []struct {
		f  *big.Rat
		at units.Value
	}{{new(big.Rat), r.dom.From}, {big.NewRat(1, 1), r.dom.To}} {
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
		mid, err := r.evaluatePose(f, r.dom.Label(f))
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
// and the interval is wider than the reading's floor, or while a requested
// margin is neither proven nor disproven by that interval and it is wider than
// the resolution. Every interval narrower than both floors stops, so the loop
// ends.
func (r *motionRun) nextRefinement(poses []*motionPose, spans []motionSpan) int {
	allClear := true
	smallest := -1
	for k, span := range spans {
		if poses[k].unbuildable != nil && poses[k+1].unbuildable != nil {
			// Never bisected: no pose between two unbuildable ones is asked.
			allClear = false
			continue
		}
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
	if smallest < 0 {
		return -1
	}
	a, b := poses[smallest], poses[smallest+1]
	if allClear && r.wideForReading(a, b) {
		if reading, _ := r.pathClearance(poses, spans[smallest].clearance, "whole-path"); reading != nil && reading.Tolerance.State != ToleranceSatisfied {
			return smallest
		}
	}
	if r.cfg.MinimumMM != nil && r.wide(a, b) && !anyViolated(poses) && !r.meetsMinimum(spans[smallest].clearance) {
		return smallest
	}
	return -1
}

// wideForReading reports whether the interval between two poses is wider
// than the reading's own floor: cfg.ReadingP when the check sets one, and
// the resolution otherwise.
func (r *motionRun) wideForReading(a, b *motionPose) bool {
	if r.cfg.ReadingP == nil {
		return r.wide(a, b)
	}
	return motionbound.ExceedsResolution(a.param, b.param, *r.cfg.ReadingP)
}

// wide reports whether the interval between two poses is wider than the
// resolution.
func (r *motionRun) wide(a, b *motionPose) bool {
	return motionbound.ExceedsResolution(a.param, b.param, r.cfg.ResolutionP)
}

// meetsMinimum reports whether a certified interval lower bound proves the
// requested minimum, compared exactly against the minimum as stated.
func (r *motionRun) meetsMinimum(clearance *Measurement) bool {
	if clearance == nil {
		return true
	}
	return proofarith.FloatRat(clearance.Value.Base()).Cmp(r.cfg.MinimumMM) >= 0
}

func anyViolated(poses []*motionPose) bool {
	return slices.ContainsFunc(poses, func(p *motionPose) bool { return p.violated })
}

// evaluatePose builds every group's pose at fraction f of the path, published
// as the parameter at, and runs every evaluated pair at it. Every bound is
// built from the exact parameter at f; motionbound.PoseDeviation charges
// whatever separates the pose the driver builds from at and the ideal pose at
// f.
func (r *motionRun) evaluatePose(f *big.Rat, at units.Value) (*motionPose, error) {
	if err := r.ctx.Err(); err != nil {
		return nil, err
	}
	param := r.dom.FromP.Lerp(r.dom.ToP, f)
	groups, err := r.drive.posesAt(f, at, param)
	if err != nil {
		var ub errPoseUnbuildable
		if !errors.As(err, &ub) {
			return nil, err
		}
		return &motionPose{f: f, param: param, where: at, unbuildable: err,
			result: PoseResult{At: at}, pairs: make([][]motionPairPose, len(r.movers))}, nil
	}
	mp := r.newPose(groups, at, at)
	mp.f, mp.param = f, param
	if err := r.runPairs(mp); err != nil {
		return nil, err
	}
	return mp, nil
}

// newPose is an empty pose over groups, published as the parameter at and
// named in finding messages by where.
func (r *motionRun) newPose(groups []motionGroupPose, at units.Value, where any) *motionPose {
	return &motionPose{
		where:  where,
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
}

// runPairs runs every evaluated pair at a built pose: every body's validity
// finding first, then each mover's row of pairs in pair order. Each mover's
// transient placement is built when its first pair needs it and dropped from
// the carrier cache when the pose is done.
func (r *motionRun) runPairs(mp *motionPose) error {
	for _, mv := range r.movers {
		r.validityFindings(mp, mv.validity)
	}
	for _, st := range r.statics {
		r.validityFindings(mp, st.validity)
	}
	placed := make([]*motionPlaced, len(r.movers))
	defer func() {
		for _, p := range placed {
			if p != nil && !p.kept {
				r.cache.Evict(&p.body.n)
			}
		}
	}()
	for i := range r.movers {
		mp.pairs[i] = make([]motionPairPose, len(r.pairs[i]))
		if err := r.evaluateMover(mp, i, placed); err != nil {
			return err
		}
	}
	return nil
}

// validityFindings carries a not-proven-valid body's own validity diagnostic
// into the pose (docs/motion-check-design.md §7), stamped with where it was
// found.
func (r *motionRun) validityFindings(mp *motionPose, validity ValidityResult) {
	if validity.Outcome == ValidityValid {
		return
	}
	for _, diag := range validity.Diagnostics {
		diag = mp.stamp(diag)
		mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
		mp.findings = append(mp.findings, diag)
	}
}

func withAt(diag Diagnostic, at units.Value) Diagnostic {
	diag.At = &at
	return diag
}

// stamp marks a finding at this pose with where it was found: the cell whose
// centre the pose is, or else the parameter.
func (mp *motionPose) stamp(diag Diagnostic) Diagnostic {
	if mp.cell == nil {
		return withAt(diag, mp.result.At)
	}
	cell := cloneJointCell(*mp.cell)
	diag.Cell = &cell
	return diag
}
