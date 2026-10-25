package motionengine

import (
	"context"
	"errors"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/verifyrun"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionrun"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyMotion (agent-docs/motion-check-design.md §3, §5-§8)
// and the engine it shares with Document.VerifyLinkage
// (agent-docs/linkage-check-design.md §6): validation, the swept-box exclusion, the
// per-pose pair proof over transient placements, the two-sided interval
// certificate, and the report assembly. motion.go owns the vocabulary and
// internal/motionbound and internal/linkagebound the bounds; linkage/verify.go
// drives the same engine over a chain of joints.
//
// The engine moves one or more GROUPS of bodies, each group rigid under one
// pose per parameter, against every other live body of the document. A
// VerifyMotion call is one group under one Motion; a VerifyLinkage call is one
// group per link. A pair is a moving body and a static body, or two moving
// bodies of different groups; two bodies of one group form no pair, since a
// shared rigid motion preserves their relation. The driver (Driver)
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
// leaves uncertified reads vocab.IntervalUndecided and makes the report Suspect;
// nothing between evaluated poses is ever assumed clear.

// TransientProducer is the reserved producer identity every transient pose
// placement is built under. It never enters the document: a transient body is
// never committed, never published, and its provenance is never read back.
const TransientProducer provenance.Producer = -1

// Run is one call's working state, for VerifyMotion and VerifyLinkage
// alike. It lives in the call alone, never on the Document.
type Run struct {
	Ctx       context.Context //nolint:containedctx // Run is per-call state and never outlives its call.
	live      []*topo.Body    // every live body of the document, in document order
	doc       *topo.Document
	hooks     Hooks
	transient *topo.Document // per-call identity counters for uncommitted pose bodies
	Dom       motionbound.Domain
	Cfg       Config
	Cache     *pairkernel.Cache
	Drive     Driver
	Movers    []Mover  // every moving body, group by group
	Statics   []Static // every other live body, in Document.Bodies() order
	// pairs is, per mover, its row of pairs in pair order: every static body
	// in order, then every body of every later group in order.
	Pairs [][]Pair
	// declared names the pairs a linkage declares as joint contacts, each in
	// both orders; nil for VerifyMotion.
	Declared map[[2]*topo.Body]struct{}
	// constPlaced holds, per mover of a constant group, the transient
	// placement built at its first pose and reused at every later one.
	constPlaced []*Placed
	// poseBodies is enabled only by VerifyJointBox. Its cells can visit the
	// same float placement repeatedly while another joint's range is split.
	PoseBodies []PoseBodyCache

	// VerifyMotion's own motion. Spec is the validated Motion; stretch and
	// stretchEnd are motionbound.PathAreaUpper's stretch base (§5.1) for a
	// pose before the end and for the end itself: exactly 1 for a Revolute
	// and a Prismatic; motionbound.BasisSigmaUpper of From, and at s = 1 the
	// larger of From's and To's, for a Between.
	Spec                motionbound.Spec
	Stretch, StretchEnd float64
}

// Driver is what a check's moving groups answer for: their poses at a
// parameter and a pair's travel bound across an interval.
type Driver interface {
	// posesAt builds every group's pose at fraction f of the path, published
	// as the parameter at, whose exact denotation is param.
	PosesAt(f *big.Rat, at units.Value, param motionbound.MotionParam) ([]GroupPose, error)
	// travel is a proven upper bound, as an exact rational, on how much the
	// distance between the two bodies of pair k of mover i can change while
	// the parameter runs from a to b; nil when no bound exists.
	Travel(i, k int, a, b motionbound.MotionParam) *big.Rat
	// projection is a second proven lower bound, as an exact rational, on
	// the distance between the two bodies of pair k of mover i at every
	// parameter between the poses a and b, which certifies the pair alone
	// when it is positive (agent-docs/linkage-check-design.md §5.8); nil when the
	// driver supplies none.
	Projection(i, k int, a, b *Pose) *big.Rat
}

// GroupPose is one moving group's pose at one parameter: the float
// transform composed onto each of its bodies' own placements, the ideal poses
// η is charged against, PathAreaUpper's stretch base, and, for a linkage, the
// joint value of the group's link.
//
// fixed marks a group whose ideal pose is the identity at every parameter:
// its bodies are measured as they stand, with no transient placement and no
// η. constant marks a group whose pose is the same at every parameter: its
// transient placements are built once and reused
// (agent-docs/linkage-check-design.md §6 step 2).
type GroupPose struct {
	Pose     r3.Transform
	Ideals   []motionbound.IdealPose
	Stretch  float64
	Value    units.Value
	Bound    units.Value // value's proven half-width; zero for a stated joint
	Fixed    bool
	Constant bool
}

// IntervalGate is a driver that can refuse an interval before any pair
// is read: a linkage drive whose loop cannot be enclosed over it
// (agent-docs/linkage-check-design.md §15.6). intervalGate returns the refusal's
// message, empty when the interval may be certified; a refused interval is
// vocab.IntervalUndecided whatever its pairs prove.
type IntervalGate interface {
	IntervalGate(a, b *Pose) string
}

// PoseUnbuildable is what a driver's posesAt returns, wrapped, for a pose
// it cannot build because a loop cannot be enclosed there
// (agent-docs/linkage-check-design.md §15.6): the pose is not evaluated, and every
// interval ending at it is vocab.IntervalUndecided.
type PoseUnbuildable interface {
	error
	Unbuildable()
}

type Mover struct {
	Body       *topo.Body
	Group      int
	Validity   ValidityResult
	Rho        float64 // ρ_max under VerifyMotion's revolute or between motion
	R0         float64 // the record radius motionbound.PoseDeviation charges at
	HeldRadius float64 // the faceted mover's current true-solid radius
	Area       float64 // a proven upper bound on the mover's surface area at rest
	Sigma      float64 // a proven lower bound on its placement's smallest singular value
}

type Static struct {
	Body     *topo.Body
	Validity ValidityResult
}

// Pair is one pair's standing for the whole call.
type Pair struct {
	// other is the partner mover's index, or −1 when the partner is the
	// static body at the pair's own position in its row.
	Other int
	// Excluded: the swept-box exclusion proved the pair apart over the whole
	// path, with lower a proven lower bound on its distance throughout.
	Excluded bool
	Lower    float64
	// Invalid: an operand is not a proven-valid body; the pair is never
	// evaluated and leaves every interval undecided.
	Invalid bool
	// Sheet: an operand is a Sheet; the pair reads DiagUnsupportedPairSheet
	// at every pose and leaves every interval undecided.
	Sheet bool
	// Unformed: the pair is not a question this check asks — a held link's
	// body against a static body, or two held links' bodies, whose relation
	// is Verify's (agent-docs/linkage-check-design.md §6 step 2). It is never
	// evaluated, raises nothing, and enters no interval.
	Unformed bool
	// Declared: a linkage's declared joint contact
	// (agent-docs/linkage-check-design.md §5.4). It is evaluated at every pose and
	// publishes a transferred collision and a measured gap row, but no
	// undecided, touching or tolerance finding, and it enters no interval
	// certificate and no whole-path reading.
	Declared bool
	// Constant: the pair's relation is the same at every configuration
	// (agent-docs/linkage-check-design.md §5.9) — its relative path is one revolute
	// joint and one of its bodies is symmetric about that joint's axis. Its
	// first evaluation reads the two bodies as they stand; once holds the
	// outcome, which every pose replays with no placement, and the pair
	// travels nothing over an interval or a cell. A constant pair whose one
	// evaluation is undecided, touching or unmeasured is evaluated at every
	// pose like any other, with constant cleared.
	Constant bool
	Once     *constantOutcome
}

// constantOutcome is a constant pair's one evaluation on the bodies as they
// stand, where δ and η are zero: a measured gap, or a proven overlap with its
// volume measured once and its tolerance finding, nil when it passes.
type constantOutcome struct {
	res       pairkernel.Result
	Collision bool
	volume    vocab.Measurement
	beyond    *Diagnostic
}

func (p Pair) Evaluated() bool { return !p.Excluded && !p.Invalid && !p.Sheet && !p.Unformed }

// SweptBox is one mover's swept box over the whole path, as exact
// rational extremes per axis; ok is false when none could be formed.
type SweptBox struct {
	Lo, Hi motionbound.RatVec
	Ok     bool
}

// PairPose is one pair's answer at one pose. A gap is a proven interval
// [lo, hi] on the distance between the IDEAL placed bodies, η already
// charged.
type PairPose struct {
	HasGap    bool
	Lo, Hi    float64
	Diam      float64
	Collision bool
}

// Pose is one evaluated pose. result carries the pose's rows in
// PoseResult's shape, with Pose the first group's pose — the whole pose for
// VerifyMotion's one group. collisions name the moving body as Moving and its
// partner, static or moving, as Static.
//
// A path's pose sits at the fraction f, whose exact parameter is param, and
// its findings carry At. A joint box's centre has neither: cell is the cell
// whose centre it is, and its findings carry Cell instead
// (agent-docs/linkage-check-design.md §14.2). where is what a finding's message
// names the pose by: the parameter on a path, the configuration in a box.
type Pose struct {
	F          *big.Rat
	Param      motionbound.MotionParam
	Cell       *vocab.JointCell
	Where      any
	Groups     []GroupPose
	Violated   bool // some pair's gap here is proven below the requested minimum
	Result     PoseResult
	Pairs      [][]PairPose
	Findings   []Diagnostic // this pose's findings, in report order
	Collisions []Collision
	// unbuildable is why the driver could not build the pose; nil for a pose
	// it built. An unbuildable pose evaluates no pair.
	Unbuildable error
}

// Placed is one mover's transient placement at one pose, with the pose
// deviation η it carries and the swept-volume allowance a collision on it
// must clear.
type Placed struct {
	Body      *topo.Body
	eta       float64
	allowance float64
	// kept: the placement outlives the pose — a fixed group's own body, or a
	// constant group's reused transient — so its carriers stay cached.
	kept bool
}

// Setup reads VerifyMotion's one moving Group: ρ_max for a revolute or a
// between, the stretch bases, and the swept box each mover's exclusion
// compares the static bodies against.
func (r *Run) Setup(moving []*topo.Body) {
	r.Dom = r.Spec.Domain
	r.Drive = &Single{r: r}
	r.ReadBodies([][]*topo.Body{moving})
	r.Stretch, r.StretchEnd = 1, 1
	if r.Spec.Kind == motionbound.MotionBetween {
		r.Stretch = motionbound.BasisSigmaUpper(r.Spec.Between.From)
		r.StretchEnd = math.Max(r.Stretch, motionbound.BasisSigmaUpper(r.Spec.Between.To))
	}
	zero := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	swept := make([]SweptBox, len(r.Movers))
	for i := range r.Movers {
		mv := &r.Movers[i]
		if r.Spec.Kind != motionbound.MotionPrismatic {
			mv.Rho = motionbound.MoverAxisRadius(mv.Body.Bounds, r.Spec.Frame)
		}
		// The swept box covers every pose from where the path's box was read:
		// for a Revolute or a Prismatic that is the mover at rest — the
		// parameter 0 — so its travel runs from 0 to the farther endpoint,
		// never merely across [From, To]; for a Between it is the From-placed
		// box at s = 0, From itself, so the travel is the whole path's.
		travel := MaxRat(
			motionbound.MoverTravel(r.Spec.Frame, mv.Rho, zero, r.Spec.FromP),
			motionbound.MoverTravel(r.Spec.Frame, mv.Rho, zero, r.Spec.ToP),
		)
		lo, hi, ok := motionbound.MoverSweptBox(mv.Body.Bounds, r.Spec.Frame, travel)
		swept[i] = SweptBox{Lo: lo, Hi: hi, Ok: ok}
	}
	r.FormPairs(swept)
}

// readBodies reads every moving group's bodies and every static Body: the
// transient identity counters, each body's validity, and each mover's record
// radius, area and singular-value bound.
func (r *Run) ReadBodies(groups [][]*topo.Body) {
	// Payload rebuilding mints level and curve denotations. Start above every
	// live identity, then mint only on this copy: successive pose bodies stay
	// distinct from static geometry without changing the caller's document.
	live := r.live
	r.transient = r.hooks.NewDocument(r.doc, live)
	isMover := make(map[*topo.Body]struct{})
	for g, bodies := range groups {
		for _, b := range bodies {
			isMover[b] = struct{}{}
			r.Movers = append(r.Movers, Mover{Body: b, Group: g, Validity: verifyrun.PublishValidityResult(b, verifyrun.BodyValidityEvidence(r.Ctx, b))})
		}
	}
	for _, b := range live {
		if _, ok := isMover[b]; ok {
			continue
		}
		r.Statics = append(r.Statics, Static{Body: b, Validity: verifyrun.PublishValidityResult(b, verifyrun.BodyValidityEvidence(r.Ctx, b))})
	}
	for i := range r.Movers {
		mv := &r.Movers[i]
		mv.R0 = moverRecordRadius(r.Ctx, mv.Body)
		mv.HeldRadius = math.Inf(1)
		if fp, ok := mv.Body.Payload.(facetedrecord.Payload); ok {
			mv.HeldRadius = motionbound.HeldRadiusUpper(fp.Verts, fp.MeshBound)
		}
		mv.Area = proofbound.AbsSumUpper(mv.Body.Area.Value.Base(), mv.Body.Area.Bound.Base())
		mv.Sigma = motionbound.BasisSigmaLower(mv.Body.Payload.Transform())
	}
}

// formPairs forms every mover's row of pairs in pair order and settles each
// pair's standing for the whole call. A (mover, static) pair is excluded when
// the mover's swept box separates from the static body's box; a pair of two
// movers is always evaluated.
func (r *Run) FormPairs(swept []SweptBox) {
	r.Pairs = make([][]Pair, len(r.Movers))
	for i, mv := range r.Movers {
		row := make([]Pair, 0, len(r.Statics))
		for _, st := range r.Statics {
			state, lower := motionrun.ClassifyStaticPair(mv.Body, st.Body,
				mv.Validity.Outcome == vocab.ValidityValid, st.Validity.Outcome == vocab.ValidityValid,
				swept[i].Lo, swept[i].Hi, swept[i].Ok)
			pair := Pair{Other: -1, Declared: r.isDeclared(mv.Body, st.Body), Lower: lower}
			switch state {
			case motionrun.StaticPairInvalid:
				pair.Invalid = true
			case motionrun.StaticPairExcluded:
				pair.Excluded = true
			case motionrun.StaticPairSheet:
				pair.Sheet = true
			}
			row = append(row, pair)
		}
		for o := i + 1; o < len(r.Movers); o++ {
			other := r.Movers[o]
			if other.Group == mv.Group {
				continue
			}
			pair := Pair{Other: o, Declared: r.isDeclared(mv.Body, other.Body)}
			switch {
			case mv.Validity.Outcome != vocab.ValidityValid || other.Validity.Outcome != vocab.ValidityValid:
				pair.Invalid = true
			case mv.Body.Kind == topo.BodySheet || other.Body.Kind == topo.BodySheet:
				pair.Sheet = true
			}
			row = append(row, pair)
		}
		r.Pairs[i] = row
	}
}

// isDeclared reports whether a and b form a declared joint contact.
func (r *Run) isDeclared(a, b *topo.Body) bool {
	_, ok := r.Declared[[2]*topo.Body{a, b}]
	return ok
}

// partner is the caller's own body pair k of mover i names beside the mover.
func (r *Run) Partner(i, k int) *topo.Body {
	if o := r.Pairs[i][k].Other; o >= 0 {
		return r.Movers[o].Body
	}
	return r.Statics[k].Body
}

// Single drives VerifyMotion's one group along its Motion. points
// keeps each mover's projection reading at each pose, read once.
type Single struct {
	r      *Run
	points map[motionPointsKey]cornerBounds
}

// motionPointsKey names one mover's projection reading at one Pose: its box
// corners, or its hull points.
type motionPointsKey struct {
	Pose  *Pose
	mover int
	hull  bool
}

func (m *Single) PosesAt(f *big.Rat, at units.Value, param motionbound.MotionParam) ([]GroupPose, error) {
	r := m.r
	pose, err := r.Spec.Motion.PoseAt(at)
	if err != nil {
		return nil, err
	}
	ideals := []motionbound.IdealPose{r.Spec.Frame.At(param)}
	stretch := r.Stretch
	if r.Spec.Kind == motionbound.MotionBetween && f.Cmp(big.NewRat(1, 1)) == 0 {
		// PoseAt(1) returns the stated To, which the exact screw of the read
		// parameters rebuilds only to rounding: the pose is charged against
		// both the ideal end and To itself (§5.1, η_1 = max(η_ideal, η_To)).
		ideals = append(ideals, r.Spec.Frame.StatedEnd())
		stretch = r.StretchEnd
	}
	return []GroupPose{{Pose: pose, Ideals: ideals, Stretch: stretch}}, nil
}

// travel is τ of agent-docs/motion-check-design.md §5.2 for mover i: every pair of
// one group's mover has a static partner, which does not move.
func (m *Single) Travel(i, _ int, a, b motionbound.MotionParam) *big.Rat {
	return motionbound.MoverTravel(m.r.Spec.Frame, m.r.Movers[i].Rho, a, b)
}

// projection is the projection bound of agent-docs/linkage-check-design.md §5.8
// for pair k of mover i over the interval between poses a and b, the motion
// read as one joint whose value is the motion's parameter
// (agent-docs/motion-check-design.md §5.2): the mover's points — its box corners,
// then its hull points — posed at each end and expanded to second order in
// the parameter along the interval's own step, the static partner's points
// as they stand. The parameter is affine in the fraction over the whole path,
// so the segment term always serves. It is the largest bound any end and
// reading gives, nil when none can be read.
func (m *Single) Projection(i, k int, a, b *Pose) *big.Rat {
	r := m.r
	if r.Pairs[i][k].Other >= 0 {
		// Unreachable: VerifyMotion moves one group, whose movers pair only
		// with static bodies.
		return nil
	}
	return linkagebound.MotionProjection(r.Spec.Frame, r.Movers[i].Rho, a.Param, b.Param,
		func(end int, hull bool) (cornerBounds, bool) {
			if end == 0 {
				return m.moverPoints(a, i, hull)
			}
			return m.moverPoints(b, i, hull)
		},
		func(hull bool) (cornerBounds, bool) { return m.staticPoints(k, hull) })
}

// moverPoints is mover i's projection reading at a Pose: its points posed by
// the motion's ideal pose there, each with its velocity per unit of the
// parameter — k × (x − c) per radian for a Revolute, the unit direction per
// millimetre for a Prismatic, θ·k × (x − c) + d·k per unit fraction for a
// Between — rounded outward, read once and kept.
func (m *Single) moverPoints(pose *Pose, i int, hull bool) (cornerBounds, bool) {
	key := motionPointsKey{Pose: pose, mover: i, hull: hull}
	if got, ok := m.points[key]; ok {
		return got, true
	}
	points, ok := BodyPoints(m.r.Movers[i].Body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := linkagebound.MotionMoverPoints(m.r.Spec.Frame, pose.Param, points)
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
func (m *Single) staticPoints(k int, hull bool) (cornerBounds, bool) {
	key := motionPointsKey{mover: -1 - k, hull: hull}
	if got, ok := m.points[key]; ok {
		return got, true
	}
	points, ok := BodyPoints(m.r.Statics[k].Body, hull)
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

// MaxRat is the larger of two optional rationals; a nil (unbounded) operand
// wins.
func MaxRat(a, b *big.Rat) *big.Rat {
	if a == nil || b == nil {
		return nil
	}
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// Span is one interval's standing between two adjacent poses.
type Span struct {
	outcome   vocab.IntervalOutcome
	clearance *vocab.Measurement
	note      string // why a driver refused the interval, appended to its finding
}

// refine evaluates the two endpoints and bisects (§6), returning the poses in
// parameter order and the intervals between them.
func (r *Run) Refine() ([]*Pose, []Span, error) {
	var poses []*Pose
	for _, end := range []struct {
		f  *big.Rat
		at units.Value
	}{{new(big.Rat), r.Dom.From}, {big.NewRat(1, 1), r.Dom.To}} {
		pose, err := r.evaluatePose(end.f, end.at)
		if err != nil {
			return nil, nil, err
		}
		poses = append(poses, pose)
	}
	spans := []Span{r.intervalVerdict(poses[0], poses[1])}
	for {
		k := r.nextRefinement(poses, spans)
		if k < 0 {
			break
		}
		f := new(big.Rat).Add(poses[k].F, poses[k+1].F)
		f.Quo(f, big.NewRat(2, 1))
		mid, err := r.evaluatePose(f, r.Dom.Label(f))
		if err != nil {
			return nil, nil, err
		}
		poses = slices.Insert(poses, k+1, mid)
		spans[k] = r.intervalVerdict(poses[k], mid)
		spans = slices.Insert(spans, k+1, r.intervalVerdict(mid, poses[k+2]))
	}
	if err := r.Ctx.Err(); err != nil {
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
func (r *Run) nextRefinement(poses []*Pose, spans []Span) int {
	allClear := true
	smallest := -1
	for k, span := range spans {
		if poses[k].Unbuildable != nil && poses[k+1].Unbuildable != nil {
			// Never bisected: no pose between two unbuildable ones is asked.
			allClear = false
			continue
		}
		startHits, endHits := len(poses[k].Collisions) > 0, len(poses[k+1].Collisions) > 0
		onset := span.outcome == vocab.IntervalColliding && startHits != endHits
		if (span.outcome == vocab.IntervalUndecided || onset) && r.wide(poses[k], poses[k+1]) {
			return k
		}
		if span.outcome != vocab.IntervalClear {
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
		if reading, _ := r.PathClearance(poses, spans[smallest].clearance, "whole-path"); reading != nil && reading.Tolerance.State != vocab.ToleranceSatisfied {
			return smallest
		}
	}
	if r.Cfg.MinimumMM != nil && r.wide(a, b) && !anyViolated(poses) && !r.MeetsMinimum(spans[smallest].clearance) {
		return smallest
	}
	return -1
}

// wideForReading reports whether the interval between two poses is wider
// than the reading's own floor: cfg.ReadingP when the check sets one, and
// the resolution otherwise.
func (r *Run) wideForReading(a, b *Pose) bool {
	if r.Cfg.ReadingP == nil {
		return r.wide(a, b)
	}
	return motionbound.ExceedsResolution(a.Param, b.Param, *r.Cfg.ReadingP)
}

// wide reports whether the interval between two poses is wider than the
// resolution.
func (r *Run) wide(a, b *Pose) bool {
	return motionbound.ExceedsResolution(a.Param, b.Param, r.Cfg.ResolutionP)
}

// meetsMinimum reports whether a certified interval lower bound proves the
// requested minimum, compared exactly against the minimum as stated.
func (r *Run) MeetsMinimum(clearance *vocab.Measurement) bool {
	if clearance == nil {
		return true
	}
	return proofarith.FloatRat(clearance.Value.Base()).Cmp(r.Cfg.MinimumMM) >= 0
}

func anyViolated(poses []*Pose) bool {
	return slices.ContainsFunc(poses, func(p *Pose) bool { return p.Violated })
}

// evaluatePose builds every group's pose at fraction f of the path, published
// as the parameter at, and runs every evaluated pair at it. Every bound is
// built from the exact parameter at f; motionbound.PoseDeviation charges
// whatever separates the pose the driver builds from at and the ideal pose at
// f.
func (r *Run) evaluatePose(f *big.Rat, at units.Value) (*Pose, error) {
	if err := r.Ctx.Err(); err != nil {
		return nil, err
	}
	param := r.Dom.FromP.Lerp(r.Dom.ToP, f)
	groups, err := r.Drive.PosesAt(f, at, param)
	if err != nil {
		if _, ok := errors.AsType[PoseUnbuildable](err); !ok {
			return nil, err
		}
		return &Pose{F: f, Param: param, Where: at, Unbuildable: err,
			Result: PoseResult{At: at}, Pairs: make([][]PairPose, len(r.Movers))}, nil
	}
	mp := r.NewPose(groups, at, at)
	mp.F, mp.Param = f, param
	if err := r.RunPairs(mp); err != nil {
		return nil, err
	}
	return mp, nil
}

// newPose is an empty pose over groups, published as the parameter at and
// named in finding messages by where.
func (r *Run) NewPose(groups []GroupPose, at units.Value, where any) *Pose {
	return &Pose{
		Where:  where,
		Groups: groups,
		Result: PoseResult{
			At:            at,
			Pose:          groups[0].Pose,
			Interferences: []Interference{},
			Clearances:    []Clearance{},
			Diagnostics:   []Diagnostic{},
		},
		Pairs: make([][]PairPose, len(r.Movers)),
	}
}

// runPairs runs every evaluated pair at a built Pose: every body's validity
// finding first, then each mover's row of pairs in pair order. Each mover's
// transient placement is built when its first pair needs it and dropped from
// the carrier cache when the pose is done.
func (r *Run) RunPairs(mp *Pose) error {
	for _, mv := range r.Movers {
		r.validityFindings(mp, mv.Validity)
	}
	for _, st := range r.Statics {
		r.validityFindings(mp, st.Validity)
	}
	placed := make([]*Placed, len(r.Movers))
	defer func() {
		for _, p := range placed {
			if p != nil && !p.kept {
				r.Cache.Evict(p.Body)
			}
		}
	}()
	for i := range r.Movers {
		mp.Pairs[i] = make([]PairPose, len(r.Pairs[i]))
		if err := r.evaluateMover(mp, i, placed); err != nil {
			return err
		}
	}
	return nil
}

// validityFindings carries a not-proven-valid body's own validity diagnostic
// into the pose (agent-docs/motion-check-design.md §7), stamped with where it was
// found.
func (r *Run) validityFindings(mp *Pose, validity ValidityResult) {
	if validity.Outcome == vocab.ValidityValid {
		return
	}
	for _, diag := range validity.Diagnostics {
		diag = mp.stamp(diag)
		mp.Result.Diagnostics = append(mp.Result.Diagnostics, diag)
		mp.Findings = append(mp.Findings, diag)
	}
}

func withAt(diag Diagnostic, at units.Value) Diagnostic {
	diag.At = &at
	return diag
}

// stamp marks a finding at this pose with where it was found: the cell whose
// centre the pose is, or else the parameter.
func (mp *Pose) stamp(diag Diagnostic) Diagnostic {
	if mp.Cell == nil {
		return withAt(diag, mp.Result.At)
	}
	cell := mp.Cell.Clone()
	diag.Cell = &cell
	return diag
}

// NewRun returns a run over doc's live bodies, which the caller lists in
// document order.
func NewRun(ctx context.Context, h Hooks, doc *topo.Document, live []*topo.Body, cfg Config) *Run {
	return &Run{Ctx: ctx, hooks: h, doc: doc, live: live, Cfg: cfg, Cache: &pairkernel.Cache{}}
}
