package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the linkage vocabulary of docs/linkage-check-design.md §2 and
// §4: Linkage, Link and the sealed Joint set, the Drive that moves them,
// Linkage.PoseAt, and the LinkageReport records. linkage_verify.go runs
// Document.VerifyLinkage over motion_verify.go's engine, and linkage_bound.go
// proves the chain travel bound its interval certificate consumes.

// Linkage is a tree of links joined by joints, rooted at the ground link. A
// link is created only through its parent, so the tree has no cycles. Every
// body of the Document that belongs to no link is static.
//
// The zero pose is the document as it stands: every joint value is 0 when the
// linkage is built, and a joint's Center, Axis and Dir are world coordinates
// at that pose. Building a linkage is not safe for concurrent use; a built
// linkage is only read, by PoseAt and VerifyLinkage.
type Linkage struct {
	ground   *Link
	links    []*Link
	member   map[*Body]*Link // the link every listed body belongs to
	contacts []DiagnosticPair
	loops    []*LinkageLoop // in Close order (linkage_loop.go)
}

// NewLinkage returns a linkage holding the ground link alone.
func NewLinkage() *Linkage {
	l := &Linkage{member: make(map[*Body]*Link)}
	l.ground = &Link{linkage: l, index: -1}
	return l
}

// Ground returns the ground link, which carries no joint and no body.
func (l *Linkage) Ground() *Link {
	return l.ground
}

// Links returns every link but the ground, in creation order. A link's parent
// always precedes it.
func (l *Linkage) Links() []*Link {
	return slices.Clone(l.links)
}

// DeclareJointContact names a pair of bodies that meet at a joint by
// construction — a pin in its bore, a slider on its rail, an arm resting on
// its pivot's cap (docs/linkage-check-design.md §2.2). VerifyLinkage checks
// the pair for proven overlap at every evaluated pose and publishes a gap it
// measures, but the pair publishes no undecided or touching finding and
// enters no interval certificate and no whole-drive reading (§5.4).
//
// a and b MUST belong to different links, or one to a link and the other to
// no link. A nil body, a == b, two bodies of one link, two bodies of no link,
// and a pair declared twice in either order are ErrDegenerate. Liveness and
// document membership are checked by VerifyLinkage.
func (l *Linkage) DeclareJointContact(a, b *Body) error {
	if l == nil {
		return fmt.Errorf(`%w: a nil linkage holds no joint`, ErrDegenerate)
	}
	if a == nil || b == nil {
		return fmt.Errorf(`%w: a joint contact names two bodies`, ErrDegenerate)
	}
	if a == b {
		return fmt.Errorf(`%w: a body cannot meet itself at a joint`, ErrDegenerate)
	}
	la, lb := l.member[a], l.member[b]
	switch {
	case la == nil && lb == nil:
		return fmt.Errorf(`%w: a joint contact needs a body of a link`, ErrDegenerate)
	case la == lb:
		return fmt.Errorf(`%w: two bodies of one link never move apart`, ErrDegenerate)
	}
	for _, c := range l.contacts {
		if (c.A == a && c.B == b) || (c.A == b && c.B == a) {
			return fmt.Errorf(`%w: a joint contact is declared twice`, ErrDegenerate)
		}
	}
	l.contacts = append(l.contacts, DiagnosticPair{A: a, B: b})
	return nil
}

// JointContacts returns every declared joint contact, in declaration order.
func (l *Linkage) JointContacts() []DiagnosticPair {
	return slices.Clone(l.contacts)
}

// Link is one rigid set of bodies and the joint that attaches it to its
// parent. Two bodies of one link never move relative to each other.
type Link struct {
	linkage *Linkage
	parent  *Link
	joint   Joint
	bodies  []*Body
	index   int // the position in Linkage.Links(); −1 for the ground
}

// Revolute attaches a new child link of p, holding bodies, on a revolute
// joint about the axis through center along axis (world coordinates at the
// zero pose). It refuses, in this order: a nil p or a Link no linkage built
// (ErrDegenerate); an empty bodies, a nil body, or a body listed twice or
// already listed in a link of this linkage (ErrDegenerate); a non-finite
// center or axis component (ErrNotFinite); the zero axis (ErrDegenerate);
// then the options' own refusals (WithJointLimits). Liveness and document
// membership are checked by VerifyLinkage.
func (p *Link) Revolute(center, axis r3.Vec, bodies []*Body, opts ...JointOption) (*Link, error) {
	if err := p.admit(bodies); err != nil {
		return nil, err
	}
	if !proofbound.FiniteVec(center) || !proofbound.FiniteVec(axis) {
		return nil, fmt.Errorf(`%w: a revolute joint's center and axis must be finite, got %v and %v`, ErrNotFinite, center, axis)
	}
	if zeroVec(axis) {
		return nil, fmt.Errorf(`%w: a zero rotation axis names no direction`, ErrDegenerate)
	}
	limits, err := resolveJointOptions(opts, units.Angle)
	if err != nil {
		return nil, err
	}
	return p.attach(RevoluteJoint{Center: center, Axis: axis, Limits: limits}, bodies), nil
}

// Prismatic attaches a new child link of p, holding bodies, on a prismatic
// joint sliding along dir (a world direction at the zero pose; only its
// direction is used). It refuses, in this order: a nil p or a Link no linkage
// built (ErrDegenerate); an empty bodies, a nil body, or a body listed twice
// or already listed in a link of this linkage (ErrDegenerate); a non-finite
// dir component (ErrNotFinite); a dir r3.Vec.Normalize reports no direction
// for (ErrDegenerate); then the options' own refusals (WithJointLimits).
// Liveness and document membership are checked by VerifyLinkage.
func (p *Link) Prismatic(dir r3.Vec, bodies []*Body, opts ...JointOption) (*Link, error) {
	if err := p.admit(bodies); err != nil {
		return nil, err
	}
	if !proofbound.FiniteVec(dir) {
		return nil, fmt.Errorf(`%w: a prismatic joint's direction must be finite, got %v`, ErrNotFinite, dir)
	}
	if _, ok := dir.Normalize(); !ok {
		return nil, fmt.Errorf(`%w: the prismatic direction %v names no direction`, ErrDegenerate, dir)
	}
	limits, err := resolveJointOptions(opts, units.Length)
	if err != nil {
		return nil, err
	}
	return p.attach(PrismaticJoint{Dir: dir, Limits: limits}, bodies), nil
}

// admit applies a child link's parent and body refusals.
func (p *Link) admit(bodies []*Body) error {
	if p == nil || p.linkage == nil {
		return fmt.Errorf(`%w: a link is attached only to a link of a linkage`, ErrDegenerate)
	}
	if len(bodies) == 0 {
		return fmt.Errorf(`%w: a link with no body moves nothing`, ErrDegenerate)
	}
	seen := make(map[*Body]struct{}, len(bodies))
	for _, b := range bodies {
		if b == nil {
			return fmt.Errorf(`%w: a nil body cannot belong to a link`, ErrDegenerate)
		}
		if _, dup := seen[b]; dup {
			return fmt.Errorf(`%w: a body is listed twice in one link`, ErrDegenerate)
		}
		if _, taken := p.linkage.member[b]; taken {
			return fmt.Errorf(`%w: a body already belongs to a link of this linkage`, ErrDegenerate)
		}
		seen[b] = struct{}{}
	}
	return nil
}

func (p *Link) attach(j Joint, bodies []*Body) *Link {
	l := p.linkage
	child := &Link{linkage: l, parent: p, joint: j, bodies: slices.Clone(bodies), index: len(l.links)}
	for _, b := range bodies {
		l.member[b] = child
	}
	l.links = append(l.links, child)
	return child
}

// Parent returns the link k is attached to; nil for the ground.
func (k *Link) Parent() *Link {
	return k.parent
}

// Joint returns the joint attaching k to its parent; nil for the ground.
func (k *Link) Joint() Joint {
	return k.joint
}

// Bodies returns k's bodies in the order they were given; none for the
// ground.
func (k *Link) Bodies() []*Body {
	return slices.Clone(k.bodies)
}

// Joint is how a link moves against its parent. The set is sealed:
// RevoluteJoint and PrismaticJoint are its only members.
type Joint interface{ joint() }

// RevoluteJoint rotates its link about the axis through Center along Axis,
// right-handed, by the joint's value, an Angle. Center and Axis are world
// coordinates at the zero pose.
type RevoluteJoint struct {
	Center r3.Vec       // a position, millimetres (core §5.2)
	Axis   r3.Vec       // a direction; only its direction is used
	Limits *JointLimits // nil when none were declared
}

func (RevoluteJoint) joint() {}

// PrismaticJoint slides its link along Dir by the joint's value, a Length.
// Dir is a world direction at the zero pose; only its direction is used.
type PrismaticJoint struct {
	Dir    r3.Vec
	Limits *JointLimits // nil when none were declared
}

// JointLimits is a joint's declared working range, in the joint's Kind:
// Min < Max. A drive whose sweep reaches outside it, or that holds an
// unlisted joint at a 0 outside it, is refused (docs/linkage-check-design.md
// §2.4).
type JointLimits struct {
	Min, Max units.Value
}

// JointOption configures a joint built by Link.Revolute or Link.Prismatic.
type JointOption interface {
	option.Interface
	jointOption()
}

type jointOption struct{ option.Interface }

func (jointOption) jointOption() {}

type identJointLimits struct{}

// WithJointLimits declares the joint's working range [minimum, maximum], in
// the joint's Kind: an Angle for a revolute, a Length for a prismatic. A
// wrong Kind is ErrUnitKind, a non-finite bound ErrNotFinite, and
// minimum >= maximum ErrDegenerate. Limits that exclude 0 are legal: the zero
// pose then lies outside the working range, and only a drive that visits it is
// refused.
func WithJointLimits(minimum, maximum units.Value) JointOption {
	return jointOption{option.New(identJointLimits{}, JointLimits{Min: minimum, Max: maximum})}
}

// resolveJointOptions folds a joint's options; the last WithJointLimits wins.
func resolveJointOptions(opts []JointOption, kind units.Kind) (*JointLimits, error) {
	var limits *JointLimits
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		v, ok := option.Get[JointLimits](o)
		if !ok {
			return nil, fmt.Errorf(`%w: a joint option carries no value`, ErrDegenerate)
		}
		if err := motionKinds(kind, v.Min, v.Max); err != nil {
			return nil, err
		}
		if err := motionFinite(v.Min, v.Max); err != nil {
			return nil, err
		}
		if c, ok := paramCompare(v.Min, v.Max); !ok || c >= 0 {
			return nil, fmt.Errorf(`%w: a joint's limits need Min < Max, got %s and %s`, ErrDegenerate, v.Min, v.Max)
		}
		limits = &v
	}
	return limits, nil
}

// paramCompare orders two finite values of one Kind by the exact quantities
// they denote (motionbound.MotionParam): −1, 0 or +1. An angle mixing whole
// turns and radians is compared through π's enclosure; ok is false when that
// enclosure cannot sign the difference, and every caller refuses then.
func paramCompare(a, b units.Value) (int, bool) {
	pa, okA := motionbound.ExactMotionParam(a)
	pb, okB := motionbound.ExactMotionParam(b)
	if !okA || !okB {
		return 0, false
	}
	dTurn := new(big.Rat).Sub(pa.Turn, pb.Turn)
	dBase := new(big.Rat).Sub(pa.Base, pb.Base)
	switch {
	case dTurn.Sign() == 0:
		return dBase.Sign(), true
	case dBase.Sign() == 0:
		return dTurn.Sign(), true
	}
	twoPi := proofbound.TwoPiInterval()
	lo := new(big.Rat).Mul(dTurn, twoPi.Lo)
	hi := new(big.Rat).Mul(dTurn, twoPi.Hi)
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	lo.Add(lo, dBase)
	hi.Add(hi, dBase)
	switch {
	case lo.Sign() > 0:
		return 1, true
	case hi.Sign() < 0:
		return -1, true
	}
	return 0, false
}

// within reports whether v lies in the closed range the limits declare, as
// far as paramCompare can decide; an undecided comparison is outside.
func (lim *JointLimits) within(v units.Value) bool {
	lo, okLo := paramCompare(lim.Min, v)
	hi, okHi := paramCompare(v, lim.Max)
	return okLo && okHi && lo <= 0 && hi <= 0
}

func (PrismaticJoint) joint() {}

// Drive is a one-parameter motion of a linkage, parameterised by the
// Dimensionless fraction s ∈ [0, 1]: each listed joint passes through its
// waypoints From, Via…, To in order, linearly between consecutive ones, and
// an unlisted joint holds 0 (docs/linkage-check-design.md §2.3). A drive
// whose sweeps carry n − 1 Via values each has n segments, and segment j
// covers s ∈ [j/n, (j+1)/n], an equal share each; with no Via the drive is
// one segment and each joint runs from From to To as s runs from 0 to 1.
type Drive []JointSweep

// JointSweep moves one link's joint. A sweep whose waypoints are all equal
// holds the joint at that value for the whole drive; From > To runs it the
// other way, and From == To with a different Via value goes out and comes
// back.
type JointSweep struct {
	Link     *Link       // the link whose joint this moves
	From, To units.Value // the joint's Kind: Angle for a revolute, Length for a prismatic
	// Via is the joint's value at each interior waypoint of the drive, in
	// order and in the joint's Kind. Every listed sweep of one drive carries
	// the same number of Via values.
	Via []units.Value
}

// LinkagePose is every link's pose at one parameter value.
type LinkagePose struct {
	At units.Value // the Dimensionless fraction s
	// Values is each link's joint value at s, in Linkage.Links() order: the
	// stated value, or for a loop's dependent joint the float midpoint of its
	// certified enclosure (docs/linkage-check-design.md §15.4).
	Values []units.Value
	// Bounds is each value's proven half-width, in its Kind: Values[k] ±
	// Bounds[k] holds the exact joint value. It is zero for every joint whose
	// value the drive states.
	Bounds []units.Value
	Poses  []r3.Transform // each link's world pose at s, in the same order
}

// PoseAt returns every link's joint value and world pose at the fraction at
// of drive (docs/linkage-check-design.md §2.4). In segment j of n, between
// waypoints w_j and w_{j+1}, a joint's value is w_j + t·(w_{j+1} − w_j) with
// t = n·at − j, carried in w_j's unit — From + at·(To − From) for a drive
// with no Via — and 0 in its Kind's base unit for an unlisted joint. A
// waypoint's fraction j/n takes segment j, and 1 the last segment. A link's pose is its own joint's motion, then its
// parent's pose: J_k(q_k).Then(Pose_parent), with J_k the rotation a Revolute
// and the translation a Prismatic of the same axis or direction build, and
// the ground's pose the identity; it composes onto each body's own placement
// as Body.Placed composes. It is the one place a linkage pose is built, so a
// renderer above and VerifyLinkage below evaluate the same transform.
//
// It refuses a nil linkage, a sweep naming no link, the ground, a link of
// another linkage or a link twice, and two sweeps carrying different numbers
// of Via values (ErrDegenerate); a sweep From, To or Via value of the wrong
// Kind for its joint (ErrUnitKind) or non-finite (ErrNotFinite); an at that is
// not a finite Dimensionless value (ErrUnitKind, ErrNotFinite); a waypoint,
// or an unlisted joint's 0, outside the joint's declared limits
// (ErrDegenerate, naming the link); and a pose r3 cannot represent
// (ErrNotFinite). An at outside [0, 1] is legal: PoseAt takes no range, the
// first segment's line extends below 0 and the last's above 1, and a drive
// whose every sweep holds is legal here.
//
// A drive that moves a loop (docs/linkage-check-design.md §15) is answered by
// a one-shot Schedule under context.Background(), with Schedule.PoseAt's
// refusals: its loop rows, an at outside [0, 1], and an at the loop cannot be
// enclosed at (ErrUnsupported). A renderer that wants one pose per frame
// calls Schedule once instead.
func (l *Linkage) PoseAt(d Drive, at units.Value) (LinkagePose, error) {
	spec, err := l.resolveDrive(d)
	if err != nil {
		return LinkagePose{}, err
	}
	if err := motionValueValid(at, units.Dimensionless, "the pose fraction"); err != nil {
		return LinkagePose{}, err
	}
	p, ok := motionbound.ExactMotionParam(at)
	if !ok {
		return LinkagePose{}, fmt.Errorf(`%w: the pose fraction is not representable`, ErrNotFinite)
	}
	if len(spec.loops) > 0 {
		ctx := context.Background()
		if err := spec.prepareLoops(ctx, big.NewRat(1, linkageScheduleFloor)); err != nil {
			return LinkagePose{}, err
		}
	}
	return spec.poseAt(context.Background(), at, p.Base)
}

// LinkageReport is what VerifyLinkage returns (docs/linkage-check-design.md
// §4): VerifyMotion's vocabulary over the links of a linkage.
// Diagnostics lists interval findings in interval order, then pose findings
// in pose order, then the whole-drive reading's own tolerance finding; it is
// empty exactly when Status is Sound, and Status is the worst
// Diagnostic.Status in it. Every At, From and To is the Dimensionless
// fraction s.
type LinkageReport struct {
	Request MotionRequest // the validated effective settings, including defaults
	// ReadingResolution is the floor the whole-drive Clearance reading refines
	// to: Request.Resolution when WithResolution was stated, and
	// units.Scalar(1.0/16384) otherwise, while the verdict stops at
	// Request.Resolution (docs/linkage-check-design.md §3).
	ReadingResolution units.Value
	Linkage           *Linkage            // the linkage as given
	Drive             Drive               // the drive as stated
	Links             []*Link             // Linkage.Links() order
	JointContacts     []DiagnosticPair    // every declared joint contact, in declaration order
	Against           []*Body             // every static body, in Document.Bodies() order
	Poses             []LinkagePoseResult // every pose evaluated, from s = 0 to s = 1
	Intervals         []MotionInterval    // between adjacent Poses, in the same order
	Collisions        []LinkCollision     // every proven collision, in traversal order then pair order
	Clearance         *ScalarReading      // the minimum gap over the whole drive; nil unless every interval is IntervalClear
	Assessment        Assessment          // against WithMinClearance; AssessmentNotEvaluated when not requested
	Diagnostics       []Diagnostic        // interval findings, then pose findings, then the whole-drive reading's
	Status            Status              // Unverified on a zero value; VerifyLinkage always returns a decided status
}

// Passed reports whether the report is Sound. It returns false for a nil
// report and for any other Status.
func (r *LinkageReport) Passed() bool {
	return r != nil && r.Status == Sound
}

// LinkagePoseResult is one evaluated pose: every link's pose there and the
// pair results at it in Verify's own shape. In every row and diagnostic, A is
// a link body and B a static body or a body of a later link (pair order,
// docs/linkage-check-design.md §4); a row names the caller's own bodies, never
// the transient placements the check evaluated. A Clearance row is a
// measurement at this pose only; the continuous claim lives on the
// MotionInterval.
type LinkagePoseResult struct {
	Pose          LinkagePose
	Interferences []Interference // proven overlap, bounded volume
	Clearances    []Clearance    // every pair proven disjoint or touching
	Diagnostics   []Diagnostic   // this pose's undecided or unsupported pairs and invalid bodies, At set
}

// LinkCollision is a proven overlap at an evaluated pose, about the ideal
// pose (docs/linkage-check-design.md §5.1): the measured overlap survives
// both bodies' deviation from their ideal poses, and Volume's Bound carries
// that allowance, so Volume.Value − Volume.Bound is a proven lower bound on
// the ideal overlap. A belongs to a link; B is a static body or a body of a
// later link. Nothing is claimed about the interval around it.
type LinkCollision struct {
	At     units.Value // the Dimensionless fraction s
	A, B   *Body
	Volume Measurement
}

// linkageSpec is a linkage and a drive read into one joint per link.
type linkageSpec struct {
	linkage *Linkage
	joints  []linkJoint  // Linkage.Links() order
	loops   []*loopDrive // every loop the drive moves (linkage_loop.go)
}

// linkJoint is one link's joint under a drive: its axis or direction, and
// its schedule — the waypoints as stated, or, for an unlisted joint, a hold
// at 0 in its Kind's base unit.
type linkJoint struct {
	link     *Link
	parent   int // the parent's position in Linkage.Links(); −1 for the ground
	revolute bool
	limits   *JointLimits
	center   r3.Vec
	axis     r3.Vec // the revolute's axis, or the prismatic's direction
	listed   bool
	kind     units.Kind
	// values are the joint's waypoints as stated — From, Via…, To — and
	// points their exact denotations; an unlisted joint holds 0 at two.
	values []units.Value
	points []motionbound.MotionParam
	// dep is the driven loop whose dependent this joint is, nil for a joint
	// whose value the drive states (docs/linkage-check-design.md §15); its
	// values and points are then a hold at 0 nobody reads, and depReach is
	// its proven reach over the certified drive.
	dep      *loopDrive
	depReach *big.Rat
}

// resolveDrive validates d against l (docs/linkage-check-design.md §2.4).
func (l *Linkage) resolveDrive(d Drive) (*linkageSpec, error) {
	if l == nil {
		return nil, fmt.Errorf(`%w: a nil linkage has no link to move`, ErrDegenerate)
	}
	spec := l.restSpec()
	segments := 0
	for _, sw := range d {
		link := sw.Link
		if link == nil || link.linkage != l || link.joint == nil {
			return nil, fmt.Errorf(`%w: a drive names a link that is not a jointed link of this linkage`, ErrDegenerate)
		}
		jt := &spec.joints[link.index]
		if jt.listed {
			return nil, fmt.Errorf(`%w: a drive names link %d twice`, ErrDegenerate, link.index)
		}
		if segments == 0 {
			segments = len(sw.Via) + 1
		}
		if len(sw.Via)+1 != segments {
			return nil, fmt.Errorf(`%w: every sweep of a drive passes the same waypoints, but link %d's sweep has %d Via values and an earlier one %d`,
				ErrDegenerate, link.index, len(sw.Via), segments-1)
		}
		if err := motionKinds(jt.kind, sw.From, sw.To); err != nil {
			return nil, err
		}
		for _, v := range sw.Via {
			if v.Kind() != jt.kind {
				return nil, fmt.Errorf(`%w: a sweep's Via value must be a %s, got %s`, ErrUnitKind, jt.kind, v.Kind())
			}
		}
		values := make([]units.Value, 0, segments+1)
		values = append(append(append(values, sw.From), sw.Via...), sw.To)
		if err := motionFinite(values...); err != nil {
			return nil, err
		}
		points := make([]motionbound.MotionParam, len(values))
		for w, v := range values {
			p, ok := motionbound.ExactMotionParam(v)
			if !ok {
				return nil, fmt.Errorf(`%w: a sweep's waypoint is not representable`, ErrNotFinite)
			}
			points[w] = p
		}
		jt.listed, jt.values, jt.points = true, values, points
	}
	if err := l.resolveLoops(spec, "drive"); err != nil {
		return nil, err
	}
	// q(s) is linear within each segment, so a drive keeps a joint inside its
	// limits exactly when every waypoint lies inside them; an unlisted joint
	// holds 0. A driven loop's dependent is held to its whole-drive hull
	// instead, once the schedule has read it (linkage_loop.go).
	for k, jt := range spec.joints {
		if jt.limits == nil || jt.dep != nil {
			continue
		}
		for w, v := range jt.values {
			if jt.limits.within(v) {
				continue
			}
			if len(jt.values) == 2 {
				return nil, fmt.Errorf(`%w: the drive takes link %d's joint from %s to %s, outside its limits [%s, %s]`,
					ErrDegenerate, k, jt.values[0], jt.values[1], jt.limits.Min, jt.limits.Max)
			}
			return nil, fmt.Errorf(`%w: the drive takes link %d's joint to %s at waypoint %d, outside its limits [%s, %s]`,
				ErrDegenerate, k, v, w, jt.limits.Min, jt.limits.Max)
		}
	}
	return spec, nil
}

// restSpec reads l into one joint per link, every joint holding 0 in its
// Kind's base unit: the schedule of a joint no drive or box lists.
func (l *Linkage) restSpec() *linkageSpec {
	spec := &linkageSpec{linkage: l, joints: make([]linkJoint, len(l.links))}
	for k, link := range l.links {
		jt := linkJoint{link: link, parent: link.parent.index}
		switch j := link.joint.(type) {
		case RevoluteJoint:
			jt.revolute, jt.center, jt.axis, jt.limits = true, j.Center, j.Axis, j.Limits
			jt.holdAtZero(units.Angle, units.Radian)
		case PrismaticJoint:
			jt.axis, jt.limits = j.Dir, j.Limits
			jt.holdAtZero(units.Length, units.Millimeter)
		}
		spec.joints[k] = jt
	}
	return spec
}

// holdAtZero sets the schedule of a joint the drive does not list.
func (jt *linkJoint) holdAtZero(kind units.Kind, unit units.Unit) {
	zero := func() motionbound.MotionParam {
		return motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	}
	jt.kind = kind
	jt.values = []units.Value{units.New(0, unit), units.New(0, unit)}
	jt.points = []motionbound.MotionParam{zero(), zero()}
}

// moves reports whether the joint's schedule changes its value anywhere: some
// waypoint differs from the first. A driven loop's dependent moves.
func (jt linkJoint) moves() bool {
	if jt.dep != nil {
		return !jt.dep.held
	}
	for _, v := range jt.values[1:] {
		if !sameMotionValue(jt.values[0], v) {
			return true
		}
	}
	return false
}

// segment is the segment of jt's schedule that holds the exact fraction s,
// as a domain from its starting waypoint to its ending one, and s's local
// fraction t = n·s − j in it (docs/linkage-check-design.md §2.3). Segment j of
// n covers [j/n, (j+1)/n]: a waypoint's fraction j/n takes segment j, 1 takes
// the last segment, and an s outside [0, 1] extends the first or the last.
// With one segment t is s itself.
func (jt linkJoint) segment(s *big.Rat) (motionDomain, *big.Rat) {
	n := len(jt.points) - 1
	j, t := 0, s
	if n > 1 {
		ns := new(big.Rat).Mul(s, big.NewRat(int64(n), 1))
		floor := new(big.Int).Div(ns.Num(), ns.Denom()) // Euclidean: the denominator is positive
		switch {
		case floor.Sign() < 0:
		case !floor.IsInt64() || floor.Int64() >= int64(n):
			j = n - 1
		default:
			j = int(floor.Int64())
		}
		t = ns.Sub(ns, big.NewRat(int64(j), 1))
	}
	return motionDomain{quantity: jt.kind, from: jt.values[j], to: jt.values[j+1], fromP: jt.points[j], toP: jt.points[j+1]}, t
}

// label is the joint's published value at the exact fraction s: its
// segment's label at the local fraction (motionDomain.label).
func (jt linkJoint) label(s *big.Rat) units.Value {
	seg, t := jt.segment(s)
	return seg.label(t)
}

// holds reports whether every listed sweep of the drive holds its joint, so
// that the drive names no motion.
func (s *linkageSpec) holds() bool {
	for _, jt := range s.joints {
		if jt.listed && jt.moves() {
			return false
		}
	}
	return true
}

// pose is the joint's own float motion at the value q.
func (jt linkJoint) pose(q units.Value) (r3.Transform, error) {
	if jt.revolute {
		return revolutePose(jt.center, jt.axis, q)
	}
	return prismaticPose(jt.axis, q)
}

// posesAt is every link's joint value and world pose at the exact fraction f
// of the drive: each joint's label there, posed by posesOf.
func (s *linkageSpec) posesAt(f *big.Rat) ([]units.Value, []r3.Transform, error) {
	values := make([]units.Value, len(s.joints))
	for k, jt := range s.joints {
		values[k] = jt.label(f)
	}
	poses, err := s.posesOf(values)
	if err != nil {
		return nil, nil, err
	}
	return values, poses, nil
}

// posesOf is every link's world pose at the joint values given, one per link
// in Linkage.Links() order: Pose_k = J_k(q_k).Then(Pose_parent), a link under
// the ground taking its joint's motion alone. It is the one place a linkage
// pose is built: Linkage.PoseAt, Linkage.Configuration, VerifyLinkage and
// VerifyJointBox all call it.
func (s *linkageSpec) posesOf(values []units.Value) ([]r3.Transform, error) {
	poses := make([]r3.Transform, len(s.joints))
	for k, jt := range s.joints {
		pose, err := jt.pose(values[k])
		if err != nil {
			return nil, err
		}
		if jt.parent >= 0 {
			if pose, err = pose.Then(poses[jt.parent]); err != nil {
				return nil, fmt.Errorf(`%w: composing a link's pose onto its parent's failed: %w`, ErrNotFinite, err)
			}
		}
		poses[k] = pose
	}
	return poses, nil
}
