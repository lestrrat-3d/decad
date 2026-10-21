package linkage

import (
	"errors"
	"fmt"
	"math/big"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/motionengine"

	"github.com/lestrrat-3d/decad/internal/motionconclude"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// evaluate runs the pair procedure at cell c's centre and classifies the
// cell from it (agent-docs/linkage-check-design.md §14.3, §14.4 step 4): blocked
// when a collision there survives every configuration of the cell, else
// colliding when some pair collides there, clear when every pair certifies
// the cell, undecided otherwise.
func (b *boxRun[B]) evaluate(c *boxCell[B]) error {
	r := b.run
	if err := r.Ctx.Err(); err != nil {
		return err
	}
	spec := b.dr.spec
	c.cell = vocab.JointCell{Min: make([]units.Value, len(spec.joints)), Max: make([]units.Value, len(spec.joints))}
	for k, jt := range spec.joints {
		c.cell.Min[k], c.cell.Max[k] = jt.label(c.lo[k]), jt.label(c.hi[k])
	}
	var values, bounds []units.Value
	var poses []r3.Transform
	var ideals []motionbound.IdealPose
	if len(spec.loops) == 0 {
		values = make([]units.Value, len(spec.joints))
		params := make([]motionbound.MotionParam, len(spec.joints))
		for k, jt := range spec.joints {
			f := c.centre(k)
			values[k], params[k] = jt.label(f), jointParam(jt, f)
		}
		var err error
		if poses, err = spec.posesOf(values); err != nil {
			return err
		}
		bounds, ideals = zeroBounds(values), idealPosesOf(spec, b.dr.frames, params)
	} else {
		lp, err := b.loopCentre(c)
		if err != nil {
			return err
		}
		values, bounds, poses, ideals = lp.values, lp.bounds, lp.poses, lp.ideals
	}
	var mp *motionengine.Pose
	if c.gate != "" && poses == nil {
		// agent-docs/linkage-check-design.md §16.3: an unbuildable centre has no
		// pose; it evaluates no pair and carries no row.
		mp = &motionengine.Pose{
			Cell: &c.cell, Unbuildable: errors.New(c.gate), Pairs: make([][]motionengine.PairPose, len(r.Movers)),
			Result: motionengine.PoseResult{Interferences: []motionengine.Interference{}, Clearances: []motionengine.Clearance{}, Diagnostics: []motionengine.Diagnostic{}},
		}
		for i := range r.Movers {
			mp.Pairs[i] = make([]motionengine.PairPose, len(r.Pairs[i]))
		}
	} else {
		groups := make([]motionengine.GroupPose, len(poses))
		for k := range poses {
			groups[k] = motionengine.GroupPose{
				Pose: poses[k], Ideals: []motionbound.IdealPose{ideals[k]}, Stretch: 1, Value: values[k], Bound: bounds[k],
				Fixed: b.dr.standing[k] == linkFixed, Constant: b.dr.standing[k] == linkConstant,
			}
		}
		mp = r.NewPose(groups, units.Value{}, "the configuration "+formatValues(values))
		mp.Cell = &c.cell
		if err := r.RunPairs(mp); err != nil {
			return err
		}
	}
	c.pose = mp
	b.evaluated = append(b.evaluated, c)
	b.note(mp)
	b.classify(c)
	return nil
}

// loopCentre reads a looped linkage's cell (agent-docs/linkage-check-design.md
// §16.2, §16.3): the centre's pose, each dependent at its enclosure's float
// midpoint and its ideal pose over the whole enclosure M_j, then per loop the
// hull H_j of each dependent over the cell's loop-axis range, δ_j =
// max(h_hi − m_lo, m_hi − h_lo) and the cell's published range for the
// dependent, H_j's ends rounded outward. A held loop's dependents stand
// still across the Cell: δ_j is zero and the range is M_j. A refused centre
// returns no pose and gates the cell; a refused hull gates it and publishes
// the dependent's range as zero values.
func (b *boxRun[B]) loopCentre(c *boxCell[B]) (loopPose, error) {
	spec := b.dr.spec
	fracs := make([]*big.Rat, len(spec.joints))
	for k := range fracs {
		fracs[k] = c.centre(k)
	}
	lp, err := spec.loopPosesOf(b.run.Ctx, b.dr.frames, fracs)
	var ub *unbuildableError[B]
	if errors.As(err, &ub) {
		c.gate, c.gateLoop = ub.Error(), ub.loop
		for _, ld := range spec.loops {
			for _, d := range ld.deps {
				c.cell.Min[d], c.cell.Max[d] = units.Value{}, units.Value{}
			}
		}
		return loopPose{}, nil
	}
	if err != nil {
		return loopPose{}, err
	}
	c.delta = make(map[int]*big.Rat)
	c.depReach = make(map[int]linkagebound.CellReach)
	for _, ld := range spec.loops {
		var hulls []proofbound.RatInterval
		if !ld.held {
			hulls, err = ld.cellHulls(b.run.Ctx, c.lo[ld.driver], c.hi[ld.driver])
			if errors.As(err, &ub) {
				if c.gate == "" {
					c.gate, c.gateLoop = ub.Error(), ld
				}
				for _, d := range ld.deps {
					c.cell.Min[d], c.cell.Max[d] = units.Value{}, units.Value{}
				}
				continue
			}
			if err != nil {
				return loopPose{}, err
			}
		}
		for j, d := range ld.deps {
			m := linkagebound.ValueInterval(lp.lo[d], lp.hi[d])
			h, delta := m, new(big.Rat)
			if hulls != nil {
				h, delta = hulls[j], linkagebound.DependentCellDelta(m, hulls[j])
			}
			c.delta[d] = delta
			c.depReach[d] = linkagebound.DependentCellReach(m, h)
			unit := lp.values[d].Unit()
			c.cell.Min[d], c.cell.Max[d] = units.New(proofbound.RatFloatDown(h.Lo), unit), units.New(proofbound.RatFloatUp(h.Hi), unit)
		}
	}
	return lp, nil
}

// classify sets a cell's outcome from its evaluated centre. A gated cell
// (agent-docs/linkage-check-design.md §16.3) is never CellClear or CellBlocked: it
// is CellColliding when its centre collides and CellUndecided otherwise.
func (b *boxRun[B]) classify(c *boxCell[B]) {
	r := b.run
	mp := c.pose
	for i := range r.Movers {
		for k := range r.Pairs[i] {
			if mp.Pairs[i][k].Collision {
				c.held = append(c.held, [2]int{i, k})
			}
		}
	}
	if c.gate != "" {
		c.outcome = CellUndecided
		if len(c.held) > 0 {
			c.outcome = CellColliding
		}
		return
	}
	if len(c.held) > 0 {
		c.outcome = CellColliding
		for _, hit := range mp.Collisions {
			for _, pair := range c.held {
				i, k := pair[0], pair[1]
				if hit.Moving == r.Movers[i].Body && hit.Static == r.Partner(i, k) && b.blocks(c, i, k, hit.Volume) {
					c.outcome = CellBlocked
					return
				}
			}
		}
		return
	}
	var lowBound *big.Rat
	var low *[2]int
	readings := make(map[cellReadingKey]cornerBounds)
	lowest, ok := r.CertifyPairs(func(i, k int) *big.Rat {
		pp := mp.Pairs[i][k]
		if !pp.HasGap {
			return nil
		}
		// agent-docs/linkage-check-design.md §14.3: the pair's bound over the cell is
		// the larger of the travel bound lo_m − τ_half and the projection
		// bound L_n(C), two proven lower bounds on one gap.
		lo := proofarith.FloatRat(pp.Lo)
		bound := lo.Sub(lo, b.halfTravel(c, i, k))
		if proj, shares := b.cellProjection(c, i, k, readings); proj != nil && proj.Cmp(bound) > 0 {
			bound = proj
			if c.projShares == nil {
				c.projShares = make(map[[2]int]map[int]*big.Rat)
			}
			c.projShares[[2]int{i, k}] = shares
		}
		if bound.Sign() <= 0 {
			return nil
		}
		if lowBound == nil || bound.Cmp(lowBound) < 0 {
			lowBound, low = bound, &[2]int{i, k}
		}
		return bound
	}, func(i, k int) {
		if !r.Pairs[i][k].Evaluated() {
			c.stuck = true
		}
		c.held = append(c.held, [2]int{i, k})
	})
	if !ok {
		c.outcome = CellUndecided
		return
	}
	c.outcome, c.clearance = CellClear, motionbound.LowerBoundMeasurement(lowest)
	if lowBound != nil && lowBound.Cmp(lowest) == 0 {
		c.low = low
	}
}

// cellProjection is agent-docs/linkage-check-design.md §5.8's cell form for pair k
// of mover i over cell c (§14.3): each body's inflated box read at the
// centre's ideal poses under the joints on its relative path — the joints
// strictly below a link-link pair's lowest common ancestor, less a symmetric
// body's own joint (linkageDriver.pathOf, §5.2) — and expanded to
// second order in h_i, half of each joint's span across the cell rounded up
// to a float; the largest separation along the six coordinate directions. A
// loop's dependent joint is read at its centre, the midpoint of its
// enclosure at the cell's centre, with h_i the farthest that centre sits
// from the hull of the cell's hull and that enclosure (linkagebound.DependentCellReach), and
// its share is charged to its driver. It returns the bound and each axis's
// share of its defect, or nil when a dependent's readings are missing, as on
// a gated cell, or a box cannot be read exactly. readings
// keeps each mover's corner reading at the centre for the cell's other
// pairs.
func (b *boxRun[B]) cellProjection(c *boxCell[B], i, k int, readings map[cellReadingKey]cornerBounds) (*big.Rat, map[int]*big.Rat) {
	r, dr := b.run, b.dr
	other := r.Pairs[i][k].Other
	below := dr.pairDepth(i, other)
	mine := dr.pathOf(i, below)
	var theirs linkBound
	if other >= 0 {
		theirs = dr.pathOf(other, below)
	}
	side := func(m int, bound linkBound, hull bool) (projectionSide, bool) {
		h := make([]*big.Rat, 0, len(bound.Path)-below)
		for _, j := range bound.Path[below:] {
			jt := dr.spec.joints[j]
			var span *big.Rat
			if jt.dep == nil {
				span = jointParam(jt, c.lo[j]).SpanUpper(jointParam(jt, c.hi[j]))
				span.Quo(span, big.NewRat(2, 1))
			} else {
				reach, ok := c.depReach[j]
				if !ok {
					return projectionSide{}, false
				}
				span = new(big.Rat).Set(reach.Half)
			}
			half := proofarith.FloatRat(proofbound.RatFloatUp(span))
			if half == nil {
				return projectionSide{}, false
			}
			h = append(h, half)
		}
		key := cellReadingKey{mover: m, below: below, hull: hull}
		corners, ok := readings[key]
		if !ok {
			points, okPoints := motionengine.BodyPoints(r.Movers[m].Body, hull)
			if !okPoints {
				return projectionSide{}, false
			}
			params := make([]motionbound.MotionParam, len(dr.spec.joints))
			for _, j := range bound.Path[below:] {
				if dr.spec.joints[j].dep != nil {
					// A dependent is read at its centre, the midpoint of its
					// enclosure at the cell's centre (§5.8).
					params[j] = motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).Set(c.depReach[j].Centre)}
					continue
				}
				params[j] = jointParam(dr.spec.joints[j], c.centre(j))
			}
			if corners, ok = linkagebound.RoundCorners(readPoints(dr.spec, dr.frames, params, bound, below, points)); !ok {
				return projectionSide{}, false
			}
			readings[key] = corners
		}
		return projectionSide{Corners: corners, H: h, Rem: linkagebound.Remainder(bound.Rho[below:], h)}, true
	}
	partner := func(hull bool) (projectionSide, bool) {
		if other >= 0 {
			return side(other, theirs, hull)
		}
		corners, ok := b.staticReading(k, hull)
		return projectionSide{Corners: corners}, ok
	}
	axisOf := func(joint int) int {
		if ld := dr.spec.joints[joint].dep; ld != nil {
			return ld.driver
		}
		return joint
	}
	a, ok := side(i, mine, false)
	if !ok {
		return nil, nil
	}
	p, ok := partner(false)
	if !ok {
		return nil, nil
	}
	bound := linkagebound.Lower(a, p)
	shares := make(map[int]*big.Rat)
	axis, sense := linkagebound.AttainedDirection(a, p, bound)
	linkagebound.AddAxisShares(shares, a, mine.Rho[below:], linkagebound.ProjectionAxes(mine.Path, below, axisOf), axis, sense)
	if other >= 0 {
		linkagebound.AddAxisShares(shares, p, theirs.Rho[below:], linkagebound.ProjectionAxes(theirs.Path, below, axisOf), axis, -sense)
	}
	// The hull bound (§5.8): the same expansion over each body's hull points
	// along every candidate direction, the larger of the two serving.
	ah, okA := side(i, mine, true)
	ph, okP := partner(true)
	if !okA || !okP {
		return bound, shares
	}
	witness := linkagebound.LowerHullWithWitnessAbove(ah, ph, bound)
	hull := witness.Bound
	if hull == nil || hull.Cmp(bound) <= 0 {
		return bound, shares
	}
	shares = make(map[int]*big.Rat)
	linkagebound.AddHullShares(shares, ah, mine.Rho[below:], linkagebound.ProjectionAxes(mine.Path, below, axisOf),
		witness.Axis, witness.Norm, witness.Sense)
	if other >= 0 {
		linkagebound.AddHullShares(shares, ph, theirs.Rho[below:], linkagebound.ProjectionAxes(theirs.Path, below, axisOf),
			witness.Axis, witness.Norm, -witness.Sense)
	}
	return hull, shares
}

// cellReadingKey names one corner reading at a cell's centre: a mover's
// body under the joints on its path from position below on, read at its box
// corners or at its hull points (linkagebound.PrismHullPoints).
type cellReadingKey struct {
	mover, below int
	hull         bool
}

// staticReading is static body k's point reading — its hull points when hull
// is set and it has them, else its box corners — rounded outward, read once
// per call and kept.
func (b *boxRun[B]) staticReading(k int, hull bool) (cornerBounds, bool) {
	key := cellReadingKey{mover: k, below: -1, hull: hull}
	if got, ok := b.statics[key]; ok {
		return got, true
	}
	points, ok := motionengine.BodyPoints(b.run.Statics[k].Body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := linkagebound.RoundCorners(points)
	if !ok {
		return cornerBounds{}, false
	}
	if b.statics == nil {
		b.statics = make(map[cellReadingKey]cornerBounds)
	}
	b.statics[key] = reading
	return reading, true
}

// pairTerms is every joint's share of pair k of mover i's travel across cell
// c: against a static body, every joint on the mover's path; against a body
// of another link, the joints strictly below the two links' lowest common
// ancestor on each branch, each with its own body's ρ. A joint's span is
// motionbound.MotionParam.SpanUpper of its exact values at the cell's two
// ends; a joint that does not vary spans 0.
func (b *boxRun[B]) pairTerms(c *boxCell[B], i, k int) []linkagebound.JointTerm {
	mine, theirs := b.branchTerms(c, i, k)
	return append(mine, theirs...)
}

// branchTerms is pairTerms split by Body: the terms that move mover i's own
// body, and those that move its partner's — none for a static partner. A
// body symmetric about its own joint's axis takes its path without that
// joint (linkageDriver.pathOf, agent-docs/linkage-check-design.md §5.2).
//
// A loop's dependent joint j (agent-docs/linkage-check-design.md §16.3) moves at
// most δ_j from its exact value at the centre to its value anywhere in the
// cell, the whole one-sided distance and not half the hull, so its share is
// 2·w_j·δ_j — twice its part of τ_half, as a stated joint's w_i·span_i is —
// charged to its loop's driver, whose halving shrinks the hull. Only a cell
// with every δ_j read reaches here: a gated cell has no pair travel.
func (b *boxRun[B]) branchTerms(c *boxCell[B], i, k int) (mine, theirs []linkagebound.JointTerm) {
	r, dr := b.run, b.dr
	if r.Pairs[i][k].Once != nil {
		// A constant pair (agent-docs/linkage-check-design.md §5.9) moves nothing
		// relative to its Partner: no joint carries a term.
		return nil, nil
	}
	span := func(joint int) (int, *big.Rat) {
		jt := dr.spec.joints[joint]
		if jt.dep != nil {
			return jt.dep.driver, new(big.Rat).Mul(c.delta[joint], big.NewRat(2, 1))
		}
		return joint, jointParam(jt, c.lo[joint]).SpanUpper(jointParam(jt, c.hi[joint]))
	}
	other := r.Pairs[i][k].Other
	below := dr.pairDepth(i, other)
	if other < 0 {
		return linkagebound.TermsOnPath(dr.pathOf(i, below), below, span), nil
	}
	return linkagebound.TermsOnPath(dr.pathOf(i, below), below, span),
		linkagebound.TermsOnPath(dr.pathOf(other, below), below, span)
}

// halfTravel is τ_half(C) of agent-docs/linkage-check-design.md §14.3 for pair k of
// mover i: half the sum of its joint terms, a proven upper bound on how far
// the pair's distance changes from the cell's centre to any configuration of
// the cell, since no joint lies farther than half its span from the centre.
func (b *boxRun[B]) halfTravel(c *boxCell[B], i, k int) *big.Rat {
	return linkagebound.HalfSum(b.pairTerms(c, i, k))
}

// blocks reports whether pair k of mover i, which collides at cell c's centre
// with the published volume, overlaps at every configuration of the cell
// (agent-docs/linkage-check-design.md §14.3, the blocked certificate). Along the
// straight joint-space segment from the centre to any configuration of the
// cell a body moves at most its own τ_half, and the overlap volume changes at
// a rate no larger than each body's surface area times its speed, so it loses
// at most SweptVolumeAllow(τ_half, A) per moving body. The cell is blocked
// when the volume's proven lower end, Value − Bound rounded down, strictly
// exceeds that sum, compared over exact rationals. A is the mover's proven
// upper bound on its area at rest (motionengine.Mover.area), which a rigid motion
// preserves; a static partner moves nothing and is charged nothing.
func (b *boxRun[B]) blocks(c *boxCell[B], i, k int, volume vocab.Measurement) bool {
	r := b.run
	value, bound := proofarith.FloatRat(volume.Value.Base()), proofarith.FloatRat(volume.Bound.Base())
	if value == nil || bound == nil {
		return false
	}
	lower := proofbound.RatFloatDown(new(big.Rat).Sub(value, bound))
	mine, theirs := b.branchTerms(c, i, k)
	allow := proofbound.SweptVolumeAllow(proofbound.RatFloatUp(linkagebound.HalfSum(mine)), r.Movers[i].Area)
	if other := r.Pairs[i][k].Other; other >= 0 {
		allow = proofbound.AbsSumUpper(allow, proofbound.SweptVolumeAllow(proofbound.RatFloatUp(linkagebound.HalfSum(theirs)), r.Movers[other].Area))
	}
	if proofbound.IsNonFinite(allow) || proofbound.IsNonFinite(lower) {
		return false
	}
	return proofarith.FloatRat(lower).Cmp(proofarith.FloatRat(allow)) > 0
}

// splitAxis is the joint a cell splits along (agent-docs/linkage-check-design.md
// §14.4 step 5), or −1 when the cell is not splittable: a cell splits only
// when it is undecided by pairs a split can help, or colliding, and some
// varying joint's span is wider than the resolution. The axis is the varying
// joint, among those wider than the resolution, with the largest share of
// the defect of the bound of any pair that held the cell back — w_i·span_i
// of its travel bound, or its projection bound's own share where that bound
// is the larger (boundShares) — ties to the earliest link.
//
// A gated cell (agent-docs/linkage-check-design.md §16.4) has no pair travel to
// rank: it splits along the refusing loop's driver, the one axis a split
// changes, while that axis is wider than the resolution and its range meets
// some stretch the decomposition certified; otherwise it is stuck.
func (b *boxRun[B]) splitAxis(c *boxCell[B]) int {
	switch {
	case c.outcome == CellColliding:
	case c.outcome == CellUndecided && !c.stuck:
	default:
		return -1
	}
	if c.gate != "" {
		ld := c.gateLoop
		if ld == nil || ld.held || !b.wide(c, ld.driver) || !ld.meetsCertified(c.lo[ld.driver], c.hi[ld.driver]) {
			return -1
		}
		return ld.driver
	}
	best := make(map[int]*big.Rat, len(b.axes))
	projected := false
	for _, pair := range c.held {
		shares, proj := b.boundShares(c, pair[0], pair[1])
		projected = projected || proj
		for joint, v := range shares {
			if cur, ok := best[joint]; !ok || v.Cmp(cur) > 0 {
				best[joint] = v
			}
		}
	}
	return linkagebound.RankAxes(b.axes, best, projected, func(k int) bool { return b.wide(c, k) })
}

// wide reports whether cell c's span along joint k, as a fraction of the
// joint's range, is wider than the resolution.
func (b *boxRun[B]) wide(c *boxCell[B], k int) bool {
	width := new(big.Rat).Sub(c.hi[k], c.lo[k])
	return width.Cmp(b.run.Cfg.ResolutionP.Base) > 0
}

// wideForReading reports whether cell c's span along joint k is wider than
// the reading floor: its own when unstated, else the resolution.
func (b *boxRun[B]) wideForReading(c *boxCell[B], k int) bool {
	if b.run.Cfg.ReadingP == nil {
		return b.wide(c, k)
	}
	width := new(big.Rat).Sub(c.hi[k], c.lo[k])
	return width.Cmp(b.run.Cfg.ReadingP.Base) > 0
}

// configuration is the joint configuration a cell's centre was evaluated at:
// the zero JointConfiguration for an unbuildable centre.
func (c *boxCell[B]) configuration() JointConfiguration {
	if c.pose.Unbuildable != nil {
		return JointConfiguration{}
	}
	n := len(c.pose.Groups)
	conf := JointConfiguration{Values: make([]units.Value, n), Bounds: make([]units.Value, n), Poses: make([]r3.Transform, n)}
	for k, g := range c.pose.Groups {
		conf.Values[k], conf.Bounds[k], conf.Poses[k] = g.Value, g.Bound, g.Pose
	}
	return conf
}

// publish assembles VerifyJointBox's report (agent-docs/linkage-check-design.md
// §14.2, §14.4 step 8).
func (b *boxRun[B]) publish(l *Linkage[B], box JointBox[B]) *JointBoxReport[B] {
	r := b.run
	report := &JointBoxReport[B]{
		Request: JointBoxRequest{
			RelativeTolerance: units.Scalar(r.Cfg.Rel),
			Resolution:        r.Cfg.Resolution,
			MinClearance:      r.Cfg.Minimum,
			CellBudget:        b.budget,
		},
		ReadingResolution: readingResolution(r.Cfg),
		Linkage:           l,
		Box:               slices.Clone(box),
		Links:             l.Links(),
		Against:           []B{},
		JointContacts:     l.JointContacts(),
		Cells:             make([]JointCellResult[B], 0, len(b.leaves)),
		CellsEvaluated:    len(b.evaluated),
		Collisions:        []JointBoxCollision[B]{},
		Diagnostics:       []vocab.DiagnosticOf[B, vocab.JointCell]{},
	}
	for _, st := range r.Statics {
		report.Against = append(report.Against, b.h.Handle(st.Body))
	}
	poses := make([]*motionengine.Pose, len(b.evaluated))
	violated := false
	for n, c := range b.evaluated {
		poses[n] = c.pose
		violated = violated || c.pose.Violated
		for _, hit := range c.pose.Collisions {
			report.Collisions = append(report.Collisions, JointBoxCollision[B]{Configuration: c.configuration(), A: b.h.Handle(hit.Moving), B: b.h.Handle(hit.Static), Volume: hit.Volume})
		}
	}
	allClear, met := true, true
	var lowest *vocab.Measurement
	for _, c := range b.leaves {
		res := JointCellResult[B]{
			Cell:          c.cell.Clone(),
			Outcome:       c.outcome,
			Center:        c.configuration(),
			Interferences: b.h.lift().Interferences(c.pose.Result.Interferences),
			Clearances:    b.h.lift().Clearances(c.pose.Result.Clearances),
			Diagnostics:   b.h.lift().Diagnostics(c.pose.Result.Diagnostics),
			Clearance:     c.clearance,
		}
		var own []vocab.DiagnosticOf[B, vocab.JointCell]
		switch {
		case c.outcome != CellClear:
			allClear, met = false, false
			if c.outcome == CellUndecided {
				msg := fmt.Sprintf("the joint cell %s is neither certified clear nor bounded by a proven collision", formatCell(c.cell))
				if c.gate != "" {
					msg += ": " + c.gate
				}
				own = append(own, b.cellFinding(c, vocab.DiagnosticOf[B, vocab.JointCell]{
					Code:    vocab.DiagMotionUndecidedInterval,
					Status:  vocab.Suspect,
					Reading: vocab.ReadingNone,
					Message: msg,
				}))
			}
		case r.Cfg.MinimumMM != nil && !r.MeetsMinimum(c.clearance):
			met = false
			if violated {
				break
			}
			obs := *c.clearance
			own = append(own, b.cellFinding(c, vocab.DiagnosticOf[B, vocab.JointCell]{
				Code:     vocab.DiagMotionUndecidedClearance,
				Status:   vocab.Suspect,
				Reading:  vocab.ReadingGap,
				Observed: &obs,
				Required: r.Cfg.Minimum,
				Message:  fmt.Sprintf("the joint cell %s is certified clear, but its proven lower bound does not reach the required minimum", formatCell(c.cell)),
			}))
		}
		if c.clearance != nil && (lowest == nil || c.clearance.Value.Base() < lowest.Value.Base()) {
			lowest = c.clearance
		}
		res.Diagnostics = append(res.Diagnostics, own...)
		report.Cells = append(report.Cells, res)
		report.Diagnostics = append(append(report.Diagnostics, b.h.lift().Diagnostics(c.pose.Findings)...), own...)
	}
	// A split cell's centre is gone from Cells, but a collision or a margin
	// violation proven there is a fact the caller asked for.
	for _, c := range b.evaluated {
		if !c.split {
			continue
		}
		for _, diag := range c.pose.Findings {
			if diag.Code == vocab.DiagMotionCollision || diag.Code == vocab.DiagMotionClearanceViolated {
				report.Diagnostics = append(report.Diagnostics, b.h.lift().Diagnostics([]motionengine.Diagnostic{diag})...)
			}
		}
	}
	if b.exhausted {
		report.Diagnostics = append(report.Diagnostics, vocab.DiagnosticOf[B, vocab.JointCell]{
			Code:    vocab.DiagJointBoxBudgetExhausted,
			Status:  vocab.Suspect,
			Reading: vocab.ReadingNone,
			Message: fmt.Sprintf("the cell budget of %d centres stopped the subdivision with %d cells left unsplit that its floors would still have split", b.budget, b.unsplit),
		})
	}
	switch {
	case r.Cfg.MinimumMM == nil:
		report.Assessment = vocab.AssessmentNotEvaluated
	case violated:
		report.Assessment = vocab.AssessmentViolated
	case met:
		report.Assessment = vocab.AssessmentMet
	default:
		report.Assessment = vocab.AssessmentUndecided
	}
	if allClear && lowest != nil {
		if reading, diag := r.PathClearance(poses, lowest, "whole-box"); reading != nil {
			report.Clearance = reading
			if diag != nil {
				report.Diagnostics = append(report.Diagnostics, b.h.lift().Diagnostics([]motionengine.Diagnostic{*diag})...)
			}
		}
	}
	report.Status = motionconclude.WorstStatus(report.Diagnostics)
	return report
}

// cellFinding marks a cell's own finding with the cell.
func (b *boxRun[B]) cellFinding(c *boxCell[B], diag vocab.DiagnosticOf[B, vocab.JointCell]) vocab.DiagnosticOf[B, vocab.JointCell] {
	cell := c.cell.Clone()
	diag.Cell = &cell
	return diag
}

// formatValues renders joint values for a finding's message.
func formatValues(values []units.Value) string {
	parts := make([]string, len(values))
	for k, v := range values {
		parts[k] = v.String()
	}
	return "(" + strings.Join(parts, ", ") + ")"
}

// formatCell renders a cell for a finding's message, one range per link.
func formatCell(c vocab.JointCell) string {
	parts := make([]string, len(c.Min))
	for k := range c.Min {
		parts[k] = fmt.Sprintf("[%s, %s]", c.Min[k], c.Max[k])
	}
	return strings.Join(parts, " × ")
}

// errLoopNotStated is the refusal of a looped linkage where every joint's
// value is the caller's to state (agent-docs/linkage-check-design.md §16.1).
func errLoopNotStated() error {
	return fmt.Errorf(`%w: a linkage with a closed loop has dependent joints whose values follow from its driver, so it cannot be posed joint by joint; pose a drive whose every sweep holds instead`, decaderr.ErrUnsupported)
}
