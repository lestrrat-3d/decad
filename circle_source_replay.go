package decad

import (
	"context"
	"math"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

const circlePlacementChainLimit = 64

// circleSourceSnapshot follows only ordered Transform producers to an
// ordinary Extrude. It does not authorize tessellation or a Boolean.
type circleSourceSnapshot struct {
	budget        *proofbound.ReplayBudget
	doc           *Document
	root, base    *Body
	baseOperation *Operation
	baseFaces     *prismrecord.CircleBuildFacesSnapshot
	steps         []*circlePlacementSnapshot
}

// liftRoundBound covers only floating-point evaluation of the recorded frame
// and held placement. Circle sampling, source displacement and earlier Then
// compositions remain separate charges.
func (s *circleSourceSnapshot) liftRoundBound(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) (float64, error) {
	if err := s.revalidate(ctx, d, budget); err != nil {
		return 0, err
	}
	if err := budget.SpendWork(ctx, 4096); err != nil {
		return 0, err
	}
	p, ok := s.root.n.Payload.(prismrecord.Payload)
	if !ok || len(p.Profile.Outer.Segments) != 1 || p.Walks == nil || len(p.Walks.Outer) != 1 {
		return 0, errCirclePlacementReplay
	}
	c, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return 0, errCirclePlacementReplay
	}
	w := p.Walks.Outer[0]
	if proofbound.IsNonFinite(w.RadiusBound) || w.RadiusBound < 0 {
		return 0, errCirclePlacementReplay
	}
	radius := proofbound.AbsSumUpper(w.Radius, w.RadiusBound)
	u := proofbound.AbsSumUpper(c.Center.U, radius)
	v := proofbound.AbsSumUpper(c.Center.V, radius)
	z := math.Max(math.Abs(p.Z0), math.Abs(p.Z1))
	maxCoordinate := math.Max(u, math.Max(v, z))
	frameRound := proofbound.RigidRoundAllow(maxCoordinate, circleMaxAbs(p.Frame.Origin()))
	local := proofbound.AbsSumUpper(circleMaxAbs(p.Frame.Origin()),
		proofbound.ProductUpper(circleMaxAbs(p.Frame.U()), u),
		proofbound.ProductUpper(circleMaxAbs(p.Frame.V()), v),
		proofbound.ProductUpper(circleMaxAbs(p.Frame.N()), z), frameRound)
	placementRound := proofbound.RigidRoundAllow(local, circleMaxAbs(p.Xform.Translation()))
	// A validated rigid placement has norm below 2, including its accepted
	// floating-point orthogonality tolerance.
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(2, frameRound), placementRound)
	if proofbound.IsNonFinite(bound) {
		return 0, errCirclePlacementReplay
	}
	return bound, nil
}

func captureCircleSource(ctx context.Context, d *Document, root *Body, budget *proofbound.ReplayBudget) (*circleSourceSnapshot, error) {
	if d == nil || root == nil || budget == nil || !budget.IsCircleReplay() {
		return nil, errCirclePlacementReplay
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1+circlePlacementChainLimit); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(circleSourceSnapshot{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, circlePlacementChainLimit, uint64(unsafe.Sizeof((*circlePlacementSnapshot)(nil)))); err != nil {
		return nil, err
	}
	s := &circleSourceSnapshot{budget: budget, doc: d, root: root, steps: make([]*circlePlacementSnapshot, 0, circlePlacementChainLimit)}
	current := root
	for {
		if err := budget.SpendWork(ctx, 64); err != nil {
			return nil, err
		}
		if current == nil || current.n.Doc != &d.n {
			return nil, errCirclePlacementReplay
		}
		d.n.LiveMu.RLock()
		op := d.producers[current]
		valid := op != nil && op.doc == &d.n && len(op.outputs) == 1 && op.outputs[0] == current
		var kind OperationKind
		if valid {
			kind = op.kind
		}
		d.n.LiveMu.RUnlock()
		if !valid {
			return nil, errCirclePlacementReplay
		}
		if kind == OperationExtrude {
			if len(op.inputs) != 0 {
				return nil, errCirclePlacementReplay
			}
			faces, err := prismrecord.CaptureCircleBuildFaces(ctx, &current.n, d.circleBuilds[current], budget)
			if err != nil {
				return nil, err
			}
			s.base, s.baseOperation, s.baseFaces = current, op, faces
			break
		}
		if kind != OperationTransform || len(s.steps) >= circlePlacementChainLimit {
			return nil, errCirclePlacementReplay
		}
		step, err := captureCirclePlacement(ctx, d, current, budget)
		if err != nil {
			return nil, err
		}
		for _, prior := range s.steps {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if prior.placed == step.source {
				return nil, errCirclePlacementReplay
			}
		}
		s.steps = append(s.steps, step)
		current = step.source
	}
	if err := s.revalidate(ctx, d, budget); err != nil {
		return nil, err
	}
	return s, nil
}

func (s *circleSourceSnapshot) revalidate(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) error {
	if s == nil || d == nil || d != s.doc || budget == nil || budget != s.budget || s.root == nil || s.base == nil {
		return errCirclePlacementReplay
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return err
	}
	current := s.root
	for _, step := range s.steps {
		if step == nil || step.placed != current {
			return errCirclePlacementReplay
		}
		if err := step.revalidate(ctx, d, budget); err != nil {
			return err
		}
		current = step.source
	}
	if current != s.base {
		return errCirclePlacementReplay
	}
	d.n.LiveMu.RLock()
	op := d.producers[s.base]
	valid := op != nil && op == s.baseOperation && op.doc == &d.n && op.kind == OperationExtrude &&
		len(op.inputs) == 0 && len(op.outputs) == 1 && op.outputs[0] == s.base
	d.n.LiveMu.RUnlock()
	if !valid {
		return errCirclePlacementReplay
	}
	_, err := s.baseFaces.Faces(ctx, &s.base.n, budget)
	if err != nil {
		return err
	}
	if s.base.n.Payload == nil || !sameCircleTransform(s.base.n.Payload.Transform(), r3.Identity()) {
		return errCirclePlacementReplay
	}
	return nil
}

// compositionBound combines only the recorded Then rounding; the frame lift
// and all tessellation error remain separate proof obligations.
func (s *circleSourceSnapshot) compositionBound(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) (float64, error) {
	if err := s.revalidate(ctx, d, budget); err != nil {
		return 0, err
	}
	var total float64
	for _, placement := range slices.Backward(s.steps) {
		step, err := placement.compositionBound(ctx, d, budget)
		if err != nil {
			return 0, err
		}
		// Valid rigid placements have operator norm below 2, including their
		// accepted floating-point orthogonality tolerance.
		total = proofbound.AbsSumUpper(proofbound.ProductUpper(2, total), step)
		if proofbound.IsNonFinite(total) {
			return 0, errCirclePlacementReplay
		}
	}
	return total, nil
}

// linearCompositionBound bounds the difference between the exact chain's
// linear map and its held composed matrix. It excludes the source frame.
func (s *circleSourceSnapshot) linearCompositionBound(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) (float64, error) {
	if err := s.revalidate(ctx, d, budget); err != nil {
		return 0, err
	}
	var total float64
	for _, placement := range slices.Backward(s.steps) {
		step, err := placement.linearCompositionBound(ctx, d, budget)
		if err != nil {
			return 0, err
		}
		total = proofbound.AbsSumUpper(proofbound.ProductUpper(2, total), step)
		if proofbound.IsNonFinite(total) {
			return 0, errCirclePlacementReplay
		}
	}
	return total, nil
}
