package decad

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// circlePlacementWitness records what Transform actually passed to Placed.
type circlePlacementWitness struct {
	source, placed   *Body
	motion, composed r3.Transform
}

type circlePlacementSnapshot struct {
	budget                   *proofbound.ReplayBudget
	witness                  *circlePlacementWitness
	operation                *Operation
	source, placed           *Body
	motion, composed         r3.Transform
	sourceFaces, placedFaces *prismrecord.CircleBuildFacesSnapshot
}

var errCirclePlacementReplay = fmt.Errorf("%w: circle placement evidence changed", ErrUnsupported)

func captureCirclePlacement(ctx context.Context, d *Document, placed *Body, budget *proofbound.ReplayBudget) (*circlePlacementSnapshot, error) {
	if d == nil || placed == nil || budget == nil || !budget.IsCircleReplay() {
		return nil, errCirclePlacementReplay
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	d.n.LiveMu.RLock()
	w := d.circlePlacements[placed]
	d.n.LiveMu.RUnlock()
	if w == nil || w.source == nil || w.placed != placed {
		return nil, errCirclePlacementReplay
	}
	a, err := prismrecord.CaptureCircleBuildFaces(ctx, &w.source.n, d.circleBuilds[w.source], budget)
	if err != nil {
		return nil, err
	}
	b, err := prismrecord.CaptureCircleBuildFaces(ctx, &placed.n, d.circleBuilds[placed], budget)
	if err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(circlePlacementSnapshot{}))); err != nil {
		return nil, err
	}
	d.n.LiveMu.RLock()
	op := d.producers[placed]
	d.n.LiveMu.RUnlock()
	s := &circlePlacementSnapshot{budget: budget, witness: w, operation: op, source: w.source, placed: placed,
		motion: w.motion, composed: w.composed, sourceFaces: a, placedFaces: b}
	if err := s.revalidate(ctx, d, budget); err != nil {
		return nil, err
	}
	return s, nil
}

// revalidate authenticates held composition and history, not its rounding bound.
func (s *circlePlacementSnapshot) revalidate(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) error {
	if s == nil || d == nil || budget == nil || budget != s.budget || s.source == nil || s.placed == nil || s.witness == nil {
		return errCirclePlacementReplay
	}
	if err := budget.SpendWork(ctx, 1024); err != nil {
		return err
	}
	w := s.witness
	if d.circlePlacements[s.placed] != w || w.source != s.source || w.placed != s.placed ||
		!sameCircleTransform(w.motion, s.motion) || !sameCircleTransform(w.composed, s.composed) ||
		s.source.n.Doc != &d.n || s.placed.n.Doc != &d.n || s.source.n.Payload == nil || s.placed.n.Payload == nil {
		return errCirclePlacementReplay
	}
	d.n.LiveMu.RLock()
	op := d.producers[s.placed]
	valid := op != nil && op == s.operation && op.doc == &d.n && op.kind == OperationTransform &&
		len(op.inputs) == 1 && op.inputs[0].Body == s.source && op.inputs[0].Role == operationRoleSource &&
		op.inputs[0].Use == InputConsumed && len(op.outputs) == 1 && op.outputs[0] == s.placed
	d.n.LiveMu.RUnlock()
	if !valid {
		return errCirclePlacementReplay
	}
	equal, err := prismrecord.SameCircleSource(ctx, s.sourceFaces, s.placedFaces, &s.source.n, &s.placed.n, budget)
	if err != nil {
		return err
	}
	if !equal {
		return errCirclePlacementReplay
	}
	composed, err := s.source.n.Payload.Transform().Then(s.motion)
	if err != nil || !sameCircleTransform(composed, s.composed) || !sameCircleTransform(s.placed.n.Payload.Transform(), s.composed) {
		return errCirclePlacementReplay
	}
	return nil
}

func sameCircleTransform(a, b r3.Transform) bool {
	av, bv := a.Basis(), b.Basis()
	return sameCircleTransformVec(a.Translation(), b.Translation()) && sameCircleTransformVec(av.EX, bv.EX) &&
		sameCircleTransformVec(av.EY, bv.EY) && sameCircleTransformVec(av.EZ, bv.EZ)
}

func sameCircleTransformVec(a, b r3.Vec) bool {
	return math.Float64bits(a.X) == math.Float64bits(b.X) && math.Float64bits(a.Y) == math.Float64bits(b.Y) &&
		math.Float64bits(a.Z) == math.Float64bits(b.Z)
}

// compositionBound encloses only the rounding committed by this Then call.
// Earlier placement steps and the frame lift need their own bounds.
func (s *circlePlacementSnapshot) compositionBound(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) (float64, error) {
	if err := s.revalidate(ctx, d, budget); err != nil {
		return 0, err
	}
	if err := budget.SpendWork(ctx, 4096); err != nil {
		return 0, err
	}
	p, ok := s.source.n.Payload.(prismrecord.Payload)
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
	local := proofbound.AbsSumUpper(circleMaxAbs(p.Frame.Origin()),
		proofbound.ProductUpper(circleMaxAbs(p.Frame.U()), u),
		proofbound.ProductUpper(circleMaxAbs(p.Frame.V()), v),
		proofbound.ProductUpper(circleMaxAbs(p.Frame.N()), z))
	basis := p.Xform.Basis()
	input := math.Max(circleMaxAbs(basis.EX), math.Max(circleMaxAbs(basis.EY), circleMaxAbs(basis.EZ)))
	column := proofbound.RigidRoundAllow(input, 0)
	translation := proofbound.RigidRoundAllow(circleMaxAbs(p.Xform.Translation()), circleMaxAbs(s.motion.Translation()))
	bound := proofbound.AbsSumUpper(translation, proofbound.ProductUpper(proofbound.ProductUpper(3, local), column))
	if proofbound.IsNonFinite(bound) {
		return 0, errCirclePlacementReplay
	}
	return bound, nil
}

func circleMaxAbs(v r3.Vec) float64 {
	return math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
}

// linearCompositionBound bounds the operator norm of this Then call's
// linear-part rounding. It does not include earlier placement steps.
func (s *circlePlacementSnapshot) linearCompositionBound(ctx context.Context, d *Document, budget *proofbound.ReplayBudget) (float64, error) {
	if err := s.revalidate(ctx, d, budget); err != nil {
		return 0, err
	}
	if err := budget.SpendWork(ctx, 1024); err != nil {
		return 0, err
	}
	basis := s.source.n.Payload.Transform().Basis()
	input := math.Max(circleMaxAbs(basis.EX), math.Max(circleMaxAbs(basis.EY), circleMaxAbs(basis.EZ)))
	// RigidRoundAllow bounds each of the three column errors in 3D. Their
	// sum bounds the difference matrix's Frobenius and operator norms.
	bound := proofbound.ProductUpper(3, proofbound.RigidRoundAllow(input, 0))
	if proofbound.IsNonFinite(bound) {
		return 0, errCirclePlacementReplay
	}
	return bound, nil
}
