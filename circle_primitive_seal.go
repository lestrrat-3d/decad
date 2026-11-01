package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// circlePrimitiveSeal keeps a construction-authenticated source attached to
// its original meter. The consumer must recheck it around each replay stage.
type circlePrimitiveSeal interface {
	circleReplayView(context.Context, *proofbound.ReplayBudget) (circlePrimitiveView, error)
	circleReplayCheck(context.Context, *proofbound.ReplayBudget) error
	circleReplayAllowance(context.Context, *proofbound.ReplayBudget) (float64, float64, error)
}

// withCirclePrimitiveCandidate keeps the candidate within a synchronous
// private use. The caller must not retain it without a separate mesh seal.
func withCirclePrimitiveCandidate[T any](ctx context.Context, source circlePrimitiveSeal, chord float64,
	budget *proofbound.ReplayBudget, use func(*topo.Mesh, [3]*topo.Face, *proofbound.ReplayBudget) (T, error)) (T, error) {
	var zero T
	if source == nil || budget == nil || use == nil {
		return zero, errCirclePlacementReplay
	}
	displacement, linear, err := source.circleReplayAllowance(ctx, budget)
	if err != nil {
		return zero, err
	}
	view, err := source.circleReplayView(ctx, budget)
	if err != nil {
		return zero, err
	}
	local, lift, charge, err := massmoment.ReplayPrismChord(ctx, view.payload.Frame, view.payload.Xform, chord, budget)
	if err != nil {
		return zero, err
	}
	if err = budget.SpendWork(ctx, 32); err != nil {
		return zero, err
	}
	candidateLift := lift
	if linear != 0 || displacement != 0 {
		if !(displacement < chord) {
			return zero, errCirclePlacementReplay
		}
		remaining := math.Nextafter(chord-displacement, 0)
		candidateLift = proofbound.AbsSumUpper(lift, linear)
		local = math.Nextafter(remaining/candidateLift, 0)
		if !(local > 0) || proofbound.IsNonFinite(local) || proofbound.IsNonFinite(candidateLift) {
			return zero, errCirclePlacementReplay
		}
	}
	if err = source.circleReplayCheck(ctx, budget); err != nil {
		return zero, err
	}
	samples, err := tessellation.CircleSamplesReplay(ctx, view.segment, view.faces[0], local, view.payload.Z1-view.payload.Z0, budget)
	if err != nil {
		return zero, err
	}
	if err = source.circleReplayCheck(ctx, budget); err != nil {
		return zero, err
	}
	mesh, err := prismtess.CircleCandidateReplay(ctx, view.body, view.payload, view.segment, view.faces, samples, candidateLift, charge, budget)
	if err != nil {
		return zero, err
	}
	if err = source.circleReplayCheck(ctx, budget); err != nil {
		return zero, err
	}
	if err = widenCirclePrimitive(ctx, view, mesh, lift, displacement, linear, budget); err != nil {
		return zero, err
	}
	if proofbound.IsNonFinite(mesh.Bound) || mesh.Bound > chord || proofbound.IsNonFinite(mesh.CoordBound) || mesh.CoordBound > chord {
		return zero, errCirclePlacementReplay
	}
	if err = budget.SpendWork(ctx, uint64(len(mesh.VertexBound))*8); err != nil {
		return zero, err
	}
	for i, bound := range mesh.VertexBound {
		if i%128 == 0 {
			if err := ctx.Err(); err != nil {
				return zero, err
			}
		}
		if proofbound.IsNonFinite(bound) || bound > chord {
			return zero, errCirclePlacementReplay
		}
	}
	for _, face := range view.faces {
		bound, ok, err := mesh.ReplayFaceBounds.Lookup(ctx, face, budget)
		if err != nil {
			return zero, err
		}
		if !ok || proofbound.IsNonFinite(bound) || bound > chord {
			return zero, errCirclePlacementReplay
		}
	}
	result, err := use(mesh, view.faces, budget)
	if err != nil {
		return zero, err
	}
	if err = source.circleReplayCheck(ctx, budget); err != nil {
		return zero, err
	}
	return result, nil
}

func (s *circleSourceSnapshot) circleReplayAllowance(ctx context.Context, budget *proofbound.ReplayBudget) (float64, float64, error) {
	if err := s.circleReplayCheck(ctx, budget); err != nil {
		return 0, 0, err
	}
	if len(s.steps) == 0 {
		return 0, 0, nil
	}
	return s.exactAffineAllowance(ctx, budget)
}

// widenCirclePrimitive composes historical placement rounding. The exact
// affine path from the held map to the recorded chain has linear error δ.
// Its area changes by at most δ(2L+δ) times local area, and every point moves
// at most displacement. The Gram check in ReplayPrismChord proves the held
// map's least stretch exceeds 1/2; δ < 1/4 keeps the path nonsingular.
func widenCirclePrimitive(ctx context.Context, view circlePrimitiveView, mesh *topo.Mesh, lift, displacement, linear float64, b *proofbound.ReplayBudget) error {
	if displacement == 0 && linear == 0 {
		return nil
	}
	if err := b.SpendWork(ctx, uint64(4096+8*len(mesh.VertexBound))); err != nil {
		return err
	}
	delta := linear
	if proofbound.IsNonFinite(delta) || delta >= .25 || displacement < 0 || proofbound.IsNonFinite(displacement) {
		return errCirclePlacementReplay
	}
	radius, err := view.segment.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || proofbound.IsNonFinite(radius) {
		return errCirclePlacementReplay
	}
	height := math.Nextafter(math.Abs(view.payload.Z1-view.payload.Z0), math.Inf(1))
	area := proofbound.ProductUpper(proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), proofbound.AbsSumUpper(radius, height))
	diff := proofbound.ProductUpper(proofbound.ProductUpper(delta, proofbound.AbsSumUpper(proofbound.ProductUpper(2, lift), delta)), area)
	pathLift := proofbound.AbsSumUpper(lift, delta)
	swept := proofbound.ProductUpper(displacement, proofbound.ProductUpper(proofbound.ProductUpper(pathLift, pathLift), area))
	if proofbound.IsNonFinite(diff) || proofbound.IsNonFinite(swept) {
		return errCirclePlacementReplay
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, diff)
	mesh.VolSymDiff = proofbound.AbsSumUpper(mesh.VolSymDiff, swept)
	mesh.CoordBound = proofbound.AbsSumUpper(mesh.CoordBound, displacement)
	mesh.Bound = proofbound.AbsSumUpper(mesh.Bound, displacement)
	for i := range mesh.VertexBound {
		if i%128 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		mesh.VertexBound[i] = proofbound.AbsSumUpper(mesh.VertexBound[i], displacement)
		if proofbound.IsNonFinite(mesh.VertexBound[i]) {
			return errCirclePlacementReplay
		}
	}
	for _, face := range view.faces {
		old, ok, err := mesh.ReplayFaceBounds.Lookup(ctx, face, b)
		if err != nil {
			return err
		}
		if !ok {
			return errCirclePlacementReplay
		}
		if err := mesh.ReplayFaceBounds.Set(ctx, face, proofbound.AbsSumUpper(old, displacement), b); err != nil {
			return err
		}
	}
	if proofbound.IsNonFinite(mesh.AreaSlack) || proofbound.IsNonFinite(mesh.VolSymDiff) || proofbound.IsNonFinite(mesh.CoordBound) || proofbound.IsNonFinite(mesh.Bound) {
		return errCirclePlacementReplay
	}
	return ctx.Err()
}

type circlePrimitiveView struct {
	body    *topo.Body
	payload prismrecord.Payload
	segment sectionrecord.CircleSeg
	walk    survey2d.SegmentWalk
	faces   [3]*topo.Face
}

func (s *circleSourceSnapshot) circleReplayCheck(ctx context.Context, budget *proofbound.ReplayBudget) error {
	if s == nil {
		return errCirclePlacementReplay
	}
	return s.revalidate(ctx, s.doc, budget)
}

func (s *circleSourceSnapshot) circleReplayView(ctx context.Context, budget *proofbound.ReplayBudget) (circlePrimitiveView, error) {
	if err := s.circleReplayCheck(ctx, budget); err != nil {
		return circlePrimitiveView{}, err
	}
	faces := s.baseFaces
	if len(s.steps) != 0 {
		faces = s.steps[0].placedFaces
	}
	boundFaces, err := faces.Faces(ctx, &s.root.n, budget)
	if err != nil {
		return circlePrimitiveView{}, err
	}
	payload, ok := s.root.n.Payload.(prismrecord.Payload)
	if !ok || len(payload.Profile.Outer.Segments) != 1 || payload.Walks == nil || len(payload.Walks.Outer) != 1 {
		return circlePrimitiveView{}, errCirclePlacementReplay
	}
	segment, ok := payload.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return circlePrimitiveView{}, errCirclePlacementReplay
	}
	return circlePrimitiveView{body: &s.root.n, payload: payload, segment: segment, walk: payload.Walks.Outer[0], faces: boundFaces}, nil
}
