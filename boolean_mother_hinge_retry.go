package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"reflect"
	"slices"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type motherHingeAncestry struct {
	target, tool, leaf, knuckle *Body
	union                       *Operation
	certificate                 *booleaneval.ShiftCertificate
	snapshot                    *booleaneval.ShiftCertificateSnapshot
	privateUnionSnapshot        *booleaneval.ShiftCertificateSnapshot
	privateUnionTarget          *Body
	privateUnionTool            *Body
	budget                      *proofbound.ReplayBudget
	separation                  motherHingeSeparation
}

type motherHingeSeparation struct {
	leafBox, boreBox booleaneval.ReplayBox
	leafFaces        []*topo.Face
	boreFaces        []*topo.Face
	leafBounds       measurement.Box
	boreBounds       measurement.Box
	chord            float64
}

func motherHingeCutContinuation(ctx context.Context, d *Document, op meshbool.OperationKind, target, bore *Body,
	cfg booleanConfig, cause error) (*Body, producerID, motherHingeAncestry, bool, bool, error) {
	var empty motherHingeAncestry
	if op != meshbool.OpCut || !coarseTargetCut(op, cfg, cause) || ctx.Value(operationHistorySuppressedKey{}) != nil {
		return nil, 0, empty, false, false, nil
	}
	ancestry, ok := motherHingeSources(d.OperationHistory(), target, bore)
	if !ok {
		return nil, 0, empty, false, false, nil
	}
	cert := ancestry.certificate
	warning := target.n.OperationWarnings[0]
	budget := proofbound.NewBrakeReplayBudget()
	snapshot, err := booleaneval.SnapshotShiftCertificate(ctx, budget, cert)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, false, false, ctx.Err()
		}
		return nil, 0, empty, false, false, nil
	}
	eval, err := booleaneval.AuthenticateStoredUnionShrinkSnapshot(ctx, snapshot,
		&target.n, &ancestry.leaf.n, &ancestry.knuckle.n, warning, budget, booleanHooks(cfg).Mesher)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, false, false, ctx.Err()
		}
		return nil, 0, empty, false, false, nil
	}
	separation, ok, err := proveMotherHingeSeparation(ctx, ancestry.leaf, bore, budget)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, false, false, ctx.Err()
		}
		return nil, 0, empty, false, false, nil
	}
	if !ok {
		return nil, 0, empty, false, false, nil
	}
	base := d.nextProducerID()
	const span producerID = 3
	if base < 0 || base > producerID(math.MaxInt)-span {
		return nil, 0, empty, false, false, nil
	}
	evalCtx, err := d.evalContext(ctx)
	if err != nil {
		return nil, 0, empty, false, false, err
	}
	adjustedNode, err := buildMotherHingeAdjustedPrism(evalCtx, envOf(documentNode(d), base), ancestry.knuckle,
		eval.SimilarityCenter, eval.SimilarityScale, eval.ShiftMove, eval.HasSimilarity, budget)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, false, false, ctx.Err()
		}
		return nil, 0, empty, false, false, nil
	}
	if !motherHingeVolumeIntervalsMeet(adjustedNode, eval.Volume, eval.VolumeBound) {
		return nil, 0, empty, false, false, nil
	}
	if err := remapMotherHingePrismFaces(adjustedNode, ancestry.knuckle); err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, false, false, ctx.Err()
		}
		return nil, 0, empty, false, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, 0, empty, false, false, err
	}
	adjusted := bodyHandle(adjustedNode)
	hooks := booleanHooks(cfg)
	privateStep := func(op meshbool.OperationKind, a, b *Body, ref producerID, chordScale float64) (*topo.Body, error) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		n, err := booleandispatch.BodyScaled(evalCtx, op, envOf(documentNode(d), ref), &a.n, &b.n, hooks, chordScale)
		if err != nil {
			return nil, err
		}
		if len(n.OperationWarnings) != 0 {
			cert, ok := n.PrivateShift.(*booleaneval.ShiftCertificate)
			if op != meshbool.OpUnion || !ok ||
				!cert.SealPrivateUnionAdjustmentWarning(n.Origin.Producer, &a.n, &b.n, n.OperationWarnings) {
				return nil, fmt.Errorf("%w: private mother-hinge Boolean produced an unauthenticated geometry warning", ErrUnsupported)
			}
		}
		return n, ctx.Err()
	}
	cutNode, err := privateStep(meshbool.OpCut, adjusted, bore, base+1, 0.75)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	cut := bodyHandle(cutNode)
	cutLineage, ok := motherHingeCutAncestry(adjusted, cut, ancestry, bore)
	if !ok {
		return nil, 0, empty, true, false, cause
	}
	joinedNode, err := privateStep(meshbool.OpUnion, ancestry.leaf, cut, base+2, 1)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	joined := bodyHandle(joinedNode)
	privateUnionCert, ok := joinedNode.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok || !privateUnionCert.IsSingleComponentAdjustment() ||
		len(joinedNode.OperationWarnings) != 1 {
		return nil, 0, empty, true, false, cause
	}
	privateUnionBudget := budget
	privateUnionSnapshot, err := booleaneval.SnapshotShiftCertificate(ctx, privateUnionBudget, privateUnionCert)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	privateWarning := joinedNode.OperationWarnings[0]
	privateEval, err := booleaneval.AuthenticatePrivateUnionAdjustmentSnapshot(ctx, privateUnionSnapshot,
		joinedNode, &ancestry.leaf.n, cutNode, privateWarning, privateUnionBudget, hooks.Mesher)
	if err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	if !privateEval.HasSimilarity || !motherHingeCutVolumeConsistent(target, joinedNode, adjustedNode, cutNode,
		eval.Payload.VolSymDiff, privateEval.Payload.VolSymDiff) {
		return nil, 0, empty, true, false, cause
	}
	if err := remapMotherHingeAncestry(joined, cut, ancestry, bore, cutLineage); err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	if err := verifyMotherHingeResult(ctx, joined, target, bore); err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	separationNow, ok, err := proveMotherHingeSeparation(ctx, ancestry.leaf, bore, budget)
	if err != nil || !ok || !sameMotherHingeSeparation(separation, separationNow) {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	if err := snapshot.Revalidate(ctx, budget, &target.n, &ancestry.leaf.n, &ancestry.knuckle.n); err != nil {
		if ctx.Err() != nil {
			return nil, 0, empty, true, false, ctx.Err()
		}
		return nil, 0, empty, true, false, cause
	}
	ancestry.certificate, ancestry.snapshot, ancestry.budget = cert, snapshot, budget
	ancestry.privateUnionSnapshot = privateUnionSnapshot
	ancestry.privateUnionTarget, ancestry.privateUnionTool = ancestry.leaf, cut
	ancestry.separation = separation
	return joined, span, ancestry, true, true, nil
}

func commitMotherHingeCut(ctx context.Context, d *Document, target, bore, body *Body,
	span, base producerID, ancestry motherHingeAncestry, original error) error {
	inputs := operationInputsForBoolean(meshbool.OpCut, target, bore)
	operation := d.operation(OperationCut, inputs, body)
	if err := d.validateOperation(operation, []*Body{target, bore}); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if err := ctx.Err(); err != nil {
		return err
	}
	if !slices.Contains(d.n.LiveSet, &target.n) || !slices.Contains(d.n.LiveSet, &bore.n) || d.n.NextProducer != base {
		return original
	}
	h := &OperationHistory{doc: &d.n, operations: d.operations, producers: d.producers, consumers: d.consumers}
	current, ok := motherHingeSources(h, target, bore)
	if !ok || !sameMotherHingeAncestry(current, ancestry) ||
		!sameMotherSourceOwners(ancestry.separation, ancestry.leaf, bore) {
		return original
	}
	if ancestry.snapshot == nil || ancestry.budget == nil ||
		!booleaneval.StoredUnionShrinkWarningMatches(&target.n, ancestry.certificate,
			target.n.OperationWarnings[0]) {
		return original
	}
	if err := ancestry.snapshot.Revalidate(ctx, ancestry.budget, &target.n, &ancestry.leaf.n, &ancestry.knuckle.n); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return original
	}
	if ancestry.privateUnionSnapshot == nil || ancestry.privateUnionTarget != ancestry.leaf || ancestry.privateUnionTool == nil ||
		len(body.n.OperationWarnings) != 1 {
		return original
	}
	privateWarning := body.n.OperationWarnings[0]
	if err := ancestry.privateUnionSnapshot.Revalidate(ctx, ancestry.budget, &body.n,
		&ancestry.privateUnionTarget.n, &ancestry.privateUnionTool.n); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return original
	}
	if !booleaneval.StoredPrivateUnionAdjustmentWarningMatches(&body.n, ancestry.privateUnionSnapshot,
		privateWarning) {
		return original
	}
	d.n.NextProducer += span
	dest := d.destinationLocked(nil, []*Body{target, bore})
	d.retire(target)
	d.retire(bore)
	d.n.LiveSet = append(d.n.LiveSet, &body.n)
	d.placeLocked(&body.n, dest)
	stampOperationWarningKind(body, OperationCut)
	d.appendOperationLocked(operation)
	return nil
}

func motherHingeSources(h *OperationHistory, target, bore *Body) (motherHingeAncestry, bool) {
	var out motherHingeAncestry
	src := pairedHistorySource{h: h}
	if h == nil || target == nil || bore == nil || target == bore ||
		!src.authenticated(target) || !src.authenticated(bore) || target.n.Doc != bore.n.Doc {
		return out, false
	}
	union := h.producers[target]
	if union == nil || union.kind != OperationUnion || !union.matchesIdentitySeal() ||
		len(union.outputs) != 1 || union.outputs[0] != target || len(union.inputs) != 2 {
		return out, false
	}
	kind, inputs, ok := src.BooleanProducer(&target.n)
	if !ok || kind != meshbool.OpUnion || len(inputs) != 2 {
		return out, false
	}
	leaf, knuckle := bodyHandle(inputs[0]), bodyHandle(inputs[1])
	cert, ok := target.n.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok || !cert.IsPureShrink() || len(target.n.OperationWarnings) != 1 {
		return out, false
	}
	warning := target.n.OperationWarnings[0]
	if warning.Operation != int(meshbool.OpUnion) || warning.PublicOperation != int(OperationUnion) ||
		warning.Code != opdiagnostic.CodeGapShiftApplied || warning.Operand != 1 ||
		warning.Change != opdiagnostic.ChangeGapShift || warning.ChainAllowance != nil {
		return out, false
	}
	out = motherHingeAncestry{target: target, tool: bore, leaf: leaf, knuckle: knuckle, union: union, certificate: cert}
	return out, true
}

// buildMotherHingeAdjustedPrism carries the authenticated whole-solid
// similarity into the one-circle prism record. The source tessellation replay
// binds that record to the witness; SectionDelta and axial deltas charge every
// held-coordinate and inverse-placement rounding used by the record rewrite.
func buildMotherHingeAdjustedPrism(ctx context.Context, env topo.Env, source *Body, center r3.Vec, scale float64,
	move r3.Vec, hasAdjustment bool, budget *proofbound.ReplayBudget) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if source == nil || budget == nil || !hasAdjustment || !proofbound.FiniteVec(center) ||
		!proofbound.FiniteVec(move) || !(scale > 0) || proofbound.IsNonFinite(scale) ||
		(scale == 1 && move == (r3.Vec{})) {
		return nil, ErrUnsupported
	}
	pp, ok := source.n.Payload.(prismrecord.Payload)
	if !ok || pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!pp.Xform.IsValid() || len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 {
		return nil, ErrUnsupported
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return nil, ErrUnsupported
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || proofbound.IsNonFinite(radius) || !(radius > 0) || circle.Radius.Kind() != units.Length ||
		circle.Radius.Unit() != units.Millimeter {
		return nil, ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 8); err != nil {
		return nil, err
	}
	sourceZ0, sourceZ1 := pp.Z0, pp.Z1
	inverse, err := pp.Xform.Inverse()
	if err != nil {
		return nil, err
	}
	anchor := r3.Vec{}
	anchorError := 0.0
	if scale != 1 {
		placedAnchor := inverse.Apply(center)
		anchor = pp.Frame.ToLocal(placedAnchor)
		if !proofbound.FiniteVec(anchor) {
			return nil, ErrUnsupported
		}
		anchorMagnitude := proofbound.AbsSumUpper(math.Abs(center.X), math.Abs(center.Y), math.Abs(center.Z),
			math.Abs(placedAnchor.X), math.Abs(placedAnchor.Y), math.Abs(placedAnchor.Z),
			math.Abs(pp.Frame.Origin().X), math.Abs(pp.Frame.Origin().Y), math.Abs(pp.Frame.Origin().Z),
			math.Abs(anchor.X), math.Abs(anchor.Y), math.Abs(anchor.Z))
		anchorError = proofbound.AnalyticRoundBound(anchorMagnitude)
		if proofbound.IsNonFinite(anchorError) {
			return nil, ErrUnsupported
		}
	}
	anchorImage := pp.Xform.Apply(pp.Frame.ToWorld(anchor))
	anchorImageRound := proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(
		math.Abs(anchorImage.X), math.Abs(anchorImage.Y), math.Abs(anchorImage.Z),
		math.Abs(center.X), math.Abs(center.Y), math.Abs(center.Z), math.Abs(anchor.X),
		math.Abs(anchor.Y), math.Abs(anchor.Z), math.Abs(pp.Frame.Origin().X),
		math.Abs(pp.Frame.Origin().Y), math.Abs(pp.Frame.Origin().Z)))
	anchorResidual := motherWorldVectorResidual(anchorImage, center,
		proofbound.AbsSumUpper(anchorError, anchorImageRound))
	if proofbound.IsNonFinite(anchorResidual) {
		return nil, ErrUnsupported
	}
	localMove := inverse.ApplyDir(move)
	moveU, moveV, moveZ := localMove.Dot(pp.Frame.U()), localMove.Dot(pp.Frame.V()), localMove.Dot(pp.Frame.N())
	if !proofbound.FiniteVec(localMove) {
		return nil, ErrUnsupported
	}
	moveError := proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(
		math.Abs(move.X), math.Abs(move.Y), math.Abs(move.Z),
		math.Abs(localMove.X), math.Abs(localMove.Y), math.Abs(localMove.Z),
		math.Abs(moveU), math.Abs(moveV), math.Abs(moveZ)))
	if proofbound.IsNonFinite(moveError) {
		return nil, ErrUnsupported
	}
	moveImage := pp.Xform.ApplyDir(pp.Frame.U().Scale(moveU).Add(pp.Frame.V().Scale(moveV)).Add(pp.Frame.N().Scale(moveZ)))
	moveResidual := motherWorldVectorResidual(moveImage, move, moveError)
	if proofbound.IsNonFinite(moveResidual) {
		return nil, ErrUnsupported
	}
	transform := func(value, about, translation float64) (float64, float64, bool) {
		delta := (value - about) * scale
		image := about + delta
		image += translation
		magnitude := proofbound.AbsSumUpper(math.Abs(about), math.Abs(value), math.Abs(delta), math.Abs(translation), math.Abs(image))
		bound := proofbound.AnalyticRoundBound(magnitude)
		return image, bound, !proofbound.IsNonFinite(image) && !proofbound.IsNonFinite(bound)
	}
	var uError, vError float64
	var good bool
	circle.Center.U, uError, good = transform(circle.Center.U, anchor.X, moveU)
	if !good {
		return nil, ErrUnsupported
	}
	circle.Center.V, vError, good = transform(circle.Center.V, anchor.Y, moveV)
	if !good {
		return nil, ErrUnsupported
	}
	placementError := proofbound.AbsSumUpper(
		proofbound.ProductUpper(4, proofbound.ProductUpper(math.Abs(1-scale), anchorResidual)),
		proofbound.ProductUpper(4, moveResidual))
	sectionError := proofbound.AbsSumUpper(uError, vError, placementError,
		proofbound.ProductUpper(math.Abs(1-scale), anchorError),
		2*moveError)
	newRadius := radius * scale
	radiusError := proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(radius, math.Abs(newRadius)))
	if proofbound.IsNonFinite(newRadius) || !(newRadius > 0) || proofbound.IsNonFinite(radiusError) {
		return nil, ErrUnsupported
	}
	circle.Radius = units.Millimeters(newRadius)
	sectionError = proofbound.AbsSumUpper(sectionError, radiusError)
	pp.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}
	var zError float64
	desiredZ0, zError, good := transform(sourceZ0, anchor.Z, moveZ)
	if !good {
		return nil, ErrUnsupported
	}
	pp.Z0 = desiredZ0
	pp.Z0Delta = proofbound.AbsSumUpper(zError, anchorError, moveError, placementError)
	desiredZ1, zError, good := transform(sourceZ1, anchor.Z, moveZ)
	if !good {
		return nil, ErrUnsupported
	}
	pp.Z1 = desiredZ1
	pp.Z1Delta = proofbound.AbsSumUpper(zError, anchorError, moveError, placementError)
	pp.Z0Denoted, pp.Z1Denoted = prismrecord.Payload{}.Z0Denoted, prismrecord.Payload{}.Z1Denoted
	pp.SectionDelta = sectionError
	pp.Walks = nil
	body, err := prismrecord.Build(ctx, env, pp, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	return body, ctx.Err()
}

func motherWorldVectorResidual(actual, expected r3.Vec, arithmeticScale float64) float64 {
	if !proofbound.FiniteVec(actual) || !proofbound.FiniteVec(expected) ||
		proofbound.IsNonFinite(arithmeticScale) || arithmeticScale < 0 {
		return math.Inf(1)
	}
	delta := actual.Sub(expected)
	return proofbound.AbsSumUpper(math.Abs(delta.X), math.Abs(delta.Y), math.Abs(delta.Z), arithmeticScale,
		proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(math.Abs(actual.X), math.Abs(actual.Y), math.Abs(actual.Z),
			math.Abs(expected.X), math.Abs(expected.Y), math.Abs(expected.Z))))
}

func remapMotherHingePrismFaces(adjusted *topo.Body, source *Body) error {
	if adjusted == nil || source == nil {
		return ErrUnsupported
	}
	adjustedFaces, sourceFaces := topobuild.Faces(adjusted), topobuild.Faces(&source.n)
	if len(adjustedFaces) == 0 || len(adjustedFaces) != len(sourceFaces) {
		return ErrUnsupported
	}
	for i, face := range adjustedFaces {
		origin := sourceFaces[i]
		if face == nil || origin == nil || reflect.TypeOf(face.Surface) != reflect.TypeOf(origin.Surface) || len(origin.Origins) == 0 {
			return ErrUnsupported
		}
		face.Origins = slices.Clone(origin.Origins)
		face.Upstream = slices.Clone(origin.Upstream)
		face.Source = &provenance.SourcePatch{Producer: source.n.Origin.Producer,
			Operand: 1, Ordinal: i, Parent: origin.Source}
	}
	return nil
}

func motherHingeVolumeIntervalsMeet(body *topo.Body, witness, witnessBound float64) bool {
	if body == nil || proofbound.IsNonFinite(witness) || proofbound.IsNonFinite(witnessBound) || witnessBound < 0 {
		return false
	}
	value, bound := body.Volume.Value.Base(), body.Volume.Bound.Base()
	if proofbound.IsNonFinite(value) || proofbound.IsNonFinite(bound) || bound < 0 {
		return false
	}
	return value+bound >= witness-witnessBound && witness+witnessBound >= value-bound
}

// motherHingeCutVolumeConsistent scopes the public Cut to the material removed
// from the adjusted knuckle. The ordinary private Cut measures that volume;
// leaf/bore separation proves it was not removed from the leaf.
func motherHingeCutVolumeConsistent(target *Body, result, adjusted, cut *topo.Body,
	firstSourceSymDiff, secondSourceSymDiff float64) bool {
	if target == nil || result == nil || adjusted == nil || cut == nil ||
		firstSourceSymDiff < 0 || secondSourceSymDiff < 0 ||
		proofbound.IsNonFinite(firstSourceSymDiff) || proofbound.IsNonFinite(secondSourceSymDiff) {
		return false
	}
	targetVolume, targetBound := target.n.Volume.Value.Base(), target.n.Volume.Bound.Base()
	adjustedVolume, adjustedBound := adjusted.Volume.Value.Base(), adjusted.Volume.Bound.Base()
	cutVolume, cutBound := cut.Volume.Value.Base(), cut.Volume.Bound.Base()
	resultVolume, resultBound := result.Volume.Value.Base(), result.Volume.Bound.Base()
	if slices.ContainsFunc([]float64{targetVolume, targetBound, adjustedVolume, adjustedBound, cutVolume,
		cutBound, resultVolume, resultBound}, proofbound.IsNonFinite) {
		return false
	}
	removed := adjustedVolume - cutVolume
	expected := targetVolume - removed
	if removed < 0 {
		return false
	}
	allowed := proofbound.AbsSumUpper(targetBound, adjustedBound, cutBound, resultBound,
		firstSourceSymDiff, secondSourceSymDiff)
	return !proofbound.IsNonFinite(expected) && !proofbound.IsNonFinite(allowed) &&
		math.Abs(resultVolume-expected) <= allowed
}

func proveMotherHingeSeparation(ctx context.Context, leaf, bore *Body,
	budget *proofbound.ReplayBudget) (motherHingeSeparation, bool, error) {
	var out motherHingeSeparation
	if leaf == nil || bore == nil || budget == nil || leaf.n.Doc != bore.n.Doc {
		return out, false, ErrUnsupported
	}
	chord, _, err := booloperand.PairChordReplay(ctx, &leaf.n, &bore.n, budget)
	if err != nil {
		return out, false, err
	}
	leafMesh, _, err := tessellateBooleanOperand(ctx, leaf, chord)
	if err != nil {
		return out, false, err
	}
	boreMesh, _, err := tessellateBooleanOperand(ctx, bore, chord)
	if err != nil {
		return out, false, err
	}
	leafBox, err := booleaneval.ReplayBodyBoxes(ctx, &leaf.n, &leafMesh.n, budget)
	if err != nil {
		return out, false, err
	}
	boreBox, err := booleaneval.ReplayBodyBoxes(ctx, &bore.n, &boreMesh.n, budget)
	if err != nil {
		return out, false, err
	}
	separated, err := booleaneval.ReplayBoxesDisjointBounded(ctx, leafBox, boreBox, budget)
	if err != nil || !separated {
		return out, false, err
	}
	leafFaces, boreFaces := topobuild.Faces(&leaf.n), topobuild.Faces(&bore.n)
	if !meshFacesOwnedBy(leafMesh.n.Source, leafFaces) || !meshFacesOwnedBy(boreMesh.n.Source, boreFaces) {
		return out, false, ErrUnsupported
	}
	out = motherHingeSeparation{leafBox: leafBox, boreBox: boreBox,
		leafFaces: slices.Clone(leafFaces), boreFaces: slices.Clone(boreFaces),
		leafBounds: leaf.n.Bounds, boreBounds: bore.n.Bounds, chord: chord}
	return out, true, nil
}

func meshFacesOwnedBy(sources []*topo.Face, owners []*topo.Face) bool {
	if len(sources) == 0 || len(owners) == 0 {
		return false
	}
	set := make(map[*topo.Face]struct{}, len(owners))
	for _, owner := range owners {
		if owner == nil {
			return false
		}
		set[owner] = struct{}{}
	}
	for _, source := range sources {
		if _, ok := set[source]; !ok {
			return false
		}
	}
	return true
}

func verifyMotherHingeResult(ctx context.Context, result, target, bore *Body) error {
	chord, _, err := booloperand.PairChord(ctx, &target.n, &bore.n)
	if err != nil {
		return err
	}
	mesh, _, err := tessellateBooleanOperand(ctx, result, chord)
	if err != nil {
		return err
	}
	volume, bound := result.n.Volume.Value.Base(), result.n.Volume.Bound.Base()
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() || !(volume > bound) ||
		proofbound.IsNonFinite(volume) || proofbound.IsNonFinite(bound) || len(result.n.Lumps) != 1 {
		return ErrUnsupported
	}
	return ctx.Err()
}

func motherHingeCutAncestry(adjusted, cut *Body, ancestry motherHingeAncestry,
	bore *Body) ([]motherFaceLineage, bool) {
	knuckleFaces := topobuild.Faces(&ancestry.knuckle.n)
	adjustedFaces := topobuild.Faces(&adjusted.n)
	cutFaces := topobuild.Faces(&cut.n)
	adjustedLineage, ok := adjustedFaceLineage(adjustedFaces, ancestry.knuckle, knuckleFaces)
	if !ok {
		return nil, false
	}
	return remapMotherAnalyticCutFaces(cutFaces, cut.n.Origin.Producer, adjustedLineage,
		directFaceLineage(bore, topobuild.Faces(&bore.n)))
}

func remapMotherHingeAncestry(result, cut *Body, ancestry motherHingeAncestry, bore *Body,
	cutLineage []motherFaceLineage) error {
	resultPayload, ok := result.n.Payload.(facetedrecord.Payload)
	if !ok || len(resultPayload.FaceOf) != len(resultPayload.Tris) || len(resultPayload.Src) != len(resultPayload.Tris) {
		return ErrUnsupported
	}
	leafFaces := topobuild.Faces(&ancestry.leaf.n)
	resultFaces := topobuild.Faces(&result.n)
	if len(cutLineage) != 4 {
		return ErrUnsupported
	}
	leafLineage := directFaceLineage(ancestry.leaf, leafFaces)
	_, ok = booleanOutputLineage(resultFaces,
		ancestry.leaf.n.Origin.Producer, cut.n.Origin.Producer, leafLineage, cutLineage)
	if !ok {
		return ErrUnsupported
	}
	if len(resultPayload.Groups) == 0 || len(resultPayload.Groups) > 65536 {
		return ErrUnsupported
	}
	for i, group := range resultPayload.Groups {
		if group.Source == nil || group.Source.Blend {
			return ErrUnsupported
		}
		var inputs []motherFaceLineage
		var expectedOperand int
		switch group.Source.Producer {
		case ancestry.leaf.n.Origin.Producer:
			inputs, expectedOperand = leafLineage, 0
		case cut.n.Origin.Producer:
			inputs, expectedOperand = cutLineage, 1
		default:
			return ErrUnsupported
		}
		if group.Source.Operand != expectedOperand || group.Source.Ordinal < 0 || group.Source.Ordinal >= len(inputs) {
			return ErrUnsupported
		}
		input := inputs[group.Source.Ordinal]
		if group.Source.Parent != input.sourceFace.Source || sourcePatchCyclic(group.Source) {
			return ErrUnsupported
		}
		if input.rootBody == nil || input.rootFace == nil {
			return ErrUnsupported
		}
		if !slices.Equal(group.Origins, input.sourceFace.Origins) ||
			!slices.Equal(group.Upstream, input.sourceFace.Upstream) {
			return ErrUnsupported
		}
		publicOperand := 0
		if input.rootBody == bore {
			publicOperand = 1
		}
		resultPayload.Groups[i].Source = facetedrecord.FaceSource(input.rootBody.n.Origin.Producer,
			publicOperand, input.rootOrdinal, input.rootFace)
		resultPayload.Groups[i].Origins = slices.Clone(input.rootFace.Origins)
		resultPayload.Groups[i].Upstream = slices.Clone(input.rootFace.Upstream)
	}
	faceGroup := make([]int, len(resultFaces))
	for i := range faceGroup {
		faceGroup[i] = -1
	}
	for tri, face := range resultPayload.FaceOf {
		group := resultPayload.Src[tri]
		if face < 0 || face >= len(faceGroup) || group < 0 || group >= len(resultPayload.Groups) {
			return ErrUnsupported
		}
		if faceGroup[face] >= 0 && faceGroup[face] != group {
			return ErrUnsupported
		}
		faceGroup[face] = group
	}
	for i, face := range resultFaces {
		if faceGroup[i] < 0 {
			return ErrUnsupported
		}
		group := resultPayload.Groups[faceGroup[i]]
		face.Source = group.Source
		face.Origins = slices.Clone(group.Origins)
		face.Upstream = slices.Clone(group.Upstream)
	}
	result.n.Payload = resultPayload
	return nil
}

func remapMotherAnalyticCutFaces(faces []*topo.Face, producer producerID,
	adjusted, bore []motherFaceLineage) ([]motherFaceLineage, bool) {
	wantRoles := map[string]struct{}{
		"side(0,0)": {}, "side(1,0)": {}, "capStart": {}, "capEnd": {},
	}
	if len(faces) != len(wantRoles) || len(adjusted) == 0 || len(bore) == 0 {
		return nil, false
	}
	out := make([]motherFaceLineage, len(faces))
	for i, face := range faces {
		if face == nil || face.Source != nil || len(face.Origins) != 1 ||
			face.Origins[0].Producer != producer {
			return nil, false
		}
		role := face.Origins[0].Role
		if _, ok := wantRoles[role]; !ok {
			return nil, false
		}
		switch role {
		case "side(0,0)", "side(1,0)":
			if _, ok := face.Surface.(surfacegeom.Cylinder); !ok {
				return nil, false
			}
		case "capStart", "capEnd":
			if _, ok := face.Surface.(surfacegeom.Plane); !ok {
				return nil, false
			}
		}
		delete(wantRoles, role)
		var match motherFaceLineage
		publicOperand := -1
		matches := 0
		for operand, inputs := range [2][]motherFaceLineage{adjusted, bore} {
			for _, input := range inputs {
				if input.sourceFace == nil || !sameMotherSurface(face.Surface, input.sourceFace.Surface) {
					continue
				}
				match = input
				publicOperand = operand
				matches++
			}
		}
		if matches != 1 || match.rootBody == nil || match.rootFace == nil ||
			match.rootOrdinal < 0 || publicOperand < 0 {
			return nil, false
		}
		face.Source = facetedrecord.FaceSource(match.rootBody.n.Origin.Producer,
			publicOperand, match.rootOrdinal, match.rootFace)
		out[i] = motherFaceLineage{sourceFace: face, rootBody: match.rootBody,
			rootFace: match.rootFace, rootOrdinal: match.rootOrdinal}
	}
	return out, len(wantRoles) == 0
}

func sameMotherSurface(a, b surfacegeom.Surface) bool {
	switch x := a.(type) {
	case surfacegeom.Plane:
		y, ok := b.(surfacegeom.Plane)
		return ok && x == y
	case surfacegeom.Cylinder:
		y, ok := b.(surfacegeom.Cylinder)
		if !ok || x.Radius != y.Radius || !sameMotherAxisDirection(x.Axis, y.Axis) {
			return false
		}
		return exactMotherAxisLine(x.Origin, y.Origin, x.Axis)
	default:
		return false
	}
}

func sameMotherAxisDirection(a, b r3.Vec) bool {
	return a != (r3.Vec{}) && b != (r3.Vec{}) && proofbound.FiniteVec(a) && proofbound.FiniteVec(b) &&
		(a == b || (a.X == -b.X && a.Y == -b.Y && a.Z == -b.Z))
}

func exactMotherAxisLine(a, b, axis r3.Vec) bool {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) || !proofbound.FiniteVec(axis) {
		return false
	}
	dx := exactFloatDifference(a.X, b.X)
	dy := exactFloatDifference(a.Y, b.Y)
	dz := exactFloatDifference(a.Z, b.Z)
	ax := new(big.Rat).SetFloat64(axis.X)
	ay := new(big.Rat).SetFloat64(axis.Y)
	az := new(big.Rat).SetFloat64(axis.Z)
	return equalRatProducts(dy, az, dz, ay) &&
		equalRatProducts(dz, ax, dx, az) &&
		equalRatProducts(dx, ay, dy, ax)
}

func exactFloatDifference(a, b float64) *big.Rat {
	return new(big.Rat).Sub(new(big.Rat).SetFloat64(a), new(big.Rat).SetFloat64(b))
}

func equalRatProducts(a, b, c, d *big.Rat) bool {
	return new(big.Rat).Mul(a, b).Cmp(new(big.Rat).Mul(c, d)) == 0
}

type motherFaceLineage struct {
	sourceFace  *topo.Face
	rootBody    *Body
	rootFace    *topo.Face
	rootOrdinal int
}

func directFaceLineage(body *Body, faces []*topo.Face) []motherFaceLineage {
	out := make([]motherFaceLineage, len(faces))
	for i, face := range faces {
		out[i] = motherFaceLineage{sourceFace: face, rootBody: body, rootFace: face, rootOrdinal: i}
	}
	return out
}

func adjustedFaceLineage(faces []*topo.Face, source *Body, sourceFaces []*topo.Face) ([]motherFaceLineage, bool) {
	out := make([]motherFaceLineage, len(faces))
	for i, face := range faces {
		if face == nil || face.Source == nil || face.Source.Blend || face.Source.Producer != source.n.Origin.Producer ||
			face.Source.Operand != 1 || face.Source.Ordinal < 0 || face.Source.Ordinal >= len(sourceFaces) {
			return nil, false
		}
		origin := sourceFaces[face.Source.Ordinal]
		if origin == nil || face.Source.Parent != origin.Source || sourcePatchCyclic(face.Source) {
			return nil, false
		}
		out[i] = motherFaceLineage{sourceFace: face, rootBody: source,
			rootFace: origin, rootOrdinal: face.Source.Ordinal}
	}
	return out, len(out) > 0
}

func booleanOutputLineage(faces []*topo.Face, producerA, producerB producerID,
	sourcesA, sourcesB []motherFaceLineage) ([]motherFaceLineage, bool) {
	out := make([]motherFaceLineage, len(faces))
	for i, face := range faces {
		if face == nil || face.Source == nil || face.Source.Blend {
			return nil, false
		}
		var sources []motherFaceLineage
		var expectedOperand int
		switch face.Source.Producer {
		case producerA:
			sources, expectedOperand = sourcesA, 0
		case producerB:
			sources, expectedOperand = sourcesB, 1
		default:
			return nil, false
		}
		if face.Source.Operand != expectedOperand || face.Source.Ordinal < 0 || face.Source.Ordinal >= len(sources) ||
			face.Source.Producer != producerA && face.Source.Producer != producerB {
			return nil, false
		}
		input := sources[face.Source.Ordinal]
		parentMatch := input.sourceFace != nil && face.Source.Parent == input.sourceFace.Source
		if !parentMatch || sourcePatchCyclic(face.Source) {
			return nil, false
		}
		out[i] = motherFaceLineage{sourceFace: face, rootBody: input.rootBody,
			rootFace: input.rootFace, rootOrdinal: input.rootOrdinal}
	}
	return out, len(out) > 0
}

func sourcePatchCyclic(patch *provenance.SourcePatch) bool {
	seen := make(map[*provenance.SourcePatch]struct{})
	for current, depth := patch, 0; current != nil; current, depth = current.Parent, depth+1 {
		if depth >= 64 {
			return true
		}
		if _, ok := seen[current]; ok {
			return true
		}
		seen[current] = struct{}{}
	}
	return false
}

func sameMotherHingeAncestry(a, b motherHingeAncestry) bool {
	return a.target == b.target && a.tool == b.tool && a.leaf == b.leaf && a.knuckle == b.knuckle &&
		a.union == b.union && a.certificate == b.certificate
}

func sameMotherSourceOwners(proof motherHingeSeparation, leaf, bore *Body) bool {
	return leaf != nil && bore != nil && leaf.n.Bounds == proof.leafBounds && bore.n.Bounds == proof.boreBounds &&
		slices.Equal(topobuild.Faces(&leaf.n), proof.leafFaces) &&
		slices.Equal(topobuild.Faces(&bore.n), proof.boreFaces)
}

func sameMotherHingeSeparation(a, b motherHingeSeparation) bool {
	return a.chord == b.chord && a.leafBounds == b.leafBounds && a.boreBounds == b.boreBounds &&
		sameMotherReplayBox(a.leafBox, b.leafBox) && sameMotherReplayBox(a.boreBox, b.boreBox) &&
		slices.Equal(a.leafFaces, b.leafFaces) && slices.Equal(a.boreFaces, b.boreFaces)
}

func sameMotherReplayBox(a, b booleaneval.ReplayBox) bool {
	for i := range 3 {
		left := [4]*big.Rat{a.TrueLo[i], a.TrueHi[i], a.HeldLo[i], a.HeldHi[i]}
		right := [4]*big.Rat{b.TrueLo[i], b.TrueHi[i], b.HeldLo[i], b.HeldHi[i]}
		for j := range left {
			if left[j] == nil || right[j] == nil || left[j].Cmp(right[j]) != 0 {
				return false
			}
		}
	}
	return true
}
