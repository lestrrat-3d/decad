package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func tryLoftCylinderUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *Body
	var teeth []*Body
	if _, ok := a.n.Payload.(prismPayload); ok {
		blank, teeth = a, []*Body{b}
	} else if _, ok := b.n.Payload.(prismPayload); ok {
		blank, teeth = b, []*Body{a}
	} else if fp, ok := a.n.Payload.(facetedPayload); ok && fp.LoftCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftCylinderJoin.Blank)
		teeth = append(bodyHandlesCopy(fp.LoftCylinderJoin.Teeth), b)
	} else if fp, ok := b.n.Payload.(facetedPayload); ok && fp.LoftCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftCylinderJoin.Blank)
		teeth = append(bodyHandlesCopy(fp.LoftCylinderJoin.Teeth), a)
	} else {
		return tryPlacedLoftCylinderUnion(ctx, d, ref, a, b)
	}
	body, ok, err := tryLoftCylinderUnionTeeth(ctx, d, ref, blank, teeth)
	if err != nil || ok {
		return body, ok, err
	}
	return tryPlacedLoftCylinderUnion(ctx, d, ref, a, b)
}

// tryLoftCylinderUnionTeeth certifies one complete circular Join from the
// blank and all of its Loft teeth in a single certificate.
func tryLoftCylinderUnionTeeth(ctx context.Context, d *Document, ref producerID,
	blank *Body, teeth []*Body) (*Body, bool, error) {
	return tryLoftCylinderUnionTeethWithUpperBridge(ctx, d, ref, blank, teeth, nil)
}

// tryLoftCylinderUnionTeethWithUpperBridge admits the blank and each tooth,
// builds every tooth's private proof body and verified mesh, and hands the
// plain proof data to loftcylinder. A non-nil upperBridges holds one
// moving-plane separation widening per tooth; the mirrored Union passes
// them for its lower slab. It stays in root because the proof teeth and
// both verified meshes are built through root Body handles.
func tryLoftCylinderUnionTeethWithUpperBridge(ctx context.Context, d *Document, ref producerID,
	blank *Body, teeth []*Body, upperBridges []float64) (*Body, bool, error) {
	if upperBridges != nil && len(upperBridges) != len(teeth) {
		return nil, false, nil
	}
	pp, ok := blank.n.Payload.(prismPayload)
	if !ok {
		return nil, false, nil
	}
	radius, ok := loftcylinder.BlankRadius(pp)
	if !ok {
		return nil, false, nil
	}
	parts := make([]loftcylinder.Tooth, len(teeth))
	proofTeeth := make([]*Body, len(teeth))
	var source *loftrecord.SourceIdentity
	for i, tooth := range teeth {
		lp, ok := tooth.n.Payload.(loftPayload)
		if !ok || !loftcylinder.ToothSections(lp, pp.Z0, pp.Z1) {
			return nil, false, nil
		}
		if source == nil {
			source = lp.SourceIdentity
		} else if source != lp.SourceIdentity {
			return nil, false, nil
		}
		proofTooth, bridge, admitted, err := loftCylinderProofTooth(ctx, d, tooth, pp.Z0, pp.Z1)
		if err != nil {
			return nil, true, err
		}
		if !admitted {
			return nil, false, nil
		}
		lp, ok = proofTooth.n.Payload.(loftPayload)
		if !ok {
			return nil, false, nil
		}
		root, ok := loftcylinder.ToothRoot(lp)
		if !ok {
			return nil, false, nil
		}
		m, err := proofTooth.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
		if err != nil {
			return nil, false, err
		}
		if !m.BoundaryVerified() || !m.VolumeVerified() {
			return nil, false, nil
		}
		parts[i] = loftcylinder.Tooth{Operand: i, Loft: lp, RootStart: root,
			MeshBound: m.Bound().Base(), MeshVolSymDiff: m.n.VolSymDiff, Bridge: bridge}
		if upperBridges != nil {
			parts[i].UpperSeparationBridge = upperBridges[i]
		}
		if !parts[i].Admit(radius, pp.Z0, pp.Z1) {
			return nil, false, nil
		}
		proofTeeth[i] = proofTooth
	}
	if !loftcylinder.Separate(parts) {
		return nil, false, nil
	}
	join := &loftcylinder.Join{Blank: bodyNode(blank), Originals: bodyNodes(teeth),
		Proofs: bodyNodes(proofTeeth), Teeth: parts, Radius: radius, Z0: pp.Z0, Z1: pp.Z1}
	if err := join.Shell(ctx); err != nil {
		return nil, true, err
	}
	blankMesh, err := blank.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		return nil, true, err
	}
	fp, err := join.Payload(ctx, &blankMesh.n)
	if err != nil {
		return nil, true, err
	}
	record := fp.LoftCylinderJoin
	if err := loftCylinderTagCaps(&fp, blank, record.CapGroups, [2]float64{pp.Z0, pp.Z1}, ref); err != nil {
		return nil, true, err
	}
	loftCylinderTagBridgedTop(&fp, record.CapGroups[1], record.AxialBridge)
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), fp))
	return body, true, err
}
