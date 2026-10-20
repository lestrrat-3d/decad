package loftcylinder

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/verifiedmesh"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// JoinUnion joins a cylinder blank and one more Loft tooth into one certified
// circular join, or extends an existing join by that tooth; a placed pair goes
// to PlacedUnion.
func JoinUnion(ctx context.Context, op meshbool.OperationKind,
	env topo.Env, mesher verifiedmesh.Mesher, a, b *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *topo.Body
	var teeth []*topo.Body
	if _, ok := a.Payload.(prismrecord.Payload); ok {
		blank, teeth = a, []*topo.Body{b}
	} else if _, ok := b.Payload.(prismrecord.Payload); ok {
		blank, teeth = b, []*topo.Body{a}
	} else if fp, ok := a.Payload.(facetedrecord.Payload); ok && fp.LoftCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = fp.LoftCylinderJoin.Blank
		teeth = append(slices.Clone(fp.LoftCylinderJoin.Teeth), b)
	} else if fp, ok := b.Payload.(facetedrecord.Payload); ok && fp.LoftCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = fp.LoftCylinderJoin.Blank
		teeth = append(slices.Clone(fp.LoftCylinderJoin.Teeth), a)
	} else {
		return PlacedUnion(ctx, env, mesher, a, b)
	}
	body, ok, err := UnionTeeth(ctx, env, mesher, blank, teeth)
	if err != nil || ok {
		return body, ok, err
	}
	return PlacedUnion(ctx, env, mesher, a, b)
}

// JoinBoreCut cuts a centered through-bore from an audited
// loft/cylinder join. A placed join goes to PlacedBoreCut;
// BoreCut admits the tool and builds the bore of an identity
// join.
func JoinBoreCut(ctx context.Context, op meshbool.OperationKind,
	env topo.Env, mesher verifiedmesh.Mesher, target, tool *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.Payload.(facetedrecord.Payload)
	if !ok || joined.LoftCylinderJoin == nil {
		return nil, false, nil
	}
	if joined.LoftCylinderJoin.Placement != nil {
		return PlacedBoreCut(ctx, env, mesher, target, tool)
	}
	body, admitted, err := BoreCut(ctx, env, joined, tool)
	return body, admitted, err
}

// UnionTeeth certifies one complete circular Join from the
// blank and all of its Loft teeth in a single certificate.
func UnionTeeth(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	blank *topo.Body, teeth []*topo.Body) (*topo.Body, bool, error) {
	return UnionTeethWithUpperBridge(ctx, env, mesher, blank, teeth, nil)
}

// UnionTeethWithUpperBridge admits the blank and each tooth,
// verifies the mesh of the private proof body ProofTooth gives
// each tooth, and hands the plain proof data to the join. A non-nil
// upperBridges holds one moving-plane separation widening per tooth; the
// mirrored Union passes them for its lower slab. The tooth and blank meshes
// are verified through the mesher.
func UnionTeethWithUpperBridge(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	blank *topo.Body, teeth []*topo.Body, upperBridges []float64) (*topo.Body, bool, error) {
	if upperBridges != nil && len(upperBridges) != len(teeth) {
		return nil, false, nil
	}
	pp, ok := blank.Payload.(prismrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	radius, ok := BlankRadius(pp)
	if !ok {
		return nil, false, nil
	}
	parts := make([]Tooth, len(teeth))
	proofTeeth := make([]*topo.Body, len(teeth))
	var source *loftrecord.SourceIdentity
	for i, tooth := range teeth {
		lp, ok := tooth.Payload.(loftrecord.Payload)
		if !ok || !ToothSections(lp, pp.Z0, pp.Z1) {
			return nil, false, nil
		}
		if source == nil {
			source = lp.SourceIdentity
		} else if source != lp.SourceIdentity {
			return nil, false, nil
		}
		proofNode, bridge, admitted, err := ProofTooth(ctx, env,
			tooth, pp.Z0, pp.Z1)
		if err != nil {
			return nil, true, err
		}
		if !admitted {
			return nil, false, nil
		}
		proofTooth := proofNode
		lp, ok = proofTooth.Payload.(loftrecord.Payload)
		if !ok {
			return nil, false, nil
		}
		root, ok := ToothRoot(lp)
		if !ok {
			return nil, false, nil
		}
		m, err := mesher.VerifyAll(ctx, proofTooth, 0.1)
		if err != nil {
			return nil, false, err
		}
		if !m.BoundaryOK || !m.SymDiffOK {
			return nil, false, nil
		}
		parts[i] = Tooth{Operand: i, Loft: lp, RootStart: root,
			MeshBound: m.Bound, MeshVolSymDiff: m.VolSymDiff, Bridge: bridge}
		if upperBridges != nil {
			parts[i].UpperSeparationBridge = upperBridges[i]
		}
		if !parts[i].Admit(radius, pp.Z0, pp.Z1) {
			return nil, false, nil
		}
		proofTeeth[i] = proofTooth
	}
	if !Separate(parts) {
		return nil, false, nil
	}
	join := &Join{Blank: blank, Originals: teeth,
		Proofs: proofTeeth, Teeth: parts, Radius: radius, Z0: pp.Z0, Z1: pp.Z1}
	if err := join.Shell(ctx); err != nil {
		return nil, true, err
	}
	blankMesh, err := mesher.VerifyAll(ctx, blank, 0.1)
	if err != nil {
		return nil, true, err
	}
	fp, err := join.Payload(ctx, blankMesh)
	if err != nil {
		return nil, true, err
	}
	record := fp.LoftCylinderJoin
	if err := TagCaps(&fp, blank, record.CapGroups, [2]float64{pp.Z0, pp.Z1}, env.Ref); err != nil {
		return nil, true, err
	}
	TagBridgedTop(&fp, record.CapGroups[1], record.AxialBridge)
	body, err := facetedrecord.Build(ctx, env, fp)
	return body, true, err
}
