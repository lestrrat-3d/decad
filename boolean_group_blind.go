package decad

import (
	"context"
	"errors"
	"maps"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/capplane"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// groupBlindCutBodyWithBudget admits a blind group only after binding its ordered public
// Union inputs, target and tool caps, exact placement, and whole-loop section.
// A proof miss leaves the pair to the ordinary boolean routes.
func groupBlindCutBodyWithBudget(ctx context.Context, target, tool *Body, ref producerID,
	budget *proofbound.ReplayBudget) (*Body, bool, error) {
	miss := func(err error) (*Body, bool, error) {
		if _, ok := errors.AsType[*proofbound.WorkLimitError](err); ok {
			return nil, false, err
		}
		if errors.Is(err, decaderr.ErrUnsupported) {
			return nil, false, nil
		}
		return nil, false, err
	}
	if target == nil || tool == nil || budget == nil || target.n.Doc == nil || target.n.Doc != tool.n.Doc ||
		target.n.Owner != target || target.n.Origin.Role != provenance.RoleBody || len(target.n.OperationWarnings) != 0 {
		return nil, false, nil
	}
	host, ok := target.n.Payload.(prismrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	direct, err := groupDirectPrismSource(ctx, target, budget)
	if err != nil || !direct {
		return miss(err)
	}
	group, ok := tool.n.Payload.(stackedrecord.Payload)
	if !ok || !group.IsGroup() {
		return nil, false, nil
	}
	isometric, err := prismplacement.ReplayWorldIsometry(ctx, host.PlacementOperand(), budget)
	if err != nil || !isometric {
		return miss(err)
	}
	source, ok, err := groupUnionSourceSnapshot(ctx, tool, budget)
	if err != nil || !ok {
		return miss(err)
	}
	for _, leaf := range source.Inputs {
		direct, err := groupDirectPrismSource(ctx, leaf, budget)
		if err != nil || !direct {
			return miss(err)
		}
	}
	slab := group.Slabs[0]
	place := prismplacement.Operand{Frame: group.Frame, Xform: group.Xform, Z0: slab.Z0, Z1: slab.Z1,
		Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	mapping, ok, err := prismplacement.ReplayParallelSignedMap(ctx, host.PlacementOperand(), place, budget)
	if err != nil || !ok {
		return miss(err)
	}
	top, ok, err := prismplacement.ReplayBlindLevels(ctx, host.PlacementOperand(), place, mapping.Shift, budget)
	if err != nil || !ok {
		return miss(err)
	}
	targetCap, ok, err := groupBlindTargetCap(ctx, &target.n, top, budget)
	if err != nil || !ok {
		return miss(err)
	}
	plane, ok, err := capplane.ReplayOfFace(ctx, targetCap, budget)
	if err != nil || !ok {
		return miss(err)
	}
	for i := range source.GroupCaps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return miss(err)
		}
		face := source.GroupCaps[i].Start
		if top {
			face = source.GroupCaps[i].End
		}
		other, found, err := capplane.ReplayOfFace(ctx, face, budget)
		if err != nil || !found {
			return miss(err)
		}
		same, err := capplane.ReplaySame(ctx, plane, other, budget)
		if err != nil || !same {
			return miss(err)
		}
	}
	cert, ok, err := prismcells.ReplayBlindGroupScene(ctx, host.Profile, slab.Regions,
		host.PlacementOperand(), place, host.SectionDelta, group.SectionDelta, budget)
	if err != nil || !ok {
		return miss(err)
	}
	payload, regions, ok, err := stackedrecord.ReplayBlindGroupPayload(ctx, host, group, cert, budget)
	if err != nil || !ok {
		return miss(err)
	}
	if len(regions) != len(source.Owners) {
		return nil, false, nil
	}
	for _, region := range regions {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return miss(err)
		}
		if region < 0 || region >= len(source.Owners) {
			return nil, false, nil
		}
	}
	node, events, built, err := stackedrecord.ReplayBlindGroupBuildWithSource(ctx, envOf(target.n.Doc, ref),
		payload, &target.n, group, cert, budget)
	if err != nil || !built {
		return miss(err)
	}
	if err := budget.ReserveArray(ctx, uint64(len(source.Inputs)),
		uint64(unsafe.Sizeof((*topo.Body)(nil)))); err != nil {
		return miss(err)
	}
	leaves := make([]*topo.Body, len(source.Inputs))
	for i, input := range source.Inputs {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return miss(err)
		}
		if input == nil {
			return nil, false, nil
		}
		leaves[i] = &input.n
	}
	ledgerInput := prismboolean.BlindFaceLedgerInput{
		Output: node, Target: &target.n, Group: &tool.n,
		Leaves: leaves, Owners: source.Owners, GroupCaps: source.GroupCaps,
		LeafCaps: source.LeafCaps, Scene: cert, Events: events,
	}
	ledger, bound, err := prismboolean.ReplayBlindFaceLedger(ctx, ledgerInput, budget)
	if err != nil || !bound {
		return miss(err)
	}
	if ledger.Len() != events.Len() {
		return nil, false, nil
	}
	bound, err = prismboolean.ReplayPublishBlindUpstream(ctx, ledgerInput, ledger, budget)
	if err != nil || !bound {
		return miss(err)
	}
	witness := ledger.PublishedWitness()
	bound, err = witness.Recheck(ctx, node, &target.n, &tool.n, leaves, budget)
	if err != nil || !bound {
		return miss(err)
	}
	bound, err = recheckBlindFaceOwners(ctx, node, budget)
	if err != nil || !bound {
		return miss(err)
	}
	result := bodyHandle(node)
	proof, bound, err := captureBlindGroupProof(ctx, result, target, tool, source.Inputs, witness, budget)
	if err != nil || !bound {
		return miss(err)
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	result.blindProof = proof
	return result, true, nil
}

// commitBlindGroupCut builds replacement document indexes under the same
// replay meter, then publishes them together with the witnessed Cut. It stays
// in the root because it owns document membership and operation history.
func commitBlindGroupCut(ctx context.Context, d *Document, target, tool, result *Body,
	budget *proofbound.ReplayBudget) error {
	if d == nil || target == nil || tool == nil || result == nil || budget == nil || target == tool || result.blindProof == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return err
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if target.n.Doc != &d.n || tool.n.Doc != &d.n || result.n.Doc != &d.n ||
		result.n.Origin.Producer != d.n.NextProducer || result.blindProof.cut != nil {
		return decaderr.ErrUnsupported
	}
	// Each input count is capped before arithmetic. The quadratic map-work
	// allowance covers worst-case probing while rebuilding the two history
	// indexes and the live container index; storage is reserved before makes.
	counts := [...]int{len(d.n.LiveSet), len(d.operations), len(d.producers), len(d.consumers),
		len(d.groups.ofBody), len(d.consumers[target]), len(d.consumers[tool])}
	var sum uint64
	for _, n := range counts {
		if n > 1<<20 {
			return decaderr.ErrUnsupported
		}
		sum += uint64(n)
	}
	np, nc, ng, nl := uint64(counts[2]+1), uint64(counts[3]+2), uint64(counts[4]+1), uint64(counts[0])
	work := 4096 + 64*(sum+1) + 16*(np*np+nc*nc+ng*ng+nl*ng)
	if err := budget.ReserveBundle(ctx, work, 8*(sum+1), 8192+1024*(sum+1)); err != nil {
		return err
	}
	inputs := operationInputsForBoolean(meshbool.OpCut, target, tool)
	operation := d.operation(OperationCut, inputs, result)
	if !result.blindProof.acceptsOperation(operation, result) {
		return decaderr.ErrUnsupported
	}
	root := d.groups.root
	if root == nil {
		root = &Container{doc: d}
	}
	targetContainer, toolContainer := d.groups.ofBody[&target.n], d.groups.ofBody[&tool.n]
	if targetContainer == nil {
		targetContainer = root
	}
	if toolContainer == nil {
		toolContainer = root
	}
	if targetContainer != toolContainer {
		return ErrCrossContainer
	}
	dest := targetContainer
	if d.resultC != nil {
		dest = d.resultC
	}
	live := make([]*topo.Body, 0, len(d.n.LiveSet))
	targetCount, toolCount := 0, 0
	for _, node := range d.n.LiveSet {
		if err := ctx.Err(); err != nil {
			return err
		}
		switch node {
		case &target.n:
			targetCount++
		case &tool.n:
			toolCount++
		default:
			live = append(live, node)
		}
	}
	if targetCount != 1 || toolCount != 1 {
		return ErrRetiredBody
	}
	live = append(live, &result.n)
	operations := make([]*Operation, len(d.operations)+1)
	copy(operations, d.operations)
	operations[len(d.operations)] = operation
	producers := make(map[*Body]*Operation, len(d.producers)+1)
	maps.Copy(producers, d.producers)
	producers[result] = operation
	consumers := make(map[*Body][]*Operation, len(d.consumers)+2)
	maps.Copy(consumers, d.consumers)
	for _, body := range [...]*Body{target, tool} {
		prior := d.consumers[body]
		next := make([]*Operation, len(prior)+1)
		copy(next, prior)
		next[len(prior)] = operation
		consumers[body] = next
	}
	containers := make(map[*topo.Body]*Container, len(live))
	for _, node := range live[:len(live)-1] {
		if c := d.groups.ofBody[node]; c != nil {
			containers[node] = c
		}
	}
	if dest != root {
		containers[&result.n] = dest
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	result.blindProof.cut = operation
	d.n.NextProducer++
	d.n.LiveSet, d.operations, d.producers, d.consumers = live, operations, producers, consumers
	d.groups.root, d.groups.ofBody = root, containers
	stampOperationWarningKind(result, OperationCut)
	return nil
}

func groupBlindTargetCap(ctx context.Context, body *topo.Body, top bool,
	budget *proofbound.ReplayBudget) (*topo.Face, bool, error) {
	if body == nil || budget == nil {
		return nil, false, nil
	}
	role := provenance.RoleCapStart
	if top {
		role = provenance.RoleCapEnd
	}
	var found *topo.Face
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if lump == nil {
			return nil, false, nil
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			if shell == nil {
				return nil, false, nil
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, false, err
				}
				if face == nil || len(face.Origins) != 1 || face.Origins[0].Role != role {
					continue
				}
				if found != nil {
					return nil, false, nil
				}
				found = face
			}
		}
	}
	if found == nil {
		return nil, false, nil
	}
	ok, err := prismboolean.ReplayPrismCapRecord(ctx, body, found, top, budget)
	return found, ok, err
}
