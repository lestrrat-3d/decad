package prismboolean

import (
	"context"
	"strconv"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// BlindFaceLedgerInput names the source records authenticated by the caller's
// ordered Union history and the private builder's sealed face events.
type BlindFaceLedgerInput struct {
	Output, Target, Group *topo.Body
	Leaves                []*topo.Body
	Owners                []int
	GroupCaps, LeafCaps   []ReplayGroupCapFaces
	Scene                 *prismcells.ReplayBlindScene
	Events                *stackedrecord.ReplayBlindFaceLog
}

// BlindFaceLedgerEntry records one output face and its immediate and primitive
// source faces. It does not assert that the output covers an entire source face.
type BlindFaceLedgerEntry struct {
	Output, Immediate, Leaf       *topo.Face
	SourceBody                    *topo.Body
	Kind                          stackedrecord.ReplayBlindFaceKind
	Loop, Segment, Region, Column int
}

// BlindFaceLedger is private replay evidence, bound to one output and meter.
type BlindFaceLedger struct {
	output    *topo.Body
	budget    *proofbound.ReplayBudget
	entries   []BlindFaceLedgerEntry
	published *BlindPublishedWitness
}

// PublishedWitness returns the immutable source and output binding captured
// before upstream publication, or nil until publication succeeds.
func (l *BlindFaceLedger) PublishedWitness() *BlindPublishedWitness {
	if l == nil {
		return nil
	}
	return l.published
}

func (l *BlindFaceLedger) Len() int {
	if l == nil {
		return 0
	}
	return len(l.entries)
}
func (l *BlindFaceLedger) Entry(i int) (BlindFaceLedgerEntry, bool) {
	if l == nil || i < 0 || i >= len(l.entries) {
		return BlindFaceLedgerEntry{}, false
	}
	return l.entries[i], true
}
func (l *BlindFaceLedger) Matches(out *topo.Body, b *proofbound.ReplayBudget) bool {
	return l != nil && l.output == out && l.budget == b
}

// Recheck repeats source and output validation under the original meter and
// compares every bound face pointer with the stored ledger.
func (l *BlindFaceLedger) Recheck(ctx context.Context, in BlindFaceLedgerInput, b *proofbound.ReplayBudget) (bool, error) {
	if !l.Matches(in.Output, b) {
		return false, nil
	}
	current, ok, err := ReplayBlindFaceLedger(ctx, in, b)
	if err != nil || !ok {
		return false, err
	}
	if len(current.entries) != len(l.entries) {
		return false, nil
	}
	for i := range l.entries {
		if err := b.SpendWork(ctx, 32); err != nil {
			return false, err
		}
		if current.entries[i] != l.entries[i] {
			return false, nil
		}
	}
	return true, nil
}

func blindSideRole(role string, loop, segment int) bool {
	var buf [64]byte
	name := append(buf[:0], "side("...)
	name = strconv.AppendInt(name, int64(loop), 10)
	name = append(name, ',')
	name = strconv.AppendInt(name, int64(segment), 10)
	name = append(name, ')')
	if len(role) != len(name) {
		return false
	}
	for i := range name {
		if role[i] != name[i] {
			return false
		}
	}
	return true
}
func blindTargetFace(ctx context.Context, body *topo.Body, loop, segment int, capRole string, b *proofbound.ReplayBudget) (*topo.Face, bool, error) {
	var found *topo.Face
	for _, lump := range body.Lumps {
		if err := b.SpendWork(ctx, 32); err != nil {
			return nil, false, err
		}
		if lump == nil {
			return nil, false, nil
		}
		for _, shell := range lump.Shells {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, false, err
			}
			if shell == nil {
				return nil, false, nil
			}
			for _, face := range shell.Faces {
				if err := b.SpendWork(ctx, 32); err != nil {
					return nil, false, err
				}
				if face == nil || face.Body != body {
					return nil, false, nil
				}
				matched := false
				for _, origin := range face.Origins {
					if len(origin.Role) > 64 {
						return nil, false, nil
					}
					if err := b.SpendWork(ctx, uint64(len(origin.Role))+32); err != nil {
						return nil, false, err
					}
					if origin.Producer == body.Origin.Producer && ((capRole != "" && origin.Role == capRole) || (capRole == "" && blindSideRole(origin.Role, loop, segment))) {
						matched = true
					}
				}
				if !matched {
					continue
				}
				if found != nil {
					return nil, false, nil
				}
				ok, err := body.ReplayPrismBuiltFace(ctx, face, b)
				if err != nil || !ok {
					return nil, false, err
				}
				found = face
			}
		}
	}
	if found == nil {
		return nil, false, nil
	}
	return found, true, nil
}
func blindCapsEqual(a, b ReplayGroupCapFaces) bool {
	return a.Wall == b.Wall && a.Start == b.Start && a.End == b.End
}

// ReplayBlindFaceLedger requires strict construction seals on every original
// source before checking its output and source-face bindings.
func ReplayBlindFaceLedger(ctx context.Context, in BlindFaceLedgerInput, b *proofbound.ReplayBudget) (*BlindFaceLedger, bool, error) {
	if b == nil {
		return nil, false, nil
	}
	if err := b.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if in.Target == nil || in.Group == nil || !in.Target.HasStrictPrismBuildSeal() || !in.Group.HasStrictPrismBuildSeal() {
		return nil, false, nil
	}
	for _, leaf := range in.Leaves {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if leaf == nil || !leaf.HasStrictPrismBuildSeal() {
			return nil, false, nil
		}
	}
	return captureBlindFaceLedgerCandidate(ctx, in, b)
}

// captureBlindFaceLedgerCandidate checks the current source and event proofs.
// It does not authorize publication while the source construction seal is incomplete.
func captureBlindFaceLedgerCandidate(ctx context.Context, in BlindFaceLedgerInput, b *proofbound.ReplayBudget) (*BlindFaceLedger, bool, error) {
	if b == nil {
		return nil, false, nil
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return nil, false, err
	}
	if in.Output == nil || in.Target == nil || in.Group == nil || in.Events == nil || in.Output.Doc == nil || in.Output.Doc != in.Target.Doc || in.Output.Doc != in.Group.Doc || in.Output == in.Target || in.Output == in.Group || in.Target == in.Group || len(in.Leaves) < 2 || len(in.Leaves) > 4096 || len(in.Owners) != len(in.Leaves) || len(in.GroupCaps) != len(in.Leaves) || len(in.LeafCaps) != len(in.Leaves) {
		return nil, false, nil
	}
	if len(in.Output.OperationWarnings) != 0 || len(in.Target.OperationWarnings) != 0 || len(in.Group.OperationWarnings) != 0 {
		return nil, false, nil
	}
	ok, err := in.Events.Recheck(ctx, in.Output, b)
	if err != nil || !ok {
		return nil, false, err
	}
	target, ok := in.Target.Payload.(prismrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	group, ok := in.Group.Payload.(stackedrecord.Payload)
	if !ok || !group.IsGroup() {
		return nil, false, nil
	}
	regions, top, ok, err := stackedrecord.ReplayBlindOutputBinding(ctx, in.Output, target, group, in.Scene, b)
	if err != nil || !ok {
		return nil, false, err
	}
	if len(regions) != len(in.Leaves) {
		return nil, false, nil
	}
	caps, ok, err := ReplayGroupCapStructure(ctx, in.Group, b)
	if err != nil || !ok || len(caps) != len(in.GroupCaps) {
		return nil, false, err
	}
	slab := group.Slabs[0]
	place := prismplacement.Operand{Frame: group.Frame, Xform: group.Xform, Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	for i, owner := range in.Owners {
		if err := b.SpendWork(ctx, 32); err != nil {
			return nil, false, err
		}
		if owner < 0 || owner >= len(in.Leaves) || !blindCapsEqual(caps[i], in.GroupCaps[i]) {
			return nil, false, nil
		}
		for j := range i {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, false, err
			}
			if in.Owners[j] == owner {
				return nil, false, nil
			}
		}
		leaf := in.Leaves[owner]
		if leaf == nil || leaf.Doc != in.Output.Doc || leaf == in.Target || leaf == in.Group || leaf == in.Output || len(leaf.OperationWarnings) != 0 {
			return nil, false, nil
		}
		lp, yes := leaf.Payload.(prismrecord.Payload)
		if !yes {
			return nil, false, nil
		}
		lf, yes, e := ReplayPrismCapStructure(ctx, leaf, b)
		if e != nil || !yes || len(lf) != 1 || !blindCapsEqual(lf[0], in.LeafCaps[owner]) {
			return nil, false, e
		}
		yes, e = ReplayWholeCircleMatch(ctx, slab.Regions[i], lp.Profile, place, lp.PlacementOperand(), b)
		if e != nil || !yes {
			return nil, false, e
		}
	}
	for i, r := range regions {
		if err := b.SpendWork(ctx, 32); err != nil {
			return nil, false, err
		}
		if r < 0 || r >= len(in.Leaves) {
			return nil, false, nil
		}
		for j := range i {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, false, err
			}
			if regions[j] == r {
				return nil, false, nil
			}
		}
	}
	count := 2 + 2*len(regions)
	if len(target.Profile.Outer.Segments) > 4096-count {
		return nil, false, nil
	}
	count += len(target.Profile.Outer.Segments)
	for _, loop := range target.Profile.Holes {
		if err := b.SpendWork(ctx, 32); err != nil {
			return nil, false, err
		}
		if len(loop.Segments) > 4096-count {
			return nil, false, nil
		}
		count += len(loop.Segments)
	}
	if in.Events.Len() != count {
		return nil, false, nil
	}
	if err := b.ReserveRecords(ctx, uint64(count)+1); err != nil {
		return nil, false, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(BlindFaceLedger{}))); err != nil {
		return nil, false, err
	}
	if err := b.ReserveArray(ctx, uint64(count), uint64(unsafe.Sizeof(BlindFaceLedgerEntry{}))); err != nil {
		return nil, false, err
	}
	ledger := &BlindFaceLedger{output: in.Output, budget: b, entries: make([]BlindFaceLedgerEntry, 0, count)}
	for i := range count {
		if err := b.SpendWork(ctx, 64); err != nil {
			return nil, false, err
		}
		event, found := in.Events.Event(i)
		if !found || event.Face == nil || event.Face.Body != in.Output {
			return nil, false, nil
		}
		entry := BlindFaceLedgerEntry{Output: event.Face, Kind: event.Kind, Loop: event.Loop, Segment: event.Segment, Column: event.Column, Region: -1}
		switch event.Kind {
		case stackedrecord.ReplayBlindWall:
			if event.Region != 0 || event.Loop < 0 || event.Segment < 0 {
				return nil, false, nil
			}
			if event.Loop <= len(target.Profile.Holes) {
				loop := target.Profile.Outer
				if event.Loop > 0 {
					loop = target.Profile.Holes[event.Loop-1]
				}
				if event.Segment >= len(loop.Segments) || event.Start != 0 || event.End != 1 {
					return nil, false, nil
				}
				face, yes, e := blindTargetFace(ctx, in.Target, event.Loop, event.Segment, "", b)
				if e != nil || !yes {
					return nil, false, e
				}
				entry.Immediate, entry.Leaf, entry.SourceBody = face, face, in.Target
			} else {
				j := event.Loop - len(target.Profile.Holes) - 1
				if j < 0 || j >= len(regions) || event.Segment != 0 || top && (event.Start != 1 || event.End != 1) || !top && (event.Start != 0 || event.End != 0) {
					return nil, false, nil
				}
				region := regions[j]
				owner := in.Owners[region]
				entry.Region = region
				entry.Immediate = in.GroupCaps[region].Wall
				entry.Leaf = in.LeafCaps[owner].Wall
				entry.SourceBody = in.Leaves[owner]
			}
		case stackedrecord.ReplayBlindCap:
			if event.Region != 0 || event.Top && event.Slab != 1 || !event.Top && event.Slab != 0 {
				return nil, false, nil
			}
			role := provenance.RoleCapStart
			if event.Top {
				role = provenance.RoleCapEnd
			}
			face, yes, e := blindTargetFace(ctx, in.Target, 0, 0, role, b)
			if e != nil || !yes {
				return nil, false, e
			}
			yes, e = ReplayPrismCapRecord(ctx, in.Target, face, event.Top, b)
			if e != nil || !yes {
				return nil, false, e
			}
			entry.Immediate, entry.Leaf, entry.SourceBody = face, face, in.Target
			entry.Loop = -1
			if event.Top {
				entry.Segment = 1
			} else {
				entry.Segment = 0
			}
		case stackedrecord.ReplayBlindPatch:
			if event.PatchIndex < 0 || event.PatchIndex >= len(regions) || event.Floor != top {
				return nil, false, nil
			}
			region := regions[event.PatchIndex]
			owner := in.Owners[region]
			entry.Region = region
			entry.Segment = 0
			entry.Loop = -1
			entry.SourceBody = in.Leaves[owner]
			if top {
				entry.Immediate = in.GroupCaps[region].Start
				entry.Leaf = in.LeafCaps[owner].Start
			} else {
				entry.Immediate = in.GroupCaps[region].End
				entry.Leaf = in.LeafCaps[owner].End
			}
		default:
			return nil, false, nil
		}
		if entry.Immediate == nil || entry.Leaf == nil || entry.SourceBody == nil {
			return nil, false, nil
		}
		for _, previous := range ledger.entries {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, false, err
			}
			if previous.Kind == entry.Kind && previous.Loop == entry.Loop && previous.Segment == entry.Segment && previous.Region == entry.Region {
				return nil, false, nil
			}
		}
		ledger.entries = append(ledger.entries, entry)
	}
	// Each interface patch must name the same emitted column as its hole wall.
	for _, entry := range ledger.entries {
		if entry.Kind != stackedrecord.ReplayBlindPatch {
			continue
		}
		matches := 0
		for _, wall := range ledger.entries {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, false, err
			}
			if wall.Kind == stackedrecord.ReplayBlindWall && wall.Region == entry.Region && wall.Column == entry.Column {
				matches++
			}
		}
		if matches != 1 {
			return nil, false, nil
		}
	}
	ok, err = in.Events.Recheck(ctx, in.Output, b)
	if err != nil || !ok {
		return nil, false, err
	}
	return ledger, true, nil
}
