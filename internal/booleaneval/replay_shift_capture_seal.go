package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// shiftCaptureSeal retains the capture-time content and pointer identities.
type shiftCaptureSeal struct {
	groups   []facetedrecord.Group
	pointers []shiftGroupPointers
	sources  []shiftSourceSeal
}

func captureShiftGroupSeal(ctx context.Context, groups []facetedrecord.Group, budget *proofbound.ReplayBudget) (*shiftCaptureSeal, error) {
	if budget == nil {
		return nil, errCaptureSealMissingCaptureBudget
	}
	if err := budget.ReserveRecords(ctx, uint64(len(groups))+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(shiftCaptureSeal{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(groups)), uint64(unsafe.Sizeof(shiftGroupPointers{}))); err != nil {
		return nil, err
	}
	copies, err := snapshotShiftGroups(ctx, groups, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(len(groups))); err != nil {
		return nil, err
	}
	pointers := make([]shiftGroupPointers, len(groups))
	var count uint64
	for i, g := range groups {
		pointers[i].cap, pointers[i].denoted = g.CapRecord, g.Denoted
		if g.CapRecord != nil {
			pointers[i].level = g.CapRecord.ZDenoted
		}
		for p, depth := g.Source, 0; p != nil; p, depth = p.Parent, depth+1 {
			if depth >= 64 {
				return nil, errCaptureSealCaptureSourceAncestryTooDeep
			}
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			count++
		}
	}
	if err := budget.ReserveRecords(ctx, count); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, count, uint64(unsafe.Sizeof(shiftSourceSeal{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, count); err != nil {
		return nil, err
	}
	sources := make([]shiftSourceSeal, 0, count)
	for _, g := range groups {
		for p := g.Source; p != nil; p = p.Parent {
			if uint64(len(sources)) >= count {
				return nil, errCaptureSealCaptureAncestryChanged
			}
			sources = append(sources, shiftSourceSeal{ptr: p, value: *p})
		}
	}
	if uint64(len(sources)) != count {
		return nil, errCaptureSealCaptureAncestryChanged
	}
	return &shiftCaptureSeal{groups: copies, pointers: pointers, sources: sources}, nil
}

func validateShiftCaptureSeal(ctx context.Context, cert *ShiftCertificate, budget *proofbound.ReplayBudget) error {
	if cert == nil || cert.captureSeal == nil || budget == nil {
		return errCaptureSealMissingCaptureSeal
	}
	seal := cert.captureSeal
	if len(cert.groups) != len(seal.groups) || len(cert.groups) != len(seal.pointers) {
		return errCaptureSealCapturedGroupsChanged
	}
	if err := budget.SpendWork(ctx, uint64(len(seal.sources))+uint64(len(seal.pointers))); err != nil {
		return err
	}
	for _, source := range seal.sources {
		if source.ptr == nil || *source.ptr != source.value {
			return errCaptureSealCapturedSourceChanged
		}
	}
	for i, a := range cert.groups {
		b := seal.groups[i]
		p := seal.pointers[i]
		if a.CapRecord != p.cap || a.Denoted != p.denoted || (a.CapRecord != nil && a.CapRecord.ZDenoted != p.level) {
			return errCaptureSealCapturedGroupPointerChanged
		}
		if len(a.Origins) != len(b.Origins) || len(a.Upstream) != len(b.Upstream) {
			return errCaptureSealCapturedProvenanceChanged
		}
		n := uint64(len(a.Origins)) + uint64(len(a.Upstream)) + 1
		if n > ^uint64(0)/512 {
			return errCaptureSealCaptureProvenanceTooLarge
		}
		if err := budget.SpendWork(ctx, n*512); err != nil {
			return err
		}
		if (a.Denoted == nil) != (b.Denoted == nil) {
			return errCaptureSealCapturedDenotationChanged
		}
		if a.Denoted != nil {
			if len(a.Denoted.Runs) != len(b.Denoted.Runs) || len(a.Denoted.Ends) != len(b.Denoted.Ends) {
				return errCaptureSealCapturedDenotationChanged
			}
			count := uint64(len(a.Denoted.Runs)) + uint64(len(a.Denoted.Ends)) + 16
			if count > ^uint64(0)/4096 {
				return errCaptureSealCapturedDenotationTooLarge
			}
			if err := budget.SpendWork(ctx, count*4096); err != nil {
				return err
			}
		}
		ca, cb := a.CapRecord, b.CapRecord
		a.CapRecord, b.CapRecord = nil, nil
		if !sameShiftGroup(a, b) || (ca == nil) != (cb == nil) {
			return errCaptureSealCapturedGroupChanged
		}
		if ca != nil {
			if !sameFrame(ca.Frame, cb.Frame) || !sameTransform(ca.Xform, cb.Xform) || !sameFloat(ca.Z, cb.Z) {
				return errCaptureSealCapturedCapChanged
			}
			if err := budget.SpendWork(ctx, 256); err != nil {
				return err
			}
			if !sameRat(ca.ZDenoted, cb.ZDenoted) {
				return errCaptureSealCapturedCapChanged
			}
		}
	}
	return nil
}
