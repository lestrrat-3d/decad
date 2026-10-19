package loftcylinder

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ChamferSource is the already audited cap and side correspondence of an
// axial loft joined to an exact circular blank. The cap loops are CCW when
// viewed from +Z, including the start cap whose public coedges are CW.
// Teeth are the private proof teeth and OriginalTeeth the exact Union
// operands, in the same order. BindBridge sets BridgeBound and TopSourceLevel.
type ChamferSource struct {
	Blank          *topo.Body
	Teeth          []*topo.Body
	OriginalTeeth  []*topo.Body
	AxialBridge    []float64
	BridgeBound    float64
	TopSourceLevel *big.Rat
	// Optional physical-height correction for a reflected, axially bridged
	// source tooth. The ring's shared held level stays unchanged.
	RingLevelShift []*big.Rat
	Outer, Inner   [2][]int
	CapGroups      [2]int
	BoreGroup      int
	BlankRadius    float64
	BoreRadius     float64
	BoreBound      float64
	InnerInset     float64
	OuterExcess    float64
	RootCollar     float64
	Z0, Z1         float64
}

// ChamferSourceOf reads the source of an audited, unplaced loft/cylinder
// join or its centered bore result. A bore record wins over a join record;
// a join source has no bore, so its BoreGroup is -1.
func ChamferSourceOf(fp facetedrecord.Payload) (ChamferSource, bool) {
	if !fp.ContactAudited || fp.Xform != r3.Identity() {
		return ChamferSource{}, false
	}
	if r := fp.LoftCylinderBore; r != nil {
		return ChamferSource{
			Blank: r.Blank, Teeth: r.ProofTeeth,
			OriginalTeeth: r.Teeth, AxialBridge: r.AxialBridge,
			Outer: r.Outer, Inner: r.Inner, CapGroups: r.CapGroups,
			BoreGroup: r.WallGroup, BlankRadius: r.BlankRadius,
			BoreRadius: r.BoreRadius, BoreBound: r.BoreBound, InnerInset: r.InnerInset,
			OuterExcess: r.OuterExcess, RootCollar: r.RootCollar,
			Z0: r.Z0, Z1: r.Z1,
		}, true
	}
	if r := fp.LoftCylinderJoin; r != nil {
		return ChamferSource{
			Blank: r.Blank, Teeth: r.ProofTeeth,
			OriginalTeeth: r.Teeth, AxialBridge: r.AxialBridge,
			Outer: r.CapOutlines, CapGroups: r.CapGroups,
			BoreGroup: -1, BlankRadius: r.Radius,
			InnerInset: r.InnerInset, OuterExcess: r.OuterExcess,
			RootCollar: r.RootCollar, Z0: r.Z0, Z1: r.Z1,
		}, true
	}
	return ChamferSource{}, false
}
