package loftcylinder

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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

// BindCapSetback admits one equal setback d for both cap bands and then
// binds the bridge (BindBridge). dDelta is the setback's asymmetric part and
// must be zero. The exact band levels z0+d and z1−d must be ordered, d must
// move each float cap level, and the bound top source level must lie above
// z1−d. The level checks run before the bind, so their refusals leave s
// unchanged.
func (s *ChamferSource) BindCapSetback(d, dDelta float64) error {
	if dDelta != 0 || d <= 0 || !proofbound.FiniteVec(r3.Vec{X: d, Y: s.Z0, Z: s.Z1}) {
		return fmt.Errorf("%w: loft/cylinder cap setback has no exact millimeter magnitude", decaderr.ErrUnsupported)
	}
	start := new(big.Rat).Add(proofarith.FloatRat(s.Z0), proofarith.FloatRat(d))
	end := new(big.Rat).Sub(proofarith.FloatRat(s.Z1), proofarith.FloatRat(d))
	if start.Cmp(end) >= 0 || s.Z0+d == s.Z0 || s.Z1-d == s.Z1 {
		return fmt.Errorf("%w: loft/cylinder cap bands reach each other or round onto a cap", decaderr.ErrUnsupported)
	}
	if err := s.BindBridge(); err != nil {
		return err
	}
	if s.TopSourceLevel.Cmp(end) <= 0 {
		return fmt.Errorf("%w: cap chamfer bridge reaches its side level", decaderr.ErrUnsupported)
	}
	return nil
}
