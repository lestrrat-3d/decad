package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// carrierClaimPossible is only an allocation precheck; the full proof below
// validates each claim and every held point.
func carrierClaimPossible(f *topo.Face) bool {
	return f != nil && (f.CapRecord != nil || f.RetainedTag && f.PlacedPlane != nil)
}

// separatedCarrierFaces admits a proximity pair only when each true patch
// and all of its held facets lie in an exact plane, and those two planes are
// parallel and distinct. It proves neither trim containment nor intersections
// with other faces.
func separatedCarrierFaces(a, b faceFacets, ma, mb *BoolMesh, budget *proofbound.WorkBudget) (bool, error) {
	if a.source == nil || b.source == nil || ma == nil || mb == nil || budget == nil {
		return false, nil
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	// Exact conversion, carrier arithmetic and per-corner plane tests allocate
	// rationals. Plane origins include finite binary64 inputs and at most one
	// 4096-bit denoted axial coordinate per native cap.
	if err := budget.ReserveBytes(256 << 10); err != nil {
		return false, err
	}
	plane := func(f *topo.Face) *exactPlane {
		if _, ok := f.Surface.(surfacegeom.Plane); !ok {
			return nil
		}
		if f.CapRecord != nil {
			c := f.CapRecord
			if c.Xform != r3.Identity() || !proofbound.FiniteVec(c.Frame.Origin()) || !proofbound.FiniteVec(c.Frame.U()) || !proofbound.FiniteVec(c.Frame.V()) || !proofbound.FiniteVec(c.Frame.N()) {
				return nil
			}
			var z *big.Rat
			if c.ZDenoted != nil {
				if c.ZDenoted.Num().BitLen() > 4096 || c.ZDenoted.Denom().BitLen() > 4096 {
					return nil
				}
				z = new(big.Rat).Set(c.ZDenoted)
			} else if f.HasAxialDelta && f.AxialDelta == 0 && !math.IsNaN(c.Z) && !math.IsInf(c.Z, 0) {
				z = new(big.Rat).SetFloat64(c.Z)
			} else {
				return nil
			}
			o, n := ratVec(c.Frame.Origin()), ratVec(c.Frame.N())
			for i := range 3 {
				o[i].Add(o[i], new(big.Rat).Mul(n[i], z))
			}
			normal := ratCross(ratVec(c.Frame.U()), ratVec(c.Frame.V()))
			if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
				return nil
			}
			return &exactPlane{o: o, n: normal}
		}
		if f.Denoted != nil {
			return nil
		}
		return trueFacePlane(f)
	}
	pa := plane(a.source)
	pb := plane(b.source)
	if pa == nil || pb == nil {
		return false, nil
	}
	cross := ratCross(pa.n, pb.n)
	if cross[0].Sign() != 0 || cross[1].Sign() != 0 || cross[2].Sign() != 0 {
		return false, nil
	}
	distance := new(big.Rat)
	for i := range 3 {
		distance.Add(distance, new(big.Rat).Mul(pa.n[i], new(big.Rat).Sub(pb.o[i], pa.o[i])))
	}
	if distance.Sign() == 0 {
		return false, nil
	}
	holds := func(m *BoolMesh, facets []int, p *exactPlane) (bool, error) {
		if len(facets) == 0 {
			return false, nil
		}
		for _, i := range facets {
			if err := budget.ReserveBytes(32 << 10); err != nil {
				return false, err
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if i < 0 || i >= len(m.Tris) {
				return false, nil
			}
			for _, v := range m.Tris[i] {
				if v < 0 || v >= len(m.Verts) || !proofbound.FiniteVec(m.Verts[v]) || !p.holds(m.Verts[v]) {
					return false, nil
				}
			}
		}
		return true, nil
	}
	ok, err := holds(ma, a.facets, pa)
	if err != nil || !ok {
		return false, err
	}
	return holds(mb, b.facets, pb)
}
