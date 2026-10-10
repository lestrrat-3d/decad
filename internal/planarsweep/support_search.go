package planarsweep

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SupportPath supplies one prepared body's geometry and motion to the plane search.
type SupportPath struct {
	Points      []proofarith.DyV3
	Solid       *planar.PlanarSolid
	Motion      MotionInput
	MovingOwner bool
}

// SupportCandidate contains the exact support set and bounds for one plane.
type SupportCandidate struct {
	Owner, Guest, Triangle   int
	Normal, Origin           proofarith.DyV3
	Local                    bool
	Heights, Rates, Spin     []*big.Rat
	Contact, Lifted          []int
	Rested                   map[int]struct{}
	NormalLow, NormalHigh    *big.Rat
	OwnerMotion, GuestMotion Motion
	Duration                 *big.Rat
}

// Curvature bounds every guest vertex's height curvature through t.
func (s SupportCandidate) Curvature(t *big.Rat) ([]*big.Rat, bool) {
	return Curvature(s.GuestMotion, s.OwnerMotion, s.Spin, s.NormalHigh, t)
}

// ClearAt checks the support's positive-height condition through t.
func (s SupportCandidate) ClearAt(t *big.Rat, k []*big.Rat, rest bool) bool {
	return ClearAt(s.Heights, s.Rates, s.Contact, s.Rested, t, k, rest)
}

// DepthAt bounds contact and rested-vertex height through t.
func (s SupportCandidate) DepthAt(t *big.Rat, k []*big.Rat, rate *big.Rat) *big.Rat {
	return DepthAt(s.Heights, s.Rates, s.Contact, s.Lifted, s.Rested, t, k, rate)
}

// ReadSupportCandidate checks the guest and the owner's face for one plane.
func ReadSupportCandidate(owner, guest SupportPath, n, a proofarith.DyV3,
	band proofarith.Dyadic, poll func() error) (SupportCandidate, bool, error) {
	read, ok, err := ReadSupport(owner.Points, guest.Points, n, a, band, owner.MovingOwner, poll)
	if err != nil || !ok {
		return SupportCandidate{}, false, err
	}
	if read.Local {
		if _, ok := planar.BuildSupportFace(owner.Solid, owner.Points, n, a); !ok {
			return SupportCandidate{}, false, nil
		}
	}
	squared := proofarith.DvDot(n, n).Rat()
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || math.IsNaN(low) || math.IsInf(low, 0) || math.IsNaN(high) || math.IsInf(high, 0) {
		return SupportCandidate{}, false, nil
	}
	return SupportCandidate{
		Normal: n, Origin: a, Local: read.Local, Heights: read.Heights,
		Contact: read.Contact, Lifted: read.Lifted,
		NormalLow: proofarith.FloatRat(low), NormalHigh: proofarith.FloatRat(high),
	}, true, nil
}

// FindSupports tries the second body first, then the first, in triangle order.
// Equal oriented planes are read once per owner.
func FindSupports(paths [2]SupportPath, band proofarith.Dyadic, rest *big.Rat,
	poll func() error) ([]SupportCandidate, error) {
	var motions [2]Motion
	var spins [2][]*big.Rat
	for i, path := range paths {
		motion, ok := MotionOf(path.Motion)
		if !ok {
			return nil, nil
		}
		spin, ok := VertexSpins(path.Points, motion)
		if !ok {
			return nil, nil
		}
		motions[i], spins[i] = motion, spin
	}
	var out []SupportCandidate
	for _, owner := range []int{1, 0} {
		guest := 1 - owner
		S, M := paths[owner], paths[guest]
		boxes := make([]proofarith.FloatBox3, len(M.Points))
		for i, v := range M.Points {
			boxes[i] = proofarith.DvFloatBox(v)
		}
		tried := make(map[string]struct{})
		var key []byte
		for triIndex, tri := range S.Solid.Tris {
			if err := poll(); err != nil {
				return nil, err
			}
			a := S.Points[tri[0]]
			n := proofarith.DvCross(proofarith.DvSub(S.Points[tri[1]], a),
				proofarith.DvSub(S.Points[tri[2]], a))
			if proofarith.DvIsZero(n) {
				continue
			}
			key = PlaneKey(key[:0], n, a)
			if _, ok := tried[string(key)]; ok {
				continue
			}
			tried[string(key)] = struct{}{}
			if SupportRuledOut(boxes, n, a, band) {
				continue
			}
			support, ok, err := ReadSupportCandidate(S, M, n, a, band, poll)
			if err != nil {
				return nil, err
			}
			if !ok {
				continue
			}
			support.Owner, support.Guest, support.Triangle = owner, guest, triIndex
			support.OwnerMotion, support.GuestMotion = motions[owner], motions[guest]
			support.Duration = paths[0].Motion.Duration
			support.Rates = Rates(M.Points, n, support.GuestMotion, support.OwnerMotion)
			support.Spin = spins[guest]
			support.Rested = RestedVertices(support.Lifted, support.Rates, support.NormalLow, rest)
			out = append(out, support)
		}
	}
	return out, nil
}
