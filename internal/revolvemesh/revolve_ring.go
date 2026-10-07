package revolvemesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// EmitRings builds each meridian sample's placed vertices and assigns its ring
// indices. It measures construction and placement rounding separately.
func EmitRings(loops [][]RevMeridian, angular RevolveAngular, basis RevolveBasis,
	idealBasis RevolveBasis3Iv, xform r3.Transform, full bool,
	budget *proofbound.WorkBudget) ([]r3.Vec, float64, float64, error) {
	var vertices []r3.Vec
	deltaC, deltaR := 0.0, 0.0
	// radials[l] is the ideal radial direction at angular index l, the term
	// of RevolveIdealPoint's sum that every ring shares; the pole's covers
	// every angle at once.
	radials := make([]proofbound.IvVec3, angular.Samples)
	for l := range angular.Samples {
		radials[l] = proofbound.IvVec3Add(proofbound.IvVec3Mul(idealBasis.E0, angular.CosIv[l]), proofbound.IvVec3Mul(idealBasis.E1, angular.SinIv[l]))
	}
	poleIv := proofbound.Interval(minusOneRat(), oneRat())
	poleRadial := proofbound.IvVec3Add(proofbound.IvVec3Mul(idealBasis.E0, poleIv), proofbound.IvVec3Mul(idealBasis.E1, poleIv))
	for li := range loops {
		for si := range loops[li] {
			s := &loops[li][si]
			count := angular.Samples
			if s.OnAxis {
				count = 1
			}
			s.Ring = make([]int, count)
			axial := proofbound.IvVec3Mul(idealBasis.W, s.ZIv)
			// docs/tessellation-design.md §9's ring-collapse detection, run
			// BEFORE and AFTER placement: a sample with ρ > 0 whose angular
			// vertices coincide is not an axis sample, and §12 forbids merging
			// it into a pole. Both stages are compared because either can
			// collapse a ring the other keeps apart.
			var prevLocal, prevPlaced r3.Vec
			var firstLocal, firstPlaced r3.Vec
			for l := range count {
				if err := budget.Step(); err != nil {
					return nil, 0, 0, err
				}
				cos, sin := angular.Cos[l], angular.Sin[l]
				local := basis.A3.Add(basis.W.Scale(s.Z)).Add(basis.E0.Scale(cos).Add(basis.E1.Scale(sin)).Scale(s.Rho))
				placed := xform.Apply(local)
				if !proofbound.FiniteVec(local) || !proofbound.FiniteVec(placed) {
					return nil, 0, 0, fmt.Errorf(`%w: a revolve mesh vertex is not finite`, decaderr.ErrUnsupported)
				}
				radial := radials[l]
				if s.OnAxis {
					// A pole's single vertex stands for the ideal sample at
					// EVERY angle, so its enclosure must cover them all.
					radial = poleRadial
				}
				ideal := proofbound.IvVec3Add(idealBasis.A3, proofbound.IvVec3Add(axial, proofbound.IvVec3Mul(radial, s.RhoIv)))
				gapC := proofbound.Radius3D(max(
					proofbound.IntervalFloatError(ideal[0], local.X),
					proofbound.IntervalFloatError(ideal[1], local.Y),
					proofbound.IntervalFloatError(ideal[2], local.Z),
				))
				gapR := ExactRigidPointRound(xform, local, placed)
				if proofbound.IsNonFinite(gapC) || proofbound.IsNonFinite(gapR) {
					return nil, 0, 0, fmt.Errorf(`%w: a revolve mesh vertex states no bound on the rounding its own construction committed`, decaderr.ErrUnsupported)
				}
				deltaC = math.Max(deltaC, gapC)
				deltaR = math.Max(deltaR, gapR)
				if l == 0 {
					firstLocal, firstPlaced = local, placed
				} else if !s.OnAxis && (local == prevLocal || placed == prevPlaced) {
					return nil, 0, 0, errRevolveRingCollapse
				}
				prevLocal, prevPlaced = local, placed
				s.Ring[l] = len(vertices)
				vertices = append(vertices, placed)
			}
			// A full turn closes onto its own first vertex, so the wrap is the
			// one adjacent pair the walk above never compared.
			if full && !s.OnAxis && count > 1 && (prevLocal == firstLocal || prevPlaced == firstPlaced) {
				return nil, 0, 0, errRevolveRingCollapse
			}
		}
	}
	return vertices, deltaC, deltaR, nil
}

func oneRat() *big.Rat      { return big.NewRat(1, 1) }
func minusOneRat() *big.Rat { return big.NewRat(-1, 1) }

var errRevolveRingCollapse = fmt.Errorf(`%w: a revolve ring at a positive radius collapses onto itself at this angular count, and docs/tessellation-design.md §9 forbids merging it into a pole; retry with a coarser tolerance`, decaderr.ErrUnsupported)
