package revolveshell

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SectorJoinPlan holds the numeric mesh plan for one certified sector.
type SectorJoinPlan struct {
	Nodes                      []pointcone.SectorNode
	Sector                     [2]float64
	RootNode                   []int
	Motion                     r3.Transform
	Charge                     massmoment.MapCharge
	PlacedRound, CrossingBound float64
	NodeRound, Snap            float64
	CapGroups                  [2]int
}

// SectorJoinInput contains the bounded source readings needed to place sectors.
type SectorJoinInput struct {
	Original         []pointcone.SectorNode
	RootContactIndex int // Index in the certified loop before subdivision.
	Edges            []pointcone.SectorEdge
	Motions          []r3.Transform
	Charges          []massmoment.MapCharge
	// Drifts holds each motion's drift charge ε per unit of radius
	// (massmoment.AxisPlacementCharge). For the canonical axis it equals the
	// charge's Stretch.
	Drifts              []float64
	SourceMeshBounds    []float64
	SourceCrossingBound float64
	Seam, MinRadius     float64
	RegularSource       []float64
	// Tilted reports that the certificate is general, so every pair's seam
	// cost takes τ = (π/2)·FrameTilt·r_max, with FrameTilt the δ_B of the
	// blank's angular frame (massmoment.FrameTilt). A canonical certificate
	// leaves it false and FrameTilt unread (agent-docs/revolve-axis-design.md
	// §5.6).
	Tilted    bool
	FrameTilt float64
	// Angles measures every placed root node's angle. Its zero value is the
	// +X convention; a general certificate passes its own AngleBasis.
	Angles facetproof.AngleBasis
}

// PrepareSectorJoinPlans selects one angular ring for all authenticated
// sectors. A later sector's PlacedRound charges its drift ε, never the bare
// Stretch, against every node's radius (agent-docs/revolve-axis-design.md
// §5.5).
func PrepareSectorJoinPlans(input SectorJoinInput) ([]SectorJoinPlan, []float64, error) {
	if len(input.Drifts) != len(input.Motions) || len(input.Charges) != len(input.Motions) {
		return nil, nil, fmt.Errorf("%w: sector placements have no drift charge", decaderr.ErrUnsupported)
	}
	plans := make([]SectorJoinPlan, len(input.Motions))
	for i := range plans {
		plan := &plans[i]
		plan.Motion, plan.Charge = input.Motions[i], input.Charges[i]
		plan.Nodes = append([]pointcone.SectorNode(nil), input.Original...)
		for j, node := range input.Original {
			point := input.Motions[i].Apply(node.P)
			if !proofbound.FiniteVec(point) {
				return nil, nil, fmt.Errorf("%w: placed cap has a non-finite station", decaderr.ErrUnsupported)
			}
			plan.Nodes[j].P = point
			if node.RootBoundary {
				plan.Nodes[j].Angle = input.Angles.AngleAt(point, input.Seam)
			}
			if i != 0 {
				maxAbs := max(math.Abs(node.P.X), math.Abs(node.P.Y), math.Abs(node.P.Z))
				plan.PlacedRound = max(plan.PlacedRound, proofbound.AbsSumUpper(
					proofbound.RigidRoundAllow(maxAbs, 0),
					proofbound.ProductUpper(input.Drifts[i], proofbound.Radius3D(maxAbs))))
			}
		}
		a, b := plan.Nodes[0].Angle, plan.Nodes[input.RootContactIndex].Angle
		if a == b || math.Abs(a-b) >= math.Pi {
			return nil, nil, fmt.Errorf("%w: placed root contacts cross the angular seam", decaderr.ErrUnsupported)
		}
		plan.Sector = [2]float64{min(a, b), max(a, b)}
		plan.CrossingBound = proofbound.AbsSumUpper(
			massmoment.PlacedBound(input.SourceCrossingBound, input.Charges[i]), plan.PlacedRound)
		if !pointcone.SectorRootEdgeOrdered(plan.Nodes, input.Edges, plan.Sector) {
			return nil, nil, fmt.Errorf("%w: placed root stations lose angular order", decaderr.ErrUnsupported)
		}
	}
	minRadius := input.MinRadius
	tau := 0.0
	if input.Tilted {
		tau = frameTiltSeamCost(plans, input.FrameTilt)
	}
	for i := range plans {
		for j := i + 1; j < len(plans); j++ {
			seamCost := proofbound.AbsSumUpper(plans[i].CrossingBound,
				plans[j].CrossingBound,
				massmoment.PlacedBound(input.SourceMeshBounds[i], input.Charges[i]),
				massmoment.PlacedBound(input.SourceMeshBounds[j], input.Charges[j]))
			if input.Tilted {
				seamCost = proofbound.AbsSumUpper(seamCost, tau)
			}
			if !(minRadius > 0 && seamCost < minRadius/2) {
				return nil, nil, fmt.Errorf("%w: root sectors have no angular error bound", decaderr.ErrUnsupported)
			}
			angleCost := proofbound.DivUpper(proofbound.ProductUpper(2, seamCost), minRadius)
			if !(facetproof.SectorAngularGap(plans[i].Sector, plans[j].Sector) > angleCost) {
				return nil, nil, fmt.Errorf("%w: root sectors are not separate", decaderr.ErrUnsupported)
			}
		}
	}
	stations := len(input.RegularSource) - 2
	if stations < 16 || stations > 8192 {
		return nil, nil, fmt.Errorf("%w: sector has no bounded angular plan", decaderr.ErrUnsupported)
	}
	regular := make([]float64, 0, stations+2*len(plans))
	for i := range stations {
		regular = append(regular, input.Seam+2*math.Pi*float64(i)/float64(stations))
	}
	for _, plan := range plans {
		regular = append(regular, plan.Sector[:]...)
	}
	outside := regular[:0]
	for _, angle := range regular {
		covered := false
		for _, plan := range plans {
			covered = covered || plan.Sector[0] < angle && angle < plan.Sector[1]
		}
		if !covered {
			outside = append(outside, angle)
		}
	}
	angles := outside
	var err error
	for i := range plans {
		angles, _, err = pointcone.SectorMergedAngles(angles, plans[i].Nodes)
		if err != nil {
			return nil, nil, err
		}
	}
	for i := range plans {
		_, plans[i].RootNode, err = pointcone.SectorMergedAngles(angles, plans[i].Nodes)
		if err != nil {
			return nil, nil, err
		}
	}
	return plans, angles, nil
}

// frameTiltSeamCost returns τ = (π/2)·δ_B·r_max rounded up, where r_max is a
// proven upper bound on |p| over every root-boundary node p of every plan,
// placed copies included (agent-docs/revolve-axis-design.md §5.6). A node's
// angle in the held frame and its angle about the axis differ by at most
// τ/ρ_min, so τ added to a pair's seam cost covers both nodes' angle errors
// once the cost is turned into an angle by 2·cost/minRadius. The seam test
// requires cost < minRadius/2, which implies δ_B·r_max < ρ_min as the bound
// requires. A non-finite reading answers +Inf, which that test refuses.
func frameTiltSeamCost(plans []SectorJoinPlan, frameTilt float64) float64 {
	rMax := 0.0
	for _, plan := range plans {
		for _, node := range plan.Nodes {
			if node.RootBoundary {
				rMax = max(rMax, proofbound.DvLenUpper(proofarith.DyVec(node.P)))
			}
		}
	}
	halfPi := proofbound.RatFloatUp(proofbound.HalfPiInterval().Hi)
	tau := proofbound.ProductUpper(halfPi, proofbound.ProductUpper(frameTilt, rMax))
	if proofbound.IsNonFinite(tau) || !(frameTilt >= 0) {
		return math.Inf(1)
	}
	return tau
}

// SectorJoinBoundInput contains the source and cap bounds shared by all sectors.
type SectorJoinBoundInput struct {
	BaseMeshBound, BaseVolSymDiff, BaseAreaSlack, BaseArea float64
	SectionSag, ConeGap, ToolGap, FarRound, Lip            float64
	UpperBound, LowerBound, UpperLip, LowerLip             float64
	CapArea, CapFactor                                     float64
}

// SectorJoinBounds accumulates source and output bounds in sector order.
type SectorJoinBounds struct {
	LocalBound, LocalRound, SourceVolume, SourceArea float64
	RefineCapLimit                                   float64
	CapRefinePossible                                bool
}

// NewSectorJoinBounds starts the ordered bounds for one joined construction.
func NewSectorJoinBounds() SectorJoinBounds {
	return SectorJoinBounds{RefineCapLimit: math.Inf(1), CapRefinePossible: true}
}

// AccumulateSectorJoinBounds charges one mesh sector after its snap check.
func AccumulateSectorJoinBounds(state *SectorJoinBounds, plan SectorJoinPlan,
	input SectorJoinBoundInput, i int) error {
	sectionSag, toolGap, farRound, lip := input.SectionSag, input.ToolGap, input.FarRound, input.Lip
	upperBound, lowerBound := input.UpperBound, input.LowerBound
	upperLip, lowerLip := input.UpperLip, input.LowerLip
	capArea, capFactor, baseArea := input.CapArea, input.CapFactor, input.BaseArea
	localBound, localRound := state.LocalBound, state.LocalRound
	sourceVolume, sourceArea := state.SourceVolume, state.SourceArea
	refineCapLimit, capRefinePossible := state.RefineCapLimit, state.CapRefinePossible
	seamAllowance := proofbound.AbsSumUpper(input.BaseMeshBound, sectionSag,
		input.ConeGap, plan.CrossingBound, toolGap, plan.PlacedRound)
	if plan.Snap > seamAllowance {
		return &sectorSnapError{sector: i, snap: plan.Snap, allowance: seamAllowance}
	}
	stretch := proofbound.AbsSumUpper(1, plan.Charge.Stretch)
	farTerm := proofbound.ProductUpper(proofbound.ProductUpper(farRound, stretch), lip)
	baseTerm := proofbound.ProductUpper(proofbound.AbsSumUpper(
		massmoment.PlacedBound(input.BaseMeshBound, plan.Charge),
		plan.CrossingBound, plan.PlacedRound), lip)
	capTerm := proofbound.ProductUpper(max(upperBound, lowerBound), stretch)
	otherBound := proofbound.AbsSumUpper(plan.Snap, plan.NodeRound,
		farTerm, baseTerm, input.ConeGap, toolGap)
	remaining := 0.1 - sectionSag - otherBound
	if remaining <= 0 || proofbound.IsNonFinite(remaining) {
		capRefinePossible = false
	} else {
		refineCapLimit = min(refineCapLimit, 0.9*remaining/stretch)
	}
	localBound = max(localBound, proofbound.AbsSumUpper(plan.Snap, plan.NodeRound,
		farTerm, baseTerm, capTerm, input.ConeGap, toolGap))
	localRound = max(localRound, proofbound.AbsSumUpper(plan.Snap, plan.NodeRound,
		proofbound.ProductUpper(proofbound.ProductUpper(farRound, stretch), lip),
		proofbound.ProductUpper(proofbound.AbsSumUpper(
			plan.CrossingBound, plan.PlacedRound), lip), input.ConeGap, toolGap))
	volumePart := proofbound.AbsSumUpper(input.BaseVolSymDiff,
		proofbound.ProductUpper(proofbound.ProductUpper(capArea,
			proofbound.AbsSumUpper(upperBound, lowerBound)), capFactor))
	areaPart := proofbound.AbsSumUpper(baseArea, input.BaseAreaSlack,
		proofbound.ProductUpper(proofbound.AbsSumUpper(1,
			proofbound.ProductUpper(upperLip, upperLip),
			proofbound.ProductUpper(lowerLip, lowerLip)),
			proofbound.AbsSumUpper(capArea, input.BaseAreaSlack)))
	if i != 0 {
		volumePart = proofbound.ProductUpper(
			proofbound.AbsSumUpper(1, plan.Charge.Volume), volumePart)
		areaPart = proofbound.ProductUpper(stretch, areaPart)
	}
	sourceVolume = proofbound.AbsSumUpper(sourceVolume, volumePart)
	sourceArea = proofbound.AbsSumUpper(sourceArea, areaPart)
	state.LocalBound, state.LocalRound = localBound, localRound
	state.SourceVolume, state.SourceArea = sourceVolume, sourceArea
	state.RefineCapLimit, state.CapRefinePossible = refineCapLimit, capRefinePossible
	return nil
}

// RootCapContact permits a retry only for a mapped cap contacting the Revolve face
// adjacent to its authenticated root span. Every candidate faces the full
// contact, topology, and measure audit again.
func RootCapContact(src, ringGroups []int, rootSpan int, plans []SectorJoinPlan, err error) bool {
	var contact *loftmesh.LoftContactError
	if !errors.As(err, &contact) ||
		contact.Reason != "share no recorded vertex, but make contact" ||
		contact.I < 0 || contact.J < 0 ||
		contact.I >= len(src) || contact.J >= len(src) ||
		len(plans) == 0 || rootSpan < 0 ||
		rootSpan >= len(ringGroups) {
		return false
	}
	prev := (rootSpan + len(ringGroups) - 1) % len(ringGroups)
	next := (rootSpan + 1) % len(ringGroups)
	isNeighbor := func(group int) bool {
		return group >= 0 && (group == ringGroups[prev] || group == ringGroups[next])
	}
	isCap := func(group int) bool {
		for _, plan := range plans {
			if group == plan.CapGroups[0] || group == plan.CapGroups[1] {
				return true
			}
		}
		return false
	}
	a, b := src[contact.I], src[contact.J]
	return isCap(a) && isNeighbor(b) || isCap(b) && isNeighbor(a)
}

// sectorSnapError reports a placed cap whose root contact lies farther from
// the ring stations than the seam allowance. A finer angular mesh moves the
// stations closer, so the refinement loop retries with a smaller sag.
type sectorSnapError struct {
	sector          int
	snap, allowance float64
}

func (e *sectorSnapError) Error() string {
	return fmt.Sprintf("%v: placed cap misses the finite root patch (sector %d snap %.9g allowance %.9g)",
		decaderr.ErrUnsupported, e.sector, e.snap, e.allowance)
}

func (e *sectorSnapError) Unwrap() error { return decaderr.ErrUnsupported }
