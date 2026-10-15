package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The source-ring audit supplies a true-result boundary point on every
// split edge. Exact distance to that ring then bounds every band facet.
func loftCylinderChamferNearBandReach(ctx context.Context, mesh *joinedMesh,
	fp facetedPayload, source loftCylinderChamferSource, ideal []proofarith.Xpt, bands [2][2]int,
	clipBound, publishedBound float64, multipleRings bool, reflectedShift []*big.Rat) error {
	reach := math.Nextafter(publishedBound-clipBound, math.Inf(-1))
	if reach <= 0 || proofbound.IsNonFinite(reach) {
		return fmt.Errorf("%w: cap band has no boundary allowance", ErrUnsupported)
	}
	reachRat := proofarith.FloatRat(reach)
	reach2 := new(big.Rat).Mul(reachRat, reachRat)
	fractions := []*big.Rat{big.NewRat(11, 20)}
	if multipleRings {
		fractions = []*big.Rat{big.NewRat(1, 4), big.NewRat(1, 2), big.NewRat(3, 4)}
	}
	teeth := make(loftCylinderRingTeeth)
	for cap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		capLevel := proofarith.FloatRat(source.z0)
		if cap == 1 {
			capLevel = proofarith.FloatRat(source.z1)
		}
		var near [][3]proofarith.Xpt
		var edges []loftmesh.ChamferSplitEdge
		for _, fraction := range fractions {
			var sourceEdges [][2]proofarith.Xpt
			for i := band[0]; i < band[1]; i++ {
				parts, edge, err := loftmesh.ChamferNearPieces(mesh.Tris[i], ideal, capLevel, fraction)
				if err != nil {
					return err
				}
				if !multipleRings {
					near = append(near, parts...)
					far, err := loftmesh.ChamferFarTriangles(mesh.Tris[i], ideal, capLevel, fraction)
					if err != nil {
						return err
					}
					near = append(near, far...)
				}
				edges = append(edges, edge)
				sourceEdges = append(sourceEdges, [2]proofarith.Xpt{edge.A, edge.B})
			}
			// The direct and the reflected check read the same edges.
			records := new(loftcylinder.RingEdgeRecords)
			if _, err := loftCylinderSourceRingInside(ctx, fp, sourceEdges, teeth, records); err != nil {
				return err
			}
			if len(reflectedShift) != 0 {
				shifted := source
				shifted.ringLevelShift = reflectedShift
				if _, err := loftCylinderSourceRingInsideInputs(ctx, shifted, sourceEdges, teeth, records); err != nil {
					return fmt.Errorf("reflected cap split ring: %w", err)
				}
			}
		}
		if multipleRings {
			for i := band[0]; i < band[1]; i++ {
				tri := mesh.Tris[i]
				near = append(near, [3]proofarith.Xpt{ideal[tri[0]], ideal[tri[1]], ideal[tri[2]]})
			}
		}
		grid, err := loftmesh.ChamferEdgeGridOf(edges, reachRat)
		if err != nil {
			return err
		}
		maxSteps := min(8_000_000, 512*len(edges))
		depth := 10
		if multipleRings {
			maxSteps = min(8_000_000, 2_048*len(edges))
			depth = 12
		}
		steps := 0
		for i, tri := range near {
			if !loftmesh.ChamferNearCovered(ctx, tri, grid, reach2, depth, &steps, maxSteps) {
				var diagnostic [4]float64
				points := [4]proofarith.Xpt{tri[0], tri[1], tri[2],
					meshbool.XCentroid(tri[0], tri[1], tri[2])}
				for k, p := range points {
					diagnostic[k] = math.Inf(1)
					for _, edge := range edges {
						diagnostic[k] = min(diagnostic[k],
							meshbool.SegSegDist3(p.Vec(), p.Vec(), edge.A.Vec(), edge.B.Vec()))
					}
				}
				return fmt.Errorf("%w: near cap band %d facet %d has no certified split-ring reach after %d checks: distances=%v bound=%.17g",
					ErrUnsupported, cap, i, steps, diagnostic, publishedBound)
			}
		}
	}
	return nil
}
