package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// loftQuarterWallOrderProof is a private source-only reading. It identifies
// the two rectangle edges crossing one paired quarter-arc wall at every
// height. It does not select output patches or certify held-mesh coverage.
type loftQuarterWallOrderProof struct {
	prism, loft  *Body
	arcSegment   int
	lower, upper int // source indices of the rectangle's lower and upper edges
	radius       [2]*big.Rat
	wallFaces    [2][]provenance.Ref
}

// certifyLoftQuarterWallOrder accepts only a concentric, first-quadrant
// quarter-arc pair and one axis-aligned rectangle. The caller passes the
// authenticated source spans and their original 4096-step source wall.
func certifyLoftQuarterWallOrder(ctx context.Context, budget *proofbound.WorkBudget,
	source loftSourceSpanCaps) (loftQuarterWallOrderProof, bool, error) {
	var no loftQuarterWallOrderProof
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if budget == nil || source.prism == nil || source.loft == nil ||
		source.prism == source.loft {
		return no, false, nil
	}
	pp, prismOK := source.prism.n.Payload.(prismPayload)
	lp, loftOK := source.loft.n.Payload.(loftPayload)
	if !prismOK || !loftOK || !lp.AuthenticatedReconstruction ||
		lp.SourceIdentity == nil || lp.SourceIdentity.Origin != source.loft.n.Origin.Producer ||
		len(lp.Alignment) > 1 || len(lp.Alignment) == 1 && lp.Alignment[0] != 0 ||
		len(pp.Profile.Outer.Segments) != 4 ||
		len(lp.Profile0.Outer.Segments) != 3 || len(lp.Profile1.Outer.Segments) != 3 ||
		len(lp.Verts) > 512 || len(lp.Tris) > 512 ||
		len(source.prismCaps[0]) != 4 || len(source.prismCaps[1]) != 4 ||
		len(source.loftCaps[0]) != 3 || len(source.loftCaps[1]) != 3 ||
		source.levels[0] == nil || source.levels[1] == nil || source.levels[2] == nil {
		return no, false, nil
	}
	checked := &loftVolumeArithmetic{ok: true}
	if checked.cmp(source.levels[0], big.NewRat(0, 1)) != 0 ||
		checked.cmp(source.levels[0], source.levels[1]) >= 0 ||
		checked.cmp(source.levels[1], source.levels[2]) >= 0 || !checked.ok {
		return no, false, nil
	}
	rect, edges, ok, err := loftQuarterWallRectangle(ctx, budget, source, pp)
	if err != nil || !ok {
		return no, false, err
	}
	proof := loftQuarterWallOrderProof{prism: source.prism, loft: source.loft,
		lower: edges[0], upper: edges[1], arcSegment: -1}
	for cap, loop := range [2]sectionrecord.LoopRecord{lp.Profile0.Outer, lp.Profile1.Outer} {
		arcIndex := -1
		axisLines := [2]bool{}
		for i, raw := range loop.Segments {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			span := source.loftCaps[cap][i]
			if span.body != source.loft || span.segment != i {
				return no, false, nil
			}
			switch segment := raw.(type) {
			case sectionrecord.ArcSeg:
				borrowed, same := span.curve.(sectionrecord.ArcSeg)
				if !same || borrowed != segment || arcIndex >= 0 ||
					segment.TStart != 0 || segment.TEnd != 1 ||
					segment.Center.U != 0 || segment.Center.V != 0 ||
					proofbound.IsNonFinite(segment.Start.U) ||
					segment.Start.U <= 0 || segment.Start.V != 0 ||
					segment.End.U != 0 || segment.End.V != segment.Start.U ||
					len(span.wallFaces) == 0 {
					return no, false, nil
				}
				arcIndex = i
				proof.radius[cap] = proofarith.FloatRat(segment.Start.U)
				proof.wallFaces[cap] = span.wallFaces
			case sectionrecord.LineSeg:
				borrowed, same := span.curve.(sectionrecord.LineSeg)
				if !same || borrowed != segment ||
					!loftcontact.QuarterWallAxisLine(segment, proofarith.FloatRat(loopArcRadius(loop))) {
					return no, false, nil
				}
				if segment.Start.V == 0 && segment.End.V == 0 {
					if axisLines[0] {
						return no, false, nil
					}
					axisLines[0] = true
				} else {
					if axisLines[1] {
						return no, false, nil
					}
					axisLines[1] = true
				}
			default:
				return no, false, nil
			}
		}
		if arcIndex < 0 || !axisLines[0] || !axisLines[1] ||
			cap == 1 && arcIndex != proof.arcSegment {
			return no, false, nil
		}
		if cap == 1 {
			if len(lp.Segment) != 1 || len(lp.Segment[0]) > 512 ||
				len(proof.wallFaces[0]) != len(proof.wallFaces[1]) {
				return no, false, nil
			}
			matched := 0
			for cell, segment := range lp.Segment[0] {
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				if segment != arcIndex {
					continue
				}
				if matched >= len(proof.wallFaces[0]) {
					return no, false, nil
				}
				for half := range 2 {
					if err := budget.Step(); err != nil {
						return no, false, err
					}
					expected := provenance.Ref{
						Producer: lp.SourceIdentity.Origin,
						Role:     fmt.Sprintf("side(0,%d,%d)", cell, half),
					}
					if proof.wallFaces[half][matched] != expected {
						return no, false, nil
					}
				}
				matched++
			}
			if matched == 0 || matched != len(proof.wallFaces[0]) {
				return no, false, nil
			}
		}
		proof.arcSegment = arcIndex
	}
	// The authenticated Loft pairs corresponding source parameters by a
	// straight rule. Matching quarter arcs therefore have radius bounded by
	// the two endpoint radii at every intervening height.
	if _, err := budget.StepN(12); err != nil {
		return no, false, err
	}
	rMin, rMax := proof.radius[0], proof.radius[1]
	if checked.cmp(rMin, rMax) > 0 {
		rMin, rMax = rMax, rMin
	}
	minSquare := checked.mul(rMin, rMin)
	maxSquare := checked.mul(rMax, rMax)
	xMinSquare := checked.mul(rect[0], rect[0])
	xMaxSquare := checked.mul(rect[1], rect[1])
	yMinSquare := checked.mul(rect[2], rect[2])
	yMaxSquare := checked.mul(rect[3], rect[3])
	leftUpper := checked.add(xMinSquare, yMaxSquare)
	rightLower := checked.add(xMaxSquare, yMinSquare)
	if !checked.ok || checked.cmp(minSquare, leftUpper) <= 0 ||
		checked.cmp(maxSquare, rightLower) >= 0 || !checked.ok {
		return no, false, nil
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	return proof, true, nil
}

func loftQuarterWallRectangle(ctx context.Context, budget *proofbound.WorkBudget,
	source loftSourceSpanCaps, pp prismPayload) ([4]*big.Rat, [2]int, bool, error) {
	var bounds [4]*big.Rat
	var horizontal [2]int
	if len(pp.Profile.Outer.Segments) != 4 {
		return bounds, horizontal, false, nil
	}
	var lines [4]sectionrecord.LineSeg
	for i, raw := range pp.Profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return bounds, horizontal, false, err
		}
		if err := budget.Step(); err != nil {
			return bounds, horizontal, false, err
		}
		line, ok := raw.(sectionrecord.LineSeg)
		if !ok {
			return bounds, horizontal, false, nil
		}
		for cap := range 2 {
			span := source.prismCaps[cap][i]
			borrowed, same := span.curve.(sectionrecord.LineSeg)
			if !same || borrowed != line || span.body != source.prism || span.segment != i {
				return bounds, horizontal, false, nil
			}
		}
		lines[i] = line
	}
	bounds, horizontal, ok, err := loftcontact.QuarterWallRectangle(budget, lines[:])
	if err != nil || !ok {
		return bounds, horizontal, ok, err
	}
	checked := &loftVolumeArithmetic{ok: true}
	for _, value := range bounds {
		checked.valid(value)
	}
	return bounds, horizontal, checked.ok, nil
}

func loopArcRadius(loop sectionrecord.LoopRecord) float64 {
	for _, raw := range loop.Segments {
		if arc, ok := raw.(sectionrecord.ArcSeg); ok {
			return arc.Start.U
		}
	}
	return 0
}
