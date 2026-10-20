package loftmesh

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// referenceComputeLoftChordedAllow is the pre-sharing implementation from
// c67090490ac21822a1eecbbdcbecf6d5130a0994. Keep its three separate
// bound calls and twist-area call independent of CellAllowsOf.
func referenceComputeLoftChordedAllow(
	pairs []LoopPair, vIdx, wIdx [][]int, verts []r3.Vec, anchor r3.Vec,
	matchedDelta, delta float64, reversed bool,
) LoftChordedAllow {
	var wallAreaUpper, wallLeg, twistVolumeUpper, maxTwistOffsetUpper, seamPerimeterUpper float64
	var capTube, areaExcess, bilinearAreaBound, twistAreaAllow float64
	piUp := math.Nextafter(math.Pi, math.Inf(1))
	twistVolumeCorrection := new(big.Rat)
	var twistMomentCorrection proofbound.RatV3
	for axis := range twistMomentCorrection {
		twistMomentCorrection[axis] = new(big.Rat)
	}
	areaCorrection := new(big.Rat)

	for i, p := range pairs {
		n := len(p.V)
		for j := range n {
			jn := (j + 1) % n
			vLo, vHi := verts[vIdx[i][j]], verts[vIdx[i][jn]]
			wLo, wHi := verts[wIdx[i][j]], verts[wIdx[i][jn]]
			// The skirt runs along every HELD seam cell (this function's
			// doc comment), so this sum precedes the charged-cell gate.
			// ArcUpper bounds the true segment, and a held chord joining two
			// displaced stations can be up to 2·delta longer, so each side
			// takes the larger of that and the held chord's own exact length.
			seamPerimeterUpper = proofbound.AbsSumUpper(seamPerimeterUpper,
				math.Max(p.ArcUpperV[j], proofbound.CellSpanUpper(vLo, vHi)),
				math.Max(p.ArcUpperW[j], proofbound.CellSpanUpper(wLo, wHi)),
			)
			if p.Faceted[j] && p.MatchedDelta[j] <= 0 {
				// A faceted cell's held triangle pair IS its boundary; its
				// vertex displacement is the accumulator's delta-keyed legs
				// (this function's doc comment).
				continue
			}

			// agent-docs/loft-design.md §5.2's matchedDelta row, at THIS cell:
			// the cell's own chord-to-curve half composed with the held
			// vertex displacement delta, which is what makes the bound a
			// claim about the chord the build actually DREW rather than the
			// ideal chord between the two points the record denotes. The
			// per-cell composition is at most the build-wide matchedDelta
			// (that term takes the same sum at the MAX cell), so it is
			// the tighter of the two readings of the same row.
			cellMatched := ChordCellDeltaUpper(p.MatchedDelta[j], delta)

			cellWallUpper := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi, p.ArcUpperV[j], p.ArcUpperW[j], cellMatched)
			wallAreaUpper = proofbound.AbsSumUpper(wallAreaUpper, cellWallUpper)
			wallLeg = proofbound.AbsSumUpper(wallLeg, proofbound.ProductUpper(cellMatched, cellWallUpper))
			joint := proofbound.ProductUpper(piUp, proofbound.ProductUpper(cellMatched, cellMatched))
			twice := proofbound.ProductUpper(2, cellMatched)
			capTube = proofbound.AbsSumUpper(
				capTube,
				proofbound.ProductUpper(twice, p.ArcUpperV[j]), joint,
				proofbound.ProductUpper(twice, p.ArcUpperW[j]), joint,
			)
			twistVolumeUpper = proofbound.AbsSumUpper(twistVolumeUpper, proofbound.CellTwistVolumeAllow(vLo, vHi, wLo, wHi))
			cellTwist := proofbound.CellTwistVolume(vLo, vHi, wLo, wHi)
			cellMoment := proofbound.CellTwistMomentFromVolume(vLo, vHi, wLo, wHi, anchor, cellTwist)
			if reversed {
				cellTwist.Neg(cellTwist)
				for axis := range cellMoment {
					cellMoment[axis].Neg(cellMoment[axis])
				}
			}
			twistVolumeCorrection.Add(twistVolumeCorrection, cellTwist)
			for axis := range cellMoment {
				twistMomentCorrection[axis].Add(twistMomentCorrection[axis], cellMoment[axis])
			}
			maxTwistOffsetUpper = math.Max(maxTwistOffsetUpper, proofbound.CellTwistOffsetUpper(vLo, vHi, wLo, wHi))

			ruledLeg := proofbound.CellChordCurveAreaAllow(
				vLo, vHi, wLo, wHi,
				p.ArcUpperV[j], p.ArcUpperW[j], cellMatched,
				p.TangentEnergyV[j], p.TangentEnergyW[j],
			)
			bilinearValue, bilinearBound := proofbound.CellBilinearArea(vLo, vHi, wLo, wHi)
			bilinearAreaBound = proofbound.AbsSumUpper(bilinearAreaBound, bilinearBound)
			vLoX := proofarith.XptOf(vLo)
			lowerLo, _ := WallTriangleArea(proofarith.Xsub(proofarith.XptOf(vHi), vLoX), proofarith.Xsub(proofarith.XptOf(wHi), vLoX))
			upperLo, _ := WallTriangleArea(proofarith.Xsub(proofarith.XptOf(wHi), vLoX), proofarith.Xsub(proofarith.XptOf(wLo), vLoX))
			areaCorrection.Add(areaCorrection, polynomial.MustRatOf(bilinearValue))
			areaCorrection.Sub(areaCorrection, polynomial.MustRatOf(lowerLo))
			areaCorrection.Sub(areaCorrection, polynomial.MustRatOf(upperLo))
			stationLeg := proofbound.CellStationShiftAreaAllow(
				vLo, vHi, wLo, wHi,
				p.ArcUpperV[j], p.ArcUpperW[j], cellMatched, delta,
			)
			areaExcess = proofbound.AbsSumUpper(areaExcess, ruledLeg, stationLeg)
			twistAreaAllow = proofbound.AbsSumUpper(twistAreaAllow, proofbound.CellTwistAreaAllow(vLo, vHi, wLo, wHi))
		}
	}

	skirtLeg := 0.0
	if delta > 0 {
		skirtLeg = proofbound.ProductUpper(proofbound.ProductUpper(matchedDelta, delta), seamPerimeterUpper)
	}
	areaCorrectionValue, _ := areaCorrection.Float64()

	return LoftChordedAllow{
		WallAreaUpper:         wallAreaUpper,
		WallLeg:               wallLeg,
		SkirtLeg:              skirtLeg,
		TwistVolumeUpper:      twistVolumeUpper,
		TwistVolumeCorrection: twistVolumeCorrection,
		TwistMomentCorrection: twistMomentCorrection,
		MaxTwistOffsetUpper:   maxTwistOffsetUpper,
		AreaCorrection:        areaCorrectionValue,
		AreaCorrectionBound:   proofarith.RationalFloatError(areaCorrection, areaCorrectionValue),
		BilinearAreaBound:     bilinearAreaBound,
		AreaExcess:            areaExcess,
		TwistAreaAllow:        twistAreaAllow,
		CapAreaExcess:         capTube,
	}
}
