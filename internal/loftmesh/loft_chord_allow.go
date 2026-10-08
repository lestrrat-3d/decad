package loftmesh

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ComputeLoftChordedAllow derives LoftChordedAllow's corrections and bounds by walking
// every wall cell of every loop pairs holds, over the SAME cell corner
// convention assembleLoft's own Table B split uses (vLo, vHi = section-0's
// two corners; wLo, wHi = section-1's) — verts/vIdx/wIdx are evalLoft's own
// assembled loftAssembly fields. anchor is the accumulator's own anchor (p0's
// placed plane origin), read by the exact first-moment correction.
//
// matchedDelta is docs/loft-design.md §5.2's own matchedDelta term, already
// COMPOSED by evalLoft as proofbound.AbsSumUpper(sectionDelta, delta) over that table's
// sectionDelta and delta rows — the build-wide PARAMETER-MATCHED displacement.
// The skirt leg reads it. delta is that same table's delta row on its own —
// the held vertex displacement — passed alongside so each cell can compose its
// OWN tighter matched reading through ChordCellDeltaUpper from the cell's own
// chord-to-curve half rather than the build-wide maximum. Neither is ever the
// sagitta alone: reading matchedDelta as sectionDelta leaves the computed
// station's own displacement uncharged on every chorded leg (§5.2's
// matchedDelta paragraph).
//
// A cell is CHARGED unless it is a FACETED cell — a LineSeg pair's cell
// (pairs[i].Faceted[j]) whose own CHORD-TO-CURVE departure is zero
// (pairs[i].MatchedDelta[j] <= 0, the cell's own half of §5.2's matchedDelta
// composition). Only such a cell's held triangle pair IS the boundary §5 gives
// it, so it has nothing to contribute to the chorded legs, and its own vertex
// displacement is the accumulator's sweptVolumeAllow and perturbAreaSum,
// charged once (docs/loft-gear-bounds-design.md §2, step 2).
//
// Every other cell is charged, whatever its departure: a circular or
// free-form cell stands for a bilinear ruled patch through its four held
// corners, and a zero departure says only that its two sides are straight. A
// degree-1 NURBSSeg pair twisted between its sections is exactly that case.
// The twist correction moves Volume onto that patch and only wallLeg pays for
// its motion, so such a cell may never be skipped. The exemption names the
// ONE arm proven faceted rather than the arms that are charged, so an arm
// added later is charged by default, and a positive departure is charged even
// on a cell flagged faceted. A charged cell contributes to wallAreaUpper
// (proofbound.CellChordCurveAreaUpper), wallLeg (that area times the cell's
// own matched departure), the exact volume, first-moment and area
// corrections, the unsigned twistVolumeUpper retained for tessellation's
// occupied-volume proof, maxTwistOffsetUpper (the MAX, never a sum, of
// proofbound.CellTwistOffsetUpper), twistAreaAllow, areaExcess and the cap
// tube.
//
// seamPerimeterUpper is the one sum that reads EVERY cell, charged or not: the
// skirt between the held seam and its projection onto the cap plane runs along
// the whole seam, and a faceted cell's seam moves by delta too (§2). Each
// side adds the larger of its arcLenUpper and its held chord's exact length,
// since the moving seam's speed is a convex combination of the two. The
// skirt leg is productUpper(productUpper(matchedDelta, delta),
// seamPerimeterUpper), and exactly 0 at delta == 0, where the held seam already
// lies in its plane; the delta > 0 gate keeps a +Inf arc-length claim on an
// unplaced build from turning that honest zero into a refusal.
//
// capAreaExcess is §4's per-cell cap tube, summed over the charged cells of
// both caps: 2·cellMatched_k·arcLenUpper_k plus a joint disk π⁺·cellMatched_k²
// per cell, proofbound.SectionDisplacementArea's own rectangle-plus-half-disks
// argument read per chord at that chord's own departure rather than at the
// build-wide maximum. A faceted cell's chord IS the curve it denotes, so the
// symmetric difference between the held cap polygon and the denoted region is
// the union of the charged cells' lenses, which lie inside that tube; the
// held vertices' own displacement is the accumulator's perturbAreaSum. Area()
// charges it beside the wall term; omitting it understates Area on every curved
// pairing (TestLoftArcWedgeAreaMatchesExtrudeOracle).
//
// areaExcess is the two residual legs of the wall's own per-cell gap after
// proofbound.CellBilinearArea has moved the nominal reading from held facets to the
// bilinear patch through the cell's four held corners:
//
//   - the RULED leg, proofbound.CellChordCurveAreaAllow (internal/proofbound/bounds.go): |bilinear − true|,
//     how far that bilinear patch's own area sits from the ruled patch through
//     the same four corners. It reads the cell's own held corners, its two
//     per-side arc-length bounds, its PARAMETER-MATCHED matchedDelta
//     and its two per-side tangent-deviation ENERGIES
//     (p.tangentEnergyV/tangentEnergyW: PerCellTangentEnergy's reading for a
//     line or circular arm, freeform.SpanTangentEnergyUpper's for a free-form
//     one), and that helper's own doc comment carries the derivation.
//   - the STATION-SHIFT leg, proofbound.CellStationShiftAreaAllow (internal/proofbound/bounds.go):
//     |ruled at the held corners − ruled at the denoted stations|, the step the
//     ruled leg stops short of, since it pins its patch at the corners
//     the build HOLDS. It reads the cell's own held corners for the same eB
//     convexity rung the ruled leg forms, its two per-side arc-length bounds,
//     the SAME composed cellMatched those legs take, and the payload's own
//     station displacement delta. It is exactly zero on a build that holds the
//     stations it denotes.
//
// proofbound.CellChordCurveAreaAllow's own composition section owns the split and why the
// ruled reading is not widened by delta to stand in for the station shift.
func ComputeLoftChordedAllow(
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

			// docs/loft-design.md §5.2's matchedDelta row, at THIS cell:
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
