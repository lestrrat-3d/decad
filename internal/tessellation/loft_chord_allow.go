package tessellation

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// LoftChordPair carries one assembled loop's per-cell proof terms.
type LoftChordPair struct {
	Cells                          int
	ArcUpperV, ArcUpperW           []float64
	MatchedDelta                   []float64
	TangentEnergyV, TangentEnergyW []float64
}

// ComputeLoftChordedAllow derives LoftChordedAllow's corrections and bounds by walking
// every wall cell of every loop pairs holds, over the SAME cell corner
// convention assembleLoft's own Table B split uses (vLo, vHi = section-0's
// two corners; wLo, wHi = section-1's) — verts/vIdx/wIdx are evalLoft's own
// assembled loftAssembly fields, read after the whole mass-accumulator add
// loop so coordUpper is complete. anchor is the accumulator's own anchor
// (p0's placed plane origin).
//
// matchedDelta is docs/loft-design.md §5.2's own matchedDelta term, already
// COMPOSED by evalLoft as proofbound.AbsSumUpper(sectionDelta, delta) over that table's
// sectionDelta and delta rows — the build-wide PARAMETER-MATCHED displacement,
// spent on every leg that table keys on it: the cap-area tube
// (proofbound.SectionDisplacementArea, below, whose capAreaAllow row names this term and
// not the sagitta, because a held cap polygon's own vertices are displaced as
// well as chorded) and proofbound.ChordedBoundarySeamAllow's own matchedDelta/posUpper
// obligations. delta is that same table's delta row on its own — the held
// vertex displacement — passed alongside so each cell can compose its OWN
// tighter matched reading through chordCellDeltaUpper (loft_build.go) from the
// cell's own chord-to-curve half rather than the build-wide maximum. Neither is
// ever the sagitta alone: reading matchedDelta as sectionDelta leaves the
// computed station's own displacement uncharged on every chorded leg (§5.2's
// matchedDelta paragraph).
//
// A cell contributes to wallAreaUpper (proofbound.CellChordCurveAreaUpper), the exact
// volume/first-moment corrections, the unsigned twistVolumeUpper retained for
// tessellation's occupied-volume proof, maxTwistOffsetUpper (the MAX,
// never a sum, of proofbound.CellTwistOffsetUpper), twistAreaAllow (the SUM of
// proofbound.CellTwistAreaAllow, retained for the tessellation's own area slack) and
// seamPerimeterUpper's own running
// total ONLY when its own CHORD-TO-CURVE departure is positive
// (pairs[i].MatchedDelta[j] > 0, the cell's own half of §5.2's matchedDelta
// composition) — NEVER keyed on a segment kind (a10-plan.md Part 3 PR 9 Task
// 1a): an exact LineSeg cell's true curve already IS its chord, so that half
// is exactly 0 and the cell has nothing to contribute to any of those four
// legs — its held triangle pair IS the boundary §5 gives it, and its own
// vertex displacement is charged by the accumulator's delta-keyed legs
// instead, never twice. The gate therefore reads the cell's chord-to-curve
// half and never the composed matched value, which a placed build makes
// positive on every cell including the straight ones.
// Skipping such a cell also keeps the bound tighter than passing a zero
// displacement through the same machinery would. Any OTHER cell with a
// positive chord-to-curve half — circular today, a same-kind Tier A free-form cell once that
// arm lands — is charged, regardless of which arm produced it: gating on the
// PROVEN quantity itself, rather than on an enum naming which arm ran, is
// what keeps a future arm from being silently exempted from this whole
// charge the way an earlier version of this evaluator's kind-keyed gate
// would have exempted it.
//
// perimeterUpperV/perimeterUpperW and walksV/walksW sum only the POSITIVE-
// matchedDelta cells of their own cap, never every cell regardless of kind:
// under the fixed-station chord-to-curve homotopy this whole function
// reasons about, a LineSeg boundary never moves at all (its chord IS the
// curve it denotes), so the symmetric difference between the held cap
// polygon and the true denoted region is the union of the curved cells' own
// lenses alone and already lies inside the tube proofbound.SectionDisplacementArea
// takes over their own perimeter — including the straight cells would only
// widen an already-sound bound, never repair an unsound one, so they are
// left out.
//
// h0 (cap0's own offset from anchor) is always exactly zero because anchor
// IS a point on cap0's own plane (evalLoft's own anchor := xform.Apply(
// plane0.Origin)) — proofbound.CapAreaVolumeAllow(0, ...) answers 0 without reading its
// second argument, so capAreaAllow0 is computed but never spent BY THE VOLUME
// LEG. h1 (cap1's own offset) is bounded by the proven exact-rational
// distance from anchor to ANY one held cap1 vertex — valid because a plane's
// own perpendicular offset from a point is never more than the distance to
// any single point ON that plane, and every cap1 vertex lies on cap1's own
// plane exactly. The root adapter refuses an underivable cap1 offset before
// this function runs; a zero would not bound an unknown offset.
//
// capAreaAllow0 and capAreaAllow1 are ALSO area()'s own AREA reading of the
// identical cap gap capVolumeUpper folds into a volume: h0 being zero sinks
// capAreaAllow0 out of capVolumeUpper (an area gap in a plane THROUGH the
// anchor sweeps no volume), but it never sinks the AREA those two allowances
// bound in the first place — a cap's published Area is capPolygonAreaRat, the
// built polygon's own exact rational, and the region the loft's construction
// actually denotes is the curved region proofbound.SectionDisplacementArea(matchedDelta,
// walks, perimeterUpper) bounds the gap to, for EITHER cap, offset or not —
// the MATCHED term, never the sagitta, because that cap polygon's own held
// vertices are displaced as well as chorded (§5.2's capAreaAllow row).
// capAreaExcess is proofbound.AbsSumUpper(capAreaAllow0, capAreaAllow1), unfolded by no
// plane-offset division at all, and area() charges it beside the wall term
// below — omitting it (an earlier version of this function did) understates
// Area on every curved pairing, caught on the shipped A10a wedge fixture
// itself (TestLoftArcWedgeAreaMatchesExtrudeOracle).
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
//     (p.tangentEnergyV/tangentEnergyW, perCellTangentEnergy's own per-arm
//     reading), and that helper's own doc comment carries the derivation. It
//     replaces an arc-minus-chord LENGTH excess times a rung length, a shape
//     third order in the cell's own sweep where the gap it stood for is
//     SECOND order wherever the ruling runs anything but square across the
//     section tangent — an understatement without bound on a twisted
//     pairing.
//   - the STATION-SHIFT leg, proofbound.CellStationShiftAreaAllow (internal/proofbound/bounds.go):
//     |ruled at the held corners − ruled at the denoted stations|, the step the
//     ruled leg stops short of, since it pins its patch at the corners
//     the build HOLDS. It reads the cell's own held corners for the same eB
//     convexity rung the ruled leg forms, its two per-side arc-length bounds,
//     the SAME composed cellMatched those legs take, and the payload's own
//     station displacement delta; that helper's own doc comment carries the
//     derivation, one dimension up from the |e x v| + |u x f| + |e x f|
//     expansion proofbound.PerturbedTriangleAreaAllow states for a single triangle. It is
//     exactly zero on a build that holds the stations it denotes.
//
// proofbound.CellChordCurveAreaAllow's own composition section owns the split and why the
// ruled reading is not widened by delta to stand in for the station shift.
//
// This is the wall term; capAreaExcess above is Area's own cap term, and the
// two together are what area() charges beside the mixed wall reading.
func ComputeLoftChordedAllow(
	pairs []LoftChordPair, vIdx, wIdx [][]int, verts []r3.Vec, anchor r3.Vec,
	h1Upper, matchedDelta, delta, distUpper float64, reversed bool,
) LoftChordedAllow {
	var wallAreaUpper, twistVolumeUpper, maxTwistOffsetUpper, seamPerimeterUpper float64
	var perimeterUpperV, perimeterUpperW, areaExcess, bilinearAreaBound, twistAreaAllow float64
	var walksV, walksW int
	twistVolumeCorrection := new(big.Rat)
	var twistMomentCorrection proofbound.RatV3
	for axis := range twistMomentCorrection {
		twistMomentCorrection[axis] = new(big.Rat)
	}
	areaCorrection := new(big.Rat)

	for i, p := range pairs {
		n := p.Cells
		for j := range n {
			jn := (j + 1) % n
			vLo, vHi := verts[vIdx[i][j]], verts[vIdx[i][jn]]
			wLo, wHi := verts[wIdx[i][j]], verts[wIdx[i][jn]]
			if p.MatchedDelta[j] <= 0 {
				// An exact LineSeg cell's own recorded chord IS the curve it
				// denotes, so its true departure is exactly zero: excluding
				// it from the cap's own perimeter/walks tally
				// (proofbound.SectionDisplacementArea's own tube-plus-joints argument)
				// is sound, not merely convenient — a zero-width segment of
				// the tube contributes nothing to widen, and a joint whose
				// own incident boundary never moves contributes no disk
				// either. Gated on the proven matchedDelta itself, never on a
				// segment kind (this function's own doc comment).
				continue
			}
			walksV++
			walksW++
			perimeterUpperV = proofbound.AbsSumUpper(perimeterUpperV, p.ArcUpperV[j])
			perimeterUpperW = proofbound.AbsSumUpper(perimeterUpperW, p.ArcUpperW[j])

			// docs/loft-design.md §5.2's matchedDelta row, at THIS cell:
			// the cell's own chord-to-curve half composed with the held
			// vertex displacement delta, which is what makes the bound a
			// claim about the chord the build actually DREW rather than the
			// ideal chord between the two points the record denotes. The
			// per-cell composition is at most the build-wide matchedDelta
			// above (that term takes the same sum at the MAX cell), so it is
			// the tighter of the two readings of the same row.
			cellMatched := ChordCellDeltaUpper(p.MatchedDelta[j], delta)

			cellWallUpper := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi, p.ArcUpperV[j], p.ArcUpperW[j], cellMatched)
			wallAreaUpper = proofbound.AbsSumUpper(wallAreaUpper, cellWallUpper)
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
			seamPerimeterUpper = proofbound.AbsSumUpper(seamPerimeterUpper, p.ArcUpperV[j], p.ArcUpperW[j])

			ruledLeg := proofbound.CellChordCurveAreaAllow(
				vLo, vHi, wLo, wHi,
				p.ArcUpperV[j], p.ArcUpperW[j], cellMatched,
				p.TangentEnergyV[j], p.TangentEnergyW[j],
			)
			bilinearValue, bilinearBound := proofbound.CellBilinearArea(vLo, vHi, wLo, wHi)
			bilinearAreaBound = proofbound.AbsSumUpper(bilinearAreaBound, bilinearBound)
			vLoX := proofbound.XptOf(vLo)
			lowerLo, _ := WallTriangleArea(proofbound.Xsub(proofbound.XptOf(vHi), vLoX), proofbound.Xsub(proofbound.XptOf(wHi), vLoX))
			upperLo, _ := WallTriangleArea(proofbound.Xsub(proofbound.XptOf(wHi), vLoX), proofbound.Xsub(proofbound.XptOf(wLo), vLoX))
			areaCorrection.Add(areaCorrection, freeform.MustRatOf(bilinearValue))
			areaCorrection.Sub(areaCorrection, freeform.MustRatOf(lowerLo))
			areaCorrection.Sub(areaCorrection, freeform.MustRatOf(upperLo))
			stationLeg := proofbound.CellStationShiftAreaAllow(
				vLo, vHi, wLo, wHi,
				p.ArcUpperV[j], p.ArcUpperW[j], cellMatched, delta,
			)
			areaExcess = proofbound.AbsSumUpper(areaExcess, ruledLeg, stationLeg)
			twistAreaAllow = proofbound.AbsSumUpper(twistAreaAllow, proofbound.CellTwistAreaAllow(vLo, vHi, wLo, wHi))
		}
	}

	capAreaAllow0 := proofbound.SectionDisplacementArea(matchedDelta, walksV, perimeterUpperV)
	capAreaAllow1 := proofbound.SectionDisplacementArea(matchedDelta, walksW, perimeterUpperW)
	// h1Upper is the root adapter's proven cap1 plane-offset bound.
	capVolumeUpper := proofbound.AbsSumUpper(
		proofbound.CapAreaVolumeAllow(0, capAreaAllow0),
		proofbound.CapAreaVolumeAllow(h1Upper, capAreaAllow1),
	)

	// posUpper (proofbound.ChordedBoundarySeamAllow's own obligation): the held
	// material's own max EUCLIDEAN distance from anchor (distUpper, tighter
	// than proofbound.Radius3D(coordUpper) — loft_moments.go's foldCoordUpper doc
	// comment), widened by matchedDelta — proofbound.ChordedBoundarySeamAllow's
	// own doc comment requires "the SAME parameter-matched displacement leg
	// (a)'s own obligation", never the sagitta alone, so this widening and
	// the matchedDelta argument beside it both read §5.2's own COMPOSED
	// matched term, never either half of it alone.
	posUpper := proofbound.AbsSumUpper(distUpper, matchedDelta)
	seamAllow := proofbound.ChordedBoundarySeamAllow(matchedDelta, posUpper, seamPerimeterUpper)
	areaCorrectionValue, _ := areaCorrection.Float64()

	return LoftChordedAllow{
		WallAreaUpper:         wallAreaUpper,
		TwistVolumeUpper:      twistVolumeUpper,
		TwistVolumeCorrection: twistVolumeCorrection,
		TwistMomentCorrection: twistMomentCorrection,
		MaxTwistOffsetUpper:   maxTwistOffsetUpper,
		CapVolumeUpper:        capVolumeUpper,
		SeamAllow:             seamAllow,
		AreaCorrection:        areaCorrectionValue,
		AreaCorrectionBound:   proofarith.RationalFloatError(areaCorrection, areaCorrectionValue),
		BilinearAreaBound:     bilinearAreaBound,
		AreaExcess:            areaExcess,
		TwistAreaAllow:        twistAreaAllow,
		CapAreaExcess:         proofbound.AbsSumUpper(capAreaAllow0, capAreaAllow1),
	}
}
