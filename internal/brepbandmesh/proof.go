package brepbandmesh

import (
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepfilletmesh"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Motion is the per-vertex displacement from the ideal polyhedron B1
// (docs/tessellation-reach-design.md §7's motion array): the store for every
// vertex, widened by the side level's displacement on a side ring vertex, and
// replaced on a cap contour vertex by its station's gap from the exact
// fraction of the side window plus its rounding and F's level displacement.
func Motion(ctx context.Context, bands []Band, store, round []float64) ([]float64, error) {
	motion := slices.Clone(store)
	budget := proofbound.NewWorkBudget(ctx)
	for bi := range bands {
		bc := &bands[bi]
		if bc.partial != nil {
			bc.partialMotion(motion, store, round)
			continue
		}
		for _, vi := range bc.sideV {
			motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], bc.sideDelta))
		}
		if err := capblend.CapMotion(budget, bc.cbp, &bc.lm); err != nil {
			return nil, err
		}
		if bc.fillet != nil {
			bc.fillet.ArcMotion(&bc.lm, bc.face.Delta, bc.band.Setback.DcDelta)
		}
		for j, vi := range bc.capV {
			motion[vi] = proofbound.AbsSumUpper(bc.lm.CapMotion[j], round[vi], bc.face.Z0Delta)
		}
		if fr := bc.fillet; fr != nil {
			// An interior ring vertex lies within its own interpolation error
			// of the point between its two ends' ideal positions.
			for k := 1; k < fr.Count; k++ {
				for c, vi := range fr.RingV[k] {
					ends := math.Max(motion[fr.RingV[0][c]], motion[fr.RingV[fr.Count][c]])
					motion[vi] = proofbound.AbsSumUpper(ends, fr.Dev[k][c], round[vi])
				}
			}
		}
	}
	return motion, nil
}

func (bc *Band) partialMotion(motion, store, round []float64) {
	if bc.fillet != nil {
		bc.fillet.PartialMotion(bc.filletInput(), bc.sideDelta, motion, store, round)
		return
	}
	brepfilletmesh.PartialBoundaryMotion(bc.filletInput(), bc.sideDelta, motion, store)
}

// VolumeTerms is the bands' share of the occupied-volume sum. Its first term
// is the circular-segment volume between the chamfer bands the record denotes
// and the ideal chord polyhedron: capblend's ChordVolume over each full
// chamfer band's loop, with the trimmed wall left to the wall terms of the
// brep tessellator. A fillet strip lies within its patch's stated distance of
// the pipe it chords, so each fillet patch then adds its area (areaUpper)
// times that distance (loop-fillet §7.1).
func VolumeTerms(bands []Band, areaUpper map[*topo.Face]float64) []float64 {
	chordVolume := 0.0
	for bi := range bands {
		bc := &bands[bi]
		if bc.fillet != nil {
			continue
		}
		if bc.partial != nil {
			continue // straight planar cells have no chord-versus-surface volume
		}
		loop := bc.lm.Proof()
		loop.ZLo, loop.ZHi = proofbound.MeasuredScalar(0, 0), proofbound.MeasuredScalar(0, 0)
		height := [2]float64{}
		if bc.start {
			height[0] = bc.band.Setback.AxialUpper()
		} else {
			height[1] = bc.band.Setback.AxialUpper()
		}
		chordVolume = proofbound.AbsSumUpper(chordVolume, tessellation.CapBlendChordVolume(height, []tessellation.CapBlendLoopProof{loop}))
	}
	terms := []float64{chordVolume}
	for bi := range bands {
		if bands[bi].fillet == nil {
			continue
		}
		for face, eps := range bands[bi].fillet.PatchEps {
			terms = append(terms, proofbound.ProductUpper(areaUpper[face], eps))
		}
	}
	return terms
}

// OccupiedVolumeAdmission is capblend.OccupiedVolumeAdmission's question for
// every band of a route L body (modify-general Table DG's DG3 and DG4): each
// band is admitted as the cap-loop chamfer admits its own loop, as a whole
// turn, or a loop whose every corner is a line-line miter or an exactly
// tangent join. refusal is the first band's reason, nil when every band is
// admitted; err is a budget or context error.
func OccupiedVolumeAdmission(budget *proofbound.WorkBudget, bp breprecord.Payload) (error, error) {
	for bi, b := range bp.LoopBands {
		if err := b.Validate(bp, bi); err != nil {
			return nil, err
		}
		if b.Kind == breprecord.BandFillet {
			// Every corner class a fillet band holds is one its strips chord
			// slice by slice; SF1 refused the rest at the build, and the LF6
			// fan's stations are the connector arc's azimuths at every ring
			// (loop-fillet DF5).
			continue
		}
		if b.Selected != nil {
			// The selected chamfer has only straight, planar strips, whose
			// shared vertices are the record's exact contour endpoints.
			continue
		}
		refusal, err := capblend.OccupiedVolumeAdmission(budget, b.TessView(bp.Faces[b.Face], bp.Xform))
		if err != nil || refusal != nil {
			return refusal, err
		}
	}
	return nil, nil //nolint:nilnil // no band refuses and no budget ran out
}
