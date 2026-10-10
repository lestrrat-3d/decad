package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file is the brep tessellator's reading of a route L body's bands
// (docs/modify-general-design.md Table DG's DG3). Each band is chorded as the
// cap-loop chamfer chords its own loop (tessellate_capblend.go): ONE count per
// wall walk of the receiver's loop, shared by the cap contour on the loop's
// face, the band's patch and the side contour — the rim at the side level of
// the wall beside the loop (docs/tessellation-reach-design.md §7). The count is
// imposed on that wall as its sample points, so the wall's rim at the side
// level and the band's side ring are the same mesh vertices, and
// emitCapBand writes the band's strip and its departure terms.

// tessView is the band read as a one-loop cap blend in F's own frame
// (brepLoopBand.view) beside its contour displacement, which the record
// charges into F's displacement (modify-general §4.2): the larger charge only
// widens what the tessellator's readers add.
func (b brepLoopBand) tessView(f brepFace, xform r3.Transform) capBlendPayload {
	cbp := b.view(f, xform)
	cbp.bandDelta = map[capBandKey]float64{{loop: 0, start: b.matSign(f) > 0}: f.delta}
	return cbp
}

// brepBandChord is one band's chording: the cap-blend loop record chordCapBlendLoop
// resolves over the receiver's loop in F's frame, the topology use of each
// side-contour piece, and the mesh vertices of both rings once placed.
type brepBandChord struct {
	band  brepLoopBand
	face  brepFace
	cbp   capBlendPayload
	lm    capBlendLoopMesh
	start bool
	// sideZ and sideDelta are the side level and its displacement.
	sideZ, sideDelta float64
	// sideUse is walk i's side-contour use (topology.open).
	sideUse []int
	// sideV and capV are the mesh vertices of the side ring at the side level
	// and of the cap contour ring at F's level.
	sideV, capV []int
	// fillet holds a fillet band's interior rings (tessellate_brep_fillet.go),
	// nil for a chamfer band.
	fillet  *filletRings
	partial *partialband.PartialBandLayout
	// curved marks LF8 seams at the start of each walk. Their interior ring
	// positions follow offset-carrier intersections instead of affine ends.
	curved []bool
}

// brepChordBands chords every band of the record and imposes each wall walk's
// count on the face beside it. The result maps a swept face's index to the
// samples its wall must take. A wall two bands reach must take one count from
// both, or the body refuses. A fillet band adds its interior ring count
// (docs/loop-fillet-design.md §7.1).
func brepChordBands(ctx context.Context, bp brepPayload, topo *brepTopology, chord float64) ([]brepBandChord, map[int]tessellation.ChordSamples[*Face], error) {
	if len(bp.loopBands) == 0 {
		return nil, nil, nil
	}
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range topo.open {
		openAt[topo.uses[ui].Key] = ui
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	bands := make([]brepBandChord, len(bp.loopBands))
	imposed := map[int]tessellation.ChordSamples[*Face]{}
	for bi, b := range bp.loopBands {
		if err := b.validate(bp, bi); err != nil {
			return nil, nil, err
		}
		f := bp.faces[b.face]
		cbp := b.tessView(f, bp.xform)
		if b.selected != nil {
			bc, err := chordPartialFilletBand(ctx, b, f, cbp, chord)
			if err != nil {
				return nil, nil, err
			}
			bands[bi] = bc
			continue
		}
		lm, err := chordCapBlendLoop(ctx, budget, cbp, 0, b.orig, chord, work)
		if err != nil {
			return nil, nil, err
		}
		sideZ, sideDelta := b.sideLevel(f)
		bc := brepBandChord{band: b, face: f, cbp: cbp, lm: lm, start: b.matSign(f) > 0, sideZ: sideZ, sideDelta: sideDelta}
		e := topo.embeds[b.face]
		for i, w := range lm.walks {
			key, _ := brepgeom.CurveKey(e, w.SegmentWalk, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, nil, fmt.Errorf(`%w: chamfer band %d's side contour is not a boundary of the record`, ErrUnsupported, bi)
			}
			bc.sideUse = append(bc.sideUse, ui)
			u := topo.uses[ui]
			if !brepIsRim(u.Part) {
				continue
			}
			wallFace := bp.faces[u.Face]
			samples, err := tessellation.ImposeBandWall[*Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: topo.embeds[u.Face],
				WallSegment: wallFace.wall, WallWalk: topo.walls[u.Face],
				WalkIndex: i, SideLevel: sideZ, WallHeight: wallFace.z1 - wallFace.z0,
				Counts: lm.count, SideStarts: lm.sideStart, SidePoints: point2ToRecordSlice(lm.sidePts),
				SideBounds: lm.sideBound, SideSag: lm.sideSag,
			})
			if err != nil {
				return nil, nil, fmt.Errorf("chamfer band %d: %w", bi, err)
			}
			if prior, ok := imposed[u.Face]; ok {
				if len(prior.Samples) != len(samples.Samples) {
					return nil, nil, fmt.Errorf(`%w: two chamfer bands chord the wall of face %d at different counts`, ErrUnsupported, u.Face)
				}
				continue
			}
			imposed[u.Face] = samples
		}
		if b.kind == brepBandFillet {
			rings := tessellation.FilletRingCount(b.setback.axialUpper(), chord)
			fr, err := filletLoopOf(budget, b.orig, b.setback.dc, f.role, work)
			if err != nil {
				return nil, nil, err
			}
			bc.curved = make([]bool, len(fr.walks))
			for k, class := range fr.loop.Corners {
				if class != filletband.CurvedMiter {
					continue
				}
				bc.curved[k] = true
				prev := fr.walks[(k+len(fr.walks)-1)%len(fr.walks)]
				length, ok := filletband.CurvedMiterLength(prev, fr.walks[k], b.setback.axialUpper())
				if !ok {
					return nil, nil, fmt.Errorf(`%w: fillet band %d's curved miter has no length enclosure`, ErrUnsupported, bi)
				}
				needed := math.Ceil(2 * proofbound.RatFloatUp(length.Hi) / chord)
				if proofbound.IsNonFinite(needed) || needed > 4096 {
					return nil, nil, fmt.Errorf(`%w: fillet band %d's curved miter exceeds the mesh ring budget`, ErrUnsupported, bi)
				}
				rings = max(rings, int(needed))
			}
			bc.fillet = &filletRings{n: rings}
		}
		bands[bi] = bc
	}
	return bands, imposed, nil
}

// place adds both rings' vertices: the side ring at the side level and the
// cap contour ring at F's level, each sample keyed by its reference
// coordinates so a wall's rim sample at the same point is the same vertex.
func (bc *brepBandChord) place(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	if bc.partial != nil {
		bc.placePartial(e, addVertex)
		return
	}
	for j, p := range bc.lm.sidePts {
		bc.sideV = append(bc.sideV, addVertex(e.Canon(p.U, p.V, bc.sideZ), bc.lm.sideBound[j]))
	}
	for j, p := range bc.lm.capPts {
		bc.capV = append(bc.capV, addVertex(e.Canon(p.U, p.V, bc.face.z0), bc.lm.capBound[j]))
	}
}

// emit writes the band's patch cells. A band whose walls rise off its face
// (sigma +1) fills a concave corner rather than removing a wedge, and its
// patches face the other way (attachBrepLoopBands), so its windings turn over.
func (bc *brepBandChord) emit(budget *proofbound.WorkBudget, m *Mesh, geom map[string]capPatchGeom,
	faceOfRole func(string) (*Face, error), bump func(*Face, float64)) error {
	if bc.partial != nil {
		first := len(m.triangles)
		if err := bc.emitPartialFillet(m, faceOfRole, bump); err != nil {
			return err
		}
		if bc.band.sigma > 0 {
			for i := first; i < len(m.triangles); i++ {
				m.triangles[i][1], m.triangles[i][2] = m.triangles[i][2], m.triangles[i][1]
			}
		}
		return nil
	}
	if bc.fillet != nil {
		first := len(m.triangles)
		if err := bc.emitFillet(m, faceOfRole, bump); err != nil {
			return err
		}
		if bc.band.sigma > 0 {
			for i := first; i < len(m.triangles); i++ {
				m.triangles[i][1], m.triangles[i][2] = m.triangles[i][2], m.triangles[i][1]
			}
		}
		return nil
	}
	lm := &bc.lm
	if bc.start {
		lm.sideLo, lm.capLoV = bc.sideV, bc.capV
	} else {
		lm.sideHi, lm.capHiV = bc.sideV, bc.capV
	}
	first := len(m.triangles)
	if err := emitCapBand(budget, m, bc.cbp, lm, bc.start, faceOfRole, geom, bump, bc.band.patchRole); err != nil {
		return err
	}
	if bc.band.sigma > 0 {
		for i := first; i < len(m.triangles); i++ {
			m.triangles[i][1], m.triangles[i][2] = m.triangles[i][2], m.triangles[i][1]
		}
	}
	return nil
}

// brepBandMotion is the per-vertex displacement from the ideal polyhedron B1
// (docs/tessellation-reach-design.md §7's motion array): the store for every
// vertex, widened by the side level's displacement on a side ring vertex, and
// replaced on a cap contour vertex by its station's gap from the exact
// fraction of the side window plus its rounding and F's level displacement.
func brepBandMotion(ctx context.Context, bands []brepBandChord, store, round []float64) ([]float64, error) {
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
		if err := capBlendCapMotion(budget, bc.cbp, &bc.lm); err != nil {
			return nil, err
		}
		if bc.fillet != nil {
			bc.fillet.arcMotion(&bc.lm, bc.face.delta, bc.band.setback.dcDelta)
		}
		for j, vi := range bc.capV {
			motion[vi] = proofbound.AbsSumUpper(bc.lm.capMotion[j], round[vi], bc.face.z0Delta)
		}
		if fr := bc.fillet; fr != nil {
			// An interior ring vertex lies within its own interpolation error
			// of the point between its two ends' ideal positions.
			for k := 1; k < fr.n; k++ {
				for c, vi := range fr.ringV[k] {
					ends := math.Max(motion[fr.ringV[0][c]], motion[fr.ringV[fr.n][c]])
					motion[vi] = proofbound.AbsSumUpper(ends, fr.dev[k][c], round[vi])
				}
			}
		}
	}
	return motion, nil
}

// brepBandChordVolume is the circular-segment volume between the bands the
// record denotes and the ideal chord polyhedron: capBlendChordVolume over each
// band's loop with the trimmed wall left to the wall terms of tessellateBrep.
func brepBandChordVolume(bands []brepBandChord) float64 {
	total := 0.0
	for bi := range bands {
		bc := &bands[bi]
		if bc.fillet != nil {
			continue
		}
		loop := bc.lm.proof()
		loop.ZLo, loop.ZHi = proofbound.MeasuredScalar(0, 0), proofbound.MeasuredScalar(0, 0)
		height := [2]float64{}
		if bc.start {
			height[0] = bc.band.setback.axialUpper()
		} else {
			height[1] = bc.band.setback.axialUpper()
		}
		total = proofbound.AbsSumUpper(total, tessellation.CapBlendChordVolume(height, []tessellation.CapBlendLoopProof{loop}))
	}
	return total
}
