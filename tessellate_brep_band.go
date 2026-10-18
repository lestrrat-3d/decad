package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
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

// brepBandChord is one band's chording: the cap-blend loop record capblend.ChordLoop
// resolves over the receiver's loop in F's frame, the topology use of each
// side-contour piece, and the mesh vertices of both rings once placed.
type brepBandChord struct {
	band        brepLoopBand
	face        brepFace
	cbp         capBlendPayload
	lm          capblend.LoopMesh
	start       bool
	bossShell   bool
	pocketShell bool
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
func brepChordBands(ctx context.Context, bp brepPayload, btopo *brepTopology, chord float64) ([]brepBandChord, map[int]tessellation.ChordSamples[*Face], error) {
	if len(bp.LoopBands) == 0 && bp.BossShell == nil && bp.PocketShell == nil {
		return nil, nil, nil
	}
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range btopo.Open {
		openAt[btopo.Uses[ui].Key] = ui
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	bands := make([]brepBandChord, len(bp.LoopBands))
	imposed := map[int]tessellation.ChordSamples[*Face]{}
	for bi, b := range bp.LoopBands {
		if err := b.Validate(bp, bi); err != nil {
			return nil, nil, err
		}
		f := bp.Faces[b.Face]
		cbp := b.TessView(f, bp.Xform)
		if b.Selected != nil {
			var bc brepBandChord
			var err error
			if b.Kind == breprecord.BandFillet {
				bc, err = chordPartialFilletBand(ctx, b, f, cbp, chord)
			} else {
				bc, err = chordPartialChamferBand(ctx, b, f, cbp, chord)
			}
			if err != nil {
				return nil, nil, err
			}
			bands[bi] = bc
			continue
		}
		lm, err := capblend.ChordLoop(ctx, budget, cbp, 0, b.Orig, chord, work)
		if err != nil {
			return nil, nil, err
		}
		sideZ, sideDelta := b.SideLevel(f)
		bc := brepBandChord{band: b, face: f, cbp: cbp, lm: lm, start: b.MatSign(f) > 0, sideZ: sideZ, sideDelta: sideDelta}
		e := btopo.Embeds[b.Face]
		for i, w := range lm.Walks {
			key, _ := brepgeom.CurveKey(e, w.SegmentWalk, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, nil, fmt.Errorf(`%w: chamfer band %d's side contour is not a boundary of the record`, ErrUnsupported, bi)
			}
			bc.sideUse = append(bc.sideUse, ui)
			u := btopo.Uses[ui]
			if !brepgeom.IsRim(u.Part) {
				continue
			}
			wallFace := bp.Faces[u.Face]
			samples, err := tessellation.ImposeBandWall[*Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: btopo.Embeds[u.Face],
				WallSegment: wallFace.Wall, WallWalk: btopo.Walls[u.Face],
				WalkIndex: i, SideLevel: sideZ, WallHeight: wallFace.Z1 - wallFace.Z0,
				Counts: lm.Count, SideStarts: lm.SideStart, SidePoints: point2ToRecordSlice(lm.SidePts),
				SideBounds: lm.SideBound, SideSag: lm.SideSag,
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
		if b.Kind == breprecord.BandFillet {
			rings := tessellation.FilletRingCount(b.Setback.AxialUpper(), chord)
			fr, err := breprecord.FilletLoopOf(budget, b.Orig, b.Setback.Dc, f.Role, work)
			if err != nil {
				return nil, nil, err
			}
			bc.curved = make([]bool, len(fr.Walks))
			for k, class := range fr.Loop.Corners {
				if class != filletband.CurvedMiter {
					continue
				}
				bc.curved[k] = true
				prev := fr.Walks[(k+len(fr.Walks)-1)%len(fr.Walks)]
				length, ok := filletband.CurvedMiterLength(prev, fr.Walks[k], b.Setback.AxialUpper())
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
	if bp.BossShell != nil {
		shell := bp.BossShell
		b := brepLoopBand{Face: shell.LedgeFace, Loop: 0, Orig: shell.SideProfile().Outer,
			Setback: capSetback{Dc: shell.T, Ds: shell.T}, Sigma: 1, Kind: breprecord.BandFillet}
		region := shell.CapProfile()
		f := brepFace{Frame: shell.Frame, Region: &region, Z0: shell.InterfaceZ, Z1: shell.InterfaceZ,
			Outward: false, Role: "bossShellBand"}
		cbp := b.TessView(f, bp.Xform)
		lm, err := capblend.ChordLoop(ctx, budget, cbp, 0, b.Orig, chord, work)
		if err != nil {
			return nil, nil, err
		}
		sideZ, sideDelta := b.SideLevel(f)
		bc := brepBandChord{band: b, face: f, cbp: cbp, lm: lm, start: false,
			sideZ: sideZ, sideDelta: sideDelta, bossShell: true,
			fillet: &filletRings{n: tessellation.FilletRingCount(b.Setback.AxialUpper(), chord)}}
		if shell.Round != nil {
			// The upper cavity wall starts on the band's smaller circle.
			// Impose the band's count and stations on that wall so the
			// quarter torus and cylinder have one shared rim mesh.
			e := btopo.Embeds[shell.LedgeFace]
			seg := region.Outer.Segments[0]
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return nil, nil, err
			}
			key, _ := brepgeom.CurveKey(e, w, shell.InterfaceZ)
			ui, ok := openAt[key]
			if !ok || !brepgeom.IsRim(btopo.Uses[ui].Part) {
				return nil, nil, fmt.Errorf(`%w: round boss shell cap ring has no cavity wall`, ErrUnsupported)
			}
			face := btopo.Uses[ui].Face
			samples, err := tessellation.ImposeBandWall[*Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: btopo.Embeds[face],
				WallSegment: bp.Faces[face].Wall, WallWalk: btopo.Walls[face],
				WalkIndex: 0, SideLevel: shell.InterfaceZ,
				WallHeight: bp.Faces[face].Z1 - bp.Faces[face].Z0,
				Counts:     lm.Count, SideStarts: lm.SideStart,
				SidePoints: point2ToRecordSlice(lm.CapPts), SideBounds: lm.CapBound,
				SideSag: lm.CapSag,
			})
			if err != nil {
				return nil, nil, err
			}
			imposed[face] = samples
		}
		bands = append(bands, bc)
	}
	if bp.PocketShell != nil {
		shell := bp.PocketShell
		bandChord := chord
		if shell.Round != nil {
			// The whole-circle torus adds azimuth, meridian, and cell-twist
			// departures to one patch bound. Give each part of that sum room
			// inside the caller's chord tolerance.
			bandChord = chord / 4
		}
		b := brepLoopBand{Face: shell.FloorFace, Loop: 0, Orig: shell.Rounded.Outer,
			Setback: capSetback{Dc: shell.T, Ds: shell.T}, Sigma: 1, Kind: breprecord.BandFillet}
		region := shell.CapProfile()
		f := brepFace{Frame: shell.Frame, Region: &region, Z0: shell.LowerZ, Z1: shell.LowerZ,
			Outward: shell.CapOutward(), Role: "pocketShellBand"}
		cbp := b.TessView(f, bp.Xform)
		lm, err := capblend.ChordLoop(ctx, budget, cbp, 0, b.Orig, bandChord, work)
		if err != nil {
			return nil, nil, err
		}
		sideZ, sideDelta := b.SideLevel(f)
		bc := brepBandChord{band: b, face: f, cbp: cbp, lm: lm, start: b.MatSign(f) > 0,
			sideZ: sideZ, sideDelta: sideDelta, pocketShell: true,
			fillet: &filletRings{n: tessellation.FilletRingCount(b.Setback.AxialUpper(), bandChord)}}
		e := btopo.Embeds[b.Face]
		for i, w := range lm.Walks {
			key, _ := brepgeom.CurveKey(e, w.SegmentWalk, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, nil, fmt.Errorf(`%w: blind-pocket shell side ring is not a boundary of the record`, ErrUnsupported)
			}
			bc.sideUse = append(bc.sideUse, ui)
			u := btopo.Uses[ui]
			if !brepgeom.IsRim(u.Part) {
				continue
			}
			wallFace := bp.Faces[u.Face]
			samples, err := tessellation.ImposeBandWall[*Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: btopo.Embeds[u.Face],
				WallSegment: wallFace.Wall, WallWalk: btopo.Walls[u.Face],
				WalkIndex: i, SideLevel: sideZ, WallHeight: wallFace.Z1 - wallFace.Z0,
				Counts: lm.Count, SideStarts: lm.SideStart, SidePoints: point2ToRecordSlice(lm.SidePts),
				SideBounds: lm.SideBound, SideSag: lm.SideSag,
			})
			if err != nil {
				return nil, nil, fmt.Errorf("blind-pocket shell band: %w", err)
			}
			imposed[u.Face] = samples
		}
		bands = append(bands, bc)
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
	for j, p := range bc.lm.SidePts {
		bc.sideV = append(bc.sideV, addVertex(e.Canon(p.U, p.V, bc.sideZ), bc.lm.SideBound[j]))
	}
	for j, p := range bc.lm.CapPts {
		bc.capV = append(bc.capV, addVertex(e.Canon(p.U, p.V, bc.face.Z0), bc.lm.CapBound[j]))
	}
}

// emit writes the band's patch cells. A band whose walls rise off its face
// (sigma +1) fills a concave corner rather than removing a wedge, and its
// patches face the other way (breprecord's attachLoopBands), so its windings turn over.
func (bc *brepBandChord) emit(budget *proofbound.WorkBudget, m *Mesh, geom map[string]capPatchGeom,
	faceOfRole func(string) (*Face, error), bump func(*Face, float64)) error {
	if bc.partial != nil {
		first := len(m.n.Triangles)
		var err error
		if bc.band.Kind == breprecord.BandFillet {
			err = bc.emitPartialFillet(m, faceOfRole, bump)
		} else {
			err = bc.emitPartialChamfer(m, faceOfRole, bump)
		}
		if err != nil {
			return err
		}
		if bc.band.Sigma > 0 {
			for i := first; i < len(m.n.Triangles); i++ {
				m.n.Triangles[i][1], m.n.Triangles[i][2] = m.n.Triangles[i][2], m.n.Triangles[i][1]
			}
		}
		return nil
	}
	if bc.fillet != nil {
		first := len(m.n.Triangles)
		if err := bc.emitFillet(m, faceOfRole, bump); err != nil {
			return err
		}
		// The pocket uses an upward virtual cap to generate the band, while
		// its recorded square underside faces down. Its generated winding
		// already opposes both neighbouring faces.
		if bc.band.Sigma > 0 && !bc.pocketShell {
			for i := first; i < len(m.n.Triangles); i++ {
				m.n.Triangles[i][1], m.n.Triangles[i][2] = m.n.Triangles[i][2], m.n.Triangles[i][1]
			}
		}
		return nil
	}
	lm := &bc.lm
	if bc.start {
		lm.SideLo, lm.CapLoV = bc.sideV, bc.capV
	} else {
		lm.SideHi, lm.CapHiV = bc.sideV, bc.capV
	}
	first := len(m.n.Triangles)
	if err := emitCapBand(budget, m, bc.cbp, lm, bc.start, faceOfRole, geom, bump, bc.band.PatchRole); err != nil {
		return err
	}
	if bc.band.Sigma > 0 {
		for i := first; i < len(m.n.Triangles); i++ {
			m.n.Triangles[i][1], m.n.Triangles[i][2] = m.n.Triangles[i][2], m.n.Triangles[i][1]
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
		if err := capblend.CapMotion(budget, bc.cbp, &bc.lm); err != nil {
			return nil, err
		}
		if bc.fillet != nil {
			bc.fillet.arcMotion(&bc.lm, bc.face.Delta, bc.band.Setback.DcDelta)
		}
		for j, vi := range bc.capV {
			motion[vi] = proofbound.AbsSumUpper(bc.lm.CapMotion[j], round[vi], bc.face.Z0Delta)
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
// record denotes and the ideal chord polyhedron: capblend.ChordVolume over each
// band's loop with the trimmed wall left to the wall terms of tessellateBrep.
func brepBandChordVolume(bands []brepBandChord) float64 {
	total := 0.0
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
		total = proofbound.AbsSumUpper(total, tessellation.CapBlendChordVolume(height, []tessellation.CapBlendLoopProof{loop}))
	}
	return total
}
