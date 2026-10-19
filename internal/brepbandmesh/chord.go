// Package brepbandmesh is the brep tessellator's reading of a route L body's
// bands (docs/modify-general-design.md Table DG's DG3). Each band is chorded as
// the cap-loop chamfer chords its own loop (internal/capblend): ONE count per
// wall walk of the receiver's loop, shared by the cap contour on the loop's
// face, the band's patch and the side contour — the rim at the side level of
// the wall beside the loop (docs/tessellation-reach-design.md §7). The count is
// imposed on that wall as its sample points, so the wall's rim at the side
// level and the band's side ring are the same mesh vertices, and
// capblend.EmitBand writes the band's strip and its departure terms.
package brepbandmesh

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepfilletmesh"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Band is one band's chording: the cap-blend loop record capblend.ChordLoop
// resolves over the receiver's loop in F's frame, the topology use of each
// side-contour piece, and the mesh vertices of both rings once placed.
type Band struct {
	band        breprecord.LoopBand
	face        breprecord.Face
	cbp         capblend.Payload
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
	// fillet holds a fillet band's interior rings (internal/brepfilletmesh/),
	// nil for a chamfer band.
	fillet  *brepfilletmesh.Rings
	partial *partialband.PartialBandLayout
	// curved marks LF8 seams at the start of each walk. Their interior ring
	// positions follow offset-carrier intersections instead of affine ends.
	curved []bool
}

func (bc *Band) filletInput() brepfilletmesh.Input {
	return brepfilletmesh.Input{
		Band: bc.band, Face: bc.face, Payload: bc.cbp, Loop: &bc.lm,
		Start: bc.start, SideZ: bc.sideZ, SideV: bc.sideV, CapV: bc.capV,
		Curved: bc.curved,
	}
}

// Chord chords every band of the record and imposes each wall walk's count
// on the face beside it. The result maps a swept face's index to the samples
// its wall must take. A wall two bands reach must take one count from both,
// or the body refuses. A fillet band adds its interior ring count
// (docs/loop-fillet-design.md §7.1). The boss and pocket shells, when the
// record holds them, append one band each after the record's own bands.
func Chord(ctx context.Context, bp breprecord.Payload, btopo *breprecord.Topology, chord float64) ([]Band, map[int]tessellation.ChordSamples[*topo.Face], error) {
	if len(bp.LoopBands) == 0 && bp.BossShell == nil && bp.PocketShell == nil {
		return nil, nil, nil
	}
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range btopo.Open {
		openAt[btopo.Uses[ui].Key] = ui
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	bands := make([]Band, len(bp.LoopBands))
	imposed := map[int]tessellation.ChordSamples[*topo.Face]{}
	for bi, b := range bp.LoopBands {
		if err := b.Validate(bp, bi); err != nil {
			return nil, nil, err
		}
		f := bp.Faces[b.Face]
		cbp := b.TessView(f, bp.Xform)
		if b.Selected != nil {
			var bc Band
			var err error
			if b.Kind == breprecord.BandFillet {
				bc, err = chordPartialFillet(ctx, b, f, cbp, chord)
			} else {
				bc, err = chordPartialChamfer(ctx, b, f, cbp, chord)
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
		bc := Band{band: b, face: f, cbp: cbp, lm: lm, start: b.MatSign(f) > 0, sideZ: sideZ, sideDelta: sideDelta}
		e := btopo.Embeds[b.Face]
		for i, w := range lm.Walks {
			key, _ := brepgeom.CurveKey(e, w.SegmentWalk, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, nil, fmt.Errorf(`%w: chamfer band %d's side contour is not a boundary of the record`, decaderr.ErrUnsupported, bi)
			}
			bc.sideUse = append(bc.sideUse, ui)
			u := btopo.Uses[ui]
			if !brepgeom.IsRim(u.Part) {
				continue
			}
			wallFace := bp.Faces[u.Face]
			samples, err := tessellation.ImposeBandWall[*topo.Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: btopo.Embeds[u.Face],
				WallSegment: wallFace.Wall, WallWalk: btopo.Walls[u.Face],
				WalkIndex: i, SideLevel: sideZ, WallHeight: wallFace.Z1 - wallFace.Z0,
				Counts: lm.Count, SideStarts: lm.SideStart, SidePoints: slices.Clone(lm.SidePts),
				SideBounds: lm.SideBound, SideSag: lm.SideSag,
			})
			if err != nil {
				return nil, nil, fmt.Errorf("chamfer band %d: %w", bi, err)
			}
			if prior, ok := imposed[u.Face]; ok {
				if len(prior.Samples) != len(samples.Samples) {
					return nil, nil, fmt.Errorf(`%w: two chamfer bands chord the wall of face %d at different counts`, decaderr.ErrUnsupported, u.Face)
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
					return nil, nil, fmt.Errorf(`%w: fillet band %d's curved miter has no length enclosure`, decaderr.ErrUnsupported, bi)
				}
				needed := math.Ceil(2 * proofbound.RatFloatUp(length.Hi) / chord)
				if proofbound.IsNonFinite(needed) || needed > 4096 {
					return nil, nil, fmt.Errorf(`%w: fillet band %d's curved miter exceeds the mesh ring budget`, decaderr.ErrUnsupported, bi)
				}
				rings = max(rings, int(needed))
			}
			bc.fillet = &brepfilletmesh.Rings{Count: rings}
		}
		bands[bi] = bc
	}
	if bp.BossShell != nil {
		shell := bp.BossShell
		b := breprecord.LoopBand{Face: shell.LedgeFace, Loop: 0, Orig: shell.SideProfile().Outer,
			Setback: capblend.Setback{Dc: shell.T, Ds: shell.T}, Sigma: 1, Kind: breprecord.BandFillet}
		region := shell.CapProfile()
		f := breprecord.Face{Frame: shell.Frame, Region: &region, Z0: shell.InterfaceZ, Z1: shell.InterfaceZ,
			Outward: false, Role: "bossShellBand"}
		cbp := b.TessView(f, bp.Xform)
		lm, err := capblend.ChordLoop(ctx, budget, cbp, 0, b.Orig, chord, work)
		if err != nil {
			return nil, nil, err
		}
		sideZ, sideDelta := b.SideLevel(f)
		bc := Band{band: b, face: f, cbp: cbp, lm: lm, start: false,
			sideZ: sideZ, sideDelta: sideDelta, bossShell: true,
			fillet: &brepfilletmesh.Rings{Count: tessellation.FilletRingCount(b.Setback.AxialUpper(), chord)}}
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
				return nil, nil, fmt.Errorf(`%w: round boss shell cap ring has no cavity wall`, decaderr.ErrUnsupported)
			}
			face := btopo.Uses[ui].Face
			samples, err := tessellation.ImposeBandWall[*topo.Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: btopo.Embeds[face],
				WallSegment: bp.Faces[face].Wall, WallWalk: btopo.Walls[face],
				WalkIndex: 0, SideLevel: shell.InterfaceZ,
				WallHeight: bp.Faces[face].Z1 - bp.Faces[face].Z0,
				Counts:     lm.Count, SideStarts: lm.SideStart,
				SidePoints: slices.Clone(lm.CapPts), SideBounds: lm.CapBound,
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
		b := breprecord.LoopBand{Face: shell.FloorFace, Loop: 0, Orig: shell.Rounded.Outer,
			Setback: capblend.Setback{Dc: shell.T, Ds: shell.T}, Sigma: 1, Kind: breprecord.BandFillet}
		region := shell.CapProfile()
		f := breprecord.Face{Frame: shell.Frame, Region: &region, Z0: shell.LowerZ, Z1: shell.LowerZ,
			Outward: shell.CapOutward(), Role: "pocketShellBand"}
		cbp := b.TessView(f, bp.Xform)
		lm, err := capblend.ChordLoop(ctx, budget, cbp, 0, b.Orig, bandChord, work)
		if err != nil {
			return nil, nil, err
		}
		sideZ, sideDelta := b.SideLevel(f)
		bc := Band{band: b, face: f, cbp: cbp, lm: lm, start: b.MatSign(f) > 0,
			sideZ: sideZ, sideDelta: sideDelta, pocketShell: true,
			fillet: &brepfilletmesh.Rings{Count: tessellation.FilletRingCount(b.Setback.AxialUpper(), bandChord)}}
		e := btopo.Embeds[b.Face]
		for i, w := range lm.Walks {
			key, _ := brepgeom.CurveKey(e, w.SegmentWalk, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, nil, fmt.Errorf(`%w: blind-pocket shell side ring is not a boundary of the record`, decaderr.ErrUnsupported)
			}
			bc.sideUse = append(bc.sideUse, ui)
			u := btopo.Uses[ui]
			if !brepgeom.IsRim(u.Part) {
				continue
			}
			wallFace := bp.Faces[u.Face]
			samples, err := tessellation.ImposeBandWall[*topo.Face](tessellation.BandWallInput{
				FaceEmbed: e, WallEmbed: btopo.Embeds[u.Face],
				WallSegment: wallFace.Wall, WallWalk: btopo.Walls[u.Face],
				WalkIndex: i, SideLevel: sideZ, WallHeight: wallFace.Z1 - wallFace.Z0,
				Counts: lm.Count, SideStarts: lm.SideStart, SidePoints: slices.Clone(lm.SidePts),
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

// chordPartialChamfer uses one side/cap column at each end of the two
// selected straight walks. The recorded patch planes contain every cell.
func chordPartialChamfer(ctx context.Context, b breprecord.LoopBand, f breprecord.Face,
	cbp capblend.Payload, chord float64) (Band, error) {
	work := freeform.NewFreeformWork()
	side, err := capblend.OneLoopCornerLoop(proofbound.NewWorkBudget(ctx), b.Orig, work)
	if err != nil {
		return Band{}, err
	}
	partial, err := partialband.PartialFilletLayout(side.Walks, b.Orig.Segments,
		f.RegionLoop(b.Loop).Segments, b.Selected, b.CapWalk, b.CapArc,
		b.Setback.Dc, chord, work)
	if err != nil {
		return Band{}, err
	}
	for _, walk := range partial.Walks {
		if walk.Sag != 0 || len(walk.Cols) != 2 {
			return Band{}, fmt.Errorf(`%w: a selected chamfer walk is not straight`, decaderr.ErrUnsupported)
		}
	}
	sideZ, sideDelta := b.SideLevel(f)
	return Band{band: b, face: f, cbp: cbp, partial: partial,
		start: b.MatSign(f) > 0, sideZ: sideZ, sideDelta: sideDelta}, nil
}

// chordPartialFillet adapts the recorded band to its internal sample layout.
func chordPartialFillet(ctx context.Context, b breprecord.LoopBand, f breprecord.Face,
	cbp capblend.Payload, chord float64) (Band, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	side, err := capblend.OneLoopCornerLoop(budget, b.Orig, work)
	if err != nil {
		return Band{}, err
	}
	contour := f.RegionLoop(b.Loop)
	partial, err := partialband.PartialFilletLayout(side.Walks, b.Orig.Segments,
		contour.Segments, b.Selected, b.CapWalk, b.CapArc, b.Setback.Dc, chord, work)
	if err != nil {
		return Band{}, err
	}
	sideZ, sideDelta := b.SideLevel(f)
	return Band{band: b, face: f, cbp: cbp, start: b.MatSign(f) > 0,
		sideZ: sideZ, sideDelta: sideDelta,
		fillet: &brepfilletmesh.Rings{Count: tessellation.FilletRingCount(b.Setback.AxialUpper(), chord)}, partial: partial}, nil
}
