package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is brepPayload — the analytically trimmed face record of
// docs/general-boolean-design.md §4 — together with the face view that states a
// prism or a stacked prism in it, the record audit, and the body build:
// topology and roles (§4.2). brep_measure.go owns the measurements (§4.3) and
// tessellate_brep.go the mesh and its occupied-volume proof (§4.4).

// brepFace is one face of a brepPayload. Exactly one of region and wall is
// set.
//
// A planar face (region set) lies in frame's plane at level z0 along
// frame.N(); z1 equals z0 and z1Delta equals z0Delta. Its region is stated in
// frame coordinates with the material on the left of every loop's walk (outer
// counter-clockwise, holes clockwise, as a section records them), and outward
// is true when its outward normal is +frame.N(). sweep is nonzero exactly
// when the face restates a straight wall as a plane: the direction, along a
// reference axis, that wall sweeps along. Two walls of one sweep meet at a
// junction, which reads the walk's turn (§4.2); a cap's lines read its loop
// role. A swept face leaves it zero: its own frame normal is its sweep.
//
// A swept face (wall set) is the wall segment, stated in frame coordinates and
// walked with the material on its left, swept along frame.N() over [z0, z1].
//
// side0 and side1 are a swept face's side-line splits (§4.2): the levels
// strictly inside (z0, z1), ascending, at which the wall's start and end
// lines are vertices of the body, so each side line is one edge per piece
// between consecutive levels.
//
// delta is the face's own section displacement and z0Delta/z1Delta its levels'
// displacements (docs/prism-boolean-design.md §7); role is the face's role,
// face(k) or wall(k) for its index k in the record.
//
// blend is "fillet" or "chamfer" on a swept face a modify op's route E built
// (docs/brep-modify-design.md §5.3, Table BB's BB4), and empty on every other
// face. The body build gives such a face a second role, fillet(k) or
// chamfer(k) for its index k, so a placement re-mints it with the record.
type brepFace struct {
	frame            r3.Frame
	region           *profileRecord
	outward          bool
	sweep            r3.Vec
	wall             curveSegment
	z0, z1           float64
	z0Delta, z1Delta float64
	side0, side1     []brepSplit
	delta            float64
	role             string
	blend            string
}

// brepStack is the slabs an A1 result was built from
// (docs/general-boolean-design.md §3 "A1 as a brep", §4.1): each slab's
// hole-free regions in the reference frame, and the section displacement
// every region carries. A further co-directional Union reads it as its
// operand's slabs (stackedUnionOperandOf); no consumer reads it, and a
// placement leaves it unchanged, since it moves xform alone.
type brepStack struct {
	slabs []stackedrecord.Slab
	delta float64
}

// brepSplit is one side-line split: a level and its displacement.
type brepSplit = brepgeom.Split

// brepPayload is the evaluator's record of an analytically trimmed body. Every
// face frame is stated in the payload's unplaced coordinates and xform places
// the whole body, as every payload does. stack is nil for every record but
// an A1 result's. loopBands holds a route L chamfer's or fillet's bands
// (docs/modify-general-design.md §4.2 step 5, docs/loop-fillet-design.md §4):
// each band's patches are faces of the body and no face of the record, and
// the body build attaches them (brep_loop_band.go, brep_loop_fillet.go). It
// is nil for every record route L did not build.
type brepPayload struct {
	faces     []brepFace
	xform     r3.Transform
	stack     *brepStack
	loopBands []brepLoopBand
	// loopPatches is each band's patch geometry, beside its role
	// chamferLoop(f,l,p), in band order, nil for a fillet band, whose
	// patches the readers take from the band record and the body's faces
	// (tessellate_brep_fillet.go, brepFilletUndercuts). The body build fills it
	// (attachBrepLoopBands); a record handed to the build carries none, and
	// the tessellator and the surveys read it from the payload the build
	// left on the body, as capBlendPayload.patches is read.
	loopPatches [][]capPatch
}

func (f brepFace) planar() bool { return f.region != nil }

// reversed is the face with its outward side exchanged, the rule a record
// applies to a face of a cavity (docs/modify-general-design.md §3.3 step 5):
// a planar face flips outward, and a swept face walks its wall the other way
// and exchanges its start-line and end-line splits. Levels, frame and
// displacements are unchanged.
func (f brepFace) reversed() brepFace {
	if f.planar() {
		f.outward = !f.outward
		return f
	}
	f.wall = reverseSegment(f.wall)
	f.side0, f.side1 = f.side1, f.side0
	return f
}

// view is the prism the face's own frame, levels and displacements describe.
// A planar face's view carries its whole region at a zero height; a swept
// face's carries its one wall segment as an open single-segment loop, which
// only the per-segment readings (extents, witnesses) consume.
func (f brepFace) view(xform r3.Transform) prismPayload {
	pp := prismPayload{
		frame: f.frame, xform: xform,
		z0: f.z0, z1: f.z1, z0Delta: f.z0Delta, z1Delta: f.z1Delta,
		sectionDelta: f.delta,
	}
	if f.planar() {
		pp.profile = *f.region
	} else {
		pp.profile = profileRecord{Outer: loopRecord{Segments: []curveSegment{f.wall}}}
	}
	return pp
}

func (bp brepPayload) transform() r3.Transform { return bp.xform }

// placed re-evaluates the record under the composed motion. A reflection is
// read at build time, as prismPayload.reflected() is, so the record itself is
// unchanged.
func (bp brepPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	bp.xform = composed
	return evalBrepContext(ctx, d, ref, bp)
}

// axialDelta is the largest level displacement over every face.
func (bp brepPayload) axialDelta() float64 {
	var out float64
	for _, f := range bp.faces {
		out = max(out, f.z0Delta, f.z1Delta)
	}
	return out
}

// sectionDelta is the largest section displacement over every face.
func (bp brepPayload) sectionDelta() float64 {
	var out float64
	for _, f := range bp.faces {
		out = max(out, f.delta)
	}
	return out
}

// assignRoles names every face by its index in the record: face(k) for a
// planar face and wall(k) for a swept one (§4.2).
func (bp brepPayload) assignRoles() {
	for k := range bp.faces {
		if bp.faces[k].planar() {
			bp.faces[k].role = fmt.Sprintf("face(%d)", k)
			continue
		}
		bp.faces[k].role = fmt.Sprintf("wall(%d)", k)
	}
}

// brepOfPrism is a prism's face view (§4.1): one swept face per recorded
// segment, then the start and end caps. It is built on demand and never stored
// for a prism. A surface result has no closed face view and is ErrUnsupported.
func brepOfPrism(pp prismPayload) (brepPayload, error) {
	bp, _, err := brepOfPrismWithRoles(pp)
	return bp, err
}

// brepOfPrismWithRoles also maps record faces to the source prism's public
// roles, so an asymmetric chamfer can resolve its reference face.
func brepOfPrismWithRoles(pp prismPayload) (brepPayload, []string, error) {
	if pp.surfaceResult {
		return brepPayload{}, nil, fmt.Errorf(`%w: a surface-result prism has no closed face view`, ErrUnsupported)
	}
	profile, allow, err := brepJoinProfile(pp.profile)
	if err != nil {
		return brepPayload{}, nil, err
	}
	pp.profile = profile
	if allow > 0 {
		pp.sectionDelta = proofbound.AbsSumUpper(pp.sectionDelta, allow)
	}
	bp := brepPayload{xform: pp.xform}
	var sourceRoles []string
	for li, loop := range append([]loopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		for si, seg := range loop.Segments {
			bp.faces = append(bp.faces, brepFace{
				frame: pp.frame, wall: seg, z0: pp.z0, z1: pp.z1,
				z0Delta: pp.z0Delta, z1Delta: pp.z1Delta, delta: pp.sectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("side(%d,%d)", li, si))
		}
	}
	region := pp.profile
	bp.faces = append(bp.faces,
		brepFace{frame: pp.frame, region: &region, z0: pp.z0, z1: pp.z0,
			z0Delta: pp.z0Delta, z1Delta: pp.z0Delta, delta: pp.sectionDelta},
		brepFace{frame: pp.frame, region: &region, outward: true, z0: pp.z1, z1: pp.z1,
			z0Delta: pp.z1Delta, z1Delta: pp.z1Delta, delta: pp.sectionDelta},
	)
	sourceRoles = append(sourceRoles, roleCapStart, roleCapEnd)
	bp.assignRoles()
	return bp, sourceRoles, nil
}

// brepOfStacked is a stacked prism's face view (§4.1): one swept face per
// segment of every loop column (docs/stacked-prism-design.md §2.3), then the
// two caps and every interface's exposed floors and ceilings. The record is
// audited first, so a stack its own build refuses has no face view either. A
// prism group (one slab of several disjoint regions) has no face view and is
// ErrUnsupported: the record states one region per cap. So is a stack
// enclosing a cavity (a closed shell, stackedEnclosesCavity).
func brepOfStacked(ctx context.Context, sp stackedPrismPayload) (brepPayload, error) {
	bp, _, err := brepOfStackedWithRoles(ctx, sp)
	return bp, err
}

// brepOfStackedWithRoles also returns each record face's role on the source
// stacked body. Chamfer uses the roles only while resolving a public face.
func brepOfStackedWithRoles(ctx context.Context, sp stackedPrismPayload) (brepPayload, []string, error) {
	if sp.isGroup() {
		return brepPayload{}, nil, fmt.Errorf(`%w: a prism group of %d disjoint regions has no face view`, ErrUnsupported, len(sp.slabs[0].Regions))
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.slabs, Interfaces: sp.interfaces}); err != nil {
		return brepPayload{}, nil, err
	}
	cavity, err := stackedEnclosesCavity(sp)
	if err != nil {
		return brepPayload{}, nil, err
	}
	if cavity {
		// A brep record's lumps are its connected face sets, so a cavity's
		// faces would read as a second solid.
		return brepPayload{}, nil, fmt.Errorf(`%w: a stacked prism enclosing a cavity has no face view`, ErrUnsupported)
	}
	sp, err = brepJoinStacked(sp)
	if err != nil {
		return brepPayload{}, nil, err
	}
	columns, _, err := stackedColumns(sp)
	if err != nil {
		return brepPayload{}, nil, err
	}
	bp := brepPayload{xform: sp.xform}
	var sourceRoles []string
	for _, col := range columns {
		first, last := sp.slabs[col.Start], sp.slabs[col.End]
		for segment, seg := range col.Loop.Segments {
			bp.faces = append(bp.faces, brepFace{
				frame: sp.frame, wall: seg, z0: first.Z0, z1: last.Z1,
				z0Delta: first.Z0Delta, z1Delta: last.Z1Delta, delta: sp.sectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
				col.Start, col.Region, col.LoopIndex, segment))
		}
	}
	addPlanar := func(region profileRecord, z, zDelta float64, outward bool, sourceRole string) {
		bp.faces = append(bp.faces, brepFace{frame: sp.frame, region: &region, outward: outward,
			z0: z, z1: z, z0Delta: zDelta, z1Delta: zDelta, delta: sp.sectionDelta})
		sourceRoles = append(sourceRoles, sourceRole)
	}
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	addPlanar(first.Regions[0], first.Z0, first.Z0Delta, false, roleCapStart)
	addPlanar(last.Regions[0], last.Z1, last.Z1Delta, true, roleCapEnd)
	for k, boundary := range sp.interfaces {
		z, zDelta := sp.slabs[k].Z1, sp.slabs[k].Z1Delta
		for e, region := range boundary.LowerExposed {
			addPlanar(region, z, zDelta, true, fmt.Sprintf("floor(%d,%d)", k, e))
		}
		for e, region := range boundary.UpperExposed {
			addPlanar(region, z, zDelta, false, fmt.Sprintf("ceiling(%d,%d)", k, e))
		}
	}
	bp.assignRoles()
	return bp, sourceRoles, nil
}

// brepJoinProfile adapts a region to brepgeom.JoinProfile.
func brepJoinProfile(p profileRecord) (profileRecord, float64, error) {
	joined, allow, err := brepgeom.JoinProfile(brepgeom.Profile{Outer: p.Outer, Holes: p.Holes})
	return profileRecord{Outer: joined.Outer, Holes: joined.Holes}, allow, err
}

// brepJoinStacked joins every region and exposed record of a
// stack, charging the largest allow to its section displacement. Equal loops
// join alike, so the columns stackedColumns derives are unchanged.
func brepJoinStacked(sp stackedPrismPayload) (stackedPrismPayload, error) {
	allow := 0.0
	join := func(p profileRecord) (profileRecord, error) {
		out, a, err := brepJoinProfile(p)
		allow = math.Max(allow, a)
		return out, err
	}
	out := sp
	out.slabs = make([]stackedrecord.Slab, len(sp.slabs))
	for k, slab := range sp.slabs {
		out.slabs[k] = slab
		out.slabs[k].Regions = make([]profileRecord, len(slab.Regions))
		for r, region := range slab.Regions {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.slabs[k].Regions[r] = joined
		}
	}
	out.interfaces = make([]stackedrecord.Interface, len(sp.interfaces))
	for k, boundary := range sp.interfaces {
		for _, region := range boundary.LowerExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.interfaces[k].LowerExposed = append(out.interfaces[k].LowerExposed, joined)
		}
		for _, region := range boundary.UpperExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.interfaces[k].UpperExposed = append(out.interfaces[k].UpperExposed, joined)
		}
	}
	if allow > 0 {
		out.sectionDelta = proofbound.AbsSumUpper(sp.sectionDelta, allow)
	}
	return out, nil
}

// brepEmbed maps one face frame's local axes onto the reference frame's: local
// axis i is reference axis axis[i] scaled by sign[i]. Every sign is ±1, so the
// map moves a float coordinate exactly in both directions.
type brepEmbed = brepgeom.Embed

// brepEmbeds states every face frame in the first face's frame, the reference
// every reading is taken in. A face frame must share the reference origin bit
// for bit and carry each reference axis, or its negation, bit for bit as one of
// its own, with the map keeping handedness. Such a map is a signed permutation,
// so every coordinate and level carries over exactly and the record's records
// compare by identity (§4.2). Any other frame is ErrUnsupported: this build has
// no exact map for it.
func brepEmbeds(faces []brepFace) ([]brepEmbed, error) {
	frames := make([]r3.Frame, len(faces))
	for fi, f := range faces {
		frames[fi] = f.frame
	}
	return brepgeom.Embeds(frames, ErrUnsupported)
}

// falsifyBrepPayload refuses a record no brep body matches (ErrDegenerate) or
// one this evaluator does not build (ErrUnsupported): a face with neither or
// both of a region and a wall, a non-finite level or displacement, an empty
// sweep interval, a planar face whose two levels differ, a role that is empty
// or repeated, or a segment other than a line, a circle or an arc (§3 B2). It
// runs before topology and before chording, and repairs nothing.
func falsifyBrepPayload(ctx context.Context, bp brepPayload) error {
	return brepgeom.ValidateFaces(ctx, len(bp.faces), func(i int) brepgeom.FaceRecord {
		f := bp.faces[i]
		var region *brepgeom.Profile
		if f.region != nil {
			region = &brepgeom.Profile{Outer: f.region.Outer, Holes: f.region.Holes}
		}
		return brepgeom.FaceRecord{
			Frame: f.frame, Region: region, Wall: f.wall,
			Z0: f.z0, Z1: f.z1, Z0Delta: f.z0Delta, Z1Delta: f.z1Delta,
			Side0: f.side0, Side1: f.side1,
			Delta: f.delta, Role: f.role,
		}
	})
}

// brepPart names which boundary piece of its face an edge use is.
type brepPart = brepgeom.Part

const (
	brepLoopSeg = brepgeom.LoopSeg // a planar face's loop segment
	brepRim0    = brepgeom.Rim0    // a swept face's wall at z0
	brepRim1    = brepgeom.Rim1    // a swept face's wall at z1
	brepSide0   = brepgeom.Side0   // a swept face's line at the wall's start
	brepSide1   = brepgeom.Side1   // a swept face's line at the wall's end
)

// brepUse is one face's use of one edge in the shared topology builder.
type brepUse = brepgeom.Use

// brepTopology is the shared reading of a record that the body build and the
// tessellator both take: each face's walks, every edge use, and how the uses
// pair into edges.
type brepTopology struct {
	embeds []brepEmbed
	// walls holds a swept face's walk, planar holds each planar face's walks
	// per region loop.
	walls  map[int]survey2d.SegmentWalk
	planar map[int][][]survey2d.SegmentWalk
	uses   []brepUse
	// edges lists the two use indices of every edge, the owner first: a rim use
	// where the edge has one, then a side line, then a loop segment. The
	// owner's natural direction is the edge's direction.
	edges [][2]int
	// edgeOf maps a use index to its edge index, -1 for an open use.
	edgeOf []int
	// open lists the uses on a route L band's boundary
	// (docs/modify-general-design.md §4.2): the cap contour's segments on the
	// band's face and the side contour's on the faces beside it. The record
	// bounds each such edge on one side only, and the band's patches bound
	// it on the other (brep_loop_band.go).
	open []int
	// faceUses lists each face's use indices in loop order: a swept face's
	// rim0, side1, rim1, side0 (a whole circle's rim0 and rim1 alone); a
	// planar face's loops in record order.
	faceUses [][]int
	// coordUpper is the largest |u|+|v| over every walk and |z| over every
	// level: the input magnitude the frame and placement lift rounds against.
	coordUpper float64
	// regions caches each planar face's integrated region (brepTopology.region).
	regions map[int]brepRegion
}

// brepTopologyContext walks every face, keys every edge use, and pairs them.
// An edge that does not bound exactly two distinct faces refuses: the build
// proves closure by counting (§4.2), so an unpaired use is a record this
// evaluator cannot close and is ErrUnsupported. The one exception is a
// route L band's boundary (brepPayload.loopBandKeys): each such edge must
// bound exactly one face of the record, and its use is listed in open.
func brepTopologyContext(ctx context.Context, bp brepPayload) (*brepTopology, error) {
	if err := falsifyBrepPayload(ctx, bp); err != nil {
		return nil, err
	}
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return nil, err
	}
	faces := make([]brepgeom.FaceWalks, len(bp.faces))
	work := freeform.NewFreeformWork()
	walk := func(seg curveSegment) (survey2d.SegmentWalk, error) {
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, "a brep face"); err != nil {
			return survey2d.SegmentWalk{}, err
		}
		return w, nil
	}
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face := brepgeom.FaceWalks{Embed: embeds[fi], IsPlanar: f.planar(),
			Outward: f.outward, Sweep: brepgeom.NoSweep,
			Z0: f.z0, Z1: f.z1, Z0Delta: f.z0Delta, Z1Delta: f.z1Delta,
			Side0: f.side0, Side1: f.side1}
		if f.planar() {
			if f.sweep != (r3.Vec{}) {
				axis, ok := brepgeom.SweepAxis(bp.faces[0].frame, f.sweep)
				if !ok || axis == embeds[fi].Axis[2] {
					return nil, fmt.Errorf(`%w: brep face %d's recorded sweep is not a reference axis in its plane`, ErrUnsupported, fi)
				}
				face.Sweep = axis
			}
			loops := append([]loopRecord{f.region.Outer}, f.region.Holes...)
			face.Planar = make([][]survey2d.SegmentWalk, len(loops))
			face.PlanarSegs = make([][]curveSegment, len(loops))
			for li, loop := range loops {
				face.PlanarSegs[li] = loop.Segments
				for _, seg := range loop.Segments {
					w, err := walk(seg)
					if err != nil {
						return nil, err
					}
					face.Planar[li] = append(face.Planar[li], w)
				}
			}
			faces[fi] = face
			continue
		}
		w, err := walk(f.wall)
		if err != nil {
			return nil, err
		}
		face.Wall, face.WallSeg = w, f.wall
		faces[fi] = face
	}
	open, err := bp.loopBandKeys(embeds, walk)
	if err != nil {
		return nil, err
	}
	built, err := brepgeom.Build(faces, open, ErrUnsupported)
	if err != nil {
		return nil, err
	}
	return &brepTopology{embeds: embeds, walls: built.Walls, planar: built.Planar,
		uses: built.Uses, edges: built.Edges, edgeOf: built.EdgeOf, open: built.Open,
		faceUses: built.FaceUses, coordUpper: built.CoordUpper}, nil
}

// forward reports whether use ui walks its edge in the edge's own direction,
// which is its owner use's natural one. A whole circle compares senses: it has
// no endpoints to compare.
func (topo *brepTopology) forward(ui int) bool {
	return brepgeom.Forward(topo.uses, topo.edges, topo.edgeOf, ui)
}

func brepIsRim(p brepPart) bool { return brepgeom.IsRim(p) }

// evalBrepContext builds the body a brepPayload records (§4.2, §4.3). Every
// edge is shared by exactly two faces, identified by record identity in the
// reference frame, so the body closes by construction and the count proves it.
// Roles are the record's own face(k)/wall(k) under ref; no capStart or capEnd
// is minted and no operand provenance is carried. A route L band's boundary
// edges bound one record face each, and the band's patches, attached after
// the record's faces, bound their other side (brep_loop_band.go).
func evalBrepContext(ctx context.Context, d *Document, ref producerID, bp brepPayload) (*Body, error) {
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	refView := bp.refView()

	// Vertices are shared by reference coordinates. Each carries the largest
	// displacement any use placing it states — its face's section displacement,
	// its level's, and the bound from its walk end to the point the use's
	// recorded segment denotes there (brepgeom.Use's StartBound and EndBound,
	// which add an arc's radial residual at its natural t = 1 end) — beside
	// the vertex's own exact frame and placement lift rounding
	// (prismPayload.liftedVertex), measured once per vertex when it is first
	// placed. Every use meeting at a vertex holds the same reference
	// coordinates, so the largest of them reaches every neighbour's denoted
	// end, as boundarywalk.JunctionVertex's bound does for a prism.
	type placedVertex struct {
		v    *Vertex
		lift float64
	}
	vertices := map[[3]float64]placedVertex{}
	// placeVertex returns the vertex at reference coordinates c, widening its
	// bound to cover one more use: faceDelta and levelDelta are that use's
	// section and level displacements and endAllow its denoted-end bound.
	placeVertex := func(c [3]float64, faceDelta, levelDelta, endAllow float64) *Vertex {
		pv, ok := vertices[c]
		if !ok {
			held, lift := refView.liftedVertex(c[0], c[1], c[2])
			// The bound starts as zero millimetres, so an Exact vertex
			// publishes a length as every other vertex does.
			pv = placedVertex{v: &Vertex{position: held, bound: units.Millimeters(0)}, lift: lift}
			vertices[c] = pv
		}
		if b := proofbound.AbsSumUpper(proofbound.AbsSumUpper(faceDelta, levelDelta, pv.lift), endAllow); b > pv.v.bound.Base() {
			pv.v.bound = units.Millimeters(b)
		}
		return pv.v
	}
	for ui, u := range topo.uses {
		// A whole circle's seam is its rim's: a loop that walks the same
		// circle from another seam places no vertex of its own. A band's
		// whole circle places its seam with its band edge (brepOpenEdges).
		closedOpen := u.Key.Closed && topo.edgeOf[ui] < 0
		if (u.Part != brepLoopSeg && !brepIsRim(u.Part)) || (u.Part == brepLoopSeg && u.Key.Closed) || closedOpen {
			continue
		}
		f := bp.faces[u.Face]
		placeVertex(u.DirFrom, f.delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.StartBound()))
		placeVertex(u.DirTo, f.delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.EndBound()))
	}
	// vertexAt is the lookup brepEdge reads: every vertex it names was already
	// placed by a use above, so it adds no displacement of its own.
	vertexAt := func(c [3]float64) *Vertex { return placeVertex(c, 0, 0, 0) }

	edges := make([]*Edge, len(topo.edges))
	for ei, pair := range topo.edges {
		edge, err := brepEdge(ctx, bp, topo, pair, vertexAt)
		if err != nil {
			return nil, err
		}
		edges[ei] = edge
	}
	open, err := brepOpenEdges(ctx, bp, topo, placeVertex)
	if err != nil {
		return nil, err
	}
	coedgeOf := func(ui int) coedge {
		if ei := topo.edgeOf[ui]; ei >= 0 {
			return coedge{edge: edges[ei], forward: topo.forward(ui)}
		}
		return open.coedge[ui]
	}

	faces := make([]*Face, len(bp.faces))
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := brepFaceBody(ctx, bp, topo, fi, body, ref)
		if err != nil {
			return nil, err
		}
		uses := topo.faceUses[fi]
		switch {
		case f.planar():
			loops := make([]*Loop, len(topo.planar[fi]))
			for li := range loops {
				loops[li] = &Loop{outer: li == 0}
			}
			for _, ui := range uses {
				l := loops[topo.uses[ui].Loop]
				l.coedges = append(l.coedges, coedgeOf(ui))
			}
			face.loops = loops
		case len(uses) == 2:
			face.loops = []*Loop{
				{coedges: []coedge{coedgeOf(uses[0])}, outer: true},
				{coedges: []coedge{coedgeOf(uses[1])}, outer: true},
			}
		default:
			l := &Loop{outer: true}
			for _, ui := range uses {
				l.coedges = append(l.coedges, coedgeOf(ui))
			}
			face.loops = []*Loop{l}
		}
		faces[fi] = face
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	bands, err := attachBrepLoopBands(ctx, body, ref, bp, open)
	if err != nil {
		return nil, err
	}
	body.lumps = sheetLumps(append(faces, bands.patches...))
	if err := measureBrepContext(ctx, bp, topo, body, bands.mass); err != nil {
		return nil, err
	}
	// Every face frame is a signed permutation of the first (brepEmbeds), so
	// each face's map has the first face's determinant and defect.
	if err := chargePrismMap(body, bp.faces[0].frame, bp.xform); err != nil {
		return nil, err
	}
	bp.loopPatches = bands.geom
	body.payload = bp
	return body, nil
}

// refView is the prism view of the reference frame (the first face's) under
// the placement: the one map every vertex, centroid and mesh point is lifted
// through.
func (bp brepPayload) refView() prismPayload {
	return prismPayload{frame: bp.faces[0].frame, xform: bp.xform}
}

// brepEdge builds one edge from its owner use: a rim takes its wall's curve and
// walk length, a side line runs bottom to top over the wall's bounded height,
// and a loop segment shared by two planar faces is a line of its walk's
// length. Convexity reads evaluator §3's walked boundary (brepEdgeConvex).
func brepEdge(ctx context.Context, bp brepPayload, topo *brepTopology, pair [2]int, vertexAt func([3]float64) *Vertex) (*Edge, error) {
	owner := topo.uses[pair[0]]
	f := bp.faces[owner.Face]
	start := vertexAt(owner.DirFrom)
	end := vertexAt(owner.DirTo)
	convex, err := brepEdgeConvex(ctx, topo, pair)
	if err != nil {
		return nil, err
	}
	switch owner.Part {
	case brepSide0, brepSide1:
		// One piece of the side line: its own two levels, read along the
		// sweep axis in reference coordinates.
		axis := topo.embeds[owner.Face].Axis[2]
		lo, hi := owner.DirFrom[axis], owner.DirTo[axis]
		if lo > hi {
			lo, hi = hi, lo
		}
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(hi, owner.SideDelta[1]), proofbound.MeasuredScalar(lo, owner.SideDelta[0]))
		return &Edge{curve: Line3{}, start: start, end: end, convex: convex,
			length: hi - lo, lengthBound: height.Bound}, nil
	case brepRim0, brepRim1:
		w := topo.walls[owner.Face]
		pp := f.view(bp.xform)
		bottom, top, _, _, err := buildWallGeometry(pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}},
			convex, w.Closed, start, end, start, end)
		if err != nil {
			return nil, err
		}
		edge := bottom
		if owner.Part == brepRim1 {
			edge = top
		}
		edge.lengthBound = proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.delta, 1))
		return edge, nil
	default:
		w := owner.Walk
		return &Edge{curve: Line3{}, start: start, end: end, convex: convex, length: w.Length,
			lengthBound: proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.delta, 1))}, nil
	}
}

// brepEdgeConvex decides an edge's walked-boundary convexity (topology.go's
// Edge.IsConvex). A rim reads the wall it runs along: a circular wall by its
// own turn, a straight wall by the role of the loop that wall belongs to,
// which the planar face's loop states — its own role when it walks the rim the
// way the wall does, the other role when it walks it the opposite way (a
// stacked floor is a reversed hole). A line two walls of one sweep share is
// a junction, convex when the walk turns left there about the sweep axis: a
// side line between two swept walls or between a swept wall and a planar face
// restating a wall along the same axis, and a line two such planar faces
// share (brepFace.sweep). Any other line reads the owner planar face's loop
// role: outer convex, hole concave.
func brepEdgeConvex(ctx context.Context, topo *brepTopology, pair [2]int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	return brepgeom.Convex(pair, topo.uses, topo.embeds, topo.walls, ErrUnsupported)
}
