package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

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
	Frame            r3.Frame
	Region           *profileRecord
	Outward          bool
	Sweep            r3.Vec
	Wall             curveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Side0, Side1     []brepSplit
	Delta            float64
	Role             string
	Blend            string
}

// brepStack is the slabs an A1 result was built from
// (docs/general-boolean-design.md §3 "A1 as a brep", §4.1): each slab's
// hole-free regions in the reference frame, and the section displacement
// every region carries. A further co-directional Union reads it as its
// operand's slabs (stackedUnionOperandOf); no consumer reads it, and a
// placement leaves it unchanged, since it moves xform alone.
type brepStack struct {
	Slabs []stackedrecord.Slab
	Delta float64
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
	Faces       []brepFace
	Xform       r3.Transform
	Stack       *brepStack
	LoopBands   []brepLoopBand
	BossShell   *prismshell.BossShellBand
	PocketShell *prismshell.PocketShellBand
	// loopPatches is each band's patch geometry, beside its role
	// chamferLoop(f,l,p), in band order, nil for a fillet band, whose
	// patches the readers take from the band record and the body's faces
	// (tessellate_brep_fillet.go, brepFilletUndercuts). The body build fills it
	// (attachBrepLoopBands); a record handed to the build carries none, and
	// the tessellator and the surveys read it from the payload the build
	// left on the body, as capBlendPayload.patches is read.
	LoopPatches [][]capPatch
}

func (f brepFace) Planar() bool { return f.Region != nil }

// reversed is the face with its outward side exchanged, the rule a record
// applies to a face of a cavity (docs/modify-general-design.md §3.3 step 5):
// a planar face flips outward, and a swept face walks its wall the other way
// and exchanges its start-line and end-line splits. Levels, frame and
// displacements are unchanged.
func (f brepFace) Reversed() brepFace {
	if f.Planar() {
		f.Outward = !f.Outward
		return f
	}
	f.Wall = brepgeom.ReverseSegment(f.Wall)
	f.Side0, f.Side1 = f.Side1, f.Side0
	return f
}

// view is the prism the face's own frame, levels and displacements describe.
// A planar face's view carries its whole region at a zero height; a swept
// face's carries its one wall segment as an open single-segment loop, which
// only the per-segment readings (extents, witnesses) consume.
func (f brepFace) View(xform r3.Transform) prismPayload {
	pp := prismrecord.Payload{
		Frame: f.Frame, Xform: xform,
		Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
		SectionDelta: f.Delta,
	}
	if f.Planar() {
		pp.Profile = *f.Region
	} else {
		pp.Profile = profileRecord{Outer: loopRecord{Segments: []curveSegment{f.Wall}}}
	}
	return pp
}

func (bp brepPayload) Transform() r3.Transform { return bp.Xform }

// Placed re-evaluates the record under the composed motion. A reflection is
// read at build time, as prismPayload.reflected() is, so the record itself is
// unchanged.
func (bp brepPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	bp.Xform = composed
	return placedNode(evalBrepContext(ctx, d, ref, bp))
}

// AxialDelta is the largest level displacement over every face.
func (bp brepPayload) AxialDelta() float64 {
	var out float64
	for _, f := range bp.Faces {
		out = max(out, f.Z0Delta, f.Z1Delta)
	}
	return out
}

// sectionDelta is the largest section displacement over every face.
func (bp brepPayload) SectionDelta() float64 {
	var out float64
	for _, f := range bp.Faces {
		out = max(out, f.Delta)
	}
	return out
}

// assignRoles names every face by its index in the record: face(k) for a
// planar face and wall(k) for a swept one (§4.2).
func (bp brepPayload) AssignRoles() {
	for k := range bp.Faces {
		if bp.Faces[k].Planar() {
			bp.Faces[k].Role = fmt.Sprintf("face(%d)", k)
			continue
		}
		bp.Faces[k].Role = fmt.Sprintf("wall(%d)", k)
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
	if pp.SurfaceResult {
		return brepPayload{}, nil, fmt.Errorf(`%w: a surface-result prism has no closed face view`, ErrUnsupported)
	}
	profile, allow, err := brepJoinProfile(pp.Profile)
	if err != nil {
		return brepPayload{}, nil, err
	}
	pp.Profile = profile
	if allow > 0 {
		pp.SectionDelta = proofbound.AbsSumUpper(pp.SectionDelta, allow)
	}
	bp := brepPayload{Xform: pp.Xform}
	var sourceRoles []string
	for li, loop := range append([]loopRecord{pp.Profile.Outer}, pp.Profile.Holes...) {
		for si, seg := range loop.Segments {
			bp.Faces = append(bp.Faces, brepFace{
				Frame: pp.Frame, Wall: seg, Z0: pp.Z0, Z1: pp.Z1,
				Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta, Delta: pp.SectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("side(%d,%d)", li, si))
		}
	}
	region := pp.Profile
	bp.Faces = append(bp.Faces,
		brepFace{Frame: pp.Frame, Region: &region, Z0: pp.Z0, Z1: pp.Z0,
			Z0Delta: pp.Z0Delta, Z1Delta: pp.Z0Delta, Delta: pp.SectionDelta},
		brepFace{Frame: pp.Frame, Region: &region, Outward: true, Z0: pp.Z1, Z1: pp.Z1,
			Z0Delta: pp.Z1Delta, Z1Delta: pp.Z1Delta, Delta: pp.SectionDelta},
	)
	sourceRoles = append(sourceRoles, provenance.RoleCapStart, provenance.RoleCapEnd)
	bp.AssignRoles()
	return bp, sourceRoles, nil
}

// brepOfStacked is a stacked prism's face view (§4.1): one swept face per
// segment of every loop column (docs/stacked-prism-design.md §2.3), then the
// two caps and every interface's exposed floors and ceilings. The record is
// audited first, so a stack its own build refuses has no face view either. A
// prism group (one slab of several disjoint regions) has no face view and is
// ErrUnsupported: the record states one region per cap. So is a stack
// enclosing a cavity (a closed shell, EnclosesCavity).
func brepOfStacked(ctx context.Context, sp stackedPrismPayload) (brepPayload, error) {
	bp, _, err := brepOfStackedWithRoles(ctx, sp)
	return bp, err
}

// brepOfStackedWithRoles also returns each record face's role on the source
// stacked body. Chamfer uses the roles only while resolving a public face.
func brepOfStackedWithRoles(ctx context.Context, sp stackedPrismPayload) (brepPayload, []string, error) {
	if sp.IsGroup() {
		return brepPayload{}, nil, fmt.Errorf(`%w: a prism group of %d disjoint regions has no face view`, ErrUnsupported, len(sp.Slabs[0].Regions))
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return brepPayload{}, nil, err
	}
	cavity, err := sp.EnclosesCavity()
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
	columns, _, err := sp.Columns()
	if err != nil {
		return brepPayload{}, nil, err
	}
	bp := brepPayload{Xform: sp.Xform}
	var sourceRoles []string
	for _, col := range columns {
		first, last := sp.Slabs[col.Start], sp.Slabs[col.End]
		for segment, seg := range col.Loop.Segments {
			bp.Faces = append(bp.Faces, brepFace{
				Frame: sp.Frame, Wall: seg, Z0: first.Z0, Z1: last.Z1,
				Z0Delta: first.Z0Delta, Z1Delta: last.Z1Delta, Delta: sp.SectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
				col.Start, col.Region, col.LoopIndex, segment))
		}
	}
	addPlanar := func(region profileRecord, z, zDelta float64, outward bool, sourceRole string) {
		bp.Faces = append(bp.Faces, brepFace{Frame: sp.Frame, Region: &region, Outward: outward,
			Z0: z, Z1: z, Z0Delta: zDelta, Z1Delta: zDelta, Delta: sp.SectionDelta})
		sourceRoles = append(sourceRoles, sourceRole)
	}
	first, last := sp.Slabs[0], sp.Slabs[len(sp.Slabs)-1]
	addPlanar(first.Regions[0], first.Z0, first.Z0Delta, false, provenance.RoleCapStart)
	addPlanar(last.Regions[0], last.Z1, last.Z1Delta, true, provenance.RoleCapEnd)
	for k, boundary := range sp.Interfaces {
		z, zDelta := sp.Slabs[k].Z1, sp.Slabs[k].Z1Delta
		for e, region := range boundary.LowerExposed {
			addPlanar(region, z, zDelta, true, fmt.Sprintf("floor(%d,%d)", k, e))
		}
		for e, region := range boundary.UpperExposed {
			addPlanar(region, z, zDelta, false, fmt.Sprintf("ceiling(%d,%d)", k, e))
		}
	}
	bp.AssignRoles()
	return bp, sourceRoles, nil
}

// brepJoinProfile adapts a region to brepgeom.JoinProfile.
func brepJoinProfile(p profileRecord) (profileRecord, float64, error) {
	joined, allow, err := brepgeom.JoinProfile(brepgeom.Profile{Outer: p.Outer, Holes: p.Holes})
	return profileRecord{Outer: joined.Outer, Holes: joined.Holes}, allow, err
}

// brepJoinStacked joins every region and exposed record of a
// stack, charging the largest allow to its section displacement. Equal loops
// join alike, so the columns Payload.Columns derives are unchanged.
func brepJoinStacked(sp stackedPrismPayload) (stackedPrismPayload, error) {
	allow := 0.0
	join := func(p profileRecord) (profileRecord, error) {
		out, a, err := brepJoinProfile(p)
		allow = math.Max(allow, a)
		return out, err
	}
	out := sp
	out.Slabs = make([]stackedrecord.Slab, len(sp.Slabs))
	for k, slab := range sp.Slabs {
		out.Slabs[k] = slab
		out.Slabs[k].Regions = make([]profileRecord, len(slab.Regions))
		for r, region := range slab.Regions {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.Slabs[k].Regions[r] = joined
		}
	}
	out.Interfaces = make([]stackedrecord.Interface, len(sp.Interfaces))
	for k, boundary := range sp.Interfaces {
		for _, region := range boundary.LowerExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.Interfaces[k].LowerExposed = append(out.Interfaces[k].LowerExposed, joined)
		}
		for _, region := range boundary.UpperExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.Interfaces[k].UpperExposed = append(out.Interfaces[k].UpperExposed, joined)
		}
	}
	if allow > 0 {
		out.SectionDelta = proofbound.AbsSumUpper(sp.SectionDelta, allow)
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
		frames[fi] = f.Frame
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
	return brepgeom.ValidateFaces(ctx, len(bp.Faces), func(i int) brepgeom.FaceRecord {
		f := bp.Faces[i]
		var region *brepgeom.Profile
		if f.Region != nil {
			region = &brepgeom.Profile{Outer: f.Region.Outer, Holes: f.Region.Holes}
		}
		return brepgeom.FaceRecord{
			Frame: f.Frame, Region: region, Wall: f.Wall,
			Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
			Side0: f.Side0, Side1: f.Side1,
			Delta: f.Delta, Role: f.Role,
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
	Embeds []brepEmbed
	// walls holds a swept face's walk, planar holds each planar face's walks
	// per region loop.
	Walls  map[int]survey2d.SegmentWalk
	Planar map[int][][]survey2d.SegmentWalk
	Uses   []brepUse
	// edges lists the two use indices of every edge, the owner first: a rim use
	// where the edge has one, then a side line, then a loop segment. The
	// owner's natural direction is the edge's direction.
	Edges [][2]int
	// edgeOf maps a use index to its edge index, -1 for an open use.
	EdgeOf []int
	// open lists the uses on a route L band's boundary
	// (docs/modify-general-design.md §4.2): the cap contour's segments on the
	// band's face and the side contour's on the faces beside it. The record
	// bounds each such edge on one side only, and the band's patches bound
	// it on the other (brep_loop_band.go).
	Open []int
	// faceUses lists each face's use indices in loop order: a swept face's
	// rim0, side1, rim1, side0 (a whole circle's rim0 and rim1 alone); a
	// planar face's loops in record order.
	FaceUses [][]int
	// coordUpper is the largest |u|+|v| over every walk and |z| over every
	// level: the input magnitude the frame and placement lift rounds against.
	CoordUpper float64
	// regions caches each planar face's integrated region (brepTopology.region).
	Regions map[int]brepRegion
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
	embeds, err := brepEmbeds(bp.Faces)
	if err != nil {
		return nil, err
	}
	faces := make([]brepgeom.FaceWalks, len(bp.Faces))
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
	for fi, f := range bp.Faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face := brepgeom.FaceWalks{Embed: embeds[fi], IsPlanar: f.Planar(),
			Outward: f.Outward, Sweep: brepgeom.NoSweep,
			Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
			Side0: f.Side0, Side1: f.Side1}
		if f.Planar() {
			if f.Sweep != (r3.Vec{}) {
				axis, ok := brepgeom.SweepAxis(bp.Faces[0].Frame, f.Sweep)
				if !ok || axis == embeds[fi].Axis[2] {
					return nil, fmt.Errorf(`%w: brep face %d's recorded sweep is not a reference axis in its plane`, ErrUnsupported, fi)
				}
				face.Sweep = axis
			}
			loops := append([]loopRecord{f.Region.Outer}, f.Region.Holes...)
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
		w, err := walk(f.Wall)
		if err != nil {
			return nil, err
		}
		face.Wall, face.WallSeg = w, f.Wall
		faces[fi] = face
	}
	open, err := bp.LoopBandKeys(embeds, walk)
	if err != nil {
		return nil, err
	}
	built, err := brepgeom.Build(faces, open, ErrUnsupported)
	if err != nil {
		return nil, err
	}
	return &brepTopology{Embeds: embeds, Walls: built.Walls, Planar: built.Planar,
		Uses: built.Uses, Edges: built.Edges, EdgeOf: built.EdgeOf, Open: built.Open,
		FaceUses: built.FaceUses, CoordUpper: built.CoordUpper}, nil
}

// forward reports whether use ui walks its edge in the edge's own direction,
// which is its owner use's natural one. A whole circle compares senses: it has
// no endpoints to compare.
func (btopo *brepTopology) Forward(ui int) bool {
	return brepgeom.Forward(btopo.Uses, btopo.Edges, btopo.EdgeOf, ui)
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
	btopo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	body := bornBody(&Body{n: topo.Body{Solid: true, Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Kind: topo.BodySolid, Doc: documentNode(d)}})
	refView := bp.RefView()

	// Vertices are shared by reference coordinates. Each carries the largest
	// displacement any use placing it states — its face's section displacement,
	// its level's, and the bound from its walk end to the point the use's
	// recorded segment denotes there (brepgeom.Use's StartBound and EndBound,
	// which add an arc's radial residual at its natural t = 1 end) — beside
	// the vertex's own exact frame and placement lift rounding
	// (prismPayload.LiftedVertex), measured once per vertex when it is first
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
			held, lift := refView.LiftedVertex(c[0], c[1], c[2])
			// The bound starts as zero millimetres, so an Exact vertex
			// publishes a length as every other vertex does.
			pv = placedVertex{v: bornVertex(&Vertex{n: topo.Vertex{Position: held, Bound: units.Millimeters(0)}}), lift: lift}
			vertices[c] = pv
		}
		if b := proofbound.AbsSumUpper(proofbound.AbsSumUpper(faceDelta, levelDelta, pv.lift), endAllow); b > pv.v.n.Bound.Base() {
			pv.v.n.Bound = units.Millimeters(b)
		}
		return pv.v
	}
	for ui, u := range btopo.Uses {
		// A whole circle's seam is its rim's: a loop that walks the same
		// circle from another seam places no vertex of its own. A band's
		// whole circle places its seam with its band edge (brepOpenEdges).
		closedOpen := u.Key.Closed && btopo.EdgeOf[ui] < 0
		if (u.Part != brepLoopSeg && !brepIsRim(u.Part)) || (u.Part == brepLoopSeg && u.Key.Closed) || closedOpen {
			continue
		}
		f := bp.Faces[u.Face]
		placeVertex(u.DirFrom, f.Delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.StartBound()))
		placeVertex(u.DirTo, f.Delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.EndBound()))
	}
	// vertexAt is the lookup brepEdge reads: every vertex it names was already
	// placed by a use above, so it adds no displacement of its own.
	vertexAt := func(c [3]float64) *Vertex { return placeVertex(c, 0, 0, 0) }

	edges := make([]*Edge, len(btopo.Edges))
	for ei, pair := range btopo.Edges {
		edge, err := brepEdge(ctx, bp, btopo, pair, vertexAt)
		if err != nil {
			return nil, err
		}
		edges[ei] = edge
	}
	open, err := brepOpenEdges(ctx, bp, btopo, placeVertex)
	if err != nil {
		return nil, err
	}
	coedgeOf := func(ui int) topo.CoEdge {
		if ei := btopo.EdgeOf[ui]; ei >= 0 {
			return topo.CoEdge{Edge: edgeNode(edges[ei]), Forward: btopo.Forward(ui)}
		}
		return open.coedge[ui]
	}

	faces := make([]*Face, len(bp.Faces))
	for fi, f := range bp.Faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := brepFaceBody(ctx, bp, btopo, fi, body, ref)
		if err != nil {
			return nil, err
		}
		uses := btopo.FaceUses[fi]
		switch {
		case f.Planar():
			loops := make([]*Loop, len(btopo.Planar[fi]))
			for li := range loops {
				loops[li] = bornLoop(&Loop{n: topo.Loop{Outer: li == 0}})
			}
			for _, ui := range uses {
				l := loops[btopo.Uses[ui].Loop]
				l.n.CoEdges = append(l.n.CoEdges, coedgeOf(ui))
			}
			face.n.Loops = loopNodes(loops)
		case len(uses) == 2:
			face.n.Loops = []*topo.Loop{
				loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{coedgeOf(uses[0])}}})),
				loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{coedgeOf(uses[1])}}})),
			}
		default:
			l := bornLoop(&Loop{n: topo.Loop{Outer: true}})
			for _, ui := range uses {
				l.n.CoEdges = append(l.n.CoEdges, coedgeOf(ui))
			}
			face.n.Loops = []*topo.Loop{loopNode(l)}
		}
		faces[fi] = face
	}
	if err := topobuild.AttachFaceLoops(ctx, faceNodes(faces)); err != nil {
		return nil, err
	}
	bands, err := attachBrepLoopBands(ctx, body, ref, bp, open)
	if err != nil {
		return nil, err
	}
	if bp.BossShell != nil {
		patches, err := attachBossShellBand(ctx, body, ref, bp, open)
		if err != nil {
			return nil, err
		}
		bands.patches = append(bands.patches, patches...)
	}
	if bp.PocketShell != nil {
		patches, err := attachPocketShellBand(ctx, body, ref, bp, open)
		if err != nil {
			return nil, err
		}
		bands.patches = append(bands.patches, patches...)
	}
	allFaces := append(faces, bands.patches...)
	if bp.BossShell != nil && bp.BossShell.Closed {
		var lumps []*Lump
		lumps, err = closedBossShellLumps(allFaces, bp.BossShell.OuterFaceCount)
		body.n.Lumps = lumpNodes(lumps)
		if err != nil {
			return nil, err
		}
	} else {
		body.n.Lumps = topobuild.SheetLumps(handleAlloc{}, faceNodes(allFaces))
	}
	if bp.BossShell != nil {
		err = measureBossShell(ctx, bp, body)
	} else if bp.PocketShell != nil {
		err = measurePocketShell(ctx, bp, body)
	} else {
		err = measureBrepContext(ctx, bp, btopo, body, bands.mass)
	}
	if err != nil {
		return nil, err
	}
	// Every face frame is a signed permutation of the first (brepEmbeds), so
	// each face's map has the first face's determinant and defect.
	if err := prismrecord.ChargeMap(bodyNode(body), bp.Faces[0].Frame, bp.Xform); err != nil {
		return nil, err
	}
	bp.LoopPatches = bands.geom
	body.n.Payload = bp
	return body, nil
}

// refView is the prism view of the reference frame (the first face's) under
// the placement: the one map every vertex, centroid and mesh point is lifted
// through.
func (bp brepPayload) RefView() prismPayload {
	return prismrecord.Payload{Frame: bp.Faces[0].Frame, Xform: bp.Xform}
}

// brepEdge builds one edge from its owner use: a rim takes its wall's curve and
// walk length, a side line runs bottom to top over the wall's bounded height,
// and a loop segment shared by two planar faces is a line of its walk's
// length. Convexity reads evaluator §3's walked boundary (brepEdgeConvex).
func brepEdge(ctx context.Context, bp brepPayload, btopo *brepTopology, pair [2]int, vertexAt func([3]float64) *Vertex) (*Edge, error) {
	owner := btopo.Uses[pair[0]]
	f := bp.Faces[owner.Face]
	start := vertexAt(owner.DirFrom)
	end := vertexAt(owner.DirTo)
	convex, err := brepEdgeConvex(ctx, btopo, pair)
	if err != nil {
		return nil, err
	}
	switch owner.Part {
	case brepSide0, brepSide1:
		// One piece of the side line: its own two levels, read along the
		// sweep axis in reference coordinates.
		axis := btopo.Embeds[owner.Face].Axis[2]
		lo, hi := owner.DirFrom[axis], owner.DirTo[axis]
		if lo > hi {
			lo, hi = hi, lo
		}
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(hi, owner.SideDelta[1]), proofbound.MeasuredScalar(lo, owner.SideDelta[0]))
		return bornEdge(&Edge{n: topo.Edge{
			Convex:      convex,
			Length:      hi - lo,
			LengthBound: height.Bound,
			Curve:       surfacegeom.Line3{},
			Start:       vertexNode(start),
			End:         vertexNode(end),
		}}), nil
	case brepRim0, brepRim1:
		w := btopo.Walls[owner.Face]
		pp := f.View(bp.Xform)
		bottom, top, _, _, err := prismrecord.BuildWallGeometry(handleAlloc{}, pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, convex, w.Closed, vertexNode(start), vertexNode(end), vertexNode(start), vertexNode(end))
		if err != nil {
			return nil, err
		}
		edge := bottom
		if owner.Part == brepRim1 {
			edge = top
		}
		edge.LengthBound = proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.Delta, 1))
		return edgeHandle(edge), nil
	default:
		w := owner.Walk
		return bornEdge(&Edge{n: topo.Edge{
			Convex:      convex,
			Length:      w.Length,
			LengthBound: proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.Delta, 1)),
			Curve:       surfacegeom.Line3{},
			Start:       vertexNode(start),
			End:         vertexNode(end),
		}}), nil
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
func brepEdgeConvex(ctx context.Context, btopo *brepTopology, pair [2]int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	return brepgeom.Convex(pair, btopo.Uses, btopo.Embeds, btopo.Walls, ErrUnsupported)
}
