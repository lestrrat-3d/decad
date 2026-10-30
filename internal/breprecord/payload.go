package breprecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Face is one face of a Payload. Exactly one of region and wall is
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
// displacements (agent-docs/prism-boolean-design.md §7); role is the face's role,
// face(k) or wall(k) for its index k in the record.
//
// blend is "fillet" or "chamfer" on a swept face a modify op's route E built
// (agent-docs/brep-modify-design.md §5.3, Table BB's BB4), and empty on every other
// face. The body build gives such a face a second role, fillet(k) or
// chamfer(k) for its index k, so a placement re-mints it with the record.
//
// wallCarrier is set on a swept circular face a class-B crossing build made
// (agent-docs/general-boolean-design.md §4.1): the operand circle the wall lies
// on, in the face's own frame. The recorded wall denotes that circle only to
// within delta. It is zero on every other face, and only route P's displaced
// reading reads it (agent-docs/brep-modify-design.md §4.4).
type Face struct {
	Frame            r3.Frame
	Region           *momentinput.Profile
	Outward          bool
	Sweep            r3.Vec
	Wall             sectionrecord.CurveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Side0, Side1     []brepgeom.Split
	Delta            float64
	Role             string
	Blend            string
	WallCarrier      brepgeom.CircleCarrier
}

// Stack is the slabs an A1 result was built from
// (agent-docs/general-boolean-design.md §3 "A1 as a brep", §4.1): each slab's
// hole-free regions in the reference frame, and the section displacement
// every region carries. A further co-directional Union reads it as its
// operand's slabs (stackedunion.OperandFromPayload), and Section reads it to
// reconstruct planar profiles. Placement leaves it unchanged because it
// moves xform alone.
type Stack struct {
	Slabs []stackedrecord.Slab
	Delta float64
}

// CutOperands are the exact prism records whose regularized Cut an eligible
// A7 BRep denotes for point queries. They do not make the BRep a Boolean
// operand; generic rebuilds discard them.
type CutOperands struct {
	Target prismrecord.Payload
	Tool   prismrecord.Payload
}

// Payload is the evaluator's record of an analytically trimmed body. Every
// face frame is stated in the payload's unplaced coordinates and xform places
// the whole body, as every payload does. stack is nil for every record but
// an A1 result's. loopBands holds a route L chamfer's or fillet's bands
// (agent-docs/modify-general-design.md §4.2 step 5, agent-docs/loop-fillet-design.md §4):
// each band's patches are faces of the body and no face of the record, and
// the body build attaches them (band.go, fillet.go). It
// is nil for every record route L did not build.
type Payload struct {
	Faces []Face
	Xform r3.Transform
	Stack *Stack
	// PointQueryCut keeps the exact source prisms of a line-only crossing blind
	// cut for point queries. Generic rebuilds drop it.
	PointQueryCut *CutOperands
	LoopBands     []LoopBand
	BossShell     *prismshell.BossShellBand
	PocketShell   *prismshell.PocketShellBand
	// loopPatches is each band's patch geometry, beside its role
	// chamferLoop(f,l,p), in band order, nil for a fillet band, whose
	// patches the readers take from the band record and the body's faces
	// (internal/brepfilletmesh/, brepFilletUndercuts). The body build fills it
	// (attachLoopBands); a record handed to the build carries none, and
	// the tessellator and the surveys read it from the payload the build
	// left on the body, as capblend.Payload.patches is read.
	LoopPatches [][]capblend.Patch
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish), the name the evaluator has always published for it.
func (Payload) SurveyClass() string { return "brepPayload" }

func (f Face) Planar() bool { return f.Region != nil }

// reversed is the face with its outward side exchanged, the rule a record
// applies to a face of a cavity (agent-docs/modify-general-design.md §3.3 step 5):
// a planar face flips outward, and a swept face walks its wall the other way
// and exchanges its start-line and end-line splits. Levels, frame and
// displacements are unchanged.
func (f Face) Reversed() Face {
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
func (f Face) View(xform r3.Transform) prismrecord.Payload {
	pp := prismrecord.Payload{
		Frame: f.Frame, Xform: xform,
		Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
		SectionDelta: f.Delta,
	}
	if f.Planar() {
		pp.Profile = *f.Region
	} else {
		pp.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{f.Wall}}}
	}
	return pp
}

func (bp Payload) Transform() r3.Transform { return bp.Xform }

// Placed re-evaluates the record under the composed motion. A reflection is
// read at build time, as prismrecord.Payload.reflected() is, so the record itself is
// unchanged.
func (bp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	bp.Xform = composed
	bp.PointQueryCut = nil
	return Build(ctx, env, bp)
}

// AxialDelta is the largest level displacement over every face.
func (bp Payload) AxialDelta() float64 {
	var out float64
	for _, f := range bp.Faces {
		out = max(out, f.Z0Delta, f.Z1Delta)
	}
	return out
}

// sectionDelta is the largest section displacement over every face.
func (bp Payload) SectionDelta() float64 {
	var out float64
	for _, f := range bp.Faces {
		out = max(out, f.Delta)
	}
	return out
}

// assignRoles names every face by its index in the record: face(k) for a
// planar face and wall(k) for a swept one (§4.2).
func (bp Payload) AssignRoles() {
	for k := range bp.Faces {
		if bp.Faces[k].Planar() {
			bp.Faces[k].Role = fmt.Sprintf("face(%d)", k)
			continue
		}
		bp.Faces[k].Role = fmt.Sprintf("wall(%d)", k)
	}
}

// Embeds states every face frame in the first face's frame, the reference
// every reading is taken in. A face frame must share the reference origin bit
// for bit and carry each reference axis, or its negation, bit for bit as one of
// its own, with the map keeping handedness. Such a map is a signed permutation,
// so every coordinate and level carries over exactly and the record's records
// compare by identity (§4.2). Any other frame is decaderr.ErrUnsupported: this build has
// no exact map for it.
func Embeds(faces []Face) ([]brepgeom.Embed, error) {
	frames := make([]r3.Frame, len(faces))
	for fi, f := range faces {
		frames[fi] = f.Frame
	}
	return brepgeom.Embeds(frames, decaderr.ErrUnsupported)
}

// Falsify refuses a record no brep body matches (ErrDegenerate) or
// one this evaluator does not build (decaderr.ErrUnsupported): a face with neither or
// both of a region and a wall, a non-finite level or displacement, an empty
// sweep interval, a planar face whose two levels differ, a role that is empty
// or repeated, or a segment other than a line, a circle or an arc (§3 B2). It
// runs before topology and before chording, and repairs nothing.
func Falsify(ctx context.Context, bp Payload) error {
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

// refView is the prism view of the reference frame (the first face's) under
// the placement: the one map every vertex, centroid and mesh point is lifted
// through.
func (bp Payload) RefView() prismrecord.Payload {
	return prismrecord.Payload{Frame: bp.Faces[0].Frame, Xform: bp.Xform}
}

// PrismFaceRecord is the face as the prism-read geometry takes it.
func PrismFaceRecord(f Face) brepgeom.PrismRectFace {
	return brepgeom.PrismRectFace{
		Frame: f.Frame, Region: f.Region, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1,
		Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Split0: len(f.Side0) != 0,
		Split1: len(f.Side1) != 0, Outward: f.Outward, Role: f.Role,
		Carrier: f.WallCarrier,
	}
}

// PrismFaceRecords is every face of bp as the prism-read geometry takes it.
func (bp Payload) PrismFaceRecords() []brepgeom.PrismRectFace {
	faces := make([]brepgeom.PrismRectFace, len(bp.Faces))
	for i, f := range bp.Faces {
		faces[i] = PrismFaceRecord(f)
	}
	return faces
}
