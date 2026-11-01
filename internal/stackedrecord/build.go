package stackedrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a stacked prism's body from its Payload: one wall column
// per recorded loop run, the caps and interface patches, and the
// measurements the finished body publishes (agent-docs/stacked-prism-design.md).

// WallColumn is a recorded wall column together with what its build produced:
// the bottom and top coedges of the column's rim and the column's proven
// perimeter.
type WallColumn struct {
	Column
	Bottom    []topo.CoEdge
	Top       []topo.CoEdge
	Perimeter proofbound.BoundedScalar
}

// Plan names a stacked body's faces and states the displacement each
// wall column's own loop carries beside the payload-wide SectionDelta. The
// payload's own plan (OwnPlan) mints §3's roles and charges no column
// displacement. A cup builds through its own plan: it keeps the cup's public
// roles and charges its offset loops their proven offset displacement, loop by
// loop.
type Plan interface {
	// WallRoleLoop is the loop index prismrecord.BuildLoopSidesAs mints into
	// column col's side(i,j) roles, and NameWalls rewrites the roles minted on
	// that column's walls.
	WallRoleLoop(col WallColumn) int
	NameWalls(ctx context.Context, col WallColumn, faces []*topo.Face, ref provenance.Producer) error
	// CapRole names the bottom (top false) or top cap of one region of the
	// first or the last slab.
	CapRole(slab, region int, top bool) string
	// PatchRole names an exposed interface patch.
	PatchRole(patch Patch) string
	// ColumnDelta is the section displacement column col's loop carries
	// beyond the payload's SectionDelta.
	ColumnDelta(col WallColumn) float64
	// Order is the body's face order, from the build order.
	Order(faces []*topo.Face) []*topo.Face
}

// OwnPlan is the stacked payload's own naming (§3).
type OwnPlan struct{}

// WallRoleLoop is the column's own loop index.
func (OwnPlan) WallRoleLoop(col WallColumn) int { return col.LoopIndex }

// NameWalls prefixes every wall role with its column's slab and region.
func (OwnPlan) NameWalls(_ context.Context, col WallColumn, faces []*topo.Face, _ provenance.Producer) error {
	for _, face := range faces {
		for i := range face.Origins {
			face.Origins[i].Role = fmt.Sprintf("slab(%d).region(%d).%s", col.Start, col.Region, face.Origins[i].Role)
		}
	}
	return nil
}

// CapRole is capEnd for a top cap and capStart for a bottom one.
func (OwnPlan) CapRole(_, _ int, top bool) string {
	if top {
		return provenance.RoleCapEnd
	}
	return provenance.RoleCapStart
}

// PatchRole is the patch record's own role.
func (OwnPlan) PatchRole(patch Patch) string { return patch.Role }

// ColumnDelta is zero: the own plan charges no column displacement.
func (OwnPlan) ColumnDelta(WallColumn) float64 { return 0 }

// Order keeps the build order.
func (OwnPlan) Order(faces []*topo.Face) []*topo.Face { return faces }

// Columns adapts recorded wall columns for the topology build.
func (sp Payload) Columns() ([]WallColumn, [][]SlabLoop, error) {
	planned, loops, err := Columns(sp.Record())
	if err != nil {
		return nil, nil, err
	}
	columns := make([]WallColumn, len(planned))
	for i, col := range planned {
		columns[i] = WallColumn{Column: col}
	}
	return columns, loops, nil
}

// InterfacePatches passes the recorded columns to the shared patch plan.
func (sp Payload) InterfacePatches(columns []WallColumn, bySlab [][]SlabLoop, k int) ([]Patch, error) {
	plannedColumns := make([]Column, len(columns))
	for i, col := range columns {
		plannedColumns[i] = col.Column
	}
	return InterfacePatches(sp.Record(), plannedColumns, bySlab, k)
}

// EnclosesCavity reports whether a stack encloses a cavity: a hole
// column that starts and ends at interfaces, so an exposed floor closes it
// below and an exposed ceiling above (§2.4's closed shell).
func (sp Payload) EnclosesCavity() (bool, error) {
	columns, _, err := sp.Columns()
	if err != nil {
		return false, err
	}
	for _, col := range columns {
		if col.LoopIndex != 0 && col.Start > 0 && col.End < len(sp.Slabs)-1 {
			return true, nil
		}
	}
	return false, nil
}

// boundsContext is the union of every outer run's box. Each run's box
// bounds its own extremes, so the union's extreme along each axis is one
// run's, and the largest run bound covers it. outerDelta is the largest
// column displacement any run's outer loop carries beyond SectionDelta
// (Plan.ColumnDelta), charged to every run as its section displacement.
func boundsContext(ctx context.Context, sp Payload, outerDelta float64, work *freeform.FreeformWork) (measurement.Box, error) {
	runs := sp.OuterRuns()
	var out measurement.Box
	bound := 0.0
	for i, run := range runs {
		if outerDelta > 0 {
			run.SectionDelta = proofbound.AbsSumUpper(run.SectionDelta, outerDelta)
		}
		box, err := prismrecord.Bounds(ctx, run, work, nil)
		if err != nil {
			return measurement.Box{}, err
		}
		bound = max(bound, box.Bound.Base())
		if i == 0 {
			out = box
			continue
		}
		out.Min = r3.NewVec(min(out.Min.X, box.Min.X), min(out.Min.Y, box.Min.Y), min(out.Min.Z, box.Min.Z))
		out.Max = r3.NewVec(max(out.Max.X, box.Max.X), max(out.Max.Y, box.Max.Y), max(out.Max.Z, box.Max.Z))
	}
	out.Exactness = measurement.ExactnessOf(bound)
	out.Bound = units.Millimeters(bound)
	return out, nil
}

// patchArea is a patch record's area: its outer loop's enclosed area
// less each hole's, with sectionDisplacementArea over every loop's walks and
// proven perimeter folded into the bound (§4), and each loop's own column
// displacement (colDelta) over that loop alone.
func patchArea(ctx context.Context, sectionDelta float64, colDelta []float64, columns []WallColumn, patch Patch) (proofbound.BoundedScalar, error) {
	area, err := momentinput.LoopEnclosedArea(ctx, patch.Record.Outer)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	for _, hole := range patch.Record.Holes {
		holeArea, err := momentinput.LoopEnclosedArea(ctx, hole)
		if err != nil {
			return proofbound.BoundedScalar{}, err
		}
		area = proofbound.BoundedSub(area, holeArea)
	}
	walks := 0
	perimeter := 0.0
	for _, pl := range patch.Loops {
		col := columns[pl.Column]
		walks += len(col.Loop.Segments)
		loopPerimeter := proofbound.AbsSumUpper(col.Perimeter.Value, col.Perimeter.Bound)
		perimeter = proofbound.AbsSumUpper(perimeter, loopPerimeter)
		if delta := colDelta[pl.Column]; delta > 0 {
			area.Bound = proofbound.AbsSumUpper(area.Bound,
				proofbound.SectionDisplacementArea(delta, len(col.Loop.Segments), loopPerimeter))
		}
	}
	area.Bound = proofbound.AbsSumUpper(area.Bound, proofbound.SectionDisplacementArea(sectionDelta, walks, perimeter))
	return area, nil
}

// lumps groups a stacked body's faces into lumps and shells. Each
// connected face set holding an anchor — a region outer's wall or an end cap,
// the faces the body's exterior is made of — is one lump's outer shell. A
// connected set holding none bounds a cavity: its walls are hole loops and its
// planar faces are interface patches, all facing into it, so it is a void
// shell (§2.4's closed shell). The cavity lies in the one outer shell's
// material; a body with several outer shells and any cavity would need a
// nesting proof this build does not run, and is ErrUnsupported.
func lumps(alloc topo.Alloc, faces []*topo.Face, anchors map[*topo.Face]struct{}) ([]*topo.Lump, error) {
	var outer []*topo.Lump
	var voids []*topo.FaceShell
	for _, group := range topobuild.SplitConnectedFaces(faces) {
		anchored := false
		for _, f := range group {
			if _, ok := anchors[f]; ok {
				anchored = true
				break
			}
		}
		shell := alloc.NewFaceShell()
		shell.Open = topobuild.ShellIsOpen(group)
		shell.Faces = group
		if !anchored {
			shell.Void = true
			voids = append(voids, shell)
			continue
		}
		lump := alloc.NewLump()
		lump.Shells = []*topo.FaceShell{shell}
		outer = append(outer, lump)
	}
	switch {
	case len(voids) == 0:
		return outer, nil
	case len(outer) != 1:
		return nil, fmt.Errorf(`%w: a stacked body with %d outer shells and %d cavities needs a nesting proof this evaluator does not run`,
			decaderr.ErrUnsupported, len(outer), len(voids))
	}
	outer[0].Shells = append(outer[0].Shells, voids...)
	return outer, nil
}

// liningCavity reports whether col is a closed shell's hole lining: an
// outer column of a record marked ClosedShell that starts and ends at an
// interface, so a floor closes it below and a ceiling above and its wall
// faces the cavity (§2.4).
func (sp Payload) liningCavity(col Column) bool {
	return sp.ClosedShell && col.LoopIndex == 0 && col.Start > 0 && col.End < len(sp.Slabs)-1
}

// slabPart is one (slab, region) prism of a stacked body: its bounded
// section area and its centroid lifted to the slab's bounded midpoint.
type slabPart struct {
	slab, region  int
	area          proofbound.BoundedScalar
	coordUpper    float64
	centroid      r3.Vec
	centroidBound float64
}

// regionPart integrates one slab region. The region's area carries
// sectionDisplacementArea for the payload's SectionDelta over all its walks,
// and each loop's own column displacement over that loop alone; a region with
// a displaced loop also widens its first moments by that displacement area
// times the farthest coordinate the displaced boundary can reach, as
// displacedRegionIntegrals does, so the centroid quotient covers it.
func regionPart(ctx context.Context, sp Payload, base prismrecord.Payload, columns []WallColumn, colDelta []float64,
	entries []SlabLoop, k, r int, work *freeform.FreeformWork, budget *proofbound.ReplayBudget,
	prepared *replayBlindBuildPlan) (slabPart, error) {
	slab := sp.Slabs[k]
	region := slab.Regions[r]
	var ig momentinput.Integrals
	var err error
	if budget == nil {
		ig, err = region.EvaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, work)
	} else if prepared != nil {
		var ok bool
		ig, ok, err = replayBlindRegionMoments(ctx, region, prepared.target, budget)
		if err == nil && !ok {
			err = fmt.Errorf("%w: blind region has no bound source moments", decaderr.ErrUnsupported)
		}
	} else {
		err = fmt.Errorf("%w: blind region has no replay plan", decaderr.ErrUnsupported)
	}
	if err != nil {
		return slabPart{}, err
	}
	if ig.Area <= 0 {
		return slabPart{}, fmt.Errorf(`%w: slab %d region %d has no material area`, decaderr.ErrDegenerate, k, r)
	}
	perimeter := proofbound.BoundedScalar{}
	walks := 0
	loopArea, loopReach := 0.0, 0.0
	for _, e := range entries {
		if e.Region != r {
			continue
		}
		col := columns[e.Column]
		if budget == nil {
			perimeter = proofbound.BoundedAdd(perimeter, col.Perimeter)
		} else {
			perimeter, err = proofbound.BoundedAddReplay(ctx, perimeter, col.Perimeter, budget)
			if err != nil {
				return slabPart{}, err
			}
		}
		walks += len(col.Loop.Segments)
		if delta := colDelta[e.Column]; delta > 0 {
			loopArea = proofbound.AbsSumUpper(loopArea, proofbound.SectionDisplacementArea(delta, len(col.Loop.Segments),
				proofbound.AbsSumUpper(col.Perimeter.Value, col.Perimeter.Bound)))
			loopReach = max(loopReach, delta)
		}
	}
	if loopArea > 0 {
		coord, err := momentinput.CoordinateEnvelope(region, work, nil)
		if err != nil {
			return slabPart{}, err
		}
		moment := proofbound.ProductUpper(loopArea, proofbound.AbsSumUpper(coord, loopReach))
		ig.AreaBound = proofbound.AbsSumUpper(ig.AreaBound, loopArea)
		ig.MuBound = proofbound.AbsSumUpper(ig.MuBound, moment)
		ig.MvBound = proofbound.AbsSumUpper(ig.MvBound, moment)
	}
	out := slabPart{slab: k, region: r, coordUpper: ig.CoordUpper,
		area: proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(ig.AreaBound,
			proofbound.SectionDisplacementArea(sp.SectionDelta, walks, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))))}
	var u, v proofbound.BoundedScalar
	if budget == nil {
		u = proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
		v = proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	} else {
		u, err = proofbound.BoundedQuotientReplay(ctx, ig.Mu, ig.MuBound, ig.Area, ig.AreaBound, budget)
		if err != nil {
			return slabPart{}, err
		}
		v, err = proofbound.BoundedQuotientReplay(ctx, ig.Mv, ig.MvBound, ig.Area, ig.AreaBound, budget)
		if err != nil {
			return slabPart{}, err
		}
	}
	u.Bound = proofbound.AbsSumUpper(u.Bound, sp.SectionDelta)
	v.Bound = proofbound.AbsSumUpper(v.Bound, sp.SectionDelta)
	first, last := proofbound.MeasuredScalar(slab.Z0, slab.Z0Delta), proofbound.MeasuredScalar(slab.Z1, slab.Z1Delta)
	var sum, mid proofbound.BoundedScalar
	if budget == nil {
		sum = proofbound.BoundedAdd(first, last)
		mid = proofbound.BoundedDiv(sum, proofbound.ExactScalar(2))
		out.centroid = base.Point(u.Value, v.Value, mid.Value)
		out.centroidBound = base.PointBound(u, v, mid)
	} else {
		sum, err = proofbound.BoundedAddReplay(ctx, first, last, budget)
		if err != nil {
			return slabPart{}, err
		}
		mid, err = proofbound.BoundedDivReplay(ctx, sum, proofbound.ExactScalar(2), budget)
		if err != nil {
			return slabPart{}, err
		}
		out.centroid, out.centroidBound, err = base.PointBoundSignedReplay(ctx, u, v, mid, budget)
		if err != nil {
			return slabPart{}, err
		}
	}
	return out, nil
}

// Build evaluates sp into a new body allocated through env.Alloc, under
// plan's naming and column displacements (§3, §4). It mints identities
// through env.Doc and roles under env.Ref, and never commits: the caller maps
// the result to its handle and commits it.
// CircleWallWitness identifies a directly built whole-circle side face.
type CircleWallWitness struct {
	Column, Start, End, Region, Loop                                         int
	Face                                                                     *topo.Face
	Circle                                                                   sectionrecord.CircleSeg
	Origin                                                                   prismcells.Origin
	HasOrigin                                                                bool
	SourceFace, LeafFace                                                     *topo.Face
	CapStart, CapEnd, SourceCapStart, SourceCapEnd, LeafCapStart, LeafCapEnd *topo.Face
	CapAmbiguous                                                             bool
	Z0, Z1, Z0Delta, Z1Delta                                                 float64
	SectionDelta                                                             float64
	Frame                                                                    r3.Frame
	Xform                                                                    r3.Transform
}

// RectangleWallWitness binds each original line to its constructed side face.
type RectangleWallWitness struct {
	Column, Start, End, Region, Loop       int
	Walls                                  [4]*topo.Face
	Lines                                  [4]sectionrecord.LineSeg
	Origins                                [4]prismcells.Origin
	HasOrigins                             bool
	CapStart, CapEnd                       *topo.Face
	CapAmbiguous                           bool
	Z0, Z1, Z0Delta, Z1Delta, SectionDelta float64
	Frame                                  r3.Frame
	Xform                                  r3.Transform
}

func BuildWithCircleWallWitnesses(ctx context.Context, env topo.Env,
	sp Payload) (*topo.Body, []CircleWallWitness, error) {
	var circles []CircleWallWitness
	body, err := build(ctx, env, sp, OwnPlan{}, nil, nil, nil, &circles, nil)
	if err != nil {
		return nil, nil, err
	}
	return body, circles, nil
}

func BuildWithBothWallWitnesses(ctx context.Context, env topo.Env,
	sp Payload) (*topo.Body, []CircleWallWitness, []RectangleWallWitness, error) {
	var circles []CircleWallWitness
	var rectangles []RectangleWallWitness
	body, err := build(ctx, env, sp, OwnPlan{}, nil, nil, nil, &circles, &rectangles)
	if err != nil {
		return nil, nil, nil, err
	}
	return body, circles, rectangles, nil
}

func Build(ctx context.Context, env topo.Env, sp Payload, plan Plan) (*topo.Body, error) {
	return build(ctx, env, sp, plan, nil, nil, nil, nil, nil)
}

func build(ctx context.Context, env topo.Env, sp Payload, plan Plan, budget *proofbound.ReplayBudget,
	prepared *replayBlindBuildPlan, log *ReplayBlindFaceLog,
	witnesses *[]CircleWallWitness, rectangles *[]RectangleWallWitness) (*topo.Body, error) {
	if budget != nil && (witnesses != nil || rectangles != nil) {
		return nil, fmt.Errorf("%w: replay build cannot mix witness modes", decaderr.ErrUnsupported)
	}
	if budget != nil && log == nil {
		return nil, fmt.Errorf("%w: blind replay has no construction face log", decaderr.ErrUnsupported)
	}
	if budget != nil {
		if _, ok := plan.(OwnPlan); !ok || prepared == nil || prepared.sourceBody == nil {
			return nil, fmt.Errorf("%w: blind replay requires its authenticated source plan", decaderr.ErrUnsupported)
		}
		if err := budget.SpendWork(ctx, uint64(len(sp.Interfaces))+1); err != nil {
			return nil, err
		}
	}
	if !sp.OpposedSectionExact() {
		return nil, fmt.Errorf(`%w: opposed holes at a stacked interface carry section displacement`, decaderr.ErrUnsupported)
	}
	var columns []WallColumn
	var bySlab [][]SlabLoop
	var err error
	if prepared == nil {
		if err := Falsify(ctx, sp.Record()); err != nil {
			return nil, err
		}
		columns, bySlab, err = sp.Columns()
		if err != nil {
			return nil, err
		}
	} else {
		columns, bySlab = prepared.columns, prepared.rows
	}
	if budget != nil {
		if err := budget.ReserveBundle(ctx, 3, 1, 2*uint64(unsafe.Sizeof(topo.Body{}))+256); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(columns)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(freeform.FreeformWork{}))+64); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, 1, 128); err != nil {
			return nil, err
		}
	}
	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySolid
	body.Doc = env.Doc
	work := freeform.NewFreeformWork()
	base := sp.OuterPrism()
	colDelta := make([]float64, len(columns))
	maxColDelta := 0.0
	var faces []*topo.Face
	// anchors are the faces on the body's own exterior: every region outer's
	// wall and the first and last slabs' caps (lumps). A closed shell's hole
	// lining band starts and ends at an interface, and its outer wall faces
	// the cavity, so it anchors nothing (Payload.ClosedShell).
	anchors := map[*topo.Face]struct{}{}
	for ci := range columns {
		col := &columns[ci]
		colDelta[ci] = plan.ColumnDelta(*col)
		maxColDelta = max(maxColDelta, colDelta[ci])
		first, last := sp.Slabs[col.Start], sp.Slabs[col.End]
		pp := base
		pp.Z0, pp.Z0Delta = first.Z0, first.Z0Delta
		pp.Z1, pp.Z1Delta = last.Z1, last.Z1Delta
		if colDelta[ci] > 0 {
			pp.SectionDelta = proofbound.AbsSumUpper(sp.SectionDelta, colDelta[ci])
		}
		var walls []*topo.Face
		var bottom, top []topo.CoEdge
		var perimeter proofbound.BoundedScalar
		var sourceWalks *momentinput.ProfileWalks
		var wallMap []*topo.Face
		if prepared != nil {
			sourceWalks = prepared.walks[ci]
		}
		if budget == nil && rectangles != nil && col.LoopIndex == 0 && len(col.Loop.Segments) == 4 {
			walls, bottom, top, perimeter, wallMap, err = prismrecord.BuildLoopSidesWithWitness(ctx,
				env, body, pp, plan.WallRoleLoop(*col), false, col.Loop, work, nil,
				denotation.LevelToken{}, denotation.LevelToken{}, false)
		} else if budget == nil {
			walls, bottom, top, perimeter, err = prismrecord.BuildLoopSidesAs(ctx, env, body, pp, plan.WallRoleLoop(*col), col.LoopIndex != 0, col.Loop, work, nil, denotation.LevelToken{}, denotation.LevelToken{}, false)
		} else {
			walls, bottom, top, perimeter, err = prismrecord.BuildLoopSidesAsReplay(ctx, env, body, pp, plan.WallRoleLoop(*col), col.LoopIndex != 0, col.Loop, work, sourceWalks, denotation.LevelToken{}, denotation.LevelToken{}, false, budget)
		}
		if err != nil {
			return nil, err
		}
		if witnesses != nil && len(col.Loop.Segments) == 1 && len(walls) == 1 {
			if circle, ok := col.Loop.Segments[0].(sectionrecord.CircleSeg); ok &&
				circle.TStart == 0 && circle.TEnd == 1 {
				*witnesses = append(*witnesses, CircleWallWitness{Column: ci,
					Start: col.Start, End: col.End, Region: col.Region, Loop: col.LoopIndex,
					Face: walls[0], Circle: circle, SectionDelta: sp.SectionDelta,
					Z0: first.Z0, Z1: last.Z1, Z0Delta: first.Z0Delta, Z1Delta: last.Z1Delta,
					Frame: sp.Frame, Xform: sp.Xform})
			}
		}
		if rectangles != nil && len(wallMap) == 4 && len(col.Loop.Segments) == 4 {
			var rec RectangleWallWitness
			rec.Column, rec.Start, rec.End, rec.Region, rec.Loop = ci, col.Start, col.End, col.Region, col.LoopIndex
			rec.SectionDelta, rec.Z0, rec.Z1, rec.Z0Delta, rec.Z1Delta, rec.Frame, rec.Xform =
				sp.SectionDelta, first.Z0, last.Z1, first.Z0Delta, last.Z1Delta, sp.Frame, sp.Xform
			valid := true
			for k, seg := range col.Loop.Segments {
				line, ok := seg.(sectionrecord.LineSeg)
				if !ok {
					valid = false
					break
				}
				rec.Lines[k], rec.Walls[k] = line, wallMap[k]
			}
			if valid {
				*rectangles = append(*rectangles, rec)
			}
		}
		if budget == nil {
			if err := plan.NameWalls(ctx, *col, walls, env.Ref); err != nil {
				return nil, err
			}
		} else {
			if err := nameOwnWallsReplay(ctx, *col, walls, budget); err != nil {
				return nil, err
			}
			if len(walls) != len(col.Loop.Segments) {
				return nil, fmt.Errorf("%w: blind replay merged source walls", decaderr.ErrUnsupported)
			}
			for segment, face := range walls {
				if err := log.record(ctx, ReplayBlindFaceEvent{Face: face, Kind: ReplayBlindWall,
					Column: ci, Segment: segment, Slab: -1, Region: col.Region,
					Loop: col.LoopIndex, Start: col.Start, End: col.End, PatchIndex: -1}); err != nil {
					return nil, err
				}
			}
		}
		col.Bottom, col.Top, col.Perimeter = bottom, top, perimeter
		if budget != nil {
			if err := reserveReplayAppend(ctx, len(faces), cap(faces), len(walls),
				uint64(unsafe.Sizeof((*topo.Face)(nil))), budget); err != nil {
				return nil, err
			}
		}
		faces = append(faces, walls...)
		if col.LoopIndex == 0 && !sp.liningCavity(col.Column) {
			for _, f := range walls {
				if budget == nil {
					anchors[f] = struct{}{}
				} else if err := replayAnchorInsert(ctx, anchors, f, budget); err != nil {
					return nil, err
				}
			}
		}
	}

	// One part per (slab, region).
	var parts []slabPart
	for k, slab := range sp.Slabs {
		for r := range slab.Regions {
			p, err := regionPart(ctx, sp, base, columns, colDelta, bySlab[k], k, r, work, budget, prepared)
			if err != nil {
				return nil, err
			}
			if budget != nil {
				if err := reserveReplayAppend(ctx, len(parts), cap(parts), 1,
					uint64(unsafe.Sizeof(slabPart{})), budget); err != nil {
					return nil, err
				}
			}
			parts = append(parts, p)
		}
	}

	// newPlane records denoted, the slab level's exact rational or nil, as the
	// face's CapRecord.ZDenoted, so a ToFace stop on a stack's cap or floor
	// reads it as a prism cap's (agent-docs/stacked-prism-design.md §2.1).
	newPlane := func(z, axial float64, denoted *big.Rat, flip bool, role string, area proofbound.BoundedScalar) (*topo.Face, error) {
		var frame r3.Frame
		var err error
		if budget == nil {
			frame, err = base.CapFrame(z, flip)
		} else {
			frame, err = base.CapFrameSignedReplay(ctx, z, flip, budget)
		}
		if err != nil {
			return nil, err
		}
		if budget != nil {
			if err := budget.ReserveBundle(ctx, 1, 4, 0); err != nil {
				return nil, err
			}
			for _, size := range []uint64{uint64(unsafe.Sizeof(topo.Face{})),
				uint64(unsafe.Sizeof(topo.CapRecord{})), uint64(unsafe.Sizeof(surfacegeom.Plane{})),
				uint64(unsafe.Sizeof(provenance.Ref{}))} {
				if err := budget.ReserveArray(ctx, 1, size); err != nil {
					return nil, err
				}
			}
		}
		return topobuild.NewFace(env.Alloc, topo.Face{
			Area:          area.Value,
			AreaBound:     area.Bound,
			AxialDelta:    axial,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: frame},
			CapRecord:     &topo.CapRecord{Frame: base.Frame, Xform: base.Xform, Z: z, ZDenoted: denoted},
			Origins:       []provenance.Ref{{Producer: env.Ref, Role: role}},
			Body:          body,
		}), nil
	}
	// One bottom cap per region of the first slab and one top cap per region
	// of the last, so CapStart(group) selects every lump's bottom face
	// (agent-docs/mirror-pattern-design.md §6.3).
	first, last := sp.Slabs[0], sp.Slabs[len(sp.Slabs)-1]
	var firstDenoted, lastDenoted *big.Rat
	if budget == nil {
		firstDenoted, lastDenoted = first.DenotedZ0(), last.DenotedZ1()
	} else {
		var ok bool
		firstDenoted, ok, err = prismplacement.ReplayDenotedLevel(ctx, first.Z0, first.Z0Delta, first.Z0Denoted, budget)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, fmt.Errorf("%w: blind start level has no bounded denotation", decaderr.ErrUnsupported)
		}
		lastDenoted, ok, err = prismplacement.ReplayDenotedLevel(ctx, last.Z1, last.Z1Delta, last.Z1Denoted, budget)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, fmt.Errorf("%w: blind end level has no bounded denotation", decaderr.ErrUnsupported)
		}
	}
	var planar []*topo.Face
	capArea := proofbound.BoundedScalar{}
	for _, p := range parts {
		for _, end := range []struct {
			slab    int
			z, d    float64
			denoted *big.Rat
			flip    bool
			top     bool
		}{
			{0, first.Z0, first.Z0Delta, firstDenoted, true, false},
			{len(sp.Slabs) - 1, last.Z1, last.Z1Delta, lastDenoted, false, true},
		} {
			if p.slab != end.slab {
				continue
			}
			f, err := newPlane(end.z, end.d, end.denoted, end.flip, plan.CapRole(end.slab, p.region, end.top), p.area)
			if err != nil {
				return nil, err
			}
			if witnesses != nil && len(sp.Slabs) == 1 {
				for i := range *witnesses {
					w := &(*witnesses)[i]
					if w.Region != p.region || w.Loop != 0 || w.Start != 0 || w.End != 0 {
						continue
					}
					if end.top {
						if w.CapEnd != nil {
							w.CapAmbiguous = true
						} else {
							w.CapEnd = f
						}
					} else if w.CapStart != nil {
						w.CapAmbiguous = true
					} else {
						w.CapStart = f
					}
				}
			}
			if rectangles != nil && len(sp.Slabs) == 1 {
				for i := range *rectangles {
					w := &(*rectangles)[i]
					if w.Region != p.region || w.Loop != 0 || w.Start != 0 || w.End != 0 {
						continue
					}
					if end.top {
						if w.CapEnd != nil {
							w.CapAmbiguous = true
						} else {
							w.CapEnd = f
						}
					} else if w.CapStart != nil {
						w.CapAmbiguous = true
					} else {
						w.CapStart = f
					}
				}
			}
			for _, e := range bySlab[end.slab] {
				if e.Region != p.region {
					continue
				}
				coedges := columns[e.Column].Bottom
				if end.top {
					coedges = columns[e.Column].Top
				}
				loop := topo.Loop{Outer: e.Loop == 0, CoEdges: coedges}
				if budget == nil {
					f.Loops = append(f.Loops, topobuild.NewLoop(env.Alloc, loop))
				} else if err := appendPlanarLoopReplay(ctx, env.Alloc, f, loop, budget); err != nil {
					return nil, err
				}
			}
			if log != nil {
				if err := log.record(ctx, ReplayBlindFaceEvent{Face: f, Kind: ReplayBlindCap,
					Column: -1, Segment: -1, Slab: end.slab, Region: p.region,
					Loop: -1, Start: -1, End: -1, PatchIndex: -1, Top: end.top}); err != nil {
					return nil, err
				}
			}
			if len(planar) == 0 {
				capArea = p.area
			} else if budget != nil {
				capArea, err = proofbound.BoundedAddReplay(ctx, capArea, p.area, budget)
				if err != nil {
					return nil, err
				}
			} else {
				capArea = proofbound.BoundedAdd(capArea, p.area)
			}
			if budget != nil {
				if err := reserveReplayAppend(ctx, len(planar), cap(planar), 1,
					uint64(unsafe.Sizeof((*topo.Face)(nil))), budget); err != nil {
					return nil, err
				}
			}
			planar = append(planar, f)
		}
	}
	capCount := len(planar)
	for _, f := range planar {
		if budget == nil {
			anchors[f] = struct{}{}
		} else if err := replayAnchorInsert(ctx, anchors, f, budget); err != nil {
			return nil, err
		}
	}
	for k := range sp.Interfaces {
		z, axial := sp.Slabs[k].Z1, sp.Slabs[k].Z1Delta
		// Falsify's I4 has refused two sides that denote different rationals,
		// and both sides hold one float and bound, so either side's record
		// names the interface's level.
		var denoted *big.Rat
		if budget == nil {
			denoted = sp.Slabs[k].DenotedZ1()
			if denoted == nil {
				denoted = sp.Slabs[k+1].DenotedZ0()
			}
		} else {
			var ok bool
			denoted, ok, err = prismplacement.ReplayDenotedLevel(ctx, sp.Slabs[k].Z1,
				sp.Slabs[k].Z1Delta, sp.Slabs[k].Z1Denoted, budget)
			if err != nil {
				return nil, err
			}
			if !ok {
				denoted, ok, err = prismplacement.ReplayDenotedLevel(ctx, sp.Slabs[k+1].Z0,
					sp.Slabs[k+1].Z0Delta, sp.Slabs[k+1].Z0Denoted, budget)
				if err != nil {
					return nil, err
				}
				if !ok {
					return nil, fmt.Errorf("%w: blind interface has no bounded denotation", decaderr.ErrUnsupported)
				}
			}
		}
		var patches []Patch
		if prepared == nil {
			patches, err = sp.InterfacePatches(columns, bySlab, k)
			if err != nil {
				return nil, err
			}
		} else {
			if k != 0 {
				return nil, fmt.Errorf("%w: blind replay has more than one interface", decaderr.ErrUnsupported)
			}
			patches = prepared.patches
		}
		for patchIndex, patch := range patches {
			var area proofbound.BoundedScalar
			if budget == nil {
				area, err = patchArea(ctx, sp.SectionDelta, colDelta, columns, patch)
			} else {
				area, err = patchAreaReplay(ctx, sp.SectionDelta, colDelta, columns, patch, budget)
			}
			if err != nil {
				return nil, err
			}
			f, err := newPlane(z, axial, denoted, !patch.Floor, plan.PatchRole(patch), area)
			if err != nil {
				return nil, err
			}
			for _, pl := range patch.Loops {
				coedges := columns[pl.Column].Bottom
				if pl.Top {
					coedges = columns[pl.Column].Top
				}
				loop := topo.Loop{Outer: pl.Outer, CoEdges: coedges}
				if budget == nil {
					f.Loops = append(f.Loops, topobuild.NewLoop(env.Alloc, loop))
				} else if err := appendPlanarLoopReplay(ctx, env.Alloc, f, loop, budget); err != nil {
					return nil, err
				}
			}
			if log != nil {
				if len(patch.Loops) != 1 {
					return nil, fmt.Errorf("%w: blind patch has multiple source columns", decaderr.ErrUnsupported)
				}
				column := patch.Loops[0].Column
				col := columns[column]
				if err := log.record(ctx, ReplayBlindFaceEvent{Face: f, Kind: ReplayBlindPatch,
					Column: column, Segment: -1, Slab: k, Region: col.Region,
					Loop: col.LoopIndex, Start: col.Start, End: col.End,
					PatchIndex: patchIndex, Floor: patch.Floor}); err != nil {
					return nil, err
				}
			}
			if budget != nil {
				if err := reserveReplayAppend(ctx, len(planar), cap(planar), 1,
					uint64(unsafe.Sizeof((*topo.Face)(nil))), budget); err != nil {
					return nil, err
				}
			}
			planar = append(planar, f)
		}
	}
	if budget == nil {
		err = topobuild.AttachFaceLoops(ctx, planar)
	} else {
		err = topobuild.AttachFaceLoopsReplay(ctx, planar, budget)
	}
	if err != nil {
		return nil, err
	}
	if budget != nil {
		if err := reserveReplayAppend(ctx, len(faces), cap(faces), len(planar),
			uint64(unsafe.Sizeof((*topo.Face)(nil))), budget); err != nil {
			return nil, err
		}
	}
	faces = append(faces, planar...)
	var bodyLumps []*topo.Lump
	if budget == nil {
		bodyLumps, err = lumps(env.Alloc, plan.Order(faces), anchors)
	} else {
		bodyLumps, err = lumpsReplay(ctx, env.Alloc, plan.Order(faces), anchors, budget)
	}
	body.Lumps = bodyLumps
	if err != nil {
		return nil, err
	}

	volume := proofbound.BoundedScalar{}
	area := capArea
	var moment [3]proofbound.BoundedScalar
	add := func(a, b proofbound.BoundedScalar) (proofbound.BoundedScalar, error) {
		if budget == nil {
			return proofbound.BoundedAdd(a, b), nil
		}
		return proofbound.BoundedAddReplay(ctx, a, b, budget)
	}
	sub := func(a, b proofbound.BoundedScalar) (proofbound.BoundedScalar, error) {
		if budget == nil {
			return proofbound.BoundedSub(a, b), nil
		}
		return proofbound.BoundedSubReplay(ctx, a, b, budget)
	}
	mul := func(a, b proofbound.BoundedScalar) (proofbound.BoundedScalar, error) {
		if budget == nil {
			return proofbound.BoundedMul(a, b), nil
		}
		return proofbound.BoundedMulReplay(ctx, a, b, budget)
	}
	div := func(a, b proofbound.BoundedScalar) (proofbound.BoundedScalar, error) {
		if budget == nil {
			return proofbound.BoundedDiv(a, b), nil
		}
		return proofbound.BoundedDivReplay(ctx, a, b, budget)
	}
	for _, p := range parts {
		slab := sp.Slabs[p.slab]
		height, err := sub(proofbound.MeasuredScalar(slab.Z1, slab.Z1Delta), proofbound.MeasuredScalar(slab.Z0, slab.Z0Delta))
		if err != nil {
			return nil, err
		}
		mass, err := mul(p.area, height)
		if err != nil {
			return nil, err
		}
		volume, err = add(volume, mass)
		if err != nil {
			return nil, err
		}
		point := p.centroid
		pointBound := p.centroidBound
		for i, value := range []float64{point.X, point.Y, point.Z} {
			term, err := mul(mass, proofbound.MeasuredScalar(value, pointBound))
			if err != nil {
				return nil, err
			}
			moment[i], err = add(moment[i], term)
			if err != nil {
				return nil, err
			}
		}
	}
	for _, col := range columns {
		a, b := sp.Slabs[col.Start], sp.Slabs[col.End]
		height, err := sub(proofbound.MeasuredScalar(b.Z1, b.Z1Delta), proofbound.MeasuredScalar(a.Z0, a.Z0Delta))
		if err != nil {
			return nil, err
		}
		term, err := mul(col.Perimeter, height)
		if err != nil {
			return nil, err
		}
		area, err = add(area, term)
		if err != nil {
			return nil, err
		}
	}
	for _, face := range planar[capCount:] {
		area, err = add(area, proofbound.MeasuredScalar(face.Area, face.AreaBound))
		if err != nil {
			return nil, err
		}
	}
	if volume.Value <= 0 {
		return nil, fmt.Errorf(`%w: a stacked prism encloses no volume`, decaderr.ErrDegenerate)
	}
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume.Value), Exactness: measurement.ExactnessOf(volume.Bound),
		Bound: units.CubicMillimeters(volume.Bound)}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(area.Value), Exactness: measurement.ExactnessOf(area.Bound),
		Bound: units.SquareMillimeters(area.Bound)}
	x, err := div(moment[0], volume)
	if err != nil {
		return nil, err
	}
	y, err := div(moment[1], volume)
	if err != nil {
		return nil, err
	}
	z, err := div(moment[2], volume)
	if err != nil {
		return nil, err
	}
	centroid := r3.Vec{X: x.Value, Y: y.Value, Z: z.Value}
	centroidBound := proofbound.Radius3D(max(x.Bound, y.Bound, z.Bound))
	if sp.SectionDelta > 0 || maxColDelta > 0 {
		if budget != nil {
			if maxColDelta > 0 {
				return nil, fmt.Errorf("%w: blind replay has displaced columns", decaderr.ErrUnsupported)
			}
			coordUpper := 0.0
			for _, part := range parts {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, err
				}
				coordUpper = max(coordUpper, part.coordUpper)
			}
			geometryBound, err := base.CentroidEnvelopeSignedReplay(ctx, coordUpper, centroid, budget)
			if err != nil {
				return nil, err
			}
			if err := budget.SpendWork(ctx, uint64(len(sp.Slabs))+1); err != nil {
				return nil, err
			}
			centroidBound = max(centroidBound, proofbound.AbsSumUpper(geometryBound,
				sp.SectionDelta, sp.AxialDelta()))
		} else {
			runs := sp.OuterRuns()
			// Every material point lies in one run's prism, so the largest run
			// envelope caps the centroid's distance from the held point.
			geometryBound := 0.0
			for _, run := range runs {
				runBound, err := run.CentroidGeometryBound(run.Profile, centroid, work, nil)
				if err != nil {
					return nil, err
				}
				geometryBound = max(geometryBound, runBound)
			}
			if sp.SectionDelta > 0 {
				centroidBound = max(centroidBound, proofbound.AbsSumUpper(geometryBound,
					sp.SectionDelta, sp.AxialDelta()))
			} else {
				// The column displacements are already charged into every part's
				// moments, so the envelope is a ceiling on that sound answer. It is
				// read off the recorded runs; the denoted body reaches up to the
				// largest column displacement farther in the plane and each level's
				// axial one along the normal, carried through the rigid map's 3·L1
				// factor, as the cup's own envelope is.
				geometryBound = proofbound.AbsSumUpper(geometryBound, proofbound.ProductUpper(3, proofbound.AbsSumUpper(
					proofbound.ProductUpper(proofbound.AbsSumUpper(VecL1(base.Frame.U()), VecL1(base.Frame.V())), maxColDelta),
					proofbound.ProductUpper(VecL1(base.Frame.N()), sp.AxialDelta()),
				)))
				centroidBound = math.Min(centroidBound, geometryBound)
			}
		}
	}
	body.Centroid = measurement.VecMeasurement{Value: centroid, Exactness: measurement.ExactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}
	outerDelta := 0.0
	for _, entries := range bySlab {
		for _, e := range entries {
			if e.Loop == 0 && (e.Region == 0 || sp.IsGroup()) {
				outerDelta = max(outerDelta, colDelta[e.Column])
			}
		}
	}
	var bounds measurement.Box
	if budget != nil && prepared != nil && prepared.sourceBody != nil {
		var ok bool
		bounds, ok, err = replayBlindSourceBounds(ctx, sp, prepared.sourceBody, prepared, prepared.cert, budget)
		if err == nil && !ok {
			err = fmt.Errorf("%w: blind result box has no authenticated source", decaderr.ErrUnsupported)
		}
	} else {
		bounds, err = boundsContext(ctx, sp, outerDelta, work)
	}
	if err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if budget == nil {
		err = prismrecord.ChargeMap(body, sp.Frame, sp.Xform)
	} else {
		err = prismrecord.ChargeMapSignedReplay(ctx, body, sp.Frame, sp.Xform, budget)
	}
	if err != nil {
		return nil, err
	}
	if budget == nil {
		err = topobuild.ValidateAnalyticMeasurements(body)
	} else {
		err = topobuild.ValidateAnalyticMeasurementsReplay(ctx, body, budget)
	}
	if err != nil {
		return nil, err
	}
	sp.Upstream = nil
	if budget != nil {
		sealed, err := body.SealPrismBuildFacesReplay(ctx, faces, budget)
		if err != nil {
			return nil, err
		}
		if !sealed {
			return nil, fmt.Errorf("%w: blind result faces have no bounded source seal", decaderr.ErrUnsupported)
		}
		if len(sp.Slabs) != 2 || len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
			return nil, fmt.Errorf("%w: blind result has no bounded section seal", decaderr.ErrUnsupported)
		}
		if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(momentinput.Profile{}))); err != nil {
			return nil, err
		}
		profiles := []momentinput.Profile{sp.Slabs[0].Regions[0], sp.Slabs[1].Regions[0]}
		sealed, err = body.SealPrismBuildPayloadReplay(ctx, sp.Frame, sp.Xform,
			sp.Slabs[0].Z0, sp.Slabs[1].Z1, sp.Slabs[0].Z0Delta, sp.Slabs[1].Z1Delta,
			sp.SectionDelta, sp.Slabs[0].Z0Denoted, sp.Slabs[1].Z1Denoted, profiles, budget)
		if err != nil {
			return nil, err
		}
		if !sealed || !body.HasStrictPrismBuildSeal() {
			return nil, fmt.Errorf("%w: blind result section has no bounded source seal", decaderr.ErrUnsupported)
		}
	} else if sp.IsGroup() {
		slab := sp.Slabs[0]
		witnessBudget := proofbound.NewReplayTrialBudget()
		if sealed, _ := body.SealPrismBuildFacesReplay(ctx, faces, witnessBudget); sealed {
			// This optional seal only enables a later bounded replay; ordinary
			// group construction does not depend on the witness.
			_, _ = body.SealPrismBuildPayloadReplay(ctx, sp.Frame, sp.Xform,
				slab.Z0, slab.Z1, slab.Z0Delta, slab.Z1Delta, sp.SectionDelta,
				slab.Z0Denoted, slab.Z1Denoted, slab.Regions, witnessBudget)
		}
	}
	body.Payload = sp
	if log != nil {
		sealed, err := log.Seal(ctx, body)
		if err != nil || !sealed {
			if err == nil {
				err = fmt.Errorf("%w: blind replay face log does not cover the built body", decaderr.ErrUnsupported)
			}
			return nil, err
		}
	}
	return body, nil
}
