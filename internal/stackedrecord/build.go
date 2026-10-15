package stackedrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a stacked prism's body from its Payload: one wall column
// per recorded loop run, the caps and interface patches, and the
// measurements the finished body publishes (docs/stacked-prism-design.md).

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

// slabPart is one (slab, region) prism of a stacked body: its bounded
// section area and its centroid lifted to the slab's bounded midpoint.
type slabPart struct {
	slab, region  int
	area          proofbound.BoundedScalar
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
	entries []SlabLoop, k, r int, work *freeform.FreeformWork) (slabPart, error) {
	slab := sp.Slabs[k]
	region := slab.Regions[r]
	ig, err := region.EvaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, work)
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
		perimeter = proofbound.BoundedAdd(perimeter, col.Perimeter)
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
	out := slabPart{slab: k, region: r, area: proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(ig.AreaBound,
		proofbound.SectionDisplacementArea(sp.SectionDelta, walks, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))))}
	u := proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
	v := proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	u.Bound = proofbound.AbsSumUpper(u.Bound, sp.SectionDelta)
	v.Bound = proofbound.AbsSumUpper(v.Bound, sp.SectionDelta)
	mid := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.MeasuredScalar(slab.Z0, slab.Z0Delta),
		proofbound.MeasuredScalar(slab.Z1, slab.Z1Delta)), proofbound.ExactScalar(2))
	out.centroid = base.Point(u.Value, v.Value, mid.Value)
	out.centroidBound = base.PointBound(u, v, mid)
	return out, nil
}

// Build evaluates sp into a new body allocated through env.Alloc, under
// plan's naming and column displacements (§3, §4). It mints identities
// through env.Doc and roles under env.Ref, and never commits: the caller maps
// the result to its handle and commits it.
func Build(ctx context.Context, env topo.Env, sp Payload, plan Plan) (*topo.Body, error) {
	if !sp.OpposedSectionExact() {
		return nil, fmt.Errorf(`%w: opposed holes at a stacked interface carry section displacement`, decaderr.ErrUnsupported)
	}
	if err := Falsify(ctx, sp.Record()); err != nil {
		return nil, err
	}
	columns, bySlab, err := sp.Columns()
	if err != nil {
		return nil, err
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
	// wall and the first and last slabs' caps (lumps).
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
		walls, bottom, top, perimeter, err := prismrecord.BuildLoopSidesAs(ctx, env, body, pp, plan.WallRoleLoop(*col), col.LoopIndex != 0, col.Loop, work, nil, denotation.LevelToken{}, denotation.LevelToken{}, false)
		if err != nil {
			return nil, err
		}
		if err := plan.NameWalls(ctx, *col, walls, env.Ref); err != nil {
			return nil, err
		}
		col.Bottom, col.Top, col.Perimeter = bottom, top, perimeter
		faces = append(faces, walls...)
		if col.LoopIndex == 0 {
			for _, f := range walls {
				anchors[f] = struct{}{}
			}
		}
	}

	// One part per (slab, region).
	var parts []slabPart
	for k, slab := range sp.Slabs {
		for r := range slab.Regions {
			p, err := regionPart(ctx, sp, base, columns, colDelta, bySlab[k], k, r, work)
			if err != nil {
				return nil, err
			}
			parts = append(parts, p)
		}
	}

	newPlane := func(z, axial float64, flip bool, role string, area proofbound.BoundedScalar) (*topo.Face, error) {
		frame, err := base.CapFrame(z, flip)
		if err != nil {
			return nil, err
		}
		return topobuild.NewFace(env.Alloc, topo.Face{
			Area:          area.Value,
			AreaBound:     area.Bound,
			AxialDelta:    axial,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: frame},
			CapRecord:     &topo.CapRecord{Frame: base.Frame, Xform: base.Xform, Z: z},
			Origins:       []provenance.Ref{{Producer: env.Ref, Role: role}},
			Body:          body,
		}), nil
	}
	// One bottom cap per region of the first slab and one top cap per region
	// of the last, so CapStart(group) selects every lump's bottom face
	// (docs/mirror-pattern-design.md §6.3).
	first, last := sp.Slabs[0], sp.Slabs[len(sp.Slabs)-1]
	var planar []*topo.Face
	capArea := proofbound.BoundedScalar{}
	for _, p := range parts {
		for _, end := range []struct {
			slab int
			z, d float64
			flip bool
			top  bool
		}{{0, first.Z0, first.Z0Delta, true, false}, {len(sp.Slabs) - 1, last.Z1, last.Z1Delta, false, true}} {
			if p.slab != end.slab {
				continue
			}
			f, err := newPlane(end.z, end.d, end.flip, plan.CapRole(end.slab, p.region, end.top), p.area)
			if err != nil {
				return nil, err
			}
			for _, e := range bySlab[end.slab] {
				if e.Region != p.region {
					continue
				}
				coedges := columns[e.Column].Bottom
				if end.top {
					coedges = columns[e.Column].Top
				}
				f.Loops = append(f.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: e.Loop == 0, CoEdges: coedges}))
			}
			if len(planar) == 0 {
				capArea = p.area
			} else {
				capArea = proofbound.BoundedAdd(capArea, p.area)
			}
			planar = append(planar, f)
		}
	}
	capCount := len(planar)
	for _, f := range planar {
		anchors[f] = struct{}{}
	}
	for k := range sp.Interfaces {
		z, axial := sp.Slabs[k].Z1, sp.Slabs[k].Z1Delta
		patches, err := sp.InterfacePatches(columns, bySlab, k)
		if err != nil {
			return nil, err
		}
		for _, patch := range patches {
			area, err := patchArea(ctx, sp.SectionDelta, colDelta, columns, patch)
			if err != nil {
				return nil, err
			}
			f, err := newPlane(z, axial, !patch.Floor, plan.PatchRole(patch), area)
			if err != nil {
				return nil, err
			}
			for _, pl := range patch.Loops {
				coedges := columns[pl.Column].Bottom
				if pl.Top {
					coedges = columns[pl.Column].Top
				}
				f.Loops = append(f.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: pl.Outer, CoEdges: coedges}))
			}
			planar = append(planar, f)
		}
	}
	if err := topobuild.AttachFaceLoops(ctx, planar); err != nil {
		return nil, err
	}
	faces = append(faces, planar...)
	var bodyLumps []*topo.Lump
	bodyLumps, err = lumps(env.Alloc, plan.Order(faces), anchors)
	body.Lumps = bodyLumps
	if err != nil {
		return nil, err
	}

	volume := proofbound.BoundedScalar{}
	area := capArea
	var moment [3]proofbound.BoundedScalar
	for _, p := range parts {
		slab := sp.Slabs[p.slab]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(slab.Z1, slab.Z1Delta), proofbound.MeasuredScalar(slab.Z0, slab.Z0Delta))
		mass := proofbound.BoundedMul(p.area, height)
		volume = proofbound.BoundedAdd(volume, mass)
		point := p.centroid
		pointBound := p.centroidBound
		for i, value := range []float64{point.X, point.Y, point.Z} {
			moment[i] = proofbound.BoundedAdd(moment[i], proofbound.BoundedMul(mass, proofbound.MeasuredScalar(value, pointBound)))
		}
	}
	for _, col := range columns {
		a, b := sp.Slabs[col.Start], sp.Slabs[col.End]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(b.Z1, b.Z1Delta), proofbound.MeasuredScalar(a.Z0, a.Z0Delta))
		area = proofbound.BoundedAdd(area, proofbound.BoundedMul(col.Perimeter, height))
	}
	for _, face := range planar[capCount:] {
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(face.Area, face.AreaBound))
	}
	if volume.Value <= 0 {
		return nil, fmt.Errorf(`%w: a stacked prism encloses no volume`, decaderr.ErrDegenerate)
	}
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume.Value), Exactness: measurement.ExactnessOf(volume.Bound),
		Bound: units.CubicMillimeters(volume.Bound)}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(area.Value), Exactness: measurement.ExactnessOf(area.Bound),
		Bound: units.SquareMillimeters(area.Bound)}
	x, y, z := proofbound.BoundedDiv(moment[0], volume), proofbound.BoundedDiv(moment[1], volume), proofbound.BoundedDiv(moment[2], volume)
	centroid := r3.Vec{X: x.Value, Y: y.Value, Z: z.Value}
	centroidBound := proofbound.Radius3D(max(x.Bound, y.Bound, z.Bound))
	if sp.SectionDelta > 0 || maxColDelta > 0 {
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
	body.Centroid = measurement.VecMeasurement{Value: centroid, Exactness: measurement.ExactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}
	outerDelta := 0.0
	for _, entries := range bySlab {
		for _, e := range entries {
			if e.Loop == 0 && (e.Region == 0 || sp.IsGroup()) {
				outerDelta = max(outerDelta, colDelta[e.Column])
			}
		}
	}
	bounds, err := boundsContext(ctx, sp, outerDelta, work)
	if err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := prismrecord.ChargeMap(body, sp.Frame, sp.Xform); err != nil {
		return nil, err
	}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = sp
	return body, nil
}
