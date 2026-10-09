package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A slab is one constant section over an axial interval. Its one region is
// kept in a slice so the record can later admit disjoint sections per slab.
type prismSlab struct {
	regions          []profileRecord
	z0, z1           float64
	z0Delta, z1Delta float64
}

type prismSlabInterface struct {
	lowerExposed []profileRecord
	upperExposed []profileRecord
}

type stackedPrismPayload struct {
	slabs        []prismSlab
	interfaces   []prismSlabInterface
	frame        r3.Frame
	xform        r3.Transform
	sectionDelta float64
}

func (sp stackedPrismPayload) transform() r3.Transform { return sp.xform }

func (sp stackedPrismPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	sp.xform = composed
	return evalStackedContext(ctx, d, ref, sp)
}

func (sp stackedPrismPayload) axialDelta() float64 {
	var delta float64
	for _, slab := range sp.slabs {
		delta = max(delta, slab.z0Delta, slab.z1Delta)
	}
	return delta
}

func (sp stackedPrismPayload) outerPrism() prismPayload {
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	return prismPayload{
		profile: profileRecord{Outer: first.regions[0].Outer},
		frame:   sp.frame, xform: sp.xform, sectionDelta: sp.sectionDelta,
		z0: first.z0, z0Delta: first.z0Delta,
		z1: last.z1, z1Delta: last.z1Delta,
	}
}

// outerRuns is one outer-only prism per maximal run of consecutive slabs that
// carry one outer loop record, over that run's own interval. A cut-built stack
// is one run, the outer-only prism on the full interval. A union-built stack
// has one run per distinct outer, and every run's prism is material of the
// body, since a union-built slab region is hole-free (§2.2's union reading).
// The union of the runs' prisms is the body's outer envelope, which is what
// Bounds, extentAlong, the centroid's geometric cap and the tolerance gate's
// witnesses read.
func (sp stackedPrismPayload) outerRuns() []prismPayload {
	base := sp.outerPrism()
	planned := stackedrecord.OuterRuns(stackedRecordOf(sp).Slabs)
	runs := make([]prismPayload, len(planned))
	for i, slab := range planned {
		run := base
		run.profile = slab.Regions[0]
		run.z0, run.z0Delta = slab.Z0, slab.Z0Delta
		run.z1, run.z1Delta = slab.Z1, slab.Z1Delta
		runs[i] = run
	}
	return runs
}

func (sp stackedPrismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	runs := sp.outerRuns()
	var lo, hi, bound float64
	for i, run := range runs {
		rlo, rhi, rbound, err := run.extentAlong(g)
		if err != nil {
			return 0, 0, 0, err
		}
		if i == 0 {
			lo, hi, bound = rlo, rhi, rbound
			continue
		}
		lo, hi, bound = min(lo, rlo), max(hi, rhi), max(bound, rbound)
	}
	return lo, hi, bound, nil
}

// stackedBoundsContext is the union of every outer run's box. Each run's box
// bounds its own extremes, so the union's extreme along each axis is one
// run's, and the largest run bound covers it. outerDelta is the largest
// column displacement any run's outer loop carries beyond sectionDelta
// (stackedPlan.columnDelta), charged to every run as its section displacement.
func stackedBoundsContext(ctx context.Context, sp stackedPrismPayload, outerDelta float64, work *freeform.FreeformWork) (Box, error) {
	runs := sp.outerRuns()
	var out Box
	bound := 0.0
	for i, run := range runs {
		if outerDelta > 0 {
			run.sectionDelta = proofbound.AbsSumUpper(run.sectionDelta, outerDelta)
		}
		box, err := prismBoundsContext(ctx, run, work, nil)
		if err != nil {
			return Box{}, err
		}
		bound = max(bound, box.Bound.Base())
		if i == 0 {
			out = box
			continue
		}
		out.Min = r3.NewVec(min(out.Min.X, box.Min.X), min(out.Min.Y, box.Min.Y), min(out.Min.Z, box.Min.Z))
		out.Max = r3.NewVec(max(out.Max.X, box.Max.X), max(out.Max.Y, box.Max.Y), max(out.Max.Z, box.Max.Z))
	}
	out.Exactness = exactnessOf(bound)
	out.Bound = units.Millimeters(bound)
	return out, nil
}

// isGroup reports whether the payload is a prism group
// (docs/mirror-pattern-design.md §6.3): one slab holding two or more regions.
func (sp stackedPrismPayload) isGroup() bool {
	return len(sp.slabs) == 1 && len(sp.slabs[0].regions) >= 2
}

// stackedRecordOf adapts the root payload's slab fields for the record audit.
func stackedRecordOf(sp stackedPrismPayload) stackedrecord.Record {
	out := stackedrecord.Record{
		Slabs:      make([]stackedrecord.Slab, len(sp.slabs)),
		Interfaces: make([]stackedrecord.Interface, len(sp.interfaces)),
	}
	for i, slab := range sp.slabs {
		out.Slabs[i] = stackedSlabRecord(slab)
	}
	for i, face := range sp.interfaces {
		out.Interfaces[i] = stackedrecord.Interface{
			LowerExposed: face.lowerExposed, UpperExposed: face.upperExposed,
		}
	}
	return out
}

func stackedSlabRecord(slab prismSlab) stackedrecord.Slab {
	return stackedrecord.Slab{
		Regions: slab.regions, Z0: slab.z0, Z1: slab.z1,
		Z0Delta: slab.z0Delta, Z1Delta: slab.z1Delta,
	}
}

// stackedInterfaces derives interface records and adapts them for the payload.
func stackedInterfaces(ctx context.Context, slabs []prismSlab, prior []prismSlabInterface) ([]prismSlabInterface, error) {
	record := stackedRecordOf(stackedPrismPayload{slabs: slabs, interfaces: prior})
	derived, err := stackedrecord.Derive(ctx, record.Slabs, record.Interfaces)
	if err != nil {
		return nil, err
	}
	out := make([]prismSlabInterface, len(derived))
	for i, face := range derived {
		out[i] = prismSlabInterface{lowerExposed: face.LowerExposed, upperExposed: face.UpperExposed}
	}
	return out, nil
}

// stackedInterfacePatches passes the recorded columns to the shared patch plan.
func stackedInterfacePatches(sp stackedPrismPayload, columns []stackedColumn, bySlab [][]stackedrecord.SlabLoop, k int) ([]stackedrecord.Patch, error) {
	plannedColumns := make([]stackedrecord.Column, len(columns))
	for i, col := range columns {
		plannedColumns[i] = col.Column
	}
	return stackedrecord.InterfacePatches(stackedRecordOf(sp), plannedColumns, bySlab, k)
}

// stackedPatchArea is a patch record's area: its outer loop's enclosed area
// less each hole's, with sectionDisplacementArea over every loop's walks and
// proven perimeter folded into the bound (§4), and each loop's own column
// displacement (colDelta) over that loop alone.
func stackedPatchArea(ctx context.Context, sectionDelta float64, colDelta []float64, columns []stackedColumn, patch stackedrecord.Patch) (proofbound.BoundedScalar, error) {
	area, err := loopEnclosedAreaContext(ctx, patch.Record.Outer)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	for _, hole := range patch.Record.Holes {
		holeArea, err := loopEnclosedAreaContext(ctx, hole)
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
		loopPerimeter := proofbound.AbsSumUpper(col.perimeter.Value, col.perimeter.Bound)
		perimeter = proofbound.AbsSumUpper(perimeter, loopPerimeter)
		if delta := colDelta[pl.Column]; delta > 0 {
			area.Bound = proofbound.AbsSumUpper(area.Bound,
				proofbound.SectionDisplacementArea(delta, len(col.Loop.Segments), loopPerimeter))
		}
	}
	area.Bound = proofbound.AbsSumUpper(area.Bound, proofbound.SectionDisplacementArea(sectionDelta, walks, perimeter))
	return area, nil
}

type stackedColumn struct {
	stackedrecord.Column
	bottom    []coedge
	top       []coedge
	perimeter proofbound.BoundedScalar
}

// stackedColumns adapts recorded wall columns for the topology build.
func stackedColumns(sp stackedPrismPayload) ([]stackedColumn, [][]stackedrecord.SlabLoop, error) {
	planned, loops, err := stackedrecord.Columns(stackedRecordOf(sp))
	if err != nil {
		return nil, nil, err
	}
	columns := make([]stackedColumn, len(planned))
	for i, col := range planned {
		columns[i] = stackedColumn{Column: col}
	}
	return columns, loops, nil
}

// stackedPlan names a stacked body's faces and states the displacement each
// wall column's own loop carries beside the payload-wide sectionDelta. The
// payload's own plan (stackedOwnPlan) mints §3's roles and charges no column
// displacement. A cup (shell_cup.go) builds through its own plan: it keeps the
// cup's public roles and charges its offset loops their proven offset
// displacement, loop by loop.
type stackedPlan interface {
	// wallRoleLoop is the loop index buildLoopSidesAs mints into column
	// col's side(i,j) roles, and nameWalls rewrites the roles minted on that
	// column's walls.
	wallRoleLoop(col stackedColumn) int
	nameWalls(ctx context.Context, col stackedColumn, faces []*Face, ref producerID) error
	// capRole names the bottom (top false) or top cap of one region of the
	// first or the last slab.
	capRole(slab, region int, top bool) string
	// patchRole names an exposed interface patch.
	patchRole(patch stackedrecord.Patch) string
	// columnDelta is the section displacement column col's loop carries
	// beyond the payload's sectionDelta.
	columnDelta(col stackedColumn) float64
	// order is the body's face order, from the build order.
	order(faces []*Face) []*Face
}

// stackedOwnPlan is the stacked payload's own naming (§3).
type stackedOwnPlan struct{}

func (stackedOwnPlan) wallRoleLoop(col stackedColumn) int { return col.LoopIndex }

func (stackedOwnPlan) nameWalls(_ context.Context, col stackedColumn, faces []*Face, _ producerID) error {
	for _, face := range faces {
		for i := range face.origins {
			face.origins[i].Role = fmt.Sprintf("slab(%d).region(%d).%s", col.Start, col.Region, face.origins[i].Role)
		}
	}
	return nil
}

func (stackedOwnPlan) capRole(_, _ int, top bool) string {
	if top {
		return roleCapEnd
	}
	return roleCapStart
}

func (stackedOwnPlan) patchRole(patch stackedrecord.Patch) string { return patch.Role }

func (stackedOwnPlan) columnDelta(stackedColumn) float64 { return 0 }

func (stackedOwnPlan) order(faces []*Face) []*Face { return faces }

func evalStackedContext(ctx context.Context, d *Document, ref producerID, sp stackedPrismPayload) (*Body, error) {
	return evalStackedPlanContext(ctx, d, ref, sp, stackedOwnPlan{})
}

// stackedEnclosesCavity reports whether a stack encloses a cavity: a hole
// column that starts and ends at interfaces, so an exposed floor closes it
// below and an exposed ceiling above (§2.4's closed shell).
func stackedEnclosesCavity(sp stackedPrismPayload) (bool, error) {
	columns, _, err := stackedColumns(sp)
	if err != nil {
		return false, err
	}
	for _, col := range columns {
		if col.LoopIndex != 0 && col.Start > 0 && col.End < len(sp.slabs)-1 {
			return true, nil
		}
	}
	return false, nil
}

// stackedLumps groups a stacked body's faces into lumps and shells. Each
// connected face set holding an anchor — a region outer's wall or an end cap,
// the faces the body's exterior is made of — is one lump's outer shell. A
// connected set holding none bounds a cavity: its walls are hole loops and its
// planar faces are interface patches, all facing into it, so it is a void
// shell (§2.4's closed shell). The cavity lies in the one outer shell's
// material; a body with several outer shells and any cavity would need a
// nesting proof this build does not run, and is ErrUnsupported.
func stackedLumps(faces []*Face, anchors map[*Face]struct{}) ([]*Lump, error) {
	var outer []*Lump
	var voids []*Shell
	for _, group := range splitConnectedFaces(faces) {
		anchored := false
		for _, f := range group {
			if _, ok := anchors[f]; ok {
				anchored = true
				break
			}
		}
		if !anchored {
			voids = append(voids, &Shell{faces: group, open: shellIsOpen(group), void: true})
			continue
		}
		outer = append(outer, &Lump{shells: []*Shell{{faces: group, open: shellIsOpen(group)}}})
	}
	switch {
	case len(voids) == 0:
		return outer, nil
	case len(outer) != 1:
		return nil, fmt.Errorf(`%w: a stacked body with %d outer shells and %d cavities needs a nesting proof this evaluator does not run`,
			ErrUnsupported, len(outer), len(voids))
	}
	outer[0].shells = append(outer[0].shells, voids...)
	return outer, nil
}

// stackedPart is one (slab, region) prism of a stacked body: its bounded
// section area and its centroid lifted to the slab's bounded midpoint.
type stackedPart struct {
	slab, region  int
	area          proofbound.BoundedScalar
	centroid      r3.Vec
	centroidBound float64
}

// stackedRegionPart integrates one slab region. The region's area carries
// sectionDisplacementArea for the payload's sectionDelta over all its walks,
// and each loop's own column displacement over that loop alone; a region with
// a displaced loop also widens its first moments by that displacement area
// times the farthest coordinate the displaced boundary can reach, as
// displacedRegionIntegrals does, so the centroid quotient covers it.
func stackedRegionPart(ctx context.Context, sp stackedPrismPayload, base prismPayload, columns []stackedColumn, colDelta []float64,
	entries []stackedrecord.SlabLoop, k, r int, work *freeform.FreeformWork) (stackedPart, error) {
	slab := sp.slabs[k]
	region := slab.regions[r]
	ig, err := region.EvaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, work)
	if err != nil {
		return stackedPart{}, err
	}
	if ig.Area <= 0 {
		return stackedPart{}, fmt.Errorf(`%w: slab %d region %d has no material area`, ErrDegenerate, k, r)
	}
	perimeter := proofbound.BoundedScalar{}
	walks := 0
	loopArea, loopReach := 0.0, 0.0
	for _, e := range entries {
		if e.Region != r {
			continue
		}
		col := columns[e.Column]
		perimeter = proofbound.BoundedAdd(perimeter, col.perimeter)
		walks += len(col.Loop.Segments)
		if delta := colDelta[e.Column]; delta > 0 {
			loopArea = proofbound.AbsSumUpper(loopArea, proofbound.SectionDisplacementArea(delta, len(col.Loop.Segments),
				proofbound.AbsSumUpper(col.perimeter.Value, col.perimeter.Bound)))
			loopReach = max(loopReach, delta)
		}
	}
	if loopArea > 0 {
		coord, err := momentinput.CoordinateEnvelope(region, work, nil)
		if err != nil {
			return stackedPart{}, err
		}
		moment := proofbound.ProductUpper(loopArea, proofbound.AbsSumUpper(coord, loopReach))
		ig.AreaBound = proofbound.AbsSumUpper(ig.AreaBound, loopArea)
		ig.MuBound = proofbound.AbsSumUpper(ig.MuBound, moment)
		ig.MvBound = proofbound.AbsSumUpper(ig.MvBound, moment)
	}
	part := stackedPart{slab: k, region: r, area: proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(ig.AreaBound,
		proofbound.SectionDisplacementArea(sp.sectionDelta, walks, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))))}
	u := proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
	v := proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	u.Bound = proofbound.AbsSumUpper(u.Bound, sp.sectionDelta)
	v.Bound = proofbound.AbsSumUpper(v.Bound, sp.sectionDelta)
	mid := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.MeasuredScalar(slab.z0, slab.z0Delta),
		proofbound.MeasuredScalar(slab.z1, slab.z1Delta)), proofbound.ExactScalar(2))
	part.centroid = base.point(u.Value, v.Value, mid.Value)
	part.centroidBound = prismPointBound(base, u, v, mid)
	return part, nil
}

// evalStackedPlanContext builds a stacked body under plan's naming and column
// displacements (§3, §4).
func evalStackedPlanContext(ctx context.Context, d *Document, ref producerID, sp stackedPrismPayload, plan stackedPlan) (*Body, error) {
	if err := stackedrecord.Falsify(ctx, stackedRecordOf(sp)); err != nil {
		return nil, err
	}
	columns, bySlab, err := stackedColumns(sp)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	work := freeform.NewFreeformWork()
	base := sp.outerPrism()
	colDelta := make([]float64, len(columns))
	maxColDelta := 0.0
	var faces []*Face
	// anchors are the faces on the body's own exterior: every region outer's
	// wall and the first and last slabs' caps (stackedLumps).
	anchors := map[*Face]struct{}{}
	for ci := range columns {
		col := &columns[ci]
		colDelta[ci] = plan.columnDelta(*col)
		maxColDelta = max(maxColDelta, colDelta[ci])
		first, last := sp.slabs[col.Start], sp.slabs[col.End]
		pp := base
		pp.z0, pp.z0Delta = first.z0, first.z0Delta
		pp.z1, pp.z1Delta = last.z1, last.z1Delta
		if colDelta[ci] > 0 {
			pp.sectionDelta = proofbound.AbsSumUpper(sp.sectionDelta, colDelta[ci])
		}
		wallFaces, bottom, top, perimeter, err := buildLoopSidesAs(ctx, body, ref, pp,
			plan.wallRoleLoop(*col), col.LoopIndex != 0, col.Loop, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		if err := plan.nameWalls(ctx, *col, wallFaces, ref); err != nil {
			return nil, err
		}
		col.bottom, col.top, col.perimeter = bottom, top, perimeter
		faces = append(faces, wallFaces...)
		if col.LoopIndex == 0 {
			for _, f := range wallFaces {
				anchors[f] = struct{}{}
			}
		}
	}

	// One part per (slab, region).
	var parts []stackedPart
	for k, slab := range sp.slabs {
		for r := range slab.regions {
			part, err := stackedRegionPart(ctx, sp, base, columns, colDelta, bySlab[k], k, r, work)
			if err != nil {
				return nil, err
			}
			parts = append(parts, part)
		}
	}

	newPlane := func(z, axial float64, flip bool, role string, area proofbound.BoundedScalar) (*Face, error) {
		frame, err := capFrame(base, z, flip)
		if err != nil {
			return nil, err
		}
		return &Face{surface: Plane{Frame: frame}, origins: []FeatureRef{{producer: ref, Role: role}},
			body: body, area: area.Value, areaBound: area.Bound,
			axialDelta: axial, hasAxialDelta: true}, nil
	}
	// One bottom cap per region of the first slab and one top cap per region
	// of the last, so CapStart(group) selects every lump's bottom face
	// (docs/mirror-pattern-design.md §6.3).
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	var planar []*Face
	capArea := proofbound.BoundedScalar{}
	for _, part := range parts {
		for _, end := range []struct {
			slab int
			z, d float64
			flip bool
			top  bool
		}{{0, first.z0, first.z0Delta, true, false}, {len(sp.slabs) - 1, last.z1, last.z1Delta, false, true}} {
			if part.slab != end.slab {
				continue
			}
			f, err := newPlane(end.z, end.d, end.flip, plan.capRole(end.slab, part.region, end.top), part.area)
			if err != nil {
				return nil, err
			}
			for _, e := range bySlab[end.slab] {
				if e.Region != part.region {
					continue
				}
				coedges := columns[e.Column].bottom
				if end.top {
					coedges = columns[e.Column].top
				}
				f.loops = append(f.loops, &Loop{coedges: coedges, outer: e.Loop == 0})
			}
			if len(planar) == 0 {
				capArea = part.area
			} else {
				capArea = proofbound.BoundedAdd(capArea, part.area)
			}
			planar = append(planar, f)
		}
	}
	capCount := len(planar)
	for _, f := range planar {
		anchors[f] = struct{}{}
	}
	for k := range sp.interfaces {
		z, axial := sp.slabs[k].z1, sp.slabs[k].z1Delta
		patches, err := stackedInterfacePatches(sp, columns, bySlab, k)
		if err != nil {
			return nil, err
		}
		for _, patch := range patches {
			area, err := stackedPatchArea(ctx, sp.sectionDelta, colDelta, columns, patch)
			if err != nil {
				return nil, err
			}
			f, err := newPlane(z, axial, !patch.Floor, plan.patchRole(patch), area)
			if err != nil {
				return nil, err
			}
			for _, pl := range patch.Loops {
				coedges := columns[pl.Column].bottom
				if pl.Top {
					coedges = columns[pl.Column].top
				}
				f.loops = append(f.loops, &Loop{coedges: coedges, outer: pl.Outer})
			}
			planar = append(planar, f)
		}
	}
	if err := attachFaceLoopsContext(ctx, planar); err != nil {
		return nil, err
	}
	faces = append(faces, planar...)
	body.lumps, err = stackedLumps(plan.order(faces), anchors)
	if err != nil {
		return nil, err
	}

	volume := proofbound.BoundedScalar{}
	area := capArea
	var moment [3]proofbound.BoundedScalar
	for _, part := range parts {
		slab := sp.slabs[part.slab]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(slab.z1, slab.z1Delta), proofbound.MeasuredScalar(slab.z0, slab.z0Delta))
		mass := proofbound.BoundedMul(part.area, height)
		volume = proofbound.BoundedAdd(volume, mass)
		point := part.centroid
		pointBound := part.centroidBound
		for i, value := range []float64{point.X, point.Y, point.Z} {
			moment[i] = proofbound.BoundedAdd(moment[i], proofbound.BoundedMul(mass, proofbound.MeasuredScalar(value, pointBound)))
		}
	}
	for _, col := range columns {
		a, b := sp.slabs[col.Start], sp.slabs[col.End]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(b.z1, b.z1Delta), proofbound.MeasuredScalar(a.z0, a.z0Delta))
		area = proofbound.BoundedAdd(area, proofbound.BoundedMul(col.perimeter, height))
	}
	for _, face := range planar[capCount:] {
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(face.area, face.areaBound))
	}
	if volume.Value <= 0 {
		return nil, fmt.Errorf(`%w: a stacked prism encloses no volume`, ErrDegenerate)
	}
	body.volume = Measurement{Value: units.CubicMillimeters(volume.Value), Exactness: exactnessOf(volume.Bound),
		Bound: units.CubicMillimeters(volume.Bound)}
	body.area = Measurement{Value: units.SquareMillimeters(area.Value), Exactness: exactnessOf(area.Bound),
		Bound: units.SquareMillimeters(area.Bound)}
	x, y, z := proofbound.BoundedDiv(moment[0], volume), proofbound.BoundedDiv(moment[1], volume), proofbound.BoundedDiv(moment[2], volume)
	centroid := r3.Vec{X: x.Value, Y: y.Value, Z: z.Value}
	centroidBound := proofbound.Radius3D(max(x.Bound, y.Bound, z.Bound))
	if sp.sectionDelta > 0 || maxColDelta > 0 {
		runs := sp.outerRuns()
		// Every material point lies in one run's prism, so the largest run
		// envelope caps the centroid's distance from the held point.
		geometryBound := 0.0
		for _, run := range runs {
			runBound, err := prismCentroidGeometryBound(run, run.profile, centroid, work, nil)
			if err != nil {
				return nil, err
			}
			geometryBound = max(geometryBound, runBound)
		}
		if sp.sectionDelta > 0 {
			centroidBound = max(centroidBound, proofbound.AbsSumUpper(geometryBound,
				sp.sectionDelta, sp.axialDelta()))
		} else {
			// The column displacements are already charged into every part's
			// moments, so the envelope is a ceiling on that sound answer. It is
			// read off the recorded runs; the denoted body reaches up to the
			// largest column displacement farther in the plane and each level's
			// axial one along the normal, carried through the rigid map's 3·L1
			// factor, as the cup's own envelope is.
			geometryBound = proofbound.AbsSumUpper(geometryBound, proofbound.ProductUpper(3, proofbound.AbsSumUpper(
				proofbound.ProductUpper(proofbound.AbsSumUpper(vecL1(base.frame.U()), vecL1(base.frame.V())), maxColDelta),
				proofbound.ProductUpper(vecL1(base.frame.N()), sp.axialDelta()),
			)))
			centroidBound = math.Min(centroidBound, geometryBound)
		}
	}
	body.centroid = VecMeasurement{Value: centroid, Exactness: exactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}
	outerDelta := 0.0
	for _, entries := range bySlab {
		for _, e := range entries {
			if e.Loop == 0 && (e.Region == 0 || sp.isGroup()) {
				outerDelta = max(outerDelta, colDelta[e.Column])
			}
		}
	}
	bounds, err := stackedBoundsContext(ctx, sp, outerDelta, work)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := chargePrismMap(body, sp.frame, sp.xform); err != nil {
		return nil, err
	}
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = sp
	return body, nil
}
