package decad

import (
	"context"
	"fmt"
	"math"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A slab is one constant section over an axial interval. Its one region is
// kept in a slice so the record can later admit disjoint sections per slab.
type prismSlab struct {
	regions          []ProfileRecord
	z0, z1           float64
	z0Delta, z1Delta float64
}

type prismSlabInterface struct {
	lowerExposed []ProfileRecord
	upperExposed []ProfileRecord
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
		profile: ProfileRecord{Outer: first.regions[0].Outer},
		frame:   sp.frame, xform: sp.xform, sectionDelta: sp.sectionDelta,
		z0: first.z0, z0Delta: first.z0Delta,
		z1: last.z1, z1Delta: last.z1Delta,
	}
}

func (sp stackedPrismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	return sp.outerPrism().extentAlong(g)
}

// stackedExclusiveHoles derives the holes present on only one side of an
// interface. Admission proves their nesting; this function compares records.
func stackedExclusiveHoles(lower, upper ProfileRecord) (lowerOnly, upperOnly []LoopRecord, err error) {
	for _, hole := range lower.Holes {
		found := false
		for _, other := range upper.Holes {
			equal, e := loopRecordsEqual(nil, hole, other)
			if e != nil {
				return nil, nil, e
			}
			if equal {
				found = true
				break
			}
		}
		if !found {
			lowerOnly = append(lowerOnly, hole)
		}
	}
	for _, hole := range upper.Holes {
		found := false
		for _, other := range lower.Holes {
			equal, e := loopRecordsEqual(nil, hole, other)
			if e != nil {
				return nil, nil, e
			}
			if equal {
				found = true
				break
			}
		}
		if !found {
			upperOnly = append(upperOnly, hole)
		}
	}
	return lowerOnly, upperOnly, nil
}

func stackedExposed(ctx context.Context, holes []LoopRecord) ([]ProfileRecord, error) {
	out := make([]ProfileRecord, 0, len(holes))
	for _, hole := range holes {
		reversed, err := reverseLoopRecordContext(ctx, hole)
		if err != nil {
			return nil, err
		}
		out = append(out, ProfileRecord{Outer: reversed})
	}
	return out, nil
}

// falsifyStackedPayload refuses any record whose interfaces cannot be built
// from whole, shared loop columns. It is run before topology and chording.
func falsifyStackedPayload(ctx context.Context, sp stackedPrismPayload) error {
	if len(sp.slabs) < 2 || len(sp.interfaces) != len(sp.slabs)-1 {
		return fmt.Errorf(`%w: a stacked prism needs two slabs and one interface between each pair`, ErrUnsupported)
	}
	for i, slab := range sp.slabs {
		if err := ctx.Err(); err != nil {
			return err
		}
		if len(slab.regions) != 1 {
			return fmt.Errorf(`%w: slab %d has no single region`, ErrUnsupported, i)
		}
		if math.IsNaN(slab.z0) || math.IsNaN(slab.z1) ||
			math.IsInf(slab.z0, 0) || math.IsInf(slab.z1, 0) || slab.z0 >= slab.z1 {
			return fmt.Errorf(`%w: slab %d has an empty interval`, ErrDegenerate, i)
		}
		if i == 0 {
			continue
		}
		prev := sp.slabs[i-1]
		if prev.z1 != slab.z0 || prev.z1Delta != slab.z0Delta {
			return fmt.Errorf(`%w: slabs %d and %d do not share one level`, ErrDegenerate, i-1, i)
		}
		equal, err := loopRecordsEqual(nil, prev.regions[0].Outer, slab.regions[0].Outer)
		if err != nil {
			return err
		}
		if !equal {
			return fmt.Errorf(`%w: a stacked prism's outer wall must be one loop`, ErrUnsupported)
		}
		lowerOnly, upperOnly, err := stackedExclusiveHoles(prev.regions[0], slab.regions[0])
		if err != nil {
			return err
		}
		if len(lowerOnly) != 0 && len(upperOnly) != 0 {
			return fmt.Errorf(`%w: both sides of interface %d have exclusive holes`, ErrUnsupported, i-1)
		}
		wantLower, err := stackedExposed(ctx, upperOnly)
		if err != nil {
			return err
		}
		wantUpper, err := stackedExposed(ctx, lowerOnly)
		if err != nil {
			return err
		}
		got := sp.interfaces[i-1]
		if !reflect.DeepEqual(got.lowerExposed, wantLower) || !reflect.DeepEqual(got.upperExposed, wantUpper) {
			return fmt.Errorf(`%w: interface %d does not record its exposed material`, ErrDegenerate, i-1)
		}
	}
	return nil
}

type stackedColumn struct {
	loop       LoopRecord
	start, end int
	loopIndex  int
	bottom     []coedge
	top        []coedge
	perimeter  proofbound.BoundedScalar
}

func stackedLoops(region ProfileRecord) []LoopRecord {
	return append([]LoopRecord{region.Outer}, region.Holes...)
}

// stackedColumns names each uninterrupted wall once. bySlab maps every
// region loop in each slab back to its column, including shared through holes.
func stackedColumns(sp stackedPrismPayload) ([]stackedColumn, [][]int, error) {
	var columns []stackedColumn
	bySlab := make([][]int, len(sp.slabs))
	for k, slab := range sp.slabs {
		loops := stackedLoops(slab.regions[0])
		bySlab[k] = make([]int, len(loops))
		for i, loop := range loops {
			found := -1
			if k != 0 {
				for j, prev := range stackedLoops(sp.slabs[k-1].regions[0]) {
					equal, err := loopRecordsEqual(nil, loop, prev)
					if err != nil {
						return nil, nil, err
					}
					if equal {
						found = bySlab[k-1][j]
						break
					}
				}
			}
			if found < 0 {
				found = len(columns)
				columns = append(columns, stackedColumn{loop: loop, start: k, end: k, loopIndex: i})
			} else {
				columns[found].end = k
			}
			bySlab[k][i] = found
		}
	}
	return columns, bySlab, nil
}

func evalStackedContext(ctx context.Context, d *Document, ref producerID, sp stackedPrismPayload) (*Body, error) {
	if err := falsifyStackedPayload(ctx, sp); err != nil {
		return nil, err
	}
	columns, bySlab, err := stackedColumns(sp)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	work := freeform.NewFreeformWork()
	base := sp.outerPrism()
	var faces []*Face
	for ci := range columns {
		col := &columns[ci]
		first, last := sp.slabs[col.start], sp.slabs[col.end]
		pp := base
		pp.z0, pp.z0Delta = first.z0, first.z0Delta
		pp.z1, pp.z1Delta = last.z1, last.z1Delta
		wallFaces, bottom, top, perimeter, err := buildLoopSidesAs(ctx, body, ref, pp,
			col.loopIndex, col.loopIndex != 0, col.loop, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		for _, face := range wallFaces {
			for i := range face.origins {
				face.origins[i].Role = fmt.Sprintf("slab(%d).region(0).%s", col.start, face.origins[i].Role)
			}
		}
		col.bottom, col.top, col.perimeter = bottom, top, perimeter
		faces = append(faces, wallFaces...)
	}

	regionArea := make([]proofbound.BoundedScalar, len(sp.slabs))
	regionCentroid := make([]r3.Vec, len(sp.slabs))
	regionCentroidBound := make([]float64, len(sp.slabs))
	for k, slab := range sp.slabs {
		ig, err := slab.regions[0].evaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, work)
		if err != nil {
			return nil, err
		}
		if ig.area <= 0 {
			return nil, fmt.Errorf(`%w: slab %d has no material area`, ErrDegenerate, k)
		}
		perimeter := proofbound.BoundedScalar{}
		walks := 0
		for _, ci := range bySlab[k] {
			perimeter = proofbound.BoundedAdd(perimeter, columns[ci].perimeter)
			walks += len(columns[ci].loop.Segments)
		}
		regionArea[k] = proofbound.MeasuredScalar(ig.area, proofbound.AbsSumUpper(ig.areaBound,
			proofbound.SectionDisplacementArea(sp.sectionDelta, walks, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))))
		u := proofbound.BoundedQuotient(ig.mu, ig.muBound, ig.area, ig.areaBound)
		v := proofbound.BoundedQuotient(ig.mv, ig.mvBound, ig.area, ig.areaBound)
		u.Bound = proofbound.AbsSumUpper(u.Bound, sp.sectionDelta)
		v.Bound = proofbound.AbsSumUpper(v.Bound, sp.sectionDelta)
		mid := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.MeasuredScalar(slab.z0, slab.z0Delta),
			proofbound.MeasuredScalar(slab.z1, slab.z1Delta)), proofbound.ExactScalar(2))
		regionCentroid[k] = base.point(u.Value, v.Value, mid.Value)
		regionCentroidBound[k] = prismPointBound(base, u, v, mid)
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
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	capStart, err := newPlane(first.z0, first.z0Delta, true, roleCapStart, regionArea[0])
	if err != nil {
		return nil, err
	}
	capEnd, err := newPlane(last.z1, last.z1Delta, false, roleCapEnd, regionArea[len(regionArea)-1])
	if err != nil {
		return nil, err
	}
	for _, ci := range bySlab[0] {
		col := columns[ci]
		capStart.loops = append(capStart.loops, &Loop{coedges: col.bottom, outer: col.loopIndex == 0})
	}
	for _, ci := range bySlab[len(bySlab)-1] {
		col := columns[ci]
		capEnd.loops = append(capEnd.loops, &Loop{coedges: col.top, outer: col.loopIndex == 0})
	}
	planar := []*Face{capStart, capEnd}
	for k, boundary := range sp.interfaces {
		z, axial := sp.slabs[k].z1, sp.slabs[k].z1Delta
		lowerOnly, upperOnly, err := stackedExclusiveHoles(
			sp.slabs[k].regions[0], sp.slabs[k+1].regions[0])
		if err != nil {
			return nil, err
		}
		for e, exposed := range boundary.lowerExposed {
			area, err := loopEnclosedAreaContext(ctx, exposed.Outer)
			if err != nil {
				return nil, err
			}
			ci, err := stackedHoleColumn(columns, bySlab[k+1][1:], upperOnly[e], k+1, true)
			if err != nil {
				return nil, err
			}
			perimeter := columns[ci].perimeter
			area.Bound = proofbound.AbsSumUpper(area.Bound, proofbound.SectionDisplacementArea(sp.sectionDelta,
				len(columns[ci].loop.Segments), proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound)))
			f, err := newPlane(z, axial, false, fmt.Sprintf("floor(%d,%d)", k, e), area)
			if err != nil {
				return nil, err
			}
			f.loops = []*Loop{{coedges: columns[ci].bottom, outer: true}}
			planar = append(planar, f)
		}
		for e, exposed := range boundary.upperExposed {
			area, err := loopEnclosedAreaContext(ctx, exposed.Outer)
			if err != nil {
				return nil, err
			}
			ci, err := stackedHoleColumn(columns, bySlab[k][1:], lowerOnly[e], k, false)
			if err != nil {
				return nil, err
			}
			perimeter := columns[ci].perimeter
			area.Bound = proofbound.AbsSumUpper(area.Bound, proofbound.SectionDisplacementArea(sp.sectionDelta,
				len(columns[ci].loop.Segments), proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound)))
			f, err := newPlane(z, axial, true, fmt.Sprintf("ceiling(%d,%d)", k, e), area)
			if err != nil {
				return nil, err
			}
			f.loops = []*Loop{{coedges: columns[ci].top, outer: true}}
			planar = append(planar, f)
		}
	}
	if err := attachFaceLoopsContext(ctx, planar); err != nil {
		return nil, err
	}
	faces = append(faces, planar...)
	body.lumps = sheetLumps(faces)

	volume := proofbound.BoundedScalar{}
	area := proofbound.BoundedAdd(regionArea[0], regionArea[len(regionArea)-1])
	var moment [3]proofbound.BoundedScalar
	for k, slab := range sp.slabs {
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(slab.z1, slab.z1Delta), proofbound.MeasuredScalar(slab.z0, slab.z0Delta))
		mass := proofbound.BoundedMul(regionArea[k], height)
		volume = proofbound.BoundedAdd(volume, mass)
		point := regionCentroid[k]
		pointBound := regionCentroidBound[k]
		for i, value := range []float64{point.X, point.Y, point.Z} {
			moment[i] = proofbound.BoundedAdd(moment[i], proofbound.BoundedMul(mass, proofbound.MeasuredScalar(value, pointBound)))
		}
	}
	for _, col := range columns {
		a, b := sp.slabs[col.start], sp.slabs[col.end]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(b.z1, b.z1Delta), proofbound.MeasuredScalar(a.z0, a.z0Delta))
		area = proofbound.BoundedAdd(area, proofbound.BoundedMul(col.perimeter, height))
	}
	for _, face := range planar[2:] {
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
	centroidBound := proofbound.Radius3D(max(x.Bound, y.Bound, z.Bound))
	if sp.sectionDelta > 0 {
		geometryBound, err := prismCentroidGeometryBound(base, base.profile,
			r3.Vec{X: x.Value, Y: y.Value, Z: z.Value}, work, nil)
		if err != nil {
			return nil, err
		}
		centroidBound = max(centroidBound, proofbound.AbsSumUpper(geometryBound,
			sp.sectionDelta, sp.axialDelta()))
	}
	body.centroid = VecMeasurement{Value: r3.Vec{X: x.Value, Y: y.Value, Z: z.Value},
		Exactness: exactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}
	bounds, err := prismBoundsContext(ctx, base, work, nil)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = sp
	return body, nil
}
