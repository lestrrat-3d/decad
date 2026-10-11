package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// allEdgeChamferPayload is the bounded box-with-one-cross-bore construction
// in modify-general §4.3b. The twelve simultaneous cuts are half spaces;
// their three planes meet at (d/2,d/2,d/2) at each original box corner.
type allEdgeChamferPayload struct {
	xform  r3.Transform
	box    [3][2]float64
	center [2]float64 // x and z of the through-y bore
	radius float64
	d      float64
}

func (p allEdgeChamferPayload) transform() r3.Transform { return p.xform }

func (p allEdgeChamferPayload) placed(ctx context.Context, d *Document, ref producerID,
	composed r3.Transform) (*Body, error) {
	p.xform = composed
	return evalAllEdgeChamfer(ctx, d, ref, p)
}

// tryAllEdgeChamfer accepts only the complete straight-edge set of an exact
// axis-aligned box with one through-y circular bore. Other breps keep their
// existing route L result or refusal.
func tryAllEdgeChamfer(ctx context.Context, source *Body, bp brepPayload,
	edges []*Edge, distance, distanceDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	if len(edges) != 12 || asym != nil || len(bp.faces) != 7 || bp.xform != r3.Identity() ||
		bp.sectionDelta() != 0 || bp.axialDelta() != 0 || len(bp.loopBands) != 0 ||
		bp.bossShell != nil || bp.pocketShell != nil {
		return nil, false, nil
	}
	if distanceDelta != 0 {
		return nil, true, fmt.Errorf(`%w: the all-edge chamfer needs an exact millimetre setback`, ErrUnsupported)
	}
	var p allEdgeChamferPayload
	p.xform = r3.Identity()
	p.d = distance
	p.box = [3][2]float64{{source.bounds.Min.X, source.bounds.Max.X},
		{source.bounds.Min.Y, source.bounds.Max.Y}, {source.bounds.Min.Z, source.bounds.Max.Z}}
	if source.bounds.Bound.Base() != 0 || !p.admitBox() {
		return nil, false, nil
	}
	selected := map[*Edge]struct{}{}
	for _, e := range edges {
		if _, dup := selected[e]; dup || !p.outerBoxEdge(e) {
			return nil, false, nil
		}
		selected[e] = struct{}{}
	}
	planes := 0
	var boreFace *Face
	var boreSurface Cylinder
	var seen [3][2]bool
	for _, f := range source.Faces() {
		switch surface := f.Surface().(type) {
		case Plane:
			axis, side, ok := p.boxPlane(f, surface)
			if !ok || seen[axis][side] {
				return nil, false, nil
			}
			seen[axis][side] = true
			planes++
			if axis == 1 {
				if len(f.loops) != 2 || len(f.loops[1].coedges) != 1 {
					return nil, false, nil
				}
				circle, ok := f.loops[1].coedges[0].edge.curve.(Circle3)
				if !ok || circle.Center.Y != p.box[1][side] ||
					circle.Axis != (r3.Vec{Y: 1}) && circle.Axis != (r3.Vec{Y: -1}) {
					return nil, false, nil
				}
				if p.radius == 0 {
					p.center, p.radius = [2]float64{circle.Center.X, circle.Center.Z}, circle.Radius.Base()
				} else if p.center != [2]float64{circle.Center.X, circle.Center.Z} || p.radius != circle.Radius.Base() {
					return nil, false, nil
				}
			} else if len(f.loops) != 1 {
				return nil, false, nil
			}
		case Cylinder:
			if boreFace != nil {
				return nil, false, nil
			}
			boreFace, boreSurface = f, surface
		default:
			return nil, false, nil
		}
	}
	if planes != 6 || boreFace == nil || p.radius <= 0 ||
		boreSurface.Axis != (r3.Vec{Y: 1}) && boreSurface.Axis != (r3.Vec{Y: -1}) ||
		boreSurface.Origin.X != p.center[0] || boreSurface.Origin.Z != p.center[1] ||
		boreSurface.Radius.Base() != p.radius || !boreFace.reversed || len(boreFace.loops) != 2 {
		return nil, false, nil
	}
	var rims [2]bool
	for _, loop := range boreFace.loops {
		if len(loop.coedges) != 1 {
			return nil, false, nil
		}
		edge := loop.coedges[0].edge
		circle, ok := edge.curve.(Circle3)
		if !ok || !edge.curveBounded || edge.curveBound != 0 ||
			circle.Radius.Base() != p.radius {
			return nil, false, nil
		}
		side := -1
		for s := range 2 {
			if circle.Center == r3.NewVec(p.center[0], p.box[1][s], p.center[1]) {
				side = s
			}
		}
		if side < 0 || rims[side] {
			return nil, false, nil
		}
		rims[side] = true
	}
	for axis := range seen {
		if !seen[axis][0] || !seen[axis][1] {
			return nil, false, nil
		}
	}
	if !p.exactCoordinates() {
		return nil, true, fmt.Errorf(`%w: an all-edge chamfer corner is not exactly representable`, ErrUnsupported)
	}
	if p.center[0]-p.radius <= p.box[0][0]+distance ||
		p.center[0]+p.radius >= p.box[0][1]-distance ||
		p.center[1]-p.radius <= p.box[2][0]+distance ||
		p.center[1]+p.radius >= p.box[2][1]-distance {
		return nil, true, fmt.Errorf(`%w: the bore reaches the all-edge chamfer`, ErrUnsupported)
	}
	body, err := evalAllEdgeChamfer(ctx, source.doc, source.doc.nextProducerID(), p)
	return body, true, err
}

func (p allEdgeChamferPayload) admitBox() bool {
	if p.d <= 0 || p.d/2 == 0 || math.IsNaN(p.d) || math.IsInf(p.d, 0) {
		return false
	}
	for _, interval := range p.box {
		if math.IsNaN(interval[0]) || math.IsNaN(interval[1]) ||
			math.IsInf(interval[0], 0) || math.IsInf(interval[1], 0) ||
			interval[1]-interval[0] <= 2*p.d || interval[0]+p.d == interval[0] ||
			interval[1]-p.d == interval[1] {
			return false
		}
	}
	return true
}

// exactCoordinates requires every published polygon corner and circle seam
// to equal its rational expression in the recorded box and setback. This
// rules out a rounded miter that would silently move a bevel plane.
func (p allEdgeChamferPayload) exactCoordinates() bool {
	for _, interval := range p.box {
		lo, hi, d := proofarith.FloatRat(interval[0]), proofarith.FloatRat(interval[1]), proofarith.FloatRat(p.d)
		half := new(big.Rat).Quo(d, big.NewRat(2, 1))
		if proofarith.FloatRat(p.d/2).Cmp(half) != 0 ||
			!rationalSumEquals(interval[0], p.d, interval[0]+p.d) ||
			!rationalSumEquals(interval[0], p.d/2, interval[0]+p.d/2) ||
			!rationalSumEquals(interval[1], -p.d, interval[1]-p.d) ||
			!rationalSumEquals(interval[1], -p.d/2, interval[1]-p.d/2) ||
			new(big.Rat).Sub(hi, lo).Cmp(new(big.Rat).Mul(d, big.NewRat(2, 1))) <= 0 {
			return false
		}
	}
	for _, center := range p.center {
		if !rationalSumEquals(center, p.radius, center+p.radius) ||
			!rationalSumEquals(center, -p.radius, center-p.radius) {
			return false
		}
	}
	return true
}

func rationalSumEquals(a, b, held float64) bool {
	return proofarith.FloatRat(held).Cmp(new(big.Rat).Add(proofarith.FloatRat(a), proofarith.FloatRat(b))) == 0
}

func rationalMidpointEquals(interval [2]float64, held float64) bool {
	sum := new(big.Rat).Add(proofarith.FloatRat(interval[0]), proofarith.FloatRat(interval[1]))
	return new(big.Rat).Mul(proofarith.FloatRat(held), big.NewRat(2, 1)).Cmp(sum) == 0
}

func (p allEdgeChamferPayload) outerBoxEdge(e *Edge) bool {
	if _, ok := e.curve.(Line3); !ok || e.start.bound.Base() != 0 || e.end.bound.Base() != 0 {
		return false
	}
	a, b := e.start.position, e.end.position
	x, y := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}
	varying := 0
	for axis := range 3 {
		if x[axis] == y[axis] {
			if x[axis] != p.box[axis][0] && x[axis] != p.box[axis][1] {
				return false
			}
			continue
		}
		varying++
		if (x[axis] != p.box[axis][0] || y[axis] != p.box[axis][1]) &&
			(x[axis] != p.box[axis][1] || y[axis] != p.box[axis][0]) {
			return false
		}
	}
	return varying == 1
}

func (p allEdgeChamferPayload) boxPlane(f *Face, plane Plane) (int, int, bool) {
	if len(f.loops) == 0 || len(f.loops[0].coedges) != 4 {
		return 0, 0, false
	}
	n := plane.Frame.N()
	axis, side := -1, -1
	for k, v := range [3]float64{n.X, n.Y, n.Z} {
		if v == 1 || v == -1 {
			if axis >= 0 {
				return 0, 0, false
			}
			axis, side = k, 0
			if v > 0 {
				side = 1
			}
		} else if v != 0 {
			return 0, 0, false
		}
	}
	if axis < 0 {
		return 0, 0, false
	}
	seen := map[[3]float64]struct{}{}
	for _, ce := range f.loops[0].coedges {
		if _, ok := ce.edge.curve.(Line3); !ok || ce.Start().bound.Base() != 0 {
			return 0, 0, false
		}
		v := ce.Start().position
		c := [3]float64{v.X, v.Y, v.Z}
		if c[axis] != p.box[axis][side] {
			return 0, 0, false
		}
		for j := range 3 {
			if j != axis && c[j] != p.box[j][0] && c[j] != p.box[j][1] {
				return 0, 0, false
			}
		}
		seen[c] = struct{}{}
	}
	return axis, side, len(seen) == 4
}

type allEdgePolygon struct {
	points []r3.Vec
	normal r3.Vec
	role   string
	area   proofbound.BoundedScalar
	hole   bool
}

func (p allEdgeChamferPayload) polygons() []allEdgePolygon {
	var out []allEdgePolygon
	for axis := range 3 {
		b, c := (axis+1)%3, (axis+2)%3
		for side := range 2 {
			coords := [][3]float64{}
			for _, pair := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
				var q [3]float64
				q[axis] = p.box[axis][side]
				q[b] = p.box[b][pair[0]] + inward(pair[0], p.d)
				q[c] = p.box[c][pair[1]] + inward(pair[1], p.d)
				coords = append(coords, q)
			}
			n := [3]float64{}
			n[axis] = float64(2*side - 1)
			widthB := proofbound.BoundedSub(
				proofbound.BoundedSub(proofbound.ExactScalar(p.box[b][1]), proofbound.ExactScalar(p.box[b][0])),
				proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.ExactScalar(p.d)))
			widthC := proofbound.BoundedSub(
				proofbound.BoundedSub(proofbound.ExactScalar(p.box[c][1]), proofbound.ExactScalar(p.box[c][0])),
				proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.ExactScalar(p.d)))
			area := proofbound.BoundedMul(
				widthB, widthC)
			out = append(out, allEdgePolygon{points: vecs(coords), normal: vec(n),
				role: fmt.Sprintf("face(%d)", 2*axis+side), area: area, hole: axis == 1})
		}
	}
	sqrt2 := proofbound.BoundedSqrt(proofbound.ExactScalar(2))
	for axis := range 3 {
		b, c := (axis+1)%3, (axis+2)%3
		for sb := range 2 {
			for sc := range 2 {
				q := func(along, ub float64) [3]float64 {
					var v [3]float64
					v[axis] = along
					v[b] = p.box[b][sb] + inward(sb, ub)
					v[c] = p.box[c][sc] + inward(sc, p.d-ub)
					return v
				}
				lo, hi := p.box[axis][0], p.box[axis][1]
				points := vecs([][3]float64{q(lo+p.d, 0), q(lo+p.d/2, p.d/2),
					q(lo+p.d, p.d), q(hi-p.d, p.d), q(hi-p.d/2, p.d/2), q(hi-p.d, 0)})
				n := [3]float64{}
				n[b], n[c] = float64(2*sb-1), float64(2*sc-1)
				length := proofbound.BoundedSub(proofbound.ExactScalar(hi), proofbound.ExactScalar(lo))
				d2 := proofbound.BoundedMul(proofbound.ExactScalar(p.d), proofbound.ExactScalar(p.d))
				span := proofbound.BoundedSub(proofbound.BoundedMul(length, proofbound.ExactScalar(p.d)),
					proofbound.BoundedMul(proofbound.ExactScalar(1.5), d2))
				area := proofbound.BoundedMul(sqrt2, span)
				out = append(out, allEdgePolygon{points: points, normal: vec(n),
					role: fmt.Sprintf("chamfer(%d,%d,%d)", axis, sb, sc), area: area})
			}
		}
	}
	return out
}

func inward(side int, d float64) float64 {
	if side == 0 {
		return d
	}
	return -d
}

func vec(c [3]float64) r3.Vec { return r3.NewVec(c[0], c[1], c[2]) }

func vecs(coords [][3]float64) []r3.Vec {
	out := make([]r3.Vec, len(coords))
	for i, c := range coords {
		out[i] = vec(c)
	}
	return out
}

func evalAllEdgeChamfer(ctx context.Context, d *Document, ref producerID,
	p allEdgeChamferPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, axisMap, err := p.placedGeometry()
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	polygons := geometry.polygons()
	vertices := map[r3.Vec]*Vertex{}
	vertex := func(v r3.Vec) *Vertex {
		if found := vertices[v]; found != nil {
			return found
		}
		made := &Vertex{position: v, bound: units.Millimeters(0)}
		vertices[v] = made
		return made
	}
	type lineKey struct{ a, b r3.Vec }
	lines := map[lineKey]*Edge{}
	faces := make([]*Face, 0, len(polygons)+1)
	for polygonIndex, poly := range polygons {
		points := slices.Clone(poly.points)
		if points[1].Sub(points[0]).Cross(points[2].Sub(points[0])).Dot(poly.normal) < 0 {
			slices.Reverse(points)
		}
		u := points[1].Sub(points[0])
		frame, err := r3.NewFrame(points[0], u, poly.normal.Cross(u))
		if err != nil {
			return nil, fmt.Errorf(`%w: all-edge chamfer plane: %s`, ErrUnsupported, err)
		}
		face := &Face{surface: Plane{Frame: frame},
			origins: []FeatureRef{{producer: ref, Role: axisMap.faceRole(polygonIndex)}},
			body:    body, area: poly.area.Value, areaBound: poly.area.Bound}
		loop := &Loop{outer: true}
		for i, a := range points {
			b := points[(i+1)%len(points)]
			key := lineKey{a, b}
			if b.X < a.X || b.X == a.X && (b.Y < a.Y || b.Y == a.Y && b.Z < a.Z) {
				key = lineKey{b, a}
			}
			e := lines[key]
			if e == nil {
				lengthSquared := proofbound.ExactScalar(0)
				for _, pair := range [][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
					v := proofbound.BoundedSub(proofbound.ExactScalar(pair[1]), proofbound.ExactScalar(pair[0]))
					lengthSquared = proofbound.BoundedAdd(lengthSquared,
						proofbound.BoundedMul(v, v))
				}
				length := proofbound.BoundedSqrt(lengthSquared)
				e = &Edge{curve: Line3{}, start: vertex(a), end: vertex(b), convex: true,
					length: length.Value, lengthBound: length.Bound}
				lines[key] = e
			}
			loop.coedges = append(loop.coedges, coedge{edge: e, forward: e.start == vertex(a)})
		}
		face.loops = []*Loop{loop}
		faces = append(faces, face)
	}
	bore, err := geometry.addBoreTopology(body, ref, faces, vertex)
	if err != nil {
		return nil, err
	}
	faces = append(faces, bore)
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	for _, e := range lines {
		if len(e.faces) != 2 {
			return nil, fmt.Errorf(`%w: all-edge chamfer has an unsewn planar edge`, ErrUnsupported)
		}
	}
	body.lumps = sheetLumps(faces)
	if len(body.lumps) != 1 || body.lumps[0].shells[0].open {
		return nil, fmt.Errorf(`%w: all-edge chamfer has an open shell`, ErrUnsupported)
	}
	if err := geometry.measure(body, polygons); err != nil {
		return nil, err
	}
	body.payload = p
	return body, nil
}

func (p allEdgeChamferPayload) addBoreTopology(body *Body, ref producerID, faces []*Face,
	vertex func(r3.Vec) *Vertex) (*Face, error) {
	if len(faces) < 4 {
		return nil, fmt.Errorf(`%w: the all-edge chamfer lacks its two bored faces`, ErrUnsupported)
	}
	pi := stitchflux.PiScalar()
	r := proofbound.ExactScalar(p.radius)
	disk := proofbound.BoundedMul(pi, proofbound.BoundedMul(r, r))
	height := proofbound.BoundedSub(proofbound.ExactScalar(p.box[1][1]), proofbound.ExactScalar(p.box[1][0]))
	wallArea := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), pi),
		proofbound.BoundedMul(r, height))
	bore := &Face{surface: Cylinder{Origin: r3.NewVec(p.center[0], p.box[1][0], p.center[1]),
		Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(p.radius)}, reversed: true,
		origins: []FeatureRef{{producer: ref, Role: "wall(6)"}}, body: body,
		area: wallArea.Value, areaBound: wallArea.Bound}
	for side := range 2 {
		face := faces[2+side]
		area := proofbound.BoundedSub(proofbound.MeasuredScalar(face.area, face.areaBound), disk)
		face.area, face.areaBound = area.Value, area.Bound
		center := r3.NewVec(p.center[0], p.box[1][side], p.center[1])
		seam := vertex(center.Add(r3.NewVec(p.radius, 0, 0)))
		circumference := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), pi), r)
		edge := &Edge{curve: Circle3{Center: center, Axis: r3.NewVec(0, -1, 0),
			Radius: units.Millimeters(p.radius)}, start: seam, end: seam, convex: false,
			length: circumference.Value, lengthBound: circumference.Bound, curveBounded: true}
		face.loops = append(face.loops, &Loop{coedges: []coedge{{edge: edge, forward: side == 1}}})
		bore.loops = append(bore.loops, &Loop{coedges: []coedge{{edge: edge, forward: side == 0}}, outer: true})
	}
	return bore, nil
}

func (p allEdgeChamferPayload) measure(body *Body, polygons []allEdgePolygon) error {
	lx := proofbound.BoundedSub(proofbound.ExactScalar(p.box[0][1]), proofbound.ExactScalar(p.box[0][0]))
	ly := proofbound.BoundedSub(proofbound.ExactScalar(p.box[1][1]), proofbound.ExactScalar(p.box[1][0]))
	lz := proofbound.BoundedSub(proofbound.ExactScalar(p.box[2][1]), proofbound.ExactScalar(p.box[2][0]))
	d := proofbound.ExactScalar(p.d)
	r := proofbound.ExactScalar(p.radius)
	pi := stitchflux.PiScalar()
	box := proofbound.BoundedMul(proofbound.BoundedMul(lx, ly), lz)
	bore := proofbound.BoundedMul(proofbound.BoundedMul(pi, proofbound.BoundedMul(r, r)), ly)
	d2 := proofbound.BoundedMul(d, d)
	removed := proofbound.BoundedSub(
		proofbound.BoundedMul(proofbound.ExactScalar(2),
			proofbound.BoundedMul(d2, proofbound.BoundedAdd(proofbound.BoundedAdd(lx, ly), lz))),
		proofbound.BoundedMul(proofbound.ExactScalar(6), proofbound.BoundedMul(d2, d)))
	volume := proofbound.BoundedSub(proofbound.BoundedSub(box, bore), removed)
	if volume.Value-volume.Bound <= 0 {
		return fmt.Errorf(`%w: the all-edge chamfer has no proven positive volume`, ErrUnsupported)
	}
	body.volume = Measurement{Value: units.CubicMillimeters(volume.Value), Bound: units.CubicMillimeters(volume.Bound),
		Exactness: exactnessOf(volume.Bound)}
	area := proofbound.ExactScalar(0)
	for _, poly := range polygons {
		area = proofbound.BoundedAdd(area, poly.area)
	}
	disks := proofbound.BoundedMul(proofbound.ExactScalar(2),
		proofbound.BoundedMul(pi, proofbound.BoundedMul(r, r)))
	wall := proofbound.BoundedMul(proofbound.ExactScalar(2),
		proofbound.BoundedMul(pi, proofbound.BoundedMul(r, ly)))
	area = proofbound.BoundedAdd(proofbound.BoundedSub(area, disks), wall)
	body.area = Measurement{Value: units.SquareMillimeters(area.Value), Bound: units.SquareMillimeters(area.Bound),
		Exactness: exactnessOf(area.Bound)}
	// The admitted bore is centered in x and z and traverses the full y range.
	// The twelve edge wedges are symmetric about all three box midplanes.
	if !rationalMidpointEquals(p.box[0], p.center[0]) ||
		!rationalMidpointEquals(p.box[2], p.center[1]) {
		return fmt.Errorf(`%w: the all-edge chamfer requires a centered bore`, ErrUnsupported)
	}
	var midpoint [3]proofbound.BoundedScalar
	centroidBound := 0.0
	for axis := range 3 {
		midpoint[axis] = proofbound.BoundedMul(proofbound.ExactScalar(0.5),
			proofbound.BoundedAdd(proofbound.ExactScalar(p.box[axis][0]), proofbound.ExactScalar(p.box[axis][1])))
		centroidBound = math.Max(centroidBound, midpoint[axis].Bound)
	}
	centroid := r3.NewVec(midpoint[0].Value, midpoint[1].Value, midpoint[2].Value)
	body.centroid = VecMeasurement{Value: centroid, Bound: units.Millimeters(centroidBound),
		Exactness: exactnessOf(centroidBound)}
	body.bounds = Box{Min: r3.NewVec(p.box[0][0], p.box[1][0], p.box[2][0]),
		Max:   r3.NewVec(p.box[0][1], p.box[1][1], p.box[2][1]),
		Bound: units.Millimeters(0), Exactness: Exact}
	return validateAnalyticBodyMeasurements(body)
}
