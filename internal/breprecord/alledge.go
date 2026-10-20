// Package breprecord holds the BRep family's payload records and their
// builders over topology nodes. It implements topo.Payload
// (agent-docs/evaluator-design.md §8) and links nodes only; root maps them to
// handles through the allocator it passes in the builder environment.
package breprecord

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// AllEdgeChamferPayload is the bounded box-with-one-cross-bore construction
// in modify-general §4.3b. The twelve simultaneous cuts are half spaces;
// their three planes meet at (d/2,d/2,d/2) at each original box corner.
type AllEdgeChamferPayload struct {
	Xform  r3.Transform
	Box    [3][2]float64
	Center [2]float64 // x and z of the through-y bore
	Radius float64
	D      float64
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab), the name the evaluator has always published for it.
func (AllEdgeChamferPayload) SurveyClass() string { return "allEdgeChamferPayload" }

// Transform is the accumulated rigid placement.
func (p AllEdgeChamferPayload) Transform() r3.Transform { return p.Xform }

// Placed re-evaluates the record under the composed placement.
func (p AllEdgeChamferPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	p.Xform = composed
	return BuildAllEdgeChamfer(ctx, env, p)
}

// AdmitBox reports whether the recorded box and setback are finite and leave
// every inset face a positive width at float precision.
func (p AllEdgeChamferPayload) AdmitBox() bool {
	if p.D <= 0 || p.D/2 == 0 || math.IsNaN(p.D) || math.IsInf(p.D, 0) {
		return false
	}
	for _, interval := range p.Box {
		if math.IsNaN(interval[0]) || math.IsNaN(interval[1]) ||
			math.IsInf(interval[0], 0) || math.IsInf(interval[1], 0) ||
			interval[1]-interval[0] <= 2*p.D || interval[0]+p.D == interval[0] ||
			interval[1]-p.D == interval[1] {
			return false
		}
	}
	return true
}

// ExactCoordinates requires every published polygon corner and circle seam
// to equal its rational expression in the recorded box and setback. This
// rules out a rounded miter that would silently move a bevel plane.
func (p AllEdgeChamferPayload) ExactCoordinates() bool {
	for _, interval := range p.Box {
		lo, hi, d := proofarith.FloatRat(interval[0]), proofarith.FloatRat(interval[1]), proofarith.FloatRat(p.D)
		half := new(big.Rat).Quo(d, big.NewRat(2, 1))
		if proofarith.FloatRat(p.D/2).Cmp(half) != 0 ||
			!rationalSumEquals(interval[0], p.D, interval[0]+p.D) ||
			!rationalSumEquals(interval[0], p.D/2, interval[0]+p.D/2) ||
			!rationalSumEquals(interval[1], -p.D, interval[1]-p.D) ||
			!rationalSumEquals(interval[1], -p.D/2, interval[1]-p.D/2) ||
			new(big.Rat).Sub(hi, lo).Cmp(new(big.Rat).Mul(d, big.NewRat(2, 1))) <= 0 {
			return false
		}
	}
	for _, center := range p.Center {
		if !rationalSumEquals(center, p.Radius, center+p.Radius) ||
			!rationalSumEquals(center, -p.Radius, center-p.Radius) {
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

// OuterBoxEdge reports whether e is a straight zero-bound edge running the
// full length of one box side.
func (p AllEdgeChamferPayload) OuterBoxEdge(e *topo.Edge) bool {
	if _, ok := e.Curve.(surfacegeom.Line3); !ok || e.Start.Bound.Base() != 0 || e.End.Bound.Base() != 0 {
		return false
	}
	a, b := e.Start.Position, e.End.Position
	x, y := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}
	varying := 0
	for axis := range 3 {
		if x[axis] == y[axis] {
			if x[axis] != p.Box[axis][0] && x[axis] != p.Box[axis][1] {
				return false
			}
			continue
		}
		varying++
		if (x[axis] != p.Box[axis][0] || y[axis] != p.Box[axis][1]) &&
			(x[axis] != p.Box[axis][1] || y[axis] != p.Box[axis][0]) {
			return false
		}
	}
	return varying == 1
}

// BoxPlane returns the axis and side of the box plane f lies on, and whether
// f is exactly one of the box's four-cornered axis-aligned faces.
func (p AllEdgeChamferPayload) BoxPlane(f *topo.Face, plane surfacegeom.Plane) (int, int, bool) {
	if len(f.Loops) == 0 || len(f.Loops[0].CoEdges) != 4 {
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
	for _, ce := range f.Loops[0].CoEdges {
		if _, ok := ce.Edge.Curve.(surfacegeom.Line3); !ok || topobuild.CoEdgeStart(ce).Bound.Base() != 0 {
			return 0, 0, false
		}
		v := topobuild.CoEdgeStart(ce).Position
		c := [3]float64{v.X, v.Y, v.Z}
		if c[axis] != p.Box[axis][side] {
			return 0, 0, false
		}
		for j := range 3 {
			if j != axis && c[j] != p.Box[j][0] && c[j] != p.Box[j][1] {
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

func (p AllEdgeChamferPayload) polygons() []allEdgePolygon {
	var out []allEdgePolygon
	for axis := range 3 {
		b, c := (axis+1)%3, (axis+2)%3
		for side := range 2 {
			coords := [][3]float64{}
			for _, pair := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
				var q [3]float64
				q[axis] = p.Box[axis][side]
				q[b] = p.Box[b][pair[0]] + inward(pair[0], p.D)
				q[c] = p.Box[c][pair[1]] + inward(pair[1], p.D)
				coords = append(coords, q)
			}
			n := [3]float64{}
			n[axis] = float64(2*side - 1)
			widthB := proofbound.BoundedSub(
				proofbound.BoundedSub(proofbound.ExactScalar(p.Box[b][1]), proofbound.ExactScalar(p.Box[b][0])),
				proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.ExactScalar(p.D)))
			widthC := proofbound.BoundedSub(
				proofbound.BoundedSub(proofbound.ExactScalar(p.Box[c][1]), proofbound.ExactScalar(p.Box[c][0])),
				proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.ExactScalar(p.D)))
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
					v[b] = p.Box[b][sb] + inward(sb, ub)
					v[c] = p.Box[c][sc] + inward(sc, p.D-ub)
					return v
				}
				lo, hi := p.Box[axis][0], p.Box[axis][1]
				points := vecs([][3]float64{q(lo+p.D, 0), q(lo+p.D/2, p.D/2),
					q(lo+p.D, p.D), q(hi-p.D, p.D), q(hi-p.D/2, p.D/2), q(hi-p.D, 0)})
				n := [3]float64{}
				n[b], n[c] = float64(2*sb-1), float64(2*sc-1)
				length := proofbound.BoundedSub(proofbound.ExactScalar(hi), proofbound.ExactScalar(lo))
				d2 := proofbound.BoundedMul(proofbound.ExactScalar(p.D), proofbound.ExactScalar(p.D))
				span := proofbound.BoundedSub(proofbound.BoundedMul(length, proofbound.ExactScalar(p.D)),
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

// BuildAllEdgeChamfer evaluates p over topology nodes allocated through
// env.Alloc: the six inset box planes, the twelve bevel planes and the bore
// wall, with every measurement proven from the record.
func BuildAllEdgeChamfer(ctx context.Context, env topo.Env, p AllEdgeChamferPayload) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := env.Ref
	geometry, axisMap, err := p.PlacedGeometry()
	if err != nil {
		return nil, err
	}
	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySolid
	body.Doc = env.Doc
	polygons := geometry.polygons()
	vertices := map[r3.Vec]*topo.Vertex{}
	vertex := func(v r3.Vec) *topo.Vertex {
		if found := vertices[v]; found != nil {
			return found
		}
		made := topobuild.NewVertex(env.Alloc, topo.Vertex{Position: v, Bound: units.Millimeters(0)})
		vertices[v] = made
		return made
	}
	type lineKey struct{ a, b r3.Vec }
	lines := map[lineKey]*topo.Edge{}
	faces := make([]*topo.Face, 0, len(polygons)+1)
	for polygonIndex, poly := range polygons {
		points := slices.Clone(poly.points)
		if points[1].Sub(points[0]).Cross(points[2].Sub(points[0])).Dot(poly.normal) < 0 {
			slices.Reverse(points)
		}
		u := points[1].Sub(points[0])
		frame, err := r3.NewFrame(points[0], u, poly.normal.Cross(u))
		if err != nil {
			return nil, fmt.Errorf(`%w: all-edge chamfer plane: %s`, decaderr.ErrUnsupported, err)
		}
		face := topobuild.NewFace(env.Alloc, topo.Face{Area: poly.area.Value, AreaBound: poly.area.Bound, Surface: surfacegeom.Plane{Frame: frame}, Origins: []provenance.Ref{{Producer: ref, Role: axisMap.faceRole(polygonIndex)}}, Body: body})
		loop := topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true})
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
				e = topobuild.NewEdge(env.Alloc, topo.Edge{
					Convex:      true,
					Length:      length.Value,
					LengthBound: length.Bound,
					Curve:       surfacegeom.Line3{},
					Start:       vertex(a),
					End:         vertex(b),
				})
				lines[key] = e
			}
			loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: e, Forward: e.Start == vertex(a)})
		}
		face.Loops = []*topo.Loop{loop}
		faces = append(faces, face)
	}
	bore, err := geometry.addBoreTopology(env.Alloc, body, ref, faces, vertex)
	if err != nil {
		return nil, err
	}
	faces = append(faces, bore)
	if err := topobuild.AttachFaceLoops(ctx, faces); err != nil {
		return nil, err
	}
	for _, e := range lines {
		if len(e.Faces) != 2 {
			return nil, fmt.Errorf(`%w: all-edge chamfer has an unsewn planar edge`, decaderr.ErrUnsupported)
		}
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, faces)
	if len(body.Lumps) != 1 || body.Lumps[0].Shells[0].Open {
		return nil, fmt.Errorf(`%w: all-edge chamfer has an open shell`, decaderr.ErrUnsupported)
	}
	if err := geometry.measure(body, polygons); err != nil {
		return nil, err
	}
	body.Payload = p
	return body, nil
}

func (p AllEdgeChamferPayload) addBoreTopology(alloc topo.Alloc, body *topo.Body, ref provenance.Producer,
	faces []*topo.Face, vertex func(r3.Vec) *topo.Vertex) (*topo.Face, error) {
	if len(faces) < 4 {
		return nil, fmt.Errorf(`%w: the all-edge chamfer lacks its two bored faces`, decaderr.ErrUnsupported)
	}
	pi := stitchflux.PiScalar()
	r := proofbound.ExactScalar(p.Radius)
	disk := proofbound.BoundedMul(pi, proofbound.BoundedMul(r, r))
	height := proofbound.BoundedSub(proofbound.ExactScalar(p.Box[1][1]), proofbound.ExactScalar(p.Box[1][0]))
	wallArea := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), pi),
		proofbound.BoundedMul(r, height))
	bore := topobuild.NewFace(alloc, topo.Face{
		Reversed:  true,
		Area:      wallArea.Value,
		AreaBound: wallArea.Bound,
		Surface: surfacegeom.Cylinder{Origin: r3.NewVec(p.Center[0], p.Box[1][0], p.Center[1]),
			Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(p.Radius)},
		Origins: []provenance.Ref{{Producer: ref, Role: "wall(6)"}},
		Body:    body,
	})
	for side := range 2 {
		face := faces[2+side]
		area := proofbound.BoundedSub(proofbound.MeasuredScalar(face.Area, face.AreaBound), disk)
		face.Area, face.AreaBound = area.Value, area.Bound
		center := r3.NewVec(p.Center[0], p.Box[1][side], p.Center[1])
		seam := vertex(center.Add(r3.NewVec(p.Radius, 0, 0)))
		circumference := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), pi), r)
		edge := topobuild.NewEdge(alloc, topo.Edge{
			Convex:       false,
			Length:       circumference.Value,
			LengthBound:  circumference.Bound,
			CurveBounded: true,
			Curve: surfacegeom.Circle3{Center: center, Axis: r3.NewVec(0, -1, 0),
				Radius: units.Millimeters(p.Radius)},
			Start: seam,
			End:   seam,
		})
		face.Loops = append(face.Loops, topobuild.NewLoop(alloc, topo.Loop{CoEdges: []topo.CoEdge{{Edge: edge, Forward: side == 1}}}))
		bore.Loops = append(bore.Loops, topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edge, Forward: side == 0}}}))
	}
	return bore, nil
}

func (p AllEdgeChamferPayload) measure(body *topo.Body, polygons []allEdgePolygon) error {
	lx := proofbound.BoundedSub(proofbound.ExactScalar(p.Box[0][1]), proofbound.ExactScalar(p.Box[0][0]))
	ly := proofbound.BoundedSub(proofbound.ExactScalar(p.Box[1][1]), proofbound.ExactScalar(p.Box[1][0]))
	lz := proofbound.BoundedSub(proofbound.ExactScalar(p.Box[2][1]), proofbound.ExactScalar(p.Box[2][0]))
	d := proofbound.ExactScalar(p.D)
	r := proofbound.ExactScalar(p.Radius)
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
		return fmt.Errorf(`%w: the all-edge chamfer has no proven positive volume`, decaderr.ErrUnsupported)
	}
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume.Value), Bound: units.CubicMillimeters(volume.Bound),
		Exactness: measurement.ExactnessOf(volume.Bound)}
	area := proofbound.ExactScalar(0)
	for _, poly := range polygons {
		area = proofbound.BoundedAdd(area, poly.area)
	}
	disks := proofbound.BoundedMul(proofbound.ExactScalar(2),
		proofbound.BoundedMul(pi, proofbound.BoundedMul(r, r)))
	wall := proofbound.BoundedMul(proofbound.ExactScalar(2),
		proofbound.BoundedMul(pi, proofbound.BoundedMul(r, ly)))
	area = proofbound.BoundedAdd(proofbound.BoundedSub(area, disks), wall)
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(area.Value), Bound: units.SquareMillimeters(area.Bound),
		Exactness: measurement.ExactnessOf(area.Bound)}
	// The admitted bore is centered in x and z and traverses the full y range.
	// The twelve edge wedges are symmetric about all three box midplanes.
	if !rationalMidpointEquals(p.Box[0], p.Center[0]) ||
		!rationalMidpointEquals(p.Box[2], p.Center[1]) {
		return fmt.Errorf(`%w: the all-edge chamfer requires a centered bore`, decaderr.ErrUnsupported)
	}
	var midpoint [3]proofbound.BoundedScalar
	centroidBound := 0.0
	for axis := range 3 {
		midpoint[axis] = proofbound.BoundedMul(proofbound.ExactScalar(0.5),
			proofbound.BoundedAdd(proofbound.ExactScalar(p.Box[axis][0]), proofbound.ExactScalar(p.Box[axis][1])))
		centroidBound = math.Max(centroidBound, midpoint[axis].Bound)
	}
	centroid := r3.NewVec(midpoint[0].Value, midpoint[1].Value, midpoint[2].Value)
	body.Centroid = measurement.VecMeasurement{Value: centroid, Bound: units.Millimeters(centroidBound),
		Exactness: measurement.ExactnessOf(centroidBound)}
	body.Bounds = measurement.Box{Min: r3.NewVec(p.Box[0][0], p.Box[1][0], p.Box[2][0]),
		Max:   r3.NewVec(p.Box[0][1], p.Box[1][1], p.Box[2][1]),
		Bound: units.Millimeters(0), Exactness: measurement.Exact}
	return topobuild.ValidateAnalyticMeasurements(body)
}
