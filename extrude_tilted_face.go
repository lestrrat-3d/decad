package decad

import (
	"context"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// tiltedStopMinCosine is the least cosine between the stop face's normal and the sweep direction for which the stop is
// built: a face nearer to parallel with the sweep reaches the sketch plane too fast for the cut to stay well conditioned.
const tiltedStopMinCosine = 1e-3

// tiltedStopMargin is the share of the sketch footprint's size by which the footprint must lie inside the stop face's
// outline, seen along the sweep.
const tiltedStopMargin = 1e-6

// extrudeToTiltedFace extrudes the profile up to a planar face of a faceted solid with exact vertices that is tilted
// from the sweep (agent-docs/orthogonal-boolean-design.md B19): the profile is extruded past the face and the part
// beyond the face is cut away by a prism standing on the face's own outline. The face's vertices are exact and
// coplanar, and the tool's bottom is that polygon, so the cut plane is the face's plane with no rounding to charge.
// The face must be a convex polygon with one loop, wholly on one side of the sketch plane, and its outline, seen along
// the sweep, must hold the whole footprint with a margin, so the tool's walls never meet the extrusion. handled is
// false, with no error, when the route does not apply, and the caller keeps its refusal.
func extrudeToTiltedFace(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, tf ToFace,
	opts []ExtrudeOption) (result *Body, handled bool, err error) {
	if tf.Offset != (units.Value{}) {
		return nil, false, nil
	}
	cfg, err := decodeExtrudeOptions(opts)
	if err != nil || cfg.Taper.Mag() != 0 || cfg.SurfaceResult {
		return nil, false, nil
	}
	stop, err := d.resolveStopBody(tf.Body, "a to-face extent")
	if err != nil {
		return nil, false, nil
	}
	if _, ok := orthogonalOperand(stop); !ok {
		return nil, false, nil
	}
	face, err := selectImpliedOneFace(stop, tf.Face, "the stop")
	if err != nil {
		return nil, false, nil
	}
	_, plane, _, certificate, err := sectioncert.RecordExtrudeProfile(s, p)
	if err != nil {
		return nil, false, nil
	}
	var frame r3.Frame
	if certificate != nil {
		frame = certificate.Frame
	} else if frame, err = r3.NewFrame(plane.Origin, plane.U, plane.V); err != nil {
		return nil, false, nil
	}
	var points []r3.Vec
	if len(face.Loops()) != 1 {
		return nil, false, nil
	}
	for _, e := range face.Edges() {
		if e.Curve() == nil || e.Start() == nil || e.End() == nil {
			return nil, false, nil
		}
		points = append(points, e.Start().Position().Value, e.End().Position().Value)
	}
	poly, ok := convexExactPolygon(points)
	if !ok {
		return nil, false, nil
	}
	sweep := frame.N()
	origin := frame.Origin()
	travel, minS, maxS, ok := polygonSide(poly, origin, sweep)
	if !ok {
		return nil, false, nil
	}
	if math.Abs(polygonNormal(poly).Dot(sweep)) < tiltedStopMinCosine {
		return nil, false, nil
	}
	length := maxS*1.25 + (maxS - minS)
	way := Along
	if travel < 0 {
		way = Against
	}
	toolCtx, capture := privateConstructionContext(ctx)
	var made []*Body
	defer func() {
		if !handled {
			removeTools(made)
		}
	}()
	base, err := Extrude(ctx, d, s, p, Distance{D: units.Millimeters(length), Dir: way})
	if err != nil {
		return nil, false, nil
	}
	made = append(made, base)
	box, err := base.Bounds()
	if err != nil || !footprintInside(poly, frame, box.Min, box.Max, box.Bound.Base(), length) {
		return nil, false, nil
	}
	verts, tris := tiltedStopTool(poly, sweep.Scale(travel), 2*length)
	m, err := NewMeshBody(toolCtx, verts, tris)
	if err != nil {
		return nil, false, nil
	}
	tool, err := SolidFromMesh(toolCtx, d, m)
	if err != nil {
		return nil, false, nil
	}
	made = append(made, tool)
	out, err := combineTools(ctx, d, OperationExtrude, base, meshbool.OpCut, []*Body{tool}, false, nil, capture)
	if err != nil {
		return nil, false, nil
	}
	removeTools([]*Body{tool})
	return out, true, nil
}

// convexExactPolygon orders the distinct points as a convex polygon, reporting false unless there are at least three,
// they are exactly coplanar, and every turn of the ordered outline is the same way. The order is by angle about the
// centroid in the points' plane and then verified exactly.
func convexExactPolygon(points []r3.Vec) ([]r3.Vec, bool) {
	var uniq []r3.Vec
	for _, p := range points {
		dup := false
		for _, q := range uniq {
			dup = dup || p == q
		}
		if !dup {
			uniq = append(uniq, p)
		}
	}
	if len(uniq) < 3 {
		return nil, false
	}
	rat := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	}
	sub := func(a, b [3]*big.Rat) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
	}
	cross := func(u, v [3]*big.Rat) [3]*big.Rat {
		t := func(a, b, c, d *big.Rat) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(a, b), new(big.Rat).Mul(c, d))
		}
		return [3]*big.Rat{t(u[1], v[2], u[2], v[1]), t(u[2], v[0], u[0], v[2]), t(u[0], v[1], u[1], v[0])}
	}
	dot := func(u, v [3]*big.Rat) *big.Rat {
		sum := new(big.Rat)
		for i := range u {
			sum.Add(sum, new(big.Rat).Mul(u[i], v[i]))
		}
		return sum
	}
	r0 := rat(uniq[0])
	var normal [3]*big.Rat
	for i := 1; i < len(uniq) && normal[0] == nil; i++ {
		for j := i + 1; j < len(uniq); j++ {
			n := cross(sub(rat(uniq[i]), r0), sub(rat(uniq[j]), r0))
			if n[0].Sign() != 0 || n[1].Sign() != 0 || n[2].Sign() != 0 {
				normal = n
				break
			}
		}
	}
	if normal[0] == nil {
		return nil, false
	}
	for _, p := range uniq {
		if dot(normal, sub(rat(p), r0)).Sign() != 0 {
			return nil, false
		}
	}
	nf := ratVec(normal)
	nf = nf.Scale(1 / nf.Len())
	var centre r3.Vec
	for _, p := range uniq {
		centre = centre.Add(p)
	}
	centre = centre.Scale(1 / float64(len(uniq)))
	u := uniq[0].Sub(centre)
	u = u.Sub(nf.Scale(u.Dot(nf)))
	if !(u.Len() > 0) {
		return nil, false
	}
	u = u.Scale(1 / u.Len())
	v := nf.Cross(u)
	angle := func(p r3.Vec) float64 {
		w := p.Sub(centre)
		return math.Atan2(w.Dot(v), w.Dot(u))
	}
	sort.Slice(uniq, func(i, j int) bool { return angle(uniq[i]) < angle(uniq[j]) })
	var sign int
	for i := range uniq {
		a, b, c := rat(uniq[i]), rat(uniq[(i+1)%len(uniq)]), rat(uniq[(i+2)%len(uniq)])
		turn := dot(normal, cross(sub(b, a), sub(c, b))).Sign()
		switch {
		case turn == 0:
		case sign == 0:
			sign = turn
		case sign != turn:
			return nil, false
		}
	}
	if sign == 0 {
		return nil, false
	}
	return uniq, true
}

// ratVec is the nearest float vector to the rational triple.
func ratVec(q [3]*big.Rat) r3.Vec {
	x, _ := q[0].Float64()
	y, _ := q[1].Float64()
	z, _ := q[2].Float64()
	return r3.NewVec(x, y, z)
}

// polygonSide reports on which side of the sketch plane the polygon lies, as +1 or −1, with the least and greatest
// distance of a vertex along that side. ok is false unless every vertex is strictly on one side.
func polygonSide(poly []r3.Vec, origin, sweep r3.Vec) (sign, least, greatest float64, ok bool) {
	pos, neg := false, false
	least, greatest = math.Inf(1), 0
	for _, p := range poly {
		h := p.Sub(origin).Dot(sweep)
		pos, neg = pos || h > 0, neg || h < 0
		least, greatest = math.Min(least, math.Abs(h)), math.Max(greatest, math.Abs(h))
	}
	if pos == neg || !(least > 0) {
		return 0, 0, 0, false
	}
	if pos {
		return 1, least, greatest, true
	}
	return -1, least, greatest, true
}

// polygonNormal is the unit normal of the polygon's plane, from its first three points.
func polygonNormal(poly []r3.Vec) r3.Vec {
	n := poly[1].Sub(poly[0]).Cross(poly[2].Sub(poly[1]))
	for i := 1; n.Len() == 0 && i+2 < len(poly); i++ {
		n = poly[i+1].Sub(poly[i]).Cross(poly[i+2].Sub(poly[i+1]))
	}
	return n.Scale(1 / n.Len())
}

// footprintInside reports whether the box, seen along the sweep, lies inside the polygon by a margin of the box's own
// bound and a share of its size. The footprint of a prism is inside its bounding box, so a box inside the polygon holds
// the footprint inside it, and the polygon is convex, so its corners decide.
func footprintInside(poly []r3.Vec, frame r3.Frame, lo, hi r3.Vec, bound, length float64) bool {
	u, v, origin := frame.U(), frame.V(), frame.Origin()
	to2 := func(p r3.Vec) [2]float64 {
		w := p.Sub(origin)
		return [2]float64{w.Dot(u), w.Dot(v)}
	}
	var outline [][2]float64
	for _, p := range poly {
		outline = append(outline, to2(p))
	}
	area := 0.0
	for i := range outline {
		a, b := outline[i], outline[(i+1)%len(outline)]
		area += a[0]*b[1] - a[1]*b[0]
	}
	if area == 0 {
		return false
	}
	orient := 1.0
	if area < 0 {
		orient = -1
	}
	size := math.Max(hi.Sub(lo).Len(), length)
	margin := bound + tiltedStopMargin*size
	for _, x := range []float64{lo.X, hi.X} {
		for _, y := range []float64{lo.Y, hi.Y} {
			for _, z := range []float64{lo.Z, hi.Z} {
				c := to2(r3.NewVec(x, y, z))
				for i := range outline {
					a, b := outline[i], outline[(i+1)%len(outline)]
					e := [2]float64{b[0] - a[0], b[1] - a[1]}
					l := math.Hypot(e[0], e[1])
					if l == 0 {
						continue
					}
					if orient*(e[0]*(c[1]-a[1])-e[1]*(c[0]-a[0]))/l < margin {
						return false
					}
				}
			}
		}
	}
	return true
}

// tiltedStopTool is the closed outward triangle mesh of the prism that stands on the polygon and runs height along up.
// The polygon is wound so its normal looks along up; the bottom is the polygon itself, with exact vertices.
func tiltedStopTool(poly []r3.Vec, up r3.Vec, height float64) ([]r3.Vec, [][3]int) {
	ring := append([]r3.Vec(nil), poly...)
	area := r3.Vec{}
	for i := 1; i+1 < len(ring); i++ {
		area = area.Add(ring[i].Sub(ring[0]).Cross(ring[i+1].Sub(ring[0])))
	}
	if area.Dot(up) < 0 {
		for i, j := 0, len(ring)-1; i < j; i, j = i+1, j-1 {
			ring[i], ring[j] = ring[j], ring[i]
		}
	}
	k := len(ring)
	verts := append([]r3.Vec(nil), ring...)
	for _, p := range ring {
		verts = append(verts, p.Add(up.Scale(height)))
	}
	var tris [][3]int
	for i := 1; i+1 < k; i++ {
		tris = append(tris, [3]int{0, i + 1, i}, [3]int{k, k + i, k + i + 1})
	}
	for i := range k {
		j := (i + 1) % k
		tris = append(tris, [3]int{i, j, k + j}, [3]int{i, k + j, k + i})
	}
	return verts, tris
}
