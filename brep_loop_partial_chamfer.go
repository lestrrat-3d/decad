package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type partialChamferQuad struct {
	points [4][3]float64 // side start/end, cap end/start, in reference coordinates
	cap    *Edge
	side   *Edge
}

// attachPartialChamferBand builds the two selected straight-walk strips.
// Every patch vertex is already an open edge of the rewritten face record.
func attachPartialChamferBand(ctx context.Context, body *Body, ref producerID,
	bp brepPayload, bi int, open brepOpenSet, e brepEmbed) ([]*Face, brepBandMass, error) {
	if err := ctx.Err(); err != nil {
		return nil, brepBandMass{}, err
	}
	b := bp.loopBands[bi]
	f := bp.faces[b.face]
	if b.loop != 0 || b.sigma != -1 || f.delta != 0 || b.setback.dc != b.setback.ds ||
		b.setback.dcDelta != 0 ||
		b.setback.dsDelta != 0 {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer requires an exact convex outer contour`, ErrUnsupported)
	}
	sideZ, sideDelta := b.sideLevel(f)
	if sideDelta != 0 {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer's side level is displaced`, ErrUnsupported)
	}
	work := freeform.NewFreeformWork()
	quads := make([]partialChamferQuad, len(b.selected))
	sideOrdinal := 0
	for i, on := range b.selected {
		if !on {
			continue
		}
		if sideOrdinal >= len(open.side[bi]) || b.capWalk[i] < 0 {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer has no selected contour edge`, ErrUnsupported)
		}
		ow, err := boundarywalk.WalkOf(b.orig.Segments[i], work)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		cw, err := boundarywalk.WalkOf(f.regionLoop(b.loop).Segments[b.capWalk[i]], work)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		if !ow.IsLine() || !cw.IsLine() {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer has a curved selected walk`, ErrUnsupported)
		}
		capEdge := open.capBySeg[bi][b.capWalk[i]]
		sideEdge := open.side[bi][sideOrdinal].edge
		if capEdge == nil || sideEdge == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer lacks a band edge`, ErrUnsupported)
		}
		quads[i] = partialChamferQuad{
			points: [4][3]float64{
				e.Canon(ow.StartU, ow.StartV, sideZ), e.Canon(ow.EndU, ow.EndV, sideZ),
				e.Canon(cw.EndU, cw.EndV, f.z0), e.Canon(cw.StartU, cw.StartV, f.z0),
			}, cap: capEdge, side: sideEdge,
		}
		if err := partialChamferExactQuad(quads[i], b.setback, e); err != nil {
			return nil, brepBandMass{}, err
		}
		sideOrdinal++
	}
	if sideOrdinal != 2 || sideOrdinal != len(open.side[bi]) || len(open.terminal[bi]) != 2 {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer does not have two strips and two terminal lines`, ErrUnsupported)
	}
	lead, trail := make([]*Edge, len(quads)), make([]*Edge, len(quads))
	for i, on := range b.selected {
		if !on {
			continue
		}
		prev, next := (i+len(quads)-1)%len(quads), (i+1)%len(quads)
		if b.selected[prev] {
			if quads[prev].cap.end != quads[i].cap.start ||
				quads[prev].side.end != quads[i].side.start ||
				quads[prev].points[2] != quads[i].points[3] ||
				quads[prev].points[1] != quads[i].points[0] {
				return nil, brepBandMass{}, fmt.Errorf(`%w: two partial chamfer strips disagree at their shared corner`, ErrUnsupported)
			}
			slant, _, err := capSlantEdge(proofbound.NewWorkBudget(ctx),
				Point2{U: e.Local(quads[i].points[3])[0], V: e.Local(quads[i].points[3])[1]},
				quads[i].cap.start, quads[i].side.start,
				e.Local(quads[i].points[0])[0], e.Local(quads[i].points[0])[1],
				f.z0, sideZ, 0, 0, survey2d.SideWalk{}, survey2d.SideWalk{}, b.setback, true)
			if err != nil {
				return nil, brepBandMass{}, err
			}
			lead[i], trail[i] = slant, slant
		}
		if !b.selected[prev] {
			lead[i] = partialChamferTerminal(open.terminal[bi], quads[i].cap.start, quads[i].side.start)
		}
		if !b.selected[next] {
			trail[next] = partialChamferTerminal(open.terminal[bi], quads[i].cap.end, quads[i].side.end)
		}
	}
	var patches []*Face
	mass := zeroBrepBandMass()
	outwardCap := f.view(bp.xform).dir(0, 0, -b.matSign(f))
	for i, on := range b.selected {
		if !on {
			continue
		}
		next := (i + 1) % len(quads)
		start, end := lead[i], trail[next]
		capE, sideE := quads[i].cap, quads[i].side
		if start == nil || end == nil || !edgeConnects(start, capE.start, sideE.start) ||
			!edgeConnects(end, capE.end, sideE.end) {
			return nil, brepBandMass{}, fmt.Errorf(`%w: partial chamfer patch %d does not close`, ErrUnsupported, i)
		}
		plane, err := planeFromThree(sideE.start.position, sideE.end.position, capE.end.position)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		loop := &Loop{coedges: []coedge{{edge: sideE, forward: true},
			{edge: end, forward: end.start == sideE.end}, {edge: capE, forward: false},
			{edge: start, forward: start.start == capE.start}}, outer: true}
		areaIv, volume3, moments, err := partialChamferPolygonMass(quads[i].points)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		held, bound := heldOf(areaIv)
		face := &Face{surface: plane, origins: []FeatureRef{{producer: ref, Role: b.patchRole(len(patches))}},
			body: body, loops: []*Loop{loop}, area: held, areaBound: bound}
		normal, err := face.NormalAt(sideE.start.position)
		if err != nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: partial chamfer patch %d has no normal: %v`,
				ErrUnsupported, i, err)
		}
		// The recorded loop can wind either way on opposite cap faces. Its
		// divergence terms follow that winding, while the patch normal must
		// always point from material toward the cap side of the removed wedge.
		aligned := normal.Value.Dot(outwardCap)
		if math.Abs(aligned) < 0.5 {
			return nil, brepBandMass{}, fmt.Errorf(`%w: partial chamfer patch %d has ambiguous outward sense`,
				ErrUnsupported, i)
		}
		if aligned < 0 {
			face.reversed = true
			volume3.Neg(volume3)
			for k := range moments {
				moments[k].Neg(moments[k])
			}
		}
		for _, ce := range loop.coedges {
			ce.edge.faces = append(ce.edge.faces, face)
		}
		patches = append(patches, face)
		mass.area = proofbound.BoundedAdd(mass.area, proofbound.MeasuredScalar(held, bound))
		mass.vol3 = proofbound.IntervalAdd(mass.vol3, proofbound.PointInterval(volume3))
		for k := range 3 {
			mass.moments[k] = proofbound.IntervalAdd(mass.moments[k], proofbound.PointInterval(moments[k]))
		}
		capE.convex, sideE.convex = true, true
	}
	return patches, mass, nil
}

func partialChamferTerminal(terminals map[brepBandTerminal]*Edge, cap, side *Vertex) *Edge {
	for _, edge := range terminals {
		if edgeConnects(edge, cap, side) {
			return edge
		}
	}
	return nil
}

// partialChamferExactQuad restricts the admission to exactly coplanar
// recorded floats. A nonzero contour/level displacement is refused above.
func partialChamferExactQuad(q partialChamferQuad, setback capSetback, e brepEmbed) error {
	r := partialChamferRats(q.points)
	a, b, c, d := r[0], r[1], r[2], r[3]
	ab, ac, ad := partialChamferSub(b, a), partialChamferSub(c, a), partialChamferSub(d, a)
	n := partialChamferCross(ab, ac)
	if partialChamferDot(n, ad).Sign() != 0 || partialChamferDot(n, n).Sign() == 0 {
		return fmt.Errorf(`%w: a partial chamfer patch is not one exact plane`, ErrUnsupported)
	}
	// A concave or self-crossing quadrilateral would make the two-triangle
	// area and divergence fan differ from the patch's actual boundary.
	second := partialChamferCross(partialChamferSub(c, a), ad)
	if partialChamferDot(n, second).Sign() <= 0 {
		return fmt.Errorf(`%w: a partial chamfer patch has inconsistent triangle winding`, ErrUnsupported)
	}
	// The cap line must be the exact offset of the side carrier. Its
	// endpoints may have different travel coordinates after a miter, so
	// compare the two line equations rather than endpoint differences.
	local := [4][3]*big.Rat{}
	for i, p := range q.points {
		v := e.Local(p)
		for k := range 3 {
			local[i][k] = proofarith.FloatRat(v[k])
		}
	}
	travel := partialChamferSub(local[1], local[0])
	if travel[2].Sign() != 0 || (travel[0].Sign() != 0) == (travel[1].Sign() != 0) {
		return fmt.Errorf(`%w: a partial chamfer's selected line is not along a face axis`, ErrUnsupported)
	}
	length := new(big.Rat).Abs(travel[0])
	if length.Sign() == 0 {
		length.Abs(travel[1])
	}
	cross2 := func(v [3]*big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(travel[0], v[1]),
			new(big.Rat).Mul(travel[1], v[0]))
	}
	dc := proofarith.FloatRat(setback.dc)
	want := new(big.Rat).Mul(dc, length)
	for _, capPoint := range [][3]*big.Rat{local[2], local[3]} {
		if cross2(partialChamferSub(capPoint, local[0])).Cmp(want) != 0 {
			return fmt.Errorf(`%w: a partial chamfer's cap line is not the exact requested offset`, ErrUnsupported)
		}
	}
	level := new(big.Rat).Abs(new(big.Rat).Sub(local[0][2], local[2][2]))
	if level.Cmp(proofarith.FloatRat(setback.ds)) != 0 {
		return fmt.Errorf(`%w: a partial chamfer's side level is not the exact requested setback`, ErrUnsupported)
	}
	return nil
}

func partialChamferRats(points [4][3]float64) [4][3]*big.Rat {
	var out [4][3]*big.Rat
	for i := range 4 {
		for k := range 3 {
			out[i][k] = proofarith.FloatRat(points[i][k])
		}
	}
	return out
}

func partialChamferSub(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for k := range 3 {
		out[k] = new(big.Rat).Sub(a[k], b[k])
	}
	return out
}

func partialChamferCross(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

func partialChamferDot(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for k := range 3 {
		out.Add(out, new(big.Rat).Mul(a[k], b[k]))
	}
	return out
}

// partialChamferPolygonMass returns one quad's area, three-times-volume
// flux and first-moment flux in the loop's geometric winding.
func partialChamferPolygonMass(points [4][3]float64) (proofbound.RatInterval, *big.Rat, [3]*big.Rat, error) {
	r := partialChamferRats(points)
	vol3 := new(big.Rat)
	var moments [3]*big.Rat
	for k := range 3 {
		moments[k] = new(big.Rat)
	}
	for _, pair := range [][2]int{{1, 2}, {2, 3}} {
		a, b, c := r[0], r[pair[0]], r[pair[1]]
		cross := partialChamferCross(partialChamferSub(b, a), partialChamferSub(c, a))
		centroid3 := [3]*big.Rat{}
		for k := range 3 {
			centroid3[k] = new(big.Rat).Add(a[k], b[k])
			centroid3[k].Add(centroid3[k], c[k])
		}
		vol3.Add(vol3, new(big.Rat).Quo(partialChamferDot(centroid3, cross), big.NewRat(6, 1)))
		for k := range 3 {
			sum := new(big.Rat)
			for _, p := range [][2]int{{0, 0}, {1, 1}, {2, 2}, {0, 1}, {1, 2}, {2, 0}} {
				v := [3]*big.Rat{a[k], b[k], c[k]}
				sum.Add(sum, new(big.Rat).Mul(v[p[0]], v[p[1]]))
			}
			moments[k].Add(moments[k], new(big.Rat).Quo(new(big.Rat).Mul(cross[k], sum), big.NewRat(24, 1)))
		}
	}
	// The two fan triangles lie in the same plane and share orientation.
	// Their area vectors are parallel, so the total area is half the sum of
	// their exact cross-vector lengths. For a trapezoid those vectors are
	// proportional, and each root is enclosed independently.
	area := proofbound.PointInterval(new(big.Rat))
	for _, pair := range [][2]int{{1, 2}, {2, 3}} {
		cross := partialChamferCross(partialChamferSub(r[pair[0]], r[0]), partialChamferSub(r[pair[1]], r[0]))
		root, ok := proofbound.SqrtFixed(partialChamferDot(cross, cross))
		if !ok {
			return proofbound.RatInterval{}, nil, [3]*big.Rat{}, fmt.Errorf(`%w: a chamfer patch has no area`, ErrUnsupported)
		}
		area = proofbound.IntervalAdd(area, proofbound.IntervalScale(root, big.NewRat(1, 2)))
	}
	return area, vol3, moments, nil
}
