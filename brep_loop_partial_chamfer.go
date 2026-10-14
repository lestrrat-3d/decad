package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type partialChamferQuad struct {
	points [4][3]float64 // side start/end, cap end/start, in reference coordinates
	cap    *Edge
	side   *Edge
}

// attachPartialChamferBand builds one concave root or two convex outer strips.
// Every patch vertex is already an open edge of the rewritten face record.
func attachPartialChamferBand(ctx context.Context, body *Body, ref producerID,
	bp brepPayload, bi int, open brepOpenSet, e brepEmbed) ([]*Face, brepBandMass, error) {
	if err := ctx.Err(); err != nil {
		return nil, brepBandMass{}, err
	}
	b := bp.loopBands[bi]
	f := bp.faces[b.face]
	selectedCount := 0
	for _, on := range b.selected {
		if on {
			selectedCount++
		}
	}
	outerPair := b.loop == 0 && b.sigma == -1 && selectedCount == 2
	rootSingle := b.loop > 0 && b.sigma == 1 && selectedCount == 1
	if (!outerPair && !rootSingle) || f.delta != 0 || b.setback.dc != b.setback.ds ||
		b.setback.dcDelta != 0 ||
		b.setback.dsDelta != 0 {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer requires an exact outer pair or inner root`, ErrUnsupported)
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
		if err := partialband.ChamferQuadExact(quads[i].points, b.setback.dc, b.setback.ds, e); err != nil {
			return nil, brepBandMass{}, err
		}
		sideOrdinal++
	}
	if sideOrdinal != selectedCount || sideOrdinal != len(open.side[bi]) ||
		len(open.terminal[bi]) != 2 {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial chamfer does not have one or two strips and two terminal lines`, ErrUnsupported)
	}
	lead, trail := make([]*Edge, len(quads)), make([]*Edge, len(quads))
	for i, on := range b.selected {
		if !on {
			continue
		}
		prev, next := (i+len(quads)-1)%len(quads), (i+1)%len(quads)
		if b.selected[prev] {
			if quads[prev].cap.n.End != quads[i].cap.n.Start ||
				quads[prev].side.n.End != quads[i].side.n.Start ||
				quads[prev].points[2] != quads[i].points[3] ||
				quads[prev].points[1] != quads[i].points[0] {
				return nil, brepBandMass{}, fmt.Errorf(`%w: two partial chamfer strips disagree at their shared corner`, ErrUnsupported)
			}
			slant, _, err := capSlantEdge(proofbound.NewWorkBudget(ctx),
				planePoint{U: e.Local(quads[i].points[3])[0], V: e.Local(quads[i].points[3])[1]},
				vertexHandle(quads[i].cap.n.Start), vertexHandle(quads[i].side.n.Start),
				e.Local(quads[i].points[0])[0], e.Local(quads[i].points[0])[1],
				f.z0, sideZ, 0, 0, survey2d.SideWalk{}, survey2d.SideWalk{}, b.setback, true)
			if err != nil {
				return nil, brepBandMass{}, err
			}
			lead[i], trail[i] = slant, slant
		}
		if !b.selected[prev] {
			lead[i] = partialChamferTerminal(open.terminal[bi], vertexHandle(quads[i].cap.n.Start), vertexHandle(quads[i].side.n.Start))
		}
		if !b.selected[next] {
			trail[next] = partialChamferTerminal(open.terminal[bi], vertexHandle(quads[i].cap.n.End), vertexHandle(quads[i].side.n.End))
		}
	}
	var patches []*Face
	mass := zeroBrepBandMass()
	capSign := -1.0
	if f.outward {
		capSign = 1
	}
	outwardCap := f.view(bp.xform).Dir(0, 0, capSign)
	for i, on := range b.selected {
		if !on {
			continue
		}
		next := (i + 1) % len(quads)
		start, end := lead[i], trail[next]
		capE, sideE := quads[i].cap, quads[i].side
		if start == nil || end == nil || !edgeConnects(start, vertexHandle(capE.n.Start), vertexHandle(sideE.n.Start)) ||
			!edgeConnects(end, vertexHandle(capE.n.End), vertexHandle(sideE.n.End)) {
			return nil, brepBandMass{}, fmt.Errorf(`%w: partial chamfer patch %d does not close`, ErrUnsupported, i)
		}
		plane, err := planeFromThree(sideE.n.Start.Position, sideE.n.End.Position, capE.n.End.Position)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		loop := &Loop{coedges: []coedge{{edge: sideE, forward: true},
			{edge: end, forward: end.n.Start == sideE.n.End}, {edge: capE, forward: false},
			{edge: start, forward: start.n.Start == capE.n.Start}}, n: topo.Loop{Outer: true}}
		areaIv, volume3, moments, err := partialband.ChamferQuadMass(quads[i].points)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		held, bound := heldOf(areaIv)
		face := bornFace(&Face{loops: []*Loop{loop}, n: topo.Face{Area: held, AreaBound: bound, Surface: plane, Origins: []provenance.Ref{{Producer: ref, Role: b.patchRole(len(patches))}}, Body: bodyNode(body)}})
		normal, err := face.NormalAt(sideE.n.Start.Position)
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
			face.n.Reversed = true
			volume3.Neg(volume3)
			for k := range moments {
				moments[k].Neg(moments[k])
			}
		}
		for _, ce := range loop.coedges {
			ce.edge.n.Faces = append(ce.edge.n.Faces, faceNode(face))
		}
		patches = append(patches, face)
		mass.area = proofbound.BoundedAdd(mass.area, proofbound.MeasuredScalar(held, bound))
		mass.vol3 = proofbound.IntervalAdd(mass.vol3, proofbound.PointInterval(volume3))
		for k := range 3 {
			mass.moments[k] = proofbound.IntervalAdd(mass.moments[k], proofbound.PointInterval(moments[k]))
		}
		capE.n.Convex, sideE.n.Convex = true, true
	}
	return patches, mass, nil
}

func partialChamferTerminal(terminals map[brepBandTerminal]*Edge, capVertex, side *Vertex) *Edge {
	for _, edge := range terminals {
		if edgeConnects(edge, capVertex, side) {
			return edge
		}
	}
	return nil
}
