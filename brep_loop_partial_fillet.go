package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// attachPartialFilletBand closes a selected straight-edge chain against its
// open cap and side contours and the terminal arcs in the record's end faces.
func attachPartialFilletBand(ctx context.Context, body *Body, ref producerID, bp brepPayload, bi int,
	open brepOpenSet, e brepEmbed, work *freeform.FreeformWork) ([]*Face, brepBandMass, error) {
	b := bp.loopBands[bi]
	f := bp.faces[b.face]
	pl := f.view(bp.xform)
	r := b.setback.Dc
	rIv, err := filletband.RadiusInterval(b.setback.Dc, b.setback.DcDelta)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	fr, err := filletLoopOf(proofbound.NewWorkBudget(ctx), b.orig, r, f.role, work)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	if len(fr.walks) != len(b.selected) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet band's walks do not match its selection`, ErrUnsupported)
	}
	pieces, err := fr.loop.PiecesSelected(b.selected)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	n := len(fr.walks)
	capEdges, side := make([]*Edge, n), make([]*Edge, n)
	si := 0
	for i, on := range b.selected {
		if !on {
			continue
		}
		if si >= len(open.side[bi]) {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet band lacks a selected contour edge`, ErrUnsupported)
		}
		capEdges[i], side[i] = open.capBySeg[bi][b.capWalk[i]], edgeHandle(open.side[bi][si].Edge)
		if b.capWalk[i] >= 0 && capEdges[i] == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet band lacks a selected cap edge`, ErrUnsupported)
		}
		si++
	}
	if si != len(open.side[bi]) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet band has extra contour edges`, ErrUnsupported)
	}
	convex := b.sigma < 0
	up := pl.Dir(0, 0, -b.matSign(f))
	sideZ, _ := b.sideLevel(f)
	lead, trail := make([]*Edge, n), make([]*Edge, n)
	capAt := func(k int) *Vertex {
		if capEdges[k] != nil {
			return vertexHandle(capEdges[k].n.Start)
		}
		if capEdges[(k+n-1)%n] != nil {
			return vertexHandle(capEdges[(k+n-1)%n].n.End)
		}
		return nil
	}
	meridianLen, meridianBound := heldOf(filletband.MeridianLength(rIv))
	meridian := func(foot planePoint, vertex planePoint, w survey2d.SideWalk, capV, sideV *Vertex) *Edge {
		nu, nv := foot.U-vertex.U, foot.V-vertex.V
		length := math.Hypot(nu, nv)
		normal := pl.Dir(nu/length, nv/length, 0)
		axis, _ := normal.Cross(up).Normalize()
		tu, tv := walkTangent(w)
		tangent := pl.Dir(tu, tv, 0)
		if axis.Dot(tangent) < 0 {
			tangent = tangent.Scale(-1)
		}
		return bornEdge(&Edge{n: topo.Edge{
			Convex:      convex,
			Length:      meridianLen,
			LengthBound: meridianBound,
			Curve: surfacegeom.Arc3{Center: pl.Point(foot.U, foot.V, sideZ), Axis: tangent,
				Radius: units.Millimeters(r)},
			Start: vertexNode(capV),
			End:   vertexNode(sideV),
		}})
	}
	for k := range n {
		prev := (k + n - 1) % n
		if !b.selected[prev] && !b.selected[k] {
			continue
		}
		var capV, sideV *Vertex
		if b.selected[k] {
			capV, sideV = capAt(k), vertexHandle(side[k].n.Start)
		} else {
			capV, sideV = vertexHandle(capEdges[prev].n.End), vertexHandle(side[prev].n.End)
		}
		if capV == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet corner has no cap vertex`, ErrUnsupported)
		}
		if b.selected[prev] != b.selected[k] {
			for _, terminal := range open.terminal[bi] {
				if (vertexHandle(terminal.n.Start) == capV && vertexHandle(terminal.n.End) == sideV) ||
					(vertexHandle(terminal.n.End) == capV && vertexHandle(terminal.n.Start) == sideV) {
					if lead[k] != nil {
						return nil, brepBandMass{}, fmt.Errorf(`%w: two terminal arcs meet one partial fillet corner`, ErrUnsupported)
					}
					lead[k], trail[k] = terminal, terminal
				}
			}
			if lead[k] == nil {
				return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet corner has no terminal arc`, ErrUnsupported)
			}
			continue
		}
		if side[k].n.Start != side[prev].n.End {
			return nil, brepBandMass{}, fmt.Errorf(`%w: selected fillet walks do not meet at one corner`, ErrUnsupported)
		}
		if fr.loop.Corners[k] == filletband.Reflex {
			arc := open.capBySeg[bi][b.capArc[k]]
			if arc == nil || arc.n.Start != capEdges[prev].n.End || arc.n.End != capEdges[k].n.Start {
				return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet's reflex connector does not meet its walks`, ErrUnsupported)
			}
			j := fr.joins[k]
			w := fr.walks[k]
			vertex := planePoint{U: w.StartU, V: w.StartV}
			trail[k] = meridian(j.PA, vertex, w, vertexHandle(arc.n.Start), vertexHandle(side[k].n.Start))
			lead[k] = meridian(j.PB, vertex, w, vertexHandle(arc.n.End), vertexHandle(side[k].n.Start))
			continue
		}
		if fr.loop.Corners[k] == filletband.Tangent {
			j := fr.joins[k]
			w := fr.walks[k]
			trail[k] = meridian(j.M, planePoint{U: w.StartU, V: w.StartV}, w, capV, sideV)
			lead[k] = trail[k]
			continue
		}
		if capEdges[k] == nil || capEdges[prev] == nil || capEdges[k].n.Start != capEdges[prev].n.End {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet's selected cap walks do not meet`, ErrUnsupported)
		}
		if fr.loop.Corners[k] != filletband.Miter {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet's selected corner is not a straight miter`, ErrUnsupported)
		}
		j := fr.joins[k]
		w := fr.walks[k]
		kappa, err := filletband.Kappa(fr.loop.Walks[prev], fr.loop.Walks[k])
		if err != nil {
			return nil, brepBandMass{}, err
		}
		lengthIv, err := filletband.QuarterEllipseLength(rIv, kappa)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		held, bound := heldOf(lengthIv)
		bu, bv := j.M.U-w.StartU, j.M.V-w.StartV
		semi := math.Hypot(bu, bv)
		major := pl.Dir(bu/semi, bv/semi, 0)
		axis, _ := major.Cross(up).Normalize()
		lead[k] = bornEdge(&Edge{n: topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.Ellipse3{Center: pl.Point(j.M.U, j.M.V, sideZ), Axis: axis,
			Major: major, SemiMajor: units.Millimeters(semi), SemiMinor: units.Millimeters(r)},
			Start: vertexNode(capV),
			End:   vertexNode(sideV),
		}})
		trail[k] = lead[k]
	}
	var patches []*Face
	area := proofbound.BoundedScalar{}
	newPatch := func(surf surfacegeom.Surface, loop *Loop, capV *Vertex, piece filletband.Piece) error {
		held, bound := heldOf(filletband.PatchArea(piece, rIv))
		face := bornFace(&Face{n: topo.Face{Area: held, AreaBound: bound, Surface: surf, Origins: []provenance.Ref{{Producer: ref, Role: b.patchRole(len(patches))}}, Body: bodyNode(body), Loops: []*topo.Loop{loopNode(loop)}}})
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(held, bound))
		nrm, err := face.NormalAt(capV.n.Position)
		if err != nil {
			return fmt.Errorf(`%w: a partial fillet patch's normal cannot be read: %v`, ErrUnsupported, err)
		}
		if nrm.Value.Dot(up) < 0 {
			face.n.Reversed = !face.n.Reversed
		}
		if b.sigma > 0 {
			face.n.Reversed = !face.n.Reversed
		}
		for _, ce := range loop.n.CoEdges {
			ce.Edge.Faces = append(ce.Edge.Faces, faceNode(face))
		}
		patches = append(patches, face)
		return nil
	}
	p := 0
	for i, on := range b.selected {
		if !on {
			continue
		}
		if p >= len(pieces) || pieces[p].Walk != i {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has no piece for its selected walk`, ErrUnsupported)
		}
		piece := pieces[p]
		w := fr.walks[i]
		next := (i + 1) % n
		start, end := lead[i], trail[next]
		if start == nil || end == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet patch has an open corner`, ErrUnsupported)
		}
		capE, sideE := capEdges[i], side[i]
		if filletband.SphereWalk(w, r) {
			pole := capAt(i)
			if piece.Kind != filletband.InnerTorus || pole == nil || pole != capAt(next) ||
				vertexHandle(start.n.Start) != pole || vertexHandle(end.n.Start) != pole || start.n.End != sideE.n.Start || end.n.End != sideE.n.End {
				return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet's sphere patch does not close`, ErrUnsupported)
			}
			surf := surfacegeom.Sphere{Center: pl.Point(w.CU, w.CV, sideZ), Radius: units.Millimeters(r)}
			loop := bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(sideE), Forward: true},
				{Edge: edgeNode(end), Forward: false}, {Edge: edgeNode(start), Forward: true}},
			}})
			if err := newPatch(surf, loop, pole, piece); err != nil {
				return nil, brepBandMass{}, err
			}
			sideE.n.Convex = convex
			p++
			continue
		}
		if piece.Kind != filletband.Cylinder || capE == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has no cylinder piece for its selected walk`, ErrUnsupported)
		}
		if !edgeConnects(start, vertexHandle(capE.n.Start), vertexHandle(sideE.n.Start)) || !edgeConnects(end, vertexHandle(capE.n.End), vertexHandle(sideE.n.End)) {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet patch does not close on its contours`, ErrUnsupported)
		}
		tu, tv := walkTangent(w)
		surf := surfacegeom.Cylinder{Origin: pl.Point(w.StartU-r*tv, w.StartV+r*tu, sideZ),
			Axis: pl.Dir(tu, tv, 0), Radius: units.Millimeters(r)}
		startForward := start.n.Start == capE.n.Start
		endForward := end.n.Start == sideE.n.End
		loop := bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(sideE), Forward: true},
			{Edge: edgeNode(end), Forward: endForward}, {Edge: edgeNode(capE), Forward: false},
			{Edge: edgeNode(start), Forward: startForward}},
		}})
		if err := newPatch(surf, loop, vertexHandle(capE.n.Start), piece); err != nil {
			return nil, brepBandMass{}, err
		}
		capE.n.Convex, sideE.n.Convex = convex, convex
		p++
		if !b.selected[next] || fr.loop.Corners[next] != filletband.Reflex {
			continue
		}
		if p >= len(pieces) || pieces[p].Kind != filletband.HornTorus || pieces[p].Corner != next {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has no horn piece for its reflex corner`, ErrUnsupported)
		}
		arc := open.capBySeg[bi][b.capArc[next]]
		v := fr.walks[next]
		torus := surfacegeom.Torus{Center: pl.Point(v.StartU, v.StartV, sideZ), Axis: pl.Dir(0, 0, 1),
			Major: units.Millimeters(r), Minor: units.Millimeters(r)}
		horn := bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(arc), Forward: true},
			{Edge: edgeNode(lead[next]), Forward: true}, {Edge: edgeNode(trail[next]), Forward: false}},
		}})
		if err := newPatch(torus, horn, vertexHandle(arc.n.Start), pieces[p]); err != nil {
			return nil, brepBandMass{}, err
		}
		p++
	}
	if p != len(pieces) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has unmatched strip pieces`, ErrUnsupported)
	}
	terms, err := filletband.MassOf(filletband.BandMassInput{
		Pieces: pieces, Radius: rIv, Level: f.z0, LevelDelta: f.z0Delta,
		MaterialSign: b.matSign(f), Sigma: b.sigma, Embed: e,
	})
	if err != nil {
		return nil, brepBandMass{}, err
	}
	mass := brepBandMass{vol3: terms.Volume3, moments: terms.Moments, area: area}
	return patches, mass, nil
}

func edgeConnects(e *Edge, a, b *Vertex) bool {
	return (vertexHandle(e.n.Start) == a && vertexHandle(e.n.End) == b) || (vertexHandle(e.n.Start) == b && vertexHandle(e.n.End) == a)
}
