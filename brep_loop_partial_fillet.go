package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// attachPartialFilletBand closes a selected straight-edge chain against its
// open cap and side contours and the terminal arcs in the record's end faces.
func attachPartialFilletBand(ctx context.Context, body *Body, ref producerID, bp brepPayload, bi int,
	open brepOpenSet, e brepEmbed, work *freeform.FreeformWork) ([]*Face, brepBandMass, error) {
	b := bp.loopBands[bi]
	f := bp.faces[b.face]
	pl := f.view(bp.xform)
	r := b.setback.dc
	rIv, err := filletRadius(b.setback)
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
		capEdges[i], side[i] = open.capBySeg[bi][b.capWalk[i]], open.side[bi][si].edge
		if b.capWalk[i] >= 0 && capEdges[i] == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet band lacks a selected cap edge`, ErrUnsupported)
		}
		si++
	}
	if si != len(open.side[bi]) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet band has extra contour edges`, ErrUnsupported)
	}
	convex := b.sigma < 0
	up := pl.dir(0, 0, -b.matSign(f))
	sideZ, _ := b.sideLevel(f)
	lead, trail := make([]*Edge, n), make([]*Edge, n)
	capAt := func(k int) *Vertex {
		if capEdges[k] != nil {
			return capEdges[k].start
		}
		if capEdges[(k+n-1)%n] != nil {
			return capEdges[(k+n-1)%n].end
		}
		return nil
	}
	meridianLen, meridianBound := heldOf(filletband.MeridianLength(rIv))
	meridian := func(foot Point2, vertex Point2, w survey2d.SideWalk, capV, sideV *Vertex) *Edge {
		nu, nv := foot.U-vertex.U, foot.V-vertex.V
		length := math.Hypot(nu, nv)
		normal := pl.dir(nu/length, nv/length, 0)
		axis, _ := normal.Cross(up).Normalize()
		tu, tv := walkTangent(w)
		tangent := pl.dir(tu, tv, 0)
		if axis.Dot(tangent) < 0 {
			tangent = tangent.Scale(-1)
		}
		return &Edge{curve: Arc3{Center: pl.point(foot.U, foot.V, sideZ), Axis: tangent,
			Radius: units.Millimeters(r)}, start: capV, end: sideV, convex: convex,
			length: meridianLen, lengthBound: meridianBound}
	}
	for k := range n {
		prev := (k + n - 1) % n
		if !b.selected[prev] && !b.selected[k] {
			continue
		}
		var capV, sideV *Vertex
		if b.selected[k] {
			capV, sideV = capAt(k), side[k].start
		} else {
			capV, sideV = capEdges[prev].end, side[prev].end
		}
		if capV == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet corner has no cap vertex`, ErrUnsupported)
		}
		if b.selected[prev] != b.selected[k] {
			for _, terminal := range open.terminal[bi] {
				if (terminal.start == capV && terminal.end == sideV) ||
					(terminal.end == capV && terminal.start == sideV) {
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
		if side[k].start != side[prev].end {
			return nil, brepBandMass{}, fmt.Errorf(`%w: selected fillet walks do not meet at one corner`, ErrUnsupported)
		}
		if fr.loop.Corners[k] == filletband.Reflex {
			arc := open.capBySeg[bi][b.capArc[k]]
			if arc == nil || arc.start != capEdges[prev].end || arc.end != capEdges[k].start {
				return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet's reflex connector does not meet its walks`, ErrUnsupported)
			}
			j := fr.joins[k]
			w := fr.walks[k]
			vertex := Point2{U: w.StartU, V: w.StartV}
			trail[k] = meridian(j.pA, vertex, w, arc.start, side[k].start)
			lead[k] = meridian(j.pB, vertex, w, arc.end, side[k].start)
			continue
		}
		if fr.loop.Corners[k] == filletband.Tangent {
			j := fr.joins[k]
			w := fr.walks[k]
			trail[k] = meridian(j.m, Point2{U: w.StartU, V: w.StartV}, w, capV, sideV)
			lead[k] = trail[k]
			continue
		}
		if capEdges[k] == nil || capEdges[prev] == nil || capEdges[k].start != capEdges[prev].end {
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
		bu, bv := j.m.U-w.StartU, j.m.V-w.StartV
		semi := math.Hypot(bu, bv)
		major := pl.dir(bu/semi, bv/semi, 0)
		axis, _ := major.Cross(up).Normalize()
		lead[k] = &Edge{curve: Ellipse3{Center: pl.point(j.m.U, j.m.V, sideZ), Axis: axis,
			Major: major, SemiMajor: units.Millimeters(semi), SemiMinor: units.Millimeters(r)},
			start: capV, end: sideV, convex: convex, length: held, lengthBound: bound}
		trail[k] = lead[k]
	}
	var patches []*Face
	area := proofbound.BoundedScalar{}
	newPatch := func(surf Surface, loop *Loop, capV *Vertex, piece filletband.Piece) error {
		held, bound := heldOf(filletband.PatchArea(piece, rIv))
		face := &Face{surface: surf, origins: []FeatureRef{{producer: ref, Role: b.patchRole(len(patches))}},
			body: body, loops: []*Loop{loop}, area: held, areaBound: bound}
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(held, bound))
		nrm, err := face.NormalAt(capV.position)
		if err != nil {
			return fmt.Errorf(`%w: a partial fillet patch's normal cannot be read: %v`, ErrUnsupported, err)
		}
		if nrm.Value.Dot(up) < 0 {
			face.reversed = !face.reversed
		}
		if b.sigma > 0 {
			face.reversed = !face.reversed
		}
		for _, ce := range loop.coedges {
			ce.edge.faces = append(ce.edge.faces, face)
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
		if filletSphereWalk(w, r) {
			pole := capAt(i)
			if piece.Kind != filletband.InnerTorus || pole == nil || pole != capAt(next) ||
				start.start != pole || end.start != pole || start.end != sideE.start || end.end != sideE.end {
				return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet's sphere patch does not close`, ErrUnsupported)
			}
			surf := Sphere{Center: pl.point(w.CU, w.CV, sideZ), Radius: units.Millimeters(r)}
			loop := &Loop{coedges: []coedge{{edge: sideE, forward: true},
				{edge: end, forward: false}, {edge: start, forward: true}}, outer: true}
			if err := newPatch(surf, loop, pole, piece); err != nil {
				return nil, brepBandMass{}, err
			}
			sideE.convex = convex
			p++
			continue
		}
		if piece.Kind != filletband.Cylinder || capE == nil {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has no cylinder piece for its selected walk`, ErrUnsupported)
		}
		if !edgeConnects(start, capE.start, sideE.start) || !edgeConnects(end, capE.end, sideE.end) {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet patch does not close on its contours`, ErrUnsupported)
		}
		tu, tv := walkTangent(w)
		surf := Cylinder{Origin: pl.point(w.StartU-r*tv, w.StartV+r*tu, sideZ),
			Axis: pl.dir(tu, tv, 0), Radius: units.Millimeters(r)}
		startForward := start.start == capE.start
		endForward := end.start == sideE.end
		loop := &Loop{coedges: []coedge{{edge: sideE, forward: true},
			{edge: end, forward: endForward}, {edge: capE, forward: false},
			{edge: start, forward: startForward}}, outer: true}
		if err := newPatch(surf, loop, capE.start, piece); err != nil {
			return nil, brepBandMass{}, err
		}
		capE.convex, sideE.convex = convex, convex
		p++
		if !b.selected[next] || fr.loop.Corners[next] != filletband.Reflex {
			continue
		}
		if p >= len(pieces) || pieces[p].Kind != filletband.HornTorus || pieces[p].Corner != next {
			return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has no horn piece for its reflex corner`, ErrUnsupported)
		}
		arc := open.capBySeg[bi][b.capArc[next]]
		v := fr.walks[next]
		torus := Torus{Center: pl.point(v.StartU, v.StartV, sideZ), Axis: pl.dir(0, 0, 1),
			Major: units.Millimeters(r), Minor: units.Millimeters(r)}
		horn := &Loop{coedges: []coedge{{edge: arc, forward: true},
			{edge: lead[next], forward: true}, {edge: trail[next], forward: false}}, outer: true}
		if err := newPatch(torus, horn, arc.start, pieces[p]); err != nil {
			return nil, brepBandMass{}, err
		}
		p++
	}
	if p != len(pieces) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: a partial fillet has unmatched strip pieces`, ErrUnsupported)
	}
	mass, err := filletStripMass(b, f, e, pieces, rIv)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	mass.area = area
	return patches, mass, nil
}

func edgeConnects(e *Edge, a, b *Vertex) bool {
	return (e.start == a && e.end == b) || (e.start == b && e.end == a)
}
