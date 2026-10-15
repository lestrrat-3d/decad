package breprecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// attachPartialFilletBand closes a selected straight-edge chain against its
// open cap and side contours and the terminal arcs in the record's end faces.
func attachPartialFilletBand(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer, bp Payload, bi int,
	open openSet, e brepgeom.Embed, work *freeform.FreeformWork) ([]*topo.Face, bandMass, error) {
	b := bp.LoopBands[bi]
	f := bp.Faces[b.Face]
	pl := f.View(bp.Xform)
	r := b.Setback.Dc
	rIv, err := filletband.RadiusInterval(b.Setback.Dc, b.Setback.DcDelta)
	if err != nil {
		return nil, bandMass{}, err
	}
	fr, err := FilletLoopOf(proofbound.NewWorkBudget(ctx), b.Orig, r, f.Role, work)
	if err != nil {
		return nil, bandMass{}, err
	}
	if len(fr.Walks) != len(b.Selected) {
		return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet band's walks do not match its selection`, decaderr.ErrUnsupported)
	}
	pieces, err := fr.Loop.PiecesSelected(b.Selected)
	if err != nil {
		return nil, bandMass{}, err
	}
	n := len(fr.Walks)
	capEdges, side := make([]*topo.Edge, n), make([]*topo.Edge, n)
	si := 0
	for i, on := range b.Selected {
		if !on {
			continue
		}
		if si >= len(open.side[bi]) {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet band lacks a selected contour edge`, decaderr.ErrUnsupported)
		}
		capEdges[i], side[i] = open.capBySeg[bi][b.CapWalk[i]], open.side[bi][si].Edge
		if b.CapWalk[i] >= 0 && capEdges[i] == nil {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet band lacks a selected cap edge`, decaderr.ErrUnsupported)
		}
		si++
	}
	if si != len(open.side[bi]) {
		return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet band has extra contour edges`, decaderr.ErrUnsupported)
	}
	convex := b.Sigma < 0
	up := pl.Dir(0, 0, -b.MatSign(f))
	sideZ, _ := b.SideLevel(f)
	lead, trail := make([]*topo.Edge, n), make([]*topo.Edge, n)
	capAt := func(k int) *topo.Vertex {
		if capEdges[k] != nil {
			return capEdges[k].Start
		}
		if capEdges[(k+n-1)%n] != nil {
			return capEdges[(k+n-1)%n].End
		}
		return nil
	}
	meridianLen, meridianBound := heldOf(filletband.MeridianLength(rIv))
	meridian := func(foot sectionrecord.Point2, vertex sectionrecord.Point2, w survey2d.SideWalk, capV, sideV *topo.Vertex) *topo.Edge {
		nu, nv := foot.U-vertex.U, foot.V-vertex.V
		length := math.Hypot(nu, nv)
		normal := pl.Dir(nu/length, nv/length, 0)
		axis, _ := normal.Cross(up).Normalize()
		tu, tv := walkTangent(w)
		tangent := pl.Dir(tu, tv, 0)
		if axis.Dot(tangent) < 0 {
			tangent = tangent.Scale(-1)
		}
		return topobuild.NewEdge(alloc, topo.Edge{
			Convex:      convex,
			Length:      meridianLen,
			LengthBound: meridianBound,
			Curve: surfacegeom.Arc3{Center: pl.Point(foot.U, foot.V, sideZ), Axis: tangent,
				Radius: units.Millimeters(r)},
			Start: capV,
			End:   sideV,
		})
	}
	for k := range n {
		prev := (k + n - 1) % n
		if !b.Selected[prev] && !b.Selected[k] {
			continue
		}
		var capV, sideV *topo.Vertex
		if b.Selected[k] {
			capV, sideV = capAt(k), side[k].Start
		} else {
			capV, sideV = capEdges[prev].End, side[prev].End
		}
		if capV == nil {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet corner has no cap vertex`, decaderr.ErrUnsupported)
		}
		if b.Selected[prev] != b.Selected[k] {
			for _, terminal := range open.terminal[bi] {
				if (terminal.Start == capV && terminal.End == sideV) ||
					(terminal.End == capV && terminal.Start == sideV) {
					if lead[k] != nil {
						return nil, bandMass{}, fmt.Errorf(`%w: two terminal arcs meet one partial fillet corner`, decaderr.ErrUnsupported)
					}
					lead[k], trail[k] = terminal, terminal
				}
			}
			if lead[k] == nil {
				return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet corner has no terminal arc`, decaderr.ErrUnsupported)
			}
			continue
		}
		if side[k].Start != side[prev].End {
			return nil, bandMass{}, fmt.Errorf(`%w: selected fillet walks do not meet at one corner`, decaderr.ErrUnsupported)
		}
		if fr.Loop.Corners[k] == filletband.Reflex {
			arc := open.capBySeg[bi][b.CapArc[k]]
			if arc == nil || arc.Start != capEdges[prev].End || arc.End != capEdges[k].Start {
				return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet's reflex connector does not meet its walks`, decaderr.ErrUnsupported)
			}
			j := fr.Joins[k]
			w := fr.Walks[k]
			vertex := sectionrecord.Point2{U: w.StartU, V: w.StartV}
			trail[k] = meridian(j.PA, vertex, w, arc.Start, side[k].Start)
			lead[k] = meridian(j.PB, vertex, w, arc.End, side[k].Start)
			continue
		}
		if fr.Loop.Corners[k] == filletband.Tangent {
			j := fr.Joins[k]
			w := fr.Walks[k]
			trail[k] = meridian(j.M, sectionrecord.Point2{U: w.StartU, V: w.StartV}, w, capV, sideV)
			lead[k] = trail[k]
			continue
		}
		if capEdges[k] == nil || capEdges[prev] == nil || capEdges[k].Start != capEdges[prev].End {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet's selected cap walks do not meet`, decaderr.ErrUnsupported)
		}
		if fr.Loop.Corners[k] != filletband.Miter {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet's selected corner is not a straight miter`, decaderr.ErrUnsupported)
		}
		j := fr.Joins[k]
		w := fr.Walks[k]
		kappa, err := filletband.Kappa(fr.Loop.Walks[prev], fr.Loop.Walks[k])
		if err != nil {
			return nil, bandMass{}, err
		}
		lengthIv, err := filletband.QuarterEllipseLength(rIv, kappa)
		if err != nil {
			return nil, bandMass{}, err
		}
		held, bound := heldOf(lengthIv)
		bu, bv := j.M.U-w.StartU, j.M.V-w.StartV
		semi := math.Hypot(bu, bv)
		major := pl.Dir(bu/semi, bv/semi, 0)
		axis, _ := major.Cross(up).Normalize()
		lead[k] = topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.Ellipse3{Center: pl.Point(j.M.U, j.M.V, sideZ), Axis: axis,
			Major: major, SemiMajor: units.Millimeters(semi), SemiMinor: units.Millimeters(r)},
			Start: capV,
			End:   sideV,
		})
		trail[k] = lead[k]
	}
	var patches []*topo.Face
	area := proofbound.BoundedScalar{}
	newPatch := func(surf surfacegeom.Surface, loop *topo.Loop, capV *topo.Vertex, piece filletband.Piece) error {
		held, bound := heldOf(filletband.PatchArea(piece, rIv))
		face := topobuild.NewFace(alloc, topo.Face{Area: held, AreaBound: bound, Surface: surf, Origins: []provenance.Ref{{Producer: ref, Role: b.PatchRole(len(patches))}}, Body: body, Loops: []*topo.Loop{loop}})
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(held, bound))
		nrm, err := topobuild.NormalAt(face, capV.Position)
		if err != nil {
			return fmt.Errorf(`%w: a partial fillet patch's normal cannot be read: %v`, decaderr.ErrUnsupported, err)
		}
		if nrm.Value.Dot(up) < 0 {
			face.Reversed = !face.Reversed
		}
		if b.Sigma > 0 {
			face.Reversed = !face.Reversed
		}
		for _, ce := range loop.CoEdges {
			ce.Edge.Faces = append(ce.Edge.Faces, face)
		}
		patches = append(patches, face)
		return nil
	}
	p := 0
	for i, on := range b.Selected {
		if !on {
			continue
		}
		if p >= len(pieces) || pieces[p].Walk != i {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet has no piece for its selected walk`, decaderr.ErrUnsupported)
		}
		piece := pieces[p]
		w := fr.Walks[i]
		next := (i + 1) % n
		start, end := lead[i], trail[next]
		if start == nil || end == nil {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet patch has an open corner`, decaderr.ErrUnsupported)
		}
		capE, sideE := capEdges[i], side[i]
		if filletband.SphereWalk(w, r) {
			pole := capAt(i)
			if piece.Kind != filletband.InnerTorus || pole == nil || pole != capAt(next) ||
				start.Start != pole || end.Start != pole || start.End != sideE.Start || end.End != sideE.End {
				return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet's sphere patch does not close`, decaderr.ErrUnsupported)
			}
			surf := surfacegeom.Sphere{Center: pl.Point(w.CU, w.CV, sideZ), Radius: units.Millimeters(r)}
			loop := topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: sideE, Forward: true},
				{Edge: end, Forward: false}, {Edge: start, Forward: true}},
			})
			if err := newPatch(surf, loop, pole, piece); err != nil {
				return nil, bandMass{}, err
			}
			sideE.Convex = convex
			p++
			continue
		}
		if piece.Kind != filletband.Cylinder || capE == nil {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet has no cylinder piece for its selected walk`, decaderr.ErrUnsupported)
		}
		if !edgeConnects(start, capE.Start, sideE.Start) || !edgeConnects(end, capE.End, sideE.End) {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet patch does not close on its contours`, decaderr.ErrUnsupported)
		}
		tu, tv := walkTangent(w)
		surf := surfacegeom.Cylinder{Origin: pl.Point(w.StartU-r*tv, w.StartV+r*tu, sideZ),
			Axis: pl.Dir(tu, tv, 0), Radius: units.Millimeters(r)}
		startForward := start.Start == capE.Start
		endForward := end.Start == sideE.End
		loop := topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: sideE, Forward: true},
			{Edge: end, Forward: endForward}, {Edge: capE, Forward: false},
			{Edge: start, Forward: startForward}},
		})
		if err := newPatch(surf, loop, capE.Start, piece); err != nil {
			return nil, bandMass{}, err
		}
		capE.Convex, sideE.Convex = convex, convex
		p++
		if !b.Selected[next] || fr.Loop.Corners[next] != filletband.Reflex {
			continue
		}
		if p >= len(pieces) || pieces[p].Kind != filletband.HornTorus || pieces[p].Corner != next {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet has no horn piece for its reflex corner`, decaderr.ErrUnsupported)
		}
		arc := open.capBySeg[bi][b.CapArc[next]]
		v := fr.Walks[next]
		torus := surfacegeom.Torus{Center: pl.Point(v.StartU, v.StartV, sideZ), Axis: pl.Dir(0, 0, 1),
			Major: units.Millimeters(r), Minor: units.Millimeters(r)}
		horn := topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: arc, Forward: true},
			{Edge: lead[next], Forward: true}, {Edge: trail[next], Forward: false}},
		})
		if err := newPatch(torus, horn, arc.Start, pieces[p]); err != nil {
			return nil, bandMass{}, err
		}
		p++
	}
	if p != len(pieces) {
		return nil, bandMass{}, fmt.Errorf(`%w: a partial fillet has unmatched strip pieces`, decaderr.ErrUnsupported)
	}
	terms, err := filletband.MassOf(filletband.BandMassInput{
		Pieces: pieces, Radius: rIv, Level: f.Z0, LevelDelta: f.Z0Delta,
		MaterialSign: b.MatSign(f), Sigma: b.Sigma, Embed: e,
	})
	if err != nil {
		return nil, bandMass{}, err
	}
	mass := bandMass{vol3: terms.Volume3, moments: terms.Moments, area: area}
	return patches, mass, nil
}

func edgeConnects(e *topo.Edge, a, b *topo.Vertex) bool {
	return (e.Start == a && e.End == b) || (e.Start == b && e.End == a)
}

type partialChamferQuad struct {
	points [4][3]float64 // side start/end, cap end/start, in reference coordinates
	cap    *topo.Edge
	side   *topo.Edge
}

// attachPartialChamferBand builds one or two concave root strips or two convex outer strips.
// Every patch vertex is already an open edge of the rewritten face record.
func attachPartialChamferBand(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer,
	bp Payload, bi int, open openSet, e brepgeom.Embed) ([]*topo.Face, bandMass, error) {
	if err := ctx.Err(); err != nil {
		return nil, bandMass{}, err
	}
	b := bp.LoopBands[bi]
	f := bp.Faces[b.Face]
	selectedCount := 0
	for _, on := range b.Selected {
		if on {
			selectedCount++
		}
	}
	outerPair := b.Loop == 0 && b.Sigma == -1 && selectedCount == 2
	rootRun := b.Loop > 0 && b.Sigma == 1 && (selectedCount == 1 || selectedCount == 2)
	if (!outerPair && !rootRun) || f.Delta != 0 || b.Setback.Dc != b.Setback.Ds ||
		b.Setback.DcDelta != 0 ||
		b.Setback.DsDelta != 0 {
		return nil, bandMass{}, fmt.Errorf(`%w: a partial chamfer requires an exact outer pair or one or two inner roots`, decaderr.ErrUnsupported)
	}
	sideZ, sideDelta := b.SideLevel(f)
	if sideDelta != 0 {
		return nil, bandMass{}, fmt.Errorf(`%w: a partial chamfer's side level is displaced`, decaderr.ErrUnsupported)
	}
	work := freeform.NewFreeformWork()
	quads := make([]partialChamferQuad, len(b.Selected))
	sideOrdinal := 0
	for i, on := range b.Selected {
		if !on {
			continue
		}
		if sideOrdinal >= len(open.side[bi]) || b.CapWalk[i] < 0 {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial chamfer has no selected contour edge`, decaderr.ErrUnsupported)
		}
		ow, err := boundarywalk.WalkOf(b.Orig.Segments[i], work)
		if err != nil {
			return nil, bandMass{}, err
		}
		cw, err := boundarywalk.WalkOf(f.RegionLoop(b.Loop).Segments[b.CapWalk[i]], work)
		if err != nil {
			return nil, bandMass{}, err
		}
		if !ow.IsLine() || !cw.IsLine() {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial chamfer has a curved selected walk`, decaderr.ErrUnsupported)
		}
		capEdge := open.capBySeg[bi][b.CapWalk[i]]
		sideEdge := open.side[bi][sideOrdinal].Edge
		if capEdge == nil || sideEdge == nil {
			return nil, bandMass{}, fmt.Errorf(`%w: a partial chamfer lacks a band edge`, decaderr.ErrUnsupported)
		}
		quads[i] = partialChamferQuad{
			points: [4][3]float64{
				e.Canon(ow.StartU, ow.StartV, sideZ), e.Canon(ow.EndU, ow.EndV, sideZ),
				e.Canon(cw.EndU, cw.EndV, f.Z0), e.Canon(cw.StartU, cw.StartV, f.Z0),
			}, cap: capEdge, side: sideEdge,
		}
		if err := partialband.ChamferQuadExact(quads[i].points, b.Setback.Dc, b.Setback.Ds, e); err != nil {
			return nil, bandMass{}, err
		}
		sideOrdinal++
	}
	if sideOrdinal != selectedCount || sideOrdinal != len(open.side[bi]) ||
		len(open.terminal[bi]) != 2 {
		return nil, bandMass{}, fmt.Errorf(`%w: a partial chamfer does not have one or two strips and two terminal lines`, decaderr.ErrUnsupported)
	}
	lead, trail := make([]*topo.Edge, len(quads)), make([]*topo.Edge, len(quads))
	for i, on := range b.Selected {
		if !on {
			continue
		}
		prev, next := (i+len(quads)-1)%len(quads), (i+1)%len(quads)
		if b.Selected[prev] {
			if quads[prev].cap.End != quads[i].cap.Start ||
				quads[prev].side.End != quads[i].side.Start ||
				quads[prev].points[2] != quads[i].points[3] ||
				quads[prev].points[1] != quads[i].points[0] {
				return nil, bandMass{}, fmt.Errorf(`%w: two partial chamfer strips disagree at their shared corner`, decaderr.ErrUnsupported)
			}
			slant, _, err := capblend.SlantEdge(alloc, proofbound.NewWorkBudget(ctx),
				sectionrecord.Point2{U: e.Local(quads[i].points[3])[0], V: e.Local(quads[i].points[3])[1]},
				quads[i].cap.Start, quads[i].side.Start,
				e.Local(quads[i].points[0])[0], e.Local(quads[i].points[0])[1],
				f.Z0, sideZ, 0, 0, survey2d.SideWalk{}, survey2d.SideWalk{}, b.Setback, true)
			if err != nil {
				return nil, bandMass{}, err
			}
			lead[i], trail[i] = slant, slant
		}
		if !b.Selected[prev] {
			lead[i] = partialChamferTerminal(open.terminal[bi], quads[i].cap.Start, quads[i].side.Start)
		}
		if !b.Selected[next] {
			trail[next] = partialChamferTerminal(open.terminal[bi], quads[i].cap.End, quads[i].side.End)
		}
	}
	var patches []*topo.Face
	mass := zeroBandMass()
	capSign := -1.0
	if f.Outward {
		capSign = 1
	}
	outwardCap := f.View(bp.Xform).Dir(0, 0, capSign)
	for i, on := range b.Selected {
		if !on {
			continue
		}
		next := (i + 1) % len(quads)
		start, end := lead[i], trail[next]
		capE, sideE := quads[i].cap, quads[i].side
		if start == nil || end == nil || !edgeConnects(start, capE.Start, sideE.Start) ||
			!edgeConnects(end, capE.End, sideE.End) {
			return nil, bandMass{}, fmt.Errorf(`%w: partial chamfer patch %d does not close`, decaderr.ErrUnsupported, i)
		}
		plane, err := capblend.PlaneFromThree(sideE.Start.Position, sideE.End.Position, capE.End.Position)
		if err != nil {
			return nil, bandMass{}, err
		}
		loop := topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: sideE, Forward: true},
			{Edge: end, Forward: end.Start == sideE.End}, {Edge: capE, Forward: false},
			{Edge: start, Forward: start.Start == capE.Start}},
		})
		areaIv, volume3, moments, err := partialband.ChamferQuadMass(quads[i].points)
		if err != nil {
			return nil, bandMass{}, err
		}
		held, bound := heldOf(areaIv)
		face := topobuild.NewFace(alloc, topo.Face{Area: held, AreaBound: bound, Surface: plane, Origins: []provenance.Ref{{Producer: ref, Role: b.PatchRole(len(patches))}}, Body: body, Loops: []*topo.Loop{loop}})
		normal, err := topobuild.NormalAt(face, sideE.Start.Position)
		if err != nil {
			return nil, bandMass{}, fmt.Errorf(`%w: partial chamfer patch %d has no normal: %v`,
				decaderr.ErrUnsupported, i, err)
		}
		// The recorded loop can wind either way on opposite cap faces. Its
		// divergence terms follow that winding, while the patch normal must
		// always point from material toward the cap side of the removed wedge.
		aligned := normal.Value.Dot(outwardCap)
		if math.Abs(aligned) < 0.5 {
			return nil, bandMass{}, fmt.Errorf(`%w: partial chamfer patch %d has ambiguous outward sense`,
				decaderr.ErrUnsupported, i)
		}
		if aligned < 0 {
			face.Reversed = true
			volume3.Neg(volume3)
			for k := range moments {
				moments[k].Neg(moments[k])
			}
		}
		for _, ce := range loop.CoEdges {
			ce.Edge.Faces = append(ce.Edge.Faces, face)
		}
		patches = append(patches, face)
		mass.area = proofbound.BoundedAdd(mass.area, proofbound.MeasuredScalar(held, bound))
		mass.vol3 = proofbound.IntervalAdd(mass.vol3, proofbound.PointInterval(volume3))
		for k := range 3 {
			mass.moments[k] = proofbound.IntervalAdd(mass.moments[k], proofbound.PointInterval(moments[k]))
		}
		capE.Convex, sideE.Convex = true, true
	}
	return patches, mass, nil
}

func partialChamferTerminal(terminals map[BandTerminal]*topo.Edge, capVertex, side *topo.Vertex) *topo.Edge {
	for _, edge := range terminals {
		if edgeConnects(edge, capVertex, side) {
			return edge
		}
	}
	return nil
}
