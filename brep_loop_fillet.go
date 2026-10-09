package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is route L's fillet arm (docs/loop-fillet-design.md, "loop-fillet
// §N" below): the corner classes of Table LF and SF1's refusal
// (filletLoopRead), the pipe band's faces, edges and roles (attachFilletBand),
// its strip terms in the body's mass sums (§5.3), the receiver's faces
// restored for those sums (filletRestored), and the band's extent along a
// direction (§5.4). internal/filletband owns every closed form.

// filletLoopRead is one fillet band's loop as Table LF reads it: the
// coalesced walks of orig, the cap contour's joins at the radius, and the
// band's patches in its own order (filletband.Loop.Pieces).
type filletLoopRead struct {
	loop   filletband.Loop
	walks  []survey2d.SideWalk
	joins  []cornerJoin
	pieces []filletband.Piece
}

// filletLoopOf classifies every corner of loop, walked with face F's material
// on its left, for a fillet of radius r (loop-fillet §4 step 2). The class is
// the exact sign of the turn the record's own coordinates take there
// (filletband.Turn): a right turn the cap contour closes with a connector arc
// is LF6; a left turn between two straight walks the contour mitres is LF4; a
// tangent join capband.JoinIsG1 proves on the recorded segments, which the
// contour reads as a G1 foot, is LF5. Every other corner — a left turn at a
// circular walk, a tangent join the record does not make exactly tangent, a
// cusp — is SF1. A loop of one whole circle (LF7) has no corner. role names
// F in the refusal.
func filletLoopOf(budget *proofbound.WorkBudget, loop LoopRecord, r float64, role string, work *freeform.FreeformWork) (filletLoopRead, error) {
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return filletLoopRead{}, err
	}
	out := filletLoopRead{walks: cl.walks}
	for _, w := range cl.walks {
		out.loop.Walks = append(out.loop.Walks, filletband.Walk{
			Circular: w.IsCircular(), Closed: w.Closed, CCW: w.Th1 > w.Th0,
			Start:  filletband.Point{U: w.StartU, V: w.StartV},
			End:    filletband.Point{U: w.EndU, V: w.EndV},
			Center: filletband.Point{U: w.CU, V: w.CV}, Radius: w.Radius,
		})
	}
	n := len(cl.walks)
	if !out.loop.WholeTurn() {
		if out.joins, err = capOffsetJoins(budget, cl, r); err != nil {
			return filletLoopRead{}, err
		}
		for k := range n {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return filletLoopRead{}, err
			}
			class, err := filletCornerClass(loop, cl.walks, out.loop.Walks, out.joins, k)
			if err != nil {
				return filletLoopRead{}, err
			}
			if class == 0 {
				return filletLoopRead{}, fmt.Errorf(`%w: a fillet of a complete loop of brep face %s meets the corner (%s, %s) between recorded segments %d and %d, where a straight walk meets a circular one or two circular walks meet, not exactly tangent; the two pipes there meet along a space curve no edge kind names and the strip the fillet removes is no polynomial in its offset (loop-fillet SF1)`,
					ErrUnsupported, role, renderCoord(cl.walks[k].StartU), renderCoord(cl.walks[k].StartV),
					cl.walks[(k+n-1)%n].Segs[len(cl.walks[(k+n-1)%n].Segs)-1], cl.walks[k].Segs[0])
			}
			out.loop.Corners = append(out.loop.Corners, class)
		}
	}
	if out.pieces, err = out.loop.Pieces(); err != nil {
		return filletLoopRead{}, err
	}
	return out, nil
}

// filletCornerClass is Table LF's class of corner k, or zero for SF1.
func filletCornerClass(loop LoopRecord, walks []survey2d.SideWalk, fw []filletband.Walk, joins []cornerJoin, k int) (filletband.Corner, error) {
	n := len(walks)
	prev, cur := walks[(k+n-1)%n], walks[k]
	turn, err := filletband.Turn(fw[(k+n-1)%n], fw[k])
	if err != nil {
		return 0, err
	}
	j := joins[k]
	switch {
	case turn < 0 && j.arc:
		return filletband.Reflex, nil
	case turn > 0 && prev.IsLine() && cur.IsLine() && !j.arc && !j.g1:
		return filletband.Miter, nil
	case turn == 0 && j.g1:
		prevSeg, err := normalizeSegment(loop.Segments[prev.Segs[len(prev.Segs)-1]])
		if err != nil {
			return 0, err
		}
		curSeg, err := normalizeSegment(loop.Segments[cur.Segs[0]])
		if err != nil {
			return 0, err
		}
		if capband.JoinIsG1(prevSeg, curSeg) {
			return filletband.Tangent, nil
		}
	}
	return 0, nil
}

// filletRadius encloses every radius the band's setback denotes: the held
// radius widened by its conversion bound.
func filletRadius(s capSetback) (proofbound.RatInterval, error) {
	r, d := proofarith.FloatRat(s.dc), proofarith.FloatRat(s.dcDelta)
	if r == nil || d == nil {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a fillet radius is not finite`, ErrNotFinite)
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(r), d), nil
}

// heldOf is an enclosure's held float beside the bound that reaches every
// value of it.
func heldOf(iv proofbound.RatInterval) (float64, float64) {
	held := brepHeld(iv)
	return held, proofbound.IntervalFloatError(iv, held)
}

// attachFilletBand builds fillet band bi of the record (loop-fillet §4 step 4
// to 6): one patch per walk of orig and one per LF6 corner, in the band's own
// order, each carrying filletLoop(f,l,p). A walk's patch is bounded by its
// side contact edge on the trimmed wall (side), the end curve at the corner
// after it, its cap contact edge on F (cap) reversed, and the end curve at
// the corner before it: an LF4 corner's end curve is one Ellipse3 the two
// cylinders share, an LF5 join's one Arc3 meridian, and an LF6 corner's two
// Arc3 meridians, at its connector arc's ends pA and pB, the horn torus
// between them closing on the corner's own side vertex. Every vertex is one
// the record's band boundary edges already placed (brepOpenEdges): the cap
// contour's at F's level, carrying F's section displacement, and the loop's
// corners at the side level. Each patch's outward normal is read off its own
// NormalAt at its cap contact against F's outward side, then turned over for
// a band that fills (sigma +1). Every contact edge and meridian is a tangent
// junction and takes the band's sense; an LF4 ellipse takes the loop
// corner's, which is the same.
//
// The band's mass terms are the strip's (loop-fillet §5.3): 3·σ·V_strip and
// σ·M_strip, mapped from F's frame onto the reference axes by F's signed
// permutation e, and the patches' summed areas. The patches' flux is never
// integrated: measureBrepContext reads the record restored (filletRestored)
// in their place.
func attachFilletBand(ctx context.Context, body *Body, ref producerID, bp brepPayload, bi int, open brepOpenSet, e brepEmbed, work *freeform.FreeformWork) ([]*Face, brepBandMass, error) {
	b := bp.loopBands[bi]
	f := bp.faces[b.face]
	pl := f.view(bp.xform)
	m := b.matSign(f)
	sideZ, _ := b.sideLevel(f)
	r := b.setback.dc
	rIv, err := filletRadius(b.setback)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	fr, err := filletLoopOf(budget, b.orig, r, f.role, work)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	convex := b.sigma < 0
	up := pl.dir(0, 0, -m)
	sideCo, capCo := open.side[bi], open.cap[bi]
	n := len(fr.walks)
	if len(sideCo) != n {
		return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's side contour holds %d edges for %d walks`, ErrUnsupported, bi, len(sideCo), n)
	}

	var patches []*Face
	area := proofbound.BoundedScalar{}
	newPatch := func(surf Surface, loops []*Loop, capV *Vertex) error {
		p := len(patches)
		held, bound := heldOf(filletband.PatchArea(fr.pieces[p], rIv))
		face := &Face{surface: surf, origins: []FeatureRef{{producer: ref, Role: b.patchRole(p)}}, body: body,
			loops: loops, area: held, areaBound: bound}
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(held, bound))
		// capV, a cap contact vertex, lies on the patch's surface; the outward
		// normal there is F's own, which faces −m along F's normal for a band
		// that removes material.
		nrm, err := face.NormalAt(capV.position)
		if err != nil {
			return fmt.Errorf(`%w: a loop fillet band patch's outward orientation cannot be certified; its own normal reading refuses at its cap contact (%v)`, ErrUnsupported, err)
		}
		if nrm.Value.Dot(up) < 0 {
			face.reversed = !face.reversed
		}
		if b.sigma > 0 {
			face.reversed = !face.reversed
		}
		for _, l := range loops {
			for _, ce := range l.coedges {
				ce.edge.faces = append(ce.edge.faces, face)
			}
		}
		patches = append(patches, face)
		return nil
	}
	torus := func(cu, cv, major float64) Torus {
		return Torus{Center: pl.point(cu, cv, sideZ), Axis: pl.dir(0, 0, 1), Major: units.Millimeters(major), Minor: units.Millimeters(r)}
	}
	walkSurface := func(w survey2d.SideWalk) Surface {
		if w.IsCircular() {
			major := w.Radius - r
			if w.Th1 < w.Th0 {
				major = w.Radius + r
			}
			return torus(w.CU, w.CV, major)
		}
		tu, tv := walkTangent(w)
		return Cylinder{Origin: pl.point(w.StartU-r*tv, w.StartV+r*tu, sideZ), Axis: pl.dir(tu, tv, 0), Radius: units.Millimeters(r)}
	}

	// straightWalk is the first of a, b (when given) that is a straight walk,
	// or nil.
	straightWalk := func(a survey2d.SideWalk, b *survey2d.SideWalk) *survey2d.SideWalk {
		if !a.IsCircular() {
			return &a
		}
		if b != nil && !b.IsCircular() {
			return b
		}
		return nil
	}

	if fr.loop.WholeTurn() {
		if len(capCo) != 1 {
			return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's cap contour is no one whole circle`, ErrUnsupported, bi)
		}
		side, capE := sideCo[0].edge, capCo[0].edge
		side.convex, capE.convex = convex, convex
		if err := newPatch(walkSurface(fr.walks[0]), []*Loop{
			{coedges: []coedge{{edge: side, forward: true}}, outer: true},
			{coedges: []coedge{{edge: capE, forward: false}}, outer: true},
		}, capE.start); err != nil {
			return nil, brepBandMass{}, err
		}
	} else {
		supplied, err := suppliedCapWalls(capCo, fr.joins, n)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		meridianLen, meridianBound := heldOf(filletband.MeridianLength(rIv))
		// meridian is the quarter circle of radius r about the ball centre
		// under the contour foot, from that foot at F's level down to the side
		// vertex.
		//
		// Its axis is the straight neighbour's own walk direction, the very
		// value that wall's cylinder carries as its Axis, up to sign, so a
		// consumer comparing the two compares equal values: the cross product
		// of the foot's normal with up is perpendicular to the walk only to
		// rounding. A meridian with no straight wall beside it (two circular
		// walks) keeps that cross product.
		meridian := func(foot Point2, v survey2d.SideWalk, straight *survey2d.SideWalk, capV, sideV *Vertex) *Edge {
			nu, nv := foot.U-v.StartU, foot.V-v.StartV
			l := math.Hypot(nu, nv)
			normal := pl.dir(nu/l, nv/l, 0)
			axis, _ := normal.Cross(up).Normalize()
			if straight != nil {
				tu, tv := walkTangent(*straight)
				tangent := pl.dir(tu, tv, 0)
				if axis.Dot(tangent) < 0 {
					tangent = tangent.Scale(-1)
				}
				axis = tangent
			}
			return &Edge{curve: Arc3{Center: pl.point(foot.U, foot.V, sideZ), Axis: axis, Radius: units.Millimeters(r)},
				start: capV, end: sideV, convex: convex, length: meridianLen, lengthBound: meridianBound}
		}
		lead, trail := make([]*Edge, n), make([]*Edge, n)
		for k := range n {
			j := fr.joins[k]
			w := fr.walks[k]
			sideV := sideCo[k].edge.start
			switch fr.loop.Corners[k] {
			case filletband.Reflex:
				arc := supplied.arc[k]
				if arc == nil {
					return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's LF6 corner %d has no connector arc`, ErrUnsupported, bi, k)
				}
				arc.convex = convex
				trail[k] = meridian(j.pA, w, straightWalk(w, nil), arc.start, sideV)
				lead[k] = meridian(j.pB, w, straightWalk(w, nil), arc.end, sideV)
			case filletband.Tangent:
				prev := fr.walks[(k+n-1)%n]
				lead[k] = meridian(j.m, w, straightWalk(prev, &w), supplied.wall[k].start, sideV)
				trail[k] = lead[k]
			default:
				kappa, err := filletband.Kappa(fr.loop.Walks[(k+n-1)%n], fr.loop.Walks[k])
				if err != nil {
					return nil, brepBandMass{}, err
				}
				lenIv, err := filletband.QuarterEllipseLength(rIv, kappa)
				if err != nil {
					return nil, brepBandMass{}, err
				}
				held, bound := heldOf(lenIv)
				bu, bv := j.m.U-w.StartU, j.m.V-w.StartV
				semi := math.Hypot(bu, bv)
				major := pl.dir(bu/semi, bv/semi, 0)
				axis, _ := major.Cross(up).Normalize()
				lead[k] = &Edge{
					curve: Ellipse3{Center: pl.point(j.m.U, j.m.V, sideZ), Axis: axis, Major: major,
						SemiMajor: units.Millimeters(semi), SemiMinor: units.Millimeters(r)},
					start: supplied.wall[k].start, end: sideV, convex: convex, length: held, lengthBound: bound,
				}
				trail[k] = lead[k]
			}
		}
		for i := range n {
			next := (i + 1) % n
			wall := supplied.wall[i]
			if wall.start != lead[i].start || wall.end != trail[next].start ||
				sideCo[i].edge.start != lead[i].end || sideCo[i].edge.end != trail[next].end {
				return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's walk %d does not close on its contour and side vertices`, ErrUnsupported, bi, i)
			}
			wall.convex, sideCo[i].edge.convex = convex, convex
			if err := newPatch(walkSurface(fr.walks[i]), []*Loop{{coedges: []coedge{
				{edge: sideCo[i].edge, forward: true},
				{edge: trail[next], forward: false},
				{edge: wall, forward: false},
				{edge: lead[i], forward: true},
			}, outer: true}}, wall.start); err != nil {
				return nil, brepBandMass{}, err
			}
			if fr.loop.Corners[next] != filletband.Reflex {
				continue
			}
			v := fr.walks[next]
			if err := newPatch(torus(v.StartU, v.StartV, r), []*Loop{{coedges: []coedge{
				{edge: supplied.arc[next], forward: true},
				{edge: lead[next], forward: true},
				{edge: trail[next], forward: false},
			}, outer: true}}, supplied.arc[next].start); err != nil {
				return nil, brepBandMass{}, err
			}
		}
	}
	if len(patches) != len(fr.pieces) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d built %d patches for %d pieces`, ErrUnsupported, bi, len(patches), len(fr.pieces))
	}
	mass, err := filletStripMass(b, f, e, fr.pieces, rIv)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	mass.area = area
	return patches, mass, nil
}

// filletStripMass is one fillet band's share of the body's divergence sums in
// the reference frame (loop-fillet §5.3): 3·σ·V_strip, and σ·M_strip along
// each reference axis F's frame lands on. The side level is F's recorded
// level plus m·r, widened by F's own level displacement.
func filletStripMass(b brepLoopBand, f brepFace, e brepEmbed, pieces []filletband.Piece, rIv proofbound.RatInterval) (brepBandMass, error) {
	m := b.matSign(f)
	h, err := filletband.HeightsOf(rIv)
	if err != nil {
		return brepBandMass{}, err
	}
	level, levelDelta := proofarith.FloatRat(f.z0), proofarith.FloatRat(f.z0Delta)
	if level == nil || levelDelta == nil {
		return brepBandMass{}, fmt.Errorf(`%w: a fillet band's face level is not finite`, ErrNotFinite)
	}
	sideIv := proofbound.IntervalWiden(proofbound.IntervalAdd(proofbound.PointInterval(level),
		proofbound.IntervalScale(rIv, big.NewRat(int64(m), 1))), levelDelta)
	strip := filletband.StripOf(filletband.SumCoefficients(pieces), h, sideIv, int64(m))
	sigma := big.NewRat(int64(b.sigma), 1)
	out := zeroBrepBandMass()
	out.vol3 = proofbound.IntervalScale(strip.Volume, new(big.Rat).Mul(sigma, big.NewRat(3, 1)))
	for i := range 3 {
		out.moments[e.Axis[i]] = proofbound.IntervalScale(strip.Moment[i], new(big.Rat).Mul(sigma, big.NewRat(int64(e.Sign[i]), 1)))
	}
	return out, nil
}

// filletRestored is the record with every fillet band undone, the faces
// loop-fillet §5.3 sums the receiver's terms over (brepLoopBand.restored's
// reading): F reads orig in its contour's place, each (sw) face beside the loop
// reads its level at F instead of the side level, and each (pl) face its
// segment on the loop back at F's level with its two neighbouring lines
// lengthened to it. The faces beside a band are the ones its side contour's
// open uses name, so the reading follows the rewritten record's own
// topology. Every other face, and every chamfer band's rewrite, is kept. Level
// and section displacements are kept as the rewritten record states them,
// which is never less than the receiver's. A record with no fillet band
// returns its faces unchanged.
func (bp brepPayload) filletRestored(topo *brepTopology) ([]brepFace, error) {
	out := bp.faces
	if !bp.hasFilletBand() {
		return out, nil
	}
	out = append([]brepFace(nil), bp.faces...)
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range topo.open {
		openAt[topo.uses[ui].Key] = ui
	}
	cloneRegion := func(fi int) *ProfileRecord {
		src := out[fi].region
		region := ProfileRecord{Outer: cloneLoopRecord(src.Outer)}
		for _, h := range src.Holes {
			region.Holes = append(region.Holes, cloneLoopRecord(h))
		}
		return &region
	}
	work := freeform.NewFreeformWork()
	for bi, b := range bp.loopBands {
		if b.kind != brepBandFillet {
			continue
		}
		f := bp.faces[b.face]
		eF := topo.embeds[b.face]
		n := eF.Axis[2]
		levelRef := eF.Canon(0, 0, f.z0)[n]
		sideZ, _ := b.sideLevel(f)
		sideRef := eF.Canon(0, 0, sideZ)[n]
		region := cloneRegion(b.face)
		if b.loop == 0 {
			region.Outer = cloneLoopRecord(b.orig)
		} else {
			region.Holes[b.loop-1] = cloneLoopRecord(b.orig)
		}
		out[b.face].region = region
		for _, seg := range b.orig.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			key, _ := brepgeom.CurveKey(eF, w, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, fmt.Errorf(`%w: fillet band %d's side contour is not a boundary of the record`, ErrUnsupported, bi)
			}
			u := topo.uses[ui]
			eA := topo.embeds[u.Face]
			a := &out[u.Face]
			switch u.Part {
			case brepRim0:
				a.z0 = eA.Sign[2]*levelRef + 0
			case brepRim1:
				a.z1 = eA.Sign[2]*levelRef + 0
			case brepLoopSeg:
				a.region = cloneRegion(u.Face)
				if err := restoreSideSegment(a, eA, u.Loop, u.Seg, n, sideRef, levelRef); err != nil {
					return nil, fmt.Errorf("fillet band %d: %w", bi, err)
				}
			default:
				return nil, fmt.Errorf(`%w: fillet band %d's side contour runs along a side line`, ErrUnsupported, bi)
			}
		}
	}
	return out, nil
}

// restoreSideSegment moves a (pl) face's segment seg of loop li, which the
// band moved to sideRef along reference axis n, back to levelRef, with the
// ends of its two neighbouring lines that meet it. All three are lines: the
// segment lies in two planes, and Table LB's LB3 admitted the neighbours.
func restoreSideSegment(a *brepFace, e brepEmbed, li, seg, n int, sideRef, levelRef float64) error {
	loop := a.region.Outer
	if li > 0 {
		loop = a.region.Holes[li-1]
	}
	count := len(loop.Segments)
	move := func(p Point2) Point2 {
		c := e.Canon(p.U, p.V, a.z0)
		if c[n] != sideRef {
			return p
		}
		c[n] = levelRef
		l := e.Local(c)
		return Point2{U: l[0], V: l[1]}
	}
	for _, k := range [3]int{(seg + count - 1) % count, seg, (seg + 1) % count} {
		line, ok := loop.Segments[k].(LineSeg)
		if !ok {
			return fmt.Errorf(`%w: a face beside a fillet band holds a curved segment where the band trimmed it`, ErrUnsupported)
		}
		line.Start, line.End = move(line.Start), move(line.End)
		loop.Segments[k] = line
	}
	return nil
}

// filletBandExtent is fillet band b's extent along world direction g
// (loop-fillet §5.4): filletband.Extreme read in F's own frame along g and
// along −g, lifted through F's frame and the placement as a prism face's
// extent is (prismextent.ExtentAlong's composition): the enclosure's own
// rounding, the frame-and-placement coefficients' allowance, and each
// endpoint sum's rounding.
func filletBandExtent(ctx context.Context, b brepLoopBand, f brepFace, xform r3.Transform, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	pl := f.view(xform)
	rIv, err := filletRadius(b.setback)
	if err != nil {
		return 0, 0, 0, err
	}
	fr, err := filletLoopOf(proofbound.NewWorkBudget(ctx), b.orig, b.setback.dc, f.role, work)
	if err != nil {
		return 0, 0, 0, err
	}
	m := b.matSign(f)
	sideZ, sideDelta := b.sideLevel(f)
	level := proofarith.FloatRat(f.z0)
	if level == nil {
		return 0, 0, 0, fmt.Errorf(`%w: a fillet band's face level is not finite`, ErrNotFinite)
	}
	sideIv := proofbound.IntervalAdd(proofbound.PointInterval(level), proofbound.IntervalScale(rIv, big.NewRat(int64(m), 1)))
	base := pl.xform.Apply(pl.frame.Origin()).Dot(g)
	coeff := [3]float64{pl.dir(1, 0, 0).Dot(g), pl.dir(0, 1, 0).Dot(g), pl.dir(0, 0, 1).Dot(g)}
	var gp, gn [3]*big.Rat
	for i, c := range coeff {
		q := proofarith.FloatRat(c)
		if q == nil {
			return 0, 0, 0, fmt.Errorf(`%w: a fillet band's extent direction is not finite`, ErrNotFinite)
		}
		gp[i], gn[i] = q, new(big.Rat).Neg(q)
	}
	hiIv, err := filletband.Extreme(fr.pieces, rIv, sideIv, int64(m), gp)
	if err != nil {
		return 0, 0, 0, err
	}
	loIv, err := filletband.Extreme(fr.pieces, rIv, sideIv, int64(m), gn)
	if err != nil {
		return 0, 0, 0, err
	}
	loIv = proofbound.IntervalNeg(loIv)
	coordUpper := 0.0
	for _, w := range fr.walks {
		coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(w.StartU), math.Abs(w.StartV)))
		coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(w.EndU), math.Abs(w.EndV)))
		if w.IsCircular() {
			coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(w.CU), math.Abs(w.CV), 2*w.Radius))
		}
	}
	coordUpper = proofbound.AbsSumUpper(coordUpper, 4*b.setback.axialUpper())
	zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(f.z0), math.Abs(sideZ)), sideDelta)
	placeAllow := prismPlacementCoeffAllow(pl, g, base, coeff[0], coeff[1], coeff[2], coordUpper, zUpper)
	lo, loErr := heldOf(loIv)
	hi, hiErr := heldOf(hiIv)
	loEnd, hiEnd := base+lo, base+hi
	bound := proofbound.AbsSumUpper(math.Max(loErr, hiErr), placeAllow, sideDelta,
		math.Max(proofbound.ExactSumRound(loEnd, base, lo), proofbound.ExactSumRound(hiEnd, base, hi)))
	return loEnd, hiEnd, bound, nil
}

// hasFilletBand reports whether the record carries a fillet band.
func (bp brepPayload) hasFilletBand() bool {
	for _, b := range bp.loopBands {
		if b.kind == brepBandFillet {
			return true
		}
	}
	return false
}

// brepFilletUndercuts folds DF7's reading of band's pipe patches into the
// running faces list and undecided flag (loop-fillet Table DF's DF7). Patch
// p of the band is read through the closed-form range of cos φ·A + sin φ·B(θ)
// of its outward normal along the pull: A from F's frame normal, B from the
// inward normal of F's region at the patch's walk (survey2d.InwardRange) or
// through the turn of a reflex corner (survey2d.CornerRange), decided by the
// three-valued rule every undercut reading uses. The patch tags are the built
// surfaces (their stamped departure is zero), so no allowance beyond that
// arithmetic arises. It reports false, a total refusal, when a patch's face is
// missing or a range cannot be enclosed.
func brepFilletUndercuts(budget *proofbound.WorkBudget, roles map[string]*Face, bp brepPayload, band brepLoopBand, pull r3.Vec,
	faces *[]*Face, undecided *bool, work *freeform.FreeformWork) bool {
	f := bp.faces[band.face]
	view := f.view(bp.xform)
	m, ok := survey2d.NewPlacedFrameMap(view.frame, view.xform)
	if !ok {
		return false
	}
	fr, err := filletLoopOf(budget, band.orig, band.setback.dc, f.role, work)
	if err != nil {
		return false
	}
	sign := int64(-1)
	if f.outward {
		sign = 1
	}
	sigma := int64(band.sigma)
	n := len(fr.walks)
	p := 0
	patch := func(rng survey2d.PullBracket, ok bool) bool {
		if !ok {
			return false
		}
		face := roles[band.patchRole(p)]
		p++
		if face == nil {
			return false
		}
		verdict, ok := survey2d.PatchPullVerdict(m, pull, sign, rng)
		return listVerdict(faces, undecided, face, verdict, ok)
	}
	for i, w := range fr.walks {
		if !patch(survey2d.InwardRange(w, m, pull, sigma)) {
			return false
		}
		if ni := (i + 1) % n; !fr.loop.WholeTurn() && fr.loop.Corners[ni] == filletband.Reflex {
			if !patch(survey2d.CornerRange(w, fr.walks[ni], m, pull, sigma)) {
				return false
			}
		}
	}
	return p == len(fr.pieces)
}

// walkTangent is a straight side walk's unit direction in its plane's (u, v).
func walkTangent(w survey2d.SideWalk) (float64, float64) {
	tu, tv := w.EndU-w.StartU, w.EndV-w.StartV
	l := math.Hypot(tu, tv)
	return tu / l, tv / l
}
