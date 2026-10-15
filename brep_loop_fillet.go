package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	Loop   filletband.Loop
	Walks  []survey2d.SideWalk
	Joins  []cornerJoin
	Pieces []filletband.Piece
}

// filletLoopOf classifies every corner of loop, walked with face F's material
// on its left, for a fillet of radius r (loop-fillet §4 step 2). The class is
// the exact sign of the turn the record's own coordinates take there
// (filletband.Turn): a right turn the cap contour closes with a connector arc
// is LF6; a left turn between two straight walks the contour mitres is LF4; a
// tangent join capband.JoinIsG1 proves on the recorded segments, which the
// contour reads as a G1 foot, is LF5. A sharp left turn involving a circular
// walk is LF8 when its offset locus is regular. A class that disagrees with
// the contour join, or a cusp, is SF1. A loop of one whole circle (LF7) has
// no corner. role names
// F in the refusal.
func filletLoopOf(budget *proofbound.WorkBudget, loop loopRecord, r float64, role string, work *freeform.FreeformWork) (filletLoopRead, error) {
	cl, err := capblend.OneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return filletLoopRead{}, err
	}
	read, joins, pieces, bad, err := filletband.ReadLoop(budget, loop, cl.Walks, r, shellTol)
	if err != nil {
		return filletLoopRead{}, err
	}
	if bad >= 0 {
		n := len(cl.Walks)
		return filletLoopRead{}, fmt.Errorf(`%w: a fillet of a complete loop of brep face %s has no supported join at corner (%s, %s) between recorded segments %d and %d (loop-fillet SF1)`,
			ErrUnsupported, role, renderCoord(cl.Walks[bad].StartU), renderCoord(cl.Walks[bad].StartV),
			cl.Walks[(bad+n-1)%n].Segs[len(cl.Walks[(bad+n-1)%n].Segs)-1], cl.Walks[bad].Segs[0])
	}
	return filletLoopRead{Loop: read, Walks: cl.Walks, Joins: joins, Pieces: pieces}, nil
}

// suppliedFilletCapWalls pairs the contour's edges with the original walks.
// A collapsed inward arc has no cap edge; its two neighbouring edges share
// the pole vertex at the arc's recorded centre.
func suppliedFilletCapWalls(co []topo.CoEdge, walks []survey2d.SideWalk, joins []cornerJoin, r float64) (suppliedCapEdges, error) {
	n := len(walks)
	out := suppliedCapEdges{Wall: make([]*topo.Edge, n), Arc: make([]*topo.Edge, n)}
	next := 0
	take := func() (*topo.Edge, error) {
		if next >= len(co) || !co[next].Forward {
			return nil, fmt.Errorf(`%w: a fillet band's cap contour does not match its walks`, ErrUnsupported)
		}
		e := co[next].Edge
		next++
		return e, nil
	}
	for i, w := range walks {
		if !filletband.SphereWalk(w, r) {
			e, err := take()
			if err != nil {
				return suppliedCapEdges{}, err
			}
			out.Wall[i] = e
		}
		j := (i + 1) % n
		if !joins[j].Arc {
			continue
		}
		e, err := take()
		if err != nil {
			return suppliedCapEdges{}, err
		}
		out.Arc[j] = e
	}
	if next != len(co) {
		return suppliedCapEdges{}, fmt.Errorf(`%w: a fillet band's cap contour has extra edges`, ErrUnsupported)
	}
	return out, nil
}

// heldOf is an enclosure's held float beside the bound that reaches every
// value of it.
func heldOf(iv proofbound.RatInterval) (float64, float64) {
	held := brepgeom.Held(iv)
	return held, proofbound.IntervalFloatError(iv, held)
}

// attachFilletBand builds fillet band bi of the record (loop-fillet §4 step 4
// to 6): one patch per walk of orig and one per LF6 corner, in the band's own
// order, each carrying filletLoop(f,l,p). A walk's patch is bounded by its
// side contact edge on the trimmed wall (side), the end curve at the corner
// after it, its cap contact edge on F (cap) reversed, and the end curve at
// the corner before it: an LF4 corner's end curve is one Ellipse3 the two
// cylinders share, an LF5 join's one Arc3 meridian, an LF8 join's one
// FilletMiter3 intersection, and an LF6 corner's two Arc3 meridians at its
// connector arc's ends pA and pB, the horn torus
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
	b := bp.LoopBands[bi]
	f := bp.Faces[b.Face]
	pl := f.View(bp.Xform)
	m := b.MatSign(f)
	sideZ, _ := b.SideLevel(f)
	r := b.Setback.Dc
	rIv, err := filletband.RadiusInterval(b.Setback.Dc, b.Setback.DcDelta)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	fr, err := filletLoopOf(budget, b.Orig, r, f.Role, work)
	if err != nil {
		return nil, brepBandMass{}, err
	}
	convex := b.Sigma < 0
	up := pl.Dir(0, 0, -m)
	sideCo, capCo := open.side[bi], open.cap[bi]
	n := len(fr.Walks)
	if len(sideCo) != n {
		return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's side contour holds %d edges for %d walks`, ErrUnsupported, bi, len(sideCo), n)
	}

	var patches []*Face
	area := proofbound.BoundedScalar{}
	newPatch := func(surf surfacegeom.Surface, loops []*Loop, capV *Vertex) error {
		p := len(patches)
		piece := fr.Pieces[p]
		areaIv := filletband.PatchArea(piece, rIv)
		if piece.Walk >= 0 && len(fr.Loop.Corners) > 0 {
			i := piece.Walk
			for _, k := range [2]int{i, (i + 1) % n} {
				if fr.Loop.Corners[k] != filletband.CurvedMiter {
					continue
				}
				prev := fr.Walks[(k+n-1)%n]
				allow, ok := filletband.CurvedMiterPatchAreaAllowance(piece, fr.Walks[i], prev, fr.Walks[k], i == k, rIv)
				if !ok {
					return fmt.Errorf(`%w: a curved fillet patch has no area allowance`, ErrUnsupported)
				}
				areaIv = proofbound.IntervalWiden(areaIv, allow)
			}
		}
		held, bound := heldOf(areaIv)
		face := bornFace(&Face{n: topo.Face{Area: held, AreaBound: bound, Surface: surf, Origins: []provenance.Ref{{Producer: ref, Role: b.PatchRole(p)}}, Body: bodyNode(body), Loops: loopNodes(loops)}})
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(held, bound))
		// capV, a cap contact vertex, lies on the patch's surface; the outward
		// normal there is F's own, which faces −m along F's normal for a band
		// that removes material.
		nrm, err := face.NormalAt(capV.n.Position)
		if err != nil {
			return fmt.Errorf(`%w: a loop fillet band patch's outward orientation cannot be certified; its own normal reading refuses at its cap contact (%v)`, ErrUnsupported, err)
		}
		if nrm.Value.Dot(up) < 0 {
			face.n.Reversed = !face.n.Reversed
		}
		if b.Sigma > 0 {
			face.n.Reversed = !face.n.Reversed
		}
		for _, l := range loops {
			for _, ce := range l.n.CoEdges {
				ce.Edge.Faces = append(ce.Edge.Faces, faceNode(face))
			}
		}
		patches = append(patches, face)
		return nil
	}
	torus := func(cu, cv, major float64) surfacegeom.Torus {
		return surfacegeom.Torus{Center: pl.Point(cu, cv, sideZ), Axis: pl.Dir(0, 0, 1), Major: units.Millimeters(major), Minor: units.Millimeters(r)}
	}
	walkSurface := func(w survey2d.SideWalk) surfacegeom.Surface {
		if w.IsCircular() {
			if filletband.SphereWalk(w, r) {
				return surfacegeom.Sphere{Center: pl.Point(w.CU, w.CV, sideZ), Radius: units.Millimeters(r)}
			}
			major := w.Radius - r
			if w.Th1 < w.Th0 {
				major = w.Radius + r
			}
			return torus(w.CU, w.CV, major)
		}
		tu, tv := walkTangent(w)
		return surfacegeom.Cylinder{Origin: pl.Point(w.StartU-r*tv, w.StartV+r*tu, sideZ), Axis: pl.Dir(tu, tv, 0), Radius: units.Millimeters(r)}
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

	if fr.Loop.WholeTurn() {
		if len(capCo) != 1 {
			return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's cap contour is no one whole circle`, ErrUnsupported, bi)
		}
		side, capE := sideCo[0].Edge, capCo[0].Edge
		side.Convex, capE.Convex = convex, convex
		if err := newPatch(walkSurface(fr.Walks[0]), []*Loop{
			bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: side, Forward: true}}}}),
			bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: capE, Forward: false}}}}),
		}, vertexHandle(capE.Start)); err != nil {
			return nil, brepBandMass{}, err
		}
	} else {
		supplied, err := suppliedFilletCapWalls(capCo, fr.Walks, fr.Joins, r)
		if err != nil {
			return nil, brepBandMass{}, err
		}
		capAt := func(k int) *Vertex {
			if supplied.Wall[k] != nil {
				return vertexHandle(supplied.Wall[k].Start)
			}
			return vertexHandle(supplied.Wall[(k+1)%n].Start)
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
		meridian := func(foot planePoint, v survey2d.SideWalk, straight *survey2d.SideWalk, capV, sideV *Vertex) *Edge {
			nu, nv := foot.U-v.StartU, foot.V-v.StartV
			l := math.Hypot(nu, nv)
			normal := pl.Dir(nu/l, nv/l, 0)
			axis, _ := normal.Cross(up).Normalize()
			if straight != nil {
				tu, tv := walkTangent(*straight)
				tangent := pl.Dir(tu, tv, 0)
				if axis.Dot(tangent) < 0 {
					tangent = tangent.Scale(-1)
				}
				axis = tangent
			}
			return bornEdge(&Edge{n: topo.Edge{Convex: convex, Length: meridianLen, LengthBound: meridianBound, Curve: surfacegeom.Arc3{Center: pl.Point(foot.U, foot.V, sideZ), Axis: axis, Radius: units.Millimeters(r)}, Start: vertexNode(capV), End: vertexNode(sideV)}})
		}
		lead, trail := make([]*Edge, n), make([]*Edge, n)
		for k := range n {
			j := fr.Joins[k]
			w := fr.Walks[k]
			sideV := vertexHandle(sideCo[k].Edge.Start)
			switch fr.Loop.Corners[k] {
			case filletband.Reflex:
				arc := supplied.Arc[k]
				if arc == nil {
					return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's LF6 corner %d has no connector arc`, ErrUnsupported, bi, k)
				}
				arc.Convex = convex
				trail[k] = meridian(j.PA, w, straightWalk(w, nil), vertexHandle(arc.Start), sideV)
				lead[k] = meridian(j.PB, w, straightWalk(w, nil), vertexHandle(arc.End), sideV)
			case filletband.Tangent:
				prev := fr.Walks[(k+n-1)%n]
				lead[k] = meridian(j.M, w, straightWalk(prev, &w), capAt(k), sideV)
				trail[k] = lead[k]
			case filletband.CurvedMiter:
				prev := fr.Walks[(k+n-1)%n]
				length, ok := filletband.CurvedMiterLength(prev, w, r)
				if !ok {
					return nil, brepBandMass{}, fmt.Errorf(`%w: a loop fillet's curved miter at (%s, %s) has no regular offset locus (loop-fillet SF1)`,
						ErrUnsupported, renderCoord(w.StartU), renderCoord(w.StartV))
				}
				held, bound := heldOf(length)
				lead[k] = bornEdge(&Edge{
					n: topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.FilletMiter3{}, Start: vertexNode(capAt(k)), End: vertexNode(sideV)}})
				trail[k] = lead[k]
			default:
				kappa, err := filletband.Kappa(fr.Loop.Walks[(k+n-1)%n], fr.Loop.Walks[k])
				if err != nil {
					return nil, brepBandMass{}, err
				}
				lenIv, err := filletband.QuarterEllipseLength(rIv, kappa)
				if err != nil {
					return nil, brepBandMass{}, err
				}
				held, bound := heldOf(lenIv)
				bu, bv := j.M.U-w.StartU, j.M.V-w.StartV
				semi := math.Hypot(bu, bv)
				major := pl.Dir(bu/semi, bv/semi, 0)
				axis, _ := major.Cross(up).Normalize()
				lead[k] = bornEdge(&Edge{n: topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.Ellipse3{Center: pl.Point(j.M.U, j.M.V, sideZ), Axis: axis, Major: major,
					SemiMajor: units.Millimeters(semi), SemiMinor: units.Millimeters(r)},
					Start: vertexNode(capAt(k)),
					End:   vertexNode(sideV),
				},
				})
				trail[k] = lead[k]
			}
		}
		for i := range n {
			next := (i + 1) % n
			wall := supplied.Wall[i]
			if wall == nil {
				pole := capAt(i)
				if pole != capAt(next) || vertexHandle(lead[i].n.Start) != pole || vertexHandle(trail[next].n.Start) != pole ||
					sideCo[i].Edge.Start != lead[i].n.End || sideCo[i].Edge.End != trail[next].n.End {
					return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's sphere patch %d does not close`, ErrUnsupported, bi, i)
				}
				sideCo[i].Edge.Convex = convex
				if err := newPatch(walkSurface(fr.Walks[i]), []*Loop{bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
					{Edge: sideCo[i].Edge, Forward: true},
					{Edge: edgeNode(trail[next]), Forward: false},
					{Edge: edgeNode(lead[i]), Forward: true},
				},
				}})}, pole); err != nil {
					return nil, brepBandMass{}, err
				}
				continue
			}
			if wall.Start != lead[i].n.Start || wall.End != trail[next].n.Start ||
				sideCo[i].Edge.Start != lead[i].n.End || sideCo[i].Edge.End != trail[next].n.End {
				return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d's walk %d does not close on its contour and side vertices`, ErrUnsupported, bi, i)
			}
			wall.Convex, sideCo[i].Edge.Convex = convex, convex
			if err := newPatch(walkSurface(fr.Walks[i]), []*Loop{bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: sideCo[i].Edge, Forward: true},
				{Edge: edgeNode(trail[next]), Forward: false},
				{Edge: wall, Forward: false},
				{Edge: edgeNode(lead[i]), Forward: true},
			},
			}})}, vertexHandle(wall.Start)); err != nil {
				return nil, brepBandMass{}, err
			}
			if fr.Loop.Corners[next] != filletband.Reflex {
				continue
			}
			v := fr.Walks[next]
			if err := newPatch(torus(v.StartU, v.StartV, r), []*Loop{bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: supplied.Arc[next], Forward: true},
				{Edge: edgeNode(lead[next]), Forward: true},
				{Edge: edgeNode(trail[next]), Forward: false},
			},
			}})}, vertexHandle(supplied.Arc[next].Start)); err != nil {
				return nil, brepBandMass{}, err
			}
		}
	}
	if len(patches) != len(fr.Pieces) {
		return nil, brepBandMass{}, fmt.Errorf(`%w: fillet band %d built %d patches for %d pieces`, ErrUnsupported, bi, len(patches), len(fr.Pieces))
	}
	terms, err := filletband.MassOf(filletband.BandMassInput{
		Pieces: fr.Pieces, Walks: fr.Walks, Corners: fr.Loop.Corners,
		Radius: rIv, Level: f.Z0, LevelDelta: f.Z0Delta,
		MaterialSign: m, Sigma: b.Sigma, Embed: e,
	})
	if err != nil {
		return nil, brepBandMass{}, err
	}
	mass := brepBandMass{vol3: terms.Volume3, moments: terms.Moments, area: area}
	return patches, mass, nil
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
func (bp brepPayload) FilletRestored(btopo *brepTopology) ([]brepFace, error) {
	out := bp.Faces
	if !bp.HasFilletBand() {
		return out, nil
	}
	out = append([]brepFace(nil), bp.Faces...)
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range btopo.Open {
		openAt[btopo.Uses[ui].Key] = ui
	}
	cloneRegion := func(fi int) *profileRecord {
		src := out[fi].Region
		region := profileRecord{Outer: cloneLoopRecord(src.Outer)}
		for _, h := range src.Holes {
			region.Holes = append(region.Holes, cloneLoopRecord(h))
		}
		return &region
	}
	work := freeform.NewFreeformWork()
	for bi, b := range bp.LoopBands {
		if b.Kind != brepBandFillet {
			continue
		}
		if b.Selected != nil {
			for fi, original := range b.Restore {
				out[fi] = original
			}
			continue
		}
		f := bp.Faces[b.Face]
		eF := btopo.Embeds[b.Face]
		n := eF.Axis[2]
		levelRef := eF.Canon(0, 0, f.Z0)[n]
		sideZ, _ := b.SideLevel(f)
		sideRef := eF.Canon(0, 0, sideZ)[n]
		region := cloneRegion(b.Face)
		if b.Loop == 0 {
			region.Outer = cloneLoopRecord(b.Orig)
		} else {
			region.Holes[b.Loop-1] = cloneLoopRecord(b.Orig)
		}
		out[b.Face].Region = region
		for _, seg := range b.Orig.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			key, _ := brepgeom.CurveKey(eF, w, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return nil, fmt.Errorf(`%w: fillet band %d's side contour is not a boundary of the record`, ErrUnsupported, bi)
			}
			u := btopo.Uses[ui]
			eA := btopo.Embeds[u.Face]
			a := &out[u.Face]
			switch u.Part {
			case brepRim0:
				a.Z0 = eA.Sign[2]*levelRef + 0
			case brepRim1:
				a.Z1 = eA.Sign[2]*levelRef + 0
			case brepLoopSeg:
				a.Region = cloneRegion(u.Face)
				if err := brepgeom.RestoreSideSegment(a.Region, a.Z0, eA, u.Loop, u.Seg, n, sideRef, levelRef); err != nil {
					return nil, fmt.Errorf("fillet band %d: %w", bi, err)
				}
			default:
				return nil, fmt.Errorf(`%w: fillet band %d's side contour runs along a side line`, ErrUnsupported, bi)
			}
		}
	}
	return out, nil
}

// filletBandExtent is fillet band b's extent along world direction g
// (loop-fillet §5.4): filletband.Extreme read in F's own frame along g and
// along −g, lifted through F's frame and the placement as a prism face's
// extent is (prismextent.ExtentAlong's composition): the enclosure's own
// rounding, the frame-and-placement coefficients' allowance, and each
// endpoint sum's rounding.
func filletBandExtent(ctx context.Context, b brepLoopBand, f brepFace, xform r3.Transform, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	pl := f.View(xform)
	rIv, err := filletband.RadiusInterval(b.Setback.Dc, b.Setback.DcDelta)
	if err != nil {
		return 0, 0, 0, err
	}
	fr, err := filletLoopOf(proofbound.NewWorkBudget(ctx), b.Orig, b.Setback.Dc, f.Role, work)
	if err != nil {
		return 0, 0, 0, err
	}
	m := b.MatSign(f)
	sideZ, sideDelta := b.SideLevel(f)
	level := proofarith.FloatRat(f.Z0)
	if level == nil {
		return 0, 0, 0, fmt.Errorf(`%w: a fillet band's face level is not finite`, ErrNotFinite)
	}
	sideIv := proofbound.IntervalAdd(proofbound.PointInterval(level), proofbound.IntervalScale(rIv, big.NewRat(int64(m), 1)))
	base := pl.Xform.Apply(pl.Frame.Origin()).Dot(g)
	coeff := [3]float64{pl.Dir(1, 0, 0).Dot(g), pl.Dir(0, 1, 0).Dot(g), pl.Dir(0, 0, 1).Dot(g)}
	loIv, hiIv, err := filletband.ExtentsOf(fr.Pieces, fr.Walks, fr.Loop.Corners,
		m, b.Setback.Dc, rIv, sideIv, coeff)
	if err != nil {
		return 0, 0, 0, err
	}
	coordUpper := filletband.WalkCoordUpper(fr.Walks, b.Setback.AxialUpper())
	zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(f.Z0), math.Abs(sideZ)), sideDelta)
	placeAllow := prismrecord.PlacementCoeffAllow(pl, g, base, coeff[0], coeff[1], coeff[2], coordUpper, zUpper)
	lo, loErr := heldOf(loIv)
	hi, hiErr := heldOf(hiIv)
	loEnd, hiEnd := base+lo, base+hi
	bound := proofbound.AbsSumUpper(math.Max(loErr, hiErr), placeAllow, sideDelta,
		math.Max(proofbound.ExactSumRound(loEnd, base, lo), proofbound.ExactSumRound(hiEnd, base, hi)))
	return loEnd, hiEnd, bound, nil
}

// hasFilletBand reports whether the record carries a fillet band.
func (bp brepPayload) HasFilletBand() bool {
	for _, b := range bp.LoopBands {
		if b.Kind == brepBandFillet {
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
	f := bp.Faces[band.Face]
	view := f.View(bp.Xform)
	m, ok := survey2d.NewPlacedFrameMap(view.Frame, view.Xform)
	if !ok {
		return false
	}
	fr, err := filletLoopOf(budget, band.Orig, band.Setback.Dc, f.Role, work)
	if err != nil {
		return false
	}
	sign := int64(-1)
	if f.Outward {
		sign = 1
	}
	sigma := int64(band.Sigma)
	n := len(fr.Walks)
	p := 0
	patch := func(rng survey2d.PullBracket, ok bool) bool {
		if !ok {
			return false
		}
		face := roles[band.PatchRole(p)]
		p++
		if face == nil {
			return false
		}
		verdict, ok := survey2d.PatchPullVerdict(m, pull, sign, rng)
		return listVerdict(faces, undecided, face, verdict, ok)
	}
	for i, w := range fr.Walks {
		if !patch(survey2d.InwardRange(w, m, pull, sigma)) {
			return false
		}
		if ni := (i + 1) % n; !fr.Loop.WholeTurn() && fr.Loop.Corners[ni] == filletband.Reflex {
			if !patch(survey2d.CornerRange(w, fr.Walks[ni], m, pull, sigma)) {
				return false
			}
		}
	}
	return p == len(fr.Pieces)
}

// walkTangent is a straight side walk's unit direction in its plane's (u, v).
func walkTangent(w survey2d.SideWalk) (float64, float64) {
	tu, tv := w.EndU-w.StartU, w.EndV-w.StartV
	l := math.Hypot(tu, tv)
	return tu / l, tv / l
}
