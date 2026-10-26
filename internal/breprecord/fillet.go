package breprecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is route L's fillet arm (agent-docs/loop-fillet-design.md, "loop-fillet
// §N" below): the corner classes of Table LF and SF1's refusal
// (FilletLoopRead), the pipe band's faces, edges and roles (attachFilletBand),
// its strip terms in the body's mass sums (§5.3), the receiver's faces
// restored for those sums (filletRestored), and the band's extent along a
// direction (§5.4). internal/filletband owns every closed form.

// FilletLoopRead is one fillet band's loop as Table LF reads it: the
// coalesced walks of orig, the cap contour's joins at the radius, and the
// band's patches in its own order (filletband.Loop.Pieces).
type FilletLoopRead struct {
	Loop   filletband.Loop
	Walks  []survey2d.SideWalk
	Joins  []offset2d.Join
	Pieces []filletband.Piece
}

// FilletLoopOf classifies every corner of loop, walked with face F's material
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
func FilletLoopOf(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, r float64, role string, work *freeform.FreeformWork) (FilletLoopRead, error) {
	cl, err := capblend.OneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return FilletLoopRead{}, err
	}
	read, joins, pieces, bad, err := filletband.ReadLoop(budget, loop, cl.Walks, r, offset2d.ShellTol)
	if err != nil {
		return FilletLoopRead{}, err
	}
	if bad >= 0 {
		n := len(cl.Walks)
		return FilletLoopRead{}, fmt.Errorf(`%w: a fillet of a complete loop of brep face %s has no supported join at corner (%s, %s) between recorded segments %d and %d (loop-fillet SF1)`,
			decaderr.ErrUnsupported, role, selectorquery.RenderCoord(cl.Walks[bad].StartU), selectorquery.RenderCoord(cl.Walks[bad].StartV),
			cl.Walks[(bad+n-1)%n].Segs[len(cl.Walks[(bad+n-1)%n].Segs)-1], cl.Walks[bad].Segs[0])
	}
	return FilletLoopRead{Loop: read, Walks: cl.Walks, Joins: joins, Pieces: pieces}, nil
}

// suppliedFilletCapWalls pairs the contour's edges with the original walks.
// A collapsed inward arc has no cap edge; its two neighbouring edges share
// the pole vertex at the arc's recorded centre.
func suppliedFilletCapWalls(co []topo.CoEdge, walks []survey2d.SideWalk, joins []offset2d.Join, r float64) (capblend.SuppliedCapEdges, error) {
	n := len(walks)
	out := capblend.SuppliedCapEdges{Wall: make([]*topo.Edge, n), Arc: make([]*topo.Edge, n)}
	next := 0
	take := func() (*topo.Edge, error) {
		if next >= len(co) || !co[next].Forward {
			return nil, fmt.Errorf(`%w: a fillet band's cap contour does not match its walks`, decaderr.ErrUnsupported)
		}
		e := co[next].Edge
		next++
		return e, nil
	}
	for i, w := range walks {
		if !filletband.SphereWalk(w, r) {
			e, err := take()
			if err != nil {
				return capblend.SuppliedCapEdges{}, err
			}
			out.Wall[i] = e
		}
		j := (i + 1) % n
		if !joins[j].Arc {
			continue
		}
		e, err := take()
		if err != nil {
			return capblend.SuppliedCapEdges{}, err
		}
		out.Arc[j] = e
	}
	if next != len(co) {
		return capblend.SuppliedCapEdges{}, fmt.Errorf(`%w: a fillet band's cap contour has extra edges`, decaderr.ErrUnsupported)
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
// the record's band boundary edges already placed (openEdges): the cap
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
// integrated: measureBody reads the record restored (filletRestored)
// in their place.
func attachFilletBand(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer, bp Payload, bi int, open openSet, e brepgeom.Embed, work *freeform.FreeformWork) ([]*topo.Face, bandMass, error) {
	b := bp.LoopBands[bi]
	f := bp.Faces[b.Face]
	pl := f.View(bp.Xform)
	m := b.MatSign(f)
	sideZ, _ := b.SideLevel(f)
	r := b.Setback.Dc
	rIv, err := filletband.RadiusInterval(b.Setback.Dc, b.Setback.DcDelta)
	if err != nil {
		return nil, bandMass{}, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	fr, err := FilletLoopOf(budget, b.Orig, r, f.Role, work)
	if err != nil {
		return nil, bandMass{}, err
	}
	convex := b.Sigma < 0
	up := pl.Dir(0, 0, -m)
	sideCo, capCo := open.side[bi], open.cap[bi]
	n := len(fr.Walks)
	if len(sideCo) != n {
		return nil, bandMass{}, fmt.Errorf(`%w: fillet band %d's side contour holds %d edges for %d walks`, decaderr.ErrUnsupported, bi, len(sideCo), n)
	}

	var patches []*topo.Face
	area := proofbound.BoundedScalar{}
	newPatch := func(surf surfacegeom.Surface, loops []*topo.Loop, capV *topo.Vertex) error {
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
					return fmt.Errorf(`%w: a curved fillet patch has no area allowance`, decaderr.ErrUnsupported)
				}
				areaIv = proofbound.IntervalWiden(areaIv, allow)
			}
		}
		held, bound := heldOf(areaIv)
		face := topobuild.NewFace(alloc, topo.Face{Area: held, AreaBound: bound, Surface: surf, Origins: []provenance.Ref{{Producer: ref, Role: b.PatchRole(p)}}, Body: body, Loops: loops})
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(held, bound))
		// capV, a cap contact vertex, lies on the patch's surface; the outward
		// normal there is F's own, which faces −m along F's normal for a band
		// that removes material.
		nrm, err := topobuild.NormalAt(face, capV.Position)
		if err != nil {
			return fmt.Errorf(`%w: a loop fillet band patch's outward orientation cannot be certified; its own normal reading refuses at its cap contact (%v)`, decaderr.ErrUnsupported, err)
		}
		if nrm.Value.Dot(up) < 0 {
			face.Reversed = !face.Reversed
		}
		if b.Sigma > 0 {
			face.Reversed = !face.Reversed
		}
		for _, l := range loops {
			for _, ce := range l.CoEdges {
				ce.Edge.Faces = append(ce.Edge.Faces, face)
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
			return nil, bandMass{}, fmt.Errorf(`%w: fillet band %d's cap contour is no one whole circle`, decaderr.ErrUnsupported, bi)
		}
		side, capE := sideCo[0].Edge, capCo[0].Edge
		side.Convex, capE.Convex = convex, convex
		if err := newPatch(walkSurface(fr.Walks[0]), []*topo.Loop{
			topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: side, Forward: true}}}),
			topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: capE, Forward: false}}}),
		}, capE.Start); err != nil {
			return nil, bandMass{}, err
		}
	} else {
		supplied, err := suppliedFilletCapWalls(capCo, fr.Walks, fr.Joins, r)
		if err != nil {
			return nil, bandMass{}, err
		}
		capAt := func(k int) *topo.Vertex {
			if supplied.Wall[k] != nil {
				return supplied.Wall[k].Start
			}
			return supplied.Wall[(k+1)%n].Start
		}
		meridianLen, meridianBound := heldOf(filletband.MeridianLength(rIv))
		// meridian is the quarter circle of radius r about the ball centre
		// under the contour foot, from that foot at F's level down to the side
		// vertex.
		//
		// Given a straight walk, its axis is that walk's own direction, the
		// very value the walk's cylinder carries as its Axis, up to sign, so a
		// consumer comparing the two compares equal values: the cross product
		// of the foot's normal with up is perpendicular to the walk only to
		// rounding. Given none it keeps that cross product. An LF5 meridian
		// passes the first straight walk of the two it joins; an LF6 meridian
		// passes only the one walk whose wall it bounds, so it keeps the cross
		// product whenever that walk is circular, even beside a straight walk.
		meridian := func(foot sectionrecord.Point2, v survey2d.SideWalk, straight *survey2d.SideWalk, capV, sideV *topo.Vertex) *topo.Edge {
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
			return topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: meridianLen, LengthBound: meridianBound, Curve: surfacegeom.Arc3{Center: pl.Point(foot.U, foot.V, sideZ), Axis: axis, Radius: units.Millimeters(r)}, Start: capV, End: sideV})
		}
		lead, trail := make([]*topo.Edge, n), make([]*topo.Edge, n)
		for k := range n {
			j := fr.Joins[k]
			w := fr.Walks[k]
			sideV := sideCo[k].Edge.Start
			switch fr.Loop.Corners[k] {
			case filletband.Reflex:
				arc := supplied.Arc[k]
				if arc == nil {
					return nil, bandMass{}, fmt.Errorf(`%w: fillet band %d's LF6 corner %d has no connector arc`, decaderr.ErrUnsupported, bi, k)
				}
				arc.Convex = convex
				// The connector arc runs from the previous walk's offset foot
				// PA to this walk's PB, so the trailing meridian stands on the
				// previous walk's wall and takes that walk's axis.
				prev := fr.Walks[(k+n-1)%n]
				trail[k] = meridian(j.PA, w, straightWalk(prev, nil), arc.Start, sideV)
				lead[k] = meridian(j.PB, w, straightWalk(w, nil), arc.End, sideV)
			case filletband.Tangent:
				prev := fr.Walks[(k+n-1)%n]
				lead[k] = meridian(j.M, w, straightWalk(prev, &w), capAt(k), sideV)
				trail[k] = lead[k]
			case filletband.CurvedMiter:
				prev := fr.Walks[(k+n-1)%n]
				length, ok := filletband.CurvedMiterLength(prev, w, r)
				if !ok {
					return nil, bandMass{}, fmt.Errorf(`%w: a loop fillet's curved miter at (%s, %s) has no regular offset locus (loop-fillet SF1)`,
						decaderr.ErrUnsupported, selectorquery.RenderCoord(w.StartU), selectorquery.RenderCoord(w.StartV))
				}
				held, bound := heldOf(length)
				lead[k] = topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.FilletMiter3{}, Start: capAt(k), End: sideV})
				trail[k] = lead[k]
			default:
				kappa, err := filletband.Kappa(fr.Loop.Walks[(k+n-1)%n], fr.Loop.Walks[k])
				if err != nil {
					return nil, bandMass{}, err
				}
				lenIv, err := filletband.QuarterEllipseLength(rIv, kappa)
				if err != nil {
					return nil, bandMass{}, err
				}
				held, bound := heldOf(lenIv)
				bu, bv := j.M.U-w.StartU, j.M.V-w.StartV
				semi := math.Hypot(bu, bv)
				major := pl.Dir(bu/semi, bv/semi, 0)
				axis, _ := major.Cross(up).Normalize()
				lead[k] = topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.Ellipse3{Center: pl.Point(j.M.U, j.M.V, sideZ), Axis: axis, Major: major,
					SemiMajor: units.Millimeters(semi), SemiMinor: units.Millimeters(r)},
					Start: capAt(k),
					End:   sideV,
				})
				trail[k] = lead[k]
			}
		}
		for i := range n {
			next := (i + 1) % n
			wall := supplied.Wall[i]
			if wall == nil {
				pole := capAt(i)
				if pole != capAt(next) || lead[i].Start != pole || trail[next].Start != pole ||
					sideCo[i].Edge.Start != lead[i].End || sideCo[i].Edge.End != trail[next].End {
					return nil, bandMass{}, fmt.Errorf(`%w: fillet band %d's sphere patch %d does not close`, decaderr.ErrUnsupported, bi, i)
				}
				sideCo[i].Edge.Convex = convex
				if err := newPatch(walkSurface(fr.Walks[i]), []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
					{Edge: sideCo[i].Edge, Forward: true},
					{Edge: trail[next], Forward: false},
					{Edge: lead[i], Forward: true},
				},
				})}, pole); err != nil {
					return nil, bandMass{}, err
				}
				continue
			}
			if wall.Start != lead[i].Start || wall.End != trail[next].Start ||
				sideCo[i].Edge.Start != lead[i].End || sideCo[i].Edge.End != trail[next].End {
				return nil, bandMass{}, fmt.Errorf(`%w: fillet band %d's walk %d does not close on its contour and side vertices`, decaderr.ErrUnsupported, bi, i)
			}
			wall.Convex, sideCo[i].Edge.Convex = convex, convex
			if err := newPatch(walkSurface(fr.Walks[i]), []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: sideCo[i].Edge, Forward: true},
				{Edge: trail[next], Forward: false},
				{Edge: wall, Forward: false},
				{Edge: lead[i], Forward: true},
			},
			})}, wall.Start); err != nil {
				return nil, bandMass{}, err
			}
			if fr.Loop.Corners[next] != filletband.Reflex {
				continue
			}
			v := fr.Walks[next]
			if err := newPatch(torus(v.StartU, v.StartV, r), []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: supplied.Arc[next], Forward: true},
				{Edge: lead[next], Forward: true},
				{Edge: trail[next], Forward: false},
			},
			})}, supplied.Arc[next].Start); err != nil {
				return nil, bandMass{}, err
			}
		}
	}
	if len(patches) != len(fr.Pieces) {
		return nil, bandMass{}, fmt.Errorf(`%w: fillet band %d built %d patches for %d pieces`, decaderr.ErrUnsupported, bi, len(patches), len(fr.Pieces))
	}
	terms, err := filletband.MassOf(filletband.BandMassInput{
		Pieces: fr.Pieces, Walks: fr.Walks, Corners: fr.Loop.Corners,
		Radius: rIv, Level: f.Z0, LevelDelta: f.Z0Delta,
		MaterialSign: m, Sigma: b.Sigma, Embed: e,
	})
	if err != nil {
		return nil, bandMass{}, err
	}
	mass := bandMass{vol3: terms.Volume3, moments: terms.Moments, area: area}
	return patches, mass, nil
}

// filletRestored is the record with every fillet band undone, the faces
// loop-fillet §5.3 sums the receiver's terms over (LoopBand.restored's
// reading): F reads orig in its contour's place, each (sw) face beside the loop
// reads its level at F instead of the side level, and each (pl) face its
// segment on the loop back at F's level with its two neighbouring lines
// lengthened to it. The faces beside a band are the ones its side contour's
// open uses name, so the reading follows the rewritten record's own
// topology. Every other face, and every chamfer band's rewrite, is kept. Level
// and section displacements are kept as the rewritten record states them,
// which is never less than the receiver's. A record with no fillet band
// returns its faces unchanged.
func (bp Payload) FilletRestored(btopo *Topology) ([]Face, error) {
	out := bp.Faces
	if !bp.HasFilletBand() {
		return out, nil
	}
	out = append([]Face(nil), bp.Faces...)
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range btopo.Open {
		openAt[btopo.Uses[ui].Key] = ui
	}
	cloneRegion := func(fi int) *momentinput.Profile {
		src := out[fi].Region
		region := momentinput.Profile{Outer: sectionrecord.CloneLoopRecord(src.Outer)}
		for _, h := range src.Holes {
			region.Holes = append(region.Holes, sectionrecord.CloneLoopRecord(h))
		}
		return &region
	}
	work := freeform.NewFreeformWork()
	for bi, b := range bp.LoopBands {
		if b.Kind != BandFillet {
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
			region.Outer = sectionrecord.CloneLoopRecord(b.Orig)
		} else {
			region.Holes[b.Loop-1] = sectionrecord.CloneLoopRecord(b.Orig)
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
				return nil, fmt.Errorf(`%w: fillet band %d's side contour is not a boundary of the record`, decaderr.ErrUnsupported, bi)
			}
			u := btopo.Uses[ui]
			eA := btopo.Embeds[u.Face]
			a := &out[u.Face]
			switch u.Part {
			case brepgeom.Rim0:
				a.Z0 = eA.Sign[2]*levelRef + 0
			case brepgeom.Rim1:
				a.Z1 = eA.Sign[2]*levelRef + 0
			case brepgeom.LoopSeg:
				a.Region = cloneRegion(u.Face)
				if err := brepgeom.RestoreSideSegment(a.Region, a.Z0, eA, u.Loop, u.Seg, n, sideRef, levelRef); err != nil {
					return nil, fmt.Errorf("fillet band %d: %w", bi, err)
				}
			default:
				return nil, fmt.Errorf(`%w: fillet band %d's side contour runs along a side line`, decaderr.ErrUnsupported, bi)
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
func filletBandExtent(ctx context.Context, b LoopBand, f Face, xform r3.Transform, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	pl := f.View(xform)
	rIv, err := filletband.RadiusInterval(b.Setback.Dc, b.Setback.DcDelta)
	if err != nil {
		return 0, 0, 0, err
	}
	fr, err := FilletLoopOf(proofbound.NewWorkBudget(ctx), b.Orig, b.Setback.Dc, f.Role, work)
	if err != nil {
		return 0, 0, 0, err
	}
	m := b.MatSign(f)
	sideZ, sideDelta := b.SideLevel(f)
	level := proofarith.FloatRat(f.Z0)
	if level == nil {
		return 0, 0, 0, fmt.Errorf(`%w: a fillet band's face level is not finite`, decaderr.ErrNotFinite)
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
func (bp Payload) HasFilletBand() bool {
	for _, b := range bp.LoopBands {
		if b.Kind == BandFillet {
			return true
		}
	}
	return false
}

// walkTangent is a straight side walk's unit direction in its plane's (u, v).
func walkTangent(w survey2d.SideWalk) (float64, float64) {
	tu, tv := w.EndU-w.StartU, w.EndV-w.StartV
	l := math.Hypot(tu, tv)
	return tu / l, tv / l
}
