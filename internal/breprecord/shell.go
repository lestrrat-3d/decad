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
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// closedBossShellLumps assigns the inner face component to the one void shell.
// stackedshell's rectangular-boss builder has already proved that the nested cavity stays clear of
// every exterior wall, the floor and the kept roof. Every source face must
// therefore lie in the one outer component, and all derived cavity faces in
// exactly one other closed component.
func closedBossShellLumps(alloc topo.Alloc, faces []*topo.Face, outerFaceCount int) ([]*topo.Lump, error) {
	if outerFaceCount <= 0 || outerFaceCount >= len(faces) {
		return nil, fmt.Errorf("%w: a closed boss shell has no distinct exterior and cavity", decaderr.ErrUnsupported)
	}
	outerFaces := make(map[*topo.Face]struct{}, outerFaceCount)
	for _, face := range faces[:outerFaceCount] {
		outerFaces[face] = struct{}{}
	}
	groups := topobuild.SplitConnectedFaces(faces)
	if len(groups) != 2 {
		return nil, fmt.Errorf("%w: a closed boss shell has %d face components, expected an exterior and one cavity",
			decaderr.ErrUnsupported, len(groups))
	}
	var outer, cavity []*topo.Face
	for _, group := range groups {
		anchors := 0
		for _, face := range group {
			if _, ok := outerFaces[face]; ok {
				anchors++
			}
		}
		switch {
		case anchors == outerFaceCount && len(group) == outerFaceCount:
			outer = group
		case anchors == 0:
			cavity = group
		default:
			return nil, fmt.Errorf("%w: a closed boss shell mixes exterior and cavity faces", decaderr.ErrUnsupported)
		}
	}
	if len(outer) != outerFaceCount || len(cavity) != len(faces)-outerFaceCount ||
		topobuild.ShellIsOpen(outer) || topobuild.ShellIsOpen(cavity) ||
		!closedBossShellEdgePairs(outer) || !closedBossShellEdgePairs(cavity) {
		return nil, fmt.Errorf("%w: a closed boss shell needs two closed face components", decaderr.ErrUnsupported)
	}
	outerShell := alloc.NewFaceShell()
	outerShell.Faces = outer
	cavityShell := alloc.NewFaceShell()
	cavityShell.Void = true
	cavityShell.Faces = cavity
	lump := alloc.NewLump()
	lump.Shells = []*topo.FaceShell{outerShell, cavityShell}
	return []*topo.Lump{lump}, nil
}

// Each component must use every edge exactly twice. A free edge or a repeated
// edge cannot pass merely because the face graph has two components. The
// coedge's forward bit records curve direction, not outward face winding.
func closedBossShellEdgePairs(faces []*topo.Face) bool {
	edges := map[*topo.Edge]int{}
	for _, face := range faces {
		for _, loop := range face.Loops {
			for _, use := range loop.CoEdges {
				edges[use.Edge]++
			}
		}
	}
	if len(edges) == 0 {
		return false
	}
	for edge, uses := range edges {
		if uses != 2 || len(edge.Faces) != 2 || edge.Faces[0] == edge.Faces[1] {
			return false
		}
	}
	return true
}

// bossShellOpenEdges supplies the two four-edge rings in the common
// counter-clockwise walk. Each ordinary face uses its edge in its own sense.
func bossShellOpenEdges(ctx context.Context, alloc topo.Alloc, bp Payload, btopo *Topology,
	openAt map[brepgeom.EdgeKey]int,
	placeVertex func([3]float64, float64, float64, float64) *topo.Vertex,
	out *openSet, bi int) error {
	band := bp.BossShell
	e := btopo.Embeds[band.LedgeFace]
	for ring, spec := range []struct {
		profile momentinput.Profile
		z       float64
	}{{band.SideProfile(), band.LowerZ}, {band.CapProfile(), band.InterfaceZ}} {
		view := prismrecord.Payload{Frame: band.Frame, Xform: bp.Xform, Z0: spec.z, Z1: spec.z}
		for _, seg := range spec.profile.Outer.Segments {
			if err := ctx.Err(); err != nil {
				return err
			}
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return err
			}
			key, _ := brepgeom.CurveKey(e, w, spec.z)
			ui, ok := openAt[key]
			if !ok {
				return fmt.Errorf("%w: a boss shell ring has no open record edge", decaderr.ErrUnsupported)
			}
			u := btopo.Uses[ui]
			from, to := e.Canon(w.StartU, w.StartV, spec.z), e.Canon(w.EndU, w.EndV, spec.z)
			start, end := placeVertex(from, 0, 0, 0), placeVertex(to, 0, 0, 0)
			edge, err := bandEdge(alloc, view, w, true, start, end)
			if err != nil {
				return err
			}
			out.coedge[ui] = topo.CoEdge{Edge: edge, Forward: u.DirFrom == from && u.DirTo == to}
			if ring == 0 {
				out.side[bi] = append(out.side[bi], topo.CoEdge{Edge: edge, Forward: true})
			}
			if ring == 1 {
				out.cap[bi] = append(out.cap[bi], topo.CoEdge{Edge: edge, Forward: true})
			}
		}
	}
	return nil
}

// attachBossShellBand builds the cavity's root transition. Every cavity
// point keeps distance t from the source's root edge, the boss outline at
// InterfaceZ, so each patch lies on the cylinder of radius t about one root
// edge (a whole circle: the torus about it), on the cavity side below and
// inside the root (agent-docs/modify-general-design.md §3.1a, §3.1c). The
// patch runs from the side ring, the boss outline at LowerZ straight below
// the axis, to the cap ring, the upper inset at InterfaceZ, beside it. Two
// rectangular patches meet on the quarter ellipse the bisector plane of their
// root corner cuts from both cylinders. The solid lies inside each cylinder,
// so every patch's outward normal points away from its axis.
func attachBossShellBand(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer, bp Payload, open openSet) ([]*topo.Face, error) {
	band := bp.BossShell
	bi := len(bp.LoopBands)
	role := LoopBand{Face: band.LedgeFace, Loop: 0, Kind: BandFillet}
	pl := Face{Frame: band.Frame}.View(bp.Xform)
	sideCo, capCo := open.side[bi], open.cap[bi]
	rIv, err := filletband.RadiusInterval(band.T, 0)
	if err != nil {
		return nil, err
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	down := pl.Dir(0, 0, -1)
	var patches []*topo.Face
	newPatch := func(surf surfacegeom.Surface, loops []*topo.Loop, areaIv proofbound.RatInterval, sideV *topo.Vertex) error {
		held, bound := heldOf(areaIv)
		face := topobuild.NewFace(alloc, topo.Face{Area: held, AreaBound: bound, Surface: surf,
			Origins: []provenance.Ref{{Producer: ref, Role: role.PatchRole(len(patches))}}, Body: body, Loops: loops})
		// The side ring lies straight below the axis, where the outward
		// normal, away from the axis, points down the frame normal.
		nrm, err := topobuild.NormalAt(face, sideV.Position)
		if err != nil {
			return fmt.Errorf(`%w: a boss shell root patch's outward orientation cannot be certified (%v)`, decaderr.ErrUnsupported, err)
		}
		if nrm.Value.Dot(down) < 0 {
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
	if circle := band.Round; circle != nil {
		if len(sideCo) != 1 || len(capCo) != 1 {
			return nil, fmt.Errorf(`%w: a round boss shell band has no single whole-circle ring pair`, decaderr.ErrUnsupported)
		}
		side, capE := sideCo[0].Edge, capCo[0].Edge
		side.Convex, capE.Convex = false, false
		// Pappus: the quarter-circle meridian of length πt/2 has its centroid
		// 2t/π inside the root circle, so the area is 2π(r − 2t/π)·πt/2.
		r := proofbound.PointInterval(proofarith.FloatRat(circle.Radius))
		areaIv := proofbound.IntervalSub(proofbound.IntervalMul(proofbound.IntervalMul(pi, pi), proofbound.IntervalMul(r, rIv)),
			proofbound.IntervalScale(proofbound.IntervalMul(pi, proofbound.IntervalMul(rIv, rIv)), big.NewRat(2, 1)))
		torus := surfacegeom.Torus{Center: pl.Point(circle.Center.U, circle.Center.V, band.InterfaceZ),
			Axis: pl.Dir(0, 0, 1), Major: units.Millimeters(circle.Radius), Minor: units.Millimeters(band.T)}
		if err := newPatch(torus, []*topo.Loop{
			topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: side, Forward: true}}}),
			topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: capE, Forward: false}}}),
		}, areaIv, side.Start); err != nil {
			return nil, err
		}
		return patches, nil
	}
	n := len(sideCo)
	if n != 4 || len(capCo) != 4 {
		return nil, fmt.Errorf(`%w: a rectangular boss shell band has %d side and %d cap edges, expected four each`,
			decaderr.ErrUnsupported, n, len(capCo))
	}
	// Every root corner is a right angle, so each seam is the quarter of the
	// ellipse with semi-axes t·√2 and t (filletband's κ = 1).
	seamIv, err := filletband.QuarterEllipseLength(rIv, proofbound.PointInterval(big.NewRat(1, 1)))
	if err != nil {
		return nil, err
	}
	seamLen, seamBound := heldOf(seamIv)
	walks := band.SideProfile().Outer.Segments
	if len(walks) != n {
		return nil, fmt.Errorf(`%w: a rectangular boss shell's root outline does not hold four edges`, decaderr.ErrUnsupported)
	}
	insets := band.CapProfile().Outer.Segments
	if len(insets) != n {
		return nil, fmt.Errorf(`%w: a rectangular boss shell's inset outline does not hold four edges`, decaderr.ErrUnsupported)
	}
	roots := make([]sectionrecord.LineSeg, n)
	seams := make([]*topo.Edge, n)
	for k := range n {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		root, ok := walks[k].(sectionrecord.LineSeg)
		inset, insetOK := insets[k].(sectionrecord.LineSeg)
		if !ok || !insetOK {
			return nil, fmt.Errorf(`%w: a rectangular boss shell's root outline holds a curved segment`, decaderr.ErrUnsupported)
		}
		roots[k] = root
		capV, sideV := capCo[k].Edge.Start, sideCo[k].Edge.Start
		// The seam's ellipse is centred on root corner k at the root level.
		// Its major axis points from the corner toward the inset corner, the
		// seam's start at the cap ring; its minor axis points down to the
		// corner at the side ring, a quarter turn on.
		center := pl.Point(root.Start.U, root.Start.V, band.InterfaceZ)
		if !bossSeamCornerPaired(root.Start, inset.Start, band.Boss, band.T) ||
			bossSeamFalsified(capV.Position, sideV.Position, center, down, band.T) {
			return nil, fmt.Errorf(`%w: a rectangular boss shell's ring corner %d is not its root corner's inset`, decaderr.ErrUnsupported, k)
		}
		major, ok := capV.Position.Sub(center).Normalize()
		if !ok {
			return nil, fmt.Errorf(`%w: a rectangular boss shell seam has no major axis`, decaderr.ErrUnsupported)
		}
		axis, ok := major.Cross(down).Normalize()
		if !ok {
			return nil, fmt.Errorf(`%w: a rectangular boss shell seam has no plane`, decaderr.ErrUnsupported)
		}
		seams[k] = topobuild.NewEdge(alloc, topo.Edge{Length: seamLen, LengthBound: seamBound,
			Curve: surfacegeom.Ellipse3{Center: center, Axis: axis, Major: major,
				SemiMajor: units.Millimeters(math.Sqrt2 * band.T), SemiMinor: units.Millimeters(band.T)},
			Start: capV, End: sideV})
	}
	for i := range n {
		next := (i + 1) % n
		side, wall := sideCo[i].Edge, capCo[i].Edge
		if wall.Start != seams[i].Start || wall.End != seams[next].Start ||
			side.Start != seams[i].End || side.End != seams[next].End {
			return nil, fmt.Errorf(`%w: a rectangular boss shell root patch %d does not close on its ring vertices`, decaderr.ErrUnsupported, i)
		}
		side.Convex, wall.Convex = false, false
		root := roots[i]
		// stackedshell's gate proved the boss width and height exact, so
		// each axis-aligned root edge's length is one exact difference.
		du, dv := root.End.U-root.Start.U, root.End.V-root.Start.V
		if (du == 0) == (dv == 0) {
			return nil, fmt.Errorf(`%w: a rectangular boss shell root edge is not axis-aligned`, decaderr.ErrUnsupported)
		}
		length := math.Abs(du) + math.Abs(dv)
		// At angle α from the side ring, the patch's axial length is the root
		// edge's length L less 2t·sin α, so its area is t·πL/2 − 2t².
		lIv := proofbound.PointInterval(proofarith.FloatRat(length))
		areaIv := proofbound.IntervalSub(proofbound.IntervalMul(proofbound.IntervalMul(rIv, lIv), proofbound.HalfPiInterval()),
			proofbound.IntervalScale(proofbound.IntervalMul(rIv, rIv), big.NewRat(2, 1)))
		cylinder := surfacegeom.Cylinder{Origin: pl.Point(root.Start.U, root.Start.V, band.InterfaceZ),
			Axis: pl.Dir(du/length, dv/length, 0), Radius: units.Millimeters(band.T)}
		if err := newPatch(cylinder, []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
			{Edge: side, Forward: true},
			{Edge: seams[next], Forward: false},
			{Edge: wall, Forward: false},
			{Edge: seams[i], Forward: true},
		}})}, areaIv, side.Start); err != nil {
			return nil, err
		}
	}
	return patches, nil
}

// bossSeamCornerPaired reports whether inset is root corner root of boss
// moved t toward the boss interior along both axes. stackedshell proved every
// inset coordinate exact (prismshell.BossRectArithmeticExact), so the
// comparison is exact equality. A root that is no corner of boss fails.
func bossSeamCornerPaired(root, inset sectionrecord.Point2, boss prismshell.BossRect, t float64) bool {
	toward := func(x, lo, hi float64) (float64, bool) {
		switch x {
		case lo:
			return x + t, true
		case hi:
			return x - t, true
		}
		return 0, false
	}
	u, okU := toward(root.U, boss.U0, boss.U1)
	v, okV := toward(root.V, boss.V0, boss.V1)
	return okU && okV && inset.U == u && inset.V == v
}

// bossSeamFalsified reports whether the placed ring vertices disprove a
// seam centred at center: the cap vertex must lie t·√2 from it and the side
// vertex at center + t·down. It only rejects. Its allowance, 1e-9 of the
// coordinates' magnitude, far exceeds the rounding of the placement and
// vertex arithmetic, so a sound pairing never fails it. A side vertex of
// another corner lies a whole root edge L away and fails. Another corner's
// inset lies sqrt((L−t)² + t²) away, only about (L − 2t)/√2 beyond t·√2, so
// this check cannot tell it apart when L is within the allowance of 2t;
// bossSeamCornerPaired's exact comparison of the recorded corners refuses
// that pairing.
func bossSeamFalsified(capPos, sidePos, center, down r3.Vec, t float64) bool {
	scale := 1 + t
	for _, c := range [...]float64{center.X, center.Y, center.Z} {
		scale = math.Max(scale, math.Abs(c)+t)
	}
	allow := 1e-9 * scale
	return math.Abs(capPos.Sub(center).Len()-math.Sqrt2*t) > allow ||
		sidePos.Sub(center.Add(down.Scale(t))).Len() > allow
}

// measureBossShell integrates its three cavity intervals exactly, with pi
// enclosed by proofbound's rational interval. Face areas already carry each
// planar or cylindrical patch's own independent bound.
func measureBossShell(ctx context.Context, bp Payload, body *topo.Body) error {
	if bp.BossShell.Round != nil {
		return measureRoundBossShell(ctx, bp, body)
	}
	b := bp.BossShell
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r prismshell.BossRect) proofbound.RatInterval {
		return mul(sub(point(r.U1), point(r.U0)), sub(point(r.V1), point(r.V0)))
	}
	centerU := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.U0), point(r.U1)), 1, 2) }
	centerV := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.V0), point(r.V1)), 1, 2) }
	t := point(b.T)
	h := sub(point(b.InterfaceZ), point(b.Z0))
	hb := sub(point(b.TopZ), point(b.InterfaceZ))
	bw, bh := point(b.Boss.Width()), point(b.Boss.Height())
	plateV, bossV := mul(area(b.Plate), h), mul(area(b.Boss), hb)
	lowerV := mul(area(b.Lower), sub(point(b.LowerZ), point(b.CavityBottomZ)))
	upperHeight := sub(point(b.CavityTopZ), point(b.InterfaceZ))
	upperV := mul(area(b.Upper), upperHeight)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	widths := add(bw, bh)
	// At s = z1 − z in [0, t] the transition void is the boss rectangle
	// inset by a(s) = sqrt(t² − s²) on every side, with sharp corners: the
	// points at distance more than t from the root edges. Integrating
	// (w − 2a)(d − 2a) over s gives wdt − (w+d)πt²/2 + 8t³/3.
	bandV := add(sub(mul(area(b.Boss), t), scale(mul(widths, mul(pi, t2)), 1, 2)), scale(t3, 8, 3))
	volume := sub(sub(add(plateV, bossV), add(lowerV, upperV)), bandV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: boss shell has no proven positive volume", decaderr.ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: measurement.ExactnessOf(ve)}
	// z is measured from the recorded frame origin, not from the plate base.
	plateCZ := scale(add(point(b.Z0), point(b.InterfaceZ)), 1, 2)
	bossCZ := scale(add(point(b.InterfaceZ), point(b.TopZ)), 1, 2)
	upperCZ := scale(add(point(b.InterfaceZ), point(b.CavityTopZ)), 1, 2)
	lowerCZ := scale(add(point(b.CavityBottomZ), point(b.LowerZ)), 1, 2)
	// The same slices weighted by their height u = t − s above LowerZ:
	// wdt²/2 + (2/3 − π/2)(w+d)t³ + 5t⁴/3.
	bandZ := add(mul(point(b.LowerZ), bandV),
		add(add(scale(mul(area(b.Boss), t2), 1, 2),
			sub(scale(mul(widths, t3), 2, 3), scale(mul(pi, mul(widths, t3)), 1, 2))), scale(t4, 5, 3)))
	mu := sub(sub(add(mul(plateV, centerU(b.Plate)), mul(bossV, centerU(b.Boss))),
		add(mul(lowerV, centerU(b.Lower)), mul(upperV, centerU(b.Upper)))), mul(bandV, centerU(b.Boss)))
	mv := sub(sub(add(mul(plateV, centerV(b.Plate)), mul(bossV, centerV(b.Boss))),
		add(mul(lowerV, centerV(b.Lower)), mul(upperV, centerV(b.Upper)))), mul(bandV, centerV(b.Boss)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, lowerCZ), mul(upperV, upperCZ))), bandZ)
	ivs := []proofbound.RatInterval{mu, mv, mz}
	var c [3]proofbound.BoundedScalar
	for i, iv := range ivs {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: boss shell centroid has no finite enclosure", decaderr.ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.RefView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value), Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var a proofbound.BoundedScalar
	for _, f := range topobuild.Faces(body) {
		a = proofbound.BoundedAdd(a, proofbound.MeasuredScalar(f.Area, f.AreaBound))
	}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(a.Value), Bound: units.SquareMillimeters(a.Bound), Exactness: measurement.ExactnessOf(a.Bound)}
	box, err := Bounds(ctx, bp)
	if err != nil {
		return err
	}
	body.Bounds = box
	return topobuild.ValidateAnalyticMeasurements(body)
}

// pocketShellOpenEdges places the rounded side ring and the collapsed square
// floor ring in the walk order the fillet band reads.
func pocketShellOpenEdges(ctx context.Context, alloc topo.Alloc, bp Payload, btopo *Topology,
	openAt map[brepgeom.EdgeKey]int,
	placeVertex func([3]float64, float64, float64, float64) *topo.Vertex,
	out *openSet, bi int) error {
	band := bp.PocketShell
	e := btopo.Embeds[band.FloorFace]
	for ring, spec := range []struct {
		loop sectionrecord.LoopRecord
		z    float64
	}{{band.Rounded.Outer, band.FloorZ}, {band.CapProfile().Outer, band.LowerZ}} {
		view := prismrecord.Payload{Frame: band.Frame, Xform: bp.Xform, Z0: spec.z, Z1: spec.z}
		for _, seg := range spec.loop.Segments {
			if err := ctx.Err(); err != nil {
				return err
			}
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return err
			}
			key, _ := brepgeom.CurveKey(e, w, spec.z)
			ui, ok := openAt[key]
			if !ok {
				return fmt.Errorf("%w: a blind-pocket shell ring has no open record edge", decaderr.ErrUnsupported)
			}
			u := btopo.Uses[ui]
			from, to := e.Canon(w.StartU, w.StartV, spec.z), e.Canon(w.EndU, w.EndV, spec.z)
			start, end := placeVertex(from, 0, 0, 0), placeVertex(to, 0, 0, 0)
			edge, err := bandEdge(alloc, view, w, true, start, end)
			if err != nil {
				return err
			}
			out.coedge[ui] = topo.CoEdge{Edge: edge, Forward: u.DirFrom == from && u.DirTo == to}
			if ring == 0 {
				out.side[bi] = append(out.side[bi], topo.CoEdge{Edge: edge, Forward: true})
			} else {
				out.cap[bi] = append(out.cap[bi], topo.CoEdge{Edge: edge, Forward: true})
			}
		}
	}
	return nil
}

// attachPocketShellBand reuses LF9's cylinder and sphere patches with a
// virtual upward floor cap. The actual floor is downward-facing and supplies
// the square cap ring; the virtual cap is only the band's frame datum.
func attachPocketShellBand(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer,
	bp Payload, open openSet) ([]*topo.Face, error) {
	band := bp.PocketShell
	bi := len(bp.LoopBands)
	b := LoopBand{Face: band.FloorFace, Loop: 0, Orig: band.Rounded.Outer,
		Setback: capblend.Setback{Dc: band.T, Ds: band.T}, Sigma: 1, Kind: BandFillet}
	capRegion := band.CapProfile()
	fake := Face{Frame: band.Frame, Region: &capRegion,
		Z0: band.LowerZ, Z1: band.LowerZ, Outward: band.CapOutward(), Role: "pocketShellBand"}
	copyBP := bp
	copyBP.Faces = append([]Face(nil), bp.Faces...)
	copyBP.Faces[band.FloorFace] = fake
	copyBP.LoopBands = append(append([]LoopBand(nil), bp.LoopBands...), b)
	embeds, err := Embeds(bp.Faces)
	if err != nil {
		return nil, err
	}
	patches, _, err := attachFilletBand(ctx, alloc, body, ref, copyBP, bi, open,
		embeds[band.FloorFace], freeform.NewFreeformWork())
	return patches, err
}

// measurePocketShell subtracts the eroded outer box and adds the exact
// Minkowski expansion of the blind pocket within that cavity.
func measurePocketShell(ctx context.Context, bp Payload, body *topo.Body) error {
	b := bp.PocketShell
	if b.Round != nil {
		return measureCounterboreShell(ctx, bp, body)
	}
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r prismshell.BossRect) proofbound.RatInterval {
		return mul(sub(point(r.U1), point(r.U0)), sub(point(r.V1), point(r.V0)))
	}
	centerU := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.U0), point(r.U1)), 1, 2) }
	centerV := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.V0), point(r.V1)), 1, 2) }
	t := point(b.T)
	outerH := sub(point(b.TopZ), point(b.Z0))
	pocketH := sub(point(b.TopZ), point(b.FloorZ))
	innerH := sub(point(b.TopZ), add(point(b.Z0), t))
	w := sub(point(b.Pocket.U1), point(b.Pocket.U0))
	d := sub(point(b.Pocket.V1), point(b.Pocket.V0))
	perimHalf := add(w, d)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	outerV := mul(area(b.Outer), outerH)
	pocketV := mul(area(b.Pocket), pocketH)
	innerV := mul(area(b.Inner), innerH)
	roundedArea := add(add(area(b.Pocket), scale(mul(perimHalf, t), 2, 1)), mul(pi, t2))
	upperV := mul(roundedArea, pocketH)
	lowerV := add(add(mul(area(b.Pocket), t), scale(mul(mul(perimHalf, pi), t2), 1, 2)),
		scale(mul(pi, t3), 2, 3))
	obstacleV := add(upperV, lowerV)
	volume := add(sub(sub(outerV, pocketV), innerV), obstacleV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: blind-pocket shell has no proven positive volume", decaderr.ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: measurement.ExactnessOf(ve)}
	outerCZ := scale(add(point(b.Z0), point(b.TopZ)), 1, 2)
	pocketCZ := scale(add(point(b.FloorZ), point(b.TopZ)), 1, 2)
	innerCZ := scale(add(add(point(b.Z0), t), point(b.TopZ)), 1, 2)
	lowerFirst := sub(mul(point(b.FloorZ), lowerV),
		add(add(scale(mul(area(b.Pocket), t2), 1, 2), scale(mul(perimHalf, t3), 2, 3)),
			scale(mul(pi, t4), 1, 4)))
	upperFirst := mul(upperV, pocketCZ)
	obstacleZ := add(lowerFirst, upperFirst)
	mu := add(sub(sub(mul(outerV, centerU(b.Outer)), mul(pocketV, centerU(b.Pocket))),
		mul(innerV, centerU(b.Inner))), mul(obstacleV, centerU(b.Pocket)))
	mv := add(sub(sub(mul(outerV, centerV(b.Outer)), mul(pocketV, centerV(b.Pocket))),
		mul(innerV, centerV(b.Inner))), mul(obstacleV, centerV(b.Pocket)))
	mz := add(sub(sub(mul(outerV, outerCZ), mul(pocketV, pocketCZ)),
		mul(innerV, innerCZ)), obstacleZ)
	ivs := []proofbound.RatInterval{mu, mv, mz}
	var c [3]proofbound.BoundedScalar
	for i, iv := range ivs {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: blind-pocket shell centroid has no finite enclosure", decaderr.ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.RefView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var areaTotal proofbound.BoundedScalar
	for _, f := range topobuild.Faces(body) {
		areaTotal = proofbound.BoundedAdd(areaTotal, proofbound.MeasuredScalar(f.Area, f.AreaBound))
	}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(areaTotal.Value),
		Bound: units.SquareMillimeters(areaTotal.Bound), Exactness: measurement.ExactnessOf(areaTotal.Bound)}
	box, err := Bounds(ctx, bp)
	if err != nil {
		return err
	}
	body.Bounds = box
	return topobuild.ValidateAnalyticMeasurements(body)
}

// measureCounterboreShell subtracts the inset rectangular cavity and adds
// the dilation of the source's stepped through bore. The quarter-torus
// transition is integrated as a circular radius over its axial interval.
func measureCounterboreShell(ctx context.Context, bp Payload, body *topo.Body) error {
	b := bp.PocketShell.Round
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r prismshell.BossRect) proofbound.RatInterval {
		return mul(sub(point(r.U1), point(r.U0)), sub(point(r.V1), point(r.V0)))
	}
	centerU := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.U0), point(r.U1)), 1, 2) }
	centerV := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.V0), point(r.V1)), 1, 2) }
	square := func(r proofbound.RatInterval) proofbound.RatInterval { return mul(r, r) }
	t, rs, rl := point(b.T), point(b.Small.Radius), point(b.Large.Radius)
	rsExpanded, rlExpanded := add(rs, t), add(rl, t)
	farH := point(math.Abs(b.StepZ - b.FarZ))
	nearH := point(math.Abs(b.NearZ - b.StepZ))
	h := add(farH, nearH)
	outerV := mul(area(b.Plate), h)
	smallHoleV := mul(pi, mul(square(rs), farH))
	largeHoleV := mul(pi, mul(square(rl), nearH))
	sourceV := sub(sub(outerV, smallHoleV), largeHoleV)
	innerV := mul(area(b.Inset), sub(h, t))
	lowerH := sub(farH, scale(t, 2, 1))
	lowerV := mul(pi, mul(square(rsExpanded), lowerH))
	upperV := mul(pi, mul(square(rlExpanded), nearH))
	t2, t3, t4 := square(t), mul(square(t), t), square(square(t))
	// At distance v below the shoulder, the expanded hole radius is
	// rl + sqrt(t²-v²), 0 <= v <= t.
	transitionArea := add(add(mul(square(rl), t), scale(mul(mul(rl, pi), t2), 1, 2)), scale(t3, 2, 3))
	transitionV := mul(pi, transitionArea)
	obstacleV := add(add(lowerV, transitionV), upperV)
	volume := add(sub(sourceV, innerV), obstacleV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: counterbore shell has no proven positive volume", decaderr.ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve),
		Exactness: measurement.ExactnessOf(ve)}
	outerW := mul(outerV, scale(h, 1, 2))
	smallHoleW := mul(smallHoleV, scale(farH, 1, 2))
	largeHoleW := mul(largeHoleV, add(farH, scale(nearH, 1, 2)))
	sourceW := sub(sub(outerW, smallHoleW), largeHoleW)
	innerW := mul(innerV, scale(add(h, t), 1, 2))
	lowerW := mul(lowerV, scale(farH, 1, 2))
	transitionVFirst := add(add(scale(mul(square(rl), t2), 1, 2),
		scale(mul(rl, t3), 2, 3)), scale(t4, 1, 4))
	transitionW := sub(mul(transitionV, farH), mul(pi, transitionVFirst))
	upperW := mul(upperV, add(farH, scale(nearH, 1, 2)))
	mw := add(sub(sourceW, innerW), add(add(lowerW, transitionW), upperW))
	mu := sub(mul(outerV, centerU(b.Plate)), mul(innerV, centerU(b.Inset)))
	mv := sub(mul(outerV, centerV(b.Plate)), mul(innerV, centerV(b.Inset)))
	holesV := add(smallHoleV, largeHoleV)
	mu = add(sub(mu, mul(holesV, point(b.Center.U))), mul(obstacleV, point(b.Center.U)))
	mv = add(sub(mv, mul(holesV, point(b.Center.V))), mul(obstacleV, point(b.Center.V)))
	mz := add(mul(point(b.FarZ), volume), mul(point(b.Sign), mw))
	var c [3]proofbound.BoundedScalar
	for i, iv := range []proofbound.RatInterval{mu, mv, mz} {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: counterbore shell centroid has no finite enclosure", decaderr.ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.RefView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var surfaceArea proofbound.BoundedScalar
	for _, face := range topobuild.Faces(body) {
		surfaceArea = proofbound.BoundedAdd(surfaceArea, proofbound.MeasuredScalar(face.Area, face.AreaBound))
	}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(surfaceArea.Value),
		Bound: units.SquareMillimeters(surfaceArea.Bound), Exactness: measurement.ExactnessOf(surfaceArea.Bound)}
	box, err := Bounds(ctx, bp)
	if err != nil {
		return err
	}
	body.Bounds = box
	return topobuild.ValidateAnalyticMeasurements(body)
}

// The void below the circular boss's inset wall keeps distance t from the
// root circle, so at height u above lowerZ its radius is
// r − sqrt(t² − (t−u)²). Its area and first height moment integrate to the
// intervals below; pi is enclosed rationally.
func measureRoundBossShell(ctx context.Context, bp Payload, body *topo.Body) error {
	b, circle := bp.BossShell, bp.BossShell.Round
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r prismshell.BossRect) proofbound.RatInterval {
		return mul(sub(point(r.U1), point(r.U0)), sub(point(r.V1), point(r.V0)))
	}
	h := sub(point(b.InterfaceZ), point(b.Z0))
	hb := sub(point(b.TopZ), point(b.InterfaceZ))
	t, r := point(b.T), point(circle.Radius)
	ri := sub(r, t)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	plateV := mul(area(b.Plate), h)
	bossV := mul(pi, mul(mul(r, r), hb))
	lowerV := mul(area(b.Lower), sub(point(b.LowerZ), point(b.CavityBottomZ)))
	upperV := mul(pi, mul(mul(ri, ri), hb))
	// π[r²t − rπt²/2 + 2t³/3]
	bandAreaIntegral := add(sub(mul(mul(r, r), t), scale(mul(mul(r, pi), t2), 1, 2)), scale(t3, 2, 3))
	bandV := mul(pi, bandAreaIntegral)
	volume := sub(sub(add(plateV, bossV), add(lowerV, upperV)), bandV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: round boss shell has no proven positive volume", decaderr.ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: measurement.ExactnessOf(ve)}
	plateCZ := scale(add(point(b.Z0), point(b.InterfaceZ)), 1, 2)
	bossCZ := scale(add(point(b.InterfaceZ), point(b.TopZ)), 1, 2)
	lowerCZ := scale(add(point(b.CavityBottomZ), point(b.LowerZ)), 1, 2)
	// π[r²t²/2 + (2/3 − π/2)rt³ + 5t⁴/12]
	rt3 := mul(r, t3)
	bandFirst := add(add(scale(mul(mul(r, r), t2), 1, 2), sub(scale(rt3, 2, 3), scale(mul(pi, rt3), 1, 2))),
		scale(t4, 5, 12))
	bandZ := add(mul(point(b.LowerZ), bandV), mul(pi, bandFirst))
	plateNet := sub(plateV, lowerV)
	bossNet := sub(sub(bossV, upperV), bandV)
	plateU := scale(add(point(b.Plate.U0), point(b.Plate.U1)), 1, 2)
	plateVCenter := scale(add(point(b.Plate.V0), point(b.Plate.V1)), 1, 2)
	mu := add(mul(plateNet, plateU), mul(bossNet, point(circle.Center.U)))
	mv := add(mul(plateNet, plateVCenter), mul(bossNet, point(circle.Center.V)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, lowerCZ), mul(upperV, bossCZ))), bandZ)
	var c [3]proofbound.BoundedScalar
	for i, iv := range []proofbound.RatInterval{mu, mv, mz} {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: round boss centroid has no finite enclosure", decaderr.ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.RefView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var surfaceArea proofbound.BoundedScalar
	for _, face := range topobuild.Faces(body) {
		surfaceArea = proofbound.BoundedAdd(surfaceArea, proofbound.MeasuredScalar(face.Area, face.AreaBound))
	}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(surfaceArea.Value),
		Bound: units.SquareMillimeters(surfaceArea.Bound), Exactness: measurement.ExactnessOf(surfaceArea.Bound)}
	box, err := Bounds(ctx, bp)
	if err != nil {
		return err
	}
	body.Bounds = box
	return topobuild.ValidateAnalyticMeasurements(body)
}
