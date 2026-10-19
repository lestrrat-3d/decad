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
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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

// attachBossShellBand uses the established analytic fillet patch constructor
// with a virtual cap at the upper ring. The cap is only a geometry datum:
// its ring is actually bounded by the upper cavity walls.
func attachBossShellBand(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer, bp Payload, open openSet) ([]*topo.Face, error) {
	band := bp.BossShell
	bi := len(bp.LoopBands)
	b := LoopBand{Face: band.LedgeFace, Loop: 0, Orig: band.SideProfile().Outer,
		Setback: capblend.Setback{Dc: band.T, Ds: band.T}, Sigma: 1, Kind: BandFillet}
	upper := band.CapProfile()
	fake := Face{Frame: band.Frame, Region: &upper,
		Z0: band.InterfaceZ, Z1: band.InterfaceZ, Outward: false, Role: "bossShellBand"}
	copyBP := bp
	copyBP.Faces = append([]Face(nil), bp.Faces...)
	copyBP.Faces[band.LedgeFace] = fake
	copyBP.LoopBands = append(append([]LoopBand(nil), bp.LoopBands...), b)
	embeds, err := Embeds(bp.Faces)
	if err != nil {
		return nil, err
	}
	patches, _, err := attachFilletBand(ctx, alloc, body, ref, copyBP, bi, open, embeds[band.LedgeFace], freeform.NewFreeformWork())
	return patches, err
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
	lowerV := mul(area(b.Lower), sub(h, scale(t, 2, 1)))
	upperHeight := sub(point(b.CavityTopZ), point(b.InterfaceZ))
	upperV := mul(area(b.Upper), upperHeight)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	widths := add(bw, bh)
	bandV := add(
		add(sub(mul(area(b.Boss), t), scale(mul(widths, t2), 2, 1)),
			scale(mul(mul(widths, pi), t2), 1, 2)),
		sub(scale(t3, 20, 3), scale(mul(pi, t3), 2, 1)),
	)
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
	bandZ := add(mul(point(b.LowerZ), bandV),
		add(sub(scale(mul(area(b.Boss), t2), 1, 2), scale(mul(widths, t3), 1, 3)), scale(t4, 1, 3)))
	mu := sub(sub(add(mul(plateV, centerU(b.Plate)), mul(bossV, centerU(b.Boss))),
		add(mul(lowerV, centerU(b.Lower)), mul(upperV, centerU(b.Upper)))), mul(bandV, centerU(b.Boss)))
	mv := sub(sub(add(mul(plateV, centerV(b.Plate)), mul(bossV, centerV(b.Boss))),
		add(mul(lowerV, centerV(b.Lower)), mul(upperV, centerV(b.Upper)))), mul(bandV, centerV(b.Boss)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, plateCZ), mul(upperV, upperCZ))), bandZ)
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

// The void below the circular boss's inset wall has radius
// (r-t)+sqrt(t²-u²) at height u above lowerZ. Its area and first height
// moment integrate to the intervals below; pi is enclosed rationally.
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
	lowerV := mul(area(b.Lower), sub(h, scale(t, 2, 1)))
	upperV := mul(pi, mul(mul(ri, ri), hb))
	bandAreaIntegral := add(add(mul(mul(ri, ri), t), scale(mul(mul(ri, pi), t2), 1, 2)), scale(t3, 2, 3))
	bandV := mul(pi, bandAreaIntegral)
	volume := sub(sub(add(plateV, bossV), add(lowerV, upperV)), bandV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: round boss shell has no proven positive volume", decaderr.ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: measurement.ExactnessOf(ve)}
	plateCZ := scale(add(point(b.Z0), point(b.InterfaceZ)), 1, 2)
	bossCZ := scale(add(point(b.InterfaceZ), point(b.TopZ)), 1, 2)
	bandFirst := add(add(scale(mul(mul(ri, ri), t2), 1, 2), scale(mul(ri, t3), 2, 3)), scale(t4, 1, 4))
	bandZ := add(mul(point(b.LowerZ), bandV), mul(pi, bandFirst))
	plateNet := sub(plateV, lowerV)
	bossNet := sub(sub(bossV, upperV), bandV)
	plateU := scale(add(point(b.Plate.U0), point(b.Plate.U1)), 1, 2)
	plateVCenter := scale(add(point(b.Plate.V0), point(b.Plate.V1)), 1, 2)
	mu := add(mul(plateNet, plateU), mul(bossNet, point(circle.Center.U)))
	mv := add(mul(plateNet, plateVCenter), mul(bossNet, point(circle.Center.V)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, plateCZ), mul(upperV, bossCZ))), bandZ)
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
